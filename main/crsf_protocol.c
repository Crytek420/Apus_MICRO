#include "crsf_protocol.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>

// Temporarily disable verbose logging for IMU debugging
#define LOG_LOCAL_LEVEL ESP_LOG_INFO

static const char *TAG = "CRSF";

/* UART Configuration for ESP32-S3 */
#define CRSF_UART_NUM UART_NUM_1
#define CRSF_UART_TX_PIN 17 // U1TXD native pin
#define CRSF_UART_RX_PIN 18 // U1RXD native pin
#define CRSF_UART_RTS_PIN UART_PIN_NO_CHANGE
#define CRSF_UART_CTS_PIN UART_PIN_NO_CHANGE
#define CRSF_UART_BUF_SIZE 512

/* CRSF Task Configuration */
#define CRSF_TASK_FREQUENCY_HZ 100
#define CRSF_TASK_PERIOD_MS (1000 / CRSF_TASK_FREQUENCY_HZ)

/* Global Variables */
static crsf_channels_t g_rc_channels = {0};
static bool g_rc_data_valid = false;
static uint32_t g_last_rc_frame_time = 0;
static crsf_link_statistics_t g_link_statistics = {0};
static bool g_link_stats_valid = false;

/* CRC8 Table for DVB-S2 */
static const uint8_t crc8_dvb_s2_table[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

/**
 * @brief Calculate CRC8 using DVB-S2 polynomial
 */
uint8_t crsf_calculate_crc(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        crc = crc8_dvb_s2_table[crc ^ data[i]];
    }
    return crc;
}

/**
 * @brief Validate CRSF frame
 */
bool crsf_validate_frame(const crsf_frame_t *frame)
{
    if (frame->frame_size < 2 || frame->frame_size > CRSF_PAYLOAD_SIZE_MAX + 2)
    {
        return false;
    }

    // Calculate CRC over type and payload
    uint8_t crc = crsf_calculate_crc((uint8_t *)&frame->type, frame->frame_size - 1);
    uint8_t received_crc = frame->payload[frame->frame_size - 2];

    return crc == received_crc;
}

/**
 * @brief Parse RC channels from CRSF payload (11-bit channels packed)
 */
void crsf_parse_rc_channels(const uint8_t *payload, crsf_channels_t *channels)
{
    channels->channels[0] = (uint16_t)((payload[0] | payload[1] << 8) & 0x07FF);
    channels->channels[1] = (uint16_t)((payload[1] >> 3 | payload[2] << 5) & 0x07FF);
    channels->channels[2] = (uint16_t)((payload[2] >> 6 | payload[3] << 2 | payload[4] << 10) & 0x07FF);
    channels->channels[3] = (uint16_t)((payload[4] >> 1 | payload[5] << 7) & 0x07FF);
    channels->channels[4] = (uint16_t)((payload[5] >> 4 | payload[6] << 4) & 0x07FF);
    channels->channels[5] = (uint16_t)((payload[6] >> 7 | payload[7] << 1 | payload[8] << 9) & 0x07FF);
    channels->channels[6] = (uint16_t)((payload[8] >> 2 | payload[9] << 6) & 0x07FF);
    channels->channels[7] = (uint16_t)((payload[9] >> 5 | payload[10] << 3) & 0x07FF);
    channels->channels[8] = (uint16_t)((payload[11] | payload[12] << 8) & 0x07FF);
    channels->channels[9] = (uint16_t)((payload[12] >> 3 | payload[13] << 5) & 0x07FF);
    channels->channels[10] = (uint16_t)((payload[13] >> 6 | payload[14] << 2 | payload[15] << 10) & 0x07FF);
    channels->channels[11] = (uint16_t)((payload[15] >> 1 | payload[16] << 7) & 0x07FF);
    channels->channels[12] = (uint16_t)((payload[16] >> 4 | payload[17] << 4) & 0x07FF);
    channels->channels[13] = (uint16_t)((payload[17] >> 7 | payload[18] << 1 | payload[19] << 9) & 0x07FF);
    channels->channels[14] = (uint16_t)((payload[19] >> 2 | payload[20] << 6) & 0x07FF);
    channels->channels[15] = (uint16_t)((payload[20] >> 5 | payload[21] << 3) & 0x07FF);
}

/**
 * @brief Prepare GPS telemetry frame
 */
uint8_t crsf_prepare_gps_frame(crsf_frame_t *frame, const crsf_gps_t *gps)
{
    frame->device_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    frame->type = CRSF_FRAMETYPE_GPS;

    memcpy(frame->payload, gps, sizeof(crsf_gps_t));

    frame->frame_size = sizeof(crsf_gps_t) + 2; // payload + type + crc

    uint8_t crc = crsf_calculate_crc((uint8_t *)&frame->type, frame->frame_size - 1);
    frame->payload[frame->frame_size - 2] = crc;

    return frame->frame_size + 2; // Total frame size including address and length
}

/**
 * @brief Prepare battery telemetry frame
 */
uint8_t crsf_prepare_battery_frame(crsf_frame_t *frame, const crsf_battery_t *battery)
{
    frame->device_addr = CRSF_ADDRESS_RADIO_TRANSMITTER; // Send to TX (via XR1)
    frame->type = CRSF_FRAMETYPE_BATTERY_SENSOR;

    // Pack battery data manually to ensure correct byte order
    uint8_t *payload = frame->payload;

    // Voltage (uint16_t, BIG endian) - MSB first
    payload[0] = (battery->voltage >> 8) & 0xFF;
    payload[1] = battery->voltage & 0xFF;

    // Current (uint16_t, BIG endian) - MSB first
    payload[2] = (battery->current >> 8) & 0xFF;
    payload[3] = battery->current & 0xFF;

    // Capacity (uint24_t, BIG endian) - MSB first - CRSF uses 24-bit!
    payload[4] = (battery->capacity >> 16) & 0xFF;
    payload[5] = (battery->capacity >> 8) & 0xFF;
    payload[6] = battery->capacity & 0xFF;

    // Remaining (uint8_t)
    payload[7] = battery->remaining;

    uint8_t payload_size = 8;
    frame->frame_size = payload_size + 2; // type + payload + crc

    uint8_t crc = crsf_calculate_crc((uint8_t *)&frame->type, frame->frame_size - 1);
    payload[payload_size] = crc;

    return frame->frame_size + 2; // addr + frame_size + content
}

/**
 * @brief Prepare attitude telemetry frame
 */
uint8_t crsf_prepare_attitude_frame(crsf_frame_t *frame, const crsf_attitude_t *attitude)
{
    frame->device_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    frame->type = CRSF_FRAMETYPE_ATTITUDE;

    memcpy(frame->payload, attitude, sizeof(crsf_attitude_t));

    frame->frame_size = sizeof(crsf_attitude_t) + 2;

    uint8_t crc = crsf_calculate_crc((uint8_t *)&frame->type, frame->frame_size - 1);
    frame->payload[frame->frame_size - 2] = crc;

    return frame->frame_size + 2;
}

/**
 * @brief Prepare flight mode telemetry frame
 */
uint8_t crsf_prepare_flight_mode_frame(crsf_frame_t *frame, const char *mode)
{
    frame->device_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    frame->type = CRSF_FRAMETYPE_FLIGHT_MODE;

    strncpy((char *)frame->payload, mode, 15);
    frame->payload[15] = '\0';

    uint8_t payload_len = strlen((char *)frame->payload) + 1;
    frame->frame_size = payload_len + 2;

    uint8_t crc = crsf_calculate_crc((uint8_t *)&frame->type, frame->frame_size - 1);
    frame->payload[frame->frame_size - 2] = crc;

    return frame->frame_size + 2;
}

/**
 * @brief Initialize CRSF UART interface
 */
esp_err_t crsf_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = CRSF_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_LOGI(TAG, "Configuring UART%d: TX=GPIO%d, RX=GPIO%d, Baud=%d",
             CRSF_UART_NUM, CRSF_UART_TX_PIN, CRSF_UART_RX_PIN, CRSF_BAUDRATE);

    ESP_ERROR_CHECK(uart_param_config(CRSF_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(CRSF_UART_NUM, CRSF_UART_TX_PIN, CRSF_UART_RX_PIN,
                                 CRSF_UART_RTS_PIN, CRSF_UART_CTS_PIN));
    ESP_ERROR_CHECK(uart_driver_install(CRSF_UART_NUM, CRSF_UART_BUF_SIZE,
                                        CRSF_UART_BUF_SIZE, 0, NULL, 0));

    ESP_LOGI(TAG, "CRSF UART initialized successfully");
    ESP_LOGI(TAG, "Waiting for data from XR1 module...");

    return ESP_OK;
}

/**
 * @brief Process incoming CRSF data
 */
static void crsf_process_frame(const uint8_t *data, size_t length)
{
    if (length < 4)
        return; // Minimum frame size

    crsf_frame_t *frame = (crsf_frame_t *)data;

    // Validate frame
    if (!crsf_validate_frame(frame))
    {
        ESP_LOGW(TAG, "Invalid CRSF frame (CRC mismatch)");
        return;
    }

    // Process frame based on type
    switch (frame->type)
    {
    case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
        crsf_parse_rc_channels(frame->payload, &g_rc_channels);
        g_rc_data_valid = true;
        g_last_rc_frame_time = xTaskGetTickCount();
        ESP_LOGD(TAG, "RC Channels: [%d, %d, %d, %d]",
                 g_rc_channels.channels[0],
                 g_rc_channels.channels[1],
                 g_rc_channels.channels[2],
                 g_rc_channels.channels[3]);
        break;

    case CRSF_FRAMETYPE_LINK_STATISTICS:
    {
        static uint32_t last_link_stats_log = 0;
        uint32_t now = xTaskGetTickCount();

        // Store link statistics
        if (frame->frame_size >= sizeof(crsf_link_statistics_t) + 2)
        {
            memcpy(&g_link_statistics, frame->payload, sizeof(crsf_link_statistics_t));
            g_link_stats_valid = true;
        }

        // Only log once per second (1 Hz)
        if ((now - last_link_stats_log) >= pdMS_TO_TICKS(1000))
        {
            crsf_link_statistics_t *stats = (crsf_link_statistics_t *)frame->payload;
            ESP_LOGI(TAG, "Link Stats - RSSI1: %d, LQ: %d%%",
                     stats->uplink_rssi_ant1, stats->uplink_link_quality);
            last_link_stats_log = now;
        }
        break;
    }

    default:
        ESP_LOGD(TAG, "Received frame type: 0x%02X", frame->type);
        break;
    }
}

/**
 * @brief Read and process CRSF data from UART
 */
static void crsf_read_data(void)
{
    static uint8_t rx_buffer[CRSF_FRAME_SIZE_MAX * 2];
    static uint8_t buffer_pos = 0;

    // Read available data into buffer
    int available = 0;
    uart_get_buffered_data_len(CRSF_UART_NUM, (size_t *)&available);

    if (available > 0)
    {
        int space_left = sizeof(rx_buffer) - buffer_pos;
        int to_read = (available > space_left) ? space_left : available;

        int length = uart_read_bytes(CRSF_UART_NUM, &rx_buffer[buffer_pos], to_read, pdMS_TO_TICKS(10));

        if (length > 0)
        {
            buffer_pos += length;
            ESP_LOGD(TAG, "UART received %d bytes, buffer now has %d bytes", length, buffer_pos);
        }
    }

    // Process complete frames from buffer
    while (buffer_pos >= 4) // Minimum: address + length + type + crc
    {
        // Look for frame start
        bool found_sync = false;
        uint8_t sync_pos = 0;

        for (uint8_t i = 0; i < buffer_pos - 3; i++)
        {
            if (rx_buffer[i] == CRSF_ADDRESS_FLIGHT_CONTROLLER ||
                rx_buffer[i] == CRSF_ADDRESS_BROADCAST)
            {
                uint8_t frame_size = rx_buffer[i + 1];

                // Validate frame size
                if (frame_size >= 2 && frame_size <= CRSF_PAYLOAD_SIZE_MAX + 2)
                {
                    // Check if we have complete frame
                    if (i + frame_size + 2 <= buffer_pos)
                    {
                        ESP_LOGD(TAG, "Processing frame: addr=0x%02X, size=%d", rx_buffer[i], frame_size);
                        crsf_process_frame(&rx_buffer[i], frame_size + 2);

                        // Remove processed frame from buffer
                        sync_pos = i + frame_size + 2;
                        found_sync = true;
                        break;
                    }
                }
            }
        }

        if (found_sync)
        {
            // Shift remaining data to start of buffer
            buffer_pos -= sync_pos;
            if (buffer_pos > 0)
            {
                memmove(rx_buffer, &rx_buffer[sync_pos], buffer_pos);
            }
        }
        else
        {
            // No valid frame found, remove first byte and try again
            if (buffer_pos > CRSF_FRAME_SIZE_MAX)
            {
                buffer_pos--;
                memmove(rx_buffer, &rx_buffer[1], buffer_pos);
            }
            else
            {
                break; // Wait for more data
            }
        }
    }
}

/**
 * @brief Send telemetry data
 */
void crsf_send_telemetry(crsf_frame_t *frame, uint8_t frame_length)
{
    uart_write_bytes(CRSF_UART_NUM, (const char *)frame, frame_length);
}

/**
 * @brief Get current RC channel values
 */
bool crsf_get_channels(crsf_channels_t *channels)
{
    if (!g_rc_data_valid)
    {
        return false;
    }

    // Check if data is still valid (timeout after 500ms)
    if ((xTaskGetTickCount() - g_last_rc_frame_time) > pdMS_TO_TICKS(500))
    {
        g_rc_data_valid = false;
        ESP_LOGW(TAG, "RC data timeout");
        return false;
    }

    memcpy(channels, &g_rc_channels, sizeof(crsf_channels_t));
    return true;
}

/**
 * @brief CRSF FreeRTOS Task - runs at 10 Hz
 */
void crsf_task(void *pvParameters)
{
    ESP_LOGI(TAG, "CRSF Task started (10 Hz)");

    // Initialize UART
    crsf_uart_init();

    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t cycle_count = 0;

    while (1)
    {
        // Read incoming CRSF data
        crsf_read_data();

        // Send telemetry every 100 cycles (1 Hz for telemetry at 100Hz task rate)
        if (cycle_count % 100 == 0)
        {
            // Send battery telemetry
            // CRSF Format: Voltage in 0.1V, Current in 0.1A, Capacity in mAh
            crsf_battery_t battery = {
                .voltage = 126,   // 12.6V (126 * 0.1V)
                .current = 15,    // 1.5A (15 * 0.1A)
                .capacity = 5000, // 5000mAh
                .remaining = 85   // 85%
            };

            crsf_frame_t frame;
            uint8_t frame_len = crsf_prepare_battery_frame(&frame, &battery);

            ESP_LOGI(TAG, "Sending battery telemetry: %.1fV, %.1fA, %lumAh, %u%%",
                     battery.voltage / 10.0f, battery.current / 10.0f,
                     (unsigned long)battery.capacity, battery.remaining);

            // Debug: Print raw frame
            ESP_LOGI(TAG, "Frame: addr=0x%02X, size=%d, type=0x%02X, len=%d",
                     frame.device_addr, frame.frame_size, frame.type, frame_len);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, &frame, frame_len, ESP_LOG_INFO);

            crsf_send_telemetry(&frame, frame_len);
        }
        cycle_count++;

        // Wait for next cycle (10ms for 100 Hz)
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CRSF_TASK_PERIOD_MS));
    }
}

/**
 * @brief Get link statistics (if available)
 */
bool crsf_get_link_stats(crsf_link_statistics_t *stats)
{
    if (stats == NULL)
    {
        return false;
    }

    if (!g_link_stats_valid)
    {
        return false;
    }

    memcpy(stats, &g_link_statistics, sizeof(crsf_link_statistics_t));
    return true;
}

/**
 * @brief Start CRSF communication task
 */
void crsf_start_task(void)
{
    xTaskCreate(crsf_task, "crsf_task", 4096, NULL, 5, NULL);
}
