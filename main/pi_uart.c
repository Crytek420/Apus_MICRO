#include "pi_uart.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "PI_UART";
static uint32_t g_heartbeat_sequence = 0;

/**
 * @brief Get current timestamp in milliseconds
 */
static inline uint32_t get_timestamp_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/**
 * @brief Calculate CRC16-CCITT checksum
 *
 * Polynomial: 0x1021 (x^16 + x^12 + x^5 + 1)
 * Initial value: 0xFFFF
 */
static uint16_t calculate_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++)
    {
        crc ^= (uint16_t)data[i] << 8;

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc = crc << 1;
            }
        }
    }

    return crc;
}

/**
 * @brief Send a packet to Raspberry Pi
 */
static esp_err_t pi_uart_send_packet(pi_packet_type_t type, const void *payload, uint8_t payload_len)
{
    if (payload == NULL && payload_len > 0)
    {
        ESP_LOGE(TAG, "Invalid payload");
        return ESP_ERR_INVALID_ARG;
    }

    if (payload_len > PI_PACKET_MAX_PAYLOAD)
    {
        ESP_LOGE(TAG, "Payload too large: %d > %d", payload_len, PI_PACKET_MAX_PAYLOAD);
        return ESP_ERR_INVALID_SIZE;
    }

    // Build packet
    uint8_t packet_buffer[4 + PI_PACKET_MAX_PAYLOAD + 2]; // header(4) + payload + crc(2)
    packet_buffer[0] = PI_PACKET_SYNC1;
    packet_buffer[1] = PI_PACKET_SYNC2;
    packet_buffer[2] = (uint8_t)type;
    packet_buffer[3] = payload_len;

    // Copy payload
    if (payload_len > 0)
    {
        memcpy(&packet_buffer[4], payload, payload_len);
    }

    // Calculate CRC over header + payload
    uint16_t crc = calculate_crc16(packet_buffer, 4 + payload_len);
    packet_buffer[4 + payload_len] = (uint8_t)(crc & 0xFF);   // CRC low byte
    packet_buffer[4 + payload_len + 1] = (uint8_t)(crc >> 8); // CRC high byte

    // Send packet
    int total_len = 4 + payload_len + 2;
    int bytes_written = uart_write_bytes(PI_UART_NUM, (const char *)packet_buffer, total_len);

    if (bytes_written != total_len)
    {
        ESP_LOGW(TAG, "Failed to send complete packet: %d/%d bytes", bytes_written, total_len);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Sent packet type 0x%02X, %d bytes payload", type, payload_len);
    return ESP_OK;
}

/**
 * @brief Initialize UART interface for Raspberry Pi Zero 2W communication
 */
esp_err_t pi_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = PI_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_LOGI(TAG, "Configuring UART%d for Raspberry Pi Zero 2W", PI_UART_NUM);
    ESP_LOGI(TAG, "PIN Configuration: TX=GPIO%d, RX=GPIO%d, Baud=%d",
             PI_UART_TX_PIN, PI_UART_RX_PIN, PI_UART_BAUDRATE);

    // Configure UART parameters
    esp_err_t ret = uart_param_config(PI_UART_NUM, &uart_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set UART pins
    ret = uart_set_pin(PI_UART_NUM, PI_UART_TX_PIN, PI_UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }

    // Install UART driver
    ret = uart_driver_install(PI_UART_NUM, PI_UART_BUF_SIZE,
                              PI_UART_BUF_SIZE, 0, NULL, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Pi UART initialized successfully");
    ESP_LOGI(TAG, "ESP32-S3 ready to communicate with Raspberry Pi Zero 2W");
    ESP_LOGI(TAG, "Protocol: Binary packets with CRC16 validation");
    ESP_LOGI(TAG, "Connection: ESP32 TX(GPIO%d) -> Pi RX(GPIO15), ESP32 RX(GPIO%d) -> Pi TX(GPIO14)",
             PI_UART_TX_PIN, PI_UART_RX_PIN);

    return ESP_OK;
}

/**
 * @brief Send IMU data to Raspberry Pi
 */
esp_err_t pi_uart_send_imu(const wt901b_data_t *imu_data)
{
    if (imu_data == NULL)
    {
        ESP_LOGE(TAG, "Invalid IMU data pointer");
        return ESP_ERR_INVALID_ARG;
    }

    pi_imu_payload_t payload = {
        .acc_x = imu_data->acc_x,
        .acc_y = imu_data->acc_y,
        .acc_z = imu_data->acc_z,
        .gyro_x = imu_data->gyro_x,
        .gyro_y = imu_data->gyro_y,
        .gyro_z = imu_data->gyro_z,
        .mag_x = imu_data->mag_x,
        .mag_y = imu_data->mag_y,
        .mag_z = imu_data->mag_z,
        .roll = imu_data->roll,
        .pitch = imu_data->pitch,
        .yaw = imu_data->yaw,
        .temperature = imu_data->temperature,
        .timestamp_ms = get_timestamp_ms()};

    return pi_uart_send_packet(PI_PACKET_TYPE_IMU, &payload, sizeof(payload));
}

/**
 * @brief Send CRSF channel data to Raspberry Pi Zero 2W
 */
esp_err_t pi_uart_send_crsf_channels(const crsf_channels_t *channels, bool failsafe)
{
    if (channels == NULL)
    {
        ESP_LOGE(TAG, "Invalid channel data pointer");
        return ESP_ERR_INVALID_ARG;
    }

    pi_crsf_channels_payload_t payload;
    memcpy(payload.channels, channels->channels, sizeof(payload.channels));
    payload.timestamp_ms = get_timestamp_ms();
    payload.failsafe = failsafe ? 1 : 0;

    return pi_uart_send_packet(PI_PACKET_TYPE_CRSF_CHANNELS, &payload, sizeof(payload));
}

/**
 * @brief Send CRSF link statistics to Raspberry Pi
 */
esp_err_t pi_uart_send_link_stats(const crsf_link_statistics_t *stats)
{
    if (stats == NULL)
    {
        ESP_LOGE(TAG, "Invalid link stats pointer");
        return ESP_ERR_INVALID_ARG;
    }

    pi_link_stats_payload_t payload = {
        .uplink_rssi_ant1 = stats->uplink_rssi_ant1,
        .uplink_rssi_ant2 = stats->uplink_rssi_ant2,
        .uplink_link_quality = stats->uplink_link_quality,
        .uplink_snr = stats->uplink_snr,
        .active_antenna = stats->active_antenna,
        .rf_mode = stats->rf_mode,
        .uplink_tx_power = stats->uplink_tx_power,
        .downlink_rssi = stats->downlink_rssi,
        .downlink_link_quality = stats->downlink_link_quality,
        .downlink_snr = stats->downlink_snr,
        .timestamp_ms = get_timestamp_ms()};

    return pi_uart_send_packet(PI_PACKET_TYPE_CRSF_LINK_STATS, &payload, sizeof(payload));
}

/**
 * @brief Send system information to Raspberry Pi
 */
esp_err_t pi_uart_send_system_info(void)
{
    pi_system_info_payload_t payload = {
        .cpu_temp = 0.0f,       // ESP32-S3 doesn't have built-in temp sensor
        .supply_voltage = 0.0f, // Would need ADC setup for voltage measurement
        .free_heap = esp_get_free_heap_size(),
        .uptime_ms = get_timestamp_ms(),
        .cpu_usage = 0, // Would need CPU profiling
        .timestamp_ms = get_timestamp_ms()};

    return pi_uart_send_packet(PI_PACKET_TYPE_SYSTEM_INFO, &payload, sizeof(payload));
}

/**
 * @brief Send heartbeat to Raspberry Pi
 */
esp_err_t pi_uart_send_heartbeat(void)
{
    pi_heartbeat_payload_t payload = {
        .sequence = g_heartbeat_sequence++,
        .timestamp_ms = get_timestamp_ms()};

    return pi_uart_send_packet(PI_PACKET_TYPE_HEARTBEAT, &payload, sizeof(payload));
}
/**
 * @brief Check if valid data has been received from Pi (deprecated)
 */
int pi_uart_receive(uint8_t *buffer, size_t max_len)
{
    if (buffer == NULL || max_len == 0)
    {
        return 0;
    }

    // Check available data
    size_t available = 0;
    esp_err_t ret = uart_get_buffered_data_len(PI_UART_NUM, &available);

    if (ret != ESP_OK || available == 0)
    {
        return 0;
    }

    // Read available data (up to max_len)
    size_t read_len = (available < max_len) ? available : max_len;
    int bytes_read = uart_read_bytes(PI_UART_NUM, buffer, read_len, 0);

    if (bytes_read > 0)
    {
        ESP_LOGD(TAG, "Received %d bytes from Pi", bytes_read);
    }

    return bytes_read;
}

/**
 * @brief Send a string to Raspberry Pi Zero 2W (deprecated)
 */
esp_err_t pi_uart_send_string(const char *str)
{
    if (str == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    size_t len = strlen(str);
    int bytes_written = uart_write_bytes(PI_UART_NUM, str, len);

    if (bytes_written != (int)len)
    {
        ESP_LOGW(TAG, "Failed to send complete string: %d/%d bytes sent",
                 bytes_written, (int)len);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Sent string to Pi: %s", str);
    return ESP_OK;
}
