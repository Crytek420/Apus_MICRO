#include "pi_mavlink.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char *TAG = "PI_MAVLINK";

// MAVLink v1 packet structure
#define MAVLINK_STX_V1 0xFE
#define MAVLINK_MAX_PAYLOAD_LEN 255

// MAVLink message IDs (common.xml)
#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_SYS_STATUS 1
#define MAVLINK_MSG_ID_ATTITUDE 30
#define MAVLINK_MSG_ID_RC_CHANNELS 65
#define MAVLINK_MSG_ID_SCALED_IMU 26
#define MAVLINK_MSG_ID_HIGHRES_IMU 105

// Task handle
static TaskHandle_t mavlink_task_handle = NULL;

/**
 * @brief Get current timestamp in milliseconds
 */
static inline uint32_t get_time_boot_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/**
 * @brief Calculate MAVLink CRC X.25
 */
static uint16_t crc_calculate(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++)
    {
        uint8_t tmp = data[i] ^ (uint8_t)(crc & 0xFF);
        tmp ^= (tmp << 4);
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    }

    return crc;
}

/**
 * @brief Add CRC extra byte for MAVLink message
 */
static uint16_t crc_accumulate(uint16_t crc, uint8_t data)
{
    uint8_t tmp = data ^ (uint8_t)(crc & 0xFF);
    tmp ^= (tmp << 4);
    crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    return crc;
}

/**
 * @brief Pack and send a MAVLink message
 */
static esp_err_t mavlink_send_message(uint8_t msgid, const uint8_t *payload, uint8_t len, uint8_t crc_extra)
{
    uint8_t buffer[MAVLINK_MAX_PAYLOAD_LEN + 12]; // Header(6) + Payload(max 255) + CRC(2)
    uint8_t seq = 0;                              // Packet sequence number (could be incremented)

    // Build header
    buffer[0] = MAVLINK_STX_V1;
    buffer[1] = len;
    buffer[2] = seq;
    buffer[3] = MAVLINK_SYSTEM_ID;
    buffer[4] = MAVLINK_COMPONENT_ID;
    buffer[5] = msgid;

    // Copy payload
    if (len > 0 && payload != NULL)
    {
        memcpy(&buffer[6], payload, len);
    }

    // Calculate CRC
    uint16_t crc = crc_calculate(&buffer[1], len + 5); // From len to end of payload
    crc = crc_accumulate(crc, crc_extra);              // Add message-specific CRC

    buffer[6 + len] = (uint8_t)(crc & 0xFF);
    buffer[6 + len + 1] = (uint8_t)(crc >> 8);

    // Send via UART
    int total_len = 6 + len + 2;
    int bytes_written = uart_write_bytes(PI_UART_NUM, (const char *)buffer, total_len);

    if (bytes_written != total_len)
    {
        ESP_LOGW(TAG, "Failed to send complete MAVLink message %d: %d/%d bytes", msgid, bytes_written, total_len);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Initialize MAVLink UART
 */
esp_err_t pi_mavlink_init(void)
{
    ESP_LOGI(TAG, "Initializing MAVLink communication on UART%d", PI_UART_NUM);
    ESP_LOGI(TAG, "TX: GPIO%d, RX: GPIO%d, Baud: %d", PI_UART_TX_PIN, PI_UART_RX_PIN, PI_UART_BAUDRATE);

    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = PI_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(PI_UART_NUM, &uart_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(PI_UART_NUM, PI_UART_TX_PIN, PI_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_driver_install(PI_UART_NUM, PI_UART_BUF_SIZE, PI_UART_BUF_SIZE, 0, NULL, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "MAVLink UART initialized successfully");
    return ESP_OK;
}

/**
 * @brief Deinitialize MAVLink
 */
esp_err_t pi_mavlink_deinit(void)
{
    if (mavlink_task_handle != NULL)
    {
        vTaskDelete(mavlink_task_handle);
        mavlink_task_handle = NULL;
    }

    uart_driver_delete(PI_UART_NUM);
    ESP_LOGI(TAG, "MAVLink deinitialized");
    return ESP_OK;
}

/**
 * @brief Send HEARTBEAT message
 */
esp_err_t pi_mavlink_send_heartbeat(void)
{
    // HEARTBEAT message (ID: 0, CRC_EXTRA: 50)
    uint8_t payload[9] = {0};
    uint32_t custom_mode = 0;
    uint8_t type = 1;      // MAV_TYPE_FIXED_WING
    uint8_t autopilot = 0; // MAV_AUTOPILOT_GENERIC
    uint8_t base_mode = 0;
    uint8_t system_status = 4; // MAV_STATE_ACTIVE
    uint8_t mavlink_version = 3;

    memcpy(&payload[0], &custom_mode, 4);
    payload[4] = type;
    payload[5] = autopilot;
    payload[6] = base_mode;
    payload[7] = system_status;
    payload[8] = mavlink_version;

    return mavlink_send_message(MAVLINK_MSG_ID_HEARTBEAT, payload, 9, 50);
}

/**
 * @brief Send ATTITUDE message (roll, pitch, yaw)
 */
static esp_err_t mavlink_send_attitude(float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
    // ATTITUDE message (ID: 30, CRC_EXTRA: 39)
    uint8_t payload[28] = {0};
    uint32_t time_boot_ms = get_time_boot_ms();

    memcpy(&payload[0], &time_boot_ms, 4);
    memcpy(&payload[4], &roll, 4);
    memcpy(&payload[8], &pitch, 4);
    memcpy(&payload[12], &yaw, 4);
    memcpy(&payload[16], &rollspeed, 4);
    memcpy(&payload[20], &pitchspeed, 4);
    memcpy(&payload[24], &yawspeed, 4);

    return mavlink_send_message(MAVLINK_MSG_ID_ATTITUDE, payload, 28, 39);
}

/**
 * @brief Send SCALED_IMU message
 */
static esp_err_t mavlink_send_scaled_imu(int16_t xacc, int16_t yacc, int16_t zacc,
                                         int16_t xgyro, int16_t ygyro, int16_t zgyro,
                                         int16_t xmag, int16_t ymag, int16_t zmag)
{
    // SCALED_IMU message (ID: 26, CRC_EXTRA: 170)
    uint8_t payload[22] = {0};
    uint32_t time_boot_ms = get_time_boot_ms();

    memcpy(&payload[0], &time_boot_ms, 4);
    memcpy(&payload[4], &xacc, 2);
    memcpy(&payload[6], &yacc, 2);
    memcpy(&payload[8], &zacc, 2);
    memcpy(&payload[10], &xgyro, 2);
    memcpy(&payload[12], &ygyro, 2);
    memcpy(&payload[14], &zgyro, 2);
    memcpy(&payload[16], &xmag, 2);
    memcpy(&payload[18], &ymag, 2);
    memcpy(&payload[20], &zmag, 2);

    return mavlink_send_message(MAVLINK_MSG_ID_SCALED_IMU, payload, 22, 170);
}

/**
 * @brief Send IMU data via MAVLink
 */
esp_err_t pi_mavlink_send_imu(const wt901b_data_t *imu_data)
{
    if (imu_data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Convert IMU data to MAVLink format
    // SCALED_IMU uses: acc in mG, gyro in mrad/s, mag in mgauss
    int16_t xacc = (int16_t)(imu_data->acc_x * 1000.0f); // g to mg
    int16_t yacc = (int16_t)(imu_data->acc_y * 1000.0f);
    int16_t zacc = (int16_t)(imu_data->acc_z * 1000.0f);

    int16_t xgyro = (int16_t)(imu_data->gyro_x * 17.453f); // deg/s to mrad/s
    int16_t ygyro = (int16_t)(imu_data->gyro_y * 17.453f);
    int16_t zgyro = (int16_t)(imu_data->gyro_z * 17.453f);

    int16_t xmag = (int16_t)(imu_data->mag_x * 10.0f); // gauss to mgauss
    int16_t ymag = (int16_t)(imu_data->mag_y * 10.0f);
    int16_t zmag = (int16_t)(imu_data->mag_z * 10.0f);

    // Send SCALED_IMU
    esp_err_t ret = mavlink_send_scaled_imu(xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Send ATTITUDE (roll, pitch, yaw in radians)
    float roll = imu_data->roll * 0.0174533f; // deg to rad
    float pitch = imu_data->pitch * 0.0174533f;
    float yaw = imu_data->yaw * 0.0174533f;

    float rollspeed = imu_data->gyro_x * 0.0174533f; // deg/s to rad/s
    float pitchspeed = imu_data->gyro_y * 0.0174533f;
    float yawspeed = imu_data->gyro_z * 0.0174533f;

    return mavlink_send_attitude(roll, pitch, yaw, rollspeed, pitchspeed, yawspeed);
}

/**
 * @brief Send RC_CHANNELS message
 */
esp_err_t pi_mavlink_send_rc_channels(const crsf_channels_t *channels, bool failsafe)
{
    if (channels == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // RC_CHANNELS message (ID: 65, CRC_EXTRA: 118)
    // Contains 18 channels, time_boot_ms, chancount, rssi
    uint8_t payload[42] = {0};
    uint32_t time_boot_ms = get_time_boot_ms();

    memcpy(&payload[0], &time_boot_ms, 4);

    // Copy up to 18 channels (CRSF has 16)
    for (int i = 0; i < 16 && i < 18; i++)
    {
        uint16_t chan_value = channels->channels[i];
        memcpy(&payload[4 + i * 2], &chan_value, 2);
    }

    // Fill remaining channels with 0
    for (int i = 16; i < 18; i++)
    {
        uint16_t zero = 0;
        memcpy(&payload[4 + i * 2], &zero, 2);
    }

    payload[40] = 16;                 // chancount
    payload[41] = failsafe ? 0 : 255; // rssi (0 = failsafe, 255 = perfect)

    return mavlink_send_message(MAVLINK_MSG_ID_RC_CHANNELS, payload, 42, 118);
}

/**
 * @brief Send SYS_STATUS message (system status and diagnostics)
 */
esp_err_t pi_mavlink_send_sys_status(void)
{
    // SYS_STATUS message (ID: 1, CRC_EXTRA: 124)
    uint8_t payload[31] = {0};

    // Get system info
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    // Get free heap
    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t min_heap = esp_get_minimum_free_heap_size();

    // Calculate load (percentage of heap used)
    uint32_t total_heap = 512 * 1024;                                           // Approximate total heap (512KB for ESP32-S3)
    uint16_t load = (uint16_t)(((total_heap - free_heap) * 1000) / total_heap); // d% (deci-percent)

    // Battery voltage (simulated - would need ADC reading)
    uint16_t voltage_battery = 12000; // 12.0V in mV
    int16_t current_battery = 1000;   // 10.0A in cA (centi-amps)
    int8_t battery_remaining = 80;    // 80%

    // System capabilities (all disabled for now)
    uint32_t onboard_control_sensors_present = 0;
    uint32_t onboard_control_sensors_enabled = 0;
    uint32_t onboard_control_sensors_health = 0;

    memcpy(&payload[0], &onboard_control_sensors_present, 4);
    memcpy(&payload[4], &onboard_control_sensors_enabled, 4);
    memcpy(&payload[8], &onboard_control_sensors_health, 4);
    memcpy(&payload[12], &load, 2);
    memcpy(&payload[14], &voltage_battery, 2);
    memcpy(&payload[16], &current_battery, 2);
    payload[18] = battery_remaining;

    // Drop rate comm, errors
    uint16_t drop_rate_comm = 0;
    uint16_t errors_comm = 0;
    uint16_t errors_count1 = 0;
    uint16_t errors_count2 = 0;
    uint16_t errors_count3 = 0;
    uint16_t errors_count4 = 0;

    memcpy(&payload[19], &drop_rate_comm, 2);
    memcpy(&payload[21], &errors_comm, 2);
    memcpy(&payload[23], &errors_count1, 2);
    memcpy(&payload[25], &errors_count2, 2);
    memcpy(&payload[27], &errors_count3, 2);
    memcpy(&payload[29], &errors_count4, 2);

    ESP_LOGI(TAG, "System: Load=%d.%d%%, Free Heap=%lu KB, Min Heap=%lu KB",
             load / 10, load % 10, free_heap / 1024, min_heap / 1024);

    return mavlink_send_message(MAVLINK_MSG_ID_SYS_STATUS, payload, 31, 124);
}

/**
 * @brief MAVLink task - handles periodic sending
 */
void pi_mavlink_task(void *pvParameters)
{
    ESP_LOGI(TAG, "MAVLink task started");

    TickType_t last_heartbeat = xTaskGetTickCount();
    TickType_t last_status = xTaskGetTickCount();

    while (1)
    {
        TickType_t now = xTaskGetTickCount();

        // Send heartbeat at 1 Hz
        if ((now - last_heartbeat) >= pdMS_TO_TICKS(1000))
        {
            pi_mavlink_send_heartbeat();
            last_heartbeat = now;
        }

        // Send system status at 1 Hz
        if ((now - last_status) >= pdMS_TO_TICKS(1000))
        {
            pi_mavlink_send_sys_status();
            last_status = now;
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz task rate
    }
}

/**
 * @brief Start MAVLink communication task
 */
esp_err_t pi_mavlink_start_task(void)
{
    if (mavlink_task_handle != NULL)
    {
        ESP_LOGW(TAG, "MAVLink task already running");
        return ESP_OK;
    }

    BaseType_t ret = xTaskCreate(
        pi_mavlink_task,
        "mavlink_task",
        4096,
        NULL,
        5,
        &mavlink_task_handle);

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create MAVLink task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "MAVLink task started successfully");
    return ESP_OK;
}
