#include "wt901b_imu.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "WT901B";

#define LOG_LOCAL_LEVEL ESP_LOG_NONE

/* Global Variables */
static wt901b_data_t g_imu_data = {0};
static bool g_data_valid = false;
static SemaphoreHandle_t g_data_mutex = NULL;
static bool g_initialized = false;

/**
 * @brief Read bytes from WT901B register
 */
static esp_err_t wt901b_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Write register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (WT901B_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    // Read data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (WT901B_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1)
    {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &data[len - 1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(WT901B_I2C_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief Write bytes to WT901B register
 */
static esp_err_t wt901b_write_bytes(uint8_t reg_addr, const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (WT901B_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(WT901B_I2C_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief Write a single word (16-bit) to WT901B register
 */
static esp_err_t wt901b_write_word(uint8_t reg_addr, uint16_t value)
{
    uint8_t data[2] = {value & 0xFF, (value >> 8) & 0xFF};
    return wt901b_write_bytes(reg_addr, data, 2);
}

/**
 * @brief Read a single word (16-bit) from WT901B register
 */
static esp_err_t wt901b_read_word(uint8_t reg_addr, int16_t *value)
{
    uint8_t data[2];
    esp_err_t ret = wt901b_read_bytes(reg_addr, data, 2);
    if (ret == ESP_OK)
    {
        *value = (int16_t)(data[0] | (data[1] << 8));
    }
    return ret;
}

/**
 * @brief Initialize WT901B IMU
 */
esp_err_t wt901b_init(void)
{
    if (g_initialized)
    {
        ESP_LOGW(TAG, "WT901B already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing WT901B IMU on I2C%d", WT901B_I2C_NUM);
    ESP_LOGI(TAG, "SDA=GPIO%d, SCL=GPIO%d, Address=0x%02X",
             WT901B_I2C_SDA_PIN, WT901B_I2C_SCL_PIN, WT901B_I2C_ADDR);

    // Configure I2C
    // WICHTIG: Externe 4.7kΩ Pull-ups an SDA und SCL zu VCC erforderlich!
    // Interne Pull-ups sind laut Datenblatt nicht ausreichend
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = WT901B_I2C_SDA_PIN,
        .scl_io_num = WT901B_I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = WT901B_I2C_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(WT901B_I2C_NUM, &conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure I2C: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(WT901B_I2C_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create mutex for data access
    g_data_mutex = xSemaphoreCreateMutex();
    if (g_data_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        i2c_driver_delete(WT901B_I2C_NUM);
        return ESP_FAIL;
    }

    // Give the sensor time to boot (WT901B benötigt Zeit nach Power-On)
    vTaskDelay(pdMS_TO_TICKS(500));

    // Scan I2C bus for devices
    ESP_LOGI(TAG, "Scanning I2C bus...");
    int devices_found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(WT901B_I2C_NUM, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Found I2C device at address 0x%02X", addr);
            devices_found++;
        }
    }
    ESP_LOGI(TAG, "I2C scan complete. Found %d device(s)", devices_found);

    // Check if sensor is connected
    if (!wt901b_is_connected())
    {
        ESP_LOGE(TAG, "WT901B not found at address 0x%02X", WT901B_I2C_ADDR);
        ESP_LOGE(TAG, "Check wiring: SDA=GPIO%d, SCL=GPIO%d", WT901B_I2C_SDA_PIN, WT901B_I2C_SCL_PIN);
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "Mögliche Ursachen:");
        ESP_LOGE(TAG, "1. WT901B ist im UART-Modus (nicht I2C-Modus)");
        ESP_LOGE(TAG, "   -> Prüfen Sie Jumper/Schalter am Modul");
        ESP_LOGE(TAG, "2. Fehlende externe 4.7kΩ Pull-up Widerstände");
        ESP_LOGE(TAG, "   -> SDA und SCL benötigen Pull-ups zu VCC (3.3V)");
        ESP_LOGE(TAG, "3. Falsche I2C-Adresse (versuchen Sie 0x51, 0x52, 0x53)");
        ESP_LOGE(TAG, "4. Lockere/falsche Verkabelung oder defektes Modul");
        i2c_driver_delete(WT901B_I2C_NUM);
        vSemaphoreDelete(g_data_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    g_initialized = true;
    ESP_LOGI(TAG, "WT901B initialized successfully");

    return ESP_OK;
}

/**
 * @brief Deinitialize WT901B IMU
 */
esp_err_t wt901b_deinit(void)
{
    if (!g_initialized)
    {
        return ESP_OK;
    }

    i2c_driver_delete(WT901B_I2C_NUM);

    if (g_data_mutex != NULL)
    {
        vSemaphoreDelete(g_data_mutex);
        g_data_mutex = NULL;
    }

    g_initialized = false;
    g_data_valid = false;

    ESP_LOGI(TAG, "WT901B deinitialized");
    return ESP_OK;
}

/**
 * @brief Check if IMU is connected
 */
bool wt901b_is_connected(void)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (WT901B_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(WT901B_I2C_NUM, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK);
}

/**
 * @brief Read raw IMU data
 */
esp_err_t wt901b_read_raw(wt901b_raw_data_t *data)
{
    if (!g_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    // Read accelerometer (3 words starting at AX)
    uint8_t accel_data[6];
    ret = wt901b_read_bytes(WT901B_REG_AX, accel_data, 6);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to read accelerometer: %s", esp_err_to_name(ret));
        return ret;
    }
    data->accel_x = (int16_t)(accel_data[0] | (accel_data[1] << 8));
    data->accel_y = (int16_t)(accel_data[2] | (accel_data[3] << 8));
    data->accel_z = (int16_t)(accel_data[4] | (accel_data[5] << 8));

    // Read gyroscope (3 words starting at GX)
    uint8_t gyro_data[6];
    ret = wt901b_read_bytes(WT901B_REG_GX, gyro_data, 6);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to read gyroscope: %s", esp_err_to_name(ret));
        return ret;
    }
    data->gyro_x = (int16_t)(gyro_data[0] | (gyro_data[1] << 8));
    data->gyro_y = (int16_t)(gyro_data[2] | (gyro_data[3] << 8));
    data->gyro_z = (int16_t)(gyro_data[4] | (gyro_data[5] << 8));

    // Read magnetometer (3 words starting at HX)
    uint8_t mag_data[6];
    ret = wt901b_read_bytes(WT901B_REG_HX, mag_data, 6);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to read magnetometer: %s", esp_err_to_name(ret));
        return ret;
    }
    data->mag_x = (int16_t)(mag_data[0] | (mag_data[1] << 8));
    data->mag_y = (int16_t)(mag_data[2] | (mag_data[3] << 8));
    data->mag_z = (int16_t)(mag_data[4] | (mag_data[5] << 8));

    // Read attitude angles (3 words starting at Roll)
    uint8_t angle_data[6];
    ret = wt901b_read_bytes(WT901B_REG_ROLL, angle_data, 6);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to read angles: %s", esp_err_to_name(ret));
        return ret;
    }
    data->roll = (int16_t)(angle_data[0] | (angle_data[1] << 8));
    data->pitch = (int16_t)(angle_data[2] | (angle_data[3] << 8));
    data->yaw = (int16_t)(angle_data[4] | (angle_data[5] << 8));

    // Read temperature
    ret = wt901b_read_word(WT901B_REG_TEMP, &data->temp);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to read temperature: %s", esp_err_to_name(ret));
        // Temperature is optional, don't fail
        data->temp = 0;
    }

    return ESP_OK;
}

/**
 * @brief Read scaled IMU data
 */
esp_err_t wt901b_read_data(wt901b_data_t *data)
{
    wt901b_raw_data_t raw;
    esp_err_t ret = wt901b_read_raw(&raw);

    if (ret != ESP_OK)
    {
        return ret;
    }

    // Convert to scaled values
    data->accel_x = raw.accel_x * WT901B_ACCEL_SCALE;
    data->accel_y = raw.accel_y * WT901B_ACCEL_SCALE;
    data->accel_z = raw.accel_z * WT901B_ACCEL_SCALE;

    data->gyro_x = raw.gyro_x * WT901B_GYRO_SCALE;
    data->gyro_y = raw.gyro_y * WT901B_GYRO_SCALE;
    data->gyro_z = raw.gyro_z * WT901B_GYRO_SCALE;

    data->mag_x = raw.mag_x * WT901B_MAG_SCALE;
    data->mag_y = raw.mag_y * WT901B_MAG_SCALE;
    data->mag_z = raw.mag_z * WT901B_MAG_SCALE;

    data->roll = raw.roll * WT901B_ANGLE_SCALE;
    data->pitch = raw.pitch * WT901B_ANGLE_SCALE;
    data->yaw = raw.yaw * WT901B_ANGLE_SCALE;

    data->temperature = raw.temp * WT901B_TEMP_SCALE;

    return ESP_OK;
}

/**
 * @brief Read attitude angles only
 */
esp_err_t wt901b_read_angles(float *roll, float *pitch, float *yaw)
{
    if (!g_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t angle_data[6];
    esp_err_t ret = wt901b_read_bytes(WT901B_REG_ROLL, angle_data, 6);

    if (ret != ESP_OK)
    {
        return ret;
    }

    int16_t raw_roll = (int16_t)(angle_data[0] | (angle_data[1] << 8));
    int16_t raw_pitch = (int16_t)(angle_data[2] | (angle_data[3] << 8));
    int16_t raw_yaw = (int16_t)(angle_data[4] | (angle_data[5] << 8));

    if (roll)
        *roll = raw_roll * WT901B_ANGLE_SCALE;
    if (pitch)
        *pitch = raw_pitch * WT901B_ANGLE_SCALE;
    if (yaw)
        *yaw = raw_yaw * WT901B_ANGLE_SCALE;

    return ESP_OK;
}

/**
 * @brief Read quaternion data
 */
esp_err_t wt901b_read_quaternion(wt901b_quaternion_t *quat)
{
    if (!g_initialized || quat == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t quat_data[8];
    esp_err_t ret = wt901b_read_bytes(WT901B_REG_Q0, quat_data, 8);

    if (ret != ESP_OK)
    {
        return ret;
    }

    int16_t q0 = (int16_t)(quat_data[0] | (quat_data[1] << 8));
    int16_t q1 = (int16_t)(quat_data[2] | (quat_data[3] << 8));
    int16_t q2 = (int16_t)(quat_data[4] | (quat_data[5] << 8));
    int16_t q3 = (int16_t)(quat_data[6] | (quat_data[7] << 8));

    // Quaternion scale is 1/32768
    quat->q0 = q0 / 32768.0f;
    quat->q1 = q1 / 32768.0f;
    quat->q2 = q2 / 32768.0f;
    quat->q3 = q3 / 32768.0f;

    return ESP_OK;
}

/**
 * @brief Calibrate accelerometer
 */
esp_err_t wt901b_calibrate_accel(void)
{
    if (!g_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting accelerometer calibration - keep sensor flat and still!");

    // Write calibration command: 0x01 for accelerometer calibration
    uint8_t cal_cmd[2] = {0x01, 0x00};
    esp_err_t ret = wt901b_write_bytes(WT901B_REG_CALSW, cal_cmd, 2);

    if (ret == ESP_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(3000)); // Wait for calibration
        ESP_LOGI(TAG, "Accelerometer calibration complete");

        // Save calibration
        uint8_t save_cmd[2] = {0x00, 0x00};
        wt901b_write_bytes(WT901B_REG_SAVE, save_cmd, 2);
    }

    return ret;
}

/**
 * @brief Calibrate magnetometer
 */
esp_err_t wt901b_calibrate_mag(void)
{
    if (!g_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting magnetometer calibration - rotate sensor in all axes!");

    // Write calibration command: 0x02 for magnetometer calibration start
    uint8_t cal_cmd[2] = {0x02, 0x00};
    esp_err_t ret = wt901b_write_bytes(WT901B_REG_CALSW, cal_cmd, 2);

    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Rotate the sensor 360° in all directions for 30 seconds...");
        vTaskDelay(pdMS_TO_TICKS(30000)); // Wait for user to rotate sensor

        // End calibration
        uint8_t end_cmd[2] = {0x00, 0x00};
        wt901b_write_bytes(WT901B_REG_CALSW, end_cmd, 2);

        ESP_LOGI(TAG, "Magnetometer calibration complete");

        // Save calibration
        uint8_t save_cmd[2] = {0x00, 0x00};
        wt901b_write_bytes(WT901B_REG_SAVE, save_cmd, 2);
    }

    return ret;
}

/**
 * @brief Set output rate
 */
esp_err_t wt901b_set_output_rate(uint16_t rate_hz)
{
    if (!g_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    // WT901B rate settings:
    // 0x01=0.1Hz, 0x02=0.5Hz, 0x03=1Hz, 0x04=2Hz, 0x05=5Hz
    // 0x06=10Hz, 0x07=20Hz, 0x08=50Hz, 0x09=100Hz, 0x0A=200Hz, 0x0B=Single
    uint8_t rate_code;

    if (rate_hz >= 200)
        rate_code = 0x0A;
    else if (rate_hz >= 100)
        rate_code = 0x09;
    else if (rate_hz >= 50)
        rate_code = 0x08;
    else if (rate_hz >= 20)
        rate_code = 0x07;
    else if (rate_hz >= 10)
        rate_code = 0x06;
    else if (rate_hz >= 5)
        rate_code = 0x05;
    else if (rate_hz >= 2)
        rate_code = 0x04;
    else if (rate_hz >= 1)
        rate_code = 0x03;
    else
        rate_code = 0x03; // Default to 1Hz

    ESP_LOGI(TAG, "Setting output rate to %d Hz (code 0x%02X)", rate_hz, rate_code);

    return wt901b_write_word(WT901B_REG_RATE, rate_code);
}

/**
 * @brief Get current IMU data (cached)
 */
bool wt901b_get_data(wt901b_data_t *data)
{
    if (!g_data_valid || data == NULL || g_data_mutex == NULL)
    {
        return false;
    }

    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        memcpy(data, &g_imu_data, sizeof(wt901b_data_t));
        xSemaphoreGive(g_data_mutex);
        return true;
    }

    return false;
}

/**
 * @brief IMU reading task with reconnection logic
 */
static void wt901b_task(void *pvParameters)
{
    ESP_LOGI(TAG, "WT901B task started (100 Hz)");

    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t log_counter = 0;
    uint32_t error_counter = 0;
    bool was_connected = true;

    while (1)
    {
        wt901b_data_t data;
        esp_err_t ret = wt901b_read_data(&data);

        if (ret == ESP_OK)
        {
            // Reset error counter on successful read
            if (error_counter > 0)
            {
                ESP_LOGI(TAG, "IMU reconnected successfully");
                error_counter = 0;
                was_connected = true;
            }

            // Update cached data
            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                memcpy(&g_imu_data, &data, sizeof(wt901b_data_t));
                g_data_valid = true;
                xSemaphoreGive(g_data_mutex);
            }

            // Log data at 1 Hz
            if (log_counter % 100 == 0)
            {
                ESP_LOGI(TAG, "Attitude: Roll=%.1f° Pitch=%.1f° Yaw=%.1f°",
                         data.roll, data.pitch, data.yaw);
                ESP_LOGD(TAG, "Accel: X=%.2fg Y=%.2fg Z=%.2fg",
                         data.accel_x, data.accel_y, data.accel_z);
                ESP_LOGD(TAG, "Gyro: X=%.1f°/s Y=%.1f°/s Z=%.1f°/s",
                         data.gyro_x, data.gyro_y, data.gyro_z);
            }
            log_counter++;
        }
        else
        {
            // Communication error
            error_counter++;
            g_data_valid = false;

            // Try to reconnect after 10 consecutive failures
            if (error_counter == 10 && was_connected)
            {
                ESP_LOGW(TAG, "IMU communication lost, attempting reconnection...");
                was_connected = false;
            }
            else if (error_counter >= 100) // Every 100 failures (1 second at 100Hz)
            {
                // Check if IMU is back online
                if (wt901b_is_connected())
                {
                    ESP_LOGI(TAG, "IMU detected, re-initializing...");

                    // Reinitialize I2C communication (soft reset)
                    vTaskDelay(pdMS_TO_TICKS(50));

                    error_counter = 0; // Reset counter to retry reading
                }
                else
                {
                    if (log_counter % 100 == 0)
                    {
                        ESP_LOGW(TAG, "IMU still disconnected, retrying...");
                    }
                    error_counter = 50; // Keep trying but avoid overflow
                }
            }
            else if (error_counter <= 10 && log_counter % 100 == 0)
            {
                ESP_LOGW(TAG, "Failed to read IMU data (error %lu)", error_counter);
            }
        }

        // Run at 100 Hz (10ms period)
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
        log_counter++;
    }
}

/**
 * @brief Start IMU reading task
 */
void wt901b_start_task(void)
{
    if (!g_initialized)
    {
        ESP_LOGE(TAG, "Cannot start task - IMU not initialized");
        return;
    }

    xTaskCreate(wt901b_task, "wt901b_task", 4096, NULL, 4, NULL);
}
