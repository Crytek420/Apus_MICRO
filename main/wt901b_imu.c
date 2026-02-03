#include "wt901b_imu.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

static const char *TAG = "WT901B";

/* Global Variables */
static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t dev_handle = NULL;
static wt901b_data_t g_imu_data = {0};
static bool g_data_valid = false;
static SemaphoreHandle_t g_data_mutex = NULL;
static bool g_initialized = false;
static TaskHandle_t g_task_handle = NULL;

/**
 * @brief Liest ein Register vom WT901B
 */
static esp_err_t wt901b_read_register(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, WT901B_I2C_TIMEOUT_MS);
}

/**
 * @brief Liest einen 16-bit Wert vom WT901B (Low und High Byte)
 */
static int16_t wt901b_read_int16(uint8_t reg_low)
{
    uint8_t data[2];
    if (wt901b_read_register(reg_low, data, 2) == ESP_OK)
    {
        return (int16_t)(data[0] | (data[1] << 8));
    }
    return 0;
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

    ESP_LOGI(TAG, "ESP32-S3 WT901B IMU Initialization");
    ESP_LOGI(TAG, "SDA: GPIO%d, SCL: GPIO%d", WT901B_I2C_SDA_PIN, WT901B_I2C_SCL_PIN);

    // I2C Master Bus initialisieren
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = WT901B_I2C_SCL_PIN,
        .sda_io_num = WT901B_I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };

    esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Device Handle für WT901B erstellen
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = WT901B_I2C_ADDR,
        .scl_speed_hz = WT901B_I2C_FREQ_HZ,
    };

    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        i2c_del_master_bus(bus_handle);
        bus_handle = NULL;
        return ret;
    }

    // Create mutex for data access
    g_data_mutex = xSemaphoreCreateMutex();
    if (g_data_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        i2c_master_bus_rm_device(dev_handle);
        i2c_del_master_bus(bus_handle);
        dev_handle = NULL;
        bus_handle = NULL;
        return ESP_FAIL;
    }

    // Give the sensor time to boot
    vTaskDelay(pdMS_TO_TICKS(100));

    // WT901B Verbindung testen
    ret = i2c_master_probe(bus_handle, WT901B_I2C_ADDR, WT901B_I2C_TIMEOUT_MS);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "WT901B gefunden bei Adresse 0x%02X", WT901B_I2C_ADDR);
    }
    else
    {
        ESP_LOGE(TAG, "WT901B nicht gefunden!");
        ESP_LOGE(TAG, "Check wiring: SDA=GPIO%d, SCL=GPIO%d", WT901B_I2C_SDA_PIN, WT901B_I2C_SCL_PIN);
        i2c_master_bus_rm_device(dev_handle);
        i2c_del_master_bus(bus_handle);
        vSemaphoreDelete(g_data_mutex);
        dev_handle = NULL;
        bus_handle = NULL;
        g_data_mutex = NULL;
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

    // Stop task if running
    if (g_task_handle != NULL)
    {
        vTaskDelete(g_task_handle);
        g_task_handle = NULL;
    }

    // Clean up I2C
    if (dev_handle != NULL)
    {
        i2c_master_bus_rm_device(dev_handle);
        dev_handle = NULL;
    }

    if (bus_handle != NULL)
    {
        i2c_del_master_bus(bus_handle);
        bus_handle = NULL;
    }

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
    if (!g_initialized || bus_handle == NULL)
    {
        return false;
    }

    esp_err_t ret = i2c_master_probe(bus_handle, WT901B_I2C_ADDR, WT901B_I2C_TIMEOUT_MS);
    return (ret == ESP_OK);
}

/**
 * @brief Liest alle Sensordaten vom WT901B
 */
esp_err_t wt901b_read_data(wt901b_data_t *imu_data)
{
    if (!g_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (imu_data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Beschleunigung lesen
    imu_data->acc_x_raw = wt901b_read_int16(AX);
    imu_data->acc_y_raw = wt901b_read_int16(AY);
    imu_data->acc_z_raw = wt901b_read_int16(AZ);

    // WT901B verwendet 32768 = 16g
    imu_data->acc_x = imu_data->acc_x_raw / 32768.0f * 16.0f;
    imu_data->acc_y = imu_data->acc_y_raw / 32768.0f * 16.0f;
    imu_data->acc_z = imu_data->acc_z_raw / 32768.0f * 16.0f;

    // Gyroskop lesen
    imu_data->gyro_x_raw = wt901b_read_int16(GX);
    imu_data->gyro_y_raw = wt901b_read_int16(GY);
    imu_data->gyro_z_raw = wt901b_read_int16(GZ);

    // WT901B verwendet 32768 = 2000°/s
    imu_data->gyro_x = imu_data->gyro_x_raw / 32768.0f * 2000.0f;
    imu_data->gyro_y = imu_data->gyro_y_raw / 32768.0f * 2000.0f;
    imu_data->gyro_z = imu_data->gyro_z_raw / 32768.0f * 2000.0f;

    // Magnetometer lesen
    imu_data->mag_x_raw = wt901b_read_int16(MAG_X);
    imu_data->mag_y_raw = wt901b_read_int16(MAG_Y);
    imu_data->mag_z_raw = wt901b_read_int16(MAG_Z);

    imu_data->mag_x = imu_data->mag_x_raw;
    imu_data->mag_y = imu_data->mag_y_raw;
    imu_data->mag_z = imu_data->mag_z_raw;

    // Winkel lesen (180° = 32768)
    imu_data->roll_raw = wt901b_read_int16(Roll);
    imu_data->pitch_raw = wt901b_read_int16(Pitch);
    imu_data->yaw_raw = wt901b_read_int16(Yaw);

    imu_data->roll = imu_data->roll_raw / 32768.0f * 180.0f;
    imu_data->pitch = imu_data->pitch_raw / 32768.0f * 180.0f;
    imu_data->yaw = imu_data->yaw_raw / 32768.0f * 180.0f;

    // Temperature lesen
    imu_data->temp_raw = wt901b_read_int16(TEMP);
    imu_data->temperature = imu_data->temp_raw / 100.0f;

    return ESP_OK;
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
 * @brief IMU reading task
 */
static void wt901b_task(void *pvParameters)
{
    ESP_LOGI(TAG, "WT901B task started (10 Hz)");

    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t log_counter = 0;
    uint32_t error_counter = 0;

    while (1)
    {
        wt901b_data_t data;
        esp_err_t ret = wt901b_read_data(&data);

        if (ret == ESP_OK)
        {
            error_counter = 0;

            // Update cached data
            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                memcpy(&g_imu_data, &data, sizeof(wt901b_data_t));
                g_data_valid = true;
                xSemaphoreGive(g_data_mutex);
            }

            // Log data at 1 Hz
            if (log_counter % 10 == 0)
            {
                ESP_LOGI(TAG, "Attitude: Roll=%.1f° Pitch=%.1f° Yaw=%.1f°",
                         data.roll, data.pitch, data.yaw);
                ESP_LOGD(TAG, "Accel: X=%.2fg Y=%.2fg Z=%.2fg",
                         data.acc_x, data.acc_y, data.acc_z);
                ESP_LOGD(TAG, "Gyro: X=%.1f°/s Y=%.1f°/s Z=%.1f°/s",
                         data.gyro_x, data.gyro_y, data.gyro_z);
            }
            log_counter++;
        }
        else
        {
            error_counter++;
            g_data_valid = false;

            if (error_counter > 10)
            {
                ESP_LOGW(TAG, "IMU communication lost");
                error_counter = 0;
            }
        }

        // Run at 10 Hz (100ms period)
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
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

    if (g_task_handle != NULL)
    {
        ESP_LOGW(TAG, "Task already running");
        return;
    }

    xTaskCreate(wt901b_task, "wt901b_task", 4096, NULL, 4, &g_task_handle);
}
