#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO 7
#define I2C_MASTER_SDA_IO 6
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TIMEOUT_MS 1000

#define MPU9250_ADDR 0x68
#define AK8963_ADDR 0x0C

#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_INT_PIN_CFG 0x37
#define MPU9250_ACCEL_XOUT_H 0x3B

static const char *TAG = "GY9250";

esp_err_t i2c_master_init(void)
{
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &config));
    return i2c_driver_install(I2C_MASTER_NUM, config.mode, 0, 0, 0);
}

esp_err_t write_i2c_byte(uint8_t addr, uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, addr, buf, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

uint8_t read_i2c_byte(uint8_t addr, uint8_t reg)
{
    uint8_t data;
    i2c_master_write_read_device(I2C_MASTER_NUM, addr, &reg, 1, &data, 1, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    return data;
}

void read_accel_gyro(int16_t *accel, int16_t *gyro)
{
    uint8_t data[14];
    uint8_t reg = MPU9250_ACCEL_XOUT_H;
    i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_ADDR, &reg, 1, data, 14, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    for (int i = 0; i < 3; i++)
        accel[i] = (int16_t)(data[i * 2] << 8 | data[i * 2 + 1]);

    for (int i = 0; i < 3; i++)
        gyro[i] = (int16_t)(data[8 + i * 2] << 8 | data[9 + i * 2]);
}

void init_mpu9250(void)
{
    write_i2c_byte(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x00); // Wake up
    vTaskDelay(pdMS_TO_TICKS(100));
    write_i2c_byte(MPU9250_ADDR, MPU9250_INT_PIN_CFG, 0x02); // Enable bypass to AK8963
    vTaskDelay(pdMS_TO_TICKS(100));
}

void scan_i2c_bus()
{
    ESP_LOGI(TAG, "I2C scan...");
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Found device at 0x%02X", addr);
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());

    init_mpu9250();

    scan_i2c_bus();

    while (1)
    {
        int16_t accel[3], gyro[3];

        read_accel_gyro(accel, gyro);

        ESP_LOGI(TAG, "Accel: X=%d Y=%d Z=%d", accel[0], accel[1], accel[2]);
        ESP_LOGI(TAG, "Gyro:  X=%d Y=%d Z=%d", gyro[0], gyro[1], gyro[2]);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
