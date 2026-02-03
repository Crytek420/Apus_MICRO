#ifndef WT901B_IMU_H
#define WT901B_IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "REG.h"

/**
 * @brief WT901B IMU Driver for ESP32-S3 via I2C
 *
 * The WT901B is a 9-axis IMU (3-axis accelerometer, 3-axis gyroscope, 3-axis magnetometer)
 * with built-in sensor fusion providing attitude angles (roll, pitch, yaw).
 *
 * I2C Connection (ESP32-S3):
 * ESP32-S3 SDA (GPIO 47)  <---> WT901B SDA
 * ESP32-S3 SCL (GPIO 48)  <---> WT901B SCL
 * 3.3V                    <---> VCC
 * GND                     <---> GND
 *
 * Default I2C Address: 0x50 (can be changed via configuration)
 */

/* I2C Configuration */
#define WT901B_I2C_SDA_PIN 47     // I2C SDA pin (GPIO 47)
#define WT901B_I2C_SCL_PIN 48     // I2C SCL pin (GPIO 48)
#define WT901B_I2C_FREQ_HZ 400000 // 400 kHz Fast mode
#define WT901B_I2C_TIMEOUT_MS 1000
#define WT901B_I2C_ADDR 0x50 // Default address (can be 0x50-0x53)

/* Scale Factors */
#define WT901B_ACCEL_SCALE (16.0f / 32768.0f)  // ±16g range -> g
#define WT901B_GYRO_SCALE (2000.0f / 32768.0f) // ±2000°/s range -> °/s
#define WT901B_MAG_SCALE 1.0f                  // Raw value
#define WT901B_ANGLE_SCALE (180.0f / 32768.0f) // -> degrees
#define WT901B_TEMP_SCALE (1.0f / 100.0f)      // -> °C

/* Data Structures */

/**
 * @brief IMU data structure with both raw and scaled values
 */
typedef struct
{
    float acc_x, acc_y, acc_z;    // Beschleunigung in g
    float gyro_x, gyro_y, gyro_z; // Winkelgeschwindigkeit in °/s
    float mag_x, mag_y, mag_z;    // Magnetfeld in μT
    float roll, pitch, yaw;       // Winkel in Grad
    float temperature;            // Temperature in °C

    // Raw-Werte für Debugging
    int16_t acc_x_raw, acc_y_raw, acc_z_raw;
    int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
    int16_t mag_x_raw, mag_y_raw, mag_z_raw;
    int16_t roll_raw, pitch_raw, yaw_raw;
    int16_t temp_raw;
} wt901b_data_t;

/* Function Prototypes */

/**
 * @brief Initialize WT901B IMU
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wt901b_init(void);

/**
 * @brief Deinitialize WT901B IMU
 * @return ESP_OK on success
 */
esp_err_t wt901b_deinit(void);

/**
 * @brief Read all sensor data from WT901B
 * @param imu_data Pointer to data structure to fill
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wt901b_read_data(wt901b_data_t *imu_data);

/**
 * @brief Check if IMU is connected and responding
 * @return true if connected, false otherwise
 */
bool wt901b_is_connected(void);

/**
 * @brief Get current IMU data (cached from last read)
 * @param data Pointer to data structure to fill
 * @return true if data is valid, false otherwise
 */
bool wt901b_get_data(wt901b_data_t *data);

/**
 * @brief Start IMU reading task
 */
void wt901b_start_task(void);

#endif // WT901B_IMU_H
