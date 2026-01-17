#ifndef WT901B_IMU_H
#define WT901B_IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief WT901B IMU Driver for ESP32-S3 via I2C
 *
 * The WT901B is a 9-axis IMU (3-axis accelerometer, 3-axis gyroscope, 3-axis magnetometer)
 * with built-in sensor fusion providing attitude angles (roll, pitch, yaw).
 *
 * I2C Connection (ESP32-S3):
 * ESP32-S3 SDA (GPIO 8)  <---> WT901B SDA
 * ESP32-S3 SCL (GPIO 9)  <---> WT901B SCL
 * 3.3V                   <---> VCC
 * GND                    <---> GND
 *
 * Default I2C Address: 0x50 (can be changed via configuration)
 */

/* I2C Configuration - Using standard I2C pins */
#define WT901B_I2C_NUM I2C_NUM_0
#define WT901B_I2C_SDA_PIN 47     // I2C SDA pin
#define WT901B_I2C_SCL_PIN 48     // I2C SCL pin
#define WT901B_I2C_FREQ_HZ 100000 // 100 kHz Standard mode (sicherer Start)
#define WT901B_I2C_ADDR 0x50      // Default address (can be 0x50-0x53)

/* WT901B Register Addresses */
#define WT901B_REG_SAVE 0x00  // Save/Reset register
#define WT901B_REG_CALSW 0x01 // Calibration switch
#define WT901B_REG_RSW 0x02   // Return rate setting
#define WT901B_REG_RATE 0x03  // Output rate setting
#define WT901B_REG_BAUD 0x04  // Baud rate setting
#define WT901B_REG_MODE 0x23  // Interface mode (UART/IIC)
#define WT901B_REG_ADDR 0x24  // IIC address setting

/* Data Registers (16-bit, low byte first) */
#define WT901B_REG_AX 0x34    // Accelerometer X
#define WT901B_REG_AY 0x35    // Accelerometer Y
#define WT901B_REG_AZ 0x36    // Accelerometer Z
#define WT901B_REG_GX 0x37    // Gyroscope X
#define WT901B_REG_GY 0x38    // Gyroscope Y
#define WT901B_REG_GZ 0x39    // Gyroscope Z
#define WT901B_REG_HX 0x3A    // Magnetometer X
#define WT901B_REG_HY 0x3B    // Magnetometer Y
#define WT901B_REG_HZ 0x3C    // Magnetometer Z
#define WT901B_REG_ROLL 0x3D  // Roll angle
#define WT901B_REG_PITCH 0x3E // Pitch angle
#define WT901B_REG_YAW 0x3F   // Yaw angle
#define WT901B_REG_TEMP 0x40  // Temperature
#define WT901B_REG_Q0 0x51    // Quaternion 0
#define WT901B_REG_Q1 0x52    // Quaternion 1
#define WT901B_REG_Q2 0x53    // Quaternion 2
#define WT901B_REG_Q3 0x54    // Quaternion 3

/* Scale Factors */
#define WT901B_ACCEL_SCALE (16.0f / 32768.0f)  // ±16g range -> g
#define WT901B_GYRO_SCALE (2000.0f / 32768.0f) // ±2000°/s range -> °/s
#define WT901B_MAG_SCALE 1.0f                  // Raw value
#define WT901B_ANGLE_SCALE (180.0f / 32768.0f) // -> degrees
#define WT901B_TEMP_SCALE (1.0f / 100.0f)      // -> °C

/* Data Structures */

/**
 * @brief Raw IMU data (16-bit signed integers)
 */
typedef struct
{
    int16_t accel_x; // Accelerometer X raw
    int16_t accel_y; // Accelerometer Y raw
    int16_t accel_z; // Accelerometer Z raw
    int16_t gyro_x;  // Gyroscope X raw
    int16_t gyro_y;  // Gyroscope Y raw
    int16_t gyro_z;  // Gyroscope Z raw
    int16_t mag_x;   // Magnetometer X raw
    int16_t mag_y;   // Magnetometer Y raw
    int16_t mag_z;   // Magnetometer Z raw
    int16_t roll;    // Roll angle raw
    int16_t pitch;   // Pitch angle raw
    int16_t yaw;     // Yaw angle raw
    int16_t temp;    // Temperature raw
} wt901b_raw_data_t;

/**
 * @brief Scaled IMU data (floating point)
 */
typedef struct
{
    float accel_x;     // Accelerometer X (g)
    float accel_y;     // Accelerometer Y (g)
    float accel_z;     // Accelerometer Z (g)
    float gyro_x;      // Gyroscope X (°/s)
    float gyro_y;      // Gyroscope Y (°/s)
    float gyro_z;      // Gyroscope Z (°/s)
    float mag_x;       // Magnetometer X (µT)
    float mag_y;       // Magnetometer Y (µT)
    float mag_z;       // Magnetometer Z (µT)
    float roll;        // Roll angle (degrees)
    float pitch;       // Pitch angle (degrees)
    float yaw;         // Yaw angle (degrees)
    float temperature; // Temperature (°C)
} wt901b_data_t;

/**
 * @brief Quaternion data
 */
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} wt901b_quaternion_t;

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
 * @brief Read raw IMU data
 * @param data Pointer to raw data structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wt901b_read_raw(wt901b_raw_data_t *data);

/**
 * @brief Read scaled IMU data
 * @param data Pointer to scaled data structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wt901b_read_data(wt901b_data_t *data);

/**
 * @brief Read attitude angles only (roll, pitch, yaw)
 * @param roll Pointer to roll angle (degrees)
 * @param pitch Pointer to pitch angle (degrees)
 * @param yaw Pointer to yaw angle (degrees)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wt901b_read_angles(float *roll, float *pitch, float *yaw);

/**
 * @brief Read quaternion data
 * @param quat Pointer to quaternion structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wt901b_read_quaternion(wt901b_quaternion_t *quat);

/**
 * @brief Calibrate accelerometer (place sensor flat and level)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wt901b_calibrate_accel(void);

/**
 * @brief Calibrate magnetometer (rotate sensor 360° in all axes)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wt901b_calibrate_mag(void);

/**
 * @brief Set output rate
 * @param rate_hz Output rate in Hz (0.1, 0.5, 1, 2, 5, 10, 20, 50, 100, 200 Hz)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wt901b_set_output_rate(uint16_t rate_hz);

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
