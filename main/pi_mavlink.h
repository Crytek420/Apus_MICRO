#ifndef PI_MAVLINK_H
#define PI_MAVLINK_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "crsf_protocol.h"
#include "wt901b_imu.h"

/**
 * @brief MAVLink Configuration for Raspberry Pi Zero 2W Communication
 *
 * PIN Connections (ESP32-S3-DEV-KIT-NXR8):
 * ESP32-S3 TX (GPIO 15) <---> Raspberry Pi Zero 2W RX (GPIO 15 / UART0 RX)
 * ESP32-S3 RX (GPIO 16) <---> Raspberry Pi Zero 2W TX (GPIO 14 / UART0 TX)
 * GND                   <---> GND
 *
 * Note: ESP32-S3 has UART0, UART1 and UART2
 * UART0 (USB-CDC) is used for debugging/flashing via USB
 * UART1 (GPIO 17/18) is used for CRSF/ELRS receiver (Radiomaster XR1)
 * UART2 (GPIO 15/16) is used for Pi Zero 2W MAVLink communication
 */

/* UART Configuration */
#define PI_UART_NUM UART_NUM_2
#define PI_UART_TX_PIN 15 // No ADC function, safe to use
#define PI_UART_RX_PIN 16 // No ADC function, safe to use
#define PI_UART_BAUDRATE 115200
#define PI_UART_BUF_SIZE 2048

/* MAVLink Configuration */
#define MAVLINK_SYSTEM_ID 1           // ESP32 is system 1
#define MAVLINK_COMPONENT_ID 1        // Main flight controller component
#define MAVLINK_HEARTBEAT_RATE_HZ 1   // 1 Hz
#define MAVLINK_IMU_RATE_HZ 20        // 20 Hz
#define MAVLINK_RC_RATE_HZ 20         // 20 Hz
#define MAVLINK_STATUS_RATE_HZ 1      // 1 Hz

/**
 * @brief Initialize MAVLink communication with Raspberry Pi
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pi_mavlink_init(void);

/**
 * @brief Deinitialize MAVLink communication
 * @return ESP_OK on success
 */
esp_err_t pi_mavlink_deinit(void);

/**
 * @brief Send IMU data via MAVLink
 * @param imu_data Pointer to IMU data structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pi_mavlink_send_imu(const wt901b_data_t *imu_data);

/**
 * @brief Send RC channel data via MAVLink
 * @param channels Pointer to CRSF channels structure
 * @param failsafe True if in failsafe mode
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pi_mavlink_send_rc_channels(const crsf_channels_t *channels, bool failsafe);

/**
 * @brief Send system status via MAVLink
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pi_mavlink_send_sys_status(void);

/**
 * @brief Send heartbeat via MAVLink
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pi_mavlink_send_heartbeat(void);

/**
 * @brief MAVLink task - handles periodic sending and receiving
 * @param pvParameters Task parameters (unused)
 */
void pi_mavlink_task(void *pvParameters);

/**
 * @brief Start MAVLink communication task
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pi_mavlink_start_task(void);

/**
 * @brief Send ESP32 hardware stats: die temperature, free heap, main-loop cycle time
 * @param temp_c        Die temperature in °C (from internal temperature sensor)
 * @param free_heap_kb  Free heap in KB
 * @param cycle_us      Main loop body duration in microseconds
 * @return ESP_OK on success
 */
esp_err_t pi_mavlink_send_esp_stats(float temp_c, float free_heap_kb, float cycle_us);

#endif // PI_MAVLINK_H
