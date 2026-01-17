#ifndef PI_UART_H
#define PI_UART_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "crsf_protocol.h"

/**
 * @brief UART Configuration for Raspberry Pi Zero 2W Communication
 *
 * PIN Connections (ESP32-S3-DEV-KIT-NXR8):
 * ESP32-S3 TX (GPIO 15) <---> Raspberry Pi Zero 2W RX (GPIO 15 / UART0 RX)
 * ESP32-S3 RX (GPIO 16) <---> Raspberry Pi Zero 2W TX (GPIO 14 / UART0 TX)
 * GND                   <---> GND
 *
 * Note: ESP32-S3 has UART0, UART1 and UART2
 * UART0 (USB-CDC) is used for debugging/flashing via USB
 * UART1 (GPIO 17/18) is used for CRSF/ELRS receiver (Radiomaster XR1)
 * UART2 (GPIO 15/16) is used for Pi Zero 2W communication
 */

/* UART Configuration */
#define PI_UART_NUM UART_NUM_2
#define PI_UART_TX_PIN 15 // No ADC function, safe to use
#define PI_UART_RX_PIN 16 // No ADC function, safe to use
#define PI_UART_BAUDRATE 115200
#define PI_UART_BUF_SIZE 512

/* Data packet structure for Pi communication */
typedef struct
{
    uint8_t header[2];     // 0xAA, 0x55 - Frame start marker
    uint16_t channels[16]; // CRSF channel data (16 channels)
    uint8_t checksum;      // Simple XOR checksum
} __attribute__((packed)) pi_data_packet_t;

/**
 * @brief Initialize UART interface for Raspberry Pi Zero 2W communication
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pi_uart_init(void);

/**
 * @brief Send CRSF channel data to Raspberry Pi Zero 2W
 *
 * @param channels Pointer to CRSF channels structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pi_uart_send_channels(const crsf_channels_t *channels);

/**
 * @brief Check if valid data has been received from Pi (optional)
 *
 * @param buffer Buffer to store received data
 * @param max_len Maximum buffer length
 * @return Number of bytes received, 0 if no data
 */
int pi_uart_receive(uint8_t *buffer, size_t max_len);

/**
 * @brief Send a string to Raspberry Pi Zero 2W
 *
 * @param str Null-terminated string to send
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pi_uart_send_string(const char *str);

#endif // PI_UART_H
