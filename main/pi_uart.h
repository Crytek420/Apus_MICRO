#ifndef PI_UART_H
#define PI_UART_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "crsf_protocol.h"
#include "wt901b_imu.h"

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
#define PI_UART_BUF_SIZE 1024

/* Protocol Definition */
#define PI_PACKET_SYNC1 0xAA
#define PI_PACKET_SYNC2 0x55
#define PI_PACKET_MAX_PAYLOAD 128

/* Packet Types */
typedef enum {
    PI_PACKET_TYPE_IMU = 0x01,          // IMU sensor data
    PI_PACKET_TYPE_CRSF_CHANNELS = 0x02, // CRSF RC channels
    PI_PACKET_TYPE_CRSF_LINK_STATS = 0x03, // CRSF link statistics
    PI_PACKET_TYPE_SYSTEM_INFO = 0x04,   // ESP32 system info (voltage, temp)
    PI_PACKET_TYPE_HEARTBEAT = 0x05,     // Heartbeat/keepalive
} pi_packet_type_t;

/* Generic Packet Structure */
typedef struct {
    uint8_t sync1;      // 0xAA
    uint8_t sync2;      // 0x55
    uint8_t type;       // Packet type
    uint8_t length;     // Payload length
    uint8_t payload[PI_PACKET_MAX_PAYLOAD]; // Variable payload
    uint16_t crc16;     // CRC16 checksum (appended after payload)
} __attribute__((packed)) pi_packet_t;

/* IMU Data Payload */
typedef struct {
    float acc_x, acc_y, acc_z;       // Acceleration (g)
    float gyro_x, gyro_y, gyro_z;    // Angular velocity (°/s)
    float mag_x, mag_y, mag_z;       // Magnetic field (μT)
    float roll, pitch, yaw;          // Attitude angles (degrees)
    float temperature;                // Temperature (°C)
    uint32_t timestamp_ms;           // Timestamp in milliseconds
} __attribute__((packed)) pi_imu_payload_t;

/* CRSF Channels Payload */
typedef struct {
    uint16_t channels[16];           // CRSF channel values (172-1811)
    uint32_t timestamp_ms;           // Timestamp in milliseconds
    uint8_t failsafe;                // Failsafe status (0=ok, 1=failsafe)
} __attribute__((packed)) pi_crsf_channels_payload_t;

/* CRSF Link Statistics Payload */
typedef struct {
    uint8_t uplink_rssi_ant1;        // Uplink RSSI antenna 1 (dBm)
    uint8_t uplink_rssi_ant2;        // Uplink RSSI antenna 2 (dBm)
    uint8_t uplink_link_quality;     // Uplink link quality (%)
    int8_t uplink_snr;               // Uplink SNR (dB)
    uint8_t active_antenna;          // Active antenna
    uint8_t rf_mode;                 // RF mode
    uint8_t uplink_tx_power;         // Uplink TX power
    uint8_t downlink_rssi;           // Downlink RSSI (dBm)
    uint8_t downlink_link_quality;   // Downlink link quality (%)
    int8_t downlink_snr;             // Downlink SNR (dB)
    uint32_t timestamp_ms;           // Timestamp in milliseconds
} __attribute__((packed)) pi_link_stats_payload_t;

/* System Info Payload */
typedef struct {
    float cpu_temp;                  // CPU temperature (°C)
    float supply_voltage;            // Supply voltage (V)
    uint32_t free_heap;              // Free heap memory (bytes)
    uint32_t uptime_ms;              // System uptime (milliseconds)
    uint8_t cpu_usage;               // CPU usage (%)
    uint32_t timestamp_ms;           // Timestamp in milliseconds
} __attribute__((packed)) pi_system_info_payload_t;

/* Heartbeat Payload */
typedef struct {
    uint32_t sequence;               // Sequence number
    uint32_t timestamp_ms;           // Timestamp in milliseconds
} __attribute__((packed)) pi_heartbeat_payload_t;

/**
 * @brief Initialize UART interface for Raspberry Pi Zero 2W communication
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pi_uart_init(void);

/**
 * @brief Send IMU data to Raspberry Pi
 *
 * @param imu_data Pointer to IMU data structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pi_uart_send_imu(const wt901b_data_t *imu_data);

/**
 * @brief Send CRSF channel data to Raspberry Pi Zero 2W
 *
 * @param channels Pointer to CRSF channels structure
 * @param failsafe Failsafe status (true if in failsafe)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pi_uart_send_crsf_channels(const crsf_channels_t *channels, bool failsafe);

/**
 * @brief Send CRSF link statistics to Raspberry Pi
 *
 * @param stats Pointer to link statistics structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pi_uart_send_link_stats(const crsf_link_statistics_t *stats);

/**
 * @brief Send system information to Raspberry Pi
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pi_uart_send_system_info(void);

/**
 * @brief Send heartbeat to Raspberry Pi
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pi_uart_send_heartbeat(void);

/**
 * @brief Receive data from Raspberry Pi (deprecated - kept for compatibility)
 *
 * @param buffer Buffer to store received data
 * @param max_len Maximum length to read
 * @return Number of bytes received
 */
int pi_uart_receive(uint8_t *buffer, size_t max_len);

/**
 * @brief Send a string to Raspberry Pi (deprecated - use structured packets)
 *
 * @param str String to send
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pi_uart_send_string(const char *str);

#endif // PI_UART_H
