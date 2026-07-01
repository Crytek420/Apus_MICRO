#ifndef CRSF_PROTOCOL_H
#define CRSF_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* CRSF Protocol Definitions */
#define CRSF_BAUDRATE 420000
#define CRSF_FRAME_SIZE_MAX 64
#define CRSF_PAYLOAD_SIZE_MAX 60

/* CRSF Frame Structure */
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_ADDRESS_RADIO_TRANSMITTER 0xEA
#define CRSF_ADDRESS_CRSF_RECEIVER 0xEC
#define CRSF_ADDRESS_BROADCAST 0x00

/* CRSF Frame Types */
#define CRSF_FRAMETYPE_GPS 0x02
#define CRSF_FRAMETYPE_BATTERY_SENSOR 0x08
#define CRSF_FRAMETYPE_HEARTBEAT 0x0B
#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAMETYPE_ATTITUDE 0x1E
#define CRSF_FRAMETYPE_FLIGHT_MODE 0x21

/* RC Channel Configuration */
#define CRSF_CHANNEL_COUNT 16
#define CRSF_CHANNEL_VALUE_MIN 172
#define CRSF_CHANNEL_VALUE_MID 992
#define CRSF_CHANNEL_VALUE_MAX 1811

/* CRSF Frame Header */
typedef struct
{
    uint8_t device_addr; // Device address
    uint8_t frame_size;  // Length of frame (type + payload + crc)
    uint8_t type;        // Frame type
    // +1 because crsf_validate_frame() reads the trailing CRC byte out of
    // this array at index (frame_size - 2), and frame_size can be as large
    // as CRSF_PAYLOAD_SIZE_MAX + 2 (type + payload + crc).
    uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1];
} __attribute__((packed)) crsf_frame_t;

/* CRSF RC Channels Structure */
typedef struct
{
    uint16_t channels[CRSF_CHANNEL_COUNT];
} crsf_channels_t;

/* CRSF GPS Telemetry */
typedef struct
{
    int32_t latitude;     // Degree / 10,000,000
    int32_t longitude;    // Degree / 10,000,000
    uint16_t groundspeed; // km/h / 10
    uint16_t heading;     // Degree / 100
    uint16_t altitude;    // Meters - 1000m
    uint8_t satellites;   // Number of satellites
} __attribute__((packed)) crsf_gps_t;

/* CRSF Battery Telemetry */
typedef struct
{
    uint16_t voltage;  // Voltage in 0.1V (decivolts)
    uint16_t current;  // Current in 0.1A (deciampere)
    uint32_t capacity; // Capacity in mAh
    uint8_t remaining; // Remaining in percent
} __attribute__((packed)) crsf_battery_t;

/* CRSF Attitude Telemetry */
typedef struct
{
    int16_t pitch; // Degree / 10000
    int16_t roll;  // Degree / 10000
    int16_t yaw;   // Degree / 10000
} __attribute__((packed)) crsf_attitude_t;

/* CRSF Flight Mode */
typedef struct
{
    char flight_mode[16]; // Null terminated string
} __attribute__((packed)) crsf_flight_mode_t;

/* CRSF Link Statistics */
typedef struct
{
    uint8_t uplink_rssi_ant1;
    uint8_t uplink_rssi_ant2;
    uint8_t uplink_link_quality;
    int8_t uplink_snr;
    uint8_t active_antenna;
    uint8_t rf_mode;
    uint8_t uplink_tx_power;
    uint8_t downlink_rssi;
    uint8_t downlink_link_quality;
    int8_t downlink_snr;
} __attribute__((packed)) crsf_link_statistics_t;

/* Function Prototypes */

/* Core Protocol Functions */
uint8_t crsf_calculate_crc(const uint8_t *data, uint8_t length);
bool crsf_validate_frame(const crsf_frame_t *frame);
void crsf_parse_rc_channels(const uint8_t *payload, crsf_channels_t *channels);

/* Telemetry Frame Preparation */
uint8_t crsf_prepare_gps_frame(crsf_frame_t *frame, const crsf_gps_t *gps);
uint8_t crsf_prepare_battery_frame(crsf_frame_t *frame, const crsf_battery_t *battery);
uint8_t crsf_prepare_attitude_frame(crsf_frame_t *frame, const crsf_attitude_t *attitude);
uint8_t crsf_prepare_flight_mode_frame(crsf_frame_t *frame, const char *mode);

/* Communication Functions */
esp_err_t crsf_uart_init(void);
void crsf_send_telemetry(crsf_frame_t *frame, uint8_t frame_length);
bool crsf_get_channels(crsf_channels_t *channels);
bool crsf_get_link_stats(crsf_link_statistics_t *stats);

/* Task Management */
void crsf_task(void *pvParameters);
void crsf_start_task(void);

#endif // CRSF_PROTOCOL_H
