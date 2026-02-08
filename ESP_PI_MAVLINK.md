# ESP32-S3 to Raspberry Pi MAVLink Communication

This document describes the MAVLink-based telemetry protocol used for communication between the ESP32-S3 flight controller and the Raspberry Pi Zero 2W.

## Overview

The communication system uses the **MAVLink v1 protocol** over UART to provide reliable, standardized telemetry data exchange. MAVLink is widely used in drone systems and provides excellent compatibility with ground control stations and analysis tools.

## Hardware Connection

### Pin Mapping

```
ESP32-S3 (UART2)          Raspberry Pi Zero 2W (UART0)
├─ GPIO 15 (TX)    ─────> GPIO 16 (RX)
├─ GPIO 16 (RX)    <───── GPIO 15 (TX)
└─ GND             ─────  GND
```

### UART Settings

- **Baud Rate**: 115200
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Flow Control**: None

## MAVLink Messages

The ESP32-S3 sends the following MAVLink messages to the Raspberry Pi:

### 1. HEARTBEAT (ID: 0)

**Rate**: 1 Hz  
**Purpose**: Keep-alive and system status

- **type**: MAV_TYPE_FIXED_WING (1)
- **autopilot**: MAV_AUTOPILOT_GENERIC (0)
- **base_mode**: 0
- **system_status**: MAV_STATE_ACTIVE (4)
- **mavlink_version**: 3

### 2. ATTITUDE (ID: 30)

**Rate**: 20 Hz  
**Purpose**: Aircraft orientation (roll, pitch, yaw)

Contains IMU attitude data in radians:

- **roll**: Roll angle (radians)
- **pitch**: Pitch angle (radians)
- **yaw**: Yaw angle (radians)
- **rollspeed**: Roll rate (rad/s)
- **pitchspeed**: Pitch rate (rad/s)
- **yawspeed**: Yaw rate (rad/s)

### 3. SCALED_IMU (ID: 26)

**Rate**: 20 Hz  
**Purpose**: Raw IMU sensor data

Contains 9-axis sensor data:

- **xacc, yacc, zacc**: Acceleration in mg (milligravity)
- **xgyro, ygyro, zgyro**: Angular velocity in mrad/s
- **xmag, ymag, zmag**: Magnetic field in mgauss

### 4. RC_CHANNELS (ID: 65)

**Rate**: 20 Hz  
**Purpose**: Remote control channel data

Contains all RC channels from CRSF receiver:

- **chan1_raw** through **chan18_raw**: Channel PWM values (172-1811)
- **chancount**: Number of active channels (16)
- **rssi**: Signal strength (0=failsafe, 255=perfect)

### 5. SYS_STATUS (ID: 1)

**Rate**: 1 Hz  
**Purpose**: System health and diagnostics

Contains ESP32-S3 system information:

- **voltage_battery**: Battery voltage in mV (simulated)
- **current_battery**: Battery current in cA (simulated)
- **battery_remaining**: Battery percentage (simulated)
- **load**: CPU load in deci-percent (based on heap usage)

## Data Flow

```
┌──────────────────┐                           ┌──────────────────┐
│   ESP32-S3       │   MAVLink over UART       │  Raspberry Pi    │
│  Flight Control  │ ─────────────────────────>│   Zero 2W        │
│                  │       115200 baud          │                  │
│  - WT901B IMU    │                           │  - GPS (SIM7600) │
│  - CRSF RX       │                           │  - 4G Modem      │
│  - Servos        │                           │  - Tailscale VPN │
│  - ESC           │                           │                  │
└──────────────────┘                           └──────────────────┘
         │                                              │
         │                                              │
         v                                              v
   20Hz: IMU, RC Data                        MAVLink UDP to Ground Station
   1Hz: Heartbeat, System Status             (192.168.x.x:14550)
```

## Message Priority

1. **Critical**: RC_CHANNELS (20 Hz) - Direct control input
2. **High**: ATTITUDE, SCALED_IMU (20 Hz) - Essential flight data
3. **Normal**: HEARTBEAT, SYS_STATUS (1 Hz) - Status monitoring

## Implementation Details

### ESP32-S3 Side

- **File**: `main/pi_mavlink.c`, `main/pi_mavlink.h`
- **UART**: UART2 (GPIO15=TX, GPIO16=RX)
- **Task**: Dedicated MAVLink task handles periodic heartbeat and system status
- **Integration**: Called from main control loop for immediate data transmission

### Raspberry Pi Side

- **File**: `esp_mavlink_receiver.py`
- **UART**: `/dev/serial0` (GPIO14=TX, GPIO15=RX)
- **Library**: `pymavlink`
- **Integration**: Integrated into `telemetry_sender.py` data provider

## Message Encoding

MAVLink v1 packet structure:

```
┌─────────┬────────┬─────┬────────┬──────────┬─────────┬──────────┬─────────┐
│ STX     │ LEN    │ SEQ │ SYS_ID │ COMP_ID  │ MSG_ID  │ PAYLOAD  │ CRC     │
│ (0xFE)  │ (1B)   │(1B) │ (1B)   │ (1B)     │ (1B)    │ (0-255B) │ (2B)    │
└─────────┴────────┴─────┴────────┴──────────┴─────────┴──────────┴─────────┘
```

- **STX**: Start of frame (0xFE for MAVLink v1)
- **LEN**: Payload length
- **SEQ**: Packet sequence number
- **SYS_ID**: System ID (ESP32 = 1)
- **COMP_ID**: Component ID (Flight Controller = 1)
- **MSG_ID**: Message type identifier
- **PAYLOAD**: Message-specific data
- **CRC**: X.25 CRC checksum with CRC_EXTRA

## Configuration

### ESP32-S3 (`pi_mavlink.h`)

```c
#define PI_UART_NUM UART_NUM_2
#define PI_UART_TX_PIN 15
#define PI_UART_RX_PIN 16
#define PI_UART_BAUDRATE 115200
#define MAVLINK_SYSTEM_ID 1
#define MAVLINK_COMPONENT_ID 1
```

### Raspberry Pi (`esp_mavlink_receiver.py`)

```python
DEFAULT_PORT = '/dev/serial0'
DEFAULT_BAUDRATE = 115200
source_system = 2  # Pi is system 2
```

## Benefits of MAVLink

1. **Standardization**: Industry-standard protocol used by ArduPilot, PX4, etc.
2. **Tool Compatibility**: Works with QGroundControl, Mission Planner, MAVProxy
3. **Reliability**: Built-in CRC checksums and sequence numbers
4. **Extensibility**: Easy to add new message types
5. **Efficiency**: Binary protocol with minimal overhead
6. **Documentation**: Extensive documentation and examples available

## Troubleshooting

### Check UART Connection

```bash
# On Raspberry Pi
stty -F /dev/serial0 115200
cat /dev/serial0  # Should show binary data
```

### Monitor MAVLink Traffic

```bash
# On Raspberry Pi
python3 -m esp_mavlink_receiver  # Run test mode
```

### ESP32 Logging

Enable verbose logging in `pi_mavlink.c`:

```c
esp_log_level_set("PI_MAVLINK", ESP_LOG_DEBUG);
```

## Future Enhancements

1. **Bidirectional Communication**: Implement command reception on ESP32
2. **Battery Monitoring**: Add real ADC readings for battery voltage
3. **GPS Integration**: Forward GPS position from Pi to ESP32
4. **Parameter System**: Implement MAVLink parameter protocol for configuration
5. **Mission Upload**: Support waypoint missions via MAVLink

## References

- [MAVLink Developer Guide](https://mavlink.io/en/)
- [MAVLink Common Message Set](https://mavlink.io/en/messages/common.html)
- [pymavlink Documentation](https://mavlink.io/en/mavgen_python/)
