# ESP32 to Raspberry Pi UART Communication Protocol

This document describes the binary telemetry protocol used for communication between the ESP32-S3 flight controller and the Raspberry Pi Zero 2W.

## Hardware Connection

### Pin Mapping

```
ESP32-S3 (UART2)          Raspberry Pi Zero 2W (UART0)
├─ GPIO 15 (TX)    ─────> GPIO 15 (RX)
├─ GPIO 16 (RX)    <───── GPIO 14 (TX)
└─ GND             ─────  GND
```

### UART Settings

- **Baud Rate**: 115200
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Flow Control**: None

## Protocol Specification

### Packet Structure

All packets follow this structure:

```
┌─────────┬─────────┬──────┬────────┬─────────────┬──────────┬───────────┐
│ SYNC1   │ SYNC2   │ TYPE │ LENGTH │ PAYLOAD     │ CRC_LOW  │ CRC_HIGH  │
│ (0xAA)  │ (0x55)  │ (1B) │ (1B)   │ (0-128B)    │ (1B)     │ (1B)      │
└─────────┴─────────┴──────┴────────┴─────────────┴──────────┴───────────┘
```

- **SYNC1, SYNC2**: Frame synchronization bytes (0xAA, 0x55)
- **TYPE**: Packet type identifier (see Packet Types)
- **LENGTH**: Payload length in bytes (0-128)
- **PAYLOAD**: Variable-length data payload
- **CRC**: CRC16-CCITT checksum (polynomial 0x1021, initial 0xFFFF)

### CRC Calculation

The CRC is calculated over the entire header + payload (SYNC1 through end of PAYLOAD):

```c
uint16_t crc = 0xFFFF;
for each byte in [SYNC1, SYNC2, TYPE, LENGTH, PAYLOAD...]:
    crc ^= byte << 8
    for bit 0 to 7:
        if (crc & 0x8000):
            crc = (crc << 1) ^ 0x1021
        else:
            crc = crc << 1
return crc
```

## Packet Types

### 0x01 - IMU Data

**Update Rate**: 20 Hz  
**Payload Size**: 56 bytes

Contains 9-axis IMU sensor data from the WT901B:

```c
struct IMUPayload {
    float acc_x, acc_y, acc_z;       // Acceleration (g)
    float gyro_x, gyro_y, gyro_z;    // Angular velocity (°/s)
    float mag_x, mag_y, mag_z;       // Magnetic field (μT)
    float roll, pitch, yaw;          // Attitude angles (degrees)
    float temperature;                // Temperature (°C)
    uint32_t timestamp_ms;           // Timestamp (milliseconds)
};
```

**Example Python Parsing**:

```python
data = struct.unpack('<13fI', payload)
imu = IMUData(
    acc_x=data[0], acc_y=data[1], acc_z=data[2],
    gyro_x=data[3], gyro_y=data[4], gyro_z=data[5],
    mag_x=data[6], mag_y=data[7], mag_z=data[8],
    roll=data[9], pitch=data[10], yaw=data[11],
    temperature=data[12], timestamp_ms=data[13]
)
```

### 0x02 - CRSF Channels

**Update Rate**: 20 Hz  
**Payload Size**: 37 bytes

Contains RC channel data from the CRSF/ELRS receiver:

```c
struct CRSFChannelsPayload {
    uint16_t channels[16];           // Channel values (172-1811)
    uint32_t timestamp_ms;           // Timestamp (milliseconds)
    uint8_t failsafe;                // Failsafe flag (0=ok, 1=failsafe)
};
```

**Channel Mapping**:

- Channel 0: Aileron (Roll)
- Channel 1: Elevator (Pitch)
- Channel 2: Throttle
- Channel 3: Rudder (Yaw)
- Channels 4-8: Switches (SA, SB, SC, SD, SE)
- Channels 9-15: Auxiliary channels

**Channel Range**: 172 (min), 992 (center), 1811 (max)

**Example Python Parsing**:

```python
data = struct.unpack('<16HIB', payload)
channels = CRSFChannels(
    channels=list(data[0:16]),
    timestamp_ms=data[16],
    failsafe=bool(data[17])
)
```

### 0x03 - CRSF Link Statistics

**Update Rate**: Variable (when available from receiver)  
**Payload Size**: 14 bytes

Contains RF link quality metrics:

```c
struct LinkStatsPayload {
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
    uint32_t timestamp_ms;           // Timestamp (milliseconds)
};
```

**Example Python Parsing**:

```python
data = struct.unpack('<BBBbBBBBBbI', payload)
stats = LinkStats(
    uplink_rssi_ant1=data[0], uplink_rssi_ant2=data[1],
    uplink_link_quality=data[2], uplink_snr=data[3],
    active_antenna=data[4], rf_mode=data[5],
    uplink_tx_power=data[6], downlink_rssi=data[7],
    downlink_link_quality=data[8], downlink_snr=data[9],
    timestamp_ms=data[10]
)
```

### 0x04 - System Information

**Update Rate**: 1 Hz  
**Payload Size**: 21 bytes

Contains ESP32 system status:

```c
struct SystemInfoPayload {
    float cpu_temp;                  // CPU temperature (°C) - currently unused
    float supply_voltage;            // Supply voltage (V) - requires ADC
    uint32_t free_heap;              // Free heap memory (bytes)
    uint32_t uptime_ms;              // System uptime (milliseconds)
    uint8_t cpu_usage;               // CPU usage (%) - currently unused
    uint32_t timestamp_ms;           // Timestamp (milliseconds)
};
```

**Example Python Parsing**:

```python
data = struct.unpack('<ffIIBI', payload)
info = SystemInfo(
    cpu_temp=data[0], supply_voltage=data[1],
    free_heap=data[2], uptime_ms=data[3],
    cpu_usage=data[4], timestamp_ms=data[5]
)
```

### 0x05 - Heartbeat

**Update Rate**: 1 Hz  
**Payload Size**: 8 bytes

Simple keepalive packet:

```c
struct HeartbeatPayload {
    uint32_t sequence;               // Sequence number (increments)
    uint32_t timestamp_ms;           // Timestamp (milliseconds)
};
```

**Example Python Parsing**:

```python
data = struct.unpack('<II', payload)
hb = Heartbeat(sequence=data[0], timestamp_ms=data[1])
```

## Data Flow Summary

```
ESP32-S3 Flight Controller
│
├─ 20 Hz: IMU Data (Roll, Pitch, Yaw, Accel, Gyro, Mag)
├─ 20 Hz: CRSF Channels (All 16 RC channels + failsafe)
├─ Variable: Link Statistics (RSSI, Link Quality, SNR)
├─ 1 Hz: System Info (Heap, Uptime)
└─ 1 Hz: Heartbeat
     │
     v
  UART2 (115200 baud)
     │
     v
Raspberry Pi Zero 2W
```

## Prototyping Workflow

### Development Setup

1. **VS Code Setup**:
   - Window 1: ESP32 project (local)
   - Window 2: SSH to Pi (`Ctrl+Shift+P` → "Remote-SSH: Connect to Host")
   - Edit Python code directly on Pi

2. **Testing Cycle**:

   ```bash
   # On PC: Flash ESP32
   idf.py flash monitor

   # On Pi: Run receiver
   python3 pi_telemetry_receiver.py
   ```

3. **Quick Iteration**:
   - Change ESP code → Flash → Instantly see on Pi
   - Change Pi code → Save → Restart receiver

### Raspberry Pi Setup

1. **Enable UART0**:

   ```bash
   sudo raspi-config
   # Interface Options → Serial Port
   # - Would you like a login shell to be accessible over serial? NO
   # - Would you like the serial port hardware to be enabled? YES
   ```

2. **Disable Bluetooth** (frees up UART0):

   ```bash
   # Add to /boot/config.txt
   dtoverlay=disable-bt

   # Disable bluetooth service
   sudo systemctl disable hciuart
   ```

3. **Install Dependencies**:

   ```bash
   sudo apt update
   sudo apt install python3-serial
   ```

4. **Test Connection**:

   ```bash
   # Copy the receiver script to Pi
   scp pi_telemetry_receiver.py pi@raspberrypi.local:~/

   # Run on Pi
   python3 pi_telemetry_receiver.py --port /dev/serial0 --baud 115200
   ```

## Performance Characteristics

### Bandwidth Usage

| Packet Type   | Size     | Rate  | Bandwidth     |
| ------------- | -------- | ----- | ------------- |
| IMU           | 62 bytes | 20 Hz | 1240 B/s      |
| CRSF Channels | 43 bytes | 20 Hz | 860 B/s       |
| Link Stats    | 20 bytes | ~1 Hz | 20 B/s        |
| System Info   | 27 bytes | 1 Hz  | 27 B/s        |
| Heartbeat     | 14 bytes | 1 Hz  | 14 B/s        |
| **Total**     |          |       | **~2161 B/s** |

At 115200 baud (~11520 bytes/s effective), this uses approximately **19% of bandwidth**.

### Latency

- **End-to-end latency**: < 5ms (measured from ESP data capture to Pi reception)
- **Packet overhead**: 6 bytes (sync + header + CRC)
- **Maximum throughput**: ~100 packets/sec with current configuration

## Error Handling

### ESP32 Side

- CRC validation before transmission
- UART buffer overflow protection (1024 byte buffer)
- Graceful handling of missing sensors

### Pi Side

- Sync byte detection with recovery
- CRC verification (drops invalid packets)
- Timeout handling (1 second read timeout)
- Error statistics tracking

## Future Enhancements

1. **Bi-directional Communication**:
   - Pi → ESP32 commands
   - Parameter updates
   - Mission waypoints

2. **Additional Sensors**:
   - GPS data
   - Barometric altitude
   - Airspeed sensor

3. **Data Logging**:
   - CSV export
   - Binary flight logs
   - Real-time visualization

4. **Compression**:
   - Delta encoding for high-rate data
   - Adaptive update rates

## Troubleshooting

### No Data Received

1. Check wiring: TX → RX, RX → TX
2. Verify UART enabled on Pi: `ls -l /dev/serial0`
3. Check ESP32 logs: `idf.py monitor`
4. Test with loopback: Connect ESP TX to RX

### CRC Errors

1. Check baud rate matches (115200)
2. Verify cable quality (use short cables)
3. Check ground connection
4. Look for EMI sources

### Packet Loss

1. Monitor Pi CPU usage: `top`
2. Check for buffer overflows in ESP logs
3. Reduce update rates if needed
4. Consider flow control

## Example Output

```
Connected to /dev/serial0 at 115200 baud
Starting telemetry receiver...
Press Ctrl+C to exit

[IMU] Roll:   -2.1° Pitch:    1.5° Yaw:  178.3° Temp: 24.5°C
[CRSF] CH0-3:  992  992 1250  992
[LINK] RSSI: -65 dBm, LQ:  98%, SNR:  +8 dB
[HB] Seq:     5, Time: 5234 ms
[SYS] Uptime: 5.2s, Free Heap: 245.3 KB
[IMU] Roll:   -2.0° Pitch:    1.6° Yaw:  178.4° Temp: 24.5°C
[CRSF] CH0-3:  992  992 1250  992
...
```

## License

This protocol and reference implementation are part of the Apus MICRO flight controller project.
