# Quick Start Guide: ESP32-Pi UART Communication

## What Was Implemented

### ✅ ESP32 Side (Fully Implemented)

- Binary packet protocol with CRC16 validation
- Sends IMU data at 20 Hz (accelerometer, gyro, magnetometer, attitude)
- Sends CRSF RC channels at 20 Hz (all 16 channels + failsafe status)
- Sends CRSF link statistics (RSSI, link quality, SNR)
- Sends ESP32 system info at 1 Hz (heap memory, uptime)
- Sends heartbeat at 1 Hz

### ✅ Python Receiver (Reference Implementation)

- Complete packet decoder with CRC validation
- Parses all 5 packet types
- Real-time console display
- Error tracking and statistics

## Files Modified/Created

### ESP32 Files

- `main/pi_uart.h` - Protocol definitions and API
- `main/pi_uart.c` - Telemetry transmission implementation
- `main/crsf_protocol.h` - Added link stats getter
- `main/crsf_protocol.c` - Added link stats storage
- `main/main.c` - Integrated telemetry sending

### Pi Files

- `pi_telemetry_receiver.py` - Python receiver example
- `PI_UART_PROTOCOL.md` - Complete protocol documentation
- `QUICK_START.md` - This file

## Build and Flash

```bash
# In your ESP32 project directory
idf.py build
idf.py flash monitor
```

## Raspberry Pi Setup

### 1. Enable UART

```bash
sudo raspi-config
# Interface Options → Serial Port
# Login shell: NO
# Hardware enabled: YES
sudo reboot
```

### 2. Disable Bluetooth (recommended)

```bash
# Edit /boot/config.txt
sudo nano /boot/config.txt

# Add this line:
dtoverlay=disable-bt

# Save and reboot
sudo systemctl disable hciuart
sudo reboot
```

### 3. Copy Python Script to Pi

```bash
# From your PC
scp pi_telemetry_receiver.py pi@raspberrypi.local:~/
```

### 4. Run Receiver

```bash
# On Raspberry Pi
python3 pi_telemetry_receiver.py
```

## Hardware Connections

```
ESP32-S3          Raspberry Pi Zero 2W
GPIO 15 (TX) ──→  GPIO 15 (RX)
GPIO 16 (RX) ←──  GPIO 14 (TX)
GND          ────  GND
```

## What You'll See

### ESP32 Monitor Output

```
I (1234) PI_UART: Pi UART initialized successfully
I (1235) PI_UART: Protocol: Binary packets with CRC16 validation
I (5234) FlightController: IMU: Roll=-2.1° Pitch=1.5° Yaw=178.3°
I (5234) FlightController: RC Sticks: Roll=992 Pitch=992 Throttle=1250 Yaw=992
```

### Raspberry Pi Output

```
[IMU] Roll:   -2.1° Pitch:    1.5° Yaw:  178.3° Temp: 24.5°C
[CRSF] CH0-3:  992  992 1250  992
[LINK] RSSI: -65 dBm, LQ:  98%, SNR:  +8 dB
[HB] Seq:     5, Time: 5234 ms
[SYS] Uptime: 5.2s, Free Heap: 245.3 KB
```

## Data Rates

| Data Type   | Update Rate | Bytes/Packet |
| ----------- | ----------- | ------------ |
| IMU Data    | 20 Hz       | 62           |
| RC Channels | 20 Hz       | 43           |
| Link Stats  | ~1 Hz       | 20           |
| System Info | 1 Hz        | 27           |
| Heartbeat   | 1 Hz        | 14           |

**Total Bandwidth**: ~2.2 KB/s (19% of 115200 baud)

## Development Workflow

### Option 1: VS Code Remote SSH (Recommended)

1. Install "Remote - SSH" extension in VS Code
2. Connect to Pi: `Ctrl+Shift+P` → "Remote-SSH: Connect to Host"
3. Open folder with Python script
4. Edit directly on Pi, run from integrated terminal

### Option 2: Local Edit + Transfer

1. Edit Python script on PC
2. Transfer: `scp pi_telemetry_receiver.py pi@raspberrypi.local:~/`
3. SSH to Pi and run: `python3 pi_telemetry_receiver.py`

### Option 3: Mounted Filesystem

1. Mount Pi filesystem: `sshfs pi@raspberrypi.local:/home/pi ~/pi_mount`
2. Edit files locally, they update on Pi immediately
3. SSH to Pi to run scripts

## Prototyping Tips

### Simultaneous Development

```
Terminal 1 (ESP32):          Terminal 2 (Raspberry Pi):
├─ idf.py build              ├─ vim pi_telemetry_receiver.py
├─ idf.py flash              ├─ python3 pi_telemetry_receiver.py
└─ idf.py monitor            └─ [watch live data]
```

### Quick Testing Cycle

1. Modify ESP code
2. Flash in one terminal
3. Immediately see results in Pi terminal
4. Adjust Pi code as needed
5. Restart Python script

### Data Logging on Pi

```python
# Add to receiver script:
import csv
import datetime

log_file = f"flight_log_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv"
with open(log_file, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['timestamp', 'roll', 'pitch', 'yaw', ...])

    # In process_packet():
    if packet_type == PacketType.IMU:
        writer.writerow([time.time(), imu.roll, imu.pitch, imu.yaw, ...])
```

## Troubleshooting

### "No such file or directory: '/dev/serial0'"

- Run `ls -l /dev/serial*` to check available ports
- Try `/dev/ttyAMA0` instead
- Enable UART in raspi-config

### "Permission denied" on serial port

```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### CRC Errors

- Check wiring (TX to RX, not TX to TX)
- Verify ground connection
- Try shorter cables (< 30cm ideal)
- Check for loose connections

### No Data Received

1. Test loopback on ESP: connect GPIO 15 to GPIO 16
2. Check ESP monitor for "Sent packet" messages
3. Try: `cat /dev/serial0` on Pi (should see binary data)
4. Verify baud rate: `stty -F /dev/serial0 115200`

## Next Steps

### 1. Process Data on Pi

```python
# Example: PID control
def calculate_pid(target, current, kp, ki, kd):
    error = target - current
    # ... PID logic
    return control_output

# Send control commands back to ESP32
```

### 2. Add Data Visualization

```python
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Real-time plotting of IMU data
```

### 3. Machine Learning Integration

```python
import tensorflow as tf
# Load trained model
model = tf.keras.models.load_model('flight_model.h5')

# Make predictions from IMU/RC data
prediction = model.predict(sensor_data)
```

### 4. ROS Integration

```python
import rospy
from sensor_msgs.msg import Imu

# Publish IMU data to ROS
imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
```

## Protocol Customization

### Add New Packet Type

1. **ESP32** (`pi_uart.h`):

   ```c
   #define PI_PACKET_TYPE_CUSTOM 0x06

   typedef struct {
       float custom_data;
       uint32_t timestamp_ms;
   } pi_custom_payload_t;
   ```

2. **ESP32** (`pi_uart.c`):

   ```c
   esp_err_t pi_uart_send_custom(float data) {
       pi_custom_payload_t payload = {
           .custom_data = data,
           .timestamp_ms = get_timestamp_ms()
       };
       return pi_uart_send_packet(PI_PACKET_TYPE_CUSTOM, &payload, sizeof(payload));
   }
   ```

3. **Python**:

   ```python
   class PacketType(IntEnum):
       CUSTOM = 0x06

   def parse_custom(self, payload):
       data = struct.unpack('<fI', payload)
       return CustomData(custom_data=data[0], timestamp_ms=data[1])
   ```

### Adjust Update Rates

In `main.c`:

```c
// Change from 20 Hz to 50 Hz
if ((now - last_telemetry_time) >= pdMS_TO_TICKS(20))  // was 50
```

## Performance Monitoring

### ESP32 Side

```c
// Add to pi_uart_send_packet():
static uint32_t total_bytes_sent = 0;
total_bytes_sent += total_len;
ESP_LOGI(TAG, "Total sent: %lu bytes", total_bytes_sent);
```

### Pi Side

```python
# Track in receiver:
def print_stats(self):
    elapsed = time.time() - self.start_time
    rate = self.packet_count / elapsed
    error_rate = self.error_count / self.packet_count if self.packet_count > 0 else 0
    print(f"Rate: {rate:.1f} pkt/s, Error: {error_rate*100:.2f}%")
```

## Additional Resources

- Full Protocol Documentation: `PI_UART_PROTOCOL.md`
- ESP32 API: `main/pi_uart.h`
- Python Receiver: `pi_telemetry_receiver.py`

## Questions?

Common questions and answers:

**Q: Can I use different GPIO pins?**  
A: Yes, just update `PI_UART_TX_PIN` and `PI_UART_RX_PIN` in `pi_uart.h`

**Q: Can I increase the baud rate?**  
A: Yes, 230400 or 460800 work well. Update both ESP32 and Pi.

**Q: How do I send commands from Pi to ESP32?**  
A: Add receiver code in ESP32 main loop using `pi_uart_receive()` and create corresponding packet types.

**Q: Can I log to SD card on ESP32?**  
A: Yes, use ESP-IDF's SD card API and log before sending to Pi.
