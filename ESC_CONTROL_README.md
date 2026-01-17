# ESC Control Module Documentation

## Overview

This module provides comprehensive ESC (Electronic Speed Controller) control for the ESP32-S3 flight controller, specifically configured for the **Hobbywing 40A V2 ESC**.

## Hardware Configuration

### ESC Specifications (Hobbywing 40A V2)

- **PWM Frequency**: 50 Hz (standard for hobby ESCs)
- **Pulse Width Range**: 1000µs to 2000µs
  - 1000µs (1.0ms) = 0% throttle (motor off)
  - 1500µs (1.5ms) = 50% throttle
  - 2000µs (2.0ms) = 100% throttle (full power)

### Pin Assignment

- **GPIO 13**: ESC PWM output signal
- **ESC Power**: Connect to external BEC (not ESP32)
- **Common Ground**: ESP32 GND must be connected to ESC GND

## Features

### 1. Low-Pass Filter

The module implements a first-order exponential low-pass filter to smooth throttle changes and prevent sudden motor speed transitions.

**Filter Equation:**

```
filtered_output = filtered_output + α × (target - filtered_output)

where α = dt / (τ + dt)
  dt = update period (20ms at 50Hz)
  τ = filter time constant (configurable)
```

**Default Settings:**

- Filter time constant: 100ms
- Update rate: 50Hz (20ms period)
- Adjustable range: 10ms to 1000ms

**Typical Values:**

- **Fast response**: 50ms (aggressive, race applications)
- **Balanced**: 100ms (default, good for most flying)
- **Smooth**: 200ms (gentle, training/stability)

### 2. Safety Features

- **Emergency Stop**: Immediately sets throttle to zero
- **Automatic Arming**: 2-second arming sequence on startup
- **No RC Signal Handling**: Auto-stops motor when RC signal lost
- **Input Clamping**: All inputs validated and clamped to safe ranges

### 3. Input Mapping

Automatic conversion from CRSF RC protocol to ESC throttle:

- CRSF input: 172-1811 (11-bit channel value)
- ESC throttle: 0-1000 (0% to 100%)

## API Functions

### Initialization

```c
esp_err_t esc_init(void);
```

Initializes the ESC PWM output and starts the control task. Must be called before any other ESC functions.

### Throttle Control

```c
void esc_set_throttle(uint16_t throttle);
```

Sets the target throttle (0-1000) with low-pass filtering applied.

```c
void esc_set_throttle_direct(uint16_t throttle);
```

Sets throttle immediately without filtering. Use for arming/calibration only.

### Filter Configuration

```c
void esc_set_filter_time(uint16_t time_ms);
```

Configures the low-pass filter time constant (10-1000ms).

### Status

```c
uint16_t esc_get_throttle(void);
```

Returns the current filtered throttle value (0-1000).

### Safety

```c
void esc_arm(void);
```

Arms the ESC by sending minimum throttle for 2 seconds. Call after initialization.

```c
void esc_emergency_stop(void);
```

Immediately stops the motor by setting throttle to zero.

### Input Conversion

```c
uint16_t esc_map_crsf_to_throttle(uint16_t channel_value);
```

Converts CRSF channel value (172-1811) to ESC throttle (0-1000).

## Integration in main.c

The ESC module is integrated into the main flight controller loop:

1. **Initialization** (in `app_main`):
   - Initialize ESC after servos
   - Arm ESC after CRSF communication starts
   - Set filter time to 100ms

2. **Main Loop** (50Hz):
   - Read throttle from RC channel 2
   - Map CRSF value to ESC throttle
   - Apply throttle with filtering
   - Log throttle percentage

3. **Failsafe**:
   - Emergency stop when RC signal is lost
   - Automatic recovery when signal returns

## Usage Example

```c
// In app_main()
esc_init();                    // Initialize ESC
vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for RC
esc_arm();                     // Arm ESC
esc_set_filter_time(100);      // Set filter time

// In main loop (50Hz)
if (crsf_get_channels(&channels)) {
    uint16_t throttle = esc_map_crsf_to_throttle(channels.channels[2]);
    esc_set_throttle(throttle);
} else {
    esc_emergency_stop();      // No RC signal
}
```

## Technical Details

### PWM Configuration

- **LEDC Timer**: TIMER_1
- **LEDC Channel**: CHANNEL_4
- **Resolution**: 14-bit (16384 steps)
- **Mode**: Low-speed mode
- **Clock**: Auto-select

### Control Task

- **Priority**: 5
- **Stack Size**: 2048 bytes
- **Update Rate**: 50Hz (20ms period)
- **Purpose**: Runs the low-pass filter continuously

### Filter Response Time

The filter time constant (τ) determines how quickly the output responds to input changes:

- **63% response**: τ milliseconds
- **95% response**: 3×τ milliseconds
- **99% response**: 5×τ milliseconds

Example with τ=100ms:

- After 100ms: output reaches 63% of target
- After 300ms: output reaches 95% of target
- After 500ms: output reaches 99% of target

## Wiring Diagram

```
ESP32-S3 (GPIO 13) ───[PWM]──→ ESC Signal (White/Orange)
                                │
ESP32 GND ──────────────────────┴─ ESC GND (Black/Brown)

External BEC/Battery ──→ ESC Power (Red)
                    └──→ Motor Power

Motor ←─ ESC Motor Wires (3-phase)
```

## Safety Notes

⚠️ **IMPORTANT SAFETY CONSIDERATIONS:**

1. **Always arm ESC properly** before applying throttle
2. **Remove propeller** during testing and development
3. **Use external power supply** for ESC (not ESP32 USB)
4. **Connect common ground** between ESP32 and ESC
5. **Test throttle response** without propeller first
6. **Implement RC failsafe** (already included)
7. **Start with high filter time** (200ms+) for initial testing

## Troubleshooting

### ESC not responding

- Check wiring (signal, ground connection)
- Verify ESC is powered externally
- Ensure ESC is armed (check logs)
- Confirm PWM signal with oscilloscope

### Motor spins unexpectedly

- Check RC transmitter throttle stick is at minimum
- Verify CRSF channel mapping (channel 2 for throttle)
- Increase filter time for smoother response

### Jerky throttle response

- Decrease filter time (try 50ms)
- Check RC signal quality
- Verify 50Hz update rate in logs

### Motor doesn't reach full power

- Check pulse width range (1000-2000µs for Hobbywing)
- Verify ESC calibration
- Check battery voltage

## Future Enhancements

Potential improvements for flight controller integration:

1. Motor temperature monitoring
2. Current sensing and limiting
3. ESC telemetry (if supported)
4. Multi-motor support for multi-rotor
5. Auto-calibration routine
6. Thrust curve linearization
7. Battery voltage compensation

## File Locations

- Header: `main/esc_control.h`
- Implementation: `main/esc_control.c`
- Integration: `main/main.c`
- Build config: `main/CMakeLists.txt`
