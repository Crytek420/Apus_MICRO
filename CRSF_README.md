# CRSF Kommunikationsmodul für ESP32-C6

## Übersicht

Dieses Modul implementiert die CRSF (Crossfire Serial Protocol) Kommunikation für den ESP32-C6 Microcontroller zur Verwendung mit dem Radiomaster XR1 Empfänger. Es ist Teil eines Flight Controllers für Fixed-Wing RC Flugzeuge.

## Funktionen

### Empfang

- **RC-Kanaldaten**: Empfängt 16 Kanäle (11-bit Auflösung, 172-1811)
- **Link-Statistiken**: RSSI, Link Quality, SNR
- **Automatische Frame-Validierung**: CRC8 (DVB-S2)
- **Timeout-Erkennung**: 500ms Timeout für verlorene Verbindung

### Senden (Telemetrie)

- **GPS-Daten**: Position, Geschwindigkeit, Höhe, Satelliten
- **Batterie**: Spannung, Strom, Kapazität, Restladung
- **Attitude**: Roll, Pitch, Yaw
- **Flight Mode**: Aktueller Flugmodus als Text

## Hardware-Konfiguration

### UART-Pins (ESP32-C6)

- **TX**: GPIO 4
- **RX**: GPIO 5
- **Baudrate**: 420.000 baud
- **Format**: 8N1 (8 Datenbit, keine Parität, 1 Stoppbit)

### Verkabelung zum Radiomaster XR1

```
ESP32-C6          XR1 Modul
--------          ----------
GPIO 4 (TX)  -->  RX
GPIO 5 (RX)  <--  TX
GND          ---  GND
5V           ---  VCC (wenn XR1 externe Versorgung benötigt)
```

## Softwarearchitektur

### FreeRTOS Task

- **Taktrate**: 10 Hz (100ms Zykluszeit)
- **Priorität**: 5
- **Stack-Größe**: 4096 Bytes
- **Task-Name**: "crsf_task"

### Datenfluss

```
XR1 Modul --> UART RX --> CRSF Parser --> RC Channels
                                      --> Link Stats

Flight Controller --> Telemetrie --> CRSF Encoder --> UART TX --> XR1 Modul
```

## API-Verwendung

### Initialisierung

```c
// CRSF Task starten (wird automatisch in app_main() aufgerufen)
crsf_start_task();
```

### RC-Kanäle abrufen

```c
crsf_channels_t channels;
if (crsf_get_channels(&channels)) {
    // Channels sind gültig
    uint16_t roll = channels.channels[0];      // Aileron
    uint16_t pitch = channels.channels[1];     // Elevator
    uint16_t throttle = channels.channels[2];  // Throttle
    uint16_t yaw = channels.channels[3];       // Rudder
}
```

### Telemetrie senden

#### GPS-Daten

```c
crsf_gps_t gps = {
    .latitude = 523456789,      // 52.3456789° (Grad * 10^7)
    .longitude = 133456789,     // 13.3456789° (Grad * 10^7)
    .groundspeed = 450,         // 45.0 km/h
    .heading = 18500,           // 185.0°
    .altitude = 1150,           // 150m (Meter + 1000)
    .satellites = 12
};

crsf_frame_t frame;
uint8_t len = crsf_prepare_gps_frame(&frame, &gps);
crsf_send_telemetry(&frame, len);
```

#### Batterie-Daten

```c
crsf_battery_t battery = {
    .voltage = 1260,    // 12.6V (Volt * 100)
    .current = 150,     // 1.5A (Ampere * 100)
    .capacity = 5000,   // 5000mAh
    .remaining = 85     // 85%
};

crsf_frame_t frame;
uint8_t len = crsf_prepare_battery_frame(&frame, &battery);
crsf_send_telemetry(&frame, len);
```

#### Attitude-Daten

```c
crsf_attitude_t attitude = {
    .pitch = 50,    // 0.005° (Grad * 10000)
    .roll = -100,   // -0.01°
    .yaw = 18500    // 1.85°
};

crsf_frame_t frame;
uint8_t len = crsf_prepare_attitude_frame(&frame, &attitude);
crsf_send_telemetry(&frame, len);
```

#### Flight Mode

```c
crsf_frame_t frame;
uint8_t len = crsf_prepare_flight_mode_frame(&frame, "STABILIZED");
crsf_send_telemetry(&frame, len);
```

## CRSF-Protokoll Details

### Frame-Struktur

```
+----------+------------+------+---------+-----+
| Address  | Frame Size | Type | Payload | CRC |
+----------+------------+------+---------+-----+
| 1 byte   | 1 byte     | 1 byte | N bytes | 1 byte |
```

### Adressen

- `0xC8`: Flight Controller
- `0xEA`: Radio Transmitter
- `0xEC`: CRSF Receiver
- `0x00`: Broadcast

### Frame-Typen

- `0x16`: RC Channels Packed (empfangen)
- `0x14`: Link Statistics (empfangen)
- `0x02`: GPS (senden)
- `0x08`: Battery Sensor (senden)
- `0x1E`: Attitude (senden)
- `0x21`: Flight Mode (senden)

### RC-Kanal-Mapping (Standard)

1. Channel 0: Aileron (Roll)
2. Channel 1: Elevator (Pitch)
3. Channel 2: Throttle
4. Channel 3: Rudder (Yaw)
5. Channel 4-15: Aux-Kanäle (Switches, Schalter)

### Wertebereiche

- **RC-Kanäle**: 172 (min) - 992 (mitte) - 1811 (max)
- **Auflösung**: 11 Bit (0-2047, effektiv 172-1811)

## Debugging

### Log-Level

```c
// In sdkconfig oder via menuconfig:
CONFIG_LOG_DEFAULT_LEVEL_DEBUG=y
```

### Log-Tags

- `CRSF`: Hauptmodul
- Ausgaben:
  - RC-Kanaldaten (DEBUG)
  - Link-Statistiken (INFO)
  - Frame-Fehler (WARNING)

### Typische Log-Ausgaben

```
I (1234) CRSF: CRSF UART initialized (TX: GPIO4, RX: GPIO5, Baud: 420000)
I (1235) CRSF: CRSF Task started (10 Hz)
D (1334) CRSF: RC Channels: [992, 992, 172, 992]
I (2334) CRSF: Link Stats - RSSI1: 95, LQ: 100%
W (3334) CRSF: RC data timeout
```

## Timing und Performance

### Task-Zyklen

- **10 Hz**: RC-Daten lesen (alle 100ms)
- **1 Hz**: Telemetrie senden (jeder 10. Zyklus)

### UART-Puffer

- **RX/TX Buffer**: 512 Bytes
- **Frame-Größe**: Max. 64 Bytes

### Latenz

- **RC-Empfang**: < 10ms (abhängig vom XR1)
- **Verarbeitung**: < 1ms
- **Gesamt**: < 20ms

## Erweiterungen für Flight Controller

Für die vollständige Flight-Controller-Implementierung werden folgende Erweiterungen benötigt:

1. **Sensor-Fusion**: Integration von IMU-Daten (MPU9250)
2. **PID-Regler**: Roll, Pitch, Yaw Kontrolle
3. **Servo-Ausgänge**: PWM für Ruder, Höhenruder, Querruder
4. **Motor-Steuerung**: ESC-Ansteuerung via PWM/OneShot
5. **Autopilot**: GPS-Navigation, Wegpunkte
6. **Failsafe**: Verhalten bei Verbindungsverlust
7. **Konfiguration**: Parameter über Telemetrie

## Bekannte Einschränkungen

1. Einfaches Frame-Sync (kann bei starken Störungen Frames verlieren)
2. Keine DMA für UART (könnte Performance verbessern)
3. Telemetrie-Rate fest auf 1 Hz (könnte konfigurierbar sein)
4. Keine Unterstützung für CRSF v3 Extended Frames

## Lizenz und Credits

Basierend auf dem CRSF-Protokoll von TBS (Team BlackSheep).
Entwickelt für ESP32-C6 mit ESP-IDF Framework.

## Version

- **Version**: 1.0
- **Datum**: November 2025
- **Autor**: Flight Controller Projekt
