# ESP32-S3 GPIO Pinbelegung - Apus MICRO Flight Controller

## Übersicht

Der ESP32-S3-DEV-KIT-NXR8 verfügt über folgende Schnittstellen:

- **UART0**: USB-CDC für Debug/Flashing (automatisch über USB)
- **UART1**: CRSF Protokoll (Radiomaster XR1) - GPIO 17/18
- **UART2**: Raspberry Pi Zero 2W Kommunikation - GPIO 15/16
- **I2C0**: WT901B IMU Sensor - GPIO 47/48 (SDA/SCL)
- **PWM (LEDC)**: 4x Servo-Ausgänge - GPIO 9/10/11/12
- **ADC1**: Frei für Strom-/Spannungsmessung - GPIO 1-8

---

## GPIO-Belegung

| GPIO | Funktion       | Beschreibung                    | Verbindung             |
| ---- | -------------- | ------------------------------- | ---------------------- |
| 9    | PWM (LEDC CH0) | Servo Linke Tragfläche Außen    | → Servo Signal         |
| 10   | PWM (LEDC CH1) | Servo Linke Tragfläche Innen    | → Servo Signal         |
| 11   | PWM (LEDC CH2) | Servo Rechte Tragfläche Innen   | → Servo Signal         |
| 12   | PWM (LEDC CH3) | Servo Rechte Tragfläche Außen   | → Servo Signal         |
| 15   | UART2 TX       | Pi Communication TX             | → Pi Zero RX (GPIO 15) |
| 16   | UART2 RX       | Pi Communication RX             | ← Pi Zero TX (GPIO 14) |
| 17   | UART1 TX       | CRSF TX (Telemetrie zu XR1)     | → XR1 RX               |
| 18   | UART1 RX       | CRSF RX (RC-Daten von XR1)      | ← XR1 TX               |
| 19   | USB D-         | USB Kommunikation (Debug/Flash) | USB Buchse             |
| 20   | USB D+         | USB Kommunikation (Debug/Flash) | USB Buchse             |
| 47   | I2C0 SDA       | IMU Data Line                   | ↔ WT901B SDA           |
| 48   | I2C0 SCL       | IMU Clock Line                  | → WT901B SCL           |

---

## ADC-Pins für Strom-/Spannungsmessung (FREI)

Diese Pins sind für analoge Messungen reserviert:

| GPIO | ADC Kanal | Empfohlene Verwendung      |
| ---- | --------- | -------------------------- |
| 1    | ADC1_CH0  | Batteriespannung (Vbat)    |
| 2    | ADC1_CH1  | Stromverbrauch (via Shunt) |
| 3    | ADC1_CH2  | Frei / Servo-Strom         |
| 4    | ADC1_CH3  | Frei                       |
| 5    | ADC1_CH4  | Frei                       |
| 6    | ADC1_CH5  | Frei                       |
| 7    | ADC1_CH6  | Frei                       |
| 8    | ADC1_CH7  | Frei                       |

**Hinweis:** ADC1 kann während WiFi-Nutzung verwendet werden. ADC2 (GPIO 11-20) ist bei aktivem WiFi blockiert!

### Spannungsmessung Beispiel:

```
Batterie (z.B. 3S LiPo 12.6V max)
        │
        ├──[10kΩ]──┬──[3.3kΩ]── GND
                   │
              GPIO 1 (ADC1_CH0)

Teilerverhältnis: 3.3/(10+3.3) = 0.248
Max. Eingangsspannung: 3.3V / 0.248 = 13.3V
```

---

## Verkabelungsschema

### 1. Radiomaster XR1 (CRSF) - UART1

```
ESP32-S3          XR1 Module
---------         ----------
GPIO 17 (TX) ---> RX
GPIO 18 (RX) <--- TX
3.3V         ---> VCC
GND          ---> GND
```

**Hinweis:** CRSF verwendet 420000 Baud, native UART1 Pins!

### 2. Raspberry Pi Zero 2W - UART2

```
ESP32-S3          Pi Zero 2W
---------         ----------
GPIO 15 (TX) ---> GPIO 15 (RX)
GPIO 16 (RX) <--- GPIO 14 (TX)
GND          ---> GND
```

**Hinweis:** 115200 Baud, 3.3V Logikpegel (kompatibel)

### 3. WT901B IMU - I2C0

```
ESP32-S3           WT901B
---------          ------
GPIO 47 (SDA) <--> SDA
GPIO 48 (SCL) ---> SCL
3.3V          ---> VCC
GND           ---> GND
```

**Hinweis:**

- I2C Adresse: 0x50 (Standard)
- I2C Geschwindigkeit: 400 kHz (Fast Mode)
- Externe 4.7kΩ Pull-ups empfohlen

### 4. Servos - PWM (LEDC)

```
ESP32-S3          Servos
---------         ------
GPIO 9       ---> Links Außen Signal (Orange/Weiß)
GPIO 10      ---> Links Innen Signal (Orange/Weiß)
GPIO 11      ---> Rechts Innen Signal (Orange/Weiß)
GPIO 12      ---> Rechts Außen Signal (Orange/Weiß)

Servo-Stromversorgung (separat!):
5V/6V BEC    ---> Servo VCC (Rot)
GND (gemeinsam) ---> Servo GND (Braun/Schwarz)
```

**Hinweis:**

- PWM Frequenz: 50 Hz
- Pulsweite: 500µs (-90°) bis 2500µs (+90°), 1500µs (Neutral)
- ESP32 GND muss mit Servo GND verbunden sein!

---

## Stromversorgung

| Komponente       | Spannung | Strombedarf (ca.) |
| ---------------- | -------- | ----------------- |
| ESP32-S3         | 3.3V     | 200-400 mA        |
| Radiomaster XR1  | 3.3-5V   | 50-100 mA         |
| WT901B IMU       | 3.3V     | 10-20 mA          |
| Servos (SG90) x4 | 5-6V     | 100-500 mA/Servo  |

**Empfehlung:**

- ESP32, XR1, IMU: Gemeinsame 3.3V Versorgung vom Regler
- Servos: Separater 5V/6V BEC mit ausreichend Strom (mind. 2A)

---

## Zusammenfassung Pin-Zuweisung

### Benutzte Pins:

| Funktion     | Pins               |
| ------------ | ------------------ |
| UART1 (CRSF) | GPIO 17, 18        |
| UART2 (Pi)   | GPIO 15, 16        |
| I2C (IMU)    | GPIO 47, 21        |
| Servos       | GPIO 9, 10, 11, 12 |
| USB          | GPIO 19, 20        |

### Reserviert für ADC (Strom/Spannung):

| Funktion | Pins                        |
| -------- | --------------------------- |
| ADC1     | GPIO 1, 2, 3, 4, 5, 6, 7, 8 |

### Freie Pins für Erweiterungen:

| Pins                                |
| ----------------------------------- |
| GPIO 13, 14, 38, 39, 40, 41, 42, 48 |

---

## Wichtige Hinweise

1. **USB-C Debug:** GPIO 19/20 werden für native USB verwendet - nicht anderweitig nutzen!

2. **Strapping Pins:** GPIO 0, 45, 46 nicht belasten beim Booten!

3. **Flash/PSRAM:** GPIO 26-37 werden intern für Flash/PSRAM verwendet - NICHT NUTZEN!

4. **RGB LED:** GPIO 48 steuert die onboard RGB-LED - kann aber als GPIO verwendet werden.

5. **ADC Genauigkeit:** Für präzise Messungen externe Referenz/Kalibrierung empfohlen.
