# VARUNA — Autonomous Flood Monitoring Buoy


VARUNA is an autonomous, tethered flood-monitoring buoy designed for real-world deployment in flood-prone river basins, canals, and urban drainage systems. The system provides continuous water-level sensing, on-device flood classification, dynamic cloud telemetry, and automated email alerts — all from a self-contained two-microcontroller embedded platform.

---

## Table of Contents

- [System Architecture](#system-architecture)
- [Hardware Components](#hardware-components)
- [Two-Microcontroller Design](#two-microcontroller-design)
  - [ESP32-S3 — Sensor Brain](#esp32-s3--sensor-brain)
  - [XIAO ESP32-C3 — Communication Bridge](#xiao-esp32-c3--communication-bridge)
- [Inter-MCU Communication](#inter-mcu-communication)
- [Sensing & Signal Processing](#sensing--signal-processing)
  - [Tether-Angle Water Height Model](#tether-angle-water-height-model)
  - [Wave Filter — 2-Second Trimmed Mean](#wave-filter--2-second-trimmed-mean)
  - [Sensor Fusion — Complementary Filter](#sensor-fusion--complementary-filter)
  - [Gravity Auto-Zero Calibration](#gravity-auto-zero-calibration)
- [Flood Detection Engine](#flood-detection-engine)
  - [Operating Modes](#operating-modes)
  - [Alert Levels and Zones](#alert-levels-and-zones)
  - [Dynamic Firebase Push Rate](#dynamic-firebase-push-rate)
- [Cloud Integration — Firebase](#cloud-integration--firebase)
  - [Data Paths](#data-paths)
  - [Wireless Console](#wireless-console)
- [Web Dashboard](#web-dashboard)
  - [Panels](#panels)
  - [Alerting System](#alerting-system)
  - [Analytics & Export](#analytics--export)
- [C3 Bridge — SD Buffering & OTA](#c3-bridge--sd-buffering--ota)
- [Calibration Procedure](#calibration-procedure)
- [Serial Console Commands (S3)](#serial-console-commands-s3)
- [Diagnostics](#diagnostics)
- [Pin Reference](#pin-reference)
- [Firmware Overview](#firmware-overview)
- [Project Goals & Context](#project-goals--context)

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        VARUNA BUOY                              │
│                                                                 │
│   ┌──────────────────────────────────┐                         │
│   │        ESP32-S3                  │                         │
│   │     Sensor Brain v3.2            │                         │
│   │                                  │                         │
│   │  MPU6050 ──► Sensor Fusion       │                         │
│   │  BMP280  ──► Pressure Baseline   │                         │
│   │  GPS     ──► Location Fix        │                         │
│   │  Battery ──► ADC Monitor         │                         │
│   │                                  │                         │
│   │  ┌────────────────────────────┐  │                         │
│   │  │  Flood Detection Engine    │  │                         │
│   │  │  4-Mode FSM | Wave Filter  │  │                         │
│   │  └────────────────────────────┘  │                         │
│   │                                  │  WiFi / Firebase        │
│   │  Firebase RTDB ◄────────────────────────────────────────► │
│   │  (direct push + poll)            │                     Cloud│
│   │                                  │                         │
│   │  GPIO 14 (SW-UART TX) ──────────┐│                         │
│   │  GPIO 43/44 (HW-UART) ◄────────┤│                         │
│   └──────────────────────────────────┘│                         │
│                                        │                         │
│   ┌────────────────────────────────────▼──────────────────────┐ │
│   │              XIAO ESP32-C3                                │ │
│   │         Communication Bridge v2.0                         │ │
│   │                                                           │ │
│   │  GPIO 10  (SW-UART RX) ◄── CSV data from S3              │ │
│   │  GPIO 20/21 (HW-UART)  ◄── Commands from S3              │ │
│   │  GPIO 2/3  ──► S3 BOOT/RESET (OTA programming)           │ │
│   │  GPIO 4-7  ──► SD Card (SPI)                             │ │
│   │                                                           │ │
│   │  SIM800L GSM Module ◄────────────────────────────────►   │ │
│   │  (GPRS cellular uplink / SMS fallback)                    │ │
│   │                                                           │ │
│   │  WiFi (primary) + SIM800L/GPRS (backup data channel)     │ │
│   └───────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                                │
                         Firebase RTDB
                     (varuna-git-1e145)
                                │
                    ┌───────────▼───────────┐
                    │   Web Dashboard v32   │
                    │  varuna_dashboard.html│
                    │                       │
                    │  Live Telemetry       │
                    │  Map + GPS            │
                    │  Analytics & Charts   │
                    │  Alert Dispatch       │
                    │  Remote Console       │
                    └───────────────────────┘
```

---

## Hardware Components

| Component | Role | Interface |
|---|---|---|
| ESP32-S3 | Primary sensor processor & flood intelligence | — |
| XIAO ESP32-C3 | Communication bridge, SD buffering, OTA programmer | — |
| MPU6050 | 6-axis IMU (accelerometer + gyroscope) | I2C Bus 0 (GPIO 8/9) |
| BMP280 | Barometric pressure + temperature | I2C Bus 1 (GPIO 4/5) |
| GPS Module | NMEA position fix (GGA + RMC) | UART (GPIO 6/7) |
| SIM800L | GSM/GPRS cellular module | Via C3 |
| SD Card | Offline CSV buffer | SPI (C3 GPIO 4-7) |
| Battery | Li-Ion power source | ADC (S3 GPIO 2, ×2 divider) |

---

## Two-Microcontroller Design

VARUNA separates concerns cleanly across two microcontrollers:

### ESP32-S3 — Sensor Brain

The S3 is responsible for **everything sensor-related and all flood intelligence**. It runs continuously at 100 Hz sensor fusion, produces wave-filtered water level estimates every 2 seconds, classifies the buoy's flood mode, computes alert levels, and pushes data directly to Firebase Realtime Database over WiFi. It also polls Firebase for remote console commands and responds to them without any C3 involvement.

Key responsibilities:
- MPU6050 sensor fusion (complementary filter, α = 0.98, 100 Hz)
- BMP280 pressure monitoring and depth estimation
- GPS NMEA parsing (GGA + RMC sentences, checksum verified)
- 2-second wave-filtered trimmed mean water height
- 4-mode flood state machine with persistence debouncing
- Dynamic Firebase push rate (2s to 2 min, based on water level %)
- NTP-synced software RTC (IST, UTC+5:30)
- H_max persistence in NVS (survives power cycles)
- Direct Firebase REST PUT/GET (WiFiClientSecure)
- Hourly IP/RSSI push for dashboard map centering

### XIAO ESP32-C3 — Communication Bridge

The C3 is a **dumb pipe**. It never interprets sensor data — it only moves it. Its primary jobs are receiving the CSV data stream from the S3, forwarding it to Firebase, buffering to SD when offline, and programming the S3 over UART when an OTA update is available. It also interfaces with the **SIM800L GSM module** for cellular connectivity and SMS capabilities as a backup communication channel.

Key responsibilities:
- Software serial receive from S3 GPIO 14 (bit-bang, interrupt-driven, 9600 baud)
- CSV→JSON conversion and Firebase push
- SD card buffering (up to 10,000 lines) when WiFi/GPRS unavailable
- SD flush in batches of 10 lines when connectivity is restored
- Firebase command polling and forwarding to S3 via hardware serial
- SIM800L GSM/GPRS management (cellular uplink, SMS alerts)
- OTA firmware programming of S3 via SLIP bootloader protocol
- S3 BOOT/RESET pin control (GPIO 2/3) for bootloader entry
- $PING ↔ $PONG health check with S3

---

## Inter-MCU Communication

Two physical channels exist between the S3 and C3:

**Channel 1 — Data (unidirectional, S3 → C3)**
- S3 GPIO 14 → C3 GPIO 10
- Software UART, 9600 baud, bit-bang on both ends
- Carries 39-field CSV sensor data rows
- Also carries `$CSV,...` prefixed debug rows and `$DIAG,...` diagnostic frames

**Channel 2 — Commands (bidirectional, S3 ↔ C3)**
- S3 GPIO 43 (TX) / GPIO 44 (RX) ↔ C3 GPIO 21 (RX) / GPIO 20 (TX)
- Hardware UART, 9600 baud
- Carries structured `$`-prefixed protocol messages:

| Message | Direction | Meaning |
|---|---|---|
| `$CFG,<hMax>` | C3 → S3 | Set H_max from website |
| `$SETHMAX,<cm>` | C3 → S3 | Set H_max from website |
| `$REALTIME,<0\|1>` | C3 → S3 | Toggle real-time push mode |
| `$DIAGRUN` | C3 → S3 | Request hardware diagnostic |
| `$PING` / `$PONG` | bidirectional | Health check |
| `$SIMSTAT,<rssi>,<reg>,<avail>` | C3 → S3 | SIM800L status report |
| `$DIAG,...` | S3 → C3 | Full diagnostic result frame |
| `$CSV,...` | S3 → C3 | CSV data frame |
| `$CFG_ACK` / `$HMAX_ACK` | S3 → C3 | Configuration acknowledgement |

---

## Sensing & Signal Processing

### Tether-Angle Water Height Model

VARUNA is a **tethered buoy** — anchored to the river/canal floor at a known tether length. As water rises, the buoy floats upward and the tether angle from vertical increases. The water height is computed from:

```
H = L × cos(θ)
```

Where:
- `H` = water height (cm)
- `L` = tether length / H_max (cm), stored in NVS, configurable from dashboard
- `θ` = combined tilt angle from vertical (degrees), computed by sensor fusion

When the buoy is submerged (`MODE_SUBMERGED`), depth is estimated from the BMP280 gauge pressure:

```
depth_cm = (gauge_pressure_Pa / (ρ × g)) × 100
H = L + depth_cm
```

### Wave Filter — 2-Second Trimmed Mean

Raw tilt-based height measurements are noisy due to wave action. VARUNA collects **200 instantaneous height samples** at 100 Hz over a 2-second window, then:

1. Sorts the 200 samples in ascending order (insertion sort)
2. Discards the **bottom 10%** (wave troughs / tether slack)
3. Discards the **top 10%** (wave crests / spike noise)
4. Averages the remaining **160 samples**

This produces one stable, wave-filtered water height reading every 2 seconds. All flood classification logic operates on this value.

### Sensor Fusion — Complementary Filter

The MPU6050 gyroscope and accelerometer are fused using a complementary filter at 100 Hz:

```
filtTiltX = α × (filtTiltX + gyroX_dps × dt) + (1 - α) × accelTiltX
filtTiltY = α × (filtTiltY + gyroY_dps × dt) + (1 - α) × accelTiltY
```

Where α = 0.98. The gyroscope dominates short-term dynamics (high frequency) while the accelerometer corrects long-term drift (low frequency).

Combined tilt angle from true vertical:
```
θ = √(tiltX² + tiltY²)
```

### Gravity Auto-Zero Calibration

The accelerometer reference is **hardcoded to the ideal vertical orientation** `(0, 0, +1g)`. This eliminates human calibration error entirely — the buoy is designed to float vertically by construction, so any measured tilt IS real displacement from vertical.

Only gyroscope bias offsets are sampled at startup (1000 samples, outlier-rejected). The device must be stationary during gyro calibration but its physical orientation is irrelevant.

---

## Flood Detection Engine

### Operating Modes

The flood state machine classifies the buoy into one of four modes, with persistence debouncing (10 consecutive readings required for mode transition, 3 for submersion):

| Mode | Condition | Meaning |
|---|---|---|
| `MODE_SLACK` (0) | Low lateral accel, low tilt | Tether slack — water below buoy rest level |
| `MODE_TAUT` (1) | Lateral accel > 0.15 m/s², tilt > 3° | Tether taut — buoy is floating and reading water level |
| `MODE_FLOOD` (2) | TAUT + flood ratio > 0.95 | Water level near or at H_max |
| `MODE_SUBMERGED` (3) | BMP280 gauge pressure > 500 Pa | Buoy fully submerged |

### Alert Levels and Zones

| Alert Level | Colour | Trigger |
|---|---|---|
| GREEN | 🟢 | SLACK mode |
| YELLOW | 🟡 | TAUT + flood ratio > 80% |
| RED | 🔴 | FLOOD mode |
| BLACK | ⚫ | SUBMERGED |

| Zone | Flood Ratio | Response Level |
|---|---|---|
| SAFE | < 50% | None |
| WATCH | 50–80% | Monitor |
| WARNING | 80–95% | Prepare |
| CRITICAL | > 95% or SUBMERGED | Act / Evacuate |

### Dynamic Firebase Push Rate

The S3 automatically adjusts how frequently it uploads to Firebase based on water level urgency:

| Water Level (% of H_max) | Push Interval |
|---|---|
| < 50% | Every 2 minutes |
| 50–80% | Every 1 minute |
| > 80% | Every 2 seconds |
| Real-time override (manual) | Every 2 seconds |

---

## Cloud Integration — Firebase

Firebase project: `varuna-git-1e145` (Asia Southeast 1 region)

Both the S3 (directly) and the C3 (via the bridge) write to the same Firebase Realtime Database. The dashboard reads from it.

### Data Paths

| Path | Writer | Content |
|---|---|---|
| `devices/VARUNA_001/raw/latest` | S3 | Full JSON sensor payload (39+ fields) |
| `devices/VARUNA_001/network` | S3 | IP address, WiFi RSSI, timestamp |
| `devices/VARUNA_001/commands/pending` | Dashboard / C3 | Command string for S3 |
| `devices/VARUNA_001/console/command` | Dashboard | Wireless console command |
| `devices/VARUNA_001/console/response` | S3 | Command response from S3 |

### Firebase Payload Fields (raw/latest)

```json
{
  "waterLevel": 142.5,
  "rawAccX": -312, "rawAccY": 88, "rawAccZ": 16201,
  "rawGyroX": 14, "rawGyroY": -6, "rawGyroZ": 3,
  "rawMpuTemp": 2840,
  "pressure": 1012.45,
  "temperature": 28.3,
  "latitude": 17.385044, "longitude": 78.486671,
  "batteryPercent": 87.2,
  "mpuAvailable": true, "bmpAvailable": true,
  "rtcAvailable": true, "gpsAvailable": true,
  "dateTime": "2026-03-25 14:32:10",
  "uptimeMs": 3821493,
  "wifiRSSI": -62
}
```

### Wireless Console

The dashboard's **Remote Console** tab writes a command string to `console/command`. The S3 polls this path every 3 seconds, executes the command, clears the path (to prevent re-execution), and writes the response to `console/response`. The dashboard then reads and displays the response.

This gives full remote access to all debug commands without any physical connection to the buoy.

---

## Web Dashboard

`varuna_dashboard_v32.html` is a single-file, self-contained web application. It connects to Firebase Realtime Database and provides a complete operator interface.

### Panels

| Panel | Description |
|---|---|
| **Overview** | Live KPI tiles (water level, tilt, flood ratio, battery, GPS, mode), 5-KPI sparkline strip, system alert log |
| **Live Data** | Full telemetry table of all fields from the Firebase payload, raw sensor values |
| **Analytics** | Historical charts (water level, pressure, temperature, tilt, battery) with zoom/pan and time range selector (1h / 6h / 24h / all). Data persisted in localStorage (up to 1,440 records). Export as `.ods` spreadsheet |
| **Map** | Live GPS marker on Leaflet map. Falls back to IP geolocation (ip-api.com) before a GPS fix is obtained |
| **Alert Center** | Log of all system alerts with severity classification (Critical / Warning / Info) |
| **Alerting** | EmailJS alert dispatch configuration and manual send |
| **Settings** | H_max configuration (modal numeric input dialog), realtime mode toggle, push rate control |
| **Remote Console** | Live command terminal — send any S3 debug command over Firebase and see the response |
| **About / Landing** | Project overview and capability documentation |

### Alerting System

Automated email alerts are dispatched via **EmailJS** (service ID: `service_57la4ol`) at two thresholds:

| Threshold | Level | Action |
|---|---|---|
| Water level ≥ 80% of H_max | HIGH FLOOD ALERT | Email dispatched automatically |
| Water level ≥ 90% of H_max | EMERGENCY FLOOD | Email dispatched automatically |

Each alert email includes: water level in metres, rate of rise (cm/15 min), GPS/map link, flood zone classification, and timestamp. A configurable cooldown prevents repeated alerts during sustained flood conditions. Manual test sends are also available from the Alerting panel.

### Analytics & Export

The dashboard accumulates all incoming Firebase updates in `localStorage` (capped at 1,440 records, approximately 24 hours at 1-minute push rate). Charts are rendered with Chart.js and support zoom/pan via the chartjs-plugin-zoom. Data can be exported as an `.ods` spreadsheet for offline analysis.

---

## C3 Bridge — SD Buffering & OTA

### Offline Buffering

When the C3 loses WiFi/GPRS connectivity, incoming CSV lines from the S3 are written to `/buffer.csv` on the SD card (SPI, GPIO 4-7). Up to 10,000 lines are retained. When connectivity is restored, the C3 flushes the buffer in batches of 10 lines every 2 seconds to avoid overwhelming Firebase.

### OTA Firmware Update

The C3 can reprogram the S3 remotely. The process:

1. Dashboard writes an OTA command with firmware URL to Firebase
2. C3 downloads the firmware binary from Firebase Storage to the SD card
3. C3 asserts S3 BOOT pin (GPIO 2) LOW and pulses S3 RESET pin (GPIO 3)
4. S3 enters ROM bootloader mode at 115,200 baud
5. C3 transfers the firmware using the ESP32 SLIP bootloader protocol
6. C3 releases BOOT pin, pulses RESET — S3 reboots into new firmware
7. C3 verifies the S3 is alive and reports OTA result to Firebase

The `$CFG` wire (S3 GPIO 44 → C3's existing TX line) is reused as the SLIP bootloader channel, eliminating any extra wiring.

---

## Calibration Procedure

On every boot, the S3 runs an automatic calibration sequence:

1. **Gyro offset calibration** — 1,000 samples collected with the device stationary. Outliers (> 20 °/s) are rejected. Resulting offsets are applied to all subsequent gyro readings. Physical orientation during this step is irrelevant.

2. **Pressure baseline calibration** — 50 BMP280 pressure samples are averaged to establish the local atmospheric baseline. All subsequent pressure readings are compared against this to detect submersion.

3. **H_max load** — The tether length / maximum water height is loaded from NVS. If no value has been saved, it defaults to 200 cm. This value survives power cycles and can be updated from the dashboard or serial console.

To force recalibration at runtime: send `RECALIBRATE` via the serial console or remote wireless console.

---

## Serial Console Commands (S3)

Connect to the S3 via USB Serial at 115,200 baud.

| Command | Description |
|---|---|
| `START` | Begin CSV streaming to serial |
| `STOP` | Stop CSV streaming |
| `PING` | Health check — returns uptime, heap, IP, RSSI |
| `GETSTATUS` | Full flood state, sensor values, push stats |
| `GETCONFIG` | Configuration parameters, calibration state |
| `GETRAW` | Raw IMU/BMP/GPS values |
| `GETTHRESH` | Current flood threshold values |
| `RECALIBRATE` | Re-run gyro + pressure calibration |
| `NTPRESYNC` | Force NTP time sync |
| `DIAGRUN` | Run full hardware diagnostic |
| `REALTIME ON` | Force 2-second push rate |
| `REALTIME OFF` | Restore dynamic push rate |
| `ALGO_ON` / `ALGO_OFF` | Enable/disable advanced flood detection |
| `FORCEUPLOAD` | Immediate Firebase push |
| `SETHMAX=<cm>` | Set tether length (saved to NVS) |
| `SETTHRESH=<a>,<w>,<d>` | Set lateral accel, flood theta, submersion Pa |
| `RESETTHRESH` | Reset all thresholds to firmware defaults |
| `SETRATE=<norm_s>,<high_s>` | Set normal and high push rates in seconds |

All commands are also available remotely via the dashboard's **Remote Console** panel over Firebase.

---

## Diagnostics

The S3 runs a full hardware diagnostic every 24 hours, or on demand (`DIAGRUN`). Each test produces a PASS/FAIL result and a cumulative health score (0–100%).

| Test | Pass Condition |
|---|---|
| MPU WHO_AM_I | Register reads 0x68 |
| Accel magnitude | 0.8 g – 1.2 g (gravity sanity check) |
| Gyro drift | < 5 °/s at rest |
| BMP Chip ID | Register reads 0x58 |
| BMP Pressure | 300 – 1200 hPa |
| BMP Temperature | -40 – 85 °C |
| NTP sync | Time successfully synced |
| GPS data age | Last NMEA sentence < 10 seconds ago |
| Battery voltage | 2.8 – 4.3 V |
| C3 PONG | C3 responds to `$PING` within 2 seconds |

Diagnostic results are printed to the serial console, forwarded to the C3 via the `$DIAG,...` frame, and pushed to Firebase for dashboard visibility.

---

## Pin Reference

### ESP32-S3

| GPIO | Function |
|---|---|
| 2 | Battery ADC (½ voltage divider) |
| 4, 5 | I2C Bus 1 SDA/SCL — BMP280 |
| 6, 7 | UART1 RX/TX — GPS |
| 8, 9 | I2C Bus 0 SDA/SCL — MPU6050 |
| 14 | SW-UART TX → C3 GPIO 10 (CSV data) |
| 43, 44 | UART2 TX/RX ↔ C3 GPIO 20/21 (commands) |

### XIAO ESP32-C3

| GPIO | Function |
|---|---|
| 2 | S3 BOOT pin (OTA programming) |
| 3 | S3 RESET/EN pin (OTA programming) |
| 4, 5, 6, 7 | SD Card SPI (CS, SCK, MOSI, MISO) |
| 10 | SW-UART RX ← S3 GPIO 14 (CSV data) |
| 20, 21 | UART RX/TX ↔ S3 GPIO 43/44 (commands) |

---

## Firmware Overview

| File | Target | Version | Description |
|---|---|---|---|
| `sketch_mar25b_UPDATED.ino` | ESP32-S3 | v3.2 | Sensor brain — fusion, flood detection, Firebase |
| `c3_git_fb_web_ino.ino` | XIAO ESP32-C3 | v2.0 | Communication bridge — SD, OTA, SIM800L |
| `varuna_dashboard_v32.html` | Browser | v32 | Single-file web dashboard |

### Key Libraries (S3)
- `Wire.h` — dual I2C buses
- `WiFi.h`, `HTTPClient.h`, `WiFiClientSecure.h` — Firebase REST
- `Preferences.h` — NVS storage
- `time.h` — NTP / software RTC

### Key Libraries (C3)
- `WiFi.h`, `HTTPClient.h`, `WiFiClientSecure.h` — Firebase REST
- `SD.h`, `SPI.h` — SD card buffering

---

## Project Goals & Context

VARUNA is designed as the proof-of-concept node for **Project Saraswati** — a nationally scalable flood monitoring network intended to fill the coverage gaps in India's existing hydrological infrastructure (CWC gauges, IMD weather stations). Each node operates independently, classifies flood risk entirely on-device, and can deliver alerts even without cloud connectivity. The architecture is designed to scale from a 50-node district pilot to a 5,000-node national grid with a single unified command dashboard for district, state, and national authorities.

The buoy platform targets installation in rivers, canals, and urban flood-prone zones across India's 20 most flood-vulnerable districts, providing real-time early warning to village panchayats, BDOs, tehsildars, SDMA control rooms, and NDRF liaisons.

---

*VARUNA — Built in Hyderabad · [github.com/jden-art/Varuna-autoBackup](https://github.com/jden-art/Varuna-autoBackup)*
