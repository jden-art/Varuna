<div align="center">

# 🌊 VARUNA
### Tethered Autonomous Flood-Monitoring Buoy

![Status](https://img.shields.io/badge/STATUS-ACTIVE-22c55e?style=for-the-badge&logo=statuspage&logoColor=white)
![Platform](https://img.shields.io/badge/PLATFORM-ESP32--S3%20+%20C3-3b82f6?style=for-the-badge&logo=espressif&logoColor=white)
![Cloud](https://img.shields.io/badge/CLOUD-FIREBASE%20RTDB-f59e0b?style=for-the-badge&logo=firebase&logoColor=white)
![Firmware](https://img.shields.io/badge/FIRMWARE-v3.2%20%2F%20v2.0-64748b?style=for-the-badge&logo=git&logoColor=white)

<br/>

**VARUNA** is a **tethered, autonomous flood-monitoring buoy** for real-world deployment in rivers, canals, and urban drainage systems. It senses, classifies, and reports water levels continuously — pushing live telemetry to Firebase and triggering email alerts when flood thresholds are crossed.

---

## ▶️ PROJECT OPERATIONAL VIDEO — Click on image to Watch

[![Watch the VARUNA Demo](https://img.youtube.com/vi/_P6k8nGx6x4/maxresdefault.jpg)](https://youtu.be/_P6k8nGx6x4)

</div>
---

## 📑 Table of Contents

- [System Overview](#-system-overview)
- [Architecture Diagram](#-architecture-diagram)
- [Hardware](#-hardware)
- [Two-MCU Design](#-two-mcu-design)
- [Inter-MCU Communication](#-inter-mcu-communication)
- [Sensing & Signal Processing](#-sensing--signal-processing)
- [Flood Detection Engine](#-flood-detection-engine)
- [Firebase Cloud Integration](#-firebase-cloud-integration)
- [Web Dashboard](#-web-dashboard)
- [SD Buffering & OTA](#-sd-buffering--ota)
- [Calibration](#-calibration)
- [Serial Commands](#-serial-commands)
- [Diagnostics](#-diagnostics)
- [Pin Reference](#-pin-reference)

---

## 🗺 System Overview

```mermaid
flowchart TD
    A[🔬 Sensors] --> B[🧠 ESP32-S3\nFlood Brain]
    B --> C[📡 ESP32-C3\nComms Bridge]
    B <--> D[📶 SIM800L GSM\nGPRS + SMS Alerts]
    C --> E[(🔥 Firebase RTDB)]
    E --> F[📊 Dashboard]
```
Sensors → ESP32-S3 (Flood Brain) → ESP32-C3 (Comms Bridge) → Firebase RTDB → Dashboard
                                         ↕
                                     SIM800L GSM
                                   (GPRS + SMS alerts)

Three layers, one purpose: **detect flooding early and report it reliably.**

| Layer | Component | Role |
|---|---|---|
| 🧠 Sensing | ESP32-S3 | All sensor fusion, flood classification, Firebase push |
| 📡 Comms | XIAO ESP32-C3 + SIM800L | Data relay, SD buffering, OTA, cellular uplink |
| ☁️ Cloud | Firebase RTDB | Telemetry store, command channel |
| 🖥️ Dashboard | `varuna_dashboard_v32.html` | Live monitoring, alerts, analytics, remote console |

---

## 🔷 Architecture Diagram

```mermaid
flowchart TD
    subgraph BUOY["🔵 VARUNA Buoy"]
        subgraph S3["ESP32-S3 · Sensor Brain v3.2"]
            MPU["📐 MPU6050\nIMU · 100 Hz fusion"]
            BMP["🌡️ BMP280\nPressure + Temp"]
            GPS["🛰️ GPS Module\nNMEA GGA/RMC"]
            BAT["🔋 Battery ADC\n16-sample avg"]
            FUSION["⚙️ Complementary Filter\nα = 0.98 · 100 Hz"]
            WAVE["🌊 Wave Filter\n200 samples · 2s window\n10% trimmed mean"]
            FSM["🚦 Flood FSM\n4 modes · persistence debounce"]
            FB_S3["☁️ Firebase Push\nDynamic rate: 2s – 2min"]
            WCMD["📟 Wireless Console\nPoll every 3s"]
        end

        subgraph C3["XIAO ESP32-C3 · Comms Bridge v2.0"]
            SIM["📶 SIM800L\nGSM/GPRS + SMS"]
            SD["💾 SD Card\nOffline buffer ≤10k rows"]
            OTA["🔧 OTA Programmer\nSLIP bootloader"]
            FB_C3["☁️ Firebase Bridge\nCSV → JSON push"]
        end

        MPU --> FUSION
        BMP --> FSM
        GPS --> FSM
        BAT --> FB_S3
        FUSION --> WAVE
        WAVE --> FSM
        FSM --> FB_S3
        FSM --> WCMD

        S3 -- "GPIO 14\nSW-UART 9600\nCSV data" --> C3
        S3 -- "GPIO 43/44\nHW-UART 9600\nCommands" --> C3
        C3 -- "Commands\n$CFG $PING $DIAG" --> S3

        SIM --> FB_C3
        SD --> FB_C3
    end

    FB_S3 --> RTDB[("🔥 Firebase RTDB\nvaruna-git-1e145")]
    FB_C3 --> RTDB
    RTDB --> DASH["🖥️ Web Dashboard v32\nLive · Map · Alerts · Console"]
    DASH -- "Remote commands\nconsole/command" --> RTDB
    RTDB -- "Poll\nconsole/response" --> WCMD
```

---

## 🔩 Hardware

| # | Component | Interface | Notes |
|---|---|---|---|
| 1 | **ESP32-S3** | — | Sensor brain, WiFi to Firebase |
| 2 | **XIAO ESP32-C3** | — | Comms bridge, OTA programmer |
| 3 | **MPU6050** | I²C Bus 0 · GPIO 8/9 | ±2g accel, ±250°/s gyro, 200 Hz, DLPF 44 Hz |
| 4 | **BMP280** | I²C Bus 1 · GPIO 4/5 | Pressure ×16 oversample, filter ×16 |
| 5 | **GPS Module** | UART1 · GPIO 6/7 | GGA + RMC, checksum verified |
| 6 | **SIM800L** | Via C3 | GSM/GPRS uplink + SMS alerts |
| 7 | **SD Card** | SPI · C3 GPIO 4–7 | Offline CSV buffer |
| 8 | **Li-Ion Battery** | ADC · S3 GPIO 2 | ×2 voltage divider, 16-sample avg |

---

## 🧠 Two-MCU Design

### ESP32-S3 — The Brain

Runs all sensing and intelligence. The C3 **never** sees or interprets sensor data.

- ✅ MPU6050 + BMP280 + GPS + Battery sensing
- ✅ 100 Hz complementary filter sensor fusion
- ✅ 2-second wave-filtered water height (trimmed mean)
- ✅ 4-mode flood state machine with persistence debounce
- ✅ Dynamic Firebase push rate (2 s → 2 min)
- ✅ NTP-synced software RTC (IST, UTC+5:30)
- ✅ H_max persisted in NVS (survives reboots)
- ✅ Direct Firebase REST `PUT` / `GET` over TLS
- ✅ Wireless console via Firebase polling (every 3 s)

### XIAO ESP32-C3 — The Pipe

A dumb relay. Moves data, doesn't think about it.

- ✅ Bit-bang SW-UART RX from S3 GPIO 14 (interrupt-driven)
- ✅ CSV → JSON conversion and Firebase push
- ✅ SD card offline buffer (≤ 10,000 rows)
- ✅ Batch flush when connectivity restores (10 rows / 2 s)
- ✅ SIM800L GSM/GPRS management + SMS
- ✅ Firebase command polling → forward to S3
- ✅ OTA firmware programming of S3 via SLIP bootloader
- ✅ `$PING` ↔ `$PONG` health check with S3

---

## 📡 Inter-MCU Communication

Two physical wires between S3 and C3:

```mermaid
sequenceDiagram
    participant S3 as ESP32-S3
    participant C3 as XIAO ESP32-C3

    Note over S3,C3: Channel 1 — GPIO 14 → GPIO 10 (SW-UART 9600 baud)
    S3->>C3: CSV row (39 fields, every push interval)
    S3->>C3: $DIAG,MPU_ID=1,ACCEL=0.99:1,...

    Note over S3,C3: Channel 2 — GPIO 43/44 ↔ GPIO 20/21 (HW-UART 9600 baud)
    C3->>S3: $CFG,200.0
    S3->>C3: $CFG_ACK
    C3->>S3: $PING
    S3->>C3: $PONG
    C3->>S3: $SIMSTAT,18,1,1
    C3->>S3: $REALTIME,1
    S3->>C3: $REALTIME_ACK,1
```

### Command Protocol

| Message | Direction | Purpose |
|---|---|---|
| `$CFG,<hMax>` | C3 → S3 | Set tether length / H_max |
| `$SETHMAX,<cm>` | C3 → S3 | Set H_max from dashboard |
| `$REALTIME,<0\|1>` | C3 → S3 | Toggle real-time push mode |
| `$DIAGRUN` | C3 → S3 | Request hardware diagnostic |
| `$PING` / `$PONG` | Both | Health check |
| `$SIMSTAT,<rssi>,<reg>,<avail>` | C3 → S3 | SIM800L status update |
| `$CFG_ACK` / `$HMAX_ACK` | S3 → C3 | Config acknowledged |
| `$DIAG,...` | S3 → C3 | Full diagnostic result frame |

---

## ⚙️ Sensing & Signal Processing

### 📐 Tether-Angle Water Height Model

VARUNA is anchored at a known tether length `L`. As water rises, the buoy floats and the tether angle `θ` increases:

```
H = L × cos(θ)
```

| Symbol | Meaning |
|---|---|
| `H` | Water height (cm) |
| `L` | Tether length = H_max (cm), stored in NVS |
| `θ` | Combined tilt angle from vertical (degrees) |

When **submerged**, BMP280 gauge pressure is used instead:

```
depth_cm = (gauge_pressure_Pa / (ρ × g)) × 100
H        = L + depth_cm
```

---

### 🌊 Wave Filter — 2-Second Trimmed Mean

```mermaid
flowchart LR
    A["100 Hz samples"] --> B["Buffer\n200 samples\n2 seconds"]
    B --> C["Sort ascending"]
    C --> D["Drop bottom 20\nwave troughs"]
    C --> E["Drop top 20\nwave crests"]
    D --> F["Average\nmiddle 160"]
    E --> F
    F --> G["✅ Stable\nwater height"]
```

> Every flood decision is made on this filtered value — never a raw reading.

---

### 🔄 Complementary Filter — 100 Hz Sensor Fusion

```
filtTiltX = 0.98 × (filtTiltX + gyroX × dt) + 0.02 × accelTiltX
filtTiltY = 0.98 × (filtTiltY + gyroY × dt) + 0.02 × accelTiltY
θ         = √(tiltX² + tiltY²)
```

| Parameter | Value |
|---|---|
| Alpha (α) | `0.98` — 98% gyro, 2% accel correction |
| Fusion rate | `100 Hz` (10,000 µs interval) |
| Gyro range | `±250 °/s` |
| Accel range | `±2 g`, DLPF 44 Hz |

---

### 🎯 Gravity Auto-Zero Calibration

```mermaid
flowchart LR
    A["⚡ Boot"] --> B["Gyro calibration\n1,000 samples\nDevice must be still\nOrientation irrelevant\nOutliers rejected"]
    B --> D["Store gyro\noffsets X/Y/Z"]
    A --> E["Accel ref\nHARDCODED\n0, 0, +1g\nNo user input"]
    D --> F["✅ Done"]
    E --> F
```

> The accelerometer reference is hardcoded to ideal vertical. No human calibration error possible.

---

## 🚦 Flood Detection Engine

### Operating Modes

```mermaid
stateDiagram-v2
    [*] --> SLACK : Boot

    SLACK --> TAUT : lateral_accel > 0.15\nAND tilt > 3°\n(10 consecutive readings)
    TAUT --> FLOOD : flood_ratio > 0.95\n(10 readings)
    TAUT --> SLACK : accel + tilt drop
    FLOOD --> TAUT : ratio drops
    FLOOD --> SUBMERGED : gauge_pressure > 500 Pa\n(3 readings)
    TAUT --> SUBMERGED : gauge_pressure > 500 Pa
    SUBMERGED --> TAUT : pressure drops

    SLACK : 🔵 SLACK\nTether loose
    TAUT : 🟢 TAUT\nReading water level
    FLOOD : 🟠 FLOOD\nNear H_max
    SUBMERGED : 🔴 SUBMERGED\nBuoy underwater
```

### Alert Levels

| Alert | Mode | Flood Ratio |
|---|---|---|
| 🟢 GREEN | SLACK | any |
| 🟡 YELLOW | TAUT | > 80% |
| 🔴 RED | FLOOD | > 95% |
| ⚫ BLACK | SUBMERGED | — |

### Zones & Response

| Zone | Flood Ratio | Response |
|---|---|---|
| 🟢 SAFE | < 50% | None |
| 🔵 WATCH | 50 – 80% | Monitor |
| 🟠 WARNING | 80 – 95% | Prepare |
| 🔴 CRITICAL | > 95% or submerged | Act / Evacuate |

### ⚡ Dynamic Push Rate

```
 0% ──────────────── 50% ──────────────── 80% ──── 100%
      Every 2 min         Every 1 min       Every 2s
      (calm, save data)   (watch closely)   (urgent!)
```

---

## ☁️ Firebase Cloud Integration

**Project:** `varuna-git-1e145` · Asia Southeast 1

### Path Map

```
/devices/VARUNA_001/
├── raw/latest        ← S3 overwrites on every push
├── network           ← S3 writes IP + RSSI hourly
├── console/
│   ├── command       ← Dashboard writes command string
│   └── response      ← S3 writes response
└── commands/pending  ← C3 forwards config commands
```

### Payload Sample (`raw/latest`)

```json
{
  "waterLevel":     142.5,
  "rawAccX":        -312,  "rawAccY": 88,   "rawAccZ": 16201,
  "rawGyroX":        14,   "rawGyroY": -6,  "rawGyroZ": 3,
  "pressure":       1012.45,
  "temperature":      28.3,
  "latitude":       17.385044,
  "longitude":      78.486671,
  "batteryPercent":  87.2,
  "mpuAvailable":   true,  "bmpAvailable": true,
  "rtcAvailable":   true,  "gpsAvailable": true,
  "dateTime":       "2026-03-25 14:32:10",
  "uptimeMs":       3821493,
  "wifiRSSI":       -62
}
```

### 📟 Wireless Console Flow

```mermaid
sequenceDiagram
    participant D as 🖥️ Dashboard
    participant F as 🔥 Firebase
    participant S3 as 🧠 ESP32-S3

    D->>F: Write "GETSTATUS" → console/command
    Note over S3: Polls every 3 seconds
    S3->>F: Read console/command
    S3->>F: Clear → null (prevents re-execution)
    S3->>S3: Execute command locally
    S3->>F: Write response → console/response
    F->>D: Dashboard reads + displays response
```

---

## 🖥️ Web Dashboard

Single self-contained file — `varuna_dashboard_v32.html`. 

| Panel | Contents |
|---|---|
| 🏠 **Overview** | Live KPI tiles · sparkline strip · alert log |
| 📊 **Live Data** | Full Firebase payload table |
| 📈 **Analytics** | Charts (water, pressure, temp, tilt, battery) · zoom/pan · 1h/6h/24h/all · localStorage (1,440 records) · `.ods` export |
| 🗺️ **Map** | GPS marker (Leaflet) · IP geolocation fallback pre-fix |
| 🔔 **Alert Center** | Classified alert log (Critical / Warning / Info) |
| 📧 **Alerting** | EmailJS auto-dispatch + manual send |
| ⚙️ **Settings** | H_max · realtime toggle · push rate |
| 💻 **Remote Console** | Firebase-backed live command terminal |

### 📧 Auto Email Alerts

| Threshold | Level |
|---|---|
| Water ≥ **80%** of H_max | 🟠 HIGH FLOOD ALERT — email dispatched |
| Water ≥ **90%** of H_max | 🔴 EMERGENCY FLOOD — email dispatched |

Each email contains: water level in metres · rise rate (cm/15 min) · GPS map link · zone · timestamp.

> EmailJS service: `service_57la4ol`. Configurable cooldown prevents alert spam.

---

## 💾 SD Buffering & OTA

### Offline Buffer

```mermaid
flowchart TD
    A["S3 sends CSV row"] --> B{"Connected?"}
    B -- Yes --> C["Push to Firebase"]
    B -- No --> D["Write to SD\n/buffer.csv"]
    D --> E["Hold up to\n10,000 rows"]
    E --> F{"Connection\nrestored?"}
    F -- Yes --> G["Flush 10 rows\nevery 2 seconds"]
    G --> C
```

### OTA Update Flow

```mermaid
flowchart LR
    A["Dashboard\nwrites OTA URL"] --> B["C3 downloads\nbinary → SD"]
    B --> C["BOOT pin LOW\nGPIO 2"]
    C --> D["Pulse RESET\nGPIO 3"]
    D --> E["S3 ROM\nbootloader\n115,200 baud"]
    E --> F["SLIP transfer\nfirmware"]
    F --> G["Release BOOT\nPulse RESET"]
    G --> H["S3 reboots\nnew firmware"]
    H --> I["✅ Report\nto Firebase"]
```

> The `$CFG` wire (S3 GPIO 44) doubles as the SLIP channel — no extra wiring needed.

---

## 🎛️ Calibration

```mermaid
flowchart LR
    A["⚡ Boot"] --> B["1️⃣ Gyro Cal\n1,000 samples\nDevice stationary\nStores X/Y/Z offsets"]
    B --> C["2️⃣ Pressure Baseline\n50 BMP280 samples\nLocal atmospheric ref"]
    C --> D["3️⃣ H_max\nLoad from NVS\nDefault: 200 cm"]
    D --> E["✅ Ready"]
```

To recalibrate at any time: send `RECALIBRATE` via serial or Remote Console.

---

## 💻 Serial Commands (115,200 baud)

### Status

| Command | Description |
|---|---|
| `PING` | Uptime · heap · IP · RSSI |
| `GETSTATUS` | Full flood state + sensor snapshot |
| `GETCONFIG` | All config + calibration state |
| `GETRAW` | Raw IMU / BMP / GPS values |
| `GETTHRESH` | Current threshold values |

### Control

| Command | Description |
|---|---|
| `RECALIBRATE` | Re-run gyro + pressure calibration |
| `NTPRESYNC` | Force NTP time sync |
| `DIAGRUN` | Run full hardware diagnostic |
| `REALTIME ON` / `OFF` | Force / restore dynamic push rate |
| `ALGO_ON` / `ALGO_OFF` | Enable / disable advanced detection |
| `FORCEUPLOAD` | Immediate Firebase push |
| `START` / `STOP` | CSV streaming to serial |

### Configuration

| Command | Description |
|---|---|
| `SETHMAX=<cm>` | Set tether length (saved to NVS) |
| `SETTHRESH=<accel>,<theta_deg>,<Pa>` | Set flood thresholds |
| `RESETTHRESH` | Restore firmware defaults |
| `SETRATE=<norm_s>,<high_s>` | Set normal / high push intervals |

> All commands work over the dashboard **Remote Console** — no USB needed.

---

## 🔬 Diagnostics

Runs every **24 hours**, or on demand (`DIAGRUN`).

| Test | Pass Condition |
|---|---|
| 🔵 MPU WHO_AM_I | Register = `0x68` |
| 🔵 Accel magnitude | `0.8 g – 1.2 g` |
| 🔵 Gyro drift | `< 5 °/s` at rest |
| 🟢 BMP Chip ID | Register = `0x58` |
| 🟢 BMP Pressure | `300 – 1,200 hPa` |
| 🟢 BMP Temperature | `-40 – 85 °C` |
| 🕐 NTP Sync | Time successfully synced |
| 🛰️ GPS Data Age | Last NMEA sentence `< 10 s` ago |
| 🔋 Battery Voltage | `2.8 – 4.3 V` |
| 📡 C3 PONG | Response within `2 s` |

Health score (0–100%) = MPU 40% + BMP 30% + NTP 20% + GPS 10%.

Results are printed to serial, forwarded to C3 via `$DIAG,...`, and pushed to Firebase.

---

## 📌 Pin Reference

### ESP32-S3

| GPIO | Role | Dir |
|---|---|---|
| `2` | Battery ADC (½ divider) | IN |
| `4` / `5` | I²C Bus 1 SDA/SCL — BMP280 | I/O |
| `6` / `7` | UART1 RX/TX — GPS | IN/OUT |
| `8` / `9` | I²C Bus 0 SDA/SCL — MPU6050 | I/O |
| `14` | SW-UART TX → C3 GPIO 10 | OUT |
| `43` / `44` | UART2 TX/RX ↔ C3 GPIO 20/21 | OUT/IN |

### XIAO ESP32-C3

| GPIO | Role | Dir |
|---|---|---|
| `2` | S3 BOOT (OTA) | OUT |
| `3` | S3 RESET/EN (OTA) | OUT |
| `4` `5` `6` `7` | SD Card SPI CS/SCK/MOSI/MISO | I/O |
| `10` | SW-UART RX ← S3 GPIO 14 | IN |
| `20` / `21` | UART RX/TX ↔ S3 GPIO 43/44 | IN/OUT |

---

## 📁 File Reference

| File | Target | Version |
|---|---|---|
| `sketch_mar25b_UPDATED.ino` | ESP32-S3 | v3.2 |
| `c3_git_fb_web_ino.ino` | XIAO ESP32-C3 | v2.0 |
| `varuna_dashboard_v32.html` | Browser | v32 |

---

*🌊 VARUNA · Built in Hyderabad, India · [github.com/jden-art/Varuna-autoBackup](https://github.com/jden-art/Varuna-autoBackup)*
