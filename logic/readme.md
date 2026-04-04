# ESP32-S3 VARUNA BUOY FIRMWARE ANALYSIS

## OVERVIEW
This is a **flood monitoring buoy system** that uses sensor fusion, wave filtering, and cloud connectivity to detect rising water levels in real-time.

---

## CORE FEATURES

### 🌊 1. **WAVE-FILTERED WATER LEVEL MEASUREMENT**
- **200-sample trimmed mean filter** over 2-second windows
- Removes wave crests (top 10%) and troughs (bottom 10%)
- Averages middle 160 samples for stable water height
- **Eliminates wave noise** for accurate flood detection

### 📐 2. **GRAVITY-BASED AUTO-ZERO CALIBRATION**
```cpp
const float refAccX = 0.0f;  // Ideal: no lateral gravity
const float refAccY = 0.0f;  
const float refAccZ = 1.0f;  // Full gravity on Z = vertical
```
- **Zero human error**: Accelerometer reference is hardcoded to ideal vertical orientation (0,0,1g)
- Only gyro offsets are calibrated (per-chip bias)
- Orientation doesn't matter during calibration—just stillness

### 🎯 3. **4-MODE FLOOD CLASSIFICATION**
| Mode | Trigger | Meaning |
|------|---------|---------|
| **SLACK** | Low tilt, low accel | Cable loose—water below sensor |
| **TAUT** | High tilt, high accel | Cable tight—water rising |
| **FLOOD** | Taut + ratio >95% | Water near critical height |
| **SUBMERGED** | Pressure spike >500Pa | Buoy underwater |

### 🚨 4. **DYNAMIC FIREBASE PUSH RATES**
```cpp
Water < 50% → Push every 120 seconds
Water 50-80% → Push every 60 seconds  
Water > 80%  → Push every 2 seconds (REALTIME)
```

### 🧮 5. **SENSOR FUSION (100Hz Complementary Filter)**
- **98% gyro integration** + **2% accelerometer correction**
- Combines MPU6050 (accel/gyro) + BMP280 (pressure/temp)
- Outputs:
  - `combinedTheta`: Total tilt from vertical
  - `lateralAccel`: Horizontal acceleration (cable tension)
  - `waterHeightCm`: Calculated water level

---

## HARDWARE COMPONENTS

### **I²C Bus 0 (MPU6050)**
- **MPU6050** @ GPIO 8/9 (400kHz)
  - ±2g accelerometer
  - ±250°/s gyroscope
  - 200Hz sample rate, 44Hz low-pass filter

### **I²C Bus 1 (BMP280)**
- **BMP280** @ GPIO 4/5 (100kHz)
  - Pressure: ×16 oversampling
  - Temperature: ×2 oversampling
  - Detects submersion via gauge pressure

### **GPS (NEO-6M)**
- Serial1 @ GPIO 6/7 (9600 baud)
- Parses NMEA GGA/RMC sentences
- Provides lat/lon/altitude/fix quality

### **Battery Monitor**
- ADC @ GPIO 2
- 12-bit resolution, 2× voltage divider
- Reports voltage + percentage (3.0V–4.2V)

### **ESP32-C3 Communication**
- **Hardware Serial** (CMD) @ GPIO 43/44
- **Software UART** (DATA) @ GPIO 14
- Exchanges diagnostics, config, CSV streams

---

## CONNECTIVITY

### **WiFi**
```cpp
SSID: TPLink_2G
Firebase: varuna-git-1e145...firebasedatabase.app
Device ID: VARUNA_001
```

### **Firebase Real-Time Database**
- **Pushes to**: `devices/VARUNA_001/raw/latest`
- **Polls from**: `devices/VARUNA_001/console/command` (wireless commands)
- **Writes to**: `devices/VARUNA_001/console/response` (command results)
- **Network info**: `devices/VARUNA_001/network` (IP, RSSI)

### **NTP Time Sync**
- Servers: `pool.ntp.org`, `time.nist.gov`
- GMT+5.5 (19800 sec offset)
- Software RTC maintained after sync

---

## ADVANCED ALGORITHMS

### **Water Height Calculation**
```cpp
if (MODE_SUBMERGED):
    height = olpLength + pressureDepth
else if (MODE_TAUT/FLOOD):
    height = olpLength × cos(theta)  // Tether geometry
else:
    height = 0  // SLACK mode
```

### **Flood Ratio**
```cpp
floodRatio = waterHeightCm / hMaxCm
if ratio > 0.95 → MODE_FLOOD
if ratio > 0.80 → ALERT_YELLOW
```

### **Alert Levels**
- **GREEN**: Normal (SLACK/low TAUT)
- **YELLOW**: Watch (TAUT + ratio >80%)
- **RED**: Flood imminent (FLOOD mode)
- **BLACK**: Buoy submerged (emergency)

---

## COMMAND INTERFACES

### **USB Serial Commands**
| Command | Action |
|---------|--------|
| `START` | Begin CSV streaming |
| `STOP` | Stop CSV streaming |
| `SETHMAX=200` | Set max water height to 200cm |
| `RECALIBRATE` | Re-zero gyro + pressure baseline |
| `GETSTATUS` | Print all sensor states |
| `DIAGRUN` | Full hardware diagnostic |
| `REALTIME ON` | Force 2s push rate |

### **Firebase Wireless Console**
- Same commands as USB, executed remotely
- Polls `console/command` every 3 seconds
- Writes responses to `console/response`
- **Guard**: Rejects commands <4 chars (prevents stale garbage)

### **C3 Inter-Processor Commands**
- `$CFG,<hMax>` — Update max height from C3
- `$PING` / `$PONG` — Health check
- `$SIMSTAT,<rssi>,<reg>,<avail>` — SIM status update

---

## DATA OUTPUTS

### **Firebase JSON Payload**
```json
{
  "waterLevel": 145.32,      // cm
  "waterHeight": 145.32,     // Alias for backwards compat
  "olpLength": 200.00,       // Tether length (for % calc)
  "rawAccX": -234,           // Raw sensor values
  "pressure": 1013.25,       // hPa
  "latitude": 12.9716,       
  "batteryPercent": 87.3,
  "wifiIP": "192.168.1.42",  // Device IP
  "dateTime": "2024-01-15 14:23:11",
  "uptimeMs": 3847291
}
```

### **CSV Format** (45 fields)
```
theta,height,tiltX,tiltY,olpLen,horizDist,pressure,temp,...
```

---

## DIAGNOSTICS

### **Auto-Run Every 24 Hours**
Tests:
- ✅ MPU6050 WHO_AM_I register
- ✅ Accelerometer magnitude (0.8–1.2g)
- ✅ Gyro drift (<5°/s)
- ✅ BMP280 chip ID
- ✅ Pressure range (300–1200 hPa)
- ✅ NTP sync status
- ✅ GPS fix recency (<10s old)
- ✅ Battery voltage (2.8–4.3V)
- ✅ C3 PONG response (<2s)

**Health Score**: 0–100% based on passed tests  
**Fault Count**: Total failed checks

---

## KEY INNOVATIONS

### 🎯 **Zero-Error Calibration**
- Accel reference = mathematical ideal (0,0,1g)
- No user sampling required
- Orientation-independent gyro cal

### 🌊 **Wave Noise Rejection**
- 10% trimmed mean filter
- 2-second statistical window
- Stable output immune to wave action

### ⚡ **Adaptive Sampling**
- Slow when safe (2 min)
- Fast when critical (2 sec)
- Saves bandwidth + battery

### 🔧 **Remote Management**
- Wireless recalibration
- Config changes via Firebase
- Live diagnostics without physical access

---

## SYSTEM ARCHITECTURE

```
┌─────────────────────────────────────────────┐
│         ESP32-S3 (Sensor Brain)             │
├─────────────────────────────────────────────┤
│ ┌─────────┐  ┌─────────┐  ┌──────────┐     │
│ │ MPU6050 │  │ BMP280  │  │ GPS NEO  │     │
│ │ Accel/  │  │ Pressure│  │ Position │     │
│ │ Gyro    │  │ /Temp   │  │ /Time    │     │
│ └────┬────┘  └────┬────┘  └─────┬────┘     │
│      │            │              │          │
│      └────────────┴──────────────┘          │
│                   │                         │
│           ┌───────▼────────┐                │
│           │ Sensor Fusion  │ 100Hz          │
│           │ Wave Filter    │ 200 samples    │
│           │ Flood Detect   │ 4 modes        │
│           └───────┬────────┘                │
│                   │                         │
│      ┌────────────┴───────────┐             │
│      │                        │             │
│  ┌───▼────┐            ┌──────▼──────┐     │
│  │ WiFi   │            │ ESP32-C3    │     │
│  │Firebase│            │ (LTE/SMS)   │     │
│  └────────┘            └─────────────┘     │
└─────────────────────────────────────────────┘
         │                      │
         ▼                      ▼
   Cloud Dashboard         SMS Alerts
```

---

## CONFIGURATION PERSISTENCE

**NVS (Non-Volatile Storage)**:
- `hMaxCm` — Maximum water height (critical threshold)
- `normalRate` — Normal push interval (seconds)
- `highRate` — High-flood push interval (seconds)

**Survives**: Power loss, resets, firmware updates

---

