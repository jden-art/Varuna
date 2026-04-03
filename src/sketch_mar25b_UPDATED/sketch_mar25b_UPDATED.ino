// ============================================================================
// VARUNA BUOY — ESP32-S3 SENSOR BRAIN FIRMWARE v3.2
// ============================================================================
// Raw sensor data → Firebase (latest only, overwrite)
// Gravity-based auto-zero calibration (no user accel sampling)
// Wave-filtered trimmed mean water level (200 samples / 2s window)
// All processing retained locally for flood detection & dynamic sampling
// CSV output on USB Serial only when triggered by debugger
// NTP-synced software RTC
// DS1307 hardware DISABLED
// ============================================================================

#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include <Preferences.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define MPU_SDA          8
#define MPU_SCL          9
#define SENS_SDA         4
#define SENS_SCL         5
#define GPS_RX_PIN       6
#define GPS_TX_PIN       7
#define C3_DATA_PIN      14
#define C3_CMD_TX_PIN    43
#define C3_CMD_RX_PIN    44
#define BATTERY_ADC_PIN  2

// ============================================================================
// I2C ADDRESSES
// ============================================================================
#define MPU6050_ADDR     0x68
#define BMP280_ADDR      0x76

// ============================================================================
// MPU6050 REGISTERS
// ============================================================================
#define MPU_REG_WHO_AM_I       0x75
#define MPU_REG_PWR_MGMT_1     0x6B
#define MPU_REG_PWR_MGMT_2     0x6C
#define MPU_REG_SMPLRT_DIV     0x19
#define MPU_REG_CONFIG         0x1A
#define MPU_REG_GYRO_CONFIG    0x1B
#define MPU_REG_ACCEL_CONFIG   0x1C
#define MPU_REG_ACCEL_XOUT_H   0x3B
#define MPU_WHO_AM_I_VAL       0x68

// ============================================================================
// BMP280 REGISTERS
// ============================================================================
#define BMP280_REG_CHIP_ID     0xD0
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_CONFIG_REG  0xF5
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_CALIB_START 0x88
#define BMP280_CHIP_ID_VAL     0x58

// ============================================================================
// SENSOR FUSION CONSTANTS
// ============================================================================
#define ALPHA                  0.98f
#define GYRO_SENSITIVITY       131.0f
#define ACCEL_SENSITIVITY      16384.0f
#define GRAVITY_MS2            9.80665f
#define FUSION_INTERVAL_US     10000

// ============================================================================
// CALIBRATION CONSTANTS (gyro only — accel is hardcoded ideal)
// ============================================================================
#define GYRO_CAL_SAMPLES       1000
#define PRESSURE_CAL_SAMPLES   50

// ============================================================================
// FLOOD DETECTION THRESHOLDS (defaults)
// ============================================================================
#define DEFAULT_LATERAL_ACCEL_TAUT   0.15f
#define DEFAULT_LATERAL_ACCEL_SLACK  0.10f
#define DEFAULT_TILT_TAUT_DEG        3.0f
#define DEFAULT_FLOOD_THETA_DEG      10.0f
#define DEFAULT_FLOOD_RATIO          0.95f
#define DEFAULT_SUBMERSION_PA        500.0f
#define WATER_DENSITY_KGM3           1000.0f
#define MODE_PERSISTENCE_COUNT       10
#define SUBMERGE_PERSISTENCE_COUNT   3

// ============================================================================
// DYNAMIC FIREBASE PUSH RATES (based on water level %)
// ============================================================================
#define PUSH_RATE_BELOW_50_MS    120000UL
#define PUSH_RATE_50_TO_80_MS    60000UL
#define PUSH_RATE_ABOVE_80_MS    2000UL
#define PUSH_RATE_REALTIME_MS    2000UL

// ============================================================================
// WAVE FILTER CONSTANTS
// ============================================================================
#define WAVE_BUFFER_SIZE         200    // 100Hz fusion x 2 second window
#define WAVE_TRIM_COUNT          20     // 10% of 200 = 20 removed from EACH end

// ============================================================================
// CONTINUOUS MONITORING INTERVAL
// ============================================================================
#define MONITOR_INTERVAL_MS      2000

// ============================================================================
// DIAGNOSTIC CONSTANTS
// ============================================================================
#define DIAG_INTERVAL_MS       86400000UL
#define PONG_TIMEOUT_MS        2000

// ============================================================================
// SOFTWARE UART CONSTANTS
// ============================================================================
#define SW_UART_BAUD           9600
#define SW_UART_BIT_TIME_US    104

// ============================================================================
// WIFI + FIREBASE CONFIG
// ============================================================================
#define S3_WIFI_SSID              "POCO M7"
#define S3_WIFI_PASSWORD          "99990000"
#define S3_FIREBASE_HOST          "varuna-git-1e145-default-rtdb.asia-southeast1.firebasedatabase.app"
#define S3_FIREBASE_AUTH          "AIzaSyAAvL-yCNXxDglQYc5aIMVhUZCJ7XXIOVo"
#define S3_DEVICE_ID              "VARUNA_001"
#define S3_WIFI_RECONNECT_MS      30000
#define S3_FIREBASE_TIMEOUT_MS    10000

// ============================================================================
// NTP CONFIG
// ============================================================================
#define NTP_SERVER_1             "pool.ntp.org"
#define NTP_SERVER_2             "time.nist.gov"
#define NTP_GMT_OFFSET_SEC       19800
#define NTP_DAYLIGHT_OFFSET_SEC  0

// ============================================================================
// GPS PARSING
// ============================================================================
#define GPS_BUFFER_SIZE        256
#define NMEA_MAX_FIELDS        20

// ============================================================================
// ENUMS
// ============================================================================
enum FloodMode {
    MODE_SLACK     = 0,
    MODE_TAUT      = 1,
    MODE_FLOOD     = 2,
    MODE_SUBMERGED = 3
};

enum AlertLevel {
    ALERT_GREEN    = 0,
    ALERT_YELLOW   = 1,
    ALERT_RED      = 2,
    ALERT_BLACK    = 3
};

enum WaterZone {
    ZONE_SAFE      = 0,
    ZONE_WATCH     = 1,
    ZONE_WARNING   = 2,
    ZONE_CRITICAL  = 3
};

enum ResponseLevel {
    RESPONSE_NONE     = 0,
    RESPONSE_MONITOR  = 1,
    RESPONSE_PREPARE  = 2,
    RESPONSE_ACT      = 3,
    RESPONSE_EVACUATE = 4
};

// ============================================================================
// BMP280 CALIBRATION STRUCTURE
// ============================================================================
struct BMP280CalibData {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    int32_t  t_fine;
};

// ============================================================================
// GPS DATA STRUCTURE
// ============================================================================
struct GPSData {
    float    latitude;
    float    longitude;
    float    altitude;
    int      satellites;
    bool     fixValid;
    int      fixQuality;
    uint8_t  hour, minute, second;
    uint8_t  day, month;
    uint16_t year;
    bool     timeValid;
    bool     dateValid;
    unsigned long lastUpdateMs;
};

// ============================================================================
// DIAGNOSTIC RESULTS STRUCTURE
// ============================================================================
struct DiagResult {
    bool  mpuWhoAmIOk;
    bool  mpuAccelMagnitudeOk;
    bool  mpuGyroDriftOk;
    float mpuAccelMagnitude;
    float mpuGyroDrift;
    bool  bmpChipIdOk;
    bool  bmpPressureRangeOk;
    bool  bmpTempRangeOk;
    float bmpPressure;
    float bmpTemperature;
    bool  gpsDataRecent;
    int   gpsSatCount;
    bool  batteryVoltageOk;
    float batteryVoltage;
    bool  c3PongReceived;
    bool  ntpSynced;
    int   totalFaults;
    int   healthScore;
};

// ============================================================================
// I2C BUS INSTANCES
// ============================================================================
TwoWire I2C_MPU  = TwoWire(0);
TwoWire I2C_SENS = TwoWire(1);

// ============================================================================
// NVS STORAGE
// ============================================================================
Preferences preferences;

// ============================================================================
// SENSOR AVAILABILITY FLAGS
// ============================================================================
bool mpuAvailable  = false;
bool bmpAvailable  = false;
bool rtcAvailable  = false;
bool gpsAvailable  = false;

// ============================================================================
// MPU6050 CALIBRATION STATE
// Accel reference is HARDCODED IDEAL — gravity = (0, 0, +1g)
// Gyro offsets are calibrated per-chip at startup
// ============================================================================
float gyroOffsetX  = 0.0f;
float gyroOffsetY  = 0.0f;
float gyroOffsetZ  = 0.0f;

// HARDCODED IDEAL — never modified by user calibration
const float refAccX  = 0.0f;    // Ideal: no lateral gravity component
const float refAccY  = 0.0f;    // Ideal: no lateral gravity component
const float refAccZ  = 1.0f;    // Ideal: full gravity on Z axis
const float refTiltX = 0.0f;    // Ideal: 0° = perfectly vertical
const float refTiltY = 0.0f;    // Ideal: 0° = perfectly vertical

bool calibrated = false;

// ============================================================================
// SENSOR FUSION STATE
// ============================================================================
float filtTiltX      = 0.0f;
float filtTiltY      = 0.0f;
float correctedTiltX = 0.0f;
float correctedTiltY = 0.0f;
float combinedTheta  = 0.0f;
float lateralAccel   = 0.0f;
unsigned long lastFusionUs = 0;

// ============================================================================
// RAW SENSOR VALUES (these go to Firebase)
// ============================================================================
int16_t rawAccX, rawAccY, rawAccZ;
int16_t rawGyroX, rawGyroY, rawGyroZ;
int16_t rawMpuTemp;

// ============================================================================
// CONVERTED VALUES (internal flood detection only)
// ============================================================================
float accXg, accYg, accZg;
float gyroXdps, gyroYdps, gyroZdps;

// ============================================================================
// BMP280 STATE
// ============================================================================
BMP280CalibData bmpCalib;
float currentPressure    = 0.0f;
float currentTemperature = 0.0f;
float baselinePressure   = 0.0f;
float pressureDeviation  = 0.0f;
float gaugePressurePa    = 0.0f;
float estimatedDepthCm   = 0.0f;
bool  baselineSet        = false;

// ============================================================================
// SOFTWARE RTC STATE (NTP-synced)
// ============================================================================
bool     ntpSynced       = false;
char     dateTimeStr[24] = "0000-00-00 00:00:00";

// ============================================================================
// GPS STATE
// ============================================================================
GPSData gps;
char    gpsBuffer[GPS_BUFFER_SIZE];
int     gpsBufferIdx = 0;

// ============================================================================
// BATTERY STATE
// ============================================================================
float batteryVoltage  = 0.0f;
float batteryPercent  = 0.0f;

// ============================================================================
// FLOOD DETECTION STATE (internal — NOT sent to Firebase)
// ============================================================================
FloodMode     currentMode        = MODE_SLACK;
FloodMode     pendingMode        = MODE_SLACK;
int           modePersistCount   = 0;
float         olpLengthCm        = 200.0f;
float         hMaxCm             = 200.0f;
float         waterHeightCm      = 0.0f;
float         horizontalDistCm   = 0.0f;
float         floodRatio         = 0.0f;
AlertLevel    floodAlertLevel    = ALERT_GREEN;
WaterZone     currentZone        = ZONE_SAFE;
ResponseLevel currentResponse    = RESPONSE_NONE;

// ============================================================================
// CONFIGURABLE THRESHOLDS
// ============================================================================
float threshLateralAccelTaut  = DEFAULT_LATERAL_ACCEL_TAUT;
float threshLateralAccelSlack = DEFAULT_LATERAL_ACCEL_SLACK;
float threshTiltTautDeg       = DEFAULT_TILT_TAUT_DEG;
float threshFloodThetaDeg     = DEFAULT_FLOOD_THETA_DEG;
float threshFloodRatio        = DEFAULT_FLOOD_RATIO;
float threshSubmersionPa      = DEFAULT_SUBMERSION_PA;

// ============================================================================
// DYNAMIC FIREBASE PUSH TIMING
// ============================================================================
unsigned long currentPushIntervalMs = PUSH_RATE_BELOW_50_MS;
unsigned long lastFirebasePushMs    = 0;
unsigned long lastMonitorMs         = 0;

// ============================================================================
// WAVE FILTER STATE
// ============================================================================
float waveBuffer[WAVE_BUFFER_SIZE];
int   waveSampleCount = 0;

// ============================================================================
// REALTIME OVERRIDE (website toggle)
// ============================================================================
bool realtimeOverride = false;

// ============================================================================
// SESSION STATISTICS (internal)
// ============================================================================
unsigned long sessionStartMs       = 0;
float         peakHeightCm         = 0.0f;
float         minHeightCm          = 99999.0f;
float         prevHeightCm         = 0.0f;
unsigned long prevHeightTimeMs     = 0;
float         rateOfRiseCmPer15Min = 0.0f;
bool          sustainedRise        = false;
int           riseConsecutive      = 0;

// ============================================================================
// CSV STREAMING STATE (debugger triggered)
// ============================================================================
bool csvStreamActive = false;

// ============================================================================
// DIAGNOSTIC SCHEDULING
// ============================================================================
unsigned long lastDiagMs    = 0;
bool          diagRequested = false;

// ============================================================================
// WIFI STATE
// ============================================================================
bool     s3WifiConnected         = false;
unsigned long s3LastWifiAttemptMs = 0;
unsigned long s3FirebasePushCount = 0;
unsigned long s3FirebaseFailCount = 0;

// ============================================================================
// WIRELESS CONSOLE STATE
// ============================================================================
#define  FIREBASE_CMD_POLL_MS    3000UL    // poll every 3 seconds
unsigned long lastCmdPollMs      = 0;

// ============================================================================
// IP ADDRESS PUSH STATE
// ============================================================================
#define  IP_PUSH_INTERVAL_MS     3600000UL // 1 hour
unsigned long lastIpPushMs       = 0;
bool          ipPushedOnBoot     = false;

// ============================================================================
// FEATURE FLAGS
// ============================================================================
bool obLightEnabled   = true;
bool algorithmEnabled = false;

// ============================================================================
// SIM FIELDS (from C3)
// ============================================================================
int  simSignalRSSI   = 0;
bool simRegistered   = false;
bool simAvailable    = false;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
void initMPU6050();
void initBMP280();
void initGPS();
void initBatteryADC();
void initSoftwareUART();
void initNTP();

bool mpuWriteRegister(uint8_t reg, uint8_t value);
uint8_t mpuReadRegister(uint8_t reg);
void mpuReadSensorData();
void mpuConvertToPhysical();

bool bmpWriteRegister(uint8_t reg, uint8_t value);
uint8_t bmpReadRegister(uint8_t reg);
void bmpReadCalibration();
void bmpReadRaw(int32_t &rawTemp, int32_t &rawPress);
float bmpCompensateTemperature(int32_t rawTemp);
float bmpCompensatePressure(int32_t rawPress);
void bmpUpdate();

void gpsProcessIncoming();
void gpsParseNMEA(const char* sentence);
void gpsParseGGA(const char* sentence);
void gpsParseRMC(const char* sentence);
float nmeaToDecimalDegrees(const char* nmeaCoord, const char* hemisphere);
int  nmeaSplitFields(const char* sentence, char fields[][20], int maxFields);

void readBattery();
void recalibrate();
void calibrateGyro();
void calibratePressureBaseline();
void runSensorFusion();

void classifyFloodMode();
void updateModePersistence(FloodMode detected);
void computeWaterHeight();
void computeFloodRatio();
void computeAlertLevel();
void computeZone();
void computeResponseLevel();
void computeDynamicPushRate();
void updateSessionStats();

void waveCollectSample(float heightCm);
void waveProcessWindow();
float waveComputeTrimmedMean(float* sorted, int n, int trimCount);
void waveSortBuffer(float* buf, int n);

void pushRawToFirebase();
void buildCSVString(char* buffer, int bufSize);
void streamCSV();

void c3UartSendByte(uint8_t b);
void c3UartSendString(const char* str);

void processC3Commands();
void processDebuggerCommands();
void handleC3Command(const char* cmd);
void handleDebuggerCommand(const char* cmd);

void runDiagnostics();
void sendDiagToC3(DiagResult &diag);
int  computeHealthScore();

void softwareRtcUpdate();

void s3WifiConnect();
void s3WifiCheck();
bool s3FirebasePut(const char* path, const char* jsonPayload);
bool s3FirebaseGet(const char* path, char* outBuf, int outBufLen);
void pollFirebaseCommands();
void handleWirelessCommand(const char* cmd);
void firebaseConsolePrint(const char* msg);

void statusPrint(const char* msg);
void errorPrint(const char* msg);
void warningPrint(const char* msg);


// ============================================================================
//                              SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    unsigned long serialWait = millis();
    while (!Serial && (millis() - serialWait < 3000)) {}
    delay(500);

    statusPrint("VARUNA S3 Firmware v3.2 — Gravity Auto-Zero + Wave-Filtered Trimmed Mean");
    statusPrint("Initializing...");

    Serial2.begin(9600, SERIAL_8N1, C3_CMD_RX_PIN, C3_CMD_TX_PIN);
    statusPrint("Serial2 (C3 cmd) on GPIO 43/44 @ 9600");

    Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    statusPrint("Serial1 (GPS) on GPIO 6/7 @ 9600");

    initSoftwareUART();
    statusPrint("SW UART TX on GPIO 14 @ 9600");

    I2C_MPU.begin(MPU_SDA, MPU_SCL, 400000);
    statusPrint("I2C Bus 0 (MPU6050) on GPIO 8/9 @ 400kHz");

    I2C_SENS.begin(SENS_SDA, SENS_SCL, 100000);
    statusPrint("I2C Bus 1 (BMP280) on GPIO 4/5 @ 100kHz");

    // Load H_max from NVS
    preferences.begin("varuna", false);
    hMaxCm = preferences.getFloat("hMaxCm", 200.0f);
    olpLengthCm = hMaxCm;
    char hmMsg[64];
    snprintf(hmMsg, sizeof(hmMsg), "H_MAX loaded from NVS: %.2f cm", hMaxCm);
    statusPrint(hmMsg);

    // Initialize sensors
    initMPU6050();
    initBMP280();
    initGPS();
    initBatteryADC();

    // WiFi + NTP
    statusPrint("Connecting to WiFi...");
    s3WifiConnect();
    initNTP();

    // Push IP address to Firebase immediately on boot
    if (s3WifiConnected) {
        char ipJson[128];
        snprintf(ipJson, sizeof(ipJson),
                 "{\"ip\":\"%s\",\"rssi\":%d,\"pushedAt\":\"%s\"}",
                 WiFi.localIP().toString().c_str(),
                 WiFi.RSSI(),
                 dateTimeStr);
        s3FirebasePut("devices/" S3_DEVICE_ID "/network", ipJson);
        ipPushedOnBoot = true;
        lastIpPushMs   = millis();
        statusPrint("IP address pushed to Firebase on boot");
    }

    // Calibration — gyro offsets only, accel is hardcoded ideal
    if (mpuAvailable) {
        statusPrint("Starting calibration (gyro only — accel ref is ideal)...");
        recalibrate();
    } else {
        errorPrint("MPU6050 not available — cannot calibrate");
    }

    if (bmpAvailable) {
        calibratePressureBaseline();
    } else {
        warningPrint("BMP280 not available — no pressure baseline");
    }

    // Session init
    sessionStartMs     = millis();
    lastFirebasePushMs = millis();
    lastMonitorMs      = millis();
    lastDiagMs         = millis();
    lastFusionUs       = micros();
    prevHeightTimeMs   = millis();

    int health = computeHealthScore();
    char readyMsg[128];
    snprintf(readyMsg, sizeof(readyMsg),
             "READY — MPU:%d BMP:%d RTC:%d GPS:%d Health:%d%%",
             mpuAvailable, bmpAvailable, rtcAvailable, gpsAvailable, health);
    statusPrint(readyMsg);
    statusPrint("Accel reference: IDEAL (0,0,1g) — zero human error");
    statusPrint("Wave filter: 200 samples/2s, 10% trimmed mean");
    statusPrint("CSV streaming OFF — send START to begin");
    statusPrint("Entering main loop");
}


// ============================================================================
//                             MAIN LOOP
// ============================================================================
void loop() {
    unsigned long nowMs = millis();
    unsigned long nowUs = micros();

    // TASK 1: Sensor fusion at 100Hz
    if ((nowUs - lastFusionUs) >= FUSION_INTERVAL_US) {
        lastFusionUs = nowUs;
        if (mpuAvailable) {
            mpuReadSensorData();
            mpuConvertToPhysical();
            runSensorFusion();
        }
    }

    // TASK 2: GPS parsing
    gpsProcessIncoming();

    // TASK 3: C3 commands
    processC3Commands();

    // TASK 4: Debugger commands
    processDebuggerCommands();

    // TASK 5: WiFi check
    s3WifiCheck();

    // TASK 6: Software RTC
    softwareRtcUpdate();

    // TASK 7: Flood monitoring (every 2s)
    // waterHeightCm is already set by waveProcessWindow() every 2 seconds
    // No call to computeWaterHeight() — wave filter handles it
    if ((nowMs - lastMonitorMs) >= MONITOR_INTERVAL_MS) {
        lastMonitorMs = nowMs;

        if (bmpAvailable) bmpUpdate();
        readBattery();

        computeFloodRatio();
        classifyFloodMode();
        computeAlertLevel();
        computeZone();
        computeResponseLevel();
        computeDynamicPushRate();
        updateSessionStats();
    }

    // TASK 8: Firebase push at dynamic rate
    if ((nowMs - lastFirebasePushMs) >= currentPushIntervalMs) {
        lastFirebasePushMs = nowMs;
        pushRawToFirebase();
    }

    // TASK 9: CSV streaming (only if active)
    if (csvStreamActive) {
        streamCSV();
    }

    // TASK 10: Diagnostics (24h)
    if ((nowMs - lastDiagMs) >= DIAG_INTERVAL_MS || diagRequested) {
        lastDiagMs    = nowMs;
        diagRequested = false;
        runDiagnostics();
    }

    // TASK 11: Wireless console — poll Firebase for commands (every 3s)
    if ((nowMs - lastCmdPollMs) >= FIREBASE_CMD_POLL_MS) {
        lastCmdPollMs = nowMs;
        pollFirebaseCommands();
    }

    // TASK 12: Hourly IP address push to Firebase
    if (s3WifiConnected && ipPushedOnBoot &&
        (nowMs - lastIpPushMs) >= IP_PUSH_INTERVAL_MS) {
        lastIpPushMs = nowMs;
        softwareRtcUpdate();
        char ipJson[128];
        snprintf(ipJson, sizeof(ipJson),
                 "{\"ip\":\"%s\",\"rssi\":%d,\"pushedAt\":\"%s\"}",
                 WiFi.localIP().toString().c_str(),
                 WiFi.RSSI(),
                 dateTimeStr);
        s3FirebasePut("devices/" S3_DEVICE_ID "/network", ipJson);
        statusPrint("Hourly IP push to Firebase OK");
    }
}


// ============================================================================
//                       NTP INITIALIZATION
// ============================================================================
void initNTP() {
    statusPrint("NTP: Syncing time...");

    if (!s3WifiConnected) {
        warningPrint("NTP: No WiFi — cannot sync");
        rtcAvailable = false;
        return;
    }

    configTime(NTP_GMT_OFFSET_SEC, NTP_DAYLIGHT_OFFSET_SEC, NTP_SERVER_1, NTP_SERVER_2);

    struct tm timeinfo;
    int attempts = 0;
    while (!getLocalTime(&timeinfo) && attempts < 20) {
        delay(500);
        attempts++;
        Serial.print(".");
    }
    Serial.println();

    if (getLocalTime(&timeinfo)) {
        ntpSynced = true;
        rtcAvailable = true;
        char msg[64];
        strftime(msg, sizeof(msg), "NTP: Synced — %Y-%m-%d %H:%M:%S", &timeinfo);
        statusPrint(msg);
    } else {
        ntpSynced = false;
        rtcAvailable = false;
        errorPrint("NTP: Sync failed");
    }
}


// ============================================================================
//                       SOFTWARE RTC UPDATE
// ============================================================================
void softwareRtcUpdate() {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
        strftime(dateTimeStr, sizeof(dateTimeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
        rtcAvailable = true;
    }
}


// ============================================================================
//                       SENSOR INITIALIZATION
// ============================================================================

void initMPU6050() {
    statusPrint("Initializing MPU6050...");

    uint8_t whoAmI = mpuReadRegister(MPU_REG_WHO_AM_I);
    if (whoAmI != MPU_WHO_AM_I_VAL) {
        char msg[64];
        snprintf(msg, sizeof(msg), "MPU6050 WHO_AM_I failed: 0x%02X != 0x%02X", whoAmI, MPU_WHO_AM_I_VAL);
        errorPrint(msg);
        mpuAvailable = false;
        return;
    }

    mpuWriteRegister(MPU_REG_PWR_MGMT_1, 0x01);
    delay(10);
    mpuWriteRegister(MPU_REG_PWR_MGMT_2, 0x00);
    delay(10);
    mpuWriteRegister(MPU_REG_SMPLRT_DIV, 39);
    mpuWriteRegister(MPU_REG_CONFIG, 0x03);
    mpuWriteRegister(MPU_REG_GYRO_CONFIG, 0x00);
    mpuWriteRegister(MPU_REG_ACCEL_CONFIG, 0x00);
    delay(100);

    mpuAvailable = true;
    statusPrint("MPU6050 OK — ±250°/s, ±2g, 200Hz, DLPF 44Hz");
}

void initBMP280() {
    statusPrint("Initializing BMP280...");

    uint8_t chipId = bmpReadRegister(BMP280_REG_CHIP_ID);
    if (chipId != BMP280_CHIP_ID_VAL) {
        char msg[64];
        snprintf(msg, sizeof(msg), "BMP280 chip ID failed: 0x%02X != 0x%02X", chipId, BMP280_CHIP_ID_VAL);
        errorPrint(msg);
        bmpAvailable = false;
        return;
    }

    bmpWriteRegister(BMP280_REG_RESET, 0xB6);
    delay(50);
    bmpReadCalibration();
    bmpWriteRegister(BMP280_REG_CONFIG_REG, (0x00 << 5) | (0x04 << 2) | 0x00);
    bmpWriteRegister(BMP280_REG_CTRL_MEAS, (0x02 << 5) | (0x05 << 2) | 0x03);
    delay(100);

    bmpAvailable = true;
    statusPrint("BMP280 OK — P×16, T×2, filter×16, normal mode");
}

void initGPS() {
    statusPrint("Initializing GPS...");

    memset(&gps, 0, sizeof(gps));
    gps.fixValid = false;
    gps.timeValid = false;
    gps.dateValid = false;
    gpsBufferIdx = 0;

    unsigned long start = millis();
    bool gotData = false;
    while (millis() - start < 2000) {
        if (Serial1.available()) { gotData = true; break; }
        delay(10);
    }

    gpsAvailable = gotData;
    statusPrint(gotData ? "GPS responding — NMEA detected" : "GPS not responding — will retry");
}

void initBatteryADC() {
    statusPrint("Initializing battery ADC on GPIO 2...");
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    readBattery();
    char msg[64];
    snprintf(msg, sizeof(msg), "Battery: %.2fV (%.1f%%)", batteryVoltage, batteryPercent);
    statusPrint(msg);
}

void initSoftwareUART() {
    pinMode(C3_DATA_PIN, OUTPUT);
    digitalWrite(C3_DATA_PIN, HIGH);
}


// ============================================================================
//                       MPU6050 LOW-LEVEL
// ============================================================================

bool mpuWriteRegister(uint8_t reg, uint8_t value) {
    I2C_MPU.beginTransmission(MPU6050_ADDR);
    I2C_MPU.write(reg);
    I2C_MPU.write(value);
    return (I2C_MPU.endTransmission() == 0);
}

uint8_t mpuReadRegister(uint8_t reg) {
    I2C_MPU.beginTransmission(MPU6050_ADDR);
    I2C_MPU.write(reg);
    I2C_MPU.endTransmission(false);
    I2C_MPU.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
    if (I2C_MPU.available()) return I2C_MPU.read();
    return 0xFF;
}

void mpuReadSensorData() {
    I2C_MPU.beginTransmission(MPU6050_ADDR);
    I2C_MPU.write(MPU_REG_ACCEL_XOUT_H);
    I2C_MPU.endTransmission(false);
    I2C_MPU.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14);

    if (I2C_MPU.available() >= 14) {
        rawAccX    = (I2C_MPU.read() << 8) | I2C_MPU.read();
        rawAccY    = (I2C_MPU.read() << 8) | I2C_MPU.read();
        rawAccZ    = (I2C_MPU.read() << 8) | I2C_MPU.read();
        rawMpuTemp = (I2C_MPU.read() << 8) | I2C_MPU.read();
        rawGyroX   = (I2C_MPU.read() << 8) | I2C_MPU.read();
        rawGyroY   = (I2C_MPU.read() << 8) | I2C_MPU.read();
        rawGyroZ   = (I2C_MPU.read() << 8) | I2C_MPU.read();
    }
}

void mpuConvertToPhysical() {
    accXg = (float)rawAccX / ACCEL_SENSITIVITY;
    accYg = (float)rawAccY / ACCEL_SENSITIVITY;
    accZg = (float)rawAccZ / ACCEL_SENSITIVITY;

    gyroXdps = ((float)rawGyroX / GYRO_SENSITIVITY) - gyroOffsetX;
    gyroYdps = ((float)rawGyroY / GYRO_SENSITIVITY) - gyroOffsetY;
    gyroZdps = ((float)rawGyroZ / GYRO_SENSITIVITY) - gyroOffsetZ;

    lateralAccel = sqrtf(accXg * accXg + accYg * accYg) * GRAVITY_MS2;
}


// ============================================================================
//                       BMP280 LOW-LEVEL
// ============================================================================

bool bmpWriteRegister(uint8_t reg, uint8_t value) {
    I2C_SENS.beginTransmission(BMP280_ADDR);
    I2C_SENS.write(reg);
    I2C_SENS.write(value);
    return (I2C_SENS.endTransmission() == 0);
}

uint8_t bmpReadRegister(uint8_t reg) {
    I2C_SENS.beginTransmission(BMP280_ADDR);
    I2C_SENS.write(reg);
    I2C_SENS.endTransmission(false);
    I2C_SENS.requestFrom((uint8_t)BMP280_ADDR, (uint8_t)1);
    if (I2C_SENS.available()) return I2C_SENS.read();
    return 0xFF;
}

void bmpReadCalibration() {
    uint8_t calData[26];
    I2C_SENS.beginTransmission(BMP280_ADDR);
    I2C_SENS.write(BMP280_REG_CALIB_START);
    I2C_SENS.endTransmission(false);
    I2C_SENS.requestFrom((uint8_t)BMP280_ADDR, (uint8_t)26);
    for (int i = 0; i < 26 && I2C_SENS.available(); i++) calData[i] = I2C_SENS.read();

    bmpCalib.dig_T1 = (uint16_t)(calData[1]  << 8 | calData[0]);
    bmpCalib.dig_T2 = (int16_t)(calData[3]   << 8 | calData[2]);
    bmpCalib.dig_T3 = (int16_t)(calData[5]   << 8 | calData[4]);
    bmpCalib.dig_P1 = (uint16_t)(calData[7]  << 8 | calData[6]);
    bmpCalib.dig_P2 = (int16_t)(calData[9]   << 8 | calData[8]);
    bmpCalib.dig_P3 = (int16_t)(calData[11]  << 8 | calData[10]);
    bmpCalib.dig_P4 = (int16_t)(calData[13]  << 8 | calData[12]);
    bmpCalib.dig_P5 = (int16_t)(calData[15]  << 8 | calData[14]);
    bmpCalib.dig_P6 = (int16_t)(calData[17]  << 8 | calData[16]);
    bmpCalib.dig_P7 = (int16_t)(calData[19]  << 8 | calData[18]);
    bmpCalib.dig_P8 = (int16_t)(calData[21]  << 8 | calData[20]);
    bmpCalib.dig_P9 = (int16_t)(calData[23]  << 8 | calData[22]);
    statusPrint("BMP280 calibration loaded");
}

void bmpReadRaw(int32_t &rawTemp, int32_t &rawPress) {
    uint8_t data[6];
    I2C_SENS.beginTransmission(BMP280_ADDR);
    I2C_SENS.write(BMP280_REG_PRESS_MSB);
    I2C_SENS.endTransmission(false);
    I2C_SENS.requestFrom((uint8_t)BMP280_ADDR, (uint8_t)6);
    for (int i = 0; i < 6 && I2C_SENS.available(); i++) data[i] = I2C_SENS.read();

    rawPress = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
    rawTemp  = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);
}

float bmpCompensateTemperature(int32_t rawTemp) {
    int32_t var1, var2;
    var1 = ((((rawTemp >> 3) - ((int32_t)bmpCalib.dig_T1 << 1))) *
            ((int32_t)bmpCalib.dig_T2)) >> 11;
    var2 = (((((rawTemp >> 4) - ((int32_t)bmpCalib.dig_T1)) *
              ((rawTemp >> 4) - ((int32_t)bmpCalib.dig_T1))) >> 12) *
            ((int32_t)bmpCalib.dig_T3)) >> 14;
    bmpCalib.t_fine = var1 + var2;
    float T = (bmpCalib.t_fine * 5 + 128) >> 8;
    return T / 100.0f;
}

float bmpCompensatePressure(int32_t rawPress) {
    int64_t var1, var2, p;
    var1 = ((int64_t)bmpCalib.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmpCalib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmpCalib.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmpCalib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmpCalib.dig_P3) >> 8) +
           ((var1 * (int64_t)bmpCalib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmpCalib.dig_P1) >> 33;
    if (var1 == 0) return 0;
    p = 1048576 - rawPress;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmpCalib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmpCalib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmpCalib.dig_P7) << 4);
    return (float)p / 25600.0f;
}

void bmpUpdate() {
    int32_t rawTemp, rawPress;
    bmpReadRaw(rawTemp, rawPress);
    currentTemperature = bmpCompensateTemperature(rawTemp);
    currentPressure    = bmpCompensatePressure(rawPress);

    if (baselineSet) {
        pressureDeviation = currentPressure - baselinePressure;
        gaugePressurePa = pressureDeviation * 100.0f;
        if (gaugePressurePa > threshSubmersionPa) {
            estimatedDepthCm = (gaugePressurePa / (WATER_DENSITY_KGM3 * GRAVITY_MS2)) * 100.0f;
        } else {
            estimatedDepthCm = 0.0f;
        }
    }
}


// ============================================================================
//                       GPS PARSING
// ============================================================================

void gpsProcessIncoming() {
    while (Serial1.available()) {
        char c = Serial1.read();
        if (c == '\n' || c == '\r') {
            if (gpsBufferIdx > 0) {
                gpsBuffer[gpsBufferIdx] = '\0';
                gpsParseNMEA(gpsBuffer);
                gpsBufferIdx = 0;
            }
        } else {
            if (gpsBufferIdx < GPS_BUFFER_SIZE - 1) gpsBuffer[gpsBufferIdx++] = c;
            else gpsBufferIdx = 0;
        }
        if (!gpsAvailable) {
            gpsAvailable = true;
            statusPrint("GPS detected — NMEA incoming");
        }
    }
}

void gpsParseNMEA(const char* sentence) {
    if (sentence[0] != '$') return;
    int len = strlen(sentence);
    int starPos = -1;
    for (int i = len - 1; i >= 0; i--) {
        if (sentence[i] == '*') { starPos = i; break; }
    }
    if (starPos > 0 && starPos + 2 < len) {
        uint8_t cs = 0;
        for (int i = 1; i < starPos; i++) cs ^= sentence[i];
        char csStr[3] = {sentence[starPos + 1], sentence[starPos + 2], '\0'};
        if (cs != (uint8_t)strtol(csStr, NULL, 16)) return;
    }
    if (strncmp(sentence + 3, "GGA", 3) == 0) gpsParseGGA(sentence);
    else if (strncmp(sentence + 3, "RMC", 3) == 0) gpsParseRMC(sentence);
}

void gpsParseGGA(const char* sentence) {
    char fields[NMEA_MAX_FIELDS][20];
    int n = nmeaSplitFields(sentence, fields, NMEA_MAX_FIELDS);
    if (n < 12) return;
    if (strlen(fields[1]) >= 6) {
        gps.hour   = (fields[1][0]-'0')*10 + (fields[1][1]-'0');
        gps.minute = (fields[1][2]-'0')*10 + (fields[1][3]-'0');
        gps.second = (fields[1][4]-'0')*10 + (fields[1][5]-'0');
        gps.timeValid = true;
    }
    if (strlen(fields[2]) > 0 && strlen(fields[3]) > 0)
        gps.latitude = nmeaToDecimalDegrees(fields[2], fields[3]);
    if (strlen(fields[4]) > 0 && strlen(fields[5]) > 0)
        gps.longitude = nmeaToDecimalDegrees(fields[4], fields[5]);
    if (strlen(fields[6]) > 0) {
        gps.fixQuality = atoi(fields[6]);
        gps.fixValid = (gps.fixQuality > 0);
    }
    if (strlen(fields[7]) > 0) gps.satellites = atoi(fields[7]);
    if (strlen(fields[9]) > 0) gps.altitude = atof(fields[9]);
    gps.lastUpdateMs = millis();
}

void gpsParseRMC(const char* sentence) {
    char fields[NMEA_MAX_FIELDS][20];
    int n = nmeaSplitFields(sentence, fields, NMEA_MAX_FIELDS);
    if (n < 10) return;
    if (strlen(fields[1]) >= 6) {
        gps.hour   = (fields[1][0]-'0')*10 + (fields[1][1]-'0');
        gps.minute = (fields[1][2]-'0')*10 + (fields[1][3]-'0');
        gps.second = (fields[1][4]-'0')*10 + (fields[1][5]-'0');
        gps.timeValid = true;
    }
    if (strlen(fields[2]) > 0) gps.fixValid = (fields[2][0] == 'A');
    if (strlen(fields[3]) > 0 && strlen(fields[4]) > 0)
        gps.latitude = nmeaToDecimalDegrees(fields[3], fields[4]);
    if (strlen(fields[5]) > 0 && strlen(fields[6]) > 0)
        gps.longitude = nmeaToDecimalDegrees(fields[5], fields[6]);
    if (strlen(fields[9]) >= 6) {
        gps.day   = (fields[9][0]-'0')*10 + (fields[9][1]-'0');
        gps.month = (fields[9][2]-'0')*10 + (fields[9][3]-'0');
        gps.year  = 2000 + (fields[9][4]-'0')*10 + (fields[9][5]-'0');
        gps.dateValid = true;
    }
    gps.lastUpdateMs = millis();
}

float nmeaToDecimalDegrees(const char* nmeaCoord, const char* hemisphere) {
    float raw = atof(nmeaCoord);
    int deg = (int)(raw / 100);
    float min = raw - (deg * 100);
    float dec = deg + (min / 60.0f);
    if (hemisphere[0] == 'S' || hemisphere[0] == 'W') dec = -dec;
    return dec;
}

int nmeaSplitFields(const char* sentence, char fields[][20], int maxFields) {
    int fi = 0, ci = 0;
    for (int i = 0; sentence[i] != '\0' && sentence[i] != '*' && fi < maxFields; i++) {
        if (sentence[i] == ',') { fields[fi][ci] = '\0'; fi++; ci = 0; }
        else { if (ci < 19) fields[fi][ci++] = sentence[i]; }
    }
    if (fi < maxFields) { fields[fi][ci] = '\0'; fi++; }
    return fi;
}


// ============================================================================
//                       BATTERY MONITORING
// ============================================================================
void readBattery() {
    uint32_t adcSum = 0;
    for (int i = 0; i < 16; i++) {
        adcSum += analogRead(BATTERY_ADC_PIN);
        delayMicroseconds(100);
    }
    float adcAvg = (float)adcSum / 16.0f;
    float adcVoltage = (adcAvg / 4095.0f) * 3.3f;
    batteryVoltage = adcVoltage * 2.0f;

    if (batteryVoltage >= 4.2f) batteryPercent = 100.0f;
    else if (batteryVoltage <= 3.0f) batteryPercent = 0.0f;
    else batteryPercent = ((batteryVoltage - 3.0f) / 1.2f) * 100.0f;
}


// ============================================================================
//                       CALIBRATION — GRAVITY AUTO-ZERO
// ============================================================================
// Accelerometer reference is HARDCODED to ideal orientation:
//   gravity = (0, 0, +1g) = buoy perfectly vertical
//   refTiltX = 0°, refTiltY = 0°
// This eliminates ALL human error from calibration.
//
// Gyro offsets ARE sampled because every chip has different bias.
// The gyro must be stationary during calibration (but orientation
// does NOT matter — only stillness).
// ============================================================================

void recalibrate() {
    statusPrint("CALIBRATION: Starting — gyro offset only");
    statusPrint("CALIBRATION: Accel reference is IDEAL (0,0,1g) — no user input needed");

    // Gyro still needs per-chip offset calibration
    calibrateGyro();

    // Accel reference is const — already set at declaration:
    //   refAccX = 0, refAccY = 0, refAccZ = 1
    //   refTiltX = 0°, refTiltY = 0°

    // Initialize complementary filter at vertical (zero tilt)
    filtTiltX = 0.0f;
    filtTiltY = 0.0f;

    calibrated = true;

    char msg[128];
    snprintf(msg, sizeof(msg),
             "CALIBRATION DONE: GyroOff=(%.3f,%.3f,%.3f) RefTilt=(0.00,0.00) IDEAL",
             gyroOffsetX, gyroOffsetY, gyroOffsetZ);
    statusPrint(msg);
}

void calibrateGyro() {
    statusPrint("CALIBRATION: Gyro offsets (1000 samples) — keep device STATIONARY...");
    double sumX = 0, sumY = 0, sumZ = 0;
    int valid = 0;

    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        mpuReadSensorData();
        float gx = (float)rawGyroX / GYRO_SENSITIVITY;
        float gy = (float)rawGyroY / GYRO_SENSITIVITY;
        float gz = (float)rawGyroZ / GYRO_SENSITIVITY;

        // Reject outliers — at rest, should be near zero
        if (fabsf(gx) < 20.0f && fabsf(gy) < 20.0f && fabsf(gz) < 20.0f) {
            sumX += gx; sumY += gy; sumZ += gz; valid++;
        }
        delay(2);
    }

    if (valid > 100) {
        gyroOffsetX = sumX / valid;
        gyroOffsetY = sumY / valid;
        gyroOffsetZ = sumZ / valid;
    } else {
        errorPrint("CALIBRATION: Too few gyro samples — using zero offsets");
        gyroOffsetX = 0.0f;
        gyroOffsetY = 0.0f;
        gyroOffsetZ = 0.0f;
    }

    char msg[128];
    snprintf(msg, sizeof(msg), "CALIBRATION: Gyro offsets = (%.4f, %.4f, %.4f) from %d samples",
             gyroOffsetX, gyroOffsetY, gyroOffsetZ, valid);
    statusPrint(msg);
}

// NOTE: calibrateAccel() is DELETED — no longer exists
// The accelerometer reference is hardcoded as const at declaration

void calibratePressureBaseline() {
    statusPrint("CALIBRATION: Pressure baseline (50 samples)...");
    double sumP = 0;
    int valid = 0;

    for (int i = 0; i < PRESSURE_CAL_SAMPLES; i++) {
        int32_t rawT, rawP;
        bmpReadRaw(rawT, rawP);
        bmpCompensateTemperature(rawT);
        float pres = bmpCompensatePressure(rawP);
        if (pres > 300.0f && pres < 1100.0f) { sumP += pres; valid++; }
        delay(50);
    }

    if (valid > 10) {
        baselinePressure = sumP / valid;
        baselineSet = true;
        char msg[64];
        snprintf(msg, sizeof(msg), "CALIBRATION: Baseline = %.2f hPa", baselinePressure);
        statusPrint(msg);
    } else {
        errorPrint("CALIBRATION: Pressure baseline failed");
        baselineSet = false;
    }
}


// ============================================================================
//                       SENSOR FUSION — COMPLEMENTARY FILTER
// ============================================================================
// With ideal reference (0,0,1g), correctedTilt = filtTilt - 0 = filtTilt
// Any non-zero tilt IS real tilt from true vertical
// ============================================================================

void runSensorFusion() {
    unsigned long nowUs = micros();
    float dt = (float)(nowUs - lastFusionUs) / 1000000.0f;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

    // Accel-derived tilt angles (absolute, from gravity)
    float accelTiltX = atan2f(accYg, sqrtf(accXg*accXg + accZg*accZg)) * 180.0f / M_PI;
    float accelTiltY = atan2f(-accXg, sqrtf(accYg*accYg + accZg*accZg)) * 180.0f / M_PI;

    // Complementary filter: 98% gyro integration + 2% accel correction
    filtTiltX = ALPHA * (filtTiltX + gyroXdps * dt) + (1.0f - ALPHA) * accelTiltX;
    filtTiltY = ALPHA * (filtTiltY + gyroYdps * dt) + (1.0f - ALPHA) * accelTiltY;

    // With ideal reference, corrected = filtered (ref is 0,0)
    correctedTiltX = filtTiltX;  // refTiltX is 0
    correctedTiltY = filtTiltY;  // refTiltY is 0

    // Combined angle from true vertical
    combinedTheta = sqrtf(correctedTiltX*correctedTiltX + correctedTiltY*correctedTiltY);

    // ---- WAVE FILTER: compute instantaneous height, feed buffer ----
    float instantHeight = 0.0f;

    if (currentMode == MODE_SUBMERGED) {
        instantHeight = olpLengthCm + estimatedDepthCm;
    } else if (currentMode == MODE_TAUT || currentMode == MODE_FLOOD) {
        float thetaRad = combinedTheta * M_PI / 180.0f;
        instantHeight = olpLengthCm * cosf(thetaRad);
    }
    // SLACK mode: instantHeight stays 0.0

    if (instantHeight < 0.0f) instantHeight = 0.0f;

    waveCollectSample(instantHeight);
}


// ============================================================================
//                       WAVE FILTER — TRIMMED MEAN
// ============================================================================
// Collects 200 instantaneous water height samples over 2 seconds.
// Sorts them, discards top 10% (wave crests) and bottom 10% (wave troughs),
// averages the remaining 160 samples to produce ONE stable water level value.
//
// This runs continuously. Every 2 seconds, waterHeightCm is updated
// with the new trimmed mean. All downstream flood logic uses this value.
// ============================================================================

void waveCollectSample(float heightCm) {
    if (waveSampleCount < WAVE_BUFFER_SIZE) {
        waveBuffer[waveSampleCount] = heightCm;
        waveSampleCount++;
    }

    // Buffer full — process the 2-second window
    if (waveSampleCount >= WAVE_BUFFER_SIZE) {
        waveProcessWindow();
        waveSampleCount = 0;
    }
}

void waveProcessWindow() {
    // Sort in-place (buffer gets refilled next window anyway)
    waveSortBuffer(waveBuffer, WAVE_BUFFER_SIZE);

    // Trimmed mean: skip bottom 20, skip top 20, average middle 160
    waterHeightCm = waveComputeTrimmedMean(waveBuffer, WAVE_BUFFER_SIZE, WAVE_TRIM_COUNT);

    if (waterHeightCm < 0.0f) waterHeightCm = 0.0f;

    // Derive horizontal distance from the summarised height
    if (waterHeightCm > 0.01f && waterHeightCm <= olpLengthCm) {
        float cosTheta = waterHeightCm / olpLengthCm;
        if (cosTheta > 1.0f) cosTheta = 1.0f;
        float effectiveTheta = acosf(cosTheta);
        horizontalDistCm = olpLengthCm * sinf(effectiveTheta);
    } else {
        horizontalDistCm = 0.0f;
    }
}

float waveComputeTrimmedMean(float* sorted, int n, int trimCount) {
    int startIdx = trimCount;          // Skip lowest 20 samples
    int endIdx   = n - trimCount;      // Skip highest 20 samples
    int count    = endIdx - startIdx;  // Average these 160 samples

    if (count <= 0) return sorted[n / 2];  // Fallback: median

    float sum = 0.0f;
    for (int i = startIdx; i < endIdx; i++) {
        sum += sorted[i];
    }
    return sum / (float)count;
}

// Insertion sort — efficient enough for N=200, called once every 2 seconds
void waveSortBuffer(float* buf, int n) {
    for (int i = 1; i < n; i++) {
        float key = buf[i];
        int j = i - 1;
        while (j >= 0 && buf[j] > key) {
            buf[j + 1] = buf[j];
            j--;
        }
        buf[j + 1] = key;
    }
}


// ============================================================================
//                       FLOOD DETECTION ENGINE
// ============================================================================

void computeWaterHeight() {
    if (currentMode == MODE_SUBMERGED) {
        waterHeightCm = olpLengthCm + estimatedDepthCm;
    } else if (currentMode == MODE_TAUT || currentMode == MODE_FLOOD) {
        float thetaRad = combinedTheta * M_PI / 180.0f;
        waterHeightCm = olpLengthCm * cosf(thetaRad);
        horizontalDistCm = olpLengthCm * sinf(thetaRad);
    } else {
        waterHeightCm = 0.0f;
        horizontalDistCm = 0.0f;
    }
    if (waterHeightCm < 0.0f) waterHeightCm = 0.0f;
}

void computeFloodRatio() {
    if (hMaxCm > 0.0f) floodRatio = waterHeightCm / hMaxCm;
    else floodRatio = 0.0f;
    if (floodRatio > 9.99f) floodRatio = 9.99f;
}

void classifyFloodMode() {
    FloodMode detected = MODE_SLACK;

    if (bmpAvailable && baselineSet && gaugePressurePa > threshSubmersionPa) {
        detected = MODE_SUBMERGED;
    } else if (lateralAccel > threshLateralAccelTaut && combinedTheta > threshTiltTautDeg) {
        if (combinedTheta < threshFloodThetaDeg && floodRatio > threshFloodRatio)
            detected = MODE_FLOOD;
        else
            detected = MODE_TAUT;
    } else if (currentMode == MODE_TAUT || currentMode == MODE_FLOOD) {
        if (lateralAccel > threshLateralAccelSlack || combinedTheta > threshTiltTautDeg) {
            if (combinedTheta < threshFloodThetaDeg && floodRatio > threshFloodRatio)
                detected = MODE_FLOOD;
            else
                detected = MODE_TAUT;
        } else {
            detected = MODE_SLACK;
        }
    }

    updateModePersistence(detected);
}

void updateModePersistence(FloodMode detected) {
    if (detected == pendingMode) modePersistCount++;
    else { pendingMode = detected; modePersistCount = 1; }

    int required = (detected == MODE_SUBMERGED) ? SUBMERGE_PERSISTENCE_COUNT : MODE_PERSISTENCE_COUNT;
    if (modePersistCount >= required) {
        if (currentMode != detected) {
            char msg[64];
            snprintf(msg, sizeof(msg), "MODE: %d -> %d", currentMode, detected);
            statusPrint(msg);
        }
        currentMode = detected;
    }
}

void computeAlertLevel() {
    switch (currentMode) {
        case MODE_SLACK:     floodAlertLevel = ALERT_GREEN; break;
        case MODE_TAUT:      floodAlertLevel = (floodRatio > 0.80f) ? ALERT_YELLOW : ALERT_GREEN; break;
        case MODE_FLOOD:     floodAlertLevel = ALERT_RED; break;
        case MODE_SUBMERGED: floodAlertLevel = ALERT_BLACK; break;
    }
}

void computeZone() {
    if (currentMode == MODE_SUBMERGED || floodRatio > 0.95f) currentZone = ZONE_CRITICAL;
    else if (floodRatio > 0.80f) currentZone = ZONE_WARNING;
    else if (floodRatio > 0.50f) currentZone = ZONE_WATCH;
    else currentZone = ZONE_SAFE;
}

void computeResponseLevel() {
    switch (currentMode) {
        case MODE_SLACK:     currentResponse = RESPONSE_NONE; break;
        case MODE_TAUT:
            if (floodRatio > 0.80f) currentResponse = RESPONSE_PREPARE;
            else if (floodRatio > 0.50f) currentResponse = RESPONSE_MONITOR;
            else currentResponse = RESPONSE_NONE;
            break;
        case MODE_FLOOD:     currentResponse = RESPONSE_ACT; break;
        case MODE_SUBMERGED: currentResponse = RESPONSE_EVACUATE; break;
    }
}


// ============================================================================
//                       DYNAMIC FIREBASE PUSH RATE
// ============================================================================
void computeDynamicPushRate() {
    if (realtimeOverride) {
        currentPushIntervalMs = PUSH_RATE_REALTIME_MS;
        return;
    }

    if (floodRatio >= 0.80f)      currentPushIntervalMs = PUSH_RATE_ABOVE_80_MS;
    else if (floodRatio >= 0.50f) currentPushIntervalMs = PUSH_RATE_50_TO_80_MS;
    else                          currentPushIntervalMs = PUSH_RATE_BELOW_50_MS;
}


// ============================================================================
//                       SESSION STATISTICS
// ============================================================================
void updateSessionStats() {
    unsigned long nowMs = millis();
    if (waterHeightCm > peakHeightCm) peakHeightCm = waterHeightCm;
    if (waterHeightCm < minHeightCm && waterHeightCm > 0.01f) minHeightCm = waterHeightCm;

    unsigned long dtMs = nowMs - prevHeightTimeMs;
    if (dtMs > 0 && prevHeightTimeMs > 0) {
        float dtMin = (float)dtMs / 60000.0f;
        if (dtMin > 0.1f) {
            float risePerMin = (waterHeightCm - prevHeightCm) / dtMin;
            rateOfRiseCmPer15Min = risePerMin * 15.0f;
            if (rateOfRiseCmPer15Min > 1.0f) {
                riseConsecutive++;
                if (riseConsecutive >= 3) sustainedRise = true;
            } else {
                riseConsecutive = 0;
                sustainedRise = false;
            }
        }
    }
    prevHeightCm = waterHeightCm;
    prevHeightTimeMs = nowMs;
}


// ============================================================================
//                       HEALTH SCORE
// ============================================================================
int computeHealthScore() {
    int score = 0;
    if (mpuAvailable) score += 40;
    if (bmpAvailable) score += 30;
    if (rtcAvailable) score += 20;
    if (gpsAvailable && gps.fixValid) score += 10;
    return score;
}


// ============================================================================
//                       FIREBASE — RAW DATA PUSH (LATEST ONLY)
// ============================================================================
void pushRawToFirebase() {
    if (!s3WifiConnected) return;

    softwareRtcUpdate();

    char json[512];
    snprintf(json, sizeof(json),
        "{"
        "\"waterLevel\":%.2f,"
        "\"rawAccX\":%d,"
        "\"rawAccY\":%d,"
        "\"rawAccZ\":%d,"
        "\"rawGyroX\":%d,"
        "\"rawGyroY\":%d,"
        "\"rawGyroZ\":%d,"
        "\"rawMpuTemp\":%d,"
        "\"pressure\":%.2f,"
        "\"temperature\":%.2f,"
        "\"latitude\":%.6f,"
        "\"longitude\":%.6f,"
        "\"batteryPercent\":%.1f,"
        "\"mpuAvailable\":%s,"
        "\"bmpAvailable\":%s,"
        "\"rtcAvailable\":%s,"
        "\"gpsAvailable\":%s,"
        "\"dateTime\":\"%s\","
        "\"uptimeMs\":%lu,"
        "\"wifiRSSI\":%d"
        "}",
        waterHeightCm,
        rawAccX, rawAccY, rawAccZ,
        rawGyroX, rawGyroY, rawGyroZ,
        rawMpuTemp,
        currentPressure, currentTemperature,
        gps.latitude, gps.longitude,
        batteryPercent,
        mpuAvailable ? "true" : "false",
        bmpAvailable ? "true" : "false",
        rtcAvailable ? "true" : "false",
        gpsAvailable ? "true" : "false",
        dateTimeStr,
        millis(),
        WiFi.RSSI()
    );

    char path[128];
    snprintf(path, sizeof(path), "devices/%s/raw/latest", S3_DEVICE_ID);

    if (s3FirebasePut(path, json)) s3FirebasePushCount++;
    else s3FirebaseFailCount++;
}


// ============================================================================
//                       CSV STREAMING (debugger triggered)
// ============================================================================
void streamCSV() {
    char csvBuffer[1024];
    buildCSVString(csvBuffer, sizeof(csvBuffer));
    Serial.println(csvBuffer);

    c3UartSendString(csvBuffer);
    c3UartSendByte('\r');
    c3UartSendByte('\n');

    Serial2.print("$CSV,");
    Serial2.println(csvBuffer);
}

void buildCSVString(char* buffer, int bufSize) {
    unsigned long sessionSec = (millis() - sessionStartMs) / 1000UL;
    float reportedMinHeight = (minHeightCm > 99990.0f) ? 0.0f : minHeightCm;

    snprintf(buffer, bufSize,
        "%.2f,%.2f,%.2f,%.2f,"
        "%.2f,%.2f,%.2f,%.2f,"
        "%.2f,%.2f,"
        "%d,%.2f,%d,%lu,"
        "%s,%d,%.3f,%d,"
        "%lu,%.2f,%.2f,"
        "%.6f,%.6f,%.1f,%d,%d,"
        "%d,%d,%d,"
        "%d,%d,%d,"
        "%.1f,%d,%d,"
        "%d,%d,%d,%d",
        combinedTheta, waterHeightCm,
        correctedTiltX, correctedTiltY,
        olpLengthCm, horizontalDistCm,
        currentPressure, currentTemperature,
        baselinePressure, pressureDeviation,
        (int)currentMode, estimatedDepthCm,
        (int)bmpAvailable, 0UL,
        dateTimeStr, (int)rtcAvailable,
        rateOfRiseCmPer15Min, (int)floodAlertLevel,
        sessionSec, peakHeightCm, reportedMinHeight,
        gps.latitude, gps.longitude, gps.altitude,
        gps.satellites, (int)gps.fixValid,
        simSignalRSSI, (int)simRegistered, (int)simAvailable,
        (int)currentZone, (int)currentResponse, (int)sustainedRise,
        batteryPercent, (int)(currentPushIntervalMs/1000),
        (int)(currentPushIntervalMs/1000),
        (int)obLightEnabled, (int)algorithmEnabled,
        (int)currentMode, computeHealthScore()
    );
}


// ============================================================================
//                       SOFTWARE UART TX (GPIO 14)
// ============================================================================
void c3UartSendByte(uint8_t b) {
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);
    digitalWrite(C3_DATA_PIN, LOW);
    delayMicroseconds(SW_UART_BIT_TIME_US);
    for (int i = 0; i < 8; i++) {
        digitalWrite(C3_DATA_PIN, (b & (1 << i)) ? HIGH : LOW);
        delayMicroseconds(SW_UART_BIT_TIME_US);
    }
    digitalWrite(C3_DATA_PIN, HIGH);
    delayMicroseconds(SW_UART_BIT_TIME_US);
    portEXIT_CRITICAL(&mux);
    delayMicroseconds(SW_UART_BIT_TIME_US / 2);
}

void c3UartSendString(const char* str) {
    while (*str) { c3UartSendByte((uint8_t)*str); str++; }
}


// ============================================================================
//                       COMMAND PROCESSING — C3
// ============================================================================
static char c3CmdBuffer[256];
static int  c3CmdBufIdx = 0;

void processC3Commands() {
    while (Serial2.available()) {
        char c = Serial2.read();
        if (c == '\n' || c == '\r') {
            if (c3CmdBufIdx > 0) {
                c3CmdBuffer[c3CmdBufIdx] = '\0';
                handleC3Command(c3CmdBuffer);
                c3CmdBufIdx = 0;
            }
        } else {
            if (c3CmdBufIdx < (int)sizeof(c3CmdBuffer) - 1) c3CmdBuffer[c3CmdBufIdx++] = c;
            else c3CmdBufIdx = 0;
        }
    }
}

void handleC3Command(const char* cmd) {
    char msg[256];

    if (strncmp(cmd, "$CFG,", 5) == 0) {
        float newHMax = atof(cmd + 5);
        if (newHMax > 0) {
            hMaxCm = newHMax;
            olpLengthCm = newHMax;
            preferences.putFloat("hMaxCm", hMaxCm);
            snprintf(msg, sizeof(msg), "STATUS:H_MAX = %.2f cm (saved)", hMaxCm);
            statusPrint(msg);
            computeDynamicPushRate();
            Serial2.println("$CFG_ACK");
        }
    }
    else if (strncmp(cmd, "$SETHMAX,", 9) == 0) {
        float h = atof(cmd + 9);
        if (h > 1.0f && h < 10000.0f) {
            hMaxCm = h;
            olpLengthCm = h;
            preferences.putFloat("hMaxCm", hMaxCm);
            snprintf(msg, sizeof(msg), "STATUS:H_MAX = %.2f cm (website)", hMaxCm);
            statusPrint(msg);
            computeDynamicPushRate();
            Serial2.println("$HMAX_ACK");
        }
    }
    else if (strcmp(cmd, "$DIAGRUN") == 0) {
        diagRequested = true;
        Serial2.println("$DIAG_ACK");
    }
    else if (strcmp(cmd, "$PING") == 0) {
        Serial2.println("$PONG");
    }
    else if (strncmp(cmd, "$SIMSTAT,", 9) == 0) {
        int rssi = 0, reg = 0, avail = 0;
        if (sscanf(cmd + 9, "%d,%d,%d", &rssi, &reg, &avail) == 3) {
            simSignalRSSI = rssi;
            simRegistered = (reg != 0);
            simAvailable  = (avail != 0);
        }
    }
    else if (strncmp(cmd, "$REALTIME,", 10) == 0) {
        int val = atoi(cmd + 10);
        realtimeOverride = (val != 0);
        computeDynamicPushRate();
        snprintf(msg, sizeof(msg), "STATUS:REALTIME %s (push every %lums)",
                 realtimeOverride ? "ON" : "OFF", currentPushIntervalMs);
        statusPrint(msg);
        Serial2.println(realtimeOverride ? "$REALTIME_ACK,1" : "$REALTIME_ACK,0");
    }
    else {
        snprintf(msg, sizeof(msg), "WARNING:Unknown C3 cmd: %s", cmd);
        warningPrint(msg);
    }
}


// ============================================================================
//                       COMMAND PROCESSING — DEBUGGER
// ============================================================================
static char dbgCmdBuffer[256];
static int  dbgCmdBufIdx = 0;

void processDebuggerCommands() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (dbgCmdBufIdx > 0) {
                dbgCmdBuffer[dbgCmdBufIdx] = '\0';
                handleDebuggerCommand(dbgCmdBuffer);
                dbgCmdBufIdx = 0;
            }
        } else {
            if (dbgCmdBufIdx < (int)sizeof(dbgCmdBuffer) - 1) dbgCmdBuffer[dbgCmdBufIdx++] = c;
            else dbgCmdBufIdx = 0;
        }
    }
}

void handleDebuggerCommand(const char* cmd) {
    char msg[512];

    if (strcmp(cmd, "START") == 0) {
        csvStreamActive = true;
        statusPrint("CSV streaming STARTED");
    }
    else if (strcmp(cmd, "STOP") == 0) {
        csvStreamActive = false;
        statusPrint("CSV streaming STOPPED");
    }
    else if (strcmp(cmd, "PING") == 0) {
        snprintf(msg, sizeof(msg),
                 "STATUS:PONG uptime=%lus heap=%u ip=%s rssi=%d",
                 millis() / 1000UL,
                 (unsigned)ESP.getFreeHeap(),
                 s3WifiConnected ? WiFi.localIP().toString().c_str() : "0.0.0.0",
                 s3WifiConnected ? WiFi.RSSI() : 0);
        statusPrint(msg);
    }
    else if (strcmp(cmd, "GETCONFIG") == 0) {
        snprintf(msg, sizeof(msg),
                 "STATUS:CONFIG hMax=%.2f olpLen=%.2f mode=%d pushInt=%lums "
                 "ratio=%.3f health=%d "
                 "MPU=%d BMP=%d RTC=%d GPS=%d cal=%d baseline=%d "
                 "csv=%d ntp=%d realtime=%d "
                 "fbOK=%lu fbFail=%lu",
                 hMaxCm, olpLengthCm, currentMode, currentPushIntervalMs,
                 floodRatio, computeHealthScore(),
                 mpuAvailable, bmpAvailable, rtcAvailable, gpsAvailable,
                 calibrated, baselineSet,
                 csvStreamActive, ntpSynced, realtimeOverride,
                 s3FirebasePushCount, s3FirebaseFailCount);
        statusPrint(msg);
    }
    else if (strcmp(cmd, "RECALIBRATE") == 0) {
        statusPrint("Recalibrating gyro — keep device STATIONARY (orientation irrelevant)");
        if (mpuAvailable) {
            recalibrate();
        } else {
            errorPrint("MPU6050 not available");
        }
        if (bmpAvailable) calibratePressureBaseline();
    }
    else if (strcmp(cmd, "GETTHRESH") == 0) {
        snprintf(msg, sizeof(msg),
                 "STATUS:THRESH accelTaut=%.3f accelSlack=%.3f tiltTaut=%.1f "
                 "floodTheta=%.1f floodRatio=%.2f submersionPa=%.1f",
                 threshLateralAccelTaut, threshLateralAccelSlack,
                 threshTiltTautDeg, threshFloodThetaDeg,
                 threshFloodRatio, threshSubmersionPa);
        statusPrint(msg);
    }
    else if (strncmp(cmd, "SETTHRESH=", 10) == 0) {
        float a = 0, w = 0, d = 0;
        if (sscanf(cmd + 10, "%f,%f,%f", &a, &w, &d) == 3) {
            if (a > 0.01f && a < 5.0f) {
                threshLateralAccelTaut = a;
                threshLateralAccelSlack = a * 0.67f;
            }
            if (w > 0.1f && w < 90.0f) threshFloodThetaDeg = w;
            if (d > 10.0f && d < 100000.0f) threshSubmersionPa = d;
            snprintf(msg, sizeof(msg), "STATUS:THRESH updated A=%.3f T=%.1f S=%.1f",
                     threshLateralAccelTaut, threshFloodThetaDeg, threshSubmersionPa);
            statusPrint(msg);
        } else {
            errorPrint("Format: SETTHRESH=accelTaut,floodTheta,submersionPa");
        }
    }
    else if (strcmp(cmd, "RESETTHRESH") == 0) {
        threshLateralAccelTaut  = DEFAULT_LATERAL_ACCEL_TAUT;
        threshLateralAccelSlack = DEFAULT_LATERAL_ACCEL_SLACK;
        threshTiltTautDeg       = DEFAULT_TILT_TAUT_DEG;
        threshFloodThetaDeg     = DEFAULT_FLOOD_THETA_DEG;
        threshFloodRatio        = DEFAULT_FLOOD_RATIO;
        threshSubmersionPa      = DEFAULT_SUBMERSION_PA;
        statusPrint("STATUS:THRESH reset to defaults");
    }
    else if (strncmp(cmd, "SETHMAX=", 8) == 0) {
        float newH = atof(cmd + 8);
        if (newH > 1.0f && newH < 10000.0f) {
            hMaxCm = newH;
            olpLengthCm = newH;
            preferences.putFloat("hMaxCm", hMaxCm);
            snprintf(msg, sizeof(msg), "STATUS:H_MAX = %.2f cm (saved to NVS)", hMaxCm);
            statusPrint(msg);
            computeDynamicPushRate();
        } else {
            errorPrint("SETHMAX: value must be 1-10000 cm");
        }
    }
    else if (strcmp(cmd, "GETSTATUS") == 0) {
        snprintf(msg, sizeof(msg),
                 "STATUS:MODE=%d ALERT=%d ZONE=%d RESP=%d "
                 "H=%.2f THETA=%.2f RATIO=%.3f "
                 "P=%.2f DEPTH=%.2f LAT_ACCEL=%.3f "
                 "BATT=%.1f%% PUSH=%lums "
                 "RISE=%.3f SUST=%d "
                 "WiFi=%d RSSI=%d NTP=%d REALTIME=%d "
                 "FB_OK=%lu FB_FAIL=%lu",
                 currentMode, floodAlertLevel, currentZone, currentResponse,
                 waterHeightCm, combinedTheta, floodRatio,
                 currentPressure, estimatedDepthCm, lateralAccel,
                 batteryPercent, currentPushIntervalMs,
                 rateOfRiseCmPer15Min, sustainedRise,
                 s3WifiConnected, WiFi.RSSI(), ntpSynced, realtimeOverride,
                 s3FirebasePushCount, s3FirebaseFailCount);
        statusPrint(msg);
    }
    else if (strcmp(cmd, "GETRAW") == 0) {
        snprintf(msg, sizeof(msg),
                 "RAW:AX=%d AY=%d AZ=%d GX=%d GY=%d GZ=%d MT=%d "
                 "P=%.2f T=%.2f LAT=%.6f LON=%.6f",
                 rawAccX, rawAccY, rawAccZ,
                 rawGyroX, rawGyroY, rawGyroZ, rawMpuTemp,
                 currentPressure, currentTemperature,
                 gps.latitude, gps.longitude);
        statusPrint(msg);
    }
    else if (strcmp(cmd, "NTPRESYNC") == 0) {
        statusPrint("Forcing NTP resync...");
        initNTP();
    }
    else if (strcmp(cmd, "DIAGRUN") == 0) {
        diagRequested = true;
        statusPrint("Diagnostic requested");
    }
    else if (strcmp(cmd, "REALTIME ON") == 0) {
        realtimeOverride = true;
        computeDynamicPushRate();
        statusPrint("REALTIME ON — pushing every 2 seconds regardless of flood level");
    }
    else if (strcmp(cmd, "REALTIME OFF") == 0) {
        realtimeOverride = false;
        computeDynamicPushRate();
        snprintf(msg, sizeof(msg), "REALTIME OFF — dynamic rate restored (currently %lums)",
                 currentPushIntervalMs);
        statusPrint(msg);
    }
    else if (strcmp(cmd, "ALGO_ON") == 0) {
        algorithmEnabled = true;
        statusPrint("STATUS:ALGO_ON — advanced flood detection enabled");
    }
    else if (strcmp(cmd, "ALGO_OFF") == 0) {
        algorithmEnabled = false;
        statusPrint("STATUS:ALGO_OFF — advanced flood detection disabled");
    }
    else if (strcmp(cmd, "FORCEUPLOAD") == 0) {
        statusPrint("STATUS:FORCEUPLOAD — forcing immediate Firebase push");
        pushRawToFirebase();
        lastFirebasePushMs = millis();
        statusPrint("STATUS:FORCEUPLOAD done");
    }
    else if (strncmp(cmd, "SETRATE=", 8) == 0) {
        // SETRATE=30,10  — normal_sec, high_sec (stored but applied to push interval logic)
        int normalSec = 0, highSec = 0;
        if (sscanf(cmd + 8, "%d,%d", &normalSec, &highSec) == 2) {
            if (normalSec > 0 && normalSec <= 3600 && highSec > 0 && highSec <= 3600) {
                // Persist to NVS for C3 / config panel consumption
                preferences.putInt("rateNormal", normalSec);
                preferences.putInt("rateHigh",   highSec);
                snprintf(msg, sizeof(msg),
                         "STATUS:SETRATE normal=%ds high=%ds (saved to NVS)",
                         normalSec, highSec);
                statusPrint(msg);
            } else {
                errorPrint("SETRATE: values must be 1-3600 seconds");
            }
        } else {
            errorPrint("Format: SETRATE=normal_sec,high_sec  e.g. SETRATE=30,10");
        }
    }
    else if (strcmp(cmd, "SETAPN") == 0 || strcmp(cmd, "REINITSIM") == 0 ||
             strcmp(cmd, "TESTGPRS") == 0) {
        Serial2.print("$");
        Serial2.println(cmd);
        snprintf(msg, sizeof(msg), "STATUS:Forwarded '%s' to C3", cmd);
        statusPrint(msg);
    }
    else {
        snprintf(msg, sizeof(msg), "ERROR:Unknown command: %s", cmd);
        Serial.println(msg);
    }
}


// ============================================================================
//                       DIAGNOSTICS
// ============================================================================
void runDiagnostics() {
    statusPrint("DIAGNOSTIC: Starting...");

    DiagResult diag;
    memset(&diag, 0, sizeof(diag));

    // MPU WHO_AM_I
    diag.mpuWhoAmIOk = (mpuReadRegister(MPU_REG_WHO_AM_I) == MPU_WHO_AM_I_VAL);
    if (!diag.mpuWhoAmIOk) diag.totalFaults++;

    // Accel magnitude
    if (mpuAvailable) {
        mpuReadSensorData();
        mpuConvertToPhysical();
        diag.mpuAccelMagnitude = sqrtf(accXg*accXg + accYg*accYg + accZg*accZg);
        diag.mpuAccelMagnitudeOk = (diag.mpuAccelMagnitude > 0.8f && diag.mpuAccelMagnitude < 1.2f);
    }
    if (!diag.mpuAccelMagnitudeOk) diag.totalFaults++;

    // Gyro drift
    if (mpuAvailable) {
        double sumD = 0;
        for (int i = 0; i < 100; i++) {
            mpuReadSensorData(); mpuConvertToPhysical();
            sumD += sqrtf(gyroXdps*gyroXdps + gyroYdps*gyroYdps + gyroZdps*gyroZdps);
            delay(2);
        }
        diag.mpuGyroDrift = (float)(sumD / 100.0);
        diag.mpuGyroDriftOk = (diag.mpuGyroDrift < 5.0f);
    }
    if (!diag.mpuGyroDriftOk) diag.totalFaults++;

    // BMP chip ID
    diag.bmpChipIdOk = (bmpReadRegister(BMP280_REG_CHIP_ID) == BMP280_CHIP_ID_VAL);
    if (!diag.bmpChipIdOk) diag.totalFaults++;

    // BMP pressure
    if (bmpAvailable) {
        bmpUpdate();
        diag.bmpPressure = currentPressure;
        diag.bmpPressureRangeOk = (currentPressure > 300.0f && currentPressure < 1200.0f);
    }
    if (!diag.bmpPressureRangeOk) diag.totalFaults++;

    // BMP temperature
    if (bmpAvailable) {
        diag.bmpTemperature = currentTemperature;
        diag.bmpTempRangeOk = (currentTemperature > -40.0f && currentTemperature < 85.0f);
    }
    if (!diag.bmpTempRangeOk) diag.totalFaults++;

    // NTP
    diag.ntpSynced = ntpSynced;
    if (!diag.ntpSynced) diag.totalFaults++;

    // GPS
    unsigned long gpsAge = millis() - gps.lastUpdateMs;
    diag.gpsDataRecent = (gps.lastUpdateMs > 0 && gpsAge < 10000);
    diag.gpsSatCount = gps.satellites;
    if (!diag.gpsDataRecent) diag.totalFaults++;

    // Battery
    readBattery();
    diag.batteryVoltage = batteryVoltage;
    diag.batteryVoltageOk = (batteryVoltage > 2.8f && batteryVoltage < 4.3f);
    if (!diag.batteryVoltageOk) diag.totalFaults++;

    // C3 PONG
    while (Serial2.available()) Serial2.read();
    Serial2.println("$PING");
    unsigned long ps = millis();
    diag.c3PongReceived = false;
    char pb[32]; int pi = 0;
    while (millis() - ps < PONG_TIMEOUT_MS) {
        if (Serial2.available()) {
            char c = Serial2.read();
            if (c == '\n' || c == '\r') {
                pb[pi] = '\0';
                if (strcmp(pb, "$PONG") == 0) { diag.c3PongReceived = true; break; }
                pi = 0;
            } else if (pi < 30) pb[pi++] = c;
        }
    }
    if (!diag.c3PongReceived) diag.totalFaults++;

    diag.healthScore = computeHealthScore();

    // Print
    char m[128];
    statusPrint("=== DIAGNOSTIC RESULTS ===");
    snprintf(m, sizeof(m), "  MPU WHO_AM_I: %s", diag.mpuWhoAmIOk ? "PASS" : "FAIL"); statusPrint(m);
    snprintf(m, sizeof(m), "  Accel Mag: %.3fg %s", diag.mpuAccelMagnitude, diag.mpuAccelMagnitudeOk ? "PASS" : "FAIL"); statusPrint(m);
    snprintf(m, sizeof(m), "  Gyro Drift: %.3f %s", diag.mpuGyroDrift, diag.mpuGyroDriftOk ? "PASS" : "FAIL"); statusPrint(m);
    snprintf(m, sizeof(m), "  BMP ID: %s", diag.bmpChipIdOk ? "PASS" : "FAIL"); statusPrint(m);
    snprintf(m, sizeof(m), "  BMP P: %.2f %s", diag.bmpPressure, diag.bmpPressureRangeOk ? "PASS" : "FAIL"); statusPrint(m);
    snprintf(m, sizeof(m), "  BMP T: %.2f %s", diag.bmpTemperature, diag.bmpTempRangeOk ? "PASS" : "FAIL"); statusPrint(m);
    snprintf(m, sizeof(m), "  NTP: %s", diag.ntpSynced ? "PASS" : "FAIL"); statusPrint(m);
    snprintf(m, sizeof(m), "  GPS: %s (sats=%d)", diag.gpsDataRecent ? "PASS" : "FAIL", diag.gpsSatCount); statusPrint(m);
    snprintf(m, sizeof(m), "  Battery: %.2fV %s", diag.batteryVoltage, diag.batteryVoltageOk ? "PASS" : "FAIL"); statusPrint(m);
    snprintf(m, sizeof(m), "  C3 PONG: %s", diag.c3PongReceived ? "PASS" : "FAIL"); statusPrint(m);
    snprintf(m, sizeof(m), "  Faults: %d  Health: %d%%", diag.totalFaults, diag.healthScore); statusPrint(m);
    statusPrint("=== END DIAGNOSTIC ===");

    sendDiagToC3(diag);
}

void sendDiagToC3(DiagResult &diag) {
    char frame[512];
    snprintf(frame, sizeof(frame),
             "$DIAG,MPU_ID=%d,ACCEL=%.3f:%d,GYRO=%.3f:%d,"
             "BMP_ID=%d,P=%.2f:%d,T=%.2f:%d,"
             "NTP=%d,GPS=%d,SATS=%d,"
             "BATT=%.2f:%d,C3=%d,"
             "FAULTS=%d,HEALTH=%d",
             diag.mpuWhoAmIOk,
             diag.mpuAccelMagnitude, diag.mpuAccelMagnitudeOk,
             diag.mpuGyroDrift, diag.mpuGyroDriftOk,
             diag.bmpChipIdOk,
             diag.bmpPressure, diag.bmpPressureRangeOk,
             diag.bmpTemperature, diag.bmpTempRangeOk,
             diag.ntpSynced, diag.gpsDataRecent, diag.gpsSatCount,
             diag.batteryVoltage, diag.batteryVoltageOk,
             diag.c3PongReceived,
             diag.totalFaults, diag.healthScore);

    Serial2.println(frame);
    c3UartSendString(frame);
    c3UartSendByte('\r');
    c3UartSendByte('\n');
}


// ============================================================================
//                       WIFI FUNCTIONS
// ============================================================================
void s3WifiConnect() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(S3_WIFI_SSID, S3_WIFI_PASSWORD);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start < 15000)) {
        delay(500); Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        s3WifiConnected = true;
        char msg[128];
        snprintf(msg, sizeof(msg), "WiFi connected — IP: %s RSSI: %d",
                 WiFi.localIP().toString().c_str(), WiFi.RSSI());
        statusPrint(msg);
    } else {
        s3WifiConnected = false;
        warningPrint("WiFi failed — will retry");
    }
    s3LastWifiAttemptMs = millis();
}

void s3WifiCheck() {
    if (WiFi.status() == WL_CONNECTED) {
        if (!s3WifiConnected) { s3WifiConnected = true; statusPrint("WiFi reconnected"); }
    } else {
        if (s3WifiConnected) { s3WifiConnected = false; warningPrint("WiFi lost"); }
        if ((millis() - s3LastWifiAttemptMs) >= S3_WIFI_RECONNECT_MS) {
            s3LastWifiAttemptMs = millis();
            WiFi.disconnect(); delay(100);
            WiFi.begin(S3_WIFI_SSID, S3_WIFI_PASSWORD);
            unsigned long start = millis();
            while (WiFi.status() != WL_CONNECTED && (millis() - start < 10000)) delay(250);
            if (WiFi.status() == WL_CONNECTED) {
                s3WifiConnected = true;
                statusPrint("WiFi reconnected OK");
                initNTP();
            }
        }
    }
}

bool s3FirebasePut(const char* path, const char* jsonPayload) {
    if (!s3WifiConnected) return false;

    WiFiClientSecure secureClient;
    secureClient.setInsecure();

    HTTPClient http;
    String url = String("https://") + S3_FIREBASE_HOST + "/" + path +
                 ".json?auth=" + S3_FIREBASE_AUTH;

    http.begin(secureClient, url);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(S3_FIREBASE_TIMEOUT_MS);

    int httpCode = http.PUT(jsonPayload);
    bool success = (httpCode >= 200 && httpCode < 300);

    if (!success) {
        char msg[128];
        snprintf(msg, sizeof(msg), "Firebase PUT failed HTTP %d", httpCode);
        errorPrint(msg);
    }

    http.end();
    return success;
}


// ============================================================================
//              FIREBASE GET (for wireless console command polling)
// ============================================================================
bool s3FirebaseGet(const char* path, char* outBuf, int outBufLen) {
    if (!s3WifiConnected) return false;

    WiFiClientSecure secureClient;
    secureClient.setInsecure();

    HTTPClient http;
    String url = String("https://") + S3_FIREBASE_HOST + "/" + path +
                 ".json?auth=" + S3_FIREBASE_AUTH;

    http.begin(secureClient, url);
    http.setTimeout(S3_FIREBASE_TIMEOUT_MS);

    int httpCode = http.GET();
    bool success = false;

    if (httpCode == 200) {
        String payload = http.getString();
        // Firebase returns "null" when the node is empty
        if (payload != "null" && payload.length() > 0 && payload.length() < (unsigned int)outBufLen) {
            strncpy(outBuf, payload.c_str(), outBufLen - 1);
            outBuf[outBufLen - 1] = '\0';
            success = true;
        }
    }

    http.end();
    return success;
}


// ============================================================================
//              FIREBASE CONSOLE — write response back to Firebase
// ============================================================================
void firebaseConsolePrint(const char* msg) {
    // Echo to USB serial as well (for local debug)
    Serial.println(msg);

    // Write response to Firebase so the dashboard can read it
    char json[768];
    // Escape quotes in msg for valid JSON
    char escaped[600];
    int ei = 0;
    for (int i = 0; msg[i] && ei < 590; i++) {
        if (msg[i] == '"')  { escaped[ei++] = '\\'; escaped[ei++] = '"'; }
        else if (msg[i] == '\\') { escaped[ei++] = '\\'; escaped[ei++] = '\\'; }
        else { escaped[ei++] = msg[i]; }
    }
    escaped[ei] = '\0';

    softwareRtcUpdate();
    snprintf(json, sizeof(json),
             "{\"msg\":\"%s\",\"ts\":\"%s\",\"uptime\":%lu}",
             escaped, dateTimeStr, millis());

    s3FirebasePut("devices/" S3_DEVICE_ID "/console/response", json);
}


// ============================================================================
//              WIRELESS CONSOLE — poll Firebase for pending commands
// ============================================================================
void pollFirebaseCommands() {
    if (!s3WifiConnected) return;

    char rawBuf[512];
    if (!s3FirebaseGet("devices/" S3_DEVICE_ID "/console/command", rawBuf, sizeof(rawBuf))) {
        return; // null or network error — nothing pending
    }

    // Firebase returns a JSON string like: "GETSTATUS"  (with the quotes)
    // Strip surrounding quotes if present
    char cmd[256];
    int len = strlen(rawBuf);
    if (len >= 2 && rawBuf[0] == '"' && rawBuf[len-1] == '"') {
        strncpy(cmd, rawBuf + 1, len - 2);
        cmd[len - 2] = '\0';
    } else {
        strncpy(cmd, rawBuf, sizeof(cmd) - 1);
        cmd[sizeof(cmd)-1] = '\0';
    }

    if (strlen(cmd) == 0) return;

    // Immediately clear the command so we don't execute it again on next poll
    s3FirebasePut("devices/" S3_DEVICE_ID "/console/command", "null");

    // Log receipt
    char logMsg[320];
    snprintf(logMsg, sizeof(logMsg), "STATUS:WCMD received: %s", cmd);
    Serial.println(logMsg);

    // Execute
    handleWirelessCommand(cmd);
}


// ============================================================================
//              WIRELESS COMMAND HANDLER
//              Mirrors handleDebuggerCommand but responses go to Firebase
// ============================================================================
void handleWirelessCommand(const char* cmd) {
    char msg[512];

    // ── PING ──────────────────────────────────────────────────────────────────
    if (strcmp(cmd, "PING") == 0) {
        snprintf(msg, sizeof(msg),
                 "STATUS:PONG uptime=%lus heap=%u ip=%s rssi=%d",
                 millis() / 1000UL,
                 (unsigned int)ESP.getFreeHeap(),
                 WiFi.localIP().toString().c_str(),
                 WiFi.RSSI());
        firebaseConsolePrint(msg);
    }

    // ── GETSTATUS ─────────────────────────────────────────────────────────────
    else if (strcmp(cmd, "GETSTATUS") == 0) {
        snprintf(msg, sizeof(msg),
                 "STATUS:MODE=%d ALERT=%d ZONE=%d RESP=%d "
                 "H=%.2f THETA=%.2f RATIO=%.3f "
                 "P=%.2f DEPTH=%.2f LAT_ACCEL=%.3f "
                 "BATT=%.1f%% PUSH=%lums "
                 "RISE=%.3f SUST=%d "
                 "WiFi=%d IP=%s RSSI=%d NTP=%d REALTIME=%d "
                 "FB_OK=%lu FB_FAIL=%lu",
                 currentMode, floodAlertLevel, currentZone, currentResponse,
                 waterHeightCm, combinedTheta, floodRatio,
                 currentPressure, estimatedDepthCm, lateralAccel,
                 batteryPercent, currentPushIntervalMs,
                 rateOfRiseCmPer15Min, (int)sustainedRise,
                 (int)s3WifiConnected,
                 WiFi.localIP().toString().c_str(),
                 WiFi.RSSI(), (int)ntpSynced, (int)realtimeOverride,
                 s3FirebasePushCount, s3FirebaseFailCount);
        firebaseConsolePrint(msg);
    }

    // ── GETCONFIG ─────────────────────────────────────────────────────────────
    else if (strcmp(cmd, "GETCONFIG") == 0) {
        snprintf(msg, sizeof(msg),
                 "STATUS:CONFIG hMax=%.2f olpLen=%.2f mode=%d pushInt=%lums "
                 "ratio=%.3f health=%d "
                 "MPU=%d BMP=%d RTC=%d GPS=%d cal=%d baseline=%d "
                 "ntp=%d realtime=%d algo=%d "
                 "fbOK=%lu fbFail=%lu",
                 hMaxCm, olpLengthCm, currentMode, currentPushIntervalMs,
                 floodRatio, computeHealthScore(),
                 (int)mpuAvailable, (int)bmpAvailable,
                 (int)rtcAvailable, (int)gpsAvailable,
                 (int)calibrated, (int)baselineSet,
                 (int)ntpSynced, (int)realtimeOverride,
                 (int)algorithmEnabled,
                 s3FirebasePushCount, s3FirebaseFailCount);
        firebaseConsolePrint(msg);
    }

    // ── GETTHRESH ─────────────────────────────────────────────────────────────
    else if (strcmp(cmd, "GETTHRESH") == 0) {
        snprintf(msg, sizeof(msg),
                 "STATUS:THRESH accelTaut=%.3f accelSlack=%.3f tiltTaut=%.1f "
                 "floodTheta=%.1f floodRatio=%.2f submersionPa=%.1f",
                 threshLateralAccelTaut, threshLateralAccelSlack,
                 threshTiltTautDeg, threshFloodThetaDeg,
                 threshFloodRatio, threshSubmersionPa);
        firebaseConsolePrint(msg);
    }

    // ── RESETTHRESH ───────────────────────────────────────────────────────────
    else if (strcmp(cmd, "RESETTHRESH") == 0) {
        threshLateralAccelTaut  = DEFAULT_LATERAL_ACCEL_TAUT;
        threshLateralAccelSlack = DEFAULT_LATERAL_ACCEL_SLACK;
        threshTiltTautDeg       = DEFAULT_TILT_TAUT_DEG;
        threshFloodThetaDeg     = DEFAULT_FLOOD_THETA_DEG;
        threshFloodRatio        = DEFAULT_FLOOD_RATIO;
        threshSubmersionPa      = DEFAULT_SUBMERSION_PA;
        firebaseConsolePrint("STATUS:THRESH reset to firmware defaults");
    }

    // ── ALGO_ON ───────────────────────────────────────────────────────────────
    else if (strcmp(cmd, "ALGO_ON") == 0) {
        algorithmEnabled = true;
        firebaseConsolePrint("STATUS:Advanced flood detection algorithm ENABLED");
    }

    // ── ALGO_OFF ──────────────────────────────────────────────────────────────
    else if (strcmp(cmd, "ALGO_OFF") == 0) {
        algorithmEnabled = false;
        firebaseConsolePrint("STATUS:Advanced flood detection algorithm DISABLED");
    }

    // ── RECALIBRATE ───────────────────────────────────────────────────────────
    else if (strcmp(cmd, "RECALIBRATE") == 0) {
        firebaseConsolePrint("STATUS:Recalibration started — keep device STATIONARY");
        if (mpuAvailable) {
            recalibrate();
            firebaseConsolePrint("STATUS:Gyro recalibration complete");
        } else {
            firebaseConsolePrint("ERROR:MPU6050 not available — cannot recalibrate");
        }
        if (bmpAvailable) {
            calibratePressureBaseline();
            firebaseConsolePrint("STATUS:Pressure baseline recalibrated");
        } else {
            firebaseConsolePrint("WARNING:BMP280 not available — no pressure baseline");
        }
    }

    // ── DIAGRUN ───────────────────────────────────────────────────────────────
    else if (strcmp(cmd, "DIAGRUN") == 0) {
        diagRequested = true;
        firebaseConsolePrint("STATUS:Full hardware diagnostic queued — results will appear in /diagnostics");
    }

    // ── FORCEUPLOAD ───────────────────────────────────────────────────────────
    else if (strcmp(cmd, "FORCEUPLOAD") == 0) {
        firebaseConsolePrint("STATUS:Force-uploading sensor data to Firebase...");
        pushRawToFirebase();
        lastFirebasePushMs = millis(); // reset timer so normal schedule isn't disrupted
        firebaseConsolePrint("STATUS:Force upload complete");
    }

    // ── SETRATE=N,H ───────────────────────────────────────────────────────────
    else if (strncmp(cmd, "SETRATE=", 8) == 0) {
        int normalSec = 0, highSec = 0;
        if (sscanf(cmd + 8, "%d,%d", &normalSec, &highSec) == 2) {
            if (normalSec >= 5 && normalSec <= 3600 &&
                highSec  >= 2 && highSec  <= 3600) {
                // Map into existing push rate constants via NVS-persisted overrides
                // We patch currentPushIntervalMs directly; also save for config awareness
                preferences.putInt("normalRate", normalSec);
                preferences.putInt("highRate",   highSec);
                // Apply immediately to current push interval
                currentPushIntervalMs = (unsigned long)normalSec * 1000UL;
                snprintf(msg, sizeof(msg),
                         "STATUS:SETRATE applied normal=%ds high=%ds — currentPush=%lums",
                         normalSec, highSec, currentPushIntervalMs);
                firebaseConsolePrint(msg);
            } else {
                firebaseConsolePrint("ERROR:SETRATE values out of range (normal 5-3600s, high 2-3600s)");
            }
        } else {
            firebaseConsolePrint("ERROR:Format: SETRATE=<normalSec>,<highSec>  e.g. SETRATE=30,10");
        }
    }

    // ── SETHMAX=X ─────────────────────────────────────────────────────────────
    else if (strncmp(cmd, "SETHMAX=", 8) == 0) {
        float newH = atof(cmd + 8);
        if (newH > 1.0f && newH < 10000.0f) {
            hMaxCm = newH;
            olpLengthCm = newH;
            preferences.putFloat("hMaxCm", hMaxCm);
            computeDynamicPushRate();
            snprintf(msg, sizeof(msg),
                     "STATUS:H_MAX set to %.2f cm (saved to NVS)", hMaxCm);
            firebaseConsolePrint(msg);
        } else {
            firebaseConsolePrint("ERROR:SETHMAX value must be 1–10000 cm");
        }
    }

    // ── SETTHRESH=A,W,D ───────────────────────────────────────────────────────
    else if (strncmp(cmd, "SETTHRESH=", 10) == 0) {
        float a = 0, w = 0, d = 0;
        if (sscanf(cmd + 10, "%f,%f,%f", &a, &w, &d) == 3) {
            if (a > 0.01f && a < 5.0f) {
                threshLateralAccelTaut  = a;
                threshLateralAccelSlack = a * 0.67f;
            }
            if (w > 0.1f && w < 90.0f)    threshFloodThetaDeg  = w;
            if (d > 10.0f && d < 100000.0f) threshSubmersionPa = d;
            snprintf(msg, sizeof(msg),
                     "STATUS:THRESH updated accelTaut=%.3f floodTheta=%.1f submersionPa=%.1f",
                     threshLateralAccelTaut, threshFloodThetaDeg, threshSubmersionPa);
            firebaseConsolePrint(msg);
        } else {
            firebaseConsolePrint("ERROR:Format: SETTHRESH=<accelTaut>,<floodTheta_deg>,<submersionPa>");
        }
    }

    // ── Unknown ───────────────────────────────────────────────────────────────
    else {
        snprintf(msg, sizeof(msg), "ERROR:Unknown wireless command: %s", cmd);
        firebaseConsolePrint(msg);
    }
}

// ============================================================================
//                       UTILITY FUNCTIONS
// ============================================================================
void statusPrint(const char* msg) { Serial.print("STATUS:"); Serial.println(msg); }
void errorPrint(const char* msg)  { Serial.print("ERROR:");  Serial.println(msg); }
void warningPrint(const char* msg){ Serial.print("WARNING:");Serial.println(msg); }