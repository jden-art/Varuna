// ===================================================================
// VARUNA FLOOD MONITORING SYSTEM — COMPLETE FIRMWARE (MODIFIED)
// Target: ESP32-S3 (Main Controller)
// ===================================================================


#include <Wire.h>
#include <HardwareSerial.h>
#include <math.h>
#include <EEPROM.h>
#include <SPIFFS.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>

/* ================================================================== */
/* ========== PIN DEFINITIONS ======================================= */
/* ================================================================== */

#define MPU6050_ADDR 0x68
#define BMP280_ADDR 0x76
#define DS1307_ADDR 0x68

#define BUS0_SDA 8
#define BUS0_SCL 9

#define BUS1_SDA 4
#define BUS1_SCL 5

#define GPS_RX_PIN 6
#define GPS_TX_PIN 7

#define SIM_RX_PIN 15
#define SIM_TX_PIN 16
#define SIM_RST_PIN 17

#define LED_PIN 3

#define ALPHA 0.98

/* ========== HC-SR04 PINS ========== */
#define HCSR04_TRIG_PIN 10
#define HCSR04_ECHO_PIN 11

/* ========== BATTERY ADC PIN ========== */
#define BATTERY_ADC_PIN 2
/* C3 Display Feed — Software Serial TX on GPIO 14 */
#define C3_FEED_PIN  14
#define C3_FEED_BAUD 9600   // Slower baud = reliable software TX

void c3FeedInit()
{
    pinMode(C3_FEED_PIN, OUTPUT);
    digitalWrite(C3_FEED_PIN, HIGH);  // Idle HIGH for UART
}

void c3FeedByte(uint8_t b)
{
    /* Software UART TX at 9600 baud = 104μs per bit — very reliable */
    const uint32_t bitDelayUs = 104;
    
    portDISABLE_INTERRUPTS();
    
    digitalWrite(C3_FEED_PIN, LOW);  // Start bit
    delayMicroseconds(bitDelayUs);
    
    for (int i = 0; i < 8; i++)
    {
        digitalWrite(C3_FEED_PIN, (b & (1 << i)) ? HIGH : LOW);
        delayMicroseconds(bitDelayUs);
    }
    
    digitalWrite(C3_FEED_PIN, HIGH);  // Stop bit
    delayMicroseconds(bitDelayUs);
    
    portENABLE_INTERRUPTS();
}

void c3FeedPrint(const char *str)
{
    while (*str)
    {
        c3FeedByte((uint8_t)*str);
        str++;
    }
}

void c3FeedPrintln(const char *str)
{
    c3FeedPrint(str);
    c3FeedByte('\r');
    c3FeedByte('\n');
}
/* ================================================================== */
/* ========== ALGORITHM TOGGLE SYSTEM =============================== */
/* ================================================================== */
/*
 * Physical push-button on GPIO 12 toggles the flood detection
 * algorithm ON or OFF.
 *
 * When algorithm is DISABLED:
 *   - System reads sensors every loop iteration (continuous)
 *   - No adaptive sampling intervals applied
 *   - No flood evaluation or alert dispatch
 *   - No light-sleep mode entered
 *   - ALGO_STATUS_LED (GPIO 13) is ON (solid)
 *   - Serial CSV output continues at 1-second intervals
 *   - Useful for raw data acquisition and bench testing
 *
 * When algorithm is ENABLED (default on boot):
 *   - Normal adaptive sampling with flood evaluation
 *   - Alerts dispatched per response level
 *   - Light-sleep entered when idle and conditions allow
 *   - ALGO_STATUS_LED is OFF
 *
 * The algorithm can also be toggled via serial commands:
 *   ALGOON   — enable algorithm
 *   ALGOOFF  — disable algorithm (continuous mode)
 *   ALGOSTATUS — report current algorithm state
 *
 * State is NOT persisted across reboots (defaults to ENABLED).
 */

#define ALGO_BUTTON_PIN       12
#define ALGO_STATUS_LED_PIN   13

#define ALGO_DEBOUNCE_MS      250UL

boolean algorithmEnabled = false;  /* START DISABLED for continuous data */
boolean algoButtonLastState = HIGH;
unsigned long algoButtonLastPress = 0;

/* ================================================================== */
/* ========== OBSTRUCTION LIGHT SYSTEM (5 RED LEDS) ================ */
/* ================================================================== */
/*
 * Marine Obstruction Lighting per IALA Recommendation O-139
 * and USCG 33 CFR 66 for fixed obstructions:
 *   - Flash character: Fixed flashing (Fl)
 *   - Default period: 4.0 seconds (0.5s ON, 3.5s OFF)
 *   - Color: Red (hardware)
 *   - All 5 LEDs flash simultaneously
 *   - Duty cycle ~12.5% to conserve battery
 *
 * International Standard Presets available:
 *   SETOBPRESET:<name>
 *     IALA_FL4S   — IALA O-139 Fl 4s   (500/3500)   DEFAULT
 *     IALA_FL2S   — IALA O-139 Fl 2.5s (300/2200)
 *     IALA_FL5S   — IALA O-139 Fl 5s   (500/4500)
 *     IALA_QFL    — IALA O-139 Quick    (250/750)
 *     IALA_VQF    — IALA O-139 V.Quick  (167/333)
 *     IALA_LFL10  — IALA O-139 LFl 10s (2000/8000)
 *     IALA_ISO4   — IALA O-139 Iso 4s  (2000/2000)
 *     IALA_ISO2   — IALA O-139 Iso 2s  (1000/1000)
 *     USCG_STD    — USCG 33CFR66 Std   (1000/4000)
 *     USCG_QFL    — USCG 33CFR66 Quick (600/600)
 *
 * Custom timing via serial:
 *   SETOBLIGHT:<on_ms>,<off_ms>
 *   Example: SETOBLIGHT:500,3500
 *   Query:   GETOBLIGHT
 *   List:    GETOBPRESETS
 */

#define OB_LED_COUNT 5
/* Obstruction LED pin assignments on ESP32-S3 */
#define OB_LED_PIN_0 2
#define OB_LED_PIN_1 2
#define OB_LED_PIN_2 2
#define OB_LED_PIN_3 2
#define OB_LED_PIN_4 2

const int obLedPins[OB_LED_COUNT] = {
    OB_LED_PIN_0,
    OB_LED_PIN_1,
    OB_LED_PIN_2,
    OB_LED_PIN_3,
    OB_LED_PIN_4
};

/* IALA Recommendation O-139 default flash timing */
#define OB_DEFAULT_ON_MS   500UL
#define OB_DEFAULT_OFF_MS  3500UL
#define OB_MIN_ON_MS       50UL
#define OB_MAX_ON_MS       5000UL
#define OB_MIN_OFF_MS      100UL
#define OB_MAX_OFF_MS      30000UL

unsigned long obOnTimeMs  = OB_DEFAULT_ON_MS;
unsigned long obOffTimeMs = OB_DEFAULT_OFF_MS;
unsigned long obLastToggleTime = 0;
boolean obLedsCurrentlyOn = false;
boolean obLightEnabled = true;

/* Current preset name tracking */
#define OB_PRESET_NAME_LEN 16
char currentObPresetName[OB_PRESET_NAME_LEN] = "IALA_FL4S";

/* ---- International Standard Presets Table ---- */
/*
 * Each preset defines a name, description string, ON time, and OFF time
 * conforming to the referenced international standard.
 *
 * References:
 *   IALA Recommendation O-139 (Edition 2)
 *   IALA Dictionary of International Terms
 *   USCG 33 CFR Part 66 — Private Aids to Navigation
 *   USCG Light List — Standard rhythmic characters
 */

#define OB_PRESET_COUNT 10

struct ObLightPreset
{
    const char *name;
    const char *description;
    unsigned long onMs;
    unsigned long offMs;
};

const ObLightPreset obPresets[OB_PRESET_COUNT] = {
    /* Index 0: IALA Fixed Flash 4-second period */
    {
        "IALA_FL4S",
        "IALA O-139 Fl 4s (default)",
        500UL,
        3500UL
    },
    /* Index 1: IALA Fixed Flash 2.5-second period */
    {
        "IALA_FL2S",
        "IALA O-139 Fl 2.5s",
        300UL,
        2200UL
    },
    /* Index 2: IALA Fixed Flash 5-second period */
    {
        "IALA_FL5S",
        "IALA O-139 Fl 5s",
        500UL,
        4500UL
    },
    /* Index 3: IALA Quick Flash (60 flashes/min) */
    {
        "IALA_QFL",
        "IALA O-139 Q (Quick 60/min)",
        250UL,
        750UL
    },
    /* Index 4: IALA Very Quick Flash (120 flashes/min) */
    {
        "IALA_VQF",
        "IALA O-139 VQ (V.Quick 120/min)",
        167UL,
        333UL
    },
    /* Index 5: IALA Long Flash 10-second period */
    {
        "IALA_LFL10",
        "IALA O-139 LFl 10s",
        2000UL,
        8000UL
    },
    /* Index 6: IALA Isophase 4-second period */
    {
        "IALA_ISO4",
        "IALA O-139 Iso 4s",
        2000UL,
        2000UL
    },
    /* Index 7: IALA Isophase 2-second period */
    {
        "IALA_ISO2",
        "IALA O-139 Iso 2s",
        1000UL,
        1000UL
    },
    /* Index 8: USCG 33 CFR 66 Standard obstruction */
    {
        "USCG_STD",
        "USCG 33CFR66 Standard",
        1000UL,
        4000UL
    },
    /* Index 9: USCG 33 CFR 66 Quick flash */
    {
        "USCG_QFL",
        "USCG 33CFR66 Quick",
        600UL,
        600UL
    }
};

/* EEPROM addresses for obstruction light persistence */
#define EEPROM_OB_ON_ADDR    290
#define EEPROM_OB_OFF_ADDR   294
#define EEPROM_OB_MAGIC_ADDR 298
#define EEPROM_OB_MAGIC_VAL  0xBE
#define EEPROM_OB_PRESET_ADDR 300

/* ================================================================== */
/* ========== DETACHABLE DEBUG SYSTEM =============================== */
/* ================================================================== */
/*
 * Debug circuit: ESP32-C3 Seeed XIAO connected via UART
 * Physical push button on a GPIO enables/disables debug output.
 *
 * When debug is DISABLED:
 *   - No debug serial writes occur (zero CPU overhead)
 *   - Debug UART is not accessed
 *   - No formatting of debug strings is performed
 *
 * When debug is ENABLED:
 *   - Periodic status packets sent to debug UART
 *   - Algorithm state, sensor readings, flood status transmitted
 *   - Formatted as structured text lines prefixed with "DBG:"
 *
 * Push button toggles debug state on each press (with debounce).
 * State is NOT persisted across reboots (defaults to disabled).
 *
 * Debug UART pins (directly to Seeed XIAO C3):
 *   ESP32-S3 TX -> XIAO C3 RX
 *   ESP32-S3 RX -> XIAO C3 TX (for future bidirectional use)
 */

#define DEBUG_UART_TX_PIN  43
#define DEBUG_UART_RX_PIN  44
#define DEBUG_UART_BAUD    115200

#define DEBUG_BUTTON_PIN   0

#define DEBUG_DEBOUNCE_MS  250UL

#define DEBUG_OUTPUT_INTERVAL_MS  2000UL

#define DEBUG_PACKET_SIZE  512

/* DebugSerial removed - was conflicting with SIM_Serial on UART2 */
/* Debug output uses software bit-bang TX on DEBUG_UART_TX_PIN instead */

boolean debugEnabled = false;
boolean debugButtonLastState = HIGH;
unsigned long debugButtonLastPress = 0;
unsigned long debugLastOutputTime = 0;

/* Debug packet buffer */
char debugPacketBuffer[DEBUG_PACKET_SIZE];

/* Debug statistics counters */
uint32_t debugPacketsSent = 0;
uint32_t debugBytesSent = 0;

/* ================================================================== */
/* ========== I2C AND UART BUS OBJECTS ============================== */
/* ================================================================== */

TwoWire I2C_BUS0 = TwoWire(0);
TwoWire I2C_BUS1 = TwoWire(1);
HardwareSerial GPS_Serial(1);

/* ================================================================== */
/* ========== SIM800L SERIAL (ESP32-S3 UART ALLOCATION) ============= */
/* ================================================================== */
/*
 * ESP32-S3 has 3 hardware UARTs: UART0 (USB/Serial), UART1, UART2.
 * UART0 = Serial (USB debug / Processing IDE communication)
 * UART1 = GPS_Serial (9600 baud, pins 6/7)
 * UART2 = SIM800L (9600 baud, pins 15/16)
 *
 * Debug output uses software bit-bang TX on DEBUG_UART_TX_PIN.
 * This avoids consuming a hardware UART for debug-only output.
 */

/* SIM800L on hardware UART2 */
HardwareSerial SIM_Serial(2);

/* Forward declaration of software debug TX function */
void debugSerialWrite(const char *data, int len);
void debugSerialPrint(const char *str);
void debugSerialPrintln(const char *str);

/* Software TX state for debug output */
#define DEBUG_SW_BAUD 115200
boolean debugSwTxInitialized = false;

/* ================================================================== */
/* ========== MPU6050 VARIABLES ===================================== */
/* ================================================================== */

int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

float gyroOffsetX = 0;
float gyroOffsetY = 0;
float gyroOffsetZ = 0;

float refAccX = 0;
float refAccY = 0;
float refAccZ = 1.0;

float refTiltX = 0;
float refTiltY = 0;

float filtTiltX = 0;
float filtTiltY = 0;

unsigned long prevTime = 0;

float olpLength = 100.0;

/* ========== INPUT BUFFER ========== */
#define INPUT_BUFFER_SIZE 128
char inputBuffer[INPUT_BUFFER_SIZE];
int inputBufferIdx = 0;

/* ================================================================== */
/* ========== BMP280 VARIABLES ====================================== */
/* ================================================================== */

boolean bmpAvailable = false;

uint16_t bmpDigT1;
int16_t bmpDigT2, bmpDigT3;
uint16_t bmpDigP1;
int16_t bmpDigP2, bmpDigP3, bmpDigP4, bmpDigP5;
int16_t bmpDigP6, bmpDigP7, bmpDigP8, bmpDigP9;
int32_t bmpTFine;

float currentPressure = 0;
float currentTemperature = 0;

#define BASELINE_SIZE 48
float baselineBuffer[BASELINE_SIZE];
int baselineIndex = 0;
int baselineCount = 0;
float baselineSum = 0;
float baselinePressure = 0;

unsigned long lastBaselineUpdate = 0;
#define BASELINE_INTERVAL 1800000

unsigned long lastBmpRead = 0;
#define BMP_READ_INTERVAL 5000

int submersionState = 0;
float pressureDeviation = 0;
float estimatedDepth = 0;

/* ================================================================== */
/* ========== DS1307 RTC VARIABLES ================================== */
/* ================================================================== */

boolean rtcAvailable = false;
boolean rtcTimeValid = false;
uint32_t currentUnixTime = 0;
uint8_t rtcSeconds, rtcMinutes, rtcHours;
uint8_t rtcDay, rtcMonth;
uint16_t rtcYear;
uint8_t rtcDayOfWeek;

unsigned long lastRtcRead = 0;
#define RTC_READ_INTERVAL 1000

uint32_t bootUnixTime = 0;
uint32_t sessionStartUnix = 0;
uint32_t sessionDuration = 0;

/* ================================================================== */
/* ========== RATE OF CHANGE VARIABLES ============================== */
/* ================================================================== */

uint32_t prevReadingTime = 0;
float prevWaterHeight = 0;
float rateOfChange = 0;
float ratePer15Min = 0;

#define MIN_RATE_ELAPSED_SEC 60
#define MAX_PLAUSIBLE_RATE 200.0

/* ================================================================== */
/* ========== DATA LOG VARIABLES ==================================== */
/* ================================================================== */

#define LOG_SIZE 120
float logHeight[LOG_SIZE];
float logTheta[LOG_SIZE];
float logPressure[LOG_SIZE];
uint32_t logTime[LOG_SIZE];
int logIndex = 0;
int logCount = 0;

unsigned long lastLogWrite = 0;
#define LOG_INTERVAL 10000

/* ================================================================== */
/* ========== GPS VARIABLES ========================================= */
/* ================================================================== */

double gpsLat = 0;
double gpsLon = 0;
float gpsAlt = 0;
float gpsSpeed = 0;
float gpsHdop = 99.9;
int gpsSatellites = 0;
boolean gpsFixValid = false;
boolean gpsAvailable = false;

uint8_t gpsHour = 0;
uint8_t gpsMin = 0;
uint8_t gpsSec = 0;
uint8_t gpsGGADay = 0;
uint8_t gpsGGAMonth = 0;
uint16_t gpsGGAYear = 0;
boolean gpsTimeValid = false;
boolean gpsTimeSynced = false;

unsigned long lastGpsRtcSync = 0;
#define GPS_RTC_SYNC_INTERVAL 86400000UL

char nmeaBuffer[120];
int nmeaIdx = 0;

unsigned long lastGpsProcess = 0;
#define GPS_PROCESS_INTERVAL 100

/* ================================================================== */
/* ========== SIM800L VARIABLES ===================================== */
/* ================================================================== */

#define SIM_RESPONSE_SIZE 256
char simResponseBuffer[SIM_RESPONSE_SIZE];

boolean simAvailable = false;
boolean simRegistered = false;
boolean simReady = false;
int simSignalRSSI = 0;
int simInitFailCode = 0;

unsigned long lastSimSignalCheck = 0;
#define SIM_SIGNAL_INTERVAL 60000

unsigned long lastSimRegistrationCheck = 0;
#define SIM_REG_CHECK_INTERVAL 30000

enum ATResult
{
    AT_SUCCESS = 0,
    AT_TIMEOUT = 1,
    AT_ERROR = 2
};

/* ================================================================== */
/* ========== FLOOD DETECTION SYSTEM (SYSTEM 1) VARIABLES ========== */
/* ================================================================== */

float alertLevelCm   = 120.0;
float warningLevelCm = 180.0;
float dangerLevelCm  = 250.0;

#define RATE_MODERATE_THRESH 2.0
#define RATE_FAST_THRESH     5.0

#define RATE_EXTREME_THRESH 30.0
#define RATE_CATASTROPHIC_THRESH 50.0

#define ZONE_NORMAL  0
#define ZONE_ALERT   1
#define ZONE_WARNING 2
#define ZONE_DANGER  3

#define RATE_FALLING  0
#define RATE_SLOW     1
#define RATE_MODERATE 2
#define RATE_FAST     3

#define RESP_NORMAL     0
#define RESP_WATCH      1
#define RESP_WARNING    2
#define RESP_FLOOD      3
#define RESP_CRITICAL   4

#define SUSTAINED_BUF_SIZE 4
float sustainedBuffer[SUSTAINED_BUF_SIZE];
uint32_t sustainedTimeBuffer[SUSTAINED_BUF_SIZE];
int sustainedBufIdx = 0;
int sustainedBufCount = 0;
boolean sustainedRise = false;

#define SUSTAINED_EPSILON_CM 0.5

int currentZone = ZONE_NORMAL;
int currentRateCategory = RATE_SLOW;
int currentResponseLevel = RESP_NORMAL;
int previousResponseLevel = RESP_NORMAL;

int floodAlertLevel = 0;

float peakHeight = 0;
float minHeight = 9999;
uint32_t peakTime = 0;
uint32_t minTime = 0;

int stepDownConsecutive = 0;
#define STEPDOWN_READINGS_REQUIRED 4
#define STEPDOWN_NORMAL_READINGS   8

#define MIN_TIME_CRITICAL  900
#define MIN_TIME_FLOOD     1800
#define MIN_TIME_WARNING   1800
#define MIN_TIME_WATCH     900
uint32_t stateEntryTime = 0;

#define MAX_CONTACTS_PER_TIER 3
#define PHONE_NUMBER_LENGTH   16

char tier1Contacts[MAX_CONTACTS_PER_TIER][PHONE_NUMBER_LENGTH];
char tier2Contacts[MAX_CONTACTS_PER_TIER][PHONE_NUMBER_LENGTH];
char tier3Contacts[MAX_CONTACTS_PER_TIER][PHONE_NUMBER_LENGTH];
int tier1Count = 0;
int tier2Count = 0;
int tier3Count = 0;

uint32_t lastAlertTimeByLevel[5] = {0, 0, 0, 0, 0};

#define ALERT_SMS_COOLDOWN_SEC 300
#define REALERT_INTERVAL_WARNING  3600
#define REALERT_INTERVAL_FLOOD    1800
#define REALERT_INTERVAL_CRITICAL 900

int highestResponseReached = RESP_NORMAL;
boolean allClearPending = false;

int readingsSinceBoot = 0;

/* ========== AUTHORIZED NUMBERS FOR SMS COMMANDS ========== */
#define MAX_AUTHORIZED 5
char authorizedNumbers[MAX_AUTHORIZED][PHONE_NUMBER_LENGTH];
int authorizedCount = 0;

/* ========== SMS ACKNOWLEDGMENT SYSTEM ========== */
boolean ackReceived = false;
uint32_t ackWaitStartTime = 0;
#define ACK_TIMEOUT_SEC 900
boolean ackEscalationDone = false;

/* ================================================================== */
/* ========== ADAPTIVE SAMPLING SYSTEM (SYSTEM 2) VARIABLES ======== */
/* ================================================================== */

unsigned long sampleIntervalMs  = 30000UL;
unsigned long transmitIntervalMs = 60000UL;

unsigned long lastFloodSampleTime = 0;
unsigned long lastTransmitTime = 0;

#define INTERVAL_2MIN   120000UL
#define INTERVAL_5MIN   300000UL
#define INTERVAL_10MIN  600000UL
#define INTERVAL_15MIN  900000UL
#define INTERVAL_30MIN  1800000UL
#define INTERVAL_60MIN  3600000UL
#define INTERVAL_120MIN 7200000UL
#define INTERVAL_240MIN 14400000UL

float batteryPercent = 100.0;
#define BATTERY_LOW_THRESH    20.0
#define BATTERY_CRITICAL_THRESH 10.0

unsigned long lastBatteryRead = 0;
#define BATTERY_READ_INTERVAL 30000UL

boolean useGprsTransmit = true;

#define TX_BUFFER_SIZE 30
float txBufferHeight[TX_BUFFER_SIZE];
float txBufferRate[TX_BUFFER_SIZE];
uint32_t txBufferTime[TX_BUFFER_SIZE];
int txBufferIdx = 0;
int txBufferCount = 0;

/* ========== BATTERY ADC CONSTANTS ========== */
#define BATTERY_FULL_MV     12700
#define BATTERY_EMPTY_MV    10500
#define BATTERY_DIVIDER_RATIO 5.7
#define ADC_MAX_MV          3300
#define ADC_RESOLUTION      4095

/* ================================================================== */
/* ========== EEPROM STATE PERSISTENCE ============================= */
/* ================================================================== */

#define EEPROM_MAGIC_ADDR       0
#define EEPROM_RESP_LEVEL_ADDR  1
#define EEPROM_SBUF0_ADDR       2
#define EEPROM_SBUF1_ADDR       6
#define EEPROM_SBUF2_ADDR       10
#define EEPROM_SBUF3_ADDR       14
#define EEPROM_SAVE_TIME_ADDR   18
#define EEPROM_ALERT_ADDR       22
#define EEPROM_WARNING_ADDR     26
#define EEPROM_DANGER_ADDR      30
#define EEPROM_OLP_ADDR         34
#define EEPROM_PEAK_ADDR        38
#define EEPROM_PEAK_TIME_ADDR   42
#define EEPROM_PREV_HEIGHT_ADDR 46
#define EEPROM_STATE_ENTRY_ADDR 50
#define EEPROM_MAGIC_VALUE      0xA5
#define EEPROM_STATE_MAX_AGE    1800

/* ========== EEPROM CONTACT PERSISTENCE ========== */
#define EEPROM_CONTACTS_BASE      54
#define EEPROM_CONTACT_MAGIC_ADDR 282
#define EEPROM_CONTACT_MAGIC      0xC3

/* ========== EEPROM WRITE THROTTLING ========== */
unsigned long lastEepromSave = 0;
#define EEPROM_SAVE_INTERVAL 1800000UL
boolean eepromDirty = false;

/* ========== SERIAL OUTPUT GATING ========== */
unsigned long lastSerialOutput = 0;
#define SERIAL_OUTPUT_INTERVAL 1000
boolean serialOutputEnabled = true;

/* ========== SENSOR HEALTH ========== */
enum SensorSource
{
    SENSOR_MPU6050,
    SENSOR_HCSR04,
    SENSOR_NONE
};

SensorSource activeSensor = SENSOR_MPU6050;
boolean mpuHealthy = true;
int mpuConsecutiveFailures = 0;
#define MPU_FAIL_THRESHOLD 5

boolean hcsr04Available = false;
float hcsr04MountHeight = 300.0;

unsigned long lastSensorHealthCheck = 0;
#define SENSOR_HEALTH_INTERVAL 60000

/* ========== SMS QUEUE (NON-BLOCKING) ========== */
#define SMS_QUEUE_SIZE 15
#define SMS_MSG_SIZE 160

struct SmsQueueEntry
{
    char phone[PHONE_NUMBER_LENGTH];
    char message[SMS_MSG_SIZE];
    boolean pending;
    uint8_t retries;
};

SmsQueueEntry smsQueue[SMS_QUEUE_SIZE];
int smsQueueHead = 0;
int smsQueueTail = 0;
int smsQueueCount = 0;

char smsMsgBuffer[SMS_MSG_SIZE];

/* ========== SMS RECEIVE STATE MACHINE ========== */
enum SmsRxState
{
    SMS_RX_IDLE,
    SMS_RX_HEADER_RECEIVED
};

SmsRxState smsRxState = SMS_RX_IDLE;
char smsRxSender[PHONE_NUMBER_LENGTH];
char smsRxBody[161];
int smsRxBodyIdx = 0;

char simUrcBuffer[256];
int simUrcIdx = 0;

/* ========== GPRS VARIABLES ========== */
char gprsApn[32] = "airtelgprs.com";
char gprsServer[64] = "http://varuna.example.com/api/data";
boolean gprsConnected = false;
int gprsConsecutiveFails = 0;
#define GPRS_MAX_FAILS 3

/* ========== SPIFFS STORAGE ========== */
boolean spiffsAvailable = false;

/* ========== HEIGHT INVERSION FLAG ========== */
boolean heightInverted = false;

/* ========== WATCHDOG TIMEOUT ========== */
#define WDT_TIMEOUT_SEC 120

/* ========== GPS LAST FIX TRACKING ========== */
unsigned long lastGpsFixTime = 0;

/* ========== COMPLEMENTARY FILTER DT EXCESSIVE FLAG ========== */
boolean dtExcessive = false;

/* ========== LAST VALID FLOOD WATER HEIGHT (for transmit) ========== */
float lastFloodWaterHeight = 0;

/* ================================================================== */
/* ========== FORWARD DECLARATIONS ================================== */
/* ================================================================== */

void readMPU();
uint8_t mpuReadReg(uint8_t reg);
void mpuWriteReg(uint8_t reg, uint8_t val);
void recalibrate();
void dumpLog();
void saveStateToEeprom();
boolean restoreStateFromEeprom();
void saveContactsToEeprom();
boolean restoreContactsFromEeprom();
void markEepromDirty();
void saveEepromIfNeeded();
void forceSaveEeprom();
void parseFloodCommand(const char *cmd);
boolean simSendSMS(const char *phoneNumber, const char *message);
void simFullInit();
void simHardwareReset();
void simFlushInput();
ATResult send_at_command(const char *command, const char *expected, unsigned long timeout_ms);
void simCheckSignalPeriodic();
void simCheckRegistrationPeriodic();
int simGetSignalStrength();
void readGPS();
void syncRTCfromGPS();
void readBatteryLevel();
void updateAdaptiveIntervals();
void evaluateFloodStatus(float waterHeight, uint32_t timestamp);
void calculateRateOfChange(float currentHeight, uint32_t currentTime);
void addToTransmitBuffer(float waterHeight, float rate, uint32_t timestamp);
void transmitBufferedData(float currentHeight);
void logReading(float height, float thetaVal, float press, uint32_t timestamp);
void checkSerialInput();
void processCommand(const char *cmd);
void checkSimURC();
void processOneSmsFromQueue();
boolean queueSMS(const char *phone, const char *message);
int sendToTier(char contacts[][PHONE_NUMBER_LENGTH], int count);
void dispatchAlerts(int responseLevel, float waterHeight, float rate,
                    int zone, boolean sustained, boolean isEscalation);
void dispatchDeescalation(int fromLevel, int toLevel, float waterHeight);
void dispatchAllClear(float waterHeight, float peak, uint32_t peakT);
void composeAlertSms(int responseLevel, float waterHeight, float rate,
                     int zone, boolean sustained);
void composeAllClearSms(float currentHeight, float peak, uint32_t peakT);
void composeDeescalationSms(int fromLevel, int toLevel, float currentHeight);
const char *responseNameStr(int level);
const char *zoneNameStr(int zone);
uint32_t getBestTimestamp();
void checkAckTimeout();
void handleAckSms(const char *sender);
void resetAckTracking();
void escalateWithVoiceCalls();
boolean simMakeVoiceCall(const char *phoneNumber, int ringDurationSec);
void archiveToStorage(float height, float rate, uint32_t timestamp,
                      int zone, int response);
void checkSensorHealth();
void i2cBusRecovery(int sdaPin, int sclPin);
boolean i2cHealthCheck(TwoWire *bus, uint8_t addr, int sdaPin, int sclPin);
void hcsr04Init();
float hcsr04ReadDistance();
float hcsr04GetWaterHeight();
float calculateWaterHeight(float thetaDeg);
boolean gprsInit();
boolean gprsPostData(const char *jsonPayload, int payloadLen);
boolean gprsUploadBuffer();
void initStorage();
void flushSustainedBufferIfIntervalChanged(unsigned long oldInterval,
        unsigned long newInterval);
boolean validateNmeaChecksum(const char *sentence);
void floatToStr(char *buf, int bufSize, float val, int decimals);
boolean contactExists(char contacts[][PHONE_NUMBER_LENGTH], int count,
                      const char *phone);
boolean initBMP280();
void bmpReadData(float *temperature, float *pressure);
void updateBaseline(float press);
int evaluateSubmersion(float press, float base);
boolean canLightSleep();
void enterLightSleep(unsigned long sleepDurationMs);

/* Obstruction light forward declarations */
void obLightInit();
void obLightUpdate(unsigned long now);
void obLightSetAll(boolean state);
void obLightSaveToEeprom();
boolean obLightRestoreFromEeprom();
boolean obLightApplyPreset(const char *presetName);
void obLightListPresets();
int obLightFindPreset(const char *presetName);

/* Algorithm toggle forward declarations */
void algoToggleInit();
void algoCheckButton(unsigned long now);
void setAlgorithmEnabled(boolean enabled);

/* Debug system forward declarations */
void debugInit();
void debugCheckButton(unsigned long now);
void debugSendPeriodicPacket(unsigned long now, float theta, float waterHeight,
                             float correctedTiltX, float correctedTiltY,
                             float horizontalDist);
void debugSendStatusEvent(const char *eventStr);
void debugSendCalibrationStatus(const char *phase, int progress, int total);
void debugSendSensorInit(const char *sensorName, boolean success);
void debugSendFloodTransition(int fromLevel, int toLevel, float waterHeight);
void debugSendAlertDispatch(int level, int smsCount);
void debugSendGpsStatus();
void debugSendSimStatus();
void debugSendBatteryStatus();
void debugSwTxByte(uint8_t b);
void debugSwTxInit();

/* ================================================================== */
/* ========== BCD CONVERSION ======================================== */
/* ================================================================== */

uint8_t bcdToDec(uint8_t val)
{
    return ((val >> 4) * 10) + (val & 0x0F);
}

uint8_t decToBcd(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

/* ================================================================== */
/* ========== FLOAT TO STRING ======================================= */
/* ================================================================== */

void floatToStr(char *buf, int bufSize, float val, int decimals)
{
    if (isnan(val))
    {
        strncpy(buf, "NaN", bufSize - 1);
        buf[bufSize - 1] = '\0';
        return;
    }
    if (isinf(val))
    {
        if (val > 0)
        {
            strncpy(buf, "Inf", bufSize - 1);
        }
        else
        {
            strncpy(buf, "-Inf", bufSize - 1);
        }
        buf[bufSize - 1] = '\0';
        return;
    }

    if (val < 0)
    {
        buf[0] = '-';
        floatToStr(buf + 1, bufSize - 1, -val, decimals);
        return;
    }

    if (val > 999999999.0f)
    {
        strncpy(buf, "OVF", bufSize - 1);
        buf[bufSize - 1] = '\0';
        return;
    }

    long intPart = (long)val;
    float fracPart = val - (float)intPart;

    if (decimals == 0)
    {
        snprintf(buf, bufSize, "%ld", intPart);
    }
    else if (decimals == 1)
    {
        int frac = (int)(fracPart * 10 + 0.5);
        if (frac >= 10)
        {
            intPart++;
            frac = 0;
        }
        snprintf(buf, bufSize, "%ld.%d", intPart, frac);
    }
    else if (decimals == 2)
    {
        int frac = (int)(fracPart * 100 + 0.5);
        if (frac >= 100)
        {
            intPart++;
            frac = 0;
        }
        snprintf(buf, bufSize, "%ld.%02d", intPart, frac);
    }
    else if (decimals == 3)
    {
        int frac = (int)(fracPart * 1000 + 0.5);
        if (frac >= 1000)
        {
            intPart++;
            frac = 0;
        }
        snprintf(buf, bufSize, "%ld.%03d", intPart, frac);
    }
    else
    {
        int frac = (int)(fracPart * 10000 + 0.5);
        if (frac >= 10000)
        {
            intPart++;
            frac = 0;
        }
        snprintf(buf, bufSize, "%ld.%04d", intPart, frac);
    }
}

/* ================================================================== */
/* ========== CONTACT DUPLICATE CHECK =============================== */
/* ================================================================== */

boolean contactExists(char contacts[][PHONE_NUMBER_LENGTH], int count,
                      const char *phone)
{
    int phoneLen = strlen(phone);
    if (phoneLen < 4)
    {
        return false;
    }

    for (int i = 0; i < count; i++)
    {
        int existingLen = strlen(contacts[i]);
        int compareLen = phoneLen < existingLen ? phoneLen : existingLen;
        if (compareLen > 10)
        {
            compareLen = 10;
        }

        if (compareLen >= 4)
        {
            if (strcmp(contacts[i] + existingLen - compareLen,
                       phone + phoneLen - compareLen) == 0)
            {
                return true;
            }
        }
    }
    return false;
}

/* ================================================================== */
/* ========== EEPROM HELPER FUNCTIONS =============================== */
/* ================================================================== */

void eepromWriteFloat(int addr, float value)
{
    uint8_t *p = (uint8_t *)&value;
    for (int i = 0; i < 4; i++)
    {
        EEPROM.write(addr + i, p[i]);
    }
}

float eepromReadFloat(int addr)
{
    float value = 0;
    uint8_t *p = (uint8_t *)&value;
    for (int i = 0; i < 4; i++)
    {
        p[i] = EEPROM.read(addr + i);
    }
    return value;
}

void eepromWriteUint32(int addr, uint32_t value)
{
    uint8_t *p = (uint8_t *)&value;
    for (int i = 0; i < 4; i++)
    {
        EEPROM.write(addr + i, p[i]);
    }
}

uint32_t eepromReadUint32(int addr)
{
    uint32_t value = 0;
    uint8_t *p = (uint8_t *)&value;
    for (int i = 0; i < 4; i++)
    {
        p[i] = EEPROM.read(addr + i);
    }
    return value;
}

/* ========== EEPROM WRITE THROTTLING ========== */

void markEepromDirty()
{
    eepromDirty = true;
}

void saveEepromIfNeeded()
{
    if (!eepromDirty)
    {
        return;
    }

    unsigned long now = millis();
    if (now - lastEepromSave < EEPROM_SAVE_INTERVAL)
    {
        return;
    }

    saveStateToEeprom();
    lastEepromSave = now;
    eepromDirty = false;
}

void forceSaveEeprom()
{
    saveStateToEeprom();
    lastEepromSave = millis();
    eepromDirty = false;
}

/* ================================================================== */
/* ========== BEST TIMESTAMP ======================================== */
/* ================================================================== */

uint32_t getBestTimestamp()
{
    if (rtcTimeValid && currentUnixTime > 0)
    {
        return currentUnixTime;
    }
    else
    {
        return (uint32_t)(millis() / 1000UL);
    }
}

/* ================================================================== */
/* ========== EEPROM STATE SAVE / RESTORE =========================== */
/* ================================================================== */

void saveStateToEeprom()
{
    uint32_t ts = getBestTimestamp();
    if (ts == 0)
    {
        return;
    }

    EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
    EEPROM.write(EEPROM_RESP_LEVEL_ADDR, (uint8_t)currentResponseLevel);

    for (int i = 0; i < SUSTAINED_BUF_SIZE; i++)
    {
        eepromWriteFloat(EEPROM_SBUF0_ADDR + (i * 4), sustainedBuffer[i]);
    }

    eepromWriteUint32(EEPROM_SAVE_TIME_ADDR, ts);
    eepromWriteFloat(EEPROM_ALERT_ADDR, alertLevelCm);
    eepromWriteFloat(EEPROM_WARNING_ADDR, warningLevelCm);
    eepromWriteFloat(EEPROM_DANGER_ADDR, dangerLevelCm);
    eepromWriteFloat(EEPROM_OLP_ADDR, olpLength);
    eepromWriteFloat(EEPROM_PEAK_ADDR, peakHeight);
    eepromWriteUint32(EEPROM_PEAK_TIME_ADDR, peakTime);
    eepromWriteFloat(EEPROM_PREV_HEIGHT_ADDR, prevWaterHeight);
    eepromWriteUint32(EEPROM_STATE_ENTRY_ADDR, stateEntryTime);

    EEPROM.commit();
}

boolean restoreStateFromEeprom()
{
    uint8_t magic = EEPROM.read(EEPROM_MAGIC_ADDR);
    if (magic != EEPROM_MAGIC_VALUE)
    {
        return false;
    }

    uint32_t savedTime = eepromReadUint32(EEPROM_SAVE_TIME_ADDR);
    uint32_t ts = getBestTimestamp();

    if (ts > 0 && savedTime > 0)
    {
        uint32_t age = 0;
        if (ts > savedTime)
        {
            age = ts - savedTime;
        }
        if (age > EEPROM_STATE_MAX_AGE)
        {
            Serial.println("STATUS:EEPROM_STATE_TOO_OLD");
            return false;
        }
    }

    currentResponseLevel = (int)EEPROM.read(EEPROM_RESP_LEVEL_ADDR);
    if (currentResponseLevel < RESP_NORMAL || currentResponseLevel > RESP_CRITICAL)
    {
        currentResponseLevel = RESP_NORMAL;
    }
    previousResponseLevel = currentResponseLevel;

    boolean bufferValid = true;
    for (int i = 0; i < SUSTAINED_BUF_SIZE; i++)
    {
        float val = eepromReadFloat(EEPROM_SBUF0_ADDR + (i * 4));

        if (isnan(val) || isinf(val) || val < 0 || val > 10000)
        {
            bufferValid = false;
            break;
        }
        sustainedBuffer[i] = val;
    }

    if (bufferValid)
    {
        sustainedBufCount = 1;
        sustainedBufIdx = 1;
        sustainedRise = false;
    }
    else
    {
        sustainedBufCount = 0;
        sustainedBufIdx = 0;
        Serial.println("WARNING:EEPROM_SUSTAINED_BUFFER_CORRUPTED");
    }

    float savedAlert = eepromReadFloat(EEPROM_ALERT_ADDR);
    float savedWarning = eepromReadFloat(EEPROM_WARNING_ADDR);
    float savedDanger = eepromReadFloat(EEPROM_DANGER_ADDR);

    if (savedAlert > 0 && savedAlert < 10000 &&
            savedWarning > savedAlert && savedDanger > savedWarning)
    {
        alertLevelCm = savedAlert;
        warningLevelCm = savedWarning;
        dangerLevelCm = savedDanger;
    }

    float savedOlp = eepromReadFloat(EEPROM_OLP_ADDR);
    if (savedOlp > 0 && savedOlp < 10000)
    {
        olpLength = savedOlp;
    }

    float savedPeak = eepromReadFloat(EEPROM_PEAK_ADDR);
    uint32_t savedPeakTime = eepromReadUint32(EEPROM_PEAK_TIME_ADDR);
    if (savedPeak > 0 && savedPeak < 10000)
    {
        peakHeight = savedPeak;
        peakTime = savedPeakTime;
    }

    float savedPrevHeight = eepromReadFloat(EEPROM_PREV_HEIGHT_ADDR);
    if (!isnan(savedPrevHeight) && !isinf(savedPrevHeight) &&
            savedPrevHeight > 0 && savedPrevHeight < 10000)
    {
        prevWaterHeight = savedPrevHeight;
    }

    uint32_t savedStateEntry = eepromReadUint32(EEPROM_STATE_ENTRY_ADDR);
    if (savedStateEntry > 0)
    {
        stateEntryTime = savedStateEntry;
    }

    Serial.print("STATUS:WARM_BOOT_RESTORED_LEVEL=");
    Serial.println(currentResponseLevel);
    Serial.print("STATUS:WARM_BOOT_SAVED_TIME=");
    Serial.println(savedTime);

    return true;
}

/* ================================================================== */
/* ========== CONTACT PERSISTENCE =================================== */
/* ================================================================== */

void saveContactsToEeprom()
{
    int addr = EEPROM_CONTACTS_BASE;

    EEPROM.write(addr++, (uint8_t)tier1Count);
    for (int i = 0; i < MAX_CONTACTS_PER_TIER; i++)
    {
        for (int j = 0; j < PHONE_NUMBER_LENGTH; j++)
        {
            EEPROM.write(addr++, (uint8_t)tier1Contacts[i][j]);
        }
    }

    EEPROM.write(addr++, (uint8_t)tier2Count);
    for (int i = 0; i < MAX_CONTACTS_PER_TIER; i++)
    {
        for (int j = 0; j < PHONE_NUMBER_LENGTH; j++)
        {
            EEPROM.write(addr++, (uint8_t)tier2Contacts[i][j]);
        }
    }

    EEPROM.write(addr++, (uint8_t)tier3Count);
    for (int i = 0; i < MAX_CONTACTS_PER_TIER; i++)
    {
        for (int j = 0; j < PHONE_NUMBER_LENGTH; j++)
        {
            EEPROM.write(addr++, (uint8_t)tier3Contacts[i][j]);
        }
    }

    EEPROM.write(addr++, (uint8_t)authorizedCount);
    for (int i = 0; i < MAX_AUTHORIZED; i++)
    {
        for (int j = 0; j < PHONE_NUMBER_LENGTH; j++)
        {
            EEPROM.write(addr++, (uint8_t)authorizedNumbers[i][j]);
        }
    }

    EEPROM.write(EEPROM_CONTACT_MAGIC_ADDR, EEPROM_CONTACT_MAGIC);
    EEPROM.commit();

    Serial.println("STATUS:CONTACTS_SAVED_TO_EEPROM");
}

boolean restoreContactsFromEeprom()
{
    if (EEPROM.read(EEPROM_CONTACT_MAGIC_ADDR) != EEPROM_CONTACT_MAGIC)
    {
        return false;
    }

    int addr = EEPROM_CONTACTS_BASE;

    tier1Count = EEPROM.read(addr++);
    if (tier1Count > MAX_CONTACTS_PER_TIER)
    {
        tier1Count = 0;
    }
    for (int i = 0; i < MAX_CONTACTS_PER_TIER; i++)
    {
        for (int j = 0; j < PHONE_NUMBER_LENGTH; j++)
        {
            tier1Contacts[i][j] = (char)EEPROM.read(addr++);
        }
    }

    tier2Count = EEPROM.read(addr++);
    if (tier2Count > MAX_CONTACTS_PER_TIER)
    {
        tier2Count = 0;
    }
    for (int i = 0; i < MAX_CONTACTS_PER_TIER; i++)
    {
        for (int j = 0; j < PHONE_NUMBER_LENGTH; j++)
        {
            tier2Contacts[i][j] = (char)EEPROM.read(addr++);
        }
    }

    tier3Count = EEPROM.read(addr++);
    if (tier3Count > MAX_CONTACTS_PER_TIER)
    {
        tier3Count = 0;
    }
    for (int i = 0; i < MAX_CONTACTS_PER_TIER; i++)
    {
        for (int j = 0; j < PHONE_NUMBER_LENGTH; j++)
        {
            tier3Contacts[i][j] = (char)EEPROM.read(addr++);
        }
    }

    authorizedCount = EEPROM.read(addr++);
    if (authorizedCount > MAX_AUTHORIZED)
    {
        authorizedCount = 0;
    }
    for (int i = 0; i < MAX_AUTHORIZED; i++)
    {
        for (int j = 0; j < PHONE_NUMBER_LENGTH; j++)
        {
            authorizedNumbers[i][j] = (char)EEPROM.read(addr++);
        }
    }

    Serial.print("STATUS:CONTACTS_RESTORED T1=");
    Serial.print(tier1Count);
    Serial.print(" T2=");
    Serial.print(tier2Count);
    Serial.print(" T3=");
    Serial.print(tier3Count);
    Serial.print(" AUTH=");
    Serial.println(authorizedCount);

    return true;
}

/* ================================================================== */
/* ========== ALGORITHM TOGGLE SYSTEM FUNCTIONS ==================== */
/* ================================================================== */
/*
 * Manages the physical button on ALGO_BUTTON_PIN that toggles
 * the flood detection algorithm on and off.
 *
 * When algorithm is disabled:
 *   - ALGO_STATUS_LED_PIN is driven HIGH (LED ON)
 *   - sampleIntervalMs is ignored; sensors read every loop
 *   - evaluateFloodStatus() is NOT called
 *   - updateAdaptiveIntervals() is NOT called
 *   - No light-sleep is entered
 *   - Data logging and serial output continue normally
 *   - Obstruction lights continue to flash
 *   - GPS, RTC, SIM800L housekeeping continue
 *
 * When algorithm is enabled (default):
 *   - ALGO_STATUS_LED_PIN is driven LOW (LED OFF)
 *   - Normal adaptive sampling and flood evaluation
 */

void algoToggleInit()
{
    /* Configure algorithm toggle button with internal pull-up */
    pinMode(ALGO_BUTTON_PIN, INPUT_PULLUP);

    /* Configure algorithm status LED as output */
    pinMode(ALGO_STATUS_LED_PIN, OUTPUT);
    
    /* Algorithm starts DISABLED by default for continuous data mode */
    /* This ensures Processing IDE gets data immediately on connect */
    algorithmEnabled = false;
    
    /* LED ON = Algorithm DISABLED (continuous mode active) */
    digitalWrite(ALGO_STATUS_LED_PIN, HIGH);
    
    algoButtonLastState = HIGH;
    algoButtonLastPress = 0;

    Serial.println("STATUS:ALGO_TOGGLE_INIT_OK");
    Serial.print("STATUS:ALGO_BUTTON_PIN=");
    Serial.println(ALGO_BUTTON_PIN);
    Serial.print("STATUS:ALGO_STATUS_LED_PIN=");
    Serial.println(ALGO_STATUS_LED_PIN);
    Serial.println("STATUS:ALGO_STATE=DISABLED");
    Serial.println("STATUS:CONTINUOUS_DATA_MODE_ACTIVE");
    Serial.println("STATUS:ALGO_STATUS_LED=ON");
}
void algoCheckButton(unsigned long now)
{
    boolean currentButtonState = digitalRead(ALGO_BUTTON_PIN);

    /* Detect falling edge (button press, active LOW with pull-up) */
    if (algoButtonLastState == HIGH && currentButtonState == LOW)
    {
        if (now - algoButtonLastPress >= ALGO_DEBOUNCE_MS)
        {
            algoButtonLastPress = now;

            /* Toggle algorithm state */
            boolean newState = !algorithmEnabled;
            setAlgorithmEnabled(newState);
        }
    }

    algoButtonLastState = currentButtonState;
}

void setAlgorithmEnabled(boolean enabled)
{
    boolean previousState = algorithmEnabled;
    algorithmEnabled = enabled;

    if (algorithmEnabled)
    {
        /* Algorithm ENABLED — turn OFF status LED */
        digitalWrite(ALGO_STATUS_LED_PIN, LOW);

        Serial.println("STATUS:ALGORITHM_ENABLED");
        Serial.println("STATUS:ADAPTIVE_SAMPLING_ACTIVE");
        Serial.println("STATUS:FLOOD_EVALUATION_ACTIVE");
        Serial.println("STATUS:SLEEP_MODE_AVAILABLE");

        debugSendStatusEvent("ALGORITHM_ENABLED");

        /* Reset the sample timer so next sample happens at the */
        /* adaptive interval from now, not immediately */
        lastFloodSampleTime = millis();
        lastTransmitTime = millis();

        /* If transitioning from disabled to enabled, update */
        /* adaptive intervals based on current state */
        if (!previousState)
        {
            updateAdaptiveIntervals();
        }
    }
    else
    {
        /* Algorithm DISABLED — turn ON status LED */
        digitalWrite(ALGO_STATUS_LED_PIN, HIGH);

        Serial.println("STATUS:ALGORITHM_DISABLED");
        Serial.println("STATUS:CONTINUOUS_DATA_MODE_ACTIVE");
        Serial.println("STATUS:NO_FLOOD_EVALUATION");
        Serial.println("STATUS:NO_SLEEP_MODE");
        Serial.println("STATUS:ALGO_STATUS_LED=ON");

        debugSendStatusEvent("ALGORITHM_DISABLED");
    }
}

/* ================================================================== */
/* ========== OBSTRUCTION LIGHT SYSTEM FUNCTIONS ==================== */
/* ================================================================== */

void obLightInit()
{
    for (int i = 0; i < OB_LED_COUNT; i++)
    {
        pinMode(obLedPins[i], OUTPUT);
        digitalWrite(obLedPins[i], LOW);
    }

    obLedsCurrentlyOn = false;
    obLastToggleTime = millis();
    obLightEnabled = true;

    boolean restored = obLightRestoreFromEeprom();
    if (restored)
    {
        char onStr[10], offStr[10];
        floatToStr(onStr, sizeof(onStr), (float)obOnTimeMs, 0);
        floatToStr(offStr, sizeof(offStr), (float)obOffTimeMs, 0);
        Serial.print("STATUS:OBLIGHT_RESTORED_ON=");
        Serial.print(onStr);
        Serial.print("ms_OFF=");
        Serial.print(offStr);
        Serial.println("ms");
    }
    else
    {
        obOnTimeMs = OB_DEFAULT_ON_MS;
        obOffTimeMs = OB_DEFAULT_OFF_MS;
        strncpy(currentObPresetName, "IALA_FL4S", OB_PRESET_NAME_LEN - 1);
        currentObPresetName[OB_PRESET_NAME_LEN - 1] = '\0';
        Serial.println("STATUS:OBLIGHT_DEFAULTS_LOADED");
    }

    unsigned long totalPeriod = obOnTimeMs + obOffTimeMs;
    float dutyCycle = ((float)obOnTimeMs / (float)totalPeriod) * 100.0;
    char dcStr[10];
    floatToStr(dcStr, sizeof(dcStr), dutyCycle, 1);
    Serial.print("STATUS:OBLIGHT_PERIOD=");
    Serial.print(totalPeriod);
    Serial.print("ms_DUTY=");
    Serial.print(dcStr);
    Serial.print("%_PRESET=");
    Serial.println(currentObPresetName);

    Serial.println("STATUS:OBLIGHT_INIT_OK");
}

void obLightSetAll(boolean state)
{
    for (int i = 0; i < OB_LED_COUNT; i++)
    {
        digitalWrite(obLedPins[i], state ? HIGH : LOW);
    }
    obLedsCurrentlyOn = state;
}

void obLightUpdate(unsigned long now)
{
    if (!obLightEnabled)
    {
        if (obLedsCurrentlyOn)
        {
            obLightSetAll(false);
        }
        return;
    }

    unsigned long elapsed = now - obLastToggleTime;

    if (obLedsCurrentlyOn)
    {
        if (elapsed >= obOnTimeMs)
        {
            obLightSetAll(false);
            obLastToggleTime = now;
        }
    }
    else
    {
        if (elapsed >= obOffTimeMs)
        {
            obLightSetAll(true);
            obLastToggleTime = now;
        }
    }
}

void obLightSaveToEeprom()
{
    eepromWriteUint32(EEPROM_OB_ON_ADDR, (uint32_t)obOnTimeMs);
    eepromWriteUint32(EEPROM_OB_OFF_ADDR, (uint32_t)obOffTimeMs);

    /* Save preset name (up to 15 chars + null) */
    int presetAddr = EEPROM_OB_PRESET_ADDR;
    for (int i = 0; i < OB_PRESET_NAME_LEN; i++)
    {
        EEPROM.write(presetAddr + i, (uint8_t)currentObPresetName[i]);
    }

    EEPROM.write(EEPROM_OB_MAGIC_ADDR, EEPROM_OB_MAGIC_VAL);
    EEPROM.commit();

    Serial.println("STATUS:OBLIGHT_TIMING_SAVED");
}

boolean obLightRestoreFromEeprom()
{
    uint8_t magic = EEPROM.read(EEPROM_OB_MAGIC_ADDR);
    if (magic != EEPROM_OB_MAGIC_VAL)
    {
        return false;
    }

    uint32_t savedOn = eepromReadUint32(EEPROM_OB_ON_ADDR);
    uint32_t savedOff = eepromReadUint32(EEPROM_OB_OFF_ADDR);

    if (savedOn >= OB_MIN_ON_MS && savedOn <= OB_MAX_ON_MS &&
            savedOff >= OB_MIN_OFF_MS && savedOff <= OB_MAX_OFF_MS)
    {
        obOnTimeMs = (unsigned long)savedOn;
        obOffTimeMs = (unsigned long)savedOff;

        /* Restore preset name */
        int presetAddr = EEPROM_OB_PRESET_ADDR;
        for (int i = 0; i < OB_PRESET_NAME_LEN; i++)
        {
            currentObPresetName[i] = (char)EEPROM.read(presetAddr + i);
        }
        currentObPresetName[OB_PRESET_NAME_LEN - 1] = '\0';

        /* Validate restored preset name is printable ASCII */
        boolean nameValid = true;
        for (int i = 0; i < OB_PRESET_NAME_LEN && currentObPresetName[i] != '\0'; i++)
        {
            if (currentObPresetName[i] < 32 || currentObPresetName[i] > 126)
            {
                nameValid = false;
                break;
            }
        }
        if (!nameValid || strlen(currentObPresetName) == 0)
        {
            strncpy(currentObPresetName, "CUSTOM", OB_PRESET_NAME_LEN - 1);
            currentObPresetName[OB_PRESET_NAME_LEN - 1] = '\0';
        }

        return true;
    }

    return false;
}

/* ---- Preset Lookup by Name ---- */

int obLightFindPreset(const char *presetName)
{
    for (int i = 0; i < OB_PRESET_COUNT; i++)
    {
        if (strcmp(presetName, obPresets[i].name) == 0)
        {
            return i;
        }
    }
    return -1;
}

/* ---- Apply Preset by Name ---- */

boolean obLightApplyPreset(const char *presetName)
{
    int idx = obLightFindPreset(presetName);
    if (idx < 0)
    {
        Serial.print("ERROR:OBLIGHT_UNKNOWN_PRESET=");
        Serial.println(presetName);
        return false;
    }

    obOnTimeMs = obPresets[idx].onMs;
    obOffTimeMs = obPresets[idx].offMs;
    strncpy(currentObPresetName, obPresets[idx].name, OB_PRESET_NAME_LEN - 1);
    currentObPresetName[OB_PRESET_NAME_LEN - 1] = '\0';

    /* Reset toggle timer so new timing takes effect cleanly */
    obLastToggleTime = millis();
    obLedsCurrentlyOn = false;
    obLightSetAll(false);

    /* Save to EEPROM for persistence across reboots */
    obLightSaveToEeprom();

    unsigned long totalPeriod = obOnTimeMs + obOffTimeMs;
    float dutyCycle = ((float)obOnTimeMs / (float)totalPeriod) * 100.0;
    char dcStr[10];
    floatToStr(dcStr, sizeof(dcStr), dutyCycle, 1);

    Serial.print("STATUS:OBLIGHT_PRESET_APPLIED=");
    Serial.println(obPresets[idx].name);
    Serial.print("STATUS:OBLIGHT_DESC=");
    Serial.println(obPresets[idx].description);
    Serial.print("STATUS:OBLIGHT_ON=");
    Serial.print(obOnTimeMs);
    Serial.print("ms_OFF=");
    Serial.print(obOffTimeMs);
    Serial.print("ms_PERIOD=");
    Serial.print(totalPeriod);
    Serial.print("ms_DUTY=");
    Serial.print(dcStr);
    Serial.println("%");

    debugSendStatusEvent("OBLIGHT_PRESET_APPLIED");

    return true;
}

/* ---- List All Available Presets ---- */

void obLightListPresets()
{
    Serial.println("PRESETS:START");
    Serial.print("PRESETS:COUNT=");
    Serial.println(OB_PRESET_COUNT);
    Serial.print("PRESETS:CURRENT=");
    Serial.println(currentObPresetName);

    for (int i = 0; i < OB_PRESET_COUNT; i++)
    {
        unsigned long totalPeriod = obPresets[i].onMs + obPresets[i].offMs;
        float dutyCycle = ((float)obPresets[i].onMs / (float)totalPeriod) * 100.0;
        char dcStr[10];
        floatToStr(dcStr, sizeof(dcStr), dutyCycle, 1);

        Serial.print("PRESETS:");
        Serial.print(i);
        Serial.print(",");
        Serial.print(obPresets[i].name);
        Serial.print(",");
        Serial.print(obPresets[i].onMs);
        Serial.print(",");
        Serial.print(obPresets[i].offMs);
        Serial.print(",");
        Serial.print(totalPeriod);
        Serial.print(",");
        Serial.print(dcStr);
        Serial.print(",");
        Serial.println(obPresets[i].description);
    }

    Serial.println("PRESETS:END");
}

/* ================================================================== */
/* ========== DEBUG SYSTEM FUNCTIONS ================================ */
/* ================================================================== */

/*
 * Software UART TX implementation for debug output.
 * Uses bit-banging on DEBUG_UART_TX_PIN at DEBUG_SW_BAUD.
 * This avoids consuming a hardware UART for debug-only output.
 * TX-only, no receive needed from the debug circuit.
 *
 * At 115200 baud, each bit is ~8.68 microseconds.
 */

void debugSwTxInit()
{
    pinMode(DEBUG_UART_TX_PIN, OUTPUT);
    digitalWrite(DEBUG_UART_TX_PIN, HIGH);
    debugSwTxInitialized = true;
}

void debugSwTxByte(uint8_t b)
{
    if (!debugSwTxInitialized)
    {
        return;
    }

    /* Bit period for 115200 baud = 8.68 microseconds */
    const uint32_t bitDelayUs = 9;

    /* Disable interrupts for timing accuracy */
    portDISABLE_INTERRUPTS();

    /* Start bit (LOW) */
    digitalWrite(DEBUG_UART_TX_PIN, LOW);
    delayMicroseconds(bitDelayUs);

    /* 8 data bits, LSB first */
    for (int i = 0; i < 8; i++)
    {
        if (b & (1 << i))
        {
            digitalWrite(DEBUG_UART_TX_PIN, HIGH);
        }
        else
        {
            digitalWrite(DEBUG_UART_TX_PIN, LOW);
        }
        delayMicroseconds(bitDelayUs);
    }

    /* Stop bit (HIGH) */
    digitalWrite(DEBUG_UART_TX_PIN, HIGH);
    delayMicroseconds(bitDelayUs);

    /* Re-enable interrupts */
    portENABLE_INTERRUPTS();
}

void debugSerialWrite(const char *data, int len)
{
    if (!debugEnabled)
    {
        return;
    }
    if (!debugSwTxInitialized)
    {
        return;
    }

    for (int i = 0; i < len; i++)
    {
        debugSwTxByte((uint8_t)data[i]);
    }

    debugBytesSent += (uint32_t)len;
}

void debugSerialPrint(const char *str)
{
    if (!debugEnabled)
    {
        return;
    }
    int len = strlen(str);
    debugSerialWrite(str, len);
}

void debugSerialPrintln(const char *str)
{
    if (!debugEnabled)
    {
        return;
    }
    debugSerialPrint(str);
    debugSerialWrite("\r\n", 2);
}

void debugInit()
{
    /* Initialize debug button with internal pull-up */
    pinMode(DEBUG_BUTTON_PIN, INPUT_PULLUP);

    /* Initialize software TX pin */
    debugSwTxInit();

    /* Debug starts DISABLED by default to avoid any overhead */
    debugEnabled = false;
    debugButtonLastState = HIGH;
    debugButtonLastPress = 0;
    debugLastOutputTime = 0;
    debugPacketsSent = 0;
    debugBytesSent = 0;

    Serial.println("STATUS:DEBUG_SYSTEM_INIT_OK");
    Serial.print("STATUS:DEBUG_BUTTON_PIN=");
    Serial.println(DEBUG_BUTTON_PIN);
    Serial.print("STATUS:DEBUG_TX_PIN=");
    Serial.println(DEBUG_UART_TX_PIN);
    Serial.println("STATUS:DEBUG_STATE=DISABLED");
}

void debugCheckButton(unsigned long now)
{
    boolean currentButtonState = digitalRead(DEBUG_BUTTON_PIN);

    /* Detect falling edge (button press, active LOW with pull-up) */
    if (debugButtonLastState == HIGH && currentButtonState == LOW)
    {
        if (now - debugButtonLastPress >= DEBUG_DEBOUNCE_MS)
        {
            debugButtonLastPress = now;

            /* Toggle debug state */
            debugEnabled = !debugEnabled;

            if (debugEnabled)
            {
                Serial.println("STATUS:DEBUG_ENABLED");
                /* Send initial handshake to debug circuit */
                debugSerialPrintln("DBG:HANDSHAKE:VARUNA_DEBUG_V1");
                debugSerialPrintln("DBG:STATUS:DEBUG_ENABLED");
            }
            else
            {
                /* Send disable notification before turning off */
                debugSerialPrintln("DBG:STATUS:DEBUG_DISABLED");
                Serial.println("STATUS:DEBUG_DISABLED");
            }
        }
    }

    debugButtonLastState = currentButtonState;
}

void debugSendStatusEvent(const char *eventStr)
{
    if (!debugEnabled)
    {
        return;
    }

    char buf[128];
    snprintf(buf, sizeof(buf), "DBG:EVENT:%s", eventStr);
    debugSerialPrintln(buf);
}

void debugSendCalibrationStatus(const char *phase, int progress, int total)
{
    if (!debugEnabled)
    {
        return;
    }

    char buf[80];
    snprintf(buf, sizeof(buf), "DBG:CAL:%s:%d/%d", phase, progress, total);
    debugSerialPrintln(buf);
}

void debugSendSensorInit(const char *sensorName, boolean success)
{
    if (!debugEnabled)
    {
        return;
    }

    char buf[64];
    snprintf(buf, sizeof(buf), "DBG:SENSOR:%s:%s",
             sensorName, success ? "OK" : "FAIL");
    debugSerialPrintln(buf);
}

void debugSendFloodTransition(int fromLevel, int toLevel, float waterHeight)
{
    if (!debugEnabled)
    {
        return;
    }

    char hStr[10];
    floatToStr(hStr, sizeof(hStr), waterHeight, 1);

    char buf[80];
    snprintf(buf, sizeof(buf), "DBG:FLOOD_TRANS:%s->%s:H=%s",
             responseNameStr(fromLevel),
             responseNameStr(toLevel),
             hStr);
    debugSerialPrintln(buf);
}

void debugSendAlertDispatch(int level, int smsCount)
{
    if (!debugEnabled)
    {
        return;
    }

    char buf[64];
    snprintf(buf, sizeof(buf), "DBG:ALERT_DISPATCH:%s:SMS=%d",
             responseNameStr(level), smsCount);
    debugSerialPrintln(buf);
}

void debugSendGpsStatus()
{
    if (!debugEnabled)
    {
        return;
    }

    char latStr[12], lonStr[12];
    floatToStr(latStr, sizeof(latStr), gpsLat, 6);
    floatToStr(lonStr, sizeof(lonStr), gpsLon, 6);

    char buf[80];
    snprintf(buf, sizeof(buf), "DBG:GPS:%s:SAT=%d:HDOP=%d:LAT=%s:LON=%s",
             gpsFixValid ? "FIX" : "NOFIX",
             gpsSatellites,
             (int)(gpsHdop * 10),
             latStr, lonStr);
    debugSerialPrintln(buf);
}

void debugSendSimStatus()
{
    if (!debugEnabled)
    {
        return;
    }

    char buf[80];
    snprintf(buf, sizeof(buf), "DBG:SIM:AVAIL=%d:REG=%d:RSSI=%d:QUEUE=%d",
             simAvailable ? 1 : 0,
             simRegistered ? 1 : 0,
             simSignalRSSI,
             smsQueueCount);
    debugSerialPrintln(buf);
}

void debugSendBatteryStatus()
{
    if (!debugEnabled)
    {
        return;
    }

    char bStr[10];
    floatToStr(bStr, sizeof(bStr), batteryPercent, 1);

    char buf[40];
    snprintf(buf, sizeof(buf), "DBG:BATT:%s%%", bStr);
    debugSerialPrintln(buf);
}

void debugSendPeriodicPacket(unsigned long now, float theta, float waterHeight,
                             float correctedTiltX, float correctedTiltY,
                             float horizontalDist)
{
    if (!debugEnabled)
    {
        return;
    }

    if (now - debugLastOutputTime < DEBUG_OUTPUT_INTERVAL_MS)
    {
        return;
    }
    debugLastOutputTime = now;

    char thetaStr[10], whStr[10], txStr[10], tyStr[10];
    char hdStr[10], olpStr[10];
    char rateStr[10], pkStr[10], mnStr2[10];
    char cpStr[10], bpStr[10], pdStr[10], edStr[10];
    char batStr[10];
    char hdopStr[10];

    floatToStr(thetaStr, sizeof(thetaStr), theta, 2);
    floatToStr(whStr, sizeof(whStr), waterHeight, 2);
    floatToStr(txStr, sizeof(txStr), correctedTiltX, 2);
    floatToStr(tyStr, sizeof(tyStr), correctedTiltY, 2);
    floatToStr(hdStr, sizeof(hdStr), horizontalDist, 2);
    floatToStr(olpStr, sizeof(olpStr), olpLength, 1);
    floatToStr(rateStr, sizeof(rateStr), ratePer15Min, 3);
    floatToStr(pkStr, sizeof(pkStr), peakHeight, 2);

    float dispMin = minHeight > 9000 ? 0.0 : minHeight;
    floatToStr(mnStr2, sizeof(mnStr2), dispMin, 2);

    floatToStr(cpStr, sizeof(cpStr), currentPressure, 2);
    floatToStr(bpStr, sizeof(bpStr), baselinePressure, 2);
    floatToStr(pdStr, sizeof(pdStr), pressureDeviation, 2);
    floatToStr(edStr, sizeof(edStr), estimatedDepth, 2);
    floatToStr(batStr, sizeof(batStr), batteryPercent, 1);
    floatToStr(hdopStr, sizeof(hdopStr), gpsHdop, 1);

    debugPacketsSent++;

    /* Build and send each line individually to avoid huge buffer */
    char line[128];

    snprintf(line, sizeof(line), "DBG:PKT_START:%lu", (unsigned long)debugPacketsSent);
    debugSerialPrintln(line);

    /* Time line */
    snprintf(line, sizeof(line), "DBG:TIME:%lu:%u-%02u-%02u %02u:%02u:%02u",
             (unsigned long)currentUnixTime,
             rtcYear, rtcMonth, rtcDay,
             rtcHours, rtcMinutes, rtcSeconds);
    debugSerialPrintln(line);

    /* Tilt line */
    snprintf(line, sizeof(line), "DBG:TILT:THETA=%s:TX=%s:TY=%s",
             thetaStr, txStr, tyStr);
    debugSerialPrintln(line);

    /* Water level line */
    snprintf(line, sizeof(line), "DBG:WATER:HEIGHT=%s:HDIST=%s:OLP=%s",
             whStr, hdStr, olpStr);
    debugSerialPrintln(line);

    /* Flood status line 1 */
    snprintf(line, sizeof(line), "DBG:FLOOD:ZONE=%s:RESP=%s:RATE=%s:SUST=%s",
             zoneNameStr(currentZone),
             responseNameStr(currentResponseLevel),
             rateStr,
             sustainedRise ? "YES" : "NO");
    debugSerialPrintln(line);

    /* Flood status line 2 */
    snprintf(line, sizeof(line), "DBG:FLOOD:PEAK=%s:MIN=%s:STEPDN=%d",
             pkStr, mnStr2, stepDownConsecutive);
    debugSerialPrintln(line);

    /* Pressure line */
    snprintf(line, sizeof(line), "DBG:PRESSURE:CUR=%s:BASE=%s:DEV=%s:SUB=%d:DEPTH=%s",
             cpStr, bpStr, pdStr, submersionState, edStr);
    debugSerialPrintln(line);

    /* GPS line */
    snprintf(line, sizeof(line), "DBG:GPS:FIX=%d:SAT=%d:HDOP=%s",
             gpsFixValid ? 1 : 0, gpsSatellites, hdopStr);
    debugSerialPrintln(line);

    /* SIM line */
    snprintf(line, sizeof(line), "DBG:SIM:AVAIL=%d:REG=%d:RSSI=%d:QUEUE=%d",
             simAvailable ? 1 : 0,
             simRegistered ? 1 : 0,
             simSignalRSSI,
             smsQueueCount);
    debugSerialPrintln(line);

    /* Battery line */
    snprintf(line, sizeof(line), "DBG:BATT:%s%%", batStr);
    debugSerialPrintln(line);

    /* Timing line */
    snprintf(line, sizeof(line), "DBG:TIMING:SAMPLE=%lus:TX=%lus:READINGS=%d",
             sampleIntervalMs / 1000UL,
             transmitIntervalMs / 1000UL,
             readingsSinceBoot);
    debugSerialPrintln(line);

    /* Sensor line */
    const char *sensorStr = "NONE";
    if (activeSensor == SENSOR_MPU6050)
    {
        sensorStr = "MPU6050";
    }
    else if (activeSensor == SENSOR_HCSR04)
    {
        sensorStr = "HCSR04";
    }
    snprintf(line, sizeof(line), "DBG:SENSOR:%s:MPU=%d:HCSR04=%d",
             sensorStr,
             mpuHealthy ? 1 : 0,
             hcsr04Available ? 1 : 0);
    debugSerialPrintln(line);

    /* Obstruction light line */
    snprintf(line, sizeof(line), "DBG:OBLIGHT:ON=%lums:OFF=%lums:STATE=%s:EN=%d:PRESET=%s",
             obOnTimeMs, obOffTimeMs,
             obLedsCurrentlyOn ? "ON" : "OFF",
             obLightEnabled ? 1 : 0,
             currentObPresetName);
    debugSerialPrintln(line);

    /* Algorithm state line */
    snprintf(line, sizeof(line), "DBG:ALGO:ENABLED=%d:MODE=%s",
             algorithmEnabled ? 1 : 0,
             algorithmEnabled ? "ADAPTIVE" : "CONTINUOUS");
    debugSerialPrintln(line);

    /* Health line */
    snprintf(line, sizeof(line), "DBG:HEALTH:MPU_FAILS=%d:UPTIME=%lus:BMP=%d:RTC=%d",
             mpuConsecutiveFailures,
             (unsigned long)sessionDuration,
             bmpAvailable ? 1 : 0,
             rtcAvailable && rtcTimeValid ? 1 : 0);
    debugSerialPrintln(line);

    /* Packet end marker */
    debugSerialPrintln("DBG:PKT_END");
}

/* ================================================================== */
/* ========== DS1307 RTC FUNCTIONS ================================== */
/* ================================================================== */

boolean rtcInit()
{
    I2C_BUS1.beginTransmission(DS1307_ADDR);
    uint8_t err = I2C_BUS1.endTransmission();
    if (err != 0)
    {
        return false;
    }

    I2C_BUS1.beginTransmission(DS1307_ADDR);
    I2C_BUS1.write(0x02);
    I2C_BUS1.endTransmission(false);
    I2C_BUS1.requestFrom((uint8_t)DS1307_ADDR, (uint8_t)1);
    uint8_t hourReg = I2C_BUS1.read();

    if (hourReg & 0x40)
    {
        Serial.println("STATUS:RTC_CONVERTING_TO_24HR");

        uint8_t hr = bcdToDec(hourReg & 0x1F);
        boolean isPM = (hourReg & 0x20) != 0;

        if (isPM && hr != 12)
        {
            hr += 12;
        }
        if (!isPM && hr == 12)
        {
            hr = 0;
        }

        I2C_BUS1.beginTransmission(DS1307_ADDR);
        I2C_BUS1.write(0x02);
        I2C_BUS1.write(decToBcd(hr) & 0x3F);
        I2C_BUS1.endTransmission();
    }

    I2C_BUS1.beginTransmission(DS1307_ADDR);
    I2C_BUS1.write(0x00);
    I2C_BUS1.endTransmission(false);
    I2C_BUS1.requestFrom((uint8_t)DS1307_ADDR, (uint8_t)1);
    uint8_t reg0 = I2C_BUS1.read();

    if (reg0 & 0x80)
    {
        I2C_BUS1.beginTransmission(DS1307_ADDR);
        I2C_BUS1.write(0x00);
        I2C_BUS1.write(reg0 & 0x7F);
        I2C_BUS1.endTransmission();
    }

    I2C_BUS1.beginTransmission(DS1307_ADDR);
    I2C_BUS1.write(0x07);
    I2C_BUS1.write(0x00);
    I2C_BUS1.endTransmission();

    return true;
}

void rtcReadRaw()
{
    I2C_BUS1.beginTransmission(DS1307_ADDR);
    I2C_BUS1.write(0x00);
    I2C_BUS1.endTransmission(false);
    I2C_BUS1.requestFrom((uint8_t)DS1307_ADDR, (uint8_t)7);

    rtcSeconds = bcdToDec(I2C_BUS1.read() & 0x7F);
    rtcMinutes = bcdToDec(I2C_BUS1.read() & 0x7F);
    rtcHours = bcdToDec(I2C_BUS1.read() & 0x3F);
    rtcDayOfWeek = bcdToDec(I2C_BUS1.read() & 0x07);
    rtcDay = bcdToDec(I2C_BUS1.read() & 0x3F);
    rtcMonth = bcdToDec(I2C_BUS1.read() & 0x1F);
    rtcYear = 2000 + bcdToDec(I2C_BUS1.read());
}

boolean rtcIsTimeValid()
{
    return (rtcYear >= 2024 && rtcMonth >= 1 && rtcMonth <= 12 &&
            rtcDay >= 1 && rtcDay <= 31);
}

uint32_t dateToUnix(uint16_t year, uint8_t month, uint8_t day,
                    uint8_t hour, uint8_t minute, uint8_t second)
{
    uint16_t y = year;
    uint8_t m = month;

    if (m <= 2)
    {
        y -= 1;
        m += 12;
    }

    uint32_t dayCount = 365UL * y + y / 4 - y / 100 + y / 400;
    dayCount += (153UL * (m - 3) + 2) / 5;
    dayCount += day;
    dayCount -= 719469UL;

    uint32_t t = dayCount * 86400UL;
    t += (uint32_t)hour * 3600UL;
    t += (uint32_t)minute * 60UL;
    t += second;

    return t;
}

uint32_t rtcGetUnixTime()
{
    rtcReadRaw();
    if (!rtcIsTimeValid())
    {
        return 0;
    }
    return dateToUnix(rtcYear, rtcMonth, rtcDay, rtcHours, rtcMinutes, rtcSeconds);
}

void unixToDate(uint32_t unixTime, uint16_t *year, uint8_t *month,
                uint8_t *day, uint8_t *hour, uint8_t *minute,
                uint8_t *second, uint8_t *dow)
{
    *second = unixTime % 60;
    unixTime /= 60;
    *minute = unixTime % 60;
    unixTime /= 60;
    *hour = unixTime % 24;
    unixTime /= 24;

    *dow = ((unixTime + 4) % 7) + 1;

    uint32_t days = unixTime;
    uint32_t y = 1970;

    while (1)
    {
        boolean leap = (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0));
        uint32_t daysInYear = leap ? 366 : 365;
        if (days < daysInYear)
        {
            break;
        }
        days -= daysInYear;
        y++;
    }
    *year = y;

    uint8_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    boolean leap = (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0));
    if (leap)
    {
        daysInMonth[1] = 29;
    }

    uint8_t mo = 0;
    while (mo < 12 && days >= daysInMonth[mo])
    {
        days -= daysInMonth[mo];
        mo++;
    }
    *month = mo + 1;
    *day = days + 1;
}

void rtcSetUnixTime(uint32_t unixTime)
{
    uint16_t y;
    uint8_t mo, d, h, mi, s, dow;
    unixToDate(unixTime, &y, &mo, &d, &h, &mi, &s, &dow);

    I2C_BUS1.beginTransmission(DS1307_ADDR);
    I2C_BUS1.write(0x00);
    I2C_BUS1.write(decToBcd(s) & 0x7F);
    I2C_BUS1.write(decToBcd(mi));
    I2C_BUS1.write(decToBcd(h));
    I2C_BUS1.write(decToBcd(dow));
    I2C_BUS1.write(decToBcd(d));
    I2C_BUS1.write(decToBcd(mo));
    I2C_BUS1.write(decToBcd(y - 2000));
    I2C_BUS1.endTransmission();
}

void rtcSetDateTime(uint16_t year, uint8_t month, uint8_t day,
                    uint8_t hour, uint8_t minute, uint8_t second)
{
    uint32_t unix = dateToUnix(year, month, day, hour, minute, second);
    rtcSetUnixTime(unix);
}

/* ================================================================== */
/* ========== NMEA CHECKSUM VALIDATION ============================== */
/* ================================================================== */

boolean validateNmeaChecksum(const char *sentence)
{
    if (sentence[0] != '$')
    {
        return false;
    }

    const char *star = strchr(sentence, '*');
    if (star == NULL)
    {
        return false;
    }

    uint8_t calcChecksum = 0;
    for (const char *p = sentence + 1; p < star; p++)
    {
        calcChecksum ^= (uint8_t)*p;
    }

    uint8_t statedChecksum = (uint8_t)strtol(star + 1, NULL, 16);

    return (calcChecksum == statedChecksum);
}

/* ================================================================== */
/* ========== GPS NMEA PARSING FUNCTIONS ============================ */
/* ================================================================== */

double nmeaToDecimal(const char *coord, char dir)
{
    double raw = atof(coord);
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100.0);
    double decimal = degrees + minutes / 60.0;
    if (dir == 'S' || dir == 'W')
    {
        decimal = -decimal;
    }
    return decimal;
}

void parseGPGGA(char *sentence)
{
    char *fields[15];
    int fieldCount = 0;
    char *ptr = sentence;

    fields[fieldCount++] = ptr;
    while (*ptr && fieldCount < 15)
    {
        if (*ptr == ',')
        {
            *ptr = '\0';
            fields[fieldCount++] = ptr + 1;
        }
        ptr++;
    }

    if (fieldCount < 10)
    {
        return;
    }

    if (strlen(fields[1]) >= 6)
    {
        char hh[3] = {fields[1][0], fields[1][1], 0};
        char mm[3] = {fields[1][2], fields[1][3], 0};
        char ss[3] = {fields[1][4], fields[1][5], 0};
        gpsHour = atoi(hh);
        gpsMin = atoi(mm);
        gpsSec = atoi(ss);
    }

    int fix = atoi(fields[6]);
    gpsSatellites = atoi(fields[7]);

    if (strlen(fields[8]) > 0)
    {
        gpsHdop = atof(fields[8]);
    }

    if (fix >= 1 && strlen(fields[2]) > 0 && strlen(fields[4]) > 0)
    {
        gpsFixValid = true;
        gpsLat = nmeaToDecimal(fields[2], fields[3][0]);
        gpsLon = nmeaToDecimal(fields[4], fields[5][0]);

        if (strlen(fields[9]) > 0)
        {
            gpsAlt = atof(fields[9]);
        }
    }
    else
    {
        gpsFixValid = false;
    }
}

void parseGPRMC(char *sentence)
{
    char *fields[14];
    int fieldCount = 0;
    char *ptr = sentence;

    fields[fieldCount++] = ptr;
    while (*ptr && fieldCount < 14)
    {
        if (*ptr == ',')
        {
            *ptr = '\0';
            fields[fieldCount++] = ptr + 1;
        }
        ptr++;
    }

    if (fieldCount < 10)
    {
        return;
    }

    if (fields[2][0] == 'A')
    {
        if (strlen(fields[3]) > 0 && strlen(fields[5]) > 0)
        {
            gpsFixValid = true;
            gpsLat = nmeaToDecimal(fields[3], fields[4][0]);
            gpsLon = nmeaToDecimal(fields[5], fields[6][0]);
        }

        if (strlen(fields[7]) > 0)
        {
            gpsSpeed = atof(fields[7]) * 1.852;
        }

        if (strlen(fields[9]) >= 6)
        {
            char dd[3] = {fields[9][0], fields[9][1], 0};
            char mm[3] = {fields[9][2], fields[9][3], 0};
            char yy[3] = {fields[9][4], fields[9][5], 0};
            gpsGGADay = atoi(dd);
            gpsGGAMonth = atoi(mm);
            gpsGGAYear = 2000 + atoi(yy);
            gpsTimeValid = true;
        }
    }

    if (strlen(fields[1]) >= 6)
    {
        char hh[3] = {fields[1][0], fields[1][1], 0};
        char mm[3] = {fields[1][2], fields[1][3], 0};
        char ss[3] = {fields[1][4], fields[1][5], 0};
        gpsHour = atoi(hh);
        gpsMin = atoi(mm);
        gpsSec = atoi(ss);
    }
}

void processNMEA(char *sentence)
{
    if (strncmp(sentence, "$GPGGA", 6) == 0 ||
            strncmp(sentence, "$GNGGA", 6) == 0)
    {
        parseGPGGA(sentence);
        gpsAvailable = true;
    }
    else if (strncmp(sentence, "$GPRMC", 6) == 0 ||
             strncmp(sentence, "$GNRMC", 6) == 0)
    {
        parseGPRMC(sentence);
        gpsAvailable = true;
    }
}

void readGPS()
{
    while (GPS_Serial.available())
    {
        char c = GPS_Serial.read();

        if (c == '$')
        {
            nmeaIdx = 0;
            nmeaBuffer[nmeaIdx++] = c;
        }
        else if (c == '\n' || c == '\r')
        {
            if (nmeaIdx > 5)
            {
                nmeaBuffer[nmeaIdx] = '\0';

                if (validateNmeaChecksum(nmeaBuffer))
                {
                    char *star = strchr(nmeaBuffer, '*');
                    if (star)
                    {
                        *star = '\0';
                    }
                    processNMEA(nmeaBuffer);
                }
            }
            nmeaIdx = 0;
        }
        else if (nmeaIdx < 118)
        {
            nmeaBuffer[nmeaIdx++] = c;
        }
    }
}

void syncRTCfromGPS()
{
    if (!gpsTimeValid || !gpsFixValid)
    {
        return;
    }
    if (!rtcAvailable)
    {
        return;
    }
    if (gpsGGAYear < 2024)
    {
        return;
    }

    if (gpsTimeSynced)
    {
        unsigned long now = millis();
        if (now - lastGpsRtcSync < GPS_RTC_SYNC_INTERVAL)
        {
            return;
        }
    }

    if (gpsHdop > 5.0 || gpsSatellites < 4)
    {
        return;
    }

    rtcSetDateTime(gpsGGAYear, gpsGGAMonth, gpsGGADay,
                   gpsHour, gpsMin, gpsSec);
    rtcReadRaw();
    rtcTimeValid = rtcIsTimeValid();
    currentUnixTime = rtcGetUnixTime();
    gpsTimeSynced = true;
    lastGpsRtcSync = millis();

    Serial.print("STATUS:RTC_SYNCED_FROM_GPS=");
    Serial.print(gpsGGAYear);
    Serial.print("-");
    Serial.print(gpsGGAMonth);
    Serial.print("-");
    Serial.print(gpsGGADay);
    Serial.print(" ");
    Serial.print(gpsHour);
    Serial.print(":");
    Serial.print(gpsMin);
    Serial.print(":");
    Serial.println(gpsSec);

    debugSendStatusEvent("RTC_SYNCED_FROM_GPS");
}
/* ================================================================== */

void simFlushInput()
{
    while (SIM_Serial.available())
    {
        SIM_Serial.read();
    }
}

ATResult send_at_command(const char *command, const char *expected,
                         unsigned long timeout_ms)
{
    simFlushInput();

    SIM_Serial.println(command);

    unsigned long startTime = millis();
    int bufIdx = 0;
    memset(simResponseBuffer, 0, SIM_RESPONSE_SIZE);

    while ((millis() - startTime) < timeout_ms)
    {
        esp_task_wdt_reset();

        while (SIM_Serial.available())
        {
            char c = SIM_Serial.read();

            if (bufIdx < SIM_RESPONSE_SIZE - 1)
            {
                simResponseBuffer[bufIdx++] = c;
                simResponseBuffer[bufIdx] = '\0';
            }

            if (strstr(simResponseBuffer, expected) != NULL)
            {
                return AT_SUCCESS;
            }

            if (strstr(simResponseBuffer, "ERROR") != NULL)
            {
                return AT_ERROR;
            }
        }
        delay(10);
    }

    return AT_TIMEOUT;
}

void simHardwareReset()
{
    Serial.println("STATUS:SIM800L_HARDWARE_RESET");
    debugSendStatusEvent("SIM800L_HARDWARE_RESET");

    pinMode(SIM_RST_PIN, OUTPUT);

    digitalWrite(SIM_RST_PIN, LOW);
    delay(200);
    digitalWrite(SIM_RST_PIN, HIGH);

    Serial.println("STATUS:SIM800L_RST_PULSE_DONE");

    delay(2000);  /* Reduced from 5000ms */
    esp_task_wdt_reset();

    Serial.println("STATUS:SIM800L_BOOT_WAIT_DONE");
}

boolean simPowerOn()
{
    Serial.println("STATUS:SIM800L_INIT_START");
    debugSendStatusEvent("SIM800L_INIT_START");

    simHardwareReset();

    boolean atOk = false;

    for (int retry = 0; retry < 5; retry++)
    {
        esp_task_wdt_reset();

        Serial.print("STATUS:SIM800L_AT_ATTEMPT=");
        Serial.println(retry + 1);

        ATResult res = send_at_command("AT", "OK", 2000);

        if (res == AT_SUCCESS)
        {
            atOk = true;
            Serial.println("STATUS:SIM800L_AT_OK");
            break;
        }

        Serial.println("STATUS:SIM800L_AT_NO_RESPONSE_RETRYING");
        delay(2000);
    }

    if (!atOk)
    {
        Serial.println("ERROR:SIM800L_UNRESPONSIVE");
        debugSendStatusEvent("SIM800L_UNRESPONSIVE");
        simInitFailCode = 1;
        return false;
    }

    send_at_command("AT", "OK", 1000);
    send_at_command("AT", "OK", 1000);

    ATResult res = send_at_command("ATE0", "OK", 2000);
    if (res == AT_SUCCESS)
    {
        Serial.println("STATUS:SIM800L_ECHO_OFF_OK");
    }
    else
    {
        Serial.println("WARNING:SIM800L_ECHO_OFF_FAIL");
    }

    res = send_at_command("AT+CMGF=1", "OK", 2000);
    if (res == AT_SUCCESS)
    {
        Serial.println("STATUS:SIM800L_SMS_MODE_OK");
    }
    else
    {
        Serial.println("WARNING:SIM800L_SMS_MODE_FAIL");
    }

    res = send_at_command("AT+CPIN?", "+CPIN: READY", 5000);
    if (res == AT_SUCCESS)
    {
        Serial.println("STATUS:SIM_CARD_READY");
        simReady = true;
    }
    else if (res == AT_TIMEOUT)
    {
        Serial.println("ERROR:SIM_CARD_TIMEOUT");
        simReady = false;
    }
    else
    {
        Serial.println("ERROR:SIM_CARD_NOT_READY");
        Serial.print("STATUS:SIM_RESPONSE=");
        Serial.println(simResponseBuffer);
        simReady = false;
    }

    res = send_at_command("ATI", "OK", 2000);
    if (res == AT_SUCCESS)
    {
        Serial.print("STATUS:SIM800L_INFO=");
        Serial.println(simResponseBuffer);
    }

    debugSendSensorInit("SIM800L", atOk);

    return true;
}

boolean simWaitForNetwork()
{
    Serial.println("STATUS:SIM800L_WAITING_NETWORK");

    for (int attempt = 1; attempt <= 15; attempt++)
    {
        esp_task_wdt_reset();

        ATResult res = send_at_command("AT+CREG?", "+CREG:", 3000);

        if (res == AT_SUCCESS)
        {
            char *cregPos = strstr(simResponseBuffer, "+CREG:");

            if (cregPos != NULL)
            {
                char *commaPos = strchr(cregPos, ',');

                if (commaPos != NULL)
                {
                    int stat = atoi(commaPos + 1);

                    if (stat == 1 || stat == 5)
                    {
                        simRegistered = true;
                        Serial.print("STATUS:SIM800L_NETWORK_OK_ATTEMPT=");
                        Serial.println(attempt);

                        if (stat == 1)
                        {
                            Serial.println("STATUS:SIM800L_REG_HOME");
                        }
                        else
                        {
                            Serial.println("STATUS:SIM800L_REG_ROAMING");
                        }

                        debugSendStatusEvent("SIM800L_NETWORK_OK");
                        return true;
                    }
                    else
                    {
                        Serial.print("STATUS:SIM800L_CREG_STAT=");
                        Serial.print(stat);
                        Serial.print("_ATTEMPT=");
                        Serial.println(attempt);
                    }
                }
            }
        }
        else
        {
            Serial.print("STATUS:SIM800L_CREG_FAIL_ATTEMPT=");
            Serial.println(attempt);
        }

        delay(2000);
    }

    Serial.println("ERROR:SIM800L_NO_NETWORK");
    debugSendStatusEvent("SIM800L_NO_NETWORK");
    simRegistered = false;
    simInitFailCode = 2;
    return false;
}

int simGetSignalStrength()
{
    ATResult res = send_at_command("AT+CSQ", "+CSQ:", 3000);

    if (res == AT_SUCCESS)
    {
        char *csqPos = strstr(simResponseBuffer, "+CSQ:");

        if (csqPos != NULL)
        {
            int rssi = atoi(csqPos + 5);

            if (rssi >= 0 && rssi <= 31)
            {
                simSignalRSSI = rssi;
                return rssi;
            }
            else if (rssi == 99)
            {
                simSignalRSSI = 0;
                return 0;
            }
        }
    }

    return -1;
}

void simFullInit()
{
    Serial.println("STATUS:SIM800L_FULL_INIT_BEGIN");

    simAvailable = false;
    simRegistered = false;
    simReady = false;
    simSignalRSSI = 0;
    simInitFailCode = 0;

    /* Quick WDT reset before long operation */
    esp_task_wdt_reset();

    boolean powered = simPowerOn();

    if (!powered)
    {
        simAvailable = false;
        Serial.println("TX:FAIL:1");
        Serial.println("STATUS:SIM800L_INIT_FAILED_NO_RESPONSE");
        Serial.println("STATUS:SIM800L_CONTINUING_WITHOUT_SIM");
        return;
    }

    simAvailable = true;
    esp_task_wdt_reset();

    if (!simReady)
    {
        Serial.println("WARNING:SIM800L_NO_SIM_CARD_DETECTED");
        Serial.println("STATUS:SIM800L_CONTINUING_WITHOUT_SIM_CARD");
        return;  /* Don't wait for network if no SIM */
    }

    /* Shorter network wait - 5 attempts instead of 15 */
    Serial.println("STATUS:SIM800L_QUICK_NETWORK_CHECK");
    
    for (int attempt = 1; attempt <= 5; attempt++)
    {
        esp_task_wdt_reset();

        ATResult res = send_at_command("AT+CREG?", "+CREG:", 2000);

        if (res == AT_SUCCESS)
        {
            char *cregPos = strstr(simResponseBuffer, "+CREG:");
            if (cregPos != NULL)
            {
                char *commaPos = strchr(cregPos, ',');
                if (commaPos != NULL)
                {
                    int stat = atoi(commaPos + 1);
                    if (stat == 1 || stat == 5)
                    {
                        simRegistered = true;
                        Serial.print("STATUS:SIM800L_NETWORK_OK_ATTEMPT=");
                        Serial.println(attempt);
                        break;
                    }
                }
            }
        }
        delay(1000);
    }

    if (!simRegistered)
    {
        Serial.println("WARNING:SIM800L_NO_NETWORK_YET_WILL_RETRY_LATER");
        /* Don't block - will check periodically */
    }

    esp_task_wdt_reset();

    int rssi = simGetSignalStrength();
    if (rssi >= 0)
    {
        Serial.print("STATUS:SIM800L_SIGNAL_RSSI=");
        Serial.println(rssi);
        Serial.print("SIGNAL:");
        Serial.println(rssi);
    }

    send_at_command("AT+CSCS=\"GSM\"", "OK", 1000);
    send_at_command("AT+CNMI=2,2,0,0,0", "OK", 2000);
    send_at_command("AT+CMGDA=\"DEL ALL\"", "OK", 3000);

    Serial.println("STATUS:SIM800L_FULL_INIT_COMPLETE");
    debugSendStatusEvent("SIM800L_INIT_COMPLETE");
}

void simCheckSignalPeriodic()
{
    if (!simAvailable)
    {
        return;
    }

    int rssi = simGetSignalStrength();

    if (rssi >= 0)
    {
        Serial.print("STATUS:SIM800L_RSSI=");
        Serial.println(rssi);
    }
}

void simCheckRegistrationPeriodic()
{
    if (!simAvailable)
    {
        return;
    }

    ATResult res = send_at_command("AT+CREG?", "+CREG:", 3000);

    if (res == AT_SUCCESS)
    {
        char *cregPos = strstr(simResponseBuffer, "+CREG:");

        if (cregPos != NULL)
        {
            char *commaPos = strchr(cregPos, ',');

            if (commaPos != NULL)
            {
                int stat = atoi(commaPos + 1);
                boolean wasRegistered = simRegistered;

                if (stat == 1 || stat == 5)
                {
                    simRegistered = true;

                    if (!wasRegistered)
                    {
                        Serial.println("STATUS:SIM800L_NETWORK_RECOVERED");
                        debugSendStatusEvent("SIM800L_NETWORK_RECOVERED");
                    }
                }
                else
                {
                    simRegistered = false;

                    if (wasRegistered)
                    {
                        Serial.print("STATUS:SIM800L_NETWORK_LOST_STAT=");
                        Serial.println(stat);
                        debugSendStatusEvent("SIM800L_NETWORK_LOST");
                    }
                }
            }
        }
    }
}

boolean simSendSMS(const char *phoneNumber, const char *message)
{
    if (!simAvailable || !simRegistered)
    {
        Serial.println("STATUS:SMS_FAIL_NOT_READY");
        return false;
    }

    char cmdBuf[40];
    snprintf(cmdBuf, sizeof(cmdBuf), "AT+CMGS=\"%s\"", phoneNumber);

    simFlushInput();
    SIM_Serial.println(cmdBuf);

    unsigned long startWait = millis();
    boolean gotPrompt = false;

    while ((millis() - startWait) < 5000)
    {
        if (SIM_Serial.available())
        {
            char c = SIM_Serial.read();

            if (c == '>')
            {
                gotPrompt = true;
                break;
            }
        }
        delay(10);
    }

    if (!gotPrompt)
    {
        Serial.println("STATUS:SMS_NO_PROMPT");
        SIM_Serial.write(0x1B);
        delay(1000);
        simFlushInput();
        return false;
    }

    SIM_Serial.print(message);
    delay(100);
    SIM_Serial.write(0x1A);

    unsigned long smsStart = millis();
    int respIdx = 0;
    memset(simResponseBuffer, 0, SIM_RESPONSE_SIZE);

    while ((millis() - smsStart) < 30000)
    {
        esp_task_wdt_reset();

        while (SIM_Serial.available())
        {
            char c = SIM_Serial.read();

            if (respIdx < SIM_RESPONSE_SIZE - 1)
            {
                simResponseBuffer[respIdx++] = c;
                simResponseBuffer[respIdx] = '\0';
            }

            if (strstr(simResponseBuffer, "+CMGS:") != NULL)
            {
                Serial.println("STATUS:SMS_SENT_OK");
                return true;
            }

            if (strstr(simResponseBuffer, "ERROR") != NULL)
            {
                Serial.println("STATUS:SMS_SEND_ERROR");
                return false;
            }
        }
        delay(100);
    }

    Serial.println("STATUS:SMS_SEND_TIMEOUT");
    return false;
}

boolean queueSMS(const char *phone, const char *message)
{
    if (smsQueueCount >= SMS_QUEUE_SIZE)
    {
        Serial.println("WARNING:SMS_QUEUE_FULL");
        return false;
    }

    strncpy(smsQueue[smsQueueTail].phone, phone, PHONE_NUMBER_LENGTH - 1);
    smsQueue[smsQueueTail].phone[PHONE_NUMBER_LENGTH - 1] = '\0';
    strncpy(smsQueue[smsQueueTail].message, message, SMS_MSG_SIZE - 1);
    smsQueue[smsQueueTail].message[SMS_MSG_SIZE - 1] = '\0';
    smsQueue[smsQueueTail].pending = true;
    smsQueue[smsQueueTail].retries = 0;

    smsQueueTail = (smsQueueTail + 1) % SMS_QUEUE_SIZE;
    smsQueueCount++;

    return true;
}

void processOneSmsFromQueue()
{
    if (smsQueueCount == 0)
    {
        return;
    }

    SmsQueueEntry *entry = &smsQueue[smsQueueHead];

    if (!entry->pending)
    {
        smsQueueHead = (smsQueueHead + 1) % SMS_QUEUE_SIZE;
        smsQueueCount--;
        return;
    }

    boolean sent = simSendSMS(entry->phone, entry->message);

    if (sent)
    {
        entry->pending = false;
        smsQueueHead = (smsQueueHead + 1) % SMS_QUEUE_SIZE;
        smsQueueCount--;
    }
    else
    {
        entry->retries++;
        if (entry->retries >= 3)
        {
            Serial.print("STATUS:SMS_DROPPED_AFTER_3_RETRIES=");
            Serial.println(entry->phone);
            entry->pending = false;
            smsQueueHead = (smsQueueHead + 1) % SMS_QUEUE_SIZE;
            smsQueueCount--;
        }
    }
}

boolean simMakeVoiceCall(const char *phoneNumber, int ringDurationSec)
{
    if (!simAvailable || !simRegistered)
    {
        Serial.println("STATUS:CALL_FAIL_NOT_READY");
        return false;
    }

    char cmdBuf[40];
    snprintf(cmdBuf, sizeof(cmdBuf), "ATD%s;", phoneNumber);

    ATResult res = send_at_command(cmdBuf, "OK", 10000);
    if (res != AT_SUCCESS)
    {
        Serial.print("STATUS:CALL_DIAL_FAIL=");
        Serial.println(phoneNumber);
        return false;
    }

    Serial.print("STATUS:CALL_RINGING=");
    Serial.println(phoneNumber);

    unsigned long callStart = millis();
    boolean answered = false;

    char callBuf[64];
    int callBufIdx = 0;
    memset(callBuf, 0, sizeof(callBuf));

    while ((millis() - callStart) < (unsigned long)ringDurationSec * 1000UL)
    {
        esp_task_wdt_reset();
        readGPS();

        if (SIM_Serial.available())
        {
            char c = SIM_Serial.read();
            if (callBufIdx < (int)sizeof(callBuf) - 1)
            {
                callBuf[callBufIdx++] = c;
                callBuf[callBufIdx] = '\0';
            }

            if (strstr(callBuf, "CONNECT") != NULL)
            {
                answered = true;
                Serial.println("STATUS:CALL_ANSWERED");
                delay(5000);
                break;
            }

            if (strstr(callBuf, "BUSY") != NULL ||
                    strstr(callBuf, "NO CARRIER") != NULL ||
                    strstr(callBuf, "NO ANSWER") != NULL)
            {
                Serial.println("STATUS:CALL_NOT_ANSWERED");
                break;
            }

            if (c == '\n')
            {
                callBufIdx = 0;
                memset(callBuf, 0, sizeof(callBuf));
            }
        }
        delay(100);
    }

    send_at_command("ATH", "OK", 5000);

    Serial.print("STATUS:CALL_ENDED=");
    Serial.print(phoneNumber);
    Serial.print(" ANSWERED=");
    Serial.println(answered ? "YES" : "NO");

    return answered;
}

void escalateWithVoiceCalls()
{
    Serial.println("STATUS:VOICE_ESCALATION_START");
    debugSendStatusEvent("VOICE_ESCALATION_START");

    for (int i = 0; i < tier3Count; i++)
    {
        if (strlen(tier3Contacts[i]) > 3)
        {
            simMakeVoiceCall(tier3Contacts[i], 30);
            delay(2000);
        }
    }

    for (int i = 0; i < tier2Count; i++)
    {
        if (strlen(tier2Contacts[i]) > 3)
        {
            simMakeVoiceCall(tier2Contacts[i], 30);
            delay(2000);
        }
    }

    Serial.println("STATUS:VOICE_ESCALATION_DONE");
    debugSendStatusEvent("VOICE_ESCALATION_DONE");
}

void resetAckTracking()
{
    ackReceived = false;
    ackWaitStartTime = 0;
    ackEscalationDone = false;
}

void handleAckSms(const char *sender)
{
    ackReceived = true;
    Serial.print("STATUS:ACK_RECEIVED_FROM=");
    Serial.println(sender);
    debugSendStatusEvent("ACK_RECEIVED");

    queueSMS(sender, "VARUNA: Alert acknowledged. Thank you.");
}

void checkAckTimeout()
{
    if (currentResponseLevel < RESP_FLOOD)
    {
        resetAckTracking();
        return;
    }

    if (ackReceived || ackEscalationDone)
    {
        return;
    }

    if (ackWaitStartTime == 0)
    {
        return;
    }

    uint32_t ts = getBestTimestamp();
    if (ts == 0)
    {
        return;
    }

    uint32_t elapsed = 0;
    if (ts > ackWaitStartTime)
    {
        elapsed = ts - ackWaitStartTime;
    }

    if (elapsed >= ACK_TIMEOUT_SEC)
    {
        Serial.println("STATUS:ACK_TIMEOUT_VOICE_ESCALATION");
        debugSendStatusEvent("ACK_TIMEOUT_ESCALATING");
        escalateWithVoiceCalls();
        ackEscalationDone = true;
    }
}

boolean isAuthorizedSender(const char *number)
{
    if (authorizedCount == 0)
    {
        return false;
    }

    for (int i = 0; i < authorizedCount; i++)
    {
        int lenAuth = strlen(authorizedNumbers[i]);
        int lenSender = strlen(number);
        int compareLen = lenAuth < lenSender ? lenAuth : lenSender;
        if (compareLen > 10)
        {
            compareLen = 10;
        }

        if (compareLen >= 10)
        {
            if (strcmp(authorizedNumbers[i] + lenAuth - 10,
                       number + lenSender - 10) == 0)
            {
                return true;
            }
        }
    }
    return false;
}

void parseSmsHeader(char *line)
{
    char *p = strstr(line, "+CMT:");
    if (p == NULL)
    {
        return;
    }
    p += 5;
    while (*p == ' ')
    {
        p++;
    }

    char *firstQuote = strchr(p, '"');
    if (firstQuote == NULL)
    {
        return;
    }

    char *secondQuote = strchr(firstQuote + 1, '"');
    if (secondQuote == NULL)
    {
        return;
    }

    int len = secondQuote - firstQuote - 1;
    if (len <= 0 || len >= PHONE_NUMBER_LENGTH)
    {
        return;
    }

    char candidate[PHONE_NUMBER_LENGTH];
    strncpy(candidate, firstQuote + 1, len);
    candidate[len] = '\0';

    if (candidate[0] != '+' && !isdigit((unsigned char)candidate[0]))
    {
        return;
    }

    memset(smsRxSender, 0, PHONE_NUMBER_LENGTH);
    strncpy(smsRxSender, candidate, PHONE_NUMBER_LENGTH - 1);

    smsRxState = SMS_RX_HEADER_RECEIVED;
    smsRxBodyIdx = 0;
    memset(smsRxBody, 0, sizeof(smsRxBody));
}

void processSmsCommand(const char *sender, const char *body)
{
    Serial.print("STATUS:SMS_RX_FROM=");
    Serial.println(sender);
    Serial.print("STATUS:SMS_RX_BODY=");
    Serial.println(body);

    if (!isAuthorizedSender(sender))
    {
        Serial.println("STATUS:SMS_RX_REJECTED_UNAUTHORIZED");
        return;
    }

    Serial.println("STATUS:SMS_RX_AUTHORIZED_PROCESSING");
    debugSendStatusEvent("SMS_CMD_RECEIVED");

    char upperBody[32];
    int bodyLen = strlen(body);
    if (bodyLen > 31)
    {
        bodyLen = 31;
    }
    for (int i = 0; i < bodyLen; i++)
    {
        upperBody[i] = toupper(body[i]);
    }
    upperBody[bodyLen] = '\0';

    char *trimmed = upperBody;
    while (*trimmed == ' ')
    {
        trimmed++;
    }
    int trimLen = strlen(trimmed);
    while (trimLen > 0 && trimmed[trimLen - 1] == ' ')
    {
        trimmed[--trimLen] = '\0';
    }

    if (strcmp(trimmed, "ACK") == 0 ||
            strcmp(trimmed, "OK") == 0 ||
            strcmp(trimmed, "RECEIVED") == 0)
    {
        handleAckSms(sender);
        return;
    }

    if (strcmp(trimmed, "ALGOON") == 0)
    {
        setAlgorithmEnabled(true);
        queueSMS(sender, "VARUNA: Algorithm ENABLED");
        return;
    }

    if (strcmp(trimmed, "ALGOOFF") == 0)
    {
        setAlgorithmEnabled(false);
        queueSMS(sender, "VARUNA: Algorithm DISABLED - continuous mode");
        return;
    }

    if (strcmp(trimmed, "CAL") == 0)
    {
        recalibrate();
        queueSMS(sender, "VARUNA: Sensors recalibrated");
        return;
    }

    if (strncmp(body, "SETALERT:", 9) == 0 ||
            strncmp(body, "SETWARN:", 8) == 0 ||
            strncmp(body, "SETDANGER:", 10) == 0 ||
            strncmp(body, "SETTHRESH:", 10) == 0 ||
            strncmp(body, "ADDTIER1:", 9) == 0 ||
            strncmp(body, "ADDTIER2:", 9) == 0 ||
            strncmp(body, "ADDTIER3:", 9) == 0 ||
            strcmp(body, "CLEARTIERS") == 0 ||
            strcmp(body, "FLOODSTATUS") == 0 ||
            strcmp(body, "RESETFLOOD") == 0)
    {
        parseFloodCommand(body);

        char confirmMsg[80];
        snprintf(confirmMsg, sizeof(confirmMsg), "VARUNA CMD OK: %.60s", body);
        queueSMS(sender, confirmMsg);
    }
    else if (strncmp(body, "OLP:", 4) == 0)
    {
        float val = atof(body + 4);
        if (val > 0 && val < 10000)
        {
            olpLength = val;
            forceSaveEeprom();

            char confirmMsg[60];
            char valStr[10];
            floatToStr(valStr, sizeof(valStr), val, 1);
            snprintf(confirmMsg, sizeof(confirmMsg),
                     "VARUNA OLP SET: %s cm", valStr);
            queueSMS(sender, confirmMsg);
        }
    }
    else if (strcmp(body, "STATUS") == 0)
    {
        char statusMsg[160];
        char heightStr[10], rateStr2[10];
        floatToStr(heightStr, sizeof(heightStr), prevWaterHeight, 1);
        floatToStr(rateStr2, sizeof(rateStr2), ratePer15Min, 1);

        snprintf(statusMsg, sizeof(statusMsg),
                 "VARUNA STATUS\nLvl:%scm %s\nResp:%s\nRate:%s/15m\n"
                 "Batt:%d%%\nSat:%d RSSI:%d\nAlgo:%s",
                 heightStr, zoneNameStr(currentZone),
                 responseNameStr(currentResponseLevel),
                 rateStr2,
                 (int)batteryPercent,
                 gpsSatellites, simSignalRSSI,
                 algorithmEnabled ? "ON" : "OFF");
        queueSMS(sender, statusMsg);
    }
    else if (strncmp(body, "ADDAUTH:", 8) == 0)
    {
        if (authorizedCount < MAX_AUTHORIZED)
        {
            const char *phone = body + 8;
            int pLen = strlen(phone);
            if (pLen > 3 && pLen < PHONE_NUMBER_LENGTH)
            {
                strncpy(authorizedNumbers[authorizedCount], phone,
                        PHONE_NUMBER_LENGTH - 1);
                authorizedNumbers[authorizedCount][PHONE_NUMBER_LENGTH - 1] = '\0';
                authorizedCount++;
                saveContactsToEeprom();
                queueSMS(sender, "VARUNA: Auth number added");
            }
        }
    }
    else if (strncmp(body, "SETOBLIGHT:", 11) == 0)
    {
        const char *params = body + 11;
        const char *comma = strchr(params, ',');
        if (comma != NULL)
        {
            unsigned long newOn = (unsigned long)atol(params);
            unsigned long newOff = (unsigned long)atol(comma + 1);

            if (newOn >= OB_MIN_ON_MS && newOn <= OB_MAX_ON_MS &&
                    newOff >= OB_MIN_OFF_MS && newOff <= OB_MAX_OFF_MS)
            {
                obOnTimeMs = newOn;
                obOffTimeMs = newOff;
                strncpy(currentObPresetName, "CUSTOM", OB_PRESET_NAME_LEN - 1);
                currentObPresetName[OB_PRESET_NAME_LEN - 1] = '\0';
                obLightSaveToEeprom();

                char confirmMsg[80];
                snprintf(confirmMsg, sizeof(confirmMsg),
                         "VARUNA OBLIGHT SET: ON=%lums OFF=%lums",
                         obOnTimeMs, obOffTimeMs);
                queueSMS(sender, confirmMsg);
            }
            else
            {
                queueSMS(sender, "VARUNA: Invalid OBLIGHT timing");
            }
        }
        else
        {
            queueSMS(sender, "VARUNA: Format: SETOBLIGHT:on_ms,off_ms");
        }
    }
    else if (strncmp(body, "SETOBPRESET:", 12) == 0)
    {
        const char *presetName = body + 12;
        boolean applied = obLightApplyPreset(presetName);
        if (applied)
        {
            char confirmMsg[80];
            snprintf(confirmMsg, sizeof(confirmMsg),
                     "VARUNA OBLIGHT PRESET: %s", presetName);
            queueSMS(sender, confirmMsg);
        }
        else
        {
            queueSMS(sender, "VARUNA: Unknown preset. Send GETOBPRESETS");
        }
    }
    else if (strcmp(body, "GETOBLIGHT") == 0)
    {
        char msg[80];
        snprintf(msg, sizeof(msg),
                 "VARUNA OBLIGHT: ON=%lums OFF=%lums EN=%d P=%s",
                 obOnTimeMs, obOffTimeMs, obLightEnabled ? 1 : 0,
                 currentObPresetName);
        queueSMS(sender, msg);
    }
    else if (strcmp(body, "OBLIGHTON") == 0)
    {
        obLightEnabled = true;
        queueSMS(sender, "VARUNA: Obstruction lights ENABLED");
    }
    else if (strcmp(body, "OBLIGHTOFF") == 0)
    {
        obLightEnabled = false;
        obLightSetAll(false);
        queueSMS(sender, "VARUNA: Obstruction lights DISABLED");
    }
    else
    {
        queueSMS(sender, "VARUNA: Unknown command");
    }
}

void checkSimURC()
{
    while (SIM_Serial.available())
    {
        char c = SIM_Serial.read();

        if (c == '\n' || c == '\r')
        {
            if (simUrcIdx > 0)
            {
                simUrcBuffer[simUrcIdx] = '\0';

                if (smsRxState == SMS_RX_IDLE)
                {
                    if (strncmp(simUrcBuffer, "+CMT:", 5) == 0)
                    {
                        parseSmsHeader(simUrcBuffer);
                    }
                }
                else if (smsRxState == SMS_RX_HEADER_RECEIVED)
                {
                    strncpy(smsRxBody, simUrcBuffer, sizeof(smsRxBody) - 1);
                    smsRxBody[sizeof(smsRxBody) - 1] = '\0';
                    processSmsCommand(smsRxSender, smsRxBody);
                    smsRxState = SMS_RX_IDLE;
                }

                simUrcIdx = 0;
            }
        }
        else
        {
            if (simUrcIdx < (int)sizeof(simUrcBuffer) - 1)
            {
                simUrcBuffer[simUrcIdx++] = c;
            }
        }
    }
}

boolean gprsInit()
{
    ATResult res;

    send_at_command("AT+HTTPTERM", "OK", 2000);
    send_at_command("AT+SAPBR=0,1", "OK", 2000);

    res = send_at_command("AT+SAPBR=3,1,\"Contype\",\"GPRS\"", "OK", 3000);
    if (res != AT_SUCCESS)
    {
        return false;
    }

    char apnCmd[64];
    snprintf(apnCmd, sizeof(apnCmd), "AT+SAPBR=3,1,\"APN\",\"%s\"", gprsApn);
    res = send_at_command(apnCmd, "OK", 3000);
    if (res != AT_SUCCESS)
    {
        return false;
    }

    res = send_at_command("AT+SAPBR=1,1", "OK", 15000);
    if (res != AT_SUCCESS)
    {
        return false;
    }

    res = send_at_command("AT+SAPBR=2,1", "+SAPBR:", 5000);
    if (res != AT_SUCCESS)
    {
        return false;
    }

    Serial.println("STATUS:GPRS_CONNECTED");
    debugSendStatusEvent("GPRS_CONNECTED");
    gprsConnected = true;
    return true;
}

boolean gprsPostData(const char *jsonPayload, int payloadLen)
{
    ATResult res;

    res = send_at_command("AT+HTTPINIT", "OK", 5000);
    if (res != AT_SUCCESS)
    {
        gprsInit();
        res = send_at_command("AT+HTTPINIT", "OK", 5000);
        if (res != AT_SUCCESS)
        {
            return false;
        }
    }

    send_at_command("AT+HTTPPARA=\"CID\",1", "OK", 3000);

    char urlCmd[128];
    snprintf(urlCmd, sizeof(urlCmd),
             "AT+HTTPPARA=\"URL\",\"%s\"", gprsServer);
    send_at_command(urlCmd, "OK", 3000);

    send_at_command("AT+HTTPPARA=\"CONTENT\",\"application/json\"", "OK", 3000);

    char dataCmd[32];
    snprintf(dataCmd, sizeof(dataCmd), "AT+HTTPDATA=%d,10000", payloadLen);
    res = send_at_command(dataCmd, "DOWNLOAD", 5000);
    if (res != AT_SUCCESS)
    {
        send_at_command("AT+HTTPTERM", "OK", 2000);
        return false;
    }

    SIM_Serial.write((const uint8_t *)jsonPayload, payloadLen);
    delay(1000);

    res = send_at_command("AT+HTTPACTION=1", "+HTTPACTION:", 30000);

    boolean success = false;
    if (res == AT_SUCCESS)
    {
        char *actionPos = strstr(simResponseBuffer, "+HTTPACTION:");
        if (actionPos)
        {
            char *firstComma = strchr(actionPos, ',');
            if (firstComma)
            {
                int httpCode = atoi(firstComma + 1);
                success = (httpCode >= 200 && httpCode < 300);

                Serial.print("STATUS:GPRS_HTTP_CODE=");
                Serial.println(httpCode);
            }
        }
    }

    send_at_command("AT+HTTPTERM", "OK", 2000);
    return success;
}

boolean gprsUploadBuffer()
{
    if (!gprsConnected)
    {
        if (!gprsInit())
        {
            return false;
        }
    }

    char jsonBuf[1024];
    int pos = 0;

    char latStr[12], lonStr[12];
    floatToStr(latStr, sizeof(latStr), gpsLat, 4);
    floatToStr(lonStr, sizeof(lonStr), gpsLon, 4);

    pos += snprintf(jsonBuf + pos, sizeof(jsonBuf) - pos,
                    "{\"d\":\"VARUNA01\",\"lat\":%s,\"lon\":%s,\"r\":[",
                    latStr, lonStr);

    for (int i = 0; i < txBufferCount && pos < 900; i++)
    {
        if (i > 0)
        {
            jsonBuf[pos++] = ',';
        }

        char hStr[10], rStr[10], bStr[6];
        floatToStr(hStr, sizeof(hStr), txBufferHeight[i], 1);
        floatToStr(rStr, sizeof(rStr), txBufferRate[i], 2);
        floatToStr(bStr, sizeof(bStr), batteryPercent, 0);

        pos += snprintf(jsonBuf + pos, sizeof(jsonBuf) - pos,
                        "{\"t\":%lu,\"h\":%s,\"r\":%s,\"z\":%d,\"l\":%d,\"b\":%s}",
                        (unsigned long)txBufferTime[i],
                        hStr, rStr,
                        currentZone,
                        currentResponseLevel,
                        bStr);
    }

    pos += snprintf(jsonBuf + pos, sizeof(jsonBuf) - pos, "]}");

    boolean ok = gprsPostData(jsonBuf, pos);

    if (ok)
    {
        gprsConsecutiveFails = 0;
        Serial.println("STATUS:GPRS_UPLOAD_OK");
    }
    else
    {
        gprsConsecutiveFails++;
        Serial.print("STATUS:GPRS_UPLOAD_FAIL_COUNT=");
        Serial.println(gprsConsecutiveFails);

        if (gprsConsecutiveFails >= GPRS_MAX_FAILS)
        {
            Serial.println("STATUS:GPRS_FALLBACK_TO_SMS");
            useGprsTransmit = false;
            gprsConnected = false;
        }
    }

    return ok;
}

void initStorage()
{
    if (SPIFFS.begin(true))
    {
        spiffsAvailable = true;
        Serial.print("STATUS:SPIFFS_OK_FREE=");
        Serial.println(SPIFFS.totalBytes() - SPIFFS.usedBytes());
    }
    else
    {
        Serial.println("WARNING:SPIFFS_INIT_FAIL");
    }
}

void archiveToStorage(float height, float rate, uint32_t timestamp,
                      int zone, int response)
{
    if (!spiffsAvailable)
    {
        return;
    }

    size_t freeBytes = SPIFFS.totalBytes() - SPIFFS.usedBytes();
    if (freeBytes < 1024)
    {
        File root = SPIFFS.open("/");
        File oldest = root.openNextFile();
        if (oldest)
        {
            char oldName[32];
            const char *rawName = oldest.name();
            if (rawName[0] == '/')
            {
                strncpy(oldName, rawName, sizeof(oldName) - 1);
            }
            else
            {
                snprintf(oldName, sizeof(oldName), "/%s", rawName);
            }
            oldName[sizeof(oldName) - 1] = '\0';
            oldest.close();
            SPIFFS.remove(oldName);
            Serial.print("STATUS:SPIFFS_DELETED_OLDEST=");
            Serial.println(oldName);
        }
        root.close();
    }

    uint16_t y;
    uint8_t mo, d, h, mi, s, dow;
    unixToDate(timestamp, &y, &mo, &d, &h, &mi, &s, &dow);

    char filename[24];
    snprintf(filename, sizeof(filename), "/log_%04d%02d%02d.csv", y, mo, d);

    File f = SPIFFS.open(filename, FILE_APPEND);
    if (f)
    {
        char hStr[10], rStr[10], bStr[6];
        floatToStr(hStr, sizeof(hStr), height, 1);
        floatToStr(rStr, sizeof(rStr), rate, 2);
        floatToStr(bStr, sizeof(bStr), batteryPercent, 0);

        char line[80];
        snprintf(line, sizeof(line), "%lu,%s,%s,%d,%d,%s\n",
                 (unsigned long)timestamp, hStr, rStr, zone, response, bStr);
        f.print(line);
        f.close();
    }
}

int classifyZone(float waterHeight)
{
    if (waterHeight >= dangerLevelCm)
    {
        return ZONE_DANGER;
    }
    else if (waterHeight >= warningLevelCm)
    {
        return ZONE_WARNING;
    }
    else if (waterHeight >= alertLevelCm)
    {
        return ZONE_ALERT;
    }
    else
    {
        return ZONE_NORMAL;
    }
}

int classifyRate(float normalizedRate)
{
    if (normalizedRate < 0)
    {
        return RATE_FALLING;
    }
    else if (normalizedRate <= RATE_MODERATE_THRESH)
    {
        return RATE_SLOW;
    }
    else if (normalizedRate <= RATE_FAST_THRESH)
    {
        return RATE_MODERATE;
    }
    else
    {
        return RATE_FAST;
    }
}

void updateSustainedBuffer(float waterHeight, uint32_t timestamp)
{
    sustainedBuffer[sustainedBufIdx] = waterHeight;
    sustainedTimeBuffer[sustainedBufIdx] = timestamp;
    sustainedBufIdx = (sustainedBufIdx + 1) % SUSTAINED_BUF_SIZE;

    if (sustainedBufCount < SUSTAINED_BUF_SIZE)
    {
        sustainedBufCount++;
    }

    if (sustainedBufCount < SUSTAINED_BUF_SIZE)
    {
        sustainedRise = false;
        return;
    }

    float ordered[SUSTAINED_BUF_SIZE];
    for (int i = 0; i < SUSTAINED_BUF_SIZE; i++)
    {
        int idx = (sustainedBufIdx + i) % SUSTAINED_BUF_SIZE;
        ordered[i] = sustainedBuffer[idx];
    }

    boolean netRising = (ordered[SUSTAINED_BUF_SIZE - 1] >
                         ordered[0] + SUSTAINED_EPSILON_CM);

    int riseCount = 0;
    for (int i = 1; i < SUSTAINED_BUF_SIZE; i++)
    {
        if (ordered[i] > ordered[i - 1] + SUSTAINED_EPSILON_CM)
        {
            riseCount++;
        }
    }

    sustainedRise = netRising && (riseCount >= 2);
}

void flushSustainedBufferIfIntervalChanged(unsigned long oldInterval,
        unsigned long newInterval)
{
    if (newInterval > 0 && oldInterval > 0 && oldInterval >= newInterval * 4)
    {
        float lastHeight = sustainedBuffer[(sustainedBufIdx + SUSTAINED_BUF_SIZE - 1)
                                            % SUSTAINED_BUF_SIZE];
        uint32_t lastTime = sustainedTimeBuffer[(sustainedBufIdx + SUSTAINED_BUF_SIZE - 1)
                                                 % SUSTAINED_BUF_SIZE];

        for (int i = 0; i < SUSTAINED_BUF_SIZE; i++)
        {
            sustainedBuffer[i] = lastHeight;
            sustainedTimeBuffer[i] = lastTime;
        }
        sustainedBufCount = 1;
        sustainedBufIdx = 1;
        sustainedRise = false;

        Serial.println("STATUS:SUSTAINED_BUFFER_FLUSHED_INTERVAL_CHANGE");
    }
}

int lookupDecisionMatrix(int zone, int rateCategory, boolean sustained)
{
    int effectiveRate = rateCategory;
    if (effectiveRate == RATE_FALLING)
    {
        effectiveRate = RATE_SLOW;
    }

    int rateIdx = effectiveRate - 1;
    if (rateIdx < 0)
    {
        rateIdx = 0;
    }
    if (rateIdx > 2)
    {
        rateIdx = 2;
    }

    static const int matrix[4][3][2] = {
        {
            {RESP_NORMAL, RESP_NORMAL},
            {RESP_NORMAL, RESP_WATCH},
            {RESP_NORMAL, RESP_WATCH}
        },
        {
            {RESP_WATCH, RESP_WATCH},
            {RESP_WATCH, RESP_WARNING},
            {RESP_WATCH, RESP_WARNING}
        },
        {
            {RESP_WARNING, RESP_FLOOD},
            {RESP_WARNING, RESP_FLOOD},
            {RESP_FLOOD, RESP_CRITICAL}
        },
        {
            {RESP_FLOOD, RESP_CRITICAL},
            {RESP_CRITICAL, RESP_CRITICAL},
            {RESP_CRITICAL, RESP_CRITICAL}
        }
    };

    return matrix[zone][rateIdx][sustained ? 1 : 0];
}

void calculateRateOfChange(float currentHeight, uint32_t currentTime)
{
    if (prevReadingTime == 0 || currentTime == 0 ||
            currentTime <= prevReadingTime)
    {
        prevReadingTime = currentTime;
        prevWaterHeight = currentHeight;
        rateOfChange = 0;
        ratePer15Min = 0;
        return;
    }

    uint32_t elapsed = currentTime - prevReadingTime;

    if (elapsed < MIN_RATE_ELAPSED_SEC)
    {
        return;
    }

    float change = currentHeight - prevWaterHeight;

    rateOfChange = change / (float)elapsed;
    ratePer15Min = change * (900.0 / (float)elapsed);

    if (ratePer15Min > MAX_PLAUSIBLE_RATE)
    {
        ratePer15Min = MAX_PLAUSIBLE_RATE;
    }
    if (ratePer15Min < -MAX_PLAUSIBLE_RATE)
    {
        ratePer15Min = -MAX_PLAUSIBLE_RATE;
    }

    prevReadingTime = currentTime;
    prevWaterHeight = currentHeight;
}

boolean canStepDown(int fromLevel, uint32_t currentTime)
{
    if (currentTime == 0)
    {
        return (stepDownConsecutive >= STEPDOWN_READINGS_REQUIRED * 2);
    }

    if (stateEntryTime == 0)
    {
        return true;
    }

    uint32_t elapsed = 0;
    if (currentTime > stateEntryTime)
    {
        elapsed = currentTime - stateEntryTime;
    }

    switch (fromLevel)
    {
        case RESP_CRITICAL:
            return (elapsed >= MIN_TIME_CRITICAL);
        case RESP_FLOOD:
            return (elapsed >= MIN_TIME_FLOOD);
        case RESP_WARNING:
            return (elapsed >= MIN_TIME_WARNING);
        case RESP_WATCH:
            return (elapsed >= MIN_TIME_WATCH);
        default:
            return true;
    }
}

int evaluateStepDown(int currentLevel, int matrixLevel, float waterHeight,
                     int rateCategory, boolean sustained, uint32_t currentTime)
{
    if (matrixLevel > currentLevel)
    {
        stepDownConsecutive = 0;
        return matrixLevel;
    }

    if (matrixLevel == currentLevel)
    {
        stepDownConsecutive = 0;
        return currentLevel;
    }

    boolean stepDownConditionMet = false;

    switch (currentLevel)
    {
        case RESP_CRITICAL:
            stepDownConditionMet = (waterHeight < dangerLevelCm) &&
                                  (rateCategory == RATE_FALLING ||
                                   rateCategory == RATE_SLOW);
            break;
        case RESP_FLOOD:
            stepDownConditionMet = (waterHeight < warningLevelCm);
            break;
        case RESP_WARNING:
            stepDownConditionMet = (waterHeight < alertLevelCm);
            break;
        case RESP_WATCH:
            stepDownConditionMet = (waterHeight < alertLevelCm) &&
                                  (rateCategory == RATE_FALLING ||
                                   rateCategory == RATE_SLOW) &&
                                  (!sustained);
            break;
        default:
            stepDownConditionMet = true;
            break;
    }

    if (stepDownConditionMet)
    {
        stepDownConsecutive++;

        int required = STEPDOWN_READINGS_REQUIRED;
        if (currentLevel == RESP_WATCH)
        {
            required = STEPDOWN_NORMAL_READINGS;
        }

        if (stepDownConsecutive >= required &&
                canStepDown(currentLevel, currentTime))
        {
            stepDownConsecutive = 0;
            int newLevel = currentLevel - 1;
            if (newLevel < RESP_NORMAL)
            {
                newLevel = RESP_NORMAL;
            }
            return newLevel;
        }
        else
        {
            return currentLevel;
        }
    }
    else
    {
        stepDownConsecutive = 0;
        return currentLevel;
    }
}

const char *responseNameStr(int level)
{
    switch (level)
    {
        case RESP_NORMAL:
            return "NORMAL";
        case RESP_WATCH:
            return "WATCH";
        case RESP_WARNING:
            return "WARNING";
        case RESP_FLOOD:
            return "FLOOD ALERT";
        case RESP_CRITICAL:
            return "CRITICAL";
        default:
            return "UNKNOWN";
    }
}

const char *zoneNameStr(int zone)
{
    switch (zone)
    {
        case ZONE_NORMAL:
            return "NORMAL";
        case ZONE_ALERT:
            return "ALERT";
        case ZONE_WARNING:
            return "WARNING";
        case ZONE_DANGER:
            return "DANGER";
        default:
            return "UNKNOWN";
    }
}

void composeAlertSms(int responseLevel, float waterHeight, float rate,
                     int zone, boolean sustained)
{
    memset(smsMsgBuffer, 0, SMS_MSG_SIZE);

    char heightStr[10], rateStr2[10], latStr[12], lonStr[12];
    floatToStr(heightStr, sizeof(heightStr), waterHeight, 1);
    floatToStr(rateStr2, sizeof(rateStr2), rate, 1);
    floatToStr(latStr, sizeof(latStr), gpsLat, 4);
    floatToStr(lonStr, sizeof(lonStr), gpsLon, 4);

    snprintf(smsMsgBuffer, SMS_MSG_SIZE,
             "VARUNA %s\nLvl:%scm Zone:%s\nRate:%scm/15m %s\nLat:%s Lon:%s\nT:%lu",
             responseNameStr(responseLevel),
             heightStr,
             zoneNameStr(zone),
             rateStr2,
             sustained ? "RISING" : "STABLE",
             latStr, lonStr,
             (unsigned long)getBestTimestamp());
}

void composeAllClearSms(float currentHeight, float peak, uint32_t peakT)
{
    memset(smsMsgBuffer, 0, SMS_MSG_SIZE);

    char heightStr[10], peakStr[10], latStr[12], lonStr[12];
    floatToStr(heightStr, sizeof(heightStr), currentHeight, 1);
    floatToStr(peakStr, sizeof(peakStr), peak, 1);
    floatToStr(latStr, sizeof(latStr), gpsLat, 4);
    floatToStr(lonStr, sizeof(lonStr), gpsLon, 4);

    snprintf(smsMsgBuffer, SMS_MSG_SIZE,
             "VARUNA ALL CLEAR\nCurrent:%scm\nPeak:%scm at %lu\nNormal ops resumed\nLat:%s Lon:%s",
             heightStr,
             peakStr, (unsigned long)peakT,
             latStr, lonStr);
}

void composeDeescalationSms(int fromLevel, int toLevel, float currentHeight)
{
    memset(smsMsgBuffer, 0, SMS_MSG_SIZE);

    char heightStr[10], latStr[12], lonStr[12];
    floatToStr(heightStr, sizeof(heightStr), currentHeight, 1);
    floatToStr(latStr, sizeof(latStr), gpsLat, 4);
    floatToStr(lonStr, sizeof(lonStr), gpsLon, 4);

    snprintf(smsMsgBuffer, SMS_MSG_SIZE,
             "VARUNA IMPROVING\n%s -> %s\nLevel:%scm\nMonitoring continues\nLat:%s Lon:%s",
             responseNameStr(fromLevel),
             responseNameStr(toLevel),
             heightStr,
             latStr, lonStr);
}

int sendToTier(char contacts[][PHONE_NUMBER_LENGTH], int count)
{
    int queued = 0;
    for (int i = 0; i < count; i++)
    {
        if (strlen(contacts[i]) > 3)
        {
            if (queueSMS(contacts[i], smsMsgBuffer))
            {
                queued++;
            }
        }
    }
    return queued;
}

void dispatchAlerts(int responseLevel, float waterHeight, float rate,
                    int zone, boolean sustained, boolean isEscalation)
{
    if (!isEscalation)
    {
        uint32_t ts = getBestTimestamp();
        if (ts > 0 && lastAlertTimeByLevel[responseLevel] > 0)
        {
            uint32_t elapsed = ts - lastAlertTimeByLevel[responseLevel];
            uint32_t cooldown = ALERT_SMS_COOLDOWN_SEC;

            switch (responseLevel)
            {
                case RESP_CRITICAL:
                    cooldown = REALERT_INTERVAL_CRITICAL;
                    break;
                case RESP_FLOOD:
                    cooldown = REALERT_INTERVAL_FLOOD;
                    break;
                case RESP_WARNING:
                    cooldown = REALERT_INTERVAL_WARNING;
                    break;
                case RESP_WATCH:
                    cooldown = ALERT_SMS_COOLDOWN_SEC;
                    break;
                default:
                    cooldown = ALERT_SMS_COOLDOWN_SEC;
                    break;
            }

            if (elapsed < cooldown)
            {
                return;
            }
        }
    }

    composeAlertSms(responseLevel, waterHeight, rate, zone, sustained);

    int totalSent = 0;

    switch (responseLevel)
    {
        case RESP_WATCH:
            totalSent += sendToTier(tier1Contacts, tier1Count);
            break;

        case RESP_WARNING:
            totalSent += sendToTier(tier1Contacts, tier1Count);
            totalSent += sendToTier(tier2Contacts, tier2Count);
            break;

        case RESP_FLOOD:
            totalSent += sendToTier(tier1Contacts, tier1Count);
            totalSent += sendToTier(tier2Contacts, tier2Count);
            totalSent += sendToTier(tier3Contacts, tier3Count);
            break;

        case RESP_CRITICAL:
            totalSent += sendToTier(tier1Contacts, tier1Count);
            totalSent += sendToTier(tier2Contacts, tier2Count);
            totalSent += sendToTier(tier3Contacts, tier3Count);
            escalateWithVoiceCalls();
            break;

        default:
            break;
    }

    if (totalSent > 0)
    {
        lastAlertTimeByLevel[responseLevel] = getBestTimestamp();

        Serial.print("STATUS:ALERT_SMS_SENT_COUNT=");
        Serial.print(totalSent);
        Serial.print("_LEVEL=");
        Serial.print(responseNameStr(responseLevel));
        Serial.print("_ESCALATION=");
        Serial.println(isEscalation ? "YES" : "NO");

        debugSendAlertDispatch(responseLevel, totalSent);

        if (responseLevel >= RESP_FLOOD && !ackReceived)
        {
            ackWaitStartTime = getBestTimestamp();
            ackEscalationDone = false;
        }
    }

    if (responseLevel > highestResponseReached)
    {
        highestResponseReached = responseLevel;
    }
}

void dispatchDeescalation(int fromLevel, int toLevel, float waterHeight)
{
    composeDeescalationSms(fromLevel, toLevel, waterHeight);

    Serial.print("STATUS:DEESCALATION_");
    Serial.print(responseNameStr(fromLevel));
    Serial.print("_TO_");
    Serial.println(responseNameStr(toLevel));

    debugSendFloodTransition(fromLevel, toLevel, waterHeight);

    switch (fromLevel)
    {
        case RESP_CRITICAL:
            sendToTier(tier1Contacts, tier1Count);
            sendToTier(tier2Contacts, tier2Count);
            sendToTier(tier3Contacts, tier3Count);
            break;

        case RESP_FLOOD:
            sendToTier(tier1Contacts, tier1Count);
            sendToTier(tier2Contacts, tier2Count);
            break;

        case RESP_WARNING:
            sendToTier(tier1Contacts, tier1Count);
            sendToTier(tier2Contacts, tier2Count);
            break;

        case RESP_WATCH:
            sendToTier(tier1Contacts, tier1Count);
            break;

        default:
            break;
    }
}

void dispatchAllClear(float waterHeight, float peak, uint32_t peakT)
{
    composeAllClearSms(waterHeight, peak, peakT);

    Serial.println("STATUS:ALL_CLEAR_SENDING");
    debugSendStatusEvent("ALL_CLEAR_SENDING");

    if (highestResponseReached >= RESP_WATCH)
    {
        sendToTier(tier1Contacts, tier1Count);
    }
    if (highestResponseReached >= RESP_WARNING)
    {
        sendToTier(tier2Contacts, tier2Count);
    }
    if (highestResponseReached >= RESP_FLOOD)
    {
        sendToTier(tier3Contacts, tier3Count);
    }

    highestResponseReached = RESP_NORMAL;
    allClearPending = false;

    peakHeight = waterHeight;
    peakTime = getBestTimestamp();
    minHeight = waterHeight;
    minTime = getBestTimestamp();

    for (int i = 0; i < 5; i++)
    {
        lastAlertTimeByLevel[i] = 0;
    }

    resetAckTracking();

    Serial.println("STATUS:ALL_CLEAR_SENT_PEAK_RESET");
}

void evaluateFloodStatus(float waterHeight, uint32_t timestamp)
{
    uint32_t ts = getBestTimestamp();

    readingsSinceBoot++;

    currentZone = classifyZone(waterHeight);
    currentRateCategory = classifyRate(ratePer15Min);
    updateSustainedBuffer(waterHeight, ts);

    boolean effectiveSustained = sustainedRise;

    if (readingsSinceBoot == 1)
    {
        int coldStartLevel;

        if (currentZone == ZONE_DANGER)
        {
            coldStartLevel = RESP_CRITICAL;
        }
        else if (currentZone == ZONE_WARNING)
        {
            coldStartLevel = RESP_WARNING;
        }
        else
        {
            coldStartLevel = RESP_NORMAL;
        }

        if (coldStartLevel != currentResponseLevel)
        {
            stateEntryTime = ts;
            previousResponseLevel = currentResponseLevel;
            currentResponseLevel = coldStartLevel;

            if (currentResponseLevel >= RESP_WARNING)
            {
                dispatchAlerts(currentResponseLevel, waterHeight, ratePer15Min,
                               currentZone, false, true);
                allClearPending = true;
            }

            Serial.print("STATUS:COLD_START_LEVEL=");
            Serial.println(responseNameStr(currentResponseLevel));
            debugSendFloodTransition(previousResponseLevel,
                                     currentResponseLevel, waterHeight);
            forceSaveEeprom();
        }

        switch (currentResponseLevel)
        {
            case RESP_NORMAL:
            case RESP_WATCH:
                floodAlertLevel = 0;
                break;
            case RESP_WARNING:
                floodAlertLevel = 1;
                break;
            case RESP_FLOOD:
                floodAlertLevel = 2;
                break;
            case RESP_CRITICAL:
                floodAlertLevel = 3;
                break;
            default:
                floodAlertLevel = 0;
                break;
        }
        return;
    }

    if (readingsSinceBoot <= SUSTAINED_BUF_SIZE &&
            currentZone >= ZONE_WARNING)
    {
        effectiveSustained = true;
        Serial.println("STATUS:COLD_START_SUSTAINED_OVERRIDE");
    }

    if (ratePer15Min > RATE_EXTREME_THRESH && currentZone >= ZONE_ALERT)
    {
        effectiveSustained = true;
        char rStr[10];
        floatToStr(rStr, sizeof(rStr), ratePer15Min, 1);
        Serial.print("STATUS:EXTREME_RATE_OVERRIDE=");
        Serial.println(rStr);
    }

    if (ratePer15Min > RATE_CATASTROPHIC_THRESH)
    {
        effectiveSustained = true;
        char rStr[10];
        floatToStr(rStr, sizeof(rStr), ratePer15Min, 1);
        Serial.print("STATUS:CATASTROPHIC_RATE_OVERRIDE=");
        Serial.println(rStr);
    }

    int matrixLevel = lookupDecisionMatrix(currentZone, currentRateCategory,
                                           effectiveSustained);

    int newResponseLevel = evaluateStepDown(currentResponseLevel, matrixLevel,
                                            waterHeight, currentRateCategory,
                                            effectiveSustained, ts);

    switch (newResponseLevel)
    {
        case RESP_NORMAL:
        case RESP_WATCH:
            floodAlertLevel = 0;
            break;
        case RESP_WARNING:
            floodAlertLevel = 1;
            break;
        case RESP_FLOOD:
            floodAlertLevel = 2;
            break;
        case RESP_CRITICAL:
            floodAlertLevel = 3;
            break;
        default:
            floodAlertLevel = 0;
            break;
    }

    previousResponseLevel = currentResponseLevel;

    if (newResponseLevel != currentResponseLevel)
    {
        boolean isEscalation = (newResponseLevel > currentResponseLevel);
        boolean isDeescalation = (newResponseLevel < currentResponseLevel);

        stateEntryTime = ts;
        currentResponseLevel = newResponseLevel;

        Serial.print("STATUS:RESPONSE_TRANSITION=");
        Serial.print(responseNameStr(previousResponseLevel));
        Serial.print("->");
        Serial.println(responseNameStr(currentResponseLevel));

        debugSendFloodTransition(previousResponseLevel,
                                 currentResponseLevel, waterHeight);

        if (isEscalation)
        {
            if (currentResponseLevel >= RESP_WATCH)
            {
                dispatchAlerts(currentResponseLevel, waterHeight, ratePer15Min,
                               currentZone, effectiveSustained, true);
            }

            if (currentResponseLevel >= RESP_WARNING)
            {
                allClearPending = true;
            }
        }
        else if (isDeescalation)
        {
            if (previousResponseLevel >= RESP_WATCH)
            {
                dispatchDeescalation(previousResponseLevel,
                                     currentResponseLevel, waterHeight);
            }

            if (previousResponseLevel >= RESP_WARNING &&
                    currentResponseLevel < RESP_WARNING)
            {
                memset(smsMsgBuffer, 0, SMS_MSG_SIZE);
                char hStr[10], pStr[10];
                floatToStr(hStr, sizeof(hStr), waterHeight, 1);
                floatToStr(pStr, sizeof(pStr), peakHeight, 1);
                snprintf(smsMsgBuffer, SMS_MSG_SIZE,
                         "VARUNA IMPROVING\nBelow WARNING level\nCurrent:%scm\n"
                         "Peak:%scm\nMonitoring continues",
                         hStr, pStr);

                if (highestResponseReached >= RESP_WATCH)
                {
                    sendToTier(tier1Contacts, tier1Count);
                }
                if (highestResponseReached >= RESP_WARNING)
                {
                    sendToTier(tier2Contacts, tier2Count);
                }
                if (highestResponseReached >= RESP_FLOOD)
                {
                    sendToTier(tier3Contacts, tier3Count);
                }

                Serial.println("STATUS:BELOW_WARNING_NOTIFICATION_SENT");
            }

            if (currentResponseLevel == RESP_NORMAL && allClearPending)
            {
                dispatchAllClear(waterHeight, peakHeight, peakTime);
            }
        }

        forceSaveEeprom();
    }
}

void readBatteryLevel()
{
    int adcRaw = analogRead(BATTERY_ADC_PIN);

    float adcMv = ((float)adcRaw / (float)ADC_RESOLUTION) * (float)ADC_MAX_MV;
    float batteryMv = adcMv * BATTERY_DIVIDER_RATIO;

    if (batteryMv >= BATTERY_FULL_MV)
    {
        batteryPercent = 100.0;
    }
    else if (batteryMv <= BATTERY_EMPTY_MV)
    {
        batteryPercent = 0.0;
    }
    else
    {
        batteryPercent = ((batteryMv - BATTERY_EMPTY_MV) /
                          (float)(BATTERY_FULL_MV - BATTERY_EMPTY_MV)) * 100.0;
    }
}

void updateAdaptiveIntervals()
{
    unsigned long oldSampleInterval = sampleIntervalMs;

    boolean isCriticalBattery = (batteryPercent < BATTERY_CRITICAL_THRESH);
    boolean isLowBattery = (batteryPercent < BATTERY_LOW_THRESH);

    if (isCriticalBattery)
    {
        if (currentResponseLevel >= RESP_WARNING)
        {
            sampleIntervalMs = INTERVAL_5MIN;
            transmitIntervalMs = INTERVAL_5MIN;
            useGprsTransmit = false;
        }
        else
        {
            sampleIntervalMs = INTERVAL_120MIN;
            transmitIntervalMs = INTERVAL_240MIN;
            useGprsTransmit = false;
        }
    }
    else if (isLowBattery)
    {
        if (currentResponseLevel >= RESP_WARNING)
        {
            sampleIntervalMs = INTERVAL_5MIN;
            transmitIntervalMs = INTERVAL_5MIN;
            useGprsTransmit = true;
        }
        else if (currentResponseLevel >= RESP_WATCH)
        {
            sampleIntervalMs = INTERVAL_15MIN;
            transmitIntervalMs = INTERVAL_30MIN;
            useGprsTransmit = true;
        }
        else
        {
            sampleIntervalMs = INTERVAL_60MIN;
            transmitIntervalMs = INTERVAL_120MIN;
            useGprsTransmit = true;
        }
    }
    else
    {
        switch (currentResponseLevel)
        {
            case RESP_CRITICAL:
                sampleIntervalMs = INTERVAL_2MIN;
                transmitIntervalMs = INTERVAL_2MIN;
                break;

            case RESP_FLOOD:
                sampleIntervalMs = INTERVAL_5MIN;
                transmitIntervalMs = INTERVAL_5MIN;
                break;

            case RESP_WARNING:
                sampleIntervalMs = INTERVAL_5MIN;
                transmitIntervalMs = INTERVAL_5MIN;
                break;

            case RESP_WATCH:
                sampleIntervalMs = INTERVAL_10MIN;
                transmitIntervalMs = INTERVAL_15MIN;
                break;

            case RESP_NORMAL:
            default:
                if (currentRateCategory == RATE_MODERATE ||
                        currentRateCategory == RATE_FAST)
                {
                    sampleIntervalMs = INTERVAL_15MIN;
                    transmitIntervalMs = INTERVAL_30MIN;
                }
                else
                {
                    sampleIntervalMs = INTERVAL_30MIN;
                    transmitIntervalMs = INTERVAL_60MIN;
                }
                break;
        }
        useGprsTransmit = true;
    }

    flushSustainedBufferIfIntervalChanged(oldSampleInterval, sampleIntervalMs);
}

void addToTransmitBuffer(float waterHeight, float rate, uint32_t timestamp)
{
    if (txBufferCount < TX_BUFFER_SIZE)
    {
        txBufferHeight[txBufferCount] = waterHeight;
        txBufferRate[txBufferCount] = rate;
        txBufferTime[txBufferCount] = timestamp;
        txBufferCount++;
    }
    else
    {
        for (int i = 0; i < TX_BUFFER_SIZE - 1; i++)
        {
            txBufferHeight[i] = txBufferHeight[i + 1];
            txBufferRate[i] = txBufferRate[i + 1];
            txBufferTime[i] = txBufferTime[i + 1];
        }
        txBufferHeight[TX_BUFFER_SIZE - 1] = waterHeight;
        txBufferRate[TX_BUFFER_SIZE - 1] = rate;
        txBufferTime[TX_BUFFER_SIZE - 1] = timestamp;
    }
}

void transmitBufferedData(float currentHeight)
{
    if (useGprsTransmit)
    {
        boolean uploaded = gprsUploadBuffer();

        if (!uploaded)
        {
            for (int i = 0; i < txBufferCount; i++)
            {
                archiveToStorage(txBufferHeight[i], txBufferRate[i],
                                 txBufferTime[i], currentZone,
                                 currentResponseLevel);
            }

            Serial.println("TX:START");
            for (int i = 0; i < txBufferCount; i++)
            {
                char hStr[10], rStr[10], bStr[6];
                floatToStr(hStr, sizeof(hStr), txBufferHeight[i], 2);
                floatToStr(rStr, sizeof(rStr), txBufferRate[i], 3);
                floatToStr(bStr, sizeof(bStr), batteryPercent, 1);

                Serial.print("TX:");
                Serial.print(txBufferTime[i]);
                Serial.print(",");
                Serial.print(hStr);
                Serial.print(",");
                Serial.print(rStr);
                Serial.print(",");
                Serial.print(currentZone);
                Serial.print(",");
                Serial.print(currentResponseLevel);
                Serial.print(",");
                Serial.println(bStr);
            }
            Serial.println("TX:END");
        }
    }
    else
    {
        if (currentResponseLevel >= RESP_WARNING)
        {
            dispatchAlerts(currentResponseLevel, currentHeight, ratePer15Min,
                           currentZone, sustainedRise, false);
        }
        else
        {
            for (int i = 0; i < txBufferCount; i++)
            {
                archiveToStorage(txBufferHeight[i], txBufferRate[i],
                                 txBufferTime[i], currentZone,
                                 currentResponseLevel);
            }
        }
    }

    txBufferCount = 0;
}
void parseFloodCommand(const char *cmd)
{
    if (strncmp(cmd, "SETALERT:", 9) == 0)
    {
        float val = atof(cmd + 9);
        if (val > 0 && val < 10000 && val < warningLevelCm)
        {
            alertLevelCm = val;
            char vStr[10];
            floatToStr(vStr, sizeof(vStr), alertLevelCm, 1);
            Serial.print("STATUS:ALERT_LEVEL_SET=");
            Serial.println(vStr);
            debugSendStatusEvent("ALERT_LEVEL_CHANGED");
            forceSaveEeprom();
        }
        else
        {
            Serial.println("ERROR:INVALID_ALERT_LEVEL");
        }
    }
    else if (strncmp(cmd, "SETWARN:", 8) == 0)
    {
        float val = atof(cmd + 8);
        if (val > alertLevelCm && val < dangerLevelCm)
        {
            warningLevelCm = val;
            char vStr[10];
            floatToStr(vStr, sizeof(vStr), warningLevelCm, 1);
            Serial.print("STATUS:WARNING_LEVEL_SET=");
            Serial.println(vStr);
            debugSendStatusEvent("WARNING_LEVEL_CHANGED");
            forceSaveEeprom();
        }
        else
        {
            Serial.println("ERROR:INVALID_WARNING_LEVEL");
        }
    }
    else if (strncmp(cmd, "SETDANGER:", 10) == 0)
    {
        float val = atof(cmd + 10);
        if (val > warningLevelCm && val < 10000)
        {
            dangerLevelCm = val;
            char vStr[10];
            floatToStr(vStr, sizeof(vStr), dangerLevelCm, 1);
            Serial.print("STATUS:DANGER_LEVEL_SET=");
            Serial.println(vStr);
            debugSendStatusEvent("DANGER_LEVEL_CHANGED");
            forceSaveEeprom();
        }
        else
        {
            Serial.println("ERROR:INVALID_DANGER_LEVEL");
        }
    }
    else if (strncmp(cmd, "SETTHRESH:", 10) == 0)
    {
        const char *params = cmd + 10;
        const char *comma1 = strchr(params, ',');
        if (comma1 != NULL)
        {
            const char *comma2 = strchr(comma1 + 1, ',');
            if (comma2 != NULL)
            {
                float a = atof(params);
                float w = atof(comma1 + 1);
                float d = atof(comma2 + 1);

                if (a > 0 && w > a && d > w && d < 10000)
                {
                    alertLevelCm = a;
                    warningLevelCm = w;
                    dangerLevelCm = d;

                    char aStr[10], wStr[10], dStr[10];
                    floatToStr(aStr, sizeof(aStr), alertLevelCm, 1);
                    floatToStr(wStr, sizeof(wStr), warningLevelCm, 1);
                    floatToStr(dStr, sizeof(dStr), dangerLevelCm, 1);
                    Serial.print("STATUS:THRESHOLDS_SET=");
                    Serial.print(aStr);
                    Serial.print(",");
                    Serial.print(wStr);
                    Serial.print(",");
                    Serial.println(dStr);
                    debugSendStatusEvent("THRESHOLDS_CHANGED");
                    forceSaveEeprom();
                }
                else
                {
                    Serial.println("ERROR:INVALID_THRESHOLDS_MUST_BE_ASCENDING");
                }
            }
            else
            {
                Serial.println("ERROR:INVALID_SETTHRESH_FORMAT");
            }
        }
        else
        {
            Serial.println("ERROR:INVALID_SETTHRESH_FORMAT");
        }
    }
    else if (strncmp(cmd, "ADDTIER1:", 9) == 0)
    {
        if (tier1Count < MAX_CONTACTS_PER_TIER)
        {
            const char *phone = cmd + 9;
            int pLen = strlen(phone);
            if (pLen > 3 && pLen < PHONE_NUMBER_LENGTH)
            {
                if (contactExists(tier1Contacts, tier1Count, phone))
                {
                    Serial.println("STATUS:TIER1_DUPLICATE_REJECTED");
                }
                else
                {
                    strncpy(tier1Contacts[tier1Count], phone,
                            PHONE_NUMBER_LENGTH - 1);
                    tier1Contacts[tier1Count][PHONE_NUMBER_LENGTH - 1] = '\0';
                    tier1Count++;
                    saveContactsToEeprom();
                    Serial.print("STATUS:TIER1_ADDED=");
                    Serial.print(phone);
                    Serial.print(" COUNT=");
                    Serial.println(tier1Count);
                }
            }
        }
        else
        {
            Serial.println("ERROR:TIER1_FULL");
        }
    }
    else if (strncmp(cmd, "ADDTIER2:", 9) == 0)
    {
        if (tier2Count < MAX_CONTACTS_PER_TIER)
        {
            const char *phone = cmd + 9;
            int pLen = strlen(phone);
            if (pLen > 3 && pLen < PHONE_NUMBER_LENGTH)
            {
                if (contactExists(tier2Contacts, tier2Count, phone))
                {
                    Serial.println("STATUS:TIER2_DUPLICATE_REJECTED");
                }
                else
                {
                    strncpy(tier2Contacts[tier2Count], phone,
                            PHONE_NUMBER_LENGTH - 1);
                    tier2Contacts[tier2Count][PHONE_NUMBER_LENGTH - 1] = '\0';
                    tier2Count++;
                    saveContactsToEeprom();
                    Serial.print("STATUS:TIER2_ADDED=");
                    Serial.print(phone);
                    Serial.print(" COUNT=");
                    Serial.println(tier2Count);
                }
            }
        }
        else
        {
            Serial.println("ERROR:TIER2_FULL");
        }
    }
    else if (strncmp(cmd, "ADDTIER3:", 9) == 0)
    {
        if (tier3Count < MAX_CONTACTS_PER_TIER)
        {
            const char *phone = cmd + 9;
            int pLen = strlen(phone);
            if (pLen > 3 && pLen < PHONE_NUMBER_LENGTH)
            {
                if (contactExists(tier3Contacts, tier3Count, phone))
                {
                    Serial.println("STATUS:TIER3_DUPLICATE_REJECTED");
                }
                else
                {
                    strncpy(tier3Contacts[tier3Count], phone,
                            PHONE_NUMBER_LENGTH - 1);
                    tier3Contacts[tier3Count][PHONE_NUMBER_LENGTH - 1] = '\0';
                    tier3Count++;
                    saveContactsToEeprom();
                    Serial.print("STATUS:TIER3_ADDED=");
                    Serial.print(phone);
                    Serial.print(" COUNT=");
                    Serial.println(tier3Count);
                }
            }
        }
        else
        {
            Serial.println("ERROR:TIER3_FULL");
        }
    }
    else if (strcmp(cmd, "CLEARTIERS") == 0)
    {
        tier1Count = 0;
        tier2Count = 0;
        tier3Count = 0;
        memset(tier1Contacts, 0, sizeof(tier1Contacts));
        memset(tier2Contacts, 0, sizeof(tier2Contacts));
        memset(tier3Contacts, 0, sizeof(tier3Contacts));
        saveContactsToEeprom();
        Serial.println("STATUS:ALL_TIERS_CLEARED");
        debugSendStatusEvent("ALL_TIERS_CLEARED");
    }
    else if (strcmp(cmd, "FLOODSTATUS") == 0)
    {
        Serial.print("STATUS:ZONE=");
        Serial.println(zoneNameStr(currentZone));
        Serial.print("STATUS:RATE_CAT=");
        Serial.println(currentRateCategory);

        char rStr[10];
        floatToStr(rStr, sizeof(rStr), ratePer15Min, 3);
        Serial.print("STATUS:RATE_PER15=");
        Serial.println(rStr);

        Serial.print("STATUS:SUSTAINED=");
        Serial.println(sustainedRise ? "YES" : "NO");
        Serial.print("STATUS:RESPONSE_LEVEL=");
        Serial.println(responseNameStr(currentResponseLevel));

        char aStr[10], wStr[10], dStr[10];
        floatToStr(aStr, sizeof(aStr), alertLevelCm, 1);
        floatToStr(wStr, sizeof(wStr), warningLevelCm, 1);
        floatToStr(dStr, sizeof(dStr), dangerLevelCm, 1);
        Serial.print("STATUS:ALERT_LVL=");
        Serial.println(aStr);
        Serial.print("STATUS:WARN_LVL=");
        Serial.println(wStr);
        Serial.print("STATUS:DANGER_LVL=");
        Serial.println(dStr);

        char bStr[10];
        floatToStr(bStr, sizeof(bStr), batteryPercent, 1);
        Serial.print("STATUS:BATTERY=");
        Serial.println(bStr);

        Serial.print("STATUS:SAMPLE_INTERVAL_MS=");
        Serial.println(sampleIntervalMs);
        Serial.print("STATUS:TRANSMIT_INTERVAL_MS=");
        Serial.println(transmitIntervalMs);
        Serial.print("STATUS:GPRS_MODE=");
        Serial.println(useGprsTransmit ? "ON" : "OFF");
        Serial.print("STATUS:READINGS_SINCE_BOOT=");
        Serial.println(readingsSinceBoot);
        Serial.print("STATUS:TIER1_COUNT=");
        Serial.println(tier1Count);
        Serial.print("STATUS:TIER2_COUNT=");
        Serial.println(tier2Count);
        Serial.print("STATUS:TIER3_COUNT=");
        Serial.println(tier3Count);

        char pStr[10];
        floatToStr(pStr, sizeof(pStr), peakHeight, 2);
        Serial.print("STATUS:PEAK_HEIGHT=");
        Serial.println(pStr);

        Serial.print("STATUS:STEPDOWN_CONSEC=");
        Serial.println(stepDownConsecutive);

        Serial.print("STATUS:OBLIGHT_ON=");
        Serial.print(obOnTimeMs);
        Serial.print("ms_OFF=");
        Serial.print(obOffTimeMs);
        Serial.print("ms_EN=");
        Serial.print(obLightEnabled ? 1 : 0);
        Serial.print("_PRESET=");
        Serial.println(currentObPresetName);

        Serial.print("STATUS:DEBUG_ENABLED=");
        Serial.println(debugEnabled ? 1 : 0);

        Serial.print("STATUS:ALGORITHM_ENABLED=");
        Serial.println(algorithmEnabled ? 1 : 0);
    }
    else if (strcmp(cmd, "RESETFLOOD") == 0)
    {
        currentResponseLevel = RESP_NORMAL;
        previousResponseLevel = RESP_NORMAL;
        currentZone = ZONE_NORMAL;
        currentRateCategory = RATE_SLOW;
        sustainedRise = false;
        sustainedBufCount = 0;
        sustainedBufIdx = 0;
        stepDownConsecutive = 0;
        highestResponseReached = RESP_NORMAL;
        allClearPending = false;
        floodAlertLevel = 0;
        stateEntryTime = 0;
        readingsSinceBoot = 0;
        resetAckTracking();
        Serial.println("STATUS:FLOOD_STATE_RESET_TO_NORMAL");
        debugSendStatusEvent("FLOOD_STATE_RESET");
        forceSaveEeprom();
    }
}

void logReading(float height, float thetaVal, float press, uint32_t timestamp)
{
    logHeight[logIndex] = height;
    logTheta[logIndex] = thetaVal;
    logPressure[logIndex] = press;
    logTime[logIndex] = timestamp;
    logIndex = (logIndex + 1) % LOG_SIZE;
    if (logCount < LOG_SIZE)
    {
        logCount++;
    }

    if (height > peakHeight)
    {
        peakHeight = height;
        peakTime = timestamp;
    }
    if (height < minHeight)
    {
        minHeight = height;
        minTime = timestamp;
    }
}

void dumpLog()
{
    Serial.println("LOG:START");
    Serial.println("LOG:UNIX,DATE_TIME,THETA,HEIGHT,PRESSURE,LAT,LON");

    int count = logCount < LOG_SIZE ? logCount : LOG_SIZE;
    int start = (logCount >= LOG_SIZE) ? logIndex : 0;

    for (int i = 0; i < count; i++)
    {
        esp_task_wdt_reset();

        int idx = (start + i) % LOG_SIZE;
        uint16_t y;
        uint8_t mo, d, h, mi, s, dow;
        unixToDate(logTime[idx], &y, &mo, &d, &h, &mi, &s, &dow);

        char tStr[10], hStr[10], pStr[10];
        char latStr[12], lonStr[12];
        floatToStr(tStr, sizeof(tStr), logTheta[idx], 2);
        floatToStr(hStr, sizeof(hStr), logHeight[idx], 2);
        floatToStr(pStr, sizeof(pStr), logPressure[idx], 2);
        floatToStr(latStr, sizeof(latStr), gpsLat, 6);
        floatToStr(lonStr, sizeof(lonStr), gpsLon, 6);

        Serial.print("LOG:");
        Serial.print(logTime[idx]);
        Serial.print(",");
        Serial.print(y);
        Serial.print("-");
        if (mo < 10) Serial.print("0");
        Serial.print(mo);
        Serial.print("-");
        if (d < 10) Serial.print("0");
        Serial.print(d);
        Serial.print(" ");
        if (h < 10) Serial.print("0");
        Serial.print(h);
        Serial.print(":");
        if (mi < 10) Serial.print("0");
        Serial.print(mi);
        Serial.print(":");
        if (s < 10) Serial.print("0");
        Serial.print(s);
        Serial.print(",");
        Serial.print(tStr);
        Serial.print(",");
        Serial.print(hStr);
        Serial.print(",");
        Serial.print(pStr);
        Serial.print(",");
        Serial.print(latStr);
        Serial.print(",");
        Serial.println(lonStr);
    }

    char pkStr[10];
    floatToStr(pkStr, sizeof(pkStr), peakHeight, 2);
    Serial.print("LOG:PEAK_HEIGHT=");
    Serial.print(pkStr);
    Serial.print(",PEAK_TIME=");
    Serial.println(peakTime);

    float dispMin = minHeight > 9000 ? 0.0 : minHeight;
    char mnStr[10];
    floatToStr(mnStr, sizeof(mnStr), dispMin, 2);
    Serial.print("LOG:MIN_HEIGHT=");
    Serial.print(mnStr);
    Serial.print(",MIN_TIME=");
    Serial.println(minTime);

    Serial.print("LOG:TOTAL_READINGS=");
    Serial.println(logCount);
    Serial.println("LOG:END");
}

void mpuWriteReg(uint8_t reg, uint8_t val)
{
    I2C_BUS0.beginTransmission(MPU6050_ADDR);
    I2C_BUS0.write(reg);
    I2C_BUS0.write(val);
    I2C_BUS0.endTransmission();
}

uint8_t mpuReadReg(uint8_t reg)
{
    I2C_BUS0.beginTransmission(MPU6050_ADDR);
    I2C_BUS0.write(reg);
    I2C_BUS0.endTransmission(false);
    I2C_BUS0.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
    return I2C_BUS0.read();
}

void readMPU()
{
    I2C_BUS0.beginTransmission(MPU6050_ADDR);
    I2C_BUS0.write(0x3B);
    uint8_t err = I2C_BUS0.endTransmission(false);

    if (err != 0)
    {
        mpuConsecutiveFailures++;
        return;
    }

    uint8_t bytesReceived = I2C_BUS0.requestFrom((uint8_t)MPU6050_ADDR,
                                                   (uint8_t)14);

    if (bytesReceived != 14)
    {
        mpuConsecutiveFailures++;
        return;
    }

    uint8_t hi, lo;

    hi = I2C_BUS0.read();
    lo = I2C_BUS0.read();
    accX = (int16_t)((hi << 8) | lo);

    hi = I2C_BUS0.read();
    lo = I2C_BUS0.read();
    accY = (int16_t)((hi << 8) | lo);

    hi = I2C_BUS0.read();
    lo = I2C_BUS0.read();
    accZ = (int16_t)((hi << 8) | lo);

    I2C_BUS0.read();
    I2C_BUS0.read();

    hi = I2C_BUS0.read();
    lo = I2C_BUS0.read();
    gyroX = (int16_t)((hi << 8) | lo);

    hi = I2C_BUS0.read();
    lo = I2C_BUS0.read();
    gyroY = (int16_t)((hi << 8) | lo);

    hi = I2C_BUS0.read();
    lo = I2C_BUS0.read();
    gyroZ = (int16_t)((hi << 8) | lo);

    mpuConsecutiveFailures = 0;
}

float calculateWaterHeight(float thetaDeg)
{
    float thetaRad = fabs(thetaDeg) * PI / 180.0;

    if (thetaRad < 0.001)
    {
        return 0;
    }

    float height = olpLength * sin(thetaRad);

    if (heightInverted)
    {
        height = -height;
    }

    if (height < 0)
    {
        height = 0;
    }

    return height;
}

void hcsr04Init()
{
    pinMode(HCSR04_TRIG_PIN, OUTPUT);
    pinMode(HCSR04_ECHO_PIN, INPUT);
    digitalWrite(HCSR04_TRIG_PIN, LOW);
    delay(50);

    hcsr04Available = false;
    for (int attempt = 0; attempt < 5; attempt++)
    {
        float testDist = hcsr04ReadDistance();
        if (testDist > 0 && testDist < 500)
        {
            hcsr04Available = true;
            break;
        }
        delay(100);
    }

    Serial.print("STATUS:HCSR04=");
    Serial.println(hcsr04Available ? "OK" : "NOT_FOUND");
    debugSendSensorInit("HCSR04", hcsr04Available);
}

float hcsr04ReadDistance()
{
    digitalWrite(HCSR04_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(HCSR04_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(HCSR04_TRIG_PIN, LOW);

    unsigned long duration = pulseIn(HCSR04_ECHO_PIN, HIGH, 30000);

    if (duration == 0)
    {
        return -1;
    }

    float distance = duration * 0.0343 / 2.0;
    return distance;
}

float hcsr04GetWaterHeight()
{
    float readings[3];
    int validCount = 0;

    for (int i = 0; i < 3; i++)
    {
        float d = hcsr04ReadDistance();
        if (d > 0 && d < 500)
        {
            readings[validCount++] = d;
        }
        delay(30);
        readGPS();
    }

    if (validCount == 0)
    {
        return -1;
    }

    if (validCount >= 3)
    {
        for (int i = 0; i < validCount - 1; i++)
        {
            for (int j = i + 1; j < validCount; j++)
            {
                if (readings[j] < readings[i])
                {
                    float tmp = readings[i];
                    readings[i] = readings[j];
                    readings[j] = tmp;
                }
            }
        }
        float distance = readings[1];
        float height = hcsr04MountHeight - distance;
        if (height < 0)
        {
            height = 0;
        }
        return height;
    }
    else
    {
        float avgDist = 0;
        for (int i = 0; i < validCount; i++)
        {
            avgDist += readings[i];
        }
        avgDist /= validCount;
        float height = hcsr04MountHeight - avgDist;
        if (height < 0)
        {
            height = 0;
        }
        return height;
    }
}

void i2cBusRecovery(int sdaPin, int sclPin)
{
    Serial.print("STATUS:I2C_BUS_RECOVERY_SDA=");
    Serial.print(sdaPin);
    Serial.print("_SCL=");
    Serial.println(sclPin);

    pinMode(sdaPin, INPUT_PULLUP);
    pinMode(sclPin, OUTPUT);

    for (int i = 0; i < 9; i++)
    {
        digitalWrite(sclPin, LOW);
        delayMicroseconds(5);
        digitalWrite(sclPin, HIGH);
        delayMicroseconds(5);
    }

    pinMode(sdaPin, OUTPUT);
    digitalWrite(sdaPin, LOW);
    delayMicroseconds(5);
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(sdaPin, HIGH);
    delayMicroseconds(5);

    Serial.println("STATUS:I2C_BUS_RECOVERY_DONE");
}

boolean i2cHealthCheck(TwoWire *bus, uint8_t addr, int sdaPin, int sclPin)
{
    bus->beginTransmission(addr);
    uint8_t err = bus->endTransmission();

    if (err == 0)
    {
        return true;
    }

    Serial.print("STATUS:I2C_ERROR=");
    Serial.print(err);
    Serial.print("_ADDR=0x");
    Serial.println(addr, HEX);

    bus->end();

    i2cBusRecovery(sdaPin, sclPin);

    bus->begin(sdaPin, sclPin, 100000);

    bus->beginTransmission(addr);
    err = bus->endTransmission();
    return (err == 0);
}

void checkSensorHealth()
{
    boolean bus0Healthy = true;

    if (activeSensor == SENSOR_MPU6050)
    {
        I2C_BUS0.beginTransmission(MPU6050_ADDR);
        uint8_t mpuErr = I2C_BUS0.endTransmission();

        if (mpuErr != 0)
        {
            bus0Healthy = false;
            mpuConsecutiveFailures++;
            Serial.print("WARNING:MPU6050_HEALTH_FAIL_COUNT=");
            Serial.println(mpuConsecutiveFailures);
        }
        else
        {
            mpuConsecutiveFailures = 0;
        }
    }

    if (bmpAvailable)
    {
        I2C_BUS1.beginTransmission(BMP280_ADDR);
        uint8_t bmpErr = I2C_BUS1.endTransmission();

        if (bmpErr != 0)
        {
            Serial.println("WARNING:BMP280_HEALTH_FAIL");
        }
    }

    if (!bus0Healthy)
    {
        I2C_BUS0.end();
        i2cBusRecovery(BUS0_SDA, BUS0_SCL);
        I2C_BUS0.begin(BUS0_SDA, BUS0_SCL, 100000);

        debugSendStatusEvent("I2C_BUS0_RECOVERY");

        if (activeSensor == SENSOR_MPU6050)
        {
            I2C_BUS0.beginTransmission(MPU6050_ADDR);
            uint8_t mpuErr2 = I2C_BUS0.endTransmission();

            if (mpuErr2 != 0)
            {
                mpuConsecutiveFailures++;
                if (mpuConsecutiveFailures >= MPU_FAIL_THRESHOLD)
                {
                    mpuHealthy = false;
                    if (hcsr04Available)
                    {
                        activeSensor = SENSOR_HCSR04;
                        Serial.println("ERROR:MPU6050_OFFLINE_SWITCHING_HCSR04");
                        debugSendStatusEvent("MPU6050_OFFLINE_HCSR04_ACTIVE");
                    }
                    else
                    {
                        activeSensor = SENSOR_NONE;
                        Serial.println("ERROR:MPU6050_OFFLINE_NO_BACKUP");
                        debugSendStatusEvent("MPU6050_OFFLINE_NO_BACKUP");
                    }
                }
            }
            else
            {
                mpuConsecutiveFailures = 0;
            }
        }

        if (bmpAvailable)
        {
            I2C_BUS1.beginTransmission(BMP280_ADDR);
            uint8_t bmpErr2 = I2C_BUS1.endTransmission();

            if (bmpErr2 != 0)
            {
                bmpAvailable = false;
                Serial.println("WARNING:BMP280_OFFLINE");
                debugSendStatusEvent("BMP280_OFFLINE");
            }
        }
    }

    if (activeSensor == SENSOR_HCSR04 && !mpuHealthy)
    {
        I2C_BUS0.beginTransmission(MPU6050_ADDR);
        uint8_t mpuRecoveryErr = I2C_BUS0.endTransmission();

        if (mpuRecoveryErr == 0)
        {
            mpuWriteReg(0x6B, 0x00);
            delay(100);
            mpuConsecutiveFailures = 0;
            mpuHealthy = true;
            activeSensor = SENSOR_MPU6050;
            Serial.println("STATUS:MPU6050_RECOVERED_SWITCHING_BACK");
            debugSendStatusEvent("MPU6050_RECOVERED");
            recalibrate();
        }
    }

    if (gpsFixValid)
    {
        lastGpsFixTime = millis();
    }
    else if (lastGpsFixTime > 0 && millis() - lastGpsFixTime > 3600000UL)
    {
        Serial.println("WARNING:GPS_NO_FIX_1HR");
        debugSendStatusEvent("GPS_NO_FIX_1HR");
    }
}

uint8_t bmpRead8(uint8_t reg)
{
    I2C_BUS1.beginTransmission(BMP280_ADDR);
    I2C_BUS1.write(reg);
    I2C_BUS1.endTransmission(false);
    I2C_BUS1.requestFrom((uint8_t)BMP280_ADDR, (uint8_t)1);
    return I2C_BUS1.read();
}

uint16_t bmpRead16LE(uint8_t reg)
{
    I2C_BUS1.beginTransmission(BMP280_ADDR);
    I2C_BUS1.write(reg);
    I2C_BUS1.endTransmission(false);
    I2C_BUS1.requestFrom((uint8_t)BMP280_ADDR, (uint8_t)2);
    uint16_t val = I2C_BUS1.read();
    val |= (uint16_t)I2C_BUS1.read() << 8;
    return val;
}

int16_t bmpReadS16LE(uint8_t reg)
{
    return (int16_t)bmpRead16LE(reg);
}

void bmpWrite8(uint8_t reg, uint8_t val)
{
    I2C_BUS1.beginTransmission(BMP280_ADDR);
    I2C_BUS1.write(reg);
    I2C_BUS1.write(val);
    I2C_BUS1.endTransmission();
}

boolean initBMP280()
{
    I2C_BUS1.beginTransmission(BMP280_ADDR);
    uint8_t err = I2C_BUS1.endTransmission();
    if (err != 0)
    {
        return false;
    }

    uint8_t chipId = bmpRead8(0xD0);
    Serial.print("STATUS:BMP_CHIP_ID=0x");
    Serial.println(chipId, HEX);

    if (chipId != 0x58 && chipId != 0x56 && chipId != 0x57 && chipId != 0x60)
    {
        return false;
    }

    bmpDigT1 = bmpRead16LE(0x88);
    bmpDigT2 = bmpReadS16LE(0x8A);
    bmpDigT3 = bmpReadS16LE(0x8C);
    bmpDigP1 = bmpRead16LE(0x8E);
    bmpDigP2 = bmpReadS16LE(0x90);
    bmpDigP3 = bmpReadS16LE(0x92);
    bmpDigP4 = bmpReadS16LE(0x94);
    bmpDigP5 = bmpReadS16LE(0x96);
    bmpDigP6 = bmpReadS16LE(0x98);
    bmpDigP7 = bmpReadS16LE(0x9A);
    bmpDigP8 = bmpReadS16LE(0x9C);
    bmpDigP9 = bmpReadS16LE(0x9E);

    bmpWrite8(0xF4, 0x57);
    bmpWrite8(0xF5, 0x10);

    delay(100);

    return true;
}

void bmpReadData(float *temperature, float *pressure)
{
    I2C_BUS1.beginTransmission(BMP280_ADDR);
    I2C_BUS1.write(0xF7);
    uint8_t txErr = I2C_BUS1.endTransmission(false);

    if (txErr != 0)
    {
        *temperature = 0;
        *pressure = 0;
        return;
    }

    uint8_t bytesReceived = I2C_BUS1.requestFrom((uint8_t)BMP280_ADDR, (uint8_t)6);

    if (bytesReceived != 6)
    {
        *temperature = 0;
        *pressure = 0;
        return;
    }

    uint8_t b0 = I2C_BUS1.read();
    uint8_t b1 = I2C_BUS1.read();
    uint8_t b2 = I2C_BUS1.read();
    uint8_t b3 = I2C_BUS1.read();
    uint8_t b4 = I2C_BUS1.read();
    uint8_t b5 = I2C_BUS1.read();

    uint32_t pressRaw = ((uint32_t)b0 << 12) | ((uint32_t)b1 << 4) | ((uint32_t)b2 >> 4);
    uint32_t tempRaw  = ((uint32_t)b3 << 12) | ((uint32_t)b4 << 4) | ((uint32_t)b5 >> 4);

    int32_t var1, var2;

    var1 = ((((int32_t)tempRaw >> 3) - ((int32_t)bmpDigT1 << 1)) *
            ((int32_t)bmpDigT2)) >> 11;
    var2 = ((((((int32_t)tempRaw >> 4) - ((int32_t)bmpDigT1)) *
              (((int32_t)tempRaw >> 4) - ((int32_t)bmpDigT1))) >> 12) *
            ((int32_t)bmpDigT3)) >> 14;
    bmpTFine = var1 + var2;

    *temperature = (float)((bmpTFine * 5 + 128) >> 8) / 100.0;

    int64_t p_var1, p_var2, p;

    p_var1 = ((int64_t)bmpTFine) - 128000;
    p_var2 = p_var1 * p_var1 * (int64_t)bmpDigP6;
    p_var2 = p_var2 + ((p_var1 * (int64_t)bmpDigP5) << 17);
    p_var2 = p_var2 + (((int64_t)bmpDigP4) << 35);
    p_var1 = ((p_var1 * p_var1 * (int64_t)bmpDigP3) >> 8) +
             ((p_var1 * (int64_t)bmpDigP2) << 12);
    p_var1 = (((((int64_t)1) << 47) + p_var1)) * ((int64_t)bmpDigP1) >> 33;

    if (p_var1 == 0)
    {
        *pressure = 0;
        return;
    }

    p = 1048576 - (int64_t)pressRaw;
    p = (((p << 31) - p_var2) * 3125) / p_var1;
    p_var1 = (((int64_t)bmpDigP9) * (p >> 13) * (p >> 13)) >> 25;
    p_var2 = (((int64_t)bmpDigP8) * p) >> 19;
    p = ((p + p_var1 + p_var2) >> 8) + (((int64_t)bmpDigP7) << 4);

    *pressure = (float)((uint32_t)p) / 25600.0;
}

void updateBaseline(float press)
{
    if (press < 300 || press > 1200)
    {
        return;
    }

    if (baselineCount >= BASELINE_SIZE)
    {
        baselineSum -= baselineBuffer[baselineIndex];
    }

    baselineBuffer[baselineIndex] = press;
    baselineSum += press;
    baselineIndex = (baselineIndex + 1) % BASELINE_SIZE;
    if (baselineCount < BASELINE_SIZE)
    {
        baselineCount++;
    }

    baselinePressure = baselineSum / baselineCount;
}

int evaluateSubmersion(float press, float base)
{
    if (press < 300 || base < 300)
    {
        return -1;
    }

    float dev = press - base;
    pressureDeviation = dev;

    if (dev > 15)
    {
        estimatedDepth = dev / 0.981;
        return 3;
    }
    else if (dev > 8)
    {
        estimatedDepth = dev / 0.981;
        return 2;
    }
    else if (dev > 5)
    {
        estimatedDepth = 0;
        return 1;
    }
    else
    {
        estimatedDepth = 0;
        return 0;
    }
}

void checkSerialInput()
{
    while (Serial.available())
    {
        char c = Serial.read();
        if (c == '\n' || c == '\r')
        {
            if (inputBufferIdx > 0)
            {
                inputBuffer[inputBufferIdx] = '\0';

                char *trimmed = inputBuffer;
                while (*trimmed == ' ')
                {
                    trimmed++;
                }
                int len = strlen(trimmed);
                while (len > 0 && trimmed[len - 1] == ' ')
                {
                    trimmed[--len] = '\0';
                }

                processCommand(trimmed);
            }
            inputBufferIdx = 0;
        }
        else
        {
            if (inputBufferIdx < INPUT_BUFFER_SIZE - 1)
            {
                inputBuffer[inputBufferIdx++] = c;
            }
        }
    }
}

void processCommand(const char *cmd)
{
    if (strcmp(cmd, "ALGOON") == 0)
    {
        setAlgorithmEnabled(true);
    }
    else if (strcmp(cmd, "ALGOOFF") == 0)
    {
        setAlgorithmEnabled(false);
    }
    else if (strcmp(cmd, "ALGOSTATUS") == 0)
    {
        Serial.print("STATUS:ALGORITHM_ENABLED=");
        Serial.println(algorithmEnabled ? 1 : 0);
        Serial.print("STATUS:ALGORITHM_MODE=");
        Serial.println(algorithmEnabled ? "ADAPTIVE" : "CONTINUOUS");
    }
    else if (strncmp(cmd, "OLP:", 4) == 0)
    {
        float val = atof(cmd + 4);
        if (val > 0 && val < 10000)
        {
            olpLength = val;
            forceSaveEeprom();
            char vStr[10];
            floatToStr(vStr, sizeof(vStr), olpLength, 1);
            Serial.print("STATUS:OLP_SET=");
            Serial.println(vStr);
            debugSendStatusEvent("OLP_CHANGED");
        }
    }
    else if (strcmp(cmd, "CAL") == 0)
    {
        recalibrate();
    }
    else if (strcmp(cmd, "DUMP") == 0)
    {
        dumpLog();
    }
    else if (strncmp(cmd, "TIME:", 5) == 0)
    {
        uint32_t ts = strtoul(cmd + 5, NULL, 10);
        if (ts > 1700000000UL)
        {
            rtcSetUnixTime(ts);
            rtcReadRaw();
            rtcTimeValid = rtcIsTimeValid();
            currentUnixTime = rtcGetUnixTime();
            Serial.print("STATUS:RTC_SET=");
            Serial.println(ts);
            debugSendStatusEvent("RTC_TIME_SET");
        }
    }
    else if (strncmp(cmd, "SETDATE:", 8) == 0)
    {
        int yr = 0, mo = 0, dy = 0, hr = 0, mi = 0, sc = 0;
        int parsed = sscanf(cmd, "SETDATE:%d-%d-%d %d:%d:%d",
                            &yr, &mo, &dy, &hr, &mi, &sc);
        if (parsed == 6 && yr >= 2024 && mo >= 1 && mo <= 12 &&
                dy >= 1 && dy <= 31)
        {
            rtcSetDateTime(yr, mo, dy, hr, mi, sc);
            rtcReadRaw();
            rtcTimeValid = rtcIsTimeValid();
            currentUnixTime = rtcGetUnixTime();
            Serial.print("STATUS:RTC_SET_DATE=");
            Serial.print(yr);
            Serial.print("-");
            Serial.print(mo);
            Serial.print("-");
            Serial.print(dy);
            Serial.print(" ");
            Serial.print(hr);
            Serial.print(":");
            Serial.print(mi);
            Serial.print(":");
            Serial.println(sc);
            debugSendStatusEvent("RTC_DATE_SET");
        }
    }
    else if (strncmp(cmd, "SMS:", 4) == 0)
    {
        const char *rest = cmd + 4;
        const char *colon = strchr(rest, ':');
        if (colon != NULL && (colon - rest) > 0)
        {
            int phoneLen = colon - rest;
            if (phoneLen < PHONE_NUMBER_LENGTH)
            {
                char phone[PHONE_NUMBER_LENGTH];
                strncpy(phone, rest, phoneLen);
                phone[phoneLen] = '\0';
                const char *msg = colon + 1;
                Serial.print("STATUS:SMS_SENDING_TO=");
                Serial.println(phone);
                queueSMS(phone, msg);
            }
        }
    }
    else if (strcmp(cmd, "SIMSTATUS") == 0)
    {
        Serial.print("STATUS:SIM_AVAILABLE=");
        Serial.println(simAvailable ? 1 : 0);
        Serial.print("STATUS:SIM_REGISTERED=");
        Serial.println(simRegistered ? 1 : 0);
        Serial.print("STATUS:SIM_READY=");
        Serial.println(simReady ? 1 : 0);
        Serial.print("STATUS:SIM_RSSI=");
        Serial.println(simSignalRSSI);
        Serial.print("STATUS:SIM_FAIL_CODE=");
        Serial.println(simInitFailCode);
    }
    else if (strcmp(cmd, "SIMREINIT") == 0)
    {
        simFullInit();
    }
    else if (strcmp(cmd, "SIMSIGNAL") == 0)
    {
        int rssi = simGetSignalStrength();
        Serial.print("SIGNAL:");
        Serial.println(rssi >= 0 ? rssi : 0);
    }
    else if (strcmp(cmd, "SIMRESET") == 0)
    {
        simHardwareReset();
        Serial.println("STATUS:SIM800L_RESET_DONE");
    }
    else if (strncmp(cmd, "ADDAUTH:", 8) == 0)
    {
        if (authorizedCount < MAX_AUTHORIZED)
        {
            const char *phone = cmd + 8;
            int pLen = strlen(phone);
            if (pLen > 3 && pLen < PHONE_NUMBER_LENGTH)
            {
                strncpy(authorizedNumbers[authorizedCount], phone,
                        PHONE_NUMBER_LENGTH - 1);
                authorizedNumbers[authorizedCount][PHONE_NUMBER_LENGTH - 1] = '\0';
                authorizedCount++;
                saveContactsToEeprom();
                Serial.print("STATUS:AUTH_ADDED=");
                Serial.println(phone);
            }
        }
    }
    else if (strcmp(cmd, "INVERTHEIGHT") == 0)
    {
        heightInverted = !heightInverted;
        Serial.print("STATUS:HEIGHT_INVERTED=");
        Serial.println(heightInverted ? "YES" : "NO");
    }
    else if (strcmp(cmd, "SERIALOFF") == 0)
    {
        serialOutputEnabled = false;
        Serial.println("STATUS:SERIAL_OUTPUT_DISABLED");
    }
    else if (strcmp(cmd, "SERIALON") == 0)
    {
        serialOutputEnabled = true;
        Serial.println("STATUS:SERIAL_OUTPUT_ENABLED");
    }
    else if (strncmp(cmd, "MOUNT:", 6) == 0)
    {
        float val = atof(cmd + 6);
        if (val > 0 && val < 10000)
        {
            hcsr04MountHeight = val;
            char vStr[10];
            floatToStr(vStr, sizeof(vStr), val, 1);
            Serial.print("STATUS:MOUNT_HEIGHT_SET=");
            Serial.println(vStr);
        }
    }
    else if (strncmp(cmd, "APN:", 4) == 0)
    {
        strncpy(gprsApn, cmd + 4, sizeof(gprsApn) - 1);
        gprsApn[sizeof(gprsApn) - 1] = '\0';
        Serial.print("STATUS:APN_SET=");
        Serial.println(gprsApn);
    }
    else if (strncmp(cmd, "SERVER:", 7) == 0)
    {
        strncpy(gprsServer, cmd + 7, sizeof(gprsServer) - 1);
        gprsServer[sizeof(gprsServer) - 1] = '\0';
        Serial.print("STATUS:SERVER_SET=");
        Serial.println(gprsServer);
    }
    else if (strcmp(cmd, "GPRSINIT") == 0)
    {
        gprsInit();
    }
    else if (strcmp(cmd, "SAVEEEPROM") == 0)
    {
        forceSaveEeprom();
        Serial.println("STATUS:EEPROM_FORCED_SAVE_DONE");
    }
    else if (strcmp(cmd, "SAVECONTACTS") == 0)
    {
        saveContactsToEeprom();
    }
    else if (strncmp(cmd, "SETOBLIGHT:", 11) == 0)
    {
        const char *params = cmd + 11;
        const char *comma = strchr(params, ',');
        if (comma != NULL)
        {
            unsigned long newOn = (unsigned long)atol(params);
            unsigned long newOff = (unsigned long)atol(comma + 1);

            if (newOn >= OB_MIN_ON_MS && newOn <= OB_MAX_ON_MS &&
                    newOff >= OB_MIN_OFF_MS && newOff <= OB_MAX_OFF_MS)
            {
                obOnTimeMs = newOn;
                obOffTimeMs = newOff;
                strncpy(currentObPresetName, "CUSTOM", OB_PRESET_NAME_LEN - 1);
                currentObPresetName[OB_PRESET_NAME_LEN - 1] = '\0';
                obLightSaveToEeprom();

                unsigned long totalPeriod = obOnTimeMs + obOffTimeMs;
                float dutyCycle = ((float)obOnTimeMs / (float)totalPeriod) * 100.0;
                char dcStr[10];
                floatToStr(dcStr, sizeof(dcStr), dutyCycle, 1);

                Serial.print("STATUS:OBLIGHT_SET_ON=");
                Serial.print(obOnTimeMs);
                Serial.print("ms_OFF=");
                Serial.print(obOffTimeMs);
                Serial.print("ms_PERIOD=");
                Serial.print(totalPeriod);
                Serial.print("ms_DUTY=");
                Serial.print(dcStr);
                Serial.println("%");

                debugSendStatusEvent("OBLIGHT_TIMING_CHANGED");
            }
            else
            {
                Serial.println("ERROR:OBLIGHT_TIMING_OUT_OF_RANGE");
                Serial.print("ERROR:ON_RANGE=");
                Serial.print(OB_MIN_ON_MS);
                Serial.print("-");
                Serial.print(OB_MAX_ON_MS);
                Serial.print("ms_OFF_RANGE=");
                Serial.print(OB_MIN_OFF_MS);
                Serial.print("-");
                Serial.print(OB_MAX_OFF_MS);
                Serial.println("ms");
            }
        }
        else
        {
            Serial.println("ERROR:OBLIGHT_FORMAT_SETOBLIGHT:on_ms,off_ms");
        }
    }
    else if (strncmp(cmd, "SETOBPRESET:", 12) == 0)
    {
        const char *presetName = cmd + 12;
        boolean applied = obLightApplyPreset(presetName);
        if (!applied)
        {
            Serial.println("ERROR:UNKNOWN_PRESET_USE_GETOBPRESETS");
        }
    }
    else if (strcmp(cmd, "GETOBPRESETS") == 0)
    {
        obLightListPresets();
    }
    else if (strcmp(cmd, "GETOBLIGHT") == 0)
    {
        unsigned long totalPeriod = obOnTimeMs + obOffTimeMs;
        float dutyCycle = ((float)obOnTimeMs / (float)totalPeriod) * 100.0;
        char dcStr[10];
        floatToStr(dcStr, sizeof(dcStr), dutyCycle, 1);

        Serial.print("STATUS:OBLIGHT_ON=");
        Serial.print(obOnTimeMs);
        Serial.print("ms_OFF=");
        Serial.print(obOffTimeMs);
        Serial.print("ms_PERIOD=");
        Serial.print(totalPeriod);
        Serial.print("ms_DUTY=");
        Serial.print(dcStr);
        Serial.print("%_EN=");
        Serial.print(obLightEnabled ? 1 : 0);
        Serial.print("_STATE=");
        Serial.print(obLedsCurrentlyOn ? "ON" : "OFF");
        Serial.print("_PRESET=");
        Serial.println(currentObPresetName);
    }
    else if (strcmp(cmd, "OBLIGHTON") == 0)
    {
        obLightEnabled = true;
        Serial.println("STATUS:OBLIGHT_ENABLED");
        debugSendStatusEvent("OBLIGHT_ENABLED");
    }
    else if (strcmp(cmd, "OBLIGHTOFF") == 0)
    {
        obLightEnabled = false;
        obLightSetAll(false);
        Serial.println("STATUS:OBLIGHT_DISABLED");
        debugSendStatusEvent("OBLIGHT_DISABLED");
    }
    else if (strcmp(cmd, "DEBUGON") == 0)
    {
        debugEnabled = true;
        Serial.println("STATUS:DEBUG_ENABLED_VIA_SERIAL");
        debugSerialPrintln("DBG:HANDSHAKE:VARUNA_DEBUG_V1");
        debugSerialPrintln("DBG:STATUS:DEBUG_ENABLED");
    }
    else if (strcmp(cmd, "DEBUGOFF") == 0)
    {
        if (debugEnabled)
        {
            debugSerialPrintln("DBG:STATUS:DEBUG_DISABLED");
        }
        debugEnabled = false;
        Serial.println("STATUS:DEBUG_DISABLED_VIA_SERIAL");
    }
    else if (strcmp(cmd, "DEBUGSTATUS") == 0)
    {
        Serial.print("STATUS:DEBUG_ENABLED=");
        Serial.println(debugEnabled ? 1 : 0);
        Serial.print("STATUS:DEBUG_PACKETS_SENT=");
        Serial.println(debugPacketsSent);
        Serial.print("STATUS:DEBUG_BYTES_SENT=");
        Serial.println(debugBytesSent);
        Serial.print("STATUS:DEBUG_BUTTON_PIN=");
        Serial.println(DEBUG_BUTTON_PIN);
        Serial.print("STATUS:DEBUG_TX_PIN=");
        Serial.println(DEBUG_UART_TX_PIN);
        Serial.print("STATUS:DEBUG_INTERVAL_MS=");
        Serial.println(DEBUG_OUTPUT_INTERVAL_MS);
    }
    else if (strncmp(cmd, "SETALERT:", 9) == 0 ||
             strncmp(cmd, "SETWARN:", 8) == 0 ||
             strncmp(cmd, "SETDANGER:", 10) == 0 ||
             strncmp(cmd, "SETTHRESH:", 10) == 0 ||
             strncmp(cmd, "ADDTIER1:", 9) == 0 ||
             strncmp(cmd, "ADDTIER2:", 9) == 0 ||
             strncmp(cmd, "ADDTIER3:", 9) == 0 ||
             strcmp(cmd, "CLEARTIERS") == 0 ||
             strcmp(cmd, "FLOODSTATUS") == 0 ||
             strcmp(cmd, "RESETFLOOD") == 0)
    {
        parseFloodCommand(cmd);
    }
    else
    {
        Serial.print("STATUS:UNKNOWN_COMMAND=");
        Serial.println(cmd);
    }
}

void recalibrate()
{
    Serial.println("STATUS:RECALIBRATING");
    debugSendStatusEvent("RECALIBRATION_START");
    debugSendCalibrationStatus("GYRO", 0, 300);

    float sumGX = 0, sumGY = 0, sumGZ = 0;
    int gyroSuccessCount = 0;
    for (int i = 0; i < 300; i++)
    {
        readMPU();
        if (mpuConsecutiveFailures == 0)
        {
            sumGX += gyroX;
            sumGY += gyroY;
            sumGZ += gyroZ;
            gyroSuccessCount++;
        }
        delay(2);
        if (i % 50 == 0)
        {
            esp_task_wdt_reset();
            debugSendCalibrationStatus("GYRO", i, 300);
        }
    }

    if (gyroSuccessCount > 50)
    {
        gyroOffsetX = sumGX / (float)gyroSuccessCount;
        gyroOffsetY = sumGY / (float)gyroSuccessCount;
        gyroOffsetZ = sumGZ / (float)gyroSuccessCount;
    }
    else
    {
        Serial.println("WARNING:GYRO_CALIBRATION_INSUFFICIENT_SAMPLES");
        debugSendStatusEvent("GYRO_CAL_INSUFFICIENT");
    }

    debugSendCalibrationStatus("ACCEL", 0, 300);

    float sumAX = 0, sumAY = 0, sumAZ = 0;
    int validCount = 0;
    for (int i = 0; i < 300; i++)
    {
        readMPU();
        if (mpuConsecutiveFailures == 0)
        {
            float ax = accX / 16384.0;
            float ay = accY / 16384.0;
            float az = accZ / 16384.0;

            float totalG = sqrt(ax * ax + ay * ay + az * az);
            if (totalG > 0.9 && totalG < 1.1)
            {
                sumAX += ax;
                sumAY += ay;
                sumAZ += az;
                validCount++;
            }
        }
        delay(2);
        if (i % 50 == 0)
        {
            esp_task_wdt_reset();
            debugSendCalibrationStatus("ACCEL", i, 300);
        }
    }

    if (validCount > 50)
    {
        refAccX = sumAX / validCount;
        refAccY = sumAY / validCount;
        refAccZ = sumAZ / validCount;
    }
    else
    {
        Serial.println("WARNING:ACCEL_CALIBRATION_INSUFFICIENT_SAMPLES");
        debugSendStatusEvent("ACCEL_CAL_INSUFFICIENT");
    }

    refTiltX = atan2(refAccY,
                     sqrt(refAccX * refAccX + refAccZ * refAccZ)) * 180.0 / PI;
    refTiltY = atan2(-refAccX,
                     sqrt(refAccY * refAccY + refAccZ * refAccZ)) * 180.0 / PI;

    filtTiltX = refTiltX;
    filtTiltY = refTiltY;

    char rxStr[10], ryStr[10];
    floatToStr(rxStr, sizeof(rxStr), refTiltX, 2);
    floatToStr(ryStr, sizeof(ryStr), refTiltY, 2);
    Serial.print("STATUS:NEW_REF_TILTX=");
    Serial.println(rxStr);
    Serial.print("STATUS:NEW_REF_TILTY=");
    Serial.println(ryStr);
    Serial.println("STATUS:RECALIBRATED_ZERO");

    debugSendCalibrationStatus("COMPLETE", 300, 300);
    debugSendStatusEvent("RECALIBRATION_DONE");
}

boolean canLightSleep()
{
    if (!algorithmEnabled)
    {
        return false;
    }

    if (sampleIntervalMs < INTERVAL_5MIN)
    {
        return false;
    }

    if (currentResponseLevel == RESP_CRITICAL)
    {
        return false;
    }

    if (smsRxState != SMS_RX_IDLE)
    {
        return false;
    }

    if (smsQueueCount > 0)
    {
        return false;
    }

    return true;
}

void enterLightSleep(unsigned long sleepDurationMs)
{
    if (sleepDurationMs < 10000)
    {
        return;
    }

    unsigned long maxSleepMs = ((unsigned long)WDT_TIMEOUT_SEC - 10UL) * 1000UL;
    if (sleepDurationMs > maxSleepMs)
    {
        sleepDurationMs = maxSleepMs;
    }

    obLightSetAll(false);

    nmeaIdx = 0;

    simUrcIdx = 0;

    Serial.print("STATUS:LIGHT_SLEEP_MS=");
    Serial.println(sleepDurationMs);
    Serial.flush();

    debugSendStatusEvent("ENTERING_LIGHT_SLEEP");

    esp_task_wdt_delete(NULL);

    esp_sleep_enable_timer_wakeup((uint64_t)sleepDurationMs * 1000ULL);

    esp_light_sleep_start();

    esp_task_wdt_add(NULL);
    esp_task_wdt_reset();

    prevTime = millis();

    while (GPS_Serial.available())
    {
        GPS_Serial.read();
    }
    nmeaIdx = 0;

    simUrcIdx = 0;

    Serial.println("STATUS:WAKE_FROM_SLEEP");
    debugSendStatusEvent("WAKE_FROM_SLEEP");
}

void setup()
{
    Serial.begin(115200);
    

    pinMode(LED_PIN, OUTPUT);

    esp_task_wdt_config_t earlyWdtConfig;
    earlyWdtConfig.timeout_ms = 180000UL;
    earlyWdtConfig.idle_core_mask = 0;
    earlyWdtConfig.trigger_panic = true;
    esp_task_wdt_init(&earlyWdtConfig);
    esp_task_wdt_add(NULL);
    Serial.println("STATUS:EARLY_WDT_180S_ENABLED");

    I2C_BUS0.begin(BUS0_SDA, BUS0_SCL, 100000);
    I2C_BUS1.begin(BUS1_SDA, BUS1_SCL, 100000);
    GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    SIM_Serial.begin(9600, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);

    delay(200);

    algoToggleInit();

    debugInit();

    c3FeedInit();

    obLightInit();

    EEPROM.begin(512);

    initStorage();

    esp_task_wdt_reset();

    rtcAvailable = rtcInit();
    if (rtcAvailable)
    {
        Serial.println("STATUS:DS1307_OK");
        debugSendSensorInit("DS1307_RTC", true);
        rtcReadRaw();
        rtcTimeValid = rtcIsTimeValid();

        if (rtcTimeValid)
        {
            currentUnixTime = rtcGetUnixTime();
            bootUnixTime = currentUnixTime;
            sessionStartUnix = currentUnixTime;

            Serial.print("STATUS:RTC_TIME=");
            Serial.print(rtcYear);
            Serial.print("-");
            if (rtcMonth < 10) Serial.print("0");
            Serial.print(rtcMonth);
            Serial.print("-");
            if (rtcDay < 10) Serial.print("0");
            Serial.print(rtcDay);
            Serial.print(" ");
            if (rtcHours < 10) Serial.print("0");
            Serial.print(rtcHours);
            Serial.print(":");
            if (rtcMinutes < 10) Serial.print("0");
            Serial.print(rtcMinutes);
            Serial.print(":");
            if (rtcSeconds < 10) Serial.print("0");
            Serial.println(rtcSeconds);
            Serial.print("STATUS:BOOT_UNIX=");
            Serial.println(bootUnixTime);
        }
        else
        {
            Serial.println("STATUS:RTC_TIME_INVALID_AWAITING_GPS_SYNC");
            rtcTimeValid = false;
        }
    }
    else
    {
        Serial.println("STATUS:DS1307_NOT_FOUND");
        debugSendSensorInit("DS1307_RTC", false);
    }

    esp_task_wdt_reset();

    boolean warmBooted = restoreStateFromEeprom();

    restoreContactsFromEeprom();

    if (!warmBooted)
    {
        Serial.println("STATUS:COLD_BOOT");
        debugSendStatusEvent("COLD_BOOT");
        currentResponseLevel = RESP_NORMAL;
        previousResponseLevel = RESP_NORMAL;
        readingsSinceBoot = 0;

        for (int i = 0; i < SUSTAINED_BUF_SIZE; i++)
        {
            sustainedBuffer[i] = 0;
            sustainedTimeBuffer[i] = 0;
        }
        sustainedBufCount = 0;
        sustainedBufIdx = 0;

        prevWaterHeight = 0;
        prevReadingTime = 0;
    }
    else
    {
        debugSendStatusEvent("WARM_BOOT");
        prevReadingTime = getBestTimestamp();
    }

    esp_task_wdt_reset();

    mpuWriteReg(0x6B, 0x00);
    delay(100);
    mpuWriteReg(0x19, 0x07);
    mpuWriteReg(0x1A, 0x03);
    mpuWriteReg(0x1B, 0x00);
    mpuWriteReg(0x1C, 0x00);
    delay(100);

    uint8_t who = mpuReadReg(0x75);
    Serial.print("STATUS:MPU_WHO_AM_I=0x");
    Serial.println(who, HEX);
    if (who != 0x68 && who != 0x72)
    {
        Serial.println("ERROR:MPU6050_NOT_FOUND");
        mpuHealthy = false;
        activeSensor = SENSOR_HCSR04;
        debugSendSensorInit("MPU6050", false);
    }
    else
    {
        Serial.println("STATUS:MPU6050_OK");
        mpuHealthy = true;
        activeSensor = SENSOR_MPU6050;
        debugSendSensorInit("MPU6050", true);
    }

    esp_task_wdt_reset();
    hcsr04Init();

    if (!mpuHealthy && !hcsr04Available)
    {
        activeSensor = SENSOR_NONE;
        Serial.println("ERROR:NO_SENSORS_AVAILABLE");
        debugSendStatusEvent("NO_SENSORS_AVAILABLE");
    }
    else if (!mpuHealthy && hcsr04Available)
    {
        activeSensor = SENSOR_HCSR04;
        Serial.println("STATUS:USING_HCSR04_AS_PRIMARY");
    }

    esp_task_wdt_reset();

    bmpAvailable = initBMP280();
    if (bmpAvailable)
    {
        Serial.println("STATUS:BMP280_OK");
        debugSendSensorInit("BMP280", true);

        float initPressure = 0;
        float initTemp = 0;
        float pressSum = 0;
        int validReadings = 0;
        for (int i = 0; i < 10; i++)
        {
            bmpReadData(&initTemp, &initPressure);
            if (initPressure > 300 && initPressure < 1200)
            {
                pressSum += initPressure;
                validReadings++;
            }
            delay(100);
        }

        if (validReadings > 0)
        {
            float avgPress = pressSum / validReadings;
            baselineBuffer[0] = avgPress;
            baselineCount = 1;
            baselineSum = avgPress;
            baselineIndex = 1;
            baselinePressure = avgPress;
            currentPressure = avgPress;
            currentTemperature = initTemp;

            char pStr[10];
            floatToStr(pStr, sizeof(pStr), avgPress, 2);
            Serial.print("STATUS:BASELINE_INIT=");
            Serial.println(pStr);
        }
    }
    else
    {
        Serial.println("STATUS:BMP280_NOT_FOUND");
        debugSendSensorInit("BMP280", false);
    }

    Serial.println("STATUS:GPS_UART_INIT");
    debugSendSensorInit("GPS_UART", true);

    esp_task_wdt_reset();
    simFullInit();

    esp_task_wdt_reset();

    if (mpuHealthy)
    {
        Serial.println("STATUS:CALIBRATING");
        Serial.println("STATUS:KEEP_BUOY_LEVEL_AND_STILL");
        debugSendStatusEvent("CALIBRATION_START");

        for (int i = 0; i < 6; i++)
        {
            digitalWrite(LED_PIN, HIGH);
            delay(250);
            digitalWrite(LED_PIN, LOW);
            delay(250);
        }

        Serial.println("STATUS:CALIBRATING_GYRO");
        debugSendCalibrationStatus("GYRO", 0, 1000);

        float sumGX = 0, sumGY = 0, sumGZ = 0;
        int gyroSuccessCount = 0;
        for (int i = 0; i < 1000; i++)
        {
            readMPU();
            if (mpuConsecutiveFailures == 0)
            {
                sumGX += gyroX;
                sumGY += gyroY;
                sumGZ += gyroZ;
                gyroSuccessCount++;
            }
            delay(2);
            if (i % 100 == 0)
            {
                esp_task_wdt_reset();
                debugSendCalibrationStatus("GYRO", i, 1000);
            }
        }

        if (gyroSuccessCount > 100)
        {
            gyroOffsetX = sumGX / (float)gyroSuccessCount;
            gyroOffsetY = sumGY / (float)gyroSuccessCount;
            gyroOffsetZ = sumGZ / (float)gyroSuccessCount;
        }
        else
        {
            Serial.println("ERROR:GYRO_CALIBRATION_INSUFFICIENT_SAMPLES");
            debugSendStatusEvent("GYRO_CAL_FAIL");
        }

        Serial.println("STATUS:CALIBRATING_REFERENCE");
        debugSendCalibrationStatus("ACCEL", 0, 500);

        float sumAX = 0, sumAY = 0, sumAZ = 0;
        int validCount = 0;

        for (int i = 0; i < 500; i++)
        {
            readMPU();

            if (mpuConsecutiveFailures == 0)
            {
                float ax = accX / 16384.0;
                float ay = accY / 16384.0;
                float az = accZ / 16384.0;

                float totalG = sqrt(ax * ax + ay * ay + az * az);
                if (totalG > 0.9 && totalG < 1.1)
                {
                    sumAX += ax;
                    sumAY += ay;
                    sumAZ += az;
                    validCount++;
                }
            }
            delay(3);
            if (i % 100 == 0)
            {
                esp_task_wdt_reset();
                debugSendCalibrationStatus("ACCEL", i, 500);
            }
        }

        if (validCount > 100)
        {
            refAccX = sumAX / validCount;
            refAccY = sumAY / validCount;
            refAccZ = sumAZ / validCount;
        }
        else
        {
            readMPU();
            refAccX = accX / 16384.0;
            refAccY = accY / 16384.0;
            refAccZ = accZ / 16384.0;
            Serial.println("WARNING:ACCEL_CAL_FALLBACK_SINGLE_READING");
        }

        refTiltX = atan2(refAccY,
                         sqrt(refAccX * refAccX + refAccZ * refAccZ)) * 180.0 / PI;
        refTiltY = atan2(-refAccX,
                         sqrt(refAccY * refAccY + refAccZ * refAccZ)) * 180.0 / PI;

        filtTiltX = refTiltX;
        filtTiltY = refTiltY;

        debugSendCalibrationStatus("COMPLETE", 500, 500);
    }

    prevTime = millis();
    lastBaselineUpdate = millis();
    lastBmpRead = millis();
    lastRtcRead = millis();
    lastLogWrite = millis();
    lastGpsProcess = millis();
    lastSimSignalCheck = millis();
    lastSimRegistrationCheck = millis();
    lastFloodSampleTime = millis();
    lastTransmitTime = millis();
    lastBatteryRead = millis();
    lastSensorHealthCheck = millis();
    lastEepromSave = millis();
    lastSerialOutput = millis();
    lastGpsFixTime = millis();

    if (!warmBooted)
    {
        prevWaterHeight = 0;
        prevReadingTime = 0;
    }

    analogReadResolution(12);
    readBatteryLevel();

    updateAdaptiveIntervals();

    if (stateEntryTime == 0)
    {
        stateEntryTime = getBestTimestamp();
    }

    esp_task_wdt_config_t opWdtConfig;
    opWdtConfig.timeout_ms = (uint32_t)WDT_TIMEOUT_SEC * 1000UL;
    opWdtConfig.idle_core_mask = 0;
    opWdtConfig.trigger_panic = true;
    esp_task_wdt_reconfigure(&opWdtConfig);
    Serial.print("STATUS:WATCHDOG_RECONFIGURED_");
    Serial.print(WDT_TIMEOUT_SEC);
    Serial.println("S");

    digitalWrite(LED_PIN, HIGH);

    if (mpuHealthy)
    {
        char rxStr[10], ryStr[10];
        char gxStr[10], gyStr2[10], gzStr[10];
        floatToStr(gxStr, sizeof(gxStr), refAccX, 3);
        floatToStr(gyStr2, sizeof(gyStr2), refAccY, 3);
        floatToStr(gzStr, sizeof(gzStr), refAccZ, 3);
        floatToStr(rxStr, sizeof(rxStr), refTiltX, 2);
        floatToStr(ryStr, sizeof(ryStr), refTiltY, 2);

        Serial.print("STATUS:REF_GRAVITY=");
        Serial.print(gxStr);
        Serial.print(",");
        Serial.print(gyStr2);
        Serial.print(",");
        Serial.println(gzStr);
        Serial.print("STATUS:REF_TILTX=");
        Serial.println(rxStr);
        Serial.print("STATUS:REF_TILTY=");
        Serial.println(ryStr);
        Serial.println("STATUS:CALIBRATION_DONE");
        Serial.println("STATUS:ANGLE_IS_NOW_ZERO");
        debugSendStatusEvent("CALIBRATION_DONE");
    }

    Serial.print("STATUS:ACTIVE_SENSOR=");
    Serial.println(activeSensor == SENSOR_MPU6050 ? "MPU6050" :
                   (activeSensor == SENSOR_HCSR04 ? "HCSR04" : "NONE"));

    char aStr[10], wStr[10], dStr[10];
    floatToStr(aStr, sizeof(aStr), alertLevelCm, 1);
    floatToStr(wStr, sizeof(wStr), warningLevelCm, 1);
    floatToStr(dStr, sizeof(dStr), dangerLevelCm, 1);
    Serial.print("STATUS:FLOOD_ALERT_LEVEL=");
    Serial.println(aStr);
    Serial.print("STATUS:FLOOD_WARNING_LEVEL=");
    Serial.println(wStr);
    Serial.print("STATUS:FLOOD_DANGER_LEVEL=");
    Serial.println(dStr);
    Serial.print("STATUS:FLOOD_RESPONSE=");
    Serial.println(responseNameStr(currentResponseLevel));
    Serial.print("STATUS:SAMPLE_INTERVAL=");
    Serial.println(sampleIntervalMs);
    Serial.print("STATUS:TRANSMIT_INTERVAL=");
    Serial.println(transmitIntervalMs);

    Serial.print("STATUS:OBLIGHT_ON=");
    Serial.print(obOnTimeMs);
    Serial.print("ms_OFF=");
    Serial.print(obOffTimeMs);
    Serial.print("ms_PRESET=");
    Serial.println(currentObPresetName);
    Serial.print("STATUS:OBLIGHT_ENABLED=");
    Serial.println(obLightEnabled ? 1 : 0);

    Serial.print("STATUS:DEBUG_ENABLED=");
    Serial.println(debugEnabled ? 1 : 0);
    Serial.print("STATUS:DEBUG_BUTTON_PIN=");
    Serial.println(DEBUG_BUTTON_PIN);

    Serial.print("STATUS:ALGORITHM_ENABLED=");
    Serial.println(algorithmEnabled ? 1 : 0);
    Serial.print("STATUS:ALGO_BUTTON_PIN=");
    Serial.println(ALGO_BUTTON_PIN);
    Serial.print("STATUS:ALGO_STATUS_LED_PIN=");
    Serial.println(ALGO_STATUS_LED_PIN);

    Serial.println("STATUS:READY");
    debugSendStatusEvent("SYSTEM_READY");
    
    /* Send immediate test data line so Processing knows we're alive */
    Serial.println("STATUS:SENDING_INITIAL_DATA_BURST");
    
    delay(500);
}

void loop()
{
    esp_task_wdt_reset();

    checkSerialInput();
    checkSimURC();
    processOneSmsFromQueue();
    readGPS();

    unsigned long now = millis();

    float dt = (now - prevTime) / 1000.0;
    dtExcessive = (dt > 2.0);
    prevTime = now;

    if (dt <= 0)
    {
        dt = 0.01;
    }
    if (dt > 2.0)
    {
        dt = 0.01;
    }

    algoCheckButton(now);

    debugCheckButton(now);

    obLightUpdate(now);

    if (rtcAvailable && (now - lastRtcRead >= RTC_READ_INTERVAL))
    {
        lastRtcRead = now;
        currentUnixTime = rtcGetUnixTime();
        rtcTimeValid = rtcIsTimeValid();
        if (currentUnixTime > 0 && sessionStartUnix > 0)
        {
            sessionDuration = currentUnixTime - sessionStartUnix;
        }
    }

    if (gpsTimeValid && rtcAvailable)
    {
        syncRTCfromGPS();
    }

    if (simAvailable && (now - lastSimSignalCheck >= SIM_SIGNAL_INTERVAL))
    {
        lastSimSignalCheck = now;
        simCheckSignalPeriodic();
    }

    if (simAvailable && (now - lastSimRegistrationCheck >= SIM_REG_CHECK_INTERVAL))
    {
        lastSimRegistrationCheck = now;
        simCheckRegistrationPeriodic();
    }

    if (now - lastBatteryRead >= BATTERY_READ_INTERVAL)
    {
        lastBatteryRead = now;
        readBatteryLevel();
    }

    if (now - lastSensorHealthCheck >= SENSOR_HEALTH_INTERVAL)
    {
        lastSensorHealthCheck = now;
        checkSensorHealth();
    }

    float theta = 0;
    float waterHeight = 0;
    float correctedTiltX = 0;
    float correctedTiltY = 0;
    float horizontalDist = 0;

    if (activeSensor == SENSOR_MPU6050 && mpuHealthy)
    {
        readMPU();

        float ax = accX / 16384.0;
        float ay = accY / 16384.0;
        float az = accZ / 16384.0;

        float gx = (gyroX - gyroOffsetX) / 131.0;
        float gy = (gyroY - gyroOffsetY) / 131.0;

        float accTiltX = atan2(ay,
                               sqrt(ax * ax + az * az)) * 180.0 / PI;
        float accTiltY = atan2(-ax,
                               sqrt(ay * ay + az * az)) * 180.0 / PI;

        if (dtExcessive)
        {
            filtTiltX = accTiltX;
            filtTiltY = accTiltY;
        }
        else
        {
            filtTiltX = ALPHA * (filtTiltX + gx * dt) + (1.0 - ALPHA) * accTiltX;
            filtTiltY = ALPHA * (filtTiltY + gy * dt) + (1.0 - ALPHA) * accTiltY;
        }

        correctedTiltX = filtTiltX - refTiltX;
        correctedTiltY = filtTiltY - refTiltY;

        theta = sqrt(correctedTiltX * correctedTiltX +
                     correctedTiltY * correctedTiltY);

        if (theta > 90.0)
        {
            theta = 90.0;
        }

        waterHeight = calculateWaterHeight(theta);
        horizontalDist = olpLength * sin(theta * PI / 180.0);
    }
    else if (activeSensor == SENSOR_HCSR04 && hcsr04Available)
    {
        waterHeight = 0;
        theta = 0;
    }

    if (bmpAvailable && (now - lastBmpRead >= BMP_READ_INTERVAL))
    {
        lastBmpRead = now;
        float temp, press;
        bmpReadData(&temp, &press);

        if (press > 300 && press < 1200)
        {
            currentPressure = press;
            currentTemperature = temp;
            submersionState = evaluateSubmersion(press, baselinePressure);

            if (submersionState == 0 &&
                    (now - lastBaselineUpdate >= BASELINE_INTERVAL))
            {
                lastBaselineUpdate = now;
                updateBaseline(press);
            }
        }
        else
        {
            submersionState = -1;
        }
    }

    if (algorithmEnabled)
    {
        if (now - lastFloodSampleTime >= sampleIntervalMs)
        {
            lastFloodSampleTime = now;

            float floodWaterHeight = waterHeight;
            float floodTheta = theta;

            if (activeSensor == SENSOR_MPU6050 && mpuHealthy)
            {
                readMPU();

                if (mpuConsecutiveFailures == 0)
                {
                    float fresh_ax = accX / 16384.0;
                    float fresh_ay = accY / 16384.0;
                    float fresh_az = accZ / 16384.0;
                    float fresh_accTiltX = atan2(fresh_ay,
                                                 sqrt(fresh_ax * fresh_ax + fresh_az * fresh_az))
                                           * 180.0 / PI - refTiltX;
                    float fresh_accTiltY = atan2(-fresh_ax,
                                                 sqrt(fresh_ay * fresh_ay + fresh_az * fresh_az))
                                           * 180.0 / PI - refTiltY;
                    floodTheta = sqrt(fresh_accTiltX * fresh_accTiltX +
                                     fresh_accTiltY * fresh_accTiltY);
                    if (floodTheta > 90.0)
                    {
                        floodTheta = 90.0;
                    }
                    floodWaterHeight = calculateWaterHeight(floodTheta);
                }
            }
            else if (activeSensor == SENSOR_HCSR04 && hcsr04Available)
            {
                float h = hcsr04GetWaterHeight();
                if (h >= 0)
                {
                    floodWaterHeight = h;
                    floodTheta = 0;
                }
            }

            if (activeSensor != SENSOR_NONE)
            {
                lastFloodWaterHeight = floodWaterHeight;

                uint32_t ts = getBestTimestamp();
                calculateRateOfChange(floodWaterHeight, ts);

                evaluateFloodStatus(floodWaterHeight, ts);

                updateAdaptiveIntervals();

                addToTransmitBuffer(floodWaterHeight, ratePer15Min, ts);

                logReading(floodWaterHeight, floodTheta, currentPressure, ts);

                markEepromDirty();
                saveEepromIfNeeded();

                checkAckTimeout();

                char fhStr[10], frStr[10], fbStr[6];
                floatToStr(fhStr, sizeof(fhStr), floodWaterHeight, 2);
                floatToStr(frStr, sizeof(frStr), ratePer15Min, 3);
                floatToStr(fbStr, sizeof(fbStr), batteryPercent, 1);

                Serial.print("FLOOD:");
                Serial.print(zoneNameStr(currentZone));
                Serial.print(",");
                Serial.print(responseNameStr(currentResponseLevel));
                Serial.print(",");
                Serial.print(frStr);
                Serial.print(",");
                Serial.print(sustainedRise ? "RISING" : "STABLE");
                Serial.print(",");
                Serial.print(fhStr);
                Serial.print(",");
                Serial.print(fbStr);
                Serial.print(",");
                Serial.print(sampleIntervalMs / 1000UL);
                Serial.print("s,");
                Serial.print(transmitIntervalMs / 1000UL);
                Serial.println("s");
            }
        }

        if (now - lastTransmitTime >= transmitIntervalMs)
        {
            lastTransmitTime = now;

            if (txBufferCount > 0)
            {
                transmitBufferedData(lastFloodWaterHeight);
            }
        }
    }
    else
    {
        if (activeSensor == SENSOR_HCSR04 && hcsr04Available)
        {
            float h = hcsr04GetWaterHeight();
            if (h >= 0)
            {
                waterHeight = h;
            }
        }

        lastFloodWaterHeight = waterHeight;

        uint32_t ts = getBestTimestamp();
        logReading(waterHeight, theta, currentPressure, ts);
    }

    switch (currentResponseLevel)
    {
        case RESP_CRITICAL:
        {
            unsigned long ledCycle = now % 200;
            digitalWrite(LED_PIN, ledCycle < 100 ? HIGH : LOW);
            break;
        }
        case RESP_FLOOD:
        {
            unsigned long ledCycle = now % 500;
            digitalWrite(LED_PIN, ledCycle < 250 ? HIGH : LOW);
            break;
        }
        case RESP_WARNING:
        {
            unsigned long ledCycle = now % 1000;
            digitalWrite(LED_PIN, ledCycle < 500 ? HIGH : LOW);
            break;
        }
        case RESP_WATCH:
        {
            unsigned long ledCycle = now % 3000;
            digitalWrite(LED_PIN, ledCycle < 1000 ? HIGH : LOW);
            break;
        }
        case RESP_NORMAL:
        default:
        {
            digitalWrite(LED_PIN, HIGH);
            break;
        }
    }

    debugSendPeriodicPacket(now, theta, waterHeight,
                            correctedTiltX, correctedTiltY,
                            horizontalDist);

        if (serialOutputEnabled && (now - lastSerialOutput >= SERIAL_OUTPUT_INTERVAL))
        {
        lastSerialOutput = now;

        char thetaStr[10], whStr[10], ctxStr[10], ctyStr[10];
        char olpStr[10], hdStr[10], cpStr[10], ctmpStr[10];
        char bpStr[10], pdStr[10], edStr[10];
        char r15Str[10], pkStr[10], mnStr[10];
        char latStr[12], lonStr[12], altStr[10];
        char batStr[6];

        floatToStr(thetaStr, sizeof(thetaStr), theta, 2);
        floatToStr(whStr, sizeof(whStr), waterHeight, 2);
        floatToStr(ctxStr, sizeof(ctxStr), correctedTiltX, 2);
        floatToStr(ctyStr, sizeof(ctyStr), correctedTiltY, 2);
        floatToStr(olpStr, sizeof(olpStr), olpLength, 2);
        floatToStr(hdStr, sizeof(hdStr), horizontalDist, 2);
        floatToStr(cpStr, sizeof(cpStr), currentPressure, 2);
        floatToStr(ctmpStr, sizeof(ctmpStr), currentTemperature, 2);
        floatToStr(bpStr, sizeof(bpStr), baselinePressure, 2);
        floatToStr(pdStr, sizeof(pdStr), pressureDeviation, 2);
        floatToStr(edStr, sizeof(edStr), estimatedDepth, 2);
        floatToStr(r15Str, sizeof(r15Str), ratePer15Min, 3);
        floatToStr(pkStr, sizeof(pkStr), peakHeight, 2);

        float dispMin = minHeight > 9000 ? 0.0 : minHeight;
        floatToStr(mnStr, sizeof(mnStr), dispMin, 2);

        floatToStr(latStr, sizeof(latStr), gpsLat, 6);
        floatToStr(lonStr, sizeof(lonStr), gpsLon, 6);
        floatToStr(altStr, sizeof(altStr), gpsAlt, 1);
        floatToStr(batStr, sizeof(batStr), batteryPercent, 1);

        Serial.print(thetaStr);
        Serial.print(",");
        Serial.print(whStr);
        Serial.print(",");
        Serial.print(ctxStr);
        Serial.print(",");
        Serial.print(ctyStr);
        Serial.print(",");
        Serial.print(olpStr);
        Serial.print(",");
        Serial.print(hdStr);
        Serial.print(",");
        Serial.print(cpStr);
        Serial.print(",");
        Serial.print(ctmpStr);
        Serial.print(",");
        Serial.print(bpStr);
        Serial.print(",");
        Serial.print(pdStr);
        Serial.print(",");
        Serial.print(submersionState);
        Serial.print(",");
        Serial.print(edStr);
        Serial.print(",");
        Serial.print(bmpAvailable ? 1 : 0);
        Serial.print(",");
        Serial.print(currentUnixTime);
        Serial.print(",");
        Serial.print(rtcYear);
        Serial.print("-");
        if (rtcMonth < 10) Serial.print("0");
        Serial.print(rtcMonth);
        Serial.print("-");
        if (rtcDay < 10) Serial.print("0");
        Serial.print(rtcDay);
        Serial.print(" ");
        if (rtcHours < 10) Serial.print("0");
        Serial.print(rtcHours);
        Serial.print(":");
        if (rtcMinutes < 10) Serial.print("0");
        Serial.print(rtcMinutes);
        Serial.print(":");
        if (rtcSeconds < 10) Serial.print("0");
        Serial.print(rtcSeconds);
        Serial.print(",");
        Serial.print(rtcAvailable && rtcTimeValid ? 1 : 0);
        Serial.print(",");
        Serial.print(r15Str);
        Serial.print(",");
        Serial.print(floodAlertLevel);
        Serial.print(",");
        Serial.print(sessionDuration);
        Serial.print(",");
        Serial.print(pkStr);
        Serial.print(",");
        Serial.print(mnStr);
        Serial.print(",");
        Serial.print(latStr);
        Serial.print(",");
        Serial.print(lonStr);
        Serial.print(",");
        Serial.print(altStr);
        Serial.print(",");
        Serial.print(gpsSatellites);
        Serial.print(",");
        Serial.print(gpsFixValid ? 1 : 0);
        Serial.print(",");
        Serial.print(simSignalRSSI);
        Serial.print(",");
        Serial.print(simRegistered ? 1 : 0);
        Serial.print(",");
        Serial.print(simAvailable ? 1 : 0);
        Serial.print(",");
        Serial.print(currentZone);
        Serial.print(",");
        Serial.print(currentResponseLevel);
        Serial.print(",");
        Serial.print(sustainedRise ? 1 : 0);
        Serial.print(",");
        Serial.print(batStr);
        Serial.print(",");
        Serial.print(sampleIntervalMs / 1000UL);
        Serial.print(",");
        Serial.print(transmitIntervalMs / 1000UL);
        Serial.print(",");
        Serial.print(obLightEnabled ? 1 : 0);
        Serial.print(",");
        Serial.print(debugEnabled ? 1 : 0);
        Serial.print(",");
        Serial.println(algorithmEnabled ? 1 : 0);

        /* === Duplicate CSV line to C3 Display === */
        {
            char csvLine[384];
            int pos = 0;

            pos += snprintf(csvLine + pos, sizeof(csvLine) - pos,
                "%s,%s,%s,%s,%s,%s,",
                thetaStr, whStr, ctxStr, ctyStr, olpStr, hdStr);
            pos += snprintf(csvLine + pos, sizeof(csvLine) - pos,
                "%s,%s,%s,%s,%d,%s,%d,",
                cpStr, ctmpStr, bpStr, pdStr, submersionState, edStr,
                bmpAvailable ? 1 : 0);
            pos += snprintf(csvLine + pos, sizeof(csvLine) - pos,
                "%lu,", (unsigned long)currentUnixTime);

            /* Date field */
            char dateBuf[20];
            snprintf(dateBuf, sizeof(dateBuf), "%u-%02u-%02u %02u:%02u:%02u",
                     rtcYear, rtcMonth, rtcDay, rtcHours, rtcMinutes, rtcSeconds);
            pos += snprintf(csvLine + pos, sizeof(csvLine) - pos, "%s,", dateBuf);

            pos += snprintf(csvLine + pos, sizeof(csvLine) - pos,
                "%d,%s,%d,%lu,%s,%s,",
                rtcAvailable && rtcTimeValid ? 1 : 0,
                r15Str, floodAlertLevel,
                (unsigned long)sessionDuration, pkStr, mnStr);
            pos += snprintf(csvLine + pos, sizeof(csvLine) - pos,
                "%s,%s,%s,%d,%d,",
                latStr, lonStr, altStr, gpsSatellites, gpsFixValid ? 1 : 0);
            pos += snprintf(csvLine + pos, sizeof(csvLine) - pos,
                "%d,%d,%d,%d,%d,%d,",
                simSignalRSSI, simRegistered ? 1 : 0, simAvailable ? 1 : 0,
                currentZone, currentResponseLevel, sustainedRise ? 1 : 0);
            pos += snprintf(csvLine + pos, sizeof(csvLine) - pos,
                "%s,%lu,%lu,%d,%d,%d",
                batStr,
                sampleIntervalMs / 1000UL, transmitIntervalMs / 1000UL,
                obLightEnabled ? 1 : 0, debugEnabled ? 1 : 0,
                algorithmEnabled ? 1 : 0);

            c3FeedPrintln(csvLine);
        }
    }

    if (algorithmEnabled)
    {
        unsigned long nextFloodSample = lastFloodSampleTime + sampleIntervalMs;
        unsigned long nextTransmit = lastTransmitTime + transmitIntervalMs;
        unsigned long nextEvent = nextFloodSample < nextTransmit ?
                                  nextFloodSample : nextTransmit;

        now = millis();

        if (now < nextEvent)
        {
            unsigned long timeToNextEvent = nextEvent - now;

            if (canLightSleep() && timeToNextEvent > 30000)
            {
                enterLightSleep(timeToNextEvent - 5000);
            }
            else
            {
                delay(50);
            }
        }
    }
    else
    {
        delay(50);
    }
}