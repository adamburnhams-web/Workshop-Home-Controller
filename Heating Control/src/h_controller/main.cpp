// ============================================================
//  H Controller — House Mega 2560
//  Controls: tank valves, electric heater (SSR cycle-burst),
//            log burner valve, 12V PSU relay, TFT display,
//            SD card logging, DS3231 RTC, inter-controller RS485
// ============================================================

#include <Arduino.h>
#include <avr/wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TFT_eSPI.h>
#include <Adafruit_GFX.h>
#include <SD.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <Wire.h>

#include "shared_types.h"
#include "rs485_packet.h"

#define DEBUG_SERIAL  // remove this line to strip all debug code from the build

// ============================================================
//  PIN DEFINITIONS  (H controller)
// ============================================================

// Inputs
#define PIN_ZERO_CROSSING    2   // INT4 (PE4) — AC zero-crossing module
#define PIN_LOG_BURNER_MOD   3   // 12VDC temp module via 10k+5.6k divider; HIGH = jacket >28°C
#define PIN_BTN_SELECT       4   // panel button, pull-up
#define PIN_BTN_UP           5   // panel button, pull-up
#define PIN_BTN_DOWN         6   // panel button, pull-up
#define PIN_ONE_WIRE        13   // DS18B20 1-Wire bus (6 sensors)
#define PIN_BUS_VOLTAGE     A0   // 15V bus via 10k+4.7k divider

// Outputs — relays
#define PIN_LOG_COLD_OPEN   22   // 8-ch ch1: log burner cold valve OPEN
#define PIN_LOG_COLD_CLOSE  23   // 8-ch ch2: log burner cold valve CLOSE
#define PIN_BOT_TANK_OPEN   24   // 8-ch ch3: bottom-of-tank valve OPEN
#define PIN_BOT_TANK_CLOSE  25   // 8-ch ch4: bottom-of-tank valve CLOSE
#define PIN_TWO_PORT_OPEN   26   // 8-ch ch5: 2-port valve OPEN (heater side)
#define PIN_HEATER_SSR      27   // direct output to SSR-40DA (NOT through relay board)
#define PIN_TWO_PORT_CLOSE  28   // 8-ch ch6: 2-port valve CLOSE (mid-tank side)
#define PIN_PSU_12V         29   // 8-ch ch7: 12VDC backup PSU relay
#define PIN_RS485_DE_LINK   30   // MAX485 DE/RE (HIGH = transmit)

// Display (SPI bus: D51=MOSI, D50=MISO, D52=SCK)
#define PIN_DISPLAY_CS      53   // active LOW
#define PIN_DISPLAY_DC      49
#define PIN_DISPLAY_RST     47   // or tie to Mega RESET
#define PIN_DISPLAY_BL      44   // backlight PWM — OC5C (Timer5); hardware wire moved from D32
#define PIN_SD_CS           48   // SD card CS, active LOW

// UART1 (D18 TX1 / D19 RX1) — inter-controller RS485 link

// ============================================================
//  DS18B20 SENSOR ADDRESSES  — fill in during commissioning
// ============================================================

static const uint8_t DS18B20_ADDRS[6][8] = {
    { 0x28, 0xA4, 0xD1, 0x14, 0x00, 0x00, 0x00, 0x3B }, // tank bottom    (sensor 7)
    { 0x28, 0xB7, 0x37, 0x15, 0x00, 0x00, 0x00, 0xF0 }, // tank middle    (sensor 8)
    { 0x28, 0xB2, 0x99, 0x12, 0x00, 0x00, 0x00, 0x0C }, // tank top       (sensor 9)
    { 0x28, 0x11, 0x6C, 0x12, 0x00, 0x00, 0x00, 0x7D }, // hot pipe       (sensor 10)
    { 0x28, 0x64, 0x27, 0x15, 0x00, 0x00, 0x00, 0x11 }, // cold pipe      (sensor 11)
    { 0x28, 0xEA, 0x8F, 0x12, 0x00, 0x00, 0x00, 0x63 }, // heater output  (sensor 12)
};
#define H_SENSOR_TANK_BOT    0
#define H_SENSOR_TANK_MID    1
#define H_SENSOR_TANK_TOP    2
#define H_SENSOR_HOT_PIPE    3
#define H_SENSOR_COLD_PIPE   4
#define H_SENSOR_HEATER_OUT  5
#define H_NUM_SENSORS        6

// ============================================================
//  COMMISSIONING FLAGS
// ============================================================

// Set true after verifying Growatt Modbus sign convention via W controller
static const bool HEATER_ENABLED = false;

// ============================================================
//  EEPROM ADDRESSES
// ============================================================

#define EE_SYSTEM_MODE       0   // uint8_t SystemMode
#define EE_NIGHT_COOLING     1   // uint8_t bool
#define EE_SOLAR_TARGET      2   // uint8_t SolarTargetMode
#define EE_DISPLAY_BRIGHT    3   // uint8_t 10–100 %
#define EE_FAN_BASE_SPEED    4   // uint8_t 0–100 % (also written by W via its own EEPROM)

// ============================================================
//  SYSTEM CONSTANTS
// ============================================================

#define DS18B20_CONV_MS         750UL
#define INTER_CTRL_POLL_MS      250UL
#define RS485_RX_TIMEOUT_MS     150UL
#define COMMS_FAULT_THRESHOLD   5
#define EEPROM_WRITE_DELAY_MS   30000UL  // 30s after last change
#define BACKLIGHT_SLEEP_MS      3600000UL // 1 hour
#define DISPLAY_INACTIVITY_MS   30000UL  // 30s → exit item/option mode
#define TIME_SYNC_INTERVAL_MS   3600000UL
#define BUS_LOW_THRESH_DV       140      // 14.0V (×0.1V units)
#define BUS_PSU_THRESH_DV       120      // 12.0V
#define BUS_PSU_HYSTERESIS_DV   125      // 12.5V
#define BUS_RESTORE_THRESH_DV   140      // 14.0V
#define BUS_LOW_DELAY_MS        10000UL  // 10s before fault
#define HEATER_ELEM_FAIL_MS     30000UL  // 30s no temp rise = element fault
#define HEATER_OVERHEAT_WARN_MS 20000UL  // 20s above 91°C = warning
#define HEATER_OVERHEAT_SHUT_MS 60000UL  // 1 min warning → shutdown

// Display colours (RGB565)
#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_CYAN    0x07FF
#define C_GREEN   0x07E0
#define C_RED     0xF800
#define C_AMBER   0xFD20
#define C_BLUE    0x001F
#define C_DKGRAY  0x39E7
#define C_LTGRAY  0x9CF3
#define C_NAVY    0x000F
#define C_YELLOW  0xFFE0

// ============================================================
//  GLOBAL OBJECTS
// ============================================================

OneWire          oneWire(PIN_ONE_WIRE);
DallasTemperature sensors(&oneWire);
TFT_eSPI         tft;
RTC_DS3231       rtc;

// ============================================================
//  H-BRIDGE VALVE  (identical to W controller implementation)
// ============================================================

enum HBridgePhase : uint8_t { HBP_IDLE, HBP_DEAD, HBP_PULSING };

struct HBridgeValve {
    uint8_t       pinA, pinB;
    uint16_t      pulseDurationMs;
    bool          isOpen;
    HBridgePhase  phase;
    bool          pendingOpen;
    bool          hasPending;
    unsigned long phaseStartMs;

    void begin(uint8_t a, uint8_t b, uint16_t dur) {
        pinA = a; pinB = b; pulseDurationMs = dur; isOpen = false;
        phase = HBP_IDLE; hasPending = false;
        pinMode(a, OUTPUT); digitalWrite(a, LOW);
        pinMode(b, OUTPUT); digitalWrite(b, LOW);
    }

    void request(bool open) {
        if (phase == HBP_PULSING) { digitalWrite(pinA, LOW); digitalWrite(pinB, LOW); }
        pendingOpen  = open;
        hasPending   = true;
        phaseStartMs = millis();
        phase        = HBP_DEAD;
    }

    void update() {
        unsigned long now = millis();
        switch (phase) {
            case HBP_IDLE: break;
            case HBP_DEAD:
                if (now - phaseStartMs >= 200UL) {
                    hasPending = false;
                    digitalWrite(pendingOpen ? pinA : pinB, HIGH);
                    phaseStartMs = now;
                    phase = HBP_PULSING;
                }
                break;
            case HBP_PULSING:
                if (now - phaseStartMs >= pulseDurationMs) {
                    digitalWrite(pinA, LOW); digitalWrite(pinB, LOW);
                    isOpen = pendingOpen;
                    phase = HBP_IDLE;
                    if (hasPending) { phaseStartMs = now; phase = HBP_DEAD; }
                }
                break;
        }
    }
    bool busy() const { return phase != HBP_IDLE; }
};

// ============================================================
//  VALVE INSTANCES
// ============================================================

HBridgeValve logBurnerCold;
HBridgeValve botTankValve;
HBridgeValve twoPortValve;  // OPEN = heater side; CLOSE = mid-tank side

// ============================================================
//  SENSOR STATE
// ============================================================

float   sTemp[H_NUM_SENSORS];
bool    sFault[H_NUM_SENSORS];
uint8_t sFailCount[H_NUM_SENSORS];
uint8_t sGoodCount[H_NUM_SENSORS];

// Last received W→H packet (populated by receiveWToHPacket)
WToHPacket lastWPkt;
bool       hasWPkt   = false;
unsigned long lastWPktMs = 0;

static inline bool tempValid(float t) {
    return !isnan(t) && t != 85.0f && t != -127.0f && t != -128.0f
           && t > -55.0f && t < 150.0f;
}

// ============================================================
//  HEATER SSR  —  zero-crossing ISR + cycle-burst control
// ============================================================

volatile unsigned long lastZCMicros  = 0;
volatile uint8_t       heaterCycleCnt = 0;    // 0–99 within 100-cycle window
volatile uint8_t       heaterTargetPct = 0;   // 0–100, written by outer loop

bool heaterRunning    = false;
bool heaterHardLockout = false;   // set on element fail / UFH hard lockout — clears on restart
bool heaterOvheatWarn = false;
unsigned long heaterOvheatWarnMs = 0;
unsigned long heaterElemFailMs   = 0;
float heaterOutAtElemCheck       = NAN;
bool gridPresent                 = true;
unsigned long lastGridLossMs     = 0;
bool gridOutageFault             = false;

// Zero-crossing ISR — runs every 20ms on 50Hz grid
// Direct port write (PA5 = D27) to avoid digitalWrite() overhead in ISR
void zeroCrossISR() {
    lastZCMicros = micros();
    if (heaterCycleCnt >= 99) heaterCycleCnt = 0;
    else                       heaterCycleCnt++;

    bool on = HEATER_ENABLED && heaterRunning && !heaterHardLockout
               && (heaterCycleCnt < heaterTargetPct);
    if (on) PORTA |= (1 << PA5);    // D27 = PA5
    else    PORTA &= ~(1 << PA5);
}

// Outer loop: compute heaterTargetPct from PV export every ~850ms
void updateHeaterDuty(int16_t pvExportW, int16_t gridImportW,
                       int16_t battSOC, bool battSocStale,
                       ManualHeaterMode manualMode, bool wSolarFault)
{
    // Grid outage → heater off always
    if (!gridPresent || !HEATER_ENABLED) {
        heaterTargetPct = 0;
        heaterRunning   = false;
        return;
    }
    // W solar sensor fault: W is in emergency UFH dump mode; suppress heater
    // to avoid adding heat to a circuit being used to dissipate excess solar heat.
    if (wSolarFault) {
        heaterTargetPct = 0;
        heaterRunning   = false;
        return;
    }
    if (heaterHardLockout) {
        heaterTargetPct = 0;
        heaterRunning   = false;
        return;
    }

    if (manualMode == MHM_OVERRIDE_SOC) {
        heaterRunning   = true;
        heaterTargetPct = 100;
        return;
    }

    if (manualMode == MHM_SOC_LIMITED) {
        if (battSocStale || battSOC <= 50) {
            heaterTargetPct = 0;
            heaterRunning   = false;
            return;
        }
        // Modulate to keep grid import at ~100W
        heaterRunning   = true;
        int32_t excess  = (int32_t)pvExportW - 100;
        if (excess < 0) excess = 0;
        heaterTargetPct = (uint8_t)constrain(excess * 100L / 3000L, 0, 100);
        return;
    }

    // Automatic summer mode
    // Trigger: pvExportW >= 500W
    if (!heaterRunning && pvExportW >= 500) heaterRunning = true;

    if (heaterRunning) {
        if (pvExportW < 100) {
            heaterRunning   = false;
            heaterTargetPct = 0;
            return;
        }
        int32_t excess  = (int32_t)pvExportW - 100;
        heaterTargetPct = (uint8_t)constrain(excess * 100L / 3000L, 0, 100);

        // Overheat power reduction (> 91°C heater output)
        if (!sFault[H_SENSOR_HEATER_OUT]) {
            float hOut = sTemp[H_SENSOR_HEATER_OUT];
            if (hOut > 92.0f) {
                heaterTargetPct = 0;
            } else if (hOut > 91.0f) {
                uint8_t reduce = (uint8_t)((hOut - 91.0f) * 100.0f);
                heaterTargetPct = heaterTargetPct * (100 - reduce) / 100;
            }
        }
    }
}

// ============================================================
//  FAULT FLAGS
// ============================================================

uint32_t hFaultFlags = 0;
void setFaultH(uint32_t m)   { hFaultFlags |= m; }
void clearFaultH(uint32_t m) { hFaultFlags &= ~m; }
bool hasFaultH(uint32_t m)   { return (hFaultFlags & m) != 0; }

#ifdef DEBUG_SERIAL
// ============================================================
//  SERIAL DEBUG INTERFACE
// ============================================================

static char    dbgBuf[64];
static uint8_t dbgLen = 0;

bool    hSimulate[H_NUM_SENSORS];
float   hSim[H_NUM_SENSORS];

bool    simLogBurnerActive = false, simLogBurnerVal = false;
bool    simPVExportActive  = false; int16_t simPVExportVal = 0;
bool    simBattSOCActive   = false; uint8_t simBattSOCVal  = 0;

enum CalPumpPhase : uint8_t { CALP_IDLE, CALP_STABILIZE, CALP_PRE_RAMP, CALP_RAMPING, CALP_DONE };
CalPumpPhase  calPumpPhase   = CALP_IDLE;
uint8_t       calSolarStepC  = 50;    // current test point °C (50–80 in 5°C steps)
uint8_t       calHeaterPct   = 0;     // heater override %
bool          calHtrOverride = false;
unsigned long calPhaseMs     = 0;     // phase start / stability timer
unsigned long calStepMs      = 0;     // ramp step timer
#endif // DEBUG_SERIAL

// ============================================================
//  BUS VOLTAGE  (A0: 10kΩ + 4.7kΩ divider, 15V → 4.80V)
// ============================================================

float busVoltageV = 15.0f;
bool  psu12vActive = false;
unsigned long busLowStartMs = 0;
bool  busLowActive = false;

void readBusVoltage() {
    int raw = analogRead(PIN_BUS_VOLTAGE);
    float vadc = raw * 5.0f / 1024.0f;
    busVoltageV = vadc * (10000.0f + 4700.0f) / 4700.0f;
}

void updateBusVoltage() {
    readBusVoltage();
    uint16_t vdv = (uint16_t)(busVoltageV * 10.0f); // in 0.1V units

    // 12V PSU relay
    if (!psu12vActive && vdv < BUS_PSU_THRESH_DV) {
        psu12vActive = true;
        digitalWrite(PIN_PSU_12V, HIGH);
    } else if (psu12vActive && vdv >= BUS_PSU_HYSTERESIS_DV) {
        psu12vActive = false;
        digitalWrite(PIN_PSU_12V, LOW);
    }

    // Bus low fault (< 14V for > 10s)
    if (vdv < BUS_LOW_THRESH_DV) {
        if (busLowStartMs == 0) busLowStartMs = millis();
        if (!busLowActive && millis() - busLowStartMs >= BUS_LOW_DELAY_MS) {
            busLowActive = true;
            setFaultH(FAULT_H_BUS_VOLTAGE_LOW);
        }
    } else {
        busLowStartMs = 0;
        if (busLowActive) {
            busLowActive = false;
            clearFaultH(FAULT_H_BUS_VOLTAGE_LOW);
        }
    }
}

// ============================================================
//  LOG BURNER TEMP MODULE  (D3, binary output, 28°C threshold)
// ============================================================

bool logBurnerHot = false;

void readLogBurnerModule() {
#ifdef DEBUG_SERIAL
    if (simLogBurnerActive) { logBurnerHot = simLogBurnerVal; return; }
#endif
    bool rawHigh = (digitalRead(PIN_LOG_BURNER_MOD) == HIGH);
    // Debounce: 3 consecutive readings before changing state
    static uint8_t hiCount = 0, loCount = 0;
    if (rawHigh) { hiCount = min(hiCount + 1, 3); loCount = 0; }
    else         { loCount = min(loCount + 1, 3); hiCount = 0; }
    if (hiCount >= 3) logBurnerHot = true;
    if (loCount >= 3) logBurnerHot = false;
}

// ============================================================
//  DS18B20 READING
// ============================================================

unsigned long lastConvStartMs = 0;
bool          convStarted     = false;
static const uint32_t sensorFaultMaskH[H_NUM_SENSORS] = {
    FAULT_H_SENSOR_TANK_BOT, FAULT_H_SENSOR_TANK_MID, FAULT_H_SENSOR_TANK_TOP,
    FAULT_H_SENSOR_HOT_PIPE, FAULT_H_SENSOR_COLD_PIPE, FAULT_H_SENSOR_HEATER_OUT
};

void startConversion() {
    sensors.setWaitForConversion(false);
    sensors.requestTemperatures();
    lastConvStartMs = millis();
    convStarted     = true;
}

void readSensors() {
    if (!convStarted || millis() - lastConvStartMs < DS18B20_CONV_MS) return;
    convStarted = false;

    for (uint8_t i = 0; i < H_NUM_SENSORS; i++) {
        float t = sensors.getTempC((uint8_t*)DS18B20_ADDRS[i]);
        if (tempValid(t)) {
            sTemp[i] = t; sFailCount[i] = 0;
            if (sGoodCount[i] < 3) sGoodCount[i]++;
            if (sGoodCount[i] >= 3 && sFault[i]) { sFault[i] = false; clearFaultH(sensorFaultMaskH[i]); }
        } else {
            sGoodCount[i] = 0;
            if (sFailCount[i] < 3) sFailCount[i]++;
            if (sFailCount[i] >= 3 && !sFault[i]) { sFault[i] = true; sTemp[i] = NAN; setFaultH(sensorFaultMaskH[i]); }
        }
    }
#ifdef DEBUG_SERIAL
    for (uint8_t i = 0; i < H_NUM_SENSORS; i++) {
        if (hSimulate[i]) { sTemp[i] = hSim[i]; sFault[i] = false; }
    }
#endif
}

// ============================================================
//  HEATER FAULT CHECKS  (called from main loop)
// ============================================================

void checkHeaterFaults() {
    if (!heaterRunning || heaterTargetPct == 0) {
        heaterElemFailMs   = 0;
        heaterOutAtElemCheck = NAN;
        heaterOvheatWarnMs = 0;
        heaterOvheatWarn   = false;
        return;
    }

    // Element fail: power applied > 30s with no temp rise
    if (!sFault[H_SENSOR_HEATER_OUT]) {
        float hOut = sTemp[H_SENSOR_HEATER_OUT];
        if (heaterElemFailMs == 0) {
            heaterElemFailMs     = millis();
            heaterOutAtElemCheck = hOut;
        } else if (millis() - heaterElemFailMs >= HEATER_ELEM_FAIL_MS) {
            if (!isnan(heaterOutAtElemCheck) && hOut < heaterOutAtElemCheck + 1.0f) {
                setFaultH(FAULT_H_HEATER_ELEMENT_FAIL);
                heaterHardLockout = true;
                heaterTargetPct   = 0;
                PORTA &= ~(1 << PA5); // D27 off
            }
            heaterElemFailMs = millis(); // reset check interval
            heaterOutAtElemCheck = hOut;
        }

        // Overheat warning: > 91°C for > 20s while hot pipe < 80°C
        bool hotPipeOk = !sFault[H_SENSOR_HOT_PIPE] && sTemp[H_SENSOR_HOT_PIPE] < 80.0f;
        if (hOut > 91.0f && hotPipeOk) {
            if (!heaterOvheatWarn) {
                heaterOvheatWarn   = true;
                heaterOvheatWarnMs = millis();
                setFaultH(FAULT_H_HEATER_OVERHEAT_WARN);
            }
            if (millis() - heaterOvheatWarnMs >= HEATER_OVERHEAT_SHUT_MS) {
                setFaultH(FAULT_H_HEATER_OVERHEAT_SHUT);
                heaterHardLockout = true;
                heaterTargetPct   = 0;
                PORTA &= ~(1 << PA5);
            }
        } else if (hOut <= 91.0f) {
            heaterOvheatWarn = false;
            clearFaultH(FAULT_H_HEATER_OVERHEAT_WARN);
            heaterOvheatWarnMs = 0;
        }
    } else {
        // Heater output sensor fault → lock out heater
        setFaultH(FAULT_H_SENSOR_HEATER_OUT);
        heaterHardLockout = true;
        heaterTargetPct   = 0;
        PORTA &= ~(1 << PA5);
    }
}

// ============================================================
//  RTC  (DS3231)
// ============================================================

bool     rtcValid = false;
DateTime rtcNow;
unsigned long lastRTCReadMs = 0;
// RTC fallback: if Mega loses power and no backup, we rely on W time sync
bool rtcBatteryLow = false;

void readRTC() {
    if (millis() - lastRTCReadMs < 1000) return;
    lastRTCReadMs = millis();
    if (rtc.begin()) {
        rtcNow  = rtc.now();
        rtcValid = rtcNow.isValid();
        rtcBatteryLow = rtc.lostPower();
    }
}

uint8_t rtcHour()   { return rtcValid ? rtcNow.hour()   : 0; }
uint8_t rtcMinute() { return rtcValid ? rtcNow.minute() : 0; }
uint8_t rtcSecond() { return rtcValid ? rtcNow.second() : 0; }

// ============================================================
//  SETTINGS  (EEPROM-backed)
// ============================================================

SystemMode      systemMode          = MODE_WINTER;
bool            nightCoolingEnabled = true;
SolarTargetMode solarTargetMode     = SOLAR_TANK_PLUS5;
uint8_t         displayBrightness   = 100;

// Not stored in EEPROM — reset on power cycle
BoostMode        boostMode          = BOOST_OFF;
float            boostTarget        = 15.5f;
ManualHeaterMode manualHeaterMode   = MHM_OFF;
uint32_t         boost8hrEndMs      = 0;  // millis() when 8hr boost ends
bool             pvExportOverride   = false; // true = PV export ≥500W in WINTER mode → send MODE_SUMMER to W

unsigned long eepromDirtyMs = 0;
bool          eepromDirty   = false;

void loadSettings() {
    uint8_t m = EEPROM.read(EE_SYSTEM_MODE);
    if (m <= 1)   systemMode          = (SystemMode)m;
    uint8_t nc = EEPROM.read(EE_NIGHT_COOLING);
    if (nc <= 1)  nightCoolingEnabled = (bool)nc;
    uint8_t st = EEPROM.read(EE_SOLAR_TARGET);
    if (st <= 1)  solarTargetMode     = (SolarTargetMode)st;
    uint8_t db = EEPROM.read(EE_DISPLAY_BRIGHT);
    if (db >= 10 && db <= 100) displayBrightness = db;
}

void saveSettingsIfDue() {
    if (!eepromDirty) return;
    if (millis() - eepromDirtyMs < EEPROM_WRITE_DELAY_MS) return;
    EEPROM.update(EE_SYSTEM_MODE,   (uint8_t)systemMode);
    EEPROM.update(EE_NIGHT_COOLING, (uint8_t)nightCoolingEnabled);
    EEPROM.update(EE_SOLAR_TARGET,  (uint8_t)solarTargetMode);
    EEPROM.update(EE_DISPLAY_BRIGHT, displayBrightness);
    eepromDirty = false;
}

void markSettingsDirty() { eepromDirty = true; eepromDirtyMs = millis(); }

// ============================================================
//  WINTER HEATING LOGIC  (H side)
// ============================================================

bool morningHeatActive = false;
bool morningHeatFired  = false;  // prevents re-trigger same calendar day
uint8_t lastHeatDay    = 255;

float getUFHStopTemp() {
    if (boostMode == BOOST_OFF) return 13.5f;
    return boostTarget + 0.5f;
}

void checkMorningTrigger() {
    if (!rtcValid) return;
    uint8_t h = rtcHour(), m = rtcMinute(), d = rtcNow.day();

    // 8hr boost: starts immediately, runs for 8 hours
    if (boostMode == BOOST_8HR && !morningHeatActive) {
        morningHeatActive = true;
        boost8hrEndMs     = millis() + 8UL * 3600000UL;
    }
    if (boostMode == BOOST_8HR && morningHeatActive) {
        if (millis() >= boost8hrEndMs) {
            morningHeatActive = false;
        }
        return;
    }

    // 5am trigger (or boost-at-5am)
    if (h == 5 && m == 0 && d != lastHeatDay) {
        lastHeatDay = d;
        // Check W air temp from last W packet
        bool wAirFault = (lastWPkt.tempWorkshopAir == TEMP_FAULT);
        float wAir = wAirFault ? 0.0f : (float)lastWPkt.tempWorkshopAir / 10.0f;
        float target = getUFHStopTemp();
        if (!wAirFault && wAir < target) {
            morningHeatActive = true;
        }
    }

    // End session when W reports UFH pump stopped after target reached
    if (morningHeatActive && lastWPkt.ufhTargetReached && !lastWPkt.ufhPumpRunning) {
        morningHeatActive = false;
    }
}

void updateHeatSourceSelection() {
    static bool prevMorningActive = false;

    if (prevMorningActive && !morningHeatActive) {
        // Session ended: return all tank valves to known safe state
        logBurnerCold.request(false);
        botTankValve.request(false);
        twoPortValve.request(true);   // heater cold side / top-of-tank
    }
    prevMorningActive = morningHeatActive;

    if (!morningHeatActive) return;

    if (lastWPkt.ufhTargetReached) {
        // W reached target: open top-of-tank path at H
        if (logBurnerHot) {
            logBurnerCold.request(true);     // open log burner cold
            twoPortValve.request(true);      // heater side → circulates through manifold to tank top
        } else {
            logBurnerCold.request(false);
        }
        botTankValve.request(false); // close bottom-of-tank
        return;
    }

    // Active heating: heat source selection
    readLogBurnerModule();
    if (logBurnerHot) {
        logBurnerCold.request(true);
        botTankValve.request(false); // close bottom-of-tank when log burner takes over
        twoPortValve.request(true);  // heater side → circulates through manifold to top of tank
    } else {
        logBurnerCold.request(false);
        botTankValve.request(true);  // open bottom-of-tank
    }

    // Mid-tank 2-port override with 2°C hysteresis
    // >32°C → mid-tank side; <28°C → heater side; 28–32°C → hold
    if (!sFault[H_SENSOR_TANK_MID]) {
        static bool twoPortMidTank = false;
        float tMid = sTemp[H_SENSOR_TANK_MID];
        if (!twoPortMidTank && tMid > 32.0f) {
            twoPortMidTank = true;
            twoPortValve.request(false); // mid-tank side
        } else if (twoPortMidTank && tMid < 28.0f) {
            twoPortMidTank = false;
            twoPortValve.request(true);  // heater cold side
        }
        // between 28–32°C: no request, hold current position
    }
}

// ============================================================
//  SUMMER STARTUP SEQUENCE  (H side)
//
//  2-port valve: OPEN (request true)  = heater cold side / top-of-tank
//                CLOSE (request false) = mid-tank side
//
//  Phase 1 — log burner cold OPEN, 2-port OPEN (heater/top-of-tank)
//             advance when |hot pipe − tank bottom| < 5°C
//  Phase 2 — log burner cold CLOSE, bottom-of-tank OPEN, 2-port CLOSE (mid-tank)
//             advance when |hot pipe − tank top| < 5°C
//  Phase 3 — 2-port OPEN (heater/top-of-tank); W resumes full modulation
//  Abort any phase → 2-port OPEN (heater/top-of-tank)
// ============================================================

uint8_t summerStartupPhase = 0;  // 0=idle 1=ph1 2=ph2 3=running

void updateSummerStartup() {
    bool solarRunning = hasWPkt && lastWPkt.solarPumpDutyPct > 0;

    if ((systemMode != MODE_SUMMER && !pvExportOverride) || !solarRunning) {
        if (summerStartupPhase != 0) {
            summerStartupPhase = 0;
            logBurnerCold.request(false);
            botTankValve.request(false);
            twoPortValve.request(true);   // heater cold side / top-of-tank on any abort
        }
        return;
    }

    switch (summerStartupPhase) {
        case 0:
            summerStartupPhase = 1;
            logBurnerCold.request(true);
            twoPortValve.request(true);   // heater cold side / top-of-tank
            break;

        case 1:
            // Advance when hot pipe equilibrates with tank bottom
            if (!sFault[H_SENSOR_HOT_PIPE] && !sFault[H_SENSOR_TANK_BOT]) {
                if (fabsf(sTemp[H_SENSOR_HOT_PIPE] - sTemp[H_SENSOR_TANK_BOT]) < 5.0f) {
                    summerStartupPhase = 2;
                    logBurnerCold.request(false);
                    botTankValve.request(true);
                    twoPortValve.request(false);  // mid-tank side
                }
            }
            break;

        case 2:
            // Advance when hot pipe equilibrates with tank top
            if (!sFault[H_SENSOR_HOT_PIPE] && !sFault[H_SENSOR_TANK_TOP]) {
                if (fabsf(sTemp[H_SENSOR_HOT_PIPE] - sTemp[H_SENSOR_TANK_TOP]) < 5.0f) {
                    summerStartupPhase = 3;
                    twoPortValve.request(true);   // heater cold side / top-of-tank
                }
            }
            break;

        case 3: {
            // Dynamic 2-port adjustment: track hot pipe vs tank top with 1°C hysteresis.
            // Sensor fault on either side → hold current position, no valve movement.
            static bool summerTwoPortTop = true; // true = heater side / top-of-tank
            if (!sFault[H_SENSOR_HOT_PIPE] && !sFault[H_SENSOR_TANK_TOP]) {
                float diff = sTemp[H_SENSOR_HOT_PIPE] - sTemp[H_SENSOR_TANK_TOP];
                if (!summerTwoPortTop && diff > 1.0f) {
                    summerTwoPortTop = true;
                    twoPortValve.request(true);   // heater cold side / top-of-tank
                } else if (summerTwoPortTop && diff < -1.0f) {
                    summerTwoPortTop = false;
                    twoPortValve.request(false);  // mid-tank side
                }
                // within ±1°C: hold current position
            }
            break;
        }
    }
}

// PV export override: in winter mode, when PV export >= 500W, tell W to run summer solar
// logic by sending MODE_SUMMER in the H→W packet without changing the stored systemMode.
void updatePVExportOverride() {
    if (systemMode != MODE_WINTER) { pvExportOverride = false; return; }
    bool pvActive = (lastWPkt.pvTotalW > 0);
    if (!pvExportOverride && lastWPkt.pvExportW >= 500) pvExportOverride = true;
    if (pvExportOverride && !pvActive)                   pvExportOverride = false;
}

// ============================================================
//  SD CARD LOGGING
// ============================================================

bool    sdAvailable   = false;
bool    sdEjected     = false;
unsigned long lastLogMs = 0;

void initSD() {
    if (SD.begin(PIN_SD_CS)) {
        sdAvailable = true;
        // Write CSV header if file doesn't exist
        if (!SD.exists("log.csv")) {
            File f = SD.open("log.csv", FILE_WRITE);
            if (f) {
                f.println(F("ts_ms,solar_hot,solar_cold,ufh_sup,ufh_tmv,w_air,out_air,"
                            "tank_top,tank_mid,tank_bot,hot_pipe,cold_pipe,htr_out,"
                            "pump_pct,htr_pct,pv1w,pv2w,pvtot,export,import,"
                            "batt_soc,batt_w,batt_v,grid_v,grid_hz,inv_tmp,inv_stat,"
                            "bus_v,fan1rpm,fan2rpm,fan_pct"));
                f.close();
            }
        }
    }
}

void logDataRow() {
    if (!sdAvailable || sdEjected) return;
    File f = SD.open("log.csv", FILE_WRITE);
    if (!f) return;

    f.print(millis()); f.print(',');
    // W temperatures
    auto pT = [&](int16_t v){ if(v==TEMP_FAULT) f.print("NaN"); else f.print(v/10.0f,1); f.print(','); };
    pT(lastWPkt.tempSolarHot); pT(lastWPkt.tempSolarCold);
    pT(lastWPkt.tempUFHSupply); pT(lastWPkt.tempUFHPostTMV);
    pT(lastWPkt.tempWorkshopAir); pT(lastWPkt.tempOutsideAir);
    // H temperatures
    for (uint8_t i = 0; i < H_NUM_SENSORS; i++) {
        if (sFault[i]) f.print("NaN"); else f.print(sTemp[i], 1); f.print(',');
    }
    // State
    f.print(lastWPkt.solarPumpDutyPct); f.print(',');
    f.print(heaterRunning ? heaterTargetPct : 0); f.print(',');
    f.print(lastWPkt.pvString1W); f.print(',');
    f.print(lastWPkt.pvString2W); f.print(',');
    f.print(lastWPkt.pvTotalW); f.print(',');
    f.print(lastWPkt.pvExportW); f.print(',');
    f.print(lastWPkt.gridImportW); f.print(',');
    f.print(lastWPkt.battSOC); f.print(',');
    f.print(lastWPkt.battW); f.print(',');
    f.print(lastWPkt.battVoltage_dV / 10.0f, 1); f.print(',');
    f.print(lastWPkt.gridVoltage_dV / 10.0f, 1); f.print(',');
    f.print(lastWPkt.gridFreq_cHz / 100.0f, 2); f.print(',');
    if (lastWPkt.inverterTemp_dC == TEMP_FAULT) f.print("NaN"); else f.print(lastWPkt.inverterTemp_dC / 10.0f, 1);
    f.print(','); f.print(lastWPkt.inverterStatus); f.print(',');
    f.print(busVoltageV, 2); f.print(',');
    f.print(0); f.print(','); // fan1RPM — not available at H (W side)
    f.print(0); f.print(','); // fan2RPM
    f.println(lastWPkt.fanDutyPct);
    f.close();
}

void safeEjectSD() {
    if (!sdAvailable) return;
    SD.end();
    sdEjected   = true;
    sdAvailable = false;
}

void checkSDReinsert() {
    if (!sdEjected) return;
    // Try to reinitialise; SD.begin() returns true if card present
    if (SD.begin(PIN_SD_CS)) {
        sdEjected   = false;
        sdAvailable = true;
        initSD();
    }
}

// ============================================================
//  FAULT HISTORY  (RAM-only, clears on restart)
// ============================================================

struct FaultEntry {
    uint32_t faultMask;
    bool     isH;           // true = H fault, false = W fault
    uint32_t onsetMs;
    uint32_t resolvedMs;    // 0 = still active
};

static const uint8_t MAX_FAULT_ENTRIES = 80; // ~40 bytes each ≈ 3.2KB
FaultEntry faultLog[MAX_FAULT_ENTRIES];
uint8_t    faultLogCount = 0;

void faultLogUpdate(uint32_t curW, uint32_t curH) {
    // Check each active fault against log — add new, resolve cleared
    static uint32_t prevW = 0, prevH = 0;
    uint32_t newW  = curW & ~prevW;
    uint32_t newH  = curH & ~prevH;
    uint32_t gonW  = prevW & ~curW;
    uint32_t gonH  = prevH & ~curH;

    auto addEntry = [&](uint32_t mask, bool isH) {
        if (faultLogCount < MAX_FAULT_ENTRIES) {
            faultLog[faultLogCount++] = { mask, isH, (uint32_t)millis(), 0 };
        }
    };
    auto resolveEntry = [&](uint32_t mask, bool isH) {
        for (uint8_t i = 0; i < faultLogCount; i++) {
            if (faultLog[i].faultMask == mask && faultLog[i].isH == isH && faultLog[i].resolvedMs == 0)
                faultLog[i].resolvedMs = (uint32_t)millis();
        }
    };

    for (uint8_t b = 0; b < 32; b++) {
        if (newW & (1UL << b)) addEntry((1UL << b), false);
        if (newH & (1UL << b)) addEntry((1UL << b), true);
        if (gonW & (1UL << b)) resolveEntry((1UL << b), false);
        if (gonH & (1UL << b)) resolveEntry((1UL << b), true);
    }
    prevW = curW; prevH = curH;
}

// ============================================================
//  DISPLAY  (ILI9488 480×320)
// ============================================================

uint8_t      currentPage      = 1;
enum NavMode { NAV_PAGE, NAV_ITEM, NAV_OPTION };
NavMode      navMode          = NAV_PAGE;
uint8_t      selectedItem     = 0;
unsigned long lastButtonMs    = 0;
bool         displayOn        = true;
bool         needFullRedraw   = true;
unsigned long lastDisplayRefreshMs = 0;
uint8_t      alertResetSeqTx       = 0;  // increment to signal alert reset to W
int8_t       pendingFanFullDeltaHr  = 0;  // one-shot: +/- hours; sent to W, cleared after transmit
int8_t       pendingFanBaseDeltaDay = 0;  // one-shot: +/- days;  sent to W, cleared after transmit

// Button state (debounced)
struct Button {
    uint8_t pin;
    bool    state, prevState;
    unsigned long lastMs;
    bool    pressed() {
        bool raw = (digitalRead(pin) == LOW);
        if (raw != prevState) { lastMs = millis(); prevState = raw; }
        if (millis() - lastMs > 50 && raw != state) { state = raw; return state; }
        return false;
    }
} btnSelect, btnUp, btnDown;

// Track double-press SELECT for boost shortcut
unsigned long lastSelectMs   = 0;
bool          selectPending  = false;

// Forward declaration for full redraw
void drawFullPage();

void setBacklight(uint8_t pct) {
    analogWrite(PIN_DISPLAY_BL, pct == 0 ? 0 : (uint8_t)map(pct, 0, 100, 0, 255));
}

void wakeDisplay() {
    if (!displayOn) {
        displayOn = true;
        setBacklight(displayBrightness);
        needFullRedraw = true;
    }
    lastButtonMs = millis();
}

// ── Status bar (top 20px) ─────────────────────────────────

void drawStatusBar() {
    tft.fillRect(0, 0, 480, 20, C_NAVY);
    tft.setTextColor(C_WHITE); tft.setTextSize(1);

    // Mode
    tft.setCursor(2, 6);
    tft.print(systemMode == MODE_WINTER ? "WINTER" : "SUMMER");

    // Boost
    tft.setCursor(60, 6);
    if (boostMode == BOOST_5AM)       tft.print("BOOST-5AM");
    else if (boostMode == BOOST_8HR)  tft.print("BOOST-8HR");

    // Night cooling
    tft.setCursor(160, 6);
    if (nightCoolingEnabled) tft.print("NC:ON"); else tft.print("NC:OFF");

    // Solar target
    tft.setCursor(215, 6);
    tft.print(solarTargetMode == SOLAR_TANK_PLUS5 ? "TK+5" : "MAX");

    // Clock
    tft.setCursor(400, 6);
    char buf[9];
    snprintf(buf, sizeof(buf), "%02u:%02u:%02u", rtcHour(), rtcMinute(), rtcSecond());
    tft.print(buf);

    // Page indicator
    tft.setCursor(250, 6);
    tft.print("P"); tft.print(currentPage); tft.print("/5");
}

// ── Fault bar (bottom 20px) ───────────────────────────────

const char* faultNameW(uint32_t mask) {
    if (mask & FAULT_W_SOLAR_OVERHEAT_COLD)  return "Sol Ovht Cold";
    if (mask & FAULT_W_SOLAR_OVERHEAT_HOT)   return "Sol Ovht Hot";
    if (mask & FAULT_W_SOLAR_PUMP)            return "Sol Pump Fail";
    if (mask & FAULT_W_UFH_OVERHEAT)          return "UFH Overheat";
    if (mask & FAULT_W_FROST_NOT_RECOVERING)  return "Frost No Rcvr";
    if (mask & FAULT_W_VAC_PUMP_OVERTIME)     return "Vac Overtime";
    if (mask & FAULT_W_GROWATT_COMMS)         return "Growatt Comms";
    if (mask & FAULT_W_INVERTER_FAULT)        return "Inverter Fault";
    if (mask & FAULT_W_RS485_COMMS)           return "W RS485 Fault";
    if (mask & FAULT_W_FAN1)                  return "Fan1 Fault";
    if (mask & FAULT_W_FAN2)                  return "Fan2 Fault";
    if (mask & FAULT_W_WINCH_OVER_OPEN)       return "Winch OverOpen";
    return "W Sensor Fault";
}
const char* faultNameH(uint32_t mask) {
    if (mask & FAULT_H_HEATER_OVERHEAT_WARN)  return "Htr Ovht Warn";
    if (mask & FAULT_H_HEATER_OVERHEAT_SHUT)  return "Htr Ovht STOP";
    if (mask & FAULT_H_HEATER_ELEMENT_FAIL)   return "Htr Elem Fail";
    if (mask & FAULT_H_RS485_COMMS)           return "H RS485 Fault";
    if (mask & FAULT_H_BUS_VOLTAGE_LOW)       return "15V Bus Low";
    if (mask & FAULT_H_GRID_OUTAGE)           return "Grid Outage";
    return "H Sensor Fault";
}

static uint8_t  faultBarIdx    = 0;
static unsigned long faultScrollMs = 0;

void drawFaultBar(uint32_t wFaults, uint32_t hFaults) {
    tft.fillRect(0, 300, 480, 20, (wFaults || hFaults) ? C_RED : C_NAVY);
    tft.setTextColor(C_WHITE); tft.setTextSize(1);

    if (!wFaults && !hFaults) {
        tft.setCursor(2, 306); tft.print("No faults");
        return;
    }

    // Build fault name for current scroll position
    uint8_t idx = 0;
    char buf[40]; buf[0] = 0;
    for (uint8_t b = 0; b < 32; b++) {
        if (wFaults & (1UL << b)) {
            if (idx == faultBarIdx) { strncpy(buf, faultNameW(1UL << b), 39); break; }
            idx++;
        }
    }
    if (!buf[0]) {
        for (uint8_t b = 0; b < 32; b++) {
            if (hFaults & (1UL << b)) {
                if (idx == faultBarIdx) { strncpy(buf, faultNameH(1UL << b), 39); break; }
                idx++;
            }
        }
    }
    if (!buf[0]) { faultBarIdx = 0; return; }

    // Append elapsed time
    char tbuf[50];
    snprintf(tbuf, sizeof(tbuf), "%s  [%lus]", buf, (millis() / 1000));
    tft.setCursor(2, 306); tft.print(tbuf);

    // Scroll every 3s
    if (millis() - faultScrollMs >= 3000) {
        faultScrollMs = millis();
        faultBarIdx = (faultBarIdx + 1) % max(idx, (uint8_t)1);
    }
}

// ── Page 1: Heating System ────────────────────────────────

void drawPage1(uint32_t wF) {
    tft.fillRect(0, 20, 480, 280, C_BLACK);
    tft.setTextSize(1);

    auto printRow = [&](uint8_t row, const char* lbl, int16_t val, bool fault) {
        uint16_t y = 24 + row * 16;
        tft.setTextColor(C_DKGRAY); tft.setCursor(4, y); tft.print(lbl);
        tft.setTextColor(fault ? C_RED : C_CYAN); tft.setCursor(130, y);
        if (fault) tft.print("FAULT");
        else { tft.print(val / 10.0f, 1); tft.print(" C"); }
    };

    // W temperatures
    printRow(0,  "Solar Hot",     lastWPkt.tempSolarHot,    lastWPkt.tempSolarHot    == TEMP_FAULT);
    printRow(1,  "Solar Cold",    lastWPkt.tempSolarCold,   lastWPkt.tempSolarCold   == TEMP_FAULT);
    printRow(2,  "UFH Supply",    lastWPkt.tempUFHSupply,   lastWPkt.tempUFHSupply   == TEMP_FAULT);
    printRow(3,  "UFH Post TMV",  lastWPkt.tempUFHPostTMV,  lastWPkt.tempUFHPostTMV  == TEMP_FAULT);
    printRow(4,  "Workshop Air",  lastWPkt.tempWorkshopAir, lastWPkt.tempWorkshopAir == TEMP_FAULT);
    printRow(5,  "Outside Air",   lastWPkt.tempOutsideAir,  lastWPkt.tempOutsideAir  == TEMP_FAULT);

    // H temperatures
    for (uint8_t i = 0; i < H_NUM_SENSORS; i++) {
        static const char* names[H_NUM_SENSORS] = {
            "Tank Bot","Tank Mid","Tank Top","Hot Pipe","Cold Pipe","Htr Out" };
        int16_t enc = sFault[i] ? TEMP_FAULT : (int16_t)(sTemp[i] * 10.0f);
        printRow(6 + i, names[i], enc, sFault[i]);
    }

    // Power row
    uint16_t y = 24 + 12 * 16;
    tft.setTextColor(C_DKGRAY); tft.setCursor(4, y); tft.print("Heater");
    tft.setTextColor(C_CYAN); tft.setCursor(130, y);
    tft.print(heaterRunning ? (heaterTargetPct * 3000 / 100) : 0); tft.print("W");

    tft.setCursor(250, y); tft.setTextColor(C_DKGRAY); tft.print("Solar Pump");
    tft.setCursor(370, y); tft.setTextColor(C_CYAN);
    tft.print(lastWPkt.solarPumpDutyPct); tft.print("%");

    // Valve states
    y = 24 + 13 * 16;
    tft.setTextColor(C_DKGRAY); tft.setCursor(4, y); tft.print("Valves:");
    tft.setCursor(60, y);
    auto vStr = [&](const char* n, bool open) {
        tft.setTextColor(open ? C_GREEN : C_DKGRAY); tft.print(n); tft.print(' ');
    };
    vStr("UFH", lastWPkt.valveStates & VSTATE_UFH_COLD_OPEN);
    vStr("SOL", lastWPkt.valveStates & VSTATE_SOLAR_COLD_OPEN);
    vStr("VAC", lastWPkt.valveStates & VSTATE_VAC_ISO_OPEN);
    vStr("LB",  logBurnerCold.isOpen);
    vStr("BOT", botTankValve.isOpen);
    vStr("2PT", twoPortValve.isOpen);

    // Winch state
    y = 24 + 14 * 16;
    tft.setTextColor(C_DKGRAY); tft.setCursor(4, y); tft.print("Winch:");
    tft.setTextColor(C_CYAN); tft.setCursor(60, y);
    const char* ws[] = {"STOP","OPEN","CLOSE"};
    tft.print(ws[lastWPkt.winchState]);
    if (lastWPkt.winchReedFlags & WREED_SAFETY_LIMIT) {
        tft.setTextColor(C_RED); tft.print(" OVER-OPEN!");
    }
}

// ── Page 2: Power & Inverter ──────────────────────────────

void drawPage2() {
    tft.fillRect(0, 20, 480, 280, C_BLACK);
    tft.setTextSize(1);

    auto row = [&](uint8_t r, const char* lbl, const char* val) {
        uint16_t y = 24 + r * 18;
        tft.setTextColor(C_DKGRAY); tft.setCursor(4, y); tft.print(lbl);
        tft.setTextColor(C_CYAN);   tft.setCursor(160, y); tft.print(val);
    };
    char buf[24];

    snprintf(buf, sizeof(buf), "%dW / %dW", lastWPkt.pvString1W, lastWPkt.pvString2W);
    row(0, "PV Strings", buf);
    snprintf(buf, sizeof(buf), "%dW", lastWPkt.pvTotalW);
    row(1, "PV Total", buf);
    snprintf(buf, sizeof(buf), "%+dW", lastWPkt.pvExportW);
    row(2, "Grid Export", buf);
    snprintf(buf, sizeof(buf), "%+dW", lastWPkt.gridImportW);
    row(3, "Grid Import", buf);
    snprintf(buf, sizeof(buf), "%dW", lastWPkt.loadW);
    row(4, "Load", buf);
    snprintf(buf, sizeof(buf), "%.1fV", (double)(lastWPkt.gridVoltage_dV / 10.0f));
    row(5, "Grid Voltage", buf);
    snprintf(buf, sizeof(buf), "%.2fHz", (double)(lastWPkt.gridFreq_cHz / 100.0f));
    row(6, "Grid Freq", buf);
    snprintf(buf, sizeof(buf), "%d%%", lastWPkt.battSOC);
    row(7, "Battery SOC", buf);
    snprintf(buf, sizeof(buf), "%+dW", lastWPkt.battW);
    row(8, "Battery Power", buf);
    snprintf(buf, sizeof(buf), "%.1fV", (double)(lastWPkt.battVoltage_dV / 10.0f));
    row(9, "Battery Voltage", buf);
    snprintf(buf, sizeof(buf), "%dWh", lastWPkt.energyTodayWh);
    row(10, "Energy Today", buf);
    if (lastWPkt.inverterTemp_dC == TEMP_FAULT) strcpy(buf, "FAULT");
    else snprintf(buf, sizeof(buf), "%.1fC", (double)(lastWPkt.inverterTemp_dC / 10.0f));
    row(11, "Inverter Temp", buf);
    snprintf(buf, sizeof(buf), "%d%s", lastWPkt.inverterStatus,
             lastWPkt.inverterFaultCode ? " FAULT" : " OK");
    row(12, "Inverter Status", buf);
    snprintf(buf, sizeof(buf), "%.2fV", (double)busVoltageV);
    row(13, "15V Bus", buf);
}

// ── Page 3: Fault History ─────────────────────────────────

uint8_t faultHistScrollOffset = 0;
static const uint8_t FAULT_VISIBLE = 12;

void drawPage3() {
    tft.fillRect(0, 20, 480, 280, C_BLACK);
    tft.setTextSize(1);

    if (faultLogCount == 0) {
        tft.setTextColor(C_DKGRAY); tft.setCursor(10, 40);
        tft.print(F("No faults recorded since restart"));
        return;
    }

    for (uint8_t i = 0; i < FAULT_VISIBLE && (faultHistScrollOffset + i) < faultLogCount; i++) {
        uint8_t idx = faultLogCount - 1 - faultHistScrollOffset - i; // most recent first
        FaultEntry& e = faultLog[idx];
        uint16_t y = 24 + i * 22;
        bool active = (e.resolvedMs == 0);

        tft.setTextColor(active ? C_RED : C_DKGRAY);
        tft.setCursor(4, y);
        const char* name = e.isH ? faultNameH(e.faultMask) : faultNameW(e.faultMask);
        tft.print(name);

        tft.setTextColor(C_LTGRAY); tft.setCursor(160, y);
        uint32_t dur = active ? (millis() - e.onsetMs) : (e.resolvedMs - e.onsetMs);
        char buf[24];
        snprintf(buf, sizeof(buf), active ? "+%lus" : "%lus", dur / 1000);
        tft.print(buf);
    }
}

// ── Page 4: System Controls ───────────────────────────────

struct CtrlItem {
    const char* label;
    uint8_t     valueType; // 0=enum, 1=float×2, 2=int, 3=action
};

static const CtrlItem PAGE4_ITEMS[] = {
    { "Boost",            0 },
    { "Boost Target",     1 },
    { "Mode",             0 },
    { "Manual Heater",    0 },
    { "Night Cooling",    0 },
    { "Solar Target",     0 },
    { "Fan Base Speed",   2 },
    { "Fan Full Timer",   2 },
    { "Fan Base Timer",   2 },
    { "Display Bright",   2 },
    { "SD Safe Remove",   3 },
    { "Alert Reset",      3 },  // only shown when fault active
};
static const uint8_t PAGE4_COUNT = 12;

void getPage4Value(uint8_t item, char* buf, uint8_t bufSz) {
    switch (item) {
        case 0: { const char* bm[] = {"Off","Boost-5am","8hr"}; strlcpy(buf, bm[boostMode], bufSz); break; }
        case 1: snprintf(buf, bufSz, "%.1fC", (double)boostTarget); break;
        case 2: strlcpy(buf, systemMode == MODE_WINTER ? "Winter" : "Summer", bufSz); break;
        case 3: { const char* mm[] = {"Off","SOC-Lim","Override"}; strlcpy(buf, mm[manualHeaterMode], bufSz); break; }
        case 4: strlcpy(buf, nightCoolingEnabled ? "On" : "Off", bufSz); break;
        case 5: strlcpy(buf, solarTargetMode == SOLAR_TANK_PLUS5 ? "Tank+5" : "Max", bufSz); break;
        case 6: snprintf(buf, bufSz, "%d%%", lastWPkt.fanDutyPct); break;
        case 7: snprintf(buf, bufSz, "%luh", lastWPkt.fanFullTimerSecs / 3600); break;
        case 8: snprintf(buf, bufSz, "%lud", lastWPkt.fanBaseTimerSecs / 86400); break;
        case 9: snprintf(buf, bufSz, "%d%%", displayBrightness); break;
        case 10: strlcpy(buf, sdAvailable ? "Eject" : "Ejected", bufSz); break;
        case 11: strlcpy(buf, "Confirm", bufSz); break;
        default: buf[0] = 0;
    }
}

void page4Adjust(uint8_t item, int8_t dir) {
    switch (item) {
        case 0: boostMode = (BoostMode)((boostMode + 3 + dir) % 3); break;
        case 1: boostTarget = constrain(boostTarget + dir * 0.5f, 13.0f, 20.0f); break;
        case 2:
            systemMode = (systemMode == MODE_WINTER) ? MODE_SUMMER : MODE_WINTER;
            markSettingsDirty();
            break;
        case 3: manualHeaterMode = (ManualHeaterMode)((manualHeaterMode + 3 + dir) % 3); break;
        case 4: nightCoolingEnabled = !nightCoolingEnabled; markSettingsDirty(); break;
        case 5: solarTargetMode = (solarTargetMode == SOLAR_TANK_PLUS5) ? SOLAR_MAX : SOLAR_TANK_PLUS5; markSettingsDirty(); break;
        case 6: { // fan base speed — transmitted to W in H→W packet
            uint8_t fs = EEPROM.read(EE_FAN_BASE_SPEED);
            fs = constrain((int)fs + dir * 10, 0, 100);
            EEPROM.update(EE_FAN_BASE_SPEED, fs);
            break;
        }
        case 7: // fan full timer: accumulate ±1hr delta; sent to W on next packet and cleared
            pendingFanFullDeltaHr = (int8_t)constrain((int)pendingFanFullDeltaHr + dir, -24, 24);
            break;
        case 8: // fan base timer: accumulate ±1 day delta; sent to W on next packet and cleared
            pendingFanBaseDeltaDay = (int8_t)constrain((int)pendingFanBaseDeltaDay + dir, -30, 30);
            break;
        case 9:
            displayBrightness = constrain((int)displayBrightness + dir * 10, 10, 100);
            markSettingsDirty();
            setBacklight(displayBrightness);
            break;
    }
}

void page4Action(uint8_t item) {
    if (item == 10) safeEjectSD();
    if (item == 11) { alertResetSeqTx++; } // H signals alert reset to W
}

uint8_t page4VisibleCount() {
    // Alert reset only visible when a fault is active
    bool faultActive = (hFaultFlags != 0 || lastWPkt.wFaultFlags != 0);
    return faultActive ? PAGE4_COUNT : PAGE4_COUNT - 1;
}

void drawPage4() {
    tft.fillRect(0, 20, 480, 280, C_BLACK);
    tft.setTextSize(1);
    uint8_t visCount = page4VisibleCount();
    for (uint8_t i = 0; i < visCount; i++) {
        uint16_t y   = 24 + i * 22;
        bool     sel = (navMode != NAV_PAGE && selectedItem == i);
        tft.fillRect(0, y - 2, 480, 20, sel ? C_NAVY : C_BLACK);
        if (sel) tft.drawRect(0, y - 2, 480, 20, C_AMBER);
        tft.setTextColor(C_LTGRAY); tft.setCursor(4, y); tft.print(PAGE4_ITEMS[i].label);
        char buf[20]; getPage4Value(i, buf, sizeof(buf));
        tft.setTextColor(C_CYAN); tft.setCursor(200, y); tft.print(buf);
    }
}

// ── Page 5: Valve States & Manual Override ────────────────

bool manualOverrideActive = false;
uint8_t overrideValveStates = 0; // OVER_* bitmask

void drawPage5() {
    tft.fillRect(0, 20, 480, 280, C_BLACK);
    tft.setTextSize(1);

    if (manualOverrideActive) {
        tft.fillRect(0, 20, 480, 16, C_RED);
        tft.setTextColor(C_WHITE); tft.setCursor(4, 22);
        tft.print(F("MANUAL OVERRIDE ACTIVE — automatic valve control suspended"));
    }

    auto vRow = [&](uint8_t r, const char* lbl, bool open, bool canOverride) {
        uint16_t y  = 40 + r * 22;
        bool     sel = (navMode != NAV_PAGE && manualOverrideActive && selectedItem == r);
        tft.fillRect(0, y - 2, 480, 20, sel ? C_NAVY : C_BLACK);
        tft.setTextColor(C_DKGRAY); tft.setCursor(4, y); tft.print(lbl);
        tft.setTextColor(open ? C_GREEN : C_RED);
        tft.setCursor(200, y); tft.print(open ? "OPEN" : "CLOSED");
    };

    vRow(0, "UFH Cold (W)",      lastWPkt.valveStates & VSTATE_UFH_COLD_OPEN,   true);
    vRow(1, "Solar Cold (W)",    lastWPkt.valveStates & VSTATE_SOLAR_COLD_OPEN,  true);
    vRow(2, "Vac Isolation (W)", lastWPkt.valveStates & VSTATE_VAC_ISO_OPEN,     true);
    vRow(3, "Log Burner Cold",   logBurnerCold.isOpen,  false);
    vRow(4, "Bottom-of-Tank",    botTankValve.isOpen,   false);
    vRow(5, "2-Port",            twoPortValve.isOpen,   false);

    // Winch details
    uint16_t y = 40 + 7 * 22;
    tft.setTextColor(C_DKGRAY); tft.setCursor(4, y); tft.print("Winch:");
    tft.setTextColor(C_CYAN); tft.setCursor(60, y);
    const char* ws[] = {"Idle","Opening","Closing"};
    tft.print(ws[min(lastWPkt.winchState, (uint8_t)2)]);
    tft.setCursor(160, y);
    if (lastWPkt.winchReedFlags & WREED_FULLY_OPEN)   { tft.setTextColor(C_AMBER); tft.print(" OPEN"); }
    if (lastWPkt.winchReedFlags & WREED_FULLY_CLOSED)  { tft.setTextColor(C_GREEN); tft.print(" CLSD"); }
    if (lastWPkt.winchReedFlags & WREED_MANUAL_LOCK)   { tft.setTextColor(C_AMBER); tft.print(" LOCK"); }
    if (lastWPkt.winchReedFlags & WREED_SAFETY_LIMIT)  { tft.setTextColor(C_RED);   tft.print(" OVER!"); }
}

// ── Full page dispatch ────────────────────────────────────

void drawFullPage() {
    tft.fillScreen(C_BLACK);
    drawStatusBar();
    switch (currentPage) {
        case 1: drawPage1(lastWPkt.wFaultFlags); break;
        case 2: drawPage2(); break;
        case 3: drawPage3(); break;
        case 4: drawPage4(); break;
        case 5: drawPage5(); break;
    }
    drawFaultBar(lastWPkt.wFaultFlags, hFaultFlags);
    needFullRedraw = false;
}

// ── Button handling ───────────────────────────────────────

void handleSelectDouble() {
    // Toggle Boost at 5am from any page
    boostMode = (boostMode == BOOST_5AM) ? BOOST_OFF : BOOST_5AM;
}

void handleButtons() {
    bool selP  = btnSelect.pressed();
    bool upP   = btnUp.pressed();
    bool downP = btnDown.pressed();
    if (!selP && !upP && !downP) return;

    wakeDisplay();
    unsigned long now = millis();
    static bool atBoundary = false; // true = scrolled past list end; SELECT exits, UP/DOWN wraps

    // Double-press SELECT in page scroll mode
    if (selP && navMode == NAV_PAGE) {
        if (now - lastSelectMs < 400) { handleSelectDouble(); needFullRedraw = true; }
        lastSelectMs = now;
    }

    // Inactivity exit to page scroll
    if (now - lastButtonMs > DISPLAY_INACTIVITY_MS && navMode != NAV_PAGE) {
        navMode = NAV_PAGE; atBoundary = false; needFullRedraw = true; return;
    }

    switch (navMode) {
        case NAV_PAGE:
            if (upP)   { if (currentPage > 1) currentPage--; needFullRedraw = true; }
            if (downP) { if (currentPage < 5) currentPage++; needFullRedraw = true; }
            if (selP)  { navMode = NAV_ITEM; selectedItem = 0; atBoundary = false; needFullRedraw = true; }
            break;

        case NAV_ITEM: {
            uint8_t maxItems = (currentPage == 4) ? page4VisibleCount() :
                               (currentPage == 5 && manualOverrideActive) ? 6 : 0;

            if (atBoundary) {
                // Boundary state: SELECT exits, UP wraps to last item, DOWN wraps to first
                if (selP)  { navMode = NAV_PAGE; atBoundary = false; needFullRedraw = true; }
                if (upP)   { selectedItem = (maxItems > 0) ? maxItems - 1 : 0; atBoundary = false; needFullRedraw = true; }
                if (downP) { selectedItem = 0;                                  atBoundary = false; needFullRedraw = true; }
            } else {
                if (upP) {
                    if (selectedItem > 0) { selectedItem--; needFullRedraw = true; }
                    else                  { atBoundary = true; needFullRedraw = true; }
                }
                if (downP) {
                    if (selectedItem < maxItems - 1) { selectedItem++; needFullRedraw = true; }
                    else                              { atBoundary = true; needFullRedraw = true; }
                }
                if (selP) {
                    if (currentPage == 4) {
                        if (PAGE4_ITEMS[selectedItem].valueType == 3) page4Action(selectedItem);
                        else { navMode = NAV_OPTION; needFullRedraw = true; }
                    } else if (currentPage == 5 && !manualOverrideActive) {
                        manualOverrideActive = true; needFullRedraw = true;
                    } else if (currentPage == 5 && manualOverrideActive) {
                        if (selectedItem < 3) {
                            uint8_t mask = (1 << selectedItem);
                            overrideValveStates ^= mask;
                            needFullRedraw = true;
                        }
                    }
                }
            }
            break;
        }

        case NAV_OPTION:
            if (upP)   { page4Adjust(selectedItem, -1); needFullRedraw = true; }
            if (downP) { page4Adjust(selectedItem,  1); needFullRedraw = true; }
            if (selP)  { navMode = NAV_ITEM; needFullRedraw = true; }
            break;
    }
}

void updateDisplayBanners() {
    // Manual override banner (all pages)
    if (manualOverrideActive) {
        tft.fillRect(0, 20, 480, 14, C_RED);
        tft.setTextColor(C_WHITE); tft.setTextSize(1); tft.setCursor(2, 22);
        tft.print(F("MANUAL OVERRIDE ACTIVE"));
    }
    // Manual heater banner
    if (manualHeaterMode == MHM_OVERRIDE_SOC) {
        tft.fillRect(0, 34, 480, 14, C_RED);
        tft.setTextColor(C_WHITE); tft.setTextSize(1); tft.setCursor(2, 36);
        tft.print(F("MANUAL HEATER — FULL 3kW — no SOC limit"));
    } else if (manualHeaterMode == MHM_SOC_LIMITED) {
        tft.fillRect(0, 34, 480, 14, C_AMBER);
        tft.setTextColor(C_BLACK); tft.setTextSize(1); tft.setCursor(2, 36);
        tft.print(F("MANUAL HEATER — SOC limited"));
    }
}

// ============================================================
//  RS485 INTER-CONTROLLER LINK
// ============================================================

uint8_t   txSeqNum       = 0;
uint8_t   missedPackets  = 0;
bool      rs485Fault     = false;
PktReceiver pktRx;

void sendHToWPacket(bool timeSyncReq) {
    HToWPacket pkt;
    memset(&pkt, 0, sizeof(pkt));

    // H temperatures
    pkt.tempTankBot    = sFault[H_SENSOR_TANK_BOT]  ? TEMP_FAULT : (int16_t)(sTemp[H_SENSOR_TANK_BOT]  * 10);
    pkt.tempTankMid    = sFault[H_SENSOR_TANK_MID]  ? TEMP_FAULT : (int16_t)(sTemp[H_SENSOR_TANK_MID]  * 10);
    pkt.tempTankTop    = sFault[H_SENSOR_TANK_TOP]  ? TEMP_FAULT : (int16_t)(sTemp[H_SENSOR_TANK_TOP]  * 10);
    pkt.tempHotPipe    = sFault[H_SENSOR_HOT_PIPE]  ? TEMP_FAULT : (int16_t)(sTemp[H_SENSOR_HOT_PIPE]  * 10);
    pkt.tempColdPipe   = sFault[H_SENSOR_COLD_PIPE] ? TEMP_FAULT : (int16_t)(sTemp[H_SENSOR_COLD_PIPE] * 10);
    pkt.tempHeaterOut  = sFault[H_SENSOR_HEATER_OUT]? TEMP_FAULT : (int16_t)(sTemp[H_SENSOR_HEATER_OUT]* 10);

    pkt.heaterPowerPct   = heaterRunning ? heaterTargetPct : 0;
    pkt.heaterRestricted = heaterOvheatWarn ? 1 : 0;

    pkt.systemMode          = (uint8_t)(pvExportOverride ? MODE_SUMMER : systemMode);
    pkt.boostMode           = (uint8_t)boostMode;
    pkt.ufhStopTemp_dC      = (int16_t)(getUFHStopTemp() * 10.0f);
    pkt.morningHeatActive   = morningHeatActive ? 1 : 0;
    pkt.summerStartupPhase  = summerStartupPhase;
    pkt.nightCoolingEnabled = nightCoolingEnabled ? 1 : 0;
    pkt.solarTargetMode     = (uint8_t)solarTargetMode;
    pkt.manualHeaterMode    = (uint8_t)manualHeaterMode;

    pkt.fanBaseSpeedPct      = EEPROM.read(EE_FAN_BASE_SPEED);
    pkt.fanFullTimerDeltaHr  = pendingFanFullDeltaHr;
    pkt.fanBaseTimerDeltaDay = pendingFanBaseDeltaDay;
    pendingFanFullDeltaHr    = 0;
    pendingFanBaseDeltaDay   = 0;

    pkt.overrideActive  = manualOverrideActive ? 1 : 0;
    pkt.overrideValves  = overrideValveStates;
    pkt.alertResetSeq   = alertResetSeqTx;
    pkt.hFaultFlags     = hFaultFlags;

    // Time sync: send whenever W requested it, or on hourly interval
    if (timeSyncReq && rtcValid) {
        pkt.timeSyncValid = 1;
        pkt.syncYear      = (uint8_t)(rtcNow.year() - 2000);
        pkt.syncMonth     = rtcNow.month();
        pkt.syncDay       = rtcNow.day();
        pkt.syncHour      = rtcNow.hour();
        pkt.syncMinute    = rtcNow.minute();
        pkt.syncSecond    = rtcNow.second();
    }

#ifdef DEBUG_SERIAL
    pkt.calPumpActive   = (calPumpPhase != CALP_IDLE && calPumpPhase != CALP_DONE) ? 1 : 0;
    pkt.calSolarTargetC = (calPumpPhase == CALP_STABILIZE) ? (int16_t)(calSolarStepC * 10) : 900;
#endif

    uint8_t frame[PKT_MAX_FRAME];
    uint16_t len = pktEncode(frame, sizeof(frame), PKT_DIR_HW, txSeqNum++,
                             &pkt, sizeof(pkt));
    digitalWrite(PIN_RS485_DE_LINK, HIGH);
    Serial1.write(frame, len);
    Serial1.flush();
    digitalWrite(PIN_RS485_DE_LINK, LOW);
}

bool receiveWToHPacket() {
    unsigned long deadline = millis() + RS485_RX_TIMEOUT_MS;
    uint8_t  outDir; uint8_t outSeq;
    uint8_t *payload; uint16_t payLen;

    while (millis() < deadline) {
        if (!Serial1.available()) continue;
        if (pktRx.feed((uint8_t)Serial1.read(), outDir, outSeq, payload, payLen)) {
            if (outDir != PKT_DIR_WH || payLen != sizeof(WToHPacket)) continue;
            memcpy(&lastWPkt, payload, sizeof(WToHPacket));
#ifdef DEBUG_SERIAL
            if (simPVExportActive) lastWPkt.pvExportW = simPVExportVal;
            if (simBattSOCActive)  lastWPkt.battSOC   = simBattSOCVal;
#endif
            hasWPkt     = true;
            lastWPktMs  = millis();
            missedPackets = 0;
            if (rs485Fault) { rs485Fault = false; clearFaultH(FAULT_H_RS485_COMMS); }

            // Update Growatt-derived heater duty on every new W packet
            // SOC is stale when W's Growatt comms are faulted
            bool battSocStale = (lastWPkt.wFaultFlags & FAULT_W_GROWATT_COMMS) != 0;
            bool wSolarFault  = (lastWPkt.wFaultFlags & (FAULT_W_SENSOR_SOLAR_HOT | FAULT_W_SENSOR_SOLAR_COLD)) != 0;
#ifdef DEBUG_SERIAL
            if (calPumpPhase == CALP_IDLE || calPumpPhase == CALP_DONE)
#endif
            updateHeaterDuty(lastWPkt.pvExportW, lastWPkt.gridImportW,
                             lastWPkt.battSOC, battSocStale, manualHeaterMode, wSolarFault);
            return true;
        }
    }
    missedPackets++;
    if (missedPackets >= COMMS_FAULT_THRESHOLD && !rs485Fault) {
        rs485Fault = true; setFaultH(FAULT_H_RS485_COMMS);
    }
    return false;
}

unsigned long lastInterCtrlMs = 0;
unsigned long lastTimeSyncSentMs = 0;

void doInterControllerExchange() {
    bool needTimeSync = hasWPkt && lastWPkt.requestTimeSync
                         && (millis() - lastTimeSyncSentMs > 5000);
    if (needTimeSync) lastTimeSyncSentMs = millis();

    // H waits: W transmits first, then H replies immediately
    bool gotPkt = receiveWToHPacket();
    sendHToWPacket(needTimeSync);
    (void)gotPkt;
}

#ifdef DEBUG_SERIAL
// ============================================================
//  SERIAL DEBUG COMMAND FUNCTIONS
// ============================================================

static void hDbgPrintTemp(uint8_t i, const __FlashStringHelper* name) {
    Serial.print(F("  ")); Serial.print(name); Serial.print(F(": "));
    if (hSimulate[i]) Serial.print(F("SIM "));
    if (sFault[i]) Serial.println(F("FAULT"));
    else           Serial.println(sTemp[i], 1);
}

static void dbgTemps() {
    hDbgPrintTemp(H_SENSOR_TANK_BOT,   F("tank_bot"));
    hDbgPrintTemp(H_SENSOR_TANK_MID,   F("tank_mid"));
    hDbgPrintTemp(H_SENSOR_TANK_TOP,   F("tank_top"));
    hDbgPrintTemp(H_SENSOR_HOT_PIPE,   F("hot_pipe"));
    hDbgPrintTemp(H_SENSOR_COLD_PIPE,  F("cold_pipe"));
    hDbgPrintTemp(H_SENSOR_HEATER_OUT, F("htr_out"));
    if (hasWPkt) {
        auto pW = [](const __FlashStringHelper* n, int16_t v) {
            Serial.print(F("  ")); Serial.print(n); Serial.print(F("(W): "));
            if (v == TEMP_FAULT) Serial.println(F("FAULT"));
            else { Serial.print(v / 10.0f, 1); Serial.println(F("C")); }
        };
        pW(F("solar_hot"),    lastWPkt.tempSolarHot);
        pW(F("solar_cold"),   lastWPkt.tempSolarCold);
        pW(F("workshop_air"), lastWPkt.tempWorkshopAir);
    }
}

static void dbgValves() {
    Serial.print(F("  log_cold: ")); Serial.println(logBurnerCold.isOpen ? F("OPEN")     : F("CLOSED"));
    Serial.print(F("  bot_tank: ")); Serial.println(botTankValve.isOpen  ? F("OPEN")     : F("CLOSED"));
    Serial.print(F("  two_port: ")); Serial.println(twoPortValve.isOpen  ? F("OPEN(htr)") : F("CLOSED(mid)"));
    if (hasWPkt) {
        Serial.print(F("  W ufh_cold:  ")); Serial.println((lastWPkt.valveStates & VSTATE_UFH_COLD_OPEN)   ? F("OPEN") : F("CLOSED"));
        Serial.print(F("  W solar_cold:")); Serial.println((lastWPkt.valveStates & VSTATE_SOLAR_COLD_OPEN) ? F("OPEN") : F("CLOSED"));
        Serial.print(F("  W vac_iso:   ")); Serial.println((lastWPkt.valveStates & VSTATE_VAC_ISO_OPEN)    ? F("OPEN") : F("CLOSED"));
    }
}

static void dbgFaults() {
    bool any = false;
    #define HF(m,n) if(hasFaultH(m)){Serial.println(F("  " n));any=true;}
    HF(FAULT_H_HEATER_OVERHEAT_WARN, "HEATER_OVERHEAT_WARN")
    HF(FAULT_H_HEATER_OVERHEAT_SHUT, "HEATER_OVERHEAT_SHUT")
    HF(FAULT_H_HEATER_ELEMENT_FAIL,  "HEATER_ELEMENT_FAIL")
    HF(FAULT_H_RS485_COMMS,          "RS485_COMMS")
    HF(FAULT_H_BUS_VOLTAGE_LOW,      "BUS_VOLTAGE_LOW")
    HF(FAULT_H_GRID_OUTAGE,          "GRID_OUTAGE")
    HF(FAULT_H_SENSOR_TANK_BOT,      "SENSOR_TANK_BOT")
    HF(FAULT_H_SENSOR_TANK_MID,      "SENSOR_TANK_MID")
    HF(FAULT_H_SENSOR_TANK_TOP,      "SENSOR_TANK_TOP")
    HF(FAULT_H_SENSOR_HOT_PIPE,      "SENSOR_HOT_PIPE")
    HF(FAULT_H_SENSOR_COLD_PIPE,     "SENSOR_COLD_PIPE")
    HF(FAULT_H_SENSOR_HEATER_OUT,    "SENSOR_HEATER_OUT")
    #undef HF
    if (hasWPkt && lastWPkt.wFaultFlags) {
        Serial.print(F("  W faults: 0x")); Serial.println(lastWPkt.wFaultFlags, HEX);
    }
    if (!any && (!hasWPkt || !lastWPkt.wFaultFlags)) Serial.println(F("  none"));
}

static void dbgMode() {
    Serial.print(F("  system_mode:  ")); Serial.println(systemMode == MODE_WINTER ? F("WINTER") : F("SUMMER"));
    Serial.print(F("  boost:        "));
    if      (boostMode == BOOST_5AM) Serial.println(F("5am"));
    else if (boostMode == BOOST_8HR) Serial.println(F("8hr"));
    else                             Serial.println(F("off"));
    Serial.print(F("  morning_heat: ")); Serial.println(morningHeatActive  ? F("active") : F("off"));
    Serial.print(F("  summer_phase: ")); Serial.println(summerStartupPhase);
    Serial.print(F("  pv_override:  ")); Serial.println(pvExportOverride   ? F("yes")    : F("no"));
    Serial.print(F("  log_burner:   ")); Serial.println(logBurnerHot       ? F("HOT")    : F("cold"));
    if (simLogBurnerActive) Serial.println(F("  (log_burner SIM active)"));
    Serial.print(F("  heater:       ")); Serial.print(heaterRunning ? F("ON ") : F("off "));
    Serial.print(heaterTargetPct); Serial.println(F("%"));
    Serial.print(F("  htr_lockout:  ")); Serial.println(heaterHardLockout  ? F("YES")    : F("no"));
    Serial.print(F("  rs485:        ")); Serial.println(rs485Fault         ? F("FAULT")  : F("ok"));
    Serial.print(F("  rtc_valid:    ")); Serial.println(rtcValid            ? F("yes")    : F("no"));
}

static void dbgHeater() {
    Serial.print(F("  running:   ")); Serial.println(heaterRunning     ? F("yes")  : F("no"));
    Serial.print(F("  duty:      ")); Serial.print(heaterTargetPct);   Serial.println(F("%"));
    Serial.print(F("  power_est: ")); Serial.print(heaterTargetPct * 3000 / 100); Serial.println(F("W"));
    Serial.print(F("  lockout:   ")); Serial.println(heaterHardLockout ? F("YES")  : F("no"));
    Serial.print(F("  ovht_warn: ")); Serial.println(heaterOvheatWarn  ? F("YES")  : F("no"));
    Serial.print(F("  grid:      ")); Serial.println(gridPresent       ? F("ok")   : F("OUTAGE"));
    Serial.print(F("  enabled:   ")); Serial.println(HEATER_ENABLED    ? F("yes")  : F("NO (commissioning flag)"));
    if (simPVExportActive) { Serial.print(F("  SIM pv_export=")); Serial.println(simPVExportVal); }
    if (simBattSOCActive)  { Serial.print(F("  SIM batt_soc=")); Serial.println(simBattSOCVal); }
}

static void dbgBus() {
    Serial.print(F("  bus_v:   ")); Serial.print(busVoltageV, 2); Serial.println(F("V"));
    Serial.print(F("  psu_12v: ")); Serial.println(psu12vActive ? F("ON")    : F("off"));
    Serial.print(F("  bus_low: ")); Serial.println(busLowActive ? F("FAULT") : F("ok"));
}

static void dbgRTC() {
    Serial.print(F("  rtc_valid: ")); Serial.println(rtcValid ? F("yes") : F("no"));
    if (rtcValid) {
        char buf[24];
        snprintf(buf, sizeof(buf), "  %02u:%02u:%02u  %02u/%02u/%04u",
                 rtcHour(), rtcMinute(), rtcSecond(),
                 rtcNow.day(), rtcNow.month(), rtcNow.year());
        Serial.println(buf);
    }
    Serial.print(F("  batt_low:  ")); Serial.println(rtcBatteryLow ? F("yes") : F("no"));
}

static void dbgSet(char* key, char* val) {
    float fval = atof(val);
    uint8_t si = 255;
    if      (!strcmp_P(key, PSTR("tank_bot")))  si = H_SENSOR_TANK_BOT;
    else if (!strcmp_P(key, PSTR("tank_mid")))  si = H_SENSOR_TANK_MID;
    else if (!strcmp_P(key, PSTR("tank_top")))  si = H_SENSOR_TANK_TOP;
    else if (!strcmp_P(key, PSTR("hot_pipe")))  si = H_SENSOR_HOT_PIPE;
    else if (!strcmp_P(key, PSTR("cold_pipe"))) si = H_SENSOR_COLD_PIPE;
    else if (!strcmp_P(key, PSTR("htr_out")))   si = H_SENSOR_HEATER_OUT;

    if (si < H_NUM_SENSORS) {
        if (fval >= 999.0f) { hSimulate[si] = false; Serial.println(F("cleared")); }
        else { hSimulate[si] = true; hSim[si] = fval; Serial.println(F("ok")); }
        return;
    }
    if (!strcmp_P(key, PSTR("log_burner")))        { simLogBurnerActive = true;  simLogBurnerVal = (fval != 0); Serial.println(F("ok")); return; }
    if (!strcmp_P(key, PSTR("log_burner_clear")))  { simLogBurnerActive = false; Serial.println(F("cleared")); return; }
    if (!strcmp_P(key, PSTR("pv_export")))         { simPVExportActive  = true;  simPVExportVal  = (int16_t)fval; Serial.println(F("ok")); return; }
    if (!strcmp_P(key, PSTR("pv_export_clear")))   { simPVExportActive  = false; Serial.println(F("cleared")); return; }
    if (!strcmp_P(key, PSTR("batt_soc")))          { simBattSOCActive   = true;  simBattSOCVal   = (uint8_t)constrain((int)fval, 0, 100); Serial.println(F("ok")); return; }
    if (!strcmp_P(key, PSTR("batt_soc_clear")))    { simBattSOCActive   = false; Serial.println(F("cleared")); return; }
    Serial.print(F("unknown key: ")); Serial.println(key);
}

static void dbgScan() {
    uint8_t addr[8];
    uint8_t count = 0;
    oneWire.reset_search();
    Serial.println(F("Scanning 1-Wire bus..."));
    while (oneWire.search(addr)) {
        if (OneWire::crc8(addr, 7) != addr[7]) {
            Serial.println(F("  CRC error — skipping"));
            continue;
        }
        Serial.print(F("  { "));
        for (uint8_t i = 0; i < 8; i++) {
            Serial.print(F("0x"));
            if (addr[i] < 0x10) Serial.print(F("0"));
            Serial.print(addr[i], HEX);
            if (i < 7) Serial.print(F(", "));
        }
        Serial.print(F(" }  family=0x"));
        Serial.println(addr[0], HEX);
        count++;
    }
    if (count == 0) Serial.println(F("  none found"));
    else { Serial.print(F("  total: ")); Serial.println(count); }
}

static void calPrintDataPoint(uint8_t solarStep, uint8_t heaterPct,
                               bool hotFault, float hotPipe,
                               bool htrFault, float htrOut,
                               uint8_t pumpDuty) {
    Serial.print(F("CAL,"));
    Serial.print(solarStep); Serial.print(',');
    Serial.print(heaterPct); Serial.print(',');
    if (hotFault) Serial.print(F("NaN")); else Serial.print(hotPipe, 1);
    Serial.print(',');
    if (htrFault) Serial.print(F("NaN")); else Serial.print(htrOut, 1);
    Serial.print(',');
    Serial.println(pumpDuty);
}

static void updateCalPump() {
    if (calPumpPhase == CALP_IDLE || calPumpPhase == CALP_DONE) return;

    unsigned long now     = millis();
    bool hotFault = sFault[H_SENSOR_HOT_PIPE];
    bool htrFault = sFault[H_SENSOR_HEATER_OUT];
    float hotPipe = hotFault ? NAN : sTemp[H_SENSOR_HOT_PIPE];
    float htrOut  = htrFault ? NAN : sTemp[H_SENSOR_HEATER_OUT];
    uint8_t pumpDuty = hasWPkt ? lastWPkt.solarPumpDutyPct : 0;

    switch (calPumpPhase) {
        case CALP_STABILIZE:
            calHtrOverride  = false;
            heaterRunning   = false;
            heaterTargetPct = 0;
            PORTA &= ~(1 << PA5);
            if (!hotFault && fabsf(hotPipe - (float)calSolarStepC) < 2.0f) {
                if (calPhaseMs == 0) calPhaseMs = now;
                if (now - calPhaseMs >= 10000UL) {
                    Serial.print(F("CAL: hot_pipe stable at ")); Serial.print(hotPipe, 1);
                    Serial.print(F("C — target ")); Serial.print(calSolarStepC);
                    Serial.println(F("C, raising solar target to 90C, heater to 5%"));
                    calPhaseMs     = now;
                    calHeaterPct   = 5;
                    calHtrOverride = true;
                    calPumpPhase   = CALP_PRE_RAMP;
                }
            } else {
                calPhaseMs = 0;
            }
            break;

        case CALP_PRE_RAMP:
            calHtrOverride  = true;
            calHeaterPct    = 5;
            heaterRunning   = true;
            heaterTargetPct = 5;
            if (!htrFault && htrOut >= 87.0f) {
                Serial.println(F("CAL: htr_out >= 87C, starting 30s ramp"));
                Serial.println(F("CAL_HDR: solar_step_C,heater_pct,hot_pipe_C,htr_out_C,pump_pct"));
                calStepMs    = now;
                calHeaterPct = 5;
                calPumpPhase = CALP_RAMPING;
                calPrintDataPoint(calSolarStepC, calHeaterPct, hotFault, hotPipe, htrFault, htrOut, pumpDuty);
            } else if (now - calPhaseMs >= 300000UL) {
                Serial.println(F("CAL: TIMEOUT — htr_out did not reach 87C in 5min, aborting"));
                calPumpPhase   = CALP_DONE;
                calHtrOverride = false;
            }
            break;

        case CALP_RAMPING: {
            calHtrOverride  = true;
            heaterRunning   = true;
            heaterTargetPct = calHeaterPct;
            // Pump at ceiling — log current state as max for this solar step and advance
            if (pumpDuty >= 100) {
                Serial.print(F("CAL: pump 100% at ")); Serial.print(calHeaterPct);
                Serial.println(F("% heater — ceiling, advancing"));
                calPrintDataPoint(calSolarStepC, calHeaterPct, hotFault, hotPipe, htrFault, htrOut, 100);
                if (calSolarStepC < 80) {
                    calSolarStepC += 5;
                    calHeaterPct   = 0;
                    calHtrOverride = false;
                    calPhaseMs     = 0;
                    calPumpPhase   = CALP_STABILIZE;
                    Serial.print(F("CAL: step done — stabilising at ")); Serial.print(calSolarStepC); Serial.println(F("C"));
                } else {
                    Serial.println(F("CAL: all steps complete"));
                    calPumpPhase   = CALP_DONE;
                    calHtrOverride = false;
                }
                break;
            }
            // 30s sweep: 19 intervals between 5%..100% → 1578ms each
            if (now - calStepMs >= 1578UL) {
                calStepMs     = now;
                calHeaterPct  = min((uint8_t)(calHeaterPct + 5), (uint8_t)100);
                heaterTargetPct = calHeaterPct;
                calPrintDataPoint(calSolarStepC, calHeaterPct, hotFault, hotPipe, htrFault, htrOut, pumpDuty);
                if (calHeaterPct >= 100) {
                    if (calSolarStepC < 80) {
                        calSolarStepC += 5;
                        calHeaterPct   = 0;
                        calHtrOverride = false;
                        calPhaseMs     = 0;
                        calPumpPhase   = CALP_STABILIZE;
                        Serial.print(F("CAL: step done — stabilising at ")); Serial.print(calSolarStepC); Serial.println(F("C"));
                    } else {
                        Serial.println(F("CAL: all steps complete"));
                        calPumpPhase   = CALP_DONE;
                        calHtrOverride = false;
                    }
                }
            }
            break;
        }
        default: break;
    }
}

static void dbgCalPump() {
    if (!HEATER_ENABLED) {
        Serial.println(F("cal_pump: HEATER_ENABLED is false — set true in firmware before calibrating"));
        return;
    }
    if (calPumpPhase != CALP_IDLE && calPumpPhase != CALP_DONE) {
        Serial.println(F("cal_pump: already running — use cal_abort to stop"));
        return;
    }
    calPumpPhase   = CALP_STABILIZE;
    calSolarStepC  = 50;
    calHeaterPct   = 0;
    calHtrOverride = false;
    calPhaseMs     = 0;
    Serial.println(F("CAL: started — ensure summer mode active and solar pump running"));
    Serial.println(F("CAL: waiting for hot_pipe to stabilise at 50C"));
}

static void dbgCalAbort() {
    calPumpPhase   = CALP_IDLE;
    calHtrOverride = false;
    heaterRunning  = false;
    heaterTargetPct = 0;
    PORTA &= ~(1 << PA5);
    Serial.println(F("CAL: aborted — heater off, normal control restored"));
}

static void handleDebugCommand(char* buf) {
    char* cmd = buf;
    while (*cmd == ' ') cmd++;
    char* p = cmd;
    while (*p && *p != ' ') p++;
    char* arg1 = (*p == ' ') ? p + 1 : p;
    if (*p) *p = 0;
    while (*arg1 == ' ') arg1++;
    char* arg2 = arg1;
    while (*arg2 && *arg2 != ' ') arg2++;
    if (*arg2 == ' ') { *arg2++ = 0; while (*arg2 == ' ') arg2++; }

    if      (!strcmp_P(cmd, PSTR("help"))) {
        Serial.println(F("temps  valves  faults  mode  status  heater  bus  rtc  page <1-5>  scan"));
        Serial.println(F("set <sensor> <val>  (val=999 clears sim)"));
        Serial.println(F("  sensors: tank_bot tank_mid tank_top hot_pipe cold_pipe htr_out"));
        Serial.println(F("set log_burner|pv_export|batt_soc <val>"));
        Serial.println(F("set log_burner_clear|pv_export_clear|batt_soc_clear 0"));
        Serial.println(F("cal_pump  (start pump cal sequence)  cal_abort"));
    }
    else if (!strcmp_P(cmd, PSTR("temps")))    dbgTemps();
    else if (!strcmp_P(cmd, PSTR("valves")))   dbgValves();
    else if (!strcmp_P(cmd, PSTR("faults")))   dbgFaults();
    else if (!strcmp_P(cmd, PSTR("mode")))     dbgMode();
    else if (!strcmp_P(cmd, PSTR("status")))   { dbgMode(); dbgTemps(); dbgFaults(); }
    else if (!strcmp_P(cmd, PSTR("heater")))   dbgHeater();
    else if (!strcmp_P(cmd, PSTR("bus")))      dbgBus();
    else if (!strcmp_P(cmd, PSTR("rtc")))      dbgRTC();
    else if (!strcmp_P(cmd, PSTR("page")))     {
        uint8_t pg = (uint8_t)atoi(arg1);
        if (pg >= 1 && pg <= 5) { currentPage = pg; needFullRedraw = true; wakeDisplay(); Serial.println(F("ok")); }
        else Serial.println(F("usage: page <1-5>"));
    }
    else if (!strcmp_P(cmd, PSTR("scan")))     dbgScan();
    else if (!strcmp_P(cmd, PSTR("set")))      dbgSet(arg1, arg2);
    else if (!strcmp_P(cmd, PSTR("cal_pump"))) dbgCalPump();
    else if (!strcmp_P(cmd, PSTR("cal_abort"))) dbgCalAbort();
    else if (cmd[0] != 0) { Serial.print(F("unknown: ")); Serial.println(cmd); }
}

static void processSerialInput() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (dbgLen > 0) {
                dbgBuf[dbgLen] = 0;
                handleDebugCommand(dbgBuf);
                dbgLen = 0;
            }
        } else if (dbgLen < (uint8_t)(sizeof(dbgBuf) - 1)) {
            dbgBuf[dbgLen++] = c;
        }
    }
}

#endif // DEBUG_SERIAL

// ============================================================
//  POWER-UP SAFE STATE SEQUENCE  (H-side)
// ============================================================

void powerUpSafeState() {
    // Close all 3 tank valves with 7s pulses (sequential per spec)
    logBurnerCold.request(false);
    unsigned long t = millis();
    while (millis() - t < 7200) { logBurnerCold.update(); wdt_reset(); }

    botTankValve.request(false);
    t = millis();
    while (millis() - t < 7200) { botTankValve.update(); wdt_reset(); }

    twoPortValve.request(true);   // default resting position: heater cold side / top-of-tank
    t = millis();
    while (millis() - t < 7200) { twoPortValve.update(); wdt_reset(); }

    // Ensure SSR is off
    PORTA &= ~(1 << PA5);
    digitalWrite(PIN_PSU_12V, LOW);
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
    wdt_disable();
#ifdef DEBUG_SERIAL
    Serial.begin(115200);
#endif

    // Output pins
    pinMode(PIN_LOG_COLD_OPEN,  OUTPUT); digitalWrite(PIN_LOG_COLD_OPEN,  LOW);
    pinMode(PIN_LOG_COLD_CLOSE, OUTPUT); digitalWrite(PIN_LOG_COLD_CLOSE, LOW);
    pinMode(PIN_BOT_TANK_OPEN,  OUTPUT); digitalWrite(PIN_BOT_TANK_OPEN,  LOW);
    pinMode(PIN_BOT_TANK_CLOSE, OUTPUT); digitalWrite(PIN_BOT_TANK_CLOSE, LOW);
    pinMode(PIN_TWO_PORT_OPEN,  OUTPUT); digitalWrite(PIN_TWO_PORT_OPEN,  LOW);
    pinMode(PIN_HEATER_SSR,     OUTPUT); digitalWrite(PIN_HEATER_SSR,     LOW);
    pinMode(PIN_TWO_PORT_CLOSE, OUTPUT); digitalWrite(PIN_TWO_PORT_CLOSE, LOW);
    pinMode(PIN_PSU_12V,        OUTPUT); digitalWrite(PIN_PSU_12V,        LOW);
    pinMode(PIN_RS485_DE_LINK,  OUTPUT); digitalWrite(PIN_RS485_DE_LINK,  LOW);
    pinMode(PIN_DISPLAY_BL,     OUTPUT); setBacklight(displayBrightness);

    // Input pins
    pinMode(PIN_ZERO_CROSSING,  INPUT);
    pinMode(PIN_LOG_BURNER_MOD, INPUT);
    pinMode(PIN_BTN_SELECT,     INPUT_PULLUP);
    pinMode(PIN_BTN_UP,         INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN,       INPUT_PULLUP);
    pinMode(PIN_BUS_VOLTAGE,    INPUT);

    // Buttons
    btnSelect.pin = PIN_BTN_SELECT; btnSelect.state = false; btnSelect.prevState = false;
    btnUp.pin     = PIN_BTN_UP;     btnUp.state     = false; btnUp.prevState     = false;
    btnDown.pin   = PIN_BTN_DOWN;   btnDown.state   = false; btnDown.prevState   = false;

    // RS485 UART
    Serial1.begin(9600);

    // DS18B20
    sensors.begin();
    sensors.setResolution(12);
    for (uint8_t i = 0; i < H_NUM_SENSORS; i++) {
        sTemp[i] = NAN; sFault[i] = false; sFailCount[i] = 0; sGoodCount[i] = 0;
    }
#ifdef DEBUG_SERIAL
    memset(hSimulate, 0, sizeof(hSimulate));
    memset(hSim,      0, sizeof(hSim));
#endif

    // I2C (RTC)
    Wire.begin();
    rtc.begin();

    // Load EEPROM settings
    loadSettings();

    // Valve objects
    logBurnerCold.begin(PIN_LOG_COLD_OPEN, PIN_LOG_COLD_CLOSE, 7000);
    botTankValve.begin(PIN_BOT_TANK_OPEN,  PIN_BOT_TANK_CLOSE,  7000);
    twoPortValve.begin(PIN_TWO_PORT_OPEN,  PIN_TWO_PORT_CLOSE,  7000);

    // TFT display
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(C_BLACK);
    tft.setTextColor(C_WHITE); tft.setTextSize(2);
    tft.setCursor(10, 140); tft.print(F("H Controller Init..."));

    // SD card
    initSD();

    // Packet receiver
    pktRx.reset();
    memset(&lastWPkt, 0, sizeof(lastWPkt));

    // Zero-crossing interrupt (D2 = INT4 on Mega 2560)
    attachInterrupt(digitalPinToInterrupt(PIN_ZERO_CROSSING), zeroCrossISR, RISING);

    // Power-up safe state
    powerUpSafeState();

    // Start first sensor conversion
    startConversion();

    // Enable watchdog
    wdt_enable(WDTO_8S);

#ifdef DEBUG_SERIAL
    Serial.println(F("H controller ready — type 'help' for commands"));
#endif

    lastButtonMs = millis();
    needFullRedraw = true;
}

// ============================================================
//  LOOP
// ============================================================

void loop() {
    wdt_reset();
#ifdef DEBUG_SERIAL
    processSerialInput();
#endif
    unsigned long now = millis();

    // Sensors
    static unsigned long lastConvReqMs = 0;
    if (!convStarted && now - lastConvReqMs >= 1100) { startConversion(); lastConvReqMs = now; }
    readSensors();

    // Log burner module
    readLogBurnerModule();

    // Bus voltage
    static unsigned long lastBusMs = 0;
    if (now - lastBusMs >= 2000) { updateBusVoltage(); lastBusMs = now; }

    // RTC
    readRTC();

    // Grid outage detection (INT4 zero-crossing)
    {
        unsigned long zcAge = micros() - lastZCMicros; // unsigned arithmetic wraps correctly
        bool gridOk = (zcAge < 60000UL); // > 60ms without crossing = outage
        if (gridOk != gridPresent) {
            gridPresent = gridOk;
            if (!gridOk) {
                setFaultH(FAULT_H_GRID_OUTAGE);
                heaterTargetPct = 0;
                PORTA &= ~(1 << PA5);
            } else {
                clearFaultH(FAULT_H_GRID_OUTAGE);
            }
        }
    }

    // Valve state machines
    logBurnerCold.update();
    botTankValve.update();
    twoPortValve.update();

    // Heater fault checks
    checkHeaterFaults();

    // 5am trigger and heating session
    checkMorningTrigger();
    updateHeatSourceSelection();
    updatePVExportOverride();
    updateSummerStartup();

    // Fault history update
    static uint32_t prevWF = 0;
    if (hasWPkt && (lastWPkt.wFaultFlags != prevWF || hFaultFlags != 0)) {
        faultLogUpdate(lastWPkt.wFaultFlags, hFaultFlags);
        prevWF = lastWPkt.wFaultFlags;
    }

    // EEPROM save (30s after last change)
    saveSettingsIfDue();

    // SD safe remove: check for reinsert
    checkSDReinsert();

    // Buttons
    handleButtons();

    // Inter-controller RS485 exchange (every 250ms, W transmits first then we reply)
    if (now - lastInterCtrlMs >= INTER_CTRL_POLL_MS) {
        lastInterCtrlMs = now;
        doInterControllerExchange();
        wdt_reset();
    }
#ifdef DEBUG_SERIAL
    updateCalPump();
#endif

    // SD logging (every 250ms = on each new W packet)
    if (hasWPkt && now - lastLogMs >= 250) {
        lastLogMs = now;
        logDataRow();
    }

    // Display refresh
    if (!displayOn) {
        // Wake on any fault
        bool anyFault = (hFaultFlags != 0 || (hasWPkt && lastWPkt.wFaultFlags != 0));
        if (anyFault) wakeDisplay();
    } else {
        bool inactivity = (now - lastButtonMs >= BACKLIGHT_SLEEP_MS);
        bool anyFault   = (hFaultFlags != 0 || (hasWPkt && lastWPkt.wFaultFlags != 0));
        if (inactivity && !anyFault) {
            displayOn = false;
            setBacklight(0);
            // Return to page 1 after 1hr inactivity
            currentPage    = 1;
            navMode        = NAV_PAGE;
            needFullRedraw = true;
        }
        // Return to page 1 after 1hr
        if (now - lastButtonMs >= BACKLIGHT_SLEEP_MS && currentPage != 1) {
            currentPage = 1; navMode = NAV_PAGE; needFullRedraw = true;
        }
    }

    if (displayOn) {
        if (needFullRedraw) {
            drawFullPage();
        } else if (now - lastDisplayRefreshMs >= 500) {
            lastDisplayRefreshMs = now;
            drawStatusBar();
            drawFaultBar(hasWPkt ? lastWPkt.wFaultFlags : 0, hFaultFlags);
            // Partial refresh of current page data (quick update of values only)
            switch (currentPage) {
                case 1: drawPage1(hasWPkt ? lastWPkt.wFaultFlags : 0); break;
                case 2: drawPage2(); break;
                case 3: /* fault history: redraw on fault change only */ break;
                case 4: drawPage4(); break;
                case 5: drawPage5(); break;
            }
            if (manualOverrideActive || manualHeaterMode != MHM_OFF) {
                updateDisplayBanners();
            }
        }
    }
}
