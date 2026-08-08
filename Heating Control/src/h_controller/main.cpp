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
#include <SPI.h>
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

// Active-LOW relay board: relay energises on LOW, de-energises on HIGH
#define RELAY_ON  LOW
#define RELAY_OFF HIGH

// Outputs — 8-ch relay board (even pins D22–D36)
#define PIN_LOG_COLD_OPEN   22   // 8-ch ch1: log burner cold valve OPEN
#define PIN_LOG_COLD_CLOSE  24   // 8-ch ch2: log burner cold valve CLOSE
#define PIN_BOT_TANK_OPEN   26   // 8-ch ch3: bottom-of-tank valve OPEN
#define PIN_BOT_TANK_CLOSE  28   // 8-ch ch4: bottom-of-tank valve CLOSE
#define PIN_TWO_PORT_OPEN   30   // 8-ch ch5: 2-port valve OPEN (heater side)
#define PIN_HEATER_SSR      27   // direct output to SSR-40DA (NOT through relay board; PA5 hardcoded in ISR)
#define PIN_TWO_PORT_CLOSE  32   // 8-ch ch6: 2-port valve CLOSE (mid-tank side)
#define PIN_PSU_12V         34   // 8-ch ch7: 12VDC backup PSU relay
// D36: 8-ch ch8 spare
#define PIN_RS485_DE_LINK   31   // MAX485 DE/RE (HIGH = transmit)
#define PIN_SOLAR_PUMP      46   // pump MOSFET gate via diode + spare wire to W

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

#define H_SENSOR_TANK_BOT    0
#define H_SENSOR_TANK_MID    1
#define H_SENSOR_TANK_TOP    2
#define H_SENSOR_HOT_PIPE    3
#define H_SENSOR_COLD_PIPE   4
#define H_SENSOR_HEATER_OUT  5
#define H_SENSOR_HEATER_OUT_2 6
#define H_NUM_SENSORS        7

static const uint8_t DS18B20_ADDRS[H_NUM_SENSORS][8] = {
    { 0x28, 0xA4, 0xD1, 0x14, 0x00, 0x00, 0x00, 0x3B }, // tank bottom    (sensor 7)
    { 0x28, 0xB7, 0x37, 0x15, 0x00, 0x00, 0x00, 0xF0 }, // tank middle    (sensor 8)
    { 0x28, 0xB2, 0x99, 0x12, 0x00, 0x00, 0x00, 0x0C }, // tank top       (sensor 9)
    { 0x28, 0x11, 0x6C, 0x12, 0x00, 0x00, 0x00, 0x7D }, // hot pipe       (sensor 10)
    { 0x28, 0x5C, 0x8E, 0x12, 0x00, 0x00, 0x00, 0x1B }, // cold pipe
    { 0x28, 0xEA, 0x8F, 0x12, 0x00, 0x00, 0x00, 0x63 }, // heater output  (sensor 12)
    { 0x28, 0x55, 0xBE, 0x27, 0x79, 0x25, 0x0B, 0xBF }, // heater output 2 (sensor 13)
};

// ============================================================
//  COMMISSIONING FLAGS
// ============================================================

// Set true after verifying Growatt Modbus sign convention via W controller
static const bool HEATER_ENABLED = true;

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

#define DS18B20_CONV_MS         375UL
#define INTER_CTRL_POLL_MS      250UL
#define RS485_RX_TIMEOUT_MS     150UL
#define COMMS_FAULT_THRESHOLD   480 // 480 * 250ms poll = 120s
#define EEPROM_WRITE_DELAY_MS   30000UL  // 30s after last change
#define BACKLIGHT_SLEEP_MS      3600000UL // 1 hour
#define DISPLAY_INACTIVITY_MS   30000UL  // 30s → exit item/option mode
#define TIME_SYNC_INTERVAL_MS   3600000UL
#define BUS_LOW_THRESH_DV       140      // 14.0V (×0.1V units)
#define BUS_PSU_THRESH_DV       125      // 12.5V
#define BUS_PSU_HYSTERESIS_DV   130      // 13.0V
#define BUS_PSU_ON_DELAY_MS     5000UL   // 5s before PSU turns on
#define BUS_RESTORE_THRESH_DV   140      // 14.0V
#define BUS_LOW_DELAY_MS        10000UL  // 10s before fault
#define HEATER_RATED_W          3000     // immersion element rated power
#define ENERGY_LOG_INTERVAL_MS  300000UL // 5 min SD flush of daily kWh totals

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
        digitalWrite(a, RELAY_OFF); pinMode(a, OUTPUT);
        digitalWrite(b, RELAY_OFF); pinMode(b, OUTPUT);
    }

    void request(bool open) {
        if (phase == HBP_IDLE   && isOpen      == open) return; // already there
        if (phase != HBP_IDLE   && pendingOpen == open) return; // already moving there
        if (phase == HBP_PULSING) { digitalWrite(pinA, RELAY_OFF); digitalWrite(pinB, RELAY_OFF); }
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
                    digitalWrite(pendingOpen ? pinA : pinB, RELAY_ON);
                    phaseStartMs = now;
                    phase = HBP_PULSING;
                }
                break;
            case HBP_PULSING:
                if (now - phaseStartMs >= pulseDurationMs) {
                    digitalWrite(pinA, RELAY_OFF); digitalWrite(pinB, RELAY_OFF);
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

// 20-level fixed-burst heater table. Each level fires on_hc half-cycles (10ms each) then
// off_hc half-cycles off, repeating. pct10 = actual duty × 10 (e.g. 143 = 14.3%).
// Levels named by exact duty, not nominal request.
struct HeaterLevel { uint8_t on_hc; uint8_t off_hc; uint16_t pct10; };
static const HeaterLevel kHeaterLevels[20] PROGMEM = {
    {  1, 19,   50 },  //  5.0%
    {  1,  9,  100 },  // 10.0%
    {  1,  7,  143 },  // 14.3%
    {  1,  4,  200 },  // 20.0%
    {  1,  3,  250 },  // 25.0%
    {  2,  5,  286 },  // 28.6%
    {  1,  2,  333 },  // 33.3%
    {  2,  3,  400 },  // 40.0%
    {  3,  4,  429 },  // 42.9%
    {  1,  1,  500 },  // 50.0%
    {  4,  3,  571 },  // 57.1%
    {  3,  2,  600 },  // 60.0%
    {  2,  1,  667 },  // 66.7%
    {  5,  2,  714 },  // 71.4%
    {  3,  1,  750 },  // 75.0%
    {  4,  1,  800 },  // 80.0%
    {  6,  1,  857 },  // 85.7%
    {  9,  1,  900 },  // 90.0%
    { 19,  1,  950 },  // 95.0%
    {  1,  0, 1000 },  // 100.0%
};

volatile unsigned long lastZCMicros  = 0;
volatile uint8_t  heaterLevelIdx  = 0;    // 0=off, 1–20=active level index
volatile uint8_t  heaterLevelCap  = 20;   // ISR-level cap (0–20; 20=no cap)
volatile uint8_t  heaterPhaseHc   = 0;    // half-cycle count within current phase
volatile bool     heaterPhaseOn   = true; // true=ON phase, false=OFF phase
volatile uint32_t zcFireCount     = 0;
#ifdef DEBUG_SERIAL
volatile bool simHeaterActive    = false;
bool          pumpTestHeaterActive = false;  // true while pump_test/stress_test owns the heater
#else
static const bool simHeaterActive = false;
#endif

bool heaterRunning    = false;
bool heaterHardLockout = false;   // set on element fail / UFH hard lockout — clears on restart
bool hotTankProtection = false;
bool morningHeatActive = false;
bool          importTripActive  = false;
unsigned long importHighStartMs = 0;
unsigned long importZeroStartMs = 0;
bool gridPresent                 = true;
unsigned long lastGridLossMs     = 0;
bool gridOutageFault             = false;
bool          pvExportOverride   = false; // true = PV export ≥500W for ≥60s in WINTER mode → allow heater + send MODE_SUMMER to W
unsigned long pvExport500StartMs = 0;

float         hPumpDuty        = 0.0f;
bool          hPumpOutputState = false;
unsigned long hPumpOnMs        = 0;
unsigned long hPumpOffMs       = 0;

enum FlushState : uint8_t { FLUSH_IDLE, FLUSH_ACTIVE, FLUSH_PAUSE };
FlushState flushState = FLUSH_IDLE;
uint32_t   flushTimer = 0;

// True when W solar-cold is open, 2-port is fully on the heater side and idle,
// AND at least one H-side return path is fully open and idle.
static bool heaterFlowPathOk() {
    if (!(lastWPkt.valveStates & VSTATE_SOLAR_COLD_OPEN)) return false;
    if (twoPortValve.busy() || !twoPortValve.isOpen) return false;
    return (logBurnerCold.isOpen && logBurnerCold.phase == HBP_IDLE)
        || (botTankValve.isOpen  && botTankValve.phase  == HBP_IDLE);
}

// Zero-crossing ISR — runs every 10ms on 50Hz grid (every half-cycle).
// Fixed-burst firing: fires on_hc consecutive half-cycles then off_hc off, repeating.
// Direct port write (PA5 = D27) to avoid digitalWrite() overhead in ISR.
void zeroCrossISR() {
    lastZCMicros = micros();
    zcFireCount++;

    bool on = false;
    uint8_t lvl = min(heaterLevelIdx, heaterLevelCap);

    if (lvl > 0 && (HEATER_ENABLED || simHeaterActive) && heaterRunning
        && (lastWPkt.valveStates & VSTATE_SOLAR_COLD_OPEN)
        && ((logBurnerCold.isOpen && logBurnerCold.phase == HBP_IDLE) ||
            (botTankValve.isOpen  && botTankValve.phase  == HBP_IDLE))
        && !heaterHardLockout) {
        uint8_t on_hc  = pgm_read_byte(&kHeaterLevels[lvl - 1].on_hc);
        uint8_t off_hc = pgm_read_byte(&kHeaterLevels[lvl - 1].off_hc);
        if (heaterPhaseOn) {
            on = true;
            if (++heaterPhaseHc >= on_hc) {
                heaterPhaseHc = 0;
                if (off_hc > 0) heaterPhaseOn = false;
            }
        } else {
            if (++heaterPhaseHc >= off_hc) {
                heaterPhaseHc = 0;
                heaterPhaseOn = true;
            }
        }
    }

    if (on) PORTA |= (1 << PA5);    // D27 = PA5
    else    PORTA &= ~(1 << PA5);
}

// Returns highest level index whose duty does not exceed pct (0–100). Returns 0 if pct==0.
static uint8_t pctToLevel(uint8_t pct) {
    uint8_t best = 0;
    for (uint8_t i = 0; i < 20; i++) {
        if (pgm_read_word(&kHeaterLevels[i].pct10) <= (uint16_t)pct * 10) best = i + 1;
        else break;
    }
    return best;
}

// Duty of current active level as integer percent (rounded), or 0 if off.
static uint8_t heaterLevelPct() {
    if (heaterLevelIdx == 0) return 0;
    return (uint8_t)((pgm_read_word(&kHeaterLevels[heaterLevelIdx - 1].pct10) + 5) / 10);
}

// Duty × 10 of current level (e.g. 143 = 14.3%), or 0 if off.
static uint16_t heaterLevelPct10() {
    if (heaterLevelIdx == 0) return 0;
    return pgm_read_word(&kHeaterLevels[heaterLevelIdx - 1].pct10);
}

uint8_t rtcHour();   // defined after RTC section

#ifdef DEBUG_SERIAL
uint8_t simHeaterVal = 0;
#endif

static inline float getHeaterOutC();  // defined after sensorFaultMaskH

// Called every RS485 packet (~250ms); Growatt data arrives via W.
void updateHeaterDuty(int16_t pvExportW, int16_t gridImportW,
                       ManualHeaterMode manualMode, bool wSolarFault)
{
#ifdef DEBUG_SERIAL
    if (simHeaterActive) {
        heaterRunning  = simHeaterVal > 0;
        heaterLevelIdx = pctToLevel(simHeaterVal);
        return;
    }
    if (pumpTestHeaterActive) return;  // pump_test/stress_test own heaterRunning
#endif
    if (gridOutageFault || !HEATER_ENABLED) {
        heaterLevelIdx = 0; heaterRunning = false; return;
    }
    // W solar sensor fault: W is in emergency UFH dump mode; suppress heater
    // to avoid adding heat to a circuit being used to dissipate excess solar heat.
    if (wSolarFault) {
        heaterLevelIdx = 0; heaterRunning = false; return;
    }
    if (heaterHardLockout) {
        heaterLevelIdx = 0; heaterRunning = false; return;
    }
    if (importTripActive) {
        heaterLevelIdx = 0; heaterRunning = false; return;
    }
    if (hotTankProtection) {
        heaterLevelIdx = 0; heaterRunning = false; return;
    }
    if (morningHeatActive) {
        heaterLevelIdx = 0; heaterRunning = false; return;
    }
    if (manualMode == MHM_FORCE_ON) {
        // Call for the flow-path valves regardless of position; actual firing stays
        // blocked until heaterFlowPathOk() confirms they've reached it.
        logBurnerCold.request(false);
        botTankValve.request(true);
        twoPortValve.request(true);
        heaterRunning = true;

        // Same hot pipe cap as the PV/SOC-surplus path: 100% at 63°C, ramp to 0 at 90°C.
        uint8_t pct = 100;
        if (!sFault[H_SENSOR_HOT_PIPE]) {
            float hp = sTemp[H_SENSOR_HOT_PIPE];
            if (hp >= 90.0f) {
                pct = 0;
            } else if (hp > 63.0f) {
                pct = (uint8_t)((90.0f - hp) * (100.0f / 27.0f));
            }
        }
        heaterLevelIdx = heaterFlowPathOk() ? pctToLevel(pct) : 0;
        return;
    }

    bool     growattOk     = lastWPkt.growattValid;
    int32_t  heaterCurrentW = (int32_t)heaterLevelPct10() * 3;
    int16_t  rawPct;

    static bool          lastHighSoc    = false;
    static unsigned long surplusStartMs = 0;
    static uint8_t       zeroCount      = 0;
    static uint8_t        rawPctBuf[8]  = {0}; // rolling window of last (up to) 8 samples
    static uint8_t        rawPctBufIdx  = 0;
    static uint8_t        rawPctBufCount = 0;
    static uint16_t       rawPctSum      = 0;

    // SOC-limited mode: 5% hysteresis, enter at 55%, leave at 50%
    uint8_t soc     = lastWPkt.battSocPct;
    bool    highSoc = (manualMode == MHM_SOC_LIM) && growattOk
                      && (lastHighSoc ? soc >= 50 : soc >= 55);

    if (highSoc) {
        // 4 kW budget from bat+grid combined; heater gets the headroom.
        // Growatt readings include heater load, so add back heaterCurrentW to find
        // non-heater draw, then subtract from budget.
        int16_t battChgW       = lastWPkt.battChargeW;
        int32_t battDischargeW = battChgW < 0 ? (int32_t)(-battChgW) : 0L;
        int32_t available      = 4000L - ((int32_t)gridImportW + battDischargeW - heaterCurrentW);
        rawPct = (int16_t)constrain(available * 100L / 3000L, 0L, 100L);
    } else if (manualMode == MHM_SOC_LIM) {
        // SOC mode but below threshold — heater off
        heaterRunning = false; heaterLevelIdx = 0; return;
    } else {
        // Normal auto mode: requires PV generating.
        if (!growattOk || (lastWPkt.pv1W + lastWPkt.pv2W) < 200) {
            heaterRunning = false; heaterLevelIdx = 0; return;
        }
        int16_t battChgW   = lastWPkt.battChargeW;
        int32_t netGridW   = (int32_t)pvExportW - gridImportW;
        int32_t battW      = (int32_t)min((int16_t)0, battChgW);
        // SOC > 85%: battery essentially full, only 200W reservation regardless of time.
        // Otherwise before 14:00: 500W reservation always.
        // After 14:00: 3kW reservation while SOC ≤ 60%, then 1kW once battery is charging freely.
        int32_t reservationW = (soc > 85) ? 200L
                             : (rtcHour() < 14) ? 500L : (soc > 60 ? 1000L : 3000L);
        int32_t available = (reservationW > 0)
            ? (int32_t)pvExportW - gridImportW + (int32_t)battChgW + heaterCurrentW - reservationW
            : netGridW + heaterCurrentW + battW - 100L;
        rawPct = (int16_t)constrain(available * 100L / 3000L, 0L, 100L);
    }

    if (highSoc != lastHighSoc) {
        heaterRunning = false; heaterLevelIdx = 0;
        surplusStartMs = 0; zeroCount = 0;
        rawPctBufIdx = 0; rawPctBufCount = 0; rawPctSum = 0;
        lastHighSoc = highSoc;
    }

    if (!heaterRunning) {
        zeroCount = 0;
        rawPctBufIdx = 0; rawPctBufCount = 0; rawPctSum = 0;
        if (rawPct == 0) return;
        if (!highSoc) {
            // 500W threshold when solar is off; 150W (5%) when solar is already running
            uint8_t startThreshPct = lastWPkt.solarPumpActive ? 5 : 17;
            if (rawPct < startThreshPct) { surplusStartMs = 0; return; }
            if (surplusStartMs == 0) surplusStartMs = millis();
            if (millis() - surplusStartMs < 5000UL) return;
        }
        heaterRunning  = true;
        surplusStartMs = 0;
    }

    // Heater wants to run: call for the flow-path valves to move into position.
    // Actual firing stays blocked (heaterLevelIdx=0 below) until they're confirmed there.
    logBurnerCold.request(false);
    botTankValve.request(true);
    twoPortValve.request(true);

    if (rawPct == 0) {
        if (++zeroCount >= 5) {
            heaterRunning = false; heaterLevelIdx = 0;
            zeroCount = 0;
            rawPctBufIdx = 0; rawPctBufCount = 0; rawPctSum = 0;
        }
        return;
    }
    zeroCount = 0;

    // Rolling average: each new packet replaces the oldest of up to 8 samples,
    // so every packet's output reflects the current sample averaged against
    // the previous 7 rather than waiting for a batch to fill.
    if (rawPctBufCount < 8) {
        rawPctBuf[rawPctBufIdx] = (uint8_t)rawPct;
        rawPctSum += (uint8_t)rawPct;
        rawPctBufCount++;
    } else {
        rawPctSum -= rawPctBuf[rawPctBufIdx];
        rawPctBuf[rawPctBufIdx] = (uint8_t)rawPct;
        rawPctSum += (uint8_t)rawPct;
    }
    rawPctBufIdx = (rawPctBufIdx + 1) % 8;
    uint8_t avgPct = (uint8_t)(rawPctSum / rawPctBufCount);

    // Hot pipe cap: 100% at 63°C, linear ramp to 0 at 90°C
    if (!sFault[H_SENSOR_HOT_PIPE]) {
        float hp = sTemp[H_SENSOR_HOT_PIPE];
        if (hp >= 90.0f) {
            avgPct = 0;
        } else if (hp > 63.0f) {
            uint8_t cap = (uint8_t)((90.0f - hp) * (100.0f / 27.0f));  // ~3.70% per °C over 27°C span
            if (avgPct > cap) avgPct = cap;
        }
    }

    // Actual power output blocked until the flow-path valves are confirmed in position.
    heaterLevelIdx = heaterFlowPathOk() ? pctToLevel(avgPct) : 0;
}

// ============================================================
//  FAULT FLAGS
// ============================================================

uint32_t hFaultFlags = 0;
void setFaultH(uint32_t m)   { hFaultFlags |= m; }
void clearFaultH(uint32_t m) { hFaultFlags &= ~m; }
bool hasFaultH(uint32_t m)   { return (hFaultFlags & m) != 0; }

// ============================================================
//  HEATER IMPORT TRIP  (inverter CB tripped while panels producing)
// ============================================================

void checkHeaterImportTrip(int16_t gridImportW, uint8_t soc, int16_t battChargeW, bool growattValid) {
    if (!growattValid) return;
    unsigned long now = millis();
    int32_t battDischargeW = battChargeW < 0 ? (int32_t)(-battChargeW) : 0L;
    bool    importHigh = soc > 15 && battDischargeW < 3900L && gridImportW > 100;

    if (!importTripActive) {
        if (importHigh) {
            if (!importHighStartMs) importHighStartMs = now;
            if (now - importHighStartMs >= 20000UL) {
                importTripActive  = true;
                importHighStartMs = 0;
                heaterRunning     = false;
                heaterLevelIdx    = 0;
                setFaultH(FAULT_H_HEATER_IMPORT_TRIP);
            }
        } else {
            importHighStartMs = 0;
        }
        importZeroStartMs = 0;
    } else {
        if (gridImportW <= 0) {
            if (!importZeroStartMs) importZeroStartMs = now;
            if (now - importZeroStartMs >= 20000UL) {
                importTripActive  = false;
                importZeroStartMs = 0;
                clearFaultH(FAULT_H_HEATER_IMPORT_TRIP);
            }
        } else {
            importZeroStartMs = 0;
        }
    }
}

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
bool    simHPumpSpdActive  = false; uint8_t simHPumpSpdVal = 0;
bool    simWPumpStop       = false;

enum CalPumpPhase : uint8_t { CALP_IDLE, CALP_RUNNING, CALP_DONE };
CalPumpPhase  calPumpPhase   = CALP_IDLE;
uint8_t       calHeaterPct   = 15;    // current step heater power %
float         calPumpBase    = 4.0f;  // pump floor for current step % (float for sub-1% precision)
uint8_t       calStepNum     = 0;     // 0 = first step (5min), 1+ = subsequent (3min)
bool          calHtrOverride = false;
bool          calCooling     = false; // true while waiting for htr_out to drop below 91°C
bool          calSensorFault = false; // true while htr_out sensor is faulted during a step
unsigned long calPhaseMs     = 0;     // step start time
volatile uint32_t calPumpOnMs  = 200;  // precomputed on-time for ISR
volatile uint32_t calPumpOffMs = 6200; // precomputed off-time for ISR

// ── Pump formula test (pump_test command) ────────────────────
// 28 values (5–100), interleaved low/mid/high so consecutive steps jump ~33%.
static const uint8_t PUMP_TEST_POWERS[] PROGMEM = {
    35, 69, 39, 73,  8, 42, 76, 11, 45, 80,
    15, 49, 83, 18, 52, 86, 21, 56, 90, 25,
    59, 93, 28, 62, 97, 32, 66, 100
};
static const uint8_t PUMP_TEST_COUNT = sizeof(PUMP_TEST_POWERS);
// Sequential low→mid range; covers all previously oscillating levels + boundary
static const uint8_t STRESS_TEST_POWERS[] PROGMEM = {
     5, 11, 15, 18, 20, 21, 25, 30, 35
};
static const uint8_t STRESS_TEST_COUNT = sizeof(STRESS_TEST_POWERS);
enum PumpTestState : uint8_t { PT_IDLE, PT_HEATING, PT_COOLDOWN };
PumpTestState pumpTestState    = PT_IDLE;
uint8_t       pumpTestPower    = 5;
uint8_t       pumpTestStepIdx  = 0;
uint32_t      pumpTestLastPrint = 0;
uint32_t      pumpTestWinStart  = 0;
float         pumpTestWinMin    = 0.0f;
float         pumpTestWinMax    = 0.0f;
uint32_t      pumpTestLastSec   = 0;
float         pumpTestMaxPump   = 0.0f;
bool          pumpTestStress    = false;
uint32_t      pumpTestCoolStart = 0;
#endif // DEBUG_SERIAL

// ============================================================
//  BUS VOLTAGE  (A0: 10kΩ + 4.7kΩ divider, 15V → 4.80V)
// ============================================================

float busVoltageV = 15.0f;
bool  psu12vActive = false;
unsigned long psuOnStartMs = 0;
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
    if (!psu12vActive) {
        if (vdv < BUS_PSU_THRESH_DV) {
            if (psuOnStartMs == 0) psuOnStartMs = millis();
            if (millis() - psuOnStartMs >= BUS_PSU_ON_DELAY_MS) {
                psu12vActive = true;
                digitalWrite(PIN_PSU_12V, RELAY_ON);
            }
        } else {
            psuOnStartMs = 0;
        }
    } else if (vdv >= BUS_PSU_HYSTERESIS_DV) {
        psu12vActive = false;
        digitalWrite(PIN_PSU_12V, RELAY_OFF);
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
    FAULT_H_SENSOR_HOT_PIPE, FAULT_H_SENSOR_COLD_PIPE,
    0, 0   // HEATER_OUT / HEATER_OUT_2 — managed by checkHeaterFaults() with 90s/120s grace
};

// How long a debounced read failure must persist before sFault[] actually flips
// true (and the fault flag/page-3 log entry/W midpoint-LED flash follow). Tank/cold-pipe
// sensors run on stale data for 8h before being declared faulted; hot-pipe gets 15s since
// checkHeaterFaults() kills the heater the moment sFault[HOT_PIPE] goes true. Heater-outlet
// sensors are excluded (0 = immediate) — checkHeaterFaults() owns their 90s/120s timing itself.
static const uint32_t sensorFaultGraceMs[H_NUM_SENSORS] = {
    28800000UL, 28800000UL, 28800000UL,
    15000UL,
    28800000UL,
    0, 0
};
unsigned long sRawFaultStartMs[H_NUM_SENSORS];
// Fast (3-sample) raw failure state, independent of sensorFaultGraceMs — drives the
// per-sensor red "FAULT" readout on page 1 immediately, ahead of the debounced sFault[]
// that gates the fault flag/page-3 log/banner/LED.
bool sRawFault[H_NUM_SENSORS];

inline void pollBtns();  // forward declaration — defined after Button struct

void startConversion() {
    sensors.setResolution(11);
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
        if (!tempValid(t)) {
            // Retry once — transient EMI CRC errors are single-cycle; scratchpad still holds valid data
            delayMicroseconds(500);
            t = sensors.getTempC((uint8_t*)DS18B20_ADDRS[i]);
        }
        pollBtns();
        if (tempValid(t)) {
            sTemp[i] = t; sFailCount[i] = 0; sRawFaultStartMs[i] = 0;
            if (sGoodCount[i] < 3) sGoodCount[i]++;
            if (sGoodCount[i] >= 3) {
                sRawFault[i] = false;
                if (sFault[i]) { sFault[i] = false; clearFaultH(sensorFaultMaskH[i]); }
            }
        } else {
            sGoodCount[i] = 0;
            if (sFailCount[i] < 3) sFailCount[i]++;
            if (sFailCount[i] >= 3) {
                sRawFault[i] = true;
                if (!sFault[i]) {
                    if (!sRawFaultStartMs[i]) sRawFaultStartMs[i] = millis();
                    if (millis() - sRawFaultStartMs[i] >= sensorFaultGraceMs[i]) {
                        sFault[i] = true;
                        // Tank/hot-pipe/cold-pipe keep the last known reading (stale data);
                        // only the heater-outlet sensors blank to NaN, per getHeaterOutC() arbitration.
                        if (i == H_SENSOR_HEATER_OUT || i == H_SENSOR_HEATER_OUT_2) sTemp[i] = NAN;
                        setFaultH(sensorFaultMaskH[i]);
                    }
                }
            }
        }
    }
#ifdef DEBUG_SERIAL
    for (uint8_t i = 0; i < H_NUM_SENSORS; i++) {
        if (hSimulate[i]) { sTemp[i] = hSim[i]; sFault[i] = false; sRawFault[i] = false; sRawFaultStartMs[i] = 0; }
    }
#endif
}

// ============================================================
//  HEATER OUTPUT TEMPERATURE  (dual-sensor arbitration)
//  Both good → max of the two.  One faulted → remaining.  Both → NAN.
// ============================================================

static inline float getHeaterOutC() {
    bool f1 = sFault[H_SENSOR_HEATER_OUT];
    bool f2 = sFault[H_SENSOR_HEATER_OUT_2];
    if (f1 && f2)  return NAN;
    if (f1)        return sTemp[H_SENSOR_HEATER_OUT_2];
    if (f2)        return sTemp[H_SENSOR_HEATER_OUT];
    return max(sTemp[H_SENSOR_HEATER_OUT], sTemp[H_SENSOR_HEATER_OUT_2]);
}

// ============================================================
//  HEATER FAULT CHECKS  (called from main loop)
// ============================================================

void checkHeaterFaults() {
    static unsigned long htrOut1FaultMs = 0;
    static unsigned long htrOut2FaultMs = 0;

    bool f1 = sFault[H_SENSOR_HEATER_OUT];
    bool f2 = sFault[H_SENSOR_HEATER_OUT_2];

    // Track fault onset; clear H fault flag immediately on recovery
    if (f1) { if (!htrOut1FaultMs) htrOut1FaultMs = millis(); }
    else    { htrOut1FaultMs = 0; clearFaultH(FAULT_H_SENSOR_HEATER_OUT); }
    if (f2) { if (!htrOut2FaultMs) htrOut2FaultMs = millis(); }
    else    { htrOut2FaultMs = 0; clearFaultH(FAULT_H_SENSOR_HEATER_OUT_2); }

    // Hard lockout clears when effective temp < 88°C and at least one sensor is live
    if (heaterHardLockout && hasFaultH(FAULT_H_HEATER_OVERHEAT_SHUT) && !(f1 && f2)) {
        float effT = getHeaterOutC();
        if (!isnan(effT) && effT < 88.0f) {
            heaterHardLockout = false;
            clearFaultH(FAULT_H_HEATER_OVERHEAT_SHUT);
        }
    }

    if (hasWPkt && millis() - lastWPktMs > 60000UL) {
        heaterLevelIdx = 0; heaterRunning = false;
    }

    if (!heaterRunning || heaterLevelIdx == 0) {
        heaterLevelCap     = 20;
        return;
    }

    // Both sensors faulted: shut down heater immediately
    if (f1 && f2) {
        heaterLevelCap = 0;
        setFaultH(FAULT_H_SENSOR_HEATER_OUT);
        setFaultH(FAULT_H_SENSOR_HEATER_OUT_2);
        return;
    }

    // Single fault: continue on remaining sensor for 90s grace, then cut heater; fault raised at 120s
    if (f1 && millis() - htrOut1FaultMs >= 90000UL) {
        heaterLevelCap = 0;
        if (millis() - htrOut1FaultMs >= 120000UL) setFaultH(FAULT_H_SENSOR_HEATER_OUT);
        return;
    }
    if (f2 && millis() - htrOut2FaultMs >= 90000UL) {
        heaterLevelCap = 0;
        if (millis() - htrOut2FaultMs >= 120000UL) setFaultH(FAULT_H_SENSOR_HEATER_OUT_2);
        return;
    }

    if (sFault[H_SENSOR_HOT_PIPE]) {
        heaterLevelCap = 0;
        setFaultH(FAULT_H_SENSOR_HOT_PIPE);
        return;
    }

    float hOut = getHeaterOutC();
    static bool ovhtValvesRequested = false;

    if (hOut >= 94.0f) {
        if (!ovhtValvesRequested) {
            twoPortValve.request(true);
            botTankValve.request(true);
            ovhtValvesRequested = true;
        }
        heaterLevelCap = 0;
        heaterLevelIdx = 0;
        heaterHardLockout = true;
        PORTA &= ~(1 << PA5);
        setFaultH(FAULT_H_HEATER_OVERHEAT_SHUT);
    } else if (hOut > 91.0f) {
        if (!ovhtValvesRequested) {
            twoPortValve.request(true);
            botTankValve.request(true);
            ovhtValvesRequested = true;
        }
        float t   = fminf(1.0f, (hOut - 91.0f) / 2.0f);  // 0 at 91°C, 1 at 93°C
        float cap = (float)heaterLevelPct() * (1.0f - t);
        heaterLevelCap = (cap < 1.0f) ? 0 : pctToLevel((uint8_t)roundf(cap));
    } else {
        ovhtValvesRequested = false;
        heaterLevelCap     = 20;
        clearFaultH(FAULT_H_SENSOR_HOT_PIPE);
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
SolarTargetMode solarTargetMode     = SOLAR_TANK_PLUS8;
uint8_t         displayBrightness   = 100;

// Not stored in EEPROM — reset on power cycle
BoostMode        boostMode          = BOOST_OFF;
float            boostTarget        = 15.5f;
ManualHeaterMode manualHeaterMode   = MHM_OFF;
uint32_t         boost8hrEndMs      = 0;  // millis() when 8hr boost ends
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

void checkHotTankProtection() {
    if (sFault[H_SENSOR_TANK_BOT]) return;
    float tBot = sTemp[H_SENSOR_TANK_BOT];
    if (!hotTankProtection && tBot > 83.0f)      hotTankProtection = true;
    else if (hotTankProtection && tBot < 82.0f)  hotTankProtection = false;

    // Re-assert every loop (request() no-ops once already there) so a manual
    // override during protection can't leave the valves stuck out of position.
    if (hotTankProtection) {
        logBurnerCold.request(true);
        botTankValve.request(false);
        twoPortValve.request(true);
    }
}

void updateHeatSourceSelection() {
    if (hotTankProtection) return;

    static bool prevMorningActive = false;

    if (prevMorningActive && !morningHeatActive) {
        // Session ended: return all tank valves to known safe state
        logBurnerCold.request(false);
        botTankValve.request(false);
        twoPortValve.request(true);   // heater cold side / top-of-tank
    }
    prevMorningActive = morningHeatActive;

    if (!morningHeatActive) {
        // Solar emergency dump with UFH pump on: isolate tank from log burner and bottom port
        if (lastWPkt.solarDumpActive) {
            logBurnerCold.request(false);
            botTankValve.request(false);
        }
        return;
    }

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
    // Gated on twoPortValve.isOpen (actual position) rather than a shadow flag,
    // so a manual-override-induced mismatch self-corrects here instead of
    // waiting on a flag transition that may never come.
    if (!sFault[H_SENSOR_TANK_MID]) {
        float tMid = sTemp[H_SENSOR_TANK_MID];
        if (twoPortValve.isOpen && tMid > 32.0f) {
            twoPortValve.request(false); // mid-tank side
        } else if (!twoPortValve.isOpen && tMid < 28.0f) {
            twoPortValve.request(true);  // heater cold side
        }
        // between 28–32°C: no request, hold current position
    }
}

// ============================================================
//  SOLAR VALVE CONTROL  (H side)
//
//  2-port valve: request(true)  = heater cold side / top-of-tank
//                request(false) = mid-tank side
//
//  Cold-start: when pump > 0%, heater off, hot-pipe < 40°C and
//  > 3°C below tank-bottom — open log-burner-cold, close bot-tank,
//  force 2-port to heater side. Exit when hot-pipe reaches tank-
//  bottom (3°C hysteresis) or heater fires: close log-burner-cold,
//  open bot-tank. On sensor fault, hold last valve state.
//
//  2-port tracked independently: heater-side when heater firing or
//  within 15s of stopping (heater latch); mid-tank when hot-pipe <
//  tank-top and pump ≥ 3%. 2-port tracking suspended during cold-start.
//
//  summerStartupPhase kept as file-scope for the H→W RS485 packet
//  (W uses summerStartupPhase >= 3 as gate). twoPortValve.isOpen is
//  sent directly as the ground-truth 2-port position — see updateSolarValves().
// ============================================================

uint8_t summerStartupPhase = 0;  // 0=idle 3=running (sent to W in packet)

void updateSolarValves() {
    static bool          solarActive     = false;
    static bool          coldStartActive = false;
    static bool          heaterLatch     = false;
    static unsigned long heaterStopMs    = 0;

    bool solarRunning = hasWPkt && lastWPkt.solarPumpActive;

    if (!solarRunning || morningHeatActive) {
        if (solarActive) {
            solarActive        = false;
            coldStartActive    = false;
            heaterLatch        = false;
            heaterStopMs       = 0;
            summerStartupPhase = 0;
            logBurnerCold.request(false);
            if (!morningHeatActive) botTankValve.request(false);
            twoPortValve.request(true);
        }
        return;
    }

    bool heaterActive  = heaterRunning && heaterLevelIdx > 0;
    bool pumpAboveZero = lastWPkt.solarPumpDutyPct > 0;
    bool hpOk          = !sFault[H_SENSOR_HOT_PIPE];
    bool botOk         = !sFault[H_SENSOR_TANK_BOT];

    if (!solarActive) {
        solarActive        = true;
        summerStartupPhase = 3;
        if (!heaterActive && pumpAboveZero && hpOk && botOk
                && sTemp[H_SENSOR_HOT_PIPE] < 40.0f
                && sTemp[H_SENSOR_HOT_PIPE] < sTemp[H_SENSOR_TANK_BOT] - 3.0f) {
            coldStartActive = true;
            logBurnerCold.request(true);
            botTankValve.request(false);
        } else {
            coldStartActive = false;
            logBurnerCold.request(false);
            botTankValve.request(true);
        }
        twoPortValve.request(true);
        return;
    }

    // Cold-start transitions (only when solar already running)
    if (!coldStartActive) {
        if (!heaterActive && pumpAboveZero && hpOk && botOk
                && sTemp[H_SENSOR_HOT_PIPE] < 40.0f
                && sTemp[H_SENSOR_HOT_PIPE] < sTemp[H_SENSOR_TANK_BOT] - 3.0f) {
            coldStartActive  = true;
            logBurnerCold.request(true);
            botTankValve.request(false);
            twoPortValve.request(true);
        }
    } else {
        if (heaterActive) {
            // Heater fired: switch to normal path immediately, 2-port heater side
            coldStartActive  = false;
            logBurnerCold.request(false);
            botTankValve.request(true);
            twoPortValve.request(true);
        } else if (hpOk && botOk && sTemp[H_SENSOR_HOT_PIPE] >= sTemp[H_SENSOR_TANK_BOT]) {
            // Hot pipe warmed up to tank bottom: switch to normal path
            coldStartActive = false;
            logBurnerCold.request(false);
            botTankValve.request(true);
            // twoPortValve falls through to latch/tracking below
        }
        // Sensor fault: hold current valve state
    }

    // 2-port: heater latch + hot-pipe/tank-top tracking (suspended during cold-start).
    // Gated on twoPortValve.isOpen (actual position) rather than a shadow flag, so a
    // manual-override-induced mismatch self-corrects on the next applicable check
    // instead of staying stuck until an unrelated edge happens to fire.
    bool pumpRunning = hasWPkt && lastWPkt.solarPumpDutyPct >= 3;

    if (heaterActive) {
        if (!twoPortValve.isOpen) twoPortValve.request(true);
        heaterLatch  = true;
        heaterStopMs = 0;
    } else if (coldStartActive) {
        // Hold 2-port on heater side; reset latch so it doesn't fire on cold-start exit
        if (!twoPortValve.isOpen) twoPortValve.request(true);
        heaterLatch  = false;
        heaterStopMs = 0;
    } else if (heaterLatch) {
        if (heaterStopMs == 0) heaterStopMs = millis();
        bool canRelease = (millis() - heaterStopMs >= 15000UL)
                          && pumpRunning
                          && !sFault[H_SENSOR_HOT_PIPE] && !sFault[H_SENSOR_TANK_TOP]
                          && sTemp[H_SENSOR_HOT_PIPE] < sTemp[H_SENSOR_TANK_TOP];
        if (canRelease) {
            heaterLatch  = false;
            heaterStopMs = 0;
            twoPortValve.request(false);
        }
    } else {
        if (!sFault[H_SENSOR_HOT_PIPE] && !sFault[H_SENSOR_TANK_TOP]) {
            float diff = sTemp[H_SENSOR_HOT_PIPE] - sTemp[H_SENSOR_TANK_TOP];
            if (!twoPortValve.isOpen && diff > -1.0f) {
                twoPortValve.request(true);
            } else if (twoPortValve.isOpen && pumpRunning && diff < -5.0f) {
                twoPortValve.request(false);
            }
        }
    }
}

// PV export override: in winter mode, when PV export >= 500W, tell W to run summer solar
// logic by sending MODE_SUMMER in the H→W packet without changing the stored systemMode.
void updatePVExportOverride() {
    if (systemMode != MODE_WINTER) { pvExportOverride = false; pvExport500StartMs = 0; return; }
    bool pvActive = lastWPkt.growattValid ? ((lastWPkt.pv1W + lastWPkt.pv2W) >= 200) : (rtcHour() < 21);
    if (!pvExportOverride) {
        if (lastWPkt.pvExportW >= 500) {
            if (!pvExport500StartMs) pvExport500StartMs = millis();
            if (millis() - pvExport500StartMs >= 60000UL) pvExportOverride = true;
        } else {
            pvExport500StartMs = 0;
        }
    }
    if (pvExportOverride && !pvActive) { pvExportOverride = false; pvExport500StartMs = 0; }
}

// ============================================================
//  DAILY ENERGY ACCUMULATION  (kWh totals shown on page 2)
// ============================================================

float kwhPV          = 0.0f;
float kwhImport      = 0.0f;   // gross import only
float kwhExport      = 0.0f;   // gross export only
float kwhHeater      = 0.0f;
float kwhConsumption = 0.0f;

bool          energyInited       = false;
unsigned long energyLastMs       = 0;
uint8_t       energyLastDay      = 0xFF;
int16_t       energyLastPvOutW   = 0;
int16_t       energyLastImportW  = 0;   // net import; negative = net export
int16_t       energyLastBattChgW = 0;
uint8_t       energyLastHeaterPct = 0;
uint8_t       energyLastSocPct    = 0;

// Called once per received W packet. Integrates the values noted at the *previous*
// packet across the elapsed time to *this* packet (uncapped — a held last-known value
// is a better estimate of a comms gap than discarding it), then notes new values.
// growattValid==0 means W sent zeroed fields (Modbus-to-inverter down), not stale data —
// hold H's own last-known-good Growatt values rather than integrate the zeros.
void accumulateEnergy() {
    unsigned long nowMs = millis();

    if (rtcValid) {
        uint8_t today = rtcNow.day();
        if (energyLastDay != 0xFF && today != energyLastDay) {
            kwhPV = kwhImport = kwhExport = kwhHeater = kwhConsumption = 0.0f;
        }
        energyLastDay = today;
    }

    if (energyInited) {
        float dtH      = (nowMs - energyLastMs) / 3600000.0f;
        float heaterW  = energyLastHeaterPct * (HEATER_RATED_W / 100.0f);
        float consumptionW = (float)energyLastPvOutW + energyLastImportW
                            - energyLastBattChgW - heaterW;
        kwhPV          += energyLastPvOutW  * dtH / 1000.0f;
        // energyLastImportW is net (gridImportW - pvExportW); the two source
        // registers are mutually exclusive, so the positive/negative parts of
        // the net value recover true gross import and gross export exactly.
        kwhImport      += max((int16_t)0, energyLastImportW)  * dtH / 1000.0f;
        kwhExport      += max((int16_t)0, (int16_t)-energyLastImportW) * dtH / 1000.0f;
        kwhHeater      += heaterW           * dtH / 1000.0f;
        kwhConsumption += consumptionW      * dtH / 1000.0f;
    }
    energyLastMs = nowMs;
    energyInited = true;

    if (lastWPkt.growattValid) {
        energyLastPvOutW   = lastWPkt.pv1W + lastWPkt.pv2W;
        energyLastImportW  = lastWPkt.gridImportW - lastWPkt.pvExportW;
        energyLastBattChgW = lastWPkt.battChargeW;
        energyLastSocPct   = lastWPkt.battSocPct;
    }
    energyLastHeaterPct = heaterRunning ? heaterLevelPct() : 0;
}

// ============================================================
//  SD CARD LOGGING
// ============================================================

bool    sdAvailable   = false;
bool    sdEjected     = false;
File    logFile;
unsigned long lastLogMs = 0;
bool          needTSSinceLastLog = false; // latched: set on any time-sync reply, read+cleared by logDataRow()
File    energyLogFile;
unsigned long lastEnergyLogMs = 0;

static const uint8_t LOG_EXPECTED_COMMAS = 39; // 40 columns = 39 commas
static const uint8_t ENERGY_LOG_EXPECTED_COMMAS = 5; // 6 columns = 5 commas

void initSD() {
    if (SD.begin(PIN_SD_CS)) {
        sdAvailable = true;
#ifdef DEBUG_SERIAL
        Serial.println(F("SD: ok"));
#endif
        bool needHeader = true;
        if (SD.exists("log.csv")) {
            File f = SD.open("log.csv", FILE_READ);
            if (f) {
                uint8_t commas = 0;
                int c;
                while ((c = f.read()) != -1 && c != '\n' && c != '\r')
                    if (c == ',') commas++;
                f.close();
                if (commas == LOG_EXPECTED_COMMAS) needHeader = false;
            }
            if (needHeader) SD.remove("log.csv");
        }
        if (needHeader) {
            File f = SD.open("log.csv", FILE_WRITE);
            if (f) {
                f.println(F("datetime,solar_hot,solar_cold,ufh_sup,ufh_tmv,w_air,out_air,"
                            "tank_bot,tank_mid,tank_top,hot_pipe,cold_pipe,htr_out,htr_out_2,"
                            "pump_pct,htr_pct,export_w,import_w,"
                            "bus_v,fan1rpm,fan2rpm,fan_pct,"
                            "h_pump_pct,pv1_w,pv2_w,batt_w,batt_soc_pct,w_faults,h_faults,"
                            "log_valve,bot_valve,two_port_valve,"
                            "ufh_cold_v,solar_cold_v,vac_iso_v,fan_flap_v,"
                            "w_rx_good,w_rx_badframe,h_needts,w_rx_age_s"));
                f.close();
            }
        }
        logFile = SD.open("log.csv", FILE_WRITE);

        bool needEnergyHeader = true;
        if (SD.exists("energy.csv")) {
            File f = SD.open("energy.csv", FILE_READ);
            if (f) {
                uint8_t commas = 0;
                int c;
                while ((c = f.read()) != -1 && c != '\n' && c != '\r')
                    if (c == ',') commas++;
                f.close();
                if (commas == ENERGY_LOG_EXPECTED_COMMAS) needEnergyHeader = false;
            }
            if (needEnergyHeader) SD.remove("energy.csv");
        }
        if (needEnergyHeader) {
            File f = SD.open("energy.csv", FILE_WRITE);
            if (f) {
                f.println(F("datetime,pv_kwh,import_kwh,heater_kwh,consumption_kwh,soc_pct"));
                f.close();
            }
        }
        energyLogFile = SD.open("energy.csv", FILE_WRITE);
    } else {
#ifdef DEBUG_SERIAL
        Serial.println(F("SD: init failed"));
        Sd2Card card;
        card.init(SPI_HALF_SPEED, PIN_SD_CS);
        Serial.print(F("SD error code: 0x"));
        Serial.println(card.errorCode(), HEX);
        Serial.print(F("SD error data: 0x"));
        Serial.println(card.errorData(), HEX);
#endif
    }
}

void logDataRow() {
    if (!sdAvailable || sdEjected || !logFile) return;

    char tsBuf[20];
    if (rtcValid)
        snprintf(tsBuf, sizeof(tsBuf), "%04u-%02u-%02u %02u:%02u:%02u",
                 rtcNow.year(), rtcNow.month(), rtcNow.day(),
                 rtcNow.hour(), rtcNow.minute(), rtcNow.second());
    else
        snprintf(tsBuf, sizeof(tsBuf), "INVALID");
    logFile.print(tsBuf); logFile.print(',');
    // W temperatures
    auto pT = [&](int16_t v){ if(v==TEMP_FAULT) logFile.print("NaN"); else logFile.print(v/10.0f,1); logFile.print(','); };
    pT(lastWPkt.tempSolarHot); pT(lastWPkt.tempSolarCold);
    pT(lastWPkt.tempUFHSupply); pT(lastWPkt.tempUFHPostTMV);
    pT(lastWPkt.tempWorkshopAir); pT(lastWPkt.tempOutsideAir);
    // H temperatures
    for (uint8_t i = 0; i < H_NUM_SENSORS; i++) {
        if (sFault[i]) logFile.print("NaN"); else logFile.print(sTemp[i], 1); logFile.print(',');
    }
    // State
    logFile.print(lastWPkt.solarPumpDutyPct); logFile.print(',');
    logFile.print(heaterRunning ? heaterLevelPct() : 0); logFile.print(',');
    logFile.print(lastWPkt.pvExportW); logFile.print(',');
    logFile.print(lastWPkt.gridImportW); logFile.print(',');
    logFile.print(busVoltageV, 2); logFile.print(',');
    logFile.print(0); logFile.print(','); // fan1RPM — not available at H (W side)
    logFile.print(0); logFile.print(','); // fan2RPM
    logFile.print(lastWPkt.fanDutyPct); logFile.print(',');
    // H pump + Growatt detail (page 2) + active faults
    logFile.print((int)roundf(hPumpDuty)); logFile.print(',');
    logFile.print(lastWPkt.pv1W); logFile.print(',');
    logFile.print(lastWPkt.pv2W); logFile.print(',');
    logFile.print(lastWPkt.battChargeW); logFile.print(',');
    logFile.print(lastWPkt.battSocPct); logFile.print(',');
    logFile.print("0x"); logFile.print(lastWPkt.wFaultFlags, HEX); logFile.print(',');
    logFile.print("0x"); logFile.print(hFaultFlags, HEX); logFile.print(',');
    logFile.print(logBurnerCold.isOpen); logFile.print(',');
    logFile.print(botTankValve.isOpen);  logFile.print(',');
    logFile.print(twoPortValve.isOpen);  logFile.print(',');
    logFile.print((lastWPkt.valveStates & VSTATE_UFH_COLD_OPEN)   ? 1 : 0); logFile.print(',');
    logFile.print((lastWPkt.valveStates & VSTATE_SOLAR_COLD_OPEN) ? 1 : 0); logFile.print(',');
    logFile.print((lastWPkt.valveStates & VSTATE_VAC_ISO_OPEN)    ? 1 : 0); logFile.print(',');
    logFile.print((lastWPkt.valveStates & VSTATE_FAN_FLAP_OPEN)   ? 1 : 0); logFile.print(',');
    // RS485 diagnostics: W's running rx counters, and whether H sent a time-sync
    // reply since the last log row (latched — a sync reply is a single 250ms
    // event that would otherwise fall between 5s log rows)
    logFile.print(lastWPkt.rs485RxGood);     logFile.print(',');
    logFile.print(lastWPkt.rs485RxBadFrame); logFile.print(',');
    logFile.print(needTSSinceLastLog ? 1 : 0); logFile.print(',');
    logFile.println(lastWPkt.rs485RxAgeS);
    needTSSinceLastLog = false;
    logFile.flush();
}

void logEnergyRow() {
    if (!sdAvailable || sdEjected || !energyLogFile) return;

    char tsBuf[20];
    if (rtcValid)
        snprintf(tsBuf, sizeof(tsBuf), "%04u-%02u-%02u %02u:%02u:%02u",
                 rtcNow.year(), rtcNow.month(), rtcNow.day(),
                 rtcNow.hour(), rtcNow.minute(), rtcNow.second());
    else
        snprintf(tsBuf, sizeof(tsBuf), "INVALID");
    energyLogFile.print(tsBuf); energyLogFile.print(',');
    energyLogFile.print(kwhPV, 3);          energyLogFile.print(',');
    energyLogFile.print(kwhImport, 3);      energyLogFile.print(',');
    energyLogFile.print(kwhHeater, 3);      energyLogFile.print(',');
    energyLogFile.print(kwhConsumption, 3); energyLogFile.print(',');
    energyLogFile.println(energyLastSocPct);
    energyLogFile.flush();
}

void safeEjectSD() {
    if (!sdAvailable) return;
    if (logFile) logFile.close();
    if (energyLogFile) energyLogFile.close();
    SD.end();
    sdEjected   = true;
    sdAvailable = false;
}

void checkSDReinsert() {
    if (sdAvailable) return;
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

static uint32_t faultLogPrevW = 0, faultLogPrevH = 0;

void faultLogUpdate(uint32_t curW, uint32_t curH) {
    // Check each active fault against log — add new, resolve cleared
    uint32_t newW  = curW & ~faultLogPrevW;
    uint32_t newH  = curH & ~faultLogPrevH;
    uint32_t gonW  = faultLogPrevW & ~curW;
    uint32_t gonH  = faultLogPrevH & ~curH;

    auto addEntry = [&](uint32_t mask, bool isH) {
        if (faultLogCount == MAX_FAULT_ENTRIES) {
            // Log full — drop the oldest entry so the newest fault is never lost
            memmove(&faultLog[0], &faultLog[1], sizeof(FaultEntry) * (MAX_FAULT_ENTRIES - 1));
            faultLogCount--;
        }
        faultLog[faultLogCount++] = { mask, isH, (uint32_t)millis(), 0 };
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
    faultLogPrevW = curW; faultLogPrevH = curH;
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
bool         needPageRedraw   = false;
uint32_t     faultFlagsAtSleep = 0;
unsigned long lastDisplayRefreshMs = 0;
uint8_t      alertResetSeqTx       = 0;  // increment to signal alert reset to W
unsigned long actionFlashEndMs     = 0;  // non-zero while action-confirmation red flash is active
int8_t       pendingFanFullDeltaHr  = 0;  // one-shot: +/- hours; sent to W, cleared after transmit
int8_t       pendingFanBaseDeltaDay = 0;  // one-shot: +/- days;  sent to W, cleared after transmit

// Button state (debounced)
struct Button {
    uint8_t pin;
    volatile bool          state, prevState;
    volatile uint8_t       pendingCount;   // capped at 10; ISR-safe (1-byte atomic on AVR)
    volatile unsigned long lastMs;
    void poll() {
        bool raw = (digitalRead(pin) == LOW);
        if (raw != prevState) { lastMs = millis(); prevState = raw; }
        if (millis() - lastMs >= 3 && raw != state) {
            state = raw;
            if (state && pendingCount < 10) pendingCount++;
        }
    }
    // Consume one press (used for SELECT — keeps double-press logic working)
    bool pressed() {
        poll();
        noInterrupts(); bool p = (pendingCount > 0); if (p) pendingCount--; interrupts();
        return p;
    }
    // Consume all queued presses at once (used for UP/DOWN navigation)
    uint8_t pressCount() {
        poll();
        noInterrupts(); uint8_t n = pendingCount; pendingCount = 0; interrupts();
        return n;
    }
} btnSelect, btnUp, btnDown;
inline void pollBtns() { btnSelect.poll(); btnUp.poll(); btnDown.poll(); }

// Forward declarations
void drawFullPage();
static void updateHPump();
static void pollRS485();

// 1ms timer ISR — polls buttons independently of loop speed so any press >4ms is detected
ISR(TIMER3_COMPA_vect) { pollBtns(); }

// 50ms timer ISR — drives H pump timing independently of loop blocking time
ISR(TIMER1_COMPA_vect) { updateHPump(); }

// Track double-press SELECT for boost shortcut
unsigned long lastSelectMs   = 0;
bool          selectPending  = false;

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
    tft.setTextColor(C_WHITE, C_NAVY); tft.setTextSize(2);

    tft.setCursor(2, 2);
    tft.print(systemMode == MODE_WINTER ? "WINTER" : "SUMMER");

    tft.setCursor(80, 2);
    if      (boostMode == BOOST_5AM) tft.print("BOOST-5AM");
    else if (boostMode == BOOST_8HR) tft.print("BOOST-8HR");
    else                             tft.print("         ");

    tft.setCursor(196, 2);
    tft.print(nightCoolingEnabled ? "NC:ON " : "NC:OFF");

    tft.setCursor(272, 2);
    tft.print(solarTargetMode == SOLAR_TANK_PLUS8 ? "TK+ " : "MAX ");

    tft.setCursor(328, 2);
    tft.print("P"); tft.print(currentPage); tft.print("/5");

    tft.setCursor(380, 2);
    char buf[9];
    snprintf(buf, sizeof(buf), "%02u:%02u:%02u", rtcHour(), rtcMinute(), rtcSecond());
    tft.print(buf);
}

// ── Fault bar (bottom 20px) ───────────────────────────────

const char* faultNameW(uint32_t mask) {
    if (mask & FAULT_W_SOLAR_OVERHEAT_COLD)  return "Sol Ovht Cld";
    if (mask & FAULT_W_SOLAR_OVERHEAT_HOT)   return "Sol Ovht Hot";
    if (mask & FAULT_W_SOLAR_PUMP)            return "Sol Pump Flt";
    if (mask & FAULT_W_SOLAR_PUMP_OVERCURRENT) return "Sol Pump OC";
    if (mask & FAULT_W_UFH_OVERHEAT)          return "UFH Overheat";
    if (mask & FAULT_W_FROST_NOT_RECOVERING)  return "Frost NoRcvr";
    if (mask & FAULT_W_VAC_PUMP_OVERTIME)     return "Vac Overtime";
    if (mask & FAULT_W_GROWATT_COMMS)          return "Growatt Cmms";
    if (mask & FAULT_W_RS485_COMMS)           return "W RS485 Err";
    if (mask & FAULT_W_FAN1)                  return "Fan1 Fault";
    if (mask & FAULT_W_FAN2)                  return "Fan2 Fault";
    if (mask & FAULT_W_WINCH_OVER_OPEN)       return "Winch OvOpen";
    return "W Sensr Flt";
}
const char* faultNameH(uint32_t mask) {
    if (mask & FAULT_H_HEATER_OVERHEAT_SHUT)  return "Htr Ovht STP";
    if (mask & FAULT_H_HEATER_ELEMENT_FAIL)   return "Htr Elem Flt";
    if (mask & FAULT_H_RS485_COMMS)           return "H RS485 Err";
    if (mask & FAULT_H_BUS_VOLTAGE_LOW)       return "15V Bus Low";
    if (mask & FAULT_H_GRID_OUTAGE)           return "Grid Outage";
    if (mask & FAULT_H_HEATER_IMPORT_TRIP)   return "Htr Import Trp";
    return "H Sensr Flt";
}

static uint8_t  faultBarIdx    = 0;
static unsigned long faultScrollMs = 0;

void drawFaultBar(uint32_t wFaults, uint32_t hFaults) {
    static uint16_t prevBg = 0xFFFF;
    uint16_t bg = (wFaults || hFaults) ? C_RED : C_NAVY;
    if (bg != prevBg) { tft.fillRect(0, 300, 480, 20, bg); prevBg = bg; }
    tft.setTextColor(C_WHITE, bg); tft.setTextSize(2);

    if (!wFaults && !hFaults) {
        tft.setCursor(2, 302); tft.print("No faults                  ");
        return;
    }

    // Build fault name for current scroll position
    uint8_t idx = 0;
    char buf[30]; buf[0] = 0;
    for (uint8_t b = 0; b < 32; b++) {
        if (wFaults & (1UL << b)) {
            if (idx == faultBarIdx) { strncpy(buf, faultNameW(1UL << b), sizeof(buf) - 1); buf[sizeof(buf)-1] = '\0'; break; }
            idx++;
        }
    }
    if (!buf[0]) {
        for (uint8_t b = 0; b < 32; b++) {
            if (hFaults & (1UL << b)) {
                if (idx == faultBarIdx) { strncpy(buf, faultNameH(1UL << b), sizeof(buf) - 1); buf[sizeof(buf)-1] = '\0'; break; }
                idx++;
            }
        }
    }
    if (!buf[0]) { faultBarIdx = 0; return; }

    // Pad to fixed width to overwrite previous text cleanly
    char padded[42];
    snprintf(padded, sizeof(padded), "%-38s", buf);
    tft.setCursor(2, 302); tft.print(padded);

    // Scroll every 3s
    if (millis() - faultScrollMs >= 3000) {
        faultScrollMs = millis();
        faultBarIdx = (faultBarIdx + 1) % max(idx, (uint8_t)1);
    }
}

// ── Page 1: Heating System ────────────────────────────────

void drawPage1(uint32_t wF) {
    tft.setTextSize(2);

    // Two columns: W temps left (lx=4, vx=155), H temps right (lx=244, vx=355)
    auto printRow = [&](uint16_t lx, uint16_t vx, uint8_t row, const char* lbl, int16_t val, bool fault) {
        uint16_t y = 30 + row * 18;
        tft.setTextColor(C_WHITE, C_BLACK); tft.setCursor(lx, y); tft.print(lbl);
        char vbuf[10], buf[10];
        if (fault) strcpy(vbuf, "FAULT");
        else { dtostrf(val / 10.0f, 1, 1, vbuf); uint8_t n = strlen(vbuf); vbuf[n]=' '; vbuf[n+1]='C'; vbuf[n+2]='\0'; }
        snprintf(buf, sizeof(buf), "%-7s", vbuf);
        tft.setTextColor(fault ? C_RED : C_CYAN, C_BLACK); tft.setCursor(vx, y); tft.print(buf);
    };

    // Left column: W temperatures (rows 0-5)
    printRow(4, 155, 0, "Solar Hot",    lastWPkt.tempSolarHot,    lastWPkt.tempSolarHot    == TEMP_FAULT);
    printRow(4, 155, 1, "Solar Cold",   lastWPkt.tempSolarCold,   lastWPkt.tempSolarCold   == TEMP_FAULT);
    printRow(4, 155, 2, "UFH Supply",   lastWPkt.tempUFHSupply,   lastWPkt.tempUFHSupply   == TEMP_FAULT);
    printRow(4, 155, 3, "UFH Post TMV", lastWPkt.tempUFHPostTMV,  lastWPkt.tempUFHPostTMV  == TEMP_FAULT);
    pollBtns();
    printRow(4, 155, 4, "Workshop Air", lastWPkt.tempWorkshopAir, lastWPkt.tempWorkshopAir == TEMP_FAULT);
    printRow(4, 155, 5, "Outside Air",  lastWPkt.tempOutsideAir,  lastWPkt.tempOutsideAir  == TEMP_FAULT);

    // Right column: H temperatures (rows 0-6); rows 5-6 are the two heater sensors
    static const uint8_t hSensorIdx[7] = {
        H_SENSOR_TANK_BOT, H_SENSOR_TANK_MID, H_SENSOR_TANK_TOP,
        H_SENSOR_HOT_PIPE, H_SENSOR_COLD_PIPE,
        H_SENSOR_HEATER_OUT_2,  // row 5: "Heater"
        H_SENSOR_HEATER_OUT     // row 6: "Htr Out"
    };
    static const char* hNames[7] = {
        "Tank Bot","Tank Mid","Tank Top","Hot Pipe","Cold Pipe","Heater","Htr Out" };
    for (uint8_t row = 0; row < 7; row++) {
        uint8_t si = hSensorIdx[row];
        bool fault = sRawFault[si];
        int16_t enc = fault ? TEMP_FAULT : (int16_t)(sTemp[si] * 10.0f);
        printRow(244, 355, row, hNames[row], enc, fault);
        if (row % 3 == 2) pollBtns();
    }

    // Power/valve/winch section — extra 12px gap after the 7-row temp block
    const uint16_t yS = 30 + 7 * 18 + 12;

    // Power row
    uint16_t y = yS;
    tft.setTextColor(C_WHITE, C_BLACK); tft.setCursor(4, y); tft.print("Heater");
    { char tmp[8], buf[8]; snprintf(tmp, sizeof(tmp), "%dW", heaterRunning ? (int)(heaterLevelPct10() * 3) : 0); snprintf(buf, sizeof(buf), "%-5s", tmp); tft.setTextColor(C_CYAN, C_BLACK); tft.setCursor(82, y); tft.print(buf); }
    tft.setTextColor(C_WHITE, C_BLACK); tft.setCursor(172, y); tft.print("Sol H:");
    { char tmp[6], buf[6]; snprintf(tmp, sizeof(tmp), "%d%%", (int)roundf(hPumpDuty)); snprintf(buf, sizeof(buf), "%-4s", tmp); tft.setTextColor(C_CYAN, C_BLACK); tft.setCursor(250, y); tft.print(buf); }
    tft.setTextColor(C_WHITE, C_BLACK); tft.setCursor(304, y); tft.print("Sol W:");
    { char tmp[6], buf[6]; snprintf(tmp, sizeof(tmp), "%d%%", lastWPkt.solarPumpDutyPct); snprintf(buf, sizeof(buf), "%-4s", tmp); tft.setTextColor(C_CYAN, C_BLACK); tft.setCursor(382, y); tft.print(buf); }

    // Valve states
    y = yS + 18;
    tft.setTextColor(C_WHITE, C_BLACK); tft.setCursor(4, y); tft.print("V:");
    tft.setCursor(30, y);
    auto vStr = [&](const char* n, bool open) {
        tft.setTextColor(open ? C_GREEN : C_WHITE, C_BLACK); tft.print(n); tft.print(' ');
    };
    vStr("UFH", lastWPkt.valveStates & VSTATE_UFH_COLD_OPEN);
    vStr("SOL", lastWPkt.valveStates & VSTATE_SOLAR_COLD_OPEN);
    vStr("VAC", lastWPkt.valveStates & VSTATE_VAC_ISO_OPEN);
    vStr("LB",  logBurnerCold.isOpen);
    vStr("BOT", botTankValve.isOpen);
    vStr("2PT", twoPortValve.isOpen);

    // Winch state
    y = yS + 36;
    tft.setTextColor(C_WHITE, C_BLACK); tft.setCursor(4, y); tft.print("Winch:");
    { const char* ws[] = {"STOP","OPEN","CLOSE"}; char wsbuf[8]; snprintf(wsbuf, sizeof(wsbuf), "%-6s", ws[min(lastWPkt.winchState, (uint8_t)2)]); tft.setTextColor(C_CYAN, C_BLACK); tft.setCursor(82, y); tft.print(wsbuf); }
    tft.setTextColor((lastWPkt.winchReedFlags & WREED_SAFETY_LIMIT) ? C_RED : C_BLACK, C_BLACK); tft.print(" OVER!");

    // 15V bus voltage
    y = yS + 54;
    tft.setTextColor(C_WHITE, C_BLACK); tft.setCursor(4, y); tft.print("Bus:");
    { char bv[10]; dtostrf(busVoltageV, 1, 2, bv);
      uint8_t n = strlen(bv); bv[n] = 'V'; bv[n+1] = ' '; bv[n+2] = '\0';
      char buf[10]; snprintf(buf, sizeof(buf), "%-8s", bv);
      tft.setTextColor(C_CYAN, C_BLACK); tft.setCursor(64, y); tft.print(buf); }
}

// ── Page 2: Energy / Growatt ──────────────────────────────

void drawPage2() {
    tft.setTextSize(2);

    bool gv = hasWPkt && lastWPkt.growattValid;

    // Helper: print label+value pair (left column lx, right column rx)
    auto wRow = [&](uint16_t lx, uint16_t lVx, uint16_t rx, uint16_t rVx, uint16_t y,
                    const char* lLabel, const char* lVal,
                    const char* rLabel, const char* rVal) {
        tft.setTextColor(C_WHITE, C_BLACK); tft.setCursor(lx, y); tft.print(lLabel);
        tft.setTextColor(C_CYAN,  C_BLACK); tft.setCursor(lVx, y); tft.print(lVal);
        tft.setTextColor(C_WHITE, C_BLACK); tft.setCursor(rx, y); tft.print(rLabel);
        tft.setTextColor(C_CYAN,  C_BLACK); tft.setCursor(rVx, y); tft.print(rVal);
    };

    char lv[10], rv[10];
    uint16_t y = 28;
    const uint16_t ROW = 22;

    // PV1 / PV2
    snprintf(lv, sizeof(lv), gv ? "%-6dW" : "--    ", gv ? lastWPkt.pv1W : 0);
    snprintf(rv, sizeof(rv), gv ? "%-6dW" : "--    ", gv ? lastWPkt.pv2W : 0);
    wRow(4, 64, 244, 304, y, "PV1:", lv, "PV2:", rv); y += ROW;

    // PV Total (PV1 + PV2)
    snprintf(lv, sizeof(lv), gv ? "%-6dW" : "--    ", gv ? (lastWPkt.pv1W + lastWPkt.pv2W) : 0);
    wRow(4, 88, 244, 304, y, "PV Tot:", lv, "            ", ""); y += ROW;

    // Import (net; negative = exporting)
    int16_t importW = gv ? (int16_t)(lastWPkt.gridImportW - lastWPkt.pvExportW) : 0;
    snprintf(lv, sizeof(lv), gv ? "%-6dW" : "--    ", importW);
    wRow(4, 64, 244, 304, y, "Import:", lv, "            ", ""); y += ROW;

    int16_t hW = heaterRunning ? (int16_t)(heaterLevelPct10() * 3) : 0;

    // Consumption (PV + import - battery charge - heater)
    snprintf(lv, sizeof(lv), gv ? "%-6dW" : "--    ",
        gv ? (int16_t)(lastWPkt.pv1W + lastWPkt.pv2W + importW - lastWPkt.battChargeW - hW) : 0);
    wRow(4, 76, 244, 304, y, "Cons:", lv, "            ", ""); y += ROW;

    // Heater W
    {
        snprintf(lv, sizeof(lv), "%-6dW", hW);
        tft.setTextColor(C_WHITE, C_BLACK); tft.setCursor(4, y); tft.print("Heater:");
        tft.setTextColor(heaterRunning ? C_GREEN : C_WHITE, C_BLACK);
        tft.setCursor(100, y); tft.print(lv);
        tft.setTextColor(gv ? C_BLACK : C_YELLOW, C_BLACK);
        tft.setCursor(244, y); tft.print(gv ? "          " : "No Growatt");
        y += ROW;
    }

    // Battery charge/discharge from Modbus r1012/r1010 (+ve=charging, -ve=discharging)
    if (gv) {
        int16_t bkw10 = (int16_t)(((int32_t)lastWPkt.battChargeW * 10) / 1000); // ×0.1kW
        snprintf(lv, sizeof(lv), "%c%d.%dkW",
            lastWPkt.battChargeW < 0 ? '-' : '+', abs(bkw10) / 10, abs(bkw10) % 10);
    } else { snprintf(lv, sizeof(lv), "--     "); }
    wRow(4, 76, 244, 304, y, "Batt:", lv, "            ", ""); y += ROW;

    // Battery SOC
    if (gv) snprintf(lv, sizeof(lv), "%u%%  ", lastWPkt.battSocPct);
    else    snprintf(lv, sizeof(lv), "--  ");
    wRow(4, 64, 244, 304, y, "SOC:", lv, "            ", ""); y += ROW;

    // Daily kWh totals
    auto kwhStr = [&](char* buf, size_t n, float v) {
        char t[8]; dtostrf(v, 1, 1, t); snprintf(buf, n, "%-6s", t);
    };
    kwhStr(lv, sizeof(lv), kwhPV);      kwhStr(rv, sizeof(rv), kwhImport);
    wRow(4, 80, 244, 336, y, "PVkWh:", lv, "ImpkWh:", rv); y += ROW;

    kwhStr(lv, sizeof(lv), kwhExport);  kwhStr(rv, sizeof(rv), kwhHeater);
    wRow(4, 96, 244, 336, y, "ExpkWh:", lv, "HtrkWh:", rv); y += ROW;

    kwhStr(lv, sizeof(lv), kwhConsumption);
    wRow(4, 100, 244, 336, y, "ConsKWh:", lv, "            ", ""); y += ROW;
}

// ── Page 3: Fault History ────────────────────────────────

uint8_t faultHistScrollOffset = 0;

// Wall-clock HH:MM (or +Xm/+Xh uptime fallback when RTC invalid) for a fault
// onset/resolved millis() mark — shared by the start and resolved columns.
static void formatFaultTimeOfDay(uint32_t markMs, char* buf, uint8_t bufSz) {
    if (rtcValid) {
        uint32_t secAgo  = (millis() - markMs) / 1000UL;
        uint32_t nowSec  = (uint32_t)rtcHour() * 3600UL + (uint32_t)rtcMinute() * 60UL + rtcSecond();
        uint32_t markSec = (nowSec + 86400UL - secAgo % 86400UL) % 86400UL;
        snprintf(buf, bufSz, "%02u:%02u", (unsigned)(markSec / 3600), (unsigned)((markSec % 3600) / 60));
    } else {
        uint32_t s = markMs / 1000UL;
        if (s < 3600) snprintf(buf, bufSz, "+%um", (unsigned)(s / 60));
        else          snprintf(buf, bufSz, "+%uh", (unsigned)(s / 3600));
    }
}

void drawPage3() {
    tft.fillRect(0, 20, 480, 280, C_BLACK);
    tft.setTextSize(2);

    if (faultLogCount == 0) {
        tft.setTextColor(C_WHITE); tft.setCursor(10, 100);
        tft.print(F("No faults since restart"));
        return;
    }

    // Every occurrence gets its own line (a fault that resolves and later
    // recurs shows as a second entry with its own start/resolved times).
    // Active (unresolved) entries float to the top regardless of onset time
    // so a current fault never scrolls off below older resolved ones; within
    // each group, ordered most-recent-first so newest occurrences stay on
    // screen even once the log grows past the visible row count.
    uint8_t dispIdx[MAX_FAULT_ENTRIES];
    uint8_t dispCount = faultLogCount;
    uint8_t dispN = 0;
    for (uint8_t i = 0; i < faultLogCount; i++) {
        uint8_t idx = faultLogCount - 1 - i;
        if (faultLog[idx].resolvedMs == 0) dispIdx[dispN++] = idx;
    }
    for (uint8_t i = 0; i < faultLogCount; i++) {
        uint8_t idx = faultLogCount - 1 - i;
        if (faultLog[idx].resolvedMs != 0) dispIdx[dispN++] = idx;
    }

    const uint8_t ROW_H      = 18;
    const uint8_t VISIBLE    = (280 - 4) / ROW_H;  // 15 rows per column
    const uint8_t NAME_CHARS = 13;                 // size2 chars reserved for the name field

    for (uint8_t col = 0; col < 2; col++) {
        uint16_t lx   = col ? 244 : 4;
        uint16_t tx   = lx + NAME_CHARS * 12;      // size2 char width = 12px
        uint8_t  base = faultHistScrollOffset + col * VISIBLE;

        for (uint8_t r = 0; r < VISIBLE && (base + r) < dispCount; r++) {
            FaultEntry& e      = faultLog[dispIdx[base + r]];
            uint16_t    y      = 30 + r * ROW_H;
            bool        active = (e.resolvedMs == 0);
            uint16_t    color  = active ? C_RED : C_WHITE;

            tft.setTextSize(2);
            tft.setTextColor(color);
            tft.setCursor(lx, y);
            char name[NAME_CHARS + 1];
            // avr-libc's vfprintf does not support '*' dynamic width — width must be literal
            snprintf(name, sizeof(name), "%-13s",
                     e.isH ? faultNameH(e.faultMask) : faultNameW(e.faultMask));
            tft.print(name);

            char startBuf[8], timeBuf[16];
            formatFaultTimeOfDay(e.onsetMs, startBuf, sizeof(startBuf));
            if (active) {
                strlcpy(timeBuf, startBuf, sizeof(timeBuf));
            } else {
                char endBuf[8];
                formatFaultTimeOfDay(e.resolvedMs, endBuf, sizeof(endBuf));
                snprintf(timeBuf, sizeof(timeBuf), "%s-%s", startBuf, endBuf);
            }
            tft.setTextSize(1);
            tft.setTextColor(color);
            tft.setCursor(tx, y + 4);   // vertically center smaller time text in the row
            tft.print(timeBuf);
        }
    }
}

// ── Page 4: System Controls ───────────────────────────────

struct CtrlItem {
    const char* label;
    uint8_t     valueType; // 0=enum, 1=float×2, 2=int, 3=action
};

static const CtrlItem PAGE4_ITEMS[] = {
    { "Boost",       0 },
    { "Boost Tgt",   1 },
    { "Mode",        0 },
    { "Man Heater",  0 },
    { "Night Cool",  0 },
    { "Solar Tgt",   0 },
    { "Fan Speed",   2 },
    { "Fan Full",    2 },
    { "Fan Base",    2 },
    { "Brightness",  2 },
    { "Set Hour",    2 },
    { "Set Min",     2 },
    { "Set Sec",     2 },
    { "SD Eject",    3 },
    { "Alrt Reset",  3 },  // always shown; clears page 3 fault history + resets heater lockout
};
static const uint8_t PAGE4_COUNT = 15;

void getPage4Value(uint8_t item, char* buf, uint8_t bufSz) {
    switch (item) {
        case 0: { const char* bm[] = {"Off","Boost-5am","8hr"}; strlcpy(buf, bm[boostMode], bufSz); break; }
        case 1: snprintf(buf, bufSz, "%.1fC", (double)boostTarget); break;
        case 2: strlcpy(buf, systemMode == MODE_WINTER ? "Winter" : "Summer", bufSz); break;
        case 3: { const char* mm[] = {"Off","SOC-Lim","Manual On"}; strlcpy(buf, mm[manualHeaterMode], bufSz); break; }
        case 4: strlcpy(buf, nightCoolingEnabled ? "On" : "Off", bufSz); break;
        case 5: strlcpy(buf, solarTargetMode == SOLAR_TANK_PLUS8 ? "Tank+" : "Max", bufSz); break;
        case 6: snprintf(buf, bufSz, "%d%%", lastWPkt.fanDutyPct); break;
        case 7: snprintf(buf, bufSz, "%luh", lastWPkt.fanFullTimerSecs / 3600); break;
        case 8: snprintf(buf, bufSz, "%lud", lastWPkt.fanBaseTimerSecs / 86400); break;
        case 9:  snprintf(buf, bufSz, "%d%%", displayBrightness); break;
        case 10: snprintf(buf, bufSz, "%02u", rtcHour());   break;
        case 11: snprintf(buf, bufSz, "%02u", rtcMinute()); break;
        case 12: snprintf(buf, bufSz, "%02u", rtcSecond()); break;
        case 13: strlcpy(buf, sdAvailable ? "Eject" : "Ejected", bufSz); break;
        case 14: strlcpy(buf, "Confirm", bufSz); break;
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
        case 5: solarTargetMode = (solarTargetMode == SOLAR_TANK_PLUS8) ? SOLAR_MAX : SOLAR_TANK_PLUS8; markSettingsDirty(); break;
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
        case 10: {
            uint8_t h = (uint8_t)((rtcNow.hour() + 24 + dir) % 24);
            rtc.adjust(DateTime(rtcNow.year(), rtcNow.month(), rtcNow.day(), h, rtcNow.minute(), rtcNow.second()));
            rtcNow = rtc.now();
            break;
        }
        case 11: {
            uint8_t m = (uint8_t)((rtcNow.minute() + 60 + dir) % 60);
            rtc.adjust(DateTime(rtcNow.year(), rtcNow.month(), rtcNow.day(), rtcNow.hour(), m, rtcNow.second()));
            rtcNow = rtc.now();
            break;
        }
        case 12: {
            uint8_t s = (uint8_t)((rtcNow.second() + 60 + dir) % 60);
            rtc.adjust(DateTime(rtcNow.year(), rtcNow.month(), rtcNow.day(), rtcNow.hour(), rtcNow.minute(), s));
            rtcNow = rtc.now();
            break;
        }
    }
}

void page4Action(uint8_t item) {
    if (item == 13) safeEjectSD();
    if (item == 14) {
        alertResetSeqTx++;
        // Clear fault log and reset change-detection so currently-active faults
        // are not immediately re-added — they'll re-appear only if they toggle again
        faultLogCount = 0;
        faultHistScrollOffset = 0;
        faultLogPrevW = hasWPkt ? lastWPkt.wFaultFlags : 0;
        faultLogPrevH = hFaultFlags;
        // Clear heater lockout so heater can restart if temperature has recovered
        heaterHardLockout  = false;
        heaterLevelCap     = 20;
        clearFaultH(FAULT_H_HEATER_OVERHEAT_SHUT);
    }
    actionFlashEndMs = millis() + 300;
}

uint8_t page4VisibleCount() {
    return PAGE4_COUNT;   // Alrt Reset always shown — it also clears page 3's fault history
}

void drawPage4() {
    tft.setTextSize(2);
    uint8_t visCount = page4VisibleCount();
    static bool    prevSel[PAGE4_COUNT] = {};
    static bool    prevBackSel4 = false;
    static uint8_t prevVisCount = 255;
    static bool    prevFlashing = false;

    // Clear content area and reset statics when item count changes so the
    // Back row (which shifts position) and newly appearing/disappearing items
    // don't leave stale text on screen.
    if (visCount != prevVisCount) {
        tft.fillRect(0, 20, 480, 280, C_BLACK);
        memset(prevSel, 0, sizeof(prevSel));
        prevBackSel4 = false;
        prevVisCount = visCount;
    }

    bool flashing = (actionFlashEndMs != 0);
    bool flashChanged = (flashing != prevFlashing);
    prevFlashing = flashing;

    for (uint8_t i = 0; i < visCount; i++) {
        uint8_t  col = (i >= 6) ? 1 : 0;
        uint8_t  row = col ? i - 6 : i;
        uint16_t y   = 30 + row * 20;
        uint16_t rx  = col ? 240 : 0;
        uint16_t lx  = col ? 244 : 4;
        uint16_t vx  = col ? 368 : 128;
        bool     sel = (navMode != NAV_PAGE && selectedItem == i);
        uint16_t bg  = (sel && flashing) ? C_RED : (sel ? C_NAVY : C_BLACK);
        if (sel != prevSel[i] || (sel && flashChanged)) { tft.fillRect(rx, y - 2, 240, 20, bg); prevSel[i] = sel; }
        tft.setTextColor(C_WHITE, bg); tft.setCursor(lx, y); tft.print(PAGE4_ITEMS[i].label);
        char buf[20]; getPage4Value(i, buf, sizeof(buf));
        if (col == 0) { char pbuf[12]; snprintf(pbuf, sizeof(pbuf), "%-9s", buf);  tft.setTextColor(flashing && sel ? C_WHITE : C_CYAN, bg); tft.setCursor(vx, y); tft.print(pbuf); }
        else          { char pbuf[10]; snprintf(pbuf, sizeof(pbuf), "%-8s", buf);  tft.setTextColor(flashing && sel ? C_WHITE : C_CYAN, bg); tft.setCursor(vx, y); tft.print(pbuf); }
        if (sel) tft.drawRect(rx, y - 2, 240, 20, flashing ? C_WHITE : (navMode == NAV_OPTION) ? C_RED : C_AMBER);
        if (i % 4 == 3) pollBtns();
    }
    pollBtns();

    // Back row (full width, below the taller column)
    bool backSel = (navMode != NAV_PAGE && selectedItem == visCount);
    uint8_t  rightRows = visCount > 6 ? visCount - 6 : 6;
    uint16_t yb  = 30 + rightRows * 20;
    uint16_t bgb = backSel ? C_NAVY : C_BLACK;
    if (backSel != prevBackSel4) { tft.fillRect(0, yb - 2, 480, 20, bgb); prevBackSel4 = backSel; }
    tft.setTextColor(C_WHITE, bgb); tft.setCursor(4, yb); tft.print("< Back to pages");
    if (backSel) tft.drawRect(0, yb - 2, 480, 20, C_AMBER);
}

// ── Page 5: Valve States & Manual Override ────────────────

bool manualOverrideActive = false;
uint8_t overrideValveStates = 0; // OVER_* bitmask

void drawPage5() {
    tft.setTextSize(2);

    // Header: only fillRect when override state changes
    static bool prevManual5 = false;
    if (manualOverrideActive != prevManual5) {
        tft.fillRect(0, 20, 480, 20, manualOverrideActive ? C_RED : C_BLACK);
        prevManual5 = manualOverrideActive;
    }
    if (manualOverrideActive) {
        tft.setTextColor(C_WHITE, C_RED); tft.setCursor(4, 22);
        tft.print(F("MANUAL OVERRIDE ACTIVE"));
    }

    static bool prevSel5[6] = {};
    auto vRow = [&](uint8_t r, const char* lbl, bool open) {
        uint16_t y   = 42 + r * 22;
        bool     sel = (navMode != NAV_PAGE && selectedItem == r);
        uint16_t bg  = sel ? C_NAVY : C_BLACK;
        if (sel != prevSel5[r]) { tft.fillRect(0, y - 2, 480, 20, bg); prevSel5[r] = sel; }
        tft.setTextColor(C_WHITE, bg); tft.setCursor(4, y); tft.print(lbl);
        char stbuf[8]; snprintf(stbuf, sizeof(stbuf), "%-7s", open ? "OPEN" : "CLOSED");
        tft.setTextColor(open ? C_GREEN : C_RED, bg); tft.setCursor(230, y); tft.print(stbuf);
        pollBtns();
    };

    vRow(0, "UFH Cold (W)",    manualOverrideActive ? (overrideValveStates & (1<<0)) : (lastWPkt.valveStates & VSTATE_UFH_COLD_OPEN));
    vRow(1, "Solar Cold (W)",  manualOverrideActive ? (overrideValveStates & (1<<1)) : (lastWPkt.valveStates & VSTATE_SOLAR_COLD_OPEN));
    vRow(2, "Vac Iso (W)",     manualOverrideActive ? (overrideValveStates & (1<<2)) : (lastWPkt.valveStates & VSTATE_VAC_ISO_OPEN));
    vRow(3, "Log Burner",      logBurnerCold.isOpen);
    vRow(4, "Bot Tank",        botTankValve.isOpen);
    vRow(5, "2-Port",          twoPortValve.isOpen);

    // Override toggle row (index 6)
    {
        static bool prevOvrSel5 = false;
        bool ovrSel = (navMode != NAV_PAGE && selectedItem == 6);
        uint16_t yo = 42 + 6 * 22;
        uint16_t bgo = ovrSel ? C_NAVY : C_BLACK;
        if (ovrSel != prevOvrSel5) { tft.fillRect(0, yo - 2, 480, 20, bgo); prevOvrSel5 = ovrSel; }
        tft.setTextColor(C_WHITE, bgo); tft.setCursor(4, yo);
        tft.print(manualOverrideActive ? "Disable Override" : "Enable Override ");
        if (ovrSel) tft.drawRect(0, yo - 2, 480, 20, C_AMBER);
        pollBtns();
    }

    // Back row (index 7)
    {
        static bool prevBackSel5 = false;
        bool backSel = (navMode != NAV_PAGE && selectedItem == 7);
        uint16_t yb = 42 + 7 * 22;
        uint16_t bgb = backSel ? C_NAVY : C_BLACK;
        if (backSel != prevBackSel5) { tft.fillRect(0, yb - 2, 480, 20, bgb); prevBackSel5 = backSel; }
        tft.setTextColor(C_WHITE, bgb); tft.setCursor(4, yb); tft.print("< Back to pages");
        if (backSel) tft.drawRect(0, yb - 2, 480, 20, C_AMBER);
        pollBtns();
    }

    // Winch details: only fillRect when state or flags change
    uint16_t y = 42 + 9 * 22;
    static uint8_t prevWinchState5  = 0xFF;
    static uint8_t prevReedFlags5   = 0xFF;
    if (lastWPkt.winchState != prevWinchState5 || lastWPkt.winchReedFlags != prevReedFlags5) {
        tft.fillRect(0, y - 2, 480, 18, C_BLACK);
        prevWinchState5 = lastWPkt.winchState;
        prevReedFlags5  = lastWPkt.winchReedFlags;
    }
    tft.setTextColor(C_WHITE, C_BLACK); tft.setCursor(4, y); tft.print("Winch:");
    { const char* ws[] = {"Idle","Open","Close"}; char wbuf[8]; snprintf(wbuf, sizeof(wbuf), "%-6s", ws[min(lastWPkt.winchState, (uint8_t)2)]); tft.setTextColor(C_CYAN, C_BLACK); tft.setCursor(82, y); tft.print(wbuf); }
    tft.setCursor(175, y);
    if (lastWPkt.winchReedFlags & WREED_FULLY_OPEN)   { tft.setTextColor(C_AMBER, C_BLACK); tft.print(" OPEN"); }
    if (lastWPkt.winchReedFlags & WREED_FULLY_CLOSED)  { tft.setTextColor(C_GREEN, C_BLACK); tft.print(" CLSD"); }
    if (lastWPkt.winchReedFlags & WREED_MANUAL_LOCK)   { tft.setTextColor(C_AMBER, C_BLACK); tft.print(" LOCK"); }
    if (lastWPkt.winchReedFlags & WREED_SAFETY_LIMIT)  { tft.setTextColor(C_RED,   C_BLACK); tft.print(" OVER!"); }
}

// ── Full page dispatch ────────────────────────────────────

void drawFullPage() {
    tft.fillRect(0, 0, 480, 20, C_NAVY);
    for (uint8_t s = 0; s < 8; s++) { tft.fillRect(0, 20 + s * 35, 480, 35, C_BLACK); pollBtns(); updateHPump(); pollRS485(); }
    drawStatusBar();
    updateHPump();
    switch (currentPage) {
        case 1: drawPage1(lastWPkt.wFaultFlags); break;
        case 2: drawPage2(); break;
        case 3: drawPage3(); break;
        case 4: drawPage4(); break;
        case 5: drawPage5(); break;
    }
    updateHPump();
    drawFaultBar(lastWPkt.wFaultFlags, hFaultFlags);
    needFullRedraw = false;
}

// ── Button handling ───────────────────────────────────────

void handleSelectDouble() {
    // Toggle Boost at 5am from any page
    boostMode = (boostMode == BOOST_5AM) ? BOOST_OFF : BOOST_5AM;
}

void handleButtons() {
    bool    selP  = btnSelect.pressed();
    uint8_t upCnt = btnUp.pressCount();
    uint8_t dnCnt = btnDown.pressCount();
    if (!selP && !upCnt && !dnCnt) return;

    wakeDisplay();
    unsigned long now = millis();

    // Double-press SELECT in page scroll mode
    if (selP && navMode == NAV_PAGE) {
        if (now - lastSelectMs < 400) { handleSelectDouble(); needFullRedraw = true; }
        lastSelectMs = now;
    }

    // Inactivity exit to page scroll
    if (now - lastButtonMs > DISPLAY_INACTIVITY_MS && navMode != NAV_PAGE) {
        navMode = NAV_PAGE; needPageRedraw = true; return;
    }

    switch (navMode) {
        case NAV_PAGE:
            if (upCnt) { currentPage = (uint8_t)(((int)currentPage - 1 + 5 - upCnt % 5) % 5 + 1); needFullRedraw = true; }
            if (dnCnt) { currentPage = (uint8_t)(((int)currentPage - 1 + dnCnt)          % 5 + 1); needFullRedraw = true; }
            if (selP && (currentPage == 4 || currentPage == 5)) { navMode = NAV_ITEM; selectedItem = 0; needPageRedraw = true; }
            break;

        case NAV_ITEM: {
            // backIdx is always the last slot; real items are 0..backIdx-1
            uint8_t realItems = (currentPage == 4) ? page4VisibleCount() : 7;
            uint8_t backIdx = realItems;
            uint8_t n = backIdx + 1;

            if (upCnt) { selectedItem = (uint8_t)((selectedItem + n - upCnt % n) % n); needPageRedraw = true; }
            if (dnCnt) { selectedItem = (uint8_t)((selectedItem     + dnCnt)      % n); needPageRedraw = true; }
            if (selP) {
                if (selectedItem == backIdx) {
                    navMode = NAV_PAGE; needPageRedraw = true;
                } else if (currentPage == 4) {
                    if (PAGE4_ITEMS[selectedItem].valueType == 3) { page4Action(selectedItem); needPageRedraw = true; }
                    else { navMode = NAV_OPTION; needPageRedraw = true; }
                } else if (currentPage == 5) {
                    if (selectedItem == 6) {
                        // Override toggle row
                        manualOverrideActive = !manualOverrideActive;
                        if (!manualOverrideActive) overrideValveStates = 0;
                        needPageRedraw = true;
                    } else if (manualOverrideActive) {
                        if      (selectedItem < 3)  { overrideValveStates ^= (1 << selectedItem); needPageRedraw = true; }
                        else if (selectedItem == 3) { logBurnerCold.request(!logBurnerCold.isOpen); needPageRedraw = true; }
                        else if (selectedItem == 4) { botTankValve.request(!botTankValve.isOpen);   needPageRedraw = true; }
                        else if (selectedItem == 5) { twoPortValve.request(!twoPortValve.isOpen);   needPageRedraw = true; }
                    }
                }
            }
            break;
        }

        case NAV_OPTION:
            if (upCnt) { for (uint8_t i = 0; i < upCnt; i++) page4Adjust(selectedItem, -1); needPageRedraw = true; }
            if (dnCnt) { for (uint8_t i = 0; i < dnCnt; i++) page4Adjust(selectedItem,  1); needPageRedraw = true; }
            if (selP)  { navMode = NAV_ITEM; needPageRedraw = true; }
            break;
    }
#ifdef DEBUG_SERIAL
    if (selP)   Serial.print(F("BTN:SEL "));
    if (upCnt)  { Serial.print(F("BTN:UP x")); Serial.print(upCnt); Serial.print(' '); }
    if (dnCnt)  { Serial.print(F("BTN:DWN x")); Serial.print(dnCnt); Serial.print(' '); }
    Serial.print(F("-> pg=")); Serial.println(currentPage);
#endif
}

void updateDisplayBanners() {
    // Manual override banner (all pages) — stacked just above the fault bar (y=300)
    if (manualOverrideActive) {
        tft.fillRect(0, 260, 480, 20, C_RED);
        tft.setTextColor(C_WHITE); tft.setTextSize(2); tft.setCursor(2, 262);
        tft.print(F("MANUAL OVERRIDE ACTIVE"));
    }
    // Manual heater banner
    if (manualHeaterMode == MHM_SOC_LIM) {
        tft.fillRect(0, 280, 480, 20, C_RED);
        tft.setTextColor(C_WHITE); tft.setTextSize(2); tft.setCursor(2, 282);
        uint8_t soc = lastWPkt.battSocPct;
        char socBuf[28];
        snprintf(socBuf, sizeof(socBuf), "SOC-LIM: %u%% — %s", soc, soc > 50 ? "HEATING" : "DONE");
        tft.print(socBuf);
    } else if (manualHeaterMode == MHM_FORCE_ON) {
        tft.fillRect(0, 280, 480, 20, C_RED);
        tft.setTextColor(C_WHITE); tft.setTextSize(2); tft.setCursor(2, 282);
        tft.print(F("HEATER MANUAL ON — FULL 3kW"));
    }
}

// ============================================================
//  RS485 INTER-CONTROLLER LINK
// ============================================================

uint8_t   txSeqNum       = 0;
uint8_t   missedPackets  = 0;
bool      rs485Fault     = false;
PktReceiver pktRx;

// ============================================================
//  H-SIDE SOLAR PUMP DIRECT DRIVE
// ============================================================

static float calcPred(uint8_t heaterPct, float hotPipeC) {
    // Per-level k and alpha fitted from new_pump SCT data (pump = k / (85-hp)^alpha);
    // alpha isotonic-smoothed across levels so higher heater % never predicts less pump
    static const uint8_t LEVELS[20] = {
          5,  10,  14,  20,  25,  29,  33,  40,  43,  50,
         57,  60,  67,  71,  75,  80,  86,  90,  95, 100
    };
    static const float K_TBL[20] = {
          11.7f,   43.7f,  109.6f,  352.8f,  785.4f,
        1345.3f, 2122.9f, 3968.6f, 4894.7f, 7145.8f,
        9249.8f,10065.2f,11824.9f,12839.2f,13980.7f,
       15805.3f,19139.8f,22614.3f,29520.0f,41692.6f
    };
    static const float A_TBL[20] = {
        0.323f, 0.638f, 0.851f, 1.111f, 1.281f,
        1.390f, 1.477f, 1.586f, 1.618f, 1.668f,
        1.691f, 1.695f, 1.699f, 1.701f, 1.704f,
        1.714f, 1.737f, 1.762f, 1.806f, 1.868f
    };

    uint8_t i = 0;
    while (i < 19 && LEVELS[i] != heaterPct) i++;
    float k     = K_TBL[i];
    float alpha = A_TBL[i];

    float denom = fmaxf(85.0f - fmaxf(hotPipeC, 20.0f), 5.0f);
    return fmaxf(k / powf(denom, alpha), 4.0f);
}

static float calcHPumpDuty() {
    if (heaterHardLockout)                                               return 100.0f;
    if (!heaterRunning || heaterLevelIdx == 0)                           return 0.0f;
    if ((sFault[H_SENSOR_HEATER_OUT] && sFault[H_SENSOR_HEATER_OUT_2])
        || sFault[H_SENSOR_HOT_PIPE])                                    return 0.0f;

    float heaterOutC = getHeaterOutC();
    float hotPipeC   = sTemp[H_SENSOR_HOT_PIPE];

    if (hotPipeC >= 80.0f) return 100.0f;

#ifdef DEBUG_SERIAL
    if (calHtrOverride) {
        float effectivePct = calPumpBase;
        if (heaterOutC >= 88.0f) {
            float t = min((heaterOutC - 88.0f) / 3.0f, 1.0f);
            effectivePct = calPumpBase + 5.0f + t * (95.0f - calPumpBase);
        } else if (heaterOutC >= 80.0f) {
            float t = (heaterOutC - 80.0f) / 8.0f;
            effectivePct = calPumpBase + t * 5.0f;
        }
        effectivePct = max(effectivePct, 0.1f);
        calPumpOnMs  = 200UL;
        calPumpOffMs = (uint32_t)(200.0f * (100.0f - effectivePct) / effectivePct);
        return effectivePct;
    }
#endif

    bool  normalMode = solarTargetMode == SOLAR_TANK_PLUS8 && !sFault[H_SENSOR_TANK_TOP]
                                                            && sTemp[H_SENSOR_TANK_TOP] <= 75.0f;
    float effTarget  = normalMode ? fminf(sTemp[H_SENSOR_TANK_TOP] + 8.0f, 87.0f) : 85.0f;
    float pred = calcPred(heaterLevelPct(), hotPipeC + fmaxf(0.0f, 85.0f - effTarget));
    float duty;

    // Upper pump ceiling at 90°C — scales with hot pipe because that's where the
    // pred formula under-predicts most. 1.3x at cold pipe, up to ~1.54x at 60°C.
    float upperMult = 1.3f + 0.008f * fmaxf(0.0f, hotPipeC - 30.0f);
    float upper     = fminf(pred * upperMult, 100.0f);

    if (normalMode) {
        // Normal mode: target tracks tank top + 8°C, capped at 87°C.
        // Above target: ramp pred→upper over target→90°C, then spike to 100% at 91°C.
        float target    = effTarget;
        float dutyAt90  = upper;
        if (heaterOutC >= 91.0f) {
            duty = 100.0f;
        } else if (heaterOutC >= 90.0f) {
            float t = heaterOutC - 90.0f;
            duty = dutyAt90 + t * (100.0f - dutyAt90);
        } else if (heaterOutC >= target) {
            duty = pred + (heaterOutC - target) * 0.2f;
        } else if (heaterOutC >= target - 7.0f) {
            float t = (heaterOutC - (target - 7.0f)) / 7.0f;
            duty = pred * 0.9f + t * (pred * 0.1f);
        } else {
            duty = pred * 0.9f;
        }
    } else {
        // MAX mode (or tank top fault): fixed 85°C target with pred→upper ramp.
        if (heaterOutC >= 91.0f) {
            duty = 100.0f;
        } else if (heaterOutC >= 90.0f) {
            float t = heaterOutC - 90.0f;
            duty = upper + t * (100.0f - upper);
        } else if (heaterOutC >= 85.0f) {
            float t = (heaterOutC - 85.0f) / 5.0f;
            duty = pred + t * (upper - pred);
        } else if (heaterOutC >= 78.0f) {
            float t = (heaterOutC - 78.0f) / 7.0f;
            duty = pred * 0.9f + t * (pred * 0.1f);
        } else {
            duty = pred * 0.9f;
        }
    }

    if (duty < 4.0f)   duty = 4.0f;
    if (duty > 100.0f) duty = 100.0f;
    return duty;
}

static void updateFlush() {
    if (!heaterRunning) { flushState = FLUSH_IDLE; return; }
    float h1     = sFault[H_SENSOR_HEATER_OUT]   ? NAN : sTemp[H_SENSOR_HEATER_OUT];
    float h2     = sFault[H_SENSOR_HEATER_OUT_2] ? NAN : sTemp[H_SENSOR_HEATER_OUT_2];
    float htrOut = getHeaterOutC();
    bool  cond   = !isnan(h1) && !isnan(h2) && !isnan(htrOut)
                   && htrOut >= 80.0f && (h2 - h1) > 3.5f;
    uint32_t now = millis();
    switch (flushState) {
        case FLUSH_IDLE:
            if (cond) { flushState = FLUSH_ACTIVE; flushTimer = now; }
            break;
        case FLUSH_ACTIVE:
            if (now - flushTimer >= 4000) { flushState = FLUSH_PAUSE; flushTimer = now; }
            break;
        case FLUSH_PAUSE:
            if (now - flushTimer >= 4000) {
                flushState = cond ? FLUSH_ACTIVE : FLUSH_IDLE;
                flushTimer = now;
            }
            break;
    }
}

static void setHPumpDuty(float duty) {
    hPumpDuty = duty;
    if (duty <= 0.0f) { digitalWrite(PIN_SOLAR_PUMP, LOW); hPumpOutputState = false; }
}

static void updateHPump() {
    if (hPumpDuty <= 0.0f) {
        if (hPumpOutputState) { digitalWrite(PIN_SOLAR_PUMP, LOW); hPumpOutputState = false; }
        return;
    }
    if (hPumpDuty >= 100.0f) {
        if (!hPumpOutputState) { digitalWrite(PIN_SOLAR_PUMP, HIGH); hPumpOutputState = true; hPumpOnMs = millis(); }
        return;
    }
    uint32_t onMs, offMs;
#ifdef DEBUG_SERIAL
    if (calHtrOverride) {
        onMs  = calPumpOnMs;
        offMs = calPumpOffMs;
    } else
#endif
    if (hPumpDuty <= 20.0f) {
        onMs  = 400UL;
        offMs = (uint32_t)(400.0f * (100.0f - hPumpDuty) / hPumpDuty);
    } else if (hPumpDuty <= 50.0f) {
        onMs  = (uint32_t)(400.0f + (hPumpDuty - 20.0f) * 400.0f / 30.0f);
        offMs = (uint32_t)((float)onMs * (100.0f - hPumpDuty) / hPumpDuty);
    } else {
        onMs  = 800UL;
        offMs = (uint32_t)(800.0f * (100.0f - hPumpDuty) / hPumpDuty);
    }
    unsigned long now = millis();
    if (hPumpOutputState) {
        if (now - hPumpOnMs >= onMs) { digitalWrite(PIN_SOLAR_PUMP, LOW); hPumpOutputState = false; hPumpOffMs = now; }
    } else {
        if (now - hPumpOffMs >= offMs) { digitalWrite(PIN_SOLAR_PUMP, HIGH); hPumpOutputState = true; hPumpOnMs = now; }
    }
}

void sendHToWPacket(bool timeSyncReq) {
    HToWPacket pkt;
    memset(&pkt, 0, sizeof(pkt));

    // H temperatures
    pkt.tempTankBot    = sFault[H_SENSOR_TANK_BOT]  ? TEMP_FAULT : (int16_t)(sTemp[H_SENSOR_TANK_BOT]  * 10);
    pkt.tempTankMid    = sFault[H_SENSOR_TANK_MID]  ? TEMP_FAULT : (int16_t)(sTemp[H_SENSOR_TANK_MID]  * 10);
    pkt.tempTankTop    = sFault[H_SENSOR_TANK_TOP]  ? TEMP_FAULT : (int16_t)(sTemp[H_SENSOR_TANK_TOP]  * 10);
    pkt.tempHotPipe    = sFault[H_SENSOR_HOT_PIPE]  ? TEMP_FAULT : (int16_t)(sTemp[H_SENSOR_HOT_PIPE]  * 10);
    pkt.tempColdPipe   = sFault[H_SENSOR_COLD_PIPE] ? TEMP_FAULT : (int16_t)(sTemp[H_SENSOR_COLD_PIPE] * 10);
    { float effT = getHeaterOutC(); pkt.tempHeaterOut = isnan(effT) ? TEMP_FAULT : (int16_t)(effT * 10.0f); }

    pkt.heaterPowerPct    = (heaterRunning && heaterLevelIdx > 0) ? max(heaterLevelPct(), (uint8_t)1) : 0;
    pkt.heaterRestricted  = (heaterLevelCap < 20) ? 1 : 0;
    pkt.twoPortHeaterSide = twoPortValve.isOpen ? 1 : 0;
    pkt.botTankOpen       = botTankValve.isOpen ? 1 : 0;

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
    if (simWPumpStop) {
        pkt.calPumpActive   = 1;
        pkt.calSolarTargetC = 0;
    } else {
        pkt.calPumpActive   = (calPumpPhase == CALP_RUNNING) ? 1 : 0;
        pkt.calSolarTargetC = 0;
    }
#endif

    pkt.hPumpDutyPct = (uint8_t)roundf(hPumpDuty);

    uint8_t frame[PKT_MAX_FRAME];
    uint16_t len = pktEncode(frame, sizeof(frame), PKT_DIR_HW, txSeqNum++,
                             &pkt, sizeof(pkt));
    digitalWrite(PIN_RS485_DE_LINK, HIGH);
    Serial1.write(frame, len);
    Serial1.flush();  // ~56ms at 9600 baud
    digitalWrite(PIN_RS485_DE_LINK, LOW);
    pollBtns();
}

unsigned long lastInterCtrlMs    = 0;
unsigned long lastTimeSyncSentMs = 0;

// Non-blocking: drains Serial1 bytes into the packet receiver each call.
// Replies immediately when a complete W packet is decoded.
static void pollRS485() {
    uint8_t  outDir, outSeq;
    uint8_t *payload; uint16_t payLen;

    while (Serial1.available()) {
        if (!pktRx.feed((uint8_t)Serial1.read(), outDir, outSeq, payload, payLen)) continue;
        if (outDir != PKT_DIR_WH || payLen != sizeof(WToHPacket)) continue;
        memcpy(&lastWPkt, payload, sizeof(WToHPacket));
#ifdef DEBUG_SERIAL
        if (simPVExportActive) lastWPkt.pvExportW = simPVExportVal;
#endif
        hasWPkt       = true;
        lastWPktMs    = millis();
        missedPackets = 0;
        if (rs485Fault) { rs485Fault = false; clearFaultH(FAULT_H_RS485_COMMS); }
        accumulateEnergy();

#ifdef DEBUG_SERIAL
        {
            DateTime now = rtc.now();
            char tbuf[10];
            snprintf(tbuf, sizeof(tbuf), "%02u:%02u:%02u",
                     now.hour(), now.minute(), now.second());
            Serial.print(F("["));
            Serial.print(tbuf);
            Serial.print(F("] W pkt | growatt="));
            if (lastWPkt.growattValid) {
                Serial.print(F("OK"));
                Serial.print(F(" pv1="));   Serial.print(lastWPkt.pv1W);
                Serial.print(F("W pv2="));  Serial.print(lastWPkt.pv2W);
                Serial.print(F("W out="));  Serial.print(lastWPkt.pvOutputW);
                Serial.print(F("W load=")); Serial.print(lastWPkt.loadW);
                Serial.print(F("W exp="));  Serial.print(lastWPkt.pvExportW);
                Serial.print(F("W imp="));  Serial.print(lastWPkt.gridImportW);
                Serial.print(F("W batt="));
                Serial.print(lastWPkt.battVoltage_dV / 10);
                Serial.print(F("."));
                Serial.print(abs(lastWPkt.battVoltage_dV % 10));
                Serial.print(F("V soc="));  Serial.print(lastWPkt.battSocPct);
                Serial.print(F("% chg="));  Serial.print(lastWPkt.battChargeW);
                Serial.print(F("W gen="));
                Serial.print(lastWPkt.dailyGenDeciKwh / 10);
                Serial.print(F("."));
                Serial.print(lastWPkt.dailyGenDeciKwh % 10);
                Serial.print(F("kWh"));
            } else {
                Serial.print(F("NO DATA"));
            }
            Serial.print(F(" | htr="));   Serial.print(heaterRunning ? heaterLevelPct() : 0);
            Serial.print(F("% pump="));   Serial.print(lastWPkt.solarPumpDutyPct);
            Serial.println(F("%"));
        }
#endif

        bool wSolarFault = (lastWPkt.wFaultFlags & (FAULT_W_SENSOR_SOLAR_HOT | FAULT_W_SENSOR_SOLAR_COLD)) != 0;
#ifdef DEBUG_SERIAL
        if ((calPumpPhase == CALP_IDLE || calPumpPhase == CALP_DONE) && pumpTestState == PT_IDLE)
#endif
        {
            checkHeaterImportTrip(lastWPkt.gridImportW, lastWPkt.battSocPct, lastWPkt.battChargeW, lastWPkt.growattValid);
            // Winter mode: hold off heater until export has been ≥500W for ≥60s (battery charges to 100% first).
            if (systemMode == MODE_WINTER && !pvExportOverride) {
                heaterLevelIdx = 0; heaterRunning = false;
            } else {
                updateHeaterDuty(lastWPkt.pvExportW, lastWPkt.gridImportW, manualHeaterMode, wSolarFault);
            }
        }

        bool needTS = lastWPkt.requestTimeSync && (millis() - lastTimeSyncSentMs > 5000);
        if (needTS) { lastTimeSyncSentMs = millis(); needTSSinceLastLog = true; }
        sendHToWPacket(needTS);
    }
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
    hDbgPrintTemp(H_SENSOR_HEATER_OUT,   F("htr_out"));
    hDbgPrintTemp(H_SENSOR_HEATER_OUT_2, F("htr_out_2"));
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

static void dbgValveState(HBridgeValve& v, bool openIsHtr = false) {
    if (v.phase == HBP_IDLE) {
        Serial.println(v.isOpen ? (openIsHtr ? F("OPEN(htr)") : F("OPEN")) : (openIsHtr ? F("CLOSED(mid)") : F("CLOSED")));
    } else {
        Serial.print(v.pendingOpen ? (openIsHtr ? F("->OPEN(htr)") : F("->OPEN")) : (openIsHtr ? F("->CLOSED(mid)") : F("->CLOSED")));
        Serial.print(v.phase == HBP_DEAD ? F(" [dead]") : F(" [pulsing]"));
        Serial.println();
    }
}

static void dbgValves() {
    Serial.print(F("  log_cold: ")); dbgValveState(logBurnerCold);
    Serial.print(F("  bot_tank: ")); dbgValveState(botTankValve);
    Serial.print(F("  two_port: ")); dbgValveState(twoPortValve, true);
    if (hasWPkt) {
        Serial.print(F("  W ufh_cold:  ")); Serial.println((lastWPkt.valveStates & VSTATE_UFH_COLD_OPEN)   ? F("OPEN") : F("CLOSED"));
        Serial.print(F("  W solar_cold:")); Serial.println((lastWPkt.valveStates & VSTATE_SOLAR_COLD_OPEN) ? F("OPEN") : F("CLOSED"));
        Serial.print(F("  W vac_iso:   ")); Serial.println((lastWPkt.valveStates & VSTATE_VAC_ISO_OPEN)    ? F("OPEN") : F("CLOSED"));
    }
}

static void dbgFaults() {
    bool any = false;
    #define HF(m,n) if(hasFaultH(m)){Serial.println(F("  " n));any=true;}
    HF(FAULT_H_HEATER_OVERHEAT_SHUT, "HEATER_OVERHEAT_SHUT")
    HF(FAULT_H_HEATER_ELEMENT_FAIL,  "HEATER_ELEMENT_FAIL")
    HF(FAULT_H_RS485_COMMS,          "RS485_COMMS")
    HF(FAULT_H_BUS_VOLTAGE_LOW,      "BUS_VOLTAGE_LOW")
    HF(FAULT_H_GRID_OUTAGE,          "GRID_OUTAGE")
    HF(FAULT_H_HEATER_IMPORT_TRIP,   "HEATER_IMPORT_TRIP")
    HF(FAULT_H_SENSOR_TANK_BOT,      "SENSOR_TANK_BOT")
    HF(FAULT_H_SENSOR_TANK_MID,      "SENSOR_TANK_MID")
    HF(FAULT_H_SENSOR_TANK_TOP,      "SENSOR_TANK_TOP")
    HF(FAULT_H_SENSOR_HOT_PIPE,      "SENSOR_HOT_PIPE")
    HF(FAULT_H_SENSOR_COLD_PIPE,     "SENSOR_COLD_PIPE")
    HF(FAULT_H_SENSOR_HEATER_OUT,    "SENSOR_HEATER_OUT")
    HF(FAULT_H_SENSOR_HEATER_OUT_2,  "SENSOR_HEATER_OUT_2")
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
    Serial.print(heaterLevelPct()); Serial.println(F("%"));
    Serial.print(F("  htr_lockout:  ")); Serial.println(heaterHardLockout  ? F("YES")    : F("no"));
    Serial.print(F("  rs485:        ")); Serial.println(rs485Fault         ? F("FAULT")  : F("ok"));
    Serial.print(F("  rtc_valid:    ")); Serial.println(rtcValid            ? F("yes")    : F("no"));
    Serial.print(F("  sd:           ")); Serial.println(sdAvailable         ? F("ok")     : F("no"));
}

static void dbgHeater() {
    Serial.print(F("  running:   ")); Serial.println(heaterRunning     ? F("yes")  : F("no"));
    Serial.print(F("  duty:      ")); Serial.print(heaterLevelPct());   Serial.println(F("%"));
    Serial.print(F("  power_est: ")); Serial.print(heaterLevelPct10() * 3); Serial.println(F("W"));
    Serial.print(F("  lockout:   ")); Serial.println(heaterHardLockout ? F("YES")  : F("no"));
    Serial.print(F("  imp_trip:  ")); Serial.println(importTripActive  ? F("YES")  : F("no"));
    Serial.print(F("  grid:      ")); Serial.println(gridPresent       ? F("ok")   : F("OUTAGE"));
    Serial.print(F("  zc_count:  ")); Serial.println(zcFireCount);
    Serial.print(F("  zc_age_ms: ")); Serial.println((micros() - lastZCMicros) / 1000UL);
    { uint8_t hi = 0, lo = 0; for (uint8_t i = 0; i < 200; i++) { if (digitalRead(PIN_ZERO_CROSSING)) hi++; else lo++; delayMicroseconds(100); }
      Serial.print(F("  zc_pin:    ")); Serial.print(hi); Serial.print(F("H/")); Serial.print(lo); Serial.println(F("L (200 samples@100us)")); }
    Serial.print(F("  enabled:   ")); Serial.println(HEATER_ENABLED    ? F("yes")  : F("NO (commissioning flag)"));
    if (simPVExportActive) { Serial.print(F("  SIM pv_export=")); Serial.println(simPVExportVal); }
    if (simHeaterActive)   { Serial.print(F("  SIM heater_pct=")); Serial.println(simHeaterVal); }
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
    else if (!strcmp_P(key, PSTR("htr_out")))    si = H_SENSOR_HEATER_OUT;
    else if (!strcmp_P(key, PSTR("htr_out_2"))) si = H_SENSOR_HEATER_OUT_2;

    if (si < H_NUM_SENSORS) {
        if (fval >= 999.0f) { hSimulate[si] = false; Serial.println(F("cleared")); }
        else { hSimulate[si] = true; hSim[si] = fval; Serial.println(F("ok")); }
        return;
    }
    if (!strcmp_P(key, PSTR("log_burner")))        { simLogBurnerActive = true;  simLogBurnerVal = (fval != 0); Serial.println(F("ok")); return; }
    if (!strcmp_P(key, PSTR("log_burner_clear")))  { simLogBurnerActive = false; Serial.println(F("cleared")); return; }
    if (!strcmp_P(key, PSTR("pv_export")))         { simPVExportActive  = true;  simPVExportVal  = (int16_t)fval; Serial.println(F("ok")); return; }
    if (!strcmp_P(key, PSTR("pv_export_clear")))   { simPVExportActive  = false; Serial.println(F("cleared")); return; }
    if (!strcmp_P(key, PSTR("heater_pct")))        { simHeaterActive    = true;  simHeaterVal    = (uint8_t)constrain((int)fval, 0, 100); Serial.println(F("ok")); return; }
    if (!strcmp_P(key, PSTR("heater_pct_clear")))  { simHeaterActive    = false; Serial.println(F("cleared")); return; }
    if (!strcmp_P(key, PSTR("pump_spd")))          { simHPumpSpdActive  = true;  simHPumpSpdVal  = (uint8_t)constrain((int)fval, 0, 100); Serial.println(F("ok")); return; }
    if (!strcmp_P(key, PSTR("pump_spd_clear")))    { simHPumpSpdActive  = false; Serial.println(F("cleared")); return; }
    if (!strcmp_P(key, PSTR("rtc"))) {
        // val format: YYYY-MM-DDTHH:MM:SS
        char* p = val;
        uint16_t yr = (uint16_t)atoi(p);
        p = strchr(p, '-'); if (!p) { Serial.println(F("fmt: YYYY-MM-DDTHH:MM:SS")); return; }
        uint8_t mo = (uint8_t)atoi(++p);
        p = strchr(p, '-'); if (!p) { Serial.println(F("fmt: YYYY-MM-DDTHH:MM:SS")); return; }
        uint8_t dy = (uint8_t)atoi(++p);
        p = strchr(p, 'T'); if (!p) { Serial.println(F("fmt: YYYY-MM-DDTHH:MM:SS")); return; }
        uint8_t hr = (uint8_t)atoi(++p);
        p = strchr(p, ':'); if (!p) { Serial.println(F("fmt: YYYY-MM-DDTHH:MM:SS")); return; }
        uint8_t mn = (uint8_t)atoi(++p);
        p = strchr(p, ':'); if (!p) { Serial.println(F("fmt: YYYY-MM-DDTHH:MM:SS")); return; }
        uint8_t sc = (uint8_t)atoi(++p);
        rtc.adjust(DateTime(yr, mo, dy, hr, mn, sc));
        rtcNow = rtc.now();
        Serial.println(F("ok"));
        return;
    }
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

static void updateCalPump() {
    if (calPumpPhase != CALP_RUNNING) return;

    unsigned long now     = millis();
    bool htrFault = sFault[H_SENSOR_HEATER_OUT] && sFault[H_SENSOR_HEATER_OUT_2];
    bool hotFault = sFault[H_SENSOR_HOT_PIPE];
    float htrOut  = htrFault ? NAN : getHeaterOutC();
    float hotPipe = hotFault ? NAN : sTemp[H_SENSOR_HOT_PIPE];

    if (htrFault) {
        if (!calSensorFault) {
            calSensorFault  = true;
            calPhaseMs      = now;
            heaterRunning   = false;
            heaterLevelIdx  = 0;
            Serial.println(F("CAL: htr_out sensor fault — heater off, restarting step when recovered"));
        }
        calHtrOverride = true;
        return;
    }
    if (calSensorFault) {
        calSensorFault = false;
        calPhaseMs     = now;
        Serial.println(F("CAL: htr_out recovered — restarting step"));
    }

    calHtrOverride  = true;
    heaterRunning   = true;
    heaterLevelIdx  = pctToLevel(calHeaterPct);

    if (htrOut >= 91.0f) {
        if (!calCooling) {
            calCooling = true;
            Serial.println(F("CAL: htr_out 91C — cooling at 100% pump"));
        }
        return;
    }
    if (calCooling) {
        calCooling = false;
        calPhaseMs = now;
        Serial.println(F("CAL: cooled below 91C — restarting step"));
    }

    unsigned long stepDuration = (calStepNum == 0) ? 300000UL : 180000UL;

    if (now - calPhaseMs >= stepDuration) {
        // Record end-of-step data
        Serial.print(F("CAL,"));
        Serial.print(calHeaterPct); Serial.print(',');
        if (htrFault) Serial.print(F("NaN")); else Serial.print(htrOut, 1);
        Serial.print(',');
        if (hotFault) Serial.print(F("NaN")); else Serial.print(hotPipe, 1);
        Serial.print(',');
        Serial.println(hPumpDuty, 1);

        if (calHeaterPct >= 100) {
            Serial.println(F("CAL: complete"));
            calPumpPhase    = CALP_DONE;
            calHtrOverride  = false;
            heaterRunning   = false;
            heaterLevelIdx  = 0;
        } else {
            calPumpBase    = min(hPumpDuty, 95.0f);
            calHeaterPct   = min((uint8_t)(calHeaterPct + 5), (uint8_t)100);
            calStepNum++;
            calPhaseMs     = now;
            Serial.print(F("CAL: step ")); Serial.print(calStepNum);
            Serial.print(F(" — heater ")); Serial.print(calHeaterPct);
            Serial.print(F("%, pump base ")); Serial.print(calPumpBase, 2); Serial.println('%');
        }
    }
}

static void dbgCalPump() {
    if (!HEATER_ENABLED) {
        Serial.println(F("cal_pump: HEATER_ENABLED is false — set true in firmware before calibrating"));
        return;
    }
    if (calPumpPhase == CALP_RUNNING) {
        Serial.println(F("cal_pump: already running — use cal_abort to stop"));
        return;
    }
    calPumpPhase    = CALP_RUNNING;
    calHeaterPct    = 15;
    calPumpBase     = 4.0f;
    calStepNum      = 0;
    calHtrOverride  = true;
    calCooling      = false;
    calSensorFault  = false;
    calPhaseMs      = millis();
    Serial.println(F("CAL: started — 15% heater, pump base 4.0%"));
    Serial.println(F("CAL_HDR: heater_pct,htr_out_C,hot_pipe_C,pump_pct"));
}

static void dbgCalAbort() {
    calPumpPhase   = CALP_IDLE;
    calHtrOverride = false;
    heaterRunning  = false;
    heaterLevelIdx = 0;
    PORTA &= ~(1 << PA5);
    Serial.println(F("CAL: aborted — heater off, normal control restored"));
}

// ── Pump formula test: runs normal calcHPumpDuty, steps 5–100% ──

static void dbgPumpTestAbort();  // forward declaration — defined below

static void pumpTestStartStep() {
    uint32_t now = millis();
    pumpTestPower = pumpTestStress
        ? pgm_read_byte(&STRESS_TEST_POWERS[pumpTestStepIdx])
        : pgm_read_byte(&PUMP_TEST_POWERS[pumpTestStepIdx]);
    logBurnerCold.request(false);
    botTankValve.request(true);
    twoPortValve.request(true);
    heaterRunning        = true;
    heaterLevelIdx       = pctToLevel(pumpTestPower);
    pumpTestHeaterActive = true;
    pumpTestState       = PT_HEATING;
    pumpTestLastPrint = now;
    pumpTestLastSec  = now;
    pumpTestWinStart = now;
    pumpTestMaxPump  = 0.0f;
    { float _t = getHeaterOutC(); float hOut = isnan(_t) ? 0.0f : _t;
      pumpTestWinMin = pumpTestWinMax = hOut; }
    if (pumpTestStress) {
        Serial.print(F("STRESS_TEST: step ")); Serial.print(pumpTestPower); Serial.println('%');
    }
}

static void pumpTestStartCooldown() {
    heaterRunning        = false;
    heaterLevelIdx       = 0;
    pumpTestHeaterActive = false;
    pumpTestState        = PT_COOLDOWN;
    pumpTestCoolStart = millis();
    Serial.println(F("STRESS_TEST: cooling down..."));
}

static void updatePumpTest() {
    if (pumpTestState == PT_IDLE) return;
    uint32_t now = millis();

    if (pumpTestState == PT_COOLDOWN) {
        heaterRunning  = false;
        heaterLevelIdx = 0;
        float _ht = getHeaterOutC(); float hOut = isnan(_ht) ? 99.0f : _ht;
        bool cooled  = (hOut < 60.0f);
        bool timeout = (now - pumpTestCoolStart >= 120000);
        if (cooled || timeout) {
            Serial.print(F("STRESS_TEST: cooldown done htr_out="));
            Serial.println(hOut, 1);
            if (pumpTestStepIdx >= STRESS_TEST_COUNT) {
                Serial.println(F("STRESS_TEST: complete"));
                dbgPumpTestAbort();
                return;
            }
            pumpTestStartStep();
        }
        return;
    }

    if (pumpTestState == PT_HEATING) {
        heaterRunning  = true;
        heaterLevelIdx = pctToLevel(pumpTestPower);

        if (hPumpDuty > pumpTestMaxPump) pumpTestMaxPump = hPumpDuty;

        if (now - pumpTestLastPrint >= 2000) {
            pumpTestLastPrint = now;
            float hotPipe = sFault[H_SENSOR_HOT_PIPE]   ? NAN : sTemp[H_SENSOR_HOT_PIPE];
            float htrOut  = getHeaterOutC();
            Serial.print(heaterLevelPct());     Serial.print(',');
            if (isnan(hotPipe)) Serial.print(F("NaN")); else Serial.print(hotPipe, 1);
            Serial.print(',');
            if (isnan(htrOut))  Serial.print(F("NaN")); else Serial.print(htrOut, 1);
            Serial.print(',');
            Serial.println(hPumpDuty, 1);
        }

        if (now - pumpTestLastSec >= 1000) {
            pumpTestLastSec = now;
            float _ho = getHeaterOutC();
            if (isnan(_ho)) {
                pumpTestWinStart = now;
            } else {
                float hOut = _ho;
                if (hOut < pumpTestWinMin) pumpTestWinMin = hOut;
                if (hOut > pumpTestWinMax) pumpTestWinMax = hOut;
                if (pumpTestWinMax - pumpTestWinMin >= 0.4f) {
                    pumpTestWinMin = pumpTestWinMax = hOut;
                    pumpTestWinStart = now;
                }
            }
        }

        if (now - pumpTestWinStart >= 30000) {
            float hotPipe = sFault[H_SENSOR_HOT_PIPE] ? NAN : sTemp[H_SENSOR_HOT_PIPE];
            float htrOut  = getHeaterOutC();
            if (pumpTestStress) {
                Serial.print(F("SSTABLE,"));
                Serial.print(pumpTestPower); Serial.print(',');
                if (isnan(hotPipe)) Serial.print(F("NaN")); else Serial.print(hotPipe, 1);
                Serial.print(',');
                if (isnan(htrOut))  Serial.print(F("NaN")); else Serial.print(htrOut, 1);
                Serial.print(',');
                Serial.print(hPumpDuty, 1);
                Serial.print(',');
                Serial.println(pumpTestMaxPump, 1);
                pumpTestStepIdx++;
                if (pumpTestStepIdx >= STRESS_TEST_COUNT) {
                    Serial.println(F("STRESS_TEST: complete"));
                    dbgPumpTestAbort();
                } else {
                    pumpTestStartStep();
                }
            } else {
                Serial.print(F("STABLE,"));
                Serial.print(pumpTestPower); Serial.print(',');
                if (isnan(hotPipe)) Serial.print(F("NaN")); else Serial.print(hotPipe, 1);
                Serial.print(',');
                if (isnan(htrOut))  Serial.print(F("NaN")); else Serial.print(htrOut, 1);
                Serial.print(',');
                Serial.println(hPumpDuty, 1);
                pumpTestStepIdx = (pumpTestStepIdx + 1) % PUMP_TEST_COUNT;
                Serial.print(F("PUMP_TEST: step ")); Serial.print(pgm_read_byte(&PUMP_TEST_POWERS[pumpTestStepIdx])); Serial.println('%');
                pumpTestStartStep();
            }
        }
    }
}

static void dbgPumpTest() {
    if (!HEATER_ENABLED) { Serial.println(F("pump_test: HEATER_ENABLED is false")); return; }
    if (pumpTestState != PT_IDLE) { Serial.println(F("pump_test: already running — use pump_test_abort")); return; }
    pumpTestStress  = false;
    pumpTestStepIdx = 0;
    pumpTestStartStep();
    Serial.println(F("PUMP_TEST: started — spread heater powers, loops continuously"));
    Serial.println(F("FMT: pwr%,hot_pipe,htr_out,pump%"));
}

static void dbgPumpTestAbort() {
    pumpTestState        = PT_IDLE;
    pumpTestStress       = false;
    pumpTestHeaterActive = false;
    heaterRunning        = false;
    heaterLevelIdx       = 0;
    PORTA &= ~(1 << PA5);
    Serial.println(F("PUMP_TEST: aborted"));
}

static void dbgStressTest() {
    if (!HEATER_ENABLED) { Serial.println(F("stress_test: HEATER_ENABLED is false")); return; }
    if (pumpTestState != PT_IDLE) { Serial.println(F("stress_test: already running — use pump_test_abort")); return; }
    pumpTestStress  = true;
    pumpTestStepIdx = 0;
    pumpTestStartStep();
    Serial.println(F("STRESS_TEST: started — 9 lowPow levels, 30s/0.4C stable window"));
    Serial.println(F("FMT: pwr%,hot_pipe,htr_out,pump%  |  SSTABLE adds max_pump%"));
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
        Serial.println(F("  sensors: tank_bot tank_mid tank_top hot_pipe cold_pipe htr_out htr_out_2"));
        Serial.println(F("set rtc YYYY-MM-DDTHH:MM:SS"));
        Serial.println(F("set log_burner|pv_export|batt_soc|heater_pct <val>"));
        Serial.println(F("set log_burner_clear|pv_export_clear|batt_soc_clear|heater_pct_clear 0"));
        Serial.println(F("cal_pump  (start pump cal sequence)  cal_abort"));
        Serial.println(F("pump_test  stress_test  pump_test_abort"));
        Serial.println(F("w_pump_stop  w_pump_run"));
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
    else if (!strcmp_P(cmd, PSTR("cal_pump")))        dbgCalPump();
    else if (!strcmp_P(cmd, PSTR("cal_abort")))       dbgCalAbort();
    else if (!strcmp_P(cmd, PSTR("pump_test")))       dbgPumpTest();
    else if (!strcmp_P(cmd, PSTR("stress_test")))     dbgStressTest();
    else if (!strcmp_P(cmd, PSTR("pump_test_abort"))) dbgPumpTestAbort();
    else if (!strcmp_P(cmd, PSTR("w_pump_stop"))) { simWPumpStop = true;  Serial.println(F("W pump stop sent")); }
    else if (!strcmp_P(cmd, PSTR("w_pump_run")))  { simWPumpStop = false; Serial.println(F("W pump run restored")); }
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
    // Close all 3 tank valves with 7s pulses (sequential per spec).
    // Assume worst case: valve may be physically open, so force isOpen=true
    // before requesting close so request() doesn't return early.
    logBurnerCold.isOpen = true;
    logBurnerCold.request(false);
    unsigned long t = millis();
    while (millis() - t < 7200) { logBurnerCold.update(); wdt_reset(); }

    botTankValve.isOpen = true;
    botTankValve.request(false);
    t = millis();
    while (millis() - t < 7200) { botTankValve.update(); wdt_reset(); }

    twoPortValve.request(true);   // default resting position: heater cold side / top-of-tank
    t = millis();
    while (millis() - t < 7200) { twoPortValve.update(); wdt_reset(); }

    // Ensure SSR is off
    PORTA &= ~(1 << PA5);
    digitalWrite(PIN_PSU_12V, RELAY_OFF);
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
    wdt_disable();
#ifdef DEBUG_SERIAL
    Serial.begin(115200);
#endif

    // Output pins — set register before pinMode to avoid glitch on active-LOW relay board
    digitalWrite(PIN_LOG_COLD_OPEN,  RELAY_OFF); pinMode(PIN_LOG_COLD_OPEN,  OUTPUT);
    digitalWrite(PIN_LOG_COLD_CLOSE, RELAY_OFF); pinMode(PIN_LOG_COLD_CLOSE, OUTPUT);
    digitalWrite(PIN_BOT_TANK_OPEN,  RELAY_OFF); pinMode(PIN_BOT_TANK_OPEN,  OUTPUT);
    digitalWrite(PIN_BOT_TANK_CLOSE, RELAY_OFF); pinMode(PIN_BOT_TANK_CLOSE, OUTPUT);
    digitalWrite(PIN_TWO_PORT_OPEN,  RELAY_OFF); pinMode(PIN_TWO_PORT_OPEN,  OUTPUT);
    digitalWrite(PIN_HEATER_SSR,     LOW);        pinMode(PIN_HEATER_SSR,     OUTPUT);
    digitalWrite(PIN_TWO_PORT_CLOSE, RELAY_OFF); pinMode(PIN_TWO_PORT_CLOSE, OUTPUT);
    digitalWrite(PIN_PSU_12V,        RELAY_OFF); pinMode(PIN_PSU_12V,        OUTPUT);
    digitalWrite(PIN_RS485_DE_LINK,  LOW);        pinMode(PIN_RS485_DE_LINK,  OUTPUT);
    digitalWrite(PIN_SOLAR_PUMP,     LOW);        pinMode(PIN_SOLAR_PUMP,     OUTPUT);
    // D53 is the Mega hardware SS pin — must be driven HIGH as output before SPI init
    // or the SPI peripheral drops into slave mode, causing SD.begin() to fail.
    // TFT library configures this pin during tft.init(), which runs after SD init.
    digitalWrite(PIN_DISPLAY_CS,     HIGH);       pinMode(PIN_DISPLAY_CS,     OUTPUT);
    pinMode(PIN_DISPLAY_BL,     OUTPUT); setBacklight(displayBrightness);

    // Input pins
    pinMode(PIN_ZERO_CROSSING,  INPUT);
    pinMode(PIN_LOG_BURNER_MOD, INPUT);
    pinMode(PIN_BTN_SELECT,     INPUT_PULLUP);
    pinMode(PIN_BTN_UP,         INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN,       INPUT_PULLUP);
    pinMode(PIN_BUS_VOLTAGE,    INPUT);

    // Buttons
    btnSelect.pin = PIN_BTN_SELECT; btnSelect.state = false; btnSelect.prevState = false; btnSelect.pendingCount = 0;
    btnUp.pin     = PIN_BTN_UP;     btnUp.state     = false; btnUp.prevState     = false; btnUp.pendingCount     = 0;
    btnDown.pin   = PIN_BTN_DOWN;   btnDown.state   = false; btnDown.prevState   = false; btnDown.pendingCount   = 0;

    // Timer3 CTC at 1kHz — drives TIMER3_COMPA ISR for button polling
    TCCR3A = 0;
    TCCR3B = (1 << WGM32) | (1 << CS31);  // CTC mode, prescaler /8 → 2MHz clock
    OCR3A  = 1999;                          // 2MHz / 2000 = 1kHz
    TIMSK3 = (1 << OCIE3A);

    // Timer1 CTC at 50ms — drives TIMER1_COMPA ISR for H pump timing
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS12);  // CTC mode, prescaler /256 → 62.5kHz clock
    OCR1A  = 3124;                          // 62500 / 3125 = 20Hz = 50ms
    TIMSK1 = (1 << OCIE1A);

    // RS485 UART
    Serial1.begin(9600);

    // DS18B20
    sensors.begin();
    for (uint8_t i = 0; i < H_NUM_SENSORS; i++) {
        sTemp[i] = NAN; sFault[i] = false; sRawFault[i] = false; sFailCount[i] = 0; sGoodCount[i] = 0;
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

    // SD card (must init before TFT — ILI9488 corrupts SPI bus state)
    initSD();

    // TFT display
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(C_BLACK);
    tft.setTextColor(C_WHITE); tft.setTextSize(2);
    tft.setCursor(10, 140); tft.print(F("H Controller Init..."));

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
    pollRS485();  // drain Serial1 immediately — display/SD later in loop can block 500ms+
    updateHPump();
#ifdef DEBUG_SERIAL
    processSerialInput();
#endif
    unsigned long now = millis();

    // Sensors
    if (!convStarted) startConversion();
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
                lastGridLossMs = millis();  // start outage timer
            } else {
                clearFaultH(FAULT_H_GRID_OUTAGE);
                gridOutageFault = false;
                lastGridLossMs  = 0;
            }
        }
        // Fault set only after 60s sustained outage — ignores brief glitches.
        // Heater keeps running through the debounce window; only cut once the fault latches.
        if (!gridPresent && !gridOutageFault && millis() - lastGridLossMs >= 60000UL) {
            gridOutageFault = true;
            setFaultH(FAULT_H_GRID_OUTAGE);
            heaterLevelIdx = 0;
            PORTA &= ~(1 << PA5);
        }
    }

    // Valve state machines
    logBurnerCold.update();
    botTankValve.update();
    twoPortValve.update();

    // Heater fault checks
    checkHeaterFaults();

    // H-side solar pump direct drive
    updateFlush();
#ifdef DEBUG_SERIAL
    updatePumpTest();
    if (pumpTestState == PT_COOLDOWN)    setHPumpDuty(10.0f);
    else if (flushState == FLUSH_ACTIVE) setHPumpDuty(100.0f);
    else if (simHPumpSpdActive)          setHPumpDuty(simHPumpSpdVal);
    else
#else
    if (flushState == FLUSH_ACTIVE)      setHPumpDuty(100.0f);
    else
#endif
    setHPumpDuty(calcHPumpDuty());
    updateHPump();

    // 5am trigger and heating session
    checkMorningTrigger();
    checkHotTankProtection();
    updateHeatSourceSelection();
    updatePVExportOverride();
    updateSolarValves();

    // Fault history update — run for H-side faults even without a W packet
    {
        static uint32_t prevWF = 0;
        static uint32_t prevHF = 0;
        uint32_t curWF = hasWPkt ? lastWPkt.wFaultFlags : 0;
        if (curWF != prevWF || hFaultFlags != prevHF) {
            faultLogUpdate(curWF, hFaultFlags);
            prevWF = curWF;
            prevHF = hFaultFlags;
        }
    }

    // EEPROM save (30s after last change)
    saveSettingsIfDue();

    // SD safe remove: check for reinsert
    checkSDReinsert();

    // Buttons
#ifdef DEBUG_SERIAL
    {
        static bool prevRaw[3] = {};
        const uint8_t bPins[3] = { PIN_BTN_SELECT, PIN_BTN_UP, PIN_BTN_DOWN };
        const char    bName[3] = { 'S', 'U', 'D' };
        for (uint8_t i = 0; i < 3; i++) {
            bool r = (digitalRead(bPins[i]) == LOW);
            if (r != prevRaw[i]) {
                prevRaw[i] = r;
                Serial.print(millis()); Serial.print(r ? F(" DN:") : F(" UP:")); Serial.println(bName[i]);
            }
        }
    }
#endif
    handleButtons();

    // Clear action flash and force a page redraw once the 300ms period expires
    if (actionFlashEndMs != 0 && millis() >= actionFlashEndMs) {
        actionFlashEndMs = 0;
        if (currentPage == 4) needPageRedraw = true;
    }

    // Inter-controller RS485: reply before any display draw — TFT redraws can take
    // 100-300ms and W's receive window is only 150ms, so comms must come first.
    pollRS485();
    updateHPump();

    // Flush display after button events (draw happens after RS485 reply)
    if (displayOn && (needFullRedraw || needPageRedraw)) {
        updateHPump();
        if (needFullRedraw) {
            drawFullPage();
        } else {
            switch (currentPage) {
                case 4: drawPage4(); break;
                case 5: drawPage5(); break;
            }
            needPageRedraw = false;
            lastDisplayRefreshMs = now;
        }
        updateHPump();
    }

    // Miss counting and WDT reset on the 250ms heartbeat
    if (now - lastInterCtrlMs >= INTER_CTRL_POLL_MS) {
        lastInterCtrlMs = now;
        missedPackets++;
        if (!rs485Fault && missedPackets >= COMMS_FAULT_THRESHOLD) {
            rs485Fault = true; setFaultH(FAULT_H_RS485_COMMS);
        }
        wdt_reset();
    }
#ifdef DEBUG_SERIAL
    updateCalPump();
#endif

    // SD logging (every 5s)
    if (hasWPkt && now - lastLogMs >= 5000) {
        lastLogMs = now;
        updateHPump();
        logDataRow();
        pollRS485();  // reply to any packet that arrived during SD write
        updateHPump();
    }

    // Daily energy kWh totals (every 5 min)
    if (hasWPkt && now - lastEnergyLogMs >= ENERGY_LOG_INTERVAL_MS) {
        lastEnergyLogMs = now;
        updateHPump();
        logEnergyRow();
        pollRS485();
        updateHPump();
    }

    // Display refresh
    if (!displayOn) {
        // Wake only on NEW faults that appeared after the display went to sleep
        uint32_t curFaults = hFaultFlags | (hasWPkt ? lastWPkt.wFaultFlags : 0);
        if (curFaults & ~faultFlagsAtSleep) wakeDisplay();
    } else {
        unsigned long msSinceBtn = millis() - lastButtonMs;
        if (msSinceBtn >= BACKLIGHT_SLEEP_MS) {
            displayOn      = false;
            setBacklight(0);
            faultFlagsAtSleep = hFaultFlags | (hasWPkt ? lastWPkt.wFaultFlags : 0);
            currentPage    = 1;
            navMode        = NAV_PAGE;
            needFullRedraw = true;
        }
    }

    if (displayOn) {
        updateHPump();
        if (needFullRedraw) {
            drawFullPage();
        } else if (needPageRedraw) {
            switch (currentPage) {
                case 4: drawPage4(); break;
                case 5: drawPage5(); break;
            }
            needPageRedraw = false;
            lastDisplayRefreshMs = now;
        } else if (now - lastDisplayRefreshMs >= 500) {
            lastDisplayRefreshMs = now;
            drawStatusBar();
            pollRS485();
            drawFaultBar(hasWPkt ? lastWPkt.wFaultFlags : 0, hFaultFlags);
            pollRS485();
            // Partial refresh of current page data (quick update of values only)
            switch (currentPage) {
                case 1: drawPage1(hasWPkt ? lastWPkt.wFaultFlags : 0); break;
                case 2: drawPage2(); break;
                case 3: /* fault history: redraw on fault change only */ break;
                case 4: drawPage4(); break;
                case 5: drawPage5(); break;
            }
            updateHPump();
            if (manualOverrideActive || manualHeaterMode != MHM_OFF) {
                updateDisplayBanners();
            }
        }
        updateHPump();
    }
    pollRS485();  // catch any packet that arrived during display draws
}
