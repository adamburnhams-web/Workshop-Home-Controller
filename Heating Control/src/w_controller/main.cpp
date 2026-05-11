// ============================================================
//  W Controller — Workshop Mega 2560
//  Controls: solar thermal, UFH pump/valves, vacuum system,
//            door lock, window winch, PC fans, external lights,
//            wall fan, Growatt Modbus, inter-controller RS485
// ============================================================

#include <Arduino.h>
#include <avr/wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_INA219.h>
#include <ModbusMaster.h>
#include <EEPROM.h>

#include "shared_types.h"
#include "rs485_packet.h"

#define DEBUG_SERIAL  // remove this line to strip all debug code from the build

// ============================================================
//  PIN DEFINITIONS  (W controller)
// ============================================================

// Digital inputs
#define PIN_PIR              2   // INT4 — 15VDC output via 10k/4.7k divider
#define PIN_DOOR_HANDLE      3   // INT5 — dry contact, pull-up, closes when pushed
#define PIN_DOOR_REED        4   // dry contact, pull-up; LOW=door open, HIGH=closed
#define PIN_WINCH_REED_OPEN  5   // pull-up; LOW=fully open
#define PIN_WINCH_REED_CLOSE 6   // pull-up; HIGH=fully closed (switch open when closed)
#define PIN_WINCH_REED_LOCK  7   // pull-up; HIGH=manual lock engaged (switch open when locked)
#define PIN_WINCH_SAFETY     8   // pull-up; LOW=over-open safety limit
#define PIN_VAC_SENSOR       9   // pull-up; LOW=full vacuum, HIGH=low vacuum
#define PIN_UNLOCK_BTN      10   // 15VDC via 10k/4.7k divider
#define PIN_LIGHT_BTN       11   // 15VDC via 10k/4.7k divider
#define PIN_MANUAL_RELOCK   12   // dry contact, pull-up; LOW=suppression ON
#define PIN_ONE_WIRE        13   // DS18B20 1-Wire bus (6 sensors)

// Fan tachometers on A8/A9 (PCINT16/17)
#define PIN_FAN1_TACH       A8   // PCINT16 (PK0) — open collector, 10k pull-up
#define PIN_FAN2_TACH       A9   // PCINT17 (PK1) — open collector, 10k pull-up
#define PIN_FAN_BTN         43   // dry contact, pull-up; press = 8hr full-speed timer
#define PIN_WIN_OPEN_BTN    38   // dry contact, pull-up; press = open window
#define PIN_WIN_CLOSE_BTN   39   // dry contact, pull-up; press = close window

// Digital outputs — relays and actuators
// Even board (D22–D36): D22 D24 D26 D28 D30 D32 D34 D36
// Odd board  (D23–D37): D25 D27 D29 D31 D33 D35 D37  (D23 spare)
#define PIN_UFH_COLD_DIR    22   // even ch1: SPDT direction relay, NO=open, NC=close
// D23: spare (odd ch1)
#define PIN_SOLAR_COLD_DIR  24   // even ch2: SPDT direction relay
#define PIN_EXT_LIGHTS      25   // odd  ch2: external LED lights
#define PIN_ALARM_SOUNDER   26   // even ch3: alarm sounder at W
#define PIN_WALL_FAN        27   // odd  ch3: wall axial fan (BES speed controller)
#define PIN_FAN_FLAP        28   // even ch4: fan flap actuator (open before fan ON)
#define PIN_WINCH_POWER     29   // odd  ch4: winch power (with direction ch5 or ch6)
#define PIN_DOOR_LOCK_A     30   // even ch5: door lock H-bridge relay 1
#define PIN_WINCH_DIR_OPEN  31   // odd  ch5: winch direction open (with power ch4)
#define PIN_DOOR_LOCK_B     32   // even ch6: door lock H-bridge relay 2
#define PIN_WINCH_DIR_CLOSE 33   // odd  ch6: winch direction close (with power ch4)
#define PIN_VAC_ISO_CLOSE   34   // even ch7: vacuum isolation valve H-bridge close relay
#define PIN_VAC_PUMP        35   // odd  ch7: 230VAC vacuum pump
#define PIN_VAC_ISO_OPEN    36   // even ch8: vacuum isolation valve H-bridge open relay
#define PIN_UFH_PUMP        37   // odd  ch8: UFH central heating pump
// 4th relay board (D41, D42, D48–D53)
#define PIN_BUZZER_SIGNAL   41   // board4 ch1: pulls 2.5mm T&E earth → buzzer at H
#define PIN_HEN_DOOR_OPEN   42   // board4 ch2: hen house door open
#define PIN_HEN_DOOR_CLOSE  48   // board4 ch3: hen house door close
#define PIN_MIDPOINT_LED    49   // board4 ch4: 12VDC status LED relay
// D50–D53: spare (board4 ch5–ch8)

#define PIN_RS485_DE_LINK   46   // MAX485 DE/RE for inter-controller link (HIGH=TX)
#define PIN_RS485_DE_GROWATT 47  // MAX485 DE/RE for Growatt Modbus (HIGH=TX)
#define PIN_SOLAR_PUMP      44   // MOSFET gate via 120Ω: solar pump clocking output
#define PIN_FAN_PWM         45   // Timer5 OC5B 25kHz PWM: PC fan MOSFET gate

// UART assignments
// UART1 (Serial1 D18/D19): inter-controller RS485 link
// UART2 (Serial2 D16/D17): Growatt Modbus RS485

// ============================================================
//  COMMISSIONING CONSTANTS — must be verified on-site
// ============================================================

// DS18B20 64-bit addresses — scan with address-scan sketch and fill in
// Sensor order: solar_hot, solar_cold, UFH_supply, UFH_post_TMV, workshop_air, outside_air
static const uint8_t DS18B20_ADDRS[6][8] = {
    { 0x28, 0xFE, 0xB5, 0x14, 0x00, 0x00, 0x00, 0xD4 }, // solar hot      (sensor 1)
    { 0x28, 0x5C, 0x8E, 0x12, 0x00, 0x00, 0x00, 0x1B }, // solar cold     (sensor 2)
    { 0x28, 0xE1, 0xEB, 0x14, 0x00, 0x00, 0x00, 0x9C }, // UFH supply     (sensor 3)
    { 0x28, 0xD1, 0x67, 0x15, 0x00, 0x00, 0x00, 0x14 }, // UFH post TMV   (sensor 4)
    { 0x28, 0x45, 0x77, 0x12, 0x00, 0x00, 0x00, 0x83 }, // workshop air   (sensor 5)
    { 0x28, 0xC9, 0x83, 0x12, 0x00, 0x00, 0x00, 0xAD }, // outside air    (sensor 6)
};
#define SENSOR_SOLAR_HOT    0
#define SENSOR_SOLAR_COLD   1
#define SENSOR_UFH_SUPPLY   2
#define SENSOR_UFH_POST_TMV 3
#define SENSOR_WORKSHOP_AIR 4
#define SENSOR_OUTSIDE_AIR  5
#define NUM_SENSORS         6

// INA219 — solar pump minimum current (amps). Calibrate on-site at 20% below lowest reading.
static const float SOLAR_PUMP_MIN_CURRENT_A = 0.05f; // TODO: calibrate

// PC fan minimum duty before stall. Calibrate on-site.
static const uint8_t FAN_MIN_DUTY_PCT = 20; // TODO: calibrate

// UFH/solar cold valve 30-second power-up wait (conservative; verify during commissioning)
static const uint16_t VALVE_POWERUP_WAIT_MS = 30000;

// Growatt Modbus slave address (default 1)
static const uint8_t GROWATT_ADDR = 1;

// ============================================================
//  SYSTEM CONSTANTS
// ============================================================

#define TEMP_SCALE          10    // temps stored as int16 = °C × 10
#define SOLAR_HOT_MIN_C    180    // 18.0°C — winter solar trigger
#define SOLAR_DIFF_STOP_C   20    // 2.0°C — stop when hot-cold < this
#define WINTER_UFH_TARGET_C 200   // 20.0°C — winter solar pump target
#define FROST_THRESH_C       20   // 2.0°C — frost protection trigger
#define FROST_STOP_C         80   // 8.0°C — frost protection stop
#define UFH_PUMP_OFF_C      280   // 28.0°C — stop UFH pump when supply drops here
#define UFH_OVERHEAT_C      450   // 45.0°C — hard lockout trigger
#define SOLAR_OVERHEAT_COLD_DELTA 100 // 10.0°C above tank bottom
#define RELOCK_TIMEOUT_MS  300000UL  // 5 minutes
#define ALARM_DURATION_MS   60000UL  // 1-minute alarm auto-stop
#define HANDLE_ALERT_NIGHT_MIN_MS 10000UL
#define GROWATT_POLL_MS     850UL
#define INTER_CTRL_POLL_MS  250UL
#define DS18B20_CONV_MS     750UL
#define RS485_RX_TIMEOUT_MS 150UL
#define COMMS_FAULT_THRESHOLD 5
#define FAN_RPM_FAULT_DELAY_MS 5000UL
#define VAC_PUMP_MAX_MS    1800000UL // 30 minutes
#define VAC_EXTRA_RUN_MS   300000UL  // 5 extra minutes after full vacuum
#define VALVE_PULSE_MS      7000UL   // H-bridge valve travel time
#define LOCK_PULSE_MS       1000UL   // door lock pulse
#define HBRIDGE_DEAD_MS      200UL   // dead-time between relay changes
#define SUMMER_MIN_STARTUP_DUTY 5    // 5% — solar pump during summer startup clocking
#define FAN_FLAP_OPEN_MS   30000UL   // motorized damper travel time; calibrate on-site
#define TIME_SYNC_INTERVAL_MS 3600000UL // 1 hour
#define EEPROM_FAN_BASE_ADDR 0       // EEPROM address for fan base speed

// ============================================================
//  GLOBAL OBJECTS
// ============================================================

OneWire        oneWire(PIN_ONE_WIRE);
DallasTemperature sensors(&oneWire);
Adafruit_INA219 ina219;   // address 0x40
ModbusMaster    growatt;

// ============================================================
//  SENSOR STATE
// ============================================================

float     sTemp[NUM_SENSORS];        // current readings in °C (NAN = fault)
bool      sFault[NUM_SENSORS];       // true = sensor currently faulted
uint8_t   sFailCount[NUM_SENSORS];   // consecutive invalid readings
uint8_t   sGoodCount[NUM_SENSORS];   // consecutive valid readings

static inline bool tempValid(float t) {
    return !isnan(t) && t != 85.0f && t != -127.0f && t != -128.0f
           && t > -55.0f && t < 150.0f;
}

static inline int16_t tempEncode(float t) {
    if (sFault[0] && isnan(t)) return TEMP_FAULT; // sentinel checked per-sensor elsewhere
    if (isnan(t)) return TEMP_FAULT;
    return (int16_t)(t * 10.0f + 0.5f);
}

// ============================================================
//  GROWATT DATA
// ============================================================

struct GrowattData {
    int16_t  pvStr1W, pvStr2W, pvTotalW;
    int16_t  pvExportW, gridImportW, loadW;
    uint8_t  battSOC;
    int16_t  battW;
    uint16_t battVoltage_dV;
    uint16_t gridVoltage_dV;
    uint16_t gridFreq_cHz;
    uint16_t energyTodayWh;
    int16_t  inverterTemp_dC;
    uint8_t  inverterStatus;
    uint16_t inverterFaultCode;
    unsigned long lastGoodMs;
    bool     valid;
} gd;

uint8_t  growattFailCount = 0;
unsigned long lastGrowattPollMs = 0;

// ============================================================
//  SOLAR PUMP
// ============================================================

uint8_t  pumpTargetDuty   = 0;   // 0-100%
bool     pumpOutputState  = false;
unsigned long pumpOnStartMs  = 0;
unsigned long pumpOffStartMs = 0;
float    solarPumpCurrentA = 0.0f;
unsigned long pumpCurrentSampleMs = 0;
bool     pumpCurrentSampled = false;

// Pump fault tracking
unsigned long pumpAboveClockingMs = 0; // time pump stayed above clocking speed
bool     pumpAboveClocking = false;

// Clocking parameters (updated by heating mode)
uint16_t pumpClockPeriodMs = 2000;
uint16_t pumpMinOnMs       = 200;

// ============================================================
//  H-BRIDGE VALVE  (7s pulse, 200ms dead-time)
// ============================================================

enum HBridgePhase : uint8_t { HBP_IDLE, HBP_DEAD, HBP_PULSING };

struct HBridgeValve {
    uint8_t       pinA, pinB;
    uint16_t      pulseDurationMs;
    bool          isOpen;          // last confirmed position
    HBridgePhase  phase;
    bool          pendingOpen;
    bool          hasPending;
    unsigned long phaseStartMs;

    void begin(uint8_t a, uint8_t b, uint16_t dur) {
        pinA = a; pinB = b; pulseDurationMs = dur;
        phase = HBP_IDLE; hasPending = false;
        pinMode(a, OUTPUT); digitalWrite(a, LOW);
        pinMode(b, OUTPUT); digitalWrite(b, LOW);
    }

    void request(bool open) {
        // Interrupt mid-pulse: drop to dead-time immediately
        if (phase == HBP_PULSING) {
            digitalWrite(pinA, LOW);
            digitalWrite(pinB, LOW);
        }
        pendingOpen   = open;
        hasPending    = true;
        phaseStartMs  = millis();
        phase         = HBP_DEAD;
    }

    void update() {
        unsigned long now = millis();
        switch (phase) {
            case HBP_IDLE: break;
            case HBP_DEAD:
                if (now - phaseStartMs >= HBRIDGE_DEAD_MS) {
                    hasPending = false;
                    digitalWrite(pendingOpen ? pinA : pinB, HIGH);
                    phaseStartMs = now;
                    phase = HBP_PULSING;
                }
                break;
            case HBP_PULSING:
                if (now - phaseStartMs >= pulseDurationMs) {
                    digitalWrite(pinA, LOW);
                    digitalWrite(pinB, LOW);
                    isOpen = pendingOpen;
                    phase = HBP_IDLE;
                    if (hasPending) {
                        // New command arrived during pulse — start dead-time
                        phaseStartMs = now;
                        phase = HBP_DEAD;
                    }
                }
                break;
        }
    }

    bool busy() const { return phase != HBP_IDLE; }
};

// ============================================================
//  DIRECTION VALVE  (single SPDT relay, no timed pulse)
//  Relay ON  → NO → Wire A (+15V) → valve opens
//  Relay OFF → NC → Wire B (+15V) → valve closes
//  GND permanently connected; auto-cutout at end stop.
// ============================================================

struct DirectionValve {
    uint8_t pin;
    bool    isOpen;

    void begin(uint8_t p) {
        pin = p;
        pinMode(p, OUTPUT);
        setClose(); // safe default
    }
    void setOpen()  { digitalWrite(pin, HIGH); isOpen = true; }
    void setClose() { digitalWrite(pin, LOW);  isOpen = false; }
    void set(bool open) { if (open) setOpen(); else setClose(); }
};

// ============================================================
//  VALVE INSTANCES
// ============================================================

HBridgeValve  vacIsoValve;   // vacuum isolation valve: D37/D38
HBridgeValve  doorLock;      // electric door lock: D29/D39
DirectionValve ufhColdValve; // UFH cold valve: D22
DirectionValve solarColdValve; // solar cold valve: D24

// ============================================================
//  WINDOW WINCH STATE MACHINE
// ============================================================

enum WinchPhase : uint8_t { WP_IDLE, WP_PAUSE, WP_MOVING };

struct WindowWinch {
    WinchPhase  phase;
    WinchState  activeDir;
    WinchState  pendingDir;
    unsigned long phaseStartMs;
    bool         openLockout;
    bool         closeLockout;

    void begin() {
        phase = WP_IDLE; activeDir = WINCH_STOP;
        openLockout = false; closeLockout = false;
        pinMode(PIN_WINCH_DIR_OPEN,  OUTPUT); digitalWrite(PIN_WINCH_DIR_OPEN,  LOW);
        pinMode(PIN_WINCH_DIR_CLOSE, OUTPUT); digitalWrite(PIN_WINCH_DIR_CLOSE, LOW);
        pinMode(PIN_WINCH_POWER,     OUTPUT); digitalWrite(PIN_WINCH_POWER,     LOW);
    }

    // Immediately cut power — no 1s pause. Used by safety triggers.
    void immediateStop() {
        digitalWrite(PIN_WINCH_DIR_OPEN,  LOW);
        digitalWrite(PIN_WINCH_DIR_CLOSE, LOW);
        digitalWrite(PIN_WINCH_POWER,     LOW);
        phase     = WP_IDLE;
        activeDir = WINCH_STOP;
    }

    // Commanded stop/open/close. Always routes through 1s pause.
    void request(WinchState dir) {
        if (dir == WINCH_STOP)                              { immediateStop(); return; }
        if (dir == WINCH_OPEN  && openLockout)              return;
        if (dir == WINCH_CLOSE && closeLockout)             return;

        // Cut current movement / ensure all relays off before pause
        digitalWrite(PIN_WINCH_DIR_OPEN,  LOW);
        digitalWrite(PIN_WINCH_DIR_CLOSE, LOW);
        digitalWrite(PIN_WINCH_POWER,     LOW);
        pendingDir    = dir;
        phaseStartMs  = millis();
        phase         = WP_PAUSE;
    }

    void update() {
        unsigned long now = millis();

        // Continuous lockout check while moving
        if (phase == WP_MOVING) {
            if ((activeDir == WINCH_OPEN  && openLockout) ||
                (activeDir == WINCH_CLOSE && closeLockout)) {
                immediateStop();
            }
            return;
        }

        if (phase == WP_PAUSE && now - phaseStartMs >= 1000UL) {
            if ((pendingDir == WINCH_OPEN  && openLockout) ||
                (pendingDir == WINCH_CLOSE && closeLockout)) {
                phase = WP_IDLE;
                return;
            }
            if (pendingDir == WINCH_OPEN)  digitalWrite(PIN_WINCH_DIR_OPEN,  HIGH);
            else                            digitalWrite(PIN_WINCH_DIR_CLOSE, HIGH);
            digitalWrite(PIN_WINCH_POWER, HIGH);
            activeDir = pendingDir;
            phase     = WP_MOVING;
        }
    }

    WinchState state() const {
        return (phase == WP_MOVING) ? activeDir : WINCH_STOP;
    }
} winch;

// ============================================================
//  VACUUM SYSTEM
// ============================================================

enum VacState : uint8_t { VAC_IDLE, VAC_OPENING, VAC_PUMPING, VAC_FULL, VAC_DONE, VAC_FAULT };

VacState     vacState           = VAC_IDLE;
unsigned long vacStateEnteredMs = 0;
bool          vacFullThisSession = false;

// ============================================================
//  SECURITY STATE
// ============================================================

bool         workshopLocked      = true;
unsigned long relockTimerStartMs  = 0;
bool          relockTimerActive   = false;
bool          alarmActive         = false;
unsigned long alarmStartMs        = 0;
bool          buzzerActive        = false;
bool          doorHandleAlertActive = false;
unsigned long handleAlertStartMs  = 0;
bool          extLightsOn         = false;

// ============================================================
//  FIRE ALARM STATE
// ============================================================

enum FirePhase : uint8_t { FIRE_IDLE, FIRE_ALERT, FIRE_SOUNDER_ON, FIRE_SOUNDER_OFF, FIRE_DONE };
FirePhase     firePhase            = FIRE_IDLE;
unsigned long firePhaseStartMs     = 0;
uint8_t       fireCycle            = 0;
bool          fireAlarmActive      = false;
uint8_t       fireAlarmTriggerSeq  = 0;  // alertResetSeq value at time of trigger

bool          gDoorOpen            = false;  // set by updateSecurity() each loop
uint8_t       gWinchReedFlags      = 0;      // set by updateWinchInputs() each loop
bool          wAirRising           = false;
float         wAirPrevMinTemp      = NAN;
unsigned long wAirRateMs           = 0;
unsigned long lastOutsideAbove25Ms = 0;

// ============================================================
//  FAN CONTROL
// ============================================================

uint8_t  fanBaseSpeedPct  = 30;    // loaded from EEPROM; updated from H
uint8_t  fanCurrentDuty   = 0;
uint32_t fanFullTimerSecs = 0;
uint32_t fanBaseTimerSecs = 0;
unsigned long fanLastTickMs = 0;

volatile uint16_t fan1PulseCount = 0;
volatile uint16_t fan2PulseCount = 0;
uint16_t fan1RPM = 0, fan2RPM = 0;
unsigned long fanRPMLastMs = 0;
unsigned long fan1FaultStartMs = 0;
unsigned long fan2FaultStartMs = 0;
bool fan1FaultActive = false, fan2FaultActive = false;

// ============================================================
//  HEATING STATE
// ============================================================

// UFH heating session state
enum WUFHState : uint8_t { WUFH_IDLE, WUFH_ACTIVE, WUFH_COOLING, WUFH_LOCKED };
WUFHState ufhState          = WUFH_IDLE;
bool      ufhTargetReached  = false;
bool      ufhHardLockout    = false; // UFH overheat — clears only on restart

unsigned long frostFaultStartMs = 0;  // frost-not-recovering fault timer
unsigned long vacDoneStartMs    = 0;  // VAC_DONE 10-second pump-off delay

// Summer startup sequence
enum SummerPhase : uint8_t { SUMPH_IDLE, SUMPH_CIRCULATE_TOP, SUMPH_CIRCULATE_BOT, SUMPH_RUNNING };
SummerPhase summerPhase     = SUMPH_IDLE;
bool        summerSeqDone   = false;

// Solar running flag
bool solarPumpActive = false;
bool solarActiveEver = false; // for vacuum system "heating active" check
bool frostProtActive = false;
bool frostProtUFHOpen = false;

unsigned long solarOverspeedStartMs = 0;
bool          solarOverspeedTracking = false;

// ============================================================
//  MID-POINT LED
// ============================================================

bool          ledBlinkHigh    = false;
unsigned long ledBlinkMs      = 0;

// ============================================================
//  RS485 INTER-CONTROLLER
// ============================================================

uint8_t    txSeqNum            = 0;
uint8_t    lastRxSeq           = 255;
uint8_t    missedPackets       = 0;
bool       rs485CommsFault     = false;
unsigned long lastRxMs         = 0;
unsigned long lastTxMs         = 0;

HToWPacket lastH;           // last valid H→W packet
bool       hasHPacket        = false;
uint8_t    lastAlertResetSeq = 0;

PktReceiver pktRx;

// ============================================================
//  TIME
// ============================================================

uint8_t  curHour = 0, curMinute = 0, curSecond = 0;
bool     timeSynced = false;
unsigned long lastTimeSyncRequestMs = 0;
unsigned long lastTimeSyncReceivedMs = 0;
unsigned long timeBaseMs = 0; // millis() at last sync

// ============================================================
//  FAULT STATE
// ============================================================

uint32_t wFaultFlags = 0;

void setFault(uint32_t mask)   { wFaultFlags |= mask; }
void clearFault(uint32_t mask) { wFaultFlags &= ~mask; }
bool hasFault(uint32_t mask)   { return (wFaultFlags & mask) != 0; }

#ifdef DEBUG_SERIAL
// ============================================================
//  SERIAL DEBUG INTERFACE
// ============================================================

static char    dbgBuf[64];
static uint8_t dbgLen = 0;

bool  sSimulate[NUM_SENSORS];
float sSim[NUM_SENSORS];

bool  simPIRActive       = false, simPIRVal       = false;
bool  simDoorActive      = false, simDoorVal      = false;
bool  simWinchClsActive  = false, simWinchClsVal  = false;
bool  simWinchLockActive = false, simWinchLockVal = false;
#endif // DEBUG_SERIAL

// ============================================================
//  GROWATT MODBUS CALLBACKS
// ============================================================

void preTransmitGrowatt()  { digitalWrite(PIN_RS485_DE_GROWATT, HIGH); }
void postTransmitGrowatt() { digitalWrite(PIN_RS485_DE_GROWATT, LOW);  }

// ============================================================
//  SENSOR READING
// ============================================================

unsigned long lastSensorConvMs = 0;
bool          sensorConvStarted = false;

void startSensorConversion() {
    sensors.setResolution(12);
    sensors.setWaitForConversion(false);
    sensors.requestTemperatures();
    lastSensorConvMs = millis();
    sensorConvStarted = true;
}

void readSensors() {
    if (!sensorConvStarted) return;
    if (millis() - lastSensorConvMs < DS18B20_CONV_MS) return;
    sensorConvStarted = false;

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        float t = sensors.getTempC((uint8_t*)DS18B20_ADDRS[i]);
        if (tempValid(t)) {
            sTemp[i]      = t;
            sFailCount[i] = 0;
            if (sGoodCount[i] < 3) sGoodCount[i]++;
            if (sGoodCount[i] >= 3 && sFault[i]) {
                sFault[i] = false;
                // Clear corresponding fault flag
                static const uint32_t sensorFaultMask[NUM_SENSORS] = {
                    FAULT_W_SENSOR_SOLAR_HOT, FAULT_W_SENSOR_SOLAR_COLD,
                    FAULT_W_SENSOR_UFH_SUPPLY, FAULT_W_SENSOR_UFH_POST_TMV,
                    FAULT_W_SENSOR_WORKSHOP_AIR, FAULT_W_SENSOR_OUTSIDE_AIR
                };
                clearFault(sensorFaultMask[i]);
            }
        } else {
            sGoodCount[i] = 0;
            if (sFailCount[i] < 3) sFailCount[i]++;
            if (sFailCount[i] >= 3 && !sFault[i]) {
                sFault[i] = true;
                sTemp[i]  = NAN;
                static const uint32_t sensorFaultMask[NUM_SENSORS] = {
                    FAULT_W_SENSOR_SOLAR_HOT, FAULT_W_SENSOR_SOLAR_COLD,
                    FAULT_W_SENSOR_UFH_SUPPLY, FAULT_W_SENSOR_UFH_POST_TMV,
                    FAULT_W_SENSOR_WORKSHOP_AIR, FAULT_W_SENSOR_OUTSIDE_AIR
                };
                setFault(sensorFaultMask[i]);
            }
        }
    }
#ifdef DEBUG_SERIAL
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (sSimulate[i]) { sTemp[i] = sSim[i]; sFault[i] = false; }
    }
#endif
}

// ============================================================
//  INA219 — solar pump current
// ============================================================

void sampleSolarPumpCurrent() {
    if (!pumpOutputState) return;
    if (millis() - pumpOnStartMs < 50) return; // 50ms spin-up delay
    solarPumpCurrentA = ina219.getCurrent_mA() / 1000.0f;
    pumpCurrentSampled = true;
}

// ============================================================
//  GROWATT MODBUS POLLING
// ============================================================

void pollGrowatt() {
    bool ok = false;
    uint8_t result;

    // Read input registers 0–38 (PV strings, grid, frequency, voltage, energy)
    result = growatt.readInputRegisters(0x0000, 39);
    if (result == ModbusMaster::ku8MBSuccess) {
        gd.inverterStatus  = (uint8_t)growatt.getResponseBuffer(0);

        // PV string 1: V=reg3, I=reg4, P=regs5-6 (32-bit, 0.1W units)
        uint32_t ppv1 = ((uint32_t)growatt.getResponseBuffer(5) << 16)
                         | growatt.getResponseBuffer(6);
        gd.pvStr1W = (int16_t)(ppv1 / 100); // convert 0.1W to 10W steps → Watts OK at 1W

        uint32_t ppv2 = ((uint32_t)growatt.getResponseBuffer(9) << 16)
                         | growatt.getResponseBuffer(10);
        gd.pvStr2W = (int16_t)(ppv2 / 100);

        // Total output power (regs 35-36, 0.1W)
        uint32_t pac = ((uint32_t)growatt.getResponseBuffer(35) << 16)
                        | growatt.getResponseBuffer(36);
        gd.pvTotalW = (int16_t)(pac / 100);

        gd.gridFreq_cHz    = growatt.getResponseBuffer(37); // 0.01Hz
        gd.gridVoltage_dV  = growatt.getResponseBuffer(38); // 0.1V

        // Energy today (regs 53-54, 0.1kWh = 100Wh units)
        uint32_t eToday = ((uint32_t)growatt.getResponseBuffer(53) << 16)
                           | growatt.getResponseBuffer(54);
        gd.energyTodayWh = (uint16_t)(eToday * 10); // 0.1kWh → Wh (up to 65535Wh)

        ok = true;
    }

    // Read input registers 1000–1024 (battery + hybrid data)
    result = growatt.readInputRegisters(0x03E8, 25);
    if (result == ModbusMaster::ku8MBSuccess) {
        // Battery discharge (regs 1009-1010, 0.1W)
        uint32_t battDis = ((uint32_t)growatt.getResponseBuffer(9) << 16)
                            | growatt.getResponseBuffer(10);
        // Battery charge (regs 1011-1012, 0.1W)
        uint32_t battChr = ((uint32_t)growatt.getResponseBuffer(11) << 16)
                            | growatt.getResponseBuffer(12);
        // positive = charging, negative = discharging
        int32_t battWNet = (int32_t)(battChr / 100) - (int32_t)(battDis / 100);
        gd.battW = (int16_t)constrain(battWNet, -32767, 32767);

        gd.battVoltage_dV = growatt.getResponseBuffer(13); // 0.1V
        gd.battSOC        = (uint8_t)growatt.getResponseBuffer(14); // direct %

        // Grid import (regs 1021-1022, 0.1W)
        uint32_t gridImp = ((uint32_t)growatt.getResponseBuffer(21) << 16)
                            | growatt.getResponseBuffer(22);
        // Grid export (regs 1023-1024, 0.1W)
        uint32_t gridExp = ((uint32_t)growatt.getResponseBuffer(23) << 16)
                            | growatt.getResponseBuffer(24);
        // ⚠ Verify sign convention during commissioning before enabling HEATER_ENABLED
        gd.gridImportW = (int16_t)(gridImp / 100);
        gd.pvExportW   = (int16_t)(gridExp / 100);

        // Load = total PV + battery discharge − battery charge − export + import (approx)
        // Simple calculation; Growatt may provide a dedicated load register in some firmware versions
        gd.loadW = (int16_t)((int32_t)gd.pvTotalW - gd.pvExportW + gd.gridImportW);

        // Inverter temperature is in register range 0-124; reg not specified in provided list.
        // Treat as unavailable until confirmed during commissioning.
        gd.inverterTemp_dC = TEMP_FAULT;

        gd.inverterFaultCode = 0; // TODO: map from status registers once register address confirmed

        ok = true;
    }

    if (ok) {
        gd.valid      = true;
        gd.lastGoodMs = millis();
        growattFailCount = 0;
        clearFault(FAULT_W_GROWATT_COMMS);
    } else {
        growattFailCount++;
        if (growattFailCount >= COMMS_FAULT_THRESHOLD) {
            setFault(FAULT_W_GROWATT_COMMS);
        }
    }
}

// ============================================================
//  SOLAR PUMP CLOCKING
// ============================================================

void setSolarPumpDuty(uint8_t duty) {
    pumpTargetDuty = duty;
    if (duty == 0) {
        digitalWrite(PIN_SOLAR_PUMP, LOW);
        pumpOutputState = false;
    }
}

void updateSolarPump() {
    if (pumpTargetDuty == 0) {
        if (pumpOutputState) {
            digitalWrite(PIN_SOLAR_PUMP, LOW);
            pumpOutputState = false;
        }
        return;
    }
    if (pumpTargetDuty == 100) {
        if (!pumpOutputState) {
            digitalWrite(PIN_SOLAR_PUMP, HIGH);
            pumpOutputState  = true;
            pumpOnStartMs    = millis();
        }
        return;
    }

    // Time-proportioning: ON for max(minOnMs, duty*period/100), then OFF
    uint16_t onMs  = max((uint32_t)pumpMinOnMs,
                          (uint32_t)pumpClockPeriodMs * pumpTargetDuty / 100);
    uint16_t offMs = pumpClockPeriodMs - onMs;
    unsigned long now = millis();

    if (pumpOutputState) {
        if (now - pumpOnStartMs >= onMs) {
            digitalWrite(PIN_SOLAR_PUMP, LOW);
            pumpOutputState = false;
            pumpOffStartMs  = now;
        }
    } else {
        if (now - pumpOffStartMs >= offMs) {
            digitalWrite(PIN_SOLAR_PUMP, HIGH);
            pumpOutputState = true;
            pumpOnStartMs   = now;
        }
    }
}

// Linear ramp: 0% at (target - 2°C), 100% at (target + 2°C)
uint8_t calcPumpDuty(float measured, float targetC) {
    float diff = measured - (targetC - 2.0f);
    if (diff <= 0.0f) return 0;
    if (diff >= 4.0f) return 100;
    return (uint8_t)(diff / 4.0f * 100.0f);
}

// ============================================================
//  PC FAN SPEED (Timer5 25kHz hardware PWM on D45 = OC5B)
// ============================================================

void setupFanPWM() {
    pinMode(PIN_FAN_PWM, OUTPUT);
    TCCR5A = 0; TCCR5B = 0;
    ICR5   = 639;  // 16MHz / (1 × (639+1)) = 25kHz
    OCR5B  = 0;
    // Fast PWM mode 14, non-inverting OC5B, clk/1
    TCCR5A = (1 << COM5B1) | (1 << WGM51);
    TCCR5B = (1 << WGM53)  | (1 << WGM52) | (1 << CS50);
}

void setFanPWM(uint8_t dutyPct) {
    OCR5B = (dutyPct == 0) ? 0 : (uint32_t)dutyPct * 639 / 100;
}

// Fan tachometer ISR — PCINT2 group (Port K: A8=PK0=PCINT16, A9=PK1=PCINT17)
ISR(PCINT2_vect) {
    static uint8_t prevK = 0;
    uint8_t curK    = PINK;
    uint8_t changed = curK ^ prevK;
    if ((changed & (1 << PK0)) && (curK & (1 << PK0))) fan1PulseCount++;
    if ((changed & (1 << PK1)) && (curK & (1 << PK1))) fan2PulseCount++;
    prevK = curK;
}

void updateFanRPM() {
    if (millis() - fanRPMLastMs < 1000) return;
    // 2 pulses/rev; pulses in last ~1s → RPM = count × 30
    uint16_t p1 = fan1PulseCount; fan1PulseCount = 0;
    uint16_t p2 = fan2PulseCount; fan2PulseCount = 0;
    fan1RPM = p1 * 30;
    fan2RPM = p2 * 30;
    fanRPMLastMs = millis();

    // Tachometer fault: zero RPM while commanded above minimum for > 5s
    bool fan1ShouldSpin = (fanCurrentDuty >= FAN_MIN_DUTY_PCT);
    bool fan2ShouldSpin = (fanCurrentDuty >= FAN_MIN_DUTY_PCT);

    if (fan1ShouldSpin && fan1RPM == 0) {
        if (!fan1FaultActive) {
            if (fan1FaultStartMs == 0) fan1FaultStartMs = millis();
            if (millis() - fan1FaultStartMs > FAN_RPM_FAULT_DELAY_MS) {
                fan1FaultActive = true; setFault(FAULT_W_FAN1);
            }
        }
    } else {
        fan1FaultStartMs = 0;
        if (fan1FaultActive) { fan1FaultActive = false; clearFault(FAULT_W_FAN1); }
    }

    if (fan2ShouldSpin && fan2RPM == 0) {
        if (!fan2FaultActive) {
            if (fan2FaultStartMs == 0) fan2FaultStartMs = millis();
            if (millis() - fan2FaultStartMs > FAN_RPM_FAULT_DELAY_MS) {
                fan2FaultActive = true; setFault(FAULT_W_FAN2);
            }
        }
    } else {
        fan2FaultStartMs = 0;
        if (fan2FaultActive) { fan2FaultActive = false; clearFault(FAULT_W_FAN2); }
    }
}

// Fan control: W computes speed autonomously from lock state + settings from H
void updateFanControl() {
    unsigned long now = millis();

    // Tick 1-second fan timer countdown
    if (now - fanLastTickMs >= 1000UL) {
        fanLastTickMs = now;
        if (fanFullTimerSecs > 0) fanFullTimerSecs--;
        if (fanBaseTimerSecs > 0) fanBaseTimerSecs--;
    }

    // Priority: 1=full-speed timer, 2=base-speed timer, 3=unlocked (base speed), 4=off
    uint8_t targetDuty = 0;
    if (fanFullTimerSecs > 0) {
        targetDuty = 100;
    } else if (fanBaseTimerSecs > 0) {
        targetDuty = max(fanBaseSpeedPct, FAN_MIN_DUTY_PCT);
    } else if (!workshopLocked) {
        targetDuty = max(fanBaseSpeedPct, FAN_MIN_DUTY_PCT);
    } else {
        targetDuty = 0;
    }

    // Minimum enforcement if tach reports stall (fan1 or fan2)
    if (targetDuty > 0 && targetDuty < FAN_MIN_DUTY_PCT) {
        targetDuty = FAN_MIN_DUTY_PCT;
    }

    fanCurrentDuty = targetDuty;
    setFanPWM(targetDuty);
}

// ============================================================
//  VACUUM SYSTEM
// ============================================================

void updateVacuum() {
    bool heatingOrSolarActive = solarPumpActive || ufhState == WUFH_ACTIVE
                                 || ufhState == WUFH_COOLING;
    bool vacSensorFull = (digitalRead(PIN_VAC_SENSOR) == LOW);
    unsigned long now  = millis();

    switch (vacState) {
        case VAC_IDLE:
            if (heatingOrSolarActive && !vacFullThisSession) {
                vacIsoValve.request(true); // open
                vacStateEnteredMs = now;
                vacState = VAC_OPENING;
            }
            break;

        case VAC_OPENING:
            vacIsoValve.update();
            if (!vacIsoValve.busy()) {
                digitalWrite(PIN_VAC_PUMP, HIGH);
                vacStateEnteredMs = now;
                vacState = VAC_PUMPING;
            }
            break;

        case VAC_PUMPING:
            if (vacSensorFull) {
                vacStateEnteredMs = now;
                vacState = VAC_FULL;
            } else if (now - vacStateEnteredMs >= VAC_PUMP_MAX_MS) {
                // Overtime fault
                digitalWrite(PIN_VAC_PUMP, LOW);
                setFault(FAULT_W_VAC_PUMP_OVERTIME);
                vacState = VAC_FAULT;
            }
            break;

        case VAC_FULL:
            // Run pump for 5 extra minutes after full vacuum, then close valve
            if (now - vacStateEnteredMs >= VAC_EXTRA_RUN_MS) {
                vacIsoValve.request(false); // close isolation valve
                vacDoneStartMs    = now;
                vacStateEnteredMs = now;
                vacState = VAC_DONE;
            }
            break;

        case VAC_DONE:
            // Valve is closing; stop pump 10 seconds after valve close command
            vacIsoValve.update();
            if (now - vacDoneStartMs >= 10000UL) {
                digitalWrite(PIN_VAC_PUMP, LOW);
                vacFullThisSession = true;
                vacState = VAC_IDLE;
            }
            break;

        case VAC_FAULT:
            break;
    }
    vacIsoValve.update();

    // Reset session flag when a new heating day starts (morningHeatActive goes 0→1)
    static bool prevMorningHeat = false;
    bool curMorningHeat = hasHPacket ? lastH.morningHeatActive : false;
    if (!prevMorningHeat && curMorningHeat) {
        vacFullThisSession = false;
        if (vacState == VAC_FAULT) clearFault(FAULT_W_VAC_PUMP_OVERTIME);
        vacState = VAC_IDLE;
    }
    prevMorningHeat = curMorningHeat;
}

// ============================================================
//  WINTER SOLAR
// ============================================================

void updateWinterSolar(float tankBottomC) {
    bool solarHotFault  = sFault[SENSOR_SOLAR_HOT];
    bool solarColdFault = sFault[SENSOR_SOLAR_COLD];
    // Tank bottom is stale if RS485 link is down or H packet is old
    bool tankBotStale = !hasHPacket || (millis() - lastRxMs > 10000UL) || isnan(tankBottomC);

    if (solarHotFault || solarColdFault) {
        // Sensor fault: dump through UFH
        ufhColdValve.setOpen();
        solarColdValve.setOpen();
        setSolarPumpDuty(100);
        setFault(FAULT_W_SOLAR_PUMP); // covered by sensor fault logic
        solarPumpActive = false;
        return;
    }

    float hot  = sTemp[SENSOR_SOLAR_HOT];
    float cold = sTemp[SENSOR_SOLAR_COLD];

    // Frost protection takes priority
    bool frostNeeded = (hot < 2.0f || cold < 2.0f);
    if (frostNeeded) {
        if (!frostProtActive) {
            frostProtActive   = true;
            frostFaultStartMs = millis();
            ufhColdValve.setOpen();
            solarColdValve.setOpen();
            setSolarPumpDuty(100);
        }
        if (hot >= 8.0f) {
            // Pump has done its job; leave UFH valve open, stop pump
            setSolarPumpDuty(0);
            frostProtUFHOpen = true;
        }
        // Frost not recovering: active > 1 min AND cold side still < 4°C
        if (cold < 4.0f && millis() - frostFaultStartMs > 60000UL) {
            setFault(FAULT_W_FROST_NOT_RECOVERING);
        }
        return;
    } else {
        frostProtActive   = false;
        frostFaultStartMs = 0;
        clearFault(FAULT_W_FROST_NOT_RECOVERING);
    }

    // Solar trigger: hot >= 18°C
    bool triggerMet = (hot >= 18.0f);

    if (!triggerMet) {
        if (solarPumpActive) {
            // Hot dropped — check if hot > cold + 2°C to keep running
            if ((hot - cold) <= 2.0f) {
                // Stop solar
                solarColdValve.setClose();
                setSolarPumpDuty(0);
                solarPumpActive = false;
            }
        }
        return;
    }

    if (!solarPumpActive) {
        ufhColdValve.setOpen();
        solarColdValve.setOpen();
        pumpClockPeriodMs = 2000;
        pumpMinOnMs       = 200;
        setSolarPumpDuty(0); // starts at clocking minimum in updateSolarPump
        solarPumpActive  = true;
        solarActiveEver  = true;
    }

    // Speed ramp: target = 20°C (UFH circuit temp)
    uint8_t duty = calcPumpDuty(cold, (float)WINTER_UFH_TARGET_C / 10.0f);
    if (duty == 0) duty = 1; // ensure minimum clocking
    setSolarPumpDuty(duty);

    // Overspeed fault: pump above clocking for > 10s AND cold > tankBottom + 10°C
    // If tank bottom is stale: skip fault check
    bool pumpAboveMin = (duty > (uint8_t)(pumpMinOnMs * 100 / pumpClockPeriodMs + 1));
    if (pumpAboveMin && !tankBotStale && !sFault[SENSOR_SOLAR_COLD]) {
        float tankBot = tankBottomC;
        if (cold > tankBot + 10.0f) {
            if (!solarOverspeedTracking) {
                solarOverspeedTracking = true;
                solarOverspeedStartMs  = millis();
            } else if (millis() - solarOverspeedStartMs > 10000UL) {
                setFault(FAULT_W_SOLAR_OVERHEAT_COLD);
                // Dump through UFH
                ufhColdValve.setOpen();
                solarColdValve.setOpen();
            }
        } else {
            solarOverspeedTracking = false;
            clearFault(FAULT_W_SOLAR_OVERHEAT_COLD);
        }
    } else {
        solarOverspeedTracking = false;
        if (!pumpAboveMin) clearFault(FAULT_W_SOLAR_OVERHEAT_COLD);
    }

    // Stop condition: hot no longer > cold + 2°C
    if ((hot - cold) <= 2.0f) {
        solarColdValve.setClose();
        setSolarPumpDuty(0);
        solarPumpActive = false;
    }
}


// ============================================================
//  SUMMER SOLAR  (simplified — full sequence coordinated with H)
// ============================================================

void updateSummerSolar() {
    static bool solarDumpActive = false;

    if (sFault[SENSOR_SOLAR_HOT] || sFault[SENSOR_SOLAR_COLD]) {
        if (!solarDumpActive) {
            // Sensor fault: emergency dump through UFH circuit.
            // Normal summer logic never runs the UFH pump, so start it explicitly here.
            // H reads wFaultFlags (FAULT_W_SENSOR_SOLAR_HOT/COLD) and must disable the
            // immersion heater in response — this is enforced on the H side.
            ufhColdValve.setOpen();
            solarColdValve.setOpen();
            digitalWrite(PIN_UFH_PUMP, HIGH);
            solarDumpActive = true;
        }
        setSolarPumpDuty(100);
        solarPumpActive = false;
        return;
    }

    if (solarDumpActive) {
        // Sensor fault cleared — stop the dump-mode UFH pump and exit dump state
        digitalWrite(PIN_UFH_PUMP, LOW);
        solarDumpActive = false;
    }

    float hot     = sTemp[SENSOR_SOLAR_HOT];
    float cold    = sTemp[SENSOR_SOLAR_COLD];
    bool pvActive = gd.valid && gd.pvTotalW > 0;

    // Summer solar startup condition (PV export >= 500W or solar >= 50°C)
    bool startTrigger = (gd.valid && gd.pvExportW >= 500)
                         || hot >= 50.0f || cold >= 50.0f;

    if (!startTrigger && !solarPumpActive) return;

    if (startTrigger && !summerSeqDone && summerPhase == SUMPH_IDLE) {
        summerPhase = SUMPH_CIRCULATE_TOP;
        pumpClockPeriodMs = 20000;
        pumpMinOnMs       = 500;
        solarColdValve.setOpen();
        ufhColdValve.setClose(); // UFH cold stays closed in summer; opens only for fault dump
        setSolarPumpDuty(SUMMER_MIN_STARTUP_DUTY);
        solarPumpActive  = true;
        solarActiveEver  = true;
    }

    if (!solarPumpActive && !startTrigger) return;

    // Advance to running when H reaches phase 3; pump modulation runs regardless of phase
    if (summerPhase == SUMPH_CIRCULATE_TOP && hasHPacket && lastH.summerStartupPhase >= 3) {
        summerPhase   = SUMPH_RUNNING;
        summerSeqDone = true;
    }

    // Determine pump target: solar hot vs target (tank+5 from H data)
    bool tankTopFault = (lastH.tempTankTop == TEMP_FAULT);
    float tankTopC    = tankTopFault ? 60.0f : (float)lastH.tempTankTop / 10.0f;

    SolarTargetMode tgtMode = hasHPacket ? (SolarTargetMode)lastH.solarTargetMode : SOLAR_TANK_PLUS5;
    float solarTarget  = (tgtMode == SOLAR_MAX) ? 80.0f : min(tankTopC + 5.0f, 80.0f);
    float heaterTarget = (tgtMode == SOLAR_MAX) ? 89.0f : min(tankTopC + 5.0f, 89.0f);

    // Cal override: H sets calPumpActive=1 and drives solarTarget remotely
    if (hasHPacket && lastH.calPumpActive) {
        solarTarget  = (float)lastH.calSolarTargetC / 10.0f;
        heaterTarget = 89.0f;
        if (!solarPumpActive) {
            solarColdValve.setOpen();
            pumpClockPeriodMs = 20000;
            pumpMinOnMs       = 500;
            solarPumpActive   = true;
            solarActiveEver   = true;
        }
    }

    // Solar hot side: ramp 0→100% over ±2°C around solarTarget
    uint8_t dutySolar = calcPumpDuty(hot, solarTarget);

    // Heater output side: ramp 0→100% over ±2°C around heaterTarget, only when heater is on
    uint8_t dutyHeater = 0;
    if (hasHPacket && lastH.heaterPowerPct > 0 && lastH.tempHeaterOut != TEMP_FAULT) {
        dutyHeater = calcPumpDuty((float)lastH.tempHeaterOut / 10.0f, heaterTarget);
    }

    uint8_t duty = max(dutySolar, dutyHeater);
    if (duty == 0 && solarPumpActive) duty = SUMMER_MIN_STARTUP_DUTY; // keep clocking
    setSolarPumpDuty(duty);

    // Overspeed overheat fault (same as winter)
    bool tankBotStale = !hasHPacket || (millis() - lastRxMs > 10000UL)
                         || lastH.tempTankBot == TEMP_FAULT;
    float tankBotC    = tankBotStale ? 0.0f : (float)lastH.tempTankBot / 10.0f;
    bool pumpAboveMin = (duty > SUMMER_MIN_STARTUP_DUTY);
    if (pumpAboveMin && !tankBotStale && !sFault[SENSOR_SOLAR_COLD]) {
        float cold2 = sTemp[SENSOR_SOLAR_COLD];
        if (cold2 > tankBotC + 10.0f) {
            if (!solarOverspeedTracking) {
                solarOverspeedTracking = true;
                solarOverspeedStartMs  = millis();
            } else if (millis() - solarOverspeedStartMs > 10000UL) {
                setFault(FAULT_W_SOLAR_OVERHEAT_COLD);
            }
        } else {
            solarOverspeedTracking = false;
            clearFault(FAULT_W_SOLAR_OVERHEAT_COLD);
        }
    }

    // Hot side overheat: tank bottom < 70°C AND solar hot > 83°C AND pump at 100%
    if (!tankBotStale && tankBotC < 70.0f && hot > 83.0f && duty == 100) {
        setFault(FAULT_W_SOLAR_OVERHEAT_HOT);
        // Dump through UFH if both sides ≤ 90°C
        if (hot <= 90.0f && sTemp[SENSOR_SOLAR_COLD] <= 90.0f) {
            ufhColdValve.setOpen();
        }
    } else if (hot <= 83.0f) {
        clearFault(FAULT_W_SOLAR_OVERHEAT_HOT);
        ufhColdValve.setClose();
    }

    // End-of-day abort (all phases): PV gone + pipe differential lost + array below 45°C.
    // Sensor faults default to abort-condition-met (cannot confirm heat present).
    // 45°C threshold gives hysteresis below the 50°C start trigger.
    bool pipeCool = sFault[SENSOR_SOLAR_HOT] || sFault[SENSOR_SOLAR_COLD]
                     || (hot - cold) < 5.0f;
    bool arrayLow = sFault[SENSOR_SOLAR_HOT] || sFault[SENSOR_SOLAR_COLD]
                     || hot < 45.0f || cold < 45.0f;
    if (!pvActive && pipeCool && arrayLow) {
        solarColdValve.setClose();
        ufhColdValve.setClose();
        setSolarPumpDuty(0);
        solarPumpActive = false;
        summerPhase     = SUMPH_IDLE;
        summerSeqDone   = false;
    }
}

// ============================================================
//  UFH HEATING SESSION  (W-side: pump + UFH cold valve)
// ============================================================

void updateUFHHeating() {
    bool morningActive = hasHPacket ? lastH.morningHeatActive : false;
    float stopTempC    = hasHPacket ? (float)lastH.ufhStopTemp_dC / 10.0f : 13.5f;
    float wAir         = sFault[SENSOR_WORKSHOP_AIR] ? NAN : sTemp[SENSOR_WORKSHOP_AIR];
    float ufhSupply    = sFault[SENSOR_UFH_SUPPLY]   ? NAN : sTemp[SENSOR_UFH_SUPPLY];

    // UFH overheat: post-TMV > 45°C while pump running → hard lockout
    if (!sFault[SENSOR_UFH_POST_TMV] && !ufhHardLockout) {
        float postTMV = sTemp[SENSOR_UFH_POST_TMV];
        if (ufhState != WUFH_IDLE && postTMV > 45.0f) {
            setSolarPumpDuty(0);
            digitalWrite(PIN_UFH_PUMP, LOW);
            ufhColdValve.setClose();
            ufhHardLockout = true;
            setFault(FAULT_W_UFH_OVERHEAT);
        }
    }
    if (ufhHardLockout && ufhState != WUFH_LOCKED) {
        // Allow frost protection to still use UFH (handled in heating functions)
        ufhState = WUFH_LOCKED;
    }

    static bool prevMorning = false;
    if (!prevMorning && morningActive) {
        // New heating session: reset per-session flags.
        // ufhHardLockout is intentionally NOT cleared here — it persists until power cycle.
        ufhTargetReached = false;
        if (!ufhHardLockout) {
            ufhState = WUFH_IDLE;
        }
    }
    prevMorning = morningActive;

    if (!morningActive) {
        if (ufhState == WUFH_ACTIVE || ufhState == WUFH_COOLING) {
            digitalWrite(PIN_UFH_PUMP, LOW);
            if (ufhState == WUFH_ACTIVE) ufhColdValve.setClose();
            ufhState = WUFH_IDLE;
        }
        return;
    }

    switch (ufhState) {
        case WUFH_IDLE:
            if (ufhHardLockout) break;
            if (!isnan(wAir) && wAir < stopTempC) {
                ufhColdValve.setOpen();
                digitalWrite(PIN_UFH_PUMP, HIGH);
                ufhState = WUFH_ACTIVE;
            } else {
                // Already at or above target
                ufhTargetReached = true;
                ufhState = WUFH_LOCKED;
            }
            break;

        case WUFH_ACTIVE:
            if (!isnan(wAir) && wAir >= stopTempC) {
                ufhTargetReached = true;
                ufhState = WUFH_COOLING;
            }
            break;

        case WUFH_COOLING:
            // Keep pump running until UFH supply (return) drops below 28°C
            if (!sFault[SENSOR_UFH_SUPPLY] && ufhSupply < 28.0f) {
                digitalWrite(PIN_UFH_PUMP, LOW);
                ufhColdValve.setClose();
                ufhState = WUFH_LOCKED;
            }
            break;

        case WUFH_LOCKED:
            // Session locked — wait for next morningActive transition
            break;
    }
}

// ============================================================
//  SECURITY & ACCESS LOGIC
// ============================================================

// Read time from our synced clock
void getCurrentTime(uint8_t &h, uint8_t &m) {
    if (!timeSynced) { h = 12; m = 0; return; } // default safe daytime if no sync
    unsigned long elapsed = (millis() - timeBaseMs) / 1000UL;
    uint32_t totalSec = (uint32_t)curHour * 3600 + curMinute * 60 + curSecond + elapsed;
    totalSec %= 86400UL;
    h = (uint8_t)(totalSec / 3600);
    m = (uint8_t)((totalSec % 3600) / 60);
}

bool isNightHours() {
    uint8_t h, m; getCurrentTime(h, m);
    return (h >= 23 || h < 6);
}

void updateSecurity() {
#ifdef DEBUG_SERIAL
    bool doorOpen     = simDoorActive    ? simDoorVal    : (digitalRead(PIN_DOOR_REED)       == LOW);
    bool pirActive    = simPIRActive     ? simPIRVal     : (digitalRead(PIN_PIR)              == HIGH);
#else
    bool doorOpen     = (digitalRead(PIN_DOOR_REED) == LOW);
    bool pirActive    = (digitalRead(PIN_PIR)        == HIGH);
#endif
    gDoorOpen = doorOpen;
    bool manualSupp   = (digitalRead(PIN_MANUAL_RELOCK) == LOW); // LOW = suppression ON
    bool handleDown   = (digitalRead(PIN_DOOR_HANDLE)   == LOW);
#ifdef DEBUG_SERIAL
    bool winchSecured = (simWinchClsActive  ? simWinchClsVal  : (digitalRead(PIN_WINCH_REED_CLOSE) == HIGH))
                     && (simWinchLockActive ? simWinchLockVal : (digitalRead(PIN_WINCH_REED_LOCK)  == HIGH));
#else
    bool winchSecured = (digitalRead(PIN_WINCH_REED_CLOSE) == HIGH)
                     && (digitalRead(PIN_WINCH_REED_LOCK)  == HIGH);
#endif
    bool unlockBtn    = (digitalRead(PIN_UNLOCK_BTN)    == HIGH);
    bool lightBtn     = (digitalRead(PIN_LIGHT_BTN)     == HIGH);
    unsigned long now = millis();

    // Unlock button: pulse door lock 1s, hold mid-point LED high
    static bool prevUnlockBtn = false;
    if (unlockBtn && !prevUnlockBtn && workshopLocked) {
        doorLock.request(true); // open = unlock
        workshopLocked = false;
    }
    prevUnlockBtn = unlockBtn;

    // External light toggle
    static bool prevLightBtn = false;
    if (!fireAlarmActive && lightBtn && !prevLightBtn) {
        extLightsOn = !extLightsOn;
        digitalWrite(PIN_EXT_LIGHTS, extLightsOn ? HIGH : LOW);
    }
    prevLightBtn = lightBtn;

    // W-side light button (same effect)
    // (connected to D11 — handled above as lightBtn)

    // Auto-relock timer: starts when ALL four conditions clear
    bool relockAllowed = !doorOpen && !pirActive && !manualSupp && winchSecured;
    if (!workshopLocked) {
        if (relockAllowed) {
            if (!relockTimerActive) {
                relockTimerActive = true;
                relockTimerStartMs = now;
            } else if (now - relockTimerStartMs >= RELOCK_TIMEOUT_MS) {
                // Relock
                doorLock.request(false);
                workshopLocked = true;
                relockTimerActive = false;
            }
        } else {
            relockTimerActive = false; // reset if any condition active
        }
    }

    // Intruder alert: PIR while locked
    static bool prevPIR = false;
    if (workshopLocked && pirActive && !prevPIR) {
        alarmActive   = true;
        alarmStartMs  = now;
        buzzerActive  = true; // relay at W + signal to H
        digitalWrite(PIN_ALARM_SOUNDER, HIGH);
    }
    if (alarmActive) {
        if (!pirActive || now - alarmStartMs >= ALARM_DURATION_MS) {
            // Auto-stop after 1 minute; stop immediately if PIR clears
            if (now - alarmStartMs >= ALARM_DURATION_MS) {
                digitalWrite(PIN_ALARM_SOUNDER, LOW);
                alarmActive = false;
            }
        }
        // Buzzer at H stays active while PIR is still active
        buzzerActive = pirActive;
        if (!pirActive) { buzzerActive = false; }
    }
    prevPIR = pirActive;

    // Door handle alert
    if (workshopLocked && handleDown) {
        if (!doorHandleAlertActive) {
            doorHandleAlertActive = true;
            handleAlertStartMs    = now;
            buzzerActive          = true;
        }
        // Night: minimum 10s buzzer; day: buzzer while held
        buzzerActive = true;
    } else if (doorHandleAlertActive) {
        bool nightHours = isNightHours();
        if (nightHours) {
            // Continue buzzer until minimum time elapsed
            if (now - handleAlertStartMs >= HANDLE_ALERT_NIGHT_MIN_MS) {
                buzzerActive = false;
                doorHandleAlertActive = false;
            }
        } else {
            buzzerActive = false;
            doorHandleAlertActive = false;
        }
    }

    // Fault-based buzzer: solar overheat, pump fault, or fire alarm
    buzzerActive |= hasFault(FAULT_W_SOLAR_OVERHEAT_COLD)
                 || hasFault(FAULT_W_SOLAR_OVERHEAT_HOT)
                 || hasFault(FAULT_W_SOLAR_PUMP)
                 || fireAlarmActive;

    digitalWrite(PIN_BUZZER_SIGNAL, buzzerActive ? HIGH : LOW);
    doorLock.update();
}

// ============================================================
//  WINDOW WINCH INPUTS  (called every loop)
// ============================================================

void updateWinchInputs() {
    bool fullOpen    = (digitalRead(PIN_WINCH_REED_OPEN)  == LOW);
#ifdef DEBUG_SERIAL
    bool fullClosed  = simWinchClsActive  ? simWinchClsVal  : (digitalRead(PIN_WINCH_REED_CLOSE) == HIGH);
    bool manualLock  = simWinchLockActive ? simWinchLockVal : (digitalRead(PIN_WINCH_REED_LOCK)  == HIGH);
#else
    bool fullClosed  = (digitalRead(PIN_WINCH_REED_CLOSE) == HIGH);
    bool manualLock  = (digitalRead(PIN_WINCH_REED_LOCK)  == HIGH);
#endif
    bool safetyLimit = (digitalRead(PIN_WINCH_SAFETY)     == LOW);

    gWinchReedFlags = 0;
    if (fullOpen)    gWinchReedFlags |= WREED_FULLY_OPEN;
    if (fullClosed)  gWinchReedFlags |= WREED_FULLY_CLOSED;
    if (manualLock)  gWinchReedFlags |= WREED_MANUAL_LOCK;
    if (safetyLimit) gWinchReedFlags |= WREED_SAFETY_LIMIT;

    // Lockout management
    winch.openLockout  = fullOpen || manualLock || safetyLimit;
    winch.closeLockout = fullClosed || manualLock;

    // Safety limit fault
    if (safetyLimit) {
        setFault(FAULT_W_WINCH_OVER_OPEN);
    } else {
        // Clears automatically when switch opens (but LED clears only on alert reset)
    }

    // Manual buttons — hold to run; reed lockouts stop the winch automatically
    bool winOpenBtn  = (digitalRead(PIN_WIN_OPEN_BTN)  == LOW);
    bool winCloseBtn = (digitalRead(PIN_WIN_CLOSE_BTN) == LOW);
    static bool prevWinOpenBtn  = false;
    static bool prevWinCloseBtn = false;

    if (winOpenBtn  && !prevWinOpenBtn)  winch.request(WINCH_OPEN);
    if (winCloseBtn && !prevWinCloseBtn) winch.request(WINCH_CLOSE);

    // Stop on release — covers both mid-pause and mid-movement
    if (!winOpenBtn  && prevWinOpenBtn) {
        if ((winch.phase == WP_MOVING && winch.activeDir  == WINCH_OPEN) ||
            (winch.phase == WP_PAUSE  && winch.pendingDir == WINCH_OPEN))  winch.immediateStop();
    }
    if (!winCloseBtn && prevWinCloseBtn) {
        if ((winch.phase == WP_MOVING && winch.activeDir  == WINCH_CLOSE) ||
            (winch.phase == WP_PAUSE  && winch.pendingDir == WINCH_CLOSE)) winch.immediateStop();
    }

    prevWinOpenBtn  = winOpenBtn;
    prevWinCloseBtn = winCloseBtn;

    winch.update();
}

// ============================================================
//  NIGHT COOLING  (wall fan only — PC fans unaffected)
// ============================================================

void updateNightCooling() {
    bool nightCoolingEnabled = hasHPacket ? lastH.nightCoolingEnabled : false;

    enum NightFanState : uint8_t { NF_OFF, NF_FLAP_OPENING, NF_ON };
    static NightFanState nfState   = NF_OFF;
    static unsigned long nfFlapMs  = 0;

    auto nightFanOff = [&]() {
        digitalWrite(PIN_WALL_FAN, LOW);
        digitalWrite(PIN_FAN_FLAP, LOW);
        nfState = NF_OFF;
    };

    if (!nightCoolingEnabled) {
        if (nfState != NF_OFF) nightFanOff();
        return;
    }

    if (sFault[SENSOR_WORKSHOP_AIR] || sFault[SENSOR_OUTSIDE_AIR]) {
        if (nfState != NF_OFF) nightFanOff();
        return;
    }

    float gap = sTemp[SENSOR_WORKSHOP_AIR] - sTemp[SENSOR_OUTSIDE_AIR];

    switch (nfState) {
        case NF_OFF:
            if (gap >= 5.0f) {
                digitalWrite(PIN_FAN_FLAP, HIGH);
                nfFlapMs = millis();
                nfState  = NF_FLAP_OPENING;
            }
            break;
        case NF_FLAP_OPENING:
            if (gap <= 2.0f) {
                nightFanOff();
            } else if (millis() - nfFlapMs >= FAN_FLAP_OPEN_MS) {
                digitalWrite(PIN_WALL_FAN, HIGH);
                nfState = NF_ON;
            }
            break;
        case NF_ON:
            if (gap <= 2.0f) nightFanOff();
            break;
    }
}

// ============================================================
//  FIRE ALARM
// ============================================================

static void updateFireRateCheck(unsigned long now) {
    if (now - wAirRateMs < 60000UL) return;
    float wAir = sFault[SENSOR_WORKSHOP_AIR] ? NAN : sTemp[SENSOR_WORKSHOP_AIR];
    if (!isnan(wAir) && !isnan(wAirPrevMinTemp)) {
        wAirRising = (wAir - wAirPrevMinTemp) >= 0.5f;
    } else {
        wAirRising = false;
    }
    wAirPrevMinTemp = wAir;
    wAirRateMs = now;
}

static bool checkFireTrigger() {
    if (sFault[SENSOR_WORKSHOP_AIR]) return false;
    float wAir = sTemp[SENSOR_WORKSHOP_AIR];

    // Absolute: workshop air > 25°C and outside hasn't been >= 25°C in last 24hrs
    bool outsideRecentlyHot = (lastOutsideAbove25Ms != 0)
                           && (millis() - lastOutsideAbove25Ms < 86400000UL);
    if (wAir > 25.0f && !outsideRecentlyHot) return true;

    // Rate of rise: >= 0.5°C/min with door closed and winch fully closed
    if (wAirRising && !gDoorOpen && (gWinchReedFlags & WREED_FULLY_CLOSED)) return true;

    return false;
}

void updateFireAlarm() {
    unsigned long now = millis();

    // Track outside temp for 24hr inhibit
    if (!sFault[SENSOR_OUTSIDE_AIR] && sTemp[SENSOR_OUTSIDE_AIR] >= 25.0f) {
        lastOutsideAbove25Ms = now;
    }

    // Reset: H controller user acknowledged the alert
    if (fireAlarmActive && hasHPacket && lastH.alertResetSeq != fireAlarmTriggerSeq) {
        firePhase = FIRE_IDLE;
        fireAlarmActive = false;
        fireCycle = 0;
        digitalWrite(PIN_ALARM_SOUNDER, LOW);
        digitalWrite(PIN_EXT_LIGHTS, extLightsOn ? HIGH : LOW); // restore to pre-alarm state
        clearFault(FAULT_W_FIRE_ALARM);
        return;
    }

    if (firePhase == FIRE_IDLE) {
        updateFireRateCheck(now);
        if (checkFireTrigger()) {
            firePhase           = FIRE_ALERT;
            firePhaseStartMs    = now;
            fireCycle           = 0;
            fireAlarmActive     = true;
            fireAlarmTriggerSeq = lastAlertResetSeq;
            setFault(FAULT_W_FIRE_ALARM);
        }
        return;
    }

    // Flash D30 at 250ms while alarm active
    bool flashOn = ((now / 250UL) % 2) == 0;
    digitalWrite(PIN_EXT_LIGHTS, flashOn ? HIGH : LOW);

    updateFireRateCheck(now);

    switch (firePhase) {
        case FIRE_ALERT:
            // D40 held on via buzzerActive in updateSecurity(); no sounder yet
            if (now - firePhaseStartMs >= 60000UL) {
                firePhase        = FIRE_SOUNDER_ON;
                firePhaseStartMs = now;
                fireCycle        = 1;
                digitalWrite(PIN_ALARM_SOUNDER, HIGH);
            }
            break;

        case FIRE_SOUNDER_ON:
            if (now - firePhaseStartMs >= 60000UL) {
                digitalWrite(PIN_ALARM_SOUNDER, LOW);
                firePhase        = FIRE_SOUNDER_OFF;
                firePhaseStartMs = now;
            }
            break;

        case FIRE_SOUNDER_OFF:
            if (now - firePhaseStartMs >= 60000UL) {
                if (fireCycle >= 5 || !wAirRising) {
                    firePhase = FIRE_DONE;
                } else {
                    fireCycle++;
                    firePhase        = FIRE_SOUNDER_ON;
                    firePhaseStartMs = now;
                    digitalWrite(PIN_ALARM_SOUNDER, HIGH);
                }
            }
            break;

        case FIRE_DONE:
            // Sounder cycles exhausted; D40 and flashing continue until H reset
            break;

        default: break;
    }
}

// ============================================================
//  MID-POINT LED UPDATE  (D34)
// ============================================================

void updateMidpointLED() {
    unsigned long now = millis();
    if (fireAlarmActive) {
        bool flashOn = ((now / 250UL) % 2) == 0;
        digitalWrite(PIN_MIDPOINT_LED, flashOn ? HIGH : LOW);
        return;
    }
    bool anyFault       = (wFaultFlags != 0) || (hasHPacket && lastH.hFaultFlags != 0);
    bool manualHeaterOn = hasHPacket && lastH.manualHeaterMode != MHM_OFF;

    uint8_t priority = anyFault ? 1 : (manualHeaterOn ? 2 : (!workshopLocked ? 3 : 4));
    static uint8_t prevPriority = 0;
    if (priority != prevPriority) {
        // Reset blink state on every priority change to ensure clean first interval
        ledBlinkMs   = now;
        ledBlinkHigh = true;
        prevPriority = priority;
    }

    if (anyFault) {
        // Priority 1: 1s on / 1s off
        if (now - ledBlinkMs >= 1000UL) {
            ledBlinkHigh = !ledBlinkHigh;
            ledBlinkMs   = now;
        }
        digitalWrite(PIN_MIDPOINT_LED, ledBlinkHigh ? HIGH : LOW);
    } else if (manualHeaterOn) {
        // Priority 2: 1s on / 5s off
        if (now - ledBlinkMs >= (ledBlinkHigh ? 1000UL : 5000UL)) {
            ledBlinkHigh = !ledBlinkHigh;
            ledBlinkMs   = now;
        }
        digitalWrite(PIN_MIDPOINT_LED, ledBlinkHigh ? HIGH : LOW);
    } else if (!workshopLocked) {
        // Priority 3: steady on
        digitalWrite(PIN_MIDPOINT_LED, HIGH);
    } else {
        // Priority 4: off
        digitalWrite(PIN_MIDPOINT_LED, LOW);
    }
}

// ============================================================
//  SOLAR PUMP CURRENT FAULT  (INA219)
// ============================================================

void checkSolarPumpFault() {
    if (!pumpOutputState || pumpTargetDuty == 0) {
        clearFault(FAULT_W_SOLAR_PUMP);
        return;
    }
    if (!pumpCurrentSampled) return;

    static unsigned long faultStartMs = 0;
    if (solarPumpCurrentA < SOLAR_PUMP_MIN_CURRENT_A) {
        if (faultStartMs == 0) faultStartMs = millis();
        if (millis() - faultStartMs > 5000UL) {
            setFault(FAULT_W_SOLAR_PUMP);
            // UFH dump if both sides ≤ 90°C
            if (!sFault[SENSOR_SOLAR_HOT] && !sFault[SENSOR_SOLAR_COLD]) {
                if (sTemp[SENSOR_SOLAR_HOT] <= 90.0f && sTemp[SENSOR_SOLAR_COLD] <= 90.0f) {
                    ufhColdValve.setOpen();
                    solarColdValve.setOpen();
                }
            }
        }
    } else {
        faultStartMs = 0;
        clearFault(FAULT_W_SOLAR_PUMP);
    }
    pumpCurrentSampled = false;
}

// ============================================================
//  RS485 INTER-CONTROLLER  LINK
// ============================================================

void sendWToHPacket() {
    WToHPacket pkt;
    memset(&pkt, 0, sizeof(pkt));

    // Temperatures
    pkt.tempSolarHot    = sFault[SENSOR_SOLAR_HOT]    ? TEMP_FAULT : (int16_t)(sTemp[SENSOR_SOLAR_HOT]    * 10);
    pkt.tempSolarCold   = sFault[SENSOR_SOLAR_COLD]   ? TEMP_FAULT : (int16_t)(sTemp[SENSOR_SOLAR_COLD]   * 10);
    pkt.tempUFHSupply   = sFault[SENSOR_UFH_SUPPLY]   ? TEMP_FAULT : (int16_t)(sTemp[SENSOR_UFH_SUPPLY]   * 10);
    pkt.tempUFHPostTMV  = sFault[SENSOR_UFH_POST_TMV] ? TEMP_FAULT : (int16_t)(sTemp[SENSOR_UFH_POST_TMV] * 10);
    pkt.tempWorkshopAir = sFault[SENSOR_WORKSHOP_AIR] ? TEMP_FAULT : (int16_t)(sTemp[SENSOR_WORKSHOP_AIR] * 10);
    pkt.tempOutsideAir  = sFault[SENSOR_OUTSIDE_AIR]  ? TEMP_FAULT : (int16_t)(sTemp[SENSOR_OUTSIDE_AIR]  * 10);

    // Growatt data
    if (gd.valid) {
        pkt.pvString1W        = gd.pvStr1W;
        pkt.pvString2W        = gd.pvStr2W;
        pkt.pvTotalW          = gd.pvTotalW;
        pkt.pvExportW         = gd.pvExportW;
        pkt.gridImportW       = gd.gridImportW;
        pkt.loadW             = gd.loadW;
        pkt.battSOC           = gd.battSOC;
        pkt.battW             = gd.battW;
        pkt.battVoltage_dV    = gd.battVoltage_dV;
        pkt.gridVoltage_dV    = gd.gridVoltage_dV;
        pkt.gridFreq_cHz      = gd.gridFreq_cHz;
        pkt.energyTodayWh     = gd.energyTodayWh;
        pkt.inverterTemp_dC   = gd.inverterTemp_dC;
        pkt.inverterStatus    = gd.inverterStatus;
        pkt.inverterFaultCode = gd.inverterFaultCode;
    }

    pkt.solarPumpDutyPct = pumpTargetDuty;

    // Valve state bitmask
    if (ufhColdValve.isOpen)      pkt.valveStates |= VSTATE_UFH_COLD_OPEN;
    if (solarColdValve.isOpen)    pkt.valveStates |= VSTATE_SOLAR_COLD_OPEN;
    if (vacIsoValve.isOpen)       pkt.valveStates |= VSTATE_VAC_ISO_OPEN;
    if (digitalRead(PIN_FAN_FLAP)) pkt.valveStates |= VSTATE_FAN_FLAP_OPEN;

    pkt.workshopLocked   = workshopLocked ? 1 : 0;
    pkt.doorOpen         = (digitalRead(PIN_DOOR_REED)    == LOW) ? 1 : 0;
    pkt.pirActive        = (digitalRead(PIN_PIR)           == HIGH) ? 1 : 0;
    pkt.manualRelockOn   = (digitalRead(PIN_MANUAL_RELOCK) == LOW)  ? 1 : 0;
    pkt.vacPumpRunning   = (digitalRead(PIN_VAC_PUMP) == HIGH) ? 1 : 0;
    pkt.vacSensorFull    = (digitalRead(PIN_VAC_SENSOR) == LOW)  ? 1 : 0;
    pkt.winchState       = (uint8_t)winch.state();

    uint8_t reedFlags = 0;
    if (digitalRead(PIN_WINCH_REED_OPEN)  == LOW) reedFlags |= WREED_FULLY_OPEN;
    if (digitalRead(PIN_WINCH_REED_CLOSE) == HIGH) reedFlags |= WREED_FULLY_CLOSED;
    if (digitalRead(PIN_WINCH_REED_LOCK)  == HIGH) reedFlags |= WREED_MANUAL_LOCK;
    if (digitalRead(PIN_WINCH_SAFETY)     == LOW) reedFlags |= WREED_SAFETY_LIMIT;
    pkt.winchReedFlags = reedFlags;

    pkt.fanDutyPct         = fanCurrentDuty;
    pkt.fanFullTimerSecs   = fanFullTimerSecs;
    pkt.fanBaseTimerSecs   = fanBaseTimerSecs;
    pkt.ufhPumpRunning     = (digitalRead(PIN_UFH_PUMP) == HIGH) ? 1 : 0;
    pkt.ufhTargetReached   = ufhTargetReached ? 1 : 0;
    pkt.wFaultFlags        = wFaultFlags;
    pkt.requestTimeSync    = (!timeSynced || millis() - lastTimeSyncReceivedMs >= TIME_SYNC_INTERVAL_MS) ? 1 : 0;

    uint8_t frame[PKT_MAX_FRAME];
    uint16_t len = pktEncode(frame, sizeof(frame), PKT_DIR_WH, txSeqNum++,
                             &pkt, sizeof(pkt));

    digitalWrite(PIN_RS485_DE_LINK, HIGH);
    Serial1.write(frame, len);
    Serial1.flush();
    digitalWrite(PIN_RS485_DE_LINK, LOW);

    lastTxMs = millis();
}

void receiveHToWPacket() {
    unsigned long deadline = millis() + RS485_RX_TIMEOUT_MS;
    uint8_t  outDir; uint8_t outSeq;
    uint8_t *payload; uint16_t payLen;

    while (millis() < deadline) {
        if (!Serial1.available()) continue;
        uint8_t b = (uint8_t)Serial1.read();
        if (pktRx.feed(b, outDir, outSeq, payload, payLen)) {
            if (outDir != PKT_DIR_HW || payLen != sizeof(HToWPacket)) continue;

            // Sequence number miss detection
            uint8_t expectedSeq = (uint8_t)(lastRxSeq + 1);
            if (lastRxSeq != 255 && outSeq != expectedSeq) {
                // Missed one or more packets but this one is valid
            }
            lastRxSeq = outSeq;

            memcpy(&lastH, payload, sizeof(HToWPacket));
            hasHPacket = true;
            lastRxMs   = millis();
            missedPackets = 0;

            if (rs485CommsFault) {
                rs485CommsFault = false;
                clearFault(FAULT_W_RS485_COMMS);
            }

            // Apply time sync if provided
            if (lastH.timeSyncValid) {
                curHour   = lastH.syncHour;
                curMinute = lastH.syncMinute;
                curSecond = lastH.syncSecond;
                timeBaseMs = millis();
                timeSynced = true;
                lastTimeSyncReceivedMs = millis();
            }

            // Apply fan base speed from H
            if (lastH.fanBaseSpeedPct != fanBaseSpeedPct) {
                fanBaseSpeedPct = lastH.fanBaseSpeedPct;
                EEPROM.update(EEPROM_FAN_BASE_ADDR, fanBaseSpeedPct);
            }

            // Apply one-shot fan timer delta commands from H
            if (lastH.fanFullTimerDeltaHr != 0) {
                long v = (long)fanFullTimerSecs + lastH.fanFullTimerDeltaHr * 3600L;
                fanFullTimerSecs = (uint32_t)constrain(v, 0L, 24L * 3600L);
            }
            if (lastH.fanBaseTimerDeltaDay != 0) {
                long v = (long)fanBaseTimerSecs + lastH.fanBaseTimerDeltaDay * 86400L;
                fanBaseTimerSecs = (uint32_t)constrain(v, 0L, 30L * 86400L);
            }

            // Alert reset acknowledgement
            if (lastH.alertResetSeq != lastAlertResetSeq) {
                lastAlertResetSeq = lastH.alertResetSeq;
                // Clear all LED-persistent faults (condition must have cleared separately)
                // The fault conditions themselves are re-evaluated each loop
            }

            // Manual valve override from H
            if (lastH.overrideActive) {
                ufhColdValve.set(lastH.overrideValves & OVER_UFH_COLD_OPEN);
                solarColdValve.set(lastH.overrideValves & OVER_SOLAR_COLD_OPEN);
                if ((lastH.overrideValves & OVER_VAC_ISO_OPEN) != vacIsoValve.isOpen) {
                    vacIsoValve.request(lastH.overrideValves & OVER_VAC_ISO_OPEN);
                }
            }

            return;
        }
    }

    // Timeout — no valid packet received
    missedPackets++;
    if (missedPackets >= COMMS_FAULT_THRESHOLD && !rs485CommsFault) {
        rs485CommsFault = true;
        setFault(FAULT_W_RS485_COMMS);
    }
}

#ifdef DEBUG_SERIAL
// ============================================================
//  SERIAL DEBUG COMMAND FUNCTIONS
// ============================================================

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

static void dbgPrintTemp(uint8_t i, const __FlashStringHelper* name) {
    Serial.print(F("  ")); Serial.print(name); Serial.print(F(": "));
    if (sSimulate[i]) Serial.print(F("SIM "));
    if (sFault[i]) Serial.println(F("FAULT"));
    else           Serial.println(sTemp[i], 1);
}

static void dbgTemps() {
    dbgPrintTemp(SENSOR_SOLAR_HOT,    F("solar_hot"));
    dbgPrintTemp(SENSOR_SOLAR_COLD,   F("solar_cold"));
    dbgPrintTemp(SENSOR_UFH_SUPPLY,   F("ufh_supply"));
    dbgPrintTemp(SENSOR_UFH_POST_TMV, F("ufh_post_tmv"));
    dbgPrintTemp(SENSOR_WORKSHOP_AIR, F("workshop_air"));
    dbgPrintTemp(SENSOR_OUTSIDE_AIR,  F("outside_air"));
}

static void dbgValves() {
    Serial.print(F("  ufh_cold:   ")); Serial.println(ufhColdValve.isOpen   ? F("OPEN")   : F("CLOSED"));
    Serial.print(F("  solar_cold: ")); Serial.println(solarColdValve.isOpen ? F("OPEN")   : F("CLOSED"));
    Serial.print(F("  vac_iso:    ")); Serial.println(vacIsoValve.isOpen    ? F("OPEN")   : F("CLOSED"));
    Serial.print(F("  fan_flap:   ")); Serial.println(digitalRead(PIN_FAN_FLAP) ? F("OPEN")   : F("CLOSED"));
    Serial.print(F("  ufh_pump:   ")); Serial.println(digitalRead(PIN_UFH_PUMP) ? F("ON")    : F("off"));
    Serial.print(F("  wall_fan:   ")); Serial.println(digitalRead(PIN_WALL_FAN) ? F("ON")    : F("off"));
}

static void dbgFaults() {
    bool any = false;
    #define WF(m,n) if(hasFault(m)){Serial.println(F("  " n));any=true;}
    WF(FAULT_W_SOLAR_OVERHEAT_COLD,  "SOLAR_OVERHEAT_COLD")
    WF(FAULT_W_SOLAR_OVERHEAT_HOT,   "SOLAR_OVERHEAT_HOT")
    WF(FAULT_W_SOLAR_PUMP,           "SOLAR_PUMP")
    WF(FAULT_W_UFH_OVERHEAT,         "UFH_OVERHEAT")
    WF(FAULT_W_FROST_NOT_RECOVERING, "FROST_NOT_RECOVERING")
    WF(FAULT_W_VAC_PUMP_OVERTIME,    "VAC_PUMP_OVERTIME")
    WF(FAULT_W_GROWATT_COMMS,        "GROWATT_COMMS")
    WF(FAULT_W_INVERTER_FAULT,       "INVERTER_FAULT")
    WF(FAULT_W_RS485_COMMS,          "RS485_COMMS")
    WF(FAULT_W_FAN1,                 "FAN1")
    WF(FAULT_W_FAN2,                 "FAN2")
    WF(FAULT_W_WINCH_OVER_OPEN,      "WINCH_OVER_OPEN")
    WF(FAULT_W_SENSOR_SOLAR_HOT,     "SENSOR_SOLAR_HOT")
    WF(FAULT_W_SENSOR_SOLAR_COLD,    "SENSOR_SOLAR_COLD")
    WF(FAULT_W_SENSOR_UFH_SUPPLY,    "SENSOR_UFH_SUPPLY")
    WF(FAULT_W_SENSOR_UFH_POST_TMV,  "SENSOR_UFH_POST_TMV")
    WF(FAULT_W_SENSOR_WORKSHOP_AIR,  "SENSOR_WORKSHOP_AIR")
    WF(FAULT_W_SENSOR_OUTSIDE_AIR,   "SENSOR_OUTSIDE_AIR")
    WF(FAULT_W_SENSOR_INVERTER_TEMP, "SENSOR_INVERTER_TEMP")
    #undef WF
    if (!any) Serial.println(F("  none"));
}

static void dbgMode() {
    Serial.print(F("  mode:        "));
    if (!hasHPacket) Serial.println(F("unknown (no H packet)"));
    else             Serial.println(lastH.systemMode == MODE_WINTER ? F("WINTER") : F("SUMMER"));
    Serial.print(F("  morning:     ")); Serial.println(hasHPacket && lastH.morningHeatActive ? F("active") : F("off"));
    Serial.print(F("  solar_pump:  ")); Serial.print(solarPumpActive ? F("ON") : F("OFF"));
    Serial.print(F(" duty=")); Serial.println(pumpTargetDuty);
    Serial.print(F("  ufh_state:   ")); Serial.println(ufhState);
    Serial.print(F("  ufh_lockout: ")); Serial.println(ufhHardLockout ? F("YES") : F("no"));
    Serial.print(F("  vac_state:   ")); Serial.println(vacState);
    Serial.print(F("  workshop:    ")); Serial.println(workshopLocked ? F("locked") : F("unlocked"));
    Serial.print(F("  rs485:       ")); Serial.println(rs485CommsFault ? F("FAULT") : F("ok"));
    Serial.print(F("  time_synced: ")); Serial.println(timeSynced ? F("yes") : F("no"));
}

static void dbgPump() {
    Serial.print(F("  duty:      ")); Serial.println(pumpTargetDuty);
    Serial.print(F("  output:    ")); Serial.println(pumpOutputState ? F("ON") : F("OFF"));
    Serial.print(F("  current_A: ")); Serial.println(solarPumpCurrentA, 3);
    Serial.print(F("  active:    ")); Serial.println(solarPumpActive ? F("yes") : F("no"));
}

static void dbgFans() {
    Serial.print(F("  duty:      ")); Serial.println(fanCurrentDuty);
    Serial.print(F("  base_pct:  ")); Serial.println(fanBaseSpeedPct);
    Serial.print(F("  full_tmr:  ")); Serial.print(fanFullTimerSecs); Serial.println(F("s"));
    Serial.print(F("  base_tmr:  ")); Serial.print(fanBaseTimerSecs); Serial.println(F("s"));
    Serial.print(F("  fan1_rpm:  ")); Serial.println(fan1RPM);
    Serial.print(F("  fan2_rpm:  ")); Serial.println(fan2RPM);
}

static void dbgSecurity() {
    Serial.print(F("  locked:     ")); Serial.println(workshopLocked ? F("yes") : F("no"));
    Serial.print(F("  door:       ")); Serial.println((digitalRead(PIN_DOOR_REED)       == LOW)  ? F("OPEN")   : F("closed"));
    Serial.print(F("  pir:        ")); Serial.println((digitalRead(PIN_PIR)              == HIGH) ? F("ACTIVE") : F("clear"));
    Serial.print(F("  winch_cls:  ")); Serial.println((digitalRead(PIN_WINCH_REED_CLOSE) == HIGH)  ? F("CLOSED") : F("open"));
    Serial.print(F("  winch_lock: ")); Serial.println((digitalRead(PIN_WINCH_REED_LOCK)  == HIGH)  ? F("LOCKED") : F("clear"));
    Serial.print(F("  alarm:      ")); Serial.println(alarmActive  ? F("ACTIVE") : F("off"));
    Serial.print(F("  buzzer:     ")); Serial.println(buzzerActive ? F("on")     : F("off"));
    if (simDoorActive)      { Serial.print(F("  SIM door=")); Serial.println(simDoorVal); }
    if (simPIRActive)       { Serial.print(F("  SIM pir=")); Serial.println(simPIRVal); }
    if (simWinchClsActive)  { Serial.print(F("  SIM winch_cls=")); Serial.println(simWinchClsVal); }
    if (simWinchLockActive) { Serial.print(F("  SIM winch_lock=")); Serial.println(simWinchLockVal); }
}

static void dbgGrowatt() {
    if (!gd.valid) { Serial.println(F("  no Growatt data")); return; }
    Serial.print(F("  pv1=")); Serial.print(gd.pvStr1W);
    Serial.print(F("W  pv2=")); Serial.print(gd.pvStr2W);
    Serial.print(F("W  tot=")); Serial.print(gd.pvTotalW); Serial.println(F("W"));
    Serial.print(F("  export=")); Serial.print(gd.pvExportW);
    Serial.print(F("W  import=")); Serial.print(gd.gridImportW); Serial.println(F("W"));
    Serial.print(F("  batt_soc=")); Serial.print(gd.battSOC);
    Serial.print(F("%  batt_w=")); Serial.print(gd.battW); Serial.println(F("W"));
    Serial.print(F("  grid=")); Serial.print(gd.gridVoltage_dV / 10.0f, 1);
    Serial.print(F("V  freq=")); Serial.print(gd.gridFreq_cHz / 100.0f, 2); Serial.println(F("Hz"));
    Serial.print(F("  inv_status=")); Serial.print(gd.inverterStatus);
    Serial.print(F("  fault_code=")); Serial.println(gd.inverterFaultCode);
}

static void dbgSet(char* key, char* val) {
    float fval = atof(val);
    uint8_t si = 255;
    if      (!strcmp_P(key, PSTR("solar_hot")))    si = SENSOR_SOLAR_HOT;
    else if (!strcmp_P(key, PSTR("solar_cold")))   si = SENSOR_SOLAR_COLD;
    else if (!strcmp_P(key, PSTR("ufh_supply")))   si = SENSOR_UFH_SUPPLY;
    else if (!strcmp_P(key, PSTR("ufh_post_tmv"))) si = SENSOR_UFH_POST_TMV;
    else if (!strcmp_P(key, PSTR("workshop_air"))) si = SENSOR_WORKSHOP_AIR;
    else if (!strcmp_P(key, PSTR("outside_air")))  si = SENSOR_OUTSIDE_AIR;

    if (si < NUM_SENSORS) {
        if (fval >= 999.0f) { sSimulate[si] = false; Serial.println(F("cleared")); }
        else { sSimulate[si] = true; sSim[si] = fval; Serial.println(F("ok")); }
        return;
    }
    if (!strcmp_P(key, PSTR("pir")))             { simPIRActive       = true;  simPIRVal       = (fval != 0); Serial.println(F("ok")); return; }
    if (!strcmp_P(key, PSTR("door")))            { simDoorActive      = true;  simDoorVal      = (fval != 0); Serial.println(F("ok")); return; }
    if (!strcmp_P(key, PSTR("winch_cls")))       { simWinchClsActive  = true;  simWinchClsVal  = (fval != 0); Serial.println(F("ok")); return; }
    if (!strcmp_P(key, PSTR("winch_lock")))      { simWinchLockActive = true;  simWinchLockVal = (fval != 0); Serial.println(F("ok")); return; }
    if (!strcmp_P(key, PSTR("pir_clear")))       { simPIRActive       = false; Serial.println(F("cleared")); return; }
    if (!strcmp_P(key, PSTR("door_clear")))      { simDoorActive      = false; Serial.println(F("cleared")); return; }
    if (!strcmp_P(key, PSTR("winch_cls_clear"))) { simWinchClsActive  = false; Serial.println(F("cleared")); return; }
    if (!strcmp_P(key, PSTR("winch_lock_clear"))){ simWinchLockActive = false; Serial.println(F("cleared")); return; }
    Serial.print(F("unknown key: ")); Serial.println(key);
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
        Serial.println(F("temps  valves  faults  mode  status  pump  fans  security  growatt"));
        Serial.println(F("scan  (1-Wire address discovery)"));
        Serial.println(F("set <sensor> <val>  (val=999 clears sim)"));
        Serial.println(F("  sensors: solar_hot solar_cold ufh_supply ufh_post_tmv workshop_air outside_air"));
        Serial.println(F("set pir|door|winch_cls|winch_lock <0|1>"));
        Serial.println(F("set pir_clear|door_clear|winch_cls_clear|winch_lock_clear 0"));
    }
    else if (!strcmp_P(cmd, PSTR("temps")))    dbgTemps();
    else if (!strcmp_P(cmd, PSTR("valves")))   dbgValves();
    else if (!strcmp_P(cmd, PSTR("faults")))   dbgFaults();
    else if (!strcmp_P(cmd, PSTR("mode")))     dbgMode();
    else if (!strcmp_P(cmd, PSTR("status")))   { dbgMode(); dbgTemps(); dbgFaults(); }
    else if (!strcmp_P(cmd, PSTR("pump")))     dbgPump();
    else if (!strcmp_P(cmd, PSTR("fans")))     dbgFans();
    else if (!strcmp_P(cmd, PSTR("security"))) dbgSecurity();
    else if (!strcmp_P(cmd, PSTR("growatt")))  dbgGrowatt();
    else if (!strcmp_P(cmd, PSTR("scan")))     dbgScan();
    else if (!strcmp_P(cmd, PSTR("set")))      dbgSet(arg1, arg2);
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
//  POWER-UP SAFE STATE SEQUENCE
//  Strictly sequential — each step completes before next.
//  ~60s total before heating logic starts.
// ============================================================

void powerUpSafeState() {
    // Step 1: Door lock — send 1s lock pulse
    doorLock.request(false); // false = close = lock
    unsigned long t = millis();
    while (millis() - t < LOCK_PULSE_MS + HBRIDGE_DEAD_MS + 200) {
        doorLock.update();
        wdt_reset();
    }
    workshopLocked = true;

    // Step 2: All tank valves at H are driven by H controller, not us.
    // W-side valves: vacuum isolation valve → close
    vacIsoValve.request(false);
    t = millis();
    while (millis() - t < VALVE_PULSE_MS + HBRIDGE_DEAD_MS + 200) {
        vacIsoValve.update();
        wdt_reset();
    }

    // Step 3: UFH cold valve → close (direction relay off = NC = Wire B = close)
    ufhColdValve.setClose();

    // Step 4: Solar cold valve → close
    solarColdValve.setClose();

    // Wait VALVE_POWERUP_WAIT_MS for auto-cutout to trip on both valves
    t = millis();
    while (millis() - t < VALVE_POWERUP_WAIT_MS) {
        wdt_reset();
        delay(100); // safe to use delay here — we're in init, not main loop
    }

    // Step 5: Ensure all relay outputs in a known safe state
    digitalWrite(PIN_UFH_PUMP,       LOW);
    digitalWrite(PIN_WALL_FAN,       LOW);
    digitalWrite(PIN_FAN_FLAP,       LOW);
    digitalWrite(PIN_EXT_LIGHTS,     LOW);
    digitalWrite(PIN_VAC_PUMP,       LOW);
    digitalWrite(PIN_ALARM_SOUNDER,  LOW);
    digitalWrite(PIN_BUZZER_SIGNAL,  LOW);
    setSolarPumpDuty(0);
    setFanPWM(0);

    // Winch: no action at power-up — state read from reed switches
    winch.begin();
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
    wdt_disable(); // Disable watchdog during init
#ifdef DEBUG_SERIAL
    Serial.begin(115200);
#endif

    // Output pins
    pinMode(PIN_UFH_COLD_DIR,    OUTPUT); digitalWrite(PIN_UFH_COLD_DIR,    LOW);
    pinMode(PIN_SOLAR_COLD_DIR,  OUTPUT); digitalWrite(PIN_SOLAR_COLD_DIR,  LOW);
    pinMode(PIN_UFH_PUMP,        OUTPUT); digitalWrite(PIN_UFH_PUMP,        LOW);
    pinMode(PIN_WALL_FAN,        OUTPUT); digitalWrite(PIN_WALL_FAN,        LOW);
    pinMode(PIN_FAN_FLAP,        OUTPUT); digitalWrite(PIN_FAN_FLAP,        LOW);
    pinMode(PIN_DOOR_LOCK_A,     OUTPUT); digitalWrite(PIN_DOOR_LOCK_A,     LOW);
    pinMode(PIN_EXT_LIGHTS,      OUTPUT); digitalWrite(PIN_EXT_LIGHTS,      LOW);
    pinMode(PIN_WINCH_DIR_OPEN,  OUTPUT); digitalWrite(PIN_WINCH_DIR_OPEN,  LOW);
    pinMode(PIN_WINCH_POWER,     OUTPUT); digitalWrite(PIN_WINCH_POWER,     LOW);
    pinMode(PIN_WINCH_DIR_CLOSE, OUTPUT); digitalWrite(PIN_WINCH_DIR_CLOSE, LOW);
    pinMode(PIN_MIDPOINT_LED,    OUTPUT); digitalWrite(PIN_MIDPOINT_LED,    LOW);
    pinMode(PIN_RS485_DE_LINK,   OUTPUT); digitalWrite(PIN_RS485_DE_LINK,   LOW);
    pinMode(PIN_RS485_DE_GROWATT,OUTPUT); digitalWrite(PIN_RS485_DE_GROWATT,LOW);
    pinMode(PIN_VAC_ISO_OPEN,    OUTPUT); digitalWrite(PIN_VAC_ISO_OPEN,    LOW);
    pinMode(PIN_VAC_ISO_CLOSE,   OUTPUT); digitalWrite(PIN_VAC_ISO_CLOSE,   LOW);
    pinMode(PIN_DOOR_LOCK_B,     OUTPUT); digitalWrite(PIN_DOOR_LOCK_B,     LOW);
    pinMode(PIN_BUZZER_SIGNAL,   OUTPUT); digitalWrite(PIN_BUZZER_SIGNAL,   LOW);
    pinMode(PIN_HEN_DOOR_OPEN,   OUTPUT); digitalWrite(PIN_HEN_DOOR_OPEN,   LOW);
    pinMode(PIN_HEN_DOOR_CLOSE,  OUTPUT); digitalWrite(PIN_HEN_DOOR_CLOSE,  LOW);
    pinMode(PIN_VAC_PUMP,        OUTPUT); digitalWrite(PIN_VAC_PUMP,        LOW);
    pinMode(PIN_ALARM_SOUNDER,   OUTPUT); digitalWrite(PIN_ALARM_SOUNDER,   LOW);
    pinMode(PIN_SOLAR_PUMP,      OUTPUT); digitalWrite(PIN_SOLAR_PUMP,      LOW);

    // Input pins
    pinMode(PIN_PIR,             INPUT);  // voltage divider, no pull-up
    pinMode(PIN_DOOR_HANDLE,     INPUT_PULLUP);
    pinMode(PIN_DOOR_REED,       INPUT_PULLUP);
    pinMode(PIN_WINCH_REED_OPEN, INPUT_PULLUP);
    pinMode(PIN_WINCH_REED_CLOSE,INPUT_PULLUP);
    pinMode(PIN_WINCH_REED_LOCK, INPUT_PULLUP);
    pinMode(PIN_WINCH_SAFETY,    INPUT_PULLUP);
    pinMode(PIN_VAC_SENSOR,      INPUT_PULLUP);
    pinMode(PIN_UNLOCK_BTN,      INPUT);  // voltage divider, no pull-up
    pinMode(PIN_LIGHT_BTN,       INPUT);  // voltage divider, no pull-up
    pinMode(PIN_MANUAL_RELOCK,   INPUT_PULLUP);
    pinMode(PIN_FAN_BTN,         INPUT_PULLUP);
    pinMode(PIN_WIN_OPEN_BTN,    INPUT_PULLUP);
    pinMode(PIN_WIN_CLOSE_BTN,   INPUT_PULLUP);
    pinMode(PIN_FAN1_TACH,       INPUT_PULLUP); // external 10k also present
    pinMode(PIN_FAN2_TACH,       INPUT_PULLUP);

    // Fan tachometer PCINT (Port K: PK0=A8=PCINT16, PK1=A9=PCINT17)
    PCMSK2 |= (1 << PCINT16) | (1 << PCINT17);
    PCICR  |= (1 << PCIE2);

    // RS485 UARTs
    Serial1.begin(9600); // inter-controller link
    Serial2.begin(9600); // Growatt Modbus

    // DS18B20
    sensors.begin();
    sensors.setResolution(12);
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        sTemp[i] = NAN;
        sFault[i] = false;
        sFailCount[i] = 0;
        sGoodCount[i] = 0;
    }
#ifdef DEBUG_SERIAL
    memset(sSimulate, 0, sizeof(sSimulate));
    memset(sSim,      0, sizeof(sSim));
#endif

    // INA219
    ina219.begin();

    // Growatt Modbus
    growatt.begin(GROWATT_ADDR, Serial2);
    growatt.preTransmission(preTransmitGrowatt);
    growatt.postTransmission(postTransmitGrowatt);
    memset(&gd, 0, sizeof(gd));
    gd.valid = false;

    // Valve objects
    vacIsoValve.begin(PIN_VAC_ISO_OPEN, PIN_VAC_ISO_CLOSE, VALVE_PULSE_MS);
    doorLock.begin(PIN_DOOR_LOCK_A,     PIN_DOOR_LOCK_B,   LOCK_PULSE_MS);
    ufhColdValve.begin(PIN_UFH_COLD_DIR);
    solarColdValve.begin(PIN_SOLAR_COLD_DIR);

    // Fan PWM
    setupFanPWM();

    // Load fan base speed from EEPROM
    uint8_t storedBase = EEPROM.read(EEPROM_FAN_BASE_ADDR);
    if (storedBase <= 100) fanBaseSpeedPct = storedBase;

    // Packet receiver reset
    pktRx.reset();

    // Power-up safe state (~60s, watchdog disabled during this)
    powerUpSafeState();

    // Request time sync immediately
    lastTimeSyncRequestMs = 0; // forces request in first TX

    // Enable 8-second hardware watchdog
    wdt_enable(WDTO_8S);

#ifdef DEBUG_SERIAL
    Serial.println(F("W controller ready — type 'help' for commands"));
#endif

    // Start first sensor conversion
    startSensorConversion();
    fanLastTickMs = millis();
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

    // Fan speed timer button (D43): sets 8-hour full-speed timer
    static bool prevFanBtn = false;
    bool fanBtn = (digitalRead(PIN_FAN_BTN) == LOW);
    if (fanBtn && !prevFanBtn) {
        fanFullTimerSecs = 8UL * 3600UL;
    }
    prevFanBtn = fanBtn;

    // Start new sensor conversion every ~1.1s
    static unsigned long lastConvStartMs = 0;
    if (!sensorConvStarted && now - lastConvStartMs >= 1100UL) {
        startSensorConversion();
        lastConvStartMs = now;
    }
    readSensors();

    // Valve & winch state machines (non-blocking)
    vacIsoValve.update();
    doorLock.update();
    updateWinchInputs();

    // Security
    updateSecurity();

    // INA219 current sampling (during pump ON period)
    sampleSolarPumpCurrent();
    checkSolarPumpFault();

    // Heating logic (mode comes from H via RS485)
    SystemMode mode = hasHPacket ? (SystemMode)lastH.systemMode : MODE_WINTER;
    float tankBotC  = (hasHPacket && lastH.tempTankBot != TEMP_FAULT)
                       ? (float)lastH.tempTankBot / 10.0f : NAN;

    if (mode == MODE_WINTER) {
        updateWinterSolar(tankBotC); // NAN = RS485 stale; function handles this safely
    } else {
        updateSummerSolar();
    }
    updateUFHHeating();
    updateSolarPump();

    // Vacuum system
    updateVacuum();

    // Fan control
    updateFanRPM();
    updateFanControl();

    // Night cooling
    updateNightCooling();

    // Fire alarm
    updateFireAlarm();

    // Mid-point LED
    updateMidpointLED();

    // Growatt polling (independent timing on UART2)
    if (now - lastGrowattPollMs >= GROWATT_POLL_MS) {
        lastGrowattPollMs = now;
        pollGrowatt();
        wdt_reset();
    }

    // Inter-controller RS485 (every 250ms)
    if (now - lastTxMs >= INTER_CTRL_POLL_MS) {
        sendWToHPacket();
        receiveHToWPacket();
        wdt_reset();
    }
}
