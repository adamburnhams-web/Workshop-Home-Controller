// ============================================================
//  W Controller — Workshop Mega 2560
//  Controls: solar thermal, UFH pump/valves, vacuum system,
//            door lock, window winch, PC fans, external lights,
//            wall fan, Growatt inverter Modbus, inter-controller RS485
// ============================================================

#include <Arduino.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_INA219.h>
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

#define PIN_RS485_DE_LINK   40   // MAX485 DE/RE for inter-controller link (HIGH=TX)
#define PIN_RS485_DE_GROWATT 47   // MAX485 DE/RE for Growatt Modbus (HIGH=TX)
#define PIN_SOLAR_PUMP      44   // MOSFET gate via 120Ω: solar pump clocking output
#define PIN_FAN_PWM         45   // Timer5 OC5B 25kHz PWM: PC fan MOSFET gate

// Active-LOW relay boards: relay energises on LOW, de-energises on HIGH
#define RELAY_ON  LOW
#define RELAY_OFF HIGH

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
    { 0x28, 0x64, 0x27, 0x15, 0x00, 0x00, 0x00, 0x11 }, // solar cold     (sensor 2)
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

// INA219 — solar pump current limits (amps). Both sampled during ON pulse only.
static const float SOLAR_PUMP_MIN_CURRENT_A = 1.00f;
static const float SOLAR_PUMP_MAX_CURRENT_A = 2.00f;

// PC fan minimum duty before stall. Calibrate on-site.
static const uint8_t FAN_MIN_DUTY_PCT = 20; // TODO: calibrate

// UFH/solar cold valve 30-second power-up wait (conservative; verify during commissioning)
static const uint16_t VALVE_POWERUP_WAIT_MS = 30000;


// ============================================================
//  SYSTEM CONSTANTS
// ============================================================

#define TEMP_SCALE          10    // temps stored as int16 = °C × 10
#define SOLAR_HOT_MIN_C    180    // 18.0°C — winter solar trigger
#define SOLAR_DIFF_STOP_C   20    // 2.0°C — stop when hot-cold < this
#define WINTER_UFH_TARGET_C 200   // 20.0°C — winter solar hot-side target
#define FROST_THRESH_C       20   // 2.0°C — frost protection trigger
#define FROST_STOP_C         80   // 8.0°C — frost protection stop
#define UFH_PUMP_OFF_C      280   // 28.0°C — stop UFH pump when supply drops here
#define UFH_OVERHEAT_C      450   // 45.0°C — hard lockout trigger
#define SOLAR_OVERHEAT_COLD_DELTA 100 // 10.0°C above tank bottom
#define RELOCK_TIMEOUT_MS  300000UL  // 5 minutes
#define ALARM_DURATION_MS   60000UL  // 1-minute alarm auto-stop
#define HANDLE_ALERT_NIGHT_MIN_MS 10000UL
#define INTER_CTRL_POLL_MS  250UL
#define DS18B20_CONV_MS     750UL
#define RS485_RX_TIMEOUT_MS 150UL
#define COMMS_FAULT_THRESHOLD 20
#define FAN_RPM_FAULT_DELAY_MS 5000UL
#define VAC_PUMP_MAX_MS    1800000UL // 30 minutes
#define VAC_EXTRA_RUN_MS   300000UL  // 5 extra minutes after full vacuum
#define VALVE_PULSE_MS      7000UL   // H-bridge valve travel time
#define LOCK_PULSE_MS       1000UL   // door lock pulse
#define HBRIDGE_DEAD_MS      200UL   // dead-time between relay changes
#define FAN_FLAP_OPEN_MS   30000UL   // motorized damper travel time; calibrate on-site
#define TIME_SYNC_INTERVAL_MS 3600000UL // 1 hour
#define EEPROM_FAN_BASE_ADDR 0       // EEPROM address for fan base speed

// ============================================================
//  GLOBAL OBJECTS
// ============================================================

OneWire        oneWire(PIN_ONE_WIRE);
DallasTemperature sensors(&oneWire);
Adafruit_INA219 ina219;   // address 0x40

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
    int16_t  pv1W;
    int16_t  pv2W;
    int16_t  pvOutputW;
    int16_t  loadW;
    int16_t  pvExportW;
    int16_t  gridImportW;
    int16_t  battVoltage_dV;
    uint8_t  battSocPct;
    int16_t  battChargeW;       // positive = charging, negative = discharging
    int16_t  dailyGenDeciKwh;
    unsigned long lastGoodMs;
    bool     valid;
} growatt;

bool    simGrowattActive  = false;
int16_t simGPvOutW        = 0;
int16_t simGPvExportW     = 0;
uint8_t simGBattSocPct    = 0;
int16_t simGBattChargeW   = 0;

bool    simHtrPctActive   = false;
uint8_t simHtrPctVal      = 0;

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

// Clocking: zone 1 (1-20%)  fixes on=200ms,  shrinks off 19800→800ms.
//           zone 2 (20-50%) fixes off=800ms,  grows   on  200→800ms.
//           zone 3 (50-100%)fixes on=800ms,   shrinks off 800→0ms.
//           100% = continuous.

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
        pinMode(a, OUTPUT); digitalWrite(a, RELAY_OFF);
        pinMode(b, OUTPUT); digitalWrite(b, RELAY_OFF);
    }

    void request(bool open) {
        // Interrupt mid-pulse: drop to dead-time immediately
        if (phase == HBP_PULSING) {
            digitalWrite(pinA, RELAY_OFF);
            digitalWrite(pinB, RELAY_OFF);
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
                    digitalWrite(pendingOpen ? pinA : pinB, RELAY_ON);
                    phaseStartMs = now;
                    phase = HBP_PULSING;
                }
                break;
            case HBP_PULSING:
                if (now - phaseStartMs >= pulseDurationMs) {
                    digitalWrite(pinA, RELAY_OFF);
                    digitalWrite(pinB, RELAY_OFF);
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
    void setOpen()  { digitalWrite(pin, RELAY_ON);  isOpen = true; }
    void setClose() { digitalWrite(pin, RELAY_OFF); isOpen = false; }
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
        pinMode(PIN_WINCH_DIR_OPEN,  OUTPUT); digitalWrite(PIN_WINCH_DIR_OPEN,  RELAY_OFF);
        pinMode(PIN_WINCH_DIR_CLOSE, OUTPUT); digitalWrite(PIN_WINCH_DIR_CLOSE, RELAY_OFF);
        pinMode(PIN_WINCH_POWER,     OUTPUT); digitalWrite(PIN_WINCH_POWER,     RELAY_OFF);
    }

    // Immediately cut power — no 1s pause. Used by safety triggers.
    void immediateStop() {
        digitalWrite(PIN_WINCH_DIR_OPEN,  RELAY_OFF);
        digitalWrite(PIN_WINCH_DIR_CLOSE, RELAY_OFF);
        digitalWrite(PIN_WINCH_POWER,     RELAY_OFF);
        phase     = WP_IDLE;
        activeDir = WINCH_STOP;
    }

    // Commanded stop/open/close. Always routes through 1s pause.
    void request(WinchState dir) {
        if (dir == WINCH_STOP)                              { immediateStop(); return; }
        if (dir == WINCH_OPEN  && openLockout)              return;
        if (dir == WINCH_CLOSE && closeLockout)             return;

        // Cut current movement / ensure all relays off before pause
        digitalWrite(PIN_WINCH_DIR_OPEN,  RELAY_OFF);
        digitalWrite(PIN_WINCH_DIR_CLOSE, RELAY_OFF);
        digitalWrite(PIN_WINCH_POWER,     RELAY_OFF);
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
            if (pendingDir == WINCH_OPEN)  digitalWrite(PIN_WINCH_DIR_OPEN,  RELAY_ON);
            else                            digitalWrite(PIN_WINCH_DIR_CLOSE, RELAY_ON);
            digitalWrite(PIN_WINCH_POWER, RELAY_ON);
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
bool solarPumpActive  = false;
bool solarActiveEver  = false; // for vacuum system "heating active" check
bool solarDumpUFHOn   = false; // UFH pump running for emergency solar dump; sent to H
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

uint16_t   rs485RxGood         = 0;  // total valid H packets received
uint16_t   rs485RxMiss         = 0;  // total timeout misses
uint16_t   rs485RxBadFrame     = 0;  // decoded frames with wrong dir/len

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
bool  simPumpCurrentActive = false; float simPumpCurrentVal = 0.0f;
#endif // DEBUG_SERIAL


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
    if (simPumpCurrentActive) solarPumpCurrentA = simPumpCurrentVal;
    pumpCurrentSampled = true;
}

// ============================================================
//  MODBUS CRC16
// ============================================================

static uint16_t sdmCRC16(const uint8_t* buf, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

// ============================================================
//  GROWATT PRODUCTION POLLING  (non-blocking state machine)
//  Phase 0: FC04 @0  x11 → pv1W (r6), pv2W (r10)
//  Phase 1: FC04 @35 x7  → pvOutputW (r36), loadW (r41)
//  Phase 2: FC04 @1000 x25 → batt, gridImport, pvExport
//  Phase 3: FC04 @55 x3  → dailyGenDeciKwh (r57 ×0.1kWh)
//  Each call either sends a request or collects bytes; never blocks >5ms.
// ============================================================

#define GROWATT_STALE_MS  120000UL

static inline int16_t reg16(const uint8_t* data, uint8_t regOffset) {
    return (int16_t)(((uint16_t)data[regOffset * 2] << 8) | data[regOffset * 2 + 1]);
}

void growattPoll() {
#ifdef DEBUG_SERIAL
    if (simGrowattActive) {
        growatt.pvOutputW   = simGPvOutW;
        growatt.pv1W        = simGPvOutW;
        growatt.pvExportW   = simGPvExportW;
        growatt.battSocPct  = simGBattSocPct;
        growatt.battChargeW = simGBattChargeW;
        growatt.lastGoodMs  = millis();
        growatt.valid       = true;
        clearFault(FAULT_W_GROWATT_COMMS);
        return;
    }
#endif
    static uint8_t       phase      = 0;
    static bool          inRecv     = false;
    static unsigned long sentMs     = 0;
    static unsigned long lastByteMs = 0;
    static bool          rxStarted  = false;
    static uint8_t       rxBuf[96];
    static uint8_t       rxN        = 0;

    if (!inRecv) {
        // SEND: drain stale bytes (5ms), then send request
        { unsigned long t = millis(); while (millis() - t < 5) if (Serial2.available()) Serial2.read(); }

        static const uint16_t phaseReg[4]   = {    0,  35, 1000, 55 };
        static const uint16_t phaseCount[4] = {   11,   7,   25,  3 };
        uint16_t startReg = phaseReg[phase];
        uint16_t count    = phaseCount[phase];

        uint8_t req[8];
        req[0] = 0x01; req[1] = 0x04;
        req[2] = (uint8_t)(startReg >> 8); req[3] = (uint8_t)startReg;
        req[4] = (uint8_t)(count >> 8);    req[5] = (uint8_t)count;
        uint16_t crc = sdmCRC16(req, 6);
        req[6] = (uint8_t)crc; req[7] = (uint8_t)(crc >> 8);

        digitalWrite(PIN_RS485_DE_GROWATT, HIGH);
        delayMicroseconds(200);
        Serial2.write(req, 8); Serial2.flush();
        delayMicroseconds(200);
        digitalWrite(PIN_RS485_DE_GROWATT, LOW);

        sentMs     = millis();
        lastByteMs = millis();
        rxStarted  = false;
        rxN        = 0;
        inRecv     = true;
        return;
    }

    // RECV: collect bytes this iteration, return if not done
    while (Serial2.available() && rxN < sizeof(rxBuf)) {
        uint8_t b = (uint8_t)Serial2.read();
        if (!rxStarted && b == 0x00) continue;
        rxStarted = true;
        rxBuf[rxN++] = b;
        lastByteMs = millis();
    }

    bool timedOut   = (millis() - sentMs > 120);
    bool packetDone = rxStarted && (millis() - lastByteMs >= 5);
    if (!timedOut && !packetDone) return;

    // Process response
    bool ok = false;
    if (rxN >= 5 && rxBuf[0] == 0x01 && !(rxBuf[1] & 0x80)) {
        uint16_t rxCrc = sdmCRC16(rxBuf, rxN - 2);
        if ((uint8_t)rxCrc == rxBuf[rxN-2] && (uint8_t)(rxCrc >> 8) == rxBuf[rxN-1]) {
            uint8_t* data = rxBuf + 3;
            switch (phase) {
                case 0:
                    growatt.pv1W = (int16_t)((uint16_t)reg16(data, 6) / 10);
                    growatt.pv2W = (int16_t)((uint16_t)reg16(data, 10) / 10);
                    ok = true; break;
                case 1:
                    growatt.pvOutputW = (int16_t)((uint16_t)reg16(data, 1) / 10);
                    growatt.loadW     = (int16_t)((uint16_t)reg16(data, 6) / 10);
                    ok = true; break;
                case 2:
                    growatt.battVoltage_dV = reg16(data, 13);
                    growatt.battSocPct     = (uint8_t)reg16(data, 14);
                    growatt.gridImportW    = (int16_t)((uint16_t)reg16(data, 16) / 10);
                    growatt.pvExportW      = (int16_t)((uint16_t)reg16(data, 24) / 10);
                    { int16_t dis = (int16_t)((uint16_t)reg16(data, 10) / 10);
                      int16_t chg = (int16_t)((uint16_t)reg16(data, 12) / 10);
                      growatt.battChargeW = chg - dis; }
                    ok = true; break;
                case 3:
                    growatt.dailyGenDeciKwh = reg16(data, 2);
                    ok = true; break;
            }
        }
    }

    if (ok) {
        growatt.lastGoodMs = millis();
        growatt.valid      = true;
        clearFault(FAULT_W_GROWATT_COMMS);
    }

    if (growatt.valid && millis() - growatt.lastGoodMs > GROWATT_STALE_MS) {
        growatt.valid = false;
        setFault(FAULT_W_GROWATT_COMMS);
    }

    if (++phase >= 4) phase = 0;
    inRecv = false;
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

    // Zone 1 (1–20%):  on=200ms fixed,  off shrinks 19800→800ms linearly.
    // Zone 2 (20–50%): off=800ms fixed, on  grows  200→800ms via duty/(1−duty).
    // Zone 3 (50–100%):on=800ms fixed,  off shrinks 800→0ms  via (1−duty)/duty.
    uint32_t onMs, offMs;
    if (pumpTargetDuty <= 20) {
        onMs  = 200UL;
        offMs = 19800UL - 1000UL * (pumpTargetDuty - 1);
    } else if (pumpTargetDuty <= 50) {
        onMs  = (uint32_t)pumpTargetDuty * 800UL / (100 - pumpTargetDuty);
        offMs = 800UL;
    } else {
        onMs  = 800UL;
        offMs = 800UL * (100 - pumpTargetDuty) / pumpTargetDuty;
    }
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

// Minimum pump % when heater is on. Proportional to heater power;
// hotter outlet (heater working hard) slightly raises the floor.
// Calibrate divisor and offset on-site.
static uint8_t heaterMinPumpPct(uint8_t heaterPct, float heaterOutC) {
    uint8_t base = max(2, heaterPct / 5);
    if (!isnan(heaterOutC) && heaterOutC > 70.0f)
        base = max(base, (uint8_t)((heaterOutC - 70.0f) / 5.0f + 2));
    return min(base, (uint8_t)25);
}

// Pump duty % given solar hot-side temperature vs target.
// heaterPct==0 → heater off branch; otherwise heater-on branch applies a floor.
static uint8_t calcPumpDuty(float hot, float targetC,
                             uint8_t heaterPct, float heaterOutC) {
    if (heaterPct == 0) {
        // Heater off: 1% far below target, slow ramp to 100% above target
        if (hot < targetC - 8.0f) return 1;
        if (hot < targetC - 2.0f) {
            // 2% → 4% linear over 6°C (target-8 to target-2)
            float t = (hot - (targetC - 8.0f)) / 6.0f;
            return (uint8_t)(2.0f + t * 2.0f);
        }
        if (hot >= targetC + 2.0f) return 100;
        // 4% → 100% linear over 4°C (target-2 to target+2)
        float t = (hot - (targetC - 2.0f)) / 4.0f;
        return (uint8_t)(4.0f + t * 96.0f);
    } else {
        // Heater on: hold floor below target-2, ramp floor→100% in upper 4°C band
        uint8_t minPct = heaterMinPumpPct(heaterPct, heaterOutC);
        if (hot >= targetC + 2.0f) return 100;
        if (hot < targetC - 2.0f)  return minPct;
        float t = (hot - (targetC - 2.0f)) / 4.0f;
        return (uint8_t)(minPct + t * (100.0f - minPct));
    }
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
                digitalWrite(PIN_VAC_PUMP, RELAY_ON);
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
                digitalWrite(PIN_VAC_PUMP, RELAY_OFF);
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
                digitalWrite(PIN_VAC_PUMP, RELAY_OFF);
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
        setSolarPumpDuty(0); // starts at clocking minimum in updateSolarPump
        solarPumpActive  = true;
        solarActiveEver  = true;
    }

    // Speed ramp: hot side targets 20°C
    uint8_t htrPct  = hasHPacket ? lastH.heaterPowerPct : 0;
    float   htrOutC = (hasHPacket && lastH.tempHeaterOut != TEMP_FAULT)
                      ? (float)lastH.tempHeaterOut / 10.0f : NAN;
    uint8_t duty = calcPumpDuty(hot, (float)WINTER_UFH_TARGET_C / 10.0f, htrPct, htrOutC);
    setSolarPumpDuty(duty);

    // Overspeed fault: pump above minimum for > 10s AND cold > tankBottom + 10°C
    // If tank bottom is stale: skip fault check
    bool pumpAboveMin = (duty > 1);
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
            digitalWrite(PIN_UFH_PUMP, RELAY_ON);
            solarDumpActive  = true;
            solarDumpUFHOn   = true;
        }
        setSolarPumpDuty(100);
        solarPumpActive = false;
        return;
    }

    if (solarDumpActive) {
        // Sensor fault cleared — stop the dump-mode UFH pump and exit dump state
        digitalWrite(PIN_UFH_PUMP, RELAY_OFF);
        solarDumpActive = false;
        solarDumpUFHOn  = false;
    }

    float hot     = sTemp[SENSOR_SOLAR_HOT];
    float cold    = sTemp[SENSOR_SOLAR_COLD];
    // pvActive goes true immediately when PV >= 200W; goes false only after 5 continuous minutes below 200W.
    // Holds last state during Growatt comms outage. End-of-day also requires arrayLow (panels < 40°C).
    static bool          lastPvActive    = false;
    static unsigned long pvBelowStartMs  = 0;
    if (growatt.valid) {
        if ((growatt.pv1W + growatt.pv2W) >= 200) {
            lastPvActive   = true;
            pvBelowStartMs = 0;
        } else {
            if (pvBelowStartMs == 0) pvBelowStartMs = millis();
            if (millis() - pvBelowStartMs >= 300000UL) lastPvActive = false;
        }
    }
    bool pvActive = lastPvActive;

    // Summer solar startup condition (PV export >= 500W, solar >= 50°C, or heater powered)
    bool startTrigger = (growatt.valid && growatt.pvExportW >= 500)
                         || hot >= 50.0f || cold >= 50.0f
                         || (hasHPacket && lastH.heaterPowerPct > 0);

    if (!startTrigger && !solarPumpActive) return;

    if (startTrigger && !summerSeqDone && summerPhase == SUMPH_IDLE) {
        summerPhase = SUMPH_CIRCULATE_TOP;
        solarColdValve.setOpen();
        ufhColdValve.setClose(); // UFH cold stays closed in summer; opens only for fault dump
        setSolarPumpDuty(1);
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
            solarPumpActive   = true;
            solarActiveEver   = true;
        }
    }

    uint8_t htrPct  = hasHPacket ? lastH.heaterPowerPct : 0;
    float   htrOutC = (hasHPacket && lastH.tempHeaterOut != TEMP_FAULT)
                      ? (float)lastH.tempHeaterOut / 10.0f : NAN;
    uint8_t duty = calcPumpDuty(hot, solarTarget, htrPct, htrOutC);

    bool tankBotStale = !hasHPacket || (millis() - lastRxMs > 10000UL)
                         || lastH.tempTankBot == TEMP_FAULT;
    float tankBotC    = tankBotStale ? 0.0f : (float)lastH.tempTankBot / 10.0f;

    // Allow full stop only when both solar pipes are below tank bottom,
    // heater off, sequence done, 2-port on heater side, and solar far below target.
    // In all other cases hold at minimum 1%.
    if (duty <= 1 && solarPumpActive && htrPct == 0) {
        bool bothBelowTankBot = !tankBotStale
                                && !sFault[SENSOR_SOLAR_HOT] && !sFault[SENSOR_SOLAR_COLD]
                                && hot < tankBotC && cold < tankBotC;
        if (bothBelowTankBot) {
            bool twoPortHeater = hasHPacket && lastH.twoPortHeaterSide;
            bool solarFarBelow = !sFault[SENSOR_SOLAR_HOT] && !sFault[SENSOR_SOLAR_COLD]
                                 && hot  < solarTarget - 15.0f
                                 && cold < solarTarget - 15.0f;
            if (!summerSeqDone || !(twoPortHeater && solarFarBelow)) duty = 1;
            // else: all suppression conditions met → allow duty = 0 (pump stops)
        } else {
            duty = 1;
        }
    }
    setSolarPumpDuty(duty);

    // Overspeed overheat fault (same as winter)
    bool pumpAboveMin = (duty > 1);
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

    // End-of-day abort (all phases): PV < 200W + differential < 6°C + both pipes below 40°C.
    // Sensor faults default to abort-condition-met (cannot confirm heat present).
    bool pipeCool = sFault[SENSOR_SOLAR_HOT] || sFault[SENSOR_SOLAR_COLD]
                     || (hot - cold) < 6.0f;
    bool arrayLow = sFault[SENSOR_SOLAR_HOT] || sFault[SENSOR_SOLAR_COLD]
                     || (hot < 40.0f && cold < 40.0f);
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
            digitalWrite(PIN_UFH_PUMP, RELAY_OFF);
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
            digitalWrite(PIN_UFH_PUMP, RELAY_OFF);
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
                digitalWrite(PIN_UFH_PUMP, RELAY_ON);
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
                digitalWrite(PIN_UFH_PUMP, RELAY_OFF);
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
    // D10 is shared with midpoint LED wire: mask button when LED relay is energised (RELAY_ON=LOW)
    bool unlockBtn    = (digitalRead(PIN_UNLOCK_BTN)    == HIGH)
                     && (digitalRead(PIN_MIDPOINT_LED) == RELAY_OFF);
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
        digitalWrite(PIN_EXT_LIGHTS, extLightsOn ? RELAY_ON : RELAY_OFF);
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
        digitalWrite(PIN_ALARM_SOUNDER, RELAY_ON);
    }
    if (alarmActive) {
        if (!pirActive || now - alarmStartMs >= ALARM_DURATION_MS) {
            // Auto-stop after 1 minute; stop immediately if PIR clears
            if (now - alarmStartMs >= ALARM_DURATION_MS) {
                digitalWrite(PIN_ALARM_SOUNDER, RELAY_OFF);
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

    digitalWrite(PIN_BUZZER_SIGNAL, buzzerActive ? RELAY_ON : RELAY_OFF);
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
        digitalWrite(PIN_WALL_FAN, RELAY_OFF);
        digitalWrite(PIN_FAN_FLAP, RELAY_OFF);
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
                digitalWrite(PIN_FAN_FLAP, RELAY_ON);
                nfFlapMs = millis();
                nfState  = NF_FLAP_OPENING;
            }
            break;
        case NF_FLAP_OPENING:
            if (gap <= 2.0f) {
                nightFanOff();
            } else if (millis() - nfFlapMs >= FAN_FLAP_OPEN_MS) {
                digitalWrite(PIN_WALL_FAN, RELAY_ON);
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
        digitalWrite(PIN_ALARM_SOUNDER, RELAY_OFF);
        digitalWrite(PIN_EXT_LIGHTS, extLightsOn ? RELAY_ON : RELAY_OFF); // restore to pre-alarm state
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
    digitalWrite(PIN_EXT_LIGHTS, flashOn ? RELAY_ON : RELAY_OFF);

    updateFireRateCheck(now);

    switch (firePhase) {
        case FIRE_ALERT:
            // D40 held on via buzzerActive in updateSecurity(); no sounder yet
            if (now - firePhaseStartMs >= 60000UL) {
                firePhase        = FIRE_SOUNDER_ON;
                firePhaseStartMs = now;
                fireCycle        = 1;
                digitalWrite(PIN_ALARM_SOUNDER, RELAY_ON);
            }
            break;

        case FIRE_SOUNDER_ON:
            if (now - firePhaseStartMs >= 60000UL) {
                digitalWrite(PIN_ALARM_SOUNDER, RELAY_OFF);
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
                    digitalWrite(PIN_ALARM_SOUNDER, RELAY_ON);
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
        digitalWrite(PIN_MIDPOINT_LED, flashOn ? RELAY_ON : RELAY_OFF);
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
        digitalWrite(PIN_MIDPOINT_LED, ledBlinkHigh ? RELAY_ON : RELAY_OFF);
    } else if (manualHeaterOn) {
        // Priority 2: 1s on / 5s off
        if (now - ledBlinkMs >= (ledBlinkHigh ? 1000UL : 5000UL)) {
            ledBlinkHigh = !ledBlinkHigh;
            ledBlinkMs   = now;
        }
        digitalWrite(PIN_MIDPOINT_LED, ledBlinkHigh ? RELAY_ON : RELAY_OFF);
    } else if (!workshopLocked) {
        // Priority 3: steady on
        digitalWrite(PIN_MIDPOINT_LED, RELAY_ON);
    } else {
        // Priority 4: off
        digitalWrite(PIN_MIDPOINT_LED, RELAY_OFF);
    }
}

// ============================================================
//  SOLAR PUMP CURRENT FAULT  (INA219)
// ============================================================

void checkSolarPumpFault() {
    enum PumpUCPhase : uint8_t { PUC_OK = 0, PUC_RAMP, PUC_FAULT };
    static PumpUCPhase   ucPhase       = PUC_OK;
    static unsigned long ucStartMs     = 0;
    static unsigned long ocStartMs     = 0;
    static bool          ufhDumpActive = false;
    static bool          ocDumpActive  = false;

    auto clearUC = [&]() {
        if (ufhDumpActive) {
            ufhColdValve.setClose();
            digitalWrite(PIN_UFH_PUMP, RELAY_OFF);
            ufhDumpActive  = false;
            solarDumpUFHOn = false;
        }
        ucPhase   = PUC_OK;
        ucStartMs = 0;
        clearFault(FAULT_W_SOLAR_PUMP);
    };

    if (!pumpOutputState || pumpTargetDuty == 0) {
        clearUC();
        ocStartMs = 0;
        if (!ocDumpActive) clearFault(FAULT_W_SOLAR_PUMP_OVERCURRENT);
        return;
    }

    // In ramp/fault phase keep 100% even between INA219 samples
    if (ucPhase != PUC_OK) setSolarPumpDuty(100);

    if (!pumpCurrentSampled) return;

    // Overcurrent — stop pump after 3s to protect motor; open UFH dump to bleed solar heat
    if (solarPumpCurrentA > SOLAR_PUMP_MAX_CURRENT_A) {
        if (ocStartMs == 0) ocStartMs = millis();
        if (millis() - ocStartMs > 3000UL) {
            setFault(FAULT_W_SOLAR_PUMP_OVERCURRENT);
            setSolarPumpDuty(0);
            if (!ocDumpActive && !sFault[SENSOR_SOLAR_HOT] && !sFault[SENSOR_SOLAR_COLD]) {
                if (sTemp[SENSOR_SOLAR_HOT] <= 90.0f && sTemp[SENSOR_SOLAR_COLD] <= 90.0f) {
                    ufhColdValve.setOpen();
                    solarColdValve.setOpen();
                    digitalWrite(PIN_UFH_PUMP, RELAY_ON);
                    ocDumpActive   = true;
                    solarDumpUFHOn = true;
                }
            }
        }
    } else {
        ocStartMs = 0;
        if (ocDumpActive) {
            ufhColdValve.setClose();
            digitalWrite(PIN_UFH_PUMP, RELAY_OFF);
            ocDumpActive   = false;
            solarDumpUFHOn = false;
        }
        clearFault(FAULT_W_SOLAR_PUMP_OVERCURRENT);
    }

    // Undercurrent (stall): ramp to 100% immediately, then open UFH dump after 5s at full speed
    bool underCurrent = (solarPumpCurrentA < SOLAR_PUMP_MIN_CURRENT_A);
    switch (ucPhase) {
        case PUC_OK:
            if (underCurrent) { ucPhase = PUC_RAMP; ucStartMs = millis(); setSolarPumpDuty(100); }
            break;
        case PUC_RAMP:
            if (!underCurrent) {
                clearUC();
            } else if (millis() - ucStartMs > 5000UL) {
                ucPhase = PUC_FAULT;
                setFault(FAULT_W_SOLAR_PUMP);
                if (!sFault[SENSOR_SOLAR_HOT] && !sFault[SENSOR_SOLAR_COLD]) {
                    if (sTemp[SENSOR_SOLAR_HOT] <= 90.0f && sTemp[SENSOR_SOLAR_COLD] <= 90.0f) {
                        ufhColdValve.setOpen();
                        solarColdValve.setOpen();
                        digitalWrite(PIN_UFH_PUMP, RELAY_ON);
                        ufhDumpActive  = true;
                        solarDumpUFHOn = true;
                    }
                }
            }
            break;
        case PUC_FAULT:
            if (!underCurrent) clearUC();
            break;
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

    // Growatt inverter data
    pkt.growattValid    = growatt.valid ? 1 : 0;
    if (growatt.valid) {
        pkt.pv1W            = growatt.pv1W;
        pkt.pv2W            = growatt.pv2W;
        pkt.pvOutputW       = growatt.pvOutputW;
        pkt.loadW           = growatt.loadW;
        pkt.pvExportW       = growatt.pvExportW;
        pkt.gridImportW     = growatt.gridImportW;
        pkt.battVoltage_dV  = growatt.battVoltage_dV;
        pkt.battSocPct      = growatt.battSocPct;
        pkt.battChargeW     = growatt.battChargeW;
        pkt.dailyGenDeciKwh = growatt.dailyGenDeciKwh;
    }

    pkt.solarPumpDutyPct  = pumpTargetDuty;
    pkt.solarPumpActive   = solarPumpActive ? 1 : 0;

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
    pkt.ufhPumpRunning     = (digitalRead(PIN_UFH_PUMP) == RELAY_ON) ? 1 : 0;
    pkt.ufhTargetReached   = ufhTargetReached ? 1 : 0;
    pkt.solarDumpActive    = solarDumpUFHOn ? 1 : 0;
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
            if (outDir != PKT_DIR_HW || payLen != sizeof(HToWPacket)) { rs485RxBadFrame++; continue; }

            // Sequence number miss detection
            uint8_t expectedSeq = (uint8_t)(lastRxSeq + 1);
            if (lastRxSeq != 255 && outSeq != expectedSeq) {
                // Missed one or more packets but this one is valid
            }
            lastRxSeq = outSeq;

            memcpy(&lastH, payload, sizeof(HToWPacket));
#ifdef DEBUG_SERIAL
            if (simHtrPctActive) lastH.heaterPowerPct = simHtrPctVal;
#endif
            hasHPacket = true;
            lastRxMs   = millis();
            missedPackets = 0;
            rs485RxGood++;

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
    rs485RxMiss++;
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

static void dbgI2CScan() {
    Serial.println(F("Scanning I2C bus..."));
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print(F("  0x")); if (addr < 0x10) Serial.print(F("0"));
            Serial.println(addr, HEX);
            found++;
        }
    }
    if (found == 0) Serial.println(F("  none found"));
    else { Serial.print(F("  total: ")); Serial.println(found); }
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
    WF(FAULT_W_GROWATT_COMMS,         "SDM230_COMMS")
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
    Serial.print(F("  growatt:     "));
    if (simGrowattActive) {
        Serial.print(F("SIM pv_out=")); Serial.print(simGPvOutW);
        Serial.print(F(" export=")); Serial.print(simGPvExportW);
        Serial.print(F(" soc=")); Serial.print(simGBattSocPct);
        Serial.print(F("% charge=")); Serial.println(simGBattChargeW);
    } else {
        Serial.println(growatt.valid ? F("live") : F("no data"));
    }
}

static void dbgBus() {
    Serial.print(F("  fault:       ")); Serial.println(rs485CommsFault ? F("YES") : F("no"));
    Serial.print(F("  consec_miss: ")); Serial.println(missedPackets);
    Serial.print(F("  rx_good:     ")); Serial.println(rs485RxGood);
    Serial.print(F("  rx_miss:     ")); Serial.println(rs485RxMiss);
    Serial.print(F("  rx_badframe: ")); Serial.println(rs485RxBadFrame);
    Serial.print(F("  last_rx_ms:  "));
    if (!hasHPacket) Serial.println(F("never"));
    else { Serial.print(millis() - lastRxMs); Serial.println(F("ms ago")); }
    Serial.print(F("  miss_rate:   "));
    uint16_t total = rs485RxGood + rs485RxMiss;
    if (total == 0) Serial.println(F("n/a"));
    else { Serial.print((uint32_t)rs485RxMiss * 100 / total); Serial.println(F("%")); }
}

static void dbgPump() {
    Serial.print(F("  duty:      ")); Serial.println(pumpTargetDuty);
    Serial.print(F("  output:    ")); Serial.println(pumpOutputState ? F("ON") : F("OFF"));
    Serial.print(F("  current_A: ")); Serial.print(solarPumpCurrentA, 3);
    if (simPumpCurrentActive) Serial.print(F(" (SIM)"));
    Serial.println();
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

// Send one Modbus request to Growatt and print the raw response.
// Uses Serial2 with DE pin driven active for TX, returns to passive (LOW) after.
static void growattReadRaw(uint8_t fc, uint16_t startReg, uint16_t count) {
    uint8_t req[8];
    req[0] = 0x01;
    req[1] = fc;
    req[2] = (uint8_t)(startReg >> 8);
    req[3] = (uint8_t)(startReg);
    req[4] = (uint8_t)(count >> 8);
    req[5] = (uint8_t)(count);
    uint16_t crc = sdmCRC16(req, 6);
    req[6] = (uint8_t)crc;
    req[7] = (uint8_t)(crc >> 8);

    // Drain, ignoring 0x00 idle-bus noise. Wait for 30ms with no non-zero bytes.
    // If non-zero bytes keep arriving for >500ms another master owns the bus.
    // Mandatory 50ms drain first — absorbs any trailing bytes from the previous exchange
    // that arrive in transit after the previous call returned.
    { unsigned long t = millis(); while (millis() - t < 50) if (Serial2.available()) Serial2.read(); }
    {
        unsigned long quietStart = millis();
        unsigned long giveUp = millis();
        while (millis() - quietStart < 30) {
            if (Serial2.available()) {
                uint8_t b = Serial2.read();
                if (b != 0x00) quietStart = millis(); // only real data resets the timer
            }
            if (millis() - giveUp > 500) {
                Serial.println(F("    (bus busy - no quiet window)"));
                return;
            }
        }
    }

    digitalWrite(PIN_RS485_DE_GROWATT, HIGH);
    delayMicroseconds(200);
    Serial2.write(req, 8);
    Serial2.flush();
    delayMicroseconds(200);
    digitalWrite(PIN_RS485_DE_GROWATT, LOW);

    // Collect response; skip leading 0x00 idle bytes, stop after 5ms gap or 300ms total.
    uint8_t buf[96];
    uint8_t n = 0;
    unsigned long t0 = millis();
    unsigned long lastByte = millis();
    bool started = false;
    while (millis() - t0 < 300 && n < sizeof(buf)) {
        if (Serial2.available()) {
            uint8_t b = (uint8_t)Serial2.read();
            if (!started && b == 0x00) continue; // skip idle noise before response
            started = true;
            buf[n++] = b;
            lastByte = millis(); // track ALL bytes — zero register values must not trigger gap
        } else if (started && millis() - lastByte > 5) break;
    }
    wdt_reset();

    Serial.print(F("  FC0")); Serial.print(fc, HEX);
    Serial.print(F(" @")); Serial.print(startReg);
    Serial.print(F(" x")); Serial.print(count);
    Serial.print(F(": [")); Serial.print(n); Serial.print(F("] "));
    for (uint8_t i = 0; i < n; i++) {
        if (buf[i] < 0x10) Serial.print('0');
        Serial.print(buf[i], HEX); Serial.print(' ');
    }
    Serial.println();

    if (n < 5) { Serial.println(F("    (no/short response)")); return; }
    if (buf[0] != 0x01) { Serial.print(F("    (wrong addr ")); Serial.print(buf[0]); Serial.println(')'); return; }
    if (buf[1] & 0x80)  { Serial.print(F("    (exception ")); Serial.print(buf[2]); Serial.println(')'); return; }

    uint16_t rxCrc = sdmCRC16(buf, n - 2);
    bool crcOk = ((rxCrc & 0xFF) == buf[n-2]) && ((rxCrc >> 8) == buf[n-1]);
    Serial.print(F("    crc ")); Serial.println(crcOk ? F("ok") : F("FAIL (registers may be truncated)"));
    if (buf[2] == 0) return;

    // Decode however many complete register pairs we actually received.
    uint8_t bc = buf[2];
    uint8_t available = (n > 3) ? (n - 3) : 0; // data bytes in buf
    if (available > bc) available = bc;          // cap at declared byteCount
    available &= ~1;                             // round down to complete pair
    for (uint8_t i = 0; i + 1 < available; i += 2) {
        uint16_t val = ((uint16_t)buf[3+i] << 8) | buf[3+i+1];
        Serial.print(F("    r")); Serial.print(startReg + i/2);
        Serial.print(F(" = 0x")); Serial.print(val, HEX);
        Serial.print(F(" (")); Serial.print(val); Serial.println(')');
    }
}

// Passively listen for 2s and dump everything that arrives — diagnose bus traffic.
static void growattListen() {
    Serial.println(F("  [listening 2s]"));
    uint8_t buf[128];
    uint8_t n = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < 2000 && n < sizeof(buf)) {
        if (Serial2.available()) buf[n++] = (uint8_t)Serial2.read();
    }
    wdt_reset();
    Serial.print(F("  [")); Serial.print(n); Serial.print(F(" bytes] "));
    for (uint8_t i = 0; i < n; i++) {
        if (buf[i] < 0x10) Serial.print('0');
        Serial.print(buf[i], HEX); Serial.print(' ');
    }
    Serial.println();
}

static void dbgGrowatt() {
    // FC04 input registers: real-time PV, battery, grid
    Serial.println(F("  [FC04 0-11: PV / status]"));
    growattReadRaw(0x04,    0, 12);
    Serial.println(F("  [FC04 10-34: gap]"));
    growattReadRaw(0x04,   10, 25);
    Serial.println(F("  [FC04 35-54: output / grid power]"));
    growattReadRaw(0x04,   35, 20);
    Serial.println(F("  [FC04 55-74: extended]"));
    growattReadRaw(0x04,   55, 20);
    Serial.println(F("  [FC04 80-109: energy totals]"));
    growattReadRaw(0x04,   80, 30);
    Serial.println(F("  [FC04 1000-1029: battery / grid block]"));
    growattReadRaw(0x04, 1000, 30);
    Serial.println(F("  [FC04 3000-3019: alt energy block]"));
    growattReadRaw(0x04, 3000, 20);
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
    if (!strcmp_P(key, PSTR("pv_out")))      { simGrowattActive = true; simGPvOutW      = (int16_t)fval; Serial.println(F("ok")); return; }
    if (!strcmp_P(key, PSTR("pv_export")))   { simGrowattActive = true; simGPvExportW   = (int16_t)fval; Serial.println(F("ok")); return; }
    if (!strcmp_P(key, PSTR("batt_soc")))    { simGrowattActive = true; simGBattSocPct  = (uint8_t)fval;  Serial.println(F("ok")); return; }
    if (!strcmp_P(key, PSTR("batt_charge"))) { simGrowattActive = true; simGBattChargeW = (int16_t)fval; Serial.println(F("ok")); return; }
    if (!strcmp_P(key, PSTR("growatt_clear"))){ simGrowattActive = false; Serial.println(F("cleared")); return; }
    if (!strcmp_P(key, PSTR("pump_current")))      { simPumpCurrentActive = true;  simPumpCurrentVal = fval; Serial.println(F("ok")); return; }
    if (!strcmp_P(key, PSTR("pump_current_clear"))) { simPumpCurrentActive = false; Serial.println(F("cleared")); return; }
    if (!strcmp_P(key, PSTR("heater_pct")))        { simHtrPctActive = true;  simHtrPctVal = (uint8_t)constrain((int)fval, 0, 100); Serial.println(F("ok")); return; }
    if (!strcmp_P(key, PSTR("heater_pct_clear")))  { simHtrPctActive = false; Serial.println(F("cleared")); return; }
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
        Serial.println(F("temps  valves  faults  mode  status  bus  pump  fans  security  growatt  growattlisten"));
        Serial.println(F("scan  (1-Wire address discovery)  i2cscan  (I2C address discovery)"));
        Serial.println(F("set <sensor> <val>  (val=999 clears sim)"));
        Serial.println(F("  sensors: solar_hot solar_cold ufh_supply ufh_post_tmv workshop_air outside_air"));
        Serial.println(F("set pir|door|winch_cls|winch_lock <0|1>"));
        Serial.println(F("set pir_clear|door_clear|winch_cls_clear|winch_lock_clear 0"));
        Serial.println(F("set pv_out|pv_export|batt_soc|batt_charge <val>  (Growatt sim)"));
        Serial.println(F("set growatt_clear 0  (clears all Growatt sim)"));
        Serial.println(F("set heater_pct <val>  set heater_pct_clear 0"));
    }
    else if (!strcmp_P(cmd, PSTR("temps")))    dbgTemps();
    else if (!strcmp_P(cmd, PSTR("valves")))   dbgValves();
    else if (!strcmp_P(cmd, PSTR("faults")))   dbgFaults();
    else if (!strcmp_P(cmd, PSTR("mode")))     dbgMode();
    else if (!strcmp_P(cmd, PSTR("status")))   { dbgMode(); dbgTemps(); dbgFaults(); }
    else if (!strcmp_P(cmd, PSTR("bus")))      dbgBus();
    else if (!strcmp_P(cmd, PSTR("pump")))     dbgPump();
    else if (!strcmp_P(cmd, PSTR("fans")))     dbgFans();
    else if (!strcmp_P(cmd, PSTR("security"))) dbgSecurity();
    else if (!strcmp_P(cmd, PSTR("growatt")))       dbgGrowatt();
    else if (!strcmp_P(cmd, PSTR("growattlisten"))) growattListen();
    else if (!strcmp_P(cmd, PSTR("scan")))     dbgScan();
    else if (!strcmp_P(cmd, PSTR("i2cscan"))) dbgI2CScan();
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
    digitalWrite(PIN_UFH_PUMP,       RELAY_OFF);
    digitalWrite(PIN_WALL_FAN,       RELAY_OFF);
    digitalWrite(PIN_FAN_FLAP,       RELAY_OFF);
    digitalWrite(PIN_EXT_LIGHTS,     RELAY_OFF);
    digitalWrite(PIN_VAC_PUMP,       RELAY_OFF);
    digitalWrite(PIN_ALARM_SOUNDER,  RELAY_OFF);
    digitalWrite(PIN_BUZZER_SIGNAL,  RELAY_OFF);
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

    // Output pins — set register before pinMode to avoid glitch on active-LOW relay boards
    pinMode(PIN_UFH_COLD_DIR,    OUTPUT); digitalWrite(PIN_UFH_COLD_DIR,    RELAY_OFF);
    pinMode(PIN_SOLAR_COLD_DIR,  OUTPUT); digitalWrite(PIN_SOLAR_COLD_DIR,  RELAY_OFF);
    pinMode(PIN_UFH_PUMP,        OUTPUT); digitalWrite(PIN_UFH_PUMP,        RELAY_OFF);
    pinMode(PIN_WALL_FAN,        OUTPUT); digitalWrite(PIN_WALL_FAN,        RELAY_OFF);
    pinMode(PIN_FAN_FLAP,        OUTPUT); digitalWrite(PIN_FAN_FLAP,        RELAY_OFF);
    pinMode(PIN_DOOR_LOCK_A,     OUTPUT); digitalWrite(PIN_DOOR_LOCK_A,     RELAY_OFF);
    pinMode(PIN_EXT_LIGHTS,      OUTPUT); digitalWrite(PIN_EXT_LIGHTS,      RELAY_OFF);
    pinMode(PIN_WINCH_DIR_OPEN,  OUTPUT); digitalWrite(PIN_WINCH_DIR_OPEN,  RELAY_OFF);
    pinMode(PIN_WINCH_POWER,     OUTPUT); digitalWrite(PIN_WINCH_POWER,     RELAY_OFF);
    pinMode(PIN_WINCH_DIR_CLOSE, OUTPUT); digitalWrite(PIN_WINCH_DIR_CLOSE, RELAY_OFF);
    pinMode(PIN_MIDPOINT_LED,    OUTPUT); digitalWrite(PIN_MIDPOINT_LED,    RELAY_OFF);
    pinMode(PIN_RS485_DE_LINK,   OUTPUT); digitalWrite(PIN_RS485_DE_LINK,   LOW);        // not a relay
    pinMode(PIN_RS485_DE_GROWATT, OUTPUT); digitalWrite(PIN_RS485_DE_GROWATT, LOW);      // not a relay
    pinMode(PIN_VAC_ISO_OPEN,    OUTPUT); digitalWrite(PIN_VAC_ISO_OPEN,    RELAY_OFF);
    pinMode(PIN_VAC_ISO_CLOSE,   OUTPUT); digitalWrite(PIN_VAC_ISO_CLOSE,   RELAY_OFF);
    pinMode(PIN_DOOR_LOCK_B,     OUTPUT); digitalWrite(PIN_DOOR_LOCK_B,     RELAY_OFF);
    pinMode(PIN_BUZZER_SIGNAL,   OUTPUT); digitalWrite(PIN_BUZZER_SIGNAL,   RELAY_OFF);
    pinMode(PIN_HEN_DOOR_OPEN,   OUTPUT); digitalWrite(PIN_HEN_DOOR_OPEN,   RELAY_OFF);
    pinMode(PIN_HEN_DOOR_CLOSE,  OUTPUT); digitalWrite(PIN_HEN_DOOR_CLOSE,  RELAY_OFF);
    pinMode(PIN_VAC_PUMP,        OUTPUT); digitalWrite(PIN_VAC_PUMP,        RELAY_OFF);
    pinMode(PIN_ALARM_SOUNDER,   OUTPUT); digitalWrite(PIN_ALARM_SOUNDER,   RELAY_OFF);
    pinMode(PIN_SOLAR_PUMP,      OUTPUT); digitalWrite(PIN_SOLAR_PUMP,      LOW);        // MOSFET gate, not relay

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
    Serial1.begin(9600);             // inter-controller link
    Serial2.begin(9600); // SDM230/Growatt RS485 — 9600 8N1

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

    // Growatt Modbus polling
    memset(&growatt, 0, sizeof(growatt));

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
    // Fault check after solar+UFH so it can override both pump duty and UFH pump pin
    checkSolarPumpFault();
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

    growattPoll();

    // Inter-controller RS485 (every 250ms)
    if (now - lastTxMs >= INTER_CTRL_POLL_MS) {
        sendWToHPacket();
        receiveHToWPacket();
        wdt_reset();
    }
}
