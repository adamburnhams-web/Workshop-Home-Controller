#pragma once
#include <stdint.h>

// ============================================================
//  Shared types, enums, and RS485 packet structures
//  Used by both W controller and H controller
// ============================================================

// Temperature sentinel: sensor fault / no reading
#define TEMP_FAULT  INT16_MIN

// ── System enums ────────────────────────────────────────────

enum SystemMode : uint8_t {
    MODE_WINTER = 0,
    MODE_SUMMER = 1
};

enum BoostMode : uint8_t {
    BOOST_OFF   = 0,
    BOOST_5AM   = 1,
    BOOST_8HR   = 2
};

enum SolarTargetMode : uint8_t {
    SOLAR_TANK_PLUS8 = 0,
    SOLAR_MAX        = 1
};

enum ManualHeaterMode : uint8_t {
    MHM_OFF      = 0,
    MHM_SOC_LIM  = 1,   // run at 100% until battery SOC drops to 50%
    MHM_FORCE_ON = 2
};

enum WinchState : uint8_t {
    WINCH_STOP  = 0,
    WINCH_OPEN  = 1,
    WINCH_CLOSE = 2
};

// ── Fault flag bitmask (shared between both controllers) ────

// W-side faults (in wFaultFlags)
#define FAULT_W_SOLAR_OVERHEAT_COLD  (1UL << 0)
#define FAULT_W_SOLAR_OVERHEAT_HOT   (1UL << 1)
#define FAULT_W_SOLAR_PUMP           (1UL << 2)
#define FAULT_W_SOLAR_PUMP_OVERCURRENT (1UL << 7)
#define FAULT_W_UFH_OVERHEAT         (1UL << 3)
#define FAULT_W_FROST_NOT_RECOVERING (1UL << 4)
#define FAULT_W_VAC_PUMP_OVERTIME    (1UL << 5)
#define FAULT_W_GROWATT_COMMS        (1UL << 6)
#define FAULT_W_RS485_COMMS          (1UL << 8)
#define FAULT_W_FAN1                 (1UL << 9)
#define FAULT_W_FAN2                 (1UL << 10)
#define FAULT_W_WINCH_OVER_OPEN      (1UL << 11)
#define FAULT_W_SENSOR_SOLAR_HOT     (1UL << 12)
#define FAULT_W_SENSOR_SOLAR_COLD    (1UL << 13)
#define FAULT_W_SENSOR_UFH_POST_TMV  (1UL << 14)
#define FAULT_W_SENSOR_UFH_SUPPLY    (1UL << 15)
#define FAULT_W_SENSOR_WORKSHOP_AIR  (1UL << 16)
#define FAULT_W_SENSOR_OUTSIDE_AIR   (1UL << 17)
#define FAULT_W_FIRE_ALARM           (1UL << 18)

// H-side faults (in hFaultFlags)
#define FAULT_H_HEATER_OVERHEAT_WARN (1UL << 0)
#define FAULT_H_HEATER_OVERHEAT_SHUT (1UL << 1)
#define FAULT_H_HEATER_ELEMENT_FAIL  (1UL << 2)
#define FAULT_H_RS485_COMMS          (1UL << 3)
#define FAULT_H_BUS_VOLTAGE_LOW      (1UL << 4)
#define FAULT_H_GRID_OUTAGE          (1UL << 5)
#define FAULT_H_SENSOR_TANK_BOT      (1UL << 6)
#define FAULT_H_SENSOR_TANK_MID      (1UL << 7)
#define FAULT_H_SENSOR_TANK_TOP      (1UL << 8)
#define FAULT_H_SENSOR_HOT_PIPE      (1UL << 9)
#define FAULT_H_SENSOR_COLD_PIPE     (1UL << 10)
#define FAULT_H_SENSOR_HEATER_OUT    (1UL << 11)
#define FAULT_H_SENSOR_HEATER_OUT_2  (1UL << 12)

// ── W-side valve state bitmask (valveStates field) ──────────
#define VSTATE_UFH_COLD_OPEN    (1 << 0)
#define VSTATE_SOLAR_COLD_OPEN  (1 << 1)
#define VSTATE_VAC_ISO_OPEN     (1 << 2)
#define VSTATE_FAN_FLAP_OPEN    (1 << 3)

// ── Winch reed flags bitmask ─────────────────────────────────
#define WREED_FULLY_OPEN    (1 << 0)
#define WREED_FULLY_CLOSED  (1 << 1)
#define WREED_MANUAL_LOCK   (1 << 2)
#define WREED_SAFETY_LIMIT  (1 << 3)

// ── H-side valve override bitmask ───────────────────────────
#define OVER_UFH_COLD_OPEN   (1 << 0)
#define OVER_SOLAR_COLD_OPEN (1 << 1)
#define OVER_VAC_ISO_OPEN    (1 << 2)

// ============================================================
//  W → H  packet  (sent every 250ms from Workshop to House)
// ============================================================
struct __attribute__((packed)) WToHPacket {
    // Temperatures: int16_t = °C × 10, TEMP_FAULT = sensor fault
    int16_t tempSolarHot;
    int16_t tempSolarCold;
    int16_t tempUFHSupply;      // at pump/return; 28°C pump-off check
    int16_t tempUFHPostTMV;     // after TMV; 45°C lockout
    int16_t tempWorkshopAir;
    int16_t tempOutsideAir;

    // Growatt inverter data
    int16_t pv1W;               // PV string 1 power W
    int16_t pv2W;               // PV string 2 power W
    int16_t pvOutputW;          // inverter AC output W (PO)
    int16_t loadW;              // load power W (PL)
    int16_t pvExportW;          // grid export W (positive = exporting)
    int16_t gridImportW;        // grid import W (positive = importing)
    int16_t battVoltage_dV;     // battery voltage ×0.1V
    uint8_t battSocPct;         // battery SOC %
    int16_t battChargeW;        // battery net W (positive = charging, negative = discharging)
    uint8_t growattValid;       // 1 = Growatt data is fresh
    int16_t dailyGenDeciKwh;    // today's PV generation ×0.1kWh (r57)

    // Solar pump
    uint8_t solarPumpDutyPct;
    uint8_t solarPumpActive;     // 1 = solar loop active (may be suppressed/clocking); 0 = solar stopped

    // W-side state
    uint8_t valveStates;        // VSTATE_* bitmask
    uint8_t workshopLocked;     // 1 = locked
    uint8_t doorOpen;
    uint8_t pirActive;
    uint8_t manualRelockOn;
    uint8_t vacPumpRunning;
    uint8_t vacSensorFull;      // 1 = full vacuum achieved
    uint8_t winchState;         // WinchState enum
    uint8_t winchReedFlags;     // WREED_* bitmask
    uint8_t fanDutyPct;
    uint32_t fanFullTimerSecs;
    uint32_t fanBaseTimerSecs;
    uint8_t ufhPumpRunning;
    uint8_t ufhTargetReached;   // W signals W-air reached target
    uint8_t solarDumpActive;    // 1 = UFH pump running for emergency solar heat dump

    // W faults
    uint32_t wFaultFlags;

    // Time sync
    uint8_t requestTimeSync;    // 1 = W needs time from H
};

// ============================================================
//  H → W  packet  (sent every 250ms from House to Workshop)
// ============================================================
struct __attribute__((packed)) HToWPacket {
    // Temperatures: int16_t = °C × 10, TEMP_FAULT = sensor fault
    int16_t tempTankBot;
    int16_t tempTankMid;
    int16_t tempTankTop;
    int16_t tempHotPipe;
    int16_t tempColdPipe;
    int16_t tempHeaterOut;

    // Heater state
    uint8_t heaterPowerPct;
    uint8_t heaterRestricted;

    // System configuration
    uint8_t systemMode;         // SystemMode enum
    uint8_t boostMode;          // BoostMode enum
    int16_t ufhStopTemp_dC;     // W stops UFH when W-air >= this (°C × 10)
    uint8_t morningHeatActive;  // 1 = 5am / boost heating session is active
    uint8_t summerStartupPhase; // 0=idle 1=seq-running 2=running
    uint8_t nightCoolingEnabled;
    uint8_t solarTargetMode;    // SolarTargetMode enum
    uint8_t manualHeaterMode;   // ManualHeaterMode enum (for LED priority at W)

    // Fan settings (W executes autonomously using lock state + these)
    uint8_t fanBaseSpeedPct;

    // Fan timer delta commands (one-shot; H sets on button press, clears after transmit)
    int8_t  fanFullTimerDeltaHr;   // +/- hours added to W's full-speed timer (0 = no change)
    int8_t  fanBaseTimerDeltaDay;  // +/- days added to W's base-speed timer (0 = no change)

    // Manual override from page 4
    uint8_t overrideActive;
    uint8_t overrideValves;     // OVER_* bitmask

    // Alert reset acknowledgement (H increments each time user confirms)
    uint8_t alertResetSeq;

    // Time sync payload
    uint8_t timeSyncValid;
    uint8_t syncYear;           // year − 2000
    uint8_t syncMonth;
    uint8_t syncDay;
    uint8_t syncHour;
    uint8_t syncMinute;
    uint8_t syncSecond;

    // H faults
    uint32_t hFaultFlags;

    // 2-port valve state (1 = heater cold side / top-of-tank, 0 = mid-tank side)
    uint8_t twoPortHeaterSide;

    // Pump calibration (populated only when DEBUG_SERIAL active; always zero in production)
    uint8_t  calPumpActive;
    int16_t  calSolarTargetC;   // solar target override °C × 10

    // H-side solar pump direct drive duty (0–100%; non-zero only when heater on)
    uint8_t hPumpDutyPct;
};
