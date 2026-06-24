# Spec Changes — Delta from HW_Requirements_Spec_v5_9 / HW_Hardware_Spec_v2_5
Captures decisions and additions made after those documents were written.
Fold into the official docs at next version bump, then reset this file.

---

## W Controller — pin reassignments

| Signal | Spec pin | Code pin | Notes |
|---|---|---|---|
| Fan tachometer 1 | D41 | A8 (PCINT16/PK0) | Changed to PCINT2 group for hardware interrupt support |
| Fan tachometer 2 | D42 | A9 (PCINT17/PK1) | Changed to PCINT2 group |
| Alarm sounder | D42 | D26 | Freed D42 for hen house door |
| Buzzer signal to H | D40 | D41 | Board ch1 on 4th relay board |
| Mid-point LED | D34 direct output | D49 relay (board4 ch4) | Now switched via relay, not direct |
| RS485 DE/RE (link) | D46 | D40 | Freed D46; moved away from SPI area |

Full W relay board layout has changed from spec. New arrangement: one 8-ch board on even pins (D22–D36), one 8-ch board on odd pins (D23–D37), plus a separate 4-ch board (D41/D42/D48–D53).

**Even board (D22–D36) — mix of 15VDC and 230VAC loads:**
| Pin | Load |
|---|---|
| D22 | UFH cold valve direction relay (SPDT) |
| D24 | Solar cold valve direction relay (SPDT) |
| D26 | 15VDC alarm sounder |
| D28 | Fan flap actuator (15VDC) |
| D30 | Door lock H-bridge relay A |
| D32 | Door lock H-bridge relay B |
| D34 | Vacuum isolation valve CLOSE relay |
| D36 | Vacuum isolation valve OPEN relay |

**Odd board (D23–D37) — mix of 15VDC and 230VAC loads:**
| Pin | Load |
|---|---|
| D23 | Spare |
| D25 | External LED lights (230VAC) |
| D27 | Wall axial fan (230VAC) |
| D29 | Window winch power relay (230VAC) |
| D31 | Window winch direction OPEN (230VAC) |
| D33 | Window winch direction CLOSE (230VAC) |
| D35 | Vacuum pump (230VAC) |
| D37 | UFH central heating pump (230VAC) |

**4th relay board (D41/D42/D48–D53):**
| Pin | Load |
|---|---|
| D41 | Buzzer signal to H (pulls 2.5mm T&E earth) |
| D42 | Hen house door OPEN |
| D48 | Hen house door CLOSE |
| D49 | Mid-point LED (15VDC via relay) |
| D50–D53 | Spare |

---

## W Controller — new hardware/features

### Window manual buttons (D38/D39)
Hold-to-run buttons at W for manual window winch control.
- D38 = open, D39 = close
- Hold to move, release to stop immediately
- Reed lockouts still apply (over-open, fully closed, manual lock)
- Not in spec; added before coding began

### Hen house door (4th relay board ch2/ch3)
- D42 = open, D48 = close — H-bridge pair, 7s pulse
- 4th relay board now: ch1=buzzer, ch2=hen door open, ch3=hen door close, ch4=mid-point LED
- Not in spec; added before coding began

### Fire alarm system
New feature, not in spec. Logic:
- **Trigger A (absolute)**: workshop air > 25°C AND outside air has not been ≥ 25°C in the last 24 hours
- **Trigger B (rate-of-rise)**: workshop air rising ≥ 0.5°C/min AND door closed AND window fully closed
- **Phase 1 (60s)**: external lights flash 250ms, buzzer at H active, no local sounder
- **Phase 2**: local sounder on 60s / off 60s, up to 5 cycles or until rate-of-rise stops
- Clears on alert reset from H display (page 4)
- Fault flag: `FAULT_W_FIRE_ALARM` (bit 18 in wFaultFlags)
- External lights flash during alarm, restore to pre-alarm state on clear

---

## H Controller — pin reassignment

| Signal | Spec pin | Code pin | Notes |
|---|---|---|---|
| Display backlight | D32 | D44 (OC5C Timer5) | Hardware wire moved; D32 freed |
| Log burner cold CLOSE | D23 | D24 | 8-ch relay board moved to even pins D22–D36 |
| Bottom-of-tank OPEN | D24 | D26 | |
| Bottom-of-tank CLOSE | D25 | D28 | |
| 2-port OPEN | D26 | D30 | |
| 2-port CLOSE | D28 | D32 | |
| 12V PSU relay | D29 | D34 | |
| RS485 DE/RE | D30 | D31 | Freed D30 for relay ch5 |

**H relay board layout (8-ch, even pins D22–D36):**
| Pin | Ch | Load |
|---|---|---|
| D22 | 1 | Log burner cold valve OPEN |
| D24 | 2 | Log burner cold valve CLOSE |
| D26 | 3 | Bottom-of-tank valve OPEN |
| D28 | 4 | Bottom-of-tank valve CLOSE |
| D30 | 5 | 2-port valve OPEN (heater side) |
| D32 | 6 | 2-port valve CLOSE (mid-tank side) |
| D34 | 7 | 12VDC backup PSU relay |
| D36 | 8 | Spare |

D27 (SSR-40DA) is odd and sits between relay channels — not connected to the relay board.

---

## H Controller — library change

| Function | Spec library | Code library |
|---|---|---|
| TFT display | MCUFRIEND_kbv + Adafruit GFX | TFT_eSPI (includes GFX) |

ILI9488 driver, SPI pins, and display dimensions configured via `build_flags` in platformio.ini.

---

## H Controller — logic changes

### Heater SSR — duty algorithm rework

`HEATER_ENABLED` changed to `true` (heater now active in production).

**Manual modes** (`ManualHeaterMode` enum) — three modes now:
- `MHM_OFF` (0) — heater stays off
- `MHM_SOC_LIM` (1) — NEW: run at 100% drawing from battery+grid until SOC drops to 50%; enters at 55% (5% hysteresis). Budget: 4 kW combined bat+grid; heater gets all headroom.
- `MHM_FORCE_ON` (2) — heater forced to 100%; also requests log_cold close, bot_tank and two_port open

**Pre-conditions (checked before any start logic, in order):**
1. Grid present AND `HEATER_ENABLED = true`
2. No W solar sensor fault
3. No heater hard lockout
4. `hotTankProtection = false` (tank_bot ≤ 83°C — see below)
5. `morningHeatActive = false`
6. Valves not closing (heater stays off while bot_tank or two_port is mid-close stroke)
7. Auto mode only: `growattValid = 1` AND PV1+PV2 ≥ 200W (end-of-day gate)

**Auto mode duty calculation** (per RS485 packet, ~250ms):
```
SOC reservation:
  post-noon (hour ≥ 12) AND soc > 80%: reservationW = 1100W
  post-noon AND soc > 90%:             reservationW = 500W
  post-noon AND soc > 97%:             reservationW = 200W
  pre-noon (hour < 12) AND soc > 50%:  reservationW = 1100W  (same values, lower SOC threshold)
  otherwise:                            reservationW = 0

If reservation > 0:
  available = pvExportW − gridImportW + battChargeW + heaterCurrentW − reservationW
Else:
  available = pvExportW − gridImportW + min(0, battChargeW) + heaterCurrentW − 100

heaterCurrentW = heaterTargetPct × 3000 / 100
rawPct         = clamp(available × 100 / 3000, 0, 100)
```

**Start:** rawPct ≥ 17% (~500W) AND sustained for 5s  
**Stop:** 5 consecutive zero packets (was immediate)  
**Smoothing:** 8-packet running average before applying to `heaterTargetPct`

**Hot pipe cap** (applied after smoothing):
- `hot_pipe ≥ 90°C` → 0%
- `hot_pipe 60–90°C` → linear cap: `(90 − hot_pipe) × 3.33%/°C` (100% at 60°C, 0% at 90°C)

**`botTankValve.request(true)`** called whenever heater is running.

### Hot tank protection (new)

New `hotTankProtection` flag managed by `checkHotTankProtection()`:
- **Enter** (tank_bot > 83°C): heater off, `logBurnerCold` open, `botTankValve` closed, `twoPortValve` open. `updateHeatSourceSelection()` skipped while active.
- **Leave** (tank_bot < 82°C): flag cleared.

### Dual heater output sensor (htr_out_2)

Seventh DS18B20 added: `H_SENSOR_HEATER_OUT_2 = 6`, `H_NUM_SENSORS = 7`, sensor 13 (address filled in).

`getHeaterOutC()` arbitrates: both valid → `max(sensor1, sensor2)`; one faulted → remaining sensor; both faulted → `NAN`.

`FAULT_H_SENSOR_HEATER_OUT_2` (bit 12) added to fault flags.

### Heater fault handling rework

**Element-fail detection removed.** The previous 30s/no-temp-rise lockout was unreliable. Replaced by:

- Both sensors faulted → `heaterPowerCapPct = 0` immediately (ISR-level cap), set both fault flags
- Single sensor faulted → raise fault flag after 5s grace; continue on the remaining sensor
- Hot pipe sensor fault → `heaterPowerCapPct = 0` immediately
- RS485 stale > 60s while running → `heaterTargetPct = 0`, `heaterRunning = false`

**Overheat sequencing** (replaces warn→shutdown threshold at 91°C):
- **91–93°C**: ramp `heaterPowerCapPct` 100→0 (ISR-level cap); request valves open; set `FAULT_H_HEATER_OVERHEAT_WARN`
- **≥ 93°C**: `heaterPowerCapPct = 0`, `heaterHardLockout = true`, SSR pin cleared, set `FAULT_H_HEATER_OVERHEAT_SHUT`
- **Auto-clear** hard lockout when effective heater temp < 88°C AND at least one sensor live (clears fault too)
- **Page 5 Ack** also clears hard lockout immediately (allows manual recovery without reboot)

`heaterPowerCapPct` is a `volatile uint8_t` applied in the ZC ISR: `heaterSpreadAcc += min(heaterTargetPct, heaterPowerCapPct)`.

Heater gate in ISR also requires `VSTATE_SOLAR_COLD_OPEN` — heater cannot fire if the solar cold valve is closed.

### H-side solar pump direct drive — rework

**`calcHPumpDuty()` now returns `float`** (was `uint8_t`).

New predictive formula `calcPred(heaterPct, hotPipeC)`:
```
excess   = heaterPct − 20
pt       = max(0, excess²) × 0.0025        // plateau term
tr       = 0.09 + max(0, excess) × 0.022   // thermal rise slope
raw      = 4 + pt + tr × (hotPipeC − 30)
minDuty  = 4 + heaterPct × 0.05
pred     = max(raw, minDuty)
```

**Two operating branches:**

*SOLAR_TANK_PLUS8 mode* (tank top ≤ 75°C):
- Target = `min(tankTop + 8°C, 87°C)`
- Below target − 7°C: `pred × 0.9` (floor)
- target − 7°C to target: ramp 0.9→1.0 × pred
- target to 90°C: `pred + (hOut − target) × 0.2%/°C`
- 90–91°C: ramp from `dutyAt90` to 100%
- ≥ 91°C: 100%

*MAX mode* (or tank top fault):
- Fixed 85°C reference
- Upper = `pred × (1.3 − clamp((pred−5)/25, 0, 1) × 0.2)`
- Below 78°C: `pred × 0.9`
- 78–85°C: ramp pred × 0.9 → pred
- 85–90°C: ramp pred → upper
- 90–91°C: ramp upper → 100%; ≥ 91°C: 100%

Returns 0 when heater is off/lockout applies; 100 during `heaterHardLockout`; minimum 4% when heater running.

**`updateHPump()` moved to Timer1 COMPA ISR** (50ms tick, hardware timer). No longer called from main loop; also called explicitly during `drawFullPage()` strip fills and at end of full redraw to prevent pump stalling during long SPI operations.

Low-duty clocking simplified: zone 0 (< 4%) removed; minimum on-time now 400ms at ≤ 20%.

**Solar target** renamed `SOLAR_TANK_PLUS8` (was `SOLAR_TANK_PLUS5`). Default target = tank top + 8°C, capped at 87°C (was +5°C, capped at 89°C). Status bar and page 4 display updated accordingly.

### Summer startup sequence simplified

Phase 0 now jumps directly to phase 3 (skips phases 1/2). Rationale: solar hot ≥ 50°C means the collector is already above tank bottom; the phase 1/2 gate conditions are trivially met. `botTankValve` opens immediately. This also handles H rebooting mid-session cleanly.

`updateSummerStartup()` aborts and resets on `morningHeatActive` (was: abort on non-summer mode).

### SD logging — header validation and datetime column

On startup, if `log.csv` exists the header row is read and comma-counted. If the count does not match `LOG_EXPECTED_COMMAS` (21 commas, 22 columns), the file is deleted and a new header is written. This prevents stale column layouts from silently corrupting the log after a firmware update.

Column changes:
- `ts_ms` → `datetime` (ISO `YYYY-MM-DD HH:MM:SS` from RTC, or `INVALID` if RTC not set)
- Column order: `tank_bot` now before `tank_mid` and `tank_top`
- Added `htr_out_2` column (after `htr_out`)

`sdAvailable` set false on open failure (prevents repeated open attempts after card removed).

### Page 1 display — 7-row temperature block

Right column expanded from 6 to 7 rows to show both heater sensors (`Heater` = htr_out_2, `Htr Out` = htr_out). Power row updated: shows `Sol H:<pct>%` (H pump) and `Sol W:<pct>%` (W pump) side-by-side.

### RS485 per-packet Growatt/heater debug print (DEBUG_SERIAL only)

On each received W packet, H prints a timestamped summary to Serial:
`[HH:MM:SS] W pkt | growatt=OK pv1=...W ... soc=...% chg=...W | htr=...% pump=...%`

### `HBridgeValve::request()` — idempotent

Returns immediately if already at the requested position (phase=IDLE, isOpen matches) or already moving there (phase≠IDLE, pendingOpen matches). Prevents redundant pulse re-triggering when called repeatedly.

### `dbgValves()` — shows pending state

Now displays in-motion valve state (pending direction + phase) rather than only final position. New `dbgValveState()` helper handles formatting.

### 2-port valve mid-tank threshold — hysteresis added
Spec: switch at 30°C (no hysteresis)
Code: switch to mid-tank at > 32°C, switch back to heater side at < 28°C; hold between 28–32°C

### Summer 2-port valve — dynamic tracking in phase 3
Spec did not define ongoing 2-port control once summer startup sequence completes.
Code: in phase 3, 2-port tracks hot pipe vs tank top with ±1°C hysteresis:
- Hot pipe > tank top + 1°C → heater/top-of-tank side
- Hot pipe < tank top − 1°C → mid-tank side

### Fault log size — 80 entries not 200
Spec suggested ~200 entries. Code uses 80 (`~40 bytes × 80 = 3.2KB`) to stay within ATmega2560 8KB RAM budget with headroom for other state.

---

## H Controller — 12V PSU relay debounce

New on/off delays before switching the 12V backup PSU relay:
- **ON**: 3s continuously below 12V → relay energises (`BUS_PSU_ON_DELAY_MS = 3000`)
- **OFF**: 5s continuously above 12.5V (hysteresis threshold) → relay de-energises (`BUS_PSU_OFF_DELAY_MS = 5000`)

Previously the relay switched immediately on threshold crossing. Delays prevent relay chatter on a fluctuating bus.

---

## H Controller — heater level table corrections

Two entries in `kHeaterLevels[20]` changed to reduce the denominator (shorter burst period, lower line-frequency stress):

| Index | Old | New |
|---|---|---|
| level 4 | `{3,7}` → 30.0% | `{2,5}` → 28.6% |
| level 13 | `{7,3}` → 70.0% | `{5,2}` → 71.4% |

---

## H Controller — SOC test time-of-day aware thresholds

SOC test start/pause SOC thresholds now depend on time of day:

| Time | Start gate | Pause gate |
|---|---|---|
| Before 13:00 | 40% | 30% |
| 13:00 and after | 65% | 55% |

Previously hardcoded at 50% (start) / 40% (pause). Afternoon sessions need a higher SOC guard because the battery does not recover overnight between steps.

New helper functions: `socTestStartSoc()`, `socTestPauseSoc()`.

---

## H Controller — SOC test flush timing reworked

- SD flush window extended from 30ms → 100ms after last W packet. The 30ms guard never fired because `pollRS485` (debug serial + `Serial1.flush`) takes ~67ms alone.
- `socTestFlushLog()` moved to **before** `pollRS485()` in the main loop (was after). This places the SD write in the 100–270ms quiet zone after the previous reply; `pollRS485` then runs immediately and catches the next W packet without delay.
- `randomSeed(analogRead(A1))` added in `setup()` so the SCT shuffle order differs each boot.
- `socTestLastLog = millis()` now set when entering SCT_HEATING from IDLE (not just on step transitions).

---

## W Controller — pin reassignments (second batch)

| Signal | Old pin | New pin | Notes |
|---|---|---|---|
| `PIN_UNLOCK_BTN` | D10 | D46 | D10 shared with midpoint LED wire; moved to avoid masking condition |
| `PIN_LIGHT_BTN` | D11 | — | Removed; functionality replaced by `PIN_LOCK_SW` |
| `PIN_LOCK_SW` | — | D11 | New: lock switch (replaces light button) |
| `PIN_ALARM_SOUNDER` | D26 | D24 | Swapped with solar cold direction relay |
| `PIN_SOLAR_COLD_DIR` | D24 | D26 | Swapped with alarm sounder |

Updated relay board layout (even board, D22–D36):

| Pin | Load |
|---|---|
| D22 | UFH cold valve direction relay |
| D24 | Alarm sounder |
| D26 | Solar cold valve direction relay |
| D28 | Fan flap actuator |
| D30 | Door lock H-bridge relay A |
| D32 | Door lock H-bridge relay B |
| D34 | Vacuum isolation valve CLOSE |
| D36 | Vacuum isolation valve OPEN |

---

## W Controller — security: light button replaced with lock switch

`PIN_LIGHT_BTN` (D11) repurposed to `PIN_LOCK_SW` (D11):
- **Removed**: external light toggle via D11 button
- **New**: rising edge on `PIN_LOCK_SW` → immediately locks workshop (`doorLock.request(false)`, `workshopLocked = true`)
- Auto-relock timer permanently disabled (`relockTimerActive = false` unconditionally; relock only via `PIN_LOCK_SW`)
- `security` debug command now shows `unlock_btn`, `lock_sw`, and `led_pin` state

---

## W Controller — fan RPM fault detection disabled

`updateFanRPM()` no longer raises stall faults. `fan1FaultStartMs`, `fan2FaultStartMs`, `fan1FaultActive`, `fan2FaultActive` are reset unconditionally each RPM period. `FAULT_W_FAN1` and `FAULT_W_FAN2` are never raised and removed from `dbgFaults()` output.

Rationale: tachometer wiring not yet commissioned; false faults would mask real issues.

---

## W Controller — vacuum system: pump prewarm before valve open

New `VAC_PUMP_START` state inserted between `VAC_IDLE` and `VAC_OPENING`:
1. `VAC_IDLE` → `VAC_PUMP_START`: pump turns on immediately
2. After `VAC_PUMP_PREWARM_MS` (2s): isolation valve opens → `VAC_OPENING`

Previously the isolation valve opened first and the pump started only after the valve fully opened. Prewarm builds pressure before the valve exposes the vacuum circuit.

`VAC_EXTRA_RUN_MS` increased from 300s (5 min) to 600s (10 min).

---

## W Controller — mid-point LED: comms faults excluded

RS485 comms faults (`FAULT_W_RS485_COMMS`, `FAULT_W_GROWATT_COMMS`, `FAULT_H_RS485_COMMS`) masked out of the `anyFault` check in `updateMidpointLED()`. Only hardware faults trigger the fault-flash pattern.

---

## New diagnostic / test utility programs

Not deployed to production controllers; compiled via separate PlatformIO environments.

| Program | Environment | Purpose |
|---|---|---|
| `src/growatt_monitor/` | *(manual)* | Standalone Mega 2560: polls Growatt Modbus (Serial2/D16-D17, DE D47) and prints all data to USB serial once per 4-phase cycle |
| `src/h_test_comms/` | *(manual)* | Standalone H-side RS485 listener: prints every valid W→H packet with uptime, sequence, temps, Growatt, pump duty, fault flags; reports missed sequences |
| `src/w_sim/` | *(manual)* | W controller simulator: sends a fixed neutral `WToHPacket` to H every 250ms over RS485; prints H→W replies to USB serial |
| `src/w_test/` | `controller_w_test` | Alternate W controller test build (`build_src_filter = +<w_test/*>`, COM5) |

---

## Build system changes

- W controller upload/monitor port: COM3 → COM5
- New PlatformIO environment `controller_w_test`: same libraries as `controller_w`, `build_src_filter = +<w_test/*>`, COM5
- Root-level `platformio.ini` added at repo root (subset; `controller_w` + `controller_h` environments)

---

## W Controller — solar pump control rework (second pass)

### `calcPumpDuty()` — heater floor removed, new curve (temporary)

Signature changed from `(float hot, float targetC, uint8_t heaterPct, float heaterOutC)` to `(float hot, float targetC)`. `heaterMinPumpPct()` function removed. Heater-on minimum floor logic removed from W entirely — heater-on pump control is now solely handled by H via direct wire.

New temporary calibration curve (to be replaced with data-derived curve after `cal_pump` run):
| Solar hot vs target | Duty |
|---|---|
| < target − 8°C | 4% |
| target − 8°C to target | 4% → 50% linear (over 8°C) |
| target to target + 2°C | 50% → 100% linear (over 2°C) |
| ≥ target + 2°C | 100% |

### Solar target — SOLAR_TANK_PLUS8

`SOLAR_TANK_PLUS5` → `SOLAR_TANK_PLUS8` throughout. Summer solar target = `min(tankTop + 8°C, 87°C)` (was `+5°C / 80°C`). Default `solarTargetMode` changed accordingly.

### Overheat hot threshold raised 83°C → 91°C

`FAULT_W_SOLAR_OVERHEAT_HOT` now triggers at solar hot > 91°C (was > 83°C) with pump at 100% and tank bottom < 70°C. Clears when solar hot ≤ 91°C.

UFH dump separated from overheat fault flag: open `ufhColdValve` when `91°C < hot < 93°C` AND solar cold < 93°C; close otherwise. Previously dump was paired with the overheat flag and closed on recovery.

Overcurrent and stall dump safety guard raised from `≤ 90°C` to `< 93°C` (matches new UFH dump threshold).

### Summer startup trigger — PV export gate removed

`(growatt.valid && pvExportW >= 500)` removed from start condition. Heater power or collector temperature alone now sufficient to start summer solar.

### Cal mode — pump-stop state (calSolarTargetC == 0)

When H sets `calPumpActive = 1` with `calSolarTargetC = 0`, W immediately stops the solar pump and returns. This allows H to halt W's pump during heater warm-up phases of the calibration sequence.

In `DEBUG_SERIAL` builds, the cal mode check wraps the seasonal routing (`updateWinterSolar` / `updateSummerSolar`) — cal target 0 skips both paths entirely.

### solarColdValve opened when heater running

`solarColdValve.setOpen()` called unconditionally when `lastH.heaterPowerPct > 0`. Ensures the heater-to-solar circuit has a return path regardless of seasonal mode.

### Two-port valve back-off condition added (summer solar)

`finalDuty = 0` when all of: H packet valid, `twoPortHeaterSide = true`, summer startup phase ≥ 3, `hot < solarTarget`, `hot < 65°C`, `cold < solarTarget`, `cold < 65°C`. Prevents W from fighting H for pump control when H has the two-port valve on the heater side and both solar sides are cold.

### `simPumpSpdActive` / `simPumpSpdVal` — new debug override

`set pump_spd <0–100>` / `set pump_spd_clear 0` — directly forces solar pump duty at end of loop. Useful for benching the pump driver independently of control logic.

### RS485 rx timeout and Growatt stale threshold

- `RS485_RX_TIMEOUT_MS` increased from 150ms to 200ms (reduces spurious timeouts on slow bytes)
- `GROWATT_STALE_MS` reduced from 120s to 60s (faster invalidation of stale inverter data)

---

## Serial monitor — log2file enabled (platformio.ini)

`monitor_filters = log2file` added to `controller_h` environment. Serial output is now automatically saved to a timestamped file in `.pio/` during `pio device monitor` sessions.

---

## Both controllers — DS18B20 resolution

Spec assumed 12-bit (750ms conversion, 0.0625°C precision).
Code: 11-bit (375ms conversion, 0.125°C precision) — adequate for all thresholds in use.

Conversion restarts immediately after each read (no inter-conversion gap). Average data age ~190ms vs ~550ms previously.

---

## Both controllers — RS485

ArduinoRS485 library not used. Custom binary framing implemented in `include/rs485_packet.h`:
- Header: `[0xAA][0x55][DIR][SEQ][LEN_LO][LEN_HI][PAYLOAD][CRC_LO][CRC_HI]`
- CRC-16/Modbus over DIR+SEQ+LEN+PAYLOAD
- `PktReceiver` state machine handles byte-by-byte receive with automatic resync on error

---

## DS18B20 addresses — filled in

Both controllers: real addresses installed, no longer TODO placeholders.
H controller: sensors 7–12. W controller: sensors 1–6.
See top of each `main.cpp` for address arrays.

---

## Solar pump calibration — `cal_pump` command (H controller, DEBUG_SERIAL only)

Not in spec. Implemented as a debug serial command on the H controller to find minimum solar pump speed as a function of heater power and hot pipe temperature.

### Procedure
- Sweeps solar inlet target from 85°C down to 40°C in 5°C steps (10 steps total, descending)
- At each step:
  1. **STABILIZE**: waits for hot pipe to settle within ±2°C of the target for 10s; heater off
  2. **PRE_RAMP**: sets solar target to 90°C (pump drops to minimum clocking); heater starts at 5%, waits for heater output ≥ 87°C (5 min timeout)
  3. **RAMP**: steps heater from 5% to 100% in ~5% increments over 30s (1578ms per step); logs one CSV row just before each increment
- If pump duty reaches 100% mid-ramp (heater output ≥ 91°C), that heater level is logged as the ceiling for that solar step and the run advances immediately to the next step
- Advances to next solar step after either 100% heater or pump-ceiling; repeats until 40°C step complete

### Serial output
CSV format: `solar_step_C, heater_pct, hot_pipe_C, htr_out_C, pump_pct`  
Header line: `CAL_HDR: solar_step_C,heater_pct,hot_pipe_C,htr_out_C,pump_pct`

### Packet fields used (HToWPacket, DEBUG_SERIAL only)
- `calPumpActive` — 1 while STABILIZE/PRE_RAMP/RAMP active; W uses this to override solar target
- `calSolarTargetC` — solar target °C × 10; 900 during PRE_RAMP/RAMP (forces pump to minimum clocking)

### Commands
- `cal_pump` — start sequence (requires `HEATER_ENABLED = true`)
- `cal_abort` — stop immediately, heater off, normal control restored

---

## Calibration TODOs (still outstanding)

- `FAN_MIN_DUTY_PCT` (W): TODO calibrate on-site
- `FAN_FLAP_OPEN_MS` (W): TODO calibrate motorized damper travel time
- `VALVE_POWERUP_WAIT_MS` (W): 30s conservative, verify on-site
- `SOLAR_PUMP_MIN_CURRENT_A` / `SOLAR_PUMP_MAX_CURRENT_A` (W, INA219): set to 1.00A / 2.00A; verify on-site

---

## H Controller — heater SSR control changes

### Spread firing (Bresenham) replaces burst firing
Spec/previous: SSR fired for the first N consecutive half-cycles of a 100-cycle window (burst).
Code: Bresenham error-accumulator distributes firing events evenly across all half-cycles.

Effect: at 10% duty the pattern is one pulse every 10 half-cycles (every 100ms) rather than 10 pulses then 900ms silence. Export meters see near-constant load regardless of polling interval, eliminating false over-export readings.

ISR variable renamed: `heaterCycleCnt` → `heaterSpreadAcc` (accumulator, not a counter).

### Manual heater test override — `set heater_pct` serial command (DEBUG_SERIAL only)
Not in spec. Allows the heater SSR to be tested independently of `HEATER_ENABLED` commissioning flag.

- `set heater_pct <0–100>` — activates sim override at given duty %; sets `simHeaterActive = true`
- `set heater_pct_clear` — cancels override, returns to normal control
- Override bypasses `HEATER_ENABLED` check in ISR only; all other lockouts (grid, overheat, lockout flag) still apply
- `simHeaterActive` declared as `volatile bool` before the ISR; `simHeaterVal` (uint8) held separately

### `zc_pin` diagnostic added to `heater` debug command
`heater` serial command now samples D2 (zero crossing pin) 200 times at 100µs intervals and reports `xH/yL` counts. Used to confirm H11AA1 optocoupler is switching correctly before enabling heater.

---

## W Controller — solar pump control rework

### Overcurrent fault — UFH dump added
Previously: overcurrent (>2.0A for 3s) stopped the pump but opened no dump path; solar fluid could stagnate and overheat in collectors.
Code: on overcurrent trigger, opens `ufhColdValve` + `solarColdValve` and starts UFH pump (same as stall/undercurrent fault), subject to same safety guards (sensor valid, temps ≤ 90°C).

Separate `ocDumpActive` flag (static, inside `checkSolarPumpFault`) tracks the overcurrent dump independently of `ufhDumpActive` (undercurrent). The dump persists while the pump is stopped; clears when the pump restarts and current returns to normal.

### Solar pump speed — new duty calculation

Replaces `calcPumpDuty` (simple ±2°C ramp) and `clockPeriodForDiff` (3-step period lookup).

W calculates duty purely from solar hot vs solar target. No heater floor — heater-on pump control is handled by H via direct wire (see H controller section below).

| Solar hot vs target | Duty |
|---|---|
| < target − 8°C | 1% |
| target − 8°C to target − 2°C | 2% → 4% linear (over 6°C) |
| target − 2°C to target + 2°C | 4% → 100% linear (over 4°C) |
| ≥ target + 2°C | 100% |

Winter trigger (18°C) unchanged — pump will not start below this regardless of duty calc.

`calcPumpDuty` signature changed to `(float hot, float targetC, uint8_t heaterPct, float heaterOutC)`.

### Solar pump speed — overrides

Three transient override modes applied on top of base duty each loop:

**Rule 1 — 60s kick:** when either solar pipe > 60°C and base duty < 4%, fire a 1s full-speed kick every 60s to prevent stagnation at very low flow.

**Rule 2 — fast-rise burst:** when solar hot crosses from ≥ target − 2.5°C upward AND reaches target − 2.0°C within 15s, run pump at 100% for 5s. Only triggers if hot was genuinely below target − 2.0°C at the crossing point.

**Rule 3 — fast-drop pause:** when solar hot was > target + 2.5°C then drops to ≤ target + 2.0°C, set pump to 0% for 10s. Rule 3 takes priority over Rule 2 if both would fire simultaneously.

Override state (`pumpSpdOvMode`) is reset at all solar pump stop points (sensor fault, trigger not met, season end).

### Solar pump — H back-off

W suppresses its own MOSFET output (D44 → 0) when all three conditions hold:
- H has a valid packet and heater is running (`lastH.heaterPowerPct > 0`)
- H's pump duty (`lastH.hPumpDutyPct`) exceeds W's calculated duty
- Solar hot is below target

W resumes output when solar hot reaches target or W's duty is ≥ H's. Diode-OR on the MOSFET gate means the higher of the two signals drives the pump regardless.

### Solar pump timing — three-zone clocking

Replaces `pumpClockPeriodMs` / `pumpMinOnMs` dynamic period scheme. Variables and `minClockingDuty()` function removed.

| Zone | Duty range | On time | Off time | Actual duty |
|---|---|---|---|---|
| 0 | 1–4% | 200ms fixed | `200×(100−d)/d` | exact |
| A | 4–8% | 200→400ms linear | 4800→4600ms linear (period=5000ms constant) | exact |
| B | 8–20% | 400ms fixed | `400×(100−d)/d` | exact |
| C | 20–50% | 400→800ms (`+(d−20)×400/30`) | `on×(100−d)/d` | exact at 20% and 50%, ±2% mid-range |
| D | 50–100% | 800ms fixed | `800×(100−d)/d` | exact |
| — | 100% | continuous | — | — |

All zone boundaries transition smoothly (same on/off at crossover points). Zones 0, A, B, D give actual duty = input% exactly. Zone C uses on growing linearly 400→800ms with off derived from the ratio, giving near-exact tracking.

---

## H Controller (h_new) — source directory split

`controller_h` and `controller_h_new` now build from separate source directories:

| Environment | Source | Description |
|---|---|---|
| `controller_h` | `src/h_controller/` | Stable production firmware (main branch) |
| `controller_h_new` | `src/h_controller_new/` | New H controller firmware (h_new branch) |

All changes below apply only to `controller_h_new` unless stated otherwise.

---

## H Controller (h_new) — `calcPred` rewrite: per-level lookup table

Replaced the global log/exp blended formula with a 20-entry lookup table keyed by exact heater level percentage. Formula per level: `pump = k / (85 − hp)^alpha`, clamped to [4, 100].

k and alpha fitted from SCT calibration data via log-linear regression per level, then smoothed as degree-2 polynomials in `log(pct)` to eliminate crossovers. Forced levels (5%, 10%, 14%) use proportionally scaled k from the 20% fit: `k = k_at_20 × (pct / 20.0)`, `alpha = alpha_at_20`.

Lookup uses an exact match scan (levels are always one of the 20 discrete values — no interpolation needed). Falls through to the last entry (100%) if no match.

| Level % | k | alpha |
|---|---|---|
| 5 | 32.3 | 0.811 |
| 10 | 64.6 | 0.811 |
| 14 | 90.4 | 0.811 |
| 20 | 129.2 | 0.811 |
| 25 | 417.8 | 1.079 |
| 29 | 838.7 | 1.232 |
| 33 | 1456.7 | 1.349 |
| 40 | 3015.8 | 1.494 |
| 43 | 3850.9 | 1.539 |
| 50 | 6090.9 | 1.619 |
| 57 | 8575.3 | 1.671 |
| 60 | 9665.5 | 1.687 |
| 67 | 12168.6 | 1.713 |
| 71 | 13531.5 | 1.722 |
| 75 | 14819.5 | 1.728 |
| 80 | 16301.9 | 1.732 |
| 86 | 17868.6 | 1.731 |
| 90 | 18775.9 | 1.728 |
| 95 | 19752.6 | 1.722 |
| 100 | 20556.2 | 1.714 |

---

## H Controller (h_new) — H pump: hot pipe ≥ 80°C forces 100%

In `calcHPumpDuty()`, after reading `hotPipeC`: if `hotPipeC >= 80.0f` return 100% immediately. Applies when heater is on — the heater check gates the function earlier. Rationale: at high inlet temperatures even low power levels risk pushing the outlet over 85°C without full pump.

---

## H Controller (h_new) — overheat thresholds reworked

Previous behaviour (from main branch):
- 91–92°C: absolute heater cap 100%→0%
- ≥ 93°C: hard lockout

New behaviour:
- **91–93.5°C**: proportional heater cap — scales the *current demand level* linearly from 100% to 0% over this range: `cap = heaterLevelPct() × (1 − (hOut − 91) / 2.5)`. Regardless of what the heater is set to, reduction starts at 91°C and reaches 0% at 93.5°C.
- **≥ 94°C**: hard lockout (was 93°C)

Using proportional rather than absolute cap means the reduction starts immediately at 91°C for any demand level, not only when the absolute cap drops below the current level.

Hard lockout auto-clears when outlet < 88°C with at least one sensor live (unchanged).

---

## H Controller (h_new) — SCT CSV: W pump column added

`sct.csv` now has 9 columns (was 8):

`timestamp, pct, hot_pipe, htr1, htr2, h_pump_pct, w_pump_pct, pred, event`

`w_pump_pct` = `lastWPkt.solarPumpDutyPct` at time of log row; 0 if no W packet received.

`make_hpump_cal.py` (both `logs/` and `logs/new_pump/`) updated:
- Detects format by column count: `len(row) >= 9` → new format
- Reads event from index 8 (new) or index 7 (old)
- Discards any data row where `w_pump_pct > 0` and resets the stability buffer — W solar pump running changes thermal conditions and invalidates H pump calibration data for that row
