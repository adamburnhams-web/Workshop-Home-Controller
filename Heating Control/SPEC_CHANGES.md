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

**Auto mode duty calculation** (per RS485 packet, ~250ms) — updated, current values:
```
SOC reservation:
  soc > 95%:                          reservationW = 0W     (battery full, no headroom needed)
  soc > 85%:                          reservationW = 200W   (regardless of time)
  before 11:00:00:      soc ≤20% 3000W,  21-25% 1000W,  >25% 500W
  11:00:00-13:00:00:    soc ≤40% 3000W,  41-55% 1000W,  >55% 500W
  13:00:01-13:59:59:    soc ≤50% 3000W,  51-80% 1000W,  >80% 400W
  14:00:00 onward:      soc ≤60% 3000W,  >60% 1000W

available = pvExportW − gridImportW + battChargeW + heaterCurrentW − reservationW

heaterCurrentW = heaterLevelPct10() × 3   (level-based, was heaterTargetPct × 30)
rawPct         = clamp(available × 100 / 3000, 0, 100)
```
**Updated (Aug 2026)**: the single 14:00 cutoff (500W before / 1000-3000W after, by SOC) replaced with four time-of-day tiers, each with its own SOC breakpoints — the SOC threshold for the 3kW/1kW/500W split tightens as the day's solar window narrows, so the heater gives up more of its claimed export earlier in the afternoon rather than only at a single hard 14:00 line. `rtcMinute()`/`rtcSecond()` forward-declared alongside the existing `rtcHour()` to build the second-of-day comparison (`nowSec`).

Previously added: the `soc > 95% → 0W` tier (previously the top tier was `soc > 85% → 200W`, unconditionally). Since `reservationW` can now be 0, the formula above is applied unconditionally rather than switching to the old dead `else` branch this section previously described — that branch (`netGridW + min(0, battChargeW) + heaterCurrentW − 100`, along with the now-unused `netGridW`/`battW` locals) has been removed rather than resurrected.

Superseded values: reservation previously switched at noon (not 14:00), was 1kW before noon / 0W until SOC > 80% after noon (then 1kW) — see git history for the exact prior thresholds. The 200W-reservation SOC gate was originally 95%, lowered to 85% to release the heater's power cap sooner once the battery is nearly full (95% is now the threshold for the new 0W tier instead).

**Start:** rawPct ≥ 17% (~500W) AND sustained for 5s  
**Stop:** 5 consecutive zero packets (was immediate)  
**Smoothing:** true rolling average of the last 8 samples, recomputed every packet (was: batch-averaged every 8th packet only — see "H Controller — heater duty rolling average" below)

**Hot pipe cap** (applied after smoothing):
- `hot_pipe ≥ 90°C` → 0%
- `hot_pipe 63–90°C` → linear cap: `(90 − hot_pipe) × 3.70%/°C` (100% at 63°C, 0% at 90°C) — threshold raised from 60°C

**Flow-path valve interlock**: see "H Controller (main) — heater control rework ported from h_new" below — `botTankValve`/`logBurnerCold`/`twoPortValve` are now actively requested into position whenever the heater wants to run, with actual firing gated on confirmation.

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

- Both sensors faulted → power cap set to 0 immediately (ISR-level cap), set both fault flags
- Single sensor faulted → raise fault flag after 5s grace; continue on the remaining sensor
- Hot pipe sensor fault → power cap set to 0 immediately
- RS485 stale > 60s while running → heater level forced to 0, `heaterRunning = false`

**Overheat sequencing** (replaces warn→shutdown threshold at 91°C):
- **91–93°C**: ramp power cap 100→0 (ISR-level cap); request valves open
- **≥ 94°C**: power cap 0, `heaterHardLockout = true`, SSR pin cleared, set `FAULT_H_HEATER_OVERHEAT_SHUT`
- **Auto-clear** hard lockout when effective heater temp < 88°C AND at least one sensor live (clears fault too)
- **Page 5 Ack** also clears hard lockout immediately (allows manual recovery without reboot)
- **≥ 95°C**: additionally sets `heaterManualLockout = true` + `FAULT_H_HEATER_MANUAL_LOCKOUT` (new bit 14) — everything the ≥94°C hard lockout does, plus it does **not** auto-clear at <88°C. Only page 4 "Alrt Reset" clears it (re-latches immediately on the next check if still ≥94/95°C). Reuses the existing generic mid-point-LED flash on W (any nonzero `hFaultFlags` bit already flashes it 1s on/1s off) — no W-side change needed.

Thresholds/behavior above are unchanged, but the implementation is now level-based (see "H Controller (main) — heater control rework ported from h_new" below): `heaterPowerCapPct`/`heaterTargetPct`/`heaterSpreadAcc` (percent-based) were replaced with `heaterLevelCap`/`heaterLevelIdx` (0–20 level index) + `heaterPhaseHc`/`heaterPhaseOn` (ISR burst-phase tracking), driving the same 20-level fixed-burst table used by `controller_h_new`.

Heater gate in ISR also requires `VSTATE_SOLAR_COLD_OPEN` — heater cannot fire if the solar cold valve is closed.

### H-side solar pump direct drive — rework — superseded

**`calcHPumpDuty()` now returns `float`** (was `uint8_t`).

New predictive formula `calcPred(heaterPct, hotPipeC)` (historical — see supersession note below):
```
excess   = heaterPct − 20
pt       = max(0, excess²) × 0.0025        // plateau term
tr       = 0.09 + max(0, excess) × 0.022   // thermal rise slope
raw      = 4 + pt + tr × (hotPipeC − 30)
minDuty  = 4 + heaterPct × 0.05
pred     = max(raw, minDuty)
```

**Two operating branches (historical):**

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

**Superseded**: both `calcPred()` and `calcHPumpDuty()` above were replaced with `controller_h_new`'s versions — see "H Controller (h_new) — `calcPred` rewrite: per-level lookup table" and "H Controller (main) — heater control rework ported from h_new" below. The per-level `k`/`alpha` lookup table replaces the quadratic formula; `upperMult` now scales with hot-pipe temperature in both branches instead of only in MAX mode.

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

## H Controller — 12V PSU relay thresholds/debounce

Thresholds and debounce revised again after commissioning — supersedes the 12.0V/3s-on, 12.5V/5s-off values this section previously documented:
- **ON**: below 12.5V continuously for 5s → relay energises (`BUS_PSU_THRESH_DV = 125`, `BUS_PSU_ON_DELAY_MS = 5000`)
- **OFF**: at/above 13.0V → relay de-energises immediately, no debounce (`BUS_PSU_HYSTERESIS_DV = 130`)

Originally (pre-debounce) the relay switched immediately on threshold crossing with no delay in either direction.

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
| D24 | Solar cold valve direction relay |
| D26 | Alarm sounder |
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

### Overtime fault — isolation valve now closes

`VAC_PUMPING` → `VAC_FAULT` (pump ran `VAC_PUMP_MAX_MS` without `vacSensorFull`) previously only turned the pump off, leaving `vacIsoValve` open on fault. Now also calls `vacIsoValve.request(false)` so the isolation valve closes along with the pump stopping — an overtime fault means something's wrong with the vacuum circuit, so it shouldn't stay exposed after the fault latches.

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

### Spread firing (Bresenham) replaces burst firing — superseded
Spec/previous: SSR fired for the first N consecutive half-cycles of a 100-cycle window (burst).
Code (historical): Bresenham error-accumulator distributed firing events evenly across all half-cycles.

Effect: at 10% duty the pattern was one pulse every 10 half-cycles (every 100ms) rather than 10 pulses then 900ms silence. Export meters saw near-constant load regardless of polling interval, eliminating false over-export readings.

ISR variable renamed: `heaterCycleCnt` → `heaterSpreadAcc` (accumulator, not a counter).

**Superseded** — see "H Controller (main) — heater control rework ported from h_new" below: `controller_h` now uses the same 20-level fixed-burst scheme as `controller_h_new`; `heaterSpreadAcc` removed.

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
- H's pump duty (`lastH.hPumpDutyPct`) is greater than or equal to W's calculated duty
- Solar hot is below target

W resumes output when solar hot reaches target or W's duty exceeds H's. Diode-OR on the MOSFET gate means the higher of the two signals drives the pump regardless.

**Updated**: comparison changed from strictly-greater to greater-or-equal (`>` → `>=`, both winter and summer solar paths). Under the old strict comparison, a tie (H and W independently computing the same duty, e.g. both at 4%) left neither side backing off — both pumps ran concurrently, compounding to roughly double the intended flow. `>=` means W now also stands down on a tie, since H always drives its own pump unconditionally (H has no equivalent check against W's duty).

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
- **91–93°C**: proportional heater cap — scales the *current demand level* linearly from 100% to 0% over this range: `cap = heaterLevelPct() × (1 − (hOut − 91) / 2)`. Regardless of what the heater is set to, reduction starts at 91°C and reaches 0% at 93°C.
- **≥ 94°C**: hard lockout (was 93°C)
- Overheat warning fault (`FAULT_H_HEATER_OVERHEAT_WARN`) removed — only the hard shutdown fault (`FAULT_H_HEATER_OVERHEAT_SHUT`) remains.

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

---

## H Controller (both) — stagnation flush (`updateFlush`)

Applies to both `controller_h` and `controller_h_new`.

**Purpose:** push trapped air out of the heater circuit. When the immersion heater is running and the two outlet sensors read differently, it indicates an air pocket near one sensor causing localised overheating rather than full water circulation. Running the pump at 100% for a short burst flushes the air pocket through.

**Condition:** all of the following must hold:
- Heater running
- Both `htr_out` and `htr_out_2` sensors valid (not faulted)
- Effective heater outlet (`getHeaterOutC()`) ≥ 80°C
- `htr_out_2 − htr_out > 3.5°C`

**State machine** (`FlushState`):
- `FLUSH_IDLE` → `FLUSH_ACTIVE` when condition first true
- `FLUSH_ACTIVE` (pump forced to 100%) → `FLUSH_PAUSE` after 4s
- `FLUSH_PAUSE` → `FLUSH_ACTIVE` if condition still true after 4s; → `FLUSH_IDLE` if condition cleared
- Resets to `FLUSH_IDLE` immediately if heater stops

`FLUSH_ACTIVE` overrides `calcHPumpDuty()` in the loop; in debug builds it also takes priority over `simHPumpSpdActive` but yields to `PT_COOLDOWN`.

---

## H Controller (main) — heater control rework ported from h_new

Brings `controller_h`'s heater subsystem (levels, pump formula, overheat handling) in line with `controller_h_new`, explicitly excluding the SCT/`soc_test` feature (which remains `h_new`-only).

### 20-level fixed-burst heater scheme (replaces Bresenham spread-firing)

`controller_h` now uses the same `kHeaterLevels[20]` fixed-burst table as `controller_h_new` (see "H Controller — heater level table corrections" above for the two corrected entries — both builds share the corrected values). Each level fires `on_hc` half-cycles then `off_hc` off, repeating, replacing the previous Bresenham per-half-cycle accumulator (see "Spread firing (Bresenham)..." above, superseded).

State renamed/replaced: `heaterSpreadAcc`/`heaterTargetPct`/`heaterPowerCapPct` (percent-based) → `heaterLevelIdx`/`heaterLevelCap` (0–20 level index) + `heaterPhaseHc`/`heaterPhaseOn` (ISR burst-phase tracking). New helpers `pctToLevel()`, `heaterLevelPct()`, `heaterLevelPct10()` convert between percent and level index.

The duplicate 91–92°C overheat power-reduction block previously inline in `updateHeaterDuty()` was removed — `checkHeaterFaults()`'s `heaterLevelCap` taper (91–93°C ramp, ≥94°C hard lockout — thresholds unchanged) already covers it.

### Pump speed formula ported from h_new

`calcPred()` and `calcHPumpDuty()` replaced with `h_new`'s versions (see "H Controller (h_new) — `calcPred` rewrite" and "H pump: hot pipe ≥ 80°C forces 100%" above) — same per-level `k`/`alpha` lookup table, `hotPipeC ≥ 80°C → 100%` fast path, `normalMode`/`effTarget` restructuring, and hot-pipe-scaled `upperMult` ceiling applied in both normal and MAX modes (was MAX-mode-only, pred-based). `socTestPriming` check omitted — no SCT feature on this build.

### Auto-mode reservation and hot-pipe-cap tuning (main-only, not mirrored to h_new)

See updated "Auto mode duty calculation" and "Hot pipe cap" above — noon → 14:00, 1kW → 500W before the threshold, new 3kW/1kW post-threshold SOC bands, 200W override above 95% SOC, and hot-pipe-cap start raised 60°C → 63°C. `controller_h_new` keeps its own separate (more granular) reservation scheme and was not changed.

### Heater flow-path interlock — 2-port valve added, active valve request

`heaterFlowPathOk()` now also requires the 2-port valve open (heater side) and idle, in addition to the existing solar-cold-open + (log-burner-cold or bot-tank-valve open+idle) checks. The heater is blocked while the 2-port valve is moving or parked mid-tank (shared with solar's own routing of the same valve).

`updateHeaterDuty()` reworked so the heater actively calls for its flow-path valves (`logBurnerCold` close, `botTankValve` open, `twoPortValve` open) as soon as the power-surplus hysteresis decides it wants to run — mirroring the existing `MHM_FORCE_ON` behavior — rather than waiting passively for another subsystem (e.g. the solar valve state machine) to position them. The power-surplus rolling average keeps accumulating in the background regardless of valve position; actual firing (`heaterLevelIdx`) stays gated on `heaterFlowPathOk()` and takes effect immediately once the valves are confirmed in place, without re-averaging. `MHM_FORCE_ON` reworked the same way: `heaterRunning` stays true while waiting for the valves; only `heaterLevelIdx` is gated.

### Page 2 display — Consumption row added

New "Cons:" row (instantaneous Watts: `PV1+PV2 + Import − BattCharge − Heater`, matching the existing `kwhConsumption` accumulator formula) added under Import. Heater row moved to directly below Consumption, above Battery (was: after SOC, at the bottom of the power block).

---

## RS485 Comms Fault — threshold 60s → 120s

Raised on both controllers: `COMMS_FAULT_THRESHOLD` 240 → 480 (`controller_h`, 250ms poll heartbeat) and `COMMS_FAULT_TIMEOUT_MS` 60000 → 120000 (`controller_w`). Link is confirmed functional for current needs; 120s reduces false-positive fault entries from brief drop-outs (the officially documented figure was already stale — spec said "5 consecutive packets missed (~1.25s)" when the code has always been a 60s-since-last-good-packet timer). `controller_h_new` was left at its own existing (shorter, WIP) threshold, not touched here. The separate 60s heater-safety cutoff (`updateHeaterDuty()`: no W packet for 60s → heater forced off) is a distinct mechanism and was not changed.

## Heater/grid-import trip fault (H controller, both builds)

New `checkHeaterImportTrip()`: if SOC > 15%, battery isn't discharging faster than 3.9A-equivalent, and grid import stays > 100W for 20s continuously while Growatt data is valid, the heater is force-stopped (`heaterRunning = false`, `heaterLevelIdx = 0`) and `FAULT_H_HEATER_IMPORT_TRIP` (new bit, `hFaultFlags` bit 13) is raised — catches the inverter/CB having tripped while panels are still producing, so the heater would otherwise keep calling for power the array can't actually deliver, silently pulling from the grid. Clears after import stays ≤0W for 20s. Unused `FAULT_H_HEATER_OVERHEAT_WARN` bit removed (superseded by the level-based overheat taper in `checkHeaterFaults()`; nothing set it anymore). `botTankOpen` (bottom-tank valve state) added to the H→W packet so W can see it — used by the new summer-solar heater-side idle check below.

**Dawn-blip hold, widened + boot-arm fix (Aug 2026)**: Growatt pauses battery discharge and pulls a small fixed import (~210-220W) during its dawn PV/grid-sync startup every morning as soon as PV first registers after a night at zero — that reads as `importHigh` but isn't a real trip, so the check is held off for `DAWN_HOLD_MS` once a qualifying zero→nonzero PV transition is seen (qualifying = PV was zero for ≥20 min, `NIGHT_PV_ZERO_MS`). Two mornings tripped the fault through the original 7-minute hold (blip duration varies morning to morning), so `DAWN_HOLD_MS` is now 60 min.

Also fixed while investigating: `pvZeroSinceMs` defaults to 0 at boot and is only updated on an *observed* nonzero→zero transition, so a reboot (watchdog reset, brownout — this system has had bus-voltage issues, see PSU relay work above) occurring less than 20 minutes before sunrise left the 20-min-of-night check unable to pass, so the dawn hold silently failed to arm for that one morning — the trip check then ran completely unprotected straight through the real Growatt blip. New `pvZeroTrackedOnce` flag: the very first zero→nonzero PV transition seen after any boot now always arms the hold, regardless of measured duration; every dawn after that uses the real event-driven duration as before (self-heals from the first full night/dawn cycle post-boot).

## Summer solar startup / end-of-day rework (W controller)

- **Start trigger**: raised from `hot/cold ≥ 50°C` to `hot/cold ≥ 80°C`, OR either solar pipe above H's reported tank-mid temperature (worth circulating even if not yet "hot") — replaces the old PV-export-based trigger entirely.
- **Solar target**: `tankTop + 13°C` (was `+8°C`), still capped at 87°C.
- **Heater-side idle check**: the "don't run solar into a heater-cooling loop" skip now also requires `botTankOpen` (bottom-tank valve confirmed open), not just `twoPortHeaterSide` — avoids a false idle read while the valve is still moving.
- **`pvActive` drop-out window**: extended 5 → 10 minutes below 200W before latching false (reduces cycling on brief cloud cover); comment updated to match.
- **End-of-day abort**: replaced again — was a compound PV/temperature-equilibrium heuristic (both solar pipes below tank-mid, PV < 300W, hot~cold within 6°C, heater off, SOC < 95%, sustained 10 min); now a straight comparison against today's actual sunset time. See "W Controller — summer solar end-of-day: sunset-table cutoff" below.

### Low-differential pump suppression (new)

`hot − cold < 0.5°C` sustained for 5 continuous minutes means the pump is circulating with no useful solar gain (fully mixed loop), so `finalDuty` is forced to 0 — same force-0 style as the existing heater-side-idle check above, gated by the same target/80°C/tank-top caps so it releases the moment real gain (or a gate condition) returns.

The sustained timer (`lowDeltaSinceMs`) resets immediately the instant the delta rises back above 0.5°C. A separate `lowDeltaWasForced` latch handles the case where the delta stays low but a gate trips (e.g. hot momentarily ≥ 80°C) mid-suppression: on the next gate-ok tick the timer is reset rather than left at its already-sustained value, so re-suppression always needs a fresh 5 minutes instead of re-firing the instant the gate clears — while the timer is otherwise left alone tick-to-tick during continuous suppression, so suppression doesn't self-interrupt every 5 minutes.

## Winch over-open fault — actually clears now

`updateWinchInputs()`: the safety-limit-switch-opens branch previously had only a comment claiming the fault "clears automatically" with no code doing it (`FAULT_W_WINCH_OVER_OPEN` stayed latched until the next full alert-reset). Now calls `clearFault(FAULT_W_WINCH_OVER_OPEN)` there, matching the documented behavior.

## H controller_new — tiered per-sensor fault debounce (WIP, not yet ported to controller_h)

New `sensorFaultGraceMs[]` per sensor: tank/cold-pipe sensors tolerate 8h of stale/failed reads before `sFault[]` actually flips (holds last known value); hot-pipe gets 15s (short, since `checkHeaterFaults()` cuts the heater the moment it faults); heater-outlet sensors are excluded (grace handled separately, below). A new fast `sRawFault[]` (3-sample debounce, no grace) drives the page-1 red "FAULT" readout immediately, decoupled from the slower, grace-gated `sFault[]` that gates the logged fault flag/banner/LED.

Heater-outlet single-sensor-fault handling reworked: previously raised `FAULT_H_SENSOR_HEATER_OUT(_2)` immediately after a 5s grace while continuing to run on the other sensor. Now: continues on the remaining sensor for 90s, then cuts `heaterLevelCap` to 0 (heater stops firing) while still not raising the fault flag until 120s — separates "stop producing heat because we've lost confidence" from "escalate to a logged fault," giving 30s of no-heat-but-no-alarm before the fault actually logs.

---

## H Controller — page 3 fault log: stuck-active entries fixed

`faultLogUpdate()` (which marks a page-3 fault-history entry resolved) was only invoked from the main loop when `curWF != prevWF || hFaultFlags != 0` — i.e. gated on `hFaultFlags` being nonzero. The one frame where an H-side fault actually clears (`hFaultFlags` transitions to 0) is exactly the frame that condition goes false if the W-side flags happen not to have changed on the same tick, so the clearing edge was silently skipped: the bottom status banner (which reads `hFaultFlags` live) correctly went quiet, but the page-3 entry stayed red with only a start time, `resolvedMs` never set, until some unrelated W-fault change incidentally triggered the next call.

Fix: added a `prevHF` shadow of `hFaultFlags`, compared the same way `prevWF` already was (`curWF != prevWF || hFaultFlags != prevHF`), so any change — including clearing — reliably triggers `faultLogUpdate()`.

## H Controller — heater duty rolling average (was: batch average)

`updateHeaterDuty()`'s `rawPct` smoothing was previously a batch average: it accumulated 8 packets (~2s) into a sum, then applied the average to `heaterLevelIdx` once every 8th packet, leaving the applied level untouched (and no smoothing in progress) on the other 7. A step change in available power landing mid-window could take up to ~4s (two windows) to fully settle, in one or two discrete jumps rather than a ramp.

Replaced with a true rolling window: a circular buffer plus a running sum, with the oldest sample evicted and the newest added every packet (no re-summing). `heaterLevelIdx` is now recomputed from `rawPctSum / rawPctBufCount` on **every** packet (~250ms), using the actual sample count (not always the full window) while the buffer is still filling after a fresh start. All existing reset points (SOC-mode toggle, heater stop, 5-consecutive-zero-packet shutoff) reset the buffer/count/sum the same way the old accumulator was reset, so a new run never blends in stale samples from before a stop.

Window widened from 8 to **12** samples (~3s at the 250ms packet rate) — smoother output at the cost of a slightly longer settling time on a genuine step change.

## H Controller — page 2: gross import/export split (was: net only)

`kwhImport` previously accumulated **net** grid flow (`gridImportW − pvExportW`; negative during net export), so any export simply reduced the displayed import figure — gross daily import and gross daily export were not separately visible.

Since `gridImportW` and `pvExportW` are mutually-exclusive Growatt registers (only one is nonzero at a time), the existing net value's positive/negative parts exactly recover true gross import and export without needing new packet fields:
- `kwhImport` now accumulates only the positive part of net (`max(net, 0)`) — gross import only
- New `kwhExport` accumulates only the negative part (`max(-net, 0)`) — gross export only

Page 2 gained a third daily-totals row: `PVkWh`/`ImpkWh`, `ExpkWh`/`HtrkWh`, `ConsKWh`. The `Cons:`/`ConsKWh` consumption calculation is unchanged and still uses the net import value internally (`energyLastImportW`) — only the accumulated/displayed totals were split. The SD energy-log CSV columns are unchanged (still `kwhPV,kwhImport,kwhHeater,kwhConsumption,soc`) — `kwhExport` is not yet logged to SD.

## H Controller — heater level table: 20 → 16 levels

Four levels removed from `kHeaterLevels[]`: 5.0%, 42.9%, 57.1%, 95.0% (previously levels 1, 9, 11, 19 of 20). Table size is now driven by a named `HEATER_LEVEL_COUNT = 16` constant rather than a hardcoded `20` scattered across the file — `pctToLevel()`'s scan bound, `heaterLevelCap`'s "no cap" sentinel/reset value (all three reset sites: normal idle, overheat-clear, page-4 alert-reset), and `pkt.heaterRestricted`'s comparison all now reference it.

`calcHPumpDuty()`'s `calcPred()` per-level k/alpha lookup table (`LEVELS[]`/`K_TBL[]`/`A_TBL[]`, see "H Controller (h_new) — `calcPred` rewrite" above) had its 4 corresponding rows (5%, 43%, 57%, 95%) dropped in lockstep — no refit needed, since each row is an independent per-level fit and the remaining levels' calibration data is unaffected by removing others. Scan bound changed from `< 19` to `< HEATER_LEVEL_COUNT - 1`.

Test sweep tables (`PUMP_TEST_POWERS[]`, `STRESS_TEST_POWERS[]`) are unaffected — they specify arbitrary target percentages resolved through `pctToLevel()` at runtime, not tied to a fixed level count.

## W Controller — summer solar end-of-day: sunset-table cutoff

Replaces the compound PV/temperature-equilibrium end-of-day heuristic in `updateSummerSolar()` (both solar pipes below tank-mid, PV < 300W, hot~cold within 6°C, heater off, SOC < 95%, all sustained 10 minutes) with a direct comparison against today's actual civil sunset time for Alton, UK. Past sunset there's no more solar gain physically possible regardless of what the pipes/PV/SOC currently read, so the heuristic is gone entirely rather than combined with the time check.

New `kSunsetUtcMin[365]` PROGMEM table (minutes-since-midnight-UTC sunset, indexed by day-of-year on a non-leap reference calendar), plus supporting helpers, all new in `w_controller/main.cpp` ahead of `updateSummerSolar()`:
- `dayOfYearForSunset(month, day)` — maps a calendar date onto the 1–365 table index. On a leap year, **Feb 29 reuses the Feb 28 entry** (day 59) rather than shifting the table, so every date from Mar 1 onward stays aligned with the non-leap reference regardless of the current year.
- `dayOfWeek()` (Sakamoto's algorithm), `daysInMonth()`, `lastSundayOfMonth()`, `isBST()` — the table is UTC (per the source data), but the RTC is kept in UK civil (wall-clock) time, so the local comparison needs to know whether British Summer Time is in effect. UK clocks go forward the last Sunday of March and back the last Sunday of October (~01:00 UTC each time); the exact transition hour is ignored since this only feeds an evening sunset check, nowhere near 1am.
- `sunsetLocalMinutes()` — today's sunset converted to UK local minutes-since-midnight (`sunsetUtc + (isBST ? 60 : 0)`).

W previously only captured `syncHour`/`syncMinute`/`syncSecond` from H's hourly time-sync packet (`HToWPacket`); `syncDay`/`syncMonth`/`syncYear` were already being sent but unused. Added `curDay`/`curMonth`/`curYear` to W's local state, captured alongside the existing time fields at the same sync point.

`updateSummerSolar()`'s end-of-day check is now: `getCurrentTime()` (existing live-clock estimate, ticks between hourly syncs) compared against `sunsetLocalMinutes()`. No debounce needed — unlike the old PV/temperature reading, a time comparison doesn't fluctuate, so it's a clean one-way trigger for the rest of the day once past sunset. Before the first time sync, `getCurrentTime()` defaults to noon, which is always before sunset, so the check simply doesn't fire until a real sync arrives.

**Placement matters**: the check sits immediately after the critical-overheat block and *before* the start-trigger logic, with an early `return` when past sunset — it does not run only as a teardown at the end of the function. An earlier version placed it at the end instead, which meant a start condition (e.g. hot pipe ≥ 80°C) true at the same time as past-sunset would open the valve and start the pump via the start-trigger block, then have the end-of-day teardown immediately undo it on the very same tick — `summerPhase` resets to `SUMPH_IDLE` either way, so the next loop (~250ms later) would repeat the exact same start-then-stop sequence indefinitely for as long as both conditions held, chattering the solar valve every loop. Gating at the top prevents the start logic from ever running once past sunset, so there's nothing to undo.

**Not affected**: manual heater-on (`MHM_FORCE_ON`) at H has no time-of-day awareness and isn't gated by this at all — it runs any time. A separate, unconditional check in the main W loop (`if (heaterPowerPct > 0) solarColdValve.setOpen();`, pre-existing, outside `updateSummerSolar()`) keeps the return flow path open for H's own direct-drive pump regardless of season/sunset. Only W's own solar collector pump is affected by the sunset cutoff, which is correct — there's no collector gain after dark regardless of what the heater is doing.

**Year handling**: `dayOfYearForSunset()` only uses month/day, never year, so the 2026-sourced table is reused unchanged for any year (sunset time for a given calendar date drifts at most ~1 minute year to year — immaterial here). `isBST()` is the only year-dependent piece (the last Sunday of March/October moves each year) and is parameterized by the actual synced year, computed live via Sakamoto's algorithm rather than a fixed table — so it doesn't run out or need updating for future years, short of the UK changing its DST rule.

## H Controller — automatic BST/GMT clock adjustment

The DS3231 RTC has no timezone/DST concept — previously it just free-ran from whatever date/time was last manually set via page 4 (Set Hour/Min/Sec) or the `rtc` serial command, meaning BST changeovers required a manual ±1h adjustment twice a year or the whole system's wall-clock read an hour wrong for half the year.

`checkAutoBST()` (called from `readRTC()`, so once per second alongside the existing RTC poll) now does this automatically, while keeping the existing UX unchanged — installers still set the RTC to whatever a wall clock reads, nothing about page 4 changes:
- Ports the same `dayOfWeek()`/`daysInMonth()`/`lastSundayOfMonth()`/`isBST()` date-math helpers added for the W-side sunset feature (see above) into `h_controller/main.cpp`, since H owns the actual RTC hardware.
- Compares today's computed BST/GMT state against a 1-byte EEPROM flag (`EE_BST_STATE`, new address 5). On mismatch, shifts the RTC by exactly ±1h via `rtc.adjust(rtcNow ± TimeSpan(3600))` and updates the flag — so the shift only ever fires once per transition, and survives a reboot near the transition boundary without double-applying or silently missing it.
- Gated on `rtcNow.hour() >= 3` before acting (mismatch detection still runs every second, but the shift itself waits): applying at midnight would either jump the clock an hour early (spring) or roll the *date* backward a day (autumn, subtracting 1h from just after midnight) — the latter would double-trigger the SD energy log's day-rollover reset. Waiting until hour ≥ 3 keeps both directions clear of any midnight boundary. Net effect: the clock settles to the correct offset within the first few hours of the transition day rather than at the exact 01:00 UTC instant — acceptable imprecision for a heating controller, nothing time-sensitive runs at 1–3am.
- First-ever boot (fresh/blank EEPROM byte, i.e. not 0 or 1): adopts the current computed state without shifting, so it doesn't surprise-shift a clock the installer just set by hand for the first time.

## H Controller — hot-pipe rate-of-rise pump override

Analysis of `Data/LOG.CSV` (~12 days, 20 historical crossings of the existing 91°C heater-outlet pump spike) found two distinct overheat mechanisms, not one:
- **Fast-rise (7 of 20)**: hot_pipe climbing 7–11°C/min while H-pump duty was still low/ramping (19–44%) — morning solar-surge events (~09:25–09:55) where hot_pipe outruns the pump before `calcHPumpDuty()`'s normal curve catches up.
- **Pump-saturated (13 of 20)**: hot_pipe climbing slowly (0–2.3°C/min) but pump was already at 68–100% duty at the 86°C crossing — the duty curve simply has no more headroom. A rate rule doesn't address this class; it would need a heater-power-cap fix instead.

First pass used a 60s-trailing-window average (7°C/min threshold), which cleanly separated the 7 fast-rise events from 48 non-overheat 86°C+ cycles with 0 false positives — but a 60s average dilutes a short sharp burst (e.g. 4°C in 30s preceded by flat readings averages to only 4°C/min over the full window, well under threshold). Replaced with a much shorter, more responsive window plus an explicit hold so a single-shot trigger doesn't chatter:

**Final rule** — `heaterOutC >= 86°C` AND hot_pipe risen `>= 1.0°C` within the trailing 10s. On trigger, pump is forced to 100% for a rolling 15s hold that keeps re-extending every loop the criteria is still true (so it releases 15s after the *last* time the fast rise was seen, not 15s after the first). Simulated against the full 12-day log this fires 18 independent hold periods (~1.5/day) — clustered around the same known morning-surge windows, not spurious noise.

**Implementation** (`h_controller/main.cpp`):
- `updateHotPipeRiseBuf()` (called once per `loop()`, alongside `readSensors()`) samples `hot_pipe` every 2s into a 6-slot circular buffer (~10–12s of history). Buffer resets on `H_SENSOR_HOT_PIPE` fault so a stale pre-fault reading never contaminates the calc after recovery.
- `getHotPipeRise10sC()` returns the °C delta against the oldest buffered sample, or `NAN` until the buffer has filled (~12s after boot/fault recovery).
- `calcHPumpDuty()` gains an early-out block with a local static hold latch (`hpHoldStartMs`/`hpHolding`): criteria true → hold timer resets to now; hold releases once 15s have elapsed since the criteria was last true. Evaluated right after the existing `hotPipeC >= 80.0f → 100%` override and before the normal/MAX-mode duty curves — doesn't touch the existing 90°C/91°C heaterOutC-based ramp-to-100% logic, this is a strictly earlier, additional trigger for the fast-surge case.

## H Controller — heater-outlet imbalance pulse (`updateImbalancePulse`)

A second, milder response to the same `htr_out`/`htr_out_2` disagreement that drives the stagnation flush above, but for the 65–80°C band below the flush's 80°C floor — the two never overlap. Intent: nudge a stalled/stratified pocket at one outlet sensor loose well before it reaches flush territory, without the flush's full 4s-on burst.

**Condition:** heater running, both outlet sensors valid, and the two disagree by more than 10°C (`|htr_out − htr_out_2| > 10°C`). The higher of the two readings selects the pulse timing:
- 65°C ≤ high reading < 70°C → 0.5s on / 20s off
- 70°C ≤ high reading < 80°C → 0.5s on / 10s off
- Outside 65–80°C, or disagreement ≤ 10°C → idle (no pulse)

**State machine** (`ImbalPulseState`): `IMBAL_IDLE` → `IMBAL_ON` (pump forced 100%) → `IMBAL_OFF` → back to `IMBAL_ON` if the condition still holds, else `IMBAL_IDLE`. Resets to `IMBAL_IDLE` immediately if the heater stops or either sensor faults.

Only `IMBAL_ON` is dispatched as an override (pump forced to 100%) — `IMBAL_OFF` is not special-cased, so it falls through to `calcHPumpDuty()` and the pump runs its normal computed duty for the 10s/20s gap between pulses, rather than sitting off. This is an *additional* short spike on top of whatever the pump is already doing, not a full on/off replacement of normal control. Evaluated in the main loop right after `updateFlush()`; in debug builds `IMBAL_ON` outranks `simHPumpSpdActive` but yields to `PT_COOLDOWN` and `FLUSH_ACTIVE`.

The original stagnation flush (`FLUSH_ACTIVE`/`FLUSH_PAUSE`, above) already worked this way — only `FLUSH_ACTIVE` is dispatched as a 100% override; `FLUSH_PAUSE` was never special-cased and has always fallen through to `calcHPumpDuty()`, so the 4s gap between flush bursts already runs normal duty rather than forcing the pump off.

## H Controller — `calcHPumpDuty()` below-target floor: 0.9 → 0.8, ramp band 7°C → 10°C

Both branches' low-end floor (well below target, where duty was pinned at a flat `pred × 0.9`) dropped to `pred × 0.8`, with the ramp back up to full `pred` starting 10°C below target instead of 7°C:

- **Normal mode** (`solarTargetMode == SOLAR_TANK_PLUS8`): floor applies below `effTarget − 10°C` (was `− 7°C`); ramps 0.8→1.0×`pred` from `effTarget − 10°C` to `effTarget`.
- **MAX mode** (or tank-top fault, fixed 85°C target): floor applies below 75°C (was 78°C); ramps 0.8→1.0×`pred` from 75°C to 85°C.

Same shape as before, just a slightly deeper underdrive further from target and a wider ramp band.
