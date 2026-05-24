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

### Heater SSR — SOC-based charge rate control (summer mode)
Spec: heater modulates purely on PV export, starts at 500W export.
Code: duty calculated every RS485 packet (~250ms) from net grid flow, battery balance, and the heater's own current consumption.

**Manual modes** (`ManualHeaterMode` enum):
- `MHM_OFF` — heater stays off
- `MHM_FORCE_ON` — heater forced to 100% (was `MHM_OVERRIDE_SOC`; `MHM_SOC_LIMITED` removed)

**Pre-conditions (all must pass before any start logic):**
- Grid present AND `HEATER_ENABLED = true`
- No W solar sensor fault
- No heater hard lockout
- `growattValid = 1` AND combined PV1+PV2 ≥ 200W (end-of-day gate)

**Charge rate target (SOC-based):**
- SOC < 60%: no charge rate contribution; battery discharge still penalises duty
- SOC 60%→80%: target = 3000W linearly interpolated to 1000W (100W per 1% SOC)
- SOC 80%→100%: target = 1000W flat

**Duty calculation (every packet):**

```
netGridW       = pvExportW − gridImportW          // +ve = exporting
battW          = battChargeW − chargeTarget        // if SOC ≥ 60%
               = min(0, battChargeW)               // if SOC < 60% (discharge-only penalty)
heaterCurrentW = heaterTargetPct × 3000 / 100     // heater's own load already in Growatt readings
available      = netGridW + heaterCurrentW + battW − 100
rawPct         = clamp(available × 100 / 3000, 0, 100)
```

Adding `heaterCurrentW` back reconstructs the system balance as if the heater weren't running, so the formula converges to the correct stable duty in one step rather than hunting.

**Start:** `rawPct ≥ 5` (~150W net surplus)  
**Stop:** `rawPct = 0` immediately

### Solar pump direct drive (D46)

Not in spec. H drives the solar pump MOSFET gate directly via D46, connected via a 40m spare wire and 1N4148 diodes (wired-OR) to W's D44 gate drive. The gate sees whichever signal is higher — both controllers are independent and failsafe.

**H pump duty** (`hPumpDutyPct`, added to `HToWPacket`):
- 0 when heater is off or `heaterTargetPct = 0`
- When heater is on: ramps from a protective minimum toward 100% based on heater outlet vs heater target
  - Minimum = `max(2, heaterPct/5)`, slightly boosted above 70°C outlet, capped at 25%
  - Ramp starts at target − 2°C, reaches 100% at target + 2°C
  - At target + 2°C or above: 100% immediately
  - Heater target = tank top + 5°C (SOLAR_TANK_PLUS5) or 89°C (SOLAR_MAX)
- Uses same 3-zone clocking algorithm as W's `updateSolarPump`
- H always drives this when heater is on; heater on always implies summer mode (pvExportOverride), so no winter-specific branch needed

**H pin:** D46 (added after `PIN_RS485_DE_LINK`)

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

| Zone | Duty range | On time | Off time |
|---|---|---|---|
| 1 | 1–20% | 200ms fixed | 19,800ms → 800ms (linear) |
| 2 | 20–50% | 200ms → 800ms (`duty×800/(100−duty)`) | 800ms fixed |
| 3 | 50–100% | 800ms fixed | 800ms → 0ms (`800×(100−duty)/duty`) |
| — | 100% | continuous | — |

Zones 1/2 transition smoothly at 20% (on=200ms, off=800ms). Zones 2/3 transition smoothly at 50% (on=800ms, off=800ms). At high duty (e.g. 90%) the pump runs 800ms on / 89ms off rather than the old 7.2s on / 800ms off.
