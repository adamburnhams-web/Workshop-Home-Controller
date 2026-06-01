# H & W Integrated Control System — Complete Project Reference
**Version 6.0** — Requirements Specification

---

## 1. Project Status

Implementation complete and running. Hardware commissioned at H side; W-side RS485 link and Growatt Modbus pending MAX485 installation. DS18B20 addresses filled in for all 13 sensors. `HEATER_ENABLED = true`.

---

## 2. Physical Infrastructure

### 2.1 Cable Runs Between H and W (40m)

| Cable | Use |
|---|---|
| 6mm T&E | Mains power in both directions (grid feed to W from H, PV export from W back to H) |
| 2.5mm T&E | Live + Neutral carry 15VDC power to both mid-point momentary switches. Earth wire carries buzzer trigger signal from W relay to buzzer at H. |
| 1mm T&E cable 1 | Live = unlock switch output signal to W / holds LED high when W is unlocked; Neutral = outside light switch output signal to W; Earth = spare |
| 1mm T&E cable 2 | Spare live + neutral used for inter-controller RS485 communication (A and B lines). Earth spare. |
| RS485 screened 24AWG cable | Carries SDM230 meter data to Growatt inverter only. SDM230 physically at W. |
| Spare wire on cable 2 | H D46 solar pump direct drive — routed from H to W gate junction via 1N4148 diode |
| 4 × 15mm pipes in vacuum-insulated outer pipe | Hot heating system pipe, cold heating system pipe, DHW pipe (future), cold fresh water pipe |
| 10mm earth cable | Earthing between H and W |

### 2.2 Pipe Bundle Detail

- The vacuum-insulated outer pipe maintains thermal insulation of the 4 inner pipes
- Vacuum pump at W runs to maintain this vacuum whenever the heating system is actively transferring heat

---

## 3. Locations & Layout

| Location | Description |
|---|---|
| H | The house/dwelling — contains heat store tank, log burner, electric heater, and associated valves and controls |
| W | The workshop — contains solar thermal array, PV inverter, UFH, and associated valves and controls. 40m from H |
| Mid-point | Near the back door of the dwelling. Contains: unlock button, outside light switch, and unlocked LED indicator. |

---

## 4. Components & Sensors

### 4.1 At W (Workshop)

**Heating**
- Thermal solar array: 2 × 30 evacuated tubes by Navitron, rated 3.6kW, south-facing wall
- 12VDC brushless pump, under 10W, 2-wire, on cold side of thermal solar. Speed controlled by clocking the 15VDC supply via MOSFET. Gate driven by W D44 and H D46 in a wired-OR circuit (1N4148 diodes).
- 2 × 12VDC motorised valves on cold heating system pipe: one feeds UFH cold side (direction relay SPDT), one feeds solar cold side via flap non-return valve (direction relay SPDT)
- 230VAC central heating pump with thermostatic mixing valve (TMV) for UFH

**PV**
- 2 × 3.6kW PV panel strings on flat roof, 10 degree slope, east/west facing
- Growatt SPH6000TL BL-UP inverter
- Growatt GBLI6532 battery
- SDM230 meter (used by Growatt to calculate grid export, read via RS485)
- PV, battery, and grid data available via Growatt Modbus RS485

**Sensors**
- DS18B20: solar hot side
- DS18B20: solar cold side
- DS18B20: UFH supply (at UFH pump return. Used for logging and pump-off check at 28°C)
- DS18B20: UFH post TMV (after thermostatic mixing valve. Used for 45°C hard lockout only)
- DS18B20: Workshop air temperature
- DS18B20: outside air temperature
- PIR sensor: 15VDC output when activated — voltage divider required
- Door reed switch: dry contact, normally open, closes when door opens (opens when door closed)
- Door handle reed switch: dry contact, closed when handle is pushed down
- Window winch reed switches: fully open, fully closed, manual lock
- Window winch safety limit switch: dry contact, normally open, closes at mechanical over-open limit
- Vacuum sensor: dry contact — normally open (HIGH via pull-up = low vacuum), closes when full vacuum achieved (LOW = full vacuum)
- INA219 current sensor module (I2C) on solar pump positive feed

**Other**
- Electric door lock (15VDC) — 2 relay H-bridge, pulsed 1s to unlock/lock then de-energised
- 230VAC wall axial fan (S&P HCBB/4-450/H, 480W) with BES fan speed controller and external flap actuator (15VDC)
- 230VAC window winch
- 230VAC vacuum pump with 15VDC motorised isolation valve (H-bridge, 2 relays)
- 230VAC external LED lights
- 15VDC alarm sounder (sounds when PIR detects intruder whilst W is locked, auto-stops after 1 minute)
- Manual latching switch at W — suppresses auto-relock whilst ON
- 2 × Fractal Design Dynamic X2 GP-14 (140mm, 3-pin, 12VDC). MOSFET switches 12VDC supply at 25kHz PWM. Powered from 12V DC-DC buck converter from 15V bus. Momentary push button at W sets 8-hour full speed timer.
- Hen house door (H-bridge, 2 relays on 4-ch board) — OPEN and CLOSE relays, 7s pulse
- 2 × manual window buttons at W (D38 open, D39 close) — hold-to-run, reed lockouts apply

### 4.2 At H (House)

**Heating**
- 1000L heat store tank with bottom, middle and top pipe connections
- 3kW electric heater: cold side fed by 2-port valve from hot heating pipe from W; output connects to shared manifold with top of tank and log burner hot pipe
- Log burner with 70L water jacket
- 12VDC motorised valve on log burner cold pipe (H-bridge, 2 relays)
- 12VDC motorised valve on bottom-of-tank pipe (H-bridge, 2 relays)
- 2-port 12VDC motorised valve on hot heating system pipe from W (H-bridge, 2 relays): switches between → electric heater cold side (top-of-tank path) OR → middle of tank
- SSR-40DA zero-crossing solid state relay for heater power control (Bresenham spread firing)
- AC zero-crossing detector module on INT4 (D2): cycle-level heater timing, instant grid outage detection

**Sensors**
- DS18B20: tank bottom
- DS18B20: tank middle
- DS18B20: tank top
- DS18B20: hot heating system pipe to W
- DS18B20: cold heating system pipe to W
- DS18B20: heater hot side output (primary)
- DS18B20: heater hot side output 2 (secondary, redundant — both mounted at heater outlet)
- 12VDC temperature module: outputs 12VDC when log burner water jacket exceeds 28°C — voltage divider required
- 15VDC bus voltage monitor: resistor divider on H Mega ADC A0

**Controls & Indicators**
- 3 × panel-mount buttons at H display: SELECT, ↑ (UP), ↓ (DOWN)
- 3.5" ILI9488 TFT display 480×320 SPI — see Section 9 for full display specification
- MicroSD card module (SPI) for data logging (log.csv, ISO datetime timestamps)
- DS3231 RTC module (I2C) — essential for 5am heating trigger and time-gated security alerts
- 12VDC backup PSU relay — switched by H Mega when bus voltage drops below 12V

### 4.3 At Mid-Point

- Momentary unlock button (15VDC from 2.5mm T&E live, output on 1mm cable 1 live)
- Momentary outside light switch (15VDC from 2.5mm T&E live, output on 1mm cable 1 neutral)
- LED indicator: controlled entirely by W Mega on D49 (via relay on 4-ch board). Priority order (highest first):
  1. Fault active — 1s on / 1s off continuous flash
  2. Workshop unlocked, no fault — steady HIGH
  3. Workshop locked, no fault — off

---

## 5. Pipe & Valve Topology

### 5.1 Cold Heating System Pipe (H to W)

Feeds from H: log burner cold valve OR bottom-of-tank valve (both connect to same cold pipe). At W: splits to → UFH cold side valve OR solar cold side valve (via non-return flap valve).

### 5.2 Hot Heating System Pipe (W to H)

From W: UFH hot output and solar hot output. At H: enters 2-port valve → switches to heater cold side (top-of-tank path) OR middle of tank. Heater output joins shared manifold → top of tank + log burner hot pipe.

---

## 6. Security & Access Logic

### 6.1 Door Lock & Unlock

- W door locked by default (15VDC electric lock, 2-relay H-bridge, pulsed 1s to unlock then de-energised)
- Unlock: pressing momentary button at mid-point sends signal to W Mega → door lock pulsed 1s → 1mm T&E live held HIGH → LED at mid-point lights steady

### 6.2 Auto-Relock Logic

Auto-relock is prevented while ANY of the following conditions are true:
- Door is open (reed switch closed)
- PIR is detecting presence (active output)
- Manual suppression switch at W is ON
- Window is not fully closed AND locked (fully closed reed switch not active OR manual lock reed switch not active)

Relock timer: starts only when ALL four conditions are simultaneously clear. After 5 continuous minutes of all four clear → door relocks.

### 6.3 Security Alerts

- **Intruder alert:** PIR detects movement whilst W is locked → alarm at W sounds for 1 minute then auto-stops; buzzer at H sounds while PIR is active; logged
- **Door handle alert:** door handle reed switch closes whilst W is locked → between 23:00–06:00 buzzer sounds minimum 10s (continues while held if longer); between 06:00–23:00 buzzer sounds only while handle held; logged with timestamp

### 6.4 Fire Alarm

New logic — not in original spec:
- **Trigger A (absolute):** workshop air > 25°C AND outside air has not been ≥ 25°C in the last 24 hours
- **Trigger B (rate-of-rise):** workshop air rising ≥ 0.5°C/min AND door closed AND window fully closed
- **Phase 1 (60s):** external lights flash 250ms, buzzer at H active, no local sounder
- **Phase 2:** local sounder on 60s / off 60s, up to 5 cycles or until rate-of-rise stops
- Clears on alert reset from H display (page 4)
- Fault flag: `FAULT_W_FIRE_ALARM`

---

## 7. Lighting & Ventilation Logic

### 7.1 External Lights at W

Controlled by: momentary switch at mid-point OR momentary switch at W. 230VAC LED external lights switched by relay.

### 7.2 Workshop Fan (S&P HCBB/4-450/H)

- 230VAC axial fan with BES manual speed controller and external flap actuator (15VDC)
- Fan on/off controlled by relay on W 230VAC relay board
- **Automatic night cooling** (230VAC fan only — PC fans unaffected): fan ON when outside air is more than 5°C below Workshop air temp; fan OFF when gap closes to 2°C or less
- Night Cooling can be disabled via display toggle at H (page 4). State shown in status bar.
- Flap actuator opens before fan relay energises; closes after fan relay de-energises

### 7.3 Window Winch

Manual trigger only — 3 relays on W 230VAC relay board (direction open, direction close, power).

Valid relay states:

| State | Direction open (D31) | Direction close (D33) | Power (D29) | Notes |
|---|---|---|---|---|
| STOP | OFF | OFF | OFF | Only safe stop state. Power-up default. |
| OPEN | ON | OFF | ON | Relay 1 and 3 energised simultaneously |
| CLOSE | OFF | ON | ON | Relay 2 and 3 energised simultaneously |
| FORBIDDEN | ON | ON | any | Hard interlock — must never occur |

Direction change: always STOP → 1 second pause → new direction.

Immediate stop triggers:
- Fully open reed switch activates → STOP + lock out OPEN direction
- Fully closed reed switch activates → STOP + lock out CLOSE direction
- Manual lock reed switch activates → STOP + lock out BOTH directions
- Safety limit switch activates → STOP + lock out OPEN direction + fault log

Manual buttons at W (D38/D39): hold-to-run. Reed lockouts still apply. Release = immediate stop.

### 7.4 PC Cooling Fans at W

2 × 140mm 3-pin 12VDC fans. MOSFET switches 12VDC supply at 25kHz PWM.

Fan control modes — priority order:

| Priority | Mode | Speed | Trigger / Duration |
|---|---|---|---|
| 1 | Full speed timer | 100% | W button press = 8hrs. Adjustable at H display in 1-hour increments (0–24hrs). Counts down regardless of lock status. |
| 2 | Base speed timer | Base % | Set at H display in 1-day increments (0–30 days). Counts down continuously. When expires → fans off until workshop unlocked. |
| 3 | Workshop unlocked | Base % | Fans run at base speed whenever workshop is unlocked. Off when locked. |
| 4 | Off | 0% | Workshop locked, no timers active. |

Base speed: set at H display in 10% increments (0–100%). Firmware enforces minimum speed if fan stall detected.

Tachometer fault: if either fan reads zero RPM when commanded above minimum speed for >5s → flash LED, log fault.

---

## 8. Heating Control Logic

### 8.1 Winter Mode

Active when Mode is set to Winter via display at H. UFH heating operates normally.

**Thermal Solar — Winter**
- Solar sequence triggers when solar hot side ≥ 18°C
- Open UFH cold valve, open solar cold valve, start solar pump clocking
- Solar pump duty calculated by `calcPumpDuty(hot, targetC)`. Winter target: 20°C (UFH circuit temperature)
- Fault condition: pump running above clocking speed for >10s AND solar cold side >10°C above tank bottom → thermal solar overheat alert
- When solar hot side ≤ solar cold side + 2°C: close solar cold valve, stop pump
- Frost protection: if either solar side drops below 2°C → open UFH valve, run solar pump at full speed until solar output reaches 8°C → stop pump but leave UFH valve open

**Morning Heating Cycle (5am daily, RTC-triggered)**
- Trigger: RTC reaches 05:00. No catch-up on power-cut restart.
- Normal mode: if W air temp < 13.5°C → open UFH valve + start W central heating pump. Stops when W air temp reaches 13.5°C. UFH locked out until next 5am once target reached.
- Boost at 5am mode: triggers at boost target temp (13–20°C, default 15.5°C), stops at boost target +0.5°C. Active until 10am the following day.
- 8hr boost mode: starts immediately, runs at boost target temp for 8 hours.
- UFH overheat: if UFH post TMV temp exceeds 45°C while UFH pump running → stop pump, close UFH valve, hard lockout until system restart
- Once W air temp reaches target: open log burner cold valve + ensure top-of-tank path open (2-port to heater side), close bottom-of-tank valve. UFH pump continues until UFH supply drops below 28°C → everything stops.

**Heat Source Selection at H (when morning heating is ON)**
- If `hotTankProtection` is active: selection function skipped entirely
- If log burner temp module active (jacket > 28°C, deactivates below 25°C): open log burner cold valve; set 2-port to heater side
- If log burner NOT active: close log burner cold valve, open bottom-of-tank valve
- 2-port valve position: switch to mid-tank at temp > 32°C, switch back to heater side at temp < 28°C (4°C hysteresis on middle-of-tank temperature)
- Log burner and solar can run simultaneously in winter — valid expected state

### 8.2 Summer Mode

Active when Mode is set to Summer via display at H. UFH is completely disabled in summer mode.

**Summer Startup Sequence**

Triggered when: solar hot side OR solar cold side ≥ 50°C, OR electric heater is powered.

Phase 0 → Phase 3 directly: collector at ≥ 50°C already exceeds tank bottom, so intermediate phases are bypassed. This also handles H rebooting mid-session cleanly.

On trigger: open log_burner_cold valve (false), open bottom-of-tank valve, 2-port valve to heater side (top-of-tank path). Solar pump starts.

If `morningHeatActive` is true: startup sequence aborts and resets.

**2-port valve — dynamic tracking (phase 3 onwards)**
- Hot pipe > tank top + 1°C → heater/top-of-tank side
- Hot pipe < tank top − 1°C → mid-tank side
- Hold between ±1°C

**Additional zero condition:** while `twoPortHeaterSide = true` AND summer startup phase ≥ 3 AND both solar hot and solar cold are below target AND below 65°C → set solar pump duty to 0. Prevents W fighting H for pump when H has the two-port on heater side and both solar sides are cold.

**Thermal Solar — Summer**

Solar pump duty calculated by `calcPumpDuty(hot, solarTarget)`:

| Solar hot vs target | Duty |
|---|---|
| < target − 8°C | 4% |
| target − 8°C to target | 4% → 50% linear |
| target to target + 2°C | 50% → 100% linear |
| ≥ target + 2°C | 100% |

Solar target = `min(tankTop + 8°C, 87°C)` (SOLAR_TANK_PLUS8 mode) or 87°C flat (SOLAR_MAX mode).

Solar target mode toggled from display page 4.

**Solar pump overrides** (applied on top of base duty each loop):
- **Rule 1 — 60s kick:** when either solar pipe > 60°C and base duty < 4%, fire a 1s full-speed kick every 60s
- **Rule 2 — fast-rise burst:** when solar hot crosses up through target − 2°C AND reaches that level within 15s, run pump at 100% for 5s
- **Rule 3 — fast-drop pause:** when solar hot was > target + 2.5°C then drops to ≤ target + 2°C, set pump to 0% for 10s. Takes priority over Rule 2.

**H back-off:** W suppresses its own pump output when H has a valid packet, H's heater is running, and H's pump duty exceeds W's calculated duty, and solar hot is below target. W resumes when solar hot reaches target or W's duty is ≥ H's. Wired-OR gate means the higher of the two signals drives the pump regardless.

**Solar cold valve:** always open when heater is running (`lastH.heaterPowerPct > 0`).

**Hot side overheat:**
- Solar hot > 91°C AND pump at 100% AND tank bottom < 70°C → set `FAULT_W_SOLAR_OVERHEAT_HOT`; clears when solar hot ≤ 91°C
- UFH dump: when `91°C < solar hot < 93°C` AND solar cold < 93°C → open `ufhColdValve`; close otherwise. Independent of fault flag.

### 8.3 Electric Heater Control

**Pre-conditions** (all must pass, in order):
1. Grid present AND `HEATER_ENABLED = true`
2. Solar cold valve open (`VSTATE_SOLAR_COLD_OPEN`)
3. No W solar sensor fault
4. No heater hard lockout (`heaterHardLockout = false`)
5. `hotTankProtection = false` (tank_bot ≤ 83°C — see §8.4)
6. `morningHeatActive = false`
7. Valves not currently mid-close stroke (prevents starting while bot_tank or two_port is moving toward closed)

**Manual heater modes** (`ManualHeaterMode` enum):

| Mode | Value | Behaviour |
|---|---|---|
| `MHM_OFF` | 0 | Heater off |
| `MHM_SOC_LIM` | 1 | Run at 100% drawing from battery+grid. Enter at SOC ≥ 55%; leave below 50% (5% hysteresis). Budget: 4 kW combined bat+grid. heater gets all remaining headroom. |
| `MHM_FORCE_ON` | 2 | Full 3kW continuous. Also requests log_cold close, bot_tank and two_port open. No SOC check. |

Manual heater state NOT stored in EEPROM — resets to Off on power cycle.

**Auto mode duty calculation** (per RS485 packet, ~250ms):

SOC-based reservation reduces the available power to preserve battery charge:

```
post-noon (hour ≥ 12):
  soc > 97% → reservationW = 200W
  soc > 90% → reservationW = 500W
  soc > 80% → reservationW = 1100W
  otherwise → reservationW = 0

pre-noon (hour < 12):
  soc > 97% → reservationW = 200W
  soc > 90% → reservationW = 500W
  soc > 50% → reservationW = 1100W    // lower threshold before noon
  otherwise → reservationW = 0

heaterCurrentW = heaterTargetPct × 3000 / 100

If reservation > 0:
  available = pvExportW − gridImportW + battChargeW + heaterCurrentW − reservationW
Else:
  available = pvExportW − gridImportW + min(0, battChargeW) + heaterCurrentW − 100

rawPct = clamp(available × 100 / 3000, 0, 100)
```

Adding `heaterCurrentW` back corrects for the heater's own load already appearing in Growatt readings, so the formula solves for the correct duty in one step.

**Start hysteresis:** rawPct ≥ 17% (~500W surplus) sustained for 5 seconds.

**Stop hysteresis:** 5 consecutive zero-rawPct packets before heater turns off (prevents brief cloud cover from cycling the heater).

**Smoothing:** 8-packet running average applied before assigning to `heaterTargetPct`.

**Hot pipe cap** (applied after smoothing):
- hot_pipe ≥ 90°C → heaterTargetPct = 0
- hot_pipe 60–90°C → cap at `(90 − hot_pipe) × 3.33%/°C` (100% at 60°C, 0% at 90°C)

**`botTankValve.request(true)`** called whenever heater is running.

**RS485 stale check:** if RS485 comms stale > 60s while heater is running → `heaterTargetPct = 0`, `heaterRunning = false`.

### 8.4 Hot Tank Protection

Prevents the heater from running into an already-hot tank.

`checkHotTankProtection()` monitors `tank_bot`:
- **Enter** (tank_bot > 83°C): `hotTankProtection = true`, log_burner_cold opens, bot_tank valve closes, two_port opens. Heat source selection logic skipped while active.
- **Leave** (tank_bot < 82°C, 1°C hysteresis): `hotTankProtection = false`.

### 8.5 H-Side Solar Pump Direct Drive

H drives the solar pump MOSFET gate directly via D46, connected through a 40m spare wire and 1N4148 diode wired-OR with W's D44 output. The gate sees whichever signal is higher — both controllers are independent and failsafe.

`calcHPumpDuty()` returns a float (0.0–100.0):
- 0 when heater is off, both heater sensors faulted, or hot pipe sensor faulted
- 100 during `heaterHardLockout` (maximum flow when fault triggers safety shutdown)

**Predictive base formula** (`calcPred`):
```
excess   = heaterPct − 20
pt       = max(0, excess²) × 0.0025        // plateau term (power delivery above 20%)
tr       = 0.09 + max(0, excess) × 0.022   // thermal rise slope
raw      = 4 + pt + tr × (hotPipeC − 30)
minDuty  = 4 + heaterPct × 0.05
pred     = max(raw, minDuty)               // 4% minimum
```

**SOLAR_TANK_PLUS8 mode** (when tank top ≤ 75°C and sensor valid):
- target = `min(tankTop + 8°C, 87°C)`
- heaterOut < target − 7°C: `pred × 0.9`
- target − 7°C to target: ramp 0.9 → 1.0 × pred
- target to 90°C: `pred + (hOut − target) × 0.2%/°C`
- 90°C to 91°C: ramp from `dutyAt90` to 100%
- ≥ 91°C: 100%

**MAX mode** (or tank top sensor fault):
- `upper = pred × (1.3 − clamp((pred−5)/25, 0, 1) × 0.2)`
- heaterOut < 78°C: `pred × 0.9`
- 78°C to 85°C: ramp pred × 0.9 → pred
- 85°C to 90°C: ramp pred → upper
- 90°C to 91°C: ramp upper → 100%; ≥ 91°C: 100%

`updateHPump()` runs in a 50ms Timer1 ISR, independent of loop() timing.

**Clocking zones:**

| Zone | Duty | On time | Off time |
|---|---|---|---|
| A | 0–20% | 400ms | `400×(100−d)/d` ms |
| B | 20–50% | `400+(d−20)×400/30` ms | `on×(100−d)/d` ms |
| C | 50–100% | 800ms | `800×(100−d)/d` ms |
| — | 100% | continuous | — |

### 8.6 Heater Fault Handling

**Dual heater output sensors:** `H_SENSOR_HEATER_OUT` (primary) and `H_SENSOR_HEATER_OUT_2` (secondary). `getHeaterOutC()` arbitrates: both valid → max of the two; one faulted → remaining sensor; both faulted → NAN.

Heater output sensors excluded from the `sensorFaultMaskH` array — handled separately by `checkHeaterFaults()` with a 5s grace period.

**Per-sensor fault logic:**
- Both sensors faulted → `heaterPowerCapPct = 0` immediately, set both fault flags
- Single sensor faulted → raise that sensor's fault flag after 5s grace; continue on remaining sensor
- Hot pipe sensor faulted → `heaterPowerCapPct = 0` immediately

**Overheat power cap** (ISR-level, via `heaterPowerCapPct`):
- 91°C to 93°C: `heaterPowerCapPct` reduced linearly 100→0; valves requested open; `FAULT_H_HEATER_OVERHEAT_WARN` set
- ≥ 93°C: `heaterPowerCapPct = 0`, `heaterHardLockout = true`, SSR pin cleared, `FAULT_H_HEATER_OVERHEAT_SHUT` set

**Hard lockout auto-clear:** clears automatically when effective heater temp < 88°C AND at least one sensor is live. Also cleared by Page 5 Ack action (allows manual recovery without reboot).

**Element fail detection:** removed. The old 30s/no-temp-rise check has been removed; dual sensor arbitration makes it redundant.

---

## 9. Display Specification

### 9.1 Hardware

- 3.5" ILI9488 TFT 480×320 SPI display at H
- 3 panel-mount buttons: SELECT, ↑ (UP), ↓ (DOWN)
- Driver: TFT_eSPI with ILI9488_DRIVER configured in platformio.ini build_flags

### 9.2 Button Behaviour

| Mode | UP ↑ | DOWN ↓ | SELECT |
|---|---|---|---|
| Page scroll mode | Previous page | Next page | Enter item mode |
| Item mode — on an item | Previous item | Next item | Enter option mode |
| Item mode — past top boundary | No action | Move to first item | Exit to page scroll |
| Item mode — past bottom boundary | Move to last item | No action | Exit to page scroll |
| Option mode | Previous option / decrement | Next option / increment | Confirm → return to item mode |

**Inactivity:** 30s no press in item/option mode → return to page scroll. 1hr no press with no fault → backlight off. Any press wakes display. If any fault active: backlight stays on.

**Boost shortcut:** Double-press SELECT from page scroll mode → toggles Boost at 5am on/off from any page.

**Alert reset:** item on page 4. Navigate to it and press SELECT to acknowledge and clear the flashing LED. If fault still present: LED re-triggers immediately.

### 9.3 Persistent Elements (all pages)

**Status bar (top):** Mode (Winter/Summer), Boost status + countdown, Night Cooling on/off, Solar Target mode (`TK+8` or `MAX`), Clock from RTC.

**Fault bar (bottom):** all active faults shown with elapsed time. Scrolls if multiple active.

**Banners (below status bar):**
- Manual override active: red `MANUAL OVERRIDE ACTIVE — automatic valve control suspended`
- SOC-Lim heater: red `SOC-LIM: XX% — HEATING` or `SOC-LIM: XX% — DONE`
- Force-on heater: red `HEATER MANUAL ON — FULL 3kW`
- Both manual heater and manual valve override active: both banners stack

### 9.4 Page 1 — Heating System

**Left column (W temperatures, 6 rows):** solar hot, solar cold, UFH supply, UFH post TMV, Workshop air, outside air

**Right column (H temperatures, 7 rows):** tank bot, tank mid, tank top, hot pipe, cold pipe, Heater (htr_out_2), Htr Out (htr_out)

**Power row:** `Heater: XXXX W` | `Sol H: XX%` (H pump duty) | `Sol W: XX%` (W pump duty)

**Valve state badges:** all 6 valves shown open/closed

**Window winch state:** open / closed / opening / closing / manual lock / over-open fault

### 9.5 Page 2 — Power & Inverter

- PV: string 1 kW, string 2 kW, total PV kW, today's generation kWh
- Grid: import/export kW with direction indicator, load power kW
- Battery: SOC %, charge/discharge power kW, battery voltage V
- 15V bus voltage V

### 9.6 Page 3 — Fault History

- All faults logged since last system restart, most recent first
- Each entry: fault name, time first occurred, time resolved, duration
- Active faults in red. Resolved faults in grey.
- Scrollable list — UP/DOWN buttons scroll entries
- Fault data held in RAM. Clears on power cycle. Maximum 80 entries (~40 bytes × 80 = 3.2KB).

### 9.7 Page 4 — System Controls

| Order | Item | UP/DOWN action | Values |
|---|---|---|---|
| 1 | Boost | Cycles: Off → Boost at 5am → 8hr boost → Off | NOT stored in EEPROM |
| 2 | Boost target | +/−0.5°C per press | 13°C – 20°C. Default: 15.5°C. NOT stored in EEPROM |
| 3 | Mode | Toggle | Winter ↔ Summer |
| 4 | Manual heater | Cycles: Off → SOC-Lim → Manual On → Off | NOT stored in EEPROM |
| 5 | Night Cooling | Toggle | On ↔ Off |
| 6 | Solar Target | Toggle | Tank+8°C ↔ Max |
| 7 | Fan base speed | +/−10% per press | 0–100% |
| 8 | Fan full speed timer | +/−1 hour per press | 0–24hrs |
| 9 | Fan base speed timer | +/−1 day per press | 0–30 days |
| 10 | Display brightness | +/−10% per press | 10–100% |
| 11 | SD safe remove | SELECT to eject | Flushes writes, closes filesystem |
| 12 | Alert reset / Ack | SELECT to acknowledge | Visible when fault alert active. Also clears heater hard lockout to allow restart. |

EEPROM storage: Mode, Night Cooling, Solar Target, Display brightness (30s timeout). All others NOT stored.

### 9.8 Page 5 — Valve States & Manual Override

- All valve states displayed: UFH cold, solar cold, vacuum isolation, log burner cold, bottom-of-tank, 2-port
- Window winch status with reed switch states
- Manual override: explicit entry via SELECT. UP/DOWN scrolls between valves, SELECT toggles state, actuates immediately
- Workshop valve overrides sent via RS485 to W for actuation
- Override NOT stored in EEPROM — always reverts to automatic on power cycle

### 9.9 Page Summary

| Page | Content |
|---|---|
| 1 | Heating system: all temps, heater W, Sol H %, Sol W %, valve overview, winch state |
| 2 | Power & inverter: all Growatt data, PV strings, battery, grid, bus voltage |
| 3 | Fault history: all faults since restart, most recent first |
| 4 | System controls: boost, mode, manual heater, night cooling, solar target, fans, brightness, SD eject, ack |
| 5 | Valve states & manual override |

---

## 10. Vacuum System

- Activates when heating or solar becomes active (either direction of heat transfer)
- Sequence: open isolation valve → start vacuum pump → run until vacuum sensor confirms full vacuum
- Once full vacuum achieved: run pump for additional 5 minutes, then close isolation valve 10 seconds before pump stops
- Once full vacuum achieved in a session: do not restart pump even if vacuum subsequently lost. Resets at next heating activation.
- Maximum runtime: 30 minutes. If not achieved in 30 min → stop pump, set `FAULT_W_VAC_PUMP_OVERTIME`.

---

## 11. Safety Monitoring & Alerts

**Global alert rules:**
- Buzzer at H sounds while fault condition is active — stops automatically when condition clears
- Mid-point LED flashes 1s on/1s off from fault onset — continues until Alert Reset selected on page 4
- If fault still present when Alert Reset selected: LED re-triggers immediately
- All faults shown on H display fault bar with elapsed time
- All faults logged with timestamp to SD card

| Fault | Trigger | Actions | Clears |
|---|---|---|---|
| Solar Overheat — cold side high | Pump above clocking speed >10s AND cold side >10°C above tank bottom | Buzzer + LED + Display + Log | Buzzer stops on clear; LED until Ack |
| Solar Overheat — hot side high | Tank bottom <70°C AND solar hot >91°C AND pump at 100% | LED + Display + Log | LED until Ack; clears when solar hot ≤ 91°C |
| Solar pump fault | Current below calibrated minimum for >5s while pump commanded ON | Buzzer + LED + Display + Log. UFH dump if both sides < 93°C | LED until Ack |
| Solar pump overcurrent | Current above maximum for >3s | Stop pump. LED + Display + Log. UFH dump if both sides < 93°C | LED until Ack; pump restarts when current normalises |
| Heater Overheat — power reduction | Heater output rising above 91°C | ISR power cap engages; fault flag set. Valve open requested | Auto-clears below 91°C. Flag cleared on Ack. |
| Heater Overheat — shutdown | Heater output ≥ 93°C | Heater hard lockout. SSR cleared. `FAULT_H_HEATER_OVERHEAT_SHUT`. Buzzer + LED | Auto-clears when temp < 88°C. Also clears on page 4 Ack. |
| UFH Overheat | UFH post TMV temp >45°C while UFH pump running | LED + Display + Log + UFH stop + UFH valve close. Hard lockout until restart. Frost protection bypasses. | LED until Ack. UFH locked until restart. |
| Frost — cold side not recovering | Frost protection active >1 min AND cold side not yet 4°C | LED + Display + Log | LED until Ack |
| Vac Pump Overtime | Vacuum pump running >30 min without full vacuum | LED + Display + Log + Pump stopped | LED until Ack |
| 15V Bus Low | Bus voltage below 14V for >10s | LED + Display + Log | Clears above 14V; LED until Ack |
| 12VDC PSU Activated | Bus voltage drops below 12V | Display + Log + PSU relay ON | PSU relay off above 12.5V |
| Grid Outage | Zero-crossing absent >50ms | Fault bar + Log + Heater off. No buzzer, no LED. | Auto-clears on grid restore |
| Growatt Comms Fault | 5+ consecutive failed Modbus polls | LED + Display + Log + Heater off | Clears when comms restore |
| Intruder Alert | PIR active whilst W locked | Buzzer at H (while PIR active) + W alarm 1 min auto-stop + Log | Buzzer stops when PIR clears |
| Door Handle Alert (night) | Handle active whilst locked, 23:00–06:00 | Buzzer minimum 10s | Stops after ≥10s when handle released |
| Door Handle Alert (day) | Handle active whilst locked, 06:00–23:00 | Buzzer while held | Stops on release |
| Window Winch Over Open | Safety limit switch while opening | LED + Display + Log + Open direction locked out | LED until Ack. Close direction still available. |
| RS485 Comms Fault | 5+ consecutive missed H↔W packets | LED + Display + Log | Clears when comms resume |
| Fan 1 / Fan 2 Fault | Tach zero while commanded >min speed for >5s | LED + Display + Log | LED until Ack |
| Fire Alarm | See §6.4 | Lights flash + buzzer at H + local sounder phase 2 | Alert reset on page 4 |
| Heater sensor fault | One sensor: 5s grace then flag; both sensors: immediate cap | LED + Display. Heater continues on remaining sensor if one remains | Clears on sensor recovery |

### 11.1 Sensor Fault Behaviour

DS18B20 fault detection: discard readings of exactly 85.00°C, −127°C, −128°C, or outside physically plausible range. Sensor considered failed after 3 consecutive invalid readings. Clears after 3 consecutive valid readings. 11-bit mode adds a single retry on transient CRC errors.

| Sensor | Action on fault |
|---|---|
| Solar hot / solar cold | Close log_cold and bot_tank valves, open two_port. Open UFH cold + solar cold valves. Start UFH pump. Force heater off. LED + Log. |
| Heater output (primary) | 5s grace. Raise `FAULT_H_SENSOR_HEATER_OUT`. Continue on sensor 2 if valid. If both faulted: immediate heaterPowerCapPct=0. |
| Heater output 2 (secondary) | 5s grace. Raise `FAULT_H_SENSOR_HEATER_OUT_2`. Continue on sensor 1 if valid. |
| Hot pipe | LED + Log. Immediate heaterPowerCapPct=0 while faulted. |
| UFH post TMV | LED + Log only. UFH continues but 45°C lockout disabled. |
| UFH supply | LED + Log only. 28°C pump-off check disabled. |
| Tank top / mid / bot | LED + Log only. No automatic action. |
| Hot pipe / cold pipe | LED + Log only. No automatic action. |
| Workshop air | LED + Log only. No automatic action. |
| Outside air | LED + Log only. No automatic action. |

---

## 12. Power Supply & Voltage Management

| Threshold | Action |
|---|---|
| Below 14V for >10s | LED + Log `15V Bus Low`. System keeps running. |
| Below 12V | Activate 12VDC backup PSU relay + Log |
| Recovers above 12.5V | Deactivate PSU relay + Log |
| Recovers above 14V | Clear LED. Log. |

Grid outage detected via zero-crossing detector at H (≤50ms). Heater forced off on grid outage regardless of manual mode.

---

## 13. Inter-Controller Communications

- RS485 link over spare 1mm T&E cable between H and W (40m)
- 9600 baud. Poll interval: 250ms (4 times per second). Custom binary framing in `rs485_packet.h`.
- Frame: `[0xAA][0x55][DIR][SEQ][LEN_LO][LEN_HI][PAYLOAD][CRC_LO][CRC_HI]`
- CRC-16/Modbus over DIR+SEQ+LEN+PAYLOAD
- If 5+ consecutive packets missed (~1.25s): RS485 Comms Fault triggered
- On failure: W continues on last known H values. H continues on last known W values.
- Growatt stale: `growattValid = 0` after 60s without a valid Modbus response (reduced from 120s)
- H RS485 rx timeout: 150ms. W RS485 rx timeout: 200ms.

### 13.1 W → H Packet (every 250ms)

- 6 W temperatures (solar hot, solar cold, UFH supply, UFH post TMV, workshop air, outside air)
- Growatt data: PV1/PV2/output/load/export/import power, battery voltage/SOC/charge, daily gen, growattValid flag
- Solar pump duty %, solarPumpActive flag
- Valve state bitmask (VSTATE_* flags)
- Security: workshopLocked, doorOpen, pirActive, manualRelockOn
- Window winch state, reed flags
- Fan: duty %, full timer secs, base timer secs
- UFH: ufhPumpRunning, ufhTargetReached, solarDumpActive
- W fault flags (32-bit)
- Time sync request flag

### 13.2 H → W Packet (every 250ms)

- 5 H temperatures (tank bot, mid, top, hot pipe, cold pipe) + effective heater output temp (max of both sensors)
- Heater state: heaterPowerPct (min 1 when running), heaterRestricted
- System config: systemMode, boostMode, ufhStopTemp, morningHeatActive, summerStartupPhase, nightCoolingEnabled, solarTargetMode, manualHeaterMode
- Fan settings: fanBaseSpeedPct, fanFullTimerDeltaHr, fanBaseTimerDeltaDay
- Manual override: overrideActive, overrideValves bitmask
- Alert reset sequence number
- Time sync payload
- H fault flags (32-bit)
- 2-port valve state (twoPortHeaterSide)
- Cal fields (DEBUG_SERIAL only): calPumpActive, calSolarTargetC (calSolarTargetC=0 signals pump-stop)
- hPumpDutyPct (0–100)

---

## 14. Data Logging

SD card at H only. All W data received via RS485 and logged centrally.

**Continuous log** (`log.csv`, 250ms interval): 22 columns — datetime (ISO `YYYY-MM-DD HH:MM:SS` from RTC), solar_hot, solar_cold, ufh_sup, ufh_tmv, w_air, out_air, tank_bot, tank_mid, tank_top, hot_pipe, cold_pipe, htr_out, htr_out_2, pump_pct, htr_pct, export_w, import_w, bus_v, fan1rpm, fan2rpm, fan_pct.

Column header is validated on startup: if the comma count in the existing file's first line doesn't match expected, the file is replaced with a fresh header. This prevents stale column layouts from corrupting the log after firmware updates.

**SD card state:** if file open fails, `sdAvailable` is set false to prevent repeated open attempts after card removal.

**SD safe remove:** page 4 option flushes writes and closes filesystem. Auto-reinitialises on card reinsert.

---

## 15. Data Analysis — SD Card & SQLite

### 15.1 Workflow

1. Remove SD card from H controller (use SD safe remove on page 4 first)
2. Insert SD card into PC
3. Run Python import script — reads CSV and imports into SQLite database
4. Run analysis scripts to produce graphs and reports
5. Reinsert SD card into H controller

### 15.2 Planned Analysis Scripts

- **kWh to heater per day** — bar graph over 1 year
- **Daily temperature graph** — all temperatures + pump/heater % for a given date. Both heater output sensors plotted.
- **Fault history report** — query all fault events, durations, frequency

---

## 16. Firmware Notes

Platform: Arduino C++ on Elegoo Mega 2560 (ATmega2560), PlatformIO build system.

- **Watchdog timer:** 8s hardware WDT on both Megas. All loop operations must complete within WDT period.
- **Non-blocking:** no `delay()` in main loop. All timing via `millis()` subtraction. `millis()` overflow safe after ~49 days: always subtract.
- **EEPROM:** write after 30s timeout following last change. Never store manual override or boost state.
- **`#ifdef DEBUG_SERIAL`:** wraps all debug code. Remove `#define DEBUG_SERIAL` at top of each main.cpp for production builds.
- **`F()` macros:** all string literals in debug/display code use `F()` to keep them in Flash, not RAM.
- **No `String` class:** char arrays and `snprintf` only.
- **Simulation overrides:** `simXxxActive` + `simXxxVal` pairs applied after sensor reads (DEBUG_SERIAL only).
- **`HEATER_ENABLED`:** compile-time constant (now `true`). Heater modulation and ISR gate both check this.
- **Heater ISR:** ZC ISR on INT4 (D2). Uses `PORTA |= (1 << PA5)` / `PORTA &= ~(1 << PA5)` to toggle D27 (PA5). Gate condition: `HEATER_ENABLED || simHeaterActive`, `heaterRunning`, `VSTATE_SOLAR_COLD_OPEN`, `!heaterHardLockout`, Bresenham fire.
- **Timer1 ISR:** 50ms COMPA interrupt drives `updateHPump()`. Prevents pump stalling during TFT SPI operations.
- **Power-up safe state:** all valves driven to known-closed state before control logic starts. UFH/solar cold valves get 30s wait for auto-cutout. Door lock always pulsed to locked state.
- **RS485 packet format:** custom binary, CRC-16/Modbus. `PktReceiver` state machine handles byte-by-byte receive with automatic resync on error. ArduinoRS485 library not used.
- **DS18B20 resolution:** 11-bit (375ms conversion, 0.125°C). Conversion restarted immediately after read. Single retry on transient CRC errors (500µs delay before retry).
- **Log burner temp module hysteresis:** activate at 28°C, deactivate below 25°C.
- **INA219 solar pump current:** sampled during ON period only, after 50ms spin-up.
- **Fan speed control:** W controls fan autonomously using its own lock state combined with settings received from H.
- **Window winch:** strict 3-state machine (STOP/OPEN/CLOSE). All transitions through STOP with 1s pause.
- **Serial monitor:** `monitor_filters = log2file` active on controller_h — serial output saved automatically during `pio device monitor` sessions.

### 16.1 Valve & Lock H-Bridge Wiring

| State | Terminal A | Terminal B | Result |
|---|---|---|---|
| Both relays OFF | GND (via NC) | GND (via NC) | Device holds position. No power drawn. |
| Relay 1 ON, Relay 2 OFF | +15VDC (via NO) | GND (via NC) | Current flows A→B. Valve opens / lock unlocks. |
| Relay 1 OFF, Relay 2 ON | GND (via NC) | +15VDC (via NO) | Current flows B→A. Valve closes / lock locks. |

Dead-time: 200ms between de-energising one relay and energising the other. `HBridgeValve::request()` is idempotent: returns immediately if valve is already at the requested position or already moving there.

### 16.2 Valve & Lock Pulse Timings

| Device | Location | Wiring | Pulse |
|---|---|---|---|
| Log burner cold valve | H | H-bridge (2 relays) | 7s open or close. Both OFF after pulse. |
| Bottom-of-tank valve | H | H-bridge (2 relays) | 7s open or close. Both OFF after pulse. |
| 2-port valve | H | H-bridge (2 relays) | 7s open or close. Both OFF after pulse. |
| Vacuum isolation valve | W | H-bridge (2 relays) | 7s open or close. Both OFF after pulse. |
| Hen house door | W | H-bridge (2 relays, 4-ch board) | 7s open or close. Both OFF after pulse. |
| UFH cold valve | W | Single SPDT relay | NO = open (Wire A). NC = close (Wire B). GND permanent. Auto-cutout — no timed pulse. |
| Solar cold valve | W | Single SPDT relay | NO = open (Wire A). NC = close (Wire B). GND permanent. Auto-cutout — no timed pulse. |
| Door lock | W | H-bridge (2 relays) | 1s pulse to lock, 1s pulse to unlock. Both OFF after pulse. |

### 16.3 Power-Up Safe State Sequence

1. Door lock → 1s lock pulse. If door reed open: log anomaly, still send pulse. Record state as locked.
2. All tank valves at H → 7s close pulse. Wait for completion.
3. Vacuum isolation valve at W → 7s close pulse. Wait for completion.
4. UFH cold + solar cold valves at W → set direction relay to close, wait 30s (`VALVE_POWERUP_WAIT_MS`), then hold relay in close position.
5. Window winch → no power-up action. State from reed switches.
6. Begin normal heating logic only after all devices in known state.

---

## 17. Growatt SPH6000TL BL-UP Modbus Register Reference

Default baud rate: 9600. Minimum command period: 850ms. Max read: 125 words.

### 17.1 Input Registers (Function 04) — 0–124 range

| Register | Description | Scale | Notes |
|---|---|---|---|
| 0 | Inverter status | | 0=standby, 1=normal, 3=fault, 5=PV+batt online, 6=batt only |
| 3 | PV string 1 voltage (Vpv1) | ×0.1V | |
| 4 | PV string 1 current (Ipv1) | ×0.1A | |
| 5–6 | PV string 1 power (Ppv1 H/L) | ×0.1W | 32-bit |
| 7 | PV string 2 voltage (Vpv2) | ×0.1V | |
| 8 | PV string 2 current (Ipv2) | ×0.1A | |
| 9–10 | PV string 2 power (Ppv2 H/L) | ×0.1W | 32-bit |
| 35–36 | Total output power (Pac H/L) | ×0.1W | 32-bit |
| 37 | Grid frequency (Fac) | ×0.01Hz | |
| 38 | Grid voltage (Vac1) | ×0.1V | |
| 53–54 | Energy today (Eac_today H/L) | ×0.1kWh | 32-bit. May undercount — use PV string registers for accurate daily total |

### 17.2 Input Registers (Function 04) — 1000 range

| Register | Description | Scale | Notes |
|---|---|---|---|
| 1000 | System work mode | | |
| 1009–1010 | Battery discharge power (H/L) | ×0.1W | 32-bit. Non-zero when discharging. |
| 1011–1012 | Battery charge power (H/L) | ×0.1W | 32-bit. Non-zero when charging. |
| 1013 | Battery voltage | ×0.1V | |
| 1014 | Battery SOC | % | Direct percentage. ⚠ Verify correct register during commissioning. |
| 1021–1022 | Grid import power (H/L) | ×0.1W | 32-bit. ⚠ Verify sign convention before enabling heater. |
| 1023–1024 | Grid export power (H/L) | ×0.1W | 32-bit. ⚠ Verify sign convention before enabling heater. |

> Battery charge and discharge are in separate registers. Grid import and export are also in separate registers. Verify all addresses against Protocol V1.20 during commissioning.

---

## 18. Outstanding / To Be Determined

| Item | Notes |
|---|---|
| Growatt Modbus RS485 link | Install MAX485 at W, test over full cable run. Verify export/import sign convention. |
| Inter-controller RS485 | Install MAX485 hardware at both ends. Test over 40m 1mm T&E. Verify line bias. |
| Solar pump minimum current | Calibrate `SOLAR_PUMP_MIN_CURRENT_A` during commissioning |
| PC fan minimum duty | Calibrate `FAN_MIN_DUTY_PCT` during commissioning |
| Fan flap travel time | Calibrate `FAN_FLAP_OPEN_MS` during commissioning |
| Valve power-up wait | Verify `VALVE_POWERUP_WAIT_MS` (30s conservative) on-site |
| H-pump calibration curve | Run `cal_pump` to derive data; update `calcPred()` formula parameters |
| W solar pump duty curve | Run cal_pump, then replace temporary `calcPumpDuty()` with data-derived curve |
| DHW pipe control | To be added at a later date |

---

## 19. Document Changelog

| Version | Changes |
|---|---|
| 1.0–5.0 | Initial through major update: 15VDC, display, valves, fans, RS485, Growatt, firmware notes |
| 5.1–5.9 | Iterative refinements: fault history, button redesign, valve wiring, fan control, RS485 packets, RTC fallback, DS18B20 sensor additions |
| 6.0 | Full update to match implemented code: DS18B20 13 total (7 at H, 6 at W); dual heater sensor with arbitration (`getHeaterOutC`); solar target SOLAR_TANK_PLUS5 → SOLAR_TANK_PLUS8 (target = tankTop+8, capped 87°C); summer startup straight to phase 3; heater duty algorithm rework (8-packet averaging, 5s start delay, hot pipe cap, SOC reservation tiers, hysteresis stop); `MHM_SOC_LIM` mode added (3 modes total); hot tank protection (83°C); H-side solar pump direct drive (`calcHPumpDuty` predictive formula, float, Timer1 ISR); heater element-fail detection removed; dual sensor 5s grace; ISR power cap `heaterPowerCapPct`; hard lockout auto-clear at 88°C and via page 4 Ack; UFH dump threshold 90°C→93°C; solar overheat threshold 83→91°C; fire alarm documented; hen house door and window manual buttons documented; SD logging: datetime column, header validation; `HEATER_ENABLED` now true; wired-OR solar pump circuit documented; TFT_eSPI library; serial log2file |
