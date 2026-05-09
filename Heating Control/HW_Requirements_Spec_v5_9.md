# H & W Integrated Control System — Complete Project Reference
**Version 5.9** — Requirements Specification

> **Note:** This document is the baseline requirements spec. Post-spec changes are captured in [SPEC_CHANGES.md](SPEC_CHANGES.md).

---

## 1. Project Status

Requirements gathering complete. Hardware specification complete. Software implementation is the next stage.

---

## 2. Physical Infrastructure

### 2.1 Cable Runs Between H and W (40m)

| Cable | Use |
|---|---|
| 6mm T&E | Mains power in both directions (grid feed to W from H, PV export from W back to H) |
| 2.5mm T&E | Live + Neutral carry 15VDC power to both mid-point momentary switches (unlock and outside light). Earth wire carries buzzer trigger signal from W relay to buzzer at H. |
| 1mm T&E cable 1 | Live = unlock switch output signal to W / holds LED high when W is unlocked; Neutral = outside light switch output signal to W; Earth = spare |
| 1mm T&E cable 2 | Both conductors and earth fully spare — intended for inter-controller RS485 communication between H and W controllers |
| RS485 screened 24AWG cable | Carries SDM230 meter data to Growatt inverter only. SDM230 physically at W; read by Growatt to calculate grid export |
| 4 × 15mm pipes in vacuum-insulated outer pipe | Hot heating system pipe, cold heating system pipe, DHW pipe (future), cold fresh water pipe |
| 10mm earth cable | Earthing between H and W |

### 2.2 Pipe Bundle Detail

- The vacuum-insulated outer pipe maintains thermal insulation of the 4 inner pipes
- Vacuum pump at W runs to maintain this vacuum whenever the heating system is actively transferring heat in either direction

---

## 3. Locations & Layout

| Location | Description |
|---|---|
| H | The house/dwelling — contains heat store tank, log burner, electric heater, and associated valves and controls |
| W | The workshop — contains solar thermal array, PV inverter, UFH, and associated valves and controls. 40m from H |
| Mid-point | Near the back door of the dwelling. Contains: unlock button, outside light switch, and unlocked LED indicator. Treated as a separate node from H in system topology |

---

## 4. Components & Sensors

### 4.1 At W (Workshop)

**Heating**
- Thermal solar array: 2 × 30 evacuated tubes by Navitron, rated 3.6kW, south-facing wall
- 12VDC brushless pump, under 10W, 2-wire, on cold side of thermal solar. Speed controlled by MOSFET switching 15VDC supply at clocking rate — no separate PWM signal wire
- 2 × 12VDC motorised valves on cold heating system pipe (2-wire, reverse voltage to close): one feeds UFH cold side, one feeds solar cold side via flap non-return valve
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
- DS18B20: UFH supply (at UFH pump — return from UFH circuit. Used for logging and pump-off check at 28°C)
- DS18B20: UFH post TMV (after thermostatic mixing valve — supply to UFH pipes. Used for 45°C hard lockout only)
- DS18B20: Workshop air temperature
- DS18B20: outside air temperature
- PIR sensor: 15VDC output when activated — voltage divider required
- Summer/winter mode — REMOVED: now controlled via display at H
- Door reed switch: dry contact, normally open, closes when door opens (opens when door closed)
- Door handle reed switch: dry contact, closed when handle is pushed down
- Window winch reed switches: fully open, fully closed, manual lock
- Window winch safety limit switch: dry contact, normally open, closes at mechanical over-open limit
- Vacuum sensor: dry contact — normally open (HIGH via pull-up = low vacuum), closes when full vacuum achieved (LOW = full vacuum)
- INA219 current sensor module (I2C) on solar pump positive feed

**Other**
- Electric door lock (15VDC) — 2 relay H-bridge required, pulsed 1s to unlock then de-energised
- 230VAC wall axial fan (S&P HCBB/4-450/H, 480W) with BES fan speed controller (manual dial) and external flap actuator (15VDC)
- 230VAC window winch
- 230VAC vacuum pump with 15VDC motorised isolation valve (2-wire, reverse voltage to close, 2 relays required)
- 230VAC external LED lights
- 15VDC alarm sounder (sounds when PIR detects intruder whilst W is locked, auto-stops after 1 minute)
- Manual latching switch at W — suppresses auto-relock whilst ON
- 2 × Fractal Design Dynamic X2 GP-14 (140mm, 3-pin, 12VDC). MOSFET (IRLZ44N) switches 12VDC supply at 25kHz PWM. Powered from 12V DC-DC buck converter from 15V bus. Momentary push button at W sets 8-hour full speed timer.

### 4.2 At H (House)

**Heating**
- 1000L heat store tank with bottom, middle and top pipe connections
- 3kW electric heater: cold side fed by 2-port valve from hot heating pipe from W; output connects to shared manifold with top of tank and log burner hot pipe
- Log burner with 70L water jacket
- 12VDC motorised valve on log burner cold pipe (2-wire, reverse voltage to close, 2 relays)
- 12VDC motorised valve on bottom-of-tank pipe (2-wire, reverse voltage to close, 2 relays)
- 2-port 12VDC motorised valve on hot heating system pipe from W (2-wire, reverse voltage to close, 2 relays): switches between → electric heater cold side OR → middle of tank
- SSR-40DA zero-crossing solid state relay for heater power control (cycle-burst method)
- AC zero-crossing detector module wired to H Mega INT0 pin — serves three functions: (1) cycle-level heater burst timing, (2) instant grid outage detection, (3) grid restore detection

**Sensors**
- DS18B20: tank bottom
- DS18B20: tank middle
- DS18B20: tank top
- DS18B20: hot heating system pipe to W
- DS18B20: cold heating system pipe to W
- DS18B20: heater hot side output
- 12VDC temperature module: outputs 12VDC when log burner water jacket exceeds 28°C — voltage divider required
- 15VDC bus voltage monitor: resistor divider (10kΩ + 4.7kΩ) on H Mega ADC pin

**Controls & Indicators**
- 3 × panel-mount buttons at H display: SELECT, ↑ (UP), ↓ (DOWN)
- 3.5" ILI9488 TFT display 480×320 SPI — see Section 9 for full display specification
- MicroSD card module (SPI) for data logging
- DS3231 RTC module (I2C) — essential for 5am heating trigger and time-gated security alerts
- 12VDC backup PSU relay — switched by H Mega when bus voltage drops below 12V

### 4.3 At Mid-Point

- Momentary unlock button (15VDC from 2.5mm T&E live, output on 1mm cable 1 live)
- Momentary outside light switch (15VDC from 2.5mm T&E live, output on 1mm cable 1 neutral)
- LED indicator: controlled entirely by W Mega on D34. Priority order (highest first):
  1. Fault active — 1s on / 1s off continuous flash
  2. Manual heater active (no fault) — 1s on / 5s off slow single flash
  3. Workshop unlocked, no fault, no manual heater — steady HIGH
  4. Workshop locked, no fault, no manual heater — off
  - LED clears to next priority state when higher priority clears.

---

## 5. Pipe & Valve Topology

### 5.1 Cold Heating System Pipe (H to W)

Feeds from H: log burner cold valve OR bottom-of-tank valve (both connect to same cold pipe). At W: splits to → UFH cold side valve OR solar cold side valve (via non-return flap valve).

### 5.2 Hot Heating System Pipe (W to H)

From W: UFH hot output and solar hot output. At H: enters 2-port valve → switches to heater cold side OR middle of tank. Heater output joins shared manifold → top of tank + log burner hot pipe.

---

## 6. Security & Access Logic

### 6.1 Door Lock & Unlock

- W door locked by default (15VDC electric lock, 2-relay H-bridge, pulsed 1s to unlock then de-energised)
- Unlock: pressing momentary button at mid-point sends signal to W Mega → door lock pulsed 1s → 1mm T&E live held HIGH → LED at mid-point lights steady
- Door reed switch: dry contact, normally open, closes when door opens, opens when door closes

### 6.2 Auto-Relock Logic

Auto-relock is prevented while ANY of the following conditions are true:
- Door is open (reed switch closed)
- PIR is detecting presence (active output)
- Manual suppression switch at W is ON
- Window is not fully closed AND locked (fully closed reed switch not active OR manual lock reed switch not active)

Relock timer: starts only when ALL four conditions are simultaneously clear. If any condition becomes active during the 5-minute countdown, timer resets to zero. After 5 continuous minutes of all four conditions clear → door relocks.

Log entries: `Auto-relock prevented — door open` / `— PIR active` / `— suppression switch on` / `— window not closed/locked`

### 6.3 Security Alerts

- **Intruder alert:** PIR detects movement whilst W is locked → alarm at W sounds for 1 minute then auto-stops; buzzer at H sounds while PIR is active; logged
- **Door handle alert:** door handle reed switch closes whilst W is locked → between 23:00–06:00 buzzer sounds minimum 10s (continues while held if longer); between 06:00–23:00 buzzer sounds only while handle held; logged with timestamp in both cases

---

## 7. Lighting & Ventilation Logic

### 7.1 External Lights at W

Controlled by: momentary switch at mid-point OR momentary switch at W. 230VAC LED external lights switched by relay.

### 7.2 Workshop Fan (S&P HCBB/4-450/H)

- 230VAC axial fan with BES manual speed controller (dial-set speed) and external flap actuator (15VDC)
- Fan on/off controlled by relay on W 230VAC relay board
- **Automatic night cooling** (230VAC fan only — PC fans are unaffected by night cooling logic): fan ON when outside air is more than 5°C below Workshop air temp; fan OFF when gap closes to 2°C or less
- Night Cooling can be disabled via display toggle at H (page 4). State shown in status bar.
- Flap actuator opens before fan relay energises; closes after fan relay de-energises

### 7.3 Window Winch

Manual trigger only — 3 relays on W 230VAC relay board (direction open, direction close, power).

Valid relay states — firmware treats as atomic group only:

| State | Relay 1 (direction open) | Relay 2 (direction close) | Relay 3 (power) | Notes |
|---|---|---|---|---|
| STOP | OFF | OFF | OFF | Only safe stop state. Power-up default. |
| OPEN | ON | OFF | ON | Relay 1 and 3 energised simultaneously |
| CLOSE | OFF | ON | ON | Relay 2 and 3 energised simultaneously |
| FORBIDDEN | ON | ON | any | Hard interlock — must never occur |
| FORBIDDEN | OFF | OFF | ON | Hard interlock — power without direction |
| FORBIDDEN | ON | OFF | OFF | Hard interlock — direction without power |
| FORBIDDEN | OFF | ON | OFF | Hard interlock — direction without power |

Direction change: always STOP (all three OFF) → 1 second pause → new direction.

Firmware implementation: single function `setWinch(STOP/OPEN/CLOSE)` — never set individual relays independently.

Immediate stop triggers — de-energise all three relays instantly then apply lockout:
- Fully open reed switch activates → STOP + lock out OPEN direction. Lockout clears when switch deactivates.
- Fully closed reed switch activates → STOP + lock out CLOSE direction. Lockout clears when switch deactivates.
- Manual lock reed switch activates → STOP + lock out BOTH directions. Lockout clears when switch deactivates.
- Safety limit switch activates → STOP + lock out OPEN direction + flash LED + log `Window Winch Over Open`. Close direction still available. Clears when switch opens.

### 7.4 PC Cooling Fans at W (Fractal Design Dynamic X2 GP-14)

2 × 140mm 3-pin 12VDC fans. MOSFET (IRLZ44N) switches 12VDC supply at 25kHz PWM. Powered from 12V DC-DC buck converter from 15V bus.

Fan control modes — priority order:

| Priority | Mode | Speed | Trigger / Duration |
|---|---|---|---|
| 1 | Full speed timer | 100% | W button press = 8hrs. Adjustable at H display in 1-hour increments (0–24hrs). Counts down regardless of lock status. |
| 2 | Base speed timer | Base % | Set at H display in 1-day increments (0–30 days). Counts down continuously regardless of lock status. When expires → fans off until workshop unlocked. |
| 3 | Workshop unlocked | Base % | Fans run at base speed whenever workshop is unlocked. Off when locked. |
| 4 | Off | 0% | Workshop locked, no timers active. |

If full speed timer and base speed timer both active: full speed until full speed timer expires → base speed until base speed timer expires → off until unlocked.

Base speed: set at H display in 10% increments (0–100%). Firmware enforces minimum speed if fan stall detected at low duty cycle.

Tachometer fault: if either fan reads zero RPM when commanded above minimum speed for >5s → flash LED, log fault. No buzzer.

Status bar on all display pages: `FAN FULL 07:23` / `FAN BASE 3d 14h` / `FAN BASE` / `FAN OFF`

---

## 8. Heating Control Logic

### 8.1 Winter Mode

Active when Mode is set to Winter via display at H. UFH heating operates normally.

**Thermal Solar — Winter**
- Solar sequence triggers when solar hot side ≥ 18°C
- Open UFH cold valve, open solar cold valve, start solar pump clocking 200ms ON every 2s
- Solar pump speed ramps up starting 2°C below target, reaching 100% at 2°C above target. Winter solar target: 20°C (UFH circuit temperature)
- Fault condition: pump running above clocking speed for >10s AND solar cold side >10°C above tank bottom temperature → thermal solar overheat alert (see Section 11). If tank bottom reading is stale due to RS485 comms fault: do not trigger this fault — assume solar is operating normally.
- When solar hot side is no longer >2°C warmer than cold side: close solar cold valve, stop pump
- Frost protection: if either solar side drops below 2°C → open UFH valve, run solar pump at full speed until solar output reaches 8°C → stop pump but leave UFH valve open until next heating cycle begins or solar reaches 18°C. Runs regardless of heating lockout state.

**Morning Heating Cycle (5am daily, RTC-triggered)**
- Trigger: RTC reaches 05:00. No catch-up on power-cut restart — wait for next 5am.
- Normal mode: if W air temp < 13.5°C → open UFH valve + start W central heating pump. Heating stops when W air temp reaches 13.5°C. Once target reached, UFH locked out until next 5am regardless of temperature drop.
- Boost at 5am mode: heating triggers at boost target temp (13–20°C, default 15.5°C), turns OFF at boost target +0.5°C. Active until 10am the following day.
- 8hr boost mode: heating starts immediately regardless of time, runs at boost target temp for 8 hours. UFH lockout still applies once target reached.
- Frost protection during heating: if solar cold side < 2°C → open solar cold valve, run solar pump at full speed until 8°C reached. Runs regardless of heating lockout state.
- UFH overheat: if UFH post TMV temp exceeds 45°C while UFH pump is running → stop pump, close UFH valve, flash LED, log `UFH Overheat`, hard lockout (does not restart until system restart). Frost protection may still use UFH regardless of this lockout.
- Once W air temp reaches target: open log burner cold valve + ensure top-of-tank path is open, close bottom-of-tank valve. W UFH pump continues running until UFH supply thermistor drops below 28°C → everything stops until 5am next day.

**Heat Source Selection at H (when morning heating is ON)**
- If log burner temp module active (jacket > 28°C, hysteresis off below 25°C): open log burner cold valve; set 2-port valve to heater cold side (circulates heat from log burner through manifold to top of tank)
- If log burner temp module NOT active: close log burner cold valve, open bottom-of-tank valve
- If middle-of-tank temp > 30°C: switch 2-port valve to middle of tank
- If middle-of-tank temp < 30°C: switch 2-port valve to heater cold side (connects to top of tank via heater)
- Log burner and solar thermal can run simultaneously in winter — both circulate through UFH circuit. This is a valid and expected state requiring no special handling.

**PV Export Override (winter day)**
If PV export reaches 0.5kW during winter mode: switch entirely to summer mode logic (including heater activation). Summer mode continues until PV production drops to zero, then revert to winter mode.

### 8.2 Summer Mode

Active when Mode is set to Summer via display at H, OR when PV export override is active. UFH is completely disabled in summer mode.

**Summer Startup Sequence**

Triggered when: PV export reaches 0.5kW OR solar hot side OR solar cold side reaches 50°C.

1. Open log burner cold valve + 2-port valve to top-of-tank path → solar pump starts clocking minimum speed → circulate until hot heating pipe reaches bottom-of-tank temperature
2. Close log burner cold valve, open bottom-of-tank valve → circulate until hot heating pipe reaches top-of-tank temperature OR electric heater becomes powered
3. Once hot pipe matches top-of-tank temperature: switch 2-port valve to heater cold side (water circulates through heater to top-of-tank manifold)

If both triggers fire simultaneously: sequence runs once only (double-start prevention flag). If solar triggers but PV export not yet at 0.5kW: valve startup sequence runs, heater activation waits for export threshold.

**Thermal Solar — Summer**
- Solar pump clocks 500ms ON every 20s at minimum speed once summer startup triggered
- Solar pump speed controlled by whichever sensor is closest to or exceeding its target:
  - Solar hot side: target = top-of-tank temp + 5°C (Tank+5 mode) OR 80°C max (Max mode). Ramps from 2°C below target to 100% at 2°C above. 80°C absolute maximum prevents overheating.
  - Heater output (only when heater powered): target = top-of-tank temp + 5°C (Tank+5 mode) OR 89°C max (Max mode). Pump reaches 100% at 91°C.
- Solar target mode (Tank+5 vs Max) toggled from display page 4. Greyed out in winter mode.
- Solar system stays active while: hot heating pipe is >5°C warmer than cold heating pipe, OR PV panels are producing any power
- When top-of-tank valve held open but heater not powered AND solar temp >15°C below target: solar pump does not clock
- Fault condition: pump running above clocking speed for >10s AND solar cold side >10°C above tank bottom temperature → thermal solar overheat alert (see Section 11). If tank bottom reading is stale due to RS485 comms fault: do not trigger this fault.

**Electric Heater Control**
- Heater powers ON when PV export first reaches 0.5kW (trigger threshold with hysteresis)
- Once running: heater power modulated via SSR cycle-burst to maintain PV export at ~100W
- Heater switches OFF if PV export drops below 0.1kW and cannot be maintained. Must reach 0.5kW again to re-engage.
- Above 91°C heater output: heater power begins reducing
- At 92°C heater output: heater fully off
- When heater stops being powered: 2-port valve remains open (stays connected to heater cold side)
- Once heater no longer powered and solar reaches its own target: top-of-tank valve no longer held open
- Minimum solar pump speed when heater powered: set by heater power level and hot pipe supply temperature — calibration required on-site

**Overheat Protection (summer)**
If heater output >92°C for >1 minute AND heater is off: close bottom-of-tank valve, open log burner cold valve, ensure top-of-tank path open. Hold until solar goes off (PV = zero) → restart from beginning of summer sequence.

### 8.3 Manual Heater Mode

Activated from display page 4. Three states:

| State | Behaviour |
|---|---|
| Off | Heater off, no manual control |
| Manual — SOC limited | Heater on, modulated via SSR cycle-burst to keep grid import at ~100W. Auto-off if battery SOC drops to 50%. SOC read from Growatt Modbus. If SOC data older than 10s — force heater off. |
| Manual — override SOC | Full 3kW continuous — SSR held fully ON every cycle. No SOC check, no modulation. Runs until manually cancelled. Display shows red `MANUAL HTR — FULL 3kW` warning. |

All heater safety limits apply in all manual states: overheat (91°C), element fail, zero-crossing SSR control. Grid outage forces heater off in all manual states. Manual heater state is NOT stored in EEPROM — resets to Off on power cycle.

Logs: `Manual heater on — SOC limited` / `Manual heater on — SOC override` / `Manual heater off` / `Manual heater auto-off — SOC limit reached`

### 8.4 Solar Pump Speed — General Rules

- Speed ramps linearly: 0% at 2°C below target, 100% at 2°C above target
- In winter: target is W UFH temperature (20°C)
- In summer: target is the lower of the two sensor targets (solar hot side or heater output when powered)
- Minimum pump speed in summer when heater powered: set by heater power level and hot pipe supply temperature — to be calibrated by on-site testing
- Pump controlled by MOSFET (IRLZ44N) switching 15VDC supply at clocking rate. Minimum ON pulse 200ms ensures pump spins reliably.
- INA219 current sampled during ON period only, after 50ms spin-up delay, to avoid false zero readings during OFF periods

---

## 9. Display Specification

### 9.1 Hardware

- 3.5" ILI9488 TFT 480×320 SPI display at H
- 3 panel-mount buttons: SELECT, ↑ (UP), ↓ (DOWN)

### 9.2 Button Behaviour

Three navigation modes. SELECT moves one level deeper; scrolling past boundaries then pressing SELECT moves one level back.

| Mode | UP ↑ | DOWN ↓ | SELECT |
|---|---|---|---|
| Page scroll mode | Previous page | Next page | Enter item mode — highlights first item on current page |
| Item mode — on an item | Move to previous item | Move to next item | Enter option mode for highlighted item |
| Item mode — past top boundary | No action (at boundary) | Move to first item | Exit item mode → return to page scroll mode |
| Item mode — past bottom boundary | Move to last item | No action (at boundary) | Exit item mode → return to page scroll mode |
| Option mode | Previous option / decrement | Next option / increment | Confirm choice → return to item mode |

**Visual feedback:**
- Current page indicated by page number in status bar
- Highlighted item shown with amber border or inverted background
- Boundary state: highlight clears, subtle exit indicator shown (e.g. `< back to pages >` hint). UP/DOWN still navigates back to items from boundary state.
- Option mode: selected item shows current value cycling with each press

**Inactivity and wake:**
- 30 seconds no button press in item or option mode → exit to page scroll mode, clear highlight
- 1 hour no button press with no active fault → display backlight turns off
- Any button press wakes display from backlight-off state
- If any fault is active: display stays on, backlight never turns off
- After 1 hour no input: display returns to page 1

**Alert reset:** Physical RESET button removed. Alert reset is an item on page 4 controls — navigate to it with SELECT + UP/DOWN, then press SELECT to acknowledge and clear the flashing LED. Alert reset item only visible on page 4 when a fault alert is active. If fault condition still present when reset selected: LED re-triggers immediately.

**Boost shortcut:** Double-press SELECT from page scroll mode (not item mode) → toggles Boost at 5am on/off from any page.

### 9.3 Persistent Elements (all pages)

- **Top status bar:** Mode (Winter/Summer), Boost status + countdown, Night Cooling on/off, Solar Target mode (Tank+5/Max), Clock (from RTC)
- **Bottom fault bar:** all active faults shown with elapsed time. Scrolls if multiple active. Grid Outage shown here, not in status bar.
- **Manual override active:** red `MANUAL OVERRIDE ACTIVE — automatic valve control suspended` banner below status bar on all pages
- **Manual heater — SOC limited:** amber `MANUAL HEATER — SOC limited` banner below status bar on all pages while active
- **Manual heater — Override SOC:** red `MANUAL HEATER — FULL 3kW — no SOC limit` banner below status bar on all pages while active
- If both manual heater and manual valve override are active simultaneously: both banners stack below status bar

### 9.4 Page 1 — Heating System

- Solar / Workshop temperatures: solar hot, solar cold, UFH supply, UFH post TMV, Workshop air, outside air
- Tank / House temperatures: tank top, tank middle, tank bottom, hot pipe, cold pipe, heater output
- Power: heater kW, solar pump %
- Boost panel (when active): Workshop target temp
- Valve states overview: all 6 valves shown as open/closed badges
- Window winch state: open / closed / opening / closing / manual lock / over-open fault

### 9.5 Page 2 — Power & Inverter

- PV: string 1 kW, string 2 kW, total PV kW, today's generation kWh
- Grid: import/export kW with direction indicator (+/−), load power kW, grid voltage V, grid frequency Hz
- Battery: SOC %, charge/discharge power kW (with direction), battery voltage V
- Inverter: status (normal/fault/EPS/standby), temperature °C, fault code if active
- 15V bus voltage V
- Note: all data sourced from Growatt Modbus at 850ms poll interval.

### 9.6 Page 3 — Fault History

- Shows all faults logged since last system restart, most recent first
- Each entry shows: fault name, time first occurred, time resolved, duration
- Active faults shown in red. Resolved faults shown in grey.
- Active faults show elapsed time since onset, updating live
- Resolved faults show total duration (resolved time minus onset time)
- If no faults since restart: shows `No faults recorded since restart`
- Scrollable list — UP/DOWN buttons scroll entries when on this page and no item is highlighted
- Fault data held in RAM — built in memory as faults occur and resolve. Clears on power cycle.
- Maximum ~200 entries before RAM pressure on ATmega2560 (8KB RAM, ~40 bytes per entry). Sufficient for any normal session.

### 9.7 Page 4 — System Controls

| Order | Item | UP/DOWN action | Values / notes |
|---|---|---|---|
| 1 | Boost | Cycles: Off → Boost at 5am → 8hr boost → Off | Default: Off. NOT stored in EEPROM. Double-press SELECT from page scroll mode also toggles Boost at 5am. |
| 2 | Boost target | +/−0.5°C per press | 13°C – 20°C. Default: 15.5°C. NOT stored in EEPROM |
| 3 | Mode | Toggle | Winter ↔ Summer |
| 4 | Manual heater | Cycles: Off → SOC limited → Override SOC → Off | NOT stored in EEPROM |
| 5 | Night Cooling | Toggle | On ↔ Off |
| 6 | Solar Target | Toggle (greyed out in winter) | Tank+5°C ↔ Max |
| 7 | Fan base speed | +/−10% per press | 0–100%. Firmware enforces minimum if stall detected. |
| 8 | Fan full speed timer | +/−1 hour per press | 0–24hrs. W button sets 8hrs directly. 0 = cancel timer. |
| 9 | Fan base speed timer | +/−1 day per press | 0–30 days. Counts down regardless of lock status. 0 = off. |
| 10 | Display brightness | +/−10% per press | 10–100% |
| 11 | SD safe remove | SELECT to eject | Flushes writes, closes filesystem. Auto-reinitialises on card reinsert. |
| 12 | Alert reset | SELECT to acknowledge | Only visible when a fault alert is active. Clears flashing LED. Re-triggers immediately if fault still present. |

EEPROM storage: Mode, Night Cooling, Solar Target, Display brightness written after 30s timeout following last change. Boost state, Boost target, Manual heater state NOT stored in EEPROM.

### 9.8 Page 5 — Valve States & Manual Override

- All valve states displayed: UFH cold (Workshop), solar cold (Workshop), vacuum isolation (Workshop), log burner cold (House), bottom-of-tank (House), 2-port (House)
- Window winch status: direction state (idle/opening/closing), reed switch states (fully open, fully closed, manual lock engaged), safety limit switch state
- Manual override entered via SELECT when override item is highlighted. Must be explicitly entered — prevents accidental valve changes.
- In override: UP/DOWN scroll between valves, SELECT toggles valve state, actuates immediately
- Workshop valve override commands sent via RS485 to W Mega for actuation
- Safety faults still function in override — safety always overrides manual state
- Override state NOT stored in EEPROM — always reverts to automatic on power cycle
- Log: `Manual override entered`, `Manual override exited`, with timestamps and valves changed

### 9.9 Page Summary

| Page | Content | Notes |
|---|---|---|
| 1 | Heating system | All temps, heater kW, solar pump %, boost panel, valve overview. Default page on power-on and after 1hr inactivity. |
| 2 | Power & inverter | All Growatt data, PV strings, battery, grid, inverter status, bus voltage |
| 3 | Fault history | All faults since restart, most recent first. Red = active, grey = resolved. |
| 4 | System controls | Boost, mode, manual heater, night cooling, solar target, brightness, SD eject |
| 5 | Valve states & manual override | All valve states. Manual override requires explicit entry. |

---

## 10. Vacuum System

- Vacuum pump (230VAC) with 15VDC motorised isolation valve (2-wire, reverse voltage to close, 2-relay H-bridge)
- Vacuum sensor: dry contact, normally open (Mega reads HIGH via pull-up = low vacuum), closes when full vacuum achieved (Mega reads LOW)
- System activates when heating or solar becomes active (either direction of heat transfer)
- Sequence: open isolation valve → start vacuum pump → run until vacuum sensor confirms full vacuum (LOW)
- Once full vacuum achieved: run pump for additional 5 minutes, then close isolation valve 10 seconds before pump stops
- Once full vacuum achieved in a session: do not restart pump even if vacuum is subsequently lost. Isolation valve remains closed. Resets at next day's heating/solar activation.
- Maximum runtime: 30 minutes. If full vacuum not achieved within 30 minutes → stop pump, flash LED, log `Vac Pump Over Runtime`. No buzzer.

---

## 11. Safety Monitoring & Alerts

**Global alert rules:**
- Buzzer at H sounds while fault condition is active — stops automatically when condition clears
- Mid-point LED flashes 1s on/1s off from fault onset — continues until Alert Reset is selected on page 4 of H display
- If fault still present when Alert Reset selected: LED re-triggers immediately
- All faults shown on H display fault bar with elapsed time
- All faults logged with timestamp to SD card
- Alert Reset does not affect any heating, pump, or valve states

| Error code | Trigger condition | Actions | Clears / resets |
|---|---|---|---|
| Thermal Solar Overheat — cold side high | Pump above clocking speed >10s AND solar cold side >10°C above tank bottom | Buzzer + LED flash + Display + Log. If both sides ≥90°C: open UFH valve, close tank valves, remove heater power, run UFH pump + solar pump until clears | Buzzer stops when clears. LED until Alert Reset selected on page 4. |
| Thermal Solar Overheat — hot side high | Tank bottom <70°C AND solar hot >83°C AND pump at 100% | Buzzer + LED flash + Display + Log. UFH dump if both sides ≥90°C | Buzzer stops when clears. LED until Alert Reset selected on page 4. |
| Solar Pump Fault | Pump commanded ON, current below calibrated minimum for >5s | Buzzer + LED flash + Display + Log. UFH dump if both sides ≥90°C | Buzzer stops when clears. LED until Alert Reset selected on page 4. |
| Heater Overheat — power reduction sustained | Heater reducing power due to >91°C output for >20s AND hot pipe <80°C | Log + Display only | Auto-clears when condition resolves. |
| Heater Overheat — heater shut down | Above condition continues >1 minute | Buzzer + LED flash + Display + Log + Heater off | Buzzer stops when clears. LED until Alert Reset selected on page 4. Heater off until restart. |
| Heater Element Fail | Power applied >30s with no temperature rise on heater output | LED flash + Display + Log + Heater off (no buzzer) | LED until Alert Reset selected on page 4. Heater off until system restart. |
| UFH Overheat | UFH post TMV temp >45°C while UFH pump running | LED flash + Display + Log + UFH pump stop + UFH valve close (no buzzer). Hard lockout — does not restart until system restart. Frost protection may still use UFH regardless. | LED until Alert Reset selected on page 4. UFH locked out until restart. |
| Frost Protection — cold side not recovering | Frost protection active >1 min AND solar cold side not yet reached 4°C | LED flash + Display + Log (no buzzer) | LED until Alert Reset selected on page 4. |
| Vac Pump Over Runtime | Vacuum pump running >30 min without achieving full vacuum | LED flash + Display + Log + Pump stopped (no buzzer) | LED until Alert Reset selected on page 4. |
| 15V Bus Low | Bus voltage below 14V for >10s | LED flash + Display + Log (no buzzer) | Clears automatically above 14V. LED until Alert Reset selected on page 4. |
| 12VDC PSU Activated | Bus voltage drops below 12V | Display + Log + 12VDC PSU relay ON | PSU relay off above 12.5V. Logged as `12VDC PSU Deactivated`. |
| 15V Bus Restored | Bus voltage recovers above 14V | Display + Log | LED clears automatically. |
| Grid Outage | Zero-crossing pulses absent >50ms | Display fault bar + Log + Heater forced off (no buzzer, no LED flash) | Auto-clears when grid restored. Logged as `Grid Restored`. |
| Inverter Fault | Non-zero fault code from Growatt Modbus | LED flash + Display + Log with fault code (no buzzer) | LED until Alert Reset selected on page 4. Clears when fault code returns zero. |
| Growatt Comms Fault | 5+ consecutive failed Modbus polls (~4.25s) | LED flash + Display + Log + Heater forced off (no buzzer) | Clears when comms restore. LED until Alert Reset selected on page 4. |
| Intruder Alert | PIR detects movement whilst W is locked | Buzzer at H (while PIR active) + W alarm 1 min auto-stop + Log | Buzzer stops when PIR clears. No LED flash. |
| Door Handle Alert 23:00–06:00 | Handle reed switch active whilst locked, night hours | Buzzer minimum 10s (continues while held if >10s) + Log | Buzzer stops when handle released (after minimum 10s). |
| Door Handle Alert 06:00–23:00 | Handle reed switch active whilst locked, day hours | Buzzer only while handle held + Log | Buzzer stops immediately on release. |
| Window Winch Over Open | Safety limit switch closes while winch opening | LED flash + Display + Log + Winch open direction locked out (no buzzer) | LED until Alert Reset selected on page 4. Close direction still available. Clears when switch opens. |
| RS485 Comms Fault | 5+ consecutive missed H↔W packets (~1.25s) | LED flash + Display + Log (no buzzer) | Clears when comms resume. LED until Alert Reset selected on page 4. |
| Workshop Fan 1 Fault | Fan 1 tach zero while commanded above minimum speed for >5s | LED flash + Display + Log (no buzzer) | LED until Alert Reset selected on page 4. |
| Workshop Fan 2 Fault | Fan 2 tach zero while commanded above minimum speed for >5s | LED flash + Display + Log (no buzzer) | LED until Alert Reset selected on page 4. |

### 11.2 Sensor Fault Behaviour

DS18B20 fault detection: discard readings of exactly 85.00°C, −127°C, or −128°C. A sensor is considered failed after 3 consecutive invalid readings. Clears when 3 consecutive valid readings received. Log `[sensor] sensor recovered` on recovery.

| Sensor | Action on fault |
|---|---|
| Solar hot side | Close log burner cold valve, bottom-of-tank valve, 2-port valve. Open UFH cold valve and solar cold valve. Start UFH pump. Force heater off (must stay off while dump is active). Flash LED. Log `Solar hot sensor fault — tank valves closed, UFH dump active, heater off`. |
| Solar cold side | Same as solar hot side fault. |
| Heater output | Heater immediately off and locked out until sensor recovers. Flash LED. Log `Heater output sensor fault — heater locked out`. |
| UFH post TMV | Flash LED + log `UFH post TMV sensor fault` only. UFH continues but 45°C lockout cannot function — firmware must make this explicit in fault message. |
| UFH supply | Flash LED + log `UFH supply sensor fault` only. No automatic action. 28°C pump-off check disabled until sensor recovers. |
| Tank top / mid / bot | Flash LED + log `[sensor] sensor fault` only. No automatic action. |
| Hot pipe / cold pipe | Flash LED + log `[sensor] sensor fault` only. No automatic action. |
| Workshop air | Flash LED + log `Workshop air sensor fault` only. No automatic action. |
| Outside air | Flash LED + log `Outside air sensor fault` only. No automatic action. |
| Inverter temperature | Flash LED + log `Inverter temp sensor fault` only. No automatic action. |

---

## 12. Power Supply & Voltage Management

System nominal bus voltage: 15VDC (battery-backed). Both Mega controllers powered from 15VDC bus via barrel jack. Voltage monitoring at H only via resistor divider on H Mega ADC pin.

| Threshold | Action |
|---|---|
| Below 14V for >10s | Flash LED + Log `15V Bus Low`. System keeps running. Buzzer does not sound. |
| Below 12V | Activate 12VDC backup PSU relay at H + Log `12VDC PSU Activated`. |
| Recovers above 12.5V | Deactivate PSU relay + Log `12VDC PSU Deactivated` |
| Recovers above 14V | Clear LED flash + Log `15V Bus Restored` |

Grid outage detected instantly via zero-crossing detector at H (≤50ms) — no separate grid voltage monitor needed. Heater forced off on grid outage regardless of manual mode.

---

## 13. Inter-Controller Communications

- RS485 link over spare 1mm T&E cable between H and W (40m)
- Both MAX485 modules at 9600 baud. Poll interval: 250ms (4 times per second)
- Full cycle time at 9600 baud: Growatt Modbus (~75ms) + W→H packet (~36ms) + H→W packet (~36ms) + processing (~20ms) = ~167ms. Comfortably within 250ms window.
- Growatt Modbus polled at 850ms (manufacturer minimum command period). Separate UART from inter-controller link.
- RS485 line bias: 560Ω pull-up on A line to 5V, 560Ω pull-down on B line to GND at one end of each RS485 run.
- If 5 or more consecutive packets missed (~1.25s): RS485 Comms Fault triggered
- On comms failure: W continues on last known H values. H continues on last known W values. Both flag fault.

### 13.1 W → H Packet (every 250ms)

- Solar hot temp, solar cold temp, UFH supply temp, UFH post TMV temp, Workshop air temp, outside air temp
- Solar pump duty %, PV export kW, grid import kW
- Fault flags (all W-side faults)
- Valve states: UFH cold, solar cold, vacuum isolation, door lock, fan flap
- Vacuum pump state, vacuum sensor state
- Door state (open/closed), door lock state (locked/unlocked), PIR state, manual relock switch state
- Window winch state: direction (idle/opening/closing), reed switch states (fully open, fully closed, manual lock), safety limit switch state
- Fan state: current speed %, full speed timer remaining, base speed timer remaining

### 13.2 H → W Packet (every 250ms)

- Tank bottom temp, tank middle temp, tank top temp
- Hot pipe temp, cold pipe temp, heater output temp
- Heater power %, boost status, boost mode (off/5am/8hr), boost target temp
- Mode (winter/summer), night cooling state, solar target mode
- Fan speed command: target duty cycle % (W executes autonomously using this + its own lock state knowledge)
- Actuator override commands: valve states to override at W (from manual override mode on page 5). Includes a command type field to distinguish override commands from normal state data.
- Alert reset acknowledgement
- Time sync (on W request at boot and every hour thereafter)

---

## 14. Data Logging

SD card at H only. All W data received via RS485 and logged centrally.

**Continuous log** (every 250ms): all 12 DS18B20 temperatures (tank top/mid/bot, hot pipe, cold pipe, heater output, solar hot, solar cold, UFH supply, UFH post TMV, Workshop air, outside air), solar pump %, heater kW, heater restricted flag, PV string 1 & 2, total PV, import/export, load, battery SOC, charge/discharge, battery voltage, grid voltage, grid frequency, inverter temp, inverter status, 15V bus voltage, fan 1 RPM, fan 2 RPM, fan speed %

**Event log:** all error codes with timestamp, all system state changes (boost on/off, mode change, manual override, grid outage etc.)

**SD card safe remove:** page 4 option flushes writes and closes filesystem cleanly before card removal. Auto-reinitialises on card reinsert.

Log format: CSV for easy import to spreadsheet or analysis tool.

---

## 15. Data Analysis — SD Card & SQLite

### 15.1 Overview

The SD card logs at 250ms intervals producing approximately 345,600 rows per day and ~126 million rows per year — too large for Excel. A SQLite database on a PC provides fast queries across the full dataset with no specialist software required.

### 15.2 Workflow

1. Remove SD card from H controller (use SD safe remove option on page 4 first)
2. Insert SD card into PC
3. Run Python import script — reads CSV and imports into SQLite database file on PC (takes a few minutes for a year of data)
4. Run analysis scripts to produce graphs and reports
5. Reinsert SD card into H controller (auto-reinitialises)

### 15.3 Planned Analysis Scripts

- **kWh to heater per day** — line/bar graph over 1 year. SQL sums heater power readings per day and converts to kWh.
- **Daily temperature graph** — for a user-specified date: line graph showing tank bottom/mid/top, solar hot/cold, heating pipe hot/cold, heater output temp. X axis = time, Y axis = 0–100 (degrees °C and pump/heater %). Solar pump % and heater power % also plotted. Heater line shown thicker when power is restricted by temperature (>91°C condition).
- Down-sampling: full 250ms resolution stored on SD card, but daily graphs display 1-minute averages (1,440 points per day) for performance.
- **Fault history report** — query all fault events, durations, and frequency from database. Useful for identifying recurring faults or patterns.
- Additional analysis scripts to be added as requirements emerge.

### 15.4 Database Schema

**Table: readings** — timestamp (Unix ms), solar_hot, solar_cold, ufh_pipe, w_air, outside_air, tank_top, tank_mid, tank_bot, hot_pipe, cold_pipe, heater_out, solar_pump_pct, heater_kw, heater_restricted (boolean), pv_string1_kw, pv_string2_kw, pv_export_kw, grid_import_kw, battery_soc_pct, battery_kw, grid_voltage, inverter_temp, bus_voltage_v

**Table: faults** — fault_code (text), onset_timestamp, resolved_timestamp (null if still active), duration_seconds

**Table: events** — timestamp, event_type, detail (for state changes: boost on/off, mode change, manual override etc.)

Index on timestamp column in readings table for fast date range queries.

---

## 16. Firmware Notes

Platform: Arduino C++ on Elegoo Mega 2560 (ATmega2560), PlatformIO build system.

- **Watchdog timer:** enable hardware watchdog on both Megas. If main loop hangs >8s, processor resets automatically.
- **EEPROM write endurance:** write to EEPROM only after 30s timeout following last button press, not on every change. ATmega2560 EEPROM rated ~100,000 write cycles per address.
- **Manual override persistence:** NEVER store override state in EEPROM. Always revert to automatic control on power cycle.
- **Boost state and target:** NOT stored in EEPROM. Reset to Off and default target on power cycle.
- **Time sync:** W requests time sync from H on first boot and every hour thereafter. H responds with RTC time.
- **Summer startup double-start prevention:** use a `sequence active or complete` flag. Reset flag when both solar and PV drop to idle.
- **Log burner temp module hysteresis:** activate at 28°C, deactivate below 25°C to prevent valve chattering.
- **PV export sign convention:** verify Growatt Modbus register sign convention during commissioning before enabling heater control.
- **DS18B20 addressing:** each sensor has a unique 64-bit address. Scan and map all sensors to physical locations during commissioning. Store addresses as firmware constants.
- **INA219 solar pump current:** sample during ON period only, after 50ms spin-up delay. Calibrate minimum current threshold during commissioning with 20% margin below lowest normal reading.
- **Heater cycle-burst control:** SSR controlled by toggling D27 at H on/off at each zero-crossing interrupt (every 20ms). Outer control loop updates target ON-cycle ratio every 850ms from Growatt Modbus data.
- **UFH overheat hard lockout:** set a persistent flag on UFH overheat fault. Cleared only on system restart. Frost protection bypasses this flag unconditionally.
- **RS485 comms resilience:** if packet missed, continue on last known values. After 5 consecutive misses, trigger RS485 Comms Fault. Clear fault immediately when comms restore.
- **Growatt comms resilience:** if 5 consecutive Modbus polls fail, trigger Growatt Comms Fault and force heater off. Resume when comms restore.
- **Battery SOC validity:** if SOC reading is older than 10s while manual heater SOC-limited mode is active, force heater off.
- **Display backlight:** PWM-dim backlight via Mega PWM pin. Off after 1hr inactivity with no fault. Any button press wakes display. Never turn off if fault active.
- **RTC fallback:** if RTC coin cell fails but Mega stays powered, use last known RTC time + millis() elapsed. Log `RTC battery low — time approximate`. If Mega loses power and RTC has no backup time, log `RTC fault — time unknown. Awaiting sync.` and disable time-dependent functions until valid time received via RS485 from H.
- **millis() overflow:** ATmega2560 millis() counter overflows after ~49 days. Always compare elapsed time using subtraction: `if (millis() - lastTime >= interval)`.
- **Growatt heater control commissioning flag:** firmware constant `HEATER_ENABLED` (default false). Heater modulation only activates when set true. Log `Heater control disabled — commissioning flag` when false.
- **Power-up safe state sequence:** strictly sequential. Approximate total time ~60 seconds minimum before heating logic starts. Do not attempt to parallelise.
- **Fan speed control:** W controls fan speed autonomously using its own lock state combined with fan speed settings received from H via RS485.
- **Window winch state machine:** strict 3-state machine (STOP/OPEN/CLOSE). All transitions through STOP with 1s pause.
- **RS485 packet format:** structured binary packet with header byte, sequence number, payload fields, and CRC checksum. Sequence numbers allow missed packet detection. CRC allows corrupted packet detection.
- **Thermal solar cold side overheat:** fault only triggers when pump running above clocking speed for more than 10 seconds AND cold side more than 10°C above tank bottom.
- **DS18B20 validity checks:** discard any reading of exactly 85.00°C, −127°C, or −128°C, or any reading outside a physically plausible range for that sensor's location.
- **UFH cold valve and solar cold valve at W:** single direction relay (SPDT). NO = open (Wire A, +15V). NC = close (Wire B, +15V). GND permanently connected. No power relay needed — auto-cutout at end stop.
- **PC fan control:** MOSFET switches 12VDC supply at 25kHz PWM. Fan base speed stored in EEPROM (written after 30s timeout). Fan timer values NOT stored in EEPROM — reset to zero on power cycle.

### 16.1 Valve & Lock Wiring — H-Bridge

All 2-wire motorised valves and the door lock use the same H-bridge wiring with 2 SPDT relay channels:

| State | Terminal A | Terminal B | Result |
|---|---|---|---|
| Both relays OFF | GND (via NC) | GND (via NC) | Device holds position. No power drawn. |
| Relay 1 ON, Relay 2 OFF | +15VDC (via NO) | GND (via NC) | Current flows A→B. Valve opens / lock unlocks. |
| Relay 1 OFF, Relay 2 ON | GND (via NC) | +15VDC (via NO) | Current flows B→A. Valve closes / lock locks. |
| Both relays ON | +15VDC | +15VDC | No current flows — harmless but avoid. Use dead-time. |

### 16.2 Valve & Lock Pulse Timings

| Device | Location | Wiring | Control |
|---|---|---|---|
| Log burner cold valve | H | H-bridge (2 relays) | 7s pulse open or close. Both relays OFF after pulse. Holds position. |
| Bottom-of-tank valve | H | H-bridge (2 relays) | 7s pulse open or close. Both relays OFF after pulse. Holds position. |
| 2-port valve | H | H-bridge (2 relays) | 7s pulse open or close. Both relays OFF after pulse. Holds position. |
| Vacuum isolation valve | W | H-bridge (2 relays) | 7s pulse open or close. Both relays OFF after pulse. Holds position. |
| UFH cold valve | W | Single direction relay (SPDT) | NO = open (Wire A). NC = close (Wire B). GND permanent. Auto-cutout at end stop. No timed pulse. 1 relay only. |
| Solar cold valve | W | Single direction relay (SPDT) | NO = open (Wire A). NC = close (Wire B). GND permanent. Auto-cutout at end stop. No timed pulse. 1 relay only. |
| Door lock | W | H-bridge (2 relays) | 1s pulse to lock, 1s pulse to unlock. Both relays OFF after pulse. Motor-driven, no spring return. |

### 16.3 Valve & Lock Control Rules

- **Dead-time:** 200ms between de-energising one relay and energising the other. Always apply regardless of direction change.
- **Interrupt handling:** if a new command arrives before the current pulse has completed, apply 200ms dead-time then start the new command for the full pulse duration. 7 seconds from any intermediate position will always reach the end stop.
- Never interrupt a pulse for anything other than a hard safety fault (e.g. UFH overheat, thermal solar overheat).
- After pulse completes: both relays return to OFF. Valve/lock holds position with no power drawn.

### 16.4 Power-Up Safe State Sequence

On every power-up, before any heating logic runs, drive all devices to a defined safe state in this order:

1. Door lock → send 1s lock pulse. If door reed switch shows door open: log `Lock pulse sent — door open` but proceed. Record state as locked.
2. All tank valves at H (log burner cold, bottom-of-tank, 2-port) → send 7s close pulse. Wait for completion.
3. Vacuum isolation valve at W → send 7s close pulse. Wait for completion.
4. UFH cold valve and solar cold valve at W → set direction relay to close, wait 30 seconds (conservative allow for auto-cutout to trip and motor to stop), then leave relay in close position. No current feedback available.
5. Window winch → no power-up action. State determined from reed switches.
6. Begin normal heating logic only after all devices are in known state.

Rationale: valve and lock positions are unknown after a power cut. Always establishing a known state before starting control logic prevents incorrect valve configurations.

### 16.5 Lock State Tracking

- Lock state tracked in software only — no position feedback from lock mechanism
- On power-up: always send lock pulse regardless of previous state
- If door reed switch shows door open when lock pulse is due: log anomaly but send pulse and record state as locked
- Lock state NOT stored in EEPROM — power-up always triggers lock pulse

---

## 17. Growatt SPH6000TL BL-UP Modbus Register Reference

Default baud rate: 9600. Minimum command period: 850ms. Max read: 125 words.

SPH register ranges: 03 (holding) 0–124, 1000–1124. 04 (input) 0–124, 1000–1124.

### 17.1 Input Registers (Function 04) — 0–124 range (live inverter data)

| Register | Description | Scale | Notes |
|---|---|---|---|
| 0 | Inverter status | | 0=standby, 1=normal, 3=fault, 5=PV+batt online, 6=batt only |
| 3 | PV string 1 voltage (Vpv1) | ×0.1V | |
| 4 | PV string 1 current (Ipv1) | ×0.1A | |
| 5–6 | PV string 1 power (Ppv1 H/L) | ×0.1W | 32-bit value across 2 registers |
| 7 | PV string 2 voltage (Vpv2) | ×0.1V | |
| 8 | PV string 2 current (Ipv2) | ×0.1A | |
| 9–10 | PV string 2 power (Ppv2 H/L) | ×0.1W | 32-bit |
| 35–36 | Total output power (Pac H/L) | ×0.1W | 32-bit |
| 37 | Grid frequency (Fac) | ×0.01Hz | |
| 38 | Grid voltage (Vac1) | ×0.1V | |
| 53–54 | Energy today (Eac_today H/L) | ×0.1kWh | 32-bit. Note: may undercount on hybrid — use PV string registers for accurate daily total |
| 55–56 | Energy total (Eac_total H/L) | ×0.1kWh | 32-bit |

### 17.2 Input Registers (Function 04) — 1000 range (battery & hybrid data)

| Register | Description | Scale | Notes |
|---|---|---|---|
| 1000 | System work mode | | 0=waiting, 1=self-test, 3=fault, 5=PV+batt online, 6=batt only |
| 1009–1010 | Battery discharge power (H/L) | ×0.1W | 32-bit. Non-zero when battery is discharging. |
| 1011–1012 | Battery charge power (H/L) | ×0.1W | 32-bit. Non-zero when battery is charging. |
| 1013 | Battery voltage | ×0.1V | |
| 1014 | Battery SOC | % | Direct percentage. No scaling. ⚠ Verify correct SOC register during commissioning. |
| 1021–1022 | Grid import power (H/L) | ×0.1W | 32-bit. ⚠ Verify sign convention during commissioning before enabling heater. |
| 1023–1024 | Grid export power (H/L) | ×0.1W | 32-bit. ⚠ Verify sign convention during commissioning before enabling heater. |

> Battery charge and discharge are in separate registers — direction determined by which register is non-zero. Grid import and export are also in separate registers. Verify all register addresses and scaling against Protocol V1.20 during commissioning. Enable `HEATER_ENABLED` flag only after confirming export/import sign convention on your specific unit.

---

## 19. Outstanding / To Be Determined

| Item | Notes |
|---|---|
| UFH cold valve pulse / travel | Power until end stop — no timed pulse. Verify end-stop protection during commissioning. |
| Solar cold valve pulse / travel | Power until end stop — no timed pulse. Verify end-stop protection during commissioning. |
| PC fan minimum PWM duty cycle | Calibrate minimum duty cycle before stall during commissioning. Set as firmware constant. |
| UFH/solar cold valve 30s power-up wait | 30s conservative wait assumed for auto-cutout to trip. Verify during commissioning and adjust firmware constant if needed. |
| Solar pump minimum current threshold | Calibrate during commissioning with 20% margin below lowest normal reading |
| Summer mode solar pump minimum speed | Minimum speed vs heater power level and supply temperature — calibrate by on-site testing |
| Vacuum sensor polarity | Confirmed: dry contact, normally open (HIGH via pull-up = low vacuum), closes = full vacuum. Verify wiring polarity during commissioning. |
| Growatt Modbus register addresses | Verify correct register addresses against protocol document during commissioning. Verify 850ms minimum poll period. Verify export/import sign convention before enabling `HEATER_ENABLED` flag. |
| RS485 line bias resistors | Check if MAX485 boards include internal bias. Add external 560Ω resistors if not. |
| DHW pipe control | To be added at a later date |
| Auto-relock reliability | Verify all 4 conditions (door, PIR, switch, window) work correctly during installation |
| VFD for workshop fan | Deferred — BES manual speed controller selected instead. |

---

## 20. Document Changelog

| Version | Changes |
|---|---|
| 1.0 | Initial complete requirements specification |
| 2.0–4.0 | Display, alerts, solar, heating refinements |
| 5.0 | Major update: 15VDC, display, valves, fans, RS485, Growatt, all firmware notes |
| 5.1–5.7 | Iterative refinements: fault history, button redesign, valve wiring, fan control, RS485 packets, RTC fallback, pre-coding review |
| 5.8 | UFH sensors: UFH supply (at pump, 28°C pump-off) and UFH post TMV (45°C lockout); W sensors 5→6, total DS18B20 11→12; sensor fault behaviour table; solar dump forces heater off; relock now 4-condition (adds window not closed/locked); Growatt register reference appendix (section 17); window relock log entries added |
| 5.9 | Heat source selection: explicit if/else conditions; log burner + solar simultaneous confirmed valid; RS485 fallback for solar cold overheat check (stale tank bottom = no fault); night cooling explicitly 230VAC fan only; power-up sequence timing noted (~60s, strictly sequential) |
