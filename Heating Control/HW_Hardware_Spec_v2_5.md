# H & W Integrated Control System — Hardware Specification
**Version 3.0** — Two Elegoo Mega 2560 controllers

---

## 1. Architecture Overview

Two Elegoo Mega 2560 controllers (ATmega2560, 16MHz, 54 digital I/O, 16 analogue inputs). Both powered from the 15VDC battery bus. Communication over RS485 at 9600 baud via spare 1mm T&E cable. All data logging at H via SD card, with W transmitting readings to H every 250ms.

| Item | W (Workshop) | H (House) |
|---|---|---|
| Controller | Elegoo Mega 2560 | Elegoo Mega 2560 |
| Power input | 15VDC battery bus via barrel jack | 15VDC battery bus via barrel jack |
| Temp sensors | 6 × DS18B20 (1-Wire bus) | 7 × DS18B20 (1-Wire bus) |
| RS485 links | 2 × MAX485 (Growatt Modbus + H link) | 1 × MAX485 (W link) |
| 230VAC switching | 8-ch relay board (odd pins D23–D37) | SSR-40DA zero-crossing only (heater) |
| 15VDC switching | 8-ch relay board (even pins D22–D36) | 8-ch relay board (even pins D22–D36) |
| Additional relay board | 4-ch board (D41/D42/D48–D53) | Not required |
| Solar pump gate | MOSFET (IRLZ44N) D44; also driven from H via D46 wired-OR | Direct drive from D46 via 40m spare wire + 1N4148 diode |
| Display | | 3.5" ILI9488 TFT 480×320 SPI |
| SD card | | MicroSD SPI module |
| RTC | | DS3231 I2C |
| Current sensing | INA219 I2C (solar pump) | |
| Voltage sensing | | Resistor divider on ADC A0 (15V bus) |
| Grid detection | | AC zero-crossing detector on INT4 (D2) |
| Heater control | | SSR-40DA driven from D27 (PA5 hardcoded in ZC ISR) |

---

## 2. Bill of Materials

All prices approximate UK retail.

### 2.1 Controllers

| Item | Qty | Unit | Notes |
|---|---|---|---|
| Elegoo Mega 2560 R3 | 2 | ~£12 | CH340 USB chip — install CH340 driver on PC once |

### 2.2 Temperature Sensors

| Item | Qty | Unit | Notes |
|---|---|---|---|
| DS18B20 waterproof probe (1m cable) | 13 | ~£2 | 7 at H, 6 at W. 1-Wire bus, individually addressable. Full 3-wire connection only. Discard readings of 85.00°C, −127°C, −128°C. |
| 4.7kΩ resistor | 2 | Pence | One pull-up per 1-Wire bus (W bus, H bus) |

### 2.3 RS485 & Communications

| Item | Qty | Unit | Notes |
|---|---|---|---|
| MAX485 RS485 module (5V) | 3 | ~£1 | 2 × at W (Growatt Modbus + H link), 1 × at H (W link) |
| 120Ω termination resistor | 4 | Pence | One at each end of each RS485 run (2 runs = 4 resistors) |
| 560Ω resistors for line bias | 2 sets | Pence | One set per RS485 run: 560Ω pull-up on A line, 560Ω pull-down on B line. Check if MAX485 boards include internally before adding. |

### 2.4 Relay Boards & Switching

| Item | Qty | Unit | Notes |
|---|---|---|---|
| 8-channel 5V optocoupled relay board | 3 | ~£6 | W 230VAC board (odd pins), W 15VDC board (even pins), H 15VDC board (even pins). Verify boards include flyback diodes on relay coils. If not, add 1N4007 across each coil. |
| 4-channel 5V optocoupled relay board | 1 | ~£3 | W: buzzer signal to H (ch1) + hen house door open (ch2) + hen house door close (ch3) + mid-point LED (ch4) |
| SSR-40DA zero-crossing solid state relay | 1 | ~£6 | At H only — heater cycle-burst control. Zero-crossing type. 40A rating gives thermal headroom at 12.5A load. |
| Heatsink for SSR | 1 | ~£2 | Mount SSR externally on H enclosure. Required. |
| IRLZ44N logic-level N-channel MOSFET | 2 | ~£1 | At W: one for solar pump (D44) and one for PC fans (D45). Fully enhanced at 5V gate drive. |
| 120Ω gate resistor (for MOSFETs) | 2 | Pence | Series resistor on Mega PWM pin to each MOSFET gate. |
| 1N4007 flyback diode (solar pump) | 1 | Pence | Across solar pump terminals to absorb back-EMF |
| 1N4148 signal diodes (solar pump wired-OR) | 2 | Pence | One on W D44 gate line, one on H D46 gate line — wired-OR allows either controller to drive the pump; higher signal wins |

### 2.5 Display, Storage & RTC (H only)

| Item | Qty | Unit | Notes |
|---|---|---|---|
| 3.5" TFT LCD ILI9488 SPI module | 1 | ~£16 | 480×320, SPI. Verify ILI9488 driver chip on listing. |
| MicroSD SPI module | 1 | ~£2 | Shares SPI bus with display. Separate CS pin. |
| MicroSD card 16GB (FAT32) | 1 | ~£5 | Years of logging capacity at 250ms interval |
| DS3231 RTC module (I2C) | 1 | ~£2 | Essential for 5am heating trigger and time-gated security alerts. I2C address 0x57/0x68. |
| Panel-mount momentary button 12mm | 3 | ~£1 | SELECT, UP (↑), DOWN (↓) at H display. |

### 2.6 Sensing & Monitoring

| Item | Qty | Unit | Notes |
|---|---|---|---|
| INA219 current sensor module (I2C) | 1 | ~£2 | At W: solar pump current monitoring. I2C address 0x40. Wire in series with pump positive feed. |
| AC zero-crossing detector module | 1 | ~£2 | At H: wired to INT4 (D2). Fires interrupt every 20ms on grid present. Grid outage = no interrupt >50ms. Used for SSR cycle timing. |
| BES fan speed controller (ref 27219) | 1 | ~£52 | S&P HCBB/4-450/H compatible. Manual dial speed control. Wired in line with fan. Relay controls on/off. |
| 12V DC-DC buck converter module (min 1A) | 1 | ~£3 | Steps down 15VDC bus to 12VDC for PC fans. Mount inside W enclosure. |
| IRLZ44N MOSFET (PC fan speed control) | 1 | ~£1 | Already included in item 2.4 above. |
| 10kΩ resistors (fan tachometer pull-up) | 2 | Pence | One per fan tachometer wire (pin 3). Pull-up to 5V. Safe directly on Mega input pin. |

### 2.7 Voltage Dividers

| Signal | Location | Resistors | Output at 12V | Output at 15V | Notes |
|---|---|---|---|---|---|
| PIR sensor output | W — D2 | 10kΩ + 4.7kΩ | 3.50V | 4.80V | 15VDC output |
| Unlock button (1mm T&E live) | W — D10 | 10kΩ + 4.7kΩ | 3.50V | 4.80V | 15VDC bus |
| Outside light button (1mm T&E) | W — D11 | 10kΩ + 4.7kΩ | 3.50V | 4.80V | 15VDC bus |
| Log burner temp module | H — D3 | 10kΩ + 5.6kΩ | 4.37V | 5.45V | 12VDC output. Use 10k+5.6k only if bus will not exceed 13V. |
| 15V bus voltage monitor | H — A0 | 10kΩ + 4.7kΩ | 3.50V | 4.80V | Scales bus voltage to ADC range |

> Vacuum sensor (D9 at W), door reed switch, door handle reed switch, window winch reed switches, safety limit switch, manual relock switch, window buttons, boost button, and display cycle buttons are all dry contacts with Mega pull-up — no voltage divider needed.

### 2.8 Power Supply Components

| Item | Qty | Unit | Notes |
|---|---|---|---|
| 1N4007 diode (reverse polarity protection) | 2 | Pence | One per controller on 15VDC feed to barrel jack |
| 100µF electrolytic capacitor | 2 | Pence | Decoupling on controller power rail |
| Fuse holder + 1A fuse | 2 | ~£1 | Inline fuse on 15VDC feed to each controller |
| 12VDC backup PSU | 1 | TBD | At H. Switched by relay on H 15VDC board. Activates when bus drops below 12V. |

### 2.9 Enclosures & Miscellaneous

| Item | Qty | Unit | Notes |
|---|---|---|---|
| Plastic project box ~200×150×80mm | 2 | ~£6 | One per controller. Cut-outs for display, terminals, USB, ventilation slots. |
| Screw terminal blocks (3.5mm pitch) | Assorted | ~£5 | For all external wire connections into enclosures |
| DuPont jumper wires / header pins | Assorted | ~£3 | Internal connections between Mega and modules |
| Heat shrink + cable ties | Assorted | ~£3 | Cable management and insulation |
| Ferrite cores | 4 | ~£1 | Two per RS485 cable end to suppress interference from adjacent 230VAC cables |

**Estimated total BOM cost** (excluding sensors and actuators already purchased): £75–£95

---

## 3. Wiring Architecture

### 3.1 Power Supply

- Both Mega controllers powered from 15VDC battery bus via: inline 1A fuse → 1N4007 reverse-polarity diode → barrel jack (centre positive, 2.1mm)
- Alternatively: wire directly to Mega VIN and GND pins via screw terminals (same 7–20V range, no barrel jack connector to work loose)
- Mega onboard regulator provides 5V and 3.3V for all modules and sensors
- 15VDC relay contact loads (valves, lock, alarm, buzzer signal etc.) fed directly from 15VDC bus through relay contacts
- 5V relay board coils powered from Mega 5V pin
- **Never connect 15VDC directly to Mega 5V pin — use barrel jack or VIN pin only**

### 3.2 Inter-Controller RS485 Link (H ↔ W)

- Runs over spare 1mm T&E cable between H and W (40m)
- MAX485 at each end. Connect: A to A, B to B, GND to GND (use earth conductor for GND)
- 120Ω termination resistor across AB at both ends
- Line bias: 560Ω pull-up on A to 5V, 560Ω pull-down on B to GND — at one end only. Check if MAX485 boards include internally.
- Baud: 9600. Poll interval: 250ms. W→H and H→W packets exchanged every 250ms.
- MAX485 DE/RE pins tied together, driven from one Mega digital pin (HIGH = transmit, LOW = receive)

### 3.3 Growatt Modbus RS485 (at W)

- Separate MAX485 module — do not share with inter-controller link
- Connects to existing RS485 screened cable to Growatt inverter
- Baud: 9600. Poll interval: 850ms (Growatt manufacturer minimum)
- Reads: PV string 1 & 2 (V, A, kW), total PV kW, grid import/export kW, battery SOC %, battery charge/discharge kW, battery voltage, load power kW, today's PV generation kWh
- Verify register addresses and export/import sign convention during commissioning before enabling heater control

### 3.4 DS18B20 1-Wire Buses

- **W bus (D13 at W):** 6 sensors — solar hot, solar cold, UFH supply (at pump/return, 28°C pump-off check), UFH post TMV (after TMV, 45°C hard lockout), Workshop air, outside air. 4.7kΩ pull-up to 5V.
- **H bus (D13 at H):** 7 sensors — tank bottom, tank middle, tank top, hot pipe, cold pipe, heater output (primary), heater output 2 (secondary, redundant). 4.7kΩ pull-up to 5V.
- Each DS18B20 has unique 64-bit address. Scan all sensors during commissioning, map to physical locations, store as firmware constants.
- Use full 3-wire connection (VDD, GND, Data) — parasitic power mode not recommended for reliability
- 11-bit resolution configured in firmware (375ms conversion time, 0.125°C precision) — adequate for all control thresholds

### 3.5 Valve & Lock Wiring

Two wiring types are used depending on valve type:

**Type A — H-bridge** (H tank valves, vacuum isolation valve, door lock):
- Relay 1 COM → Device terminal A | Relay 2 COM → Device terminal B
- Both relay NO → +15VDC | Both relay NC → GND
- Relay 1 ON, Relay 2 OFF: terminal A = +15V, terminal B = GND → opens/unlocks
- Relay 1 OFF, Relay 2 ON: terminal A = GND, terminal B = +15V → closes/locks
- Both OFF: both terminals at GND → holds position, no power drawn
- Pulse: 7 seconds for valves, 1 second for door lock. Both relays OFF after pulse.
- Interrupt: new command mid-pulse → 200ms dead-time → new command for full pulse duration

**Type B — Direction relay (SPDT)** (W UFH cold valve, W solar cold valve):
- These valves have a permanently connected GND and two +15VDC wires (Wire A = open, Wire B = close)
- Single SPDT relay: COM to valve power wire; NO = Wire A (open); NC = Wire B (close); GND always connected
- To open: energise relay → +15V to Wire A. To close: de-energise relay → +15V to Wire B via NC
- No timed pulse — auto-cutout at mechanical end stop. Motor stalls safely.
- No power relay needed — valve draws no current once end stop reached (auto-cutout trips)

### 3.6 Solar Pump Control — Wired-OR Direct Drive

The solar pump MOSFET gate is driven by a wired-OR circuit:

- **W side (D44):** Mega PWM → 120Ω → 1N4148 diode (anode to resistor, cathode to gate junction)
- **H side (D46):** Mega digital output → 40m spare wire → 1N4148 diode (same orientation) → same gate junction
- MOSFET gate node: cathode of both diodes meet. Gate sees whichever is higher.
- Either controller can drive the pump at 100%; the other sees the gate already pulled high through the diode
- Both controllers are completely independent and failsafe — H can drive the pump directly without W's involvement

Solar pump clocking: MOSFET switching at duty-cycle rates (not high-frequency PWM). Minimum ON pulse 200ms. INA219 current sampled during ON period only, after 50ms spin-up.

### 3.7 PC Fan Control (at W)

- 2 × Fractal Design Dynamic X2 GP-14 (3-pin, 12VDC). Both fans share same PWM signal and run at same speed.
- 12V DC-DC buck converter steps down 15VDC bus to 12VDC for fan supply (pin 2 of each fan)
- Fan GND (pin 1) connected to MOSFET (IRLZ44N) drain. MOSFET source to GND.
- Mega D45 PWM → 120Ω → IRLZ44N gate. 25kHz PWM frequency for smooth 3-pin fan speed control.
- Fan tachometer (pin 3): open collector. 10kΩ pull-up to 5V on each fan separately. Safe directly on Mega pins A8 and A9 (PCINT16/17).
- Fan 1 tachometer → A8. Fan 2 tachometer → A9. Both on PCINT2 interrupt group for hardware interrupt support.
- Momentary push button at W door → D43 (pull-up enabled) — press sets 8-hour full speed timer
- Manual open/close buttons at W door → D38 (open), D39 (close) — hold-to-run for window winch

### 3.8 Electric Heater Control (at H)

- 3kW heater switched by SSR-40DA zero-crossing solid state relay. Mount SSR on heatsink externally on H enclosure.
- SSR control input wired to Mega D27 (PA5, direct port manipulation in ISR — not via digitalWrite).
- Heater control method: Bresenham spread firing. ISR increments accumulator by target% each zero-crossing; fires when accumulator ≥ 100 (wraps). Distributes half-cycles evenly — at 10% duty, one pulse every 10 half-cycles (100ms gaps), not burst firing.
- Zero-crossing detector module wired to Mega INT4 (D2) at H. Fires interrupt every 20ms when grid present.
- Grid outage detection: if >50ms without zero-crossing interrupt → grid is down. Heater forced off. Logged immediately.
- ISR-level power cap `heaterPowerCapPct` (volatile uint8_t): `accumulator += min(heaterTargetPct, heaterPowerCapPct)`. Allows emergency reduction without outer-loop latency.

> **All 230VAC wiring must be installed and verified by a competent person. Ensure correct fusing and isolation.**
> **SSR carries up to 12.5A at 3kW. Use 4mm² cable, rated fuse, and confirmed adequate heatsink.**
> **Confirm SSR purchased is zero-crossing type (SSR-40DA).**

### 3.9 Voltage Monitoring (at H)

- Resistor divider: 10kΩ top + 4.7kΩ bottom across 15VDC bus at H
- Divider midpoint to H Mega A0. At 15V: output = 4.80V. At 12V: output = 3.84V. Both within 5V ADC range.
- Calibrate during commissioning: measure actual bus voltage with multimeter, compare to ADC reading, apply correction factor in firmware if needed.

### 3.10 SPI Bus at H (display + SD card)

- ILI9488 display and MicroSD module share SPI bus (MOSI=D51, MISO=D50, SCK=D52)
- Display CS: D53. SD card CS: D48. Display DC: D49. Display RST: D47 (or tie to Mega RESET).
- Only one device active at a time via CS pins.
- Display driver: TFT_eSPI library with ILI9488_DRIVER and SPI pins configured via build_flags in platformio.ini.

### 3.11 I2C Buses

- **H:** DS3231 RTC: SDA=D20, SCL=D21. Address 0x57/0x68.
- **W:** INA219: SDA=D20, SCL=D21, address 0x40.

### 3.12 H-side Solar Pump Timing (Timer1)

H drives the pump via a 50ms Timer1 COMPA ISR (`updateHPump()`). This is independent of loop() timing, so the pump clocking continues correctly even during long SPI operations (TFT screen fills). The 5-zone clocking scheme is:

| Zone | Duty range | On time | Off time |
|---|---|---|---|
| A | 0–20% | 400ms fixed | `400×(100−d)/d` |
| B | 20–50% | `400+(d−20)×400/30` ms | `on×(100−d)/d` |
| C | 50–100% | 800ms fixed | `800×(100−d)/d` |
| — | 100% | continuous | — |

### 3.13 Enclosure Notes

- Passive ventilation slots required in both enclosures — relay boards and Mega generate heat in enclosed space
- SSR heatsink must be mounted externally on H enclosure — do not enclose inside plastic box
- Ferrite cores on RS485 cables at each end to suppress 230VAC interference
- Label all terminals clearly during installation
- All low-voltage (5V, 15VDC) wiring routed and separated from 230VAC wiring inside enclosures

---

## 4. Pin Assignments

### 4.1 W Controller — Digital Inputs

| Pin | Signal | Notes |
|---|---|---|
| D2 (INT4) | PIR sensor output | 15VDC output — 10kΩ + 4.7kΩ divider required. |
| D3 (INT5) | Door handle reed switch | Dry contact. Pull-up enabled. Closed when handle pushed. |
| D4 | Door reed switch | Dry contact. Pull-up enabled. Closes when door OPENS. |
| D5 | Winch reed: fully open | Dry contact. Pull-up enabled. |
| D6 | Winch reed: fully closed | Dry contact. Pull-up enabled. |
| D7 | Winch reed: manual lock | Dry contact. Pull-up enabled. |
| D8 | Window winch safety limit switch | Dry contact, normally open. Pull-up enabled. Closes at over-open limit. |
| D9 | Vacuum sensor | Dry contact. Pull-up enabled. HIGH (open) = low vacuum. LOW (closed) = full vacuum. |
| D10 | Unlock button signal (1mm T&E live) | 15VDC bus — 10kΩ + 4.7kΩ divider required. |
| D11 | Outside light button signal (1mm T&E) | 15VDC bus — 10kΩ + 4.7kΩ divider required. |
| D12 | Manual relock suppression switch | Dry contact. Pull-up enabled. HIGH = suppression ON. |
| D13 | DS18B20 1-Wire bus (W — 6 sensors) | 4.7kΩ pull-up to 5V. |
| A8 (PCINT16) | PC fan 1 tachometer input | Open collector. 10kΩ pull-up to 5V. 2 pulses/revolution. |
| A9 (PCINT17) | PC fan 2 tachometer input | Open collector. 10kΩ pull-up to 5V. 2 pulses/revolution. |
| D38 | Window manual OPEN button | Dry contact. Pull-up enabled. Hold-to-run. |
| D39 | Window manual CLOSE button | Dry contact. Pull-up enabled. Hold-to-run. |
| D43 | Fan full speed timer button (momentary) | Dry contact. Pull-up enabled. Press = sets 8-hour full speed timer. |

### 4.2 W Controller — Digital Outputs

| Pin | Signal | Notes |
|---|---|---|
| D22 | UFH cold valve — direction relay (SPDT) | Even board ch1. NO = Wire A (open). NC = Wire B (close). GND permanent. |
| D24 | Solar cold valve — direction relay (SPDT) | Even board ch2. NO = Wire A (open). NC = Wire B (close). GND permanent. |
| D26 | Alarm sounder (15VDC) | Even board ch3. Sounds 1 min on intrusion then auto-stops. |
| D28 | Fan flap 15VDC actuator | Even board ch4. Open before fan ON, close after fan OFF. |
| D30 | Door lock H-bridge relay A | Even board ch5. H-bridge pair with D32. |
| D32 | Door lock H-bridge relay B | Even board ch6. H-bridge pair with D30. |
| D34 | Vacuum isolation valve — CLOSE | Even board ch7. H-bridge pair with D36. |
| D36 | Vacuum isolation valve — OPEN | Even board ch8. H-bridge pair with D34. |
| D25 | 230VAC external LED lights | Odd board ch2. |
| D27 | 230VAC wall fan on/off | Odd board ch3. Fan speed set manually via BES controller. |
| D29 | Window winch — power | Odd board ch4. Energise simultaneously with D31 or D33 only. |
| D31 | Window winch — direction open | Odd board ch5. Energise simultaneously with D29 only. Never with D33. |
| D33 | Window winch — direction close | Odd board ch6. Energise simultaneously with D29 only. Never with D31. |
| D35 | 230VAC vacuum pump | Odd board ch7. |
| D37 | UFH central heating pump (230VAC) | Odd board ch8. |
| D41 | Buzzer signal to H (2.5mm T&E earth) | 4-ch board ch1. W relay pulls earth wire to trigger buzzer at H. |
| D42 | Hen house door OPEN | 4-ch board ch2. H-bridge pair with D48. 7s pulse. |
| D48 | Hen house door CLOSE | 4-ch board ch3. H-bridge pair with D42. 7s pulse. |
| D49 | Mid-point LED (15VDC via relay) | 4-ch board ch4. Steady HIGH = unlocked. 1s/1s = fault. |
| D40 | RS485 DE/RE — inter-controller link | HIGH = transmit, LOW = receive. |
| D47 | RS485 DE/RE — Growatt Modbus | HIGH = transmit, LOW = receive. |

### 4.3 W Controller — PWM & Analogue Outputs

| Pin | Signal | Notes |
|---|---|---|
| D44 | Solar pump clocking output | To IRLZ44N gate via 120Ω + 1N4148 (wired-OR with H D46). Minimum ON pulse 200ms. |
| D45 | PC fan speed (25kHz PWM) | Timer5. To separate IRLZ44N gate via 120Ω resistor. |

### 4.4 W Controller — UART / RS485

| Pin | Signal | Notes |
|---|---|---|
| TX1 (D18) | RS485 TX — inter-controller link | To MAX485 DI pin. |
| RX1 (D19) | RS485 RX — inter-controller link | From MAX485 RO pin. |
| TX2 (D16) | RS485 TX — Growatt Modbus | To MAX485 DI pin. |
| RX2 (D17) | RS485 RX — Growatt Modbus | From MAX485 RO pin. |

### 4.5 W Controller — I2C & 1-Wire

| Pin | Signal | Notes |
|---|---|---|
| SDA (D20) | INA219 current sensor SDA | Address 0x40. Solar pump current monitoring. |
| SCL (D21) | INA219 current sensor SCL | |
| D13 | DS18B20 1-Wire bus (W — 6 sensors) | 4.7kΩ pull-up to 5V. |

### 4.6 H Controller — Digital Inputs

| Pin | Signal | Notes |
|---|---|---|
| D2 (INT4) | AC zero-crossing detector | Fires every 20ms when grid present. >50ms gap = grid outage. |
| D3 (INT5) | Log burner 12VDC temp module | 12VDC output — 10kΩ + 5.6kΩ divider required. HIGH when jacket > 28°C. |
| D4 | SELECT button | Dry contact. Pull-up enabled. Panel-mount momentary. |
| D5 | UP (↑) button | Dry contact. Pull-up enabled. Panel-mount momentary. |
| D6 | DOWN (↓) button | Dry contact. Pull-up enabled. Panel-mount momentary. |
| D13 | DS18B20 1-Wire bus (H — 7 sensors) | 4.7kΩ pull-up to 5V. |
| A0 | 15VDC bus voltage monitor | Resistor divider: 10kΩ + 4.7kΩ. 15V → 4.80V. 12V → 3.84V. |

### 4.7 H Controller — Digital Outputs

| Pin | Signal | Notes |
|---|---|---|
| D22 | Log burner cold valve — OPEN | 8-ch relay board ch1. H-bridge pair with D24. |
| D24 | Log burner cold valve — CLOSE | 8-ch relay board ch2. H-bridge pair with D22. |
| D26 | Bottom-of-tank valve — OPEN | 8-ch relay board ch3. H-bridge pair with D28. |
| D27 | Heater SSR control (PA5) | Direct output to SSR-40DA. NOT through relay board. Port-manipulated in ZC ISR. |
| D28 | Bottom-of-tank valve — CLOSE | 8-ch relay board ch4. H-bridge pair with D26. |
| D30 | 2-port valve — OPEN (heater side) | 8-ch relay board ch5. H-bridge pair with D32. |
| D31 | RS485 DE/RE — inter-controller link | HIGH = transmit, LOW = receive. |
| D32 | 2-port valve — CLOSE (mid-tank side) | 8-ch relay board ch6. H-bridge pair with D30. |
| D34 | 12VDC backup PSU relay | 8-ch relay board ch7. Energise when bus < 12V. De-energise above 12.5V. |
| D36 | Spare | 8-ch relay board ch8. |
| D44 | Display backlight PWM (OC5C, Timer5) | Hardware wire moved from spec pin D32. Dims/off on inactivity. Full on fault active. |
| D46 | Solar pump direct drive | To 1N4148 diode anode (cathode to W gate junction). Drives pump via wired-OR when heater is on. |

### 4.8 H Controller — SPI Bus (display + SD)

| Pin | Signal | Notes |
|---|---|---|
| D51 (MOSI) | SPI MOSI — display + SD | Shared bus. |
| D50 (MISO) | SPI MISO — display + SD | Shared bus. |
| D52 (SCK) | SPI SCK — display + SD | Shared bus. |
| D53 | Display CS (ILI9488) | Active LOW. |
| D49 | Display DC (data/command) | |
| D47 | Display RST | Or tie to Mega RESET pin. |
| D48 | SD card CS | Active LOW. |

### 4.9 H Controller — I2C & 1-Wire

| Pin | Signal | Notes |
|---|---|---|
| SDA (D20) | DS3231 RTC SDA | Address 0x57/0x68. |
| SCL (D21) | DS3231 RTC SCL | |
| D13 | DS18B20 1-Wire bus (H — 7 sensors) | 4.7kΩ pull-up to 5V. Sensors: tank bottom, tank middle, tank top, hot pipe, cold pipe, heater output, heater output 2. |

---

## 5. Relay Board Channel Allocation

### 5.1 W — 230VAC Relay Board (8-channel, odd pins D23–D37)

| Pin | Ch | Load | Notes |
|---|---|---|---|
| D23 | 1 | Spare | |
| D25 | 2 | 230VAC external LED lights | |
| D27 | 3 | 230VAC wall fan (via BES speed controller) | On/off only. Speed set manually. |
| D29 | 4 | Window winch — power | Energise simultaneously with ch5 or ch6 only. |
| D31 | 5 | Window winch — direction open | Energise simultaneously with ch4 only. Never with ch6. |
| D33 | 6 | Window winch — direction close | Energise simultaneously with ch4 only. Never with ch5. |
| D35 | 7 | 230VAC vacuum pump | |
| D37 | 8 | UFH central heating pump (230VAC) | |

### 5.2 W — 15VDC Relay Board (8-channel, even pins D22–D36)

| Pin | Ch | Load | Notes |
|---|---|---|---|
| D22 | 1 | UFH cold valve — direction relay (SPDT) | COM to valve. NO = Wire A (open). NC = Wire B (close). GND permanent. |
| D24 | 2 | Solar cold valve — direction relay (SPDT) | COM to valve. NO = Wire A (open). NC = Wire B (close). GND permanent. |
| D26 | 3 | 15VDC alarm sounder | 1 min auto-stop on intrusion |
| D28 | 4 | Fan flap 15VDC actuator | |
| D30 | 5 | Door lock relay A | H-bridge pair with ch6. Pulse 1s to unlock/lock. |
| D32 | 6 | Door lock relay B | H-bridge pair with ch5. |
| D34 | 7 | Vacuum isolation valve — CLOSE | H-bridge pair with ch8. |
| D36 | 8 | Vacuum isolation valve — OPEN | H-bridge pair with ch7. |

### 5.3 W — 4-channel Relay Board (D41/D42/D48–D53)

| Pin | Ch | Load | Notes |
|---|---|---|---|
| D41 | 1 | Buzzer signal to H (2.5mm T&E earth) | W relay pulls earth wire to trigger buzzer at H |
| D42 | 2 | Hen house door OPEN | H-bridge pair with ch3. 7s pulse. |
| D48 | 3 | Hen house door CLOSE | H-bridge pair with ch2. 7s pulse. |
| D49 | 4 | Mid-point LED (15VDC via relay) | Steady HIGH = unlocked. 1s/1s = fault. |

### 5.4 H — 15VDC Relay Board (8-channel, even pins D22–D36)

| Pin | Ch | Load | Notes |
|---|---|---|---|
| D22 | 1 | Log burner cold valve — OPEN | H-bridge pair with ch2 |
| D24 | 2 | Log burner cold valve — CLOSE | H-bridge pair with ch1 |
| D26 | 3 | Bottom-of-tank valve — OPEN | H-bridge pair with ch4 |
| D28 | 4 | Bottom-of-tank valve — CLOSE | H-bridge pair with ch3 |
| D30 | 5 | 2-port valve — OPEN (heater side) | H-bridge pair with ch6. |
| D32 | 6 | 2-port valve — CLOSE (mid-tank side) | H-bridge pair with ch5. |
| D34 | 7 | 12VDC backup PSU relay | Energise below 12V, de-energise above 12.5V |
| D36 | 8 | Spare | |

---

## 6. Software Libraries (PlatformIO / Arduino)

| Library | Function | PlatformIO ID |
|---|---|---|
| OneWire + DallasTemperature | DS18B20 1-Wire sensors | paulstoffregen/OneWire + milesburton/DallasTemperature |
| Adafruit INA219 | Solar pump current sensing | adafruit/Adafruit INA219 |
| TFT_eSPI | ILI9488 3.5" TFT display (replaces MCUFRIEND_kbv) | bodmer/TFT_eSPI |
| Adafruit GFX | Graphics primitives for TFT | adafruit/Adafruit GFX Library |
| SD | SD card read/write | arduino-libraries/SD |
| ModbusMaster | Growatt RS485 Modbus RTU | 4-20ma/ModbusMaster |
| RTClib | DS3231 RTC — timestamps and 5am trigger | adafruit/RTClib |
| avr/wdt.h | Hardware watchdog timer | Built-in AVR library — no install needed |
| EEPROM.h | EEPROM storage for settings | Built-in Arduino library — no install needed |

ILI9488 driver, SPI pins, and display dimensions configured via `build_flags` in platformio.ini (not by editing TFT_eSPI library headers).

---

## 7. Commissioning Steps

Follow in order before running any control logic.

1. Install CH340 driver on development PC. Verify both Megas appear as COM ports in PlatformIO.
2. Upload `scan` sketch (or use built-in `scan` debug command) to each Mega. Record 64-bit address of every sensor and map to physical location. Update firmware constants. H: 7 sensors. W: 6 sensors.
3. Test each relay channel in isolation with multimeter before connecting any loads. Verify NC/NO orientation and flyback diode presence.
4. Test each valve H-bridge: energise open relay, confirm valve opens; de-energise, energise close relay, confirm closes. Verify 200ms dead-time interlock.
5. Verify RS485 inter-controller link: send test packets from W, confirm receipt at H over full 40m run. Check for framing errors.
6. Verify RS485 line bias: confirm stable idle state on A and B lines with no active driver.
7. Verify Growatt Modbus: confirm all register readings match Growatt app values. Verify export/import sign convention. Verify 850ms poll rate causes no errors.
8. Calibrate voltage divider at H: measure actual 15VDC bus voltage with multimeter, compare to ADC-calculated value. Apply correction factor in firmware.
9. Calibrate INA219: run solar pump at known duty cycles, record current draw. Set minimum-current fault threshold at 20% below lowest normal reading.
10. Test zero-crossing detector at H: confirm interrupt fires at 50Hz. Simulate grid outage (disconnect AC feed) and verify Mega detects within 50ms.
11. Test all safety fault conditions with simulated inputs before connecting heating loads.
12. Test DS3231 RTC: set time, remove power for 30 minutes, restore power and confirm time retained correctly.
13. Run full system in monitoring-only mode (no actuator outputs) for 24h. Verify SD card logging is complete and consistent.
14. Verify watchdog timer: introduce deliberate hang in test firmware, confirm Mega resets within 8 seconds.
15. Run `cal_pump` H-controller calibration sequence to characterise H-side pump duty curve. Update `calcPred()` formula parameters if required.

---

## 8. Electrical Safety Notes

> **All 230VAC wiring must be installed and verified by a competent person. Incorrect mains wiring is a fire and electrocution risk.**
> **Ensure all 230VAC relay outputs are correctly fused. Relay boards typically rated 10A per channel — verify before use.**
> **SSR-40DA carries up to 12.5A. Use 4mm² cable on heater circuit, rated fuse, and ensure heatsink is mounted externally and is adequate.**
> **Never work on 230VAC wiring with mains supply live.**
> **Confirm SSR is zero-crossing type before installation.**

- All low-voltage (5V, 15VDC) wiring neatly routed and separated from 230VAC wiring inside enclosures
- Label all terminals clearly on installation
- Ferrite cores on RS485 cables at each end to suppress interference from nearby 230VAC cables
- Verify relay boards include flyback suppression diodes. Add 1N4007 across each relay coil if not present.

---

## 9. Outstanding / To Be Determined

| Item | Notes |
|---|---|
| UFH cold / solar cold valve end-stop | Verify end-stop protection during commissioning before relying on it |
| PC fan minimum PWM duty cycle | Calibrate `FAN_MIN_DUTY_PCT` during commissioning |
| Solar pump minimum current threshold | Calibrate `SOLAR_PUMP_MIN_CURRENT_A` during commissioning |
| Fan flap travel time | Calibrate `FAN_FLAP_OPEN_MS` during commissioning |
| H-pump duty calibration curve | Run `cal_pump`, then update `calcPred()` formula in H controller |
| W solar pump duty curve | Replace temporary `calcPumpDuty()` curve after cal_pump data collected |
| RS485 bias resistors | Check if MAX485 boards include internal bias before adding external |
| 12VDC backup PSU specification | Select and specify PSU rated for full load at H |
| Relay board flyback diodes | Verify presence during commissioning before connecting loads |
| Growatt Modbus sign convention | Verify export/import sign polarity during commissioning |

---

## 10. Document Changelog

| Version | Changes |
|---|---|
| 1.0 | Initial hardware specification |
| 2.5 | W DS18B20 count 5→6; pin and relay layout first pass |
| 3.0 | Full update to match implemented code: H sensors 6→7 (htr_out_2 added); W relay boards reorganised to even/odd pin split; W 4-ch board updated (buzzer ch1, hen door ch2/ch3, mid-point LED ch4); H pin reassignments (RS485 DE/RE D30→D31, backlight D32→D44, solar pump D46 added); fan tachs moved to A8/A9 (PCINT16/17); wired-OR solar pump circuit documented; BOM: 12→13 sensors, 1N4148 diodes added; library: MCUFRIEND_kbv → TFT_eSPI; commissioning step for cal_pump added; DS18B20 resolution noted as 11-bit; Timer1 H-pump timing documented |
