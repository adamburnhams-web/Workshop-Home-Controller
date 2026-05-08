# Heating Control System — Claude Code Context

## Permissions
Allow without asking: `cd`, `dir`, `type`, `git status`, `git log`, `git diff`, `platformio run`, `pio`

## Project overview
Home heating controller split across two Arduino Mega 2560s communicating over RS485.

- **H controller** (house side): hot water tank, immersion heater, TFT display, RTC, SD card, 6× DS18B20 sensors
- **W controller** (workshop side): solar thermal pump, UFH pump, PC fans, door lock, window winch, vacuum system, Growatt inverter Modbus, 6× DS18B20 sensors

Source lives in `Heating Control/` as a PlatformIO project.

## PlatformIO environments
| Environment | Target | Notes |
|---|---|---|
| `controller_h` | H controller | MCUFRIEND TFT, RTClib, SD, Adafruit GFX |
| `controller_w` | W controller | INA219, ModbusMaster, OneWire, DallasTemperature |

Build commands (run from `Heating Control/`):
```
platformio run -e controller_h
platformio run -e controller_w
platformio run -e controller_h --target upload
platformio run -e controller_w --target upload
```

## Key files
| File | Purpose |
|---|---|
| `Heating Control/src/h_controller/main.cpp` | H controller — ~2000 lines |
| `Heating Control/src/w_controller/main.cpp` | W controller — ~2100 lines |
| `Heating Control/include/shared_types.h` | Shared enums, packet structs, fault flags |
| `Heating Control/include/rs485_packet.h` | RS485 framing, CRC-16/Modbus, encode/decode |
| `Heating Control/platformio.ini` | Build environments |

## RS485 inter-controller link
- **W → H**: `WToHPacket` (sensors, Growatt data, valve/security state, fan state)
- **H → W**: `HToWPacket` (system mode, heater state, fan settings, time sync, overrides)
- 250ms poll interval, 9600 baud on Serial1, CRC-16/Modbus framing
- Frame: `[0xAA][0x55][DIR][SEQ][LEN_LO][LEN_HI][PAYLOAD][CRC_LO][CRC_HI]`

## Temperature encoding
- `int16_t` = °C × 10 (e.g. 18.5°C → 185)
- `TEMP_FAULT = INT16_MIN` — sensor fault sentinel

## Debug serial interface
Both controllers have `#define DEBUG_SERIAL` at the top. Remove to strip all debug code from production builds. Serial at 115200 baud on USB (Serial).

### H controller commands
`temps` `valves` `faults` `mode` `status` `heater` `bus` `rtc` `page <1-5>` `scan` `set <sensor> <val>`

Set sensors: `tank_bot` `tank_mid` `tank_top` `hot_pipe` `cold_pipe` `htr_out` `log_burner` `pv_export` `batt_soc`
Use `val=999` to clear a sim override.

### W controller commands
`temps` `valves` `faults` `mode` `status` `pump` `fans` `security` `growatt` `scan` `set <sensor> <val>`

Set sensors: `solar_hot` `solar_cold` `ufh_supply` `ufh_post_tmv` `workshop_air` `outside_air`
Set binary: `pir` `door` `winch_cls` `winch_lock` (value `0` or `1`; append `_clear` to key to remove sim)

### `scan` command (both controllers)
Scans the 1-Wire bus and prints all DS18B20 addresses in array-literal format, ready to paste into `DS18B20_ADDRS`. Sensors must be identified one at a time (warm each physically while watching `temps`).

## DS18B20 sensor addresses
All addresses are currently **TODO placeholders** — must be filled in after running `scan` on each controller and physically identifying sensors.

- H controller: [src/h_controller/main.cpp ~line 60](Heating Control/src/h_controller/main.cpp) — `tank_bot`, `tank_mid`, `tank_top`, `hot_pipe`, `cold_pipe`, `htr_out`
- W controller: [src/w_controller/main.cpp ~line 82](Heating Control/src/w_controller/main.cpp) — `solar_hot`, `solar_cold`, `ufh_supply`, `ufh_post_tmv`, `workshop_air`, `outside_air`

## Commissioning TODOs
- [ ] Run `scan` on W controller, identify and fill in 6 DS18B20 addresses
- [ ] Run `scan` on H controller, identify and fill in 6 DS18B20 addresses
- [ ] Calibrate `SOLAR_PUMP_MIN_CURRENT_A` (INA219, W controller)
- [ ] Calibrate `FAN_MIN_DUTY_PCT` (W controller)
- [ ] Calibrate `FAN_FLAP_OPEN_MS` (motorized damper travel time, W controller)
- [ ] Verify `VALVE_POWERUP_WAIT_MS` (30s conservative, W controller)
- [ ] Install MAX485 hardware and test RS485 inter-controller link
- [ ] Test Growatt Modbus on Serial2 (W controller, separate MAX485)

## Code conventions
- No `String` class anywhere — char arrays and `F()` macros only
- All loop operations must be non-blocking — WDT is 8s on both controllers
- Simulation overrides: `simXxxActive` + `simXxxVal` pairs, applied after sensor reads
- `#ifdef DEBUG_SERIAL` guards wrap all debug code
- No comments explaining what code does — only non-obvious WHY comments
