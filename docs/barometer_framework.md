# Barometer Framework

**Language / 语言：** **English** | [简体中文](./barometer_framework.zh-CN.md)

## Current Scope

This stage only adds a barometer data path and a future altitude-hold scaffold.

Implemented now:

- decode `ATK-MS901M` barometer frames
- store barometer state in firmware
- expose barometer fields through telemetry
- display and export those fields in CLI and GUI
- reserve future altitude-hold data structures

Not implemented now:

- altitude-hold control loop
- throttle closed loop
- automatic takeoff or landing
- barometer influence on the current attitude, rate, mixer, or motor-output path

## Module Boundary

The current firmware boundary is:

- `firmware/main/barometer/barometer.h`
- `firmware/main/barometer/barometer.c`

The current API only owns:

- `barometer_init()`
- `barometer_update_from_module_frame()`
- `barometer_get_latest()`
- `barometer_get_altitude_hold_reserved_state()`

In other words, the barometer module is currently a state store, a simple vertical-speed estimator, and a telemetry outlet. It is not part of the controller yet.

## Device Telemetry Fields

The current telemetry tail now includes:

- `baro_pressure_pa`
- `baro_temperature_c`
- `baro_altitude_m`
- `baro_vspeed_mps`
- `baro_update_age_us`
- `baro_valid`
- `baro_health`

Current behavior:

- `baro_altitude_m` prefers the altitude directly reported by the module barometer frame, converted from `cm` to `m`
- `baro_vspeed_mps` is currently a simple first-difference estimate from neighboring altitude samples
- `baro_health` currently reports only `INIT`, `OK`, `STALE`, or `INVALID`

## Future Altitude-Hold Reservation

`firmware/main/esp_drone_types.h` already includes:

- `barometer_state_t`
- `altitude_hold_reserved_state_t`
- `CONTROL_MODE_HEIGHT_HOLD_RESERVED`

Those definitions do not enable a control loop by themselves.

If altitude hold is implemented later, the recommended extension points are:

1. `barometer.c`
   Refine filtering and altitude or vertical-speed estimation.
2. `estimator/`
   Add an altitude estimator that can fuse barometer data with other altitude sources.
3. `controller/`
   Add the altitude outer loop.
4. `safety/`
   Add mode-switch conditions and failure policy for altitude mode.
5. `console`, Python core, and GUI
   Add observable altitude-hold setpoint and status fields.

## CLI Access

General telemetry:

```powershell
esp-drone-cli --serial COM4 log --telemetry --timeout 5
```

Direct barometer view:

```powershell
esp-drone-cli --serial COM4 baro --timeout 5
```

or:

```powershell
esp-drone-cli --serial COM4 watch-baro --timeout 5
```

CSV export:

```powershell
esp-drone-cli --serial COM4 dump-csv telemetry.csv --duration 5
```

## GUI Surface

The GUI currently exposes barometer data in:

- the live telemetry table through `baro_*` fields
- key status cards through `baro_health` and `baro_altitude_m`
- the `Barometer` chart group
