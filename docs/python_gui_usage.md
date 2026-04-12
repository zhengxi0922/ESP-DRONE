# Python GUI Usage

**Language / 语言:** **English** | [简体中文](./python_gui_usage.zh-CN.md)

## Goal

`esp-drone-gui` is the manual bench-debug workbench for the Python toolchain.

It uses the same `esp_drone_cli.core.device_session.DeviceSession` as the CLI, so GUI and CLI share:

- framing
- serial and UDP transport
- telemetry decoding
- parameter commands
- device commands
- CSV logging

The GUI is a bench workbench, not a second protocol stack.

## Scope Warning

The new `Hang Attitude` controls are bench-only and only for a constrained circular-rod or hanging rig.

They are not a free-flight stabilize interface.
They are not an angle-flight-ready interface.
Never use them on a prop-on free-flight vehicle.

## Install

```powershell
cd tools\esp_drone_cli
pip install -e .[gui]
```

If only the CLI is needed:

```powershell
cd tools\esp_drone_cli
pip install -e .
```

## Start

```powershell
esp-drone-gui
```

or:

```powershell
python -m esp_drone_cli.gui_main
```

## VS Code Run Without Debugging

1. Open the repository root in VS Code.
2. Select the Python interpreter that should run the toolchain.
3. Install GUI dependencies:

```powershell
cd tools\esp_drone_cli
python -m pip install -e ".[gui]"
```

4. In the Run and Debug view, select `ESP-DRONE GUI (No Debug)`.
5. Press `Ctrl+F5` or choose `Run Without Debugging`.

The VS Code launch configuration starts the GUI with:

```powershell
python -m esp_drone_cli.gui_main
```

It intentionally uses the package module entry point instead of running
`esp_drone_cli/gui/main_window.py` directly, so package imports, the working
directory, and GUI environment setup stay aligned with the installed
`esp-drone-gui` script.

## Window Layout

The workbench keeps the current three-column layout plus a collapsible bottom log:

- left column:
  - connection
  - safety control
  - debug actions
- center column:
  - large realtime chart
  - live numeric telemetry table below the chart
- right column:
  - key status cards
  - parameter search, edit, save, reset, export, import
- bottom log:
  - recent command results
  - recent errors
  - last result
  - last log path

The chart and numeric telemetry stay visible together.

## Language

- default UI language: Chinese
- language switch: `中文 / English`

## Connection Rules

Bench bring-up should use USB CDC:

- `UART0` stays dedicated to `ATK-MS901M`
- `USB CDC` stays dedicated to CLI, GUI, and debug telemetry

Typical serial connection:

1. Select `Serial`.
2. Pick the COM port.
3. Keep `115200` unless the firmware changes later.
4. Click `Connect`.

## Debug Actions

The left-side `Debug Actions` area now covers both the original rate-test path and the new bench-only hang-attitude path.

Rate-test controls:

- axis selector: `roll / pitch / yaw`
- value input: target rate in dps
- `Start`: sends `rate-test`
- `Stop`: sends `rate-test <axis> 0`

Hang-attitude controls:

- `Capture Ref`
- `Attitude Test Start`
- `Attitude Test Stop`
- `Base Duty`
- quick-edit boxes for:
  - `attitude_kp_roll`
  - `attitude_kp_pitch`
  - `attitude_rate_limit_roll`
  - `attitude_rate_limit_pitch`
  - `attitude_error_deadband_deg`
  - `attitude_trip_deg`

The GUI explicitly labels this area as a constrained-rig bench path and not a free-flight mode.

Command results and command failures are pushed into the bottom log.

## Charts

The chart group selector includes:

- `Rate Roll`
- `Rate Pitch`
- `Rate Yaw`
- `Hang Attitude Roll`
- `Hang Attitude Pitch`

The hang-attitude charts are designed for the new outer-loop bring-up:

- `Hang Attitude Roll`:
  - `attitude_err_roll_deg`
  - `attitude_rate_sp_roll`
  - `pid_out_roll`
  - `motor1..motor4`
- `Hang Attitude Pitch`:
  - `attitude_err_pitch_deg`
  - `attitude_rate_sp_pitch`
  - `pid_out_pitch`
  - `motor1..motor4`

The telemetry table also includes:

- `attitude_ref_valid`
- `attitude_ref_qw`
- `attitude_ref_qx`
- `attitude_ref_qy`
- `attitude_ref_qz`
- `base_duty_active`
- existing `pid_out_*`, `imu_age_us`, and `loop_dt_us`

Use the chart chain to verify:

```text
manual disturbance -> attitude error -> outer-loop rate setpoint -> PID output -> motor mix
```

## Parameter Tuning

Use the right-side parameter area for bench tuning.

Rate-loop parameters remain available:

- `rate_kp_*`
- `rate_ki_*`
- `rate_kd_*`
- `rate_integral_limit`
- `rate_output_limit`

Hang-attitude parameters are also available:

- `attitude_kp_roll`
- `attitude_kp_pitch`
- `attitude_rate_limit_roll`
- `attitude_rate_limit_pitch`
- `attitude_error_deadband_deg`
- `attitude_trip_deg`
- `attitude_test_base_duty`
- read-only runtime flag `attitude_ref_valid`

The GUI parameter path still goes through the shared `DeviceSession`.

## Recommended GUI Bench Flow

For the bench-only hang-attitude bring-up:

1. Remove props and keep the airframe constrained on the circular rod or hanging fixture.
2. Connect over `Serial`.
3. Click `Stream On`.
4. Confirm IMU freshness and basic hand-motion telemetry.
5. Put the airframe into its natural hanging equilibrium and click `Capture Ref`.
6. Confirm `attitude_ref_valid` becomes true.
7. Arm only after the rig is confirmed safe.
8. Keep conservative `Base Duty`, `attitude_kp_*`, and `attitude_rate_limit_*`.
9. Click `Attitude Test Start`.
10. Select `Hang Attitude Roll` or `Hang Attitude Pitch`.
11. Apply small manual disturbances and verify corrective sign:
    `+roll` disturbance must drive `M2/M3` up and `M1/M4` down.
    `+pitch` disturbance must drive `M1/M2` up and `M3/M4` down.
12. Click `Attitude Test Stop` immediately if the sign chain is wrong, a trip occurs, or the rig is not stable.
13. `Disarm` when finished.

## Common Errors

Typical GUI-side failures show up clearly in the event log:

- device not connected
- arm required for `Attitude Test Start`
- reference not captured
- IMU not ready or not fresh
- trip limit exceeded
- invalid parameter value rejected by firmware

## Scope Boundary

Current GUI scope remains intentionally limited:

- suitable for constrained bench connection, telemetry, charts, rate test, hang-attitude bring-up, parameter tuning, and CSV capture
- not a second host stack
- not an automation surface
- not a free-flight stabilize, angle, or autotune interface
