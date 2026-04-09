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

The chart and numeric telemetry stay visible together. They are not split into mutually exclusive tabs.

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

## Rate Test Controls

The left-side `Debug Actions` area now includes a usable rate-test panel for all three axes:

- axis selector: `roll / pitch / yaw`
- value input: target rate in dps
- `Start` button: sends `rate-test`
- `Stop` button: sends `rate-test <axis> 0`
- bench warning: props removed or frame restrained only

Command results and command failures are pushed into the bottom log. The GUI no longer treats rejected device commands as silent success.

## Rate-Focused Charts

The center chart group selector now includes:

- `Rate Roll`
- `Rate Pitch`
- `Rate Yaw`

Each axis-focused chart is designed for bench rate tuning:

- `Rate Roll`:
  - `gyro_y`
  - `rate_setpoint_roll`
  - `pid_out_roll`
  - `motor1..motor4`
- `Rate Pitch`:
  - `gyro_x`
  - `rate_setpoint_pitch`
  - `pid_out_pitch`
  - `motor1..motor4`
- `Rate Yaw`:
  - `gyro_z`
  - `rate_setpoint_yaw`
  - `pid_out_yaw`
  - `motor1..motor4`

The telemetry table also includes:

- `rate_pid_p_*`
- `rate_pid_i_*`
- `rate_pid_d_*`
- `pid_out_*`
- `imu_age_us`
- `loop_dt_us`

## Parameter Tuning

Use the right-side parameter area for bench tuning:

- search with `rate_`
- edit:
  - `rate_kp_*`
  - `rate_ki_*`
  - `rate_kd_*`
  - `rate_integral_limit`
  - `rate_output_limit`
- write one parameter at a time
- observe the effect immediately in telemetry and charts
- keep using:
  - `Save`
  - `Reset`
  - `Export JSON`
  - `Import JSON`

The GUI parameter path still goes through the shared `DeviceSession`.

## Recommended GUI Bench Flow

1. Remove props or fully restrain the frame.
2. Connect over `Serial`.
3. Click `Stream On`.
4. Select `Rate Roll`, `Rate Pitch`, or `Rate Yaw` in the chart group.
5. Arm only when the frame is safe for bench testing.
6. In the left rate-test panel, choose the axis and start with a conservative command.
7. Watch the matching gyro field, setpoint, `pid_out`, and motor split.
8. Search `rate_` on the right and tune gains in small steps.
9. Click `Stop` for the active axis and `Disarm` when finished.

## Common Errors

Typical GUI-side failures now show up clearly in the event log:

- device not connected
- arm required for nonzero `rate-test`
- disarm required for `motor_test` or `axis_test`
- IMU not ready
- invalid parameter value rejected by firmware

## Scope Boundary

Current GUI scope remains intentionally limited:

- suitable for bench connection, telemetry, charts, rate test, parameter tuning, and CSV capture
- not a second host stack
- not an automation surface
- not an angle-mode or autotune interface

