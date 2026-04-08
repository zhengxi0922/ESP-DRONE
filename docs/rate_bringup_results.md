# Rate Bring-Up Results

**Language / 语言:** **English** | [简体中文](./rate_bringup_results.zh-CN.md)

## Scope

This stage is still bench-only and rate-only:

- no free flight
- no angle outer loop
- no autotune
- props removed or the frame fully restrained

The transport and architecture constraints remain locked:

- `UART0` stays dedicated to `ATK-MS901M`
- `USB CDC` stays dedicated to CLI, GUI, and debug telemetry
- CLI and GUI share the same `esp_drone_cli.core.device_session.DeviceSession`

## Locked Sign Conventions

Body frame:

- `+Y` = nose / front
- `+X` = right side
- `+Z` = up

Project attitude naming:

- `+pitch` = nose up
- `+roll` = right side down
- `+yaw` = nose right

Mapped rate definitions used by firmware, CLI, GUI, and docs:

- `pitch_rate = gyro_x`
- `roll_rate = -gyro_y`
- `yaw_rate = -gyro_z`

Expected positive mixer directions:

- `+roll` -> `M1/M4` increase, `M2/M3` decrease
- `+pitch` -> `M3/M4` increase, `M1/M2` decrease
- `+yaw` -> `M1/M3` increase, `M2/M4` decrease

See [axis_truth_table.md](./axis_truth_table.md) and [motor_map.md](./motor_map.md).

## Current Software Status

The repository now has a usable three-axis rate closed loop for bench bring-up:

- firmware closes `rate roll`, `rate pitch`, and `rate yaw` through setpoint, mapped gyro feedback, per-axis PID, mixer, motor outputs, and safety gating
- the rate controller only updates on fresh IMU samples, matching the stage-2.5 design rule
- all rate PID parameters are live in the control chain:
  - `rate_kp_roll`, `rate_ki_roll`, `rate_kd_roll`
  - `rate_kp_pitch`, `rate_ki_pitch`, `rate_kd_pitch`
  - `rate_kp_yaw`, `rate_ki_yaw`, `rate_kd_yaw`
  - `rate_integral_limit`, `rate_output_limit`
- conservative parameter validation now rejects obviously unsafe rate PID values and output-limit combinations
- USB telemetry exposes the rate-debug fields needed by both CLI and GUI:
  - `gyro_x/y/z`
  - `roll/pitch/yaw`
  - `rate_setpoint_roll/pitch/yaw`
  - `rate_pid_p/i/d_roll/pitch/yaw`
  - `pid_out_roll/pitch/yaw`
  - `motor1..motor4`
  - `imu_age_us`, `loop_dt_us`, `arm_state`, `control_mode`, `failsafe_reason`

## Control Chain Summary

Per fresh IMU sample, the active rate axis follows this path:

1. `rate-test <axis> <value>` updates the rate setpoint for one axis.
2. Firmware maps IMU gyro data into project axes:
   - pitch uses `gyro_x`
   - roll uses `-gyro_y`
   - yaw uses `-gyro_z`
3. The matching per-axis PID computes:
   - `rate_pid_p_*`
   - `rate_pid_i_*`
   - `rate_pid_d_*`
   - `pid_out_*`
4. The mixer converts the signed PID outputs into `motor1..motor4`.
5. Arm state, IMU health, and command validation gate whether the command is allowed to run.

## Per-Axis Bench Table

| Axis | Physical motion | CLI command | Main gyro field | Main setpoint | Main PID fields | Expected positive motor split |
|---|---|---|---|---|---|---|
| `roll` | right side down / left side down | `rate-test roll 20` | `gyro_y` with project feedback `-gyro_y` | `rate_setpoint_roll` | `rate_pid_p_roll`, `rate_pid_i_roll`, `rate_pid_d_roll`, `pid_out_roll` | `+roll`: `M1/M4` up, `M2/M3` down |
| `pitch` | nose up / nose down | `rate-test pitch 20` | `gyro_x` | `rate_setpoint_pitch` | `rate_pid_p_pitch`, `rate_pid_i_pitch`, `rate_pid_d_pitch`, `pid_out_pitch` | `+pitch`: `M3/M4` up, `M1/M2` down |
| `yaw` | nose right / nose left | `rate-test yaw 20` | `gyro_z` with project feedback `-gyro_z` | `rate_setpoint_yaw` | `rate_pid_p_yaw`, `rate_pid_i_yaw`, `rate_pid_d_yaw`, `pid_out_yaw` | `+yaw`: `M1/M3` up, `M2/M4` down |

## CLI Bench Procedure

1. Connect through USB CDC.
2. Start telemetry.
3. Confirm disarmed hand-motion telemetry first.
4. Arm only after the frame is safe for bench testing.
5. Test one axis at a time.
6. Use `rate-status` or `watch-rate` while testing.
7. Stop the test with `rate-test <axis> 0` or `kill` immediately if the sign is wrong.

Example session:

```powershell
python -m esp_drone_cli --serial COM7 connect
python -m esp_drone_cli --serial COM7 stream on
python -m esp_drone_cli --serial COM7 log --timeout 3 --telemetry
python -m esp_drone_cli --serial COM7 arm
python -m esp_drone_cli --serial COM7 rate-status roll --timeout 5
python -m esp_drone_cli --serial COM7 rate-test roll 20
python -m esp_drone_cli --serial COM7 rate-test roll 0
python -m esp_drone_cli --serial COM7 disarm
```

## GUI Bench Procedure

1. Connect over `Serial` in the left column.
2. Click `Stream On`.
3. Switch the center chart group to `Rate Roll`, `Rate Pitch`, or `Rate Yaw`.
4. In `Debug Actions`, pick the axis, enter the rate target in dps, then click `Start`.
5. Watch the matching gyro field, `rate_setpoint_*`, `pid_out_*`, and `motor1..motor4`.
6. Use the right-side parameter search with `rate_` to tune gains.
7. Click `Stop` for the active axis, then `Save` if the new parameters are acceptable.

## Acceptance Checklist

Software path completed in this repository:

- `rate-test roll/pitch/yaw` goes through the real control chain
- CLI and GUI both use the same `DeviceSession`
- GUI exposes axis-specific rate test controls and rate chart groups
- parameter set, save, export, and import flows remain compatible
- command rejections are surfaced explicitly instead of silently ignored

Current software verification:

- `pytest tools/esp_drone_cli/tests -q` passed with `26` tests
- `.\tools\idf.ps1 build` completed successfully for the firmware image

## Still Requires Real Hardware Verification

These checks still need an actual restrained bench session:

- verify `+roll`, `+pitch`, and `+yaw` motor direction on the airframe
- verify the mapped gyro sign while physically moving the frame
- verify real USB CDC communication on the target board
- tune `Kp/Ki/Kd` from conservative defaults on hardware

