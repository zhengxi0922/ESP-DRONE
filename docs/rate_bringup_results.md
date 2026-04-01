# Rate Bring-Up Results

## Scope

This document records the single-axis rate-loop bring-up stage only.

- No free flight
- No angle mode
- No outer-loop tuning
- Props removed or the airframe fully restrained during bench tests

## Current Status

- Firmware support for single-axis `rate-test` is implemented.
- Telemetry now exposes `gyro_x/y/z`, `roll/pitch/yaw`, `rate_setpoint_*`, `rate_pid_p/i/d_*`, `pid_out_*`, `motor1..motor4`, `imu_age_us`, `loop_dt_us`, `arm_state`, `control_mode`, `failsafe_reason`.
- Host build and host-side direction tests pass.
- Hardware observations are still pending because the aircraft is not currently connected to this workstation.

## Attitude Source Used In This Stage

- The minimal rate loop uses the mapped gyro path as its primary feedback input.
- In `direct` mode, attitude/quaternion fields come from the `ATK-MS901M` output after the project mapping step.
- In `raw` mode, the future attitude source is documented in [raw_mode_attitude_plan.md](/D:/0Work/Codex/ESP-drone/docs/raw_mode_attitude_plan.md), but that estimator is not yet enabled for angle mode.
- Because this stage is rate-only, the main direction gate is still: physical motion -> mapped gyro sign -> rate PID sign -> mixer direction.

## Common Procedure

1. Connect over USB CDC and enable telemetry streaming.
2. Verify disarmed hand-motion logging before arming.
3. Arm only on the bench with props removed or the frame restrained.
4. Run one `rate-test` axis at a time.
5. Stop immediately with `kill` if any sign or motor direction is wrong.

## Per-Axis Bring-Up Table

| Axis under test | Physical action | CLI command | Main gyro field to watch | Main setpoint field | Main PID fields to watch | Expected motor direction | Actual observed result | Status |
|---|---|---|---|---|---|---|---|---|
| X-axis physical rotation -> project pitch-rate | Rotate nose up / nose down about body `+X` | `rate-test pitch 30` and `rate-test pitch -30` | `gyro_x` positive for nose-up rate, negative for nose-down rate | `rate_setpoint_pitch` | `rate_pid_p_pitch`, `rate_pid_i_pitch`, `rate_pid_d_pitch`, `pid_out_pitch` | `+pitch`: `M3/M4` above `M1/M2`; `-pitch`: `M1/M2` above `M3/M4` | Hardware pending | Pending |
| Y-axis physical rotation -> project roll-rate | Rotate right side down / left side down about body `+Y` naming convention | `rate-test roll 30` and `rate-test roll -30` | `gyro_y` negative for `+roll`, positive for `-roll` | `rate_setpoint_roll` | `rate_pid_p_roll`, `rate_pid_i_roll`, `rate_pid_d_roll`, `pid_out_roll` | `+roll`: `M1/M4` above `M2/M3`; `-roll`: `M2/M3` above `M1/M4` | Hardware pending | Pending |
| Z-axis physical rotation -> project yaw-rate | Rotate nose right / nose left in yaw | `rate-test yaw 30` and `rate-test yaw -30` | `gyro_z` negative for `+yaw`, positive for `-yaw` | `rate_setpoint_yaw` | `rate_pid_p_yaw`, `rate_pid_i_yaw`, `rate_pid_d_yaw`, `pid_out_yaw` | `+yaw`: `M1/M3` above `M2/M4`; `-yaw`: `M2/M4` above `M1/M3` | Hardware pending | Pending |

## Software-Verified Baseline

- Project gyro naming is fixed as:
  - `pitch_rate = gyro_x`
  - `roll_rate = -gyro_y`
  - `yaw_rate = -gyro_z`
- Mixer contract tests confirm:
  - `+roll` -> `M1/M4` increase
  - `+pitch` -> `M3/M4` increase
  - `+yaw` -> `M1/M3` increase
- The `flight_control_task` only updates estimator and rate PID when a fresh IMU sample arrives.

## Gate To Angle Mode

Angle mode design must remain blocked until all of the following are physically verified on hardware:

- rate-loop single-axis direction is correct
- mixer output direction is correct
- parameter validation rejects invalid mappings and unsafe limits
- the active attitude source for `raw` or `direct` mode is explicitly understood for the current test configuration
