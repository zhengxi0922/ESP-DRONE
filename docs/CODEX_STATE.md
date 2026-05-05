# CODEX_STATE

This file is the short project memory for Codex. Keep it current and short. Do not turn it into a long debug log.

## Current phase

- Current focus: docs-code consistency, repeatable diagnostics, and preparing a clean path toward minimal stabilize behavior.
- The repository currently contains diagnostic and verification paths. It does not yet declare a free-flight stabilize/angle mode ready for prop-on use.
- Do not keep extending `liftoff-verify`, `short-hop`, `hang-attitude`, or `ground-tune` as if they were the final flight mode.

## Workflow memory

- User preference: do not create new branches for this repository; keep future work on `main`.

## Locked hardware and frame facts

- Body frame: `+Y` nose/forward, `+X` right side, `+Z` up.
- Naming: `+pitch` nose up, `+roll` right side down, `+yaw` nose right.
- Desired logical motor order: `M1=left-front`, `M2=right-front`, `M3=right-rear`, `M4=left-rear`.
- User-verified GUI motor-test physical result from previous bring-up: `motor1=right-front`, `motor2=right-rear`, `motor3=left-rear`, `motor4=left-front`.
- Hardware should not be rewired in the current phase. Prefer software mapping/parameters.

## Current code facts

- `CONSOLE_PROTOCOL_VERSION` in `firmware/main/console/console_protocol.h` is `0x09`.
- `CONSOLE_FEATURE_UDP_MANUAL_CONTROL` is bit `1 << 6`.
- Current UDP manual command IDs are `13..18` for enable/disable/setpoint/takeoff/land/stop.
- Current firmware default rate PID in `params.c` is `rate_kp_roll=0.0007`, `rate_kp_pitch=0.0007`, `rate_kp_yaw=0.0005`, and all rate I/D terms are `0`.
- `motor.c` uses parameterized PWM resolution via `motor_pwm_resolution_bits`. Default is `10` (LEDC_TIMER_10_BIT). Supports 8/10/12 bits.
- Coreless brushed motor defaults are `motor_pwm_freq_hz=24000`, `motor_pwm_resolution_bits=10`, `motor_idle_duty=0.03`, `motor_startup_boost_duty=0.05`, and `motor_slew_limit_per_tick=0.02`.
- PWM parameter validation allows `motor_pwm_freq_hz` in `8000..40000` only when `freq_hz * 2^motor_pwm_resolution_bits <= 80000000`.
- `motor_pwm_resolution_bits` is intended to be changed only while disarmed / on bench. Do not change during flight or while motors are actively driving.
- Recommended per-motor compensation entries are `motor_scale_m1..m4` and `motor_offset_m1..m4`. Legacy `motor_trim_scale_*` / `motor_trim_offset_*` parameter aliases are removed from the registry.
- `motor_trim_m1..m4` remains registered for schema 10 but is deprecated, defaults to `0`, and should not be used for new tuning.
- `motor_min_start_m1..m4` defaults to `0`; treat that as an initial value only and set real start thresholds from single-motor testing.
- IMU publish remains gyro+acc-triggered. Attitude/quaternion frames alone do not trigger publish.
- Attitude and quaternion staleness tracked independently by per-frame update timestamps (`last_attitude_us` / `last_quat_us`).
- Stale attitude and quaternion (>10ms older than gyro+acc) are marked invalid (`has_attitude=false`, `has_quaternion=false`) and sync_status set to IMU_SYNC_STALE_ATTITUDE / IMU_SYNC_STALE_QUATERNION. DIRECT mode rejects samples without valid attitude.
- Gyro/level calibration uses raw/uncompensated samples (`s_latest_raw_sample`), not already-compensated samples. Repeated calibration keeps the same bias/trim (not accumulating, not clearing to zero).
- Calibration reset functions available: `imu_reset_gyro_calibration()`, `imu_reset_level_calibration()`, `imu_reset_all_calibration()`.
- PID D term uses D-on-Measurement (derivative of measured, not error). `kd=0` produces same output as before.
- Mixer uses unified desaturation scaling (proportional reduction of axis_mix when any motor hits limits) before final clamp. Reports `mixer_desat_scale` and `mixer_saturated` via `mixer_get_desat_state()`.
- `estimator_get_control_attitude(source, ...)` provides unified attitude source selection. `ground_tune.c` uses it for Kalman validity checks, but full migration of all control paths is deferred.
- Biquad filter framework available (`biquad/`), with self-test (`biquad_self_test()`), but NOT wired into estimator by default. Default filter behavior unchanged.

## Current implemented paths

- Rate-loop bench path for roll/pitch/yaw.
- Hang-attitude bench outer-loop path.
- Ground tune / attitude ground verify diagnostics.
- Low-risk liftoff verify diagnostics.
- UDP manual experimental bench/manual control.
- All-motor test.
- Params, telemetry, capability, and device-info style host tooling.

## Not implemented / not claimed ready

- Finished free-flight stabilize/angle mode.
- Autonomous takeoff controller.
- Closed-loop altitude hold.
- Position hold.
- Biquad/notch filter integration into estimator signal chain (framework exists with smoke test, not wired).
- Kalman magnetometer fusion.
- Roll/pitch/yaw priority mixer (simple proportional desaturation only).
- Full migration of all control paths to `estimator_get_control_attitude()` (only ground_tune uses it currently).
- Attitude outer-loop I term.
- Rate feedforward.
