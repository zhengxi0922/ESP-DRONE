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

## Current code facts checked during docs sync

- `CONSOLE_PROTOCOL_VERSION` in `firmware/main/console/console_protocol.h` is `0x09`.
- `CONSOLE_FEATURE_UDP_MANUAL_CONTROL` is bit `1 << 6`.
- Current UDP manual command IDs are `13..18` for enable/disable/setpoint/takeoff/land/stop.
- Current firmware default rate PID in `params.c` is `rate_kp_roll=0.0007`, `rate_kp_pitch=0.0007`, `rate_kp_yaw=0.0005`, and all rate I/D terms are `0`.
- `motor.c` uses parameterized PWM resolution via `motor_pwm_resolution_bits`. Default is `10` (LEDC_TIMER_10_BIT). Can be set to 8, 10, or 12 bits.
- IMU publish now syncs to gyro+acc frame arrival. Attitude/quaternion frames alone do not trigger publish. Stale attitude frames (>10ms older than gyro+acc) are flagged with IMU_SYNC_STALE_ATTITUDE.
- Gyro and level calibration no longer accumulate (`=` replaces `+=`). New `imu_reset_*_calibration()` functions available.
- PID D term uses D-on-Measurement (derivative of measured, not error).
- Mixer uses axis desaturation scaling before final clamp.
- Biquad filter framework available (`biquad/`) but not wired into estimator by default.

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
- Biquad/notch filter integration into estimator signal chain (framework exists but not wired).
- Kalman magnetometer fusion.
