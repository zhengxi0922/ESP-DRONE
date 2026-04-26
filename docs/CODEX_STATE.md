# CODEX_STATE

This file is the short project memory for Codex. Keep it current and short. Do not turn it into a long debug log.

## Current phase

- Current focus: docs-code consistency, repeatable diagnostics, and preparing a clean path toward minimal stabilize behavior.
- The repository currently contains diagnostic and verification paths. It does not yet declare a free-flight stabilize/angle mode ready for prop-on use.
- Do not keep extending `liftoff-verify`, `short-hop`, `hang-attitude`, or `ground-tune` as if they were the final flight mode.

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
- Current firmware default rate PID in `params.c` is `rate_kp_roll=0.0030`, `rate_kp_pitch=0.0030`, `rate_kp_yaw=0.0030`, and all rate I/D terms are `0`.
- The previously discussed conservative baseline `0.0007 / 0.0007 / 0.0005` is a desired tuning candidate, not the current GitHub default unless `params.c` is changed.
- `motor.c` uses fixed `LEDC_TIMER_8_BIT` PWM resolution. Only `motor_pwm_freq_hz` is parameterized.

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
- Per-motor thrust compensation with `scale/offset/min_start/deadband/gamma`.
- PWM resolution parameterization.
