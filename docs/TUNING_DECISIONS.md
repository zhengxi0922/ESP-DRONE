# TUNING_DECISIONS

This file stores decisions that should not be re-litigated every Codex run.

## Direction and mapping

- Keep the locked body frame: `+Y` nose, `+X` right, `+Z` up.
- Keep the naming convention: `+pitch` nose up, `+roll` right side down, `+yaw` nose right.
- Do not change axis signs or mixer signs without an explicit axis/motor-map evidence trail.

## Mode separation

- `rate-test`, `hang-attitude`, `ground-tune`, `liftoff-verify`, `short-hop`, and `all-motor-test` are diagnostics or verification paths.
- They are not the final free-flight stabilize mode.
- New flight behavior should be added as a clearly named mode rather than being hidden inside a diagnostic path.

## PID defaults vs. tuning candidates

- Documentation must distinguish current firmware defaults from experimental tuning candidates.
- Current GitHub `params.c` defaults are `rate_kp_roll=0.0030`, `rate_kp_pitch=0.0030`, `rate_kp_yaw=0.0030`, and I/D = `0`.
- The conservative candidate `roll=0.0007`, `pitch=0.0007`, `yaw=0.0005` is not a docs-code-consistent default unless firmware defaults are changed.

## Motor output compensation status

- `motor_output_map` is an implemented channel mapping feature.
- It is not the same as per-motor thrust compensation.
- Current code has global idle/max/startup-boost/slew parameters, not per-motor `scale/offset/min_start/deadband/gamma` compensation.
