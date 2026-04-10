# Bring-Up Checklist

**Language / 语言:** **English** | [简体中文](./bringup_checklist.zh-CN.md)

This checklist must be executed between the bottom-layer framework stage and the estimator or controller stage.

## Gate Rule

No PID tuning, angle-mode work, or free-flight testing are allowed before every hardware validation item below is marked `PASS`.

## Current Stage Status

- firmware and CLI support for every bring-up item in this checklist is implemented
- host build and host-side contract tests pass
- on `2026-04-10`, this workstation detected `COM4` and completed a constrained circular-rod roll bench session with natural `+Z down`
- the validated live roll baseline used:
  - `rate_kp_roll = 0.0026`
  - `rate_ki_roll = 0.0`
  - `rate_kd_roll = 0.0`
- the same session tested `rate_kp_roll = 0.0028`; it remained sign-correct but raised `noise_or_jitter_risk`, so it was rejected
- `rate_kp_roll = 0.0026` was written back to flash after that comparison

## Connectivity

| Item | Reproducible command or evidence | Current status |
|---|---|---|
| USB CDC binary handshake | `esp-drone-cli --serial COMx connect` | Code complete, verified on `2026-04-10` with `COM4` |
| IMU UART valid frame receive | `esp-drone-cli --serial COMx stream on` and observe `imu seq/good/err/age_us` events | Code complete, verified on `2026-04-10` with `COM4` |
| LED state-machine transitions | Observe `INIT_WAIT_IMU`, `DISARMED_READY`, and `FAULT_LOCK` paths during boot and command tests | Code complete, hardware pending |

## Power And ADC

| Item | Reproducible command or evidence | Current status |
|---|---|---|
| `BAT_ADC` raw value readable | telemetry fields `battery_adc_raw` and `battery_voltage` | Code complete, hardware pending |
| Divider-based battery conversion valid | compare telemetry voltage against a DMM using the `100k / 100k` divider | Code complete, hardware pending |
| Battery thresholds visible in telemetry | inspect `battery_voltage`, `arm_state`, and `failsafe_reason` | Code complete, hardware pending |

## Motor And Safety

| Item | Reproducible command or evidence | Current status |
|---|---|---|
| `motor_test m1..m4` drives one motor only | `esp-drone-cli --serial COMx motor-test m1 0.12` and repeat for `m2..m4` | Code complete, hardware pending |
| `M1..M4` order matches the documented map | run `motor-test` in order and compare with [motor_map.md](./motor_map.md) | Code complete, hardware pending |
| `arm / disarm / kill` work through CLI | run `arm`, `disarm`, and `kill`; watch `arm_state` and `failsafe_reason` | Code complete, verified for `arm/disarm` during the `2026-04-10` roll session |
| Stick-gesture arm or disarm | deferred until RC input exists in stage 4 | Not in scope yet |

## Direction Validation

| Item | Reproducible command or evidence | Current status |
|---|---|---|
| Physical motion matches telemetry sign convention | move the frame by hand and inspect `gyro_x/y/z`, `roll/pitch/yaw` against [axis_truth_table.md](./axis_truth_table.md) | Code complete, verified for roll sign chain on `2026-04-10` |
| `+roll` raises `M1` and `M4` | `axis-test roll 0.05` while disarmed, inspect motor outputs and telemetry | Code complete, hardware pending |
| `+pitch` raises `M3` and `M4` | `axis-test pitch 0.05` while disarmed, inspect motor outputs and telemetry | Code complete, hardware pending |
| `+yaw` raises `M1` and `M3` | `axis-test yaw 0.05` while disarmed, inspect motor outputs and telemetry | Code complete, hardware pending |
| Minimal closed-loop rate response works per axis | `arm` then `rate-test roll|pitch|yaw <dps>` and inspect `rate_setpoint_*`, `pid_out_*`, and motor outputs | Code complete, verified for roll on `2026-04-10` |

## Roll Single-Axis Bench Acceptance

This section is the current active gate for roll bring-up.

| Item | Reproducible command or evidence | Current status |
|---|---|---|
| `rate_setpoint_roll` follows the commanded step | `rate-status roll` or `axis-bench roll` and inspect `setpoint_path_ok` | Verified on `2026-04-10` |
| Roll sign is correct | `sign_ok=True` and direct `rate-test roll +/-20` observations remain consistent with `roll_rate = -gyro_y` | Verified on `2026-04-10` |
| Roll motor split is correct | `motor_split_ok=True`; `+roll` increases `M1/M4` and decreases `M2/M3` | Verified on `2026-04-10` |
| Response is measurable | `measurable_response=True` | Verified on `2026-04-10` |
| Saturation is acceptable | `saturation_risk=False` | Verified on `2026-04-10` |
| Return to zero is clean | `return_to_zero_quality=PASS` | Verified on `2026-04-10` |
| Low-duty floor risk stays only at warning level | `low_duty_motor_stability=PASS_WITH_WARNING` for the largest probe step | Verified on `2026-04-10` |
| `kp` tuning gate is enforced | only continue when `kp_tuning_allowed=True`; do not continue if `sign_ok` or `motor_split_ok` fails | Verified in tooling and followed in the `2026-04-10` session |
