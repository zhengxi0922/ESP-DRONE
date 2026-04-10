# Bring-Up Checklist

**Language / 语言:** **English** | [简体中文](./bringup_checklist.zh-CN.md)

This checklist must be executed between the bottom-layer framework stage and any later estimator or controller stage.

## Gate Rule

No PID expansion, no free-flight stabilize work, no angle-mode work, and no prop-on flight testing are allowed before every required hardware validation item below is marked `PASS`.

The new hang-attitude path is bench-only and does not change that gate.

## Current Stage Status

- firmware and CLI support for every rate-loop bring-up item in this checklist is implemented
- firmware, CLI, and GUI support for the bench-only hang-attitude bring-up path are implemented
- host build and host-side Python tests pass
- hardware execution status still depends on a real connected rig session

## Connectivity

| Item | Reproducible command or evidence | Current status |
|---|---|---|
| USB CDC binary handshake | `esp-drone-cli --serial COMx connect` | Code complete, hardware pending |
| IMU UART valid frame receive | `esp-drone-cli --serial COMx stream on` and observe `imu seq/good/err/age_us` events | Code complete, hardware pending |
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
| `arm / disarm / kill` work through CLI | run `arm`, `disarm`, and `kill`; watch `arm_state` and `failsafe_reason` | Code complete, hardware pending |
| Safety gating still applies in hang-attitude mode | confirm idle limits, output limits, arm gates, and failsafe still stop the mode | Code complete, hardware pending |
| Stick-gesture arm or disarm | deferred until RC input exists in stage 4 | Not in scope yet |

## Direction Validation

| Item | Reproducible command or evidence | Current status |
|---|---|---|
| Physical motion matches telemetry sign convention | move the frame by hand and inspect `gyro_x/y/z`, `roll/pitch/yaw` against [axis_truth_table.md](./axis_truth_table.md) | Code complete, hardware pending |
| `+roll` raises `M1` and `M4` in direct mixer testing | `axis-test roll 0.05` while disarmed, inspect motor outputs and telemetry | Code complete, hardware pending |
| `+pitch` raises `M3` and `M4` in direct mixer testing | `axis-test pitch 0.05` while disarmed, inspect motor outputs and telemetry | Code complete, hardware pending |
| `+yaw` raises `M1` and `M3` in direct mixer testing | `axis-test yaw 0.05` while disarmed, inspect motor outputs and telemetry | Code complete, hardware pending |
| Minimal closed-loop rate response works per axis | `arm` then `rate-test roll|pitch|yaw <dps>` and inspect `rate_setpoint_*`, `pid_out_*`, and motor outputs | Code complete, hardware pending |

## Bench-Only Hang Attitude Bring-Up

All items in this section apply only to the constrained circular-rod or hanging rig.
None of them authorize free flight.

| Item | Reproducible command or evidence | Current status |
|---|---|---|
| Reference capture works in the natural `+Z down` hanging pose | place the frame in natural equilibrium and run `attitude-capture-ref`; verify `attitude_ref_valid=1` | Code complete, hardware pending |
| Start is rejected before capture | run `attitude-test start` without capture and expect a clear `reference not captured` error | Code complete, hardware pending |
| Outer loop uses reference-relative attitude, not world-level zero Euler target | inspect [hang_attitude_bringup_plan.md](./hang_attitude_bringup_plan.md) and firmware behavior around `q_rel = q_ref^-1 * q_now` | Code complete, hardware pending |
| Hang telemetry chain is visible | run `attitude-status` or `watch-attitude all` and verify `attitude_ref_valid`, `attitude_err_*`, `attitude_rate_sp_*`, `pid_out_*`, `base_duty_active`, `motor1..motor4`, and reference quaternion | Code complete, hardware pending |
| Positive `roll` disturbance produces negative roll correction | disturb to `right side down`; expect negative corrective action and `M2/M3` increase with `M1/M4` decrease | Code complete, hardware pending |
| Positive `pitch` disturbance produces negative pitch correction | disturb to `nose up`; expect negative corrective action and `M1/M2` increase with `M3/M4` decrease | Code complete, hardware pending |
| Yaw is not in the attitude outer-loop acceptance for this stage | verify `rate_sp_yaw` remains zero-path / unchanged by attitude hold expectations | Code complete, hardware pending |
| Trip protection stops the mode | exceed `attitude_trip_deg` carefully on the constrained rig and verify the mode stops with zeroed outer-loop request | Code complete, hardware pending |
| Stop conditions clear residual outer-loop output | trigger `kill`, `attitude-test stop`, IMU stale, failsafe, or arm abnormal and confirm no residual rate setpoint remains | Code complete, hardware pending |
