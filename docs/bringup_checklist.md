# Bring-Up Checklist

## 中文摘要

- 这份清单是阶段 2 到阶段 3 之间的硬闸门。
- 在所有 bring-up 项都 `PASS` 之前，不允许进入 PID 调参、angle 模式或自由飞。
- 当前文档记录的是“代码路径是否具备”和“硬件是否完成实测”两层状态。

This checklist must be executed between the bottom-layer framework stage and the estimator/controller stage.

## Gate Rule

No PID tuning, angle mode work or free-flight testing is allowed before every hardware validation item below is marked `PASS`.

## Current Stage Status

- Firmware and CLI support for every bring-up item in this checklist is now implemented.
- Host build and host-side contract tests pass.
- On `2026-04-01`, this workstation reported `no ports found` for `python -m serial.tools.list_ports -v`, so no hardware bring-up item was executed in this session.

## Connectivity

| Item | Reproducible command or evidence | Current status |
|---|---|---|
| USB CDC binary handshake | `esp-drone-cli --serial COMx connect` | Code complete, hardware pending |
| IMU UART valid frame receive | `esp-drone-cli --serial COMx stream on` and observe `imu seq/good/err/age_us` events | Code complete, hardware pending |
| LED state machine transitions | Observe `INIT_WAIT_IMU`, `DISARMED_READY`, `FAULT_LOCK` paths during boot and command tests | Code complete, hardware pending |

## Power And ADC

| Item | Reproducible command or evidence | Current status |
|---|---|---|
| `BAT_ADC` raw value readable | telemetry fields `battery_adc_raw` and `battery_voltage` | Code complete, hardware pending |
| Divider-based battery conversion valid | compare telemetry voltage against DMM using `100k / 100k` divider | Code complete, hardware pending |
| Battery thresholds visible in telemetry | inspect `battery_voltage`, `arm_state`, `failsafe_reason` | Code complete, hardware pending |

## Motor And Safety

| Item | Reproducible command or evidence | Current status |
|---|---|---|
| `motor_test m1..m4` drives one motor only | `esp-drone-cli --serial COMx motor-test m1 0.12` and repeat for `m2..m4` | Code complete, hardware pending |
| `M1..M4` order matches the documented map | run `motor-test` in order and compare with [motor_map.md](./motor_map.md) | Code complete, hardware not executed in this session |
| `arm / disarm / kill` work through CLI | `arm`, `disarm`, `kill` and watch `arm_state/failsafe_reason` | Code complete, hardware pending |
| Stick-gesture arm/disarm | deferred until RC input exists in stage 4 | Not in scope yet |

## Direction Validation

| Item | Reproducible command or evidence | Current status |
|---|---|---|
| Physical motion matches telemetry sign convention | move frame by hand and inspect `gyro_x/y/z`, `roll/pitch/yaw` against [axis_truth_table.md](./axis_truth_table.md) | Code complete, hardware not executed in this session |
| `+roll` raises `M1` and `M4` | `axis-test roll 0.05` disarmed, inspect motor outputs and telemetry | Code complete, hardware not executed in this session |
| `+pitch` raises `M3` and `M4` | `axis-test pitch 0.05` disarmed, inspect motor outputs and telemetry | Code complete, hardware not executed in this session |
| `+yaw` raises `M1` and `M3` | `axis-test yaw 0.05` disarmed, inspect motor outputs and telemetry | Code complete, hardware not executed in this session |
| Minimal closed-loop rate response works per axis | `arm` then `rate-test roll|pitch|yaw <dps>` and inspect `rate_setpoint_*`, `pid_out_*`, motor outputs | Code complete, hardware not executed in this session |
