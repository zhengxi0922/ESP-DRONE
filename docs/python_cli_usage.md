# Python CLI Usage

**Language / 语言:** **English** | [简体中文](./python_cli_usage.zh-CN.md)

## Goal

`esp-drone-cli` is the scripted and bench-debug entrypoint for the Python toolchain.

It uses the same `esp_drone_cli.core.device_session.DeviceSession` as the GUI, so CLI and GUI share:

- framing
- serial and UDP transport
- telemetry decoding
- parameter commands
- device commands
- CSV logging

The project does not maintain a second host protocol stack.

## Install

CLI only:

```powershell
cd tools\esp_drone_cli
pip install -e .
```

CLI + GUI dependencies:

```powershell
cd tools\esp_drone_cli
pip install -e .[gui]
```

## Transport Rules

Bench bring-up is expected to use `USB CDC`.

- `UART0` remains reserved for `ATK-MS901M`
- CLI examples below therefore use `--serial COMx`

Generic connection examples:

```powershell
python -m esp_drone_cli --serial COM7 connect
python -m esp_drone_cli --udp 192.168.4.1:2391 connect
```

## Common Commands

Connection and stream:

```powershell
python -m esp_drone_cli --serial COM7 connect
python -m esp_drone_cli --serial COM7 stream on
python -m esp_drone_cli --serial COM7 stream off
python -m esp_drone_cli --serial COM7 log --timeout 3 --telemetry
```

Rate closed-loop bench commands:

```powershell
python -m esp_drone_cli --serial COM7 rate-test roll 20
python -m esp_drone_cli --serial COM7 rate-test pitch 20
python -m esp_drone_cli --serial COM7 rate-test yaw 20
python -m esp_drone_cli --serial COM7 rate-test roll 0
```

Rate-focused telemetry watch:

```powershell
python -m esp_drone_cli --serial COM7 rate-status roll --timeout 5
python -m esp_drone_cli --serial COM7 rate-status pitch --timeout 5
python -m esp_drone_cli --serial COM7 rate-status yaw --timeout 5
python -m esp_drone_cli --serial COM7 watch-rate all --timeout 5 --interval 0.2
```

The `rate-status` output is intended for bench tuning and prints:

- setpoint
- mapped feedback rate
- source gyro field
- PID `p/i/d`
- `pid_out`
- `motor1..motor4`
- `arm_state`, `control_mode`, `imu_age_us`, `loop_dt_us`

Single-axis bench automation:

```powershell
python -m esp_drone_cli --serial COM7 axis-bench roll --auto-arm --small-step 10 --large-step 15
python -m esp_drone_cli --serial COM7 axis-bench pitch --auto-arm --kp 0.0028 --small-step 10 --large-step 15
python -m esp_drone_cli --serial COM7 rate-bench yaw --auto-arm --kp 0.0026 --small-step 10 --large-step 15 --save-params
```

`axis-bench` and `rate-bench` are the same shared command. They use the same core logic as the GUI/session layer and save:

- telemetry CSV
- JSON summary
- Markdown summary

The bench summary now reports:

- `setpoint_path_ok`
- `sign_ok`
- `motor_split_ok`
- `measurable_response`
- `saturation_risk`
- `return_to_zero_quality`
- `noise_or_jitter_risk`
- `low_duty_motor_stability`
- `axis_result` as `PASS`, `PASS_WITH_WARNING`, or `FAIL`

CSV capture:

```powershell
python -m esp_drone_cli --serial COM7 dump-csv telemetry.csv --duration 5
```

## Rate Axis Meaning

Project rate mapping is fixed:

- `pitch_rate = gyro_x`
- `roll_rate = -gyro_y`
- `yaw_rate = -gyro_z`

Positive motor expectations are fixed:

- `+roll` -> `M1/M4` up, `M2/M3` down
- `+pitch` -> `M3/M4` up, `M1/M2` down
- `+yaw` -> `M1/M3` up, `M2/M4` down

## Parameter Tuning Flow

The parameter path remains shared and compatible with firmware storage:

```powershell
python -m esp_drone_cli --serial COM7 get rate_kp_roll
python -m esp_drone_cli --serial COM7 set rate_kp_roll float 0.0035
python -m esp_drone_cli --serial COM7 set rate_kd_pitch float 0.0002
python -m esp_drone_cli --serial COM7 save
python -m esp_drone_cli --serial COM7 export params.json
python -m esp_drone_cli --serial COM7 import params.json --save
```

Relevant rate parameters:

- `rate_kp_roll`, `rate_ki_roll`, `rate_kd_roll`
- `rate_kp_pitch`, `rate_ki_pitch`, `rate_kd_pitch`
- `rate_kp_yaw`, `rate_ki_yaw`, `rate_kd_yaw`
- `rate_integral_limit`
- `rate_output_limit`

If the device rejects a write, the CLI now reports the failure explicitly instead of silently printing a fake success.

## Recommended Bench Workflow

1. Remove props or fully restrain the frame.
2. Connect over USB CDC.
3. Start telemetry and verify hand-motion signs first.
4. Arm only when the bench condition is safe.
5. Run one axis at a time with `rate-test`.
6. Watch the active axis with `rate-status`.
7. Change only one PID term at a time.
8. Save parameters only after the new values are accepted.

## Error Handling

Device command rejections now surface as clear CLI errors, including cases such as:

- invalid argument
- arm required
- disarm required
- IMU not ready
- unsupported command

The process exit code follows the firmware status code for rejected commands, which makes bench scripts easier to diagnose.
