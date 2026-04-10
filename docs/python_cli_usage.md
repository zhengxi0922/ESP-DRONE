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

## Scope Warning

The new `attitude-test` path is bench-only and is intended only for a circular-rod, hanging, or otherwise constrained rig.

It is not a free-flight stabilize mode.
It is not a free-flight angle mode.
Do not use it on a prop-on free-flight vehicle.

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

Bench-only hang-attitude commands:

```powershell
python -m esp_drone_cli --serial COM7 attitude-capture-ref
python -m esp_drone_cli --serial COM7 arm
python -m esp_drone_cli --serial COM7 attitude-test start --base-duty 0.05
python -m esp_drone_cli --serial COM7 attitude-status --timeout 5
python -m esp_drone_cli --serial COM7 watch-attitude roll --timeout 5 --interval 0.2
python -m esp_drone_cli --serial COM7 watch-attitude pitch --timeout 5 --interval 0.2
python -m esp_drone_cli --serial COM7 watch-attitude all --timeout 5 --interval 0.2
python -m esp_drone_cli --serial COM7 attitude-test stop
python -m esp_drone_cli --serial COM7 disarm
```

`attitude-test start` is rejected if:

- the airframe is not armed
- `attitude_ref_valid` is false
- IMU health, freshness, or quaternion readiness is not acceptable

The rejection is reported explicitly instead of silently ignored.

## Telemetry Watch

Rate-focused telemetry watch:

```powershell
python -m esp_drone_cli --serial COM7 rate-status roll --timeout 5
python -m esp_drone_cli --serial COM7 rate-status pitch --timeout 5
python -m esp_drone_cli --serial COM7 rate-status yaw --timeout 5
python -m esp_drone_cli --serial COM7 watch-rate all --timeout 5 --interval 0.2
```

Hang-attitude telemetry watch:

```powershell
python -m esp_drone_cli --serial COM7 attitude-status --timeout 5
python -m esp_drone_cli --serial COM7 watch-attitude all --timeout 5 --interval 0.2
```

The hang-attitude status view prints:

- `attitude_ref_valid`
- `attitude_err_roll_deg`
- `attitude_err_pitch_deg`
- `attitude_rate_sp_roll`
- `attitude_rate_sp_pitch`
- `pid_out_roll`
- `pid_out_pitch`
- `base_duty_active`
- `motor1..motor4`
- `control_mode`
- reference quaternion `attitude_ref_qw/qx/qy/qz`

Use that chain to verify:

```text
manual disturbance -> attitude error -> outer-loop rate setpoint -> rate PID output -> motor mix
```

## Bench Automation

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

The bench summary reports:

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

## Axis Meaning

Project rate mapping is fixed:

- `pitch_rate = gyro_x`
- `roll_rate = -gyro_y`
- `yaw_rate = -gyro_z`

Positive mixer expectations are fixed:

- `+roll` -> `M1/M4` up, `M2/M3` down
- `+pitch` -> `M3/M4` up, `M1/M2` down
- `+yaw` -> `M1/M3` up, `M2/M4` down

For the bench-only hang-attitude outer loop, the required corrective sign checks are:

- disturb to `right side down` (`+roll`) -> controller must command `-roll` correction -> `M2/M3` increase, `M1/M4` decrease
- disturb to `nose up` (`+pitch`) -> controller must command `-pitch` correction -> `M1/M2` increase, `M3/M4` decrease

## Parameter Tuning

The parameter path remains shared and compatible with firmware storage:

```powershell
python -m esp_drone_cli --serial COM7 get rate_kp_roll
python -m esp_drone_cli --serial COM7 set rate_kp_roll float 0.0035
python -m esp_drone_cli --serial COM7 set attitude_kp_roll float 2.0
python -m esp_drone_cli --serial COM7 set attitude_test_base_duty float 0.05
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

Relevant hang-attitude parameters:

- `attitude_kp_roll`
- `attitude_kp_pitch`
- `attitude_rate_limit_roll`
- `attitude_rate_limit_pitch`
- `attitude_error_deadband_deg`
- `attitude_trip_deg`
- `attitude_test_base_duty`
- `attitude_ref_valid` as a runtime read-only flag

If the device rejects a write, the CLI reports the failure explicitly.

## Recommended Bench Workflow

For rate-loop bench work:

1. Remove props or fully restrain the frame.
2. Connect over USB CDC.
3. Start telemetry and verify hand-motion signs first.
4. Arm only when the bench condition is safe.
5. Run one axis at a time with `rate-test`.
6. Watch the active axis with `rate-status`.
7. Change only one PID term at a time.
8. Save parameters only after the new values are accepted.

For hang-attitude bench work on the constrained rig:

1. Keep the airframe on the circular rod or hanging fixture and verify that the natural equilibrium is the intended captured reference.
2. Start telemetry and confirm IMU freshness before capture.
3. Run `attitude-capture-ref` while the frame is in its natural hanging pose.
4. Arm only after the rig is confirmed restrained and safe.
5. Start with conservative `attitude_kp_*`, `attitude_rate_limit_*`, and `attitude_test_base_duty`.
6. Run `attitude-test start`.
7. Use `watch-attitude` to verify error sign, rate-setpoint sign, PID output, and motor split while applying small manual disturbances.
8. Stop immediately with `attitude-test stop` or `kill` if the sign chain is wrong, a trip occurs, or the IMU is not fresh.
9. Disarm after the test.

## Error Handling

Device command rejections surface as clear CLI errors, including cases such as:

- invalid argument
- arm required
- disarm required
- IMU not ready
- reference not captured
- unsupported command

The process exit code follows the firmware status code for rejected commands, which makes bench scripts easier to diagnose.
