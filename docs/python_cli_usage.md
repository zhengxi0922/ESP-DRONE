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

## Scope Boundary

The workflow in this document is only for single-axis rate-loop bench work.

It does not add:

- angle outer loop
- attitude hang test
- free-flight tuning
- any change to the fixed `+roll`, `roll_rate = -gyro_y`, or motor-map conventions

The live roll workflow documented here was executed on a constrained circular-rod bench with natural `+Z down`.
That bench orientation does not change the rate-loop acceptance because the pass criteria are rate sign, PID output sign, and motor split sign.

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

Single-axis roll bench commands:

```powershell
python -m esp_drone_cli --serial COM7 rate-test roll 20
python -m esp_drone_cli --serial COM7 rate-test roll -20
python -m esp_drone_cli --serial COM7 rate-test roll 0
python -m esp_drone_cli --serial COM7 rate-status roll --timeout 5
python -m esp_drone_cli --serial COM7 watch-rate all --timeout 5 --interval 0.2
python -m esp_drone_cli --serial COM7 axis-bench roll --auto-arm --small-step 10 --large-step 15
```

## Rate-Status Output

`rate-status roll` now prints explicit roll bench fields:

- `rate_setpoint_roll`
- `roll_rate`
- `source_expr=-gyro_y`
- `raw_gyro_y`
- `rate_pid_p_roll`
- `rate_pid_i_roll`
- `rate_pid_d_roll`
- `pid_out_roll`
- `motor1..motor4`
- `arm_state`
- `control_mode`
- `imu_age_us`
- `loop_dt_us`

This is the intended chain:

```text
commanded roll rate -> mapped roll feedback -> PID p/i/d -> pid_out_roll -> motor1..motor4
```

## Axis Meaning

Project rate mapping is fixed:

- `pitch_rate = gyro_x`
- `roll_rate = -gyro_y`
- `yaw_rate = -gyro_z`

Positive motor expectations are fixed:

- `+roll` -> `M1/M4` up, `M2/M3` down
- `-roll` -> `M2/M3` up, `M1/M4` down
- `+pitch` -> `M3/M4` up, `M1/M2` down
- `+yaw` -> `M1/M3` up, `M2/M4` down

## Roll Bench Workflow

Use this order on the constrained bench:

1. Power on and connect over USB CDC.
2. Run `stream on`.
3. Move the frame by hand and verify the sign chain:
   `roll_rate = -gyro_y`.
4. Run `rate-test roll +30` and `rate-test roll -30`.
5. Observe `rate-status roll` and `watch-rate all`.
6. Run `axis-bench roll` or `rate-bench roll` to generate telemetry CSV, JSON summary, and Markdown summary.
7. Only if `sign_ok` and `motor_split_ok` are both true, consider a small `rate_kp_roll` change.
8. Re-run the same roll workflow after each change.

The dedicated workflow is documented in:

- [roll_rate_bench_workflow.md](./roll_rate_bench_workflow.md)
- [roll_bench_summary_sample.md](./roll_bench_summary_sample.md)

## Bench Automation

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
- `safe_to_continue`
- `kp_tuning_allowed`
- `axis_result`

`kp_tuning_allowed` is intentionally strict:

- if `sign_ok` is false, stop and inspect axis mapping or rate feedback first
- if `motor_split_ok` is false, stop and inspect roll motor split first
- only when `kp_tuning_allowed` is true may you continue with `rate_kp_roll` tuning

CSV capture:

```powershell
python -m esp_drone_cli --serial COM7 dump-csv telemetry.csv --duration 5
```

## Parameter Tuning Flow

Parameter reads, writes, save, export, and import remain shared with firmware storage:

```powershell
python -m esp_drone_cli --serial COM7 get rate_kp_roll
python -m esp_drone_cli --serial COM7 set rate_kp_roll float 0.0026
python -m esp_drone_cli --serial COM7 get rate_ki_roll
python -m esp_drone_cli --serial COM7 get rate_kd_roll
python -m esp_drone_cli --serial COM7 save
python -m esp_drone_cli --serial COM7 export params.json
python -m esp_drone_cli --serial COM7 import params.json --save
```

Relevant rate parameters for this stage:

- `rate_kp_roll`
- `rate_ki_roll`
- `rate_kd_roll`
- `rate_integral_limit`
- `rate_output_limit`
- `bringup_test_base_duty`

For the documented live roll session:

- `rate_kp_roll = 0.0026`
- `rate_ki_roll = 0.0`
- `rate_kd_roll = 0.0`

If the device rejects a write, the CLI reports the failure explicitly.

## Roll Kp Criteria

Treat the results like this:

- acceptable:
  - positive and negative roll commands keep the correct sign
  - motor split stays correct
  - measurable response exists
  - return-to-zero stays clean
  - no obvious low-duty instability
- `kp` too low:
  - `measurable_response` is weak
  - setpoint is present, but `pid_out_roll` and actual response stay too small
  - return to zero feels slow
- `kp` too high:
  - `saturation_risk` rises
  - `return_to_zero_quality` degrades
  - `noise_or_jitter_risk` rises
  - you see fighting, oscillation, or overshoot
- if anything abnormal appears:
  - stop the test
  - do not continue increasing `rate_kp_roll`

## Error Handling

Device command rejections surface as clear CLI errors, including cases such as:

- invalid argument
- arm required
- disarm required
- IMU not ready
- unsupported command

The process exit code follows the firmware status code for rejected commands, which makes bench scripts easier to diagnose.
