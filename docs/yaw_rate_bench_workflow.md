# Yaw Rate Bench Workflow

## Scope

This workflow is only for single-axis yaw rate-loop bring-up on the ground.

It is not:

- stabilize or angle mode tuning
- yaw heading hold
- hang-attitude outer loop
- free-flight tuning

The documented live session used ground placement with the airframe in `+Z up`.
This is not a hang-attitude setup and must not be described as free-flight readiness.

Serial command examples stay generic as `COMx`.
The live session captured in this document used `COM4`.

## Fixed Conventions

Do not change any of these:

- `+yaw = nose right`
- `yaw_rate = -gyro_z`
- `+yaw` command expects `M1/M3` increase and `M2/M4` decrease
- `-yaw` command expects `M2/M4` increase and `M1/M3` decrease
- ground `+Z up` placement does not change the yaw rate-loop sign convention

## Yaw Workflow

1. Power on and connect:
   `python -m esp_drone_cli --serial COMx connect`
2. Start telemetry:
   `python -m esp_drone_cli --serial COMx stream on`
3. Read current yaw P-only parameters:
   `python -m esp_drone_cli --serial COMx get rate_kp_yaw`
   `python -m esp_drone_cli --serial COMx get rate_ki_yaw`
   `python -m esp_drone_cli --serial COMx get rate_kd_yaw`
4. Stop before the physical sign check and ask the operator to manually disturb yaw.
   This step does not need motor rotation.
   The operator must gently twist the airframe nose right once (`+yaw`) and nose left once (`-yaw`) on the ground in `+Z up` placement.
   Do not mark the bench final until the operator explicitly confirms this physical `yaw_rate = -gyro_z` sign-chain check.
5. Run direct yaw commands only after manual disturbance confirmation:
   `python -m esp_drone_cli --serial COMx rate-test yaw 20`
   `python -m esp_drone_cli --serial COMx rate-test yaw -20`
   `python -m esp_drone_cli --serial COMx rate-test yaw 0`
6. Observe:
   `python -m esp_drone_cli --serial COMx rate-status yaw --timeout 5`
   `python -m esp_drone_cli --serial COMx watch-rate all --timeout 5 --interval 0.2`
7. Run:
   `python -m esp_drone_cli --serial COMx axis-bench yaw --auto-arm --small-step 10 --large-step 15`
8. Inspect the saved CSV, JSON summary, and Markdown summary.
9. Only if `sign_ok`, `motor_split_ok`, and `kp_tuning_allowed` are all true, try a small `rate_kp_yaw` change.
10. Re-run the same yaw workflow after each change.

If the operator confirmation for the manual disturbance step is still missing, the yaw result remains provisional and must not be treated as physical acceptance.

## Required Rate Status Fields

`rate-status yaw --timeout 5` must expose the yaw rate-loop path clearly enough to verify the software side:

- `rate_setpoint_yaw`
- `yaw_rate`
- `source_expr=-gyro_z`
- `raw_gyro_z`
- `rate_pid_p_yaw`
- `rate_pid_i_yaw`
- `rate_pid_d_yaw`
- `pid_out_yaw`
- `motor1..motor4`
- `arm_state`
- `control_mode`
- `imu_age_us`
- `loop_dt_us`

## Summary Gating

The yaw bench summary is the primary gate.

Do not continue Kp tuning unless all of these are satisfied:

- `setpoint_path_ok = True`
- `sign_ok = True`
- `motor_split_ok = True`
- `kp_tuning_allowed = True`

Review these items every round:

- `measurable_response`
- `saturation_risk`
- `return_to_zero_quality`
- `noise_or_jitter_risk`
- `low_duty_motor_stability`
- `axis_result`

Rules:

- if `sign_ok` is not true, stop and check sign or mapping first
- if `motor_split_ok` is not true, stop and check mixer first
- only allow Kp probing after `sign_ok=True`, `motor_split_ok=True`, and `kp_tuning_allowed=True`

## Kp Criteria

Acceptable:

- command sign is correct in both directions
- motor split is correct
- measurable response exists
- return to zero is clean
- no obvious low-duty instability

Kp too low:

- `measurable_response` is weak
- setpoint is present, but `pid_out_yaw` and actual response stay too small
- return to zero feels slow

Kp too high:

- `saturation_risk` rises
- `return_to_zero_quality` degrades
- `noise_or_jitter_risk` rises
- oscillation, fighting, or overshoot becomes visible

If anything abnormal appears:

1. stop the test
2. do not continue increasing `rate_kp_yaw`

## Live Sign Check

Observed on the `2026-04-11` ground `+Z up` live session on `COM4`:

- the operator manually twisted the frame around yaw, nose right and nose left, and confirmed the physical `yaw_rate = -gyro_z` sign chain before final acceptance
- `rate-status yaw` reported `source_expr=-gyro_z`
- `+yaw` command produced positive `pid_out_yaw` and the expected `M1/M3` up, `M2/M4` down split in the bench summary
- `-yaw` command produced negative `pid_out_yaw` and the expected `M2/M4` up, `M1/M3` down split in the bench summary

## Live Session Result

Manual-confirmed ground `+Z up` yaw bench on `2026-04-11`:

| Candidate | sign_ok | motor_split_ok | measurable_response | saturation_risk | return_to_zero_quality | noise_or_jitter_risk | low_duty_motor_stability | Decision |
|---|---|---|---|---|---|---|---|---|
| `rate_kp_yaw = 0.0026` | `True` | `True` | `True` | `False` | `PASS` | `False` | `PASS_WITH_WARNING` | keep |
| `rate_kp_yaw = 0.0024` | `True` | `True` | `True` | `False` | `PASS` | `False` | `PASS_WITH_WARNING` | compare only |

Recommended yaw parameters for this bench:

- `rate_kp_yaw = 0.0026`
- `rate_ki_yaw = 0.0`
- `rate_kd_yaw = 0.0`

Reason:

- manual-confirmed `0.0026` passes sign, split, response, and return-to-zero checks
- `noise_or_jitter_risk = False` for both `0.0026` and the lower `0.0024` candidate
- both values keep `low_duty_motor_stability = PASS_WITH_WARNING`, so the lower candidate does not remove the low-duty warning
- because the baseline already passed the gate, retaining the existing P-only baseline is the conservative choice

These values were written back to flash and saved after the comparison.

## Sample Summary

- [yaw_bench_summary_sample.md](./yaw_bench_summary_sample.md)
- [yaw_bench_summary_sample.json](./yaw_bench_summary_sample.json)
