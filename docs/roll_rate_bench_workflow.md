# Roll Rate Bench Workflow

**Language / 语言:** **English** | [简体中文](./roll_rate_bench_workflow.zh-CN.md)

## Scope

This workflow is only for single-axis roll rate-loop bring-up on a constrained bench.

It is not:

- angle outer loop
- attitude hang test
- free-flight tuning

The documented live session used a circular-rod bench with natural `+Z down`.
That detail matters for mechanical setup, but the acceptance in this workflow remains rate-only.

## Fixed Conventions

Do not change any of these:

- `+roll = right side down`
- `roll_rate = -gyro_y`
- `+roll` command expects `M1/M4` increase and `M2/M3` decrease
- `-roll` command expects `M2/M3` increase and `M1/M4` decrease

## Roll Workflow

1. Power on and connect:
   `python -m esp_drone_cli --serial COMx connect`
2. Start telemetry:
   `python -m esp_drone_cli --serial COMx stream on`
3. Move the frame by hand and verify the sign chain:
   `roll_rate = -gyro_y`
   Do not mark the roll bench final until the operator explicitly confirms this physical sign check on the constrained bench.
4. Run direct roll commands:
   `rate-test roll 30`, `rate-test roll -30`, then `rate-test roll 0`
5. Observe:
   `rate-status roll --timeout 5`
   `watch-rate all --timeout 5 --interval 0.2`
6. Run:
   `axis-bench roll --auto-arm --small-step 10 --large-step 15`
7. Inspect the saved CSV, JSON summary, and Markdown summary.
8. Only if `sign_ok` and `motor_split_ok` are both true, try a small `rate_kp_roll` change.
9. Re-run the same roll workflow after each change.

If the operator confirmation for the manual disturbance step is still missing, the roll result remains provisional and must not be treated as physical acceptance.
On live serial links, the CLI bench helper may briefly pause telemetry streaming while switching rate-test steps so command responses are not lost under telemetry load. This does not change the acceptance criteria.

## Summary Gating

The roll bench summary is the primary gate.

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

## Kp Criteria

Acceptable:

- command sign is correct in both directions
- motor split is correct
- measurable response exists
- return to zero is clean
- no obvious low-duty instability

Kp too low:

- `measurable_response` is weak
- setpoint is present, but `pid_out_roll` and actual response stay too small
- return to zero feels slow

Kp too high:

- `saturation_risk` rises
- `return_to_zero_quality` degrades
- `noise_or_jitter_risk` rises
- oscillation, fighting, or overshoot becomes visible

If anything abnormal appears:

1. stop the test
2. do not continue increasing `rate_kp_roll`

## Live Sign Check

Observed on the `2026-04-11` constrained-bench live session on `COM4`:

- the operator manually disturbed the frame around the roll axis and confirmed the physical `roll_rate = -gyro_y` sign chain before final acceptance
- `rate-status roll` reported `source_expr=-gyro_y`
- `+roll` command produced positive `pid_out_roll` and the expected `M1/M4` up, `M2/M3` down split
- `-roll` command produced negative `pid_out_roll` and the expected `M2/M3` up, `M1/M4` down split

## Live Session Result

Manual-confirmed constrained-bench rerun on `2026-04-11`:

| Candidate | sign_ok | motor_split_ok | measurable_response | saturation_risk | return_to_zero_quality | noise_or_jitter_risk | low_duty_motor_stability | Decision |
|---|---|---|---|---|---|---|---|---|
| `rate_kp_roll = 0.0026` | `True` | `True` | `True` | `False` | `PASS` | `False` | `PASS_WITH_WARNING` | keep |

Recommended roll parameters for this bench:

- `rate_kp_roll = 0.0026`
- `rate_ki_roll = 0.0`
- `rate_kd_roll = 0.0`

Reason:

- manual-confirmed `0.0026` passes sign, split, response, and return-to-zero checks
- `noise_or_jitter_risk = False` on the completed `2026-04-11` rerun
- the largest probe step already approaches the low-duty floor, so a conservative Kp is preferable

These values were written back to flash after the comparison.

## Sample Summary

- [roll_bench_summary_sample.md](./roll_bench_summary_sample.md)
- [roll_bench_summary_sample.json](./roll_bench_summary_sample.json)
