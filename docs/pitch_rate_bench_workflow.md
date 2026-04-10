# Pitch Rate Bench Workflow

## Scope

This workflow is only for single-axis pitch rate-loop bring-up on a constrained bench.

It is not:

- angle outer loop
- hang-attitude outer loop
- free-flight tuning

The documented live session used a circular-rod constrained bench with natural `+Z down`.
That detail only affects the mechanical setup.
The acceptance in this workflow remains pitch rate-loop only.

Serial command examples stay generic as `COMx`.
The live session captured in this document used `COM4`.

## Fixed Conventions

Do not change any of these:

- `+pitch = nose up`
- `pitch_rate = gyro_x`
- `+pitch` command expects `M3/M4` increase and `M1/M2` decrease
- `-pitch` command expects `M1/M2` increase and `M3/M4` decrease
- the circular-rod bench running in natural `+Z down` does not change the pitch rate-loop sign convention

## Pitch Workflow

1. Power on and connect:
   `python -m esp_drone_cli --serial COMx connect`
2. Start telemetry:
   `python -m esp_drone_cli --serial COMx stream on`
3. Move the frame by hand and verify the sign chain:
   `pitch_rate = gyro_x`
   Do not mark the bench final until the operator confirms this physical sign check on the constrained bench.
4. Run direct pitch commands:
   `python -m esp_drone_cli --serial COMx rate-test pitch 20`
   `python -m esp_drone_cli --serial COMx rate-test pitch -20`
   `python -m esp_drone_cli --serial COMx rate-test pitch 0`
5. Observe:
   `python -m esp_drone_cli --serial COMx rate-status pitch --timeout 5`
   `python -m esp_drone_cli --serial COMx watch-rate all --timeout 5 --interval 0.2`
6. Run:
   `python -m esp_drone_cli --serial COMx axis-bench pitch --auto-arm --small-step 10 --large-step 15`
7. Inspect the saved CSV, JSON summary, and Markdown summary.
8. Only if `sign_ok`, `motor_split_ok`, and `kp_tuning_allowed` are all true, try a small `rate_kp_pitch` change.
9. Re-run the same pitch workflow after each change.

## Summary Gating

The bench summary is the primary gate.

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
- setpoint is present, but `pid_out_pitch` and actual response stay too small
- return to zero feels slow

Kp too high:

- `saturation_risk` rises
- `return_to_zero_quality` degrades
- `noise_or_jitter_risk` rises
- oscillation, fighting, or overshoot becomes visible

If anything abnormal appears:

1. stop the test
2. do not continue increasing `rate_kp_pitch`

## Live Sign Check

Observed on the `2026-04-10` constrained-bench live session on `COM4`:

- the operator manually disturbed the frame around the pitch axis and confirmed the physical `pitch_rate = gyro_x` sign chain before final acceptance
- `rate-status pitch` reported `source_expr=gyro_x`
- `+pitch` command produced positive `pid_out_pitch` and the expected `M3/M4` up, `M1/M2` down split
- `-pitch` command produced negative `pid_out_pitch` and the expected `M1/M2` up, `M3/M4` down split

## Live Session Result

Live constrained-bench comparison on `2026-04-10` after manual pitch disturbance confirmation:

| Candidate | sign_ok | motor_split_ok | measurable_response | saturation_risk | return_to_zero_quality | noise_or_jitter_risk | low_duty_motor_stability | Decision |
|---|---|---|---|---|---|---|---|---|
| `rate_kp_pitch = 0.0026` | `True` | `True` | `True` | `False` | `PASS` | `True` | `PASS_WITH_WARNING` | reject |
| `rate_kp_pitch = 0.0024` | `True` | `True` | `True` | `False` | `PASS` | `False` | `PASS_WITH_WARNING` | keep |

Recommended pitch parameters for this constrained bench session:

- `rate_kp_pitch = 0.0024`
- `rate_ki_pitch = 0.0`
- `rate_kd_pitch = 0.0`

Reason:

- after the user-confirmed manual disturbance check, both `0.0026` and `0.0024` passed sign, split, measurable response, and return-to-zero checks
- the `0.0026` bench round raised `noise_or_jitter_risk = True`
- the `0.0024` bench round kept `measurable_response = True` while clearing the jitter flag
- both rounds still showed `low_duty_motor_stability = PASS_WITH_WARNING`, so the lower Kp is the safer choice on this bench

These values were written back to flash and saved after the comparison.

## Sample Summary

- [pitch_bench_summary_sample.md](./pitch_bench_summary_sample.md)
- [pitch_bench_summary_sample.json](./pitch_bench_summary_sample.json)
