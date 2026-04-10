# Pitch Bench Round

- axis: `pitch`
- run_id: `20260410-205538-kp0024-after-manual`
- started_local: `2026-04-10T20:55:38+08:00`
- serial_hint: `COM4`
- orientation: Current bench orientation is +Z down circular-rod constrained bench. This round is pitch rate-loop bring-up only; not hang-attitude outer loop and not free-flight tuning. Manual pitch disturbance check was confirmed by the user before this bench round.

## Params

- `rate_kp_pitch` = `0.002400`
- `rate_ki_pitch` = `0.000000`
- `rate_kd_pitch` = `0.000000`
- `rate_integral_limit` = `100.000000`
- `rate_output_limit` = `0.200000`
- `bringup_test_base_duty` = `0.050000`
- `motor_idle_duty` = `0.020000`
- `motor_max_duty` = `0.950000`
- `telemetry_usb_hz` = `150.000000`

## Summary

- setpoint_path_ok: `True`
- sign_ok: `True`
- motor_split_ok: `True`
- measurable_response: `True`
- saturation_risk: `False`
- return_to_zero_warning: `False`
- return_to_zero_quality: `PASS`
- noise_or_jitter_risk: `False`
- low_duty_motor_stability: `PASS_WITH_WARNING`
- safe_to_continue: `True`
- kp_tuning_allowed: `True`
- accept: `True`
- axis_result: `PASS_WITH_WARNING`
- next_action_hint: warn: the largest probe step pushes one motor group close to the low-duty floor; keep gains conservative

## Steps

| step | cmd_dps | samples | sp_mean | fb_mean | pid_out_mean | split_mean | sat_ratio | sign_ok | response | zero_ok |
|---|---:|---:|---:|---:|---:|---:|---:|---|---|---|
| pre_zero | 0.0 | 51 | 0.00 | 0.06 | 0.0000 | 0.0000 | 0.00 | None | None | None |
| pos_small | 10.0 | 87 | 10.00 | -0.12 | 0.0243 | 0.0485 | 0.00 | True | True | None |
| zero_after_pos_small | 0.0 | 49 | 0.00 | -0.08 | 0.0000 | 0.0000 | 0.00 | None | None | True |
| neg_small | -10.0 | 86 | -10.00 | 0.18 | -0.0244 | -0.0481 | 0.00 | True | True | None |
| zero_after_neg_small | 0.0 | 49 | 0.00 | 0.22 | 0.0000 | 0.0000 | 0.00 | None | None | True |
| pos_large | 15.0 | 78 | 15.00 | 0.71 | 0.0343 | 0.0632 | 0.00 | True | True | None |
| zero_after_pos_large | 0.0 | 48 | 0.00 | 0.20 | 0.0000 | 0.0000 | 0.00 | None | None | True |
| neg_large | -15.0 | 80 | -15.00 | -0.92 | -0.0338 | -0.0624 | 0.00 | True | True | None |
| post_zero | 0.0 | 50 | 0.00 | 0.24 | 0.0000 | 0.0000 | 0.00 | None | None | True |
