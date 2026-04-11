# Roll Bench Round

- axis: `roll`
- run_id: `20260411-191740-kp0026-after-manual-stream-paused`
- started_local: `2026-04-11T19:17:40+08:00`
- serial_hint: `COM4`
- orientation: Current bench orientation is +Z down circular-rod constrained bench. This round is roll rate-loop bring-up only; not hang-attitude outer loop and not free-flight tuning. Manual roll disturbance check was confirmed by the user before this bench round.

## Params

- `rate_kp_roll` = `0.002600`
- `rate_ki_roll` = `0.000000`
- `rate_kd_roll` = `0.000000`
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
| pre_zero | 0.0 | 50 | 0.00 | -0.15 | 0.0000 | 0.0000 | 0.00 | None | None | None |
| pos_small | 10.0 | 83 | 10.00 | -1.05 | 0.0287 | 0.0543 | 0.00 | True | True | None |
| zero_after_pos_small | 0.0 | 50 | 0.00 | -0.03 | 0.0000 | 0.0000 | 0.00 | None | None | True |
| neg_small | -10.0 | 84 | -10.00 | 0.16 | -0.0265 | -0.0518 | 0.00 | True | True | None |
| zero_after_neg_small | 0.0 | 49 | 0.00 | -0.14 | 0.0000 | 0.0000 | 0.00 | None | None | True |
| pos_large | 15.0 | 81 | 15.00 | 1.48 | 0.0352 | 0.0633 | 0.00 | True | True | None |
| zero_after_pos_large | 0.0 | 50 | 0.00 | -0.78 | 0.0000 | 0.0000 | 0.00 | None | None | True |
| neg_large | -15.0 | 81 | -15.00 | -0.50 | -0.0379 | -0.0683 | 0.00 | True | True | None |
| post_zero | 0.0 | 51 | 0.00 | -0.34 | 0.0000 | 0.0000 | 0.00 | None | None | True |
