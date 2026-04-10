# Pitch Bench Round

- axis: `pitch`
- run_id: `20260410-201954-kp0026`
- started_local: `2026-04-10T20:19:54+08:00`
- serial_hint: `COM4`
- orientation: Current bench orientation is +Z down circular-rod constrained bench. This round is pitch rate-loop bring-up only; not hang-attitude outer loop and not free-flight tuning.

## Params

- `rate_kp_pitch` = `0.002600`
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
| pre_zero | 0.0 | 51 | 0.00 | 0.12 | 0.0000 | 0.0000 | 0.00 | None | None | None |
| pos_small | 10.0 | 86 | 10.00 | -0.14 | 0.0264 | 0.0516 | 0.00 | True | True | None |
| zero_after_pos_small | 0.0 | 50 | 0.00 | 0.11 | 0.0000 | 0.0000 | 0.00 | None | None | True |
| neg_small | -10.0 | 84 | -10.00 | 0.15 | -0.0264 | -0.0514 | 0.00 | True | True | None |
| zero_after_neg_small | 0.0 | 51 | 0.00 | 0.03 | 0.0000 | 0.0000 | 0.00 | None | None | True |
| pos_large | 15.0 | 78 | 15.00 | -0.15 | 0.0393 | 0.0716 | 0.00 | True | True | None |
| zero_after_pos_large | 0.0 | 51 | 0.00 | -0.18 | 0.0000 | 0.0000 | 0.00 | None | None | True |
| neg_large | -15.0 | 82 | -15.00 | -0.06 | -0.0384 | -0.0689 | 0.00 | True | True | None |
| post_zero | 0.0 | 51 | 0.00 | 0.28 | 0.0000 | 0.0000 | 0.00 | None | None | True |
