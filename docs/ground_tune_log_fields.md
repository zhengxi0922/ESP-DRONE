# Ground Tune Log Fields

Each `ground-bench` run creates:

```text
logs/YYYYMMDD_HHMMSS_ground_<axis>_<mode>/
  telemetry.csv
  summary.json
  summary.md
  params_snapshot.json
  device_info.json
  events.log
```

Example:

```text
logs/20260413_211530_ground_roll_manual_perturb/
```

The one-shot recorder creates a simpler CSV:

```text
logs/YYYYMMDD_HHMMSS_ground_tune_log.csv
logs/YYYYMMDD_HHMMSS_attitude_ground_verify_log.csv
```

## Required Telemetry Columns

- `timestamp_us`
- `sample_seq`
- `imu_age_us`
- `loop_dt_us`
- `arm_state`
- `control_mode`
- `failsafe_reason`
- `battery_voltage`
- `battery_adc_raw`
- `battery_valid`
- `raw_gyro_x`, `raw_gyro_y`, `raw_gyro_z`
- `filtered_gyro_x`, `filtered_gyro_y`, `filtered_gyro_z`
- `raw_acc_x`, `raw_acc_y`, `raw_acc_z`
- `filtered_acc_x`, `filtered_acc_y`, `filtered_acc_z`
- `raw_roll_deg`, `raw_pitch_deg`, `raw_yaw_deg`
- `raw_quat_w`, `raw_quat_x`, `raw_quat_y`, `raw_quat_z`
- `kalman_roll_deg`
- `kalman_pitch_deg`
- `attitude_err_roll_deg`
- `attitude_err_pitch_deg`
- `angle_target_roll`, `angle_target_pitch`, `angle_target_yaw`
- `angle_measured_roll`, `angle_measured_pitch`, `angle_measured_yaw`
- `angle_error_roll`, `angle_error_pitch`, `angle_error_yaw`
- `outer_loop_rate_target_roll`, `outer_loop_rate_target_pitch`, `outer_loop_rate_target_yaw`
- `outer_loop_clamp_flag`
- `inner_loop_clamp_flag`
- `control_submode`
- `rate_setpoint_roll`, `rate_setpoint_pitch`, `rate_setpoint_yaw`
- `rate_meas_roll_raw`, `rate_meas_pitch_raw`, `rate_meas_yaw_raw`
- `rate_meas_roll_filtered`, `rate_meas_pitch_filtered`, `rate_meas_yaw_filtered`
- `rate_err_roll`, `rate_err_pitch`, `rate_err_yaw`
- `rate_pid_p_roll`, `rate_pid_p_pitch`, `rate_pid_p_yaw`
- `rate_pid_i_roll`, `rate_pid_i_pitch`, `rate_pid_i_yaw`
- `rate_pid_d_roll`, `rate_pid_d_pitch`, `rate_pid_d_yaw`
- `pid_out_roll`, `pid_out_pitch`, `pid_out_yaw`
- `mixer_throttle`, `mixer_roll`, `mixer_pitch`, `mixer_yaw`
- `motor1`, `motor2`, `motor3`, `motor4`
- `base_duty_active`
- `reference_valid`
- `motor_saturation_flag`
- `integrator_freeze_flag`

Additional useful columns include:

- `attitude_valid`
- `kalman_valid`
- `ground_ref_valid`
- `ground_trip_reason`

## Summary Fields

`summary.json` and `summary.md` include:

- `sign_ok`
- `motor_split_ok`
- `measurable_response`
- `oscillation_risk`
- `saturation_risk`
- `return_to_ref_quality`
- `steady_state_bias`
- `noise_or_jitter_risk`
- `safe_to_continue`
- `next_action_hint`

## Conservative Hint Rules

- Weak response: suggest a small Kp increase.
- Overshoot or jitter: suggest reducing Kp or Kd, or increasing filtering.
- Long residual bias after a clean rate loop: suggest a very small Ki increase late in tuning.
- Motor limit contact: suggest reducing `ground_test_base_duty`, `rate_output_limit`, or ground attitude rate limit.

`safe_to_continue=false` means stop tuning and inspect the CSV and event log before the next run.
