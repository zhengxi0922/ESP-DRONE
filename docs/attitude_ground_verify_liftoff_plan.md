# Attitude Ground Verify And Low-Risk Liftoff Plan

This plan starts from the confirmed flat-ground, +Z-up rate P-only baseline:

- `rate_kp_roll = 0.0007`
- `rate_kp_pitch = 0.0007`
- `rate_kp_yaw = 0.0005`
- `rate_ki_* = 0`
- `rate_kd_* = 0`

Do not add rate I or D until logs show a specific need and the saturation, noise, and windup risk are understood.

## Unified Control Chain

All current flat-ground verification paths use the same controlled chain:

```text
angle target -> angle error -> ground attitude outer loop -> rate target
-> tuned rate PID -> mixer -> motor outputs
```

The firmware keeps the legacy constrained-rig hang-attitude path separate in
`CONTROL_MODE_ATTITUDE_HANG_TEST`, but the flat-ground paths below do not use it.

- `CONTROL_MODE_ATTITUDE_GROUND_TUNE` + `GROUND_TUNE_SUBMODE_RATE_ONLY`: rate-only ground tune.
- `CONTROL_MODE_ATTITUDE_GROUND_TUNE` + `GROUND_TUNE_SUBMODE_ATTITUDE_VERIFY`: flat-ground attitude outer-loop verification.
- `CONTROL_MODE_ATTITUDE_GROUND_TUNE` + `GROUND_TUNE_SUBMODE_LOW_RISK_LIFTOFF`: short conservative liftoff verification preparation.
- `CONTROL_MODE_UDP_MANUAL` + `GROUND_TUNE_SUBMODE_UDP_MANUAL`: UDP/manual throttle with the same flat-ground roll/pitch outer loop and the same tuned rate PID.

## Attitude Ground Verify

This mode verifies signs, clamps, telemetry, and protection only. It is not an
angle PID tuning mode and it is not proof of hover stability.

Default protections:

- Requires armed state, fresh IMU, valid ground reference, and valid Kalman when `ground_tune_use_kalman_attitude=true`.
- Roll/pitch targets are clamped by `ground_att_target_limit_deg`, default `2 deg`.
- Yaw angle target is forced to zero; yaw remains rate-limited and conservative.
- Outer-loop output is clamped by `ground_att_rate_limit_roll/pitch`.
- Motor output is clamped around the ramped base duty by `ground_test_max_extra_duty`.
- Watchdog, battery critical, IMU stale, Kalman invalid, saturation, and angle trip paths stop the motors.

CLI:

```powershell
python -m esp_drone_cli --serial COM4 ground-capture-ref
python -m esp_drone_cli --serial COM4 attitude-ground-verify start --base-duty 0.08
python -m esp_drone_cli --serial COM4 attitude-ground-verify target roll 1.0
python -m esp_drone_cli --serial COM4 attitude-ground-verify target pitch -1.0
python -m esp_drone_cli --serial COM4 attitude-ground-log --duration 10
python -m esp_drone_cli --serial COM4 attitude-ground-verify stop
```

One-command very small verification round:

```powershell
python -m esp_drone_cli --serial COM4 attitude-ground-round --target-deg 1.0 --base-duty 0.08 --auto-arm
```

GUI:

- `Capture Ground Ref`
- `Att Verify Start`
- `Att Verify Stop`
- `Att Verify Log 10s`

The log file is named like:

```text
logs/YYYYMMDD_HHMMSS_attitude_ground_verify_log.csv
```

## Telemetry Added For Verification

Protocol V5 telemetry appends:

- `angle_target_roll`, `angle_target_pitch`, `angle_target_yaw`
- `angle_measured_roll`, `angle_measured_pitch`, `angle_measured_yaw`
- `angle_error_roll`, `angle_error_pitch`, `angle_error_yaw`
- `outer_loop_rate_target_roll`, `outer_loop_rate_target_pitch`, `outer_loop_rate_target_yaw`
- `outer_loop_clamp_flag`
- `inner_loop_clamp_flag`
- `control_submode`

These are recorded together with raw/filtered IMU, Kalman validity, rate target,
rate measured, rate error, PID P/I/D terms, mixer values, motor outputs, and battery
fields.

## Stop Point For Ground Work

Stop ground-only attitude work once logs show:

- Small roll/pitch commands generate rate targets in the expected direction.
- Small manual disturbances produce corrective rate targets.
- No new estimator invalid, sensor timeout, battery abnormal, clamp storm, saturation trip, or failsafe occurs.

Do not claim angle PID is tuned from this state. Ground contact changes the dynamics
and can hide or create coupling that will not match free flight.

## Low-Risk Liftoff Verify Preparation

The liftoff verification mode is only a prepared, bounded test entry. It is not an
autonomous takeoff controller.

Protections:

- Uses the same ground reference outer loop and tuned rate PID.
- `liftoff_verify_base_duty`, default `0.10`.
- `liftoff_verify_max_extra_duty`, default `0.04`.
- `liftoff_verify_ramp_duty_per_s`, default `0.10`.
- `liftoff_verify_auto_disarm_ms`, default `2500`.
- `liftoff_verify_att_trip_deg`, default `8`.
- Invalid estimator, stale IMU, battery critical, saturation trip, or watchdog timeout stops the mode.

CLI:

```powershell
python -m esp_drone_cli --serial COM4 ground-capture-ref
python -m esp_drone_cli --serial COM4 liftoff-verify start --base-duty 0.10
python -m esp_drone_cli --serial COM4 liftoff-verify stop
```

GUI:

- `Liftoff Verify Start`
- `Liftoff Verify Stop`

First physical liftoff verification should be short, low, and conservative. The
first pass only checks whether the vehicle immediately tips or runs away; it does
not try to tune hover quality.
