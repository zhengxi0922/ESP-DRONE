# Ground Tune Bring-up Plan

This plan is for prop-on, flat-ground, low-throttle tuning only. It is not a free-flight stabilize mode.

## Scope

- Aircraft body frame stays unchanged: +Y nose, +X right, +Z up.
- `CONTROL_MODE_ATTITUDE_GROUND_TUNE` is independent from `CONTROL_MODE_ATTITUDE_HANG_TEST`.
- Roll and pitch use a captured flat-ground reference and an attitude outer loop.
- Yaw remains rate inner-loop only. No yaw heading hold is enabled.
- Existing `rate-test`, `axis-bench`, `rate-bench`, `attitude-capture-ref`, `attitude-test`, `watch-attitude`, and UDP manual paths remain separate.

## Firmware Path

1. IMU raw sample is preserved.
2. Estimator adds filtered state:
   - raw gyro XYZ
   - filtered gyro XYZ
   - raw accel XYZ
   - filtered accel XYZ
   - raw roll/pitch/yaw
   - raw quaternion
   - roll/pitch 1D Kalman angles
   - raw and filtered roll/pitch/yaw rates
3. Ground tune captures `q_ref_ground` and optional Kalman roll/pitch reference.
4. Ground outer loop computes roll/pitch error from the relative reference. If `ground_tune_use_kalman_attitude=true`, the captured Kalman deltas are used as the roll/pitch measurement for the outer loop.
5. Rate PID consumes filtered rate when `ground_tune_use_filtered_rate=true`.
6. Mixer output is clamped around `ground_test_base_duty` by `ground_test_max_extra_duty`.

## Conservative Defaults

- `ground_test_base_duty = 0.08`: start slightly above the stable spin point and tune per motor.
- `ground_att_kp_roll = 1.2`
- `ground_att_kp_pitch = 1.2`
- `ground_att_rate_limit_roll = 12 deg/s`
- `ground_att_rate_limit_pitch = 12 deg/s`
- `ground_att_error_deadband_deg = 0.8 deg`
- `ground_att_trip_deg = 12 deg`
- `ground_test_max_extra_duty = 0.05`
- `ground_test_motor_balance_limit = 0.08`
- `ground_test_auto_disarm_ms = 15000`
- `gyro_lpf_hz = 40 Hz`
- `accel_lpf_hz = 20 Hz`
- `rate_lpf_hz = 30 Hz`
- `kalman_enable = true`
- `kalman_q_angle = 0.0025`
- `kalman_q_bias = 0.0030`
- `kalman_r_measure = 0.0800`

## Stop Conditions

Ground tune stops and logs an event for:

- `ground ref missing`
- `kalman invalid`
- `angle trip`
- `saturation trip`
- `imu stale`
- `watchdog timeout`
- `rate output jitter trip`
- `failsafe`
- `ground tune stopped normally`

Battery critical and failsafe paths request stop plus disarm.

## Not Free-flight Ready

- No yaw angle hold.
- No position, velocity, altitude, or takeoff controller.
- No prop-off safety assumption; the mode assumes restrained, flat-ground, low-throttle operation.
- Ground contact changes dynamics, so passing ground tune does not prove airborne stability.
