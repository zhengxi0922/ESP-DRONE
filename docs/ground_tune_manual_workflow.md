# Ground Tune Manual Workflow

Use this workflow in order. Do not skip a failed stage.

## Phase A: Sensor And Sign Check, Motors Stopped

1. Connect USB CDC.
2. Run `stream on`.
3. Record 10 seconds of raw and filtered data:
   - CLI: `ground-log --duration 10 --output-dir logs`
   - GUI: Ground Tune `Ground Log 10s`
4. Move the aircraft gently by hand and verify:
   - `roll_rate = -gyro_y`
   - `pitch_rate = gyro_x`
   - `yaw_rate = -gyro_z`
   - `kalman_roll_deg` and `kalman_pitch_deg` have the same sign as the physical motion.
5. If any sign is wrong, do not continue. Fix IMU mapping or estimator signs first.

## Minimum Estimator Telemetry Smoke Check

Run this before low-throttle ground tests. Keep motors stopped and leave the aircraft disarmed.

1. Start GUI streaming or run `dump-csv logs/estimator_smoke.csv --duration 10`.
2. Hold the aircraft flat and still. Confirm `raw_acc_z` is near +1g, `kalman_valid=1`, and `attitude_valid=1`.
3. Move by hand through about +/-10 degrees roll, +/-10 degrees pitch, and a small yaw twist.
4. Confirm GUI and CSV fields:
   - `raw_gyro_x`, `raw_gyro_y`, `raw_gyro_z` respond to motion.
   - `filtered_gyro_x`, `filtered_gyro_y`, `filtered_gyro_z` follow the raw gyro response.
   - `filtered_acc_x`, `filtered_acc_y`, `filtered_acc_z` stay on the same axes as `raw_acc_x`, `raw_acc_y`, `raw_acc_z`.
   - `kalman_roll_deg` and `kalman_pitch_deg` change with roll/pitch motion.
   - `kalman_valid=1` and `attitude_valid=1` remain set while the aircraft is flat and moved gently.
   - `battery_valid=1`, `battery_voltage` is non-zero, and `battery_adc_raw` is non-zero.
5. The GUI Record buttons are telemetry-only; they should not write PID, filter, or Kalman params.

## Phase B: Low-throttle Motor Split Check

1. Place the aircraft flat on the ground.
2. Run `ground-capture-ref`.
   This captures runtime reference state only; it should not write PID, filter, Kalman, or runtime status params.
3. Keep `ground_tune_enable_attitude_outer=false`.
4. Arm only when the area is safe.
5. Run `ground-test start --base-duty <low_value>`.
   Base duty ramps up; it should not step immediately to the target.
6. Check telemetry:
   - `rate_setpoint_roll`, `rate_setpoint_pitch`, and `rate_setpoint_yaw` stay at zero.
   - `rate_meas_*`, `rate_err_*`, `rate_pid_p_*`, `pid_out_*`, `mixer_*`, and `motor1..motor4` are populated.
   - `motor_saturation_flag=0`, `integrator_freeze_flag=0`, and `ground_trip_reason=0`.
7. Gently push:
   - right side down
   - left side down
   - nose up
   - nose down
8. Check that measured rate, rate error, PID output, mixer control, and motor split have the expected direction.
9. Record one roll set and one pitch set:
   - `ground-bench roll --duration 5`
   - `ground-bench pitch --duration 5`
10. Run `ground-test stop`.

## Phase C: Inner-loop PID Retune

1. Tune roll first, then pitch, then yaw.
2. Keep `ground_tune_enable_attitude_outer=false`, Ki at zero, and Kd at zero for the first pass.
3. Each axis should use small manual perturbations around a zero rate target. The current ground bench records the response and summary.
4. Each run writes `summary.json` and `summary.md`.
5. Default strategy:
   - keep Ki at 0 first
   - tune Kp first
   - add a very small Kd only when direction and damping are clean
   - decide on Ki last, only for remaining steady bias
6. After each axis, change parameters only. Do not change code. Repeat the bench.

## Phase D: Outer-loop P Adjustment

1. Set `ground_tune_enable_attitude_outer=true` only after Phase C passes.
2. Disturb the airframe by about +/-2 to +/-5 degrees.
3. Watch return speed, overshoot, jitter, and steady-state bias.
4. Tune in this order:
   - `ground_att_kp_roll` / `ground_att_kp_pitch`
   - `ground_att_rate_limit_roll` / `ground_att_rate_limit_pitch`
   - `ground_att_error_deadband_deg`
5. Yaw remains rate mode. Do not add yaw heading hold at this stage.

## Phase E: Whole-airframe Ground Integration

1. Use Ground mode plus UDP manual at low throttle only after signs and PID are verified.
2. Roll/pitch outer loop stays active.
3. Yaw stays rate-only.
4. Throttle uses a ramp or conservative base-duty changes.
5. Record one all-axis run:
   - `ground-bench all --duration 5`
6. Continue only when `safe_to_continue=true` and the hint does not ask for a stop.

## CLI Quick Reference

- `ground-capture-ref`
- `ground-log --duration 10 --output-dir logs`
- `ground-test start --base-duty 0.08`
- `ground-test stop`
- `ground-status --timeout 5`
- `watch-ground all --timeout 10 --interval 0.1`
- `ground-bench roll --duration 5`
- `ground-bench pitch --duration 5`
- `ground-bench yaw --duration 5`
- `ground-bench all --duration 5`

## GUI Quick Reference

Use the Ground Tune block:

- Capture Ground Ref
- Ground Test Start
- Ground Test Stop
- Ground Log 5s
- Ground Log 10s
- Run Ground Bench Roll
- Run Ground Bench Pitch
- Run Ground Bench Yaw
- Run Ground Bench All

The bottom log reports the last saved directory, recording result, `safe_to_continue`, and `next_action_hint`.
