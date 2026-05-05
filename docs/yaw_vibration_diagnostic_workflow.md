# Yaw Vibration Diagnostic Workflow

## Scope

Use this workflow when yaw raw or filtered rate shows high-frequency vibration in the hundreds of dps.

This is a diagnostic workflow only. It is not PID tuning, motor trim tuning, liftoff verification, free-flight stabilize, altitude hold, or takeoff validation.

The workflow uses existing host-side CSV capture and `analyze-vibration-log`. It does not require firmware rebuild or reflashing when the CLI command already exists.

Do not use:

- `save`
- `takeoff`
- `liftoff`
- free-flight commands
- PID increases
- motor trim changes

## Known Signal Facts

- `rate_meas_yaw_raw == -raw_gyro_z`.
- Raw gyro values are in `deg/s`.
- PID input is also in `deg/s`.
- No rad/s versus deg/s unit mix is currently indicated by the yaw-rate signal chain.
- Current low-duty vibration diagnosis assumes `motor_scale_m1..m4 = 1.0`, `motor_offset_m1..m4 = 0.0`, and `motor_trim_m1..m4 = 0.0`.
- Legacy `motor-trim-estimate` compensation is not part of this workflow.

## Phase A: Static IMU Baseline

Goal: confirm the gyro, IMU mounting, estimator validity, and CSV analysis are normal while motors are not rotating.

Run from `tools/esp_drone_cli`:

```powershell
cd D:\0Work\Codex\ESP-drone-main-merge\tools\esp_drone_cli
python -m esp_drone_cli.cli.main --serial COM8 disarm
python -m esp_drone_cli.cli.main --serial COM8 dump-csv logs\diag_static_gyro_10s.csv --duration 10
python -m esp_drone_cli.cli.main analyze-vibration-log logs\diag_static_gyro_10s.csv --mode static --output logs\diag_static_gyro_10s_summary.csv
```

Review these summary fields before any motor test:

- `rate_meas_yaw_raw_min/max/rms/peak`
- `rate_meas_yaw_filtered_min/max/rms/peak`
- `gyro_raw_z_min/max/rms/peak`
- `gyro_filtered_z_min/max/rms/peak`
- `loop_dt_us_min/max`
- `acc_norm_min/max/mean`
- `kalman_valid_min`
- `ground_trip_reason_max`
- `warning`

Stop before motor testing if any of these are true:

- yaw raw or filtered is already clearly abnormal while static
- `kalman_valid_min = 0`
- `ground_trip_reason_max = 2`
- `acc_norm_min < 0.60` or `acc_norm_max > 1.40`
- COM timeout or disconnect appears
- warning indicates an invalid or incomplete run

If static is abnormal, inspect IMU module mounting, IMU power, IMU communication, flight-controller board mounting, and the sensor/filter/log field chain before spinning motors.

## Phase B: All-Motor Low-Duty Open Loop

Goal: find the first collective low duty where yaw vibration appears.

Only run this phase after Phase A is normal. Do not batch all duties. Start with `0.05`, analyze the summary, and increase one step only if the previous step is acceptable.

First run only:

```powershell
cd D:\0Work\Codex\ESP-drone-main-merge\tools\esp_drone_cli
python -m esp_drone_cli.cli.main --serial COM8 all-motor-test --duty 0.05 --duration-s 1.5 --output-dir logs
$csv = Get-ChildItem .\logs\*_all_motor_test_05pct.csv | Sort-Object LastWriteTime -Descending | Select-Object -First 1
$summary = $csv.FullName -replace '\.csv$','_vibration_summary.csv'
python -m esp_drone_cli.cli.main analyze-vibration-log $csv.FullName --mode all-motor --duty 0.05 --output $summary
```

Only if `0.05` is acceptable, repeat the same pattern for `0.06`. Continue to `0.07`, then `0.08`, only after each previous duty is reviewed.

Stop increasing duty if any of these are true:

- yaw raw or filtered peak exceeds `800 deg/s`
- `acc_norm_min < 0.60` or `acc_norm_max > 1.40`
- `kalman_valid_min = 0`
- `ground_trip_reason_max = 2`
- motor saturation persists
- COM timeout or flight controller disconnect appears
- warning indicates an invalid or unsafe run
- operator reports motor squeal, heat, abnormal airflow, or frame movement/jumping

Interpretation:

- Static normal but all-motor `0.05` abnormal points first to frame resonance, IMU mounting, flight-controller or battery mounting, or whole-airframe motor vibration.
- If the first abnormal duty is `0.06`, `0.07`, or `0.08`, record that threshold and use nearby duties for single-motor localization.
- If all low duties are acceptable, continue to Phase C.

## Phase C: Single-Motor Low-Duty Localization

Goal: determine whether one motor, propeller, arm, mount, or supply path is introducing the yaw vibration.

Only run this phase after Phase A is normal and Phase B low-duty all-motor results are acceptable. Start with `0.05,0.06`; do not jump to higher duty first.

Run from `tools/esp_drone_cli`:

```powershell
cd D:\0Work\Codex\ESP-drone-main-merge\tools\esp_drone_cli
python -m esp_drone_cli.cli.main --serial COM8 motor-thrust-balance --duties 0.05,0.06 --duration-s 1.0 --settle-s 0.3 --rest-s 1.5 --motors M1,M2,M3,M4 --output-dir logs --no-trim
$csv = Get-ChildItem .\logs\*_motor_thrust_balance.csv | Sort-Object LastWriteTime -Descending | Select-Object -First 1
$summary = $csv.FullName -replace '\.csv$','_vibration_summary.csv'
python -m esp_drone_cli.cli.main analyze-vibration-log $csv.FullName --mode single-motor --output $summary
```

Review each motor and duty for:

- `rate_meas_yaw_raw_rms/peak`
- `rate_meas_yaw_filtered_rms/peak`
- `gyro_raw_z_rms/peak`
- `gyro_filtered_z_rms/peak`
- `gyro_raw_x/y/z_rms/peak`
- `gyro_filtered_x/y/z_rms/peak`
- `acc_norm_min/max/mean`
- `kalman_valid_min`
- `ground_trip_reason_max`
- `battery_min`
- `warning`

If `0.05,0.06` are acceptable, only then run `0.07,0.08`:

```powershell
cd D:\0Work\Codex\ESP-drone-main-merge\tools\esp_drone_cli
python -m esp_drone_cli.cli.main --serial COM8 motor-thrust-balance --duties 0.07,0.08 --duration-s 1.0 --settle-s 0.3 --rest-s 1.5 --motors M1,M2,M3,M4 --output-dir logs --no-trim
$csv = Get-ChildItem .\logs\*_motor_thrust_balance.csv | Sort-Object LastWriteTime -Descending | Select-Object -First 1
$summary = $csv.FullName -replace '\.csv$','_vibration_summary.csv'
python -m esp_drone_cli.cli.main analyze-vibration-log $csv.FullName --mode single-motor --output $summary
```

Interpretation:

- One motor clearly higher than the others points first to that motor's prop balance, prop centering, motor shaft, motor mount or arm stiffness, solder/power path, or local sensitivity of the IMU mounting.
- M4 clearly higher is an M4 high-risk result: inspect M4 prop dynamic balance, prop eccentricity, shaft eccentricity, mount/arm looseness, solder/power issues, and whether IMU mounting is especially sensitive to the M4 corner.
- All four single-motor results high points first to IMU mounting, flight-controller board mounting, frame resonance, battery mounting, or missing damping.
- Raw not high but filtered high points first to filter behavior, loop `dt`, field mapping, unit conversion, or analysis statistics.

## Report Format

When reviewing a returned summary or CSV, report only:

1. Whether this test round is valid.
2. Key metrics: yaw raw peak, yaw filtered peak, gyro raw/filtered Z peak, `acc_norm` min/max, `kalman_valid_min`, `ground_trip_reason_max`, and `battery_min` if present.
3. Classification: static IMU abnormal, whole-airframe vibration, single motor/prop abnormal, M4 high risk, IMU/flight-controller mounting issue, filtered/field/statistics issue, or normal and ready for the next low-duty stage.
4. One minimal next step.
5. Stop condition if any threshold was crossed.
