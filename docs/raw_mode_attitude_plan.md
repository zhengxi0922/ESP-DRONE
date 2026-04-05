# RAW Mode Attitude Plan

**Language / 语言：** **English** | [简体中文](./raw_mode_attitude_plan.zh-CN.md)

## Goal

Provide a project-owned attitude source for `IMU_MODE_RAW` so that upper layers can consume the same `imu_sample_t` contract in both modes:

- `timestamp_us`
- `gyro_xyz_dps`
- `acc_xyz_g`
- `quat_wxyz`
- `roll_pitch_yaw_deg`
- `health`
- `update_age_us`

## Planned Estimator

The planned raw-mode estimator is a Mahony-style quaternion filter running on the ESP32-S3.

Reasons:

- low computational cost
- stable quaternion output for the project body frame
- straightforward gyro and accelerometer fusion
- optional magnetometer correction path without changing the upper interface

## Planned Update Rate

- the raw-mode estimator will run only on fresh IMU samples
- the default IMU return rate remains `200 Hz`
- the optional high-rate mode remains `250 Hz`
- the estimator will not run at `1 kHz` without fresh IMU data

This keeps the estimator aligned with [runtime_frequency_plan.md](./runtime_frequency_plan.md).

## Inputs

Required inputs:

- mapped body-frame gyro from the single IMU mapping entry
- mapped body-frame accelerometer from the same mapping entry

Optional inputs:

- mapped magnetometer, only when explicitly enabled and validated

## Magnetometer Policy

- the magnetometer stays disabled by default during early bring-up
- when enabled later, it is used only for slow yaw correction
- magnetometer data must pass magnitude and freshness checks before entering the estimator

## Yaw Drift Strategy

Without a magnetometer:

- yaw is gyro-integrated
- roll and pitch remain gravity-corrected
- heading observability is reduced
- `imu_health` should report `DEGRADED` rather than `OK` when the system depends on a drifting yaw source for attitude output

With a validated magnetometer:

- yaw correction uses a slower gain than roll or pitch correction
- yaw jumps must be rejected by innovation gating

## Calibration Plan

- gyro bias calibration: stationary average during explicit `calib gyro`
- level calibration: optional body-frame trim for the accelerometer or attitude zero
- those calibrations must live above the module-to-body mapping so the project frame stays the single truth

## Mode Switching Rule

- `IMU_MODE_DIRECT`: use module quaternion or attitude after project mapping
- `IMU_MODE_RAW`: use ESP32 Mahony output after project mapping and calibration
- upper layers must not branch on the source after `imu_sample_t` has been filled

## Bring-Up Rule

Before angle mode is designed or enabled:

- direct-mode signs must be physically verified on the bench
- raw-mode Mahony output must be compared against direct mode on the bench
- yaw behavior without magnetometer must be explicitly accepted for the current test scope

## Current Stage Boundary

This document is design only for the current round.

- no full raw-mode attitude estimator is implemented yet
- no angle PID work should start from this document alone
- the immediate control-stage focus remains single-axis rate-loop validation
