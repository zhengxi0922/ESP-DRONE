# Hang Attitude Bring-Up Plan

**Language / 语言:** **English** | [简体中文](./hang_attitude_bringup_plan.zh-CN.md)

## Purpose

This stage is a bench-only attitude outer-loop bring-up for a circular-rod, hanging, or otherwise constrained rig.

It is not a free-flight stabilize mode.
It is not an angle-flight-ready mode.
It must never be used on a prop-on free-flight vehicle.

The target rig is unusual because the natural equilibrium after mounting is `+Z down`, not `+Z up`.
That is why the controller does not use a world-level `roll=0 / pitch=0` target.

## Control Strategy

Reference capture is explicit:

1. Place the airframe in its natural hanging equilibrium.
2. Run `attitude-capture-ref`.
3. Firmware stores the current quaternion as `q_ref`.

After capture, the outer loop works from relative quaternion error:

```text
q_rel = q_ref^-1 * q_now
```

The controller then extracts small-angle roll and pitch error from `q_rel`.
It does not subtract global Euler angles against a fixed zero target, so it avoids the `+-180 deg` jump and flip problems that show up on this rig.

This stage only controls `roll / pitch`.
Yaw stays out of the attitude outer loop and defaults to `rate_sp_yaw = 0`.

The outer loop is intentionally minimal:

```text
rate_sp_roll  = clamp(-attitude_kp_roll  * err_roll_deg,  -attitude_rate_limit_roll,  attitude_rate_limit_roll)
rate_sp_pitch = clamp(-attitude_kp_pitch * err_pitch_deg, -attitude_rate_limit_pitch, attitude_rate_limit_pitch)
rate_sp_yaw   = 0
```

Those rate setpoints feed the existing tuned rate inner loop.
Throttle stays open-loop through `attitude_test_base_duty`.

## Parameters

Recommended conservative defaults:

| Parameter | Default | Notes |
|---|---:|---|
| `attitude_kp_roll` | `2.0` | roll outer-loop P gain |
| `attitude_kp_pitch` | `2.0` | pitch outer-loop P gain |
| `attitude_rate_limit_roll` | `25.0` | roll rate-setpoint limit in dps |
| `attitude_rate_limit_pitch` | `25.0` | pitch rate-setpoint limit in dps |
| `attitude_error_deadband_deg` | `1.0` | zeroes small residual error |
| `attitude_trip_deg` | `30.0` | immediate stop threshold |
| `attitude_test_base_duty` | `0.05` | fixed open-loop base duty |
| `attitude_ref_valid` | runtime | set true only after capture |

`attitude_ref_valid` is runtime-only and is not intended as a persisted tuning parameter.

## Telemetry

Use these fields to verify the full chain:

- `attitude_ref_valid`
- `attitude_err_roll_deg`
- `attitude_err_pitch_deg`
- `attitude_rate_sp_roll`
- `attitude_rate_sp_pitch`
- `attitude_ref_qw`, `attitude_ref_qx`, `attitude_ref_qy`, `attitude_ref_qz`
- `control_mode`
- `base_duty_active`
- existing `pid_out_roll`, `pid_out_pitch`, and `motor1..motor4`

The intended debugging path is:

```text
manual disturbance -> attitude error -> outer-loop rate setpoint -> rate PID output -> motor mix
```

## CLI Workflow

Minimum command set:

```powershell
python -m esp_drone_cli --serial COM7 attitude-capture-ref
python -m esp_drone_cli --serial COM7 arm
python -m esp_drone_cli --serial COM7 attitude-test start --base-duty 0.05
python -m esp_drone_cli --serial COM7 attitude-status --timeout 5
python -m esp_drone_cli --serial COM7 watch-attitude all --timeout 10 --interval 0.2
python -m esp_drone_cli --serial COM7 attitude-test stop
python -m esp_drone_cli --serial COM7 disarm
```

Rejection behavior is explicit:

- `attitude-test start` is rejected if the airframe is not armed
- `attitude-test start` is rejected if `attitude_ref_valid` is false
- IMU health, freshness, and quaternion availability are checked before capture or start
- command failures are returned as clear CLI errors instead of silent ignore

## GUI Workflow

The GUI adds a dedicated `Hang Attitude` area under `Debug Actions`.

Available controls:

- `Capture Ref`
- `Attitude Test Start`
- `Attitude Test Stop`
- `Base Duty`
- quick-edit controls for:
  - `attitude_kp_roll`
  - `attitude_kp_pitch`
  - `attitude_rate_limit_roll`
  - `attitude_rate_limit_pitch`
  - `attitude_error_deadband_deg`
  - `attitude_trip_deg`

The chart selector also adds:

- `Hang Attitude Roll`
- `Hang Attitude Pitch`

Each chart shows:

- measured attitude error
- generated rate setpoint
- inner-loop `pid_out`
- `motor1..motor4`

## Safety Stop Rules

The firmware immediately stops `CONTROL_MODE_ATTITUDE_HANG_TEST` and clears the outer-loop output if any of these conditions occur:

- IMU sample is stale or health is not acceptable
- reference attitude is not valid
- `abs(attitude_err_roll_deg)` or `abs(attitude_err_pitch_deg)` exceeds `attitude_trip_deg`
- arm state becomes abnormal
- failsafe triggers
- user sends `kill` or `attitude-test stop`

Stopping this mode clears the attitude outer-loop request so no residual rate setpoint is left behind.

## Acceptance Checklist

These sign checks are mandatory and must be verified on the constrained rig before any later work:

1. Disturb the airframe to `right side down`, which is positive `roll` in this project.
2. The controller must command a negative roll correction to pull back toward the captured reference.
3. Expected motor response: `M2 / M3` increase, `M1 / M4` decrease.

And:

1. Disturb the airframe to `nose up`, which is positive `pitch` in this project.
2. The controller must command a negative pitch correction to pull back toward the captured reference.
3. Expected motor response: `M1 / M2` increase, `M3 / M4` decrease.

Yaw is not part of the attitude-hold acceptance for this stage.
