# Axis Truth Table

## Body Frame

- `+Y`: nose / front
- `+X`: right side
- `+Z`: up

## Positive Attitude Names

- `+pitch`: nose up
- `+roll`: right side down
- `+yaw`: nose right, clockwise viewed from above

## Important Note

The body frame is right-handed. The positive attitude names are project-defined and must be used exactly as listed here. In particular:

- `+pitch` matches positive rotation about `+X`
- `+roll` corresponds to negative mathematical rotation about `+Y`
- `+yaw` corresponds to negative mathematical rotation about `+Z`

Every direction-sensitive implementation must reference this table.

## Log Interpretation Note

Telemetry uses two different sign systems that must not be confused:

- `gyro_x/y/z` are body-axis angular-rate fields and follow the body-axis right-hand rule
- `roll_deg / pitch_deg / yaw_deg` are project-defined attitude names and follow this repository's sign convention

That means:

- `gyro_x > 0` corresponds to positive pitch-rate motion
- `gyro_y < 0` corresponds to positive roll-rate motion in this project
- `gyro_z < 0` corresponds to positive yaw-rate motion in this project

## Truth Table

| Physical action | Axis sign | Expected main log field changes | Motor trend |
|---|---:|---|---|
| Nose up | `+pitch` | `gyro_x` becomes positive while moving; `pitch_deg` becomes positive after the attitude changes | Rear motors increase, front motors decrease |
| Nose down | `-pitch` | `gyro_x` becomes negative while moving; `pitch_deg` becomes negative after the attitude changes | Front motors increase, rear motors decrease |
| Right side down | `+roll` | `gyro_y` becomes negative while moving; `roll_deg` becomes positive after the attitude changes | Left motors increase, right motors decrease |
| Left side down | `-roll` | `gyro_y` becomes positive while moving; `roll_deg` becomes negative after the attitude changes | Right motors increase, left motors decrease |
| Nose right, clockwise viewed from above | `+yaw` | `gyro_z` becomes negative while moving; `yaw_deg` becomes positive after the heading changes | CCW motors increase, CW motors decrease |
| Nose left, counter-clockwise viewed from above | `-yaw` | `gyro_z` becomes positive while moving; `yaw_deg` becomes negative after the heading changes | CW motors increase, CCW motors decrease |
