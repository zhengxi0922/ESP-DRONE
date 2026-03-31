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

## Truth Table

| Physical action | Axis sign | Motor trend |
|---|---:|---|
| Nose up | `+pitch` | Rear motors increase, front motors decrease |
| Nose down | `-pitch` | Front motors increase, rear motors decrease |
| Right side down | `+roll` | Left motors increase, right motors decrease |
| Left side down | `-roll` | Right motors increase, left motors decrease |
| Nose right, clockwise viewed from above | `+yaw` | CCW motors increase, CW motors decrease |
| Nose left, counter-clockwise viewed from above | `-yaw` | CW motors increase, CCW motors decrease |
