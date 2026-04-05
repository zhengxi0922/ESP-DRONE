# Motor Map

**Language / 语言：** **English** | [简体中文](./motor_map.zh-CN.md)

## Physical Layout

Viewed from above:

```text
        Nose (+Y)

   M1 (LF, CCW)     M2 (RF, CW)

   M4 (LR, CW)      M3 (RR, CCW)

        Tail (-Y)
```

## Logical-To-Hardware Mapping

| Motor | Physical position | Spin | GPIO |
|---|---|---|---|
| `M1` | Left-front | CCW | `IO5` |
| `M2` | Right-front | CW | `IO6` |
| `M3` | Right-rear | CCW | `IO3` |
| `M4` | Left-rear | CW | `IO4` |

## Mixer Direction Truth Table

| Command | Increase | Decrease |
|---|---|---|
| `+roll` | `M1`, `M4` | `M2`, `M3` |
| `-roll` | `M2`, `M3` | `M1`, `M4` |
| `+pitch` | `M3`, `M4` | `M1`, `M2` |
| `-pitch` | `M1`, `M2` | `M3`, `M4` |
| `+yaw` | `M1`, `M3` | `M2`, `M4` |
| `-yaw` | `M2`, `M4` | `M1`, `M3` |

## Bring-Up Validation Path

- `motor-test m1..m4 <duty>` verifies logical motor order one channel at a time
- `axis-test roll|pitch|yaw <value>` verifies open-loop mixer direction while disarmed
- `rate-test roll|pitch|yaw <dps>` verifies the minimal fresh-sample rate loop while armed

The first two checks are stage-2.5 gate items. The third check is the first allowed stage-3 control check after the bring-up gate is wired in.

## Implementation Rule

The firmware must keep these concepts separate:

- physical output channel
- logical motor ID
- physical position
- spin direction

The mixer computes from position and spin direction only. GPIO numbers are not part of the mixer math.
