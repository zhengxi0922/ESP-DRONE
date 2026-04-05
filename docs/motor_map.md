# Motor Map

## 中文摘要

- 俯视布局固定为：`M1 左前 CCW`、`M2 右前 CW`、`M3 右后 CCW`、`M4 左后 CW`。
- 电机逻辑编号、物理位置、旋向和 GPIO 是分离概念，不能直接写死到 mixer 公式里。
- 单轴方向验证时，必须检查 `+roll / +pitch / +yaw` 对应的四电机增减方向是否与此表一致。

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

The first two checks are stage 2.5 gate items. The third check is the first allowed stage 3 control check after the bring-up gate is wired in.

## Implementation Rule

The firmware must keep these concepts separate:

- physical output channel
- logical motor ID
- physical position
- spin direction

The mixer computes from position and spin direction only. GPIO numbers are not part of the mixer math.
