# 电机映射

**语言 / Language：** [English](./motor_map.md) | **简体中文**

## 物理布局

俯视布局如下：

```text
        Nose (+Y)

   M1 (LF, CCW)     M2 (RF, CW)

   M4 (LR, CW)      M3 (RR, CCW)

        Tail (-Y)
```

## 逻辑到硬件映射

| 电机 | 物理位置 | 旋向 | GPIO |
|---|---|---|---|
| `M1` | 左前 | CCW | `IO5` |
| `M2` | 右前 | CW | `IO6` |
| `M3` | 右后 | CCW | `IO3` |
| `M4` | 左后 | CW | `IO4` |

## Mixer 方向真值表

| 指令 | 增大 | 减小 |
|---|---|---|
| `+roll` | `M1`、`M4` | `M2`、`M3` |
| `-roll` | `M2`、`M3` | `M1`、`M4` |
| `+pitch` | `M3`、`M4` | `M1`、`M2` |
| `-pitch` | `M1`、`M2` | `M3`、`M4` |
| `+yaw` | `M1`、`M3` | `M2`、`M4` |
| `-yaw` | `M2`、`M4` | `M1`、`M3` |

## Bring-up 验证路径

- `motor-test m1..m4 <duty>`：逐通道验证逻辑电机顺序
- `axis-test roll|pitch|yaw <value>`：在未解锁状态下验证开环 mixer 方向
- `rate-test roll|pitch|yaw <dps>`：在解锁状态下验证最小 fresh-sample rate loop

前两项属于阶段 2.5 的闸门项，第三项是 bring-up 闸门接通后的第一个允许阶段 3 控制检查。

## 实现规则

固件必须严格区分以下概念：

- 物理输出通道
- 逻辑电机编号
- 物理位置
- 旋向

Mixer 只能依据位置和旋向计算，GPIO 编号不属于 mixer 数学的一部分。
