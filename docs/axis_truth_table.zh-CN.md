# 轴定义真值表

**语言 / Language：** [English](./axis_truth_table.md) | **简体中文**

## 机体系

- `+Y`：机头 / 前方
- `+X`：机体右侧
- `+Z`：朝上

## 姿态正方向命名

- `+pitch`：抬头
- `+roll`：右侧下沉
- `+yaw`：机头右转，俯视顺时针

## 重要说明

机体系本身是右手系，但姿态正方向命名是本项目自定义约定，必须严格按本表使用。特别是：

- `+pitch` 对应绕 `+X` 的数学正旋转
- `+roll` 对应绕 `+Y` 的数学负旋转
- `+yaw` 对应绕 `+Z` 的数学负旋转

所有方向敏感实现都必须以本表为准。

## 日志解释说明

Telemetry 同时包含两套不能混淆的符号系统：

- `gyro_x/y/z` 是机体系角速度字段，遵循机体系右手规则
- `roll_deg / pitch_deg / yaw_deg` 是项目定义的姿态命名，遵循本仓库的符号约定

因此：

- `gyro_x > 0` 表示正 `pitch-rate`
- `gyro_y < 0` 表示本项目中的正 `roll-rate`
- `gyro_z < 0` 表示本项目中的正 `yaw-rate`

## Bring-up 观察字段

在阶段 2.5 和最小阶段 3 的 rate-loop 工作中，建议重点观察：

- 手持动作验证：`gyro_x/y/z`，然后看 `roll_deg/pitch_deg/yaw_deg`
- 未解锁方向检查：`setpoint_roll/pitch/yaw` 配合 `motor1..motor4`
- 最小闭环速率检查：`rate_setpoint_roll/pitch/yaw`、`pid_out_roll/pitch/yaw`，再看 `motor1..motor4`

## 真值表

| 物理动作 | 轴符号 | 主要日志变化 | 电机趋势 |
|---|---:|---|---|
| 抬头 | `+pitch` | 动作过程中 `gyro_x` 变正；姿态建立后 `pitch_deg` 变正 | 后电机增大，前电机减小 |
| 低头 | `-pitch` | 动作过程中 `gyro_x` 变负；姿态建立后 `pitch_deg` 变负 | 前电机增大，后电机减小 |
| 右侧下沉 | `+roll` | 动作过程中 `gyro_y` 变负；姿态建立后 `roll_deg` 变正 | 左电机增大，右电机减小 |
| 左侧下沉 | `-roll` | 动作过程中 `gyro_y` 变正；姿态建立后 `roll_deg` 变负 | 右电机增大，左电机减小 |
| 机头右转，俯视顺时针 | `+yaw` | 动作过程中 `gyro_z` 变负；航向建立后 `yaw_deg` 变正 | CCW 电机增大，CW 电机减小 |
| 机头左转，俯视逆时针 | `-yaw` | 动作过程中 `gyro_z` 变正；航向建立后 `yaw_deg` 变负 | CW 电机增大，CCW 电机减小 |
