# 速率环 Bring-up 结果

**语言 / Language：** [English](./rate_bringup_results.md) | **简体中文**

## 范围

本文只记录单轴 `rate-loop` bring-up 阶段。

- 不涉及自由飞
- 不涉及 angle 模式
- 不涉及外环调参
- 台架测试时必须拆桨或将机体完全固定

## 当前状态

- 单轴 `rate-test` 的固件支持已经实现
- telemetry 已经暴露 `gyro_x/y/z`、`roll/pitch/yaw`、`rate_setpoint_*`、`rate_pid_p/i/d_*`、`pid_out_*`、`motor1..motor4`、`imu_age_us`、`loop_dt_us`、`arm_state`、`control_mode` 和 `failsafe_reason`
- 主机构建和主机侧方向测试通过
- 在 `2026-04-01`，本工作站执行 `python -m serial.tools.list_ports -v` 时返回 `no ports found`，因此那次会话没有采集到真实台架测量结果

## 本阶段使用的姿态来源

- 最小 rate loop 主要使用映射后的 gyro 路径作为反馈输入
- 在 `direct` 模式下，姿态和四元数字段来自 `ATK-MS901M` 输出并经过项目映射
- 在 `raw` 模式下，未来姿态来源见 [raw_mode_attitude_plan.zh-CN.md](./raw_mode_attitude_plan.zh-CN.md)，但该估计器目前尚未用于 angle 模式
- 因为本阶段只做 rate，主方向闸门仍然是：物理动作 -> 映射后 gyro 符号 -> rate PID 符号 -> mixer 方向

## 通用流程

1. 通过 USB CDC 连接并开启 telemetry 流。
2. 在解锁前先完成手持动作日志验证。
3. 只允许在拆桨或机体受限固定的台架上解锁。
4. 每次只测试一个 `rate-test` 轴。
5. 若符号或电机方向有误，立刻执行 `kill`。

## 分轴 Bring-up 表

| 测试轴 | 物理动作 | CLI 命令 | 主要观察的 gyro 字段 | 主要 setpoint 字段 | 主要观察的 PID 字段 | 期望电机方向 | 实际观察结果 | 状态 |
|---|---|---|---|---|---|---|---|---|
| X 轴物理转动，对应项目 `pitch-rate` | 围绕机体系 `+X` 做抬头或低头转动 | `rate-test pitch 30` 和 `rate-test pitch -30` | 抬头角速度时 `gyro_x` 为正，低头角速度时为负 | `rate_setpoint_pitch` | `rate_pid_p_pitch`、`rate_pid_i_pitch`、`rate_pid_d_pitch`、`pid_out_pitch` | `+pitch` 时 `M3/M4` 高于 `M1/M2`；`-pitch` 反之 | 本次会话因无 USB CDC 端口而未执行 | Blocked |
| Y 轴物理转动，对应项目 `roll-rate` | 按项目命名约定做右侧下沉或左侧下沉 | `rate-test roll 30` 和 `rate-test roll -30` | `+roll` 时 `gyro_y` 为负，`-roll` 时为正 | `rate_setpoint_roll` | `rate_pid_p_roll`、`rate_pid_i_roll`、`rate_pid_d_roll`、`pid_out_roll` | `+roll` 时 `M1/M4` 高于 `M2/M3`；`-roll` 反之 | 本次会话因无 USB CDC 端口而未执行 | Blocked |
| Z 轴物理转动，对应项目 `yaw-rate` | 做机头右转或左转的 yaw 动作 | `rate-test yaw 30` 和 `rate-test yaw -30` | `+yaw` 时 `gyro_z` 为负，`-yaw` 时为正 | `rate_setpoint_yaw` | `rate_pid_p_yaw`、`rate_pid_i_yaw`、`rate_pid_d_yaw`、`pid_out_yaw` | `+yaw` 时 `M1/M3` 高于 `M2/M4`；`-yaw` 反之 | 本次会话因无 USB CDC 端口而未执行 | Blocked |

## 已通过软件验证的基线

- 项目中的 gyro 命名固定为：
  - `pitch_rate = gyro_x`
  - `roll_rate = -gyro_y`
  - `yaw_rate = -gyro_z`
- mixer 契约测试已确认：
  - `+roll` -> `M1/M4` 增大
  - `+pitch` -> `M3/M4` 增大
  - `+yaw` -> `M1/M3` 增大
- `flight_control_task` 只会在收到 fresh IMU sample 时更新 estimator 和 rate PID

## 进入 Angle 模式的闸门

在以下条件全部通过硬件实测前，angle 模式设计都必须继续阻塞：

- 单轴 rate-loop 方向正确
- mixer 输出方向正确
- 参数校验能拒绝非法映射和不安全限制
- 当前测试配置下，`raw` 或 `direct` 模式的实际姿态来源已经被明确理解
