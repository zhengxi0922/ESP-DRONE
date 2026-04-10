# Bring-up 清单

**语言 / Language:** [English](./bringup_checklist.md) | **简体中文**

这份清单必须在底层框架阶段和后续 estimator / controller 阶段之间执行。

## 闸门规则

在下面所有必需硬件验证项标记为 `PASS` 之前，不允许进入 PID 扩展、不允许进入自由飞 stabilize、不允许进入 angle 模式，也不允许带桨飞行测试。

圆棍姿态外环路径是 bench-only，但它不改变这个闸门规则。

## 当前阶段状态

- 本清单中 rate 环 bring-up 所需的固件和 CLI 支持已实现
- bench-only 圆棍姿态外环 bring-up 所需的固件、CLI、GUI 支持已实现
- 主机构建和 Python tests 通过
- 在 `2026-04-10`，本工作站检测到 `COM4`，并在圆棍受限台架上完成了一次自然姿态为 `+Z 朝下` 的 live roll 会话
- 该 live roll 基线参数为：
  - `rate_kp_roll = 0.0026`
  - `rate_ki_roll = 0.0`
  - `rate_kd_roll = 0.0`
- 同一会话测试了 `rate_kp_roll = 0.0028`；虽然符号仍然正确，但 `noise_or_jitter_risk` 上升，因此被拒绝
- 比较完成后，`rate_kp_roll = 0.0026` 已重新写回 flash

## 连通性

| 项目 | 可复现命令或证据 | 当前状态 |
|---|---|---|
| USB CDC 二进制握手 | `esp-drone-cli --serial COMx connect` | 代码完成，已在 `2026-04-10` 用 `COM4` 验证 |
| IMU UART 有效帧接收 | `esp-drone-cli --serial COMx stream on` 并观察 `imu seq/good/err/age_us` 事件 | 代码完成，已在 `2026-04-10` 用 `COM4` 验证 |
| LED 状态机切换 | 启动和命令测试期间观察 `INIT_WAIT_IMU`、`DISARMED_READY`、`FAULT_LOCK` 路径 | 代码完成，硬件待验证 |

## 供电与 ADC

| 项目 | 可复现命令或证据 | 当前状态 |
|---|---|---|
| 可读出 `BAT_ADC` 原始值 | telemetry 字段 `battery_adc_raw` 和 `battery_voltage` | 代码完成，硬件待验证 |
| 分压换算有效 | 使用 `100k / 100k` 分压，将 telemetry 电压与万用表对比 | 代码完成，硬件待验证 |
| telemetry 中能看到电池阈值相关状态 | 检查 `battery_voltage`、`arm_state` 和 `failsafe_reason` | 代码完成，硬件待验证 |

## 电机与安全

| 项目 | 可复现命令或证据 | 当前状态 |
|---|---|---|
| `motor_test m1..m4` 只驱动单个电机 | `esp-drone-cli --serial COMx motor-test m1 0.12`，并对 `m2..m4` 重复 | 代码完成，硬件待验证 |
| `M1..M4` 顺序与文档一致 | 按顺序运行 `motor-test`，并与 [motor_map.zh-CN.md](./motor_map.zh-CN.md) 对照 | 代码完成，硬件待验证 |
| `arm / disarm / kill` 可通过 CLI 生效 | 运行 `arm`、`disarm` 和 `kill`，观察 `arm_state` 与 `failsafe_reason` | 代码完成，`arm/disarm` 已在 `2026-04-10` 的 roll 会话中验证 |
| 姿态外环模式下安全门限仍生效 | 确认 idle 限制、输出限幅、arm gating 和 failsafe 仍会停掉该模式 | 代码完成，硬件待验证 |
| 摇杆手势解锁 / 上锁 | 等阶段 4 有 RC 输入后再验证 | 当前不在范围内 |

## 方向验证

| 项目 | 可复现命令或证据 | 当前状态 |
|---|---|---|
| 物理动作与 telemetry 符号约定一致 | 手持机体，观察 `gyro_x/y/z`、`roll/pitch/yaw`，并与 [axis_truth_table.zh-CN.md](./axis_truth_table.zh-CN.md) 对照 | 代码完成，roll 符号链已在 `2026-04-10` 验证 |
| `+roll` 让 `M1` 和 `M4` 增大 | `axis-test roll 0.05`，检查电机输出和 telemetry | 代码完成，硬件待验证 |
| `+pitch` 让 `M3` 和 `M4` 增大 | `axis-test pitch 0.05`，检查电机输出和 telemetry | 代码完成，硬件待验证 |
| `+yaw` 让 `M1` 和 `M3` 增大 | `axis-test yaw 0.05`，检查电机输出和 telemetry | 代码完成，硬件待验证 |
| 每一轴的最小闭环 rate 响应有效 | `arm` 后执行 `rate-test roll|pitch|yaw <dps>`，检查 `rate_setpoint_*`、`pid_out_*` 和电机输出 | 代码完成，roll 已在 `2026-04-10` 验证 |

## Roll 单轴台架验收

这一节是当前 roll bring-up 的有效闸门。

| 项目 | 可复现命令或证据 | 当前状态 |
|---|---|---|
| `rate_setpoint_roll` 跟随命令步阶 | 用 `rate-status roll` 或 `axis-bench roll` 检查 `setpoint_path_ok` | 已在 `2026-04-10` 验证 |
| Roll 符号正确 | `sign_ok=True`，且直接 `rate-test roll +/-20` 观察与 `roll_rate = -gyro_y` 一致 | 已在 `2026-04-10` 验证 |
| Roll 电机分配正确 | `motor_split_ok=True`；`+roll` 让 `M1/M4` 增，`M2/M3` 减 | 已在 `2026-04-10` 验证 |
| 响应可测 | `measurable_response=True` | 已在 `2026-04-10` 验证 |
| 饱和风险可接受 | `saturation_risk=False` | 已在 `2026-04-10` 验证 |
| 回零干净 | `return_to_zero_quality=PASS` | 已在 `2026-04-10` 验证 |
| 最大探测步阶只到低 duty 警告级别 | `low_duty_motor_stability=PASS_WITH_WARNING` | 已在 `2026-04-10` 验证 |
| `kp` 调参闸门已固化 | 只有 `kp_tuning_allowed=True` 时才允许继续；如果 `sign_ok` 或 `motor_split_ok` 失败，禁止继续调 `kp` | 工具链已固化，并在 `2026-04-10` 会话中按此执行 |

## Bench-Only 圆棍姿态外环 Bring-Up

本节所有项目都只适用于圆棍或吊架等受限台架。
这些项目都不意味着允许自由飞。

| 项目 | 可复现命令或证据 | 当前状态 |
|---|---|---|
| 能在自然 `+Z 朝下` 悬挂姿态下成功 capture 参考姿态 | 让机体回到自然平衡姿态后执行 `attitude-capture-ref`，并确认 `attitude_ref_valid=1` | 代码完成，硬件待验证 |
| capture 前启动会被拒绝 | 未 capture 时执行 `attitude-test start`，应看到明确的 `reference not captured` 错误 | 代码完成，硬件待验证 |
| 外环确实使用参考相对姿态，而不是世界系零欧拉角目标 | 检查 [hang_attitude_bringup_plan.zh-CN.md](./hang_attitude_bringup_plan.zh-CN.md) 和固件中 `q_rel = q_ref^-1 * q_now` 行为 | 代码完成，硬件待验证 |
| 圆棍姿态外环遥测链路可见 | 执行 `attitude-status` 或 `watch-attitude all`，确认 `attitude_ref_valid`、`attitude_err_*`、`attitude_rate_sp_*`、`pid_out_*`、`base_duty_active`、`motor1..motor4` 和参考四元数均可见 | 代码完成，硬件待验证 |
| 正 `roll` 扰动会产生负 roll 修正 | 把机体扰成“右侧下沉”，应看到负向纠正，并且 `M2/M3` 增、`M1/M4` 减 | 代码完成，硬件待验证 |
| 正 `pitch` 扰动会产生负 pitch 修正 | 把机体扰成“机头抬起”，应看到负向纠正，并且 `M1/M2` 增、`M3/M4` 减 | 代码完成，硬件待验证 |
| 本阶段 yaw 不进入姿态外环验收 | 确认 `rate_sp_yaw` 保持零路径，且不作为姿态保持验收项 | 代码完成，硬件待验证 |
| Trip 保护能停掉模式 | 在受限台架上谨慎超过 `attitude_trip_deg`，并确认该模式退出且外环请求被清零 | 代码完成，硬件待验证 |
| 停机条件会清除残留外环输出 | 触发 `kill`、`attitude-test stop`、IMU 失新、failsafe 或 arm 异常后，确认没有残留 rate setpoint | 代码完成，硬件待验证 |
