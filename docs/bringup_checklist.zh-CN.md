# Bring-up 清单

**语言 / Language:** [English](./bringup_checklist.md) | **简体中文**

这份清单必须在底层框架阶段和 estimator / controller 阶段之间执行。

## 闸门规则

在下面所有硬件验证项标记为 `PASS` 之前，不允许进入 PID 扩展、angle 模式工作或自由飞测试。

## 当前阶段状态

- 本清单中所有 bring-up 项目的固件和 CLI 支持都已实现
- 主机构建和主机侧 contract tests 通过
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
