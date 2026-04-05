# Bring-up 清单

**语言 / Language：** [English](./bringup_checklist.md) | **简体中文**

这份清单必须在底层框架阶段和 estimator / controller 阶段之间执行。

## 闸门规则

在下面所有硬件验证项都标记为 `PASS` 之前，不允许进入 PID 调参、angle 模式工作或自由飞测试。

## 当前阶段状态

- 本清单中的所有 bring-up 项，固件和 CLI 支持都已经实现
- 主机构建与主机侧契约测试通过
- 在 `2026-04-01`，本工作站执行 `python -m serial.tools.list_ports -v` 时返回 `no ports found`，因此当次会话没有完成任何硬件 bring-up 项

## 连通性

| 项目 | 可复现命令或证据 | 当前状态 |
|---|---|---|
| USB CDC 二进制握手 | `esp-drone-cli --serial COMx connect` | 代码完成，硬件待测 |
| IMU UART 有效帧接收 | `esp-drone-cli --serial COMx stream on`，观察 `imu seq/good/err/age_us` 事件 | 代码完成，硬件待测 |
| LED 状态机切换 | 启动和命令测试期间观察 `INIT_WAIT_IMU`、`DISARMED_READY`、`FAULT_LOCK` 路径 | 代码完成，硬件待测 |

## 供电与 ADC

| 项目 | 可复现命令或证据 | 当前状态 |
|---|---|---|
| 可读取 `BAT_ADC` 原始值 | telemetry 字段 `battery_adc_raw` 和 `battery_voltage` | 代码完成，硬件待测 |
| 分压换算有效 | 使用 `100k / 100k` 分压，将 telemetry 电压与万用表对比 | 代码完成，硬件待测 |
| telemetry 中能看到电池阈值相关状态 | 检查 `battery_voltage`、`arm_state` 和 `failsafe_reason` | 代码完成，硬件待测 |

## 电机与安全

| 项目 | 可复现命令或证据 | 当前状态 |
|---|---|---|
| `motor_test m1..m4` 只驱动单个电机 | `esp-drone-cli --serial COMx motor-test m1 0.12`，并对 `m2..m4` 重复 | 代码完成，硬件待测 |
| `M1..M4` 顺序与文档一致 | 按顺序运行 `motor-test`，并与 [motor_map.zh-CN.md](./motor_map.zh-CN.md) 对照 | 代码完成，本次会话未做硬件实测 |
| `arm / disarm / kill` 可通过 CLI 生效 | 运行 `arm`、`disarm` 和 `kill`，观察 `arm_state` 与 `failsafe_reason` | 代码完成，硬件待测 |
| 摇杆手势解锁 / 上锁 | 等阶段 4 存在 RC 输入后再验证 | 当前不在范围内 |

## 方向验证

| 项目 | 可复现命令或证据 | 当前状态 |
|---|---|---|
| 物理动作与 telemetry 符号约定一致 | 手持机体，观察 `gyro_x/y/z`、`roll/pitch/yaw`，并与 [axis_truth_table.zh-CN.md](./axis_truth_table.zh-CN.md) 对照 | 代码完成，本次会话未做硬件实测 |
| `+roll` 使 `M1` 和 `M4` 增大 | 未解锁时执行 `axis-test roll 0.05`，检查电机输出和 telemetry | 代码完成，本次会话未做硬件实测 |
| `+pitch` 使 `M3` 和 `M4` 增大 | 未解锁时执行 `axis-test pitch 0.05`，检查电机输出和 telemetry | 代码完成，本次会话未做硬件实测 |
| `+yaw` 使 `M1` 和 `M3` 增大 | 未解锁时执行 `axis-test yaw 0.05`，检查电机输出和 telemetry | 代码完成，本次会话未做硬件实测 |
| 每一轴的最小闭环 rate 响应有效 | `arm` 后执行 `rate-test roll|pitch|yaw <dps>`，检查 `rate_setpoint_*`、`pid_out_*` 和电机输出 | 代码完成，本次会话未做硬件实测 |
