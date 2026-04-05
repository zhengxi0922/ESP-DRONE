# 运行频率规划

**语言 / Language：** [English](./runtime_frequency_plan.md) | **简体中文**

## 默认目标

- 电机输出调度：`1000 Hz`
- IMU UART 默认回传率：`200 Hz`
- IMU UART 可选高档位：`250 Hz`
- estimator 更新：仅在 fresh IMU sample 到来时进行
- rate PID：仅在 fresh IMU sample 到来时进行
- angle PID：bring-up 闸门通过前暂缓，默认规划为 `125 Hz`
- USB CDC telemetry：`200 Hz`
- UDP telemetry：`100 Hz`

## 当前参数护栏

阶段 3 当前对 telemetry 频率参数做如下限制：

- `telemetry_usb_hz`：`1..200`
- `telemetry_udp_hz`：`1..100`

这些限制与当前 bring-up 范围一致，能把二进制 CDC/UDP telemetry 负载控制在台架测试可接受的安全范围内。

## 为什么控制路径采用事件驱动

IMU 主动上传频率最高只有 `250 Hz`。如果在 `1000 Hz` 下反复运行完整 estimator 和 rate loop，而输入仍是旧数据，不会提升可观测性，反而会让时序语义变得模糊。

因此本次重写明确分离：

- `1 kHz` 的输出与监管调度器
- 基于 fresh sample 的 estimator 和 rate 更新路径

## 飞控任务契约

`flight_control_task` 以 `1 kHz` 运行，但：

- estimator 只在收到新的 IMU sample 时更新
- rate PID 只在收到新的 IMU sample 时更新
- angle PID 不属于当前最小阶段 3 闸门
- 电机输出整形、急停、饱和、斜率限制和循环计时每个 tick 都运行

## 最小阶段 3 范围

在开始 angle-loop 之前，当前允许的闭环路径只有：

- fresh IMU sample
- estimator update
- rate PID update
- mixer update
- motor output update
- safety gating

这条路径通过 `rate-test` bring-up 命令及其 telemetry 字段暴露。本阶段不启用 angle 外环。

## 样本陈旧策略

当没有新的 IMU sample 时：

- 更新 `imu_age_us`
- 持续执行 safety 和超时检查
- 保持上一拍控制输出
- 保持输出调度继续运行

当没有新的 IMU sample 时，固件禁止：

- 重新计算完整 estimator
- 积分出一个虚构的新姿态样本
- 假装存在新的 rate 测量值
