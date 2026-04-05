# RAW 模式姿态方案

**语言 / Language：** [English](./raw_mode_attitude_plan.md) | **简体中文**

## 目标

为 `IMU_MODE_RAW` 提供项目自有的姿态来源，使上层在两种模式下都能消费同一个 `imu_sample_t` 契约：

- `timestamp_us`
- `gyro_xyz_dps`
- `acc_xyz_g`
- `quat_wxyz`
- `roll_pitch_yaw_deg`
- `health`
- `update_age_us`

## 计划中的估计器

计划中的 RAW 模式姿态解算器是运行在 ESP32-S3 上的 Mahony 风格四元数滤波器。

原因：

- 计算开销低
- 能为项目机体系提供稳定的四元数输出
- 陀螺仪与加速度计融合路径直接
- 可在不改变上层接口的前提下增加磁力计修正路径

## 计划中的更新频率

- RAW 模式估计器只在 fresh IMU sample 到来时运行
- 默认 IMU 回传率保持 `200 Hz`
- 可选高档位保持 `250 Hz`
- 在没有 fresh IMU 数据时，估计器不会假装以 `1 kHz` 运行

这与 [runtime_frequency_plan.zh-CN.md](./runtime_frequency_plan.zh-CN.md) 的约束保持一致。

## 输入

必需输入：

- 来自统一 IMU 映射入口的、映射到机体系后的陀螺仪数据
- 来自同一映射入口的、映射到机体系后的加速度计数据

可选输入：

- 只有在明确启用并完成验证后，才接入映射后的磁力计数据

## 磁力计策略

- 早期 bring-up 阶段默认关闭磁力计
- 后续启用时，也只用于慢速 yaw 修正
- 磁力计数据进入估计器前必须通过模长和新鲜度检查

## Yaw 漂移策略

没有磁力计时：

- yaw 通过陀螺积分获得
- roll 和 pitch 仍由重力参考修正
- 航向可观测性会下降
- 如果姿态输出依赖会漂移的 yaw 源，`imu_health` 应报告 `DEGRADED`，而不是 `OK`

有已验证磁力计时：

- yaw 修正增益应低于 roll 和 pitch 修正增益
- 必须通过创新门限拒绝突跳 yaw 修正

## 标定方案

- 陀螺零偏标定：在显式 `calib gyro` 期间做静止平均
- 水平标定：为加速度计或姿态零点提供可选的机体系 trim
- 这些标定必须位于模块到机体系映射之后，保证项目坐标系始终是单一真源

## 模式切换规则

- `IMU_MODE_DIRECT`：使用模块直出的四元数或姿态，再做项目映射
- `IMU_MODE_RAW`：使用 ESP32 侧 Mahony 输出，再做项目映射与标定
- 一旦填充完 `imu_sample_t`，上层不得再根据来源分支处理

## Bring-up 规则

在设计或启用 angle 模式之前：

- 必须先在台架上验证 direct 模式的符号方向
- 必须把 RAW 模式 Mahony 输出与 direct 模式进行台架对照
- 必须明确接受当前测试范围下“无磁力计时 yaw 会漂移”的行为

## 当前阶段边界

这份文档在当前轮次中仅代表设计方案。

- 目前尚未实现完整的 RAW 模式姿态估计器
- 不能仅凭这份文档就开始 angle PID 工作
- 当前控制阶段的直接重点仍然是单轴 rate-loop 验证
