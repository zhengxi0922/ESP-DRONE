# Python GUI 使用说明

**语言 / Language:** [English](./python_gui_usage.md) | **简体中文**

## 目标

`esp-drone-gui` 是 Python 工具链中面向人工台架调试的工作台。

它与 CLI 共用同一个 `esp_drone_cli.core.device_session.DeviceSession`，因此 GUI 和 CLI 共用：

- framing
- serial / UDP 传输
- telemetry 解码
- 参数命令
- 设备命令
- CSV 日志

GUI 仍然只是台架工作台，不是第二套协议栈。

## 范围警告

新的 `Hang Attitude` 控件只用于受限圆棍或吊架台架。

它不是自由飞 stabilize 界面。
它不是 angle-ready 界面。
绝不能直接用于带桨自由飞。

## 安装

```powershell
cd tools\esp_drone_cli
pip install -e .[gui]
```

如果只需要 CLI：

```powershell
cd tools\esp_drone_cli
pip install -e .
```

## 启动

```powershell
esp-drone-gui
```

或：

```powershell
python -m esp_drone_cli.gui_main
```

## 窗口布局

工作台继续保持“三列 + 底部可折叠日志”的布局：

- 左侧：
  - 连接
  - 安全控制
  - 调试动作
- 中间：
  - 大实时图表
  - 图表下方实时数字 telemetry
- 右侧：
  - 关键状态卡片
  - 参数搜索、编辑、保存、重置、导入、导出
- 底部日志：
  - 最近命令结果
  - 最近错误
  - 最近结果
  - 最近日志路径

图表和数字 telemetry 会同时可见。

## 语言

- 默认 UI 语言：中文
- 语言切换：`中文 / English`

## 连接约束

当前 bring-up 建议使用 USB CDC：

- `UART0` 继续专用于 `ATK-MS901M`
- `USB CDC` 继续专用于 CLI、GUI 和调试遥测

典型串口连接流程：

1. 选择 `Serial`。
2. 选择 COM 口。
3. 波特率保持 `115200`。
4. 点击 `Connect`。

## Debug Actions

左侧 `Debug Actions` 现在同时覆盖原有 rate-test 路径和新的 bench-only 圆棍姿态外环路径。

Rate-test 控件：

- 轴选择：`roll / pitch / yaw`
- 数值输入：目标角速度 dps
- `Start`：发送 `rate-test`
- `Stop`：发送 `rate-test <axis> 0`

姿态外环控件：

- `Capture Ref`
- `Attitude Test Start`
- `Attitude Test Stop`
- `Base Duty`
- 以下参数的快捷编辑框：
  - `attitude_kp_roll`
  - `attitude_kp_pitch`
  - `attitude_rate_limit_roll`
  - `attitude_rate_limit_pitch`
  - `attitude_error_deadband_deg`
  - `attitude_trip_deg`

GUI 会明确把这一区域标注为“受限台架专用，不是自由飞模式”。

命令结果和命令失败都会进入底部事件日志。

## 图表

图表组选择器现在包括：

- `Rate Roll`
- `Rate Pitch`
- `Rate Yaw`
- `Hang Attitude Roll`
- `Hang Attitude Pitch`

姿态外环图表用于这一轮 outer-loop bring-up：

- `Hang Attitude Roll`：
  - `attitude_err_roll_deg`
  - `attitude_rate_sp_roll`
  - `pid_out_roll`
  - `motor1..motor4`
- `Hang Attitude Pitch`：
  - `attitude_err_pitch_deg`
  - `attitude_rate_sp_pitch`
  - `pid_out_pitch`
  - `motor1..motor4`

telemetry 表也包含：

- `attitude_ref_valid`
- `attitude_ref_qw`
- `attitude_ref_qx`
- `attitude_ref_qy`
- `attitude_ref_qz`
- `base_duty_active`
- 既有的 `pid_out_*`、`imu_age_us` 和 `loop_dt_us`

建议按下面这条链路确认符号：

```text
手动扰动 -> 姿态误差 -> 外环 rate setpoint -> PID 输出 -> 电机混控
```

## 参数调试

右侧参数区可直接用于台架调试。

rate 环参数仍然可用：

- `rate_kp_*`
- `rate_ki_*`
- `rate_kd_*`
- `rate_integral_limit`
- `rate_output_limit`

姿态外环参数也已经接入：

- `attitude_kp_roll`
- `attitude_kp_pitch`
- `attitude_rate_limit_roll`
- `attitude_rate_limit_pitch`
- `attitude_error_deadband_deg`
- `attitude_trip_deg`
- `attitude_test_base_duty`
- 只读运行态 `attitude_ref_valid`

GUI 参数路径仍然完全复用共享 `DeviceSession`。

## 推荐 GUI 台架流程

针对圆棍姿态外环 bring-up：

1. 拆桨，并确保机体受限固定在圆棍或吊架上。
2. 通过 `Serial` 连接。
3. 点击 `Stream On`。
4. 先确认 IMU 新鲜度和基础手持运动 telemetry 正常。
5. 让机体回到自然悬挂平衡姿态，点击 `Capture Ref`。
6. 确认 `attitude_ref_valid` 变为 true。
7. 只有在台架安全时才 arm。
8. 保持保守的 `Base Duty`、`attitude_kp_*` 和 `attitude_rate_limit_*`。
9. 点击 `Attitude Test Start`。
10. 选择 `Hang Attitude Roll` 或 `Hang Attitude Pitch` 图表。
11. 施加小扰动并检查纠正方向：
    `+roll` 扰动必须让 `M2/M3` 增、`M1/M4` 减。
    `+pitch` 扰动必须让 `M1/M2` 增、`M3/M4` 减。
12. 如果符号链路错误、触发 trip 或台架不稳定，立即点击 `Attitude Test Stop`。
13. 结束后 `Disarm`。

## 常见错误

以下问题现在都会明确显示在事件日志里：

- 设备未连接
- `Attitude Test Start` 需要先 arm
- 参考姿态未捕获
- IMU 未就绪或不新鲜
- 超过 trip 限幅
- 参数值被固件校验拒绝

## 范围边界

当前 GUI 仍然只覆盖以下范围：

- 适合受限台架连接、遥测、图表、rate test、圆棍姿态外环 bring-up、参数调试和 CSV 采集
- 不引入第二套主机协议栈
- 不是自动化入口
- 不做自由飞 stabilize、angle 或 autotune 界面
