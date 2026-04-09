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

工作台继续保持“三栏 + 底部可折叠日志”的布局：

- 左侧：
  - 连接
  - 安全控制
  - 调试动作
- 中间：
  - 大实时图表
  - 图表下方实时数值 telemetry
- 右侧：
  - 关键状态卡片
  - 参数搜索、编辑、保存、恢复、导入、导出
- 底部日志：
  - 最近命令结果
  - 最近错误
  - 最近结果
  - 最近日志路径

图表和数值 telemetry 会同时可见，不再拆成互斥 tab。

## 语言

- 默认语言：中文
- 右上角切换：`中文 / English`

## 连接约束

当前台架 bring-up 建议走 USB CDC：

- `UART0` 继续只给 `ATK-MS901M`
- `USB CDC` 继续只给 CLI、GUI 和调试遥测

典型串口连接流程：

1. 选择 `Serial`
2. 选择 COM 口
3. 波特率保持 `115200`
4. 点击 `Connect`

## Rate Test 控件

左侧 `Debug Actions` 区现在已经具备完整的三轴 rate test 控件：

- 轴选择：`roll / pitch / yaw`
- 数值输入：目标角速度 dps
- `Start`：发送 `rate-test`
- `Stop`：发送 `rate-test <axis> 0`
- 明确台架警告：仅限拆桨或严格约束状态

命令结果和命令失败都会进入底部日志。GUI 不再把设备端拒绝误当成成功。

## Rate 调试图表

中间图表组现在包含：

- `Rate Roll`
- `Rate Pitch`
- `Rate Yaw`

每个图表组都直接服务于对应轴的台架调试：

- `Rate Roll`：
  - `gyro_y`
  - `rate_setpoint_roll`
  - `pid_out_roll`
  - `motor1..motor4`
- `Rate Pitch`：
  - `gyro_x`
  - `rate_setpoint_pitch`
  - `pid_out_pitch`
  - `motor1..motor4`
- `Rate Yaw`：
  - `gyro_z`
  - `rate_setpoint_yaw`
  - `pid_out_yaw`
  - `motor1..motor4`

实时数值表也包含以下调试字段：

- `rate_pid_p_*`
- `rate_pid_i_*`
- `rate_pid_d_*`
- `pid_out_*`
- `imu_age_us`
- `loop_dt_us`

## 参数调试

右侧参数区可直接用于 rate PID 调试：

- 搜索 `rate_`
- 编辑：
  - `rate_kp_*`
  - `rate_ki_*`
  - `rate_kd_*`
  - `rate_integral_limit`
  - `rate_output_limit`
- 写入后可立刻在 telemetry 和图表里观察变化
- 保留：
  - `Save`
  - `Reset`
  - `Export JSON`
  - `Import JSON`

GUI 参数通路仍然完全复用共享 `DeviceSession`。

## 推荐 GUI 台架流程

1. 拆桨或把机体完全约束。
2. 用 `Serial` 连接。
3. 点击 `Stream On`。
4. 在图表组切换到 `Rate Roll`、`Rate Pitch` 或 `Rate Yaw`。
5. 只有在台架安全条件满足时才解锁。
6. 在左侧 rate test 面板选择轴，从保守值开始点击 `Start`。
7. 观察对应的 gyro、setpoint、`pid_out` 和电机分配。
8. 在右侧搜索 `rate_`，小步修改 PID。
9. 测试结束后点击 `Stop`，再 `Disarm`。

## 常见错误

以下问题现在都会明确显示在事件日志里：

- 设备未连接
- 非零 `rate-test` 需要先 arm
- `motor_test` 或 `axis_test` 需要先 disarm
- IMU 未就绪
- 参数值被固件校验拒绝

## 范围边界

当前 GUI 仍然只覆盖以下范围：

- 适合台架连接、遥测、图表、rate test、参数调试和 CSV 采集
- 不引入第二套主机协议栈
- 不是自动化入口
- 不做 angle mode 或 autotune 界面

