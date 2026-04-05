# Python GUI 使用说明

**语言 / Language：** [English](./python_gui_usage.md) | **简体中文**

## 目标

`esp-drone-gui` 是 Python 工具链中的人工台架调试工作台。

它与 CLI 共用同一个 `esp_drone_cli.core.device_session.DeviceSession`，因此 GUI 与 CLI 共享：

- framing
- serial transport
- UDP transport
- telemetry 解码
- 参数命令
- 设备命令
- CSV logging

GUI 面向人工参与的台架调试。自动化和脚本测试仍以 CLI 为首选入口。

## 在 Windows 上安装

仅安装 CLI：

```powershell
cd tools\esp_drone_cli
pip install -e .
```

安装 CLI + GUI：

```powershell
cd tools\esp_drone_cli
pip install -e .[gui]
```

`gui` extra 会安装：

- `PyQt5`
- `pyqtgraph`

如果缺少 `PyQt5` 或 `pyqtgraph`：

- `esp-drone-cli` 仍可运行
- `esp-drone-gui` 会退出并给出明确安装提示

## 启动 GUI

安装后的入口：

```powershell
esp-drone-gui
```

模块入口：

```powershell
python -m esp_drone_cli.gui_main
```

## 窗口布局

GUI 采用“三列工作台 + 可折叠底部日志”的结构：

- 左列，较窄：
  - 连接
  - 安全控制
  - 调试动作
- 中列，最大：
  - 一张大型实时图表
  - 下方实时数值 telemetry 表
- 右列：
  - 关键状态卡片
  - 大型参数表和紧凑编辑 / 详情区
- 底部日志：
  - 最近事件
  - 最近结果
  - 最近日志路径
  - 最近错误

图表和实时数值 telemetry 可以同时显示，不再被拆成互斥 tab。

## 语言

- 默认 UI 语言：中文
- 右上角语言切换：`中文 / English`

主要控件、标签、分组标题和状态文本都做了面向台架使用的本地化。

## Serial 连接示例

1. 将飞行器接入 USB CDC。
2. 启动 GUI。
3. 在 `Connection` 区域：
   - 选择 `Serial`
   - 选择 COM 端口，或手动输入
   - 除非固件后续调整，否则波特率保持 `115200`
4. 点击 `Connect`。

预期结果：

- 连接状态标记切换为已连接
- 参数列表自动刷新
- 点击 `Stream On` 后开始刷新 telemetry

## UDP 连接示例

1. 给飞行器上电，并确认 UDP 端点可达。
2. 在 `Connection` 区域：
   - 选择 `UDP`
   - 设置主机地址，例如 `192.168.4.1`
   - 设置端口，默认 `2391`
3. 点击 `Connect`。

预期结果：

- 连接状态标记切换为已连接
- 命令通过与 serial 相同的 `DeviceSession` 路径发送

## GUI 区域说明

### 左列

连接区：

- 选择 serial 或 UDP
- serial 模式只显示 COM 和波特率控件
- UDP 模式只显示 host 和 port 控件
- 刷新串口列表
- connect / disconnect
- 查看当前 session 信息与最近连接错误

安全区：

- `Arm`
- `Disarm`
- `Kill`
- `Reboot`

调试动作区：

- `motor_test`
- `calib gyro`
- `calib level`
- `rate_test`
- `Start Log`
- `Stop Log`
- `Dump CSV`

这些动作仅用于受限台架环境。

### 中列

主图区域：

- 一张基于 `pyqtgraph` 的大型图
- 可切换图表组：
  - `Gyro`
  - `Attitude`
  - `Motors`
  - `Battery`
  - `Barometer`
- pause / resume
- clear history
- auto scale
- reset view
- `5s` / `10s` / `30s` 时间窗
- 每通道可见性勾选框

实时数值 telemetry：

- `Stream On` / `Stream Off`
- 通过 `telemetry_usb_hz` 或 `telemetry_udp_hz` 调整目标 telemetry 频率
- 查看 gyro、姿态、rate setpoint、电机、电池、barometer、循环计时和安全状态等实时字段
- 支持用 `Ctrl+C` 复制选中表格单元格

### 右列

关键状态卡片：

- `arm_state`
- `failsafe_reason`
- `control_mode`
- `imu_mode`
- `stream`
- `battery_voltage`
- `baro_altitude_m`
- `baro_health`
- `imu_age_us`
- `loop_dt_us`

参数调试区：

- 刷新参数列表
- 按参数名搜索
- 选中单个参数并编辑新值
- 查看本地提示和紧凑说明区
- `Save`
- `Reset`
- `Export JSON`
- `Import JSON`

GUI 中的提示只作为辅助，最终校验仍由设备端完成。

### 底部日志

- 用于显示最近命令结果和错误的事件日志区
- 支持 clear、copy、save log
- 默认折叠为较低高度
- 可通过箭头按钮展开或折叠

## QSettings 状态

GUI 会通过 `QSettings` 保存本地状态，包括：

- 窗口几何信息
- splitter 位置
- 上次链路类型
- 上次 serial 端口
- 上次 UDP host / port
- 上次图表组
- 上次图表时间窗
- 上次参数搜索文本
- 上次 CSV 输出路径
- 上次所选语言
- 底部日志折叠 / 展开状态

## 常见错误

`PyQt5 and pyqtgraph are required for esp-drone-gui`

- 使用 `pip install -e .[gui]` 安装 GUI 依赖

No serial port selected

- 连接前先选择或输入有效 COM 端口

No UDP host provided

- 连接前输入有效 host

`HELLO` timeout 或连接后立刻断开

- 检查线缆质量
- 确认板子运行的是应用程序，而不是 ROM downloader
- 确认没有其它程序占用串口

连接后没有 telemetry 更新

- 点击 `Stream On`
- 确认连接的是正确传输方式
- 确认固件正在运行，且未处于 reset 或 fault

`set_param` 或 `save_params` 失败

- 设备端参数校验拒绝了该值
- 根据固件文档检查参数范围和参数间约束

`dump csv` 失败

- 检查目标路径
- 确认设备已连接且能够启动数据流

## 范围边界

当前 GUI 范围刻意聚焦于：

- 适合用于台架连接、安全命令、telemetry、图表、参数、电机 / 速率测试和 CSV 采集
- 不是第二套协议栈
- 不是自动化首选入口
- 暂时也不是重型波形分析平台
