# Python GUI 界面规划

**语言 / Language：** [English](./python_gui_ui_plan.md) | **简体中文**

## 目标

把 GUI 重构成在 Windows 全屏环境下真正可用的台架调试工作台：

- 更大的主图区域
- 更少的重复状态展示
- 更高密度的参数编辑
- 更紧凑的低频控制侧栏
- 以中文优先、英文回退的 UI

ownership 模型不变：

- session 与 command flow：`esp_drone_cli.core.device_session.DeviceSession`
- protocol：`esp_drone_cli.core.protocol.*`
- transport：`esp_drone_cli.core.transport.*`
- telemetry 与参数模型：`esp_drone_cli.core.models`

GUI 必须继续调用 `DeviceSession`，不能自行组帧、直接访问 transport，或引入第二套 client / session 抽象。

## 范围边界

本轮 GUI 会修改：

- GUI 布局和信息架构
- GUI 侧绘图和本地状态持久化
- GUI 本地化
- README、GUI 文档和 GUI smoke tests

本轮 GUI 不修改：

- firmware
- core protocol
- transport 实现
- `DeviceSession`
- CLI 命令名

## 布局方向

采用“三列工作台 + 紧凑底部日志”的结构：

```text
+-------------------------------------------------------------------------------------------+
| Left rail              | Center main workspace                    | Right info + params   |
|                        |                                           |                       |
| Connection             | Large main chart                          | Status cards          |
| Safety                 |                                           |                       |
| Debug actions          |                                           | Parameter table       |
|                        |                                           | + compact editor      |
|                        | Live telemetry table                      |                       |
+-------------------------------------------------------------------------------------------+
| Bottom log: recent result + events + last log path + last error (collapsible)            |
+-------------------------------------------------------------------------------------------+
```

## 区域职责

### 左列

只保留低频动作：

- 传输方式选择
- serial 或 UDP 配置
- connect / disconnect
- arm、disarm、kill、reboot
- `calib gyro`、`calib level`
- motor test
- rate test
- CSV 日志控制

左侧栏应保持视觉上偏窄，避免抢占主图宽度。

### 中列

这是主要台架区域，必须得到最多宽度。

使用一个大型主图配合图表组切换，而不是多个等尺寸小图。

支持的图表组：

- gyro
- attitude
- motors
- battery

控制项：

- pause / resume
- clear
- auto scale
- reset view
- `5s` / `10s` / `30s` 时间窗
- 每通道可见性勾选框

实时数值表保留在主图下方，不再藏在独立 tab 里。

### 右列

上半部分：

- 关键状态卡片
  - `arm_state`
  - `failsafe_reason`
  - `control_mode`
  - `imu_mode`
  - `stream`
  - `battery_voltage`
  - `imu_age_us`
  - `loop_dt_us`

下半部分：

- 参数搜索
- 参数表
- 当前值 / 新值
- set、save、reset、import、export
- 紧凑说明区

参数表应占据主要垂直空间，详情面板保持紧凑。

### 底部日志

默认高度应更小，并支持折叠 / 展开。

显示内容：

- 最近命令结果
- 事件日志
- 最近日志路径
- 最近错误

## 信息收敛规则

- 不要在顶部状态条和侧边栏重复展示同一状态
- 连接状态只保留在连接区域
- 运行时状态只保留在右侧状态卡片
- 不要把 stream 状态分散到多个无关控件中

## 本地化计划

默认 UI 语言：

- 中文

支持切换：

- `中文`
- `English`

通过 GUI 翻译表统一覆盖：

- 分组标题
- 按钮文本
- 标签
- 占位提示
- 台架警告
- 状态文本
- 常见错误
- 最近结果文本

## 本地状态

使用 `QSettings` 保存：

- 窗口几何信息
- 主、中央、右侧、参数区和底部 splitter 状态
- 上次链路类型
- 上次串口
- 上次 UDP 主机 / 端口
- 上次图表组
- 上次图表时间窗
- 上次参数过滤文本
- 上次 CSV 路径
- 上次语言
- 日志折叠 / 展开状态

## 本轮验收标准

- 图表区域足够用于受限台架波形观察
- 实时数值表与主图可同时可见
- 参数编辑密度比旧版更高
- serial 模式只显示 serial 控件
- UDP 模式只显示 UDP 控件
- 中文是默认 GUI 语言
- CLI 兼容性保持不变
- GUI 与 CLI 继续共享 `DeviceSession`
