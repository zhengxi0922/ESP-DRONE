# Python 工具 GUI + CLI 重构计划

**语言 / Language：** [English](./python_tool_gui_refactor_plan.md) | **简体中文**

## 目标

将 `tools/esp_drone_cli/` 从“CLI 优先”工具升级为共享的 `core + cli + gui` 结构，并把 GUI 从 `PySide6` 改为 `PyQt5`。

约束如下：

- 尽量保持现有 CLI 命令风格稳定
- 保持 `DeviceSession` 作为唯一会话与命令 owner
- 协议和传输实现都收拢到 `core/`
- GUI 只作为交互外壳，而不是第二套协议栈
- 自动化和可重复检查仍然坚持 CLI 优先

## 当前状态

### Phase B

Phase B 已经在代码中完成。

- `esp_drone_cli.core.device_session.DeviceSession` 是唯一的会话与命令 owner
- `esp_drone_cli.core.protocol.*` 是唯一的协议 owner
- `esp_drone_cli.core.transport.*` 是唯一的传输 owner
- `esp_drone_cli.core.models` 是唯一的 telemetry 与参数快照 owner
- CLI 直接调用 `DeviceSession`
- GUI 直接调用 `DeviceSession`
- 顶层旧的 `client.py`、`protocol/*` 和 `transport/*` 模块都只是兼容层

### Phase C

Phase C 的最小 GUI 已经基于同一个 `DeviceSession` 实现。

- GUI 只通过调用 `DeviceSession` 连接 serial 或 UDP
- GUI 不拥有 framing、transport 或命令编码
- GUI 已支持 connect / disconnect、stream 控制、telemetry、参数操作、`arm`、`disarm`、`kill`、`reboot`、`motor-test`、校准、`rate-test`、CSV logging 和 CSV dump

### Phase D

Phase D 聚焦于收口和人工调试可用性。

- README 已经说明 CLI 与 GUI 的安装和启动流程
- `docs/` 下已有 GUI 使用文档和人工检查清单
- smoke tests 已覆盖 GUI 可选依赖行为、GUI 启动 / 关闭，以及 GUI 到 session 的动作路由
- GUI 已迁移到 `PyQt5 + pyqtgraph`
- core ownership 保持不变：GUI 和 CLI 仍共享同一个 `DeviceSession`

## 计划中的包结构

```text
tools/esp_drone_cli/esp_drone_cli/
|- core/
|  |- protocol/
|  |- transport/
|  |- device_session.py
|  |- models.py
|  `- csv_log.py
|- cli/
|  `- main.py
|- gui/
|  `- main_window.py
|- __main__.py
|- gui_main.py
|- client.py
|- protocol/
`- transport/
```

## 文件迁移与兼容策略

真正的实现都放在 `core/`。现有顶层模块只保留为兼容层。

| 当前文件 | 新 owner | 兼容规则 |
|---|---|---|
| `esp_drone_cli/protocol/messages.py` | `esp_drone_cli/core/protocol/messages.py` | 旧路径重新导出新符号 |
| `esp_drone_cli/protocol/framing.py` | `esp_drone_cli/core/protocol/framing.py` | 旧路径重新导出新符号 |
| `esp_drone_cli/transport/serial_link.py` | `esp_drone_cli/core/transport/serial_link.py` | 旧路径重新导出新类 |
| `esp_drone_cli/transport/udp_link.py` | `esp_drone_cli/core/transport/udp_link.py` | 旧路径重新导出新类 |
| `esp_drone_cli/client.py` | `esp_drone_cli/core/device_session.py` + `esp_drone_cli/core/models.py` | 旧模块保留为兼容 facade |
| `esp_drone_cli/__main__.py` | `esp_drone_cli/cli/main.py` | `python -m esp_drone_cli ...` 继续作为 CLI 入口 |

## 稳定的外部接口

以下接口保持稳定或刻意兼容：

- `python -m esp_drone_cli ...`
- `esp-drone-cli ...`
- `esp-drone-gui`
- serial 与 UDP 传输选择
- `connect`、`arm`、`disarm`、`kill`、`motor-test`、`axis-test`、`rate-test`、`dump-csv` 等命令名

## Core 层职责

`core/` 负责：

- 帧编码与解码
- serial 传输
- UDP 传输
- telemetry 解析
- 参数编码与解码
- 会话生命周期
- 命令与响应流
- telemetry 订阅
- CSV 日志导出

其中核心对象是 `DeviceSession`。

## 单一 Owner 表

| 关注点 | 单一 owner |
|---|---|
| 帧编码与解码 | `esp_drone_cli.core.protocol.framing` |
| 协议消息 ID 与帧类型 | `esp_drone_cli.core.protocol.messages` |
| serial 传输 | `esp_drone_cli.core.transport.serial_link` |
| UDP 传输 | `esp_drone_cli.core.transport.udp_link` |
| telemetry 载荷解码 | `esp_drone_cli.core.models` |
| 参数值编码与解码 | `esp_drone_cli.core.models` |
| JSON 参数快照导入导出 | `esp_drone_cli.core.device_session` |
| 设备命令流 | `esp_drone_cli.core.device_session` |
| stream 生命周期 | `esp_drone_cli.core.device_session` |
| CSV 日志导出 | `esp_drone_cli.core.csv_log` + `esp_drone_cli.core.device_session` |

## GUI 技术栈

- GUI 绑定：`PyQt5`
- 绘图库：`pyqtgraph`
- session bridge：`gui/main_window.py` 中的 callback 到 Qt signal 转换

如果 GUI 依赖缺失：

- CLI 仍然可以工作
- `esp-drone-gui` 会快速失败，并给出明确安装提示

## GUI 范围

当前 GUI 覆盖：

- connect / disconnect
- stream on / off
- `arm`、`disarm`、`kill`、`reboot`
- telemetry 表格
- 实时图表
- 参数刷新、设置、保存、恢复、导入和导出
- 电机测试
- 校准命令
- 速率测试
- CSV logging
- 基于 `QSettings` 的本地 UI 状态

仍然刻意延后的内容：

- RC 或 stock App 兼容工作流
- GUI 自己拥有协议逻辑
- 大型仪表盘样式和重型可视化
