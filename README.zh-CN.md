# ESP-DRONE

**语言 / Language:** [English](./README.md) | **简体中文**

这是一个面向自制四轴的 ESP-IDF 飞控固件仓库，并提供共用核心层的 Python CLI / GUI 工具链，目标平台为 `ESP32-S3-WROOM-1-N16R8`。

## 固定约束

- `ESP-IDF v5.5.1`
- 机体系固定为 `+Y` 机头、`+X` 机体右侧、`+Z` 向上
- 项目命名固定为 `+pitch` 抬头、`+roll` 右侧下沉、`+yaw` 机头右转
- `UART0` 只给 `ATK-MS901M`
- `USB CDC` 只给 CLI、GUI 和调试遥测
- CLI 与 GUI 共用同一个 `DeviceSession`

所有方向敏感逻辑都必须遵循：

- [docs/axis_truth_table.zh-CN.md](./docs/axis_truth_table.zh-CN.md)
- [docs/motor_map.zh-CN.md](./docs/motor_map.zh-CN.md)

## 仓库结构

- [docs/](./docs/README.zh-CN.md)：设计约束、bring-up 记录、CLI / GUI 使用说明
- [firmware/](./firmware)：ESP-IDF 固件工程
- [tools/esp_drone_cli/](./tools/esp_drone_cli)：共享 `core + cli + gui` Python 工具链

## 当前状态

仓库已经具备可用的三轴 rate-loop 台架路径：

- 固件支持 `roll`、`pitch`、`yaw` 三轴 `rate-test`
- 三轴 rate PID 参数已真实接入控制链
- CLI 支持 `rate-test`、`rate-status`、`watch-rate`、参数编辑、保存、导入导出，以及 `axis-bench` / `rate-bench`
- PyQt5 GUI 支持 rate-test 控件、rate 调试图表、rate PID 编辑和共享会话命令处理
- 软件验证包括固件构建成功和 Python tests

当前阶段的 roll 调试仍然只限于 bench-only 的 rate-loop：

- 不做 angle outer loop
- 这条 workflow 不做 attitude hang test
- 不做自由飞调参
- 不改现有 `+roll`、`roll_rate = -gyro_y` 和电机映射约定

当前文档记录的 live roll 会话是在圆棍受限台架上完成的，机体自然姿态为 `+Z 朝下`。
这个前提不会改变 roll rate-loop 的符号验收，因为这一步只看 `rate_setpoint_roll`、映射后的 roll 反馈、PID 输出和电机分配，不看 angle 外环。

## 固件构建

推荐 Windows 流程：

```powershell
. .\tools\esp-idf-env.ps1
.\tools\idf.ps1 set-target esp32s3
.\tools\idf.ps1 build
```

也可以直接使用原生 ESP-IDF：

```powershell
cd firmware
idf.py set-target esp32s3
idf.py build
```

## Python 工具安装

只安装 CLI：

```powershell
cd tools\esp_drone_cli
pip install -e .
```

同时安装 CLI + GUI：

```powershell
cd tools\esp_drone_cli
pip install -e .[gui]
```

## 主要文档

- [docs/roll_rate_bench_workflow.zh-CN.md](./docs/roll_rate_bench_workflow.zh-CN.md)
- [docs/roll_bench_summary_sample.md](./docs/roll_bench_summary_sample.md)
- [docs/python_cli_usage.zh-CN.md](./docs/python_cli_usage.zh-CN.md)
- [docs/bringup_checklist.zh-CN.md](./docs/bringup_checklist.zh-CN.md)
- [docs/python_gui_usage.zh-CN.md](./docs/python_gui_usage.zh-CN.md)
- [docs/rate_bringup_results.zh-CN.md](./docs/rate_bringup_results.zh-CN.md)
- [docs/README.zh-CN.md](./docs/README.zh-CN.md)
