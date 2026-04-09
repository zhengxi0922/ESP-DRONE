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

仓库现在已经具备可用的三轴 rate 闭环台架路径：

- 固件支持 `roll / pitch / yaw` 三轴 `rate-test`
- 三轴 rate PID 参数已经真实接入控制链
- CLI 支持 `rate-test`、`rate-status`、参数编辑、保存、导入、导出
- PyQt5 GUI 支持 rate-test 控件、rate 调试图表、rate PID 编辑和共享会话命令处理
- 当前软件验证已经覆盖 Python 测试和固件构建成功

本阶段仍然不做：

- angle 外环
- autotune
- altitude hold 闭环
- 自由飞行调优

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

- [docs/python_cli_usage.zh-CN.md](./docs/python_cli_usage.zh-CN.md)
- [docs/python_gui_usage.zh-CN.md](./docs/python_gui_usage.zh-CN.md)
- [docs/rate_bringup_results.zh-CN.md](./docs/rate_bringup_results.zh-CN.md)
- [docs/README.zh-CN.md](./docs/README.zh-CN.md)

