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

仓库现在有两条“受限台架”控制路径：

- 已可用的 `roll / pitch / yaw` 三轴 rate 闭环台架路径
- 新增的圆棍 / 吊架专用姿态外环 bring-up 路径，面向自然平衡姿态为 `+Z 朝下` 的受限台架

新的姿态外环 bring-up 明确受限于以下范围：

- 固件新增 `CONTROL_MODE_ATTITUDE_HANG_TEST`
- 进入前必须显式执行 `attitude-capture-ref`
- 控制误差使用相对四元数 `q_rel = q_ref^-1 * q_now`，而不是拿全局 `roll=0 / pitch=0` 直接相减
- 这一轮只做 `roll / pitch` 外环
- 外环先做 P-only，再把结果喂给现有 rate 内环
- 推力保持开环固定 `attitude_test_base_duty`

这不是自由飞 stabilize，也不是 angle-ready 模式。

`CONTROL_MODE_ATTITUDE_HANG_TEST` 仅用于受限台架，绝不能直接用于带桨自由飞。

本阶段仍然不做：

- 自由飞 stabilize / angle 调参
- yaw heading hold
- altitude hold 闭环
- auto takeoff
- autotune

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

- [docs/hang_attitude_bringup_plan.zh-CN.md](./docs/hang_attitude_bringup_plan.zh-CN.md)
- [docs/python_cli_usage.zh-CN.md](./docs/python_cli_usage.zh-CN.md)
- [docs/python_gui_usage.zh-CN.md](./docs/python_gui_usage.zh-CN.md)
- [docs/bringup_checklist.zh-CN.md](./docs/bringup_checklist.zh-CN.md)
- [docs/python_gui_manual_checklist.zh-CN.md](./docs/python_gui_manual_checklist.zh-CN.md)
- [docs/rate_bringup_results.zh-CN.md](./docs/rate_bringup_results.zh-CN.md)
- [docs/README.zh-CN.md](./docs/README.zh-CN.md)
