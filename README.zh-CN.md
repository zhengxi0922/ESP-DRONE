# ESP-DRONE

**语言 / Language：** [English](./README.md) | **简体中文**

这是一个面向自制四轴无人机的从零重写飞控工程，硬件平台为 `ESP32-S3-WROOM-1-N16R8`，同时提供配套的 ESP-IDF 固件和 Python 工具链。

## 项目概览

仓库当前包含：

- 目标板卡的 ESP-IDF 飞控固件
- 位于 [`docs/`](./docs/README.zh-CN.md) 的设计约束、bring-up 和验收文档
- 同时提供 CLI 与 GUI 的共享 Python 工具链

当前工程的核心约束如下：

- 固件与文档统一锁定 `ESP-IDF v5.5.1`
- 机体系固定为 `+Y` 机头、`+X` 机体右侧、`+Z` 朝上
- 姿态正方向固定为 `+pitch` 抬头、`+roll` 右侧下沉、`+yaw` 机头右转
- `UART0` 只给 `ATK-MS901M`
- `USB CDC` 只给调试日志和 CLI 协议
- Python CLI 和 GUI 共用同一个 `DeviceSession`

## 固定环境

- MCU：`ESP32-S3-WROOM-1-N16R8`
- ESP-IDF：`v5.5.1`
- RTOS：ESP-IDF FreeRTOS
- IMU 链路：`UART0 -> ATK-MS901M`
- console 与 CLI 链路：`USB CDC`

## 坐标与姿态约定

整个工程使用固定机体系：

- `+Y`：机头 / 前方
- `+X`：机体右侧
- `+Z`：朝上

整个工程使用固定姿态正方向命名：

- `+pitch`：抬头
- `+roll`：右侧下沉
- `+yaw`：机头右转，俯视顺时针

机体系本身是右手系，但项目中 `roll` 与 `yaw` 的正方向命名不等同于默认数学正旋转。所有方向敏感实现都必须遵循：

- [`docs/axis_truth_table.zh-CN.md`](./docs/axis_truth_table.zh-CN.md)
- [`docs/motor_map.zh-CN.md`](./docs/motor_map.zh-CN.md)

## 仓库结构

- [`docs/`](./docs/README.zh-CN.md)：设计约束、协议提取、bring-up 记录和工具说明
- [`firmware/`](./firmware)：ESP-IDF 固件工程
- [`tools/esp_drone_cli/`](./tools/esp_drone_cli)：共享 `core + cli + gui` Python 工具链

## 构建说明

本仓库固定使用 `ESP-IDF v5.5.1`。

Windows 环境下，推荐先阅读 [`docs/esp_idf_env.zh-CN.md`](./docs/esp_idf_env.zh-CN.md)，然后使用仓库自带脚本：

```powershell
. .\tools\esp-idf-env.ps1
.\tools\idf.ps1 build
```

全新检出后，首次构建前先设置目标：

```powershell
.\tools\idf.ps1 set-target esp32s3
```

如果你更习惯原生 ESP-IDF 流程，也可以：

```powershell
cd firmware
idf.py set-target esp32s3
idf.py build
```

## 当前状态

仓库目前已经具备：

- 阶段 1 设计文档
- 阶段 2 底层框架与基础模块
- 阶段 2.5 的 `motor-test`、`axis-test`、`rate-test`、IMU 映射、mixer 方向校验与结构化 telemetry 路径
- 最小单轴 `rate-loop` 骨架，包括 fresh-sample estimator update、rate PID、mixer、电机输出和安全门控
- 共享 Python `core`，负责协议、传输、遥测解码、参数快照和设备命令
- 面向自动化和脚本检查的 CLI
- 面向人工台架调试的 PyQt5 GUI
- 面向 `ATK-MS901M` 的第一阶段气压计数据链路

当前验证状态：

- 主机构建与主机侧方向测试通过
- 串口 CLI 的当前非起飞联机验证已经有记录
- 完整的受限台架硬件验证仍按阶段推进

后续阶段仍待完成：

- 更完整的 safety / failsafe 策略细化
- 单轴速率模式的完整硬件 bring-up
- bring-up 闸门通过后的 direct 模式 angle 外环工作
- 更完整的 RC、legacy UDP 和非台架工作流覆盖

## 气压计框架

仓库已经接入 `ATK-MS901M` 的第一阶段气压计数据链路。

当前范围：

- 在固件中解析模块气压计帧
- 通过 USB CDC 输出气压计 telemetry
- 在 CLI、GUI 和 CSV 导出中显示气压计字段
- 预留定高模式所需的状态结构与控制模式枚举

当前尚未纳入范围：

- 定高 PID
- 油门闭环
- 自动起降
- 将气压计数据接入现有姿态或速率控制链路

详情见 [`docs/barometer_framework.zh-CN.md`](./docs/barometer_framework.zh-CN.md)。

## Python 工具链

Python 工具链同时支持：

- CLI：面向自动化、脚本化回归和批量导出
- GUI：面向人工台架调试

两者共用同一个 `DeviceSession`，不会维护两套协议或传输栈。

安装方式：

- 仅 CLI：

```powershell
cd tools\esp_drone_cli
pip install -e .
```

- CLI + GUI：

```powershell
cd tools\esp_drone_cli
pip install -e .[gui]
```

如果没有安装 `PyQt5`，CLI 仍可正常使用，GUI 会给出明确的依赖安装提示。

CLI 示例：

```powershell
python -m esp_drone_cli --serial COM7 connect
python -m esp_drone_cli --serial COM7 arm
python -m esp_drone_cli --serial COM7 dump-csv telemetry.csv --duration 5
```

安装后的入口：

```powershell
esp-drone-cli --serial COM7 connect
esp-drone-gui
```

相关文档：

- [`docs/python_tool_gui_refactor_plan.zh-CN.md`](./docs/python_tool_gui_refactor_plan.zh-CN.md)
- [`docs/python_gui_ui_plan.zh-CN.md`](./docs/python_gui_ui_plan.zh-CN.md)
- [`docs/python_gui_usage.zh-CN.md`](./docs/python_gui_usage.zh-CN.md)
- [`docs/python_gui_manual_checklist.zh-CN.md`](./docs/python_gui_manual_checklist.zh-CN.md)
- [`docs/cli_live_test_matrix.zh-CN.md`](./docs/cli_live_test_matrix.zh-CN.md)
- [`docs/cli_live_test_results.zh-CN.md`](./docs/cli_live_test_results.zh-CN.md)

## Stage-2 Console 规则

自阶段 2 起，调试链路固定分离：

- `UART0` 只给 `ATK-MS901M` IMU
- 应用 console、调试日志和 CLI 传输必须全部走 `USB CDC`
- `sdkconfig.defaults` 必须保持 UART console 关闭，避免应用日志泄漏到 IMU 串口

## 文档入口

建议从文档索引页开始：

- [`docs/README.zh-CN.md`](./docs/README.zh-CN.md)

推荐优先阅读：

- [`docs/hardware_extract.zh-CN.md`](./docs/hardware_extract.zh-CN.md)
- [`docs/axis_truth_table.zh-CN.md`](./docs/axis_truth_table.zh-CN.md)
- [`docs/motor_map.zh-CN.md`](./docs/motor_map.zh-CN.md)
- [`docs/runtime_frequency_plan.zh-CN.md`](./docs/runtime_frequency_plan.zh-CN.md)
- [`docs/python_gui_usage.zh-CN.md`](./docs/python_gui_usage.zh-CN.md)

## 致谢

本仓库是面向自制四轴的从零重写工程，但会明确致敬对兼容目标与 bring-up 过程有参考价值的开源工作：

- `ESP-Drone` 及其 legacy App / UDP 生态，用作行为与兼容参考
- `Bitcraze Crazyflie`，其开放飞控生态影响了很多小型飞控软件栈
- `ALIENTEK / ATK`，为重建 IMU 协议提供了 `ATK-MS901M` 资料与示例

需要特别说明：

- 本仓库不默认继承旧工程的机体系、姿态符号、mixer 或控制链实现
- 引用 legacy 工程主要是为了协议、视觉语义和硬件 bring-up
- 所有方向敏感逻辑都以 [`docs/axis_truth_table.zh-CN.md`](./docs/axis_truth_table.zh-CN.md) 和 [`docs/motor_map.zh-CN.md`](./docs/motor_map.zh-CN.md) 中重新定义的规则为准
