# ESP-DRONE

语言 / Language: [English](./README.md) | 简体中文

这是一个面向自制四轴的 ESP-IDF 飞控固件仓库，并提供共用核心层的 Python CLI / GUI 工具链，目标平台为 `ESP32-S3-WROOM-1-N16R8`。

## 固定约束

- `ESP-IDF v5.5.1`
- 机体系固定为 `+Y` 机头、`+X` 机体右侧、`+Z` 向上
- 项目命名固定为 `+pitch` 抬头、`+roll` 右侧下沉、`+yaw` 机头右转
- `UART0` 只给 `ATK-MS901M`
- `USB CDC` 只给 CLI、GUI 和调试遥测
- CLI 与 GUI 共用同一个 `DeviceSession`

所有方向敏感逻辑都必须遵循：

- [docs/axis_truth_table.zh-CN.md](docs/axis_truth_table.zh-CN.md)
- [docs/motor_map.zh-CN.md](docs/motor_map.zh-CN.md)

## 仓库结构

- [docs/](docs/)：设计约束、bring-up 记录、CLI / GUI 使用说明
- [firmware/](firmware/)：ESP-IDF 固件工程
- [tools/esp_drone_cli/](tools/esp_drone_cli/)：共享 `core + cli + gui` Python 工具链

## 当前状态

当前仓库包含的是诊断、验证和实验性手动控制路径。这些路径可以用于 bring-up、遥测、安全检查和受控电机/姿态实验，但不能等同于已经完成的自由飞行 stabilize 模式。

已经实现的诊断 / 验证 / 实验路径：

- `roll / pitch / yaw` 三轴 rate-loop 台架路径
- 面向圆棍/吊架、自然 `+Z down` 平衡姿态的 hang-attitude 台架外环路径
- 使用地面参考姿态的 ground tune / attitude ground verify 路径
- low-risk liftoff verify 路径
- all-motor test 路径
- ESP32 SoftAP + binary UDP 的 CLI/GUI 连接路径
- 通过 `CONTROL_MODE_UDP_MANUAL` 进入的实验性 UDP manual control
- 参数、遥测、capability、device-info 类主机工具链

尚未实现或尚未声明可用：

- 可直接带桨自由飞的 stabilize / angle 模式
- 自动起飞控制器
- 闭环定高
- 定点 / position hold

hang-attitude 路径是受限诊断路径：

- 固件添加 `CONTROL_MODE_ATTITUDE_HANG_TEST`
- 必须通过 `attitude-capture-ref` 显式捕获参考姿态
- 控制使用相对四元数误差 `q_rel = q_ref^-1 * q_now`，不是直接对全局欧拉角 `roll=0 / pitch=0` 做差
- 该阶段只有 `roll / pitch` 进入外环
- 外环是 P-only，并喂给现有 rate 内环
- 推力仍通过 `attitude_test_base_duty` 开环给定

已有 roll 现场记录来自受限圆棍台架，且自然平衡姿态为 `+Z down`。这个方向不会改变 roll rate-loop 的符号约定，因为该流程只评估 `rate_setpoint_roll`、映射后的 roll rate 反馈、PID 输出和电机分裂。

## 当前参数事实

文档必须区分“当前固件默认值”和“调参候选值”。

当前 GitHub 版本 `firmware/main/params/params.c` 里的 rate PID 默认值是：

- `rate_kp_roll = 0.0030`
- `rate_kp_pitch = 0.0030`
- `rate_kp_yaw = 0.0030`
- `rate_ki_* = 0`
- `rate_kd_* = 0`

之前讨论过的保守值 `roll=0.0007`、`pitch=0.0007`、`yaw=0.0005` 只能写成调参候选值；除非同步修改 `params.c`，否则不能写成当前默认值。

## 电机输出状态

当前 `firmware/main/motor/motor.c` 使用固定 `LEDC_TIMER_8_BIT` PWM 分辨率。PWM 频率通过 `motor_pwm_freq_hz` 参数化，但 PWM 分辨率没有参数化。

已经实现：

- `motor_output_map[4]`
- 全局 `motor_idle_duty`
- 全局 `motor_max_duty`
- 全局 `motor_startup_boost_duty`
- 全局 `motor_slew_limit_per_tick`
- PWM 频率参数化

尚未实现：

- 每电机推力 `scale`
- 每电机 `offset`
- 每电机 `min_start`
- 每电机 `deadband`
- 每电机 `gamma`
- PWM 分辨率参数化

`motor_output_map` 是通道映射功能，不能和每电机推力补偿混为一谈。

## SoftAP UDP Transport

固件默认启动 ESP32 SoftAP：

- SSID: `ESP-DRONE`
- Password: `12345678`
- AP IP: `192.168.4.1`
- UDP protocol port: `2391`

GUI 连接顺序：

1. 电脑 Wi-Fi 连接 `ESP-DRONE` SoftAP。
2. 启动 Python GUI，把 `Link` 设为 `UDP`。
3. 使用 `UDP Host = 192.168.4.1`、`UDP Port = 2391`。
4. 点击 `Connect`。

Wi-Fi 或 UDP 启动失败时，仍应优先使用 Serial / USB CDC 调试。见 [docs/softap_udp_transport.md](docs/softap_udp_transport.md)。

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

- [docs/README.zh-CN.md](docs/README.zh-CN.md)
- [docs/CODEX_STATE.md](docs/CODEX_STATE.md)
- [docs/TUNING_DECISIONS.md](docs/TUNING_DECISIONS.md)
- [docs/axis_truth_table.zh-CN.md](docs/axis_truth_table.zh-CN.md)
- [docs/motor_map.zh-CN.md](docs/motor_map.zh-CN.md)
- [docs/hang_attitude_bringup_plan.zh-CN.md](docs/hang_attitude_bringup_plan.zh-CN.md)
- [docs/python_cli_usage.zh-CN.md](docs/python_cli_usage.zh-CN.md)
- [docs/python_gui_usage.zh-CN.md](docs/python_gui_usage.zh-CN.md)
