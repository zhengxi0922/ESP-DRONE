# Python CLI 使用说明

语言 / Language: 简体中文 | [English](./python_cli_usage.md)

## 目标

`esp-drone-cli` 是 Python 工具链的脚本化入口，也是台架调试入口。

它和 GUI 共用 `esp_drone_cli.core.device_session.DeviceSession`，所以 CLI 与 GUI 共用：

- framing
- 串口和 UDP transport
- 遥测解码
- 参数命令
- 设备命令
- CSV 日志

项目不维护第二套 host 协议栈。

## 范围边界

CLI 已经不只是最早的两条受限台架流程。当前实际覆盖：

- `roll / pitch / yaw` rate-loop 台架调试
- 受限 rig 的 hang-attitude 台架 bring-up
- ground tune / attitude ground verify 诊断
- low-risk liftoff verify 诊断
- 实验性 UDP manual control
- all-motor test
- params / telemetry / capability / device-info 工作流

CLI 仍然没有声明任何可带桨自由飞的 stabilize 或 angle 模式。ground / liftoff verify 是诊断和安全检查，不是已完成自由飞行模式的证明。

## 安装

只安装 CLI：

```powershell
cd tools\esp_drone_cli
pip install -e .
```

安装 CLI + GUI 依赖：

```powershell
cd tools\esp_drone_cli
pip install -e .[gui]
```

## 连接规则

台架 bring-up 优先使用 `USB CDC`。

- `UART0` 保留给 `ATK-MS901M`
- 下方串口示例使用 `--serial COMx`
- UDP 示例使用 SoftAP binary protocol：`192.168.4.1:2391`

通用连接示例：

```powershell
python -m esp_drone_cli --serial COM7 connect
python -m esp_drone_cli --udp 192.168.4.1:2391 connect
```

## 常用命令

连接、capability 和 stream：

```powershell
python -m esp_drone_cli --serial COM7 connect
python -m esp_drone_cli --serial COM7 capabilities
python -m esp_drone_cli --serial COM7 stream on
python -m esp_drone_cli --serial COM7 stream off
python -m esp_drone_cli --serial COM7 log --timeout 3 --telemetry
```

参数查看和修改：

```powershell
python -m esp_drone_cli --serial COM7 param-list
python -m esp_drone_cli --serial COM7 get-param rate_kp_roll
python -m esp_drone_cli --serial COM7 set-param rate_kp_roll 0.0030
python -m esp_drone_cli --serial COM7 save-params
python -m esp_drone_cli --serial COM7 reset-params
```

当前 GitHub 版本 `params.c` 中的固件默认 rate PID 是：

- `rate_kp_roll = 0.0030`
- `rate_kp_pitch = 0.0030`
- `rate_kp_yaw = 0.0030`
- `rate_ki_* = 0`
- `rate_kd_* = 0`

如果使用 `0.0007 / 0.0007 / 0.0005` 这类更保守的值，必须写成“本地调参候选值”或 RAM/NVS 覆盖值，不能写成当前固件默认值。

Rate-loop 台架命令：

```powershell
python -m esp_drone_cli --serial COM7 rate-test roll 20
python -m esp_drone_cli --serial COM7 rate-test pitch 20
python -m esp_drone_cli --serial COM7 rate-test yaw 20
python -m esp_drone_cli --serial COM7 rate-test roll 0
```

Hang-attitude 台架命令：

```powershell
python -m esp_drone_cli --serial COM7 attitude-capture-ref
python -m esp_drone_cli --serial COM7 attitude-test start --base-duty 0.05
python -m esp_drone_cli --serial COM7 attitude-status
python -m esp_drone_cli --serial COM7 attitude-test stop
```

Ground tune / attitude ground verify 命令：

```powershell
python -m esp_drone_cli --serial COM7 ground-capture-ref
python -m esp_drone_cli --serial COM7 ground-test start --base-duty 0.05
python -m esp_drone_cli --serial COM7 ground-test stop
python -m esp_drone_cli --serial COM7 attitude-ground-verify start --base-duty 0.05
python -m esp_drone_cli --serial COM7 attitude-ground-verify stop
```

Low-risk liftoff verify 命令：

```powershell
python -m esp_drone_cli --serial COM7 liftoff-verify start --base-duty 0.10
python -m esp_drone_cli --serial COM7 liftoff-verify stop
```

All-motor test 命令：

```powershell
python -m esp_drone_cli --serial COM7 all-motor-test start --duty 0.05 --duration-ms 1000
python -m esp_drone_cli --serial COM7 all-motor-test stop
```

实验性 UDP manual 示例：

```powershell
python -m esp_drone_cli --udp 192.168.4.1:2391 connect
python -m esp_drone_cli --udp 192.168.4.1:2391 udp-manual enable
python -m esp_drone_cli --udp 192.168.4.1:2391 udp-manual setpoint --throttle 0.05 --roll 0 --pitch 0 --yaw 0
python -m esp_drone_cli --udp 192.168.4.1:2391 udp-manual stop
```

## 注意

- 对旧固件运行新流程前，先执行 `capabilities`。
- 不要假设旧 protocol version 或 command id。协议源头以 `firmware/main/console/console_protocol.h` 为准。
- 诊断流程通过不等于自由飞行准备完成。
