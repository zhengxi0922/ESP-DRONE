# Python CLI 使用说明

**语言 / Language:** [English](./python_cli_usage.md) | **简体中文**

## 目标

`esp-drone-cli` 是 Python 工具链中面向脚本和台架调试的命令行入口。

它与 GUI 共用同一个 `esp_drone_cli.core.device_session.DeviceSession`，因此 CLI 和 GUI 共用：

- framing
- serial / UDP 传输
- telemetry 解码
- 参数命令
- 设备命令
- CSV 日志

项目没有维护第二套主机协议栈。

## 安装

只安装 CLI：

```powershell
cd tools\esp_drone_cli
pip install -e .
```

同时安装 CLI + GUI 依赖：

```powershell
cd tools\esp_drone_cli
pip install -e .[gui]
```

## 传输约束

当前台架 bring-up 默认走 `USB CDC`。

- `UART0` 继续专用于 `ATK-MS901M`
- 下面的 CLI 示例因此统一使用 `--serial COMx`

连接示例：

```powershell
python -m esp_drone_cli --serial COM7 connect
python -m esp_drone_cli --udp 192.168.4.1:2391 connect
```

## 常用命令

连接与开流：

```powershell
python -m esp_drone_cli --serial COM7 connect
python -m esp_drone_cli --serial COM7 stream on
python -m esp_drone_cli --serial COM7 stream off
python -m esp_drone_cli --serial COM7 log --timeout 3 --telemetry
```

三轴 rate 闭环台架命令：

```powershell
python -m esp_drone_cli --serial COM7 rate-test roll 20
python -m esp_drone_cli --serial COM7 rate-test pitch 20
python -m esp_drone_cli --serial COM7 rate-test yaw 20
python -m esp_drone_cli --serial COM7 rate-test roll 0
```

按轴观察 rate 遥测：

```powershell
python -m esp_drone_cli --serial COM7 rate-status roll --timeout 5
python -m esp_drone_cli --serial COM7 rate-status pitch --timeout 5
python -m esp_drone_cli --serial COM7 rate-status yaw --timeout 5
python -m esp_drone_cli --serial COM7 watch-rate all --timeout 5 --interval 0.2
```

`rate-status` 面向台架调参，输出包括：

- setpoint
- 映射后的反馈 rate
- 来源 gyro 字段
- PID `p/i/d`
- `pid_out`
- `motor1..motor4`
- `arm_state`、`control_mode`、`imu_age_us`、`loop_dt_us`

导出 CSV：

```powershell
python -m esp_drone_cli --serial COM7 dump-csv telemetry.csv --duration 5
```

## 三轴 rate 定义

项目 rate 映射固定为：

- `pitch_rate = gyro_x`
- `roll_rate = -gyro_y`
- `yaw_rate = -gyro_z`

正向电机分配固定为：

- `+roll` -> `M1/M4` 增，`M2/M3` 减
- `+pitch` -> `M3/M4` 增，`M1/M2` 减
- `+yaw` -> `M1/M3` 增，`M2/M4` 减

## 参数调试流程

参数读写、保存、导入导出流程继续兼容固件参数存储：

```powershell
python -m esp_drone_cli --serial COM7 get rate_kp_roll
python -m esp_drone_cli --serial COM7 set rate_kp_roll float 0.0035
python -m esp_drone_cli --serial COM7 set rate_kd_pitch float 0.0002
python -m esp_drone_cli --serial COM7 save
python -m esp_drone_cli --serial COM7 export params.json
python -m esp_drone_cli --serial COM7 import params.json --save
```

重点参数：

- `rate_kp_roll`、`rate_ki_roll`、`rate_kd_roll`
- `rate_kp_pitch`、`rate_ki_pitch`、`rate_kd_pitch`
- `rate_kp_yaw`、`rate_ki_yaw`、`rate_kd_yaw`
- `rate_integral_limit`
- `rate_output_limit`

如果设备拒绝写入，CLI 现在会明确报错，不再把失败伪装成成功。

## 推荐台架流程

1. 拆桨或把机体完全约束。
2. 通过 USB CDC 连接。
3. 先开流并确认手持运动符号。
4. 只有在台架安全条件满足时才解锁。
5. 每次只测一根轴。
6. 测试时用 `rate-status` 观察当前轴。
7. PID 一次只改一个量。
8. 只有确认新值可接受后再执行 `save`。

## 错误处理

设备端命令拒绝现在会明确体现在 CLI 输出中，常见情况包括：

- 参数非法
- 需要先 arm
- 需要先 disarm
- IMU 未就绪
- 固件不支持该命令

CLI 进程退出码会跟随固件状态码，便于脚本和台架记录排查问题。

