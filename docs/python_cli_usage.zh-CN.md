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

## 范围警告

新的 `attitude-test` 路径只用于圆棍、吊架或其他受限台架。

它不是自由飞 stabilize。
它不是自由飞 angle 模式。
不能直接用于带桨自由飞。

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

当前 bring-up 默认使用 `USB CDC`。

- `UART0` 继续专用于 `ATK-MS901M`
- 下方 CLI 示例统一使用 `--serial COMx`

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

圆棍姿态外环 bench-only 命令：

```powershell
python -m esp_drone_cli --serial COM7 attitude-capture-ref
python -m esp_drone_cli --serial COM7 arm
python -m esp_drone_cli --serial COM7 attitude-test start --base-duty 0.05
python -m esp_drone_cli --serial COM7 attitude-status --timeout 5
python -m esp_drone_cli --serial COM7 watch-attitude roll --timeout 5 --interval 0.2
python -m esp_drone_cli --serial COM7 watch-attitude pitch --timeout 5 --interval 0.2
python -m esp_drone_cli --serial COM7 watch-attitude all --timeout 5 --interval 0.2
python -m esp_drone_cli --serial COM7 attitude-test stop
python -m esp_drone_cli --serial COM7 disarm
```

以下情况会明确拒绝 `attitude-test start`：

- 机体未 arm
- `attitude_ref_valid` 为 false
- IMU 健康度、新鲜度或四元数可用性不满足

拒绝会直接报错，不会 silent ignore。

## 遥测观察

按轴观察 rate 遥测：

```powershell
python -m esp_drone_cli --serial COM7 rate-status roll --timeout 5
python -m esp_drone_cli --serial COM7 rate-status pitch --timeout 5
python -m esp_drone_cli --serial COM7 rate-status yaw --timeout 5
python -m esp_drone_cli --serial COM7 watch-rate all --timeout 5 --interval 0.2
```

观察圆棍姿态外环遥测：

```powershell
python -m esp_drone_cli --serial COM7 attitude-status --timeout 5
python -m esp_drone_cli --serial COM7 watch-attitude all --timeout 5 --interval 0.2
```

姿态外环状态输出包含：

- `attitude_ref_valid`
- `attitude_err_roll_deg`
- `attitude_err_pitch_deg`
- `attitude_rate_sp_roll`
- `attitude_rate_sp_pitch`
- `pid_out_roll`
- `pid_out_pitch`
- `base_duty_active`
- `motor1..motor4`
- `control_mode`
- 参考四元数 `attitude_ref_qw/qx/qy/qz`

建议按下面这条链路检查符号：

```text
手动扰动 -> 姿态误差 -> 外环 rate setpoint -> rate PID 输出 -> 电机混控
```

## 台架自动化

单轴台架自动化：

```powershell
python -m esp_drone_cli --serial COM7 axis-bench roll --auto-arm --small-step 10 --large-step 15
python -m esp_drone_cli --serial COM7 axis-bench pitch --auto-arm --kp 0.0028 --small-step 10 --large-step 15
python -m esp_drone_cli --serial COM7 rate-bench yaw --auto-arm --kp 0.0026 --small-step 10 --large-step 15 --save-params
```

`axis-bench` 和 `rate-bench` 是同一个共享命令。它们会复用 GUI / session 层同一套逻辑，并保存：

- telemetry CSV
- JSON summary
- Markdown summary

汇总结果会包含：

- `setpoint_path_ok`
- `sign_ok`
- `motor_split_ok`
- `measurable_response`
- `saturation_risk`
- `return_to_zero_quality`
- `noise_or_jitter_risk`
- `low_duty_motor_stability`
- `axis_result`，取值为 `PASS`、`PASS_WITH_WARNING` 或 `FAIL`

导出 CSV：

```powershell
python -m esp_drone_cli --serial COM7 dump-csv telemetry.csv --duration 5
```

## 轴含义

项目 rate 映射固定为：

- `pitch_rate = gyro_x`
- `roll_rate = -gyro_y`
- `yaw_rate = -gyro_z`

正向 mixer 约定固定为：

- `+roll` -> `M1/M4` 增，`M2/M3` 减
- `+pitch` -> `M3/M4` 增，`M1/M2` 减
- `+yaw` -> `M1/M3` 增，`M2/M4` 减

对于圆棍姿态外环，这一轮必须满足的纠正方向是：

- 扰成“右侧下沉”即正 `roll` -> 控制器必须给负 roll 修正 -> `M2/M3` 增，`M1/M4` 减
- 扰成“机头抬起”即正 `pitch` -> 控制器必须给负 pitch 修正 -> `M1/M2` 增，`M3/M4` 减

## 参数调试

参数读写、保存、导入导出流程继续兼容固件参数存储：

```powershell
python -m esp_drone_cli --serial COM7 get rate_kp_roll
python -m esp_drone_cli --serial COM7 set rate_kp_roll float 0.0035
python -m esp_drone_cli --serial COM7 set attitude_kp_roll float 2.0
python -m esp_drone_cli --serial COM7 set attitude_test_base_duty float 0.05
python -m esp_drone_cli --serial COM7 save
python -m esp_drone_cli --serial COM7 export params.json
python -m esp_drone_cli --serial COM7 import params.json --save
```

重点 rate 参数：

- `rate_kp_roll`、`rate_ki_roll`、`rate_kd_roll`
- `rate_kp_pitch`、`rate_ki_pitch`、`rate_kd_pitch`
- `rate_kp_yaw`、`rate_ki_yaw`、`rate_kd_yaw`
- `rate_integral_limit`
- `rate_output_limit`

重点圆棍姿态外环参数：

- `attitude_kp_roll`
- `attitude_kp_pitch`
- `attitude_rate_limit_roll`
- `attitude_rate_limit_pitch`
- `attitude_error_deadband_deg`
- `attitude_trip_deg`
- `attitude_test_base_duty`
- 只读运行态 `attitude_ref_valid`

如果设备拒绝写入，CLI 会明确报告失败原因。

## 推荐台架流程

用于 rate 环台架时：

1. 拆桨，或把机体完全约束。
2. 通过 USB CDC 连接。
3. 先开流并确认手持运动符号。
4. 只有在台架安全条件满足时才 arm。
5. 每次只跑一根轴，用 `rate-test`。
6. 用 `rate-status` 观察当前轴。
7. 一次只改一个 PID 项。
8. 只有确认新值可接受后再执行 `save`。

用于圆棍 / 吊架姿态外环台架时：

1. 确保机体固定在圆棍或吊架上，并确认自然平衡姿态就是准备 capture 的参考姿态。
2. 开流并先确认 IMU 新鲜度正常。
3. 让机体回到自然悬挂姿态，执行 `attitude-capture-ref`。
4. 只有在台架受限且安全时才 arm。
5. 使用保守的 `attitude_kp_*`、`attitude_rate_limit_*` 和 `attitude_test_base_duty` 起步。
6. 执行 `attitude-test start`。
7. 用 `watch-attitude` 在小扰动下确认误差符号、rate setpoint、PID 输出和电机分配链路正确。
8. 只要符号错误、触发 trip 或 IMU 不新鲜，就立即 `attitude-test stop` 或 `kill`。
9. 测试结束后 `disarm`。

## 错误处理

设备端命令拒绝现在都会明确显示在 CLI 输出里，常见情况包括：

- 参数非法
- 需要先 arm
- 需要先 disarm
- IMU 未就绪
- 参考姿态未捕获
- 固件不支持该命令

CLI 进程退出码会跟随固件状态码，便于脚本和台架记录排查问题。
