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

## 范围边界

CLI 现在覆盖两条受限台架 workflow：

- `roll / pitch / yaw` 的 rate-loop 台架调试
- 受限圆棍 / 吊架的 bench-only 姿态外环 bring-up

它们都不代表任何带桨自由飞 stabilize / angle 模式已经 ready。

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

roll 单轴台架命令：

```powershell
python -m esp_drone_cli --serial COM7 rate-test roll 20
python -m esp_drone_cli --serial COM7 rate-test roll -20
python -m esp_drone_cli --serial COM7 rate-test roll 0
python -m esp_drone_cli --serial COM7 rate-status roll --timeout 5
python -m esp_drone_cli --serial COM7 watch-rate all --timeout 5 --interval 0.2
python -m esp_drone_cli --serial COM7 axis-bench roll --auto-arm --small-step 10 --large-step 15
```

圆棍姿态外环 bench-only 命令：

```powershell
python -m esp_drone_cli --serial COM7 attitude-capture-ref
python -m esp_drone_cli --serial COM7 arm
python -m esp_drone_cli --serial COM7 attitude-test start --base-duty 0.05
python -m esp_drone_cli --serial COM7 attitude-status --timeout 5
python -m esp_drone_cli --serial COM7 watch-attitude all --timeout 5 --interval 0.2
python -m esp_drone_cli --serial COM7 attitude-test stop
python -m esp_drone_cli --serial COM7 disarm
```

## Rate-Status 输出

`rate-status roll` 会直接打印 roll 台架需要的明确字段：

- `rate_setpoint_roll`
- `roll_rate`
- `source_expr=-gyro_y`
- `raw_gyro_y`
- `rate_pid_p_roll`
- `rate_pid_i_roll`
- `rate_pid_d_roll`
- `pid_out_roll`
- `motor1..motor4`
- `arm_state`
- `control_mode`
- `imu_age_us`
- `loop_dt_us`

这一轮 rate-only 观察链路是：

```text
命令 roll 角速度 -> 映射后的 roll 反馈 -> PID p/i/d -> pid_out_roll -> motor1..motor4
```

## 轴含义

项目 rate 映射固定为：

- `pitch_rate = gyro_x`
- `roll_rate = -gyro_y`
- `yaw_rate = -gyro_z`

正向电机分配约定固定为：

- `+roll` -> `M1/M4` 增，`M2/M3` 减
- `-roll` -> `M2/M3` 增，`M1/M4` 减
- `+pitch` -> `M3/M4` 增，`M1/M2` 减
- `+yaw` -> `M1/M3` 增，`M2/M4` 减

圆棍姿态外环阶段要求的纠正方向是：

- 扰成“右侧下沉”即正 `roll` -> 控制器必须给负 roll 修正 -> `M2/M3` 增，`M1/M4` 减
- 扰成“机头抬起”即正 `pitch` -> 控制器必须给负 pitch 修正 -> `M1/M2` 增，`M3/M4` 减

## Roll 台架 Workflow

在受限台架上按这个顺序做：

1. 上电并通过 USB CDC 连接。
2. 执行 `stream on`。
3. 手动晃动机体，确认符号链：
   `roll_rate = -gyro_y`。
4. 运行 `rate-test roll +30` 和 `rate-test roll -30`。
5. 观察 `rate-status roll` 和 `watch-rate all`。
6. 运行 `axis-bench roll` 或 `rate-bench roll`，生成 telemetry CSV、JSON summary 和 Markdown summary。
7. 只有在 `sign_ok` 和 `motor_split_ok` 都为 true 时，才允许小步修改 `rate_kp_roll`。
8. 每次修改后都重复同样的 roll workflow。

相关文档：

- [roll_rate_bench_workflow.zh-CN.md](./roll_rate_bench_workflow.zh-CN.md)
- [roll_bench_summary_sample.md](./roll_bench_summary_sample.md)

## 台架自动化

`axis-bench` 和 `rate-bench` 是同一个共享命令。它们会复用 GUI / session 层同一套逻辑，并保存：

- telemetry CSV
- JSON summary
- Markdown summary

summary 会输出：

- `setpoint_path_ok`
- `sign_ok`
- `motor_split_ok`
- `measurable_response`
- `saturation_risk`
- `return_to_zero_quality`
- `noise_or_jitter_risk`
- `low_duty_motor_stability`
- `safe_to_continue`
- `kp_tuning_allowed`
- `axis_result`

`kp_tuning_allowed` 是故意做成严格门限的：

- 如果 `sign_ok` 为 false，先查轴映射或 rate feedback，不准继续调 `kp`
- 如果 `motor_split_ok` 为 false，先查 roll 电机分配，不准继续调 `kp`
- 只有 `kp_tuning_allowed=true` 时，才允许继续微调 `rate_kp_roll`

导出 CSV：

```powershell
python -m esp_drone_cli --serial COM7 dump-csv telemetry.csv --duration 5
```

## 参数调试流程

参数读写、保存、导入导出继续兼容固件参数存储：

```powershell
python -m esp_drone_cli --serial COM7 get rate_kp_roll
python -m esp_drone_cli --serial COM7 set rate_kp_roll float 0.0026
python -m esp_drone_cli --serial COM7 get rate_ki_roll
python -m esp_drone_cli --serial COM7 get rate_kd_roll
python -m esp_drone_cli --serial COM7 save
python -m esp_drone_cli --serial COM7 export params.json
python -m esp_drone_cli --serial COM7 import params.json --save
```

roll 台架阶段相关参数：

- `rate_kp_roll`
- `rate_ki_roll`
- `rate_kd_roll`
- `rate_integral_limit`
- `rate_output_limit`
- `bringup_test_base_duty`

圆棍姿态外环相关参数：

- `attitude_kp_roll`
- `attitude_kp_pitch`
- `attitude_rate_limit_roll`
- `attitude_rate_limit_pitch`
- `attitude_error_deadband_deg`
- `attitude_trip_deg`
- `attitude_test_base_duty`
- `attitude_ref_valid`

当前文档记录的 live roll 推荐值是：

- `rate_kp_roll = 0.0026`
- `rate_ki_roll = 0.0`
- `rate_kd_roll = 0.0`

如果设备拒绝写入，CLI 会明确报告失败原因。

## Roll Kp 判据

按下面这组规则解读 roll 台架结果：

- 可接受：
  - 正负 roll 指令方向都正确
  - 电机分配方向正确
  - 有可测响应
  - 回零干净
  - 没有明显低 duty 不稳定
- `kp` 偏低：
  - `measurable_response` 弱
  - setpoint 已经有了，但 `pid_out_roll` 和实际响应都太小
  - 回零慢
- `kp` 偏高：
  - `saturation_risk` 上升
  - `return_to_zero_quality` 变差
  - `noise_or_jitter_risk` 上升
  - 开始出现振荡、打架或过冲
- 一旦出现异常：
  - 立即停 test
  - 不要继续加大 `rate_kp_roll`

## 圆棍姿态外环备注

圆棍姿态外环路径仍然只限受限台架：

- 不能直接用于带桨自由飞
- 那一阶段 yaw 不进入姿态外环
- `attitude-test start` 前必须先 capture ref

## 错误处理

设备端命令拒绝都会明确显示在 CLI 输出里，常见情况包括：

- 参数非法
- 需要先 arm
- 需要先 disarm
- IMU 未就绪
- 参考姿态未捕获
- 固件不支持该命令

CLI 进程退出码会跟随固件状态码，便于脚本和台架记录排查问题。
