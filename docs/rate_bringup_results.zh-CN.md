# 速率环 Bring-Up 结果

**语言 / Language:** [English](./rate_bringup_results.md) | **简体中文**

## 范围

当前阶段仍然只做台架速率闭环：

- 不做自由飞行
- 不做 angle 外环
- 不做 autotune
- 测试时必须拆桨或将机体完全约束

仓库约束继续保持不变：

- `UART0` 只给 `ATK-MS901M`
- `USB CDC` 只给 CLI、GUI 和调试遥测
- CLI 与 GUI 继续共用 `esp_drone_cli.core.device_session.DeviceSession`

## 固定符号约定

机体系：

- `+Y` = 机头 / 前方
- `+X` = 机体右侧
- `+Z` = 向上

项目姿态正方向：

- `+pitch` = 抬头
- `+roll` = 右侧下沉
- `+yaw` = 机头右转

固件、CLI、GUI、文档统一使用的 rate 映射：

- `pitch_rate = gyro_x`
- `roll_rate = -gyro_y`
- `yaw_rate = -gyro_z`

正向 mixer 预期：

- `+roll` -> `M1/M4` 增，`M2/M3` 减
- `+pitch` -> `M3/M4` 增，`M1/M2` 减
- `+yaw` -> `M1/M3` 增，`M2/M4` 减

详见 [axis_truth_table.zh-CN.md](./axis_truth_table.zh-CN.md) 与 [motor_map.zh-CN.md](./motor_map.zh-CN.md)。

## 当前软件状态

仓库现在已经具备可用的三轴 rate 闭环台架路径：

- 固件中的 `rate roll`、`rate pitch`、`rate yaw` 都已经真实接入 setpoint、映射后的 gyro 反馈、各轴 PID、mixer、电机输出和安全门控
- rate 控制只在收到 fresh IMU sample 时更新，保持 stage-2.5 设计约束
- 以下 rate PID 参数全部真实生效：
  - `rate_kp_roll`、`rate_ki_roll`、`rate_kd_roll`
  - `rate_kp_pitch`、`rate_ki_pitch`、`rate_kd_pitch`
  - `rate_kp_yaw`、`rate_ki_yaw`、`rate_kd_yaw`
  - `rate_integral_limit`、`rate_output_limit`
- 参数校验已经补到固件侧，可拒绝明显不安全的 rate PID 数值和输出限制组合
- USB 遥测已补齐 rate 调试所需字段，CLI 和 GUI 都可直接读取：
  - `gyro_x/y/z`
  - `roll/pitch/yaw`
  - `rate_setpoint_roll/pitch/yaw`
  - `rate_pid_p/i/d_roll/pitch/yaw`
  - `pid_out_roll/pitch/yaw`
  - `motor1..motor4`
  - `imu_age_us`、`loop_dt_us`、`arm_state`、`control_mode`、`failsafe_reason`

## 控制链摘要

每次收到 fresh IMU sample 时，当前 rate 轴会走完整链路：

1. `rate-test <axis> <value>` 更新该轴的 rate setpoint。
2. 固件把 IMU gyro 映射到项目轴定义：
   - pitch 用 `gyro_x`
   - roll 用 `-gyro_y`
   - yaw 用 `-gyro_z`
3. 对应轴 PID 计算：
   - `rate_pid_p_*`
   - `rate_pid_i_*`
   - `rate_pid_d_*`
   - `pid_out_*`
4. mixer 把带符号的 PID 输出分配到 `motor1..motor4`。
5. arm 状态、IMU 健康度和命令参数校验共同决定命令是否允许执行。

## 分轴台架表

| 轴 | 物理动作 | CLI 命令 | 主要 gyro 字段 | 主要 setpoint 字段 | 主要 PID 字段 | 正向预期电机分配 |
|---|---|---|---|---|---|---|
| `roll` | 右侧下沉 / 左侧下沉 | `rate-test roll 20` | `gyro_y`，项目反馈为 `-gyro_y` | `rate_setpoint_roll` | `rate_pid_p_roll`、`rate_pid_i_roll`、`rate_pid_d_roll`、`pid_out_roll` | `+roll`：`M1/M4` 上升，`M2/M3` 下降 |
| `pitch` | 抬头 / 低头 | `rate-test pitch 20` | `gyro_x` | `rate_setpoint_pitch` | `rate_pid_p_pitch`、`rate_pid_i_pitch`、`rate_pid_d_pitch`、`pid_out_pitch` | `+pitch`：`M3/M4` 上升，`M1/M2` 下降 |
| `yaw` | 机头右转 / 左转 | `rate-test yaw 20` | `gyro_z`，项目反馈为 `-gyro_z` | `rate_setpoint_yaw` | `rate_pid_p_yaw`、`rate_pid_i_yaw`、`rate_pid_d_yaw`、`pid_out_yaw` | `+yaw`：`M1/M3` 上升，`M2/M4` 下降 |

## CLI 台架流程

1. 通过 USB CDC 连接设备。
2. 打开 telemetry。
3. 先在未解锁状态下做手持动作确认遥测方向。
4. 只有在安全台架条件下才允许解锁。
5. 每次只测试一根轴。
6. 测试时配合 `rate-status` 或 `watch-rate` 观察闭环状态。
7. 如果符号或电机方向不对，立即执行 `rate-test <axis> 0` 或 `kill`。

示例：

```powershell
python -m esp_drone_cli --serial COM7 connect
python -m esp_drone_cli --serial COM7 stream on
python -m esp_drone_cli --serial COM7 log --timeout 3 --telemetry
python -m esp_drone_cli --serial COM7 arm
python -m esp_drone_cli --serial COM7 rate-status roll --timeout 5
python -m esp_drone_cli --serial COM7 rate-test roll 20
python -m esp_drone_cli --serial COM7 rate-test roll 0
python -m esp_drone_cli --serial COM7 disarm
```

## GUI 台架流程

1. 在左侧 `Connection` 区使用 `Serial` 连接。
2. 点击 `Stream On`。
3. 在中间图表组切到 `Rate Roll`、`Rate Pitch` 或 `Rate Yaw`。
4. 在左侧 `Debug Actions` 的 `rate test` 区选择轴、输入 dps、点击 `Start`。
5. 观察对应的 gyro、`rate_setpoint_*`、`pid_out_*` 和 `motor1..motor4`。
6. 在右侧参数区搜索 `rate_` 并调整 PID。
7. 停止该轴测试后，再决定是否 `Save` 参数。

## 验收状态

仓库内的软件路径已经完成：

- `rate-test roll/pitch/yaw` 会走真实控制链
- CLI 与 GUI 共用同一条 `DeviceSession`
- GUI 已提供分轴 rate test 控件和分轴图表组
- 参数 `set/save/export/import` 流程保持兼容
- 设备端命令拒绝会明确反馈，不再静默忽略

当前软件验证结果：

- `pytest tools/esp_drone_cli/tests -q` 通过，共 `26` 项
- `.\tools\idf.ps1 build` 已完成固件构建

## 仍需线下硬件验证

这些内容仍然必须在线下台架完成：

- 验证实际机体上的 `+roll`、`+pitch`、`+yaw` 电机方向
- 验证实际手持运动时的 gyro 映射符号
- 验证目标板上的 USB CDC 实机通信
- 在真实硬件上从保守默认值开始调 `Kp/Ki/Kd`

