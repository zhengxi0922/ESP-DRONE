# 圆棍姿态外环 Bring-Up 计划

**语言 / Language:** [English](./hang_attitude_bringup_plan.md) | **简体中文**

## 目标

这一阶段是面向圆棍、吊架或其他受限台架的 bench-only 姿态外环 bring-up。

它不是自由飞 stabilize 模式。
它不是 angle-ready 模式。
它绝不能直接用于带桨自由飞。

当前目标台架的特点是：机体安装后，自然平衡姿态不是 `+Z 朝上`，而是 `+Z 朝下`。
因此控制器不能把世界系水平 `roll=0 / pitch=0` 当成目标。

## 控制策略

参考姿态必须显式捕获：

1. 让机体回到自然悬挂平衡姿态。
2. 执行 `attitude-capture-ref`。
3. 固件把当前四元数存成 `q_ref`。

捕获后，外环使用相对四元数误差：

```text
q_rel = q_ref^-1 * q_now
```

随后从 `q_rel` 中提取小角度 roll / pitch 误差。
这里不直接拿全局欧拉角减固定零点，所以可以避开这个台架在 `+-180 deg` 附近容易出现的跳变和翻转问题。

这一轮只控制 `roll / pitch`。
Yaw 不进入姿态外环，默认保持 `rate_sp_yaw = 0`。

外环故意保持最小实现：

```text
rate_sp_roll  = clamp(-attitude_kp_roll  * err_roll_deg,  -attitude_rate_limit_roll,  attitude_rate_limit_roll)
rate_sp_pitch = clamp(-attitude_kp_pitch * err_pitch_deg, -attitude_rate_limit_pitch, attitude_rate_limit_pitch)
rate_sp_yaw   = 0
```

这些 rate setpoint 再送入现有已调好的 rate 内环。
油门保持开环，通过 `attitude_test_base_duty` 提供固定 base duty。

## 参数

建议保守默认值：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `attitude_kp_roll` | `2.0` | roll 外环 P 增益 |
| `attitude_kp_pitch` | `2.0` | pitch 外环 P 增益 |
| `attitude_rate_limit_roll` | `25.0` | roll rate setpoint 限幅，单位 dps |
| `attitude_rate_limit_pitch` | `25.0` | pitch rate setpoint 限幅，单位 dps |
| `attitude_error_deadband_deg` | `1.0` | 小误差死区 |
| `attitude_trip_deg` | `30.0` | 超限立即停机阈值 |
| `attitude_test_base_duty` | `0.05` | 固定开环 base duty |
| `attitude_ref_valid` | 运行态 | 只有 capture 成功后才为 true |

`attitude_ref_valid` 是运行态字段，不用于存储调参值。

## 遥测字段

建议至少观察这些字段，确认整条链路符号正确：

- `attitude_ref_valid`
- `attitude_err_roll_deg`
- `attitude_err_pitch_deg`
- `attitude_rate_sp_roll`
- `attitude_rate_sp_pitch`
- `attitude_ref_qw`、`attitude_ref_qx`、`attitude_ref_qy`、`attitude_ref_qz`
- `control_mode`
- `base_duty_active`
- 既有的 `pid_out_roll`、`pid_out_pitch` 与 `motor1..motor4`

目标观察链路是：

```text
手动扰动 -> 姿态误差 -> 外环 rate setpoint -> rate PID 输出 -> 电机混控
```

## CLI 使用流程

最小命令集：

```powershell
python -m esp_drone_cli --serial COM7 attitude-capture-ref
python -m esp_drone_cli --serial COM7 arm
python -m esp_drone_cli --serial COM7 attitude-test start --base-duty 0.05
python -m esp_drone_cli --serial COM7 attitude-status --timeout 5
python -m esp_drone_cli --serial COM7 watch-attitude all --timeout 10 --interval 0.2
python -m esp_drone_cli --serial COM7 attitude-test stop
python -m esp_drone_cli --serial COM7 disarm
```

命令拒绝会明确报错：

- 未 arm 时，`attitude-test start` 会被拒绝
- `attitude_ref_valid` 为 false 时，`attitude-test start` 会被拒绝
- capture 和 start 前都会检查 IMU 健康度、新鲜度和四元数可用性
- CLI 会直接显示设备拒绝原因，不再 silent ignore

## GUI 使用流程

GUI 在 `Debug Actions` 下新增独立的 `Hang Attitude` 区域。

可用控件包括：

- `Capture Ref`
- `Attitude Test Start`
- `Attitude Test Stop`
- `Base Duty`
- 以下参数的快捷编辑：
  - `attitude_kp_roll`
  - `attitude_kp_pitch`
  - `attitude_rate_limit_roll`
  - `attitude_rate_limit_pitch`
  - `attitude_error_deadband_deg`
  - `attitude_trip_deg`

图表选择器新增：

- `Hang Attitude Roll`
- `Hang Attitude Pitch`

每组图至少同时显示：

- measured attitude error
- generated rate setpoint
- inner-loop `pid_out`
- `motor1..motor4`

## 安全停机规则

以下任一条件触发时，固件都会立即停掉 `CONTROL_MODE_ATTITUDE_HANG_TEST`，并清零外环输出：

- IMU 不新鲜，或 health 不满足
- 参考姿态无效
- `abs(attitude_err_roll_deg)` 或 `abs(attitude_err_pitch_deg)` 超过 `attitude_trip_deg`
- arm 状态异常
- failsafe 触发
- 用户发送 `kill` 或 `attitude-test stop`

停止该模式时会同步清除姿态外环请求，避免残留 rate setpoint。

## 验收清单

下面两条符号检查必须在受限台架上逐项确认：

1. 把机体扰成“右侧下沉”，也就是本项目中的正 `roll`。
2. 控制器必须给出负 roll 修正，把机体拉回 capture 的参考姿态。
3. 期望电机方向：`M2 / M3` 增，`M1 / M4` 减。

以及：

1. 把机体扰成“机头抬起”，也就是本项目中的正 `pitch`。
2. 控制器必须给出负 pitch 修正，把机体拉回 capture 的参考姿态。
3. 期望电机方向：`M1 / M2` 增，`M3 / M4` 减。

Yaw 不属于这一轮姿态保持验收项。
