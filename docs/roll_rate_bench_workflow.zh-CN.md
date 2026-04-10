# Roll Rate 台架 Workflow

**语言 / Language:** [English](./roll_rate_bench_workflow.md) | **简体中文**

## 范围

这份 workflow 只用于受限台架上的 roll 单轴 rate-loop bring-up。

它不是：

- angle outer loop
- attitude hang test
- 自由飞调参

这里记录的 live 会话使用的是圆棍受限台架，机体自然姿态为 `+Z 朝下`。
这个机械前提只影响安装方式，不改变本 workflow 的 rate-only 验收规则。

## 固定约定

下面这些都不能改：

- `+roll = 右侧下沉`
- `roll_rate = -gyro_y`
- `+roll` 命令时，期望 `M1/M4` 增、`M2/M3` 减
- `-roll` 命令时，期望 `M2/M3` 增、`M1/M4` 减

## Roll Workflow

1. 上电并连接：
   `python -m esp_drone_cli --serial COMx connect`
2. 开流：
   `python -m esp_drone_cli --serial COMx stream on`
3. 手动晃动机体，确认符号链：
   `roll_rate = -gyro_y`
4. 跑直接命令：
   `rate-test roll 30`、`rate-test roll -30`，然后 `rate-test roll 0`
5. 观察：
   `rate-status roll --timeout 5`
   `watch-rate all --timeout 5 --interval 0.2`
6. 执行：
   `axis-bench roll --auto-arm --small-step 10 --large-step 15`
7. 检查保存下来的 CSV、JSON summary 和 Markdown summary。
8. 只有 `sign_ok` 和 `motor_split_ok` 都为 true 时，才允许小步修改 `rate_kp_roll`。
9. 每次修改后都重复同样的 roll workflow。

## Summary 闸门

roll bench summary 是主要闸门。

只有当下面这些都满足时，才允许继续调 `kp`：

- `setpoint_path_ok = True`
- `sign_ok = True`
- `motor_split_ok = True`
- `kp_tuning_allowed = True`

每一轮都要重点看：

- `measurable_response`
- `saturation_risk`
- `return_to_zero_quality`
- `noise_or_jitter_risk`
- `low_duty_motor_stability`
- `axis_result`

## Kp 判据

可接受：

- 正负方向命令都正确
- 电机分配方向正确
- 有可测响应
- 回零干净
- 没有明显低 duty 不稳定

`kp` 偏低：

- `measurable_response` 弱
- setpoint 已经有了，但 `pid_out_roll` 和实际响应都太小
- 回零慢

`kp` 偏高：

- `saturation_risk` 上升
- `return_to_zero_quality` 变差
- `noise_or_jitter_risk` 上升
- 开始出现振荡、打架或过冲

一旦出现异常：

1. 立即停 test
2. 不要继续加大 `rate_kp_roll`

## Live 会话结果

`2026-04-10` 的 live 受限台架对比如下：

| 候选值 | sign_ok | motor_split_ok | measurable_response | saturation_risk | return_to_zero_quality | noise_or_jitter_risk | low_duty_motor_stability | 结论 |
|---|---|---|---|---|---|---|---|---|
| `rate_kp_roll = 0.0026` | `True` | `True` | `True` | `False` | `PASS` | `False` | `PASS_WITH_WARNING` | 保留 |
| `rate_kp_roll = 0.0028` | `True` | `True` | `True` | `False` | `PASS` | `True` | `PASS_WITH_WARNING` | 拒绝 |

这根台架上推荐的 roll 参数：

- `rate_kp_roll = 0.0026`
- `rate_ki_roll = 0.0`
- `rate_kd_roll = 0.0`

理由：

- 基线 `0.0026` 已经通过符号、电机分配、可测响应和回零检查
- `0.0028` 没带来更好的结论，反而抬高了 jitter 风险
- 最大探测步阶已经接近低 duty 地板，因此 `kp` 应保持保守

这些值在比较结束后已经重新写回 flash。

## Summary 样例

- [roll_bench_summary_sample.md](./roll_bench_summary_sample.md)
- [roll_bench_summary_sample.json](./roll_bench_summary_sample.json)
