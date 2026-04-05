# CLI 联机测试结果

**语言 / Language：** [English](./cli_live_test_results.md) | **简体中文**

## 环境

- 日期：`2026-04-03`
- 主机环境：Windows PowerShell
- 目标串口：`COM4`
- 安全前提：仅非起飞命令，已拆桨或机体受限固定
- 本轮覆盖的传输：`serial`
- 本轮延后的传输：`udp`

## Stage D 预检查

- README 中关于 GUI / CLI 的说明：已确认存在
- GUI smoke tests：已确认存在并在本地通过
- core 单一 owner 规则：已确认
  - CLI 使用 `DeviceSession`
  - GUI 使用 `DeviceSession`
  - `client.py` 只是兼容层
  - `gui/` 不拥有协议或传输实现

联机前的本地证明：

- `python -m pytest tools\esp_drone_cli\tests -q` -> `20 passed`
- `powershell -ExecutionPolicy Bypass -File tools\idf.ps1 build` -> `PASS`

## 本次验证中包含的固件与工具修复

最终 serial 复测前，已合入以下修复：

- `DIRECT` IMU 模式现在会请求 `attitude + quaternion + gyro/acc`
  - 修复了 direct 模式 telemetry 中 `gyro_x/y/z` 为零的问题
  - 使 `calib gyro` 无需切回 `RAW` 即可工作
  - 使 `rate-test` 命令路径能够拿到实时 gyro 数据
- CLI `connect` 不再发送第二个冗余的 `HELLO_REQ`
- `reboot` 现在会：
  - 先停止流和测试输出
  - 发送 ACK
  - 等待一个很短的 USB 帧窗口
  - 然后再调用 `esp_restart()`

## Serial 联机测试总结

本轮联机过程中生成了若干本地工件，供台架检查使用：

- 参数导出 JSON
- telemetry CSV dump
- serial 命令矩阵 JSON 摘要

这些工件故意只作为本地测试副产物保留，不要求纳入仓库跟踪。

[CLI 联机测试矩阵](./cli_live_test_matrix.zh-CN.md) 中列出的所有 serial 非起飞命令都已在真机上执行，并得到可接受结果。

## 命令结果

| 命令 | 示例输入 | 预期 | 实际 | 结果 |
|---|---|---|---|---|
| `connect` | `python -m esp_drone_cli --serial COM4 connect` | 打印 `HELLO_RESP` 设备信息 | `DeviceInfo(protocol_version=1, imu_mode=1, arm_state=0, stream_enabled=0, feature_bitmap=15)` | `PASS` |
| `disconnect` | 隐式关闭会话 | 每次命令结束后串口都能干净关闭 | 在整套矩阵中，同一 `COM4` 端口可重复重连 | `PASS` |
| `get` | `... get telemetry_usb_hz` | 读取当前参数 | reset 前返回 `ParamValue(... value=150)`，reset 后返回 `200` | `PASS` |
| `list` | `... list` | 枚举完整参数表 | 返回全部当前参数 | `PASS` |
| `stream on` | `... stream on` | 启用 telemetry 流 | 返回成功，无报错 | `PASS` |
| `stream off` | `... stream off` | 关闭 telemetry 流 | 返回成功，无报错 | `PASS` |
| `set` | `... set telemetry_usb_hz u32 120` | 写入单个参数 | 值更新成功，并可回读 | `PASS` |
| `save` | `... save` | 持久化当前参数集 | 返回成功，无报错 | `PASS` |
| `reset` | `... reset` | 恢复默认值 | `telemetry_usb_hz` 恢复为 `200` | `PASS` |
| `export` | `... export docs/_tmp_cli_live/params_export.json` | 写出快照 JSON | 文件成功创建 | `PASS` |
| `import` | `... import ... --save` | 恢复快照 JSON | 输出 `applied 44 parameters ...`，并从快照恢复对应参数 | `PASS` |
| `arm` | `... arm` | 在台架安全条件满足时解锁 | 返回成功，无报错 | `PASS` |
| `disarm` | `... disarm` | 上锁并停止当前台架模式 | 返回成功，无报错 | `PASS` |
| `kill` | `... kill` | 立即停止输出并进入急停路径 | 返回成功，无报错 | `PASS` |
| `reboot` | `... reboot` | ACK 后重启板卡 | 返回成功，随后同一 `COM4` 可重连 | `PASS` |
| `calib gyro` | `... calib gyro` | IMU 健康时执行陀螺零偏校准 | 在 direct 模式 gyro feed 修复后成功返回 | `PASS` |
| `calib level` | `... calib level` | 执行水平姿态校准 | 返回成功 | `PASS` |
| `motor-test` | `... motor-test m1 0.05`，再 `0.0` | 逐个电机执行开 / 关命令 | `m1..m4` 的低 duty 开关命令都返回成功 | `PASS` |
| `axis-test` | `... axis-test roll 0.05`，再 `0.0` | 按轴施加台架 mixer 偏置 | `roll`、`pitch`、`yaw` 的正向与归零命令都返回成功 | `PASS` |
| `rate-test` | `... rate-test roll 20`，再 `0` | 按轴施加低幅值 rate bench 模式 | `roll`、`pitch`、`yaw` 的正向与归零命令都返回成功 | `PASS` |
| `log` | `... log --timeout 2 --telemetry` | 打印实时事件和 telemetry 数据 | 成功打印 telemetry 字典 | `PASS` |
| `dump-csv` | `... dump-csv docs/_tmp_cli_live/log_dump.csv --duration 2` | 采集 telemetry 到 CSV | 写入 `335` 行 telemetry | `PASS` |

## Telemetry 观察要点

在 `log --timeout 2 --telemetry` 期间观察到：

- `gyro_x/y/z` 在 `DIRECT` 模式下已正确填充
- `battery_voltage` 和 `battery_adc_raw` 存在
- `imu_age_us` 和 `loop_dt_us` 存在
- `arm_state`、`failsafe_reason` 和 `control_mode` 存在
- 测试期间出现过的示例值：
  - `gyro_x = -0.1220703125`
  - `gyro_y = -0.244140625`
  - `gyro_z = 0.06103515625`
  - `battery_voltage ~ 4.20V`
  - `loop_dt_us ~ 1000`
  - `imu_mode = 1`
  - `imu_health = 1`

## Serial 可接受性结论

对当前非起飞范围而言，serial CLI 验证结果可接受：

- `connect` / `disconnect` 在 `COM4` 上稳定
- 参数读写、保存、恢复、导出、导入全部通过
- 安全与系统命令 `arm`、`disarm`、`kill`、`reboot` 全部通过
- 标定与台架动作 `calib`、`motor-test`、`axis-test`、`rate-test` 全部通过
- 日志输出和 CSV 导出均通过

## 延后项

- UDP 联机验证是延后，不是失败
- 原因：当前固件尚未实现设备侧 UDP CLI 路径，因此不阻断 serial CLI 的验收结论

## 失败项与修复记录

在 `2026-04-03` 的复测后，已没有剩余的 serial CLI 阻断故障。

本轮已解决：

1. `calib gyro` 之前在默认 `DIRECT` 模式下失败
   - 原因：direct 模式回传集合中不含 `gyro/acc`
   - 修复：在 direct 模式下请求 `attitude + quaternion + gyro/acc`
   - 复测：`PASS`
2. `reboot` 之前虽然 ACK 成功，但 CLI 无法重新连接
   - 原因：重启序列对 USB CDC 会话来说过于激进
   - 修复：先 ACK，停止流和测试输出，加入短延迟，再执行重启
   - 复测：`PASS`
