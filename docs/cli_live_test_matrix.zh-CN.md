# CLI 联机测试矩阵

**语言 / Language：** [English](./cli_live_test_matrix.md) | **简体中文**

这份矩阵用于记录当前 Python 工具链中“非起飞命令”的 CLI 联机验证范围。

默认前提：

- 已拆桨或机体已固定
- 不进行自由飞
- `motor-test` 仅限低功率
- `axis-test` 与 `rate-test` 仅限低幅值
- 优先覆盖 serial，只有设备侧路径实际可用时才补测 UDP

| 命令 / 区域 | 示例输入 | 需要真机 | 需要电机上电 | 可在无桨状态下执行 | Serial 目标 | UDP 目标 | 当前状态 |
|---|---|---:|---:|---:|---:|---:|---|
| `connect` | `python -m esp_drone_cli --serial COM4 connect` | yes | no | yes | yes | yes | `PASS` on `COM4` |
| `disconnect` | 命令退出后隐式关闭会话 | yes | no | yes | yes | yes | 通过 serial 矩阵中的重复开关验证，`PASS` |
| `get` | `... get telemetry_usb_hz` | yes | no | yes | yes | yes | serial `PASS` |
| `list` | `... list` | yes | no | yes | yes | yes | serial `PASS` |
| `stream on` | `... stream on` | yes | no | yes | yes | yes | serial `PASS` |
| `stream off` | `... stream off` | yes | no | yes | yes | yes | serial `PASS` |
| `set` | `... set telemetry_usb_hz u32 100` | yes | no | yes | yes | yes | serial `PASS` |
| `save` | `... save` | yes | no | yes | yes | yes | serial `PASS` |
| `reset` | `... reset` | yes | no | yes | yes | yes | serial `PASS` |
| `export` | `... export params.json` | yes | no | yes | yes | yes | serial `PASS` |
| `import` | `... import params.json` | yes | no | yes | yes | yes | serial `PASS` |
| `arm` | `... arm` | yes | yes | yes | yes | yes | serial 台架 `PASS` |
| `disarm` | `... disarm` | yes | yes | yes | yes | yes | serial 台架 `PASS` |
| `kill` | `... kill` | yes | yes | yes | yes | yes | serial 台架 `PASS` |
| `reboot` | `... reboot` | yes | no | yes | yes | yes | serial `PASS`，重启后重连已验证 |
| `calib gyro` | `... calib gyro` | yes | no | yes | yes | yes | serial `PASS` |
| `calib level` | `... calib level` | yes | no | yes | yes | yes | serial `PASS` |
| `motor-test` | `... motor-test m1 0.05` | yes | yes | yes | yes | yes | serial 低 duty `PASS`，覆盖 `m1..m4` 开关 |
| `axis-test` | `... axis-test roll 0.05` | yes | yes | yes | yes | yes | serial 命令路径对 `roll/pitch/yaw` 均 `PASS` |
| `rate-test` | `... rate-test roll 20` | yes | yes | yes | yes | yes | serial 命令路径对 `roll/pitch/yaw` 均 `PASS` |
| `log` | `... log --timeout 3 --telemetry` | yes | no | yes | yes | yes | serial `PASS` |
| `dump-csv` | `... dump-csv sample.csv --duration 3` | yes | no | yes | yes | yes | serial `PASS` |

## 说明

- 当前没有独立的 `disconnect` 子命令；进程退出时由 `DeviceSession.close()` 隐式断开
- serial 联机验证于 `2026-04-03` 在 `COM4` 上完成
- `DIRECT` 模式现在会请求 `attitude + quaternion + gyro/acc`，因此无需切回 `RAW` 就能使用 `gyro_x/y/z`、`calib gyro` 和 `rate-test` 路径
- 只有在同一 `COM4` 端口上能够再次 `connect` 时，`reboot` 才算验证通过
- UDP 验证仍然延后；当前固件任务图还未真正暴露设备侧 UDP CLI 路径，因此这不阻断 serial CLI 的可接受结论
