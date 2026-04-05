# 待确认问题

**语言 / Language：** [English](./open_questions.md) | **简体中文**

当前已经没有阻断阶段 1 推进的硬性问题。下面这些项都属于非阻断项，依赖设计默认值加 bring-up 实测来完成确认。

## 非阻断项

- 通过物理动作和 telemetry 符号检查，在硬件上验证默认 IMU 安装方向假设
- 在 bring-up 之后决定 `250 Hz` 是否应该升级为推荐 IMU 档位，还是继续仅作为可选高档位
- 如果后续需要更完整的 stock App 对等性，抓包并验证是否存在独立的 legacy `arm`、`disarm` 或 `mode` 包
- 在 bring-up 中验证 LED 有效电平，因为当前阶段默认假设 GPIO 高电平表示 LED 点亮
- 在 bring-up 中验证电机 PWM 极性，尤其是增大 duty 是否真的会提高有刷电机输出

## 已锁定决策

- ESP-IDF 版本固定为 `v5.5.1`
- `ATK-IMU901` 和 `ATK-MS901M` 视为同一模块族
- legacy UDP 兼容范围已明确做了裁剪
