# 旧版 LED 行为参考

**语言 / Language：** [English](./old_led_behavior.md) | **简体中文**

## 来源范围

旧仓库只提供了部分 LED 语义。它可以作为视觉语义参考，但不能直接当作这次重写的完整状态机。

## 提取出的 Legacy 语义

| Legacy 语义 | 观察到的行为 |
|---|---|
| `alive` | 慢速心跳脉冲 |
| `lowbat` | 低电压告警图案 |
| `charging` | 重复的慢速脉冲 |
| `charged` | 常亮式指示 |
| `linkUp` | 短暂活动闪烁 |
| `linkDown` | 在另一条链路 LED 上出现短闪 |
| `testPassed` | 启动脉冲序列 |
| `testFailed` | 系统 LED 启动脉冲序列 |

## 重写决策

新固件使用独立的 `led_status` 状态机。其它模块只能上报逻辑状态，不能直接驱动 GPIO。

## 新默认状态表

| 状态 | LED 行为 |
|---|---|
| `INIT_WAIT_IMU` | 黄色慢闪 |
| `DISARMED_READY` | 黄色常亮 |
| `ARMED_HEALTHY` | 绿色常亮 |
| `LOW_BAT` | 红色慢闪 |
| `FAILSAFE` | 红色快闪 |
| `IMU_ERROR` | 红色快闪 |
| `RC_LOSS` | 红色快闪 |
| `FAULT_LOCK` | 红色常亮 |
| `CALIBRATING` | 绿黄交替 |
| `PARAM_SAVE` | 绿黄交替 |

## 优先级

`FAULT_LOCK > FAILSAFE/IMU/RC error > LOW_BAT > CALIB/SAVE > ARMED_HEALTHY > DISARMED_READY > INIT_WAIT_IMU`
