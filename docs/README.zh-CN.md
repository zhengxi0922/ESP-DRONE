# 文档索引

**语言 / Language：** [English](./README.md) | **简体中文**

本目录收录仓库中的设计约束、bring-up 说明、兼容性决策以及 Python 工具文档。

## 核心约定

- [硬件提取](./hardware_extract.zh-CN.md)
- [轴定义真值表](./axis_truth_table.zh-CN.md)
- [电机映射](./motor_map.zh-CN.md)
- [运行频率规划](./runtime_frequency_plan.zh-CN.md)
- [IMU 协议提取](./imu_protocol_extract.zh-CN.md)
- [旧版 LED 行为参考](./old_led_behavior.zh-CN.md)
- [待确认问题](./open_questions.zh-CN.md)

## 固件与联调

- [ESP-IDF 环境](./esp_idf_env.zh-CN.md)
- [Bring-up 清单](./bringup_checklist.zh-CN.md)
- [速率环 Bring-up 结果](./rate_bringup_results.zh-CN.md)
- [RAW 模式姿态方案](./raw_mode_attitude_plan.zh-CN.md)
- [气压计框架](./barometer_framework.zh-CN.md)
- [ESP-Drone UDP 兼容说明](./esp_drone_udp_compat.zh-CN.md)

## Python 工具链

- [Python 工具 GUI + CLI 重构计划](./python_tool_gui_refactor_plan.zh-CN.md)
- [Python GUI 界面规划](./python_gui_ui_plan.zh-CN.md)
- [Python GUI 使用说明](./python_gui_usage.zh-CN.md)
- [Python GUI 人工检查清单](./python_gui_manual_checklist.zh-CN.md)
- [CLI 联机测试矩阵](./cli_live_test_matrix.zh-CN.md)
- [CLI 联机测试结果](./cli_live_test_results.zh-CN.md)

## 说明

- 英文文档优先链接英文版本。
- 中文文档优先链接中文版本；若未来个别页面暂未翻译，再回退到英文版本。
- `docs/` 下的用户文档现在统一采用“按语言拆分文件 + 顶部语言导航”的结构。
