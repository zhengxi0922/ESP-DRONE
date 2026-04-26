# 文档目录

语言 / Language: 简体中文 | [English](./README.md)

本目录包含固定约定、bring-up 记录、docs-code 同步说明和 Python 工具文档。

## 优先阅读

- [CODEX_STATE](./CODEX_STATE.md)：给 Codex 使用的短项目记忆
- [TUNING_DECISIONS](./TUNING_DECISIONS.md)：不应每轮重复争论的技术决定
- [Axis Truth Table](./axis_truth_table.zh-CN.md)
- [Motor Map](./motor_map.zh-CN.md)

## 固件和 Bring-Up

- [ESP-IDF Environment](./esp_idf_env.zh-CN.md)
- [Bring-Up Checklist](./bringup_checklist.zh-CN.md)
- [Hang Attitude Bring-Up Plan](./hang_attitude_bringup_plan.zh-CN.md)
- [Ground Tune Bring-Up Plan](./ground_tune_bringup_plan.md)
- [Ground Tune Manual Workflow](./ground_tune_manual_workflow.md)
- [Ground Tune Log Fields](./ground_tune_log_fields.md)
- [Attitude Ground Verify And Low-Risk Liftoff Plan](./attitude_ground_verify_liftoff_plan.md)
- [Roll Rate Bench Workflow](./roll_rate_bench_workflow.zh-CN.md)
- [Pitch Rate Bench Workflow](./pitch_rate_bench_workflow.md)
- [Yaw Rate Bench Workflow](./yaw_rate_bench_workflow.md)
- [Rate Bring-Up Results](./rate_bringup_results.zh-CN.md)
- [RAW Mode Attitude Plan](./raw_mode_attitude_plan.zh-CN.md)
- [Barometer Framework](./barometer_framework.zh-CN.md)
- [ESP-Drone UDP Compatibility](./esp_drone_udp_compat.zh-CN.md)
- [ESP32 SoftAP + UDP Transport](./softap_udp_transport.md)
- [Experimental UDP Manual Control](./udp_manual_control_protocol.md)

## Python 工具链

- [Python CLI Usage](./python_cli_usage.zh-CN.md)
- [Python GUI Usage](./python_gui_usage.zh-CN.md)
- [Python GUI Manual Checklist](./python_gui_manual_checklist.zh-CN.md)
- [Python GUI UI Plan](./python_gui_ui_plan.zh-CN.md)
- [Python Tool GUI + CLI Refactor Plan](./python_tool_gui_refactor_plan.zh-CN.md)
- [CLI Live Test Matrix](./cli_live_test_matrix.zh-CN.md)
- [CLI Live Test Results](./cli_live_test_results.zh-CN.md)

## Docs-Code 同步规则

- protocol version、feature bits、command IDs 必须和 `firmware/main/console/console_protocol.h` 一致。
- 当前固件默认参数必须和 `firmware/main/params/params.c` 一致。
- 文档必须明确区分 implemented 和 TODO/planned。
- `motor_output_map` 是通道映射，不是每电机推力补偿。
- 当前 `motor.c` 使用固定 8-bit LEDC PWM 分辨率，只有 PWM 频率参数化。
