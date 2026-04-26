# Documentation

Language / 语言: English | [简体中文](./README.zh-CN.md)

This directory contains locked conventions, bring-up notes, docs-code sync notes, and Python tool documentation.

## Start Here

- [CODEX_STATE](./CODEX_STATE.md): short current project memory for Codex
- [TUNING_DECISIONS](./TUNING_DECISIONS.md): decisions that should not be re-litigated every run
- [Axis Truth Table](./axis_truth_table.md)
- [Motor Map](./motor_map.md)

## Firmware And Bring-Up

- [ESP-IDF Environment](./esp_idf_env.md)
- [Bring-Up Checklist](./bringup_checklist.md)
- [Hang Attitude Bring-Up Plan](./hang_attitude_bringup_plan.md)
- [Ground Tune Bring-Up Plan](./ground_tune_bringup_plan.md)
- [Ground Tune Manual Workflow](./ground_tune_manual_workflow.md)
- [Ground Tune Log Fields](./ground_tune_log_fields.md)
- [Attitude Ground Verify And Low-Risk Liftoff Plan](./attitude_ground_verify_liftoff_plan.md)
- [Roll Rate Bench Workflow](./roll_rate_bench_workflow.md)
- [Pitch Rate Bench Workflow](./pitch_rate_bench_workflow.md)
- [Yaw Rate Bench Workflow](./yaw_rate_bench_workflow.md)
- [Rate Bring-Up Results](./rate_bringup_results.md)
- [RAW Mode Attitude Plan](./raw_mode_attitude_plan.md)
- [Barometer Framework](./barometer_framework.md)
- [ESP-Drone UDP Compatibility](./esp_drone_udp_compat.md)
- [ESP32 SoftAP + UDP Transport](./softap_udp_transport.md)
- [Experimental UDP Manual Control](./udp_manual_control_protocol.md)

## Python Tooling

- [Python CLI Usage](./python_cli_usage.md)
- [Python GUI Usage](./python_gui_usage.md)
- [Python GUI Manual Checklist](./python_gui_manual_checklist.md)
- [Python GUI UI Plan](./python_gui_ui_plan.md)
- [Python Tool GUI + CLI Refactor Plan](./python_tool_gui_refactor_plan.md)
- [CLI Live Test Matrix](./cli_live_test_matrix.md)
- [CLI Live Test Results](./cli_live_test_results.md)

## Docs-Code Sync Rules

- Protocol version, feature bits, and command IDs must match `firmware/main/console/console_protocol.h`.
- Current firmware parameter defaults must match `firmware/main/params/params.c`.
- Documentation must clearly distinguish implemented features from TODO/planned features.
- `motor_output_map` is channel mapping, not per-motor thrust compensation.
- Current `motor.c` uses fixed 8-bit LEDC PWM resolution and parameterized PWM frequency.
