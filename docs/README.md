# Documentation

**Language / 语言：** **English** | [简体中文](./README.zh-CN.md)

This directory contains the repository's design constraints, bring-up notes, compatibility decisions, and Python tool documentation.

## Core Conventions

- [Hardware Extract](./hardware_extract.md)
- [Axis Truth Table](./axis_truth_table.md)
- [Motor Map](./motor_map.md)
- [Runtime Frequency Plan](./runtime_frequency_plan.md)
- [IMU Protocol Extract](./imu_protocol_extract.md)
- [Old LED Behavior](./old_led_behavior.md)
- [Open Questions](./open_questions.md)

## Firmware And Integration

- [ESP-IDF Environment](./esp_idf_env.md)
- [Bring-Up Checklist](./bringup_checklist.md)
- [Rate Bring-Up Results](./rate_bringup_results.md)
- [RAW Mode Attitude Plan](./raw_mode_attitude_plan.md)
- [Barometer Framework](./barometer_framework.md)
- [ESP-Drone UDP Compatibility](./esp_drone_udp_compat.md)

## Python Tooling

- [Python Tool GUI + CLI Refactor Plan](./python_tool_gui_refactor_plan.md)
- [Python GUI UI Plan](./python_gui_ui_plan.md)
- [Python GUI Usage](./python_gui_usage.md)
- [Python GUI Manual Checklist](./python_gui_manual_checklist.md)
- [CLI Live Test Matrix](./cli_live_test_matrix.md)
- [CLI Live Test Results](./cli_live_test_results.md)

## Notes

- English pages always link to English pages when a matching file exists.
- Chinese pages link to Chinese pages first, then fall back to English only when needed.
- All user-facing Markdown pages in `docs/` now follow the same language-split pattern.
