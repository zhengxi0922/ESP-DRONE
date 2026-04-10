# ESP-DRONE

**Language / 语言:** **English** | [简体中文](./README.zh-CN.md)

Custom ESP-IDF flight-controller firmware and shared Python tooling for a brushed quadcopter based on `ESP32-S3-WROOM-1-N16R8`.

## Locked Constraints

- `ESP-IDF v5.5.1`
- body frame fixed to `+Y` nose, `+X` right side, `+Z` up
- project naming fixed to `+pitch` nose up, `+roll` right side down, `+yaw` nose right
- `UART0` reserved for `ATK-MS901M`
- `USB CDC` reserved for CLI, GUI, and debug telemetry
- CLI and GUI share one `DeviceSession`

Direction-sensitive logic must follow:

- [docs/axis_truth_table.md](./docs/axis_truth_table.md)
- [docs/motor_map.md](./docs/motor_map.md)

## Repository Layout

- [docs/](./docs/README.md): design constraints, bring-up notes, CLI and GUI usage
- [firmware/](./firmware): ESP-IDF firmware project
- [tools/esp_drone_cli/](./tools/esp_drone_cli): shared Python `core + cli + gui` toolchain

## Current Status

The repository contains a usable three-axis rate-loop bench path:

- firmware supports `rate-test` for `roll`, `pitch`, and `yaw`
- per-axis rate PID parameters are live in the control chain
- CLI supports `rate-test`, `rate-status`, `watch-rate`, parameter edit, save, export, import, and `axis-bench` / `rate-bench`
- the PyQt5 GUI supports rate-test controls, rate-focused charts, rate PID editing, and shared-session command handling
- software verification includes firmware build success and Python tests

For the current bring-up stage, roll tuning remains bench-only and rate-loop only:

- no angle outer loop
- no attitude hang test in this workflow
- no free-flight tuning
- no change to the existing `+roll`, `roll_rate = -gyro_y`, or motor-map conventions

The current documented live roll session was run on a constrained circular-rod bench with natural `+Z down` orientation.
That orientation does not change the roll rate-loop sign convention because this workflow evaluates only `rate_setpoint_roll`, mapped roll feedback, PID output, and motor split.

## Build Firmware

Recommended Windows flow:

```powershell
. .\tools\esp-idf-env.ps1
.\tools\idf.ps1 set-target esp32s3
.\tools\idf.ps1 build
```

Or raw ESP-IDF:

```powershell
cd firmware
idf.py set-target esp32s3
idf.py build
```

## Install Python Tools

CLI only:

```powershell
cd tools\esp_drone_cli
pip install -e .
```

CLI + GUI:

```powershell
cd tools\esp_drone_cli
pip install -e .[gui]
```

## Main Docs

- [docs/roll_rate_bench_workflow.md](./docs/roll_rate_bench_workflow.md)
- [docs/roll_bench_summary_sample.md](./docs/roll_bench_summary_sample.md)
- [docs/python_cli_usage.md](./docs/python_cli_usage.md)
- [docs/bringup_checklist.md](./docs/bringup_checklist.md)
- [docs/python_gui_usage.md](./docs/python_gui_usage.md)
- [docs/rate_bringup_results.md](./docs/rate_bringup_results.md)
- [docs/README.md](./docs/README.md)
