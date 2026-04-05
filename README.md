# ESP-DRONE

**Language / 语言：** **English** | [简体中文](./README.zh-CN.md)

Custom ESP-IDF flight-controller firmware and Python tooling for a brushed quadcopter based on `ESP32-S3-WROOM-1-N16R8`.

## Overview

This repository is a from-scratch flight-controller rewrite for a custom quadcopter. It includes:

- ESP-IDF firmware for the target board
- design, bring-up, and validation documents under [`docs/`](./docs/README.md)
- shared Python tooling with both CLI and GUI entrypoints

The current project constraints are:

- firmware and documentation are locked to `ESP-IDF v5.5.1`
- the body frame is fixed to `+Y` nose, `+X` right side, `+Z` up
- positive attitude naming is fixed to `+pitch` nose up, `+roll` right side down, `+yaw` nose right
- `UART0` is reserved for `ATK-MS901M`
- `USB CDC` is reserved for debug logs and the CLI protocol
- the Python CLI and GUI share one `DeviceSession`

## Locked Environment

- MCU: `ESP32-S3-WROOM-1-N16R8`
- ESP-IDF: `v5.5.1`
- RTOS: ESP-IDF FreeRTOS
- IMU transport: `UART0` to `ATK-MS901M`
- console and CLI transport: `USB CDC`

## Conventions

The project uses one fixed body frame:

- `+Y`: nose / front
- `+X`: right side
- `+Z`: up

The project also uses one fixed attitude naming convention:

- `+pitch`: nose up
- `+roll`: right side down
- `+yaw`: nose right, clockwise viewed from above

The body frame is right-handed, but the project-defined positive names for `roll` and `yaw` do not follow the default mathematical positive rotation about `+Y` and `+Z`. Every direction-sensitive implementation must follow:

- [`docs/axis_truth_table.md`](./docs/axis_truth_table.md)
- [`docs/motor_map.md`](./docs/motor_map.md)

## Repository Layout

- [`docs/`](./docs/README.md): extracted constraints, protocol notes, bring-up records, and tool documentation
- [`firmware/`](./firmware): ESP-IDF firmware project
- [`tools/esp_drone_cli/`](./tools/esp_drone_cli): shared Python `core + cli + gui` toolchain

## Build Notes

This repository is written for `ESP-IDF v5.5.1`.

On Windows, the verified local installation is documented in [`docs/esp_idf_env.md`](./docs/esp_idf_env.md). The recommended shell flow is:

```powershell
. .\tools\esp-idf-env.ps1
.\tools\idf.ps1 build
```

For a clean checkout, set the target once before the first build:

```powershell
.\tools\idf.ps1 set-target esp32s3
```

If you prefer the raw ESP-IDF flow:

```powershell
cd firmware
idf.py set-target esp32s3
idf.py build
```

## Current Status

The repository currently contains:

- stage-1 design documents
- stage-2 firmware skeleton and bottom-layer components
- stage-2.5 bring-up paths for `motor-test`, `axis-test`, `rate-test`, IMU mapping, mixer direction checks, and structured telemetry
- a minimal single-axis rate-loop skeleton with fresh-sample estimator updates, rate PID, mixer, motor output, and safety gating
- a shared Python tool core for protocol, transport, telemetry decoding, parameter snapshots, and device commands
- a CLI entrypoint for automation and scripted checks
- a PyQt5 GUI workbench for manual bench debugging on Windows
- a first-stage barometer telemetry path for `ATK-MS901M`

Current validation status:

- host build and host-side direction tests pass
- serial CLI live verification has been recorded for the current non-flight scope
- full restrained bench validation remains a staged hardware task

Still pending in later stages:

- full safety and failsafe policy refinement
- completed hardware bring-up for single-axis rate mode
- direct-mode angle outer-loop work after the bring-up gate is passed
- fuller RC, legacy UDP, and non-bench workflow coverage

## Barometer Framework

The repository includes a first-stage barometer data path for `ATK-MS901M`.

Current scope:

- decode module barometer frames in firmware
- expose barometer telemetry through USB CDC
- show barometer fields in CLI, GUI, and CSV export
- reserve altitude-hold state structures and control-mode enum values

Not in scope yet:

- altitude-hold PID
- throttle closed loop
- automatic takeoff or landing
- feeding barometer data into the current attitude or rate control path

See [`docs/barometer_framework.md`](./docs/barometer_framework.md) for details.

## Python Tooling

The Python tooling supports both:

- CLI for automation, scripted regression checks, and bulk export
- GUI for human-in-the-loop bench debugging

Both entrypoints share the same `DeviceSession`. They do not maintain separate protocol or transport stacks.

Installation:

- CLI only:

```powershell
cd tools\esp_drone_cli
pip install -e .
```

- CLI + GUI:

```powershell
cd tools\esp_drone_cli
pip install -e .[gui]
```

If `PyQt5` is not installed, the CLI still works and the GUI exits with a clear install hint.

CLI examples:

```powershell
python -m esp_drone_cli --serial COM7 connect
python -m esp_drone_cli --serial COM7 arm
python -m esp_drone_cli --serial COM7 dump-csv telemetry.csv --duration 5
```

Installed entrypoints:

```powershell
esp-drone-cli --serial COM7 connect
esp-drone-gui
```

Related documentation:

- [`docs/python_tool_gui_refactor_plan.md`](./docs/python_tool_gui_refactor_plan.md)
- [`docs/python_gui_ui_plan.md`](./docs/python_gui_ui_plan.md)
- [`docs/python_gui_usage.md`](./docs/python_gui_usage.md)
- [`docs/python_gui_manual_checklist.md`](./docs/python_gui_manual_checklist.md)
- [`docs/cli_live_test_matrix.md`](./docs/cli_live_test_matrix.md)
- [`docs/cli_live_test_results.md`](./docs/cli_live_test_results.md)

## Stage-2 Console Rule

Stage 2 locks the debug transport split as follows:

- `UART0` is reserved for the `ATK-MS901M` IMU only
- application console, debug log, and CLI transport must use `USB CDC` only
- `sdkconfig.defaults` must keep the UART console disabled so no application console traffic leaks onto the IMU UART

## Documentation

Start with the documentation hub:

- [`docs/README.md`](./docs/README.md)

Recommended first reads:

- [`docs/hardware_extract.md`](./docs/hardware_extract.md)
- [`docs/axis_truth_table.md`](./docs/axis_truth_table.md)
- [`docs/motor_map.md`](./docs/motor_map.md)
- [`docs/runtime_frequency_plan.md`](./docs/runtime_frequency_plan.md)
- [`docs/python_gui_usage.md`](./docs/python_gui_usage.md)

## Acknowledgements

This project is a from-scratch rewrite for a custom quadcopter, but it intentionally acknowledges open-source work that helped define compatibility and bring-up targets:

- `ESP-Drone` and the related legacy app and UDP ecosystem, used here as behavior and compatibility references
- `Bitcraze Crazyflie`, whose open flight-control ecosystem influenced many small-drone software stacks
- `ALIENTEK / ATK`, for the `ATK-MS901M` documentation and example materials used to reconstruct the IMU protocol

Important notes:

- this repository does not inherit the old repository's body frame, attitude sign convention, mixer, or control-chain implementation by default
- legacy projects are referenced for protocol, visual semantics, and hardware bring-up only
- all direction-sensitive logic in this repository is explicitly redefined in [`docs/axis_truth_table.md`](./docs/axis_truth_table.md) and [`docs/motor_map.md`](./docs/motor_map.md)
