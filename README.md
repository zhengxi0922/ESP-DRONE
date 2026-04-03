# ESP-DRONE

Custom ESP-IDF flight-controller firmware and Python CLI for a brushed quadcopter based on `ESP32-S3-WROOM-1-N16R8`.

## Locked Environment

- MCU: `ESP32-S3-WROOM-1-N16R8`
- ESP-IDF: `v5.5.1`
- RTOS: ESP-IDF FreeRTOS
- IMU transport: `UART0` to `ATK-MS901M`
- Console / CLI transport: `USB CDC`

## Body And Attitude Convention

The whole project uses this body frame:

- `+Y`: nose / front
- `+X`: right side
- `+Z`: upward

The whole project uses these positive attitude names:

- `+pitch`: nose up
- `+roll`: right side down
- `+yaw`: nose right, clockwise viewed from above

Important: the body frame is right-handed, but the positive names for `roll` and `yaw` do not follow the default mathematical positive rotation around `+Y` and `+Z`. Every direction-sensitive module must reference `docs/axis_truth_table.md` and `docs/motor_map.md`.

## Repository Layout

- `docs/`: extracted constraints, protocol notes, bring-up and validation docs
- `firmware/`: ESP-IDF firmware project
- `tools/esp_drone_cli/`: Python CLI and protocol helpers

## Build Notes

This repository is written for ESP-IDF `v5.5.1`.

On this Windows workstation, the verified local installation is documented in `docs/esp_idf_env.md` and can be activated directly from PowerShell:

```powershell
. .\tools\esp-idf-env.ps1
```

The repository also includes a wrapper for running `idf.py` against `firmware/` without manually changing directories:

```powershell
.\tools\idf.ps1 --version
.\tools\idf.ps1 build
```

For a clean checkout, set the target once before the first build:

```powershell
.\tools\idf.ps1 set-target esp32s3
```

If you prefer the raw ESP-IDF shell flow, the firmware can still be built this way:

```powershell
cd firmware
idf.py set-target esp32s3
idf.py build
```

## Current Stage

The repository currently contains:

- Stage 1 design documents
- Stage 2 firmware skeleton and bottom-layer components
- Stage 2.5 bring-up code paths for `motor-test`, `axis-test`, `rate-test`, IMU mapping, mixer direction checks and structured telemetry
- Minimal single-axis rate-loop skeleton: fresh-sample estimator update, rate PID, mixer, motor output and safety gating
- Baseline safety shell for `arm / disarm / kill`, state ownership, and a small set of hard stop triggers
- Shared Python tool core for protocol, transport, telemetry decode, parameter snapshots and device commands
- CLI entrypoint for scripting and automation
- PyQt5 GUI workbench for manual bench debugging on Windows, backed by the same shared `DeviceSession`

Current hardware validation status:

- Host build and host-side direction tests pass
- Real bench validation is still blocked until the aircraft enumerates on USB CDC and can be exercised on a restrained, prop-removed stand

Still pending in later stages:

- Full safety / failsafe logic with RC gesture arming, link supervision, richer fault classification, telemetry reason reporting, and final policy refinement
- Completed hardware bench validation for single-axis rate mode
- DIRECT-mode angle outer loop and restrained bench angle validation
- Legacy RC / UDP compatibility work and the full CLI surface

## Python Tool Usage

The Python tooling now supports both a script-first CLI and a desktop GUI workbench. They share the same `DeviceSession` core and do not maintain separate protocol stacks.

Related docs:

- [GUI refactor plan](./docs/python_tool_gui_refactor_plan.md)
- [GUI UI plan](./docs/python_gui_ui_plan.md)
- [GUI usage guide](./docs/python_gui_usage.md)
- [GUI manual checklist](./docs/python_gui_manual_checklist.md)

The Python tooling now uses a shared `core + cli + gui` structure documented in [python_tool_gui_refactor_plan.md](./docs/python_tool_gui_refactor_plan.md).

Phase B ownership rule:

- `esp_drone_cli.core.device_session.DeviceSession` is the only session / command owner
- `esp_drone_cli.core.protocol.*` is the only protocol owner
- `esp_drone_cli.core.transport.*` is the only transport owner
- top-level `client.py`, `protocol/*`, and `transport/*` modules are compatibility shims only

This is intentional. CLI and GUI must both call the same core session layer rather than maintain separate protocol stacks.

### Windows Install

CLI only:

```powershell
cd tools\esp_drone_cli
pip install -e .
```

This installs the Python package with the CLI entrypoint only. It does not pull in the optional GUI dependencies.

CLI + GUI:

```powershell
cd tools\esp_drone_cli
pip install -e .[gui]
```

This installs the same package plus the optional `PyQt5 + pyqtgraph` dependencies required by the GUI.

`PyQt5` is optional. If it is not installed:

- CLI still imports and runs normally
- GUI startup fails fast with a clear error telling you to install `pip install -e .[gui]`

### CLI Start

Examples:

```powershell
python -m esp_drone_cli --serial COM7 connect
python -m esp_drone_cli --serial COM7 arm
python -m esp_drone_cli --serial COM7 dump-csv telemetry.csv --duration 5
```

The installed script entrypoint is also available:

```powershell
esp-drone-cli --serial COM7 connect
```

### GUI Start

```powershell
esp-drone-gui
```

or:

```powershell
python -m esp_drone_cli.gui_main
```

The GUI is intended for manual bench debugging. CLI remains the primary interface for automation and scripted tests.

### GUI Capability Boundary

The current GUI is a PyQt5 workbench focused on restrained bench debugging. It is intended to speed up manual debugging sessions, not to replace the CLI or grow a second protocol stack.

Current GUI capabilities:

- serial / UDP connect and disconnect
- `arm / disarm / kill / reboot`
- `stream on / off`
- realtime telemetry table with copy support
- pyqtgraph charts for gyro, attitude, motor outputs and battery voltage
- parameter refresh, search, set selected, save, reset, import and export
- `motor-test`
- `calib gyro` / `calib level`
- `rate-test`
- CSV logging and CSV dump
- local GUI state persistence through `QSettings`

Still intentionally deferred:

- GUI-only protocol or transport behavior
- stock App / RC / legacy UDP compatibility workflows

For automation, scripted regression checks and repeatable data capture, CLI remains the preferred interface.

## Stage-2 Console Rule

Stage 2 locks the debug transport split as follows:

- `UART0` is reserved for the `ATK-MS901M` IMU only
- Application console, debug log and CLI transport must use `USB CDC` only
- `sdkconfig.defaults` must keep UART console disabled so no application console traffic leaks onto the IMU UART

## Acknowledgements

This project is a from-scratch rewrite for a custom quadcopter, but it intentionally acknowledges the open-source work that helped define the compatibility and bring-up targets:

- `ESP-Drone` and the related legacy app / UDP ecosystem, used here as a behavior and compatibility reference
- `Bitcraze Crazyflie`, whose open flight-control ecosystem influenced many small-drone software stacks
- `ALIENTEK / ATK`, for the `ATK-MS901M` module documentation and example materials used to reconstruct the IMU protocol

Important:

- This repository does **not** inherit the old repository's body frame, attitude sign convention, mixer, or control-chain implementation by default.
- Legacy projects are referenced for protocol, visual semantics, and hardware bring-up only; direction-sensitive logic in this repository is redefined explicitly in `docs/axis_truth_table.md` and `docs/motor_map.md`.
