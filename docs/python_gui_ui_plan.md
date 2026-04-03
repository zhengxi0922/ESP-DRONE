# Python GUI UI Plan

## Goal

Upgrade the Python GUI from the earlier minimal PySide6 shell to a denser bench-debug workbench based on `PyQt5` and `pyqtgraph`, while keeping `DeviceSession` as the only session owner.

## Scope Boundary

This GUI round only changes:

- Python GUI binding: `PySide6 -> PyQt5`
- GUI packaging extras
- GUI layout and interaction flow
- GUI-side plotting and local state persistence
- README / GUI docs / GUI smoke tests

This GUI round does **not** change:

- firmware
- core protocol
- transport implementations
- `DeviceSession` ownership model
- CLI command names

## Ownership Rule

Single owner remains unchanged:

- session / command flow: `esp_drone_cli.core.device_session.DeviceSession`
- protocol: `esp_drone_cli.core.protocol.*`
- transport: `esp_drone_cli.core.transport.*`
- telemetry / parameter models: `esp_drone_cli.core.models`

GUI must keep calling `DeviceSession`. It must not assemble frames, talk to transport objects directly, or add a second client/session abstraction.

## Files To Change

- `tools/esp_drone_cli/pyproject.toml`
- `tools/esp_drone_cli/esp_drone_cli/gui_main.py`
- `tools/esp_drone_cli/esp_drone_cli/gui/main_window.py`
- `tools/esp_drone_cli/tests/test_core_session.py`
- `README.md`
- `docs/python_gui_usage.md`
- `docs/python_gui_manual_checklist.md`
- `docs/python_tool_gui_refactor_plan.md`

## Dependency Change

- remove GUI binding dependency on `PySide6`
- add GUI binding dependency on `PyQt5`
- add lightweight plotting dependency on `pyqtgraph`

Expected install flow:

- CLI only: `pip install -e .`
- CLI + GUI: `pip install -e .[gui]`

## Layout Sketch

```text
+----------------------------------------------------------------------------------+
| Connection / Status | Realtime telemetry table | Parameters / detail / set/save |
| Safety control      | Charts                   | search / import / export         |
| Debug actions       |                          |                                  |
+----------------------------------------------------------------------------------+
| Event log / errors / recent result / last log path                              |
+----------------------------------------------------------------------------------+
```

## GUI Areas

- Left:
  - connection
  - safety control
  - debug actions
- Center:
  - telemetry table
  - pyqtgraph charts
- Right:
  - parameter table
  - selected parameter detail
- Bottom:
  - event log
  - error text
  - recent result

## Chart Set

First chart bundle:

- gyro_x / gyro_y / gyro_z
- roll_deg / pitch_deg / yaw_deg
- motor1 / motor2 / motor3 / motor4
- battery_voltage

Controls:

- pause/resume updates
- clear history
- 5s / 10s / 30s window
- per-channel checkbox visibility

## Local GUI State

Use `QSettings` for:

- window geometry
- splitter state
- last link type
- last serial port
- last baudrate
- last UDP host / port
- last chart window
- last parameter filter
- last CSV path

## Test Targets

- missing `PyQt5` or `pyqtgraph` does not break CLI
- GUI entry reports clear missing dependency error
- main window can create and close
- close path disconnects session
- GUI buttons route to mocked `DeviceSession`

## Acceptance For This Round

- GUI launches on Windows with `PyQt5`
- CLI remains compatible
- GUI + CLI both use `DeviceSession`
- GUI supports manual bench debugging with table + charts + parameters + logging
