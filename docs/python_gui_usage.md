# Python GUI Usage

**Language / 语言：** **English** | [简体中文](./python_gui_usage.zh-CN.md)

## Goal

`esp-drone-gui` is the manual bench-debug workbench for the Python toolchain.

It uses the same `esp_drone_cli.core.device_session.DeviceSession` as the CLI, so GUI and CLI share:

- framing
- serial transport
- UDP transport
- telemetry decoding
- parameter commands
- device commands
- CSV logging

The GUI is for human-in-the-loop bench debugging. The CLI remains the preferred entry for automation and scripted tests.

## Install On Windows

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

The `gui` extra installs:

- `PyQt5`
- `pyqtgraph`

If `PyQt5` or `pyqtgraph` is missing:

- `esp-drone-cli` still works
- `esp-drone-gui` exits with a clear install hint

## Start GUI

Installed entrypoint:

```powershell
esp-drone-gui
```

Module entrypoint:

```powershell
python -m esp_drone_cli.gui_main
```

## Window Layout

The GUI is arranged as a three-column workbench plus a collapsible bottom log:

- left column, narrow:
  - connection
  - safety control
  - debug actions
- center column, largest:
  - one large realtime chart
  - live numeric telemetry table below it
- right column:
  - key status cards
  - large parameter table and compact edit or detail area
- bottom log:
  - recent events
  - last result
  - last log path
  - last error

The chart and numeric telemetry stay visible at the same time. They are no longer separated into mutually exclusive tabs.

## Language

- default UI language: Chinese
- top-right language switch: `中文 / English`

The main controls, labels, group titles, and status texts are localized for bench use.

## Serial Connection Example

1. Plug the aircraft into USB CDC.
2. Start the GUI.
3. In the `Connection` area:
   - choose `Serial`
   - choose the COM port or type it manually
   - keep the baudrate at `115200` unless the firmware changes later
4. Click `Connect`.

Expected result:

- the connection badge switches to connected
- the parameter list refreshes automatically
- telemetry starts updating after `Stream On`

## UDP Connection Example

1. Power the aircraft and make sure the UDP endpoint is reachable.
2. In the `Connection` area:
   - choose `UDP`
   - set the host, for example `192.168.4.1`
   - set the port, default `2391`
3. Click `Connect`.

Expected result:

- the connection badge switches to connected
- commands are sent through the same `DeviceSession` path as serial

## GUI Areas

### Left Column

Connection:

- choose serial or UDP
- serial mode only shows COM and baudrate controls
- UDP mode only shows host and port controls
- refresh serial ports
- connect and disconnect
- inspect current session info and the last connection error

Safety:

- `Arm`
- `Disarm`
- `Kill`
- `Reboot`

Debug actions:

- `motor_test`
- `calib gyro`
- `calib level`
- `rate_test`
- `Start Log`
- `Stop Log`
- `Dump CSV`

These actions are for restrained bench use only.

### Center Column

Main chart:

- one large plot driven by `pyqtgraph`
- switch chart group:
  - `Gyro`
  - `Attitude`
  - `Motors`
  - `Battery`
  - `Barometer`
- pause or resume
- clear history
- auto scale
- reset view
- `5s`, `10s`, `30s` window
- per-channel visibility checkboxes

Live numeric telemetry:

- `Stream On` or `Stream Off`
- apply target telemetry rate through `telemetry_usb_hz` or `telemetry_udp_hz`
- view live values for gyro, attitude, rate setpoints, motors, battery, barometer, loop timing, and safety state
- copy selected table cells with `Ctrl+C`

### Right Column

Key status cards:

- `arm_state`
- `failsafe_reason`
- `control_mode`
- `imu_mode`
- `stream`
- `battery_voltage`
- `baro_altitude_m`
- `baro_health`
- `imu_age_us`
- `loop_dt_us`

Parameter debug:

- refresh the parameter list
- search by parameter name
- select one parameter and edit a new value
- view a local hint and compact description area
- `Save`
- `Reset`
- `Export JSON`
- `Import JSON`

GUI hints are advisory only. Final validation still happens on the device.

### Bottom Log

- event-log area for recent command results and errors
- clear, copy, and save log
- collapsed by default to a compact height
- can be expanded or collapsed through the arrow button

## QSettings State

The GUI stores local state through `QSettings`, including:

- window geometry
- splitter positions
- last link type
- last serial port
- last UDP host or port
- last chart group
- last chart window
- last parameter search text
- last CSV output path
- last selected language
- bottom-log collapsed or expanded state

## Common Errors

`PyQt5 and pyqtgraph are required for esp-drone-gui`

- install GUI dependencies with `pip install -e .[gui]`

No serial port selected

- choose or type a valid COM port before connecting

No UDP host provided

- enter a valid host before connecting

`HELLO` timeout or immediate disconnect

- check cable quality
- verify that the board is running the application rather than the ROM downloader
- confirm that no other program owns the serial port

No telemetry updates after connect

- click `Stream On`
- verify that the correct transport is connected
- verify that the firmware is running and not held in reset or fault

`set_param` or `save_params` fails

- device-side parameter validation rejected the value
- check parameter ranges and inter-parameter constraints in the firmware docs

`dump csv` fails

- check the target path
- verify that the device is connected and the stream can be started

## Scope Boundary

Current GUI scope is intentionally focused:

- good for bench connection, safety commands, telemetry, charts, parameters, motor or rate test, and CSV capture
- not a second protocol stack
- not the primary automation surface
- not yet a heavy waveform-analysis suite
