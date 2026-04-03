# Python GUI Usage

## Goal

`esp-drone-gui` is the manual bench-debug workbench for the Python toolchain.

It uses the same `esp_drone_cli.core.device_session.DeviceSession` as the CLI, so GUI and CLI share:

- framing
- serial transport
- UDP transport
- telemetry decode
- parameter commands
- device commands
- CSV logging

GUI is for human-in-the-loop bench debugging. CLI remains the preferred entry for automation and scripted tests.

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

The GUI is arranged as a bench-debug workbench:

- left column:
  - connection
  - safety control
  - debug actions
- center column:
  - realtime telemetry table
  - realtime charts
- right column:
  - parameter list
  - selected parameter detail / edit
- bottom:
  - event log
  - last error
  - last result
  - last log path

## Serial Connection Example

1. Plug the aircraft into USB CDC.
2. Start the GUI.
3. In the `Connection` area:
   - select `serial`
   - choose the COM port or type it manually
   - keep baudrate at `115200` unless firmware changes it later
4. Click `Connect`

Expected result:

- connection badge switches to `Connected`
- parameter list refreshes automatically
- telemetry starts updating after `Stream On`

## UDP Connection Example

1. Power the aircraft and make sure the UDP endpoint is reachable.
2. In the `Connection` area:
   - select `udp`
   - set host, for example `192.168.4.1`
   - set port, default `2391`
3. Click `Connect`

Expected result:

- connection badge switches to `Connected`
- commands are sent through the same `DeviceSession` path as serial

## GUI Areas

### Connection

- choose serial or UDP
- configure COM port / baudrate or host / port
- refresh serial ports
- connect and disconnect
- check current connection state
- inspect the last connection error

### Safety Control

- `Arm`
- `Disarm`
- `Kill`
- `Reboot`
- current `arm_state`, `failsafe_reason`, `control_mode`, `imu_mode`
- current stream state badge

### Realtime Telemetry

- `Stream On` / `Stream Off`
- apply target telemetry rate through `telemetry_usb_hz` or `telemetry_udp_hz`
- view live values for gyro, attitude, rate setpoints, motors, battery, loop timing and safety state
- copy selected table cells with `Ctrl+C`

### Charts

The chart area uses `pyqtgraph` and currently provides:

- gyro_x / gyro_y / gyro_z
- roll_deg / pitch_deg / yaw_deg
- motor1 / motor2 / motor3 / motor4
- battery_voltage

Chart controls:

- pause / resume chart updates
- clear history
- 5s / 10s / 30s window
- per-channel visibility checkboxes

### Parameter Debug

- refresh parameter list
- search by parameter name
- select one parameter and edit a new value
- view a local hint / description placeholder
- `Save`
- `Reset`
- `Export JSON`
- `Import JSON`

GUI hints are advisory only. Final validation still happens on the device.

### Debug Actions

- `motor_test`
- `calib gyro`
- `calib level`
- `rate_test`

These actions are for restrained bench use only.

### Log Export

- choose CSV output file
- `Start Log`
- `Stop Log`
- `Dump CSV`
- inspect last log path and latest error/event

## QSettings State

The GUI stores local state through `QSettings`, including:

- window geometry
- splitter positions
- last link type
- last serial port
- last UDP host / port
- last chart window
- last parameter search text
- last CSV output path

## Common Errors

`PyQt5 and pyqtgraph are required for esp-drone-gui`

- install GUI dependencies with `pip install -e .[gui]`

`connect_serial: no serial port selected`

- choose or type a valid COM port before connecting

`connect_udp: no UDP host provided`

- enter a valid host before connecting

`HELLO` timeout or immediate disconnect

- check cable quality
- verify the board is actually running the application, not the ROM downloader
- confirm no other program owns the serial port

No telemetry updates after connect

- click `Stream On`
- verify the correct transport is connected
- verify firmware is running and not held in reset / fault

`set_param` or `save_params` fails

- device-side parameter validation rejected the value
- check parameter ranges and inter-parameter constraints in firmware docs

`dump csv` fails

- check the target path
- verify the device is connected and stream can be started

## Scope Boundary

Current GUI scope is intentionally focused:

- good for bench connection, safety commands, telemetry, charts, parameters, motor/rate test and CSV capture
- not a second protocol stack
- not the primary automation surface
- not yet a heavy waveform-analysis suite
