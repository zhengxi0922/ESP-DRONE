# Python GUI Usage

## Goal

`esp-drone-gui` is the manual bench-debug shell for the Python toolchain.

It is built on the same `esp_drone_cli.core.device_session.DeviceSession` used by CLI, so GUI and CLI share:

- framing
- serial transport
- UDP transport
- telemetry decode
- parameter commands
- device commands
- CSV logging

GUI is for human-in-the-loop debugging. CLI remains the preferred entry for automation and scripted tests.

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

If `PySide6` is missing, `esp-drone-cli` still works. `esp-drone-gui` exits with a clear install hint.

## Start GUI

Installed entrypoint:

```powershell
esp-drone-gui
```

Module entrypoint:

```powershell
python -m esp_drone_cli.gui_main
```

## Serial Connection Example

1. Plug the aircraft into USB CDC.
2. Start the GUI.
3. In the `Connection` area:
   - select `serial`
   - choose the COM port or type it manually
   - keep baudrate at `115200` unless firmware changes it later
4. Click `Connect`

Expected result:

- status changes to `Connected: ...`
- parameter list can be refreshed
- telemetry starts updating after `Stream On`

## UDP Connection Example

1. Power the aircraft and make sure the UDP endpoint is reachable.
2. In the `Connection` area:
   - select `udp`
   - set host, for example `192.168.4.1`
   - set port, default `2391`
3. Click `Connect`

Expected result:

- status changes to `Connected: ...`
- commands are sent through the same `DeviceSession` path as serial

## GUI Areas

### Connection

- choose serial or UDP
- configure COM port / baudrate or host / port
- connect and disconnect
- check current connection state

### Safety Control

- `Arm`
- `Disarm`
- `Kill`
- `Reboot`
- current `arm_state`, `failsafe_reason`, `control_mode`

### Realtime Telemetry

- `Stream On` / `Stream Off`
- apply target telemetry rate through `telemetry_usb_hz` or `telemetry_udp_hz`
- view live values for gyro, attitude, setpoints, motors, battery, loop timing and safety state

### Parameter Debug

- refresh parameter list
- search by parameter name
- select one parameter and edit a new value
- `Save`
- `Reset`
- `Export JSON`
- `Import JSON`

### Debug Actions

- `motor_test`
- `calib gyro`
- `calib level`
- `rate_test`

### Log Export

- choose CSV output file
- `Start Log`
- `Stop Log`
- `Dump CSV`
- inspect last log path and latest error/event

## Common Errors

`PySide6 is required for esp-drone-gui`

- install GUI dependencies with `pip install -e .[gui]`

`connect: no serial port selected`

- choose or type a valid COM port before connecting

`Disconnected` after a short time

- check the USB cable, COM port ownership, firmware boot state and whether the device really exposes USB CDC

`set ...` or `save params` fails

- device-side parameter validation rejected the value
- check parameter ranges and inter-parameter constraints in firmware docs

No telemetry updates after connect

- click `Stream On`
- verify the correct transport is connected
- verify firmware is running and not held in reset/fault

## Scope Boundary

Current GUI scope is intentionally small:

- good for bench connection, safety commands, telemetry, params, motor/rate test and CSV capture
- not yet a plotting workstation
- not a second protocol stack
- not the primary automation surface
