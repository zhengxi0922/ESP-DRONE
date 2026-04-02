# Python GUI Manual Checklist

Use this checklist after installing `esp-drone-gui` with:

```powershell
cd tools\esp_drone_cli
pip install -e .[gui]
```

Bench assumptions:

- props removed, or frame restrained
- firmware already flashed
- USB CDC or UDP endpoint available

## Startup

- [ ] `esp-drone-gui` starts successfully
- [ ] closing the window does not hang
- [ ] if `PySide6` is missing, GUI prints a clear install error instead of crashing obscurely

## Connection

- [ ] serial mode can list or accept a typed COM port
- [ ] serial connect succeeds
- [ ] UDP mode can connect to `host:port`
- [ ] disconnect returns the GUI to `Disconnected`

## Stream And Telemetry

- [ ] `Stream On` starts telemetry updates
- [ ] `Stream Off` stops telemetry updates or clearly stops new samples
- [ ] telemetry table refreshes with gyro / attitude / motor / battery / loop timing fields
- [ ] changing target stream rate applies without breaking the session

## Parameters

- [ ] `Refresh Params` loads the parameter list
- [ ] search box filters the list
- [ ] selecting one parameter fills the edit box
- [ ] setting one parameter returns without GUI error
- [ ] `Save` succeeds
- [ ] `Reset` succeeds
- [ ] `Export JSON` writes a snapshot file
- [ ] `Import JSON` applies a snapshot file

## Safety Commands

- [ ] `Arm` reaches the session layer and returns a result
- [ ] `Disarm` reaches the session layer and returns a result
- [ ] `Kill` reaches the session layer and returns a result
- [ ] `Reboot` reaches the session layer and returns a result

## Debug Actions

- [ ] `motor_test` starts the selected motor command
- [ ] stopping `motor_test` sends zero duty
- [ ] `calib gyro` command succeeds
- [ ] `calib level` command succeeds
- [ ] `rate_test` sends the selected axis/value
- [ ] stopping `rate_test` sends zero rate

## Logging

- [ ] `Start Log` creates or opens the selected CSV path
- [ ] `Stop Log` stops active CSV logging
- [ ] `Dump CSV` writes a CSV file for the requested duration
- [ ] `Last log` updates to the most recent output path
- [ ] `Last error` remains clear during a nominal session

## Shutdown

- [ ] closing the main window releases the transport cleanly
- [ ] no stale serial lock remains after close
- [ ] GUI can be started again immediately after closing
