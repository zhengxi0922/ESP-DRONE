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
- [ ] if `PyQt5` or `pyqtgraph` is missing, GUI prints a clear install error instead of crashing obscurely

## Connection

- [ ] serial mode can list or accept a typed COM port
- [ ] serial connect succeeds
- [ ] UDP mode can connect to `host:port`
- [ ] disconnect returns the GUI to `Disconnected`
- [ ] last connection error field is readable and meaningful on failure

## Stream And Telemetry

- [ ] `Stream On` starts telemetry updates
- [ ] `Stream Off` stops telemetry updates or clearly stops new samples
- [ ] telemetry table refreshes with gyro / attitude / motor / battery / loop timing fields
- [ ] telemetry table selection can be copied with `Ctrl+C`
- [ ] changing target stream rate applies without breaking the session

## Charts

- [ ] gyro chart updates while streaming
- [ ] attitude chart updates while streaming
- [ ] motor chart updates while streaming
- [ ] battery chart updates while streaming
- [ ] pause / resume works
- [ ] clear chart history works
- [ ] switching chart window length works
- [ ] channel visibility checkboxes work

## Parameters

- [ ] `Refresh` loads the parameter list
- [ ] search box filters the list
- [ ] selecting one parameter fills the detail pane
- [ ] local hint text updates when editing a value
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
- [ ] event log can be cleared, copied, and saved to text

## State Persistence

- [ ] last serial port is restored after restart
- [ ] last UDP target is restored after restart
- [ ] window geometry is restored after restart
- [ ] chart window selection is restored after restart
- [ ] last parameter search text is restored after restart

## Shutdown

- [ ] closing the main window releases the transport cleanly
- [ ] no stale serial lock remains after close
- [ ] GUI can be started again immediately after closing
