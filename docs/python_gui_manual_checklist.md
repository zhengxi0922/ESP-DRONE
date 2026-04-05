# Python GUI Manual Checklist

**Language / 语言：** **English** | [简体中文](./python_gui_manual_checklist.zh-CN.md)

Use this checklist after installing `esp-drone-gui` with:

```powershell
cd tools\esp_drone_cli
pip install -e .[gui]
```

Bench assumptions:

- props removed, or the frame restrained
- firmware already flashed
- USB CDC or UDP endpoint available

## Startup

- [ ] `esp-drone-gui` starts successfully
- [ ] closing the window does not hang
- [ ] if `PyQt5` or `pyqtgraph` is missing, the GUI prints a clear install error instead of crashing obscurely
- [ ] the main window uses the new three-column layout instead of stacked tabs
- [ ] the center chart area is visibly larger than the control rail and parameter editor
- [ ] the bottom log starts in compact form and can be expanded
- [ ] the language switch defaults to Chinese and can switch to English

## Connection

- [ ] serial mode can list or accept a typed COM port
- [ ] serial mode only shows serial controls
- [ ] UDP mode only shows host and port controls
- [ ] serial connect succeeds
- [ ] UDP mode can connect to `host:port`
- [ ] disconnect returns the GUI to `Disconnected`
- [ ] the last-connection-error field is readable and meaningful on failure

## Stream And Telemetry

- [ ] `Stream On` starts telemetry updates
- [ ] `Stream Off` stops telemetry updates or clearly stops new samples
- [ ] the telemetry table refreshes with gyro, attitude, motor, battery, and loop-timing fields
- [ ] telemetry-table selection can be copied with `Ctrl+C`
- [ ] changing the target stream rate applies without breaking the session
- [ ] the chart area and telemetry table are visible at the same time

## Charts

- [ ] the gyro chart updates while streaming
- [ ] the attitude chart updates while streaming
- [ ] the motor chart updates while streaming
- [ ] the battery chart updates while streaming
- [ ] the main chart is large enough to inspect waveforms without switching tabs
- [ ] pause or resume works
- [ ] clearing chart history works
- [ ] switching chart group works
- [ ] switching chart-window length works
- [ ] channel-visibility checkboxes work
- [ ] auto scale works
- [ ] reset view works

## Parameters

- [ ] `Refresh` loads the parameter list
- [ ] the search box filters the list
- [ ] the parameter table shows enough rows for practical bench tuning
- [ ] selecting one parameter fills the compact detail pane
- [ ] local hint text updates when editing a value
- [ ] setting one parameter returns without a GUI error
- [ ] `Save` succeeds
- [ ] `Reset` succeeds
- [ ] `Export JSON` writes a snapshot file
- [ ] `Import JSON` applies a snapshot file
- [ ] parameter-table selection can be copied

## Safety Commands

- [ ] `Arm` reaches the session layer and returns a result
- [ ] `Disarm` reaches the session layer and returns a result
- [ ] `Kill` reaches the session layer and returns a result
- [ ] `Kill` remains the highest-visibility dangerous button
- [ ] `Reboot` reaches the session layer and returns a result

## Debug Actions

- [ ] `motor_test` starts the selected motor command
- [ ] stopping `motor_test` sends zero duty
- [ ] `calib gyro` succeeds
- [ ] `calib level` succeeds
- [ ] `rate_test` sends the selected axis and value
- [ ] stopping `rate_test` sends zero rate

## Logging

- [ ] `Start Log` creates or opens the selected CSV path
- [ ] `Stop Log` stops active CSV logging
- [ ] `Dump CSV` writes a CSV file for the requested duration
- [ ] `Last log` updates to the most recent output path
- [ ] `Last error` stays clear during a nominal session
- [ ] the event log can be cleared, copied, and saved to text
- [ ] the event log can be expanded and collapsed without breaking the layout

## State Persistence

- [ ] the last serial port is restored after restart
- [ ] the last UDP target is restored after restart
- [ ] window geometry is restored after restart
- [ ] chart-group selection is restored after restart
- [ ] chart-window selection is restored after restart
- [ ] the last parameter-search text is restored after restart
- [ ] the last language selection is restored after restart

## Shutdown

- [ ] closing the main window releases the transport cleanly
- [ ] no stale serial lock remains after close
- [ ] the GUI can be started again immediately after closing
