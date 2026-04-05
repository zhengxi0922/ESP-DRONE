# Python GUI Manual Checklist

## 中文摘要

- 这份清单用于人工验收 `esp-drone-gui` 是否达到台架调试最低可用标准。
- 检查范围包括：GUI 启动、serial/udp 连接、stream、遥测刷新、参数读写、`arm/disarm/kill`、`motor_test`、`dump csv` 等。
- 重点是确认 GUI 只是 `DeviceSession` 的交互外壳，而不是第二套协议栈。

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
- [ ] main window uses the new three-column layout instead of stacked tabs
- [ ] the center chart area is visibly larger than the control rail and parameter editor
- [ ] the bottom log starts in compact form and can be expanded
- [ ] the language switch defaults to Chinese and can switch to English

## Connection

- [ ] serial mode can list or accept a typed COM port
- [ ] serial mode only shows serial controls
- [ ] UDP mode only shows host / port controls
- [ ] serial connect succeeds
- [ ] UDP mode can connect to `host:port`
- [ ] disconnect returns the GUI to `未连接 / Disconnected`
- [ ] last connection error field is readable and meaningful on failure

## Stream And Telemetry

- [ ] `开始流 / Stream On` starts telemetry updates
- [ ] `停止流 / Stream Off` stops telemetry updates or clearly stops new samples
- [ ] telemetry table refreshes with gyro / attitude / motor / battery / loop timing fields
- [ ] telemetry table selection can be copied with `Ctrl+C`
- [ ] changing target stream rate applies without breaking the session
- [ ] chart area and telemetry table are visible at the same time

## Charts

- [ ] gyro chart updates while streaming
- [ ] attitude chart updates while streaming
- [ ] motor chart updates while streaming
- [ ] battery chart updates while streaming
- [ ] the main chart is large enough to inspect waveforms without switching tabs
- [ ] pause / resume works
- [ ] clear chart history works
- [ ] switching chart group works
- [ ] switching chart window length works
- [ ] channel visibility checkboxes work
- [ ] auto scale works
- [ ] reset view works

## Parameters

- [ ] `Refresh` loads the parameter list
- [ ] search box filters the list
- [ ] parameter table shows enough rows for practical bench tuning
- [ ] selecting one parameter fills the compact detail pane
- [ ] local hint text updates when editing a value
- [ ] setting one parameter returns without GUI error
- [ ] `Save` succeeds
- [ ] `Reset` succeeds
- [ ] `Export JSON` writes a snapshot file
- [ ] `Import JSON` applies a snapshot file
- [ ] parameter table selection can be copied

## Safety Commands

- [ ] `Arm` reaches the session layer and returns a result
- [ ] `Disarm` reaches the session layer and returns a result
- [ ] `Kill` reaches the session layer and returns a result
- [ ] `Kill` remains the highest-visibility dangerous button
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
- [ ] event log can be expanded and collapsed without breaking the layout

## State Persistence

- [ ] last serial port is restored after restart
- [ ] last UDP target is restored after restart
- [ ] window geometry is restored after restart
- [ ] chart group selection is restored after restart
- [ ] chart window selection is restored after restart
- [ ] last parameter search text is restored after restart
- [ ] last language selection is restored after restart

## Shutdown

- [ ] closing the main window releases the transport cleanly
- [ ] no stale serial lock remains after close
- [ ] GUI can be started again immediately after closing
