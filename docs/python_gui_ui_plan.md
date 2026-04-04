# Python GUI UI Plan

## Goal

Rework the GUI into a bench-debug workbench that is actually usable on a maximized Windows screen:

- larger main chart area
- less duplicated state
- better parameter editing density
- compact low-frequency control rail
- Chinese-first UI with English fallback

The ownership model does not change:

- session / command flow: `esp_drone_cli.core.device_session.DeviceSession`
- protocol: `esp_drone_cli.core.protocol.*`
- transport: `esp_drone_cli.core.transport.*`
- telemetry / parameter models: `esp_drone_cli.core.models`

GUI must keep calling `DeviceSession`. It must not assemble frames, talk to transport objects directly, or add a second client/session abstraction.

## Scope Boundary

This GUI round changes:

- GUI layout and information architecture
- GUI-side plotting and local state persistence
- GUI localization
- README / GUI docs / GUI smoke tests

This GUI round does not change:

- firmware
- core protocol
- transport implementations
- `DeviceSession`
- CLI command names

## Layout Direction

Use a three-column workbench plus a compact bottom log:

```text
+-------------------------------------------------------------------------------------------+
| Left rail              | Center main workspace                    | Right info + params   |
|                        |                                           |                       |
| Connection             | Large main chart                          | Status cards          |
| Safety                 |                                           |                       |
| Debug actions          |                                           | Parameter table       |
|                        |                                           | + compact editor      |
|                        | Live telemetry table                      |                       |
+-------------------------------------------------------------------------------------------+
| Bottom log: recent result + events + last log path + last error (collapsible)            |
+-------------------------------------------------------------------------------------------+
```

## Area Responsibilities

### Left Column

Keep only low-frequency actions:

- transport choice
- serial / UDP configuration
- connect / disconnect
- arm / disarm / kill / reboot
- calib gyro / calib level
- motor test
- rate test
- CSV log controls

The left rail should stay visually narrow so it does not steal width from the main plot.

### Center Column

This is the primary bench area and must get the most width.

Use one large main plot with a channel-group switch instead of several small equally sized charts.

Supported groups:

- gyro
- attitude
- motors
- battery

Controls:

- pause / resume
- clear
- auto scale
- reset view
- 5s / 10s / 30s window
- per-channel visibility checkboxes

Realtime numeric values remain visible below the plot. They are no longer hidden behind a separate tab.

### Right Column

Upper half:

- key status cards
  - arm_state
  - failsafe_reason
  - control_mode
  - imu_mode
  - stream
  - battery_voltage
  - imu_age_us
  - loop_dt_us

Lower half:

- parameter search
- parameter table
- current value / new value
- set / save / reset / import / export
- compact description area

The parameter table should dominate the vertical space; the detail panel stays compact.

### Bottom Log

Keep a smaller default height and allow collapse / expand.

Show:

- recent command results
- event log
- last log path
- last error

## Information Reduction Rules

- do not duplicate the same state in a top status strip and the side panel
- keep connection state in the connection group
- keep runtime state in the right-side status cards
- do not spread stream state across multiple unrelated widgets

## Localization Plan

Default UI language:

- Chinese

Supported switch:

- `中文`
- `English`

Translate through the GUI translation table:

- group titles
- button text
- labels
- placeholders
- bench warnings
- status text
- common errors
- recent-result text

## Local State

Use `QSettings` for:

- window geometry
- main / center / right / params / bottom splitter state
- last link type
- last serial port
- last UDP host / port
- last chart group
- last chart window
- last parameter filter
- last CSV path
- last language
- log collapsed / expanded state

## Acceptance For This Round

- chart area is large enough for restrained bench waveform inspection
- realtime table and plot are visible together
- parameter editing is denser than before
- serial mode only shows serial controls
- UDP mode only shows UDP controls
- Chinese is the default GUI language
- CLI remains compatible
- GUI and CLI still share `DeviceSession`
