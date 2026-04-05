# Python Tool GUI + CLI Refactor Plan

**Language / 语言：** **English** | [简体中文](./python_tool_gui_refactor_plan.zh-CN.md)

## Goal

Upgrade `tools/esp_drone_cli/` from a CLI-first tool into a shared `core + cli + gui` structure, with the GUI implemented on `PyQt5` instead of `PySide6`.

Constraints:

- keep the existing CLI command style as stable as possible
- keep `DeviceSession` as the only session and command owner
- keep protocol and transport ownership inside `core/`
- let the GUI remain an interaction shell rather than a second protocol stack
- keep automation and repeatable checks CLI-first

## Current Status

### Phase B

Phase B is implemented in code.

- `esp_drone_cli.core.device_session.DeviceSession` is the only session and command owner
- `esp_drone_cli.core.protocol.*` is the only protocol owner
- `esp_drone_cli.core.transport.*` is the only transport owner
- `esp_drone_cli.core.models` is the only telemetry and parameter-snapshot owner
- CLI calls `DeviceSession` directly
- GUI calls `DeviceSession` directly
- old top-level `client.py`, `protocol/*`, and `transport/*` modules are compatibility shims only

### Phase C

Phase C minimal GUI is implemented on the same `DeviceSession`.

- GUI connects through serial or UDP only by calling `DeviceSession`
- GUI does not own framing, transport, or command encoding
- GUI supports connect or disconnect, stream control, telemetry, parameter operations, `arm`, `disarm`, `kill`, `reboot`, `motor-test`, calibration, `rate-test`, CSV logging, and CSV dump

### Phase D

Phase D focuses on closure and manual-debug usability.

- README documents CLI versus GUI install and startup flows
- GUI usage and manual checklist docs exist under `docs/`
- smoke tests cover optional GUI dependency behavior, GUI startup or close, and GUI-to-session action routing
- GUI has moved to `PyQt5 + pyqtgraph`
- core ownership remains unchanged: GUI and CLI still share one `DeviceSession`

## Planned Package Layout

```text
tools/esp_drone_cli/esp_drone_cli/
|- core/
|  |- protocol/
|  |- transport/
|  |- device_session.py
|  |- models.py
|  `- csv_log.py
|- cli/
|  `- main.py
|- gui/
|  `- main_window.py
|- __main__.py
|- gui_main.py
|- client.py
|- protocol/
`- transport/
```

## File Movement And Compatibility

The real implementation lives in `core/`. Existing top-level modules remain only as compatibility shims.

| Current file | New owner | Compatibility rule |
|---|---|---|
| `esp_drone_cli/protocol/messages.py` | `esp_drone_cli/core/protocol/messages.py` | old path re-exports new symbols |
| `esp_drone_cli/protocol/framing.py` | `esp_drone_cli/core/protocol/framing.py` | old path re-exports new symbols |
| `esp_drone_cli/transport/serial_link.py` | `esp_drone_cli/core/transport/serial_link.py` | old path re-exports the new class |
| `esp_drone_cli/transport/udp_link.py` | `esp_drone_cli/core/transport/udp_link.py` | old path re-exports the new class |
| `esp_drone_cli/client.py` | `esp_drone_cli/core/device_session.py` + `esp_drone_cli/core/models.py` | old module stays as a compatibility facade |
| `esp_drone_cli/__main__.py` | `esp_drone_cli/cli/main.py` | `python -m esp_drone_cli ...` remains the CLI entry |

## Stable External Interfaces

The following remain stable or intentionally compatible:

- `python -m esp_drone_cli ...`
- `esp-drone-cli ...`
- `esp-drone-gui`
- serial and UDP transport choices
- command names such as `connect`, `arm`, `disarm`, `kill`, `motor-test`, `axis-test`, `rate-test`, and `dump-csv`

## Core Layer Responsibilities

`core/` owns:

- frame encoding and decoding
- serial transport
- UDP transport
- telemetry parsing
- parameter encode and decode
- session lifecycle
- command and response flow
- telemetry subscription
- CSV log export

The central object is `DeviceSession`.

## Single-Owner Table

| Concern | Single owner |
|---|---|
| frame encode and decode | `esp_drone_cli.core.protocol.framing` |
| protocol message IDs and frame type | `esp_drone_cli.core.protocol.messages` |
| serial transport | `esp_drone_cli.core.transport.serial_link` |
| UDP transport | `esp_drone_cli.core.transport.udp_link` |
| telemetry payload decode | `esp_drone_cli.core.models` |
| parameter value encode and decode | `esp_drone_cli.core.models` |
| JSON parameter snapshot import and export | `esp_drone_cli.core.device_session` |
| device command flow | `esp_drone_cli.core.device_session` |
| stream lifecycle | `esp_drone_cli.core.device_session` |
| CSV log export | `esp_drone_cli.core.csv_log` + `esp_drone_cli.core.device_session` |

## GUI Technology Stack

- GUI binding: `PyQt5`
- plotting: `pyqtgraph`
- session bridge: callback-to-Qt-signal translation in `gui/main_window.py`

If GUI dependencies are missing:

- the CLI still works
- `esp-drone-gui` fails fast with a clear install hint

## GUI Scope

Current GUI scope:

- connect or disconnect
- stream on or off
- `arm`, `disarm`, `kill`, `reboot`
- telemetry table
- realtime charts
- parameter refresh, set, save, reset, import, and export
- motor test
- calibration commands
- rate test
- CSV logging
- `QSettings`-backed local UI state

Still intentionally deferred:

- RC or stock-App compatibility workflows
- GUI-owned protocol logic
- large dashboard styling or heavy visualization
