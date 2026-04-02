# Python Tool GUI + CLI Refactor Plan

## Goal

Upgrade `tools/esp_drone_cli/` from a CLI-only tool into a shared `core + cli + gui` structure.

Constraints:

- Keep the existing CLI command style as stable as possible
- Add a Windows desktop GUI with `PySide6`
- GUI and CLI must reuse one protocol / transport / device-command implementation
- GUI is only an interaction shell; automation and tests remain CLI-first

## Phase B Status

Phase B is now implemented in code.

- `esp_drone_cli.core.device_session.DeviceSession` is the only session / command owner
- `esp_drone_cli.core.protocol.*` is the only protocol owner
- `esp_drone_cli.core.transport.*` is the only transport owner
- `esp_drone_cli.core.models` is the only telemetry / parameter snapshot owner
- CLI calls `DeviceSession` directly
- GUI calls `DeviceSession` directly
- old top-level `client.py`, `protocol/*`, and `transport/*` modules are compatibility shims only

This means there is no second client implementation that owns command flow, framing, or transport behavior.

## Phase C Status

Phase C minimal GUI is now implemented on top of the same `DeviceSession`.

- GUI connects through serial or UDP only by calling `DeviceSession`
- GUI does not own framing, transport, or command encoding
- GUI currently supports connect / disconnect, stream control, telemetry table, parameter operations, `arm / disarm / kill / reboot`, `motor-test`, `calib`, `rate-test`, and CSV logging
- GUI remains intentionally minimal; plotting and heavier desktop UX work are still deferred

## Phase D Status

Phase D focuses on closure rather than feature growth.

- README now documents CLI vs GUI install and startup flows
- dedicated GUI usage and manual checklist docs exist under `docs/`
- smoke tests cover optional `PySide6`, basic GUI startup/close, and GUI-to-session action routing
- core ownership remains unchanged: GUI and CLI still share one `DeviceSession`

## Planned Package Layout

```text
tools/esp_drone_cli/esp_drone_cli/
├─ core/
│  ├─ protocol/
│  ├─ transport/
│  ├─ device_session.py
│  ├─ models.py
│  └─ csv_log.py
├─ cli/
│  └─ main.py
├─ gui/
│  └─ main_window.py
├─ __main__.py
├─ gui_main.py
├─ client.py
├─ protocol/
└─ transport/
```

## File Movement And Compatibility

The real implementation moves into `core/`. Existing top-level modules become compatibility shims.

| Current file | New owner | Compatibility rule |
|---|---|---|
| `esp_drone_cli/protocol/messages.py` | `esp_drone_cli/core/protocol/messages.py` | old path re-exports new symbols |
| `esp_drone_cli/protocol/framing.py` | `esp_drone_cli/core/protocol/framing.py` | old path re-exports new symbols |
| `esp_drone_cli/transport/serial_link.py` | `esp_drone_cli/core/transport/serial_link.py` | old path re-exports new class |
| `esp_drone_cli/transport/udp_link.py` | `esp_drone_cli/core/transport/udp_link.py` | old path re-exports new class |
| `esp_drone_cli/client.py` | `esp_drone_cli/core/device_session.py` + `esp_drone_cli/core/models.py` | old module stays as compatibility facade |
| `esp_drone_cli/__main__.py` | `esp_drone_cli/cli/main.py` | `python -m esp_drone_cli ...` remains the CLI entry |

## Stable External Interfaces

The following remain stable or intentionally compatible:

- `python -m esp_drone_cli ...`
- `esp-drone-cli ...`
- serial and UDP transport choices
- existing command names such as `connect`, `arm`, `disarm`, `kill`, `motor-test`, `axis-test`, `rate-test`, `dump-csv`

The following are added without breaking existing commands:

- `esp-drone-gui`
- JSON parameter export / import through shared core helpers

## Core Layer Responsibilities

`core/` owns:

- frame encoding / decoding
- serial transport
- UDP transport
- telemetry parsing
- parameter encode / decode
- session lifecycle
- command / response flow
- telemetry subscription
- CSV log export

The central object is `DeviceSession`.

## Single-Owner Table

| Concern | Single owner |
|---|---|
| frame encode / decode | `esp_drone_cli.core.protocol.framing` |
| protocol message ids / frame type | `esp_drone_cli.core.protocol.messages` |
| serial transport | `esp_drone_cli.core.transport.serial_link` |
| UDP transport | `esp_drone_cli.core.transport.udp_link` |
| telemetry payload decode | `esp_drone_cli.core.models` |
| parameter value encode / decode | `esp_drone_cli.core.models` |
| JSON parameter snapshot import / export | `esp_drone_cli.core.device_session` |
| device command flow | `esp_drone_cli.core.device_session` |
| stream lifecycle | `esp_drone_cli.core.device_session` |
| CSV log export | `esp_drone_cli.core.csv_log` + `esp_drone_cli.core.device_session` |

## DeviceSession Contract

`DeviceSession` will expose:

- `connect_serial()`
- `connect_udp()`
- `connect_transport()`
- `disconnect()`
- `hello()`
- `arm()`
- `disarm()`
- `kill()`
- `reboot()`
- `get_param()`
- `set_param()`
- `list_params()`
- `save_params()`
- `reset_params()`
- `export_params()`
- `import_params()`
- `start_stream()`
- `stop_stream()`
- `motor_test()`
- `axis_test()`
- `rate_test()`
- `calib_gyro()`
- `calib_level()`
- `send_rc()`
- `subscribe_telemetry()`
- `subscribe_event_log()`
- `subscribe_connection_state()`

## Concurrency Model

- All transport receive work lives in one background reader thread inside `DeviceSession`
- Sync CLI commands wait on response frames through a thread-safe queue
- GUI only talks to `DeviceSession`; it never reads the transport directly
- GUI receives telemetry and event updates through callback -> Qt signal bridging

## GUI Class Split

First revision uses a small class set:

- `MainWindow`
  - owns the visible panels and button wiring
- `QtSessionBridge`
  - translates core callbacks into Qt signals
- `TelemetryPanel`
  - shows current values in a compact form/table
- `ParamsPanel`
  - handles search, refresh, set, save, reset, import, export

The first implementation may keep these panels inside one file if that keeps the code smaller and clearer.

## GUI Layout Sketch

```text
+--------------------------------------------------------------+
| Connection: [serial|udp] [port/ip] [baud/port] [connect]    |
+----------------------+---------------------------------------+
| Safety               | Realtime Telemetry                    |
| arm disarm kill      | gyro / rpy / setpoint / motors / bat |
| reboot               | stream on/off, rate param selector    |
+----------------------+---------------------------------------+
| Params: search, list, set selected, save/reset, import/export|
+----------------------+---------------------------------------+
| Debug Actions        | Log Export                            |
| motor_test           | choose file                           |
| calib gyro/level     | start log / stop log / dump csv       |
| rate test            | last path / last error                |
+--------------------------------------------------------------+
```

## GUI Scope For First Revision

Must work:

- connect / disconnect
- stream on / off
- arm / disarm / kill / reboot
- telemetry display
- get / set / save / reset / export / import params
- motor test
- calibration command buttons
- rate test entry
- CSV log export

Explicitly deferred:

- advanced plotting
- RC / UDP compatibility work
- high-polish styling

## Risk Notes

- The firmware does not yet implement every future protocol feature, so `send_rc()` will initially stay as a reserved core API surface.
- PySide6 must stay optional; GUI import failures must not affect CLI startup.
- Top-level compatibility modules must remain thin re-exports only. Any future business logic added there would reintroduce a forbidden dual-owner design.
