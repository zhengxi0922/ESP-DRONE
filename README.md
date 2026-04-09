# ESP-DRONE

**Language / 语言:** **English** | [简体中文](./README.zh-CN.md)

Custom ESP-IDF flight-controller firmware and shared Python tooling for a brushed quadcopter based on `ESP32-S3-WROOM-1-N16R8`.

## Locked Constraints

- `ESP-IDF v5.5.1`
- body frame fixed to `+Y` nose, `+X` right side, `+Z` up
- project naming fixed to `+pitch` nose up, `+roll` right side down, `+yaw` nose right
- `UART0` reserved for `ATK-MS901M`
- `USB CDC` reserved for CLI, GUI, and debug telemetry
- CLI and GUI share one `DeviceSession`

Direction-sensitive logic must follow:

- [docs/axis_truth_table.md](./docs/axis_truth_table.md)
- [docs/motor_map.md](./docs/motor_map.md)

## Repository Layout

- [docs/](./docs/README.md): design constraints, bring-up notes, CLI and GUI usage
- [firmware/](./firmware): ESP-IDF firmware project
- [tools/esp_drone_cli/](./tools/esp_drone_cli): shared Python `core + cli + gui` toolchain

## Current Status

The repository now contains two constrained-rig control paths:

- a usable three-axis rate-loop bench path for `roll / pitch / yaw`
- a bench-only hang-attitude outer-loop bring-up path for a circular-rod or hanging rig with a natural `+Z down` equilibrium

The new hang-attitude path is intentionally limited:

- firmware adds `CONTROL_MODE_ATTITUDE_HANG_TEST`
- reference attitude must be captured explicitly with `attitude-capture-ref`
- control uses relative quaternion error `q_rel = q_ref^-1 * q_now`, not global Euler subtraction against `roll=0 / pitch=0`
- only `roll / pitch` go through the outer loop in this stage
- outer loop is P-only and feeds the existing rate inner loop
- thrust stays open-loop through `attitude_test_base_duty`

This is not a free-flight stabilize mode and not an angle-flight-ready mode.

Never use `CONTROL_MODE_ATTITUDE_HANG_TEST` on a prop-on free-flight vehicle.

Still out of scope for this stage:

- free-flight stabilize or angle tuning
- yaw heading hold
- altitude-hold closed loop
- auto takeoff
- autotune

## Build Firmware

Recommended Windows flow:

```powershell
. .\tools\esp-idf-env.ps1
.\tools\idf.ps1 set-target esp32s3
.\tools\idf.ps1 build
```

Or raw ESP-IDF:

```powershell
cd firmware
idf.py set-target esp32s3
idf.py build
```

## Install Python Tools

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

## Main Docs

- [docs/hang_attitude_bringup_plan.md](./docs/hang_attitude_bringup_plan.md)
- [docs/python_cli_usage.md](./docs/python_cli_usage.md)
- [docs/python_gui_usage.md](./docs/python_gui_usage.md)
- [docs/bringup_checklist.md](./docs/bringup_checklist.md)
- [docs/python_gui_manual_checklist.md](./docs/python_gui_manual_checklist.md)
- [docs/rate_bringup_results.md](./docs/rate_bringup_results.md)
- [docs/README.md](./docs/README.md)
