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
- an ESP32 SoftAP + binary UDP transport path for CLI/GUI connectivity and experimental UDP manual control

The hang-attitude path is intentionally limited:

- firmware adds `CONTROL_MODE_ATTITUDE_HANG_TEST`
- reference attitude must be captured explicitly with `attitude-capture-ref`
- control uses relative quaternion error `q_rel = q_ref^-1 * q_now`, not global Euler subtraction against `roll=0 / pitch=0`
- only `roll / pitch` go through the outer loop in that stage
- outer loop is P-only and feeds the existing rate inner loop
- thrust stays open-loop through `attitude_test_base_duty`

Current roll tuning remains bench-only and rate-loop only:

- no angle outer loop
- no attitude outer loop in the roll workflow
- no free-flight tuning
- no change to the fixed `+roll`, `roll_rate = -gyro_y`, or motor-map conventions

The documented live roll session was run on a constrained circular-rod bench with natural `+Z down`.
That orientation does not change the roll rate-loop sign convention because the workflow evaluates only `rate_setpoint_roll`, mapped roll feedback, PID output, and motor split.

This repository still does not declare any free-flight stabilize or angle mode ready for prop-on use.

## SoftAP UDP Transport

Firmware starts an ESP32 SoftAP by default:

- SSID: `ESP-DRONE`
- Password: `12345678`
- AP IP: `192.168.4.1`
- UDP protocol port: `2391`

GUI connection order:

1. Connect the PC Wi-Fi to the `ESP-DRONE` SoftAP.
2. Start the Python GUI and set `Link` to `UDP`.
3. Use `UDP Host = 192.168.4.1` and `UDP Port = 2391`.
4. Click `Connect`.

Serial / USB CDC debugging is unchanged and should be used first if Wi-Fi or UDP startup fails.
See [docs/softap_udp_transport.md](./docs/softap_udp_transport.md).

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
- [docs/roll_rate_bench_workflow.md](./docs/roll_rate_bench_workflow.md)
- [docs/roll_bench_summary_sample.md](./docs/roll_bench_summary_sample.md)
- [docs/python_cli_usage.md](./docs/python_cli_usage.md)
- [docs/python_gui_usage.md](./docs/python_gui_usage.md)
- [docs/softap_udp_transport.md](./docs/softap_udp_transport.md)
- [docs/udp_manual_control_protocol.md](./docs/udp_manual_control_protocol.md)
- [docs/bringup_checklist.md](./docs/bringup_checklist.md)
- [docs/python_gui_manual_checklist.md](./docs/python_gui_manual_checklist.md)
- [docs/rate_bringup_results.md](./docs/rate_bringup_results.md)
- [docs/README.md](./docs/README.md)
