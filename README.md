# ESP-DRONE

Language / 语言: English | [简体中文](./README.zh-CN.md)

Custom ESP-IDF flight-controller firmware and shared Python tooling for a brushed quadcopter based on `ESP32-S3-WROOM-1-N16R8`.

## Locked Constraints

- `ESP-IDF v5.5.1`
- body frame fixed to `+Y` nose, `+X` right side, `+Z` up
- project naming fixed to `+pitch` nose up, `+roll` right side down, `+yaw` nose right
- `UART0` reserved for `ATK-MS901M`
- `USB CDC` reserved for CLI, GUI, and debug telemetry
- CLI and GUI share one `DeviceSession`

Direction-sensitive logic must follow:

- [docs/axis_truth_table.md](docs/axis_truth_table.md)
- [docs/motor_map.md](docs/motor_map.md)

## Repository Layout

- [docs/](docs/): design constraints, bring-up notes, CLI and GUI usage
- [firmware/](firmware/): ESP-IDF firmware project
- [tools/esp_drone_cli/](tools/esp_drone_cli/): shared Python `core + cli + gui` toolchain

## Current Status

This repository currently contains diagnostic and verification paths, plus experimental manual/bench control. These paths are useful for bring-up, telemetry, safety checks, and controlled motor/attitude experiments. They are not the same as a finished free-flight stabilize mode.

Implemented diagnostic / verification / experimental paths:

- three-axis rate-loop bench path for `roll / pitch / yaw`
- bench-only hang-attitude outer-loop bring-up path for a circular-rod or hanging rig with a natural `+Z down` equilibrium
- ground tune / attitude ground verify path using a captured ground reference
- low-risk liftoff verify path
- all-motor test path
- ESP32 SoftAP + binary UDP transport path for CLI/GUI connectivity
- experimental UDP manual control through `CONTROL_MODE_UDP_MANUAL`
- params, telemetry, capability, and device-info style host tooling

Not implemented or not claimed ready:

- finished free-flight stabilize / angle mode ready for prop-on use
- autonomous takeoff controller
- closed-loop altitude hold
- position hold

The hang-attitude path is intentionally limited:

- firmware adds `CONTROL_MODE_ATTITUDE_HANG_TEST`
- reference attitude must be captured explicitly with `attitude-capture-ref`
- control uses relative quaternion error `q_rel = q_ref^-1 * q_now`, not global Euler subtraction against `roll=0 / pitch=0`
- only `roll / pitch` go through the outer loop in that stage
- outer loop is P-only and feeds the existing rate inner loop
- thrust stays open-loop through `attitude_test_base_duty`

The documented live roll session was run on a constrained circular-rod bench with natural `+Z down`. That orientation does not change the roll rate-loop sign convention because the workflow evaluates only `rate_setpoint_roll`, mapped roll feedback, PID output, and motor split.

## Current Parameter Facts

Documentation must distinguish current firmware defaults from tuning candidates.

Current GitHub firmware defaults in `firmware/main/params/params.c` are:

- `rate_kp_roll = 0.0030`
- `rate_kp_pitch = 0.0030`
- `rate_kp_yaw = 0.0030`
- `rate_ki_* = 0`
- `rate_kd_* = 0`

The conservative values `roll=0.0007`, `pitch=0.0007`, `yaw=0.0005` are tuning candidates only unless `params.c` is changed.

## Motor Output Status

Current `firmware/main/motor/motor.c` uses fixed `LEDC_TIMER_8_BIT` PWM resolution. The PWM frequency is parameterized through `motor_pwm_freq_hz`, but PWM resolution is not parameterized.

Implemented motor output controls:

- `motor_output_map[4]`
- global `motor_idle_duty`
- global `motor_max_duty`
- global `motor_startup_boost_duty`
- global `motor_slew_limit_per_tick`
- parameterized PWM frequency

Not implemented as of this docs-code sync:

- per-motor thrust `scale`
- per-motor `offset`
- per-motor `min_start`
- per-motor `deadband`
- per-motor `gamma`
- PWM resolution parameterization

`motor_output_map` is a channel mapping feature. It must not be described as per-motor thrust compensation.

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

Serial / USB CDC debugging is unchanged and should be used first if Wi-Fi or UDP startup fails. See [docs/softap_udp_transport.md](docs/softap_udp_transport.md).

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

- [docs/README.md](docs/README.md)
- [docs/CODEX_STATE.md](docs/CODEX_STATE.md)
- [docs/TUNING_DECISIONS.md](docs/TUNING_DECISIONS.md)
- [docs/axis_truth_table.md](docs/axis_truth_table.md)
- [docs/motor_map.md](docs/motor_map.md)
- [docs/hang_attitude_bringup_plan.md](docs/hang_attitude_bringup_plan.md)
- [docs/ground_tune_manual_workflow.md](docs/ground_tune_manual_workflow.md)
- [docs/attitude_ground_verify_liftoff_plan.md](docs/attitude_ground_verify_liftoff_plan.md)
- [docs/python_cli_usage.md](docs/python_cli_usage.md)
- [docs/python_gui_usage.md](docs/python_gui_usage.md)
- [docs/softap_udp_transport.md](docs/softap_udp_transport.md)
- [docs/udp_manual_control_protocol.md](docs/udp_manual_control_protocol.md)
