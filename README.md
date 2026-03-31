# ESP-DRONE

Custom ESP-IDF flight-controller firmware and Python CLI for a brushed quadcopter based on `ESP32-S3-WROOM-1-N16R8`.

## Locked Environment

- MCU: `ESP32-S3-WROOM-1-N16R8`
- ESP-IDF: `v5.5.1`
- RTOS: ESP-IDF FreeRTOS
- IMU transport: `UART0` to `ATK-MS901M`
- Console / CLI transport: `USB CDC`

## Body And Attitude Convention

The whole project uses this body frame:

- `+Y`: nose / front
- `+X`: right side
- `+Z`: upward

The whole project uses these positive attitude names:

- `+pitch`: nose up
- `+roll`: right side down
- `+yaw`: nose right, clockwise viewed from above

Important: the body frame is right-handed, but the positive names for `roll` and `yaw` do not follow the default mathematical positive rotation around `+Y` and `+Z`. Every direction-sensitive module must reference `docs/axis_truth_table.md` and `docs/motor_map.md`.

## Repository Layout

- `docs/`: extracted constraints, protocol notes, bring-up and validation docs
- `firmware/`: ESP-IDF firmware project
- `tools/esp_drone_cli/`: Python CLI and protocol helpers

## Build Notes

This repository is written for ESP-IDF `v5.5.1`.

On this Windows workstation, the verified local installation is documented in `docs/esp_idf_env.md` and can be activated directly from PowerShell:

```powershell
. .\tools\esp-idf-env.ps1
```

The repository also includes a wrapper for running `idf.py` against `firmware/` without manually changing directories:

```powershell
.\tools\idf.ps1 --version
.\tools\idf.ps1 build
```

For a clean checkout, set the target once before the first build:

```powershell
.\tools\idf.ps1 set-target esp32s3
```

If you prefer the raw ESP-IDF shell flow, the firmware can still be built this way:

```powershell
cd D:\0Work\Codex\ESP-drone\firmware
idf.py set-target esp32s3
idf.py build
```

## Current Stage

The repository currently contains:

- Stage 1 design documents
- Stage 2 firmware skeleton and bottom-layer components
- Baseline safety shell for `arm / disarm / kill`, state ownership, and a small set of hard stop triggers
- Python CLI protocol, transport and basic command surface

Still pending in later stages:

- Full safety / failsafe logic with RC gesture arming, link supervision, richer fault classification, telemetry reason reporting, and final policy refinement
- Estimator, controller, mixer, telemetry compatibility and the full CLI surface

## Stage-2 Console Rule

Stage 2 locks the debug transport split as follows:

- `UART0` is reserved for the `ATK-MS901M` IMU only
- Application console, debug log and CLI transport must use `USB CDC` only
- `sdkconfig.defaults` must keep UART console disabled so no application console traffic leaks onto the IMU UART

## Acknowledgements

This project is a from-scratch rewrite for a custom quadcopter, but it intentionally acknowledges the open-source work that helped define the compatibility and bring-up targets:

- `ESP-Drone` and the related legacy app / UDP ecosystem, used here as a behavior and compatibility reference
- `Bitcraze Crazyflie`, whose open flight-control ecosystem influenced many small-drone software stacks
- `ALIENTEK / ATK`, for the `ATK-MS901M` module documentation and example materials used to reconstruct the IMU protocol

Important:

- This repository does **not** inherit the old repository's body frame, attitude sign convention, mixer, or control-chain implementation by default.
- Legacy projects are referenced for protocol, visual semantics, and hardware bring-up only; direction-sensitive logic in this repository is redefined explicitly in `docs/axis_truth_table.md` and `docs/motor_map.md`.
