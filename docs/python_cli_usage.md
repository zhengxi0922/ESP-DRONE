# Python CLI Usage

Language / 语言: English | [简体中文](./python_cli_usage.zh-CN.md)

## Goal

`esp-drone-cli` is the scripted and bench-debug entrypoint for the Python toolchain.

It uses the same `esp_drone_cli.core.device_session.DeviceSession` as the GUI, so CLI and GUI share:

- framing
- serial and UDP transport
- telemetry decoding
- parameter commands
- device commands
- CSV logging

The project does not maintain a second host protocol stack.

## Scope Boundary

The CLI covers more than the original two constrained bench workflows. Current supported areas include:

- rate-loop bench work for `roll / pitch / yaw`
- bench-only hang-attitude bring-up for a constrained rig
- ground tune / attitude ground verify diagnostics
- low-risk liftoff verify diagnostics
- experimental UDP manual control
- all-motor test
- params / telemetry / capability / device-info workflows

The CLI still does not declare any free-flight stabilize or angle mode ready for prop-on use. Ground and liftoff verification commands are diagnostics and safety checks, not proof that a finished free-flight mode exists.

## Install

CLI only:

```powershell
cd tools\esp_drone_cli
pip install -e .
```

CLI + GUI dependencies:

```powershell
cd tools\esp_drone_cli
pip install -e .[gui]
```

## Transport Rules

Bench bring-up is expected to use `USB CDC` first.

- `UART0` remains reserved for `ATK-MS901M`
- CLI examples below therefore use `--serial COMx`
- UDP examples default to the SoftAP binary protocol on `192.168.4.1:2391`
- STA mode uses the same UDP protocol and port; pass the drone IP with `--host` or `--drone-ip`

Recommended networking:

- Short term: keep the PC Wi-Fi on the drone SoftAP and use wired Ethernet or phone USB tethering for internet.
- Long term: configure the drone with `wifi_mode=sta` so it joins a phone hotspot/router, then run the PC and drone on the same LAN.
- Keep SoftAP as the fallback configuration path.

Generic connection examples:

```powershell
python -m esp_drone_cli --serial COM7 connect
python -m esp_drone_cli --udp 192.168.4.1:2391 connect
python -m esp_drone_cli --udp connect
python -m esp_drone_cli --host 192.168.50.42 connect
```

`--udp` without an endpoint uses the default SoftAP target. `--host` and `--drone-ip` are aliases for STA/custom UDP targets; `--udp-port` defaults to `2391`.

## Common Commands

Connection, capability, and stream:

```powershell
python -m esp_drone_cli --serial COM7 connect
python -m esp_drone_cli --serial COM7 capabilities
python -m esp_drone_cli --serial COM7 stream on
python -m esp_drone_cli --serial COM7 stream off
python -m esp_drone_cli --serial COM7 log --timeout 3 --telemetry
```

Parameter inspection and editing:

```powershell
python -m esp_drone_cli --serial COM7 list
python -m esp_drone_cli --serial COM7 get rate_kp_roll
python -m esp_drone_cli --serial COM7 set rate_kp_roll float 0.0030
python -m esp_drone_cli --serial COM7 save
python -m esp_drone_cli --serial COM7 reset
```

Current GitHub firmware default rate PID values in `params.c` are:

- `rate_kp_roll = 0.0030`
- `rate_kp_pitch = 0.0030`
- `rate_kp_yaw = 0.0030`
- `rate_ki_* = 0`
- `rate_kd_* = 0`

If lower values such as `0.0007 / 0.0007 / 0.0005` are used, document them as local tuning candidates or RAM/NVS overrides, not current firmware defaults.

Rate-loop bench commands:

```powershell
python -m esp_drone_cli --serial COM7 rate-test roll 20
python -m esp_drone_cli --serial COM7 rate-test pitch 20
python -m esp_drone_cli --serial COM7 rate-test yaw 20
python -m esp_drone_cli --serial COM7 rate-test roll 0
```

Hang-attitude bench commands:

```powershell
python -m esp_drone_cli --serial COM7 attitude-capture-ref
python -m esp_drone_cli --serial COM7 attitude-test start --base-duty 0.05
python -m esp_drone_cli --serial COM7 attitude-status
python -m esp_drone_cli --serial COM7 attitude-test stop
```

Ground tune / attitude ground verify commands:

```powershell
python -m esp_drone_cli --serial COM7 ground-capture-ref
python -m esp_drone_cli --serial COM7 ground-test start --base-duty 0.05
python -m esp_drone_cli --serial COM7 ground-test stop
python -m esp_drone_cli --serial COM7 attitude-ground-verify start --base-duty 0.05
python -m esp_drone_cli --serial COM7 attitude-ground-verify stop
```

Low-risk liftoff verify commands:

```powershell
python -m esp_drone_cli --serial COM7 liftoff-verify start --base-duty 0.10
python -m esp_drone_cli --serial COM7 liftoff-verify stop
```

All-motor test commands:

```powershell
python -m esp_drone_cli --serial COM7 all-motor-test start --duty 0.05 --duration-ms 1000
python -m esp_drone_cli --serial COM7 all-motor-test stop
```

Motor thrust balance diagnostics:

```powershell
python -m esp_drone_cli --serial COM7 motor-thrust-balance --duties 0.20,0.25,0.30
python -m esp_drone_cli --serial COM7 motor-thrust-balance --duties 0.20,0.25,0.30 --no-trim
python -m esp_drone_cli --serial COM7 motor-trim-estimate logs\YYYYMMDD_HHMMSS_motor_thrust_balance_summary.csv --apply
```

`motor-thrust-balance` uses the single-motor `motor-test` path, which still passes through the firmware `motor_set_armed_outputs` compensation layer. The summary prints input duty, `motor_scale`, `motor_offset`, and the scale/offset target duty for every tested motor and duty. `--no-trim` temporarily writes `motor_scale_m1..m4=1.0` and `motor_offset_m1..m4=0.0` for the run, then restores the previous values, so it can be used as the untrimmed comparison run.

The balance `score` is based on IMU vibration/disturbance response, not a thrust-stand force measurement. Do not treat `ratio_to_M1` from `motor-trim-estimate` as a real thrust ratio; it is only an empirical diagnostic signal for conservative `motor_scale` suggestions.

Experimental UDP manual examples:

```powershell
python -m esp_drone_cli --udp 192.168.4.1:2391 connect
python -m esp_drone_cli --udp 192.168.4.1:2391 udp-manual enable
python -m esp_drone_cli --udp 192.168.4.1:2391 udp-manual setpoint --throttle 0.05 --roll 0 --pitch 0 --yaw 0
python -m esp_drone_cli --udp 192.168.4.1:2391 udp-manual stop
```

STA examples after firmware logs `sta connected ip=192.168.50.42`:

```powershell
python -m esp_drone_cli --host 192.168.50.42 connect
python -m esp_drone_cli --host 192.168.50.42 get wifi_mode
python -m esp_drone_cli --host 192.168.50.42 udp-manual enable
python -m esp_drone_cli --host 192.168.50.42 udp-manual stop
```

UDP timeout messages distinguish the default SoftAP target from STA/custom targets. `ACK timeout` means the host reached the UDP transport but did not receive the expected command acknowledgement.

## Notes

- Use `capabilities` before running newer workflows against an older firmware image.
- Do not assume old protocol versions or command IDs. The protocol source of truth is `firmware/main/console/console_protocol.h`.
- Do not treat diagnostic passes as free-flight readiness.
