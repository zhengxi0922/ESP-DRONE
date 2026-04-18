# Experimental UDP Manual Control

This feature is experimental bench/manual control only. It is not a mature free-flight mode, not closed-loop altitude hold, not heading hold, and not a stabilize/angle-ready workflow.

## Scope

- Transport: binary CLI/GUI UDP protocol on port `2391`, reachable through the ESP32 SoftAP transport described in [softap_udp_transport.md](./softap_udp_transport.md).
- GUI: `UDP Control` tab in the Python GUI.
- Firmware mode: `CONTROL_MODE_UDP_MANUAL`.
- Control style: throttle is a collective/base duty target; roll/pitch use the flat-ground reference outer loop to generate rate setpoints, and yaw remains mapped through the existing rate PID before mixer output.
- Legacy UDP compatibility on port `2390` is unchanged.

## Capability Gate

The firmware advertises:

- `protocol_version=5`
- `CONSOLE_FEATURE_UDP_MANUAL_CONTROL = 1 << 6`
- feature name in host tools: `udp_manual_control`

Host tools must reject UDP manual commands when the device does not advertise this capability.

## Protocol Additions

Commands:

- `CMD_UDP_MANUAL_ENABLE = 13`
- `CMD_UDP_MANUAL_DISABLE = 14`
- `CMD_UDP_MANUAL_SETPOINT = 15`
- `CMD_UDP_TAKEOFF = 16`
- `CMD_UDP_LAND = 17`
- `CMD_UDP_MANUAL_STOP = 18`

Message:

- `MSG_UDP_MANUAL_SETPOINT = 0x50`

Setpoint payload, little-endian:

```c
typedef struct __attribute__((packed)) {
    float throttle;
    float pitch;
    float roll;
    float yaw;
} console_udp_manual_setpoint_t;
```

The host API is:

```python
session.udp_manual_enable()
session.udp_manual_disable()
session.udp_manual_setpoint(throttle, pitch, roll, yaw)
session.udp_takeoff()
session.udp_land()
session.udp_manual_stop()
```

## Parameters

These parameters are exposed through normal CLI/GUI parameter read/write and saved in the firmware parameter blob:

- `udp_manual_max_pwm`: max manual collective/base duty fraction.
- `udp_takeoff_pwm`: takeoff collective/base duty ramp target.
- `udp_land_min_pwm`: landing/timeout safe duty floor.
- `udp_manual_timeout_ms`: setpoint watchdog timeout, valid range `200..1000 ms`, default `1000 ms`.
- `udp_manual_axis_limit`: normalized command clamp for manual inputs. In current firmware, yaw at the clamp limit maps to about `45 dps`; roll/pitch inputs are not mixed directly while attitude hold is active.

The GUI displays max duty as `Max PWM (%)`, but firmware stores normalized duty fractions.

## Firmware Safety Behavior

- UDP manual setpoints, landing, stop, and disable are honored in `CONTROL_MODE_UDP_MANUAL`; `UDP_TAKEOFF` may enter that mode from idle/disarmed state.
- `UDP_MANUAL_ENABLE` requires idle mode and disarmed state unless the mode is already active.
- `UDP_TAKEOFF` requests arm through the existing safety layer and ramps base duty to `udp_takeoff_pwm`.
- `UDP_LAND` keeps the rate loop active, ramps base duty down, and auto-disarms when the ramp reaches the safe floor.
- After `UDP_LAND` has entered the landing stage, later manual setpoint frames are acknowledged but ignored so they cannot cancel the descent.
- `UDP_MANUAL_STOP` and `UDP_MANUAL_DISABLE` clear setpoints, stop motors, leave manual mode, and request disarm.
- Setpoints are finite-checked and clamped by firmware.
- During `CONTROL_MODE_UDP_MANUAL`, throttle is the collective/base duty target. Roll/pitch rate setpoints come from `ground_tune_compute()` using the captured flat-ground reference and then pass through the existing rate PID. Yaw keeps the manual normalized-input-to-rate-setpoint path before the same rate PID.
- UDP manual enable/takeoff requires a valid flat-ground reference. If one is not already captured, firmware captures the current IMU quaternion through the ground-tune reference path.
- If no setpoint arrives for `udp_manual_timeout_ms`, firmware zeros manual yaw, keeps roll/pitch on the attitude outer loop, and reduces throttle toward `udp_land_min_pwm`.
- If the timeout persists for `3 * udp_manual_timeout_ms`, firmware requests disarm and stops the manual mode.
- Kill remains highest priority through the existing `CMD_KILL` path.

## GUI Behavior

The `UDP Control` tab includes:

- Red experimental safety warning.
- Enable/Disable/Stop buttons.
- Takeoff/Land buttons.
- Forward/Backward, Yaw Left/Yaw Right, Up/Down controls.
- Max PWM and setpoint inputs.
- Watchdog, mode, armed, and battery status labels.
- Event log entries for every manual action.

Directional buttons send bounded manual setpoints through `DeviceSession`; they do not write private GUI-only protocol.

Screenshot: `docs/images/udp_control_tab.png`.

## Manual Verification Steps

1. Build and flash firmware that advertises `protocol_version=5` and `udp_manual_control=True`.
2. Connect the PC Wi-Fi to the default `ESP-DRONE` SoftAP.
3. Start the Python GUI with `pip install -e tools/esp_drone_cli[gui]` installed.
4. Connect using UDP host `192.168.4.1`, port `2391`, or the target network endpoint.
5. Confirm capabilities show `udp_manual_control=True`.
6. Set a conservative `Max PWM (%)`, for example `10..12`.
7. Click `Enable UDP Manual`.
8. Verify repeated setpoint frames are visible in the event log and that watchdog age stays below the timeout while enabled.
9. Press and release `Forward`, `Backward`, `Yaw Left`, `Yaw Right`, `Up`, and `Down`; confirm setpoints return to zero when expected.
10. Click `Takeoff` only for the bounded low-risk verification workflow with prop safety handled; confirm ramp behavior, not a step jump.
11. Click `Land`; confirm ramp-down and auto-disarm behavior.
12. Click `Stop / Zero` and then `Kill`; confirm both override any previous setpoint and no high throttle continues after disconnect or timeout.

Do not use this workflow as proof that the vehicle is free-flight ready.
