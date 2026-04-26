# Experimental UDP Manual Control

This feature is experimental bench/manual control only. It is not a mature free-flight mode, not closed-loop altitude hold, not heading hold, and not a stabilize/angle-ready workflow.

## Scope

- Transport: binary CLI/GUI UDP protocol on port `2391`, reachable through the ESP32 SoftAP transport described in [softap_udp_transport.md](./softap_udp_transport.md).
- GUI: `UDP Control` tab in the Python GUI.
- Firmware mode: `CONTROL_MODE_UDP_MANUAL`.
- Control style: throttle is a collective/base duty target; roll/pitch use the flat-ground reference outer loop to generate rate setpoints, and yaw remains mapped through the existing rate PID before mixer output.
- Legacy UDP compatibility on port `2390` is unchanged.

## Capability Gate

The firmware advertises the current protocol and feature bitmap through `console_hello_resp_t`.

Current code facts from `firmware/main/console/console_protocol.h`:

- `CONSOLE_PROTOCOL_VERSION = 0x0A`
- `CONSOLE_FEATURE_UDP_MANUAL_CONTROL = 1 << 6`
- host feature name: `udp_manual_control`

Host tools must reject UDP manual commands when the device does not advertise this capability. Do not hard-code an older protocol version v5 in documentation or host checks.

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
- `udp_manual_timeout_ms`: setpoint watchdog timeout, valid range `200..1000 ms`.
- `udp_manual_axis_limit`: normalized command clamp for manual inputs.

The GUI displays max duty as `Max PWM (%)`, but firmware stores normalized duty fractions.

## Firmware Safety Behavior

- UDP manual setpoints, landing, stop, and disable are honored in `CONTROL_MODE_UDP_MANUAL`; `UDP_TAKEOFF` may enter that mode from idle/disarmed state.
- `UDP_MANUAL_ENABLE` requires idle mode and disarmed state unless the mode is already active.
- `UDP_TAKEOFF` requests arm through the existing safety layer and ramps base duty to `udp_takeoff_pwm`.
- `UDP_LAND` keeps the rate loop active, ramps base duty down, and auto-disarms when the ramp reaches the safe floor.
- After `UDP_LAND` has entered the landing stage, later manual setpoint frames are acknowledged but ignored so they cannot cancel the descent.
- `UDP_MANUAL_STOP` and `UDP_MANUAL_DISABLE` clear setpoints, stop motors, leave manual mode, and request disarm.
- Setpoints are finite-checked and clamped by firmware.
- During `CONTROL_MODE_UDP_MANUAL`, throttle is the collective/base duty target.

Roll/pitch rate setpoints come from `ground_tune_compute()` using the captured flat-ground reference and then pass through the existing rate PID. Yaw keeps the manual normalized-input-to-rate-setpoint path before the same rate PID.

## Ground Reference Requirement

UDP manual depends on a valid flat-ground reference.

- `UDP_MANUAL_ENABLE` / `UDP_TAKEOFF` may attempt to capture the current IMU quaternion through the ground-tune reference path if no reference has already been captured.
- The runtime control loop does **not** continuously auto-recapture the reference.
- If `CONTROL_MODE_UDP_MANUAL` is running and `runtime_state_get_ground_tune_state().ref_valid` becomes false, `app_main.c` stops UDP manual immediately with `udp manual stopped: ground reference missing`.
- If estimator/attitude validity fails during runtime, UDP manual is also stopped rather than silently falling back to an unsafe mode.

This means a valid reference is a preflight/entry requirement and a runtime safety gate. Do not document it as an automatic in-flight recovery mechanism.

## Not A Finished Flight Mode

Current UDP manual is experimental bench/manual control. It should not be described as:

- finished free-flight stabilize mode
- autonomous takeoff
- altitude hold
- heading hold
- position hold

Use it as a controlled bring-up path only.


## Runtime ground reference rule

UDP manual enable/takeoff may try to capture a ground reference before the mode starts. During runtime, the firmware does not keep recapturing it automatically. If `ground_ref_valid` becomes false, `app_main.c` stops UDP manual immediately instead of continuing with a missing reference.
