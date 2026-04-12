# ESP-Drone UDP Compatibility

**Language / 语言：** **English** | [简体中文](./esp_drone_udp_compat.zh-CN.md)

## Compatibility Goal

Compatibility is limited to the legacy control entry path. The rewrite does not promise full stock-App feature parity.

## Legacy Endpoint

- port: `2390`
- checksum model: `sum(payload) & 0xFF`
- accepted legacy data:
  - 12-byte old App control packet
  - CRTP-over-UDP packet path

## Confirmed Compatible Items

- basic 4-channel semantic compatibility for `roll`, `pitch`, `yaw`, and `thrust`
- basic downlink path through the old UDP or CRTP compatibility mechanism
- legacy control-port number `2390`

## Not Promised In V1

- full stock-App feature parity
- legacy transport support for new parameter APIs
- legacy transport support for bulk log export
- explicit stock-App `arm`, `disarm`, or `mode` packet compatibility unless later verified by capture

## Replacement Strategy

- `2390`: legacy compatibility layer for control ingress and basic downlink
- `2391`: new binary CLI/GUI UDP protocol, reachable at `192.168.4.1:2391` after joining the default ESP32 SoftAP
- `USB CDC`: the same new binary protocol as `2391`
- explicit `arm`, `disarm`, and `kill`: always available through the new CLI
- the legacy control path can still arm or disarm through stick gestures and the safety state machine

## Experimental UDP Manual Control

Protocol version `5` adds `udp_manual_control` on the new binary UDP port `2391`.
This path is explicitly experimental. The `throttle` field is treated as a
collective/base duty target, while roll/pitch/yaw commands are mapped through the
existing rate PID before mixer output. It adds manual enable/disable/stop, setpoint,
takeoff, and land commands with firmware-side watchdog and max-duty clamps. It is not
a mature free-flight takeoff/land mode.

See [udp_manual_control_protocol.md](./udp_manual_control_protocol.md).
For SoftAP setup and GUI UDP transport connection steps, see [softap_udp_transport.md](./softap_udp_transport.md).
