# ESP-Drone UDP Compatibility

## 中文摘要

- 本项目只承诺旧 `ESP-Drone` 控制入口的基础兼容，不承诺完整 stock App 特性对等。
- `2390` 端口保留给 legacy 控制兼容层。
- `2391` 和 `USB CDC` 走新的二进制 CLI 协议。

## Compatibility Goal

Compatibility is limited to the legacy control entry path. The rewrite does not promise full stock-App feature parity.

## Legacy Endpoint

- Port: `2390`
- Checksum model: `sum(payload) & 0xFF`
- Accepted legacy data:
  - 12-byte old App control packet
  - CRTP-over-UDP packet path

## Confirmed Compatible Items

- Basic 4-channel semantic compatibility for `roll`, `pitch`, `yaw`, `thrust`
- Basic downlink path through the old UDP/CRTP compatibility mechanism
- Legacy control-port number `2390`

## Not Promised In V1

- Full stock-App feature parity
- Legacy transport support for new parameter APIs
- Legacy transport support for bulk log export
- Explicit stock-App `arm/disarm/mode` packet compatibility unless later verified by capture

## Replacement Strategy

- `2390`: legacy compatibility layer for control ingress and basic downlink
- `2391`: new binary CLI UDP protocol
- `USB CDC`: same new binary protocol as `2391`
- Explicit `arm/disarm/kill`: always available through the new CLI
- Legacy control path can still arm or disarm through stick gestures and the safety state machine
