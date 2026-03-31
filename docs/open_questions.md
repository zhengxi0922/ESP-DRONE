# Open Questions

There are no remaining Stage-1 blockers. The following items are non-blocking and are handled by design defaults plus bring-up validation.

## Non-Blocking Items

- Verify the default IMU installation assumption on hardware using physical motion and telemetry sign checks.
- Decide after bring-up whether `250 Hz` should become a recommended IMU profile or remain an optional high-rate mode only.
- If full stock-App parity becomes important later, capture and verify whether a dedicated legacy `arm/disarm/mode` packet exists.

## Locked Decisions

- ESP-IDF version is `v5.5.1`.
- `ATK-IMU901` and `ATK-MS901M` are treated as the same module.
- Legacy UDP compatibility is explicitly scope-limited.
