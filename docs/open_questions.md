# Open Questions

## 中文摘要

- 当前没有阻断阶段推进的硬冲突，剩下的都是 bring-up 阶段需要实测确认的非阻断项。
- 重点待实测项包括：IMU 安装方向、LED 有效电平、电机 PWM 极性、`250 Hz` 是否值得长期作为推荐档位。

There are no remaining Stage-1 blockers. The following items are non-blocking and are handled by design defaults plus bring-up validation.

## Non-Blocking Items

- Verify the default IMU installation assumption on hardware using physical motion and telemetry sign checks.
- Decide after bring-up whether `250 Hz` should become a recommended IMU profile or remain an optional high-rate mode only.
- If full stock-App parity becomes important later, capture and verify whether a dedicated legacy `arm/disarm/mode` packet exists.
- Verify LED active level on hardware during bring-up, because the current stage assumes GPIO high means LED on.
- Verify motor PWM polarity during bring-up, specifically whether increasing commanded duty really increases thrust on the brushed motor driver path.

## Locked Decisions

- ESP-IDF version is `v5.5.1`.
- `ATK-IMU901` and `ATK-MS901M` are treated as the same module.
- Legacy UDP compatibility is explicitly scope-limited.
