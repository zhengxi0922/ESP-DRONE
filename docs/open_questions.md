# Open Questions

**Language / 语言：** **English** | [简体中文](./open_questions.zh-CN.md)

There are no remaining stage-1 blockers. The following items are non-blocking and are handled by design defaults plus bring-up validation.

## Non-Blocking Items

- verify the default IMU installation assumption on hardware using physical motion and telemetry sign checks
- decide after bring-up whether `250 Hz` should become a recommended IMU profile or remain an optional high-rate mode only
- if full stock-App parity becomes important later, capture and verify whether a dedicated legacy `arm`, `disarm`, or `mode` packet exists
- verify LED active level on hardware during bring-up, because the current stage assumes GPIO high means LED on
- verify motor PWM polarity during bring-up, specifically whether increasing commanded duty really increases thrust on the brushed motor-driver path

## Locked Decisions

- ESP-IDF version is `v5.5.1`
- `ATK-IMU901` and `ATK-MS901M` are treated as the same module
- legacy UDP compatibility is explicitly scope-limited
