# Motor Map

Language / 璇█: English | [绠€浣撲腑鏂嘳(./motor_map.zh-CN.md)

## Body Frame

- `+Y` = nose / forward
- `+X` = right side
- `+Z` = up
- `+pitch` = nose up
- `+roll` = right side down
- `+yaw` = nose right

## Desired Logical Motor Order

The flight-control documentation uses this logical order:

- `M1` = left-front
- `M2` = right-front
- `M3` = right-rear
- `M4` = left-rear

Direction-sensitive logic must stay consistent with `axis_truth_table.md`.

## Implemented Mapping Feature

The firmware exposes `motor_output_map[4]` through parameters as:

- `motor_output_map0`
- `motor_output_map1`
- `motor_output_map2`
- `motor_output_map3`

`motor.c` writes a logical motor to a physical LEDC channel through this mapping before calling `ledc_set_duty()`.

This is a channel mapping feature. It can absorb motor channel wiring/order differences in software.

## Current Motor Output Controls

Current implemented output controls include:

- `motor_output_map[4]`
- global `motor_idle_duty`
- global `motor_max_duty`
- global `motor_startup_boost_duty`
- global `motor_slew_limit_per_tick`
- parameterized `motor_pwm_freq_hz`

`motor.c` uses parameterized PWM resolution via `motor_pwm_resolution_bits`. Default is 10-bit (`LEDC_TIMER_10_BIT`). Supports 8, 10, and 12 bits.

## Per-motor thrust compensation

Per-motor thrust compensation is implemented in `motor_apply_compensation()`:

- per-motor `motor_trim` — additive trim before gamma
- per-motor `motor_gamma` — power-law gamma correction
- per-motor `motor_scale` — multiplicative scale
- per-motor `motor_offset` — additive offset
- per-motor `motor_deadband` — deadband threshold (output zero below this)
- per-motor `motor_min_start` — minimum start duty

`motor_output_map` is channel mapping, not per-motor thrust compensation. Mapping and thrust compensation are different layers.

## Current motor output implementation note

`motor.c` uses parameterized PWM resolution (`motor_pwm_resolution_bits`, default 10-bit). Both PWM frequency and resolution are parameterized.

`motor_output_map` maps logical motor order to physical outputs. Per-motor thrust compensation (`motor_trim`, `motor_gamma`, `motor_scale`, `motor_offset`, `motor_deadband`, `motor_min_start`) is implemented in `motor_apply_compensation()`.
