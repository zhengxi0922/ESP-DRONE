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

`motor.c` currently uses fixed `LEDC_TIMER_8_BIT` PWM resolution. PWM frequency is parameterized, but PWM resolution is not.

## Not Yet Implemented

The current code does not implement per-motor thrust compensation with:

- per-motor `scale`
- per-motor `offset`
- per-motor `min_start`
- per-motor `deadband`
- per-motor `gamma`

Do not describe `motor_output_map` as per-motor thrust compensation. Mapping and thrust compensation are different layers.

## Current motor output implementation note

Current `motor.c` uses fixed 8-bit LEDC PWM resolution (`LEDC_TIMER_8_BIT`). `motor_pwm_freq_hz` parameterizes PWM frequency only; PWM resolution is not parameterized.

`motor_output_map` maps logical motor order to physical outputs. It is not per-motor thrust compensation. The current firmware does not yet implement per-motor `scale`, `offset`, `min_start`, `deadband`, or `gamma` compensation.
