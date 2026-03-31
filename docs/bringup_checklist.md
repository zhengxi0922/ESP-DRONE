# Bring-Up Checklist

This checklist must be executed between the bottom-layer framework stage and the estimator/controller stage.

## Connectivity

- USB CDC enumerates and accepts the binary CLI handshake
- IMU UART receives valid `ATK-MS901M` frames
- LED state machine changes between init, ready and fault states

## Power And ADC

- `BAT_ADC` raw value is readable
- battery voltage conversion matches the `100k / 100k` divider
- battery low thresholds can be observed in telemetry

## Motor And Safety

- `motor_test m1..m4 <duty>` drives exactly one motor at a time
- `M1..M4` order matches the documented map
- `arm`, `disarm`, `kill` work through the CLI
- stick-gesture `arm/disarm` works with the configured hold times

## Direction Validation

- Physical motion matches the telemetry sign convention
- `+roll` command raises `M1` and `M4`
- `+pitch` command raises `M3` and `M4`
- `+yaw` command raises `M1` and `M3`

No PID or free-flight tuning is allowed before this checklist passes.
