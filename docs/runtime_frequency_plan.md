# Runtime Frequency Plan

## Default Targets

- Motor output scheduling: `1000 Hz`
- IMU UART default return rate: `200 Hz`
- IMU UART optional high rate: `250 Hz`
- Estimator update: on fresh IMU samples only
- Rate PID: on fresh IMU samples only
- Angle PID: deferred until the bring-up gate is physically passed, then default `125 Hz`
- USB CDC telemetry: `200 Hz`
- UDP telemetry: `100 Hz`

## Current Parameter Guardrails

Stage 3 currently constrains the telemetry-rate parameters to:

- `telemetry_usb_hz`: `1..200`
- `telemetry_udp_hz`: `1..100`

These limits match the current bring-up scope and keep the binary CDC/UDP telemetry load within the intended safe range for bench testing.

## Why The Control Path Is Event-Driven

The IMU only uploads at up to `250 Hz`. Re-running the full estimator and rate loop at `1000 Hz` with stale data would not improve observability and would make timing semantics ambiguous.

The rewrite therefore separates:

- a `1 kHz` output and supervision scheduler
- a fresh-sample estimator/rate-update path

## Flight-Control Task Contract

The `flight_control_task` runs at `1 kHz`, but:

- estimator updates only when a new IMU sample arrives
- rate PID updates only when a new IMU sample arrives
- angle PID is not part of the current minimal stage 3 gate
- motor output shaping, kills, saturations, slew limits and loop timing run every tick

## Minimal Stage 3 Scope

Before any angle loop work, the allowed closed-loop path is only:

- fresh IMU sample
- estimator update
- rate PID update
- mixer update
- motor output update
- safety gating

That path is exposed through the `rate-test` bring-up command and its telemetry fields. No angle outer loop is enabled in this stage.

## Sample-Staleness Policy

When no new IMU sample is available:

- update `imu_age_us`
- keep safety and timeout checks running
- keep the last control outputs
- keep output scheduling running

When no new IMU sample is available, the firmware must not:

- recompute the full estimator
- integrate a new virtual attitude sample
- pretend a new rate measurement exists
