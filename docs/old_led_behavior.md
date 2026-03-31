# Old LED Behavior

## Source Scope

The old repository provides partial LED semantics. It is useful as a visual reference, not as a complete state machine for the rewrite.

## Extracted Legacy Semantics

| Legacy semantic | Observed behavior |
|---|---|
| `alive` | Slow heartbeat pulse |
| `lowbat` | Low-battery warning pattern |
| `charging` | Repeating slow pulse |
| `charged` | Solid-on style indication |
| `linkUp` | Short activity flash |
| `linkDown` | Short activity flash on the alternate link LED |
| `testPassed` | Startup pulse sequence |
| `testFailed` | Startup pulse sequence on system LED |

## Rewrite Decision

The new firmware uses a dedicated `led_status` state machine. Other modules submit logical status only; they never drive GPIO directly.

## New Default State Table

| State | LED behavior |
|---|---|
| `INIT_WAIT_IMU` | Yellow slow blink |
| `DISARMED_READY` | Yellow solid |
| `ARMED_HEALTHY` | Green solid |
| `LOW_BAT` | Red slow blink |
| `FAILSAFE` | Red fast blink |
| `IMU_ERROR` | Red fast blink |
| `RC_LOSS` | Red fast blink |
| `FAULT_LOCK` | Red solid |
| `CALIBRATING` | Green / Yellow alternate |
| `PARAM_SAVE` | Green / Yellow alternate |

## Priority

`FAULT_LOCK > FAILSAFE/IMU/RC error > LOW_BAT > CALIB/SAVE > ARMED_HEALTHY > DISARMED_READY > INIT_WAIT_IMU`
