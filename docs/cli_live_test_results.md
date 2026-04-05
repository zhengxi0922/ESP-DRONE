# CLI Live Test Results

**Language / 语言：** **English** | [简体中文](./cli_live_test_results.zh-CN.md)

## Environment

- date: `2026-04-03`
- host: Windows PowerShell
- target serial port: `COM4`
- safety intent: non-flight commands only, props removed or frame restrained
- transport covered in this round: `serial`
- transport deferred in this round: `udp`

## Stage-D Precheck

- README GUI or CLI wording: confirmed present
- GUI smoke tests: confirmed present and passing locally
- core ownership rule: confirmed
  - CLI uses `DeviceSession`
  - GUI uses `DeviceSession`
  - `client.py` is a compatibility shim
  - `gui/` does not own protocol or transport

Local proof before live testing:

- `python -m pytest tools\esp_drone_cli\tests -q` -> `20 passed`
- `powershell -ExecutionPolicy Bypass -File tools\idf.ps1 build` -> `PASS`

## Firmware And Tool Fixes Included In This Verification

The following fixes were applied before the final serial rerun:

- `DIRECT` IMU mode now requests `attitude + quaternion + gyro/acc`
  - fixes zeroed `gyro_x/y/z` in direct-mode telemetry
  - allows `calib gyro` without switching to `RAW`
  - allows the `rate-test` command path to receive live gyro data
- CLI `connect` no longer sends a second redundant `HELLO_REQ`
- `reboot` now:
  - stops stream and test outputs first
  - sends ACK
  - waits one short USB frame window
  - then calls `esp_restart()`

## Serial Live Test Summary

Local artifacts were captured during the run for bench-side inspection:

- parameter export JSON
- telemetry CSV dump
- JSON summary of the serial command matrix

These artifacts are intentionally kept as local test byproducts and are not required as tracked repository files.

All serial non-flight commands listed in [CLI Live Test Matrix](./cli_live_test_matrix.md) were executed on hardware and reached an acceptable result.

## Command Results

| Command | Example Input | Expected | Actual | Result |
|---|---|---|---|---|
| `connect` | `python -m esp_drone_cli --serial COM4 connect` | print `HELLO_RESP` device info | `DeviceInfo(protocol_version=1, imu_mode=1, arm_state=0, stream_enabled=0, feature_bitmap=15)` | `PASS` |
| `disconnect` | implicit session close | port closes cleanly after each command | repeated reconnects on the same `COM4` port succeeded across the matrix | `PASS` |
| `get` | `... get telemetry_usb_hz` | read the current parameter | returned `ParamValue(... value=150)` before reset and `200` after reset | `PASS` |
| `list` | `... list` | enumerate the full parameter table | returned all current parameters | `PASS` |
| `stream on` | `... stream on` | enable telemetry stream | returned success, no error | `PASS` |
| `stream off` | `... stream off` | disable telemetry stream | returned success, no error | `PASS` |
| `set` | `... set telemetry_usb_hz u32 120` | write one parameter | value updated and could be read back | `PASS` |
| `save` | `... save` | persist the current parameter set | returned success, no error | `PASS` |
| `reset` | `... reset` | restore defaults | `telemetry_usb_hz` returned to `200` | `PASS` |
| `export` | `... export docs/_tmp_cli_live/params_export.json` | write a snapshot JSON | file created successfully | `PASS` |
| `import` | `... import ... --save` | restore a snapshot JSON | `applied 44 parameters ...`, value restored from snapshot | `PASS` |
| `arm` | `... arm` | arm while bench-safe conditions are valid | returned success, no error | `PASS` |
| `disarm` | `... disarm` | disarm and stop active bench modes | returned success, no error | `PASS` |
| `kill` | `... kill` | immediately stop outputs and enter the kill path | returned success, no error | `PASS` |
| `reboot` | `... reboot` | ACK, then restart the board | returned success, and later reconnect on the same `COM4` port succeeded | `PASS` |
| `calib gyro` | `... calib gyro` | perform gyro zeroing when IMU health is OK | returned success in default direct mode after the direct-mode gyro feed fix | `PASS` |
| `calib level` | `... calib level` | perform attitude level trim | returned success | `PASS` |
| `motor-test` | `... motor-test m1 0.05`, then `0.0` | command on or off for one motor at a time | `m1..m4` low-duty on or off command paths all returned success | `PASS` |
| `axis-test` | `... axis-test roll 0.05`, then `0.0` | command bench mixer bias per axis | `roll`, `pitch`, and `yaw` positive and zero commands all returned success | `PASS` |
| `rate-test` | `... rate-test roll 20`, then `0` | command low-amplitude rate bench mode per axis | `roll`, `pitch`, and `yaw` positive and zero commands all returned success | `PASS` |
| `log` | `... log --timeout 2 --telemetry` | print live event and telemetry data | telemetry dictionaries printed successfully | `PASS` |
| `dump-csv` | `... dump-csv docs/_tmp_cli_live/log_dump.csv --duration 2` | collect telemetry to CSV | wrote `335` telemetry rows | `PASS` |

## Telemetry Proof Points

Observed during `log --timeout 2 --telemetry`:

- `gyro_x/y/z` now populate in `DIRECT` mode
- `battery_voltage` and `battery_adc_raw` are present
- `imu_age_us` and `loop_dt_us` are present
- `arm_state`, `failsafe_reason`, and `control_mode` are present
- example values seen during the test:
  - `gyro_x = -0.1220703125`
  - `gyro_y = -0.244140625`
  - `gyro_z = 0.06103515625`
  - `battery_voltage ~ 4.20V`
  - `loop_dt_us ~ 1000`
  - `imu_mode = 1`
  - `imu_health = 1`

## Serial Acceptance Conclusion

Serial CLI verification is acceptable for the current non-flight scope:

- connect or disconnect behavior is stable on `COM4`
- parameter read, write, save, reset, export, and import all pass
- safety and system commands `arm`, `disarm`, `kill`, and `reboot` all pass
- calibration and bench actions `calib`, `motor-test`, `axis-test`, and `rate-test` all pass
- logging and CSV export both pass

## Deferred Items

- UDP live validation is deferred, not failed
- reason: the current firmware still does not implement the device-side UDP CLI path, so serial acceptance is not blocked by this

## Failure Items And Fix Record

No remaining serial CLI failure blocks were open after the `2026-04-03` rerun.

Resolved during this round:

1. `calib gyro` previously failed in default `DIRECT` mode
   - cause: the direct-mode return set did not include `gyro/acc`
   - fix: request `attitude + quaternion + gyro/acc` in direct mode
   - retest: `PASS`
2. `reboot` previously ACKed but left the CLI unable to reconnect
   - cause: the restart sequence was too abrupt for the USB CDC session
   - fix: ACK first, stop stream or test outputs, add a short delay, then restart
   - retest: `PASS`
