# CLI Live Test Matrix

**Language / 语言：** **English** | [简体中文](./cli_live_test_matrix.zh-CN.md)

This matrix tracks non-flight CLI verification for the current Python toolchain.

Assumptions:

- props removed or the frame restrained
- no free flight
- low-power `motor-test` only
- low-amplitude `axis-test` and `rate-test` only
- serial coverage first, UDP only if the device-side path is actually available

| Command / Area | Example Input | Needs real hardware | Needs motors powered | Allowed without props | Serial target | UDP target | Current status |
|---|---|---:|---:|---:|---:|---:|---|
| `connect` | `python -m esp_drone_cli --serial COM4 connect` | yes | no | yes | yes | yes | `PASS` on `COM4` |
| `disconnect` | implicit CLI session close after command exit | yes | no | yes | yes | yes | `PASS` by repeated open or close across the serial matrix |
| `get` | `... get telemetry_usb_hz` | yes | no | yes | yes | yes | `PASS` on serial |
| `list` | `... list` | yes | no | yes | yes | yes | `PASS` on serial |
| `stream on` | `... stream on` | yes | no | yes | yes | yes | `PASS` on serial |
| `stream off` | `... stream off` | yes | no | yes | yes | yes | `PASS` on serial |
| `set` | `... set telemetry_usb_hz u32 100` | yes | no | yes | yes | yes | `PASS` on serial |
| `save` | `... save` | yes | no | yes | yes | yes | `PASS` on serial |
| `reset` | `... reset` | yes | no | yes | yes | yes | `PASS` on serial |
| `export` | `... export params.json` | yes | no | yes | yes | yes | `PASS` on serial |
| `import` | `... import params.json` | yes | no | yes | yes | yes | `PASS` on serial |
| `arm` | `... arm` | yes | yes | yes | yes | yes | `PASS` on serial bench setup |
| `disarm` | `... disarm` | yes | yes | yes | yes | yes | `PASS` on serial bench setup |
| `kill` | `... kill` | yes | yes | yes | yes | yes | `PASS` on serial bench setup |
| `reboot` | `... reboot` | yes | no | yes | yes | yes | `PASS` on serial; reconnect verified after reboot |
| `calib gyro` | `... calib gyro` | yes | no | yes | yes | yes | `PASS` on serial |
| `calib level` | `... calib level` | yes | no | yes | yes | yes | `PASS` on serial |
| `motor-test` | `... motor-test m1 0.05` | yes | yes | yes | yes | yes | `PASS` on serial at low duty (`m1..m4`, on or off) |
| `axis-test` | `... axis-test roll 0.05` | yes | yes | yes | yes | yes | `PASS` on the serial command path for `roll`, `pitch`, and `yaw` |
| `rate-test` | `... rate-test roll 20` | yes | yes | yes | yes | yes | `PASS` on the serial command path for `roll`, `pitch`, and `yaw` |
| `log` | `... log --timeout 3 --telemetry` | yes | no | yes | yes | yes | `PASS` on serial |
| `dump-csv` | `... dump-csv sample.csv --duration 3` | yes | no | yes | yes | yes | `PASS` on serial |

## Notes

- there is no standalone `disconnect` subcommand today; disconnect is implicit in `DeviceSession.close()` when the process exits
- serial validation was completed on `COM4` on `2026-04-03`
- `DIRECT` mode now requests `attitude + quaternion + gyro/acc`, so `gyro_x/y/z`, `calib gyro`, and the `rate-test` command path work without switching to `RAW`
- `reboot` is considered verified only after `connect` succeeds again on the same `COM4` port
- UDP validation is still deferred; the current firmware task graph does not yet expose the device-side UDP CLI path, so this does not block serial CLI acceptance
