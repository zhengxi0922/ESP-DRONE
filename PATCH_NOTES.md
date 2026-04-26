# ESP-DRONE docs-code consistency patch notes

This overlay is documentation-only plus one docs sync checker and one pytest wrapper. It does not modify flight-control logic.

## Important discrepancy found

The requested instruction said to update current defaults to:

- `rate_kp_roll = 0.0007`
- `rate_kp_pitch = 0.0007`
- `rate_kp_yaw = 0.0005`

However, the current GitHub `firmware/main/params/params.c` inspected during this pass sets:

- `rate_kp_roll = 0.0030`
- `rate_kp_pitch = 0.0030`
- `rate_kp_yaw = 0.0030`
- all rate I/D terms = `0`

Because this task is explicitly docs-code consistency and must not change flight-control logic, the replacement docs describe `0.0030 / 0.0030 / 0.0030` as the current code defaults and describe `0.0007 / 0.0007 / 0.0005` as conservative tuning candidates only.

If you want `0.0007 / 0.0007 / 0.0005` to become current defaults, that is a separate firmware change to `params.c`, not a docs-only consistency pass.

## Files included

- `AGENTS.md`
- `README.md`
- `README.zh-CN.md`
- `docs/README.md`
- `docs/README.zh-CN.md`
- `docs/CODEX_STATE.md`
- `docs/TUNING_DECISIONS.md`
- `docs/udp_manual_control_protocol.md`
- `docs/python_cli_usage.md`
- `docs/python_cli_usage.zh-CN.md`
- `docs/motor_map.md`
- `docs/motor_map.zh-CN.md`
- `tools/check_docs_sync.py`
- `tools/esp_drone_cli/tests/test_docs_sync.py`

## Replacement method

Copy this overlay into the repository root, preserving paths.

PowerShell example:

```powershell
# after extracting this zip
Copy-Item -Recurse -Force .\esp_drone_docs_sync_patch\* D:\0Work\Codex\ESP-drone-main-merge\
cd D:\0Work\Codex\ESP-drone-main-merge
python tools\check_docs_sync.py --repo-root .
python -m pytest tools\esp_drone_cli\tests\test_docs_sync.py
```

## Scope intentionally not changed

- No firmware control logic changed.
- No PID defaults changed in `params.c`.
- No motor PWM implementation changed.
- No CLI command implementation changed.
