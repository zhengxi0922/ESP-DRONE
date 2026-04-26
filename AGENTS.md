# AGENTS.md

## Project goal

This repository is the ESP-DRONE bring-up and flight-control debugging project. The current priority is docs-code consistency, repeatable diagnostics, and a clean path toward a minimal stabilize workflow on the existing hardware.

## Fixed conventions

- ESP-IDF version: `v5.5.1`.
- Body frame: `+Y` nose/forward, `+X` right side, `+Z` up.
- Naming: `+pitch` nose up, `+roll` right side down, `+yaw` nose right.
- Desired logical motor order: `M1=left-front`, `M2=right-front`, `M3=right-rear`, `M4=left-rear`.
- Direction-sensitive logic must follow `docs/axis_truth_table.md` and `docs/motor_map.md`.
- Hardware cannot be changed during the current bring-up phase; absorb differences through software mapping, parameters, trim, and diagnostics.

## Token-saving rules for Codex

- Start every task by reading this file and `docs/CODEX_STATE.md`.
- Do not re-scan the whole repository unless the task explicitly requires it.
- On Windows, prefer PowerShell-native commands. Use `Get-ChildItem`, `Select-String`, and `Get-Content`; do not assume `rg` is available.
- Search only the files related to the current task first. Expand scope only when the local evidence is insufficient.
- Do not paste large code blocks or whole CSV/log files in the answer unless explicitly requested.
- For log/CSV tasks, write or use a summarizer and report only the extracted facts.

## Safety and scope rules

- Do not silently change axis signs, motor ordering, mixer signs, or IMU mapping.
- Treat `hang-attitude`, `ground-tune`, `liftoff-verify`, and `short-hop` as diagnostics/verification paths, not as a finished free-flight mode.
- Do not claim free-flight stabilize, autonomous takeoff, altitude hold, or position hold is implemented unless the code and tests prove it.
- Documentation-only tasks must not change firmware control logic.
- If documentation and code disagree, update documentation to match code and clearly mark any desired values as recommendations, not current defaults.

## Required output format

For each task, output only:

1. Checked files
2. Root cause / docs-code mismatch
3. Changed files
4. Tests run
5. Next step
