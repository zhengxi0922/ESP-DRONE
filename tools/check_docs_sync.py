#!/usr/bin/env python3
"""Check that key documentation is synchronized with firmware constants.

This script is intentionally conservative:
- It checks the current console protocol version from firmware code.
- It checks that UDP manual docs mention that current version.
- It parses command IDs, feature bits, and selected parameter defaults for future use.
- It flags known obsolete PID default values only when they are written as current defaults.

It must not modify files.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


PARAM_KEYS = (
    "rate_kp_roll",
    "rate_kp_pitch",
    "rate_kp_yaw",
    "rate_ki_roll",
    "rate_ki_pitch",
    "rate_ki_yaw",
    "rate_kd_roll",
    "rate_kd_pitch",
    "rate_kd_yaw",
    "motor_pwm_freq",
    "motor_pwm_freq_hz",
    "motor_idle_duty",
    "motor_max_duty",
    "udp_manual_max_pwm",
)

OBSOLETE_PID_VALUES = {
    "0.0024",
    "0.0026",
    "0.0028",
}

DEFAULT_CONTEXT_RE = re.compile(
    r"(current|default|defaults|firmware default|code default|params\.c default)",
    re.IGNORECASE,
)


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8-sig", errors="replace")


def parse_protocol_version(console_protocol_h: Path) -> tuple[str, int]:
    text = read_text(console_protocol_h)
    match = re.search(
        r"#define\s+CONSOLE_PROTOCOL_VERSION\s+(0x[0-9A-Fa-f]+|\d+)u?\b",
        text,
    )
    if not match:
        raise ValueError(f"cannot find CONSOLE_PROTOCOL_VERSION in {console_protocol_h}")

    raw = match.group(1)
    value = int(raw, 16) if raw.lower().startswith("0x") else int(raw, 10)
    return f"0x{value:02X}", value


def parse_feature_bits(console_protocol_h: Path) -> dict[str, str]:
    text = read_text(console_protocol_h)
    result: dict[str, str] = {}
    pattern = re.compile(
        r"#define\s+(CONSOLE_FEATURE_[A-Z0-9_]+)\s+(.+?)(?:\r?\n|$)"
    )
    for name, value in pattern.findall(text):
        result[name] = value.strip()
    return result


def parse_command_ids(console_protocol_h: Path) -> dict[str, str]:
    text = read_text(console_protocol_h)
    result: dict[str, str] = {}

    define_pattern = re.compile(
        r"#define\s+(CONSOLE_CMD_[A-Z0-9_]+)\s+(.+?)(?:\r?\n|$)"
    )
    for name, value in define_pattern.findall(text):
        result[name] = value.strip()

    enum_pattern = re.compile(
        r"\b(CONSOLE_CMD_[A-Z0-9_]+)\s*=\s*([^,\r\n}]+)"
    )
    for name, value in enum_pattern.findall(text):
        result[name] = value.strip()

    return result


def parse_param_defaults(params_c: Path) -> dict[str, str]:
    text = read_text(params_c)
    result: dict[str, str] = {}

    for key in PARAM_KEYS:
        line_match = re.search(rf"^.*\b{re.escape(key)}\b.*$", text, re.MULTILINE)
        if not line_match:
            continue

        line = line_match.group(0)
        value_match = re.search(
            r"([-+]?(?:0x[0-9A-Fa-f]+|\d+(?:\.\d+)?))(?:[fFuUlL]*)\b",
            line,
        )
        if value_match:
            result[key] = value_match.group(1)

    return result


def markdown_files(repo_root: Path) -> list[Path]:
    candidates: list[Path] = []

    for name in ("README.md", "README.zh-CN.md", "PATCH_NOTES.md", "AGENTS.md"):
        path = repo_root / name
        if path.exists():
            candidates.append(path)

    docs_dir = repo_root / "docs"
    if docs_dir.exists():
        candidates.extend(sorted(docs_dir.rglob("*.md")))

    return candidates


def check_udp_manual_doc(
    repo_root: Path,
    current_protocol_hex: str,
    current_protocol_dec: int,
) -> list[str]:
    errors: list[str] = []
    path = repo_root / "docs" / "udp_manual_control_protocol.md"
    if not path.exists():
        errors.append("docs/udp_manual_control_protocol.md is missing")
        return errors

    text = read_text(path)

    if current_protocol_hex not in text:
        errors.append(
            f"{path.relative_to(repo_root)}: missing current protocol version "
            f"{current_protocol_hex}"
        )

    stale_hexes = re.findall(
        r"CONSOLE_PROTOCOL_VERSION\s*=\s*(0x[0-9A-Fa-f]+)",
        text,
    )
    for stale_hex in stale_hexes:
        if stale_hex.lower() != current_protocol_hex.lower():
            errors.append(
                f"{path.relative_to(repo_root)}: stale CONSOLE_PROTOCOL_VERSION "
                f"{stale_hex}; current is {current_protocol_hex}"
            )

    stale_decimal = re.findall(r"protocol_version\s*=\s*(\d+)", text)
    for value in stale_decimal:
        if int(value) != current_protocol_dec:
            errors.append(
                f"{path.relative_to(repo_root)}: stale protocol_version={value}; "
                f"current is {current_protocol_dec}"
            )

    required_phrases = (
        "ground reference",
        "ground_ref_valid",
        "stopped",
    )
    lower_text = text.lower()
    for phrase in required_phrases:
        if phrase.lower() not in lower_text:
            errors.append(
                f"{path.relative_to(repo_root)}: UDP manual ground reference "
                f"description should mention {phrase!r}"
            )

    return errors


def check_obsolete_pid_defaults(repo_root: Path) -> list[str]:
    errors: list[str] = []

    for path in markdown_files(repo_root):
        text = read_text(path)
        rel = path.relative_to(repo_root)

        for line_no, line in enumerate(text.splitlines(), start=1):
            if not DEFAULT_CONTEXT_RE.search(line):
                continue

            for value in OBSOLETE_PID_VALUES:
                if value in line:
                    errors.append(
                        f"{rel}:{line_no}: obsolete PID value {value} appears "
                        "in a current/default context"
                    )

    return errors


def check_motor_docs(repo_root: Path) -> list[str]:
    errors: list[str] = []
    candidate_paths = [
        repo_root / "docs" / "motor_map.md",
        repo_root / "docs" / "motor_map.zh-CN.md",
    ]
    existing = [path for path in candidate_paths if path.exists()]
    if not existing:
        errors.append("motor docs are missing: docs/motor_map.md or zh-CN version")
        return errors

    combined = "\n".join(read_text(path) for path in existing).lower()

    required_terms = (
        "8-bit",
        "motor_output_map",
        "scale",
        "offset",
        "deadband",
        "gamma",
    )
    for term in required_terms:
        if term.lower() not in combined:
            errors.append(f"motor docs should mention {term!r}")

    if "resolution parameter" in combined and "not" not in combined:
        errors.append(
            "motor docs may imply PWM resolution is parameterized; current motor.c "
            "uses fixed LEDC_TIMER_8_BIT"
        )

    return errors


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
    )
    args = parser.parse_args(argv)

    repo_root = args.repo_root.resolve()
    console_protocol_h = repo_root / "firmware" / "main" / "console" / "console_protocol.h"
    params_c = repo_root / "firmware" / "main" / "params" / "params.c"

    errors: list[str] = []

    try:
        protocol_hex, protocol_dec = parse_protocol_version(console_protocol_h)
    except Exception as exc:
        errors.append(str(exc))
        protocol_hex, protocol_dec = "UNKNOWN", -1

    try:
        parse_feature_bits(console_protocol_h)
        parse_command_ids(console_protocol_h)
    except Exception as exc:
        errors.append(f"failed to parse console protocol details: {exc}")

    if params_c.exists():
        try:
            parse_param_defaults(params_c)
        except Exception as exc:
            errors.append(f"failed to parse params defaults: {exc}")
    else:
        errors.append(f"missing params file: {params_c}")

    if protocol_dec >= 0:
        errors.extend(check_udp_manual_doc(repo_root, protocol_hex, protocol_dec))

    errors.extend(check_obsolete_pid_defaults(repo_root))
    errors.extend(check_motor_docs(repo_root))

    if errors:
        print("docs-code sync check FAILED:")
        for error in errors:
            print(f"- {error}")
        return 1

    print("docs-code sync check passed")
    print(f"- CONSOLE_PROTOCOL_VERSION = {protocol_hex}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

