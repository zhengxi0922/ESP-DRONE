# ============================================================
# @file client.py
# @brief ESP-DRONE ???????
# @details ????????????????????? DeviceSession ???????
# @author Codex
# @date 2026-04-05
# @version 1.0
# ============================================================

from __future__ import annotations

"""Compatibility shim for legacy imports.

The session / command owner is now ``esp_drone_cli.core.device_session.DeviceSession``.
This module intentionally keeps no independent protocol, transport, or command logic.
"""

from .core.device_session import DeviceSession
from .core.models import ParamValue, TELEMETRY_CSV_FIELDS, TELEMETRY_STRUCT, decode_param_value


EspDroneClient = DeviceSession


__all__ = [
    "DeviceSession",
    "EspDroneClient",
    "ParamValue",
    "TELEMETRY_CSV_FIELDS",
    "TELEMETRY_STRUCT",
    "decode_param_value",
]
