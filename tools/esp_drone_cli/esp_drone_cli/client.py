"""旧版 `esp_drone_cli.client` 导入路径的兼容层。

该模块不再维护独立的协议、传输或命令逻辑，只转发到
`esp_drone_cli.core.device_session.DeviceSession`。
"""

from __future__ import annotations

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
