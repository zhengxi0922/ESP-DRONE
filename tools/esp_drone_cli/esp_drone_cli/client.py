from __future__ import annotations

from .core.device_session import DeviceSession
from .core.models import ParamValue, TELEMETRY_CSV_FIELDS, TELEMETRY_STRUCT, decode_param_value


class EspDroneClient(DeviceSession):
    def __init__(self, transport) -> None:
        super().__init__(transport=transport)


__all__ = [
    "DeviceSession",
    "EspDroneClient",
    "ParamValue",
    "TELEMETRY_CSV_FIELDS",
    "TELEMETRY_STRUCT",
    "decode_param_value",
]
