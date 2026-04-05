# ============================================================
# @file models.py
# @brief ESP-DRONE Python 数据模型
# @details 统一定义 DeviceSession / CLI / GUI 共用的 telemetry、参数快照和设备信息结构。
# @author Codex
# @date 2026-04-05
# @version 1.0
# ============================================================

from __future__ import annotations

import json
import struct
from dataclasses import asdict, dataclass
from pathlib import Path


CMD_REQ_STRUCT = struct.Struct("<BBHf")
CMD_RESP_STRUCT = struct.Struct("<BBH")
HELLO_RESP_STRUCT = struct.Struct("<BBBBI")
TELEMETRY_STRUCT_V1 = struct.Struct("<Q" + "f" * 36 + "III8B")
TELEMETRY_STRUCT_V2 = struct.Struct("<Q" + "f" * 36 + "III8B" + "ffffI4B")
TELEMETRY_STRUCT = TELEMETRY_STRUCT_V2

TELEMETRY_CSV_FIELDS = [
    "timestamp_us",
    "gyro_x", "gyro_y", "gyro_z",
    "acc_x", "acc_y", "acc_z",
    "quat_w", "quat_x", "quat_y", "quat_z",
    "roll_deg", "pitch_deg", "yaw_deg",
    "setpoint_roll", "setpoint_pitch", "setpoint_yaw",
    "rate_setpoint_roll", "rate_setpoint_pitch", "rate_setpoint_yaw",
    "rate_pid_p_roll", "rate_pid_p_pitch", "rate_pid_p_yaw",
    "rate_pid_i_roll", "rate_pid_i_pitch", "rate_pid_i_yaw",
    "rate_pid_d_roll", "rate_pid_d_pitch", "rate_pid_d_yaw",
    "pid_out_roll", "pid_out_pitch", "pid_out_yaw",
    "motor1", "motor2", "motor3", "motor4",
    "battery_voltage", "battery_adc_raw", "loop_dt_us", "imu_age_us",
    "imu_mode", "imu_health", "arm_state", "failsafe_reason", "control_mode",
    "baro_pressure_pa", "baro_temperature_c", "baro_altitude_m", "baro_vspeed_mps",
    "baro_update_age_us", "baro_valid", "baro_health",
]

"""这里定义的是 Python 工具链共享的数据模型。

CLI、GUI、CSV 导出都以这些结构为准，避免同一份 telemetry 在不同界面里各自维护字段表。
"""


@dataclass(slots=True)
class ParamValue:
    name: str
    type_id: int
    value: object


@dataclass(slots=True)
class DeviceInfo:
    protocol_version: int
    imu_mode: int
    arm_state: int
    stream_enabled: int
    feature_bitmap: int


@dataclass(slots=True)
class ParamSnapshot:
    schema: int
    firmware: dict[str, object]
    params: list[dict[str, object]]

    def write_json(self, path: Path) -> None:
        path.write_text(json.dumps(asdict(self), indent=2, ensure_ascii=True) + "\n", encoding="utf-8")


@dataclass(slots=True)
class TelemetrySample:
    timestamp_us: int
    gyro_x: float
    gyro_y: float
    gyro_z: float
    acc_x: float
    acc_y: float
    acc_z: float
    quat_w: float
    quat_x: float
    quat_y: float
    quat_z: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    setpoint_roll: float
    setpoint_pitch: float
    setpoint_yaw: float
    rate_setpoint_roll: float
    rate_setpoint_pitch: float
    rate_setpoint_yaw: float
    rate_pid_p_roll: float
    rate_pid_p_pitch: float
    rate_pid_p_yaw: float
    rate_pid_i_roll: float
    rate_pid_i_pitch: float
    rate_pid_i_yaw: float
    rate_pid_d_roll: float
    rate_pid_d_pitch: float
    rate_pid_d_yaw: float
    pid_out_roll: float
    pid_out_pitch: float
    pid_out_yaw: float
    motor1: float
    motor2: float
    motor3: float
    motor4: float
    battery_voltage: float
    battery_adc_raw: int
    loop_dt_us: int
    imu_age_us: int
    imu_mode: int
    imu_health: int
    arm_state: int
    failsafe_reason: int
    control_mode: int
    baro_pressure_pa: float
    baro_temperature_c: float
    baro_altitude_m: float
    baro_vspeed_mps: float
    baro_update_age_us: int
    baro_valid: int
    baro_health: int

    @classmethod
    def from_payload(cls, payload: bytes) -> "TelemetrySample":
        """统一解析 telemetry。

        当前仓库同时兼容：
        - V1: 不带气压计字段的旧 telemetry 结构
        - V2: 在旧结构尾部追加 barometer 字段的新结构
        """

        if len(payload) == TELEMETRY_STRUCT_V2.size:
            values = list(TELEMETRY_STRUCT_V2.unpack(payload))
            trimmed = values[:45] + values[48:55]
            return cls(*trimmed)

        if len(payload) == TELEMETRY_STRUCT_V1.size:
            values = list(TELEMETRY_STRUCT_V1.unpack(payload))
            trimmed = values[:45]
            trimmed.extend([
                0.0,  # baro_pressure_pa
                0.0,  # baro_temperature_c
                0.0,  # baro_altitude_m
                0.0,  # baro_vspeed_mps
                0,    # baro_update_age_us
                0,    # baro_valid
                0,    # baro_health
            ])
            return cls(*trimmed)

        raise ValueError(
            f"unsupported telemetry payload length {len(payload)}, "
            f"expected {TELEMETRY_STRUCT_V1.size} or {TELEMETRY_STRUCT_V2.size}"
        )

    def to_csv_row(self) -> list[object]:
        return [getattr(self, name) for name in TELEMETRY_CSV_FIELDS]

    def to_display_map(self) -> dict[str, object]:
        return {name: getattr(self, name) for name in TELEMETRY_CSV_FIELDS}


def decode_param_value(payload: bytes) -> ParamValue:
    type_id = payload[0]
    name_len = payload[1]
    name = payload[2 : 2 + name_len].decode("ascii")
    value_bytes = payload[2 + name_len :]
    if type_id == 0:
        value = value_bytes[0] != 0
    elif type_id == 1:
        value = value_bytes[0]
    elif type_id == 2:
        value = struct.unpack("<I", value_bytes)[0]
    elif type_id == 3:
        value = struct.unpack("<i", value_bytes)[0]
    elif type_id == 4:
        value = struct.unpack("<f", value_bytes)[0]
    else:
        value = value_bytes
    return ParamValue(name=name, type_id=type_id, value=value)


def decode_device_info(payload: bytes) -> DeviceInfo:
    protocol_version, imu_mode, arm_state, stream_enabled, feature_bitmap = HELLO_RESP_STRUCT.unpack(payload)
    return DeviceInfo(protocol_version, imu_mode, arm_state, stream_enabled, feature_bitmap)


def decode_event_text(payload: bytes) -> str:
    if len(payload) < 2:
        return ""
    text_len = payload[1]
    return payload[2 : 2 + text_len].decode("utf-8", errors="replace")


def encode_param_value(type_id: int, value_text: str) -> bytes:
    if type_id == 0:
        return bytes([1 if value_text.lower() in {"1", "true", "yes", "on"} else 0])
    if type_id == 1:
        return struct.pack("<B", int(value_text))
    if type_id == 2:
        return struct.pack("<I", int(value_text))
    if type_id == 3:
        return struct.pack("<i", int(value_text))
    if type_id == 4:
        return struct.pack("<f", float(value_text))
    raise ValueError(f"unsupported param type {type_id}")
