"""CLI、GUI 与脚本共享的数据模型。"""

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

RATE_AXIS_SOURCES = {
    "roll": ("gyro_y", -1.0),
    "pitch": ("gyro_x", 1.0),
    "yaw": ("gyro_z", -1.0),
}


@dataclass(slots=True)
class ParamValue:
    """参数项的名称、类型和值。"""

    name: str
    type_id: int
    value: object


@dataclass(slots=True)
class DeviceInfo:
    """握手阶段返回的设备摘要信息。"""

    protocol_version: int
    imu_mode: int
    arm_state: int
    stream_enabled: int
    feature_bitmap: int


@dataclass(slots=True)
class ParamSnapshot:
    """参数导出快照。

    Attributes:
        schema: 导出文件格式版本。
        firmware: 设备侧固件摘要信息。
        params: 参数列表，每一项包含名称、类型和值。
    """

    schema: int
    firmware: dict[str, object]
    params: list[dict[str, object]]

    def write_json(self, path: Path) -> None:
        """将快照写入 JSON 文件。

        Args:
            path: 输出文件路径。父目录需要已存在。

        Returns:
            None.

        Raises:
            OSError: 目标文件不可写时抛出。
            TypeError: 快照中包含不可序列化对象时抛出。
        """

        path.write_text(json.dumps(asdict(self), indent=2, ensure_ascii=True) + "\n", encoding="utf-8")


@dataclass(slots=True)
class TelemetrySample:
    """一帧遥测样本的结构化表示。"""

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

    def axis_rate_feedback_dps(self, axis_name: str) -> float:
        """Return the project-defined rate feedback for one axis."""

        try:
            source_field, sign = RATE_AXIS_SOURCES[axis_name]
        except KeyError as exc:
            raise ValueError(f"unsupported axis {axis_name}") from exc
        return float(getattr(self, source_field)) * sign

    def axis_rate_source(self, axis_name: str) -> tuple[str, float]:
        """Return the raw gyro field name and value used by one rate axis."""

        try:
            source_field, _sign = RATE_AXIS_SOURCES[axis_name]
        except KeyError as exc:
            raise ValueError(f"unsupported axis {axis_name}") from exc
        return source_field, float(getattr(self, source_field))

    def axis_rate_debug_map(self, axis_name: str) -> dict[str, object]:
        """Return the shared rate-debug view used by CLI and GUI."""

        if axis_name not in RATE_AXIS_SOURCES:
            raise ValueError(f"unsupported axis {axis_name}")

        source_field, sign = RATE_AXIS_SOURCES[axis_name]
        source_value = float(getattr(self, source_field))
        return {
            "axis": axis_name,
            "source_field": source_field,
            "source_value": source_value,
            "feedback_field": f"{axis_name}_rate",
            "feedback_expr": f"{'-' if sign < 0.0 else ''}{source_field}",
            "setpoint_field": f"rate_setpoint_{axis_name}",
            "pid_p_field": f"rate_pid_p_{axis_name}",
            "pid_i_field": f"rate_pid_i_{axis_name}",
            "pid_d_field": f"rate_pid_d_{axis_name}",
            "pid_out_field": f"pid_out_{axis_name}",
            "feedback_dps": self.axis_rate_feedback_dps(axis_name),
            "setpoint_dps": float(getattr(self, f"rate_setpoint_{axis_name}")),
            "pid_p": float(getattr(self, f"rate_pid_p_{axis_name}")),
            "pid_i": float(getattr(self, f"rate_pid_i_{axis_name}")),
            "pid_d": float(getattr(self, f"rate_pid_d_{axis_name}")),
            "pid_out": float(getattr(self, f"pid_out_{axis_name}")),
            "motor_outputs": (self.motor1, self.motor2, self.motor3, self.motor4),
            "arm_state": self.arm_state,
            "control_mode": self.control_mode,
            "imu_age_us": self.imu_age_us,
            "loop_dt_us": self.loop_dt_us,
        }

    @classmethod
    def from_payload(cls, payload: bytes) -> "TelemetrySample":
        """从设备 payload 解析遥测样本。

        Args:
            payload: 设备协议返回的原始遥测负载。

        Returns:
            解析后的遥测样本对象。

        Raises:
            ValueError: 负载长度既不匹配 V1 也不匹配 V2 结构时抛出。

        注意:
            当前仓库同时兼容旧版无气压计字段的 V1 结构和追加气压计字段的 V2 结构。
        """

        if len(payload) == TELEMETRY_STRUCT_V2.size:
            values = list(TELEMETRY_STRUCT_V2.unpack(payload))
            trimmed = values[:45] + values[48:55]
            return cls(*trimmed)

        if len(payload) == TELEMETRY_STRUCT_V1.size:
            values = list(TELEMETRY_STRUCT_V1.unpack(payload))
            trimmed = values[:45]
            trimmed.extend([
                0.0,
                0.0,
                0.0,
                0.0,
                0,
                0,
                0,
            ])
            return cls(*trimmed)

        raise ValueError(
            f"unsupported telemetry payload length {len(payload)}, "
            f"expected {TELEMETRY_STRUCT_V1.size} or {TELEMETRY_STRUCT_V2.size}"
        )

    def to_csv_row(self) -> list[object]:
        """按 CSV 列顺序导出当前样本。

        Returns:
            与 `TELEMETRY_CSV_FIELDS` 一一对应的值列表。
        """

        return [getattr(self, name) for name in TELEMETRY_CSV_FIELDS]

    def to_display_map(self) -> dict[str, object]:
        """将样本转换为便于界面展示的字典。

        Returns:
            以字段名为键、字段值为值的映射。
        """

        return {name: getattr(self, name) for name in TELEMETRY_CSV_FIELDS}


def decode_param_value(payload: bytes) -> ParamValue:
    """解析参数值响应负载。

    Args:
        payload: 设备返回的 `MSG_PARAM_VALUE` 负载。

    Returns:
        解析后的参数对象。

    Raises:
        IndexError: 负载过短且缺少类型或名称长度字段时抛出。
        struct.error: 数值字段长度不足时抛出。
    """

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
    """解析握手响应负载。

    Args:
        payload: 设备返回的 `MSG_HELLO_RESP` 负载。

    Returns:
        设备摘要信息。

    Raises:
        struct.error: 负载长度与握手结构不匹配时抛出。
    """

    protocol_version, imu_mode, arm_state, stream_enabled, feature_bitmap = HELLO_RESP_STRUCT.unpack(payload)
    return DeviceInfo(protocol_version, imu_mode, arm_state, stream_enabled, feature_bitmap)


def decode_event_text(payload: bytes) -> str:
    """解析文本事件负载。

    Args:
        payload: 设备返回的 `MSG_EVENT_LOG_TEXT` 负载。

    Returns:
        UTF-8 文本内容；负载过短时返回空字符串。
    """

    if len(payload) < 2:
        return ""
    text_len = payload[1]
    return payload[2 : 2 + text_len].decode("utf-8", errors="replace")


def coerce_param_value(type_id: int, value: object) -> object:
    """Coerce a user-provided value into the protocol scalar type."""

    if type_id == 0:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in {"1", "true", "yes", "on"}
        return bool(value)
    if type_id == 1:
        return int(value)
    if type_id == 2:
        return int(value)
    if type_id == 3:
        return int(value)
    if type_id == 4:
        return float(value)
    raise ValueError(f"unsupported param type {type_id}")


def encode_param_value(type_id: int, value_text: str) -> bytes:
    """将文本参数值编码为协议字节串。

    Args:
        type_id: 参数类型编号。
        value_text: 来自 CLI 或导入文件的文本值。

    Returns:
        可直接写入 `MSG_PARAM_SET` 的参数值字节串。

    Raises:
        ValueError: 参数类型不受支持，或文本无法转换为目标数值类型时抛出。
    """

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
