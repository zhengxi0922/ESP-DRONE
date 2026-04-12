from __future__ import annotations

import json
import struct
from dataclasses import asdict, dataclass
from pathlib import Path


CMD_REQ_STRUCT = struct.Struct("<BBHf")
CMD_RESP_STRUCT = struct.Struct("<BBH")
UDP_MANUAL_SETPOINT_STRUCT = struct.Struct("<ffff")
HELLO_RESP_STRUCT = struct.Struct("<BBBBI")
HELLO_RESP_STRUCT_V2 = struct.Struct("<BBBBI16s24s")
TELEMETRY_STRUCT_V1 = struct.Struct("<Q" + "f" * 36 + "III8B")
TELEMETRY_STRUCT_V2 = struct.Struct("<Q" + "f" * 36 + "III8B" + "ffffI4B")
TELEMETRY_STRUCT_V3 = struct.Struct("<Q" + "f" * 36 + "III8B" + "ffffI4B" + "f" * 9 + "4B")
TELEMETRY_STRUCT = TELEMETRY_STRUCT_V3

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
    "attitude_err_roll_deg", "attitude_err_pitch_deg",
    "attitude_rate_sp_roll", "attitude_rate_sp_pitch",
    "attitude_ref_qw", "attitude_ref_qx", "attitude_ref_qy", "attitude_ref_qz",
    "base_duty_active", "attitude_ref_valid",
]

RATE_AXIS_SOURCES = {
    "roll": ("gyro_y", -1.0),
    "pitch": ("gyro_x", 1.0),
    "yaw": ("gyro_z", -1.0),
}

ATTITUDE_AXIS_FIELDS = {
    "roll": ("attitude_err_roll_deg", "attitude_rate_sp_roll", "pid_out_roll"),
    "pitch": ("attitude_err_pitch_deg", "attitude_rate_sp_pitch", "pid_out_pitch"),
}

FEATURE_PARAMS = 1 << 0
FEATURE_STREAMING = 1 << 1
FEATURE_MOTOR_AXIS_TEST = 1 << 2
FEATURE_RATE_TEST = 1 << 3
FEATURE_BARO_TELEMETRY = 1 << 4
FEATURE_ATTITUDE_HANG_BENCH = 1 << 5
FEATURE_UDP_MANUAL_CONTROL = 1 << 6
MIN_ATTITUDE_HANG_PROTOCOL_VERSION = 3
MIN_UDP_MANUAL_PROTOCOL_VERSION = 5

FEATURE_NAMES = {
    FEATURE_PARAMS: "params",
    FEATURE_STREAMING: "streaming",
    FEATURE_MOTOR_AXIS_TEST: "motor_axis_test",
    FEATURE_RATE_TEST: "rate_test",
    FEATURE_BARO_TELEMETRY: "baro_telemetry",
    FEATURE_ATTITUDE_HANG_BENCH: "attitude_hang_bench",
    FEATURE_UDP_MANUAL_CONTROL: "udp_manual_control",
}


class CapabilityError(RuntimeError):
    pass


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
    build_git_hash: str | None = None
    build_time_utc: str | None = None

    def supports_feature(self, feature_bit: int) -> bool:
        return (self.feature_bitmap & feature_bit) == feature_bit

    def feature_names(self) -> list[str]:
        return [name for bit, name in FEATURE_NAMES.items() if self.supports_feature(bit)]

    def require_attitude_hang_bench(self) -> None:
        if self.protocol_version >= MIN_ATTITUDE_HANG_PROTOCOL_VERSION and self.supports_feature(FEATURE_ATTITUDE_HANG_BENCH):
            return
        raise CapabilityError(
            "device firmware does not advertise bench-only hang-attitude support "
            f"(need protocol_version>={MIN_ATTITUDE_HANG_PROTOCOL_VERSION} and "
            f"feature attitude_hang_bench/0x{FEATURE_ATTITUDE_HANG_BENCH:02x}; "
            f"got protocol_version={self.protocol_version}, "
            f"feature_bitmap=0x{self.feature_bitmap:08x}, "
            f"build_git_hash={self.build_git_hash or 'unknown'}, "
            f"build_time_utc={self.build_time_utc or 'unknown'}). "
            "Rebuild and flash the current main firmware before running hang-attitude commands."
        )

    def require_udp_manual_control(self) -> None:
        if self.protocol_version >= MIN_UDP_MANUAL_PROTOCOL_VERSION and self.supports_feature(FEATURE_UDP_MANUAL_CONTROL):
            return
        raise CapabilityError(
            "device firmware does not advertise experimental UDP manual control support "
            f"(need protocol_version>={MIN_UDP_MANUAL_PROTOCOL_VERSION} and "
            f"feature udp_manual_control/0x{FEATURE_UDP_MANUAL_CONTROL:02x}; "
            f"got protocol_version={self.protocol_version}, "
            f"feature_bitmap=0x{self.feature_bitmap:08x}, "
            f"build_git_hash={self.build_git_hash or 'unknown'}, "
            f"build_time_utc={self.build_time_utc or 'unknown'}). "
            "Rebuild and flash the current main firmware before using UDP Control."
        )


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
    attitude_err_roll_deg: float
    attitude_err_pitch_deg: float
    attitude_rate_sp_roll: float
    attitude_rate_sp_pitch: float
    attitude_ref_qw: float
    attitude_ref_qx: float
    attitude_ref_qy: float
    attitude_ref_qz: float
    base_duty_active: float
    attitude_ref_valid: int

    def axis_rate_feedback_dps(self, axis_name: str) -> float:
        try:
            source_field, sign = RATE_AXIS_SOURCES[axis_name]
        except KeyError as exc:
            raise ValueError(f"unsupported axis {axis_name}") from exc
        return float(getattr(self, source_field)) * sign

    def axis_rate_source(self, axis_name: str) -> tuple[str, float]:
        try:
            source_field, _sign = RATE_AXIS_SOURCES[axis_name]
        except KeyError as exc:
            raise ValueError(f"unsupported axis {axis_name}") from exc
        return source_field, float(getattr(self, source_field))

    def axis_rate_debug_map(self, axis_name: str) -> dict[str, object]:
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
            "base_duty_active": self.base_duty_active,
        }

    def axis_attitude_debug_map(self, axis_name: str) -> dict[str, object]:
        try:
            error_field, rate_sp_field, pid_out_field = ATTITUDE_AXIS_FIELDS[axis_name]
        except KeyError as exc:
            raise ValueError(f"unsupported axis {axis_name}") from exc

        return {
            "axis": axis_name,
            "error_deg": float(getattr(self, error_field)),
            "rate_sp_dps": float(getattr(self, rate_sp_field)),
            "pid_out": float(getattr(self, pid_out_field)),
            "motor_outputs": (self.motor1, self.motor2, self.motor3, self.motor4),
            "arm_state": self.arm_state,
            "control_mode": self.control_mode,
            "imu_age_us": self.imu_age_us,
            "loop_dt_us": self.loop_dt_us,
            "base_duty_active": self.base_duty_active,
            "attitude_ref_valid": int(self.attitude_ref_valid),
            "attitude_ref_q": (
                self.attitude_ref_qw,
                self.attitude_ref_qx,
                self.attitude_ref_qy,
                self.attitude_ref_qz,
            ),
        }

    @classmethod
    def _from_v1_values(cls, values: list[object]) -> "TelemetrySample":
        return cls(
            timestamp_us=int(values[0]),
            gyro_x=float(values[1]),
            gyro_y=float(values[2]),
            gyro_z=float(values[3]),
            acc_x=float(values[4]),
            acc_y=float(values[5]),
            acc_z=float(values[6]),
            quat_w=float(values[7]),
            quat_x=float(values[8]),
            quat_y=float(values[9]),
            quat_z=float(values[10]),
            roll_deg=float(values[11]),
            pitch_deg=float(values[12]),
            yaw_deg=float(values[13]),
            setpoint_roll=float(values[14]),
            setpoint_pitch=float(values[15]),
            setpoint_yaw=float(values[16]),
            rate_setpoint_roll=float(values[17]),
            rate_setpoint_pitch=float(values[18]),
            rate_setpoint_yaw=float(values[19]),
            rate_pid_p_roll=float(values[20]),
            rate_pid_p_pitch=float(values[21]),
            rate_pid_p_yaw=float(values[22]),
            rate_pid_i_roll=float(values[23]),
            rate_pid_i_pitch=float(values[24]),
            rate_pid_i_yaw=float(values[25]),
            rate_pid_d_roll=float(values[26]),
            rate_pid_d_pitch=float(values[27]),
            rate_pid_d_yaw=float(values[28]),
            pid_out_roll=float(values[29]),
            pid_out_pitch=float(values[30]),
            pid_out_yaw=float(values[31]),
            motor1=float(values[32]),
            motor2=float(values[33]),
            motor3=float(values[34]),
            motor4=float(values[35]),
            battery_voltage=float(values[36]),
            battery_adc_raw=int(values[37]),
            loop_dt_us=int(values[38]),
            imu_age_us=int(values[39]),
            imu_mode=int(values[40]),
            imu_health=int(values[41]),
            arm_state=int(values[42]),
            failsafe_reason=int(values[43]),
            control_mode=int(values[44]),
            baro_pressure_pa=0.0,
            baro_temperature_c=0.0,
            baro_altitude_m=0.0,
            baro_vspeed_mps=0.0,
            baro_update_age_us=0,
            baro_valid=0,
            baro_health=0,
            attitude_err_roll_deg=0.0,
            attitude_err_pitch_deg=0.0,
            attitude_rate_sp_roll=0.0,
            attitude_rate_sp_pitch=0.0,
            attitude_ref_qw=0.0,
            attitude_ref_qx=0.0,
            attitude_ref_qy=0.0,
            attitude_ref_qz=0.0,
            base_duty_active=0.0,
            attitude_ref_valid=0,
        )

    @classmethod
    def _from_v2_values(cls, values: list[object]) -> "TelemetrySample":
        sample = cls._from_v1_values(values[:45])
        sample.baro_pressure_pa = float(values[48])
        sample.baro_temperature_c = float(values[49])
        sample.baro_altitude_m = float(values[50])
        sample.baro_vspeed_mps = float(values[51])
        sample.baro_update_age_us = int(values[52])
        sample.baro_valid = int(values[53])
        sample.baro_health = int(values[54])
        return sample

    @classmethod
    def _from_v3_values(cls, values: list[object]) -> "TelemetrySample":
        sample = cls._from_v2_values(values[:57])
        sample.attitude_err_roll_deg = float(values[57])
        sample.attitude_err_pitch_deg = float(values[58])
        sample.attitude_rate_sp_roll = float(values[59])
        sample.attitude_rate_sp_pitch = float(values[60])
        sample.attitude_ref_qw = float(values[61])
        sample.attitude_ref_qx = float(values[62])
        sample.attitude_ref_qy = float(values[63])
        sample.attitude_ref_qz = float(values[64])
        sample.base_duty_active = float(values[65])
        sample.attitude_ref_valid = int(values[66])
        return sample

    @classmethod
    def from_payload(cls, payload: bytes) -> "TelemetrySample":
        if len(payload) == TELEMETRY_STRUCT_V3.size:
            return cls._from_v3_values(list(TELEMETRY_STRUCT_V3.unpack(payload)))
        if len(payload) == TELEMETRY_STRUCT_V2.size:
            return cls._from_v2_values(list(TELEMETRY_STRUCT_V2.unpack(payload)))
        if len(payload) == TELEMETRY_STRUCT_V1.size:
            return cls._from_v1_values(list(TELEMETRY_STRUCT_V1.unpack(payload)))
        raise ValueError(
            f"unsupported telemetry payload length {len(payload)}, "
            f"expected {TELEMETRY_STRUCT_V1.size}, {TELEMETRY_STRUCT_V2.size}, or {TELEMETRY_STRUCT_V3.size}"
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
    if len(payload) == HELLO_RESP_STRUCT.size:
        protocol_version, imu_mode, arm_state, stream_enabled, feature_bitmap = HELLO_RESP_STRUCT.unpack(payload)
        return DeviceInfo(protocol_version, imu_mode, arm_state, stream_enabled, feature_bitmap)
    if len(payload) == HELLO_RESP_STRUCT_V2.size:
        (
            protocol_version,
            imu_mode,
            arm_state,
            stream_enabled,
            feature_bitmap,
            build_git_hash,
            build_time_utc,
        ) = HELLO_RESP_STRUCT_V2.unpack(payload)
        return DeviceInfo(
            protocol_version,
            imu_mode,
            arm_state,
            stream_enabled,
            feature_bitmap,
            build_git_hash.split(b"\x00", 1)[0].decode("ascii", errors="replace") or None,
            build_time_utc.split(b"\x00", 1)[0].decode("ascii", errors="replace") or None,
        )
    raise ValueError(
        f"unsupported HELLO_RESP payload length {len(payload)}, "
        f"expected {HELLO_RESP_STRUCT.size} or {HELLO_RESP_STRUCT_V2.size}"
    )


def decode_event_text(payload: bytes) -> str:
    if len(payload) < 2:
        return ""
    text_len = payload[1]
    return payload[2 : 2 + text_len].decode("utf-8", errors="replace")


def coerce_param_value(type_id: int, value: object) -> object:
    if type_id == 0:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in {"1", "true", "yes", "on"}
        return bool(value)
    if type_id in {1, 2, 3}:
        return int(value)
    if type_id == 4:
        return float(value)
    raise ValueError(f"unsupported param type {type_id}")


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
