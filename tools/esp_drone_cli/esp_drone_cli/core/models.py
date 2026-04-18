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
TELEMETRY_STRUCT_V4 = struct.Struct("<Q" + "f" * 36 + "III8B" + "ffffI4B" + "f" * 9 + "4B" + "f" * 17 + "I8B")
TELEMETRY_STRUCT_V5 = struct.Struct("<Q" + "f" * 36 + "III8B" + "ffffI4B" + "f" * 9 + "4B" + "f" * 17 + "I8B" + "f" * 12 + "4B")
TELEMETRY_STRUCT_CURRENT = TELEMETRY_STRUCT_V5
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
    "battery_voltage", "battery_adc_raw", "battery_valid", "loop_dt_us", "imu_age_us",
    "imu_mode", "imu_health", "arm_state", "failsafe_reason", "control_mode",
    "baro_pressure_pa", "baro_temperature_c", "baro_altitude_m", "baro_vspeed_mps",
    "baro_update_age_us", "baro_valid", "baro_health",
    "attitude_err_roll_deg", "attitude_err_pitch_deg",
    "attitude_rate_sp_roll", "attitude_rate_sp_pitch",
    "angle_target_roll", "angle_target_pitch", "angle_target_yaw",
    "angle_measured_roll", "angle_measured_pitch", "angle_measured_yaw",
    "angle_error_roll", "angle_error_pitch", "angle_error_yaw",
    "outer_loop_rate_target_roll", "outer_loop_rate_target_pitch", "outer_loop_rate_target_yaw",
    "outer_loop_clamp_flag", "inner_loop_clamp_flag", "control_submode",
    "attitude_ref_qw", "attitude_ref_qx", "attitude_ref_qy", "attitude_ref_qz",
    "base_duty_active", "attitude_ref_valid",
    "filtered_gyro_x", "filtered_gyro_y", "filtered_gyro_z",
    "filtered_acc_x", "filtered_acc_y", "filtered_acc_z",
    "kalman_roll_deg", "kalman_pitch_deg",
    "rate_meas_roll_raw", "rate_meas_pitch_raw", "rate_meas_yaw_raw",
    "rate_meas_roll_filtered", "rate_meas_pitch_filtered", "rate_meas_yaw_filtered",
    "rate_err_roll", "rate_err_pitch", "rate_err_yaw",
    "mixer_throttle", "mixer_roll", "mixer_pitch", "mixer_yaw",
    "sample_seq", "attitude_valid", "kalman_valid",
    "motor_saturation_flag", "integrator_freeze_flag",
    "ground_ref_valid", "reference_valid", "ground_trip_reason",
    "battery_v",
    "raw_gyro_x", "raw_gyro_y", "raw_gyro_z",
    "raw_acc_x", "raw_acc_y", "raw_acc_z",
    "raw_roll_deg", "raw_pitch_deg", "raw_yaw_deg",
    "raw_quat_w", "raw_quat_x", "raw_quat_y", "raw_quat_z",
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
FEATURE_GROUND_TUNE = 1 << 7
FEATURE_ATTITUDE_GROUND_VERIFY = 1 << 8
FEATURE_LOW_RISK_LIFTOFF_VERIFY = 1 << 9
MIN_ATTITUDE_HANG_PROTOCOL_VERSION = 3
MIN_UDP_MANUAL_PROTOCOL_VERSION = 5
MIN_GROUND_TUNE_PROTOCOL_VERSION = 6
MIN_ATTITUDE_GROUND_VERIFY_PROTOCOL_VERSION = 8
MIN_LOW_RISK_LIFTOFF_PROTOCOL_VERSION = 8

FEATURE_NAMES = {
    FEATURE_PARAMS: "params",
    FEATURE_STREAMING: "streaming",
    FEATURE_MOTOR_AXIS_TEST: "motor_axis_test",
    FEATURE_RATE_TEST: "rate_test",
    FEATURE_BARO_TELEMETRY: "baro_telemetry",
    FEATURE_ATTITUDE_HANG_BENCH: "attitude_hang_bench",
    FEATURE_UDP_MANUAL_CONTROL: "udp_manual_control",
    FEATURE_GROUND_TUNE: "ground_tune",
    FEATURE_ATTITUDE_GROUND_VERIFY: "attitude_ground_verify",
    FEATURE_LOW_RISK_LIFTOFF_VERIFY: "low_risk_liftoff_verify",
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

    def require_ground_tune(self) -> None:
        if self.protocol_version >= MIN_GROUND_TUNE_PROTOCOL_VERSION and self.supports_feature(FEATURE_GROUND_TUNE):
            return
        raise CapabilityError(
            "device firmware does not advertise flat-ground tune support "
            f"(need protocol_version>={MIN_GROUND_TUNE_PROTOCOL_VERSION} and "
            f"feature ground_tune/0x{FEATURE_GROUND_TUNE:02x}; "
            f"got protocol_version={self.protocol_version}, "
            f"feature_bitmap=0x{self.feature_bitmap:08x}, "
            f"build_git_hash={self.build_git_hash or 'unknown'}, "
            f"build_time_utc={self.build_time_utc or 'unknown'}). "
            "Rebuild and flash the current main firmware before running ground tune commands."
        )

    def require_attitude_ground_verify(self) -> None:
        if (
            self.protocol_version >= MIN_ATTITUDE_GROUND_VERIFY_PROTOCOL_VERSION
            and self.supports_feature(FEATURE_ATTITUDE_GROUND_VERIFY)
        ):
            return
        raise CapabilityError(
            "device firmware does not advertise flat-ground attitude verification support "
            f"(need protocol_version>={MIN_ATTITUDE_GROUND_VERIFY_PROTOCOL_VERSION} and "
            f"feature attitude_ground_verify/0x{FEATURE_ATTITUDE_GROUND_VERIFY:02x}; "
            f"got protocol_version={self.protocol_version}, "
            f"feature_bitmap=0x{self.feature_bitmap:08x}, "
            f"build_git_hash={self.build_git_hash or 'unknown'}, "
            f"build_time_utc={self.build_time_utc or 'unknown'}). "
            "Rebuild and flash the current main firmware before running attitude ground verify commands."
        )

    def require_low_risk_liftoff_verify(self) -> None:
        if (
            self.protocol_version >= MIN_LOW_RISK_LIFTOFF_PROTOCOL_VERSION
            and self.supports_feature(FEATURE_LOW_RISK_LIFTOFF_VERIFY)
        ):
            return
        raise CapabilityError(
            "device firmware does not advertise low-risk liftoff verification support "
            f"(need protocol_version>={MIN_LOW_RISK_LIFTOFF_PROTOCOL_VERSION} and "
            f"feature low_risk_liftoff_verify/0x{FEATURE_LOW_RISK_LIFTOFF_VERIFY:02x}; "
            f"got protocol_version={self.protocol_version}, "
            f"feature_bitmap=0x{self.feature_bitmap:08x}, "
            f"build_git_hash={self.build_git_hash or 'unknown'}, "
            f"build_time_utc={self.build_time_utc or 'unknown'}). "
            "Rebuild and flash the current main firmware before running liftoff verify commands."
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
    battery_valid: int
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
    angle_target_roll: float
    angle_target_pitch: float
    angle_target_yaw: float
    angle_measured_roll: float
    angle_measured_pitch: float
    angle_measured_yaw: float
    angle_error_roll: float
    angle_error_pitch: float
    angle_error_yaw: float
    outer_loop_rate_target_roll: float
    outer_loop_rate_target_pitch: float
    outer_loop_rate_target_yaw: float
    outer_loop_clamp_flag: int
    inner_loop_clamp_flag: int
    control_submode: int
    attitude_ref_qw: float
    attitude_ref_qx: float
    attitude_ref_qy: float
    attitude_ref_qz: float
    base_duty_active: float
    attitude_ref_valid: int
    filtered_gyro_x: float
    filtered_gyro_y: float
    filtered_gyro_z: float
    filtered_acc_x: float
    filtered_acc_y: float
    filtered_acc_z: float
    kalman_roll_deg: float
    kalman_pitch_deg: float
    rate_meas_roll_raw: float
    rate_meas_pitch_raw: float
    rate_meas_yaw_raw: float
    rate_meas_roll_filtered: float
    rate_meas_pitch_filtered: float
    rate_meas_yaw_filtered: float
    rate_err_roll: float
    rate_err_pitch: float
    rate_err_yaw: float
    sample_seq: int
    attitude_valid: int
    kalman_valid: int
    motor_saturation_flag: int
    integrator_freeze_flag: int
    ground_ref_valid: int
    reference_valid: int
    ground_trip_reason: int

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
            battery_valid=1 if int(values[37]) > 0 and float(values[36]) > 0.1 else 0,
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
            angle_target_roll=0.0,
            angle_target_pitch=0.0,
            angle_target_yaw=0.0,
            angle_measured_roll=0.0,
            angle_measured_pitch=0.0,
            angle_measured_yaw=0.0,
            angle_error_roll=0.0,
            angle_error_pitch=0.0,
            angle_error_yaw=0.0,
            outer_loop_rate_target_roll=0.0,
            outer_loop_rate_target_pitch=0.0,
            outer_loop_rate_target_yaw=0.0,
            outer_loop_clamp_flag=0,
            inner_loop_clamp_flag=0,
            control_submode=0,
            attitude_ref_qw=0.0,
            attitude_ref_qx=0.0,
            attitude_ref_qy=0.0,
            attitude_ref_qz=0.0,
            base_duty_active=0.0,
            attitude_ref_valid=0,
            filtered_gyro_x=float(values[1]),
            filtered_gyro_y=float(values[2]),
            filtered_gyro_z=float(values[3]),
            filtered_acc_x=float(values[4]),
            filtered_acc_y=float(values[5]),
            filtered_acc_z=float(values[6]),
            kalman_roll_deg=0.0,
            kalman_pitch_deg=0.0,
            rate_meas_roll_raw=-float(values[2]),
            rate_meas_pitch_raw=float(values[1]),
            rate_meas_yaw_raw=-float(values[3]),
            rate_meas_roll_filtered=-float(values[2]),
            rate_meas_pitch_filtered=float(values[1]),
            rate_meas_yaw_filtered=-float(values[3]),
            rate_err_roll=0.0,
            rate_err_pitch=0.0,
            rate_err_yaw=0.0,
            sample_seq=0,
            attitude_valid=1 if (abs(float(values[7])) + abs(float(values[8])) + abs(float(values[9])) + abs(float(values[10]))) > 0.0 else 0,
            kalman_valid=0,
            motor_saturation_flag=0,
            integrator_freeze_flag=0,
            ground_ref_valid=0,
            reference_valid=0,
            ground_trip_reason=0,
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
        sample.angle_measured_roll = sample.attitude_err_roll_deg
        sample.angle_measured_pitch = sample.attitude_err_pitch_deg
        sample.angle_error_roll = -sample.attitude_err_roll_deg
        sample.angle_error_pitch = -sample.attitude_err_pitch_deg
        sample.outer_loop_rate_target_roll = sample.attitude_rate_sp_roll
        sample.outer_loop_rate_target_pitch = sample.attitude_rate_sp_pitch
        sample.attitude_ref_qw = float(values[61])
        sample.attitude_ref_qx = float(values[62])
        sample.attitude_ref_qy = float(values[63])
        sample.attitude_ref_qz = float(values[64])
        sample.base_duty_active = float(values[65])
        sample.attitude_ref_valid = int(values[66])
        return sample

    @classmethod
    def _from_v4_values(cls, values: list[object]) -> "TelemetrySample":
        sample = cls._from_v3_values(values[:67])
        v4_offset = 70
        sample.filtered_gyro_x = float(values[v4_offset + 0])
        sample.filtered_gyro_y = float(values[v4_offset + 1])
        sample.filtered_gyro_z = float(values[v4_offset + 2])
        sample.filtered_acc_x = float(values[v4_offset + 3])
        sample.filtered_acc_y = float(values[v4_offset + 4])
        sample.filtered_acc_z = float(values[v4_offset + 5])
        sample.kalman_roll_deg = float(values[v4_offset + 6])
        sample.kalman_pitch_deg = float(values[v4_offset + 7])
        sample.rate_meas_roll_raw = float(values[v4_offset + 8])
        sample.rate_meas_pitch_raw = float(values[v4_offset + 9])
        sample.rate_meas_yaw_raw = float(values[v4_offset + 10])
        sample.rate_meas_roll_filtered = float(values[v4_offset + 11])
        sample.rate_meas_pitch_filtered = float(values[v4_offset + 12])
        sample.rate_meas_yaw_filtered = float(values[v4_offset + 13])
        sample.rate_err_roll = float(values[v4_offset + 14])
        sample.rate_err_pitch = float(values[v4_offset + 15])
        sample.rate_err_yaw = float(values[v4_offset + 16])
        sample.sample_seq = int(values[v4_offset + 17])
        sample.attitude_valid = int(values[v4_offset + 18])
        sample.kalman_valid = int(values[v4_offset + 19])
        sample.motor_saturation_flag = int(values[v4_offset + 20])
        sample.integrator_freeze_flag = int(values[v4_offset + 21])
        sample.ground_ref_valid = int(values[v4_offset + 22])
        sample.reference_valid = int(values[v4_offset + 23])
        sample.ground_trip_reason = int(values[v4_offset + 24])
        sample.battery_valid = int(values[v4_offset + 25])
        return sample

    @classmethod
    def _from_v5_values(cls, values: list[object]) -> "TelemetrySample":
        sample = cls._from_v4_values(values[:96])
        v5_offset = 96
        sample.angle_target_roll = float(values[v5_offset + 0])
        sample.angle_target_pitch = float(values[v5_offset + 1])
        sample.angle_target_yaw = float(values[v5_offset + 2])
        sample.angle_measured_roll = float(values[v5_offset + 3])
        sample.angle_measured_pitch = float(values[v5_offset + 4])
        sample.angle_measured_yaw = float(values[v5_offset + 5])
        sample.angle_error_roll = float(values[v5_offset + 6])
        sample.angle_error_pitch = float(values[v5_offset + 7])
        sample.angle_error_yaw = float(values[v5_offset + 8])
        sample.outer_loop_rate_target_roll = float(values[v5_offset + 9])
        sample.outer_loop_rate_target_pitch = float(values[v5_offset + 10])
        sample.outer_loop_rate_target_yaw = float(values[v5_offset + 11])
        sample.outer_loop_clamp_flag = int(values[v5_offset + 12])
        sample.inner_loop_clamp_flag = int(values[v5_offset + 13])
        sample.control_submode = int(values[v5_offset + 14])
        return sample

    @classmethod
    def from_payload(cls, payload: bytes) -> "TelemetrySample":
        if len(payload) == TELEMETRY_STRUCT_V5.size:
            return cls._from_v5_values(list(TELEMETRY_STRUCT_V5.unpack(payload)))
        if len(payload) == TELEMETRY_STRUCT_V4.size:
            return cls._from_v4_values(list(TELEMETRY_STRUCT_V4.unpack(payload)))
        if len(payload) == TELEMETRY_STRUCT_V3.size:
            return cls._from_v3_values(list(TELEMETRY_STRUCT_V3.unpack(payload)))
        if len(payload) == TELEMETRY_STRUCT_V2.size:
            return cls._from_v2_values(list(TELEMETRY_STRUCT_V2.unpack(payload)))
        if len(payload) == TELEMETRY_STRUCT_V1.size:
            return cls._from_v1_values(list(TELEMETRY_STRUCT_V1.unpack(payload)))
        raise ValueError(
            f"unsupported telemetry payload length {len(payload)}, "
            f"expected {TELEMETRY_STRUCT_V1.size}, {TELEMETRY_STRUCT_V2.size}, "
            f"{TELEMETRY_STRUCT_V3.size}, {TELEMETRY_STRUCT_V4.size}, or {TELEMETRY_STRUCT_V5.size}"
        )

    def to_csv_row(self) -> list[object]:
        display = self.to_display_map()
        return [display[name] for name in TELEMETRY_CSV_FIELDS]

    def to_display_map(self) -> dict[str, object]:
        data = {field: getattr(self, field) for field in self.__dataclass_fields__}
        data.update(
            {
                "battery_v": self.battery_voltage,
                "raw_gyro_x": self.gyro_x,
                "raw_gyro_y": self.gyro_y,
                "raw_gyro_z": self.gyro_z,
                "raw_acc_x": self.acc_x,
                "raw_acc_y": self.acc_y,
                "raw_acc_z": self.acc_z,
                "raw_roll_deg": self.roll_deg,
                "raw_pitch_deg": self.pitch_deg,
                "raw_yaw_deg": self.yaw_deg,
                "raw_quat_w": self.quat_w,
                "raw_quat_x": self.quat_x,
                "raw_quat_y": self.quat_y,
                "raw_quat_z": self.quat_z,
                "mixer_throttle": self.base_duty_active,
                "mixer_roll": self.pid_out_roll,
                "mixer_pitch": self.pid_out_pitch,
                "mixer_yaw": self.pid_out_yaw,
            }
        )
        return data


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
