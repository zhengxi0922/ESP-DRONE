from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import queue
import threading
import time
from typing import Any, Callable

from .device_session import DeviceSession
from .models import TELEMETRY_CSV_FIELDS, ParamValue, TelemetrySample
from .protocol.messages import CmdId, ensure_command_ok


CONTROL_MODE_ATTITUDE_GROUND_TUNE = 6
GROUND_TUNE_SUBMODE_ATTITUDE_VERIFY = 1
ARM_STATE_ARMED = 1

GROUND_TRIP_REASON_TEXT = {
    0: "none",
    1: "ground ref missing",
    2: "kalman invalid",
    3: "angle trip",
    4: "saturation trip",
    5: "imu stale",
    6: "watchdog timeout",
    7: "rate jitter trip",
    8: "failsafe",
    9: "ground tune stopped normally",
    10: "battery invalid or critical",
}

DEFAULT_MAX_DUTY = 0.22
DEFAULT_ANGLE_TRIP_DEG = 25.0
DEFAULT_HARD_ANGLE_TRIP_DEG = 35.0
DEFAULT_TELEMETRY_HZ = 50
DEFAULT_MOTOR_LIMIT_TRIP_S = 0.30
DEFAULT_KALMAN_INVALID_TRIP_S = 0.30
DEFAULT_GROUND_TEST_MAX_EXTRA_DUTY = 0.10
DEFAULT_GROUND_TEST_MOTOR_BALANCE_LIMIT = 0.30
DEFAULT_GROUND_TEST_RAMP_DUTY_PER_S = 0.25
DEFAULT_ANGLE_RATE_LIMIT_DPS = 10.0
ANGLE_RATE_LIMIT_DPS_CHOICES = (5.0, 6.0, 7.0, 8.0, 10.0, 12.0)
DEFAULT_TELEMETRY_STALL_TIMEOUT_S = 1.0
AUTO_DISARM_MARGIN_S = 3.0
AUTO_DISARM_MAX_MS = 120000
SAMPLE_POLL_S = 0.01

SHORT_HOP_DEFAULT_DUTY = 0.35
SHORT_HOP_DEFAULT_DURATION_S = 1.4
SHORT_HOP_DEFAULT_MAX_DUTY = 0.45
SHORT_HOP_DEFAULT_MAX_EXTRA_DUTY = 0.10
SHORT_HOP_DEFAULT_ANGLE_RATE_LIMIT_DPS = 6.0
SHORT_HOP_DEFAULT_ANGLE_TRIP_DEG = 10.0
SHORT_HOP_DEFAULT_HARD_ANGLE_TRIP_DEG = 10.0
SHORT_HOP_DEFAULT_MOTOR_LIMIT_TRIP_S = 0.10
SHORT_HOP_DEFAULT_MOTOR_STEP_TRIP = 0.28
SHORT_HOP_DEFAULT_PID_STEP_TRIP = 0.28
SHORT_HOP_CSV_STEM = "short_hop_verify"
SHORT_HOP_DETECT_CSV_STEM = "short_hop_detect"
SHORT_HOP_DETECT_DEFAULT_START_DUTY = 0.34
SHORT_HOP_DETECT_DEFAULT_END_DUTY = 0.38
SHORT_HOP_DETECT_DEFAULT_RAMP_S = 2.0
SHORT_HOP_DETECT_DEFAULT_POST_LIFTOFF_S = 0.25
SHORT_HOP_DETECT_DEFAULT_MAX_DUTY = 0.48
SHORT_HOP_DETECT_DEFAULT_ATTITUDE_JUMP_TRIP_DEG = 3.0
SHORT_HOP_DETECT_RATE_SPIKE_DPS = 18.0
SHORT_HOP_DETECT_ATTITUDE_JUMP_DEG = 0.8
SHORT_HOP_DETECT_MOTOR_SPREAD = 0.08
SHORT_HOP_DETECT_MOTOR_STEP = 0.06
SHORT_HOP_DETECT_ACC_Z_DELTA = 0.08
SHORT_HOP_DETECT_ANALYSIS_PRE_S = 0.50
SHORT_HOP_DETECT_ANALYSIS_POST_S = 0.30

LIFTOFF_THRESHOLD_CSV_FIELDS = [
    "timestamp_us",
    "base_duty",
    *[name for name in TELEMETRY_CSV_FIELDS if name != "timestamp_us"],
]

MOTOR_TRIM_SCALE_FIELDS = [
    "motor_trim_scale_m1",
    "motor_trim_scale_m2",
    "motor_trim_scale_m3",
    "motor_trim_scale_m4",
]
MOTOR_TRIM_OFFSET_FIELDS = [
    "motor_trim_offset_m1",
    "motor_trim_offset_m2",
    "motor_trim_offset_m3",
    "motor_trim_offset_m4",
]
MOTOR_TRIM_CSV_FIELDS = [*MOTOR_TRIM_SCALE_FIELDS, *MOTOR_TRIM_OFFSET_FIELDS]
SHORT_HOP_CSV_FIELDS = [*LIFTOFF_THRESHOLD_CSV_FIELDS, *MOTOR_TRIM_CSV_FIELDS]


@dataclass(slots=True)
class LiftoffThresholdOptions:
    duty: float
    duration_s: float
    max_duty: float = DEFAULT_MAX_DUTY
    angle_trip_deg: float = DEFAULT_ANGLE_TRIP_DEG
    hard_angle_trip_deg: float = DEFAULT_HARD_ANGLE_TRIP_DEG
    telemetry_hz: int = DEFAULT_TELEMETRY_HZ
    output_dir: Path = Path("logs")
    motor_limit_trip_s: float = DEFAULT_MOTOR_LIMIT_TRIP_S
    kalman_invalid_trip_s: float = DEFAULT_KALMAN_INVALID_TRIP_S
    ground_test_max_extra_duty: float = DEFAULT_GROUND_TEST_MAX_EXTRA_DUTY
    ground_test_motor_balance_limit: float = DEFAULT_GROUND_TEST_MOTOR_BALANCE_LIMIT
    ground_test_ramp_duty_per_s: float = DEFAULT_GROUND_TEST_RAMP_DUTY_PER_S
    angle_rate_limit_dps: float = DEFAULT_ANGLE_RATE_LIMIT_DPS
    telemetry_stall_timeout_s: float = DEFAULT_TELEMETRY_STALL_TIMEOUT_S
    settle_timeout_s: float = 2.0
    arm_timeout_s: float = 2.0
    mode_timeout_s: float = 2.0
    csv_stem: str = "liftoff_threshold"
    late_drift_angle_deg: float = 5.0
    max_motor_step_trip: float | None = None
    max_pid_step_trip: float | None = None
    trip_on_inner_loop_clamp: bool = True
    ramp_start_duty: float | None = None
    ramp_end_duty: float | None = None
    ramp_s: float | None = None
    post_liftoff_s: float = SHORT_HOP_DETECT_DEFAULT_POST_LIFTOFF_S
    observed_liftoff_time_s: float | None = None
    max_attitude_jump_trip_deg: float | None = None
    include_motor_trim: bool = False
    liftoff_detect_rate_spike_dps: float = SHORT_HOP_DETECT_RATE_SPIKE_DPS
    liftoff_detect_attitude_jump_deg: float = SHORT_HOP_DETECT_ATTITUDE_JUMP_DEG
    liftoff_detect_motor_spread: float = SHORT_HOP_DETECT_MOTOR_SPREAD
    liftoff_detect_motor_step: float = SHORT_HOP_DETECT_MOTOR_STEP
    liftoff_detect_acc_z_delta: float = SHORT_HOP_DETECT_ACC_Z_DELTA

    def validate(self) -> None:
        if not math.isfinite(self.duty) or self.duty <= 0.0:
            raise ValueError("duty must be > 0")
        if not math.isfinite(self.duration_s) or self.duration_s <= 0.0:
            raise ValueError("duration-s must be > 0")
        if not math.isfinite(self.max_duty) or self.max_duty <= 0.0:
            raise ValueError("max-duty must be > 0")
        if self.duty > self.max_duty:
            raise ValueError("duty must be <= max-duty")
        if not math.isfinite(self.angle_trip_deg) or self.angle_trip_deg <= 0.0 or self.angle_trip_deg > 30.0:
            raise ValueError("angle-trip-deg must be within (0, 30]")
        if not math.isfinite(self.hard_angle_trip_deg) or self.hard_angle_trip_deg <= 0.0:
            raise ValueError("hard-angle-trip-deg must be > 0")
        if self.hard_angle_trip_deg < self.angle_trip_deg:
            raise ValueError("hard-angle-trip-deg must be >= angle-trip-deg")
        if self.telemetry_hz <= 0:
            raise ValueError("telemetry-hz must be > 0")
        if self.motor_limit_trip_s <= 0.0:
            raise ValueError("motor-limit-trip-s must be > 0")
        if self.kalman_invalid_trip_s <= 0.0:
            raise ValueError("kalman-invalid-trip-s must be > 0")
        if self.telemetry_stall_timeout_s <= 0.0:
            raise ValueError("telemetry-stall-timeout-s must be > 0")
        if self.ground_test_max_extra_duty < 0.0 or self.ground_test_max_extra_duty > 0.20:
            raise ValueError("ground_test_max_extra_duty must be within [0, 0.20]")
        if self.ground_test_motor_balance_limit < 0.0 or self.ground_test_motor_balance_limit > 0.30:
            raise ValueError("ground_test_motor_balance_limit must be within [0, 0.30]")
        if self.ground_test_ramp_duty_per_s < 0.01 or self.ground_test_ramp_duty_per_s > 2.0:
            raise ValueError("ground_test_ramp_duty_per_s must be within [0.01, 2.0]")
        if self.angle_rate_limit_dps not in ANGLE_RATE_LIMIT_DPS_CHOICES:
            choices = ", ".join(f"{value:g}" for value in ANGLE_RATE_LIMIT_DPS_CHOICES)
            raise ValueError(f"angle_rate_limit_dps must be one of: {choices}")
        if not self.csv_stem:
            raise ValueError("csv_stem must not be empty")
        if self.late_drift_angle_deg <= 0.0:
            raise ValueError("late_drift_angle_deg must be > 0")
        if self.max_motor_step_trip is not None and self.max_motor_step_trip <= 0.0:
            raise ValueError("max_motor_step_trip must be > 0")
        if self.max_pid_step_trip is not None and self.max_pid_step_trip <= 0.0:
            raise ValueError("max_pid_step_trip must be > 0")
        if self.max_attitude_jump_trip_deg is not None and self.max_attitude_jump_trip_deg <= 0.0:
            raise ValueError("max_attitude_jump_trip_deg must be > 0")
        if self.post_liftoff_s <= 0.0:
            raise ValueError("post_liftoff_s must be > 0")
        if self.observed_liftoff_time_s is not None and self.observed_liftoff_time_s < 0.0:
            raise ValueError("observed_liftoff_time_s must be >= 0")
        if self.ramp_enabled:
            if self.ramp_start_duty is None or self.ramp_end_duty is None or self.ramp_s is None:
                raise ValueError("ramp_start_duty, ramp_end_duty, and ramp_s must be set together")
            if self.ramp_start_duty <= 0.0:
                raise ValueError("ramp_start_duty must be > 0")
            if self.ramp_end_duty <= self.ramp_start_duty:
                raise ValueError("ramp_end_duty must be > ramp_start_duty")
            if self.ramp_end_duty > self.max_duty:
                raise ValueError("ramp_end_duty must be <= max-duty")
            if self.ramp_s <= 0.0:
                raise ValueError("ramp_s must be > 0")
            if self.duty != self.ramp_end_duty:
                raise ValueError("duty must match ramp_end_duty when ramp mode is enabled")

    def auto_disarm_ms(self) -> int:
        total_ms = int(math.ceil((self.duration_s + AUTO_DISARM_MARGIN_S) * 1000.0))
        if total_ms > AUTO_DISARM_MAX_MS:
            raise ValueError(
                "liftoff-threshold requested duration exceeds firmware ground_test_auto_disarm_ms limit"
            )
        return max(1000, total_ms)

    def csv_path(self) -> Path:
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        return self.output_dir / f"{stamp}_{self.csv_stem}.csv"

    @property
    def ramp_enabled(self) -> bool:
        return self.ramp_start_duty is not None or self.ramp_end_duty is not None or self.ramp_s is not None

    def initial_duty(self) -> float:
        return self.ramp_start_duty if self.ramp_enabled and self.ramp_start_duty is not None else self.duty


@dataclass(slots=True)
class LiftoffThresholdResult:
    duty: float
    duration_s: float
    csv_path: Path | None = None
    stop_reason: str = "not_started"
    max_roll_deg: float = 0.0
    max_pitch_deg: float = 0.0
    max_motor_spread: float = 0.0
    max_motor_saturation_run_s: float = 0.0
    max_inner_loop_clamp_run_s: float = 0.0
    max_outer_loop_clamp_run_s: float = 0.0
    max_attitude_jump_deg: float = 0.0
    max_motor_step: float = 0.0
    max_pid_step: float = 0.0
    late_drift_seen: bool = False
    late_drift_start_s: float | None = None
    clamp_seen: bool = False
    saturation_seen: bool = False
    kalman_invalid_seen: bool = False
    battery_zero_seen: bool = False
    liftoff_detected: bool = False
    liftoff_time_s: float | None = None
    liftoff_duty: float | None = None
    liftoff_reason: str | None = None
    liftoff_window_summary: dict[str, Any] | None = None
    liftoff_diagnosis: str | None = None
    divergence_direction: str | None = None
    liftoff_window_pass: bool | None = None
    motor_trim_snapshot: dict[str, float] | None = None
    return_code: int = 1
    interrupted: bool = False
    error_text: str | None = None


@dataclass(slots=True)
class LiftoffThresholdTracker:
    max_duty: float
    angle_trip_deg: float
    hard_angle_trip_deg: float
    motor_limit_trip_s: float
    kalman_invalid_trip_s: float
    duration_s: float | None = None
    late_drift_angle_deg: float = 5.0
    max_motor_step_trip: float | None = None
    max_pid_step_trip: float | None = None
    trip_on_inner_loop_clamp: bool = True
    max_attitude_jump_trip_deg: float | None = None
    max_roll_deg: float = 0.0
    max_pitch_deg: float = 0.0
    max_motor_spread: float = 0.0
    max_motor_saturation_run_s: float = 0.0
    max_inner_loop_clamp_run_s: float = 0.0
    max_outer_loop_clamp_run_s: float = 0.0
    max_attitude_jump_deg: float = 0.0
    max_motor_step: float = 0.0
    max_pid_step: float = 0.0
    late_drift_seen: bool = False
    late_drift_start_s: float | None = None
    clamp_seen: bool = False
    saturation_seen: bool = False
    kalman_invalid_seen: bool = False
    battery_zero_seen: bool = False
    start_timestamp_us: int | None = None
    clamp_active_since_us: int | None = None
    motor_saturation_since_us: int | None = None
    inner_loop_clamp_since_us: int | None = None
    outer_loop_clamp_since_us: int | None = None
    kalman_invalid_since_us: int | None = None
    previous_roll_deg: float | None = None
    previous_pitch_deg: float | None = None
    previous_motors: list[float] | None = None
    previous_pid: list[float] | None = None

    def _update_run_duration(
        self,
        *,
        active: bool,
        timestamp_us: int,
        since_field: str,
        max_field: str,
    ) -> None:
        since = getattr(self, since_field)
        if active:
            if since is None:
                setattr(self, since_field, timestamp_us)
                return
            elapsed_s = (timestamp_us - since) / 1_000_000.0
            setattr(self, max_field, max(getattr(self, max_field), elapsed_s))
        else:
            setattr(self, since_field, None)

    def update(self, sample: TelemetrySample) -> str | None:
        if self.start_timestamp_us is None:
            self.start_timestamp_us = sample.timestamp_us

        roll_deg = float(sample.angle_measured_roll)
        pitch_deg = float(sample.angle_measured_pitch)
        self.max_roll_deg = max(self.max_roll_deg, abs(roll_deg))
        self.max_pitch_deg = max(self.max_pitch_deg, abs(pitch_deg))
        if self.previous_roll_deg is not None and self.previous_pitch_deg is not None:
            attitude_jump = max(abs(roll_deg - self.previous_roll_deg), abs(pitch_deg - self.previous_pitch_deg))
            self.max_attitude_jump_deg = max(self.max_attitude_jump_deg, attitude_jump)
            if (
                self.max_attitude_jump_trip_deg is not None
                and attitude_jump >= self.max_attitude_jump_trip_deg
            ):
                return f"attitude_jump_trip:{attitude_jump:.3f}deg"
        self.previous_roll_deg = roll_deg
        self.previous_pitch_deg = pitch_deg

        motors = [float(sample.motor1), float(sample.motor2), float(sample.motor3), float(sample.motor4)]
        self.max_motor_spread = max(self.max_motor_spread, max(motors) - min(motors))
        if self.previous_motors is not None:
            motor_step = max(abs(current - previous) for previous, current in zip(self.previous_motors, motors))
            self.max_motor_step = max(self.max_motor_step, motor_step)
            if self.max_motor_step_trip is not None and motor_step >= self.max_motor_step_trip:
                return f"motor_step_trip:{motor_step:.4f}"
        self.previous_motors = motors

        pid_values = [float(sample.pid_out_roll), float(sample.pid_out_pitch), float(sample.pid_out_yaw)]
        if self.previous_pid is not None:
            pid_step = max(abs(current - previous) for previous, current in zip(self.previous_pid, pid_values))
            self.max_pid_step = max(self.max_pid_step, pid_step)
            if self.max_pid_step_trip is not None and pid_step >= self.max_pid_step_trip:
                return f"pid_step_trip:{pid_step:.4f}"
        self.previous_pid = pid_values

        if sample.battery_voltage <= 0.0:
            self.battery_zero_seen = True

        self._update_run_duration(
            active=bool(sample.motor_saturation_flag),
            timestamp_us=sample.timestamp_us,
            since_field="motor_saturation_since_us",
            max_field="max_motor_saturation_run_s",
        )
        self._update_run_duration(
            active=bool(sample.inner_loop_clamp_flag),
            timestamp_us=sample.timestamp_us,
            since_field="inner_loop_clamp_since_us",
            max_field="max_inner_loop_clamp_run_s",
        )
        self._update_run_duration(
            active=bool(sample.outer_loop_clamp_flag),
            timestamp_us=sample.timestamp_us,
            since_field="outer_loop_clamp_since_us",
            max_field="max_outer_loop_clamp_run_s",
        )

        clamp_active = bool(sample.motor_saturation_flag) or (
            self.trip_on_inner_loop_clamp and bool(sample.inner_loop_clamp_flag)
        )
        if sample.inner_loop_clamp_flag:
            self.clamp_seen = True
        if sample.motor_saturation_flag:
            self.saturation_seen = True
        if clamp_active:
            if self.clamp_active_since_us is None:
                self.clamp_active_since_us = sample.timestamp_us
            clamp_elapsed_s = (sample.timestamp_us - self.clamp_active_since_us) / 1_000_000.0
            if clamp_elapsed_s > self.motor_limit_trip_s:
                return "sustained_motor_limit"
        else:
            self.clamp_active_since_us = None

        if not sample.kalman_valid:
            self.kalman_invalid_seen = True
            if self.kalman_invalid_since_us is None:
                self.kalman_invalid_since_us = sample.timestamp_us
            kalman_elapsed_s = (sample.timestamp_us - self.kalman_invalid_since_us) / 1_000_000.0
            if kalman_elapsed_s > self.kalman_invalid_trip_s:
                return "sustained_kalman_invalid"
        else:
            self.kalman_invalid_since_us = None

        if sample.failsafe_reason != 0:
            return f"failsafe:{sample.failsafe_reason}"
        if sample.ground_trip_reason != 0:
            return f"ground_trip:{ground_trip_reason_text(sample.ground_trip_reason)}"

        max_abs_angle = max(abs(roll_deg), abs(pitch_deg))
        if (
            self.duration_s is not None
            and self.start_timestamp_us is not None
            and not self.late_drift_seen
            and max_abs_angle >= self.late_drift_angle_deg
        ):
            elapsed_s = (sample.timestamp_us - self.start_timestamp_us) / 1_000_000.0
            if elapsed_s >= self.duration_s * 0.65:
                self.late_drift_seen = True
                self.late_drift_start_s = elapsed_s
        if max_abs_angle >= self.hard_angle_trip_deg:
            return f"hard_angle_trip:{max_abs_angle:.2f}deg"
        if max_abs_angle >= self.angle_trip_deg:
            return f"angle_trip:{max_abs_angle:.2f}deg"
        if sample.base_duty_active > self.max_duty + 1e-6:
            return f"max_duty_exceeded:{sample.base_duty_active:.4f}"
        return None


def ground_trip_reason_text(reason: int) -> str:
    return GROUND_TRIP_REASON_TEXT.get(int(reason), f"unknown({int(reason)})")


def _sample_time_s(sample: TelemetrySample, start_timestamp_us: int) -> float:
    return (sample.timestamp_us - start_timestamp_us) / 1_000_000.0


def _motor_values(sample: TelemetrySample) -> tuple[float, float, float, float]:
    return (float(sample.motor1), float(sample.motor2), float(sample.motor3), float(sample.motor4))


def _motor_spread(sample: TelemetrySample) -> float:
    motors = _motor_values(sample)
    return max(motors) - min(motors)


def _motor_step(sample: TelemetrySample, previous: TelemetrySample | None) -> float:
    if previous is None:
        return 0.0
    return max(abs(current - old) for old, current in zip(_motor_values(previous), _motor_values(sample)))


def _attitude_jump(sample: TelemetrySample, previous: TelemetrySample | None) -> float:
    if previous is None:
        return 0.0
    return max(
        abs(float(sample.angle_measured_roll) - float(previous.angle_measured_roll)),
        abs(float(sample.angle_measured_pitch) - float(previous.angle_measured_pitch)),
    )


def _acc_z_value(sample: TelemetrySample) -> float:
    filtered = float(sample.filtered_acc_z)
    return filtered if math.isfinite(filtered) and abs(filtered) > 1e-6 else float(sample.acc_z)


def _max_run_s(samples: list[TelemetrySample], predicate: Callable[[TelemetrySample], bool]) -> float:
    active_since_us: int | None = None
    max_run_s = 0.0
    for sample in samples:
        if predicate(sample):
            if active_since_us is None:
                active_since_us = sample.timestamp_us
                continue
            max_run_s = max(max_run_s, (sample.timestamp_us - active_since_us) / 1_000_000.0)
        else:
            active_since_us = None
    return max_run_s


def _rms(values: list[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(value * value for value in values) / len(values))


def _window_summary(samples: list[TelemetrySample]) -> dict[str, Any]:
    if not samples:
        return {
            "samples": 0,
            "max_roll_deg": 0.0,
            "max_pitch_deg": 0.0,
            "max_rate_target_dps": 0.0,
            "max_rate_meas_dps": 0.0,
            "rate_err_roll_rms_dps": 0.0,
            "rate_err_pitch_rms_dps": 0.0,
            "max_pid_out": 0.0,
            "motor_min": 0.0,
            "motor_max": 0.0,
            "max_motor_spread": 0.0,
            "sat_run_ms": 0.0,
            "inner_clamp_run_ms": 0.0,
            "outer_clamp_run_ms": 0.0,
            "battery_min_v": 0.0,
            "battery_zero_seen": False,
            "acc_z_mean": 0.0,
            "acc_z_min": 0.0,
            "acc_z_max": 0.0,
        }

    motors = [motor for sample in samples for motor in _motor_values(sample)]
    acc_z = [_acc_z_value(sample) for sample in samples]
    return {
        "samples": len(samples),
        "max_roll_deg": round(max(abs(float(sample.angle_measured_roll)) for sample in samples), 3),
        "max_pitch_deg": round(max(abs(float(sample.angle_measured_pitch)) for sample in samples), 3),
        "max_rate_target_dps": round(
            max(
                max(abs(float(sample.outer_loop_rate_target_roll)), abs(float(sample.outer_loop_rate_target_pitch)))
                for sample in samples
            ),
            3,
        ),
        "max_rate_meas_dps": round(
            max(
                max(abs(float(sample.rate_meas_roll_filtered)), abs(float(sample.rate_meas_pitch_filtered)))
                for sample in samples
            ),
            3,
        ),
        "rate_err_roll_rms_dps": round(
            _rms([float(sample.outer_loop_rate_target_roll) - float(sample.rate_meas_roll_filtered) for sample in samples]),
            3,
        ),
        "rate_err_pitch_rms_dps": round(
            _rms([float(sample.outer_loop_rate_target_pitch) - float(sample.rate_meas_pitch_filtered) for sample in samples]),
            3,
        ),
        "max_pid_out": round(
            max(
                max(abs(float(sample.pid_out_roll)), abs(float(sample.pid_out_pitch)), abs(float(sample.pid_out_yaw)))
                for sample in samples
            ),
            5,
        ),
        "motor_min": round(min(motors), 4),
        "motor_max": round(max(motors), 4),
        "max_motor_spread": round(max(_motor_spread(sample) for sample in samples), 4),
        "sat_run_ms": round(_max_run_s(samples, lambda sample: bool(sample.motor_saturation_flag)) * 1000.0, 1),
        "inner_clamp_run_ms": round(_max_run_s(samples, lambda sample: bool(sample.inner_loop_clamp_flag)) * 1000.0, 1),
        "outer_clamp_run_ms": round(_max_run_s(samples, lambda sample: bool(sample.outer_loop_clamp_flag)) * 1000.0, 1),
        "battery_min_v": round(min(float(sample.battery_voltage) for sample in samples), 3),
        "battery_zero_seen": any(float(sample.battery_voltage) <= 0.0 for sample in samples),
        "acc_z_mean": round(sum(acc_z) / len(acc_z), 4),
        "acc_z_min": round(min(acc_z), 4),
        "acc_z_max": round(max(acc_z), 4),
    }


def _divergence_direction(post_samples: list[TelemetrySample]) -> str:
    if len(post_samples) < 2:
        return "n/a"
    first = post_samples[0]
    last = post_samples[-1]
    roll_delta = float(last.angle_measured_roll) - float(first.angle_measured_roll)
    pitch_delta = float(last.angle_measured_pitch) - float(first.angle_measured_pitch)
    if max(abs(roll_delta), abs(pitch_delta)) < 0.5:
        return "none"
    if abs(pitch_delta) >= abs(roll_delta):
        return "pitch_positive" if pitch_delta > 0.0 else "pitch_negative"
    return "roll_positive" if roll_delta > 0.0 else "roll_negative"


def _diagnose_liftoff_window(pre_summary: dict[str, Any], post_summary: dict[str, Any]) -> str:
    if int(post_summary["samples"]) == 0:
        return "no_post_liftoff_window"
    max_post_angle = max(float(post_summary["max_roll_deg"]), float(post_summary["max_pitch_deg"]))
    sat_or_margin_limited = (
        float(post_summary["sat_run_ms"]) > 0.0
        or float(post_summary["motor_max"]) >= 0.98
    )
    if sat_or_margin_limited:
        return "output_margin_limited"
    pitch_err = float(post_summary["rate_err_pitch_rms_dps"])
    roll_err = float(post_summary["rate_err_roll_rms_dps"])
    if pitch_err >= 8.0 and pitch_err >= roll_err * 1.2:
        return "pitch_rate_tracking_insufficient"
    if roll_err >= 8.0 and roll_err >= pitch_err * 1.2:
        return "roll_rate_tracking_insufficient"
    if (
        float(post_summary["max_motor_spread"]) >= max(0.12, float(pre_summary["max_motor_spread"]) + 0.04)
        and max_post_angle <= 8.0
    ):
        return "ground_release_or_thrust_imbalance_transient"
    if max_post_angle <= 8.0 and float(post_summary["sat_run_ms"]) < 50.0:
        return "liftoff_transient_controlled"
    return "needs_window_review"


def summarize_liftoff_window(
    samples: list[TelemetrySample],
    *,
    liftoff_time_s: float,
    start_timestamp_us: int,
) -> tuple[dict[str, Any], str, str]:
    pre_samples: list[TelemetrySample] = []
    post_samples: list[TelemetrySample] = []
    for sample in samples:
        elapsed_s = _sample_time_s(sample, start_timestamp_us)
        if liftoff_time_s - SHORT_HOP_DETECT_ANALYSIS_PRE_S <= elapsed_s < liftoff_time_s:
            pre_samples.append(sample)
        if liftoff_time_s <= elapsed_s <= liftoff_time_s + SHORT_HOP_DETECT_ANALYSIS_POST_S:
            post_samples.append(sample)
    pre_summary = _window_summary(pre_samples)
    post_summary = _window_summary(post_samples)
    summary = {
        "pre_500ms": pre_summary,
        "post_300ms": post_summary,
    }
    divergence = _divergence_direction(post_samples)
    diagnosis = _diagnose_liftoff_window(pre_summary, post_summary)
    return summary, divergence, diagnosis


def liftoff_window_passes(summary: dict[str, Any]) -> bool:
    post_summary = dict(summary.get("post_300ms", {}))
    if int(post_summary.get("samples", 0)) == 0:
        return False
    max_post_angle = max(
        float(post_summary.get("max_roll_deg", 0.0)),
        float(post_summary.get("max_pitch_deg", 0.0)),
    )
    return max_post_angle <= 8.0 and float(post_summary.get("sat_run_ms", 0.0)) < 50.0


def detect_liftoff_event(
    sample: TelemetrySample,
    previous_sample: TelemetrySample | None,
    *,
    start_timestamp_us: int,
    baseline_acc_z: float | None,
    options: LiftoffThresholdOptions,
) -> str | None:
    elapsed_s = _sample_time_s(sample, start_timestamp_us)
    if options.observed_liftoff_time_s is not None and elapsed_s >= options.observed_liftoff_time_s:
        return "observed_marker"

    start_duty = options.ramp_start_duty if options.ramp_start_duty is not None else options.initial_duty()
    end_duty = options.ramp_end_duty if options.ramp_end_duty is not None else options.duty
    if float(sample.base_duty_active) < start_duty - 0.01:
        return None

    reasons: list[str] = []
    rate_mag = max(abs(float(sample.rate_meas_roll_filtered)), abs(float(sample.rate_meas_pitch_filtered)))
    if rate_mag >= options.liftoff_detect_rate_spike_dps:
        reasons.append(f"rate_spike:{rate_mag:.1f}dps")

    jump = _attitude_jump(sample, previous_sample)
    if jump >= options.liftoff_detect_attitude_jump_deg:
        reasons.append(f"attitude_jump:{jump:.2f}deg")

    motor_spread = _motor_spread(sample)
    if motor_spread >= options.liftoff_detect_motor_spread:
        reasons.append(f"motor_spread:{motor_spread:.3f}")

    motor_step = _motor_step(sample, previous_sample)
    if motor_step >= options.liftoff_detect_motor_step:
        reasons.append(f"motor_step:{motor_step:.3f}")

    if baseline_acc_z is not None:
        acc_delta = abs(_acc_z_value(sample) - baseline_acc_z)
        if acc_delta >= options.liftoff_detect_acc_z_delta:
            reasons.append(f"acc_z_delta:{acc_delta:.3f}")

    high_duty_threshold = start_duty + max(0.0, end_duty - start_duty) * 0.75
    if float(sample.base_duty_active) >= high_duty_threshold and rate_mag >= options.liftoff_detect_rate_spike_dps * 0.65:
        reasons.append(f"high_duty_free_rate:{rate_mag:.1f}dps")

    if len(reasons) >= 2 or rate_mag >= options.liftoff_detect_rate_spike_dps * 1.8:
        return "+".join(reasons)
    return None


def _read_motor_trim_snapshot(session: DeviceSession) -> dict[str, float]:
    snapshot: dict[str, float] = {}
    for name in MOTOR_TRIM_CSV_FIELDS:
        snapshot[name] = float(session.get_param(name).value)
    return snapshot


def format_liftoff_threshold_summary(result: LiftoffThresholdResult) -> list[str]:
    lines = [
        f"csv={result.csv_path or 'n/a'}",
        f"duty={result.duty:.3f}",
        f"duration_s={result.duration_s:.3f}",
        f"stop_reason={result.stop_reason}",
        f"max_roll_deg={result.max_roll_deg:.3f}",
        f"max_pitch_deg={result.max_pitch_deg:.3f}",
        f"max_motor_spread={result.max_motor_spread:.4f}",
        f"sat_run_ms={result.max_motor_saturation_run_s * 1000.0:.1f}",
        f"outer_clamp_run_ms={result.max_outer_loop_clamp_run_s * 1000.0:.1f}",
        f"attitude_jump_deg={result.max_attitude_jump_deg:.3f}",
        f"max_motor_step={result.max_motor_step:.4f}",
        f"max_pid_step={result.max_pid_step:.4f}",
        f"late_drift={result.late_drift_seen}",
        f"late_drift_start_s={result.late_drift_start_s if result.late_drift_start_s is not None else 'n/a'}",
        f"clamp_seen={result.clamp_seen} saturation_seen={result.saturation_seen}",
        f"kalman_invalid_seen={result.kalman_invalid_seen}",
        f"battery_zero_seen={result.battery_zero_seen}",
    ]
    if result.motor_trim_snapshot is not None:
        trim = result.motor_trim_snapshot
        lines.extend(
            [
                "motor_trim_scales="
                + ",".join(f"{trim[name]:.6f}" for name in MOTOR_TRIM_SCALE_FIELDS),
                "motor_trim_offsets="
                + ",".join(f"{trim[name]:.6f}" for name in MOTOR_TRIM_OFFSET_FIELDS),
            ]
        )
    if (
        result.liftoff_time_s is not None
        or result.liftoff_detected
        or result.liftoff_window_summary is not None
        or result.liftoff_diagnosis is not None
    ):
        lines.extend(
            [
                f"liftoff_detected={result.liftoff_detected}",
                f"liftoff_time_s={result.liftoff_time_s if result.liftoff_time_s is not None else 'n/a'}",
                f"liftoff_duty={result.liftoff_duty if result.liftoff_duty is not None else 'n/a'}",
                f"liftoff_reason={result.liftoff_reason or 'n/a'}",
                f"liftoff_window_pass={result.liftoff_window_pass if result.liftoff_window_pass is not None else 'n/a'}",
                f"divergence_direction={result.divergence_direction or 'n/a'}",
                f"diagnosis={result.liftoff_diagnosis or 'n/a'}",
            ]
        )
        if result.liftoff_window_summary is not None:
            lines.append(
                "liftoff_window="
                + json.dumps(result.liftoff_window_summary, sort_keys=True, separators=(",", ":"))
            )
    return lines


def run_liftoff_threshold(
    session: DeviceSession,
    options: LiftoffThresholdOptions,
    *,
    progress: Callable[[str], None] | None = None,
) -> LiftoffThresholdResult:
    options.validate()
    options.output_dir.mkdir(parents=True, exist_ok=True)

    result = LiftoffThresholdResult(
        duty=options.duty,
        duration_s=options.duration_s,
        csv_path=options.csv_path(),
    )
    sample_queue: queue.Queue[TelemetrySample] = queue.Queue()
    latest_sample_lock = threading.Lock()
    latest_sample: list[TelemetrySample | None] = [None]
    connection_state = {"connected": True, "error": None}
    tracker = LiftoffThresholdTracker(
        max_duty=options.max_duty,
        angle_trip_deg=options.angle_trip_deg,
        hard_angle_trip_deg=options.hard_angle_trip_deg,
        motor_limit_trip_s=options.motor_limit_trip_s,
        kalman_invalid_trip_s=options.kalman_invalid_trip_s,
        duration_s=options.duration_s,
        late_drift_angle_deg=options.late_drift_angle_deg,
        max_motor_step_trip=options.max_motor_step_trip,
        max_pid_step_trip=options.max_pid_step_trip,
        trip_on_inner_loop_clamp=options.trip_on_inner_loop_clamp,
        max_attitude_jump_trip_deg=options.max_attitude_jump_trip_deg,
    )
    target_duty_state = {"value": options.initial_duty()}
    history: list[TelemetrySample] = []
    start_timestamp_us: int | None = None
    previous_sample: TelemetrySample | None = None
    baseline_acc_values: list[float] = []
    baseline_acc_z: float | None = None
    ramp_start_elapsed_s: float | None = None
    liftoff_post_hold_until_us: int | None = None
    last_ramp_param_update_s = 0.0
    originals: dict[str, ParamValue] = {}
    started_stream = False
    started_log = False
    started_verify = False
    telemetry_token: int | None = None
    event_token: int | None = None
    connection_token: int | None = None

    def emit_progress(message: str) -> None:
        if progress is not None:
            progress(message)

    def on_telemetry(sample: TelemetrySample) -> None:
        with latest_sample_lock:
            latest_sample[0] = sample
        sample_queue.put(sample)

    def on_event(_message: str) -> None:
        return None

    def on_connection(payload: dict[str, object]) -> None:
        connection_state["connected"] = bool(payload.get("connected", False))
        connection_state["error"] = payload.get("error")

    def get_latest_sample() -> TelemetrySample | None:
        with latest_sample_lock:
            return latest_sample[0]

    def drain_samples() -> list[TelemetrySample]:
        samples: list[TelemetrySample] = []
        while True:
            try:
                samples.append(sample_queue.get_nowait())
            except queue.Empty:
                return samples

    def wait_for_sample(timeout_s: float, *, reason: str) -> TelemetrySample:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for sample in drain_samples():
                return sample
            time.sleep(SAMPLE_POLL_S)
        raise TimeoutError(reason)

    def wait_until(
        predicate: Callable[[TelemetrySample], bool],
        timeout_s: float,
        *,
        reason: str,
    ) -> TelemetrySample:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            latest = get_latest_sample()
            for sample in drain_samples():
                latest = sample
                if predicate(sample):
                    return sample
            if latest is not None and predicate(latest):
                return latest
            if connection_state["connected"] is False:
                raise RuntimeError(f"connection lost: {connection_state['error'] or 'unknown error'}")
            time.sleep(SAMPLE_POLL_S)
        raise TimeoutError(reason)

    def remember_param(name: str) -> ParamValue:
        value = session.get_param(name)
        originals[name] = value
        return value

    def set_and_track_param(name: str, type_id: int, value: object) -> None:
        if name not in originals:
            remember_param(name)
        session.set_param(name, type_id, value)

    def apply_params() -> None:
        set_and_track_param("telemetry_usb_hz", 2, options.telemetry_hz)
        set_and_track_param("ground_tune_use_kalman_attitude", 0, False)
        set_and_track_param("ground_att_trip_deg", 4, options.angle_trip_deg)
        set_and_track_param("ground_test_base_duty", 4, options.initial_duty())
        set_and_track_param("ground_test_max_extra_duty", 4, options.ground_test_max_extra_duty)
        set_and_track_param("ground_test_motor_balance_limit", 4, options.ground_test_motor_balance_limit)
        set_and_track_param("ground_test_ramp_duty_per_s", 4, options.ground_test_ramp_duty_per_s)
        set_and_track_param("ground_att_rate_limit_roll", 4, options.angle_rate_limit_dps)
        set_and_track_param("ground_att_rate_limit_pitch", 4, options.angle_rate_limit_dps)
        set_and_track_param("ground_test_auto_disarm_ms", 2, options.auto_disarm_ms())

    def restore_params() -> None:
        for name, original in originals.items():
            try:
                session.set_param(name, original.type_id, original.value)
            except Exception:
                continue

    def stop_control_path() -> None:
        if not started_verify:
            return
        try:
            ensure_command_ok(CmdId.ATTITUDE_GROUND_VERIFY_STOP, session.attitude_ground_verify_stop())
        except Exception:
            try:
                session.attitude_ground_verify_stop()
            except Exception:
                pass

    def copy_tracker_to_result() -> None:
        result.max_roll_deg = tracker.max_roll_deg
        result.max_pitch_deg = tracker.max_pitch_deg
        result.max_motor_spread = tracker.max_motor_spread
        result.max_motor_saturation_run_s = tracker.max_motor_saturation_run_s
        result.max_inner_loop_clamp_run_s = tracker.max_inner_loop_clamp_run_s
        result.max_outer_loop_clamp_run_s = tracker.max_outer_loop_clamp_run_s
        result.max_attitude_jump_deg = tracker.max_attitude_jump_deg
        result.max_motor_step = tracker.max_motor_step
        result.max_pid_step = tracker.max_pid_step
        result.late_drift_seen = tracker.late_drift_seen
        result.late_drift_start_s = tracker.late_drift_start_s
        result.clamp_seen = tracker.clamp_seen
        result.saturation_seen = tracker.saturation_seen
        result.kalman_invalid_seen = tracker.kalman_invalid_seen
        result.battery_zero_seen = tracker.battery_zero_seen

    try:
        emit_progress("applying liftoff-threshold parameters")
        telemetry_token = session.subscribe_telemetry(on_telemetry)
        event_token = session.subscribe_event_log(on_event)
        connection_token = session.subscribe_connection_state(on_connection)
        apply_params()
        if options.include_motor_trim:
            result.motor_trim_snapshot = _read_motor_trim_snapshot(session)

        csv_fieldnames = SHORT_HOP_CSV_FIELDS if options.include_motor_trim else LIFTOFF_THRESHOLD_CSV_FIELDS

        session.start_csv_log(
            result.csv_path,
            fieldnames=csv_fieldnames,
            extra_row_fn=lambda _sample: {
                "base_duty": target_duty_state["value"],
                **(result.motor_trim_snapshot or {}),
            },
        )
        started_log = True

        emit_progress("starting telemetry stream")
        session.start_stream()
        started_stream = True
        wait_for_sample(options.settle_timeout_s, reason="timed out waiting for telemetry")

        emit_progress("capturing ground reference")
        ensure_command_ok(CmdId.GROUND_CAPTURE_REF, session.ground_capture_ref())

        emit_progress("arming")
        ensure_command_ok(CmdId.ARM, session.arm())
        wait_until(
            lambda sample: sample.arm_state == ARM_STATE_ARMED,
            options.arm_timeout_s,
            reason="timed out waiting for armed telemetry state",
        )

        emit_progress("starting closed-loop attitude ground verify path")
        ensure_command_ok(
            CmdId.ATTITUDE_GROUND_VERIFY_START,
            session.attitude_ground_verify_start(base_duty=options.initial_duty()),
        )
        started_verify = True
        active_sample = wait_until(
            lambda sample: (
                sample.control_mode == CONTROL_MODE_ATTITUDE_GROUND_TUNE
                and sample.control_submode == GROUND_TUNE_SUBMODE_ATTITUDE_VERIFY
            ),
            options.mode_timeout_s,
            reason="timed out waiting for attitude-ground-verify mode",
        )
        start_timestamp_us = active_sample.timestamp_us
        history.append(active_sample)

        stop_reason: str | None = None
        last_sample_time = time.monotonic()
        deadline = time.monotonic() + options.duration_s
        while time.monotonic() < deadline:
            new_samples = drain_samples()
            if new_samples:
                last_sample_time = time.monotonic()
            for sample in new_samples:
                history.append(sample)
                sample_stop_reason = tracker.update(sample)
                if sample_stop_reason is not None:
                    stop_reason = sample_stop_reason
                    break
                if (
                    sample.control_mode != CONTROL_MODE_ATTITUDE_GROUND_TUNE
                    or sample.control_submode != GROUND_TUNE_SUBMODE_ATTITUDE_VERIFY
                ):
                    stop_reason = f"control_mode_left_path:{sample.control_mode}/{sample.control_submode}"
                    break
                if connection_state["connected"] is False:
                    stop_reason = f"serial_disconnected:{connection_state['error'] or 'unknown'}"
                    break
                if options.ramp_enabled and start_timestamp_us is not None:
                    start_duty = float(options.ramp_start_duty)
                    end_duty = float(options.ramp_end_duty)
                    elapsed_s = _sample_time_s(sample, start_timestamp_us)
                    if ramp_start_elapsed_s is None and float(sample.base_duty_active) >= start_duty - 0.003:
                        ramp_start_elapsed_s = elapsed_s
                    if ramp_start_elapsed_s is not None:
                        progress_fraction = min(1.0, max(0.0, (elapsed_s - ramp_start_elapsed_s) / float(options.ramp_s)))
                        next_target = start_duty + (end_duty - start_duty) * progress_fraction
                        now_s = time.monotonic()
                        if (
                            abs(next_target - target_duty_state["value"]) >= 0.0005
                            and now_s - last_ramp_param_update_s >= 0.04
                        ):
                            target_duty_state["value"] = next_target
                            set_and_track_param("ground_test_base_duty", 4, next_target)
                            last_ramp_param_update_s = now_s

                    if (
                        baseline_acc_z is None
                        and float(sample.base_duty_active) <= start_duty + 0.01
                        and len(baseline_acc_values) < 20
                    ):
                        baseline_acc_values.append(_acc_z_value(sample))
                        if len(baseline_acc_values) >= 5:
                            baseline_acc_z = sum(baseline_acc_values) / len(baseline_acc_values)

                    if not result.liftoff_detected:
                        liftoff_reason = detect_liftoff_event(
                            sample,
                            previous_sample,
                            start_timestamp_us=start_timestamp_us,
                            baseline_acc_z=baseline_acc_z,
                            options=options,
                        )
                        if liftoff_reason is not None:
                            result.liftoff_detected = True
                            result.liftoff_time_s = elapsed_s
                            result.liftoff_duty = float(sample.base_duty_active)
                            result.liftoff_reason = liftoff_reason
                            liftoff_post_hold_until_us = sample.timestamp_us + int(
                                options.post_liftoff_s * 1_000_000.0
                            )
                            emit_progress(f"liftoff detected: {liftoff_reason}")
                    elif liftoff_post_hold_until_us is not None and sample.timestamp_us >= liftoff_post_hold_until_us:
                        stop_reason = "liftoff_detected_post_hold_complete"
                        break

                    if (
                        not result.liftoff_detected
                        and ramp_start_elapsed_s is not None
                        and target_duty_state["value"] >= end_duty - 0.0005
                        and float(sample.base_duty_active) >= end_duty - 0.003
                    ):
                        stop_reason = "no_liftoff_detected_at_end_duty"
                        break
                previous_sample = sample
            if stop_reason is not None:
                break
            if time.monotonic() - last_sample_time > options.telemetry_stall_timeout_s:
                stop_reason = "telemetry_stall"
                break
            time.sleep(SAMPLE_POLL_S)

        copy_tracker_to_result()
        if start_timestamp_us is not None and result.liftoff_detected and result.liftoff_time_s is not None:
            (
                result.liftoff_window_summary,
                result.divergence_direction,
                result.liftoff_diagnosis,
            ) = summarize_liftoff_window(
                history,
                liftoff_time_s=result.liftoff_time_s,
                start_timestamp_us=start_timestamp_us,
            )
            result.liftoff_window_pass = liftoff_window_passes(result.liftoff_window_summary)
        elif options.ramp_enabled:
            result.liftoff_diagnosis = "no_liftoff_detected"
            result.liftoff_window_pass = False

        if stop_reason is None:
            if options.ramp_enabled:
                result.stop_reason = (
                    "liftoff_detected_timeout_before_post_hold"
                    if result.liftoff_detected
                    else "no_liftoff_detected_before_timeout"
                )
                result.return_code = 2
            else:
                result.stop_reason = "completed_requested_duration"
                result.return_code = 0
        else:
            result.stop_reason = stop_reason
            result.return_code = (
                0
                if stop_reason == "liftoff_detected_post_hold_complete" and result.liftoff_window_pass is not False
                else 2
            )
    except KeyboardInterrupt:
        copy_tracker_to_result()
        result.stop_reason = "keyboard_interrupt"
        result.return_code = 130
        result.interrupted = True
    except Exception as exc:
        copy_tracker_to_result()
        result.stop_reason = f"exception:{exc}"
        result.return_code = 1
        result.error_text = str(exc)
    finally:
        try:
            stop_control_path()
        except Exception:
            pass
        try:
            ensure_command_ok(CmdId.DISARM, session.disarm())
        except Exception:
            try:
                session.disarm()
            except Exception:
                pass
        try:
            if started_stream:
                session.stop_stream()
        except Exception:
            pass
        try:
            restore_params()
        except Exception:
            pass
        try:
            if started_log:
                session.stop_csv_log()
        except Exception:
            pass
        if telemetry_token is not None:
            try:
                session.unsubscribe(telemetry_token)
            except Exception:
                pass
        if event_token is not None:
            try:
                session.unsubscribe(event_token)
            except Exception:
                pass
        if connection_token is not None:
            try:
                session.unsubscribe(connection_token)
            except Exception:
                pass

    return result
