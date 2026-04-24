from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import queue
import threading
import time
from typing import Callable

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
DEFAULT_GROUND_TEST_MAX_EXTRA_DUTY = 0.08
DEFAULT_GROUND_TEST_MOTOR_BALANCE_LIMIT = 0.30
DEFAULT_GROUND_TEST_RAMP_DUTY_PER_S = 0.25
DEFAULT_TELEMETRY_STALL_TIMEOUT_S = 1.0
AUTO_DISARM_MARGIN_S = 3.0
AUTO_DISARM_MAX_MS = 120000
SAMPLE_POLL_S = 0.01

LIFTOFF_THRESHOLD_CSV_FIELDS = [
    "timestamp_us",
    "base_duty",
    *[name for name in TELEMETRY_CSV_FIELDS if name != "timestamp_us"],
]


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
    telemetry_stall_timeout_s: float = DEFAULT_TELEMETRY_STALL_TIMEOUT_S
    settle_timeout_s: float = 2.0
    arm_timeout_s: float = 2.0
    mode_timeout_s: float = 2.0

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

    def auto_disarm_ms(self) -> int:
        total_ms = int(math.ceil((self.duration_s + AUTO_DISARM_MARGIN_S) * 1000.0))
        if total_ms > AUTO_DISARM_MAX_MS:
            raise ValueError(
                "liftoff-threshold requested duration exceeds firmware ground_test_auto_disarm_ms limit"
            )
        return max(1000, total_ms)

    def csv_path(self) -> Path:
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        return self.output_dir / f"{stamp}_liftoff_threshold.csv"


@dataclass(slots=True)
class LiftoffThresholdResult:
    duty: float
    duration_s: float
    csv_path: Path | None = None
    stop_reason: str = "not_started"
    max_roll_deg: float = 0.0
    max_pitch_deg: float = 0.0
    max_motor_spread: float = 0.0
    clamp_seen: bool = False
    saturation_seen: bool = False
    kalman_invalid_seen: bool = False
    battery_zero_seen: bool = False
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
    max_roll_deg: float = 0.0
    max_pitch_deg: float = 0.0
    max_motor_spread: float = 0.0
    clamp_seen: bool = False
    saturation_seen: bool = False
    kalman_invalid_seen: bool = False
    battery_zero_seen: bool = False
    clamp_active_since_us: int | None = None
    kalman_invalid_since_us: int | None = None

    def update(self, sample: TelemetrySample) -> str | None:
        self.max_roll_deg = max(self.max_roll_deg, abs(float(sample.angle_measured_roll)))
        self.max_pitch_deg = max(self.max_pitch_deg, abs(float(sample.angle_measured_pitch)))
        motors = [float(sample.motor1), float(sample.motor2), float(sample.motor3), float(sample.motor4)]
        self.max_motor_spread = max(self.max_motor_spread, max(motors) - min(motors))

        if sample.battery_voltage <= 0.0:
            self.battery_zero_seen = True

        clamp_active = bool(sample.inner_loop_clamp_flag) or bool(sample.motor_saturation_flag)
        if sample.inner_loop_clamp_flag:
            self.clamp_seen = True
        if sample.motor_saturation_flag:
            self.saturation_seen = True
        if clamp_active:
            if self.clamp_active_since_us is None:
                self.clamp_active_since_us = sample.timestamp_us
            clamp_elapsed_s = (sample.timestamp_us - self.clamp_active_since_us) / 1_000_000.0
            if clamp_elapsed_s >= self.motor_limit_trip_s:
                return "sustained_motor_limit"
        else:
            self.clamp_active_since_us = None

        if not sample.kalman_valid:
            self.kalman_invalid_seen = True
            if self.kalman_invalid_since_us is None:
                self.kalman_invalid_since_us = sample.timestamp_us
            kalman_elapsed_s = (sample.timestamp_us - self.kalman_invalid_since_us) / 1_000_000.0
            if kalman_elapsed_s >= self.kalman_invalid_trip_s:
                return "sustained_kalman_invalid"
        else:
            self.kalman_invalid_since_us = None

        if sample.failsafe_reason != 0:
            return f"failsafe:{sample.failsafe_reason}"
        if sample.ground_trip_reason != 0:
            return f"ground_trip:{ground_trip_reason_text(sample.ground_trip_reason)}"

        max_abs_angle = max(abs(float(sample.angle_measured_roll)), abs(float(sample.angle_measured_pitch)))
        if max_abs_angle >= self.hard_angle_trip_deg:
            return f"hard_angle_trip:{max_abs_angle:.2f}deg"
        if max_abs_angle >= self.angle_trip_deg:
            return f"angle_trip:{max_abs_angle:.2f}deg"
        if sample.base_duty_active > self.max_duty + 1e-6:
            return f"max_duty_exceeded:{sample.base_duty_active:.4f}"
        return None


def ground_trip_reason_text(reason: int) -> str:
    return GROUND_TRIP_REASON_TEXT.get(int(reason), f"unknown({int(reason)})")


def format_liftoff_threshold_summary(result: LiftoffThresholdResult) -> list[str]:
    return [
        f"csv={result.csv_path or 'n/a'}",
        f"duty={result.duty:.3f}",
        f"duration_s={result.duration_s:.3f}",
        f"stop_reason={result.stop_reason}",
        f"max_roll_deg={result.max_roll_deg:.3f}",
        f"max_pitch_deg={result.max_pitch_deg:.3f}",
        f"max_motor_spread={result.max_motor_spread:.4f}",
        f"clamp_seen={result.clamp_seen} saturation_seen={result.saturation_seen}",
        f"kalman_invalid_seen={result.kalman_invalid_seen}",
        f"battery_zero_seen={result.battery_zero_seen}",
    ]


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
    )
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
        set_and_track_param("ground_test_base_duty", 4, options.duty)
        set_and_track_param("ground_test_max_extra_duty", 4, options.ground_test_max_extra_duty)
        set_and_track_param("ground_test_motor_balance_limit", 4, options.ground_test_motor_balance_limit)
        set_and_track_param("ground_test_ramp_duty_per_s", 4, options.ground_test_ramp_duty_per_s)
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

        session.start_csv_log(
            result.csv_path,
            fieldnames=LIFTOFF_THRESHOLD_CSV_FIELDS,
            extra_row_fn=lambda _sample: {"base_duty": options.duty},
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
            session.attitude_ground_verify_start(base_duty=options.duty),
        )
        started_verify = True
        wait_until(
            lambda sample: (
                sample.control_mode == CONTROL_MODE_ATTITUDE_GROUND_TUNE
                and sample.control_submode == GROUND_TUNE_SUBMODE_ATTITUDE_VERIFY
            ),
            options.mode_timeout_s,
            reason="timed out waiting for attitude-ground-verify mode",
        )

        stop_reason: str | None = None
        last_sample_time = time.monotonic()
        deadline = time.monotonic() + options.duration_s
        while time.monotonic() < deadline:
            new_samples = drain_samples()
            if new_samples:
                last_sample_time = time.monotonic()
            for sample in new_samples:
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
            if stop_reason is not None:
                break
            if time.monotonic() - last_sample_time > options.telemetry_stall_timeout_s:
                stop_reason = "telemetry_stall"
                break
            time.sleep(SAMPLE_POLL_S)

        copy_tracker_to_result()

        if stop_reason is None:
            result.stop_reason = "completed_requested_duration"
            result.return_code = 0
        else:
            result.stop_reason = stop_reason
            result.return_code = 2
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
