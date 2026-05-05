from __future__ import annotations

import csv
from dataclasses import dataclass, field
import math
from pathlib import Path
import statistics
import time
from typing import Callable

from .device_session import DeviceSession
from .models import TELEMETRY_CSV_FIELDS, ParamValue, TelemetrySample
from .protocol.messages import CmdId, CmdStatus, ensure_command_ok


MOTOR_BALANCE_DEFAULT_DUTIES = (0.20, 0.25, 0.30, 0.35)
MOTOR_BALANCE_DEFAULT_DURATION_S = 0.9
MOTOR_BALANCE_DEFAULT_REST_S = 3.0
MOTOR_BALANCE_DEFAULT_TELEMETRY_HZ = 100
MOTOR_BALANCE_MAX_DUTY = 0.35
MOTOR_BALANCE_CSV_STEM = "motor_thrust_balance"

MOTOR_BALANCE_CSV_FIELDS = [
    "test_trial_id",
    "test_motor",
    "test_duty",
    "test_input_duty",
    "test_trim_scale",
    "test_trim_offset",
    "test_trim_target_duty",
    "test_phase",
    *TELEMETRY_CSV_FIELDS,
]

MOTOR_BALANCE_SUMMARY_FIELDS = [
    "trial_id",
    "motor",
    "duty",
    "input_duty",
    "trim_scale",
    "trim_offset",
    "trim_target_duty",
    "sample_count",
    "battery_min_v",
    "battery_mean_v",
    "gyro_rms_dps",
    "gyro_peak_dps",
    "gyro_x_mean_dps",
    "gyro_y_mean_dps",
    "gyro_z_mean_dps",
    "acc_rms_g",
    "acc_std_g",
    "response_score",
    "relative_to_duty_mean",
    "classification",
]

MOTOR_TRIM_SCALE_FIELDS = {
    "M1": "motor_scale_m1",
    "M2": "motor_scale_m2",
    "M3": "motor_scale_m3",
    "M4": "motor_scale_m4",
}
MOTOR_TRIM_OFFSET_FIELDS = {
    "M1": "motor_offset_m1",
    "M2": "motor_offset_m2",
    "M3": "motor_offset_m3",
    "M4": "motor_offset_m4",
}

MOTOR_BALANCE_TRIM_PATH = "motor-test -> motor_set_test_output -> motor_set_armed_outputs"
MOTOR_BALANCE_TRIM_PARAMS = (
    "motor_trim,motor_gamma,motor_scale,motor_offset,motor_deadband,motor_min_start"
)
MOTOR_BALANCE_SCORE_NOTE = (
    "score is IMU vibration/disturbance response, not thrust-stand force; "
    "ratio_to_M1 is not a real thrust ratio"
)


@dataclass(slots=True)
class MotorBalanceOptions:
    duties: tuple[float, ...] = MOTOR_BALANCE_DEFAULT_DUTIES
    duration_s: float = MOTOR_BALANCE_DEFAULT_DURATION_S
    rest_s: float = MOTOR_BALANCE_DEFAULT_REST_S
    settle_s: float = 0.3
    telemetry_hz: int = MOTOR_BALANCE_DEFAULT_TELEMETRY_HZ
    output_dir: Path = Path("logs")
    motors: tuple[int, ...] = (1, 2, 3, 4)
    use_trim: bool = True

    def validate(self) -> None:
        if not self.duties:
            raise ValueError("at least one duty is required")
        for duty in self.duties:
            if not math.isfinite(duty) or duty <= 0.0 or duty > MOTOR_BALANCE_MAX_DUTY:
                raise ValueError(f"each duty must be within (0, {MOTOR_BALANCE_MAX_DUTY:.2f}]")
        if not math.isfinite(self.duration_s) or self.duration_s < 0.2 or self.duration_s > 2.0:
            raise ValueError("duration-s must be within [0.2, 2.0]")
        if not math.isfinite(self.rest_s) or self.rest_s < 0.0:
            raise ValueError("rest-s must be >= 0")
        if not math.isfinite(self.settle_s) or self.settle_s < 0.0:
            raise ValueError("settle-s must be >= 0")
        if self.telemetry_hz <= 0:
            raise ValueError("telemetry-hz must be > 0")
        for motor in self.motors:
            if motor < 1 or motor > 4:
                raise ValueError("motors must be in 1..4")

    def csv_path(self) -> Path:
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        return self.output_dir / f"{stamp}_{MOTOR_BALANCE_CSV_STEM}.csv"

    def summary_path(self, csv_path: Path) -> Path:
        return csv_path.with_name(csv_path.stem + "_summary.csv")


@dataclass(slots=True)
class MotorBalanceTrial:
    trial_id: int
    motor: int
    duty: float
    trim_scale: float
    trim_offset: float
    trim_target_duty: float
    samples: list[TelemetrySample]


@dataclass(slots=True)
class MotorBalanceResult:
    csv_path: Path
    summary_path: Path
    trials: list[dict[str, object]]
    weak_candidates: list[str]
    trim_targets: list[dict[str, object]] = field(default_factory=list)
    trim_applied: bool = True
    trim_mode: str = "motor_scale/motor_offset"
    return_code: int = 0
    stop_reason: str = "completed"
    completed_trials: int = 0
    expected_trials: int = 0
    summary_incomplete: bool = False


@dataclass(slots=True)
class MotorTrimEstimate:
    source_path: Path
    scales: dict[str, float]
    offsets: dict[str, float]
    ratios: dict[str, float]
    applied: bool = False
    readback_scales: dict[str, float] = field(default_factory=dict)
    readback_offsets: dict[str, float] = field(default_factory=dict)
    serial_port: str = ""


def parse_duties(text: str) -> tuple[float, ...]:
    values = tuple(float(part.strip()) for part in text.split(",") if part.strip())
    if not values:
        raise ValueError("duties must not be empty")
    return values


def parse_motors(text: str) -> tuple[int, ...]:
    motors: list[int] = []
    for part in text.split(","):
        token = part.strip().lower()
        if not token:
            continue
        motor = int(token[1:]) if token.startswith("m") else int(token)
        motors.append(motor)
    if not motors:
        raise ValueError("motors must not be empty")
    return tuple(motors)


def _clamp_float(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def _rms(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else 0.0


def _std(values: list[float]) -> float:
    return statistics.pstdev(values) if len(values) > 1 else 0.0


def _sample_gyro_mag(sample: TelemetrySample) -> float:
    return math.sqrt(float(sample.gyro_x) ** 2 + float(sample.gyro_y) ** 2 + float(sample.gyro_z) ** 2)


def _sample_acc_mag(sample: TelemetrySample) -> float:
    return math.sqrt(float(sample.acc_x) ** 2 + float(sample.acc_y) ** 2 + float(sample.acc_z) ** 2)


def _motor_label(motor: int) -> str:
    return f"M{motor}"


def _trim_target_duty(input_duty: float, scale: float, offset: float) -> float:
    return _clamp_float(input_duty * scale + offset, 0.0, 1.0)


def _read_trim_settings(session: DeviceSession) -> tuple[dict[str, float], dict[str, float]]:
    scales: dict[str, float] = {}
    offsets: dict[str, float] = {}
    for motor in ("M1", "M2", "M3", "M4"):
        scales[motor] = float(session.get_param(MOTOR_TRIM_SCALE_FIELDS[motor]).value)
        offsets[motor] = float(session.get_param(MOTOR_TRIM_OFFSET_FIELDS[motor]).value)
    return scales, offsets


def _build_trim_targets(
    duties: tuple[float, ...],
    motors: tuple[int, ...],
    scales: dict[str, float],
    offsets: dict[str, float],
) -> list[dict[str, object]]:
    targets: list[dict[str, object]] = []
    for duty in duties:
        for motor in motors:
            label = _motor_label(motor)
            scale = scales[label]
            offset = offsets[label]
            targets.append(
                {
                    "motor": label,
                    "duty": duty,
                    "input_duty": duty,
                    "trim_scale": scale,
                    "trim_offset": offset,
                    "trim_target_duty": _trim_target_duty(duty, scale, offset),
                }
            )
    return targets


def _disarm_and_wait(session: DeviceSession, wait_s: float = 0.05) -> None:
    """Send disarm and wait for the firmware control loop to process it."""

    try:
        session.disarm()
    except Exception:
        pass
    time.sleep(wait_s)


def _motor_test_with_retry(
    session: DeviceSession,
    motor_index: int,
    duty: float,
    retries: int = 3,
) -> int:
    """Send motor-test with retries on DISARM_REQUIRED."""

    for attempt in range(retries):
        status = session.motor_test(motor_index, duty)
        if status == CmdStatus.OK:
            return status
        if status == CmdStatus.DISARM_REQUIRED and attempt < retries - 1:
            _disarm_and_wait(session)
            continue
        ensure_command_ok(CmdId.MOTOR_TEST, status)
    return status


def summarize_motor_balance_trials(trials: list[MotorBalanceTrial]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for trial in trials:
        samples = trial.samples
        gyro_mag = [_sample_gyro_mag(sample) for sample in samples]
        acc_mag = [_sample_acc_mag(sample) for sample in samples]
        acc_dynamic = [value - _mean(acc_mag) for value in acc_mag]
        gyro_rms = _rms(gyro_mag)
        acc_std = _std(acc_mag)
        response_score = gyro_rms + acc_std * 100.0
        rows.append(
            {
                "trial_id": trial.trial_id,
                "motor": f"M{trial.motor}",
                "duty": trial.duty,
                "input_duty": trial.duty,
                "trim_scale": trial.trim_scale,
                "trim_offset": trial.trim_offset,
                "trim_target_duty": trial.trim_target_duty,
                "sample_count": len(samples),
                "battery_min_v": min((float(sample.battery_voltage) for sample in samples), default=0.0),
                "battery_mean_v": _mean([float(sample.battery_voltage) for sample in samples]),
                "gyro_rms_dps": gyro_rms,
                "gyro_peak_dps": max(gyro_mag, default=0.0),
                "gyro_x_mean_dps": _mean([float(sample.gyro_x) for sample in samples]),
                "gyro_y_mean_dps": _mean([float(sample.gyro_y) for sample in samples]),
                "gyro_z_mean_dps": _mean([float(sample.gyro_z) for sample in samples]),
                "acc_rms_g": _rms(acc_dynamic),
                "acc_std_g": acc_std,
                "response_score": response_score,
                "relative_to_duty_mean": 1.0,
                "classification": "n/a",
            }
        )

    for duty in sorted({float(row["duty"]) for row in rows}):
        duty_rows = [row for row in rows if float(row["duty"]) == duty]
        mean_score = _mean([float(row["response_score"]) for row in duty_rows])
        for row in duty_rows:
            relative = float(row["response_score"]) / mean_score if mean_score > 0.0 else 1.0
            row["relative_to_duty_mean"] = relative
            if int(row["sample_count"]) == 0:
                row["classification"] = "no_samples"
            elif relative < 0.75:
                row["classification"] = "weak_response"
            elif relative > 1.25:
                row["classification"] = "strong_response"
            else:
                row["classification"] = "normal"
    return rows


def write_motor_balance_summary(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=MOTOR_BALANCE_SUMMARY_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    key: (
                        f"{float(row[key]):.6f}"
                        if isinstance(row.get(key), float)
                        else row.get(key)
                    )
                    for key in MOTOR_BALANCE_SUMMARY_FIELDS
                }
            )


def estimate_motor_trim_from_summary(
    path: Path,
    *,
    reference_motor: str = "M1",
    max_adjust: float = 0.10,
) -> MotorTrimEstimate:
    rows: list[dict[str, str]] = []
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows.extend(reader)

    scores_by_duty_motor: dict[tuple[float, str], float] = {}
    for row in rows:
        try:
            duty = float(row["duty"])
            motor = str(row["motor"]).upper()
            score = float(row["response_score"])
            samples = int(float(row.get("sample_count", "0")))
        except (KeyError, ValueError):
            continue
        if samples <= 0 or score <= 0.0:
            continue
        scores_by_duty_motor[(duty, motor)] = score

    low = 1.0 - max_adjust
    high = 1.0 + max_adjust
    scales: dict[str, float] = {motor: 1.0 for motor in MOTOR_TRIM_SCALE_FIELDS}
    offsets: dict[str, float] = {motor: 0.0 for motor in MOTOR_TRIM_SCALE_FIELDS}
    ratios: dict[str, float] = {motor: 1.0 for motor in MOTOR_TRIM_SCALE_FIELDS}

    for motor in MOTOR_TRIM_SCALE_FIELDS:
        if motor == reference_motor:
            continue
        raw_ratios: list[float] = []
        scale_votes: list[float] = []
        for (duty, score_motor), score in scores_by_duty_motor.items():
            if score_motor != motor:
                continue
            reference_score = scores_by_duty_motor.get((duty, reference_motor))
            if reference_score is None or reference_score <= 0.0:
                continue
            response_ratio = score / reference_score
            raw_ratios.append(response_ratio)
            scale_votes.append(_clamp_float(math.sqrt(reference_score / score), low, high))
        if scale_votes:
            scales[motor] = statistics.median(scale_votes)
            ratios[motor] = statistics.median(raw_ratios)

    return MotorTrimEstimate(source_path=path, scales=scales, offsets=offsets, ratios=ratios)


def apply_motor_trim_estimate(session: DeviceSession, estimate: MotorTrimEstimate) -> None:
    for motor, scale in estimate.scales.items():
        session.set_param(MOTOR_TRIM_SCALE_FIELDS[motor], 4, float(scale))
    for motor, offset in estimate.offsets.items():
        session.set_param(MOTOR_TRIM_OFFSET_FIELDS[motor], 4, float(offset))
    estimate.readback_scales = {
        motor: float(session.get_param(MOTOR_TRIM_SCALE_FIELDS[motor]).value)
        for motor in ("M1", "M2", "M3", "M4")
    }
    estimate.readback_offsets = {
        motor: float(session.get_param(MOTOR_TRIM_OFFSET_FIELDS[motor]).value)
        for motor in ("M1", "M2", "M3", "M4")
    }
    estimate.applied = True


def format_motor_trim_estimate(estimate: MotorTrimEstimate) -> list[str]:
    lines = [
        f"source={estimate.source_path}",
        f"applied={estimate.applied}",
        f"score_note={MOTOR_BALANCE_SCORE_NOTE}",
    ]
    for motor in ("M1", "M2", "M3", "M4"):
        lines.append(
            f"{motor} ratio_to_M1={estimate.ratios[motor]:.3f} "
            f"scale={estimate.scales[motor]:.4f} offset={estimate.offsets[motor]:.4f}"
        )
    if estimate.applied:
        lines.append("write_confirm:")
        for motor in ("M1", "M2", "M3", "M4"):
            scale_param = MOTOR_TRIM_SCALE_FIELDS[motor]
            offset_param = MOTOR_TRIM_OFFSET_FIELDS[motor]
            scale_readback = estimate.readback_scales.get(motor, float("nan"))
            offset_readback = estimate.readback_offsets.get(motor, float("nan"))
            lines.append(
                f"{motor} scale_param={scale_param} written={estimate.scales[motor]:.6f} "
                f"readback={scale_readback:.6f} offset_param={offset_param} "
                f"written={estimate.offsets[motor]:.6f} readback={offset_readback:.6f}"
            )
    serial = estimate.serial_port or "<PORT>"
    lines.append("powershell_set_commands:")
    for motor in ("M1", "M2", "M3", "M4"):
        lines.append(f"python -m esp_drone_cli.cli.main --serial {serial} set {MOTOR_TRIM_SCALE_FIELDS[motor]} float {estimate.scales[motor]:.6f}")
    for motor in ("M1", "M2", "M3", "M4"):
        lines.append(f"python -m esp_drone_cli.cli.main --serial {serial} set {MOTOR_TRIM_OFFSET_FIELDS[motor]} float {estimate.offsets[motor]:.6f}")
    return lines


def format_motor_balance_summary(result: MotorBalanceResult) -> list[str]:
    lines = [
        f"csv={result.csv_path}",
        f"summary_csv={result.summary_path}",
        f"stop_reason={result.stop_reason}",
        f"completed_trials={result.completed_trials} expected_trials={result.expected_trials}",
        f"summary_incomplete={result.summary_incomplete}",
        f"trim_applied={result.trim_applied}",
        f"trim_mode={result.trim_mode}",
        f"trim_path={MOTOR_BALANCE_TRIM_PATH}",
        f"trim_params={MOTOR_BALANCE_TRIM_PARAMS}",
        f"score_note={MOTOR_BALANCE_SCORE_NOTE}",
    ]
    if result.trim_targets:
        lines.append("trim_targets:")
        for duty in sorted({float(row["duty"]) for row in result.trim_targets}):
            duty_targets = [row for row in result.trim_targets if float(row["duty"]) == duty]
            duty_targets.sort(key=lambda row: int(str(row["motor"])[1:]))
            parts = [f"duty={duty:.3f}"]
            for row in duty_targets:
                parts.append(
                    "{motor} input={input_duty:.3f} trim_target={trim_target_duty:.4f} "
                    "scale={trim_scale:.4f} offset={trim_offset:.4f}".format(**row)
                )
            lines.append(" ".join(parts))
    for row in result.trials:
        lines.append(
            "trial={trial_id} motor={motor} input_duty={input_duty:.3f} "
            "trim_target_duty={trim_target_duty:.4f} scale={trim_scale:.4f} "
            "offset={trim_offset:.4f} samples={sample_count} "
            "battery_min={battery_min_v:.3f} gyro_rms={gyro_rms_dps:.2f} "
            "gyro_peak={gyro_peak_dps:.2f} acc_std={acc_std_g:.4f} "
            "score={response_score:.2f} rel={relative_to_duty_mean:.2f} class={classification}".format(
                **row
            )
        )
    if result.summary_incomplete or result.stop_reason != "completed":
        lines.append("weak_candidates=unknown_incomplete")
    elif result.weak_candidates:
        lines.append("weak_candidates=" + ",".join(result.weak_candidates))
    else:
        lines.append("weak_candidates=none")
    reason_lower = result.stop_reason.lower()
    if "timeout" in reason_lower or "timed out" in reason_lower:
        lines.append(
            "timeout_suggestions=reduce telemetry-hz, split by duty, increase rest-s, "
            "check USB cable / serial reliability"
        )
    return lines


def run_motor_thrust_balance(
    session: DeviceSession,
    options: MotorBalanceOptions,
    *,
    progress: Callable[[str], None] | None = None,
) -> MotorBalanceResult:
    options.validate()
    expected_trials = len(options.duties) * len(options.motors)
    options.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = options.csv_path()
    summary_path = options.summary_path(csv_path)
    phase_state: dict[str, object] = {
        "trial_id": 0,
        "motor": "",
        "duty": 0.0,
        "input_duty": 0.0,
        "trim_scale": 1.0,
        "trim_offset": 0.0,
        "trim_target_duty": 0.0,
        "phase": "idle",
    }
    trials: list[MotorBalanceTrial] = []
    trim_targets: list[dict[str, object]] = []
    trim_mode = "motor_scale/motor_offset active"
    current_samples: list[TelemetrySample] = []
    originals: dict[str, ParamValue] = {}
    started_stream = False
    started_log = False
    telemetry_token: int | None = None

    def emit(message: str) -> None:
        if progress is not None:
            progress(message)

    def remember_param(name: str) -> ParamValue | None:
        try:
            value = session.get_param(name)
        except Exception:
            return None
        originals[name] = value
        return value

    def set_and_track_param(name: str, type_id: int, value: object) -> None:
        if name not in originals:
            remember_param(name)
        session.set_param(name, type_id, value)

    def restore_params() -> None:
        for name, value in originals.items():
            try:
                session.set_param(name, value.type_id, value.value)
            except Exception:
                continue

    def on_telemetry(sample: TelemetrySample) -> None:
        if phase_state["phase"] == "active":
            current_samples.append(sample)

    telemetry_token = session.subscribe_telemetry(on_telemetry)
    try:
        trim_scales, trim_offsets = _read_trim_settings(session)
        if not options.use_trim:
            for motor in ("M1", "M2", "M3", "M4"):
                set_and_track_param(MOTOR_TRIM_SCALE_FIELDS[motor], 4, 1.0)
                set_and_track_param(MOTOR_TRIM_OFFSET_FIELDS[motor], 4, 0.0)
            trim_scales = {motor: 1.0 for motor in MOTOR_TRIM_SCALE_FIELDS}
            trim_offsets = {motor: 0.0 for motor in MOTOR_TRIM_OFFSET_FIELDS}
            trim_mode = "no-trim: motor_scale=1.0 motor_offset=0.0 temporary"
        trim_targets = _build_trim_targets(options.duties, options.motors, trim_scales, trim_offsets)

        set_and_track_param("telemetry_usb_hz", 2, options.telemetry_hz)
        session.start_csv_log(
            csv_path,
            fieldnames=MOTOR_BALANCE_CSV_FIELDS,
            extra_row_fn=lambda _sample: {
                "test_trial_id": phase_state["trial_id"],
                "test_motor": phase_state["motor"],
                "test_duty": phase_state["duty"],
                "test_input_duty": phase_state["input_duty"],
                "test_trim_scale": phase_state["trim_scale"],
                "test_trim_offset": phase_state["trim_offset"],
                "test_trim_target_duty": phase_state["trim_target_duty"],
                "test_phase": phase_state["phase"],
            },
        )
        started_log = True
        session.start_stream()
        started_stream = True
        _disarm_and_wait(session)

        trial_id = 0
        for duty in options.duties:
            for motor in options.motors:
                trial_id += 1
                motor_label = _motor_label(motor)
                trim_scale = trim_scales[motor_label]
                trim_offset = trim_offsets[motor_label]
                trim_target_duty = _trim_target_duty(duty, trim_scale, trim_offset)
                current_samples = []
                phase_state.update(
                    {
                        "trial_id": trial_id,
                        "motor": motor_label,
                        "duty": duty,
                        "input_duty": duty,
                        "trim_scale": trim_scale,
                        "trim_offset": trim_offset,
                        "trim_target_duty": trim_target_duty,
                        "phase": "settle",
                    }
                )
                emit(
                    f"trial {trial_id}: {motor_label} input_duty={duty:.3f} "
                    f"trim_target_duty={trim_target_duty:.4f} "
                    f"scale={trim_scale:.4f} offset={trim_offset:.4f}"
                )
                _motor_test_with_retry(session, motor - 1, duty)
                if options.settle_s > 0.0:
                    time.sleep(options.settle_s)
                phase_state["phase"] = "active"
                time.sleep(options.duration_s)
                session.motor_test(motor - 1, 0.0)
                phase_state["phase"] = "idle"
                trials.append(
                    MotorBalanceTrial(
                        trial_id,
                        motor,
                        duty,
                        trim_scale,
                        trim_offset,
                        trim_target_duty,
                        list(current_samples),
                    )
                )
                _disarm_and_wait(session)
                if options.rest_s > 0.0:
                    time.sleep(options.rest_s)

        summary_rows = summarize_motor_balance_trials(trials)
        write_motor_balance_summary(summary_path, summary_rows)
        weak_candidates = sorted(
            {
                str(row["motor"])
                for row in summary_rows
                if row["classification"] == "weak_response"
            }
        )
        return MotorBalanceResult(
            csv_path=csv_path,
            summary_path=summary_path,
            trials=summary_rows,
            weak_candidates=weak_candidates,
            trim_targets=trim_targets,
            trim_applied=options.use_trim,
            trim_mode=trim_mode,
            return_code=0,
            stop_reason="completed",
            completed_trials=len(trials),
            expected_trials=expected_trials,
            summary_incomplete=False,
        )
    except KeyboardInterrupt:
        return MotorBalanceResult(
            csv_path=csv_path,
            summary_path=summary_path,
            trials=summarize_motor_balance_trials(trials),
            weak_candidates=["unknown_incomplete"],
            trim_targets=trim_targets,
            trim_applied=options.use_trim,
            trim_mode=trim_mode,
            return_code=130,
            stop_reason="keyboard_interrupt",
            completed_trials=len(trials),
            expected_trials=expected_trials,
            summary_incomplete=True,
        )
    except Exception as exc:
        return MotorBalanceResult(
            csv_path=csv_path,
            summary_path=summary_path,
            trials=summarize_motor_balance_trials(trials),
            weak_candidates=["unknown_incomplete"],
            trim_targets=trim_targets,
            trim_applied=options.use_trim,
            trim_mode=trim_mode,
            return_code=1,
            stop_reason=f"exception:{exc}",
            completed_trials=len(trials),
            expected_trials=expected_trials,
            summary_incomplete=True,
        )
    finally:
        phase_state["phase"] = "stopping"
        for motor in range(4):
            try:
                session.motor_test(motor, 0.0)
            except Exception:
                pass
        try:
            session.disarm()
        except Exception:
            pass
        if started_stream:
            try:
                session.stop_stream()
            except Exception:
                pass
        if started_log:
            try:
                session.stop_csv_log()
            except Exception:
                pass
        try:
            restore_params()
        except Exception:
            pass
        if telemetry_token is not None:
            try:
                session.unsubscribe(telemetry_token)
            except Exception:
                pass
