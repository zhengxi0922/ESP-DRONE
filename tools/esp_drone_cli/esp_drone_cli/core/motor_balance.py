from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from pathlib import Path
import statistics
import time
from typing import Callable

from .device_session import DeviceSession
from .models import TELEMETRY_CSV_FIELDS, ParamValue, TelemetrySample
from .protocol.messages import CmdId, ensure_command_ok


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
    "test_phase",
    *TELEMETRY_CSV_FIELDS,
]

MOTOR_BALANCE_SUMMARY_FIELDS = [
    "trial_id",
    "motor",
    "duty",
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
    "M1": "motor_trim_scale_m1",
    "M2": "motor_trim_scale_m2",
    "M3": "motor_trim_scale_m3",
    "M4": "motor_trim_scale_m4",
}
MOTOR_TRIM_OFFSET_FIELDS = {
    "M1": "motor_trim_offset_m1",
    "M2": "motor_trim_offset_m2",
    "M3": "motor_trim_offset_m3",
    "M4": "motor_trim_offset_m4",
}


@dataclass(slots=True)
class MotorBalanceOptions:
    duties: tuple[float, ...] = MOTOR_BALANCE_DEFAULT_DUTIES
    duration_s: float = MOTOR_BALANCE_DEFAULT_DURATION_S
    rest_s: float = MOTOR_BALANCE_DEFAULT_REST_S
    telemetry_hz: int = MOTOR_BALANCE_DEFAULT_TELEMETRY_HZ
    output_dir: Path = Path("logs")
    motors: tuple[int, ...] = (1, 2, 3, 4)

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
    samples: list[TelemetrySample]


@dataclass(slots=True)
class MotorBalanceResult:
    csv_path: Path
    summary_path: Path
    trials: list[dict[str, object]]
    weak_candidates: list[str]
    return_code: int = 0
    stop_reason: str = "completed"


@dataclass(slots=True)
class MotorTrimEstimate:
    source_path: Path
    scales: dict[str, float]
    offsets: dict[str, float]
    ratios: dict[str, float]
    applied: bool = False


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
    estimate.applied = True


def format_motor_trim_estimate(estimate: MotorTrimEstimate) -> list[str]:
    lines = [
        f"source={estimate.source_path}",
        f"applied={estimate.applied}",
    ]
    for motor in ("M1", "M2", "M3", "M4"):
        lines.append(
            f"{motor} ratio_to_M1={estimate.ratios[motor]:.3f} "
            f"scale={estimate.scales[motor]:.4f} offset={estimate.offsets[motor]:.4f}"
        )
    lines.append("powershell_set_commands:")
    for motor in ("M1", "M2", "M3", "M4"):
        lines.append(f"python -m esp_drone_cli.cli.main --serial COM4 set {MOTOR_TRIM_SCALE_FIELDS[motor]} float {estimate.scales[motor]:.6f}")
    for motor in ("M1", "M2", "M3", "M4"):
        lines.append(f"python -m esp_drone_cli.cli.main --serial COM4 set {MOTOR_TRIM_OFFSET_FIELDS[motor]} float {estimate.offsets[motor]:.6f}")
    return lines


def format_motor_balance_summary(result: MotorBalanceResult) -> list[str]:
    lines = [
        f"csv={result.csv_path}",
        f"summary_csv={result.summary_path}",
        f"stop_reason={result.stop_reason}",
    ]
    for row in result.trials:
        lines.append(
            "trial={trial_id} motor={motor} duty={duty:.2f} samples={sample_count} "
            "battery_min={battery_min_v:.3f} gyro_rms={gyro_rms_dps:.2f} "
            "gyro_peak={gyro_peak_dps:.2f} acc_std={acc_std_g:.4f} "
            "score={response_score:.2f} rel={relative_to_duty_mean:.2f} class={classification}".format(
                **row
            )
        )
    if result.weak_candidates:
        lines.append("weak_candidates=" + ",".join(result.weak_candidates))
    else:
        lines.append("weak_candidates=none")
    return lines


def run_motor_thrust_balance(
    session: DeviceSession,
    options: MotorBalanceOptions,
    *,
    progress: Callable[[str], None] | None = None,
) -> MotorBalanceResult:
    options.validate()
    options.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = options.csv_path()
    summary_path = options.summary_path(csv_path)
    phase_state: dict[str, object] = {
        "trial_id": 0,
        "motor": "",
        "duty": 0.0,
        "phase": "idle",
    }
    trials: list[MotorBalanceTrial] = []
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
        set_and_track_param("telemetry_usb_hz", 2, options.telemetry_hz)
        session.start_csv_log(
            csv_path,
            fieldnames=MOTOR_BALANCE_CSV_FIELDS,
            extra_row_fn=lambda _sample: {
                "test_trial_id": phase_state["trial_id"],
                "test_motor": phase_state["motor"],
                "test_duty": phase_state["duty"],
                "test_phase": phase_state["phase"],
            },
        )
        started_log = True
        session.start_stream()
        started_stream = True
        try:
            ensure_command_ok(CmdId.DISARM, session.disarm())
        except Exception:
            session.disarm()

        trial_id = 0
        for duty in options.duties:
            for motor in options.motors:
                trial_id += 1
                current_samples = []
                phase_state.update(
                    {
                        "trial_id": trial_id,
                        "motor": f"M{motor}",
                        "duty": duty,
                        "phase": "active",
                    }
                )
                emit(f"trial {trial_id}: M{motor} duty={duty:.2f}")
                ensure_command_ok(CmdId.MOTOR_TEST, session.motor_test(motor - 1, duty))
                time.sleep(options.duration_s)
                ensure_command_ok(CmdId.MOTOR_TEST, session.motor_test(motor - 1, 0.0))
                phase_state["phase"] = "idle"
                trials.append(MotorBalanceTrial(trial_id, motor, duty, list(current_samples)))
                try:
                    ensure_command_ok(CmdId.DISARM, session.disarm())
                except Exception:
                    session.disarm()
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
            return_code=0,
            stop_reason="completed",
        )
    except KeyboardInterrupt:
        return MotorBalanceResult(
            csv_path=csv_path,
            summary_path=summary_path,
            trials=summarize_motor_balance_trials(trials),
            weak_candidates=[],
            return_code=130,
            stop_reason="keyboard_interrupt",
        )
    except Exception as exc:
        return MotorBalanceResult(
            csv_path=csv_path,
            summary_path=summary_path,
            trials=summarize_motor_balance_trials(trials),
            weak_candidates=[],
            return_code=1,
            stop_reason=f"exception:{exc}",
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
