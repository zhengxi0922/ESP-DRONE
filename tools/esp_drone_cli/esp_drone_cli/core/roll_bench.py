"""Bench-only helpers for constrained single-axis rate tuning."""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from statistics import fmean

from .device_session import DeviceSession
from .models import TelemetrySample
from .protocol.messages import CmdId, ensure_command_ok


ARM_STATE_DISARMED = 0
ARM_STATE_ARMED = 1

AXIS_INDEX = {"roll": 0, "pitch": 1, "yaw": 2}


@dataclass(frozen=True, slots=True)
class AxisBenchSpec:
    axis_name: str
    feedback_desc: str
    positive_motor_desc: str


AXIS_BENCH_SPECS = {
    "roll": AxisBenchSpec(
        axis_name="roll",
        feedback_desc="-gyro_y",
        positive_motor_desc="M1/M4 increase, M2/M3 decrease",
    ),
    "pitch": AxisBenchSpec(
        axis_name="pitch",
        feedback_desc="gyro_x",
        positive_motor_desc="M3/M4 increase, M1/M2 decrease",
    ),
    "yaw": AxisBenchSpec(
        axis_name="yaw",
        feedback_desc="-gyro_z",
        positive_motor_desc="M1/M3 increase, M2/M4 decrease",
    ),
}

ROLL_BENCH_ORIENTATION_NOTE = (
    "Current bench orientation is sensor +Z down (inverted / upside-down bench). "
    "Evaluation is rate-only and uses rate_setpoint_roll, mapped roll feedback = -gyro_y, "
    "rate_pid_p/i/d_roll, pid_out_roll, and the +roll motor split (M1/M4 increase, M2/M3 decrease). "
    "Attitude angle is not a pass criterion."
)


class RollBenchSafetyTrip(RuntimeError):
    """Raised when live telemetry indicates bench risk."""


@dataclass(slots=True)
class RollBenchStep:
    """One conservative single-axis rate bench step."""

    name: str
    command_dps: float
    duration_s: float
    role: str


@dataclass(slots=True)
class RollBenchStepMetrics:
    """Aggregated metrics for one step window."""

    name: str
    role: str
    command_dps: float
    duration_s: float
    sample_count: int
    setpoint_mean: float
    feedback_mean: float
    feedback_peak_abs: float
    pid_out_mean: float
    pid_out_peak_abs: float
    pid_out_saturation_ratio: float
    motor_split_mean: float
    motor_split_peak_abs: float
    motor_low_clip_ratio: float
    motor_high_clip_ratio: float
    motors_mean: tuple[float, float, float, float]
    setpoint_path_ok: bool
    sign_ok: bool | None
    measurable_response: bool | None
    saturation_warning: bool
    return_to_zero_ok: bool | None


@dataclass(slots=True)
class RollBenchSummary:
    """Round-level evaluation summary."""

    setpoint_path_ok: bool
    sign_ok: bool
    motor_split_ok: bool
    measurable_response: bool
    saturation_risk: bool
    return_to_zero_warning: bool
    return_to_zero_quality: str
    noise_or_jitter_risk: bool
    low_duty_motor_stability: str
    safe_to_continue: bool
    kp_tuning_allowed: bool
    accept: bool
    axis_result: str
    next_action_hint: str


@dataclass(slots=True)
class RollBenchRoundResult:
    """Persisted result for one bench round."""

    axis_name: str
    run_id: str
    serial_hint: str | None
    started_local: str
    orientation_note: str
    params: dict[str, float]
    steps: list[RollBenchStep]
    metrics: list[RollBenchStepMetrics]
    summary: RollBenchSummary
    csv_path: str
    json_path: str
    markdown_path: str
    telemetry_samples: int


def build_default_roll_step_plan(
    small_step_dps: float = 10.0,
    large_step_dps: float = 20.0,
    active_duration_s: float = 0.8,
    zero_duration_s: float = 0.7,
) -> list[RollBenchStep]:
    """Build a conservative default step sequence."""

    return [
        RollBenchStep("pre_zero", 0.0, zero_duration_s, "zero"),
        RollBenchStep("pos_small", +small_step_dps, active_duration_s, "command"),
        RollBenchStep("zero_after_pos_small", 0.0, zero_duration_s, "zero"),
        RollBenchStep("neg_small", -small_step_dps, active_duration_s, "command"),
        RollBenchStep("zero_after_neg_small", 0.0, zero_duration_s, "zero"),
        RollBenchStep("pos_large", +large_step_dps, active_duration_s, "command"),
        RollBenchStep("zero_after_pos_large", 0.0, zero_duration_s, "zero"),
        RollBenchStep("neg_large", -large_step_dps, active_duration_s, "command"),
        RollBenchStep("post_zero", 0.0, zero_duration_s, "zero"),
    ]


def _axis_bench_orientation_note(axis_name: str, bench_desc: str) -> str:
    spec = _bench_spec(axis_name)
    return (
        f"Current bench orientation is {bench_desc}. "
        f"Evaluation is rate-only and uses rate_setpoint_{axis_name}, mapped {axis_name} feedback = {spec.feedback_desc}, "
        f"rate_pid_p/i/d_{axis_name}, pid_out_{axis_name}, and the +{axis_name} motor split ({spec.positive_motor_desc}). "
        "Attitude angle is not a pass criterion."
    )


def _bench_spec(axis_name: str) -> AxisBenchSpec:
    if axis_name not in AXIS_BENCH_SPECS:
        raise ValueError(f"unsupported axis {axis_name}")
    return AXIS_BENCH_SPECS[axis_name]


def _axis_param_names(axis_name: str) -> tuple[str, ...]:
    return (
        f"rate_kp_{axis_name}",
        f"rate_ki_{axis_name}",
        f"rate_kd_{axis_name}",
        "rate_integral_limit",
        "rate_output_limit",
        "bringup_test_base_duty",
        "motor_idle_duty",
        "motor_max_duty",
        "telemetry_usb_hz",
    )


def _mean(values: list[float]) -> float:
    return float(fmean(values)) if values else 0.0


def _motor_outputs(sample: TelemetrySample) -> tuple[float, float, float, float]:
    return (float(sample.motor1), float(sample.motor2), float(sample.motor3), float(sample.motor4))


def _motor_split(sample: TelemetrySample, axis_name: str) -> float:
    if axis_name == "roll":
        return ((sample.motor1 + sample.motor4) - (sample.motor2 + sample.motor3)) * 0.5
    if axis_name == "pitch":
        return ((sample.motor3 + sample.motor4) - (sample.motor1 + sample.motor2)) * 0.5
    if axis_name == "yaw":
        return ((sample.motor1 + sample.motor3) - (sample.motor2 + sample.motor4)) * 0.5
    raise ValueError(f"unsupported axis {axis_name}")


def _clip_ratio(values: list[float], *, low: bool) -> float:
    if not values:
        return 0.0
    if low:
        hits = sum(1 for value in values if value <= 0.005)
    else:
        hits = sum(1 for value in values if value >= 0.95)
    return hits / len(values)


def _analysis_samples(samples: list[TelemetrySample], role: str) -> list[TelemetrySample]:
    if len(samples) <= 2:
        return samples
    if role == "command":
        return samples[len(samples) // 4 :]
    return samples[len(samples) // 2 :]


def _feedback_signed_peak(samples: list[TelemetrySample], axis_name: str, command_dps: float) -> tuple[float, float]:
    feedback_values = [sample.axis_rate_feedback_dps(axis_name) for sample in samples]
    if not feedback_values:
        return 0.0, 0.0
    if command_dps > 0.0:
        return max(feedback_values), abs(min(feedback_values))
    if command_dps < 0.0:
        return abs(min(feedback_values)), max(feedback_values)
    return 0.0, 0.0


def analyze_axis_bench_round(
    step_windows: list[tuple[RollBenchStep, list[TelemetrySample]]],
    params: dict[str, float],
    *,
    axis_name: str,
) -> tuple[list[RollBenchStepMetrics], RollBenchSummary]:
    """Analyze one single-axis bench round."""

    output_limit = float(params.get("rate_output_limit", 0.0))
    metrics: list[RollBenchStepMetrics] = []
    previous_nonzero_command = 0.0

    for step, raw_samples in step_windows:
        samples = _analysis_samples(raw_samples, step.role)
        setpoints = [float(getattr(sample, f"rate_setpoint_{axis_name}")) for sample in samples]
        feedbacks = [sample.axis_rate_feedback_dps(axis_name) for sample in samples]
        pid_outs = [float(getattr(sample, f"pid_out_{axis_name}")) for sample in samples]
        splits = [_motor_split(sample, axis_name) for sample in samples]
        motors = [_motor_outputs(sample) for sample in samples]
        motor1 = [values[0] for values in motors]
        motor2 = [values[1] for values in motors]
        motor3 = [values[2] for values in motors]
        motor4 = [values[3] for values in motors]
        pid_sat_ratio = (
            sum(1 for value in pid_outs if abs(value) >= (0.95 * output_limit)) / len(pid_outs)
            if pid_outs and output_limit > 0.0
            else 0.0
        )
        low_clip_ratio = _clip_ratio([value for values in motors for value in values], low=True)
        high_clip_ratio = _clip_ratio([value for values in motors for value in values], low=False)
        setpoint_mean = _mean(setpoints)
        feedback_mean = _mean(feedbacks)
        feedback_peak_abs = max((abs(value) for value in feedbacks), default=0.0)
        pid_out_mean = _mean(pid_outs)
        pid_out_peak_abs = max((abs(value) for value in pid_outs), default=0.0)
        split_mean = _mean(splits)
        split_peak_abs = max((abs(value) for value in splits), default=0.0)
        motors_mean = (_mean(motor1), _mean(motor2), _mean(motor3), _mean(motor4))

        if abs(step.command_dps) > 0.0:
            min_setpoint = max(1.0, abs(step.command_dps) * 0.8)
            setpoint_path_ok = setpoint_mean * step.command_dps > 0.0 and abs(setpoint_mean) >= min_setpoint
            signed_peak, _ = _feedback_signed_peak(samples, axis_name, step.command_dps)
            response_threshold = max(1.0, abs(step.command_dps) * 0.08)
            measurable_response = signed_peak >= response_threshold
            feedback_sign_ok = signed_peak >= response_threshold
            sign_ok = (
                setpoint_path_ok
                and (pid_out_mean * step.command_dps) > 0.002
                and (split_mean * step.command_dps) > 0.002
                and feedback_sign_ok
            )
            saturation_warning = (
                pid_sat_ratio >= 0.35
                or low_clip_ratio >= 0.45
                or high_clip_ratio >= 0.10
            )
            return_to_zero_ok = None
            previous_nonzero_command = step.command_dps
        else:
            setpoint_path_ok = abs(setpoint_mean) <= 0.5
            measurable_response = None
            sign_ok = None
            saturation_warning = pid_sat_ratio >= 0.35
            if previous_nonzero_command == 0.0 or not feedbacks:
                return_to_zero_ok = None
            else:
                zero_threshold = max(4.0, abs(previous_nonzero_command) * 0.25)
                return_to_zero_ok = _mean([abs(value) for value in feedbacks]) <= zero_threshold

        metrics.append(
            RollBenchStepMetrics(
                name=step.name,
                role=step.role,
                command_dps=step.command_dps,
                duration_s=step.duration_s,
                sample_count=len(samples),
                setpoint_mean=setpoint_mean,
                feedback_mean=feedback_mean,
                feedback_peak_abs=feedback_peak_abs,
                pid_out_mean=pid_out_mean,
                pid_out_peak_abs=pid_out_peak_abs,
                pid_out_saturation_ratio=pid_sat_ratio,
                motor_split_mean=split_mean,
                motor_split_peak_abs=split_peak_abs,
                motor_low_clip_ratio=low_clip_ratio,
                motor_high_clip_ratio=high_clip_ratio,
                motors_mean=motors_mean,
                setpoint_path_ok=setpoint_path_ok,
                sign_ok=sign_ok,
                measurable_response=measurable_response,
                saturation_warning=saturation_warning,
                return_to_zero_ok=return_to_zero_ok,
            )
        )

    command_metrics = [item for item in metrics if item.role == "command"]
    zero_metrics = [item for item in metrics if item.role == "zero" and item.return_to_zero_ok is not None]

    setpoint_path_ok = all(item.setpoint_path_ok for item in command_metrics)
    sign_ok = all(item.sign_ok is True for item in command_metrics)
    motor_split_ok = all((item.motor_split_mean * item.command_dps) > 0.002 for item in command_metrics)
    measurable_response = all(item.measurable_response is True for item in command_metrics)
    saturation_risk = any(item.saturation_warning for item in command_metrics)
    return_to_zero_warning = any(item.return_to_zero_ok is False for item in zero_metrics)
    worst_zero_peak_abs = max((item.feedback_peak_abs for item in zero_metrics), default=0.0)
    if not return_to_zero_warning:
        return_to_zero_quality = "PASS"
    elif worst_zero_peak_abs >= 6.0:
        return_to_zero_quality = "FAIL"
    else:
        return_to_zero_quality = "PASS_WITH_WARNING"

    noise_or_jitter_risk = any(
        (
            item.feedback_peak_abs >= max(abs(item.command_dps) * 0.9, abs(item.feedback_mean) * 12.0, 8.0)
            or (
                abs(item.pid_out_mean) >= 0.01
                and item.pid_out_peak_abs >= max(abs(item.pid_out_mean) * 3.5, 0.05)
            )
        )
        for item in command_metrics
    )

    motor_idle_duty = float(params.get("motor_idle_duty", 0.0))
    max_abs_command = max((abs(item.command_dps) for item in command_metrics), default=0.0)
    low_duty_motor_stability = "PASS"
    for item in command_metrics:
        min_motor_mean = min(item.motors_mean)
        max_probe_step = max_abs_command > 0.0 and abs(item.command_dps) >= (0.99 * max_abs_command)
        hard_low_duty = item.motor_low_clip_ratio >= 0.15 or min_motor_mean <= 0.005
        soft_low_duty = item.motor_low_clip_ratio >= 0.05 or min_motor_mean < (motor_idle_duty + 0.002)

        if hard_low_duty and not max_probe_step:
            low_duty_motor_stability = "FAIL"
            break
        if hard_low_duty or soft_low_duty:
            low_duty_motor_stability = "PASS_WITH_WARNING"

    safe_to_continue = setpoint_path_ok and sign_ok and motor_split_ok and not saturation_risk
    kp_tuning_allowed = safe_to_continue
    accept = (
        kp_tuning_allowed
        and measurable_response
        and return_to_zero_quality != "FAIL"
        and low_duty_motor_stability != "FAIL"
    )

    if not accept:
        axis_result = "FAIL"
    elif (
        return_to_zero_quality != "PASS"
        or noise_or_jitter_risk
        or low_duty_motor_stability != "PASS"
    ):
        axis_result = "PASS_WITH_WARNING"
    else:
        axis_result = "PASS"

    if not setpoint_path_ok:
        next_action_hint = f"stop: rate_setpoint_{axis_name} did not follow the commanded step"
    elif not sign_ok:
        next_action_hint = f"stop: pid_out_{axis_name} or motor split sign disagrees with +{axis_name} expectation"
    elif not motor_split_ok:
        next_action_hint = f"stop: motor split does not match the +{axis_name} expectation"
    elif saturation_risk:
        next_action_hint = "stop: pid output or motor outputs spent too long near a limit"
    elif not measurable_response:
        next_action_hint = "continue: response is weak, consider a small Kp increase"
    elif return_to_zero_quality == "FAIL" and float(params.get(f"rate_ki_{axis_name}", 0.0)) > 0.0:
        next_action_hint = "stop: return-to-zero degraded after adding Ki, reduce Ki first"
    elif return_to_zero_quality == "FAIL":
        next_action_hint = "stop: return-to-zero remained too loose for the current bench window"
    elif return_to_zero_warning and float(params.get(f"rate_ki_{axis_name}", 0.0)) > 0.0:
        next_action_hint = "continue: return-to-zero is weak, consider reducing Ki"
    elif noise_or_jitter_risk and float(params.get(f"rate_kd_{axis_name}", 0.0)) > 0.0:
        next_action_hint = "warn: D-term noise risk increased, reduce Kd or add filtering before increasing gains"
    elif noise_or_jitter_risk:
        next_action_hint = "warn: response is usable but peak/jitter risk is rising on the constrained bench"
    elif low_duty_motor_stability == "FAIL":
        next_action_hint = "stop: motor outputs touched the low-duty floor before the maximum probe step"
    elif low_duty_motor_stability == "PASS_WITH_WARNING":
        next_action_hint = "warn: the largest probe step pushes one motor group close to the low-duty floor; keep gains conservative"
    elif return_to_zero_warning:
        next_action_hint = "warn: return-to-zero is soft on the constrained bench, but sign and split are usable"
    else:
        next_action_hint = "accept: sign, response, and saturation checks are bench-usable"

    summary = RollBenchSummary(
        setpoint_path_ok=setpoint_path_ok,
        sign_ok=sign_ok,
        motor_split_ok=motor_split_ok,
        measurable_response=measurable_response,
        saturation_risk=saturation_risk,
        return_to_zero_warning=return_to_zero_warning,
        return_to_zero_quality=return_to_zero_quality,
        noise_or_jitter_risk=noise_or_jitter_risk,
        low_duty_motor_stability=low_duty_motor_stability,
        safe_to_continue=safe_to_continue,
        kp_tuning_allowed=kp_tuning_allowed,
        accept=accept,
        axis_result=axis_result,
        next_action_hint=next_action_hint,
    )
    return metrics, summary


def analyze_roll_bench_round(
    step_windows: list[tuple[RollBenchStep, list[TelemetrySample]]],
    params: dict[str, float],
) -> tuple[list[RollBenchStepMetrics], RollBenchSummary]:
    return analyze_axis_bench_round(step_windows, params, axis_name="roll")


def _render_markdown(result: RollBenchRoundResult) -> str:
    axis_name = result.axis_name
    lines = [
        f"# {axis_name.capitalize()} Bench Round",
        "",
        f"- axis: `{axis_name}`",
        f"- run_id: `{result.run_id}`",
        f"- started_local: `{result.started_local}`",
        f"- serial_hint: `{result.serial_hint or 'unknown'}`",
        f"- orientation: {result.orientation_note}",
        "",
        "## Params",
        "",
    ]
    for name, value in result.params.items():
        lines.append(f"- `{name}` = `{value:.6f}`")
    lines.extend(
        [
            "",
            "## Summary",
            "",
            f"- setpoint_path_ok: `{result.summary.setpoint_path_ok}`",
            f"- sign_ok: `{result.summary.sign_ok}`",
            f"- motor_split_ok: `{result.summary.motor_split_ok}`",
            f"- measurable_response: `{result.summary.measurable_response}`",
            f"- saturation_risk: `{result.summary.saturation_risk}`",
            f"- return_to_zero_warning: `{result.summary.return_to_zero_warning}`",
            f"- return_to_zero_quality: `{result.summary.return_to_zero_quality}`",
            f"- noise_or_jitter_risk: `{result.summary.noise_or_jitter_risk}`",
            f"- low_duty_motor_stability: `{result.summary.low_duty_motor_stability}`",
            f"- safe_to_continue: `{result.summary.safe_to_continue}`",
            f"- kp_tuning_allowed: `{result.summary.kp_tuning_allowed}`",
            f"- accept: `{result.summary.accept}`",
            f"- axis_result: `{result.summary.axis_result}`",
            f"- next_action_hint: {result.summary.next_action_hint}",
            "",
            "## Steps",
            "",
            "| step | cmd_dps | samples | sp_mean | fb_mean | pid_out_mean | split_mean | sat_ratio | sign_ok | response | zero_ok |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---|---|---|",
        ]
    )
    for item in result.metrics:
        lines.append(
            f"| {item.name} | {item.command_dps:.1f} | {item.sample_count} | "
            f"{item.setpoint_mean:.2f} | {item.feedback_mean:.2f} | {item.pid_out_mean:.4f} | "
            f"{item.motor_split_mean:.4f} | {item.pid_out_saturation_ratio:.2f} | "
            f"{item.sign_ok} | {item.measurable_response} | {item.return_to_zero_ok} |"
        )
    lines.append("")
    return "\n".join(lines)


def _read_axis_params(session: DeviceSession, axis_name: str) -> dict[str, float]:
    params: dict[str, float] = {}
    for name in _axis_param_names(axis_name):
        params[name] = float(session.get_param(name).value)
    return params


def _read_roll_params(session: DeviceSession) -> dict[str, float]:
    return _read_axis_params(session, "roll")


def _format_state_detail(sample: TelemetrySample | None) -> str:
    if sample is None:
        return "arm_state=unknown failsafe_reason=unknown control_mode=unknown"
    return (
        f"arm_state={sample.arm_state} "
        f"failsafe_reason={sample.failsafe_reason} "
        f"control_mode={sample.control_mode}"
    )


def _send_rate_test_checked(
    session: DeviceSession,
    axis_index: int,
    value_dps: float,
    *,
    retries: int = 2,
) -> None:
    """Send a rate-test command, tolerating delayed/lost command responses."""

    last_timeout: TimeoutError | None = None
    for attempt in range(retries + 1):
        paused_stream = False
        try:
            if hasattr(session, "command"):
                try:
                    session.stop_stream(timeout=3.0)
                    paused_stream = True
                except Exception:
                    paused_stream = False
                status = session.command(CmdId.RATE_TEST, arg_u8=axis_index, arg_f32=value_dps, timeout=3.0)
            else:
                status = session.rate_test(axis_index, value_dps)
            ensure_command_ok(CmdId.RATE_TEST, status)
            return
        except TimeoutError as exc:
            last_timeout = exc
            if attempt >= retries:
                raise
            time.sleep(0.05)
        finally:
            if paused_stream:
                try:
                    session.start_stream(timeout=3.0)
                except Exception:
                    pass
    if last_timeout is not None:
        raise last_timeout


def _send_simple_command_checked(
    session: DeviceSession,
    cmd_id: int,
    fallback_send,
    *,
    retries: int = 2,
) -> None:
    """Send a no-argument command with a longer bench timeout."""

    last_timeout: TimeoutError | None = None
    for attempt in range(retries + 1):
        try:
            if hasattr(session, "command"):
                status = session.command(cmd_id, timeout=3.0)
            else:
                status = fallback_send()
            ensure_command_ok(cmd_id, status)
            return
        except TimeoutError as exc:
            last_timeout = exc
            if attempt >= retries:
                raise
            time.sleep(0.05)
    if last_timeout is not None:
        raise last_timeout


def apply_axis_bench_params(session: DeviceSession, updates: dict[str, float]) -> dict[str, float]:
    """Apply live axis-bench parameter updates."""

    applied: dict[str, float] = {}
    for name, value in updates.items():
        if value is None:
            continue
        applied[name] = float(session.set_param(name, 4, value).value)
    return applied


def apply_roll_bench_params(session: DeviceSession, updates: dict[str, float]) -> dict[str, float]:
    return apply_axis_bench_params(session, updates)


def _wait_for_telemetry(session: DeviceSession, timeout_s: float = 2.0) -> TelemetrySample:
    deadline = time.monotonic() + timeout_s
    latest_timestamp = None
    while time.monotonic() < deadline:
        sample = session.get_latest_telemetry()
        if sample is not None and sample.imu_health == 1:
            if sample.timestamp_us != latest_timestamp:
                return sample
            latest_timestamp = sample.timestamp_us
        time.sleep(0.02)
    raise TimeoutError("timed out waiting for fresh healthy telemetry")


def _wait_for_arm_state(session: DeviceSession, expected_state: int, timeout_s: float = 2.0) -> TelemetrySample:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        sample = session.get_latest_telemetry()
        if sample is not None and sample.imu_health == 1 and sample.arm_state == expected_state:
            return sample
        time.sleep(0.02)
    latest = session.get_latest_telemetry()
    raise TimeoutError(
        f"timed out waiting for arm_state={expected_state}; latest {_format_state_detail(latest)}"
    )


def run_axis_bench_round(
    session: DeviceSession,
    output_root: Path,
    *,
    axis_name: str,
    auto_arm: bool,
    small_step_dps: float = 10.0,
    large_step_dps: float = 20.0,
    active_duration_s: float = 0.8,
    zero_duration_s: float = 0.7,
    serial_hint: str | None = None,
    tag: str | None = None,
    orientation_note: str | None = None,
) -> RollBenchRoundResult:
    """Run one conservative constrained single-axis bench round."""

    spec = _bench_spec(axis_name)
    effective_orientation_note = orientation_note or _axis_bench_orientation_note(axis_name, "an unspecified constrained bench")
    now = datetime.now().astimezone()
    run_id = now.strftime("%Y%m%d-%H%M%S")
    if tag:
        run_id = f"{run_id}-{tag}"
    output_dir = output_root / run_id
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "telemetry.csv"
    json_path = output_dir / "summary.json"
    markdown_path = output_dir / "summary.md"
    step_plan = build_default_roll_step_plan(
        small_step_dps=small_step_dps,
        large_step_dps=large_step_dps,
        active_duration_s=active_duration_s,
        zero_duration_s=zero_duration_s,
    )
    captured_samples: list[tuple[float, TelemetrySample]] = []
    current_command = {"value": 0.0}
    live_state = {"bad_sign_count": 0, "sat_count": 0, "trip_reason": None}

    params = _read_axis_params(session, axis_name)
    output_limit = float(params.get("rate_output_limit", 0.0))
    axis_index = AXIS_INDEX[axis_name]

    def on_telemetry(sample: TelemetrySample) -> None:
        captured_samples.append((time.monotonic(), sample))
        command = current_command["value"]
        if abs(command) <= 0.01 or live_state["trip_reason"] is not None:
            live_state["bad_sign_count"] = 0
            live_state["sat_count"] = 0
            return

        split = _motor_split(sample, axis_name)
        pid_out = float(getattr(sample, f"pid_out_{axis_name}"))
        wrong_sign = (pid_out * command) < -0.01 and (split * command) < -0.01
        saturated = output_limit > 0.0 and abs(pid_out) >= (0.98 * output_limit)

        live_state["bad_sign_count"] = live_state["bad_sign_count"] + 1 if wrong_sign else 0
        live_state["sat_count"] = live_state["sat_count"] + 1 if saturated else 0

        if live_state["bad_sign_count"] >= 20:
            live_state["trip_reason"] = (
                f"live sign check failed: pid_out_{axis_name} and motor split oppose the commanded {axis_name} sign"
            )
        elif live_state["sat_count"] >= 80:
            live_state["trip_reason"] = f"live saturation check failed: pid_out_{axis_name} stayed near rate_output_limit"

    telemetry_token = session.subscribe_telemetry(on_telemetry)
    armed_by_helper = False
    started_stream = False
    step_windows: list[tuple[RollBenchStep, list[TelemetrySample]]] = []
    should_kill_on_abort = False

    try:
        session.start_csv_log(csv_path)
        session.start_stream()
        started_stream = True
        sample = _wait_for_telemetry(session, timeout_s=2.0)
        if sample.arm_state not in (ARM_STATE_DISARMED, ARM_STATE_ARMED):
            raise RuntimeError(
                f"{axis_name}-bench requires a disarmed or armed device before the round can start; "
                f"latest {_format_state_detail(sample)}. Clear the bench fault state first."
            )

        if auto_arm:
            _send_simple_command_checked(session, CmdId.ARM, session.arm)
            _wait_for_arm_state(session, ARM_STATE_ARMED, timeout_s=2.0)
            armed_by_helper = True
        elif sample.arm_state != ARM_STATE_ARMED:
            raise RuntimeError(
                f"{axis_name}-bench needs the device armed before any rate-test step. "
                "Use --auto-arm or arm the device manually."
            )

        _send_rate_test_checked(session, axis_index, 0.0)
        time.sleep(0.2)

        for step in step_plan:
            step_start = time.monotonic()
            current_command["value"] = step.command_dps
            _send_rate_test_checked(session, axis_index, step.command_dps)
            if abs(step.command_dps) > 0.01:
                should_kill_on_abort = True
            latest_count = len(captured_samples)
            latest_change = time.monotonic()

            while (time.monotonic() - step_start) < step.duration_s:
                if live_state["trip_reason"] is not None:
                    raise RollBenchSafetyTrip(live_state["trip_reason"])
                if len(captured_samples) != latest_count:
                    latest_count = len(captured_samples)
                    latest_change = time.monotonic()
                elif (time.monotonic() - latest_change) > 0.35:
                    raise RollBenchSafetyTrip("telemetry stalled during active bench step")
                time.sleep(0.01)

            step_end = time.monotonic()
            window_samples = [
                sample
                for sample_time, sample in captured_samples
                if step_start <= sample_time <= step_end
            ]
            step_windows.append((step, window_samples))

        current_command["value"] = 0.0
        _send_rate_test_checked(session, axis_index, 0.0)
        time.sleep(0.25)
        if armed_by_helper:
            _send_simple_command_checked(session, CmdId.DISARM, session.disarm)
            armed_by_helper = False

        metrics, summary = analyze_axis_bench_round(step_windows, params, axis_name=axis_name)
        result = RollBenchRoundResult(
            axis_name=spec.axis_name,
            run_id=run_id,
            serial_hint=serial_hint,
            started_local=now.isoformat(timespec="seconds"),
            orientation_note=effective_orientation_note,
            params=params,
            steps=step_plan,
            metrics=metrics,
            summary=summary,
            csv_path=str(csv_path),
            json_path=str(json_path),
            markdown_path=str(markdown_path),
            telemetry_samples=len(captured_samples),
        )
        json_path.write_text(json.dumps(asdict(result), indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
        markdown_path.write_text(_render_markdown(result), encoding="utf-8")
        return result
    except Exception as exc:
        try:
            current_command["value"] = 0.0
            session.rate_test(axis_index, 0.0)
        except Exception:
            pass
        if isinstance(exc, RollBenchSafetyTrip) and should_kill_on_abort:
            try:
                session.kill()
            except Exception:
                pass
        try:
            session.disarm()
        except Exception:
            pass
        raise
    finally:
        session.unsubscribe(telemetry_token)
        session.stop_csv_log()
        if started_stream:
            try:
                session.stop_stream()
            except Exception:
                pass


def run_roll_bench_round(
    session: DeviceSession,
    output_root: Path,
    *,
    auto_arm: bool,
    small_step_dps: float = 10.0,
    large_step_dps: float = 20.0,
    active_duration_s: float = 0.8,
    zero_duration_s: float = 0.7,
    serial_hint: str | None = None,
    tag: str | None = None,
    orientation_note: str | None = None,
) -> RollBenchRoundResult:
    return run_axis_bench_round(
        session,
        output_root,
        axis_name="roll",
        auto_arm=auto_arm,
        small_step_dps=small_step_dps,
        large_step_dps=large_step_dps,
        active_duration_s=active_duration_s,
        zero_duration_s=zero_duration_s,
        serial_hint=serial_hint,
        tag=tag,
        orientation_note=orientation_note or ROLL_BENCH_ORIENTATION_NOTE,
    )
