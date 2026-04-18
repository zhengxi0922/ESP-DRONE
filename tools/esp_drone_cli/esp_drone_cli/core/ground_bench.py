"""Flat-ground low-throttle tune bench automation."""

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


ARM_STATE_ARMED = 1


def _mean(values: list[float]) -> float:
    return float(fmean(values)) if values else 0.0


def _motor_split(sample: TelemetrySample, axis_name: str) -> float:
    if axis_name == "roll":
        return ((sample.motor1 + sample.motor4) - (sample.motor2 + sample.motor3)) * 0.5
    if axis_name == "pitch":
        return ((sample.motor3 + sample.motor4) - (sample.motor1 + sample.motor2)) * 0.5
    if axis_name == "yaw":
        return ((sample.motor1 + sample.motor3) - (sample.motor2 + sample.motor4)) * 0.5
    return 0.0


@dataclass(slots=True)
class GroundBenchSummary:
    axis: str
    mode: str
    sample_count: int
    sign_ok: bool
    motor_split_ok: bool
    measurable_response: bool
    oscillation_risk: bool
    saturation_risk: bool
    return_to_ref_quality: str
    steady_state_bias: str
    noise_or_jitter_risk: bool
    safe_to_continue: bool
    next_action_hint: str


@dataclass(slots=True)
class GroundBenchResult:
    axis: str
    mode: str
    output_dir: str
    telemetry_csv: str
    summary_json: str
    summary_md: str
    params_snapshot: str
    device_info: str
    events_log: str
    summary: GroundBenchSummary


GROUND_PARAM_NAMES = [
    "rate_kp_roll", "rate_ki_roll", "rate_kd_roll",
    "rate_kp_pitch", "rate_ki_pitch", "rate_kd_pitch",
    "rate_kp_yaw", "rate_ki_yaw", "rate_kd_yaw",
    "rate_integral_limit", "rate_output_limit",
    "ground_att_kp_roll", "ground_att_kp_pitch",
    "ground_att_rate_limit_roll", "ground_att_rate_limit_pitch",
    "ground_att_error_deadband_deg", "ground_att_trip_deg",
    "ground_test_base_duty", "ground_test_max_extra_duty",
    "ground_test_motor_balance_limit", "ground_test_auto_disarm_ms",
    "gyro_lpf_hz", "accel_lpf_hz", "rate_lpf_hz",
    "kalman_enable", "kalman_q_angle", "kalman_q_bias", "kalman_r_measure",
    "ground_tune_enable_attitude_outer", "ground_tune_use_kalman_attitude", "ground_tune_use_filtered_rate",
    "ground_test_ramp_duty_per_s",
]


def _read_ground_params(session: DeviceSession) -> dict[str, object]:
    params: dict[str, object] = {}
    for name in GROUND_PARAM_NAMES:
        try:
            params[name] = session.get_param(name, timeout=2.0).value
        except Exception as exc:
            params[name] = f"unavailable: {exc}"
    return params


def _write_json(path: Path, data: object) -> None:
    path.write_text(json.dumps(data, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")


def _analyze_axis(axis: str, samples: list[TelemetrySample], params: dict[str, object]) -> GroundBenchSummary:
    if not samples:
        return GroundBenchSummary(
            axis=axis,
            mode="ground",
            sample_count=0,
            sign_ok=False,
            motor_split_ok=False,
            measurable_response=False,
            oscillation_risk=False,
            saturation_risk=True,
            return_to_ref_quality="NO_DATA",
            steady_state_bias="NO_DATA",
            noise_or_jitter_risk=True,
            safe_to_continue=False,
            next_action_hint="stop: no telemetry was captured",
        )

    if axis in {"roll", "pitch"}:
        errors = [float(getattr(sample, f"attitude_err_{axis}_deg")) for sample in samples]
        rate_sps = [float(getattr(sample, f"attitude_rate_sp_{axis if axis != 'roll' else 'roll'}")) for sample in samples]
    else:
        errors = [0.0 for _sample in samples]
        rate_sps = [float(sample.rate_setpoint_yaw) for sample in samples]

    pid_outs = [float(getattr(sample, f"pid_out_{axis}")) for sample in samples]
    splits = [_motor_split(sample, axis) for sample in samples]
    filtered_rates = [float(getattr(sample, f"rate_meas_{axis}_filtered")) for sample in samples]
    saturation_flags = [int(sample.motor_saturation_flag) for sample in samples]
    freeze_flags = [int(sample.integrator_freeze_flag) for sample in samples]
    active = [
        (err, sp, out, split, rate)
        for err, sp, out, split, rate in zip(errors, rate_sps, pid_outs, splits, filtered_rates, strict=False)
        if abs(err) >= 0.6 or abs(sp) >= 1.0 or abs(out) >= 0.003
    ]

    if active:
        sign_ok = all((sp * err) <= 0.05 for err, sp, _out, _split, _rate in active if abs(err) >= 0.6)
        motor_split_ok = all((split * out) >= -0.001 for _err, _sp, out, split, _rate in active if abs(out) >= 0.003)
        measurable_response = max(abs(rate) for _err, _sp, _out, _split, rate in active) >= 0.8
    else:
        sign_ok = False
        motor_split_ok = False
        measurable_response = False

    saturation_risk = (sum(saturation_flags) / len(saturation_flags)) >= 0.10
    output_limit = float(params.get("rate_output_limit", 0.0) or 0.0)
    if output_limit > 0.0:
        saturation_risk = saturation_risk or (
            sum(1 for value in pid_outs if abs(value) >= 0.95 * output_limit) / len(pid_outs)
        ) >= 0.10

    sign_flips = 0
    last_sign = 0
    for value in pid_outs:
        sign = 1 if value > 0.01 else -1 if value < -0.01 else 0
        if sign and last_sign and sign != last_sign:
            sign_flips += 1
        if sign:
            last_sign = sign
    oscillation_risk = sign_flips >= max(6, len(samples) // 20)
    rate_peak = max((abs(value) for value in filtered_rates), default=0.0)
    rate_mean_abs = _mean([abs(value) for value in filtered_rates])
    noise_or_jitter_risk = oscillation_risk or rate_peak >= max(8.0, rate_mean_abs * 6.0)

    tail = samples[-max(5, len(samples) // 5) :]
    tail_errors = [abs(float(getattr(sample, f"attitude_err_{axis}_deg", 0.0))) for sample in tail] if axis != "yaw" else [0.0]
    tail_error = _mean(tail_errors)
    return_to_ref_quality = "PASS" if tail_error <= 2.0 else "PASS_WITH_WARNING" if tail_error <= 5.0 else "FAIL"
    steady_state_bias = "PASS" if tail_error <= 1.5 else "PASS_WITH_WARNING" if tail_error <= 4.0 else "FAIL"
    safe_to_continue = sign_ok and motor_split_ok and measurable_response and not saturation_risk and return_to_ref_quality != "FAIL"

    if not sign_ok:
        next_action_hint = "stop: attitude error, rate setpoint, PID output, or motor split sign is inconsistent"
    elif not motor_split_ok:
        next_action_hint = "stop: motor split does not follow PID output direction"
    elif saturation_risk:
        next_action_hint = "stop: reduce base duty, output limit, or rate limit because motors touched limits"
    elif not measurable_response:
        next_action_hint = "continue: response is weak, consider a small Kp increase"
    elif oscillation_risk or noise_or_jitter_risk:
        next_action_hint = "warn: reduce Kp or Kd, or lower D/rate noise with filtering"
    elif steady_state_bias != "PASS":
        next_action_hint = "continue: if rate loop is already clean, consider a very small Ki increase late in tuning"
    else:
        next_action_hint = "accept: signs, split, and conservative ground response are usable"

    if any(freeze_flags) and not saturation_risk:
        next_action_hint += "; integrator freeze was observed"

    return GroundBenchSummary(
        axis=axis,
        mode="ground",
        sample_count=len(samples),
        sign_ok=sign_ok,
        motor_split_ok=motor_split_ok,
        measurable_response=measurable_response,
        oscillation_risk=oscillation_risk,
        saturation_risk=saturation_risk,
        return_to_ref_quality=return_to_ref_quality,
        steady_state_bias=steady_state_bias,
        noise_or_jitter_risk=noise_or_jitter_risk,
        safe_to_continue=safe_to_continue,
        next_action_hint=next_action_hint,
    )


def _render_markdown(result: GroundBenchResult) -> str:
    summary = result.summary
    lines = [
        f"# Ground Bench {result.axis}",
        "",
        f"- output_dir: `{result.output_dir}`",
        f"- sample_count: `{summary.sample_count}`",
        f"- sign_ok: `{summary.sign_ok}`",
        f"- motor_split_ok: `{summary.motor_split_ok}`",
        f"- measurable_response: `{summary.measurable_response}`",
        f"- oscillation_risk: `{summary.oscillation_risk}`",
        f"- saturation_risk: `{summary.saturation_risk}`",
        f"- return_to_ref_quality: `{summary.return_to_ref_quality}`",
        f"- steady_state_bias: `{summary.steady_state_bias}`",
        f"- noise_or_jitter_risk: `{summary.noise_or_jitter_risk}`",
        f"- safe_to_continue: `{summary.safe_to_continue}`",
        f"- next_action_hint: {summary.next_action_hint}",
        "",
    ]
    return "\n".join(lines)


def run_ground_bench_round(
    session: DeviceSession,
    output_root: Path,
    *,
    axis: str,
    duration_s: float = 5.0,
    base_duty: float | None = None,
    auto_arm: bool = False,
    mode: str = "manual_perturb",
) -> GroundBenchResult:
    if axis not in {"roll", "pitch", "yaw", "all"}:
        raise ValueError(f"unsupported ground bench axis {axis}")

    session.require_ground_tune()
    now = datetime.now().astimezone()
    output_dir = output_root / now.strftime(f"%Y%m%d_%H%M%S_ground_{axis}_{mode}")
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "telemetry.csv"
    summary_json_path = output_dir / "summary.json"
    summary_md_path = output_dir / "summary.md"
    params_snapshot_path = output_dir / "params_snapshot.json"
    device_info_path = output_dir / "device_info.json"
    events_path = output_dir / "events.log"

    events: list[str] = []
    samples: list[TelemetrySample] = []

    def on_event(message: str) -> None:
        events.append(f"{time.time():.3f} {message}")

    def on_telemetry(sample: TelemetrySample) -> None:
        samples.append(sample)

    params = _read_ground_params(session)
    _write_json(params_snapshot_path, {"schema": 1, "params": params})
    info = session.device_info or session.hello()
    _write_json(device_info_path, asdict(info))

    event_token = session.subscribe_event_log(on_event)
    telemetry_token = session.subscribe_telemetry(on_telemetry)
    armed_by_helper = False
    started_stream = False
    try:
        session.start_csv_log(csv_path)
        session.start_stream()
        started_stream = True
        time.sleep(0.3)
        if auto_arm:
            ensure_command_ok(CmdId.ARM, session.arm())
            armed_by_helper = True
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                latest = session.get_latest_telemetry()
                if latest is not None and latest.arm_state == ARM_STATE_ARMED:
                    break
                time.sleep(0.05)
        ensure_command_ok(CmdId.GROUND_CAPTURE_REF, session.ground_capture_ref())
        ensure_command_ok(CmdId.GROUND_TEST_START, session.ground_test_start(base_duty=base_duty))
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            latest = session.get_latest_telemetry()
            if latest is not None and latest.ground_trip_reason not in {0, 9}:
                events.append(f"{time.time():.3f} ground trip reason={latest.ground_trip_reason}")
                break
            time.sleep(0.02)
        ensure_command_ok(CmdId.GROUND_TEST_STOP, session.ground_test_stop())
        if armed_by_helper:
            ensure_command_ok(CmdId.DISARM, session.disarm())
            armed_by_helper = False
    finally:
        try:
            session.ground_test_stop()
        except Exception:
            pass
        if armed_by_helper:
            try:
                session.disarm()
            except Exception:
                pass
        if started_stream:
            try:
                session.stop_stream()
            except Exception:
                pass
        session.stop_csv_log()
        session.unsubscribe(event_token)
        session.unsubscribe(telemetry_token)
        events_path.write_text("\n".join(events) + ("\n" if events else ""), encoding="utf-8")

    axes = ("roll", "pitch", "yaw") if axis == "all" else (axis,)
    axis_summaries = [_analyze_axis(item, samples, params) for item in axes]
    safe_to_continue = all(item.safe_to_continue for item in axis_summaries)
    next_action_hint = "; ".join(f"{item.axis}: {item.next_action_hint}" for item in axis_summaries)
    combined = GroundBenchSummary(
        axis=axis,
        mode=mode,
        sample_count=len(samples),
        sign_ok=all(item.sign_ok for item in axis_summaries),
        motor_split_ok=all(item.motor_split_ok for item in axis_summaries),
        measurable_response=all(item.measurable_response for item in axis_summaries),
        oscillation_risk=any(item.oscillation_risk for item in axis_summaries),
        saturation_risk=any(item.saturation_risk for item in axis_summaries),
        return_to_ref_quality="FAIL" if any(item.return_to_ref_quality == "FAIL" for item in axis_summaries) else "PASS",
        steady_state_bias="FAIL" if any(item.steady_state_bias == "FAIL" for item in axis_summaries) else "PASS",
        noise_or_jitter_risk=any(item.noise_or_jitter_risk for item in axis_summaries),
        safe_to_continue=safe_to_continue,
        next_action_hint=next_action_hint,
    )
    result = GroundBenchResult(
        axis=axis,
        mode=mode,
        output_dir=str(output_dir),
        telemetry_csv=str(csv_path),
        summary_json=str(summary_json_path),
        summary_md=str(summary_md_path),
        params_snapshot=str(params_snapshot_path),
        device_info=str(device_info_path),
        events_log=str(events_path),
        summary=combined,
    )
    _write_json(
        summary_json_path,
        {
            **asdict(result),
            "axis_summaries": [asdict(item) for item in axis_summaries],
        },
    )
    summary_md_path.write_text(_render_markdown(result), encoding="utf-8")
    return result
