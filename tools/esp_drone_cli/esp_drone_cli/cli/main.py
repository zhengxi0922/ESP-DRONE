"""命令行前端实现。"""

from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime
import sys
import time
from pathlib import Path

from esp_drone_cli.core import DeviceSession, TelemetrySample
from esp_drone_cli.core.models import (
    FEATURE_ATTITUDE_HANG_BENCH,
    FEATURE_ATTITUDE_GROUND_VERIFY,
    FEATURE_GROUND_TUNE,
    FEATURE_LOW_RISK_LIFTOFF_VERIFY,
    FEATURE_NAMES,
    FEATURE_UDP_MANUAL_CONTROL,
    MIN_ATTITUDE_HANG_PROTOCOL_VERSION,
    MIN_ATTITUDE_GROUND_VERIFY_PROTOCOL_VERSION,
    MIN_GROUND_TUNE_PROTOCOL_VERSION,
    MIN_LOW_RISK_LIFTOFF_PROTOCOL_VERSION,
    MIN_UDP_MANUAL_PROTOCOL_VERSION,
)
from esp_drone_cli.core.ground_bench import run_ground_bench_round
from esp_drone_cli.core.roll_bench import (
    apply_axis_bench_params,
    run_axis_bench_round,
)
from esp_drone_cli.core.protocol.messages import CmdId, CommandError, ensure_command_ok

CONTROL_MODE_ATTITUDE_GROUND_TUNE = 6
GROUND_TUNE_SUBMODE_ATTITUDE_VERIFY = 1
GROUND_TUNE_SUBMODE_LOW_RISK_LIFTOFF = 2
GROUND_VERIFY_SAFE_PARAMS = (
    ("telemetry_usb_hz", 2, 50),
    ("ground_att_kp_roll", 4, 0.8),
    ("ground_att_kp_pitch", 4, 0.8),
    ("ground_att_rate_limit_roll", 4, 4.0),
    ("ground_att_rate_limit_pitch", 4, 4.0),
    ("ground_att_target_limit_deg", 4, 2.0),
    ("ground_att_error_deadband_deg", 4, 0.2),
    ("ground_test_base_duty", 4, 0.08),
    ("ground_test_max_extra_duty", 4, 0.03),
    ("ground_test_motor_balance_limit", 4, 0.06),
    ("ground_test_ramp_duty_per_s", 4, 0.15),
    ("ground_test_auto_disarm_ms", 2, 15000),
)
LIFTOFF_VERIFY_SAFE_PARAMS = (
    ("telemetry_usb_hz", 2, 50),
    ("ground_att_kp_roll", 4, 0.8),
    ("ground_att_kp_pitch", 4, 0.8),
    ("ground_att_rate_limit_roll", 4, 4.0),
    ("ground_att_rate_limit_pitch", 4, 4.0),
    ("ground_att_target_limit_deg", 4, 2.0),
    ("ground_att_error_deadband_deg", 4, 0.2),
    ("ground_test_motor_balance_limit", 4, 0.05),
    ("liftoff_verify_max_extra_duty", 4, 0.03),
    ("liftoff_verify_auto_disarm_ms", 2, 2300),
    ("liftoff_verify_ramp_duty_per_s", 4, 0.08),
    ("liftoff_verify_att_trip_deg", 4, 7.0),
)
LIFTOFF_STATE_NO = "no liftoff"
LIFTOFF_STATE_NEAR = "near liftoff / unloading"
LIFTOFF_STATE_CONFIRMED = "confirmed liftoff"
LIFTOFF_NEAR_BARO_DELTA_M = 0.04
LIFTOFF_CONFIRMED_BARO_DELTA_M = 0.12
UNIFIED_PATH_TOLERANCE_DPS = 0.05
UNIFIED_PATH_MIN_RATIO = 0.98
UNIFIED_PATH_MAX_TRANSIENT_DPS = 0.25


def axis_name_to_index(name: str) -> int:
    """将轴名称转换为协议使用的轴编号。

    Args:
        name: 轴名称，只支持 `roll`、`pitch` 或 `yaw`。

    Returns:
        对应的轴编号，范围为 `0` 到 `2`。

    Raises:
        SystemExit: 轴名称不受支持时抛出。
    """

    names = {"roll": 0, "pitch": 1, "yaw": 2}
    if name not in names:
        raise SystemExit(f"unsupported axis {name}")
    return names[name]


def axis_index_to_name(index: int) -> str:
    names = ("roll", "pitch", "yaw")
    if index < 0 or index >= len(names):
        raise SystemExit(f"unsupported axis index {index}")
    return names[index]


def format_rate_status_line(sample: TelemetrySample, axis_name: str) -> str:
    snapshot = sample.axis_rate_debug_map(axis_name)
    motors = ", ".join(f"{value:.3f}" for value in snapshot["motor_outputs"])
    return (
        f"{axis_name} "
        f"{snapshot['setpoint_field']}={snapshot['setpoint_dps']:.3f} "
        f"{snapshot['feedback_field']}={snapshot['feedback_dps']:.3f} "
        f"source_expr={snapshot['feedback_expr']} "
        f"raw_{snapshot['source_field']}={snapshot['source_value']:.3f} "
        f"{snapshot['pid_p_field']}={snapshot['pid_p']:.4f} "
        f"{snapshot['pid_i_field']}={snapshot['pid_i']:.4f} "
        f"{snapshot['pid_d_field']}={snapshot['pid_d']:.4f} "
        f"{snapshot['pid_out_field']}={snapshot['pid_out']:.4f} "
        f"motor1..motor4=[{motors}] "
        f"arm_state={snapshot['arm_state']} "
        f"control_mode={snapshot['control_mode']} "
        f"imu_age_us={snapshot['imu_age_us']} "
        f"loop_dt_us={snapshot['loop_dt_us']}"
    )


def format_rate_status_line_all(sample: TelemetrySample) -> str:
    parts = [format_rate_status_line(sample, axis_name) for axis_name in ("roll", "pitch", "yaw")]
    return " | ".join(parts)


def format_attitude_status_line(sample: TelemetrySample, axis_name: str) -> str:
    snapshot = sample.axis_attitude_debug_map(axis_name)
    motors = ", ".join(f"{value:.3f}" for value in snapshot["motor_outputs"])
    return (
        f"{axis_name} "
        f"ref_valid={snapshot['attitude_ref_valid']} "
        f"err={snapshot['error_deg']:.3f}deg "
        f"rate_sp={snapshot['rate_sp_dps']:.3f}dps "
        f"pid_out={snapshot['pid_out']:.4f} "
        f"base={snapshot['base_duty_active']:.3f} "
        f"motors=[{motors}] "
        f"arm={snapshot['arm_state']} "
        f"mode={snapshot['control_mode']} "
        f"imu_age_us={snapshot['imu_age_us']} "
        f"loop_dt_us={snapshot['loop_dt_us']}"
    )


def format_attitude_status_line_all(sample: TelemetrySample) -> str:
    parts = [format_attitude_status_line(sample, axis_name) for axis_name in ("roll", "pitch")]
    ref_q = (
        f"ref_q=[{sample.attitude_ref_qw:.4f}, {sample.attitude_ref_qx:.4f}, "
        f"{sample.attitude_ref_qy:.4f}, {sample.attitude_ref_qz:.4f}]"
    )
    return " | ".join(parts + [ref_q])


def format_ground_status_line(sample: TelemetrySample, axis_name: str) -> str:
    motors = ", ".join(f"{value:.3f}" for value in (sample.motor1, sample.motor2, sample.motor3, sample.motor4))
    if axis_name in {"roll", "pitch"}:
        target = float(getattr(sample, f"angle_target_{axis_name}"))
        measured = float(getattr(sample, f"angle_measured_{axis_name}"))
        err = float(getattr(sample, f"angle_error_{axis_name}"))
        rate_sp = float(getattr(sample, f"outer_loop_rate_target_{axis_name}"))
    else:
        target = 0.0
        measured = 0.0
        err = 0.0
        rate_sp = float(sample.rate_setpoint_yaw)
    return (
        f"{axis_name} "
        f"ref_valid={sample.reference_valid} "
        f"ground_ref={sample.ground_ref_valid} "
        f"kalman_valid={sample.kalman_valid} "
        f"trip={sample.ground_trip_reason} "
        f"target={target:.3f}deg "
        f"measured={measured:.3f}deg "
        f"angle_err={err:.3f}deg "
        f"rate_sp={rate_sp:.3f}dps "
        f"raw_rate={float(getattr(sample, f'rate_meas_{axis_name}_raw')):.3f} "
        f"filtered_rate={float(getattr(sample, f'rate_meas_{axis_name}_filtered')):.3f} "
        f"rate_err={float(getattr(sample, f'rate_err_{axis_name}')):.3f} "
        f"pid_p={float(getattr(sample, f'rate_pid_p_{axis_name}')):.4f} "
        f"pid_i={float(getattr(sample, f'rate_pid_i_{axis_name}')):.4f} "
        f"pid_d={float(getattr(sample, f'rate_pid_d_{axis_name}')):.4f} "
        f"pid_out={float(getattr(sample, f'pid_out_{axis_name}')):.4f} "
        f"base={sample.base_duty_active:.3f} "
        f"mixer=[thr:{sample.base_duty_active:.3f},r:{sample.pid_out_roll:.4f},p:{sample.pid_out_pitch:.4f},y:{sample.pid_out_yaw:.4f}] "
        f"outer_clamp={sample.outer_loop_clamp_flag} "
        f"inner_clamp={sample.inner_loop_clamp_flag} "
        f"sat={sample.motor_saturation_flag} "
        f"i_freeze={sample.integrator_freeze_flag} "
        f"battery_valid={sample.battery_valid} "
        f"motors=[{motors}] "
        f"arm={sample.arm_state} mode={sample.control_mode} submode={sample.control_submode} imu_age_us={sample.imu_age_us}"
    )


def format_ground_status_line_all(sample: TelemetrySample) -> str:
    parts = [format_ground_status_line(sample, axis_name) for axis_name in ("roll", "pitch", "yaw")]
    return " | ".join(parts)


def _mean_field(samples: list[TelemetrySample], field: str) -> float:
    if not samples:
        return 0.0
    return sum(float(getattr(sample, field)) for sample in samples) / len(samples)


def _sign_label(value: float, deadband: float = 1e-5) -> str:
    if value > deadband:
        return "+"
    if value < -deadband:
        return "-"
    return "0"


def _axis_motor_delta(sample: TelemetrySample, axis_name: str) -> float:
    if axis_name == "roll":
        return (sample.motor1 + sample.motor4) - (sample.motor2 + sample.motor3)
    if axis_name == "pitch":
        return (sample.motor3 + sample.motor4) - (sample.motor1 + sample.motor2)
    raise ValueError(f"unsupported attitude verify axis {axis_name}")


def _same_sign_stats(samples: list[TelemetrySample], lhs, rhs, lhs_deadband: float, rhs_deadband: float) -> tuple[int, int, float]:
    considered = 0
    matched = 0
    for sample in samples:
        left = float(lhs(sample) if callable(lhs) else getattr(sample, lhs))
        right = float(rhs(sample) if callable(rhs) else getattr(sample, rhs))
        if abs(left) <= lhs_deadband or abs(right) <= rhs_deadband:
            continue
        considered += 1
        if left * right > 0.0:
            matched += 1
    if considered == 0:
        return 0, 0, 0.0
    return considered, matched, matched / considered


def _same_sign_ratio(samples: list[TelemetrySample], lhs, rhs, lhs_deadband: float, rhs_deadband: float) -> float:
    return _same_sign_stats(samples, lhs, rhs, lhs_deadband, rhs_deadband)[2]


def analyze_attitude_ground_verify_samples(samples: list[TelemetrySample], target_deg: float) -> dict[str, object]:
    active = [
        sample for sample in samples
        if sample.control_mode == CONTROL_MODE_ATTITUDE_GROUND_TUNE
        and sample.control_submode == GROUND_TUNE_SUBMODE_ATTITUDE_VERIFY
    ]
    result: dict[str, object] = {
        "samples": len(samples),
        "active_samples": len(active),
        "validity_ok": False,
        "safety_ok": False,
        "yaw_ok": False,
        "outer_clamp_max": max((sample.outer_loop_clamp_flag for sample in active), default=0),
        "inner_clamp_max": max((sample.inner_loop_clamp_flag for sample in active), default=0),
        "inner_motor_clamp_max": max((sample.inner_loop_clamp_flag & 0x01 for sample in active), default=0),
        "inner_integrator_freeze_count": sum(
            1 for sample in active if (sample.inner_loop_clamp_flag & 0x02) != 0
        ),
        "motor_saturation_max": max((sample.motor_saturation_flag for sample in active), default=0),
        "segments": {},
    }
    if not active:
        return result

    result["validity_ok"] = all(
        sample.kalman_valid and sample.attitude_valid and sample.ground_ref_valid for sample in active
    )
    result["safety_ok"] = all(
        sample.failsafe_reason == 0 and sample.ground_trip_reason == 0 for sample in active
    )
    result["yaw_ok"] = all(
        abs(sample.angle_target_yaw) <= 0.01 and
        abs(sample.outer_loop_rate_target_yaw) <= 0.05 and
        abs(sample.rate_setpoint_yaw) <= 0.05
        for sample in active
    )

    segments: dict[str, dict[str, object]] = {}
    threshold = max(0.25, abs(target_deg) * 0.5)
    for axis_name in ("roll", "pitch"):
        for direction, label in ((1.0, "pos"), (-1.0, "neg")):
            segment_samples = [
                sample for sample in active
                if direction * float(getattr(sample, f"angle_target_{axis_name}")) > threshold
            ]
            err_mean = _mean_field(segment_samples, f"angle_error_{axis_name}")
            rate_sp_mean = _mean_field(segment_samples, f"outer_loop_rate_target_{axis_name}")
            rate_err_mean = _mean_field(segment_samples, f"rate_err_{axis_name}")
            pid_p_mean = _mean_field(segment_samples, f"rate_pid_p_{axis_name}")
            pid_out_mean = _mean_field(segment_samples, f"pid_out_{axis_name}")
            motor_delta_mean = (
                sum(_axis_motor_delta(sample, axis_name) for sample in segment_samples) / len(segment_samples)
                if segment_samples else 0.0
            )
            expected = direction
            outer_link_ok = bool(segment_samples) and err_mean * expected > 0.0 and rate_sp_mean * err_mean > 0.0
            rate_pid_sign_ratio = _same_sign_ratio(
                segment_samples,
                f"rate_err_{axis_name}",
                f"rate_pid_p_{axis_name}",
                0.05,
                1e-6,
            )
            pid_out_sign_ratio = _same_sign_ratio(
                segment_samples,
                f"rate_pid_p_{axis_name}",
                f"pid_out_{axis_name}",
                1e-6,
                1e-6,
            )
            mixer_sign_ratio = _same_sign_ratio(
                segment_samples,
                lambda sample, axis=axis_name: getattr(sample, f"pid_out_{axis}"),
                lambda sample, axis=axis_name: _axis_motor_delta(sample, axis),
                1e-6,
                1e-6,
            )
            sign_ok = bool(segment_samples) and outer_link_ok and rate_pid_sign_ratio >= 0.90 and \
                pid_out_sign_ratio >= 0.95 and mixer_sign_ratio >= 0.95
            segments[f"{axis_name}_{label}"] = {
                "count": len(segment_samples),
                "sign_ok": sign_ok,
                "outer_link_ok": outer_link_ok,
                "rate_pid_sign_ratio": rate_pid_sign_ratio,
                "pid_out_sign_ratio": pid_out_sign_ratio,
                "mixer_sign_ratio": mixer_sign_ratio,
                "angle_error_mean": err_mean,
                "outer_rate_target_mean": rate_sp_mean,
                "rate_error_mean": rate_err_mean,
                "pid_p_mean": pid_p_mean,
                "pid_out_mean": pid_out_mean,
                "motor_delta_mean": motor_delta_mean,
                "signs": {
                    "angle_error": _sign_label(err_mean),
                    "outer_rate_target": _sign_label(rate_sp_mean),
                    "rate_error": _sign_label(rate_err_mean),
                    "pid_p": _sign_label(pid_p_mean),
                    "pid_out": _sign_label(pid_out_mean),
                    "motor_delta": _sign_label(motor_delta_mean),
                },
            }
    result["segments"] = segments
    result["chain_ok"] = all(segment["sign_ok"] for segment in segments.values())
    result["passed"] = bool(
        result["validity_ok"] and
        result["safety_ok"] and
        result["yaw_ok"] and
        result["chain_ok"] and
        int(result["outer_clamp_max"]) == 0 and
        int(result["inner_motor_clamp_max"]) == 0 and
        int(result["motor_saturation_max"]) == 0
    )
    return result


def format_attitude_ground_verify_summary(result: dict[str, object]) -> list[str]:
    lines = [
        f"samples={result['samples']} active_samples={result['active_samples']}",
        (
            f"validity_ok={result['validity_ok']} safety_ok={result['safety_ok']} yaw_ok={result['yaw_ok']} "
            f"outer_clamp_max={result['outer_clamp_max']} inner_clamp_max={result['inner_clamp_max']} "
            f"inner_motor_clamp_max={result.get('inner_motor_clamp_max', 0)} "
            f"inner_integrator_freeze_count={result.get('inner_integrator_freeze_count', 0)} "
            f"motor_saturation_max={result['motor_saturation_max']} passed={result.get('passed', False)}"
        ),
    ]
    for name, segment in dict(result.get("segments", {})).items():
        signs = dict(segment["signs"])
        lines.append(
            f"{name}: count={segment['count']} sign_ok={segment['sign_ok']} "
            f"outer_ok={segment.get('outer_link_ok', False)} "
            f"rate_pid_ratio={segment.get('rate_pid_sign_ratio', 0.0):.2f} "
            f"mixer_ratio={segment.get('mixer_sign_ratio', 0.0):.2f} "
            f"err={segment['angle_error_mean']:.4f}({signs['angle_error']}) "
            f"outer_rate={segment['outer_rate_target_mean']:.4f}({signs['outer_rate_target']}) "
            f"rate_err={segment['rate_error_mean']:.4f}({signs['rate_error']}) "
            f"pid_p={segment['pid_p_mean']:.6f}({signs['pid_p']}) "
            f"pid_out={segment['pid_out_mean']:.6f}({signs['pid_out']}) "
            f"motor_delta={segment['motor_delta_mean']:.6f}({signs['motor_delta']})"
        )
    return lines


def analyze_liftoff_verify_samples(samples: list[TelemetrySample], base_duty: float) -> dict[str, object]:
    active = [
        sample for sample in samples
        if sample.control_mode == CONTROL_MODE_ATTITUDE_GROUND_TUNE
        and sample.control_submode == GROUND_TUNE_SUBMODE_LOW_RISK_LIFTOFF
    ]
    terminal_trip = 0
    for sample in reversed(samples):
        if sample.ground_trip_reason != 0:
            terminal_trip = int(sample.ground_trip_reason)
            break

    result: dict[str, object] = {
        "samples": len(samples),
        "active_samples": len(active),
        "steady_samples": 0,
        "active_duration_s": 0.0,
        "validity_ok": False,
        "failsafe_reason": 0,
        "ground_trip_reason": 0,
        "safety_ok": False,
        "terminal_trip_reason": terminal_trip,
        "terminal_trip_ok": terminal_trip in {0, 6, 9},
        "unified_path_ok": False,
        "unified_path_count": 0,
        "unified_path_match": 0,
        "unified_path_ratio": 0.0,
        "unified_path_max_abs_error_dps": 0.0,
        "chain_ok": False,
        "yaw_ok": False,
        "tilt_ok": False,
        "control_safe_pass": False,
        "physical_liftoff_state": LIFTOFF_STATE_NO,
        "physical_liftoff_confirmed": False,
        "free_flight_pass": False,
        "probable_liftoff": False,
        "base_duty_target": base_duty,
        "base_duty_max": max((sample.base_duty_active for sample in active), default=0.0),
        "motor_max": max(
            (max(sample.motor1, sample.motor2, sample.motor3, sample.motor4) for sample in active),
            default=0.0,
        ),
        "outer_clamp_max": max((sample.outer_loop_clamp_flag for sample in active), default=0),
        "inner_clamp_max": max((sample.inner_loop_clamp_flag for sample in active), default=0),
        "raw_inner_motor_clamp_max": max((sample.inner_loop_clamp_flag & 0x01 for sample in active), default=0),
        "inner_motor_clamp_max": 0,
        "startup_motor_clamp_count": 0,
        "inner_integrator_freeze_count": sum(
            1 for sample in active if (sample.inner_loop_clamp_flag & 0x02) != 0
        ),
        "raw_motor_saturation_max": max((sample.motor_saturation_flag for sample in active), default=0),
        "motor_saturation_max": 0,
        "max_abs_roll_deg": max((abs(sample.angle_measured_roll) for sample in active), default=0.0),
        "max_abs_pitch_deg": max((abs(sample.angle_measured_pitch) for sample in active), default=0.0),
        "max_abs_yaw_rate_dps": max((abs(sample.rate_meas_yaw_filtered) for sample in active), default=0.0),
        "mean_abs_yaw_rate_dps": (
            sum(abs(sample.rate_meas_yaw_filtered) for sample in active) / len(active)
            if active else 0.0
        ),
        "battery_sag_v": 0.0,
        "baro_altitude_delta_m": 0.0,
        "axis": {},
    }
    if not active:
        return result

    result["active_duration_s"] = max(0.0, (active[-1].timestamp_us - active[0].timestamp_us) / 1000000.0)
    steady_threshold = max(0.05, base_duty * 0.5)
    steady = [sample for sample in active if sample.base_duty_active >= steady_threshold]
    if not steady:
        steady = active
    result["steady_samples"] = len(steady)
    result["inner_motor_clamp_max"] = max((sample.inner_loop_clamp_flag & 0x01 for sample in steady), default=0)
    result["motor_saturation_max"] = max((sample.motor_saturation_flag for sample in steady), default=0)
    result["startup_motor_clamp_count"] = sum(
        1 for sample in active
        if sample.base_duty_active < steady_threshold and
        ((sample.inner_loop_clamp_flag & 0x01) != 0 or sample.motor_saturation_flag != 0)
    )
    result["failsafe_reason"] = max((int(sample.failsafe_reason) for sample in active), default=0)
    result["ground_trip_reason"] = max((int(sample.ground_trip_reason) for sample in active), default=0)
    valid_battery = [
        sample.battery_voltage
        for sample in active
        if sample.battery_valid and sample.battery_voltage > 0.1
    ]
    if valid_battery:
        result["battery_sag_v"] = max(valid_battery) - min(valid_battery)
    valid_baro = [sample.baro_altitude_m for sample in active if sample.baro_valid]
    if valid_baro:
        result["baro_altitude_delta_m"] = max(valid_baro) - min(valid_baro)

    result["validity_ok"] = all(
        sample.kalman_valid and sample.attitude_valid and sample.ground_ref_valid and sample.battery_valid
        for sample in active
    )
    result["safety_ok"] = all(
        sample.failsafe_reason == 0 and sample.ground_trip_reason == 0 for sample in active
    )
    unified_errors = [
        max(
            abs(sample.rate_setpoint_roll - sample.outer_loop_rate_target_roll),
            abs(sample.rate_setpoint_pitch - sample.outer_loop_rate_target_pitch),
            abs(sample.rate_setpoint_yaw - sample.outer_loop_rate_target_yaw),
        )
        for sample in active
    ]
    unified_match = sum(1 for error in unified_errors if error <= UNIFIED_PATH_TOLERANCE_DPS)
    result["unified_path_count"] = len(unified_errors)
    result["unified_path_match"] = unified_match
    result["unified_path_ratio"] = unified_match / len(unified_errors) if unified_errors else 0.0
    result["unified_path_max_abs_error_dps"] = max(unified_errors, default=0.0)
    result["unified_path_ok"] = bool(
        unified_errors and
        float(result["unified_path_ratio"]) >= UNIFIED_PATH_MIN_RATIO and
        float(result["unified_path_max_abs_error_dps"]) <= UNIFIED_PATH_MAX_TRANSIENT_DPS
    )
    result["yaw_ok"] = (
        all(
            abs(sample.angle_target_yaw) <= 0.01 and
            abs(sample.outer_loop_rate_target_yaw) <= 0.05 and
            abs(sample.rate_setpoint_yaw) <= 0.05
            for sample in active
        ) and
        float(result["max_abs_yaw_rate_dps"]) <= 80.0 and
        float(result["mean_abs_yaw_rate_dps"]) <= 35.0
    )
    result["tilt_ok"] = (
        float(result["max_abs_roll_deg"]) <= 6.5 and
        float(result["max_abs_pitch_deg"]) <= 6.5
    )
    baro_delta = float(result["baro_altitude_delta_m"])
    if baro_delta >= LIFTOFF_CONFIRMED_BARO_DELTA_M:
        result["physical_liftoff_state"] = LIFTOFF_STATE_CONFIRMED
        result["physical_liftoff_confirmed"] = True
    elif baro_delta >= LIFTOFF_NEAR_BARO_DELTA_M:
        result["physical_liftoff_state"] = LIFTOFF_STATE_NEAR
    result["probable_liftoff"] = bool(result["physical_liftoff_confirmed"])

    axis_results: dict[str, dict[str, object]] = {}
    for axis_name in ("roll", "pitch"):
        outer_count, outer_match, outer_ratio = _same_sign_stats(
            active,
            f"angle_error_{axis_name}",
            f"outer_loop_rate_target_{axis_name}",
            0.25,
            0.05,
        )
        rate_count, rate_match, rate_ratio = _same_sign_stats(
            active,
            f"rate_err_{axis_name}",
            f"rate_pid_p_{axis_name}",
            0.05,
            1e-6,
        )
        mixer_count, mixer_match, mixer_ratio = _same_sign_stats(
            active,
            lambda sample, axis=axis_name: getattr(sample, f"pid_out_{axis}"),
            lambda sample, axis=axis_name: _axis_motor_delta(sample, axis),
            1e-6,
            1e-6,
        )
        outer_ok = outer_count == 0 or outer_ratio >= 0.90
        rate_ok = rate_count == 0 or rate_ratio >= 0.90
        mixer_ok = mixer_count == 0 or mixer_ratio >= 0.95
        axis_results[axis_name] = {
            "outer_count": outer_count,
            "outer_match": outer_match,
            "outer_ratio": outer_ratio,
            "rate_pid_count": rate_count,
            "rate_pid_match": rate_match,
            "rate_pid_ratio": rate_ratio,
            "mixer_count": mixer_count,
            "mixer_match": mixer_match,
            "mixer_ratio": mixer_ratio,
            "axis_ok": outer_ok and rate_ok and mixer_ok,
        }
    result["axis"] = axis_results
    result["chain_ok"] = all(axis["axis_ok"] for axis in axis_results.values())
    result["control_safe_pass"] = bool(
        result["validity_ok"] and
        result["safety_ok"] and
        result["terminal_trip_ok"] and
        result["unified_path_ok"] and
        result["chain_ok"] and
        result["yaw_ok"] and
        result["tilt_ok"] and
        int(result["outer_clamp_max"]) == 0 and
        int(result["inner_motor_clamp_max"]) == 0 and
        int(result["motor_saturation_max"]) == 0
    )
    result["passed"] = result["control_safe_pass"]
    return result


def format_liftoff_verify_summary(result: dict[str, object]) -> list[str]:
    lines = [
        f"samples={result['samples']} active_samples={result['active_samples']} "
        f"steady_samples={result.get('steady_samples', 0)} active_duration_s={result['active_duration_s']:.3f}",
        (
            f"validity_ok={result['validity_ok']} failsafe_reason={result.get('failsafe_reason', 0)} "
            f"ground_trip_reason={result.get('ground_trip_reason', 0)} "
            f"terminal_trip={result['terminal_trip_reason']} terminal_trip_ok={result['terminal_trip_ok']}"
        ),
        (
            f"unified_path_ok={result['unified_path_ok']} chain_ok={result['chain_ok']} "
            f"yaw_ok={result['yaw_ok']} tilt_ok={result['tilt_ok']} "
            f"safety_ok={result['safety_ok']} control_safe_pass={result.get('control_safe_pass', False)}"
        ),
        (
            f"unified_path_ratio={result.get('unified_path_ratio', 0.0):.3f}"
            f"({result.get('unified_path_match', 0)}/{result.get('unified_path_count', 0)}) "
            f"unified_path_max_abs_error_dps={result.get('unified_path_max_abs_error_dps', 0.0):.3f}"
        ),
        (
            f"base_duty_max={result['base_duty_max']:.4f}/{result['base_duty_target']:.4f} "
            f"motor_max={result['motor_max']:.4f} outer_clamp_max={result['outer_clamp_max']} "
            f"steady_inner_motor_clamp_max={result['inner_motor_clamp_max']} "
            f"raw_inner_motor_clamp_max={result.get('raw_inner_motor_clamp_max', 0)} "
            f"startup_motor_clamp_count={result.get('startup_motor_clamp_count', 0)} "
            f"inner_integrator_freeze_count={result['inner_integrator_freeze_count']} "
            f"steady_motor_saturation_max={result['motor_saturation_max']} "
            f"raw_motor_saturation_max={result.get('raw_motor_saturation_max', 0)}"
        ),
        (
            f"max_roll_deg={result['max_abs_roll_deg']:.3f} "
            f"max_pitch_deg={result['max_abs_pitch_deg']:.3f} "
            f"max_abs_yaw_rate_dps={result['max_abs_yaw_rate_dps']:.3f} "
            f"mean_abs_yaw_rate_dps={result['mean_abs_yaw_rate_dps']:.3f} "
            f"battery_sag_v={result.get('battery_sag_v', 0.0):.3f} "
            f"baro_delta_m={result['baro_altitude_delta_m']:.3f}"
        ),
        (
            f"physical_liftoff_state={result.get('physical_liftoff_state', LIFTOFF_STATE_NO)} "
            f"physical_liftoff_confirmed={result.get('physical_liftoff_confirmed', False)} "
            f"free_flight_pass={result.get('free_flight_pass', False)}"
        ),
        (
            "final_verdict: "
            f"safety/control={'control-safe pass' if result.get('control_safe_pass', False) else 'not pass'} "
            f"physical_liftoff={'confirmed' if result.get('physical_liftoff_confirmed', False) else 'not confirmed'}"
        ),
    ]
    for axis_name, axis in dict(result.get("axis", {})).items():
        lines.append(
            f"{axis_name}: axis_ok={axis['axis_ok']} "
            f"outer_ratio={axis['outer_ratio']:.2f}({axis['outer_match']}/{axis['outer_count']}) "
            f"rate_pid_ratio={axis['rate_pid_ratio']:.2f}({axis['rate_pid_match']}/{axis['rate_pid_count']}) "
            f"mixer_ratio={axis['mixer_ratio']:.2f}({axis['mixer_match']}/{axis['mixer_count']})"
        )
    return lines


def wait_for_one_sample(session: DeviceSession, timeout: float) -> TelemetrySample:
    samples: deque[TelemetrySample] = deque(maxlen=1)

    def on_telemetry(sample: TelemetrySample) -> None:
        samples.append(sample)

    token = session.subscribe_telemetry(on_telemetry)
    try:
        session.start_stream()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if samples:
                return samples[-1]
            time.sleep(0.02)
    finally:
        try:
            session.stop_stream()
        except Exception:
            pass
        session.unsubscribe(token)
    raise TimeoutError("attitude-status did not receive telemetry within timeout")


def connect_session_from_args(args) -> DeviceSession:
    """根据命令行参数创建并连接设备会话。

    Args:
        args: `argparse` 解析后的参数对象，需包含 `serial`、`udp` 和 `baudrate` 字段。

    Returns:
        已建立连接的 `DeviceSession`。

    Raises:
        SystemExit: 未提供 `--serial` 或 `--udp` 时抛出。
        ValueError: UDP 端口无法转换为整数时抛出。
        RuntimeError: 设备连接或握手失败时由下层会话抛出。
    """

    session = DeviceSession()
    if args.serial:
        session.connect_serial(args.serial, baudrate=args.baudrate)
    elif args.udp:
        host, _, port = args.udp.partition(":")
        session.connect_udp(host, int(port or "2391"))
    else:
        raise SystemExit("one of --serial or --udp is required")
    return session


def add_common_transport_args(parser: argparse.ArgumentParser) -> None:
    """为命令解析器补充通用连接参数。

    Args:
        parser: 目标解析器。

    Returns:
        None.
    """

    parser.add_argument("--serial", help="Serial port, for example COM7")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--udp", help="UDP endpoint host[:port], default port 2391")


def cmd_connect(session: DeviceSession, _args) -> int:
    """执行握手并打印设备摘要。

    Args:
        session: 已连接的设备会话。
        _args: 当前命令未使用的参数对象。

    Returns:
        固定返回 `0`。
    """

    print(session.device_info or session.hello())
    return 0


def cmd_capabilities(session: DeviceSession, _args) -> int:
    """Print firmware build identity and advertised feature bits."""

    info = session.device_info or session.hello()
    print(f"protocol_version={info.protocol_version}")
    print(f"feature_bitmap=0x{info.feature_bitmap:08x}")
    print(f"build_git_hash={info.build_git_hash or 'unknown'}")
    print(f"build_time_utc={info.build_time_utc or 'unknown'}")
    for bit, name in FEATURE_NAMES.items():
        print(f"{name}={info.supports_feature(bit)}")
    try:
        info.require_attitude_hang_bench()
        print("attitude_hang_ready=True")
    except Exception as exc:
        print(f"attitude_hang_ready=False ({exc})")
        return 1
    return 0


def require_attitude_hang_capability(session: DeviceSession) -> None:
    """Reject bench-only hang-attitude commands before touching params or sending opcodes."""

    if hasattr(session, "require_attitude_hang_bench"):
        session.require_attitude_hang_bench()
        return
    info = session.device_info or session.hello()
    if hasattr(info, "require_attitude_hang_bench"):
        info.require_attitude_hang_bench()
        return
    protocol_version = int(getattr(info, "protocol_version", 0))
    feature_bitmap = int(getattr(info, "feature_bitmap", 0))
    if protocol_version >= MIN_ATTITUDE_HANG_PROTOCOL_VERSION and (feature_bitmap & FEATURE_ATTITUDE_HANG_BENCH):
        return
    raise RuntimeError(
        "device firmware does not advertise bench-only hang-attitude support "
        f"(need protocol_version>={MIN_ATTITUDE_HANG_PROTOCOL_VERSION} and "
        f"feature attitude_hang_bench/0x{FEATURE_ATTITUDE_HANG_BENCH:02x}; "
        f"got protocol_version={protocol_version}, feature_bitmap=0x{feature_bitmap:08x}). "
        "Rebuild and flash the current main firmware before running hang-attitude commands."
    )


def require_udp_manual_capability(session: DeviceSession) -> None:
    """Reject experimental UDP manual commands before sending manual-control opcodes."""

    if hasattr(session, "require_udp_manual_control"):
        session.require_udp_manual_control()
        return
    info = session.device_info or session.hello()
    if hasattr(info, "require_udp_manual_control"):
        info.require_udp_manual_control()
        return
    protocol_version = int(getattr(info, "protocol_version", 0))
    feature_bitmap = int(getattr(info, "feature_bitmap", 0))
    if protocol_version >= MIN_UDP_MANUAL_PROTOCOL_VERSION and (feature_bitmap & FEATURE_UDP_MANUAL_CONTROL):
        return
    raise RuntimeError(
        "device firmware does not advertise experimental UDP manual control support "
        f"(need protocol_version>={MIN_UDP_MANUAL_PROTOCOL_VERSION} and "
        f"feature udp_manual_control/0x{FEATURE_UDP_MANUAL_CONTROL:02x}; "
        f"got protocol_version={protocol_version}, feature_bitmap=0x{feature_bitmap:08x}). "
        "Rebuild and flash the current main firmware before using UDP Control."
    )


def require_ground_tune_capability(session: DeviceSession) -> None:
    """Reject flat-ground tune commands before sending ground opcodes."""

    if hasattr(session, "require_ground_tune"):
        session.require_ground_tune()
        return
    info = session.device_info or session.hello()
    if hasattr(info, "require_ground_tune"):
        info.require_ground_tune()
        return
    protocol_version = int(getattr(info, "protocol_version", 0))
    feature_bitmap = int(getattr(info, "feature_bitmap", 0))
    if protocol_version >= MIN_GROUND_TUNE_PROTOCOL_VERSION and (feature_bitmap & FEATURE_GROUND_TUNE):
        return
    raise RuntimeError(
        "device firmware does not advertise flat-ground tune support "
        f"(need protocol_version>={MIN_GROUND_TUNE_PROTOCOL_VERSION} and "
        f"feature ground_tune/0x{FEATURE_GROUND_TUNE:02x}; "
        f"got protocol_version={protocol_version}, feature_bitmap=0x{feature_bitmap:08x}). "
        "Rebuild and flash the current main firmware before running ground tune commands."
    )


def require_attitude_ground_verify_capability(session: DeviceSession) -> None:
    """Reject attitude ground verify commands before sending new opcodes."""

    if hasattr(session, "require_attitude_ground_verify"):
        session.require_attitude_ground_verify()
        return
    info = session.device_info or session.hello()
    if hasattr(info, "require_attitude_ground_verify"):
        info.require_attitude_ground_verify()
        return
    protocol_version = int(getattr(info, "protocol_version", 0))
    feature_bitmap = int(getattr(info, "feature_bitmap", 0))
    if protocol_version >= MIN_ATTITUDE_GROUND_VERIFY_PROTOCOL_VERSION and (feature_bitmap & FEATURE_ATTITUDE_GROUND_VERIFY):
        return
    raise RuntimeError(
        "device firmware does not advertise flat-ground attitude verification support "
        f"(need protocol_version>={MIN_ATTITUDE_GROUND_VERIFY_PROTOCOL_VERSION} and "
        f"feature attitude_ground_verify/0x{FEATURE_ATTITUDE_GROUND_VERIFY:02x}; "
        f"got protocol_version={protocol_version}, feature_bitmap=0x{feature_bitmap:08x}). "
        "Rebuild and flash the current main firmware before running attitude ground verify commands."
    )


def require_low_risk_liftoff_capability(session: DeviceSession) -> None:
    """Reject low-risk liftoff verify commands before sending new opcodes."""

    if hasattr(session, "require_low_risk_liftoff_verify"):
        session.require_low_risk_liftoff_verify()
        return
    info = session.device_info or session.hello()
    if hasattr(info, "require_low_risk_liftoff_verify"):
        info.require_low_risk_liftoff_verify()
        return
    protocol_version = int(getattr(info, "protocol_version", 0))
    feature_bitmap = int(getattr(info, "feature_bitmap", 0))
    if protocol_version >= MIN_LOW_RISK_LIFTOFF_PROTOCOL_VERSION and (feature_bitmap & FEATURE_LOW_RISK_LIFTOFF_VERIFY):
        return
    raise RuntimeError(
        "device firmware does not advertise low-risk liftoff verification support "
        f"(need protocol_version>={MIN_LOW_RISK_LIFTOFF_PROTOCOL_VERSION} and "
        f"feature low_risk_liftoff_verify/0x{FEATURE_LOW_RISK_LIFTOFF_VERIFY:02x}; "
        f"got protocol_version={protocol_version}, feature_bitmap=0x{feature_bitmap:08x}). "
        "Rebuild and flash the current main firmware before running liftoff verify commands."
    )


def cmd_arm(session: DeviceSession, _args) -> int:
    """发送解锁命令。

    Args:
        session: 已连接的设备会话。
        _args: 当前命令未使用的参数对象。

    Returns:
        设备返回的命令状态码。
    """

    ensure_command_ok(CmdId.ARM, session.arm())
    return 0


def cmd_disarm(session: DeviceSession, _args) -> int:
    """发送上锁命令。

    Args:
        session: 已连接的设备会话。
        _args: 当前命令未使用的参数对象。

    Returns:
        设备返回的命令状态码。
    """

    ensure_command_ok(CmdId.DISARM, session.disarm())
    return 0


def cmd_kill(session: DeviceSession, _args) -> int:
    """发送急停命令。

    Args:
        session: 已连接的设备会话。
        _args: 当前命令未使用的参数对象。

    Returns:
        设备返回的命令状态码。
    """

    ensure_command_ok(CmdId.KILL, session.kill())
    return 0


def cmd_reboot(session: DeviceSession, _args) -> int:
    """发送重启命令。

    Args:
        session: 已连接的设备会话。
        _args: 当前命令未使用的参数对象。

    Returns:
        设备返回的命令状态码。
    """

    ensure_command_ok(CmdId.REBOOT, session.reboot())
    return 0


def cmd_get(session: DeviceSession, args) -> int:
    """读取并打印单个参数。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `name`。

    Returns:
        固定返回 `0`。
    """

    print(session.get_param(args.name))
    return 0


def cmd_set(session: DeviceSession, args) -> int:
    """写入单个参数并打印设备回显值。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `name`、`type` 和 `value`。

    Returns:
        固定返回 `0`。

    Raises:
        KeyError: 参数类型不在支持集合内时抛出。
        ValueError: 文本值无法转换为目标参数类型时抛出。
    """

    type_id = {"bool": 0, "u8": 1, "u32": 2, "i32": 3, "float": 4}[args.type]
    print(session.set_param(args.name, type_id, args.value))
    return 0


def cmd_list(session: DeviceSession, _args) -> int:
    """枚举并打印全部参数。

    Args:
        session: 已连接的设备会话。
        _args: 当前命令未使用的参数对象。

    Returns:
        固定返回 `0`。
    """

    for item in session.list_params(timeout=3.0):
        print(item)
    return 0


def cmd_save(session: DeviceSession, _args) -> int:
    """请求设备持久化当前参数。

    Args:
        session: 已连接的设备会话。
        _args: 当前命令未使用的参数对象。

    Returns:
        固定返回 `0`。
    """

    session.save_params()
    return 0


def cmd_reset(session: DeviceSession, _args) -> int:
    """请求设备恢复默认参数。

    Args:
        session: 已连接的设备会话。
        _args: 当前命令未使用的参数对象。

    Returns:
        固定返回 `0`。
    """

    session.reset_params()
    return 0


def cmd_export(session: DeviceSession, args) -> int:
    """导出当前参数快照到 JSON 文件。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `output`。

    Returns:
        固定返回 `0`。

    Raises:
        OSError: 输出文件不可写时抛出。
    """

    session.export_params(Path(args.output))
    print(args.output)
    return 0


def cmd_import(session: DeviceSession, args) -> int:
    """从 JSON 文件导入参数。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `input` 和 `save`。

    Returns:
        固定返回 `0`。

    Raises:
        OSError: 输入文件不可读时抛出。
        ValueError: 文件内容无法转换为协议参数值时抛出。
    """

    applied = session.import_params(Path(args.input), save_after=args.save)
    print(f"applied {len(applied)} parameters from {args.input}")
    return 0


def cmd_stream(session: DeviceSession, args) -> int:
    """打开或关闭遥测流。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `state`，取值为 `on` 或 `off`。

    Returns:
        固定返回 `0`。
    """

    if args.state == "on":
        session.start_stream()
    else:
        session.stop_stream()
    return 0


def cmd_log(session: DeviceSession, args) -> int:
    """在给定时长内打印事件日志，可选打印遥测。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `timeout` 和 `telemetry`。

    Returns:
        固定返回 `0`。

    注意:
        若开启 `--telemetry`，该命令会主动打开遥测流，但不会在结束时显式停流。
    """

    def on_event(message: str) -> None:
        print(f"EVENT {message}")

    def on_telemetry(sample: TelemetrySample) -> None:
        if not args.telemetry:
            return
        print(sample.to_display_map())

    event_token = session.subscribe_event_log(on_event)
    telemetry_token = session.subscribe_telemetry(on_telemetry)
    try:
        if args.telemetry:
            session.start_stream()
        time.sleep(args.timeout)
        return 0
    finally:
        session.unsubscribe(event_token)
        session.unsubscribe(telemetry_token)


def cmd_rate_status(session: DeviceSession, args) -> int:
    """Print rate-loop bench telemetry with per-axis filtering."""

    samples: deque[TelemetrySample] = deque(maxlen=1)

    def on_telemetry(sample: TelemetrySample) -> None:
        samples.append(sample)

    token = session.subscribe_telemetry(on_telemetry)
    had_sample = False
    next_emit = time.monotonic()
    try:
        session.start_stream()
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            if not samples:
                time.sleep(0.02)
                continue
            if time.monotonic() < next_emit:
                time.sleep(0.02)
                continue

            sample = samples[-1]
            had_sample = True
            if args.axis == "all":
                print(format_rate_status_line_all(sample))
            else:
                print(format_rate_status_line(sample, args.axis))
            next_emit = time.monotonic() + args.interval

        if not had_sample:
            print("rate-status did not receive telemetry within timeout")
            return 1
        return 0
    finally:
        try:
            session.stop_stream()
        except Exception:
            pass
        session.unsubscribe(token)


def cmd_attitude_capture_ref(session: DeviceSession, _args) -> int:
    """Capture the natural hanging reference for the bench-only attitude test."""

    require_attitude_hang_capability(session)
    ensure_command_ok(CmdId.ATTITUDE_CAPTURE_REF, session.attitude_capture_ref())
    return 0


def cmd_attitude_test(session: DeviceSession, args) -> int:
    """Start or stop the bench-only hang-attitude outer loop."""

    require_attitude_hang_capability(session)
    if args.action == "start":
        if args.base_duty is not None:
            session.set_param("attitude_test_base_duty", 4, args.base_duty)
        ensure_command_ok(CmdId.ATTITUDE_TEST_START, session.attitude_test_start())
        return 0

    ensure_command_ok(CmdId.ATTITUDE_TEST_STOP, session.attitude_test_stop())
    return 0


def cmd_attitude_status(session: DeviceSession, args) -> int:
    """Print one hang-attitude bench status snapshot."""

    require_attitude_hang_capability(session)
    sample = wait_for_one_sample(session, timeout=args.timeout)
    print(format_attitude_status_line_all(sample))
    return 0


def cmd_watch_attitude(session: DeviceSession, args) -> int:
    """Watch hang-attitude bench telemetry for roll, pitch, or both axes."""

    require_attitude_hang_capability(session)
    samples: deque[TelemetrySample] = deque(maxlen=1)

    def on_telemetry(sample: TelemetrySample) -> None:
        samples.append(sample)

    token = session.subscribe_telemetry(on_telemetry)
    had_sample = False
    next_emit = time.monotonic()
    try:
        session.start_stream()
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            if not samples:
                time.sleep(0.02)
                continue
            if time.monotonic() < next_emit:
                time.sleep(0.02)
                continue

            sample = samples[-1]
            had_sample = True
            if args.axis == "all":
                print(format_attitude_status_line_all(sample))
            else:
                print(format_attitude_status_line(sample, args.axis))
            next_emit = time.monotonic() + args.interval

        if not had_sample:
            print("watch-attitude did not receive telemetry within timeout")
            return 1
        return 0
    finally:
        try:
            session.stop_stream()
        except Exception:
            pass
        session.unsubscribe(token)


def cmd_ground_capture_ref(session: DeviceSession, _args) -> int:
    require_ground_tune_capability(session)
    ensure_command_ok(CmdId.GROUND_CAPTURE_REF, session.ground_capture_ref())
    return 0


def cmd_ground_test(session: DeviceSession, args) -> int:
    require_ground_tune_capability(session)
    if args.action == "start":
        ensure_command_ok(CmdId.GROUND_TEST_START, session.ground_test_start(base_duty=args.base_duty))
        return 0
    ensure_command_ok(CmdId.GROUND_TEST_STOP, session.ground_test_stop())
    return 0


def cmd_ground_status(session: DeviceSession, args) -> int:
    require_ground_tune_capability(session)
    sample = wait_for_one_sample(session, timeout=args.timeout)
    print(format_ground_status_line_all(sample))
    return 0


def cmd_watch_ground(session: DeviceSession, args) -> int:
    require_ground_tune_capability(session)
    samples: deque[TelemetrySample] = deque(maxlen=1)

    def on_telemetry(sample: TelemetrySample) -> None:
        samples.append(sample)

    token = session.subscribe_telemetry(on_telemetry)
    had_sample = False
    next_emit = time.monotonic()
    try:
        session.start_stream()
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            if not samples:
                time.sleep(0.02)
                continue
            if time.monotonic() < next_emit:
                time.sleep(0.02)
                continue
            sample = samples[-1]
            had_sample = True
            if args.axis == "all":
                print(format_ground_status_line_all(sample))
            else:
                print(format_ground_status_line(sample, args.axis))
            next_emit = time.monotonic() + args.interval
        if not had_sample:
            print("watch-ground did not receive telemetry within timeout")
            return 1
        return 0
    finally:
        try:
            session.stop_stream()
        except Exception:
            pass
        session.unsubscribe(token)


def cmd_ground_bench(session: DeviceSession, args) -> int:
    require_ground_tune_capability(session)
    result = run_ground_bench_round(
        session,
        Path(args.output_root),
        axis=args.axis,
        duration_s=args.duration,
        base_duty=args.base_duty,
        auto_arm=args.auto_arm,
        mode=args.mode,
    )
    print(f"saved_dir={result.output_dir}")
    print(f"safe_to_continue={result.summary.safe_to_continue}")
    print(f"next_action_hint={result.summary.next_action_hint}")
    print(f"telemetry_csv={result.telemetry_csv}")
    print(f"summary_json={result.summary_json}")
    print(f"summary_md={result.summary_md}")
    return 0 if result.summary.safe_to_continue else 2


def cmd_ground_log(session: DeviceSession, args) -> int:
    require_ground_tune_capability(session)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    path = output_dir / f"{stamp}_ground_tune_log.csv"
    rows = session.dump_csv(path, duration_s=args.duration)
    print(f"ground_tune_log={path}")
    print(f"rows={rows}")
    return 0 if rows > 0 else 1


def cmd_attitude_ground_verify(session: DeviceSession, args) -> int:
    require_attitude_ground_verify_capability(session)
    if args.action == "start":
        ensure_command_ok(
            CmdId.ATTITUDE_GROUND_VERIFY_START,
            session.attitude_ground_verify_start(base_duty=args.base_duty),
        )
        return 0
    if args.action == "stop":
        ensure_command_ok(CmdId.ATTITUDE_GROUND_VERIFY_STOP, session.attitude_ground_verify_stop())
        return 0
    if args.action == "target":
        ensure_command_ok(
            CmdId.ATTITUDE_GROUND_SET_TARGET,
            session.attitude_ground_set_target(axis_name_to_index(args.axis), args.deg),
        )
        return 0
    raise SystemExit(f"unsupported attitude-ground-verify action {args.action}")


def cmd_attitude_ground_log(session: DeviceSession, args) -> int:
    require_attitude_ground_verify_capability(session)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    path = output_dir / f"{stamp}_attitude_ground_verify_log.csv"
    rows = session.dump_csv(path, duration_s=args.duration)
    print(f"attitude_ground_verify_log={path}")
    print(f"rows={rows}")
    return 0 if rows > 0 else 1


def cmd_attitude_ground_verify_round(session: DeviceSession, args) -> int:
    require_attitude_ground_verify_capability(session)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    path = output_dir / f"{stamp}_attitude_ground_verify_round.csv"
    samples: list[TelemetrySample] = []
    result: dict[str, object] | None = None
    round_error: Exception | None = None

    if not args.no_apply_safe_params:
        for name, type_id, value in GROUND_VERIFY_SAFE_PARAMS:
            session.set_param(name, type_id, value)

    target_deg = abs(float(args.target_deg))
    if target_deg > 2.0:
        raise ValueError("attitude ground verify target must be <= 2 deg")

    def on_telemetry(sample: TelemetrySample) -> None:
        samples.append(sample)

    token = session.subscribe_telemetry(on_telemetry)
    started_stream = False
    started_log = False
    started_verify = False
    try:
        session.start_csv_log(path)
        started_log = True
        time.sleep(args.settle_s)

        ensure_command_ok(CmdId.GROUND_CAPTURE_REF, session.ground_capture_ref())
        if args.auto_arm:
            ensure_command_ok(CmdId.ARM, session.arm())
            time.sleep(0.4)

        ensure_command_ok(
            CmdId.ATTITUDE_GROUND_VERIFY_START,
            session.attitude_ground_verify_start(base_duty=args.base_duty),
        )
        started_verify = True
        session.start_stream()
        started_stream = True
        time.sleep(args.zero_s)

        sequence = [
            ("roll", target_deg),
            ("roll", 0.0),
            ("roll", -target_deg),
            ("roll", 0.0),
            ("pitch", target_deg),
            ("pitch", 0.0),
            ("pitch", -target_deg),
            ("pitch", 0.0),
        ]
        for axis_name, target in sequence:
            ensure_command_ok(
                CmdId.ATTITUDE_GROUND_SET_TARGET,
                session.attitude_ground_set_target(axis_name_to_index(axis_name), target),
            )
            time.sleep(args.zero_s if abs(target) <= 0.01 else args.step_s)

        result = analyze_attitude_ground_verify_samples(samples, target_deg)
    except Exception as exc:
        round_error = exc
    finally:
        if started_verify:
            try:
                session.attitude_ground_set_target(axis_name_to_index("roll"), 0.0)
                session.attitude_ground_set_target(axis_name_to_index("pitch"), 0.0)
                session.attitude_ground_verify_stop()
            except Exception:
                pass
        if args.auto_arm:
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
            session.stop_csv_log()
        session.unsubscribe(token)

    if result is None:
        result = analyze_attitude_ground_verify_samples(samples, target_deg)
    if round_error is not None:
        result["passed"] = False
        result["round_error"] = str(round_error)

    print(f"attitude_ground_verify_round_log={path}")
    if round_error is not None:
        print(f"round_error={round_error}")
    for line in format_attitude_ground_verify_summary(result):
        print(line)
    return 0 if result.get("passed", False) else 2


def cmd_liftoff_verify(session: DeviceSession, args) -> int:
    require_low_risk_liftoff_capability(session)
    if args.action == "start":
        ensure_command_ok(CmdId.LIFTOFF_VERIFY_START, session.liftoff_verify_start(base_duty=args.base_duty))
        return 0
    ensure_command_ok(CmdId.LIFTOFF_VERIFY_STOP, session.liftoff_verify_stop())
    return 0


def cmd_liftoff_verify_round(session: DeviceSession, args) -> int:
    require_low_risk_liftoff_capability(session)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    path = output_dir / f"{stamp}_liftoff_verify_round.csv"
    samples: list[TelemetrySample] = []
    result: dict[str, object] | None = None
    round_error: Exception | None = None

    base_duty = float(args.base_duty)
    duration_s = float(args.duration_s)
    if base_duty < 0.0 or base_duty > 0.12:
        raise ValueError("first liftoff verify base duty must be <= 0.12")
    if duration_s <= 0.0 or duration_s > 2.2:
        raise ValueError("first liftoff verify duration must be >0 and <=2.2 seconds")

    if not args.no_apply_safe_params:
        for name, type_id, value in LIFTOFF_VERIFY_SAFE_PARAMS:
            session.set_param(name, type_id, value)
        session.set_param("liftoff_verify_base_duty", 4, base_duty)

    def on_telemetry(sample: TelemetrySample) -> None:
        samples.append(sample)

    token = session.subscribe_telemetry(on_telemetry)
    started_stream = False
    started_log = False
    started_liftoff = False
    auto_arm = not args.no_auto_arm
    try:
        session.start_csv_log(path)
        started_log = True
        time.sleep(args.settle_s)

        ensure_command_ok(CmdId.GROUND_CAPTURE_REF, session.ground_capture_ref())
        if auto_arm:
            ensure_command_ok(CmdId.ARM, session.arm())
            time.sleep(0.35)

        ensure_command_ok(CmdId.LIFTOFF_VERIFY_START, session.liftoff_verify_start(base_duty=base_duty))
        started_liftoff = True
        session.start_stream()
        started_stream = True

        deadline = time.monotonic() + duration_s
        saw_active = False
        while time.monotonic() < deadline:
            if samples:
                latest = samples[-1]
                active = (
                    latest.control_mode == CONTROL_MODE_ATTITUDE_GROUND_TUNE and
                    latest.control_submode == GROUND_TUNE_SUBMODE_LOW_RISK_LIFTOFF
                )
                saw_active = saw_active or active
                if saw_active and not active:
                    break
            time.sleep(0.03)

        result = analyze_liftoff_verify_samples(samples, base_duty)
    except Exception as exc:
        round_error = exc
    finally:
        if started_liftoff:
            try:
                session.liftoff_verify_stop()
            except Exception:
                pass
        if auto_arm:
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
            session.stop_csv_log()
        session.unsubscribe(token)

    if result is None:
        result = analyze_liftoff_verify_samples(samples, base_duty)
    if round_error is not None:
        result["control_safe_pass"] = False
        result["passed"] = False
        result["round_error"] = str(round_error)

    print(f"liftoff_verify_round_log={path}")
    if round_error is not None:
        print(f"round_error={round_error}")
    for line in format_liftoff_verify_summary(result):
        print(line)
    return 0 if result.get("passed", False) else 2


def cmd_udp_manual(session: DeviceSession, args) -> int:
    """Run experimental UDP manual-control commands."""

    require_udp_manual_capability(session)
    action = args.action
    if action == "enable":
        ensure_command_ok(CmdId.UDP_MANUAL_ENABLE, session.udp_manual_enable())
        return 0
    if action == "disable":
        ensure_command_ok(CmdId.UDP_MANUAL_DISABLE, session.udp_manual_disable())
        return 0
    if action == "stop":
        ensure_command_ok(CmdId.UDP_MANUAL_STOP, session.udp_manual_stop())
        return 0
    if action == "takeoff":
        ensure_command_ok(CmdId.UDP_TAKEOFF, session.udp_takeoff())
        return 0
    if action == "land":
        ensure_command_ok(CmdId.UDP_LAND, session.udp_land())
        return 0
    if action == "setpoint":
        ensure_command_ok(
            CmdId.UDP_MANUAL_SETPOINT,
            session.udp_manual_setpoint(
                throttle=args.throttle,
                pitch=args.pitch,
                roll=args.roll,
                yaw=args.yaw,
            ),
        )
        return 0
    raise SystemExit(f"unsupported udp-manual action {action}")


def cmd_dump_csv(session: DeviceSession, args) -> int:
    """在指定时长内采集遥测并导出 CSV。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `output` 和 `duration`。

    Returns:
        固定返回 `0`。
    """

    count = session.dump_csv(Path(args.output), duration_s=args.duration)
    print(f"wrote {count} telemetry rows to {args.output}")
    return 0


def cmd_baro(session: DeviceSession, args) -> int:
    """在限定时间内观察气压计遥测。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `timeout`。

    Returns:
        `0` 表示收到有效气压计数据，`1` 表示协议版本不支持、超时或始终无有效数据。

    注意:
        该命令会在退出前主动停止遥测流。
    """

    info = session.device_info or session.hello()
    if info.protocol_version < 2:
        print("current firmware does not advertise baro telemetry support")
        return 1

    samples: deque[TelemetrySample] = deque()

    def on_telemetry(sample: TelemetrySample) -> None:
        samples.append(sample)

    token = session.subscribe_telemetry(on_telemetry)
    had_sample = False
    had_valid_baro = False
    try:
        session.start_stream()
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            if not samples:
                time.sleep(0.05)
                continue

            sample = samples.popleft()
            had_sample = True
            had_valid_baro = had_valid_baro or bool(sample.baro_valid)
            print(
                "pressure_pa={:.1f} temperature_c={:.2f} altitude_m={:.3f} "
                "valid={} update_age_us={}".format(
                    sample.baro_pressure_pa,
                    sample.baro_temperature_c,
                    sample.baro_altitude_m,
                    bool(sample.baro_valid),
                    sample.baro_update_age_us,
                )
            )

        if not had_sample:
            print("device stream did not produce telemetry within timeout")
            return 1
        if not had_valid_baro:
            print("current telemetry stream has no valid baro data")
            return 1
        return 0
    finally:
        try:
            session.stop_stream()
        except Exception:
            pass
        session.unsubscribe(token)


def cmd_motor_test(session: DeviceSession, args) -> int:
    """执行单电机点动测试。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `motor` 和 `duty`。

    Returns:
        设备返回的命令状态码。

    注意:
        `motor` 同时支持 `m1` 这类名称和纯数字索引。
    """

    motor_index = int(args.motor[1]) - 1 if args.motor.lower().startswith("m") else int(args.motor)
    ensure_command_ok(CmdId.MOTOR_TEST, session.motor_test(motor_index, float(args.duty)))
    return 0


def cmd_axis_test(session: DeviceSession, args) -> int:
    """执行开环轴向测试。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `axis` 和 `value`。

    Returns:
        设备返回的命令状态码。
    """

    ensure_command_ok(CmdId.AXIS_TEST, session.axis_test(axis_name_to_index(args.axis), float(args.value)))
    return 0


def cmd_rate_test(session: DeviceSession, args) -> int:
    """执行速率环测试。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `axis` 和 `value`。

    Returns:
        设备返回的命令状态码。
    """

    ensure_command_ok(CmdId.RATE_TEST, session.rate_test(axis_name_to_index(args.axis), float(args.value)))
    return 0


def cmd_calib(session: DeviceSession, args) -> int:
    """执行陀螺或水平校准。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `kind`，取值为 `gyro` 或 `level`。

    Returns:
        设备返回的命令状态码。
    """

    if args.kind == "gyro":
        ensure_command_ok(CmdId.CALIB_GYRO, session.calib_gyro())
    else:
        ensure_command_ok(CmdId.CALIB_LEVEL, session.calib_level())
    return 0


def cmd_roll_bench(session: DeviceSession, args) -> int:
    """Run one conservative constrained roll bench round."""

    args.axis = "roll"
    if not getattr(args, "output_dir", None):
        args.output_dir = "bench_runs/roll"
    return cmd_axis_bench(session, args)


def cmd_axis_bench(session: DeviceSession, args) -> int:
    """Run one conservative constrained single-axis rate bench round."""

    axis_name = args.axis
    output_dir = args.output_dir or f"bench_runs/{axis_name}"

    updates = {
        f"rate_kp_{axis_name}": args.kp,
        f"rate_ki_{axis_name}": args.ki,
        f"rate_kd_{axis_name}": args.kd,
        "rate_integral_limit": args.integral_limit,
        "rate_output_limit": args.output_limit,
        "motor_idle_duty": args.motor_idle_duty,
        "bringup_test_base_duty": args.base_duty,
    }
    applied = apply_axis_bench_params(session, {name: value for name, value in updates.items() if value is not None})
    result = run_axis_bench_round(
        session,
        Path(output_dir),
        axis_name=axis_name,
        auto_arm=args.auto_arm,
        small_step_dps=args.small_step,
        large_step_dps=args.large_step,
        active_duration_s=args.active_duration,
        zero_duration_s=args.zero_duration,
        serial_hint=args.serial,
        tag=args.tag,
        orientation_note=args.orientation_note,
    )
    if args.save_params and applied:
        session.save_params()

    print(result.orientation_note)
    if applied:
        print("applied params:")
        for name, value in applied.items():
            print(f"  {name}={value:.6f}")
    print(
        f"summary axis={axis_name} result={result.summary.axis_result} "
        f"accept={result.summary.accept} "
        f"safe_to_continue={result.summary.safe_to_continue} "
        f"kp_tuning_allowed={result.summary.kp_tuning_allowed}"
    )
    print(
        f"checks setpoint_path_ok={result.summary.setpoint_path_ok} "
        f"sign_ok={result.summary.sign_ok} "
        f"motor_split_ok={result.summary.motor_split_ok} "
        f"measurable_response={result.summary.measurable_response} "
        f"saturation_risk={result.summary.saturation_risk} "
        f"return_to_zero_quality={result.summary.return_to_zero_quality} "
        f"noise_or_jitter_risk={result.summary.noise_or_jitter_risk} "
        f"low_duty_motor_stability={result.summary.low_duty_motor_stability}"
    )
    print(f"next_action_hint={result.summary.next_action_hint}")
    print(f"csv={result.csv_path}")
    print(f"json={result.json_path}")
    print(f"markdown={result.markdown_path}")
    for item in result.metrics:
        print(
            f"{item.name} cmd={item.command_dps:.1f} sp={item.setpoint_mean:.2f} "
            f"fb={item.feedback_mean:.2f} out={item.pid_out_mean:.4f} split={item.motor_split_mean:.4f} "
            f"sat={item.pid_out_saturation_ratio:.2f} sign={item.sign_ok} response={item.measurable_response} "
            f"zero_ok={item.return_to_zero_ok}"
        )
    return 0 if result.summary.safe_to_continue else 2


def build_parser() -> argparse.ArgumentParser:
    """构建 CLI 命令解析器。

    Returns:
        已配置全部子命令和连接参数的解析器对象。
    """

    parser = argparse.ArgumentParser(prog="esp-drone-cli")
    add_common_transport_args(parser)
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("connect")
    sub.add_parser("capabilities", aliases=["device-info"])
    sub.add_parser("arm")
    sub.add_parser("disarm")
    sub.add_parser("kill")
    sub.add_parser("reboot")

    get_p = sub.add_parser("get")
    get_p.add_argument("name")

    set_p = sub.add_parser("set")
    set_p.add_argument("name")
    set_p.add_argument("type", choices=["bool", "u8", "u32", "i32", "float"])
    set_p.add_argument("value")

    sub.add_parser("list")
    sub.add_parser("save")
    sub.add_parser("reset")

    export_p = sub.add_parser("export")
    export_p.add_argument("output")

    import_p = sub.add_parser("import")
    import_p.add_argument("input")
    import_p.add_argument("--save", action="store_true")

    stream_p = sub.add_parser("stream")
    stream_p.add_argument("state", choices=["on", "off"])

    log_p = sub.add_parser("log")
    log_p.add_argument("--timeout", type=float, default=10.0)
    log_p.add_argument("--telemetry", action="store_true")

    rate_status_p = sub.add_parser(
        "rate-status",
        aliases=["watch-rate"],
        help="watch rate-loop telemetry for roll, pitch, yaw, or all axes",
        description="Print rate-loop bench telemetry with per-axis filtering.",
    )
    rate_status_p.add_argument("axis", nargs="?", choices=["roll", "pitch", "yaw", "all"], default="all")
    rate_status_p.add_argument("--timeout", type=float, default=5.0)
    rate_status_p.add_argument("--interval", type=float, default=0.2)

    baro_p = sub.add_parser("baro", aliases=["watch-baro"])
    baro_p.add_argument("--timeout", type=float, default=5.0)

    csv_p = sub.add_parser("dump-csv")
    csv_p.add_argument("output")
    csv_p.add_argument("--duration", type=float, default=5.0)

    motor_p = sub.add_parser("motor-test")
    motor_p.add_argument("motor")
    motor_p.add_argument("duty")

    axis_p = sub.add_parser("axis-test")
    axis_p.add_argument("axis", choices=["roll", "pitch", "yaw"])
    axis_p.add_argument("value")

    rate_p = sub.add_parser("rate-test")
    rate_p.add_argument("axis", choices=["roll", "pitch", "yaw"])
    rate_p.add_argument("value")

    attitude_capture_p = sub.add_parser(
        "attitude-capture-ref",
        help="capture the current +Z-down hanging pose as the bench-only attitude reference",
        description="Capture the current +Z-down hanging pose as the reference for the bench-only attitude outer-loop. Never use this workflow as a free-flight stabilize/angle mode.",
    )
    attitude_capture_p.set_defaults(command="attitude-capture-ref")

    attitude_test_p = sub.add_parser(
        "attitude-test",
        help="bench-only hang attitude outer-loop bring-up; never use for free flight",
        description="Bench-only hang attitude outer-loop bring-up for a constrained rod/hanging rig. This is not a free-flight stabilize/angle mode.",
    )
    attitude_test_sub = attitude_test_p.add_subparsers(dest="action", required=True)
    attitude_test_start_p = attitude_test_sub.add_parser(
        "start",
        help="start the bench-only hang attitude outer loop",
    )
    attitude_test_start_p.add_argument("--base-duty", type=float)
    attitude_test_sub.add_parser("stop", help="stop the bench-only hang attitude outer loop")

    attitude_status_p = sub.add_parser(
        "attitude-status",
        help="print one bench-only hang attitude status snapshot",
        description="Print one telemetry snapshot for the bench-only hang attitude bring-up path.",
    )
    attitude_status_p.add_argument("--timeout", type=float, default=2.0)

    watch_attitude_p = sub.add_parser(
        "watch-attitude",
        help="watch bench-only hang attitude telemetry",
        description="Watch bench-only hang attitude telemetry. Do not treat this as a free-flight stabilize/angle mode.",
    )
    watch_attitude_p.add_argument("axis", nargs="?", choices=["roll", "pitch", "all"], default="all")
    watch_attitude_p.add_argument("--timeout", type=float, default=5.0)
    watch_attitude_p.add_argument("--interval", type=float, default=0.2)

    ground_capture_p = sub.add_parser(
        "ground-capture-ref",
        help="capture the current flat-ground pose as the ground tune reference",
        description="Capture the current flat-ground pose for the low-throttle ground tune mode.",
    )
    ground_capture_p.set_defaults(command="ground-capture-ref")

    ground_test_p = sub.add_parser(
        "ground-test",
        help="start or stop flat-ground low-throttle tune mode",
        description="Flat-ground low-throttle attitude tune mode. Roll/pitch use the ground outer loop; yaw remains rate-only.",
    )
    ground_test_sub = ground_test_p.add_subparsers(dest="action", required=True)
    ground_test_start_p = ground_test_sub.add_parser("start")
    ground_test_start_p.add_argument("--base-duty", type=float)
    ground_test_sub.add_parser("stop")

    ground_status_p = sub.add_parser("ground-status", help="print one ground tune telemetry snapshot")
    ground_status_p.add_argument("--timeout", type=float, default=5.0)

    watch_ground_p = sub.add_parser("watch-ground", help="watch ground tune telemetry")
    watch_ground_p.add_argument("axis", nargs="?", choices=["roll", "pitch", "yaw", "all"], default="all")
    watch_ground_p.add_argument("--timeout", type=float, default=10.0)
    watch_ground_p.add_argument("--interval", type=float, default=0.1)

    ground_bench_p = sub.add_parser(
        "ground-bench",
        help="run a flat-ground low-throttle bench recording and summary",
        description="Run ground-capture-ref, ground-test start/stop, telemetry logging, and conservative summary generation.",
    )
    ground_bench_p.add_argument("axis", choices=["roll", "pitch", "yaw", "all"])
    ground_bench_p.add_argument("--output-root", default="logs")
    ground_bench_p.add_argument("--duration", type=float, default=5.0)
    ground_bench_p.add_argument("--base-duty", type=float)
    ground_bench_p.add_argument("--auto-arm", action="store_true")
    ground_bench_p.add_argument("--mode", default="manual_perturb")

    ground_log_p = sub.add_parser(
        "ground-log",
        help="record a clearly named flat-ground tune telemetry CSV",
        description="Start the telemetry stream, record a ground tune CSV for the requested duration, then close the file.",
    )
    ground_log_p.add_argument("--duration", type=float, default=10.0)
    ground_log_p.add_argument("--output-dir", default="logs")

    attitude_ground_verify_p = sub.add_parser(
        "attitude-ground-verify",
        help="verify the +Z-up flat-ground attitude outer-loop chain",
        description="Flat-ground attitude outer-loop verification only. It verifies signs, clamps, telemetry, and failsafes before any low-risk liftoff work.",
    )
    attitude_ground_verify_sub = attitude_ground_verify_p.add_subparsers(dest="action", required=True)
    attitude_ground_verify_start_p = attitude_ground_verify_sub.add_parser("start")
    attitude_ground_verify_start_p.add_argument("--base-duty", type=float)
    attitude_ground_verify_sub.add_parser("stop")
    attitude_ground_verify_target_p = attitude_ground_verify_sub.add_parser("target")
    attitude_ground_verify_target_p.add_argument("axis", choices=["roll", "pitch", "yaw"])
    attitude_ground_verify_target_p.add_argument("deg", type=float)

    attitude_ground_log_p = sub.add_parser(
        "attitude-ground-log",
        help="record a clearly named attitude ground verification CSV",
        description="Start telemetry streaming, record an attitude ground verify CSV, then close the file.",
    )
    attitude_ground_log_p.add_argument("--duration", type=float, default=10.0)
    attitude_ground_log_p.add_argument("--output-dir", default="logs")

    attitude_ground_round_p = sub.add_parser(
        "attitude-ground-round",
        help="run one very small +Z-up attitude ground verify sequence and record CSV",
        description="Capture flat-ground reference, optionally arm, start attitude-ground-verify, apply +/- small roll/pitch angle targets, record telemetry, stop, and print sign/clamp checks.",
    )
    attitude_ground_round_p.add_argument("--target-deg", type=float, default=1.0)
    attitude_ground_round_p.add_argument("--base-duty", type=float, default=0.08)
    attitude_ground_round_p.add_argument("--step-s", type=float, default=1.0)
    attitude_ground_round_p.add_argument("--zero-s", type=float, default=0.6)
    attitude_ground_round_p.add_argument("--settle-s", type=float, default=0.8)
    attitude_ground_round_p.add_argument("--output-dir", default="logs")
    attitude_ground_round_p.add_argument("--auto-arm", action="store_true")
    attitude_ground_round_p.add_argument("--no-apply-safe-params", action="store_true")

    liftoff_verify_p = sub.add_parser(
        "liftoff-verify",
        help="start or stop the low-risk short liftoff verification mode",
        description="Explicit low-risk liftoff verification mode. It uses the same ground-reference attitude outer loop and tuned rate PID path with conservative limits.",
    )
    liftoff_verify_sub = liftoff_verify_p.add_subparsers(dest="action", required=True)
    liftoff_verify_start_p = liftoff_verify_sub.add_parser("start")
    liftoff_verify_start_p.add_argument("--base-duty", type=float)
    liftoff_verify_sub.add_parser("stop")

    liftoff_round_p = sub.add_parser(
        "liftoff-round",
        help="run one very short low-risk liftoff verification round and record CSV",
        description="Capture a flat-ground reference, arm, run one conservative low-risk liftoff verify attempt, record telemetry, stop, and print safety/path checks. This is not hover tuning.",
    )
    liftoff_round_p.add_argument("--base-duty", type=float, default=0.10)
    liftoff_round_p.add_argument("--duration-s", type=float, default=2.0)
    liftoff_round_p.add_argument("--settle-s", type=float, default=1.0)
    liftoff_round_p.add_argument("--output-dir", default="logs")
    liftoff_round_p.add_argument("--no-auto-arm", action="store_true")
    liftoff_round_p.add_argument("--no-apply-safe-params", action="store_true")

    udp_manual_p = sub.add_parser(
        "udp-manual",
        help="experimental UDP manual control; not free-flight ready",
        description="Experimental UDP manual control for restrained testing only. Throttle is the base duty target; roll/pitch use the flat-ground reference outer loop and yaw uses the rate PID before mixing. This is not a mature takeoff/land/free-flight mode.",
    )
    udp_manual_sub = udp_manual_p.add_subparsers(dest="action", required=True)
    udp_manual_sub.add_parser("enable")
    udp_manual_sub.add_parser("disable")
    udp_manual_sub.add_parser("stop")
    udp_manual_sub.add_parser("takeoff")
    udp_manual_sub.add_parser("land")
    udp_manual_setpoint_p = udp_manual_sub.add_parser("setpoint")
    udp_manual_setpoint_p.add_argument("--throttle", type=float, required=True)
    udp_manual_setpoint_p.add_argument("--pitch", type=float, default=0.0)
    udp_manual_setpoint_p.add_argument("--roll", type=float, default=0.0)
    udp_manual_setpoint_p.add_argument("--yaw", type=float, default=0.0)

    calib_p = sub.add_parser("calib")
    calib_p.add_argument("kind", choices=["gyro", "level"])

    axis_bench_p = sub.add_parser(
        "axis-bench",
        aliases=["rate-bench"],
        help="run one conservative constrained single-axis rate bench round",
        description="Run a bench-only single-axis rate step sequence, save telemetry artifacts, and summarize sign, response, saturation, jitter, and low-duty checks.",
    )
    axis_bench_p.add_argument("axis", choices=["roll", "pitch", "yaw"])
    axis_bench_p.add_argument("--output-dir")
    axis_bench_p.add_argument("--tag")
    axis_bench_p.add_argument("--small-step", type=float, default=10.0)
    axis_bench_p.add_argument("--large-step", type=float, default=20.0)
    axis_bench_p.add_argument("--active-duration", type=float, default=0.8)
    axis_bench_p.add_argument("--zero-duration", type=float, default=0.7)
    axis_bench_p.add_argument("--auto-arm", action="store_true")
    axis_bench_p.add_argument("--kp", type=float)
    axis_bench_p.add_argument("--ki", type=float)
    axis_bench_p.add_argument("--kd", type=float)
    axis_bench_p.add_argument("--integral-limit", type=float)
    axis_bench_p.add_argument("--output-limit", type=float)
    axis_bench_p.add_argument("--motor-idle-duty", type=float)
    axis_bench_p.add_argument("--base-duty", type=float)
    axis_bench_p.add_argument("--orientation-note")
    axis_bench_p.add_argument("--save-params", action="store_true")

    roll_bench_p = sub.add_parser(
        "roll-bench",
        help="run one conservative constrained rate-roll bench round",
        description="Run a bench-only rate roll step sequence, save telemetry artifacts, and summarize sign/response/saturation checks.",
    )
    roll_bench_p.add_argument("--output-dir")
    roll_bench_p.add_argument("--tag")
    roll_bench_p.add_argument("--small-step", type=float, default=10.0)
    roll_bench_p.add_argument("--large-step", type=float, default=20.0)
    roll_bench_p.add_argument("--active-duration", type=float, default=0.8)
    roll_bench_p.add_argument("--zero-duration", type=float, default=0.7)
    roll_bench_p.add_argument("--auto-arm", action="store_true")
    roll_bench_p.add_argument("--kp", type=float)
    roll_bench_p.add_argument("--ki", type=float)
    roll_bench_p.add_argument("--kd", type=float)
    roll_bench_p.add_argument("--integral-limit", type=float)
    roll_bench_p.add_argument("--output-limit", type=float)
    roll_bench_p.add_argument("--motor-idle-duty", type=float)
    roll_bench_p.add_argument("--base-duty", type=float)
    roll_bench_p.add_argument("--orientation-note")
    roll_bench_p.add_argument("--save-params", action="store_true")

    return parser


def main(argv: list[str] | None = None) -> int:
    """执行 CLI 主流程。

    Args:
        argv: 可选命令行参数列表；为 `None` 时读取进程实际参数。

    Returns:
        命令返回码。成功通常为 `0`，设备命令失败时返回设备状态码或子命令定义的错误码。

    Raises:
        SystemExit: `argparse` 参数校验失败时抛出。
        TimeoutError: 设备在规定时间内未返回预期响应时由底层会话抛出。

    注意:
        函数结束时始终会关闭设备会话。
    """

    parser = build_parser()
    args = parser.parse_args(argv)
    session: DeviceSession | None = None
    try:
        session = connect_session_from_args(args)
        dispatch = {
            "connect": cmd_connect,
            "capabilities": cmd_capabilities,
            "device-info": cmd_capabilities,
            "arm": cmd_arm,
            "disarm": cmd_disarm,
            "kill": cmd_kill,
            "reboot": cmd_reboot,
            "get": cmd_get,
            "set": cmd_set,
            "list": cmd_list,
            "save": cmd_save,
            "reset": cmd_reset,
            "export": cmd_export,
            "import": cmd_import,
            "stream": cmd_stream,
            "log": cmd_log,
            "rate-status": cmd_rate_status,
            "watch-rate": cmd_rate_status,
            "baro": cmd_baro,
            "watch-baro": cmd_baro,
            "dump-csv": cmd_dump_csv,
            "motor-test": cmd_motor_test,
            "axis-test": cmd_axis_test,
            "rate-test": cmd_rate_test,
            "attitude-capture-ref": cmd_attitude_capture_ref,
            "attitude-test": cmd_attitude_test,
            "attitude-status": cmd_attitude_status,
            "watch-attitude": cmd_watch_attitude,
            "ground-capture-ref": cmd_ground_capture_ref,
            "ground-test": cmd_ground_test,
            "ground-status": cmd_ground_status,
            "watch-ground": cmd_watch_ground,
            "ground-bench": cmd_ground_bench,
            "ground-log": cmd_ground_log,
            "attitude-ground-verify": cmd_attitude_ground_verify,
            "attitude-ground-log": cmd_attitude_ground_log,
            "attitude-ground-round": cmd_attitude_ground_verify_round,
            "liftoff-verify": cmd_liftoff_verify,
            "liftoff-round": cmd_liftoff_verify_round,
            "udp-manual": cmd_udp_manual,
            "calib": cmd_calib,
            "axis-bench": cmd_axis_bench,
            "rate-bench": cmd_axis_bench,
            "roll-bench": cmd_roll_bench,
        }
        return int(dispatch[args.command](session, args) or 0)
    except CommandError as exc:
        print(str(exc), file=sys.stderr)
        return int(exc.status or 1)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        if session is not None:
            session.close()


if __name__ == "__main__":
    sys.exit(main())
