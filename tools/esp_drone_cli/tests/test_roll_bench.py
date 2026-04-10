from __future__ import annotations

from types import SimpleNamespace

import pytest

from esp_drone_cli.core.models import TelemetrySample
from esp_drone_cli.core.roll_bench import (
    RollBenchSafetyTrip,
    RollBenchStep,
    analyze_axis_bench_round,
    analyze_roll_bench_round,
    run_roll_bench_round,
)


def build_telemetry_payload() -> bytes:
    values = [
        123456789,
        1.0, 2.0, 3.0,
        0.1, 0.2, 0.3,
        1.0, 0.0, 0.0, 0.0,
        4.0, 5.0, 6.0,
        0.0, 0.0, 0.0,
        10.0, 11.0, 12.0,
        0.5, 0.6, 0.7,
        0.1, 0.2, 0.3,
        0.01, 0.02, 0.03,
        0.9, 1.0, 1.1,
        0.20, 0.21, 0.22, 0.23,
        3.8,
        2048, 1000, 500,
        1, 1, 0, 0, 2, 0, 0, 0,
        100845.0, 26.5, 1.25, -0.10,
        15000,
        1, 1, 0, 0,
    ]
    from esp_drone_cli.core.models import TELEMETRY_STRUCT

    return TELEMETRY_STRUCT.pack(*values)


def _sample_with_roll(command_dps: float, feedback_dps: float, pid_out: float, split: float) -> TelemetrySample:
    sample = TelemetrySample.from_payload(build_telemetry_payload())
    sample.rate_setpoint_roll = command_dps
    sample.gyro_y = -feedback_dps
    sample.pid_out_roll = pid_out
    sample.motor1 = 0.15 + split
    sample.motor4 = 0.15 + split
    sample.motor2 = 0.15 - split
    sample.motor3 = 0.15 - split
    return sample


def _sample_with_pitch(command_dps: float, feedback_dps: float, pid_out: float, split: float) -> TelemetrySample:
    sample = TelemetrySample.from_payload(build_telemetry_payload())
    sample.rate_setpoint_pitch = command_dps
    sample.gyro_x = feedback_dps
    sample.pid_out_pitch = pid_out
    sample.motor1 = 0.15 - split
    sample.motor2 = 0.15 - split
    sample.motor3 = 0.15 + split
    sample.motor4 = 0.15 + split
    return sample


def _sample_with_yaw(command_dps: float, feedback_dps: float, pid_out: float, split: float) -> TelemetrySample:
    sample = TelemetrySample.from_payload(build_telemetry_payload())
    sample.rate_setpoint_yaw = command_dps
    sample.gyro_z = -feedback_dps
    sample.pid_out_yaw = pid_out
    sample.motor1 = 0.15 + split
    sample.motor2 = 0.15 - split
    sample.motor3 = 0.15 + split
    sample.motor4 = 0.15 - split
    return sample


class FakeRollBenchSession:
    def __init__(
        self,
        *,
        arm_state: int = 0,
        failsafe_reason: int = 0,
        trip_on_nonzero: bool = False,
    ) -> None:
        self.calls: list[str] = []
        self._callbacks: dict[int, object] = {}
        self._next_callback_id = 1
        self._timestamp_us = 123456789
        self._trip_on_nonzero = trip_on_nonzero
        self._latest = _sample_with_roll(0.0, 0.0, 0.0, 0.0)
        self._latest.arm_state = arm_state
        self._latest.failsafe_reason = failsafe_reason
        self._latest.control_mode = 0
        self._latest.imu_health = 1
        self._latest.timestamp_us = self._timestamp_us
        self._params = {
            "rate_kp_roll": 0.003,
            "rate_ki_roll": 0.0,
            "rate_kd_roll": 0.0,
            "rate_integral_limit": 100.0,
            "rate_output_limit": 0.2,
            "bringup_test_base_duty": 0.15,
            "motor_idle_duty": 0.05,
            "motor_max_duty": 1.0,
            "telemetry_usb_hz": 200.0,
        }

    def _emit(self, *, command_dps: float, feedback_dps: float, pid_out: float, split: float, repeats: int) -> None:
        for _ in range(repeats):
            self._timestamp_us += 1000
            sample = _sample_with_roll(command_dps, feedback_dps, pid_out, split)
            sample.arm_state = self._latest.arm_state
            sample.failsafe_reason = self._latest.failsafe_reason
            sample.control_mode = self._latest.control_mode
            sample.imu_health = 1
            sample.timestamp_us = self._timestamp_us
            self._latest = sample
            for callback in list(self._callbacks.values()):
                callback(sample)

    def subscribe_telemetry(self, callback):
        callback_id = self._next_callback_id
        self._next_callback_id += 1
        self._callbacks[callback_id] = callback
        return callback_id

    def unsubscribe(self, callback_id: int) -> None:
        self._callbacks.pop(callback_id, None)

    def start_csv_log(self, output_path) -> None:
        output_path.write_text("", encoding="utf-8")
        self.calls.append("start_csv_log")

    def stop_csv_log(self) -> None:
        self.calls.append("stop_csv_log")

    def start_stream(self) -> None:
        self.calls.append("start_stream")
        self._emit(command_dps=0.0, feedback_dps=0.0, pid_out=0.0, split=0.0, repeats=1)

    def stop_stream(self) -> None:
        self.calls.append("stop_stream")

    def get_latest_telemetry(self) -> TelemetrySample:
        return self._latest

    def get_param(self, name: str):
        return SimpleNamespace(name=name, value=self._params[name])

    def arm(self) -> int:
        self.calls.append("arm")
        self._latest.arm_state = 1
        self._latest.failsafe_reason = 0
        self._emit(command_dps=0.0, feedback_dps=0.0, pid_out=0.0, split=0.0, repeats=1)
        return 0

    def disarm(self) -> int:
        self.calls.append("disarm")
        self._latest.arm_state = 0
        self._emit(command_dps=0.0, feedback_dps=0.0, pid_out=0.0, split=0.0, repeats=1)
        return 0

    def kill(self) -> int:
        self.calls.append("kill")
        self._latest.arm_state = 3
        self._latest.failsafe_reason = 1
        self._emit(command_dps=0.0, feedback_dps=0.0, pid_out=0.0, split=0.0, repeats=1)
        return 0

    def rate_test(self, axis_index: int, value_dps: float) -> int:
        self.calls.append(f"rate_test:{axis_index}:{value_dps:.1f}")
        if self._latest.arm_state != 1:
            return 1
        sign = 1.0 if value_dps > 0.0 else -1.0 if value_dps < 0.0 else 0.0
        if abs(value_dps) <= 0.01:
            self._latest.control_mode = 4
            self._emit(command_dps=0.0, feedback_dps=0.0, pid_out=0.0, split=0.0, repeats=2)
            return 0
        if self._trip_on_nonzero:
            self._latest.control_mode = 4
            self._emit(
                command_dps=value_dps,
                feedback_dps=-2.0 * sign,
                pid_out=-0.19 * sign,
                split=-0.08 * sign,
                repeats=25,
            )
            return 0
        self._latest.control_mode = 4
        self._emit(
            command_dps=value_dps,
            feedback_dps=0.30 * value_dps,
            pid_out=0.03 * sign,
            split=0.02 * sign,
            repeats=3,
        )
        return 0


def test_analyze_roll_bench_round_accepts_correct_sign_and_response():
    steps = [
        (
            RollBenchStep("pos_small", 10.0, 0.8, "command"),
            [_sample_with_roll(10.0, 2.8, 0.035, 0.030) for _ in range(16)],
        ),
        (
            RollBenchStep("zero_after", 0.0, 0.7, "zero"),
            [_sample_with_roll(0.0, 1.2, 0.0, 0.0) for _ in range(16)],
        ),
        (
            RollBenchStep("neg_small", -10.0, 0.8, "command"),
            [_sample_with_roll(-10.0, -2.9, -0.034, -0.031) for _ in range(16)],
        ),
        (
            RollBenchStep("post_zero", 0.0, 0.7, "zero"),
            [_sample_with_roll(0.0, 1.0, 0.0, 0.0) for _ in range(16)],
        ),
    ]
    metrics, summary = analyze_roll_bench_round(
        steps,
        {"rate_output_limit": 0.2, "rate_ki_roll": 0.0},
    )

    assert metrics[0].sign_ok is True
    assert metrics[0].measurable_response is True
    assert metrics[2].sign_ok is True
    assert summary.setpoint_path_ok is True
    assert summary.sign_ok is True
    assert summary.measurable_response is True
    assert summary.saturation_risk is False
    assert summary.accept is True


def test_analyze_roll_bench_round_flags_saturation_and_sign_failure():
    bad_sign = [_sample_with_roll(10.0, -1.5, -0.18, -0.08) for _ in range(16)]
    saturated = [_sample_with_roll(-10.0, -1.8, -0.195, -0.075) for _ in range(16)]
    for sample in saturated:
        sample.motor1 = 0.0
        sample.motor4 = 0.0
    steps = [
        (RollBenchStep("pos_small", 10.0, 0.8, "command"), bad_sign),
        (RollBenchStep("neg_small", -10.0, 0.8, "command"), saturated),
    ]
    metrics, summary = analyze_roll_bench_round(
        steps,
        {"rate_output_limit": 0.2, "rate_ki_roll": 0.0},
    )

    assert metrics[0].sign_ok is False
    assert metrics[1].saturation_warning is True
    assert summary.sign_ok is False
    assert summary.saturation_risk is True
    assert summary.safe_to_continue is False
    assert summary.kp_tuning_allowed is False


def test_analyze_axis_bench_round_accepts_pitch_mapping_and_split():
    steps = [
        (
            RollBenchStep("pos_small", 10.0, 0.8, "command"),
            [_sample_with_pitch(10.0, 2.6, 0.030, 0.026) for _ in range(16)],
        ),
        (
            RollBenchStep("zero_after", 0.0, 0.7, "zero"),
            [_sample_with_pitch(0.0, 1.1, 0.0, 0.0) for _ in range(16)],
        ),
        (
            RollBenchStep("neg_small", -10.0, 0.8, "command"),
            [_sample_with_pitch(-10.0, -2.7, -0.031, -0.027) for _ in range(16)],
        ),
        (
            RollBenchStep("post_zero", 0.0, 0.7, "zero"),
            [_sample_with_pitch(0.0, 1.0, 0.0, 0.0) for _ in range(16)],
        ),
    ]
    metrics, summary = analyze_axis_bench_round(
        steps,
        {"rate_output_limit": 0.2, "rate_ki_pitch": 0.0},
        axis_name="pitch",
    )

    assert metrics[0].sign_ok is True
    assert metrics[0].motor_split_mean > 0.0
    assert metrics[2].sign_ok is True
    assert metrics[2].motor_split_mean < 0.0
    assert summary.sign_ok is True
    assert summary.measurable_response is True
    assert summary.accept is True


def test_analyze_axis_bench_round_accepts_yaw_mapping_and_split():
    steps = [
        (
            RollBenchStep("pos_small", 8.0, 0.8, "command"),
            [_sample_with_yaw(8.0, 2.4, 0.024, 0.022) for _ in range(16)],
        ),
        (
            RollBenchStep("zero_after", 0.0, 0.7, "zero"),
            [_sample_with_yaw(0.0, 0.8, 0.0, 0.0) for _ in range(16)],
        ),
        (
            RollBenchStep("neg_small", -8.0, 0.8, "command"),
            [_sample_with_yaw(-8.0, -2.5, -0.025, -0.023) for _ in range(16)],
        ),
        (
            RollBenchStep("post_zero", 0.0, 0.7, "zero"),
            [_sample_with_yaw(0.0, 0.7, 0.0, 0.0) for _ in range(16)],
        ),
    ]
    metrics, summary = analyze_axis_bench_round(
        steps,
        {"rate_output_limit": 0.2, "rate_ki_yaw": 0.0},
        axis_name="yaw",
    )

    assert metrics[0].sign_ok is True
    assert metrics[0].motor_split_mean > 0.0
    assert metrics[2].sign_ok is True
    assert metrics[2].motor_split_mean < 0.0
    assert summary.sign_ok is True
    assert summary.measurable_response is True
    assert summary.accept is True


def test_analyze_axis_bench_round_treats_max_probe_low_duty_as_warning():
    pos_small = [_sample_with_roll(10.0, 2.8, 0.028, 0.030) for _ in range(16)]
    pos_large = [_sample_with_roll(15.0, 3.5, 0.042, 0.070) for _ in range(16)]
    for sample in pos_large[:6]:
        sample.motor2 = 0.002
        sample.motor3 = 0.003
    steps = [
        (RollBenchStep("pos_small", 10.0, 0.8, "command"), pos_small),
        (RollBenchStep("pos_large", 15.0, 0.8, "command"), pos_large),
    ]
    metrics, summary = analyze_axis_bench_round(
        steps,
        {"rate_output_limit": 0.2, "rate_ki_roll": 0.0, "motor_idle_duty": 0.02},
        axis_name="roll",
    )

    assert metrics[1].motor_low_clip_ratio > 0.0
    assert summary.low_duty_motor_stability == "PASS_WITH_WARNING"
    assert summary.accept is True
    assert summary.axis_result == "PASS_WITH_WARNING"


def test_run_roll_bench_round_auto_arms_before_first_rate_test(tmp_path):
    session = FakeRollBenchSession(arm_state=0)

    result = run_roll_bench_round(
        session,
        tmp_path,
        auto_arm=True,
        small_step_dps=10.0,
        large_step_dps=10.0,
        active_duration_s=0.03,
        zero_duration_s=0.03,
        serial_hint="COMx",
        tag="unit",
    )

    first_rate_test_index = next(i for i, call in enumerate(session.calls) if call.startswith("rate_test"))
    assert session.calls.index("arm") < first_rate_test_index
    assert result.summary.safe_to_continue is True
    assert result.summary.kp_tuning_allowed is True


def test_run_roll_bench_round_setup_failure_does_not_kill(tmp_path):
    session = FakeRollBenchSession(arm_state=3, failsafe_reason=1)

    with pytest.raises(RuntimeError, match="Clear the bench fault state first"):
        run_roll_bench_round(
            session,
            tmp_path,
            auto_arm=False,
            small_step_dps=10.0,
            large_step_dps=10.0,
            active_duration_s=0.03,
            zero_duration_s=0.03,
        )

    assert "kill" not in session.calls


def test_run_roll_bench_round_live_trip_kills_then_disarms(tmp_path):
    session = FakeRollBenchSession(arm_state=0, trip_on_nonzero=True)

    with pytest.raises(RollBenchSafetyTrip):
        run_roll_bench_round(
            session,
            tmp_path,
            auto_arm=True,
            small_step_dps=10.0,
            large_step_dps=10.0,
            active_duration_s=0.03,
            zero_duration_s=0.03,
        )

    assert "kill" in session.calls
    assert session.calls.index("kill") < session.calls.index("disarm")
