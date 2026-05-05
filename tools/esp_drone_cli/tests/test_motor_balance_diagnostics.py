from __future__ import annotations

from pathlib import Path

import pytest

from esp_drone_cli.cli.main import build_parser
from esp_drone_cli.core.models import ParamValue
from esp_drone_cli.core.motor_balance import (
    MotorBalanceOptions,
    apply_motor_trim_estimate,
    estimate_motor_trim_from_summary,
    format_motor_balance_summary,
    format_motor_trim_estimate,
    run_motor_thrust_balance,
)
from esp_drone_cli.core.protocol.messages import CmdStatus


class MotorBalanceFakeSession:
    def __init__(self) -> None:
        self.calls: list[tuple[str, tuple, dict]] = []
        self._params = [
            ParamValue("motor_scale_m1", 4, 1.0),
            ParamValue("motor_scale_m2", 4, 1.10),
            ParamValue("motor_scale_m3", 4, 0.9126),
            ParamValue("motor_scale_m4", 4, 1.10),
            ParamValue("motor_offset_m1", 4, 0.0),
            ParamValue("motor_offset_m2", 4, 0.0),
            ParamValue("motor_offset_m3", 4, 0.0),
            ParamValue("motor_offset_m4", 4, 0.0),
        ]

    def _record(self, name: str, *args, **kwargs) -> None:
        self.calls.append((name, args, kwargs))

    def subscribe_telemetry(self, callback):
        self._record("subscribe_telemetry", callback)
        return 1

    def unsubscribe(self, callback_id: int) -> None:
        self._record("unsubscribe", callback_id)

    def get_param(self, name: str, timeout: float = 1.0) -> ParamValue:
        self._record("get_param", name, timeout)
        for item in self._params:
            if item.name == name:
                return ParamValue(item.name, item.type_id, item.value)
        raise KeyError(name)

    def set_param(self, name: str, type_id: int, value):
        self._record("set_param", name, type_id, value)
        cast_value = float(value) if type_id == 4 else int(value)
        for index, item in enumerate(self._params):
            if item.name == name:
                self._params[index] = ParamValue(name, type_id, cast_value)
                return self._params[index]
        item = ParamValue(name, type_id, cast_value)
        self._params.append(item)
        return item

    def start_csv_log(self, output_path: Path, **kwargs) -> None:
        self._record("start_csv_log", output_path, **kwargs)

    def stop_csv_log(self) -> None:
        self._record("stop_csv_log")

    def start_stream(self) -> None:
        self._record("start_stream")

    def stop_stream(self) -> None:
        self._record("stop_stream")

    def disarm(self) -> int:
        self._record("disarm")
        return CmdStatus.OK

    def motor_test(self, motor_index: int, duty: float) -> int:
        self._record("motor_test", motor_index, duty)
        return CmdStatus.OK


def test_cli_parser_accepts_motor_balance_no_trim_and_settle():
    args = build_parser().parse_args(
        ["--serial", "COM7", "motor-thrust-balance", "--no-trim", "--settle-s", "0"]
    )

    assert args.command == "motor-thrust-balance"
    assert args.no_trim is True
    assert args.settle_s == pytest.approx(0.0)


def test_motor_thrust_balance_reports_trim_targets(monkeypatch, tmp_path: Path):
    from esp_drone_cli.core import motor_balance

    session = MotorBalanceFakeSession()
    monkeypatch.setattr(motor_balance.time, "sleep", lambda _duration: None)

    result = run_motor_thrust_balance(
        session,
        MotorBalanceOptions(
            duties=(0.20,),
            duration_s=0.2,
            settle_s=0.0,
            rest_s=0.0,
            motors=(1, 2, 3, 4),
            output_dir=tmp_path,
        ),
    )
    output = "\n".join(format_motor_balance_summary(result))

    assert result.return_code == 0
    assert "trim_applied=True" in output
    assert "M2 input=0.200 trim_target=0.2200 scale=1.1000 offset=0.0000" in output
    assert "M3 input=0.200 trim_target=0.1825 scale=0.9126 offset=0.0000" in output
    assert "trial=2 motor=M2 input_duty=0.200 trim_target_duty=0.2200" in output
    summary_text = result.summary_path.read_text(encoding="utf-8")
    assert "input_duty,trim_scale,trim_offset,trim_target_duty" in summary_text
    assert "0.220000" in summary_text


def test_motor_thrust_balance_no_trim_temporarily_neutralizes_and_restores(monkeypatch, tmp_path: Path):
    from esp_drone_cli.core import motor_balance

    session = MotorBalanceFakeSession()
    monkeypatch.setattr(motor_balance.time, "sleep", lambda _duration: None)

    result = run_motor_thrust_balance(
        session,
        MotorBalanceOptions(
            duties=(0.20,),
            duration_s=0.2,
            settle_s=0.0,
            rest_s=0.0,
            motors=(2, 3),
            output_dir=tmp_path,
            use_trim=False,
        ),
    )
    output = "\n".join(format_motor_balance_summary(result))

    assert result.return_code == 0
    assert "trim_applied=False" in output
    assert "M2 input=0.200 trim_target=0.2000 scale=1.0000 offset=0.0000" in output
    assert "M3 input=0.200 trim_target=0.2000 scale=1.0000 offset=0.0000" in output
    assert ("set_param", ("motor_scale_m2", 4, 1.0), {}) in session.calls
    assert ("set_param", ("motor_scale_m3", 4, 1.0), {}) in session.calls
    assert ("set_param", ("motor_scale_m2", 4, 1.1), {}) in session.calls
    assert ("set_param", ("motor_scale_m3", 4, 0.9126), {}) in session.calls
    assert session.get_param("motor_scale_m2").value == pytest.approx(1.10)
    assert session.get_param("motor_scale_m3").value == pytest.approx(0.9126)


def test_motor_trim_estimate_apply_prints_readback_and_score_note(tmp_path: Path):
    summary = tmp_path / "summary.csv"
    summary.write_text(
        "\n".join(
            [
                "trial_id,motor,duty,sample_count,battery_min_v,battery_mean_v,gyro_rms_dps,gyro_peak_dps,gyro_x_mean_dps,gyro_y_mean_dps,gyro_z_mean_dps,acc_rms_g,acc_std_g,response_score,relative_to_duty_mean,classification",
                "1,M1,0.30,80,4.0,4.0,40,80,0,0,0,0,0,100,1.0,normal",
                "2,M2,0.30,80,4.0,4.0,80,120,0,0,0,0,0,160,1.6,strong_response",
                "3,M3,0.30,80,4.0,4.0,10,30,0,0,0,0,0,20,0.2,weak_response",
                "4,M4,0.30,80,4.0,4.0,45,80,0,0,0,0,0,100,1.0,normal",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    session = MotorBalanceFakeSession()
    estimate = estimate_motor_trim_from_summary(summary)

    apply_motor_trim_estimate(session, estimate)
    output = "\n".join(format_motor_trim_estimate(estimate))

    assert ("set_param", ("motor_scale_m2", 4, 0.9), {}) in session.calls
    assert "score_note=score is IMU vibration/disturbance response, not thrust-stand force" in output
    assert "ratio_to_M1 is not a real thrust ratio" in output
    assert "M2 scale_param=motor_scale_m2 written=0.900000 readback=0.900000" in output
    assert "motor_trim_scale_m2" not in output
