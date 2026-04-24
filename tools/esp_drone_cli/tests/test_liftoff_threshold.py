from __future__ import annotations

import csv
from pathlib import Path
import threading
import time

import pytest

from esp_drone_cli.cli import main as cli_main
from esp_drone_cli.core.csv_log import CsvTelemetryLogger
from esp_drone_cli.core.liftoff_threshold import (
    DEFAULT_ANGLE_TRIP_DEG,
    DEFAULT_HARD_ANGLE_TRIP_DEG,
    DEFAULT_MAX_DUTY,
    DEFAULT_TELEMETRY_HZ,
    LIFTOFF_THRESHOLD_CSV_FIELDS,
    LiftoffThresholdTracker,
)
from esp_drone_cli.core.models import ParamValue, TelemetrySample


REAL_SLEEP = time.sleep


def make_sample(**overrides) -> TelemetrySample:
    data = {name: 0 for name in TelemetrySample.__dataclass_fields__}
    data.update(
        {
            "quat_w": 1.0,
            "battery_voltage": 3.8,
            "battery_valid": 1,
            "loop_dt_us": 2000,
            "imu_age_us": 1000,
            "imu_mode": 1,
            "imu_health": 1,
            "baro_pressure_pa": 100845.0,
            "baro_temperature_c": 26.0,
            "baro_valid": 1,
            "baro_health": 1,
            "attitude_valid": 1,
            "kalman_valid": 1,
            "reference_valid": 1,
            "ground_ref_valid": 1,
            "attitude_ref_valid": 1,
        }
    )
    data.update(overrides)
    base_duty = float(data["base_duty_active"])
    for motor_name in ("motor1", "motor2", "motor3", "motor4"):
        if motor_name not in overrides:
            data[motor_name] = base_duty
    if "roll_deg" not in overrides:
        data["roll_deg"] = data["angle_measured_roll"]
    if "pitch_deg" not in overrides:
        data["pitch_deg"] = data["angle_measured_pitch"]
    if "yaw_deg" not in overrides:
        data["yaw_deg"] = data["angle_measured_yaw"]
    return TelemetrySample(**data)


class LiftoffThresholdFakeSession:
    def __init__(self, *, fail_on_verify_start: bool = False) -> None:
        self.calls: list[tuple[str, tuple, dict]] = []
        self.last_log_path: Path | None = None
        self._telemetry_callbacks: dict[int, object] = {}
        self._event_callbacks: dict[int, object] = {}
        self._connection_callbacks: dict[int, object] = {}
        self._next_token = 1
        self._csv_logger: CsvTelemetryLogger | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._stream_active = False
        self._connected = True
        self._armed = False
        self._ground_ref_valid = False
        self._verify_active = False
        self._timestamp_us = 0
        self._sample_seq = 0
        self._active_base_duty = 0.0
        self._fail_on_verify_start = fail_on_verify_start
        self.device_info = type(
            "DeviceInfoStub",
            (),
            {
                "protocol_version": 8,
                "imu_mode": 1,
                "arm_state": 0,
                "stream_enabled": 0,
                "feature_bitmap": 1 << 8,
            },
        )()
        self._params: dict[str, ParamValue] = {
            "telemetry_usb_hz": ParamValue("telemetry_usb_hz", 2, 100),
            "ground_tune_use_kalman_attitude": ParamValue("ground_tune_use_kalman_attitude", 0, True),
            "ground_att_trip_deg": ParamValue("ground_att_trip_deg", 4, 12.0),
            "ground_test_base_duty": ParamValue("ground_test_base_duty", 4, 0.08),
            "ground_test_max_extra_duty": ParamValue("ground_test_max_extra_duty", 4, 0.03),
            "ground_test_motor_balance_limit": ParamValue("ground_test_motor_balance_limit", 4, 0.06),
            "ground_test_auto_disarm_ms": ParamValue("ground_test_auto_disarm_ms", 2, 15000),
            "ground_test_ramp_duty_per_s": ParamValue("ground_test_ramp_duty_per_s", 4, 0.15),
        }

    def _record(self, name: str, *args, **kwargs) -> None:
        self.calls.append((name, args, kwargs))

    def _allocate_token(self) -> int:
        token = self._next_token
        self._next_token += 1
        return token

    def subscribe_telemetry(self, callback):
        token = self._allocate_token()
        self._telemetry_callbacks[token] = callback
        return token

    def subscribe_event_log(self, callback):
        token = self._allocate_token()
        self._event_callbacks[token] = callback
        return token

    def subscribe_connection_state(self, callback):
        token = self._allocate_token()
        self._connection_callbacks[token] = callback
        return token

    def unsubscribe(self, callback_id: int) -> None:
        self._telemetry_callbacks.pop(callback_id, None)
        self._event_callbacks.pop(callback_id, None)
        self._connection_callbacks.pop(callback_id, None)

    def _emit_connection(self, *, connected: bool, error: str | None = None) -> None:
        payload = {"connected": connected, "device_info": self.device_info, "error": error}
        for callback in list(self._connection_callbacks.values()):
            callback(payload)

    def _emit_telemetry(self, sample: TelemetrySample) -> None:
        if self._csv_logger is not None:
            self._csv_logger.write(sample)
        for callback in list(self._telemetry_callbacks.values()):
            callback(sample)

    def _telemetry_loop(self) -> None:
        while not self._stop_event.is_set():
            if self._stream_active and self._connected:
                telemetry_hz = max(1, int(self._params["telemetry_usb_hz"].value))
                dt_s = 1.0 / float(telemetry_hz)
                ramp_per_s = float(self._params["ground_test_ramp_duty_per_s"].value)
                target_duty = float(self._params["ground_test_base_duty"].value) if self._verify_active else 0.0
                max_delta = ramp_per_s * dt_s
                if self._active_base_duty < target_duty:
                    self._active_base_duty = min(self._active_base_duty + max_delta, target_duty)
                else:
                    self._active_base_duty = max(self._active_base_duty - max_delta, target_duty)
                self._sample_seq += 1
                self._timestamp_us += int(dt_s * 1_000_000.0)

                sample = make_sample(
                    timestamp_us=self._timestamp_us,
                    sample_seq=self._sample_seq,
                    arm_state=1 if self._armed else 0,
                    control_mode=6 if self._verify_active else 0,
                    control_submode=1 if self._verify_active else 0,
                    base_duty_active=self._active_base_duty,
                    ground_ref_valid=1 if self._ground_ref_valid else 0,
                    reference_valid=1 if self._ground_ref_valid else 0,
                    angle_measured_roll=1.25 if self._verify_active else 0.0,
                    angle_measured_pitch=-1.75 if self._verify_active else 0.0,
                    attitude_ref_qw=1.0,
                    motor1=self._active_base_duty,
                    motor2=self._active_base_duty + 0.002,
                    motor3=self._active_base_duty + 0.004,
                    motor4=self._active_base_duty + 0.001,
                )
                self._emit_telemetry(sample)
                REAL_SLEEP(dt_s)
            else:
                REAL_SLEEP(0.005)

    def _ensure_thread(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self._thread.start()

    def require_attitude_ground_verify(self) -> None:
        self._record("require_attitude_ground_verify")

    def get_param(self, name: str) -> ParamValue:
        self._record("get_param", name)
        return self._params[name]

    def set_param(self, name: str, type_id: int, value):
        self._record("set_param", name, type_id, value)
        if type_id == 0:
            cast_value = bool(value)
        elif type_id == 4:
            cast_value = float(value)
        else:
            cast_value = int(value)
        item = ParamValue(name, type_id, cast_value)
        self._params[name] = item
        return item

    def start_csv_log(self, output_path: Path, *, fieldnames=None, extra_row_fn=None) -> None:
        self._record("start_csv_log", output_path, fieldnames=fieldnames, extra_row_fn=extra_row_fn)
        self.last_log_path = output_path
        self._csv_logger = CsvTelemetryLogger(output_path, fieldnames=fieldnames, extra_row_fn=extra_row_fn)

    def stop_csv_log(self) -> Path | None:
        self._record("stop_csv_log")
        if self._csv_logger is not None:
            self._csv_logger.close()
            self._csv_logger = None
        return self.last_log_path

    def start_stream(self, timeout: float = 1.0) -> None:
        self._record("start_stream", timeout)
        self._stream_active = True
        self._ensure_thread()

    def stop_stream(self, timeout: float = 1.0) -> None:
        self._record("stop_stream", timeout)
        self._stream_active = False

    def ground_capture_ref(self) -> int:
        self._record("ground_capture_ref")
        self._ground_ref_valid = True
        return 0

    def arm(self) -> int:
        self._record("arm")
        self._armed = True
        return 0

    def disarm(self) -> int:
        self._record("disarm")
        self._armed = False
        return 0

    def attitude_ground_verify_start(self, base_duty: float | None = None) -> int:
        self._record("attitude_ground_verify_start", base_duty)
        if self._fail_on_verify_start:
            raise RuntimeError("forced verify start failure")
        if base_duty is not None:
            self._params["ground_test_base_duty"] = ParamValue("ground_test_base_duty", 4, float(base_duty))
        self._verify_active = True
        return 0

    def attitude_ground_verify_stop(self) -> int:
        self._record("attitude_ground_verify_stop")
        self._verify_active = False
        return 0

    def all_motor_test_start(self, duty: float, duration_s: float) -> int:
        self._record("all_motor_test_start", duty, duration_s)
        return 0

    def all_motor_test_stop(self) -> int:
        self._record("all_motor_test_stop")
        return 0

    def close(self) -> None:
        self._record("close")
        self._stream_active = False
        self._connected = False
        self._emit_connection(connected=False)
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
        if self._csv_logger is not None:
            self._csv_logger.close()
            self._csv_logger = None


def test_liftoff_threshold_help_and_parse_single_point_args():
    parser = cli_main.build_parser()
    args = parser.parse_args(
        ["--serial", "COM7", "liftoff-threshold", "--duty", "0.13", "--duration-s", "1.0"]
    )
    help_text = parser.format_help()
    command_help = parser.parse_args(["--serial", "COM7", "liftoff-threshold", "--duty", "0.13", "--duration-s", "1.0"])

    assert args.command == "liftoff-threshold"
    assert command_help.duty == pytest.approx(0.13)
    assert command_help.duration_s == pytest.approx(1.0)
    assert args.max_duty == DEFAULT_MAX_DUTY
    assert args.angle_trip_deg == DEFAULT_ANGLE_TRIP_DEG
    assert args.hard_angle_trip_deg == DEFAULT_HARD_ANGLE_TRIP_DEG
    assert args.telemetry_hz == DEFAULT_TELEMETRY_HZ
    assert "liftoff-threshold" in help_text

    with pytest.raises(SystemExit):
        parser.parse_args(["--serial", "COM7", "liftoff-threshold", "--duration-s", "1.0"])
    with pytest.raises(SystemExit):
        parser.parse_args(["--serial", "COM7", "liftoff-threshold", "--duty", "0.13"])

    subparser_help = cli_main.build_parser().parse_args
    assert subparser_help is not None


def test_liftoff_threshold_rejects_removed_scan_args():
    parser = cli_main.build_parser()
    for removed_arg in ("--base-duty", "--end-duty", "--step-duty", "--hold-s", "--log"):
        with pytest.raises(SystemExit):
            parser.parse_args(
                [
                    "--serial",
                    "COM7",
                    "liftoff-threshold",
                    "--duty",
                    "0.13",
                    "--duration-s",
                    "1.0",
                    removed_arg,
                    "0.10",
                ]
            )


@pytest.mark.parametrize("mode", ["keyboard_interrupt", "runtime_exception"])
def test_liftoff_threshold_always_disarms_on_interrupt_or_exception(monkeypatch, tmp_path: Path, mode: str):
    session = LiftoffThresholdFakeSession(fail_on_verify_start=mode == "runtime_exception")
    monkeypatch.setattr(cli_main, "connect_session_from_args", lambda _args: session)

    if mode == "keyboard_interrupt":
        from esp_drone_cli.core import liftoff_threshold as liftoff_threshold_module

        def interrupting_sleep(duration: float) -> None:
            if any(name == "attitude_ground_verify_start" for name, _args, _kwargs in session.calls):
                raise KeyboardInterrupt()
            REAL_SLEEP(min(duration, 0.001))

        monkeypatch.setattr(liftoff_threshold_module.time, "sleep", interrupting_sleep)

    rc = cli_main.main(
        [
            "--serial",
            "COM7",
            "liftoff-threshold",
            "--duty",
            "0.13",
            "--duration-s",
            "0.06",
            "--output-dir",
            str(tmp_path),
        ]
    )

    assert rc == (130 if mode == "keyboard_interrupt" else 1)
    call_names = [name for name, _args, _kwargs in session.calls]
    assert "disarm" in call_names


def test_liftoff_threshold_records_csv_by_default_and_uses_closed_loop_path(monkeypatch, tmp_path: Path, capsys):
    session = LiftoffThresholdFakeSession()
    monkeypatch.setattr(cli_main, "connect_session_from_args", lambda _args: session)

    rc = cli_main.main(
        [
            "--serial",
            "COM7",
            "liftoff-threshold",
            "--duty",
            "0.13",
            "--duration-s",
            "0.08",
            "--output-dir",
            str(tmp_path),
        ]
    )

    assert rc == 0
    call_names = [name for name, _args, _kwargs in session.calls]
    assert "require_attitude_ground_verify" in call_names
    assert "ground_capture_ref" in call_names
    assert "arm" in call_names
    assert ("attitude_ground_verify_start", (0.13,), {}) in session.calls
    assert "attitude_ground_verify_stop" in call_names
    assert "all_motor_test_start" not in call_names
    assert "all_motor_test_stop" not in call_names
    assert "start_csv_log" in call_names
    assert session.last_log_path is not None
    assert session.last_log_path.exists()

    output = capsys.readouterr().out
    assert "csv=" in output
    assert "duty=0.130" in output
    assert "duration_s=0.080" in output
    assert "max_roll_deg=" in output
    assert "max_pitch_deg=" in output
    assert "max_motor_spread=" in output

    with session.last_log_path.open("r", encoding="utf-8", newline="") as handle:
        header = next(csv.reader(handle))
    assert header == LIFTOFF_THRESHOLD_CSV_FIELDS


def test_liftoff_threshold_sets_telemetry_rate_before_logging(monkeypatch, tmp_path: Path):
    session = LiftoffThresholdFakeSession()
    monkeypatch.setattr(cli_main, "connect_session_from_args", lambda _args: session)

    rc = cli_main.main(
        [
            "--serial",
            "COM7",
            "liftoff-threshold",
            "--duty",
            "0.13",
            "--duration-s",
            "0.06",
            "--output-dir",
            str(tmp_path),
        ]
    )

    assert rc == 0
    telemetry_index = next(
        index
        for index, (name, args, _kwargs) in enumerate(session.calls)
        if name == "set_param" and args[0] == "telemetry_usb_hz"
    )
    log_index = next(index for index, (name, _args, _kwargs) in enumerate(session.calls) if name == "start_csv_log")
    assert telemetry_index < log_index


def test_liftoff_threshold_tracker_debounce_and_anomaly_flags():
    tracker = LiftoffThresholdTracker(
        max_duty=0.22,
        angle_trip_deg=25.0,
        hard_angle_trip_deg=35.0,
        motor_limit_trip_s=0.30,
        kalman_invalid_trip_s=0.30,
    )

    for timestamp_us in (0, 100_000, 200_000, 299_000):
        stop_reason = tracker.update(
            make_sample(
                timestamp_us=timestamp_us,
                base_duty_active=0.12,
                inner_loop_clamp_flag=1,
                motor_saturation_flag=1,
            )
        )
        assert stop_reason is None

    assert tracker.clamp_seen is True
    assert tracker.saturation_seen is True
    assert tracker.update(
        make_sample(
            timestamp_us=310_000,
            base_duty_active=0.12,
            inner_loop_clamp_flag=1,
            motor_saturation_flag=1,
        )
    ) == "sustained_motor_limit"

    kalman_tracker = LiftoffThresholdTracker(
        max_duty=0.22,
        angle_trip_deg=25.0,
        hard_angle_trip_deg=35.0,
        motor_limit_trip_s=0.30,
        kalman_invalid_trip_s=0.30,
    )
    assert kalman_tracker.update(
        make_sample(
            timestamp_us=0,
            base_duty_active=0.12,
            kalman_valid=0,
            battery_voltage=0.0,
            battery_valid=0,
        )
    ) is None
    assert kalman_tracker.kalman_invalid_seen is True
    assert kalman_tracker.battery_zero_seen is True
    assert kalman_tracker.update(
        make_sample(
            timestamp_us=310_000,
            base_duty_active=0.12,
            kalman_valid=0,
        )
    ) == "sustained_kalman_invalid"
