# ============================================================
# @file test_core_session.py
# @brief ESP-DRONE Core Session ????
# @details ?? DeviceSession?CLI?GUI smoke ?????? shim ???
# @author Codex
# @date 2026-04-05
# @version 1.0
# ============================================================

from __future__ import annotations

import builtins
import importlib
import importlib.util
import json
import queue
import re
import socket
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from esp_drone_cli.core import DeviceSession
from esp_drone_cli.cli.main import (
    analyze_attitude_ground_verify_samples,
    analyze_liftoff_verify_samples,
    current_liftoff_pre_start_readiness,
    format_liftoff_verify_summary,
    format_rate_status_line,
)
from esp_drone_cli.core.models import (
    CMD_REQ_STRUCT,
    CapabilityError,
    DeviceInfo,
    FEATURE_ATTITUDE_GROUND_VERIFY,
    FEATURE_ATTITUDE_HANG_BENCH,
    FEATURE_ALL_MOTOR_TEST,
    FEATURE_LOW_RISK_LIFTOFF_VERIFY,
    HELLO_RESP_STRUCT,
    HELLO_RESP_STRUCT_V2,
    ParamSnapshot,
    ParamValue,
    TELEMETRY_CSV_FIELDS,
    TELEMETRY_STRUCT,
    TELEMETRY_STRUCT_V1,
    TELEMETRY_STRUCT_V3,
    TELEMETRY_STRUCT_V4,
    TELEMETRY_STRUCT_V5,
    TelemetrySample,
    UDP_MANUAL_SETPOINT_STRUCT,
    decode_device_info,
    decode_param_value,
    encode_param_value,
)
from esp_drone_cli.core.protocol.framing import decode_frame, encode_frame, encode_serial_packet
from esp_drone_cli.core.protocol.messages import CmdId, CmdStatus, Frame, MsgType


def encode_param_payload(name: str, type_id: int, value_bytes: bytes) -> bytes:
    name_bytes = name.encode("ascii")
    return bytes([type_id, len(name_bytes)]) + name_bytes + value_bytes


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
        2.5, -3.5,
        -5.0, 7.0,
        0.7, 0.1, -0.2, 0.3,
        0.05,
        1, 0, 0, 0,
    ]
    return TELEMETRY_STRUCT.pack(*values)


def build_telemetry_payload_v1() -> bytes:
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
    ]
    return TELEMETRY_STRUCT_V1.pack(*values)


def build_telemetry_payload_v4() -> bytes:
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
        2.5, -3.5,
        -5.0, 7.0,
        0.7, 0.1, -0.2, 0.3,
        0.05,
        1, 0, 0, 0,
        1.25, -2.5, 3.75,
        0.12, -0.34, 0.98,
        9.5, -8.25,
        -21.0, 22.0, -23.0,
        -11.0, 12.0, -13.0,
        0.31, -0.32, 0.33,
        4242,
        1, 1, 0, 1, 1, 1, 3, 1,
    ]
    return TELEMETRY_STRUCT_V4.pack(*values)


def build_telemetry_payload_v5() -> bytes:
    values = list(TELEMETRY_STRUCT_V4.unpack(build_telemetry_payload_v4()))
    values.extend(
        [
            1.0, -2.0, 0.0,
            0.4, -0.6, 0.0,
            0.6, -1.4, 0.0,
            0.72, -1.68, 0.0,
            1, 3, 1, 0,
        ]
    )
    return TELEMETRY_STRUCT_V5.pack(*values)


class MockTransport:
    def __init__(self) -> None:
        self.sent: list[tuple[int, bytes]] = []
        self._frames: queue.Queue[Frame] = queue.Queue()
        self.closed = False

    def send(self, _data: bytes) -> None:
        raise AssertionError("raw send should not be used in tests")

    def send_message(self, msg_type: int, payload: bytes = b"", flags: int = 0, seq: int = 0) -> None:
        self.sent.append((msg_type, payload))
        if msg_type == MsgType.HELLO_REQ:
            hello = HELLO_RESP_STRUCT.pack(5, 1, 0, 0, 0x7F)
            self.inject(Frame(MsgType.HELLO_RESP, flags, seq, hello))
            return
        if msg_type == MsgType.CMD_REQ:
            cmd_id = payload[0]
            self.inject(Frame(MsgType.CMD_RESP, flags, seq, bytes([cmd_id, 0, 0, 0])))
            return
        if msg_type == MsgType.PARAM_GET:
            name_len = payload[0]
            name = payload[1 : 1 + name_len].decode("ascii")
            self.inject(Frame(MsgType.PARAM_VALUE, flags, seq, encode_param_payload(name, 4, b"\x00\x00\x20@")))
            return
        if msg_type == MsgType.PARAM_SET:
            type_id = payload[0]
            name_len = payload[1]
            name = payload[2 : 2 + name_len].decode("ascii")
            value_bytes = payload[2 + name_len :]
            self.inject(Frame(MsgType.PARAM_VALUE, flags, seq, encode_param_payload(name, type_id, value_bytes)))
            return
        if msg_type == MsgType.PARAM_LIST_REQ:
            self.inject(Frame(MsgType.PARAM_VALUE, flags, seq, encode_param_payload("alpha", 2, b"\x2A\x00\x00\x00")))
            self.inject(Frame(MsgType.PARAM_VALUE, flags, seq, encode_param_payload("beta", 0, b"\x01")))
            self.inject(Frame(MsgType.PARAM_LIST_END, flags, seq, b""))
            return
        if msg_type in {MsgType.PARAM_SAVE, MsgType.PARAM_RESET, MsgType.STREAM_CTRL}:
            self.inject(Frame(msg_type, flags, seq, payload))
            return
        if msg_type == MsgType.UDP_MANUAL_SETPOINT:
            self.inject(Frame(MsgType.CMD_RESP, flags, seq, bytes([CmdId.UDP_MANUAL_SETPOINT, 0, 0, 0])))

    def recv_frame(self, timeout: float) -> Frame:
        try:
            return self._frames.get(timeout=timeout)
        except queue.Empty as exc:
            raise TimeoutError("mock timeout") from exc

    def inject(self, frame: Frame) -> None:
        self._frames.put(frame)

    def close(self) -> None:
        self.closed = True


class TimeoutHelloTransport(MockTransport):
    def send_message(self, msg_type: int, payload: bytes = b"", flags: int = 0, seq: int = 0) -> None:
        self.sent.append((msg_type, payload))
        if msg_type == MsgType.HELLO_REQ:
            return
        super().send_message(msg_type, payload, flags=flags, seq=seq)


class FakeSession:
    def __init__(self) -> None:
        self.calls: list[tuple[str, tuple, dict]] = []
        self.is_connected = False
        self.last_log_path: Path | None = None
        self.device_info = None
        self._telemetry_callbacks = []
        self._event_callbacks = []
        self._connection_callbacks = []
        self._arm_state = 0
        self._failsafe_reason = 0
        self._control_mode = 0
        self._latest_telemetry = self._runtime_state_sample()
        self._params = [
            ParamValue("alpha", 2, 42),
            ParamValue("beta", 4, 1.5),
            ParamValue("udp_manual_max_pwm", 4, 0.12),
            ParamValue("wifi_ap_enable", 0, True),
            ParamValue("wifi_ap_channel", 1, 6),
            ParamValue("wifi_udp_port", 2, 2391),
            ParamValue("wifi_mode", 5, "softap"),
            ParamValue("sta_ssid", 5, ""),
        ]

    def subscribe_telemetry(self, callback):
        self._telemetry_callbacks.append(callback)
        return len(self._telemetry_callbacks)

    def subscribe_event_log(self, callback):
        self._event_callbacks.append(callback)
        return len(self._event_callbacks)

    def subscribe_connection_state(self, callback):
        self._connection_callbacks.append(callback)
        return len(self._connection_callbacks)

    def unsubscribe(self, _callback_id: int) -> None:
        return None

    def _record(self, name: str, *args, **kwargs):
        self.calls.append((name, args, kwargs))

    def _runtime_state_sample(self):
        return SimpleNamespace(
            arm_state=self._arm_state,
            failsafe_reason=self._failsafe_reason,
            control_mode=self._control_mode,
        )

    def _telemetry_sample(self, timestamp_us: int, duty: float, control_mode: int | None = None):
        mode = self._control_mode if control_mode is None else control_mode
        return SimpleNamespace(
            timestamp_us=timestamp_us,
            arm_state=self._arm_state,
            failsafe_reason=self._failsafe_reason,
            control_mode=mode,
            control_submode=0,
            motor1=duty,
            motor2=duty,
            motor3=duty,
            motor4=duty,
            battery_voltage=4.0,
            battery_valid=1,
            base_duty_active=duty,
            ground_trip_reason=0,
        )

    def _set_runtime_state(
        self,
        *,
        arm_state: int | None = None,
        failsafe_reason: int | None = None,
        control_mode: int | None = None,
    ) -> None:
        if arm_state is not None:
            self._arm_state = arm_state
        if failsafe_reason is not None:
            self._failsafe_reason = failsafe_reason
        if control_mode is not None:
            self._control_mode = control_mode
        self._latest_telemetry = self._runtime_state_sample()

    def _emit_connection(self, error: str | None = None) -> None:
        payload = {
            "connected": self.is_connected,
            "device_info": "fake-device",
            "error": error,
        }
        for callback in self._connection_callbacks:
            callback(payload)

    def emit_event(self, message: str) -> None:
        for callback in self._event_callbacks:
            callback(message)

    def emit_telemetry(self, sample: TelemetrySample) -> None:
        self._latest_telemetry = sample
        for callback in self._telemetry_callbacks:
            callback(sample)

    def connect_serial(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 0.2,
        open_retry_timeout_s: float = 5.0,
    ):
        self._record("connect_serial", port, baudrate, timeout, open_retry_timeout_s)
        self.is_connected = True
        self.device_info = type("DeviceInfoStub", (), {
            "protocol_version": 5,
            "imu_mode": 1,
            "arm_state": 0,
            "stream_enabled": 0,
            "feature_bitmap": 0x7F,
        })()
        self._emit_connection()
        return "fake-device"

    def connect_udp(self, host: str, port: int = 2391, timeout: float = 1.0):
        self._record("connect_udp", host, port, timeout)
        self.is_connected = True
        self.device_info = type("DeviceInfoStub", (), {
            "protocol_version": 5,
            "imu_mode": 1,
            "arm_state": 0,
            "stream_enabled": 0,
            "feature_bitmap": 0x7F,
        })()
        self._emit_connection()
        return "fake-device"

    def disconnect(self, safe_stop_stream: bool = True) -> None:
        self._record("disconnect", safe_stop_stream)
        self.is_connected = False
        self.device_info = None
        self._emit_connection()

    def close(self) -> None:
        self._record("close")
        self.disconnect()

    def arm(self) -> int:
        self._record("arm")
        self._set_runtime_state(arm_state=1, failsafe_reason=0)
        return CmdStatus.OK

    def disarm(self) -> int:
        self._record("disarm")
        self._set_runtime_state(arm_state=0, failsafe_reason=0, control_mode=0)
        return CmdStatus.OK

    def kill(self) -> int:
        self._record("kill")
        return 0

    def reboot(self) -> int:
        self._record("reboot")
        return 0

    def start_stream(self, timeout: float = 1.0) -> None:
        self._record("start_stream", timeout)

    def stop_stream(self, timeout: float = 1.0) -> None:
        self._record("stop_stream", timeout)

    def list_params(self, timeout: float = 1.0) -> list[ParamValue]:
        self._record("list_params", timeout)
        return [ParamValue(item.name, item.type_id, item.value) for item in self._params]

    def get_param(self, name: str, timeout: float = 1.0) -> ParamValue:
        self._record("get_param", name, timeout)
        for item in self._params:
            if item.name == name:
                return ParamValue(item.name, item.type_id, item.value)
        raise KeyError(name)

    def set_param(self, name: str, type_id: int, value):
        self._record("set_param", name, type_id, value)
        for index, item in enumerate(self._params):
            if item.name == name:
                if type_id == 4:
                    cast_value = float(value)
                elif type_id == 5:
                    cast_value = str(value)
                else:
                    cast_value = int(value)
                self._params[index] = ParamValue(name, type_id, cast_value)
                return self._params[index]
        item = ParamValue(name, type_id, value)
        self._params.append(item)
        return item

    def save_params(self, timeout: float = 1.0) -> None:
        self._record("save_params", timeout)

    def reset_params(self, timeout: float = 1.0) -> None:
        self._record("reset_params", timeout)

    def export_params(self, output_path: Path) -> ParamSnapshot:
        self._record("export_params", output_path)
        snapshot = ParamSnapshot(schema=1, firmware={"protocol_version": 1}, params=[{"name": item.name, "type_id": item.type_id, "value": item.value} for item in self._params])
        snapshot.write_json(output_path)
        return snapshot

    def import_params(self, input_path: Path, save_after: bool = False) -> list[ParamValue]:
        self._record("import_params", input_path, save_after)
        data = json.loads(input_path.read_text(encoding="utf-8"))
        applied = []
        for item in data.get("params", []):
            applied.append(self.set_param(str(item["name"]), int(item["type_id"]), item["value"]))
        return applied

    def motor_test(self, motor_index: int, duty: float) -> int:
        self._record("motor_test", motor_index, duty)
        return 0

    def require_all_motor_test(self) -> None:
        self._record("require_all_motor_test")

    def all_motor_test_start(self, duty: float, duration_s: float) -> int:
        self._record("all_motor_test_start", duty, duration_s)
        if self._arm_state != 1:
            return CmdStatus.ARM_REQUIRED
        if self._control_mode != 0:
            return CmdStatus.CONFLICT
        self._set_runtime_state(control_mode=7)
        self.emit_event(f"all-motor test started duty={duty:.3f} duration_ms={int(round(duration_s * 1000.0))}")
        self.emit_telemetry(self._telemetry_sample(0, duty, control_mode=7))
        self.emit_telemetry(self._telemetry_sample(int(round(duration_s * 1_000_000.0)), duty, control_mode=7))
        return CmdStatus.OK

    def all_motor_test_stop(self) -> int:
        self._record("all_motor_test_stop")
        self.emit_event("all-motor test stopped normally")
        self._set_runtime_state(control_mode=0)
        return CmdStatus.OK

    def get_latest_telemetry(self):
        self._record("get_latest_telemetry")
        return self._latest_telemetry

    def calib_gyro(self) -> int:
        self._record("calib_gyro")
        return 0

    def calib_level(self) -> int:
        self._record("calib_level")
        return 0

    def rate_test(self, axis_index: int, value_dps: float) -> int:
        self._record("rate_test", axis_index, value_dps)
        return 0

    def attitude_capture_ref(self) -> int:
        self._record("attitude_capture_ref")
        return 0

    def attitude_test_start(self) -> int:
        self._record("attitude_test_start")
        return 0

    def attitude_test_stop(self) -> int:
        self._record("attitude_test_stop")
        return 0

    def ground_capture_ref(self) -> int:
        self._record("ground_capture_ref")
        return 0

    def ground_test_start(self, base_duty: float | None = None) -> int:
        self._record("ground_test_start", base_duty)
        return 0

    def ground_test_stop(self) -> int:
        self._record("ground_test_stop")
        return 0

    def require_ground_tune(self) -> None:
        self._record("require_ground_tune")

    def require_attitude_ground_verify(self) -> None:
        self._record("require_attitude_ground_verify")

    def require_low_risk_liftoff_verify(self) -> None:
        self._record("require_low_risk_liftoff_verify")

    def attitude_ground_verify_start(self, base_duty: float | None = None) -> int:
        self._record("attitude_ground_verify_start", base_duty)
        return 0

    def attitude_ground_verify_stop(self) -> int:
        self._record("attitude_ground_verify_stop")
        return 0

    def attitude_ground_set_target(self, axis_index: int, target_deg: float) -> int:
        self._record("attitude_ground_set_target", axis_index, target_deg)
        return 0

    def liftoff_verify_start(self, base_duty: float | None = None) -> int:
        self._record("liftoff_verify_start", base_duty)
        return 0

    def liftoff_verify_stop(self) -> int:
        self._record("liftoff_verify_stop")
        return 0

    def require_udp_manual_control(self) -> None:
        self._record("require_udp_manual_control")

    def udp_manual_enable(self) -> int:
        self._record("udp_manual_enable")
        return 0

    def udp_manual_disable(self) -> int:
        self._record("udp_manual_disable")
        return 0

    def udp_manual_stop(self) -> int:
        self._record("udp_manual_stop")
        return 0

    def udp_takeoff(self) -> int:
        self._record("udp_takeoff")
        return 0

    def udp_land(self) -> int:
        self._record("udp_land")
        return 0

    def udp_manual_setpoint(self, throttle: float, pitch: float, roll: float, yaw: float, timeout: float = 1.0) -> int:
        self._record("udp_manual_setpoint", throttle, pitch, roll, yaw, timeout)
        return 0

    def start_csv_log(self, output_path: Path, **kwargs) -> None:
        self._record("start_csv_log", output_path, **kwargs)
        self.last_log_path = output_path

    def stop_csv_log(self) -> Path | None:
        self._record("stop_csv_log")
        return self.last_log_path

    def dump_csv(self, output_path: Path, duration_s: float = 5.0) -> int:
        self._record("dump_csv", output_path, duration_s)
        self.last_log_path = output_path
        return 3

    def hello(self):
        self._record("hello")
        self.device_info = type("DeviceInfoStub", (), {
            "protocol_version": 5,
            "imu_mode": 1,
            "arm_state": 0,
            "stream_enabled": 0,
            "feature_bitmap": 0x7F,
        })()
        return self.device_info


def seed_motor_scale_offset_params(session: FakeSession) -> None:
    for name, value in (
        ("motor_scale_m1", 1.0),
        ("motor_scale_m2", 1.10),
        ("motor_scale_m3", 0.9126),
        ("motor_scale_m4", 1.10),
        ("motor_offset_m1", 0.0),
        ("motor_offset_m2", 0.0),
        ("motor_offset_m3", 0.0),
        ("motor_offset_m4", 0.0),
    ):
        session.set_param(name, 4, value)
    session.calls.clear()


def test_device_session_mock_roundtrip(tmp_path: Path):
    session = DeviceSession()
    transport = MockTransport()
    info = session.connect_transport(transport)
    assert info.protocol_version == 5
    assert TELEMETRY_STRUCT.size == TELEMETRY_STRUCT_V3.size
    assert session.arm() == 0
    assert session.disarm() == 0
    assert session.kill() == 0
    session.start_stream()
    session.stop_stream()
    assert session.get_param("demo").value == 2.5
    assert session.set_param("demo", 4, "3.5").value == 3.5

    params = session.list_params(timeout=1.0)
    assert [item.name for item in params] == ["alpha", "beta"]

    exported = session.export_params(tmp_path / "params.json")
    assert exported.schema == 1
    payload = json.loads((tmp_path / "params.json").read_text(encoding="utf-8"))
    assert payload["params"][0]["name"] == "alpha"

    import_path = tmp_path / "import.json"
    import_path.write_text(
        json.dumps({"params": [{"name": "alpha", "type_id": 2, "value": 7}]}),
        encoding="utf-8",
    )
    applied = session.import_params(import_path)
    assert applied[0].name == "alpha"
    session.disconnect()
    assert transport.closed


def test_connect_transport_failure_preserves_error_and_callback():
    session = DeviceSession()
    events: list[dict[str, object]] = []
    session.subscribe_connection_state(events.append)
    transport = TimeoutHelloTransport()

    with pytest.raises(TimeoutError):
        session.connect_transport(transport, hello_timeout=0.05)

    assert transport.closed
    assert session.last_error is not None
    assert "timed out waiting" in session.last_error
    assert events
    assert events[-1]["connected"] is False
    assert events[-1]["error"] == session.last_error


def test_connect_serial_retries_one_hello_timeout_without_reporting_first_failure(monkeypatch):
    from esp_drone_cli.core import device_session as device_session_module

    first = TimeoutHelloTransport()
    second = MockTransport()
    transports = [first, second]

    def fake_serial_transport(*_args, **_kwargs):
        return transports.pop(0)

    monkeypatch.setattr(device_session_module, "SerialTransport", fake_serial_transport)
    session = device_session_module.DeviceSession()
    events: list[dict[str, object]] = []
    session.subscribe_connection_state(events.append)

    info = session.connect_serial("COM7", hello_timeout=0.05)

    assert info.protocol_version == 5
    assert first.closed
    assert session.last_error is None
    assert [event["connected"] for event in events] == [True]
    session.disconnect()


def test_connect_serial_reports_real_error_after_retry_exhaustion(monkeypatch):
    from esp_drone_cli.core import device_session as device_session_module

    original_transports = [TimeoutHelloTransport(), TimeoutHelloTransport()]
    transports = list(original_transports)

    def fake_serial_transport(*_args, **_kwargs):
        return transports.pop(0)

    monkeypatch.setattr(device_session_module, "SerialTransport", fake_serial_transport)
    session = device_session_module.DeviceSession()
    events: list[dict[str, object]] = []
    session.subscribe_connection_state(events.append)

    with pytest.raises(TimeoutError):
        session.connect_serial("COM7", hello_timeout=0.05)

    assert all(transport.closed for transport in original_transports)
    assert session.last_error is not None
    assert "timed out waiting" in session.last_error
    assert events
    assert events[-1]["connected"] is False
    assert events[-1]["error"] == session.last_error


def test_hello_resp_v2_decodes_build_identity_and_capabilities():
    payload = HELLO_RESP_STRUCT_V2.pack(
        4,
        1,
        0,
        0,
        0x3F,
        b"dfcc7a779368\x00\x00\x00\x00",
        b"2026-04-11T12:00:00Z\x00\x00\x00\x00",
    )

    info = decode_device_info(payload)

    assert info.protocol_version == 4
    assert info.feature_bitmap == 0x3F
    assert info.build_git_hash == "dfcc7a779368"
    assert info.build_time_utc == "2026-04-11T12:00:00Z"
    assert info.supports_feature(FEATURE_ATTITUDE_HANG_BENCH)
    assert "attitude_hang_bench" in info.feature_names()


def test_hello_resp_v2_decodes_attitude_ground_verify_capabilities():
    payload = HELLO_RESP_STRUCT_V2.pack(
        8,
        1,
        0,
        0,
        0x3FF,
        b"att-v8".ljust(16, b"\x00"),
        b"2026-04-18T00:00:00Z".ljust(24, b"\x00"),
    )

    info = decode_device_info(payload)

    assert info.protocol_version == 8
    assert info.supports_feature(FEATURE_ATTITUDE_GROUND_VERIFY)
    assert info.supports_feature(FEATURE_LOW_RISK_LIFTOFF_VERIFY)
    assert "attitude_ground_verify" in info.feature_names()
    assert "low_risk_liftoff_verify" in info.feature_names()
    info.require_attitude_ground_verify()
    info.require_low_risk_liftoff_verify()


def test_hello_resp_v2_decodes_all_motor_test_capability():
    payload = HELLO_RESP_STRUCT_V2.pack(
        9,
        1,
        0,
        0,
        0x7FF,
        b"all-v9".ljust(16, b"\x00"),
        b"2026-04-19T00:00:00Z".ljust(24, b"\x00"),
    )

    info = decode_device_info(payload)

    assert info.protocol_version == 9
    assert info.supports_feature(FEATURE_ALL_MOTOR_TEST)
    assert "all_motor_test" in info.feature_names()
    info.require_all_motor_test()


def test_attitude_capture_ref_encodes_expected_opcode():
    session = DeviceSession()
    transport = MockTransport()
    session.connect_transport(transport)
    transport.sent.clear()

    assert session.attitude_capture_ref() == 0

    cmd_frames = [payload for msg_type, payload in transport.sent if msg_type == MsgType.CMD_REQ]
    assert cmd_frames
    cmd_id, arg_u8, _reserved, arg_f32 = CMD_REQ_STRUCT.unpack(cmd_frames[-1])
    assert cmd_id == CmdId.ATTITUDE_CAPTURE_REF
    assert arg_u8 == 0
    assert arg_f32 == pytest.approx(0.0)
    session.disconnect()


def test_attitude_capture_ref_rejects_old_firmware_before_opcode():
    class OldFirmwareTransport(MockTransport):
        def send_message(self, msg_type: int, payload: bytes = b"", flags: int = 0, seq: int = 0) -> None:
            self.sent.append((msg_type, payload))
            if msg_type == MsgType.HELLO_REQ:
                hello = HELLO_RESP_STRUCT.pack(2, 1, 0, 0, 0x1F)
                self.inject(Frame(MsgType.HELLO_RESP, flags, seq, hello))
                return
            if msg_type == MsgType.CMD_REQ:
                raise AssertionError("old firmware capability gate should reject before CMD_REQ")
            super().send_message(msg_type, payload, flags=flags, seq=seq)

    session = DeviceSession()
    transport = OldFirmwareTransport()
    session.connect_transport(transport)

    with pytest.raises(CapabilityError, match="does not advertise bench-only hang-attitude support"):
        session.attitude_capture_ref()

    assert [msg_type for msg_type, _payload in transport.sent].count(MsgType.CMD_REQ) == 0
    session.disconnect()


def test_device_session_ground_capture_ref_waits_for_cmd_resp_not_param_value():
    class GroundTuneTransport(MockTransport):
        def send_message(self, msg_type: int, payload: bytes = b"", flags: int = 0, seq: int = 0) -> None:
            if msg_type == MsgType.HELLO_REQ:
                self.sent.append((msg_type, payload))
                hello = HELLO_RESP_STRUCT_V2.pack(
                    6,
                    1,
                    0,
                    0,
                    0xFF,
                    b"unit-test".ljust(16, b"\x00"),
                    b"2026-04-14T00:00:00Z".ljust(24, b"\x00"),
                )
                self.inject(Frame(MsgType.HELLO_RESP, flags, seq, hello))
                return
            super().send_message(msg_type, payload, flags=flags, seq=seq)

    session = DeviceSession()
    transport = GroundTuneTransport()
    session.connect_transport(transport)
    transport.sent.clear()

    assert session.ground_capture_ref() == 0

    cmd_frames = [payload for msg_type, payload in transport.sent if msg_type == MsgType.CMD_REQ]
    assert len(cmd_frames) == 1
    cmd_id, arg_u8, _reserved, arg_f32 = CMD_REQ_STRUCT.unpack(cmd_frames[0])
    assert cmd_id == CmdId.GROUND_CAPTURE_REF
    assert arg_u8 == 0
    assert arg_f32 == pytest.approx(0.0)
    assert MsgType.PARAM_GET not in [msg_type for msg_type, _payload in transport.sent]
    assert MsgType.PARAM_SET not in [msg_type for msg_type, _payload in transport.sent]
    session.disconnect()


def test_device_session_attitude_ground_and_liftoff_commands_encode_expected_opcodes():
    class AttitudeGroundTransport(MockTransport):
        def send_message(self, msg_type: int, payload: bytes = b"", flags: int = 0, seq: int = 0) -> None:
            if msg_type == MsgType.HELLO_REQ:
                self.sent.append((msg_type, payload))
                hello = HELLO_RESP_STRUCT_V2.pack(
                    8,
                    1,
                    0,
                    0,
                    0x3FF,
                    b"unit-test".ljust(16, b"\x00"),
                    b"2026-04-18T00:00:00Z".ljust(24, b"\x00"),
                )
                self.inject(Frame(MsgType.HELLO_RESP, flags, seq, hello))
                return
            super().send_message(msg_type, payload, flags=flags, seq=seq)

    session = DeviceSession()
    transport = AttitudeGroundTransport()
    session.connect_transport(transport)
    transport.sent.clear()

    assert session.attitude_ground_verify_start(base_duty=0.08) == 0
    assert session.attitude_ground_set_target(0, 1.5) == 0
    assert session.attitude_ground_verify_stop() == 0
    assert session.liftoff_verify_start(base_duty=0.10) == 0
    assert session.liftoff_verify_stop() == 0

    cmd_frames = [payload for msg_type, payload in transport.sent if msg_type == MsgType.CMD_REQ]
    decoded = [CMD_REQ_STRUCT.unpack(payload) for payload in cmd_frames]
    assert [item[0] for item in decoded] == [
        CmdId.ATTITUDE_GROUND_VERIFY_START,
        CmdId.ATTITUDE_GROUND_SET_TARGET,
        CmdId.ATTITUDE_GROUND_VERIFY_STOP,
        CmdId.LIFTOFF_VERIFY_START,
        CmdId.LIFTOFF_VERIFY_STOP,
    ]
    assert decoded[0][3] == pytest.approx(0.08)
    assert decoded[1][1] == 0
    assert decoded[1][3] == pytest.approx(1.5)
    assert decoded[3][3] == pytest.approx(0.10)
    session.disconnect()


def test_device_session_all_motor_test_encodes_expected_opcodes():
    class AllMotorTransport(MockTransport):
        def send_message(self, msg_type: int, payload: bytes = b"", flags: int = 0, seq: int = 0) -> None:
            if msg_type == MsgType.HELLO_REQ:
                self.sent.append((msg_type, payload))
                hello = HELLO_RESP_STRUCT_V2.pack(
                    9,
                    1,
                    0,
                    0,
                    0x7FF,
                    b"unit-test".ljust(16, b"\x00"),
                    b"2026-04-19T00:00:00Z".ljust(24, b"\x00"),
                )
                self.inject(Frame(MsgType.HELLO_RESP, flags, seq, hello))
                return
            super().send_message(msg_type, payload, flags=flags, seq=seq)

    session = DeviceSession()
    transport = AllMotorTransport()
    session.connect_transport(transport)
    transport.sent.clear()

    assert session.all_motor_test_start(0.30, 2.0) == 0
    assert session.all_motor_test_stop() == 0

    cmd_frames = [payload for msg_type, payload in transport.sent if msg_type == MsgType.CMD_REQ]
    decoded = [CMD_REQ_STRUCT.unpack(payload) for payload in cmd_frames]
    assert [item[0] for item in decoded] == [CmdId.ALL_MOTOR_TEST_START, CmdId.ALL_MOTOR_TEST_STOP]
    assert decoded[0][1] == 20
    assert decoded[0][3] == pytest.approx(0.30)
    session.disconnect()


def test_device_session_stream_ack_must_match_requested_state():
    session = DeviceSession()
    transport = MockTransport()
    session.connect_transport(transport)
    transport.sent.clear()

    transport.inject(Frame(MsgType.STREAM_CTRL, 0, 99, b"\x01"))
    session.stop_stream()

    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline and not transport._frames.empty():
        time.sleep(0.01)

    leftovers: list[Frame] = []
    while not session._response_queue.empty():
        leftovers.append(session._response_queue.get_nowait())
    leftovers.extend(session._pending_response_frames)

    assert [payload for msg_type, payload in transport.sent if msg_type == MsgType.STREAM_CTRL] == [b"\x00"]
    assert not [frame for frame in leftovers if frame.msg_type == MsgType.STREAM_CTRL]
    session.disconnect()


def test_device_session_param_value_must_match_requested_name():
    class StaleParamTransport(MockTransport):
        def send_message(self, msg_type: int, payload: bytes = b"", flags: int = 0, seq: int = 0) -> None:
            if msg_type == MsgType.PARAM_GET:
                self.sent.append((msg_type, payload))
                self.inject(Frame(MsgType.PARAM_VALUE, flags, seq, encode_param_payload("other", 4, b"\x00\x00\x00@")))
                self.inject(Frame(MsgType.PARAM_VALUE, flags, seq, encode_param_payload("target", 4, b"\x00\x00\x80@")))
                return
            super().send_message(msg_type, payload, flags=flags, seq=seq)

    session = DeviceSession()
    transport = StaleParamTransport()
    session.connect_transport(transport)

    result = session.get_param("target")

    assert result.name == "target"
    assert result.value == pytest.approx(4.0)
    session.disconnect()


def test_param_string_value_round_trips_over_host_protocol():
    payload = encode_param_payload("wifi_mode", 5, b"sta")
    decoded = decode_param_value(payload)

    assert decoded == ParamValue("wifi_mode", 5, "sta")
    assert encode_param_value(5, "apsta") == b"apsta"

    session = DeviceSession()
    transport = MockTransport()
    session.connect_transport(transport)

    result = session.set_param("sta_ssid", 5, "bench-hotspot")

    assert result == ParamValue("sta_ssid", 5, "bench-hotspot")
    session.disconnect()


def test_udp_manual_setpoint_encodes_expected_payload():
    class StabilizeMinTransport(MockTransport):
        def send_message(self, msg_type: int, payload: bytes = b"", flags: int = 0, seq: int = 0) -> None:
            if msg_type == MsgType.HELLO_REQ:
                self.sent.append((msg_type, payload))
                hello = HELLO_RESP_STRUCT.pack(10, 1, 0, 0, 0x1FFF)
                self.inject(Frame(MsgType.HELLO_RESP, flags, seq, hello))
                return
            super().send_message(msg_type, payload, flags=flags, seq=seq)

    session = DeviceSession()
    transport = StabilizeMinTransport()
    session.connect_transport(transport)
    transport.sent.clear()

    assert session.udp_manual_setpoint(throttle=0.10, pitch=-0.02, roll=0.01, yaw=0.03) == 0

    setpoint_frames = [payload for msg_type, payload in transport.sent if msg_type == MsgType.UDP_MANUAL_SETPOINT]
    assert setpoint_frames
    throttle, pitch, roll, yaw = UDP_MANUAL_SETPOINT_STRUCT.unpack(setpoint_frames[-1])
    assert throttle == pytest.approx(0.10)
    assert pitch == pytest.approx(-0.02)
    assert roll == pytest.approx(0.01)
    assert yaw == pytest.approx(0.03)
    session.disconnect()


def test_udp_transport_connects_and_sends_manual_control_frames():
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(("127.0.0.1", 0))
    server.settimeout(2.0)
    host, port = server.getsockname()
    received: list[Frame] = []

    def handle_once(expected_type: int, response_type: int, response_payload: bytes) -> None:
        data, addr = server.recvfrom(2048)
        frame = decode_frame(data)
        received.append(frame)
        assert frame.msg_type == expected_type
        server.sendto(encode_frame(response_type, response_payload, seq=frame.seq), addr)

    def server_thread() -> None:
        try:
            hello = HELLO_RESP_STRUCT_V2.pack(
                10,
                1,
                0,
                0,
                0x1FFF,
                b"udp-test\x00\x00\x00\x00\x00\x00\x00\x00",
                b"2026-04-12T00:00:00Z\x00\x00\x00\x00",
            )
            handle_once(MsgType.HELLO_REQ, MsgType.HELLO_RESP, hello)
            handle_once(MsgType.CMD_REQ, MsgType.CMD_RESP, bytes([CmdId.UDP_MANUAL_ENABLE, 0, 0, 0]))
            handle_once(
                MsgType.UDP_MANUAL_SETPOINT,
                MsgType.CMD_RESP,
                bytes([CmdId.UDP_MANUAL_SETPOINT, 0, 0, 0]),
            )
        finally:
            server.close()

    thread = threading.Thread(target=server_thread, daemon=True)
    thread.start()

    session = DeviceSession()
    info = session.connect_udp(host, port=port, timeout=1.0)
    assert info.protocol_version == 10
    assert session.udp_manual_enable() == 0
    assert session.udp_manual_setpoint(throttle=0.08, pitch=-0.01, roll=0.0, yaw=0.02) == 0
    session.disconnect()
    thread.join(timeout=1.0)

    assert [frame.msg_type for frame in received] == [
        MsgType.HELLO_REQ,
        MsgType.CMD_REQ,
        MsgType.UDP_MANUAL_SETPOINT,
    ]
    throttle, pitch, roll, yaw = UDP_MANUAL_SETPOINT_STRUCT.unpack(received[-1].payload)
    assert throttle == pytest.approx(0.08)
    assert pitch == pytest.approx(-0.01)
    assert roll == pytest.approx(0.0)
    assert yaw == pytest.approx(0.02)


def test_device_session_telemetry_subscription_receives_samples():
    session = DeviceSession()
    transport = MockTransport()
    session.connect_transport(transport)
    received = []
    token = session.subscribe_telemetry(received.append)
    transport.inject(Frame(MsgType.TELEMETRY_SAMPLE, 0, 0, build_telemetry_payload()))

    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline and not received:
        time.sleep(0.01)

    session.unsubscribe(token)
    session.disconnect()
    assert received
    assert received[0].gyro_x == 1.0
    assert received[0].control_mode == 2
    assert received[0].baro_altitude_m == pytest.approx(1.25)
    assert received[0].attitude_err_roll_deg == pytest.approx(2.5)
    assert received[0].attitude_rate_sp_pitch == pytest.approx(7.0)
    assert received[0].attitude_ref_valid == 1


def test_telemetry_sample_rate_debug_projection_uses_project_axis_truth():
    sample = TelemetrySample.from_payload(build_telemetry_payload())
    roll = sample.axis_rate_debug_map("roll")
    pitch = sample.axis_rate_debug_map("pitch")
    yaw = sample.axis_rate_debug_map("yaw")

    assert roll["source_field"] == "gyro_y"
    assert roll["source_value"] == pytest.approx(2.0)
    assert roll["feedback_field"] == "roll_rate"
    assert roll["feedback_expr"] == "-gyro_y"
    assert roll["setpoint_field"] == "rate_setpoint_roll"
    assert roll["feedback_dps"] == pytest.approx(-2.0)
    assert roll["setpoint_dps"] == pytest.approx(10.0)

    assert pitch["source_field"] == "gyro_x"
    assert pitch["feedback_expr"] == "gyro_x"
    assert pitch["feedback_dps"] == pytest.approx(1.0)
    assert pitch["setpoint_dps"] == pytest.approx(11.0)

    assert yaw["source_field"] == "gyro_z"
    assert yaw["feedback_expr"] == "-gyro_z"
    assert yaw["feedback_dps"] == pytest.approx(-3.0)
    assert yaw["setpoint_dps"] == pytest.approx(12.0)


def test_format_rate_status_line_uses_explicit_roll_field_names():
    sample = TelemetrySample.from_payload(build_telemetry_payload())
    line = format_rate_status_line(sample, "roll")

    assert "rate_setpoint_roll=10.000" in line
    assert "roll_rate=-2.000" in line
    assert "source_expr=-gyro_y" in line
    assert "raw_gyro_y=2.000" in line
    assert "rate_pid_p_roll=0.5000" in line
    assert "rate_pid_i_roll=0.1000" in line
    assert "rate_pid_d_roll=0.0100" in line
    assert "pid_out_roll=0.9000" in line
    assert "motor1..motor4=[" in line
    assert "arm_state=0" in line
    assert "control_mode=2" in line


def test_telemetry_sample_v1_payload_keeps_baro_defaults():
    sample = TelemetrySample.from_payload(build_telemetry_payload_v1())
    assert sample.gyro_x == 1.0
    assert sample.baro_valid == 0
    assert sample.baro_pressure_pa == 0.0


def test_telemetry_sample_v4_decodes_estimator_fields_after_reserved_bytes():
    sample = TelemetrySample.from_payload(build_telemetry_payload_v4())
    row = dict(zip(TELEMETRY_CSV_FIELDS, sample.to_csv_row()))

    assert sample.attitude_ref_valid == 1
    assert sample.filtered_gyro_x == pytest.approx(1.25)
    assert sample.filtered_gyro_y == pytest.approx(-2.5)
    assert sample.filtered_gyro_z == pytest.approx(3.75)
    assert sample.filtered_acc_x == pytest.approx(0.12)
    assert sample.filtered_acc_y == pytest.approx(-0.34)
    assert sample.filtered_acc_z == pytest.approx(0.98)
    assert sample.kalman_roll_deg == pytest.approx(9.5)
    assert sample.kalman_pitch_deg == pytest.approx(-8.25)
    assert sample.rate_meas_roll_raw == pytest.approx(-21.0)
    assert sample.rate_meas_pitch_filtered == pytest.approx(12.0)
    assert sample.rate_err_yaw == pytest.approx(0.33)
    assert sample.sample_seq == 4242
    assert sample.attitude_valid == 1
    assert sample.kalman_valid == 1
    assert sample.integrator_freeze_flag == 1
    assert sample.ground_ref_valid == 1
    assert sample.reference_valid == 1
    assert sample.ground_trip_reason == 3
    assert sample.battery_valid == 1

    assert row["raw_gyro_x"] == pytest.approx(1.0)
    assert row["filtered_gyro_x"] == pytest.approx(1.25)
    assert row["filtered_acc_z"] == pytest.approx(0.98)
    assert row["mixer_throttle"] == pytest.approx(0.05)
    assert row["mixer_roll"] == pytest.approx(0.9)
    assert row["kalman_valid"] == 1
    assert row["attitude_valid"] == 1
    assert row["battery_valid"] == 1


def test_telemetry_sample_v5_decodes_attitude_ground_verify_fields():
    sample = TelemetrySample.from_payload(build_telemetry_payload_v5())
    row = dict(zip(TELEMETRY_CSV_FIELDS, sample.to_csv_row()))

    assert sample.angle_target_roll == pytest.approx(1.0)
    assert sample.angle_target_pitch == pytest.approx(-2.0)
    assert sample.angle_measured_roll == pytest.approx(0.4)
    assert sample.angle_measured_pitch == pytest.approx(-0.6)
    assert sample.angle_error_roll == pytest.approx(0.6)
    assert sample.angle_error_pitch == pytest.approx(-1.4)
    assert sample.outer_loop_rate_target_roll == pytest.approx(0.72)
    assert sample.outer_loop_rate_target_pitch == pytest.approx(-1.68)
    assert sample.outer_loop_clamp_flag == 1
    assert sample.inner_loop_clamp_flag == 3
    assert sample.control_submode == 1
    assert row["angle_target_roll"] == pytest.approx(1.0)
    assert row["outer_loop_clamp_flag"] == 1
    assert row["control_submode"] == 1


def test_attitude_ground_verify_analysis_accepts_small_symmetric_targets():
    samples = []
    for axis_name, target, pid_out in (
        ("roll", 1.0, 0.0006),
        ("roll", -1.0, -0.0006),
        ("pitch", 1.0, 0.0006),
        ("pitch", -1.0, -0.0006),
    ):
        for _ in range(4):
            sample = TelemetrySample.from_payload(build_telemetry_payload_v5())
            sample.control_mode = 6
            sample.control_submode = 1
            sample.kalman_valid = 1
            sample.attitude_valid = 1
            sample.ground_ref_valid = 1
            sample.failsafe_reason = 0
            sample.ground_trip_reason = 0
            sample.outer_loop_clamp_flag = 0
            sample.inner_loop_clamp_flag = 0
            sample.motor_saturation_flag = 0
            sample.angle_target_roll = target if axis_name == "roll" else 0.0
            sample.angle_target_pitch = target if axis_name == "pitch" else 0.0
            sample.angle_target_yaw = 0.0
            sample.angle_error_roll = target if axis_name == "roll" else 0.0
            sample.angle_error_pitch = target if axis_name == "pitch" else 0.0
            sample.outer_loop_rate_target_roll = 0.8 * target if axis_name == "roll" else 0.0
            sample.outer_loop_rate_target_pitch = 0.8 * target if axis_name == "pitch" else 0.0
            sample.outer_loop_rate_target_yaw = 0.0
            sample.rate_setpoint_yaw = 0.0
            if axis_name == "roll":
                sample.rate_err_roll = 0.8 * target
                sample.rate_pid_p_roll = pid_out
                sample.pid_out_roll = pid_out
                sample.motor1 = 0.08 + pid_out
                sample.motor4 = 0.08 + pid_out
                sample.motor2 = 0.08 - pid_out
                sample.motor3 = 0.08 - pid_out
            else:
                sample.rate_err_pitch = 0.8 * target
                sample.rate_pid_p_pitch = pid_out
                sample.pid_out_pitch = pid_out
                sample.motor3 = 0.08 + pid_out
                sample.motor4 = 0.08 + pid_out
                sample.motor1 = 0.08 - pid_out
                sample.motor2 = 0.08 - pid_out
            samples.append(sample)

    result = analyze_attitude_ground_verify_samples(samples, target_deg=1.0)

    assert result["validity_ok"] is True
    assert result["safety_ok"] is True
    assert result["yaw_ok"] is True
    assert result["chain_ok"] is True
    assert result["passed"] is True


def test_attitude_ground_verify_analysis_accepts_rate_overshoot_correction():
    samples = []
    cases = (
        ("roll", 1.0, 0.8, 0.0006),
        ("roll", -1.0, 0.4, 0.0003),
        ("pitch", 1.0, -0.5, -0.0004),
        ("pitch", -1.0, -0.8, -0.0006),
    )
    for axis_name, target, rate_error, pid_out in cases:
        for _ in range(4):
            sample = TelemetrySample.from_payload(build_telemetry_payload_v5())
            sample.control_mode = 6
            sample.control_submode = 1
            sample.kalman_valid = 1
            sample.attitude_valid = 1
            sample.ground_ref_valid = 1
            sample.failsafe_reason = 0
            sample.ground_trip_reason = 0
            sample.outer_loop_clamp_flag = 0
            sample.inner_loop_clamp_flag = 2
            sample.motor_saturation_flag = 0
            sample.angle_target_roll = target if axis_name == "roll" else 0.0
            sample.angle_target_pitch = target if axis_name == "pitch" else 0.0
            sample.angle_target_yaw = 0.0
            sample.angle_error_roll = target if axis_name == "roll" else 0.0
            sample.angle_error_pitch = target if axis_name == "pitch" else 0.0
            sample.outer_loop_rate_target_roll = 0.8 * target if axis_name == "roll" else 0.0
            sample.outer_loop_rate_target_pitch = 0.8 * target if axis_name == "pitch" else 0.0
            sample.outer_loop_rate_target_yaw = 0.0
            sample.rate_setpoint_yaw = 0.0
            if axis_name == "roll":
                sample.rate_err_roll = rate_error
                sample.rate_pid_p_roll = pid_out
                sample.pid_out_roll = pid_out
                sample.motor1 = 0.08 + pid_out
                sample.motor4 = 0.08 + pid_out
                sample.motor2 = 0.08 - pid_out
                sample.motor3 = 0.08 - pid_out
            else:
                sample.rate_err_pitch = rate_error
                sample.rate_pid_p_pitch = pid_out
                sample.pid_out_pitch = pid_out
                sample.motor3 = 0.08 + pid_out
                sample.motor4 = 0.08 + pid_out
                sample.motor1 = 0.08 - pid_out
                sample.motor2 = 0.08 - pid_out
            samples.append(sample)

    result = analyze_attitude_ground_verify_samples(samples, target_deg=1.0)

    assert result["inner_clamp_max"] == 2
    assert result["inner_motor_clamp_max"] == 0
    assert result["chain_ok"] is True
    assert result["passed"] is True


def make_liftoff_sample(
    index: int,
    *,
    base_duty_active: float,
    angle_roll: float = 0.5,
    angle_pitch: float = -0.5,
    timestamp_start_us: int = 5_000_000,
    step_us: int = 20_000,
) -> TelemetrySample:
    sample = TelemetrySample.from_payload(build_telemetry_payload_v5())
    sample.timestamp_us = timestamp_start_us + index * step_us
    sample.control_mode = 6
    sample.control_submode = 2
    sample.kalman_valid = 1
    sample.attitude_valid = 1
    sample.ground_ref_valid = 1
    sample.battery_valid = 1
    sample.failsafe_reason = 0
    sample.ground_trip_reason = 0
    sample.outer_loop_clamp_flag = 0
    sample.inner_loop_clamp_flag = 0
    sample.motor_saturation_flag = 0
    sample.base_duty_active = base_duty_active
    sample.angle_target_roll = 0.0
    sample.angle_target_pitch = 0.0
    sample.angle_target_yaw = 0.0
    sample.angle_measured_roll = angle_roll
    sample.angle_measured_pitch = angle_pitch
    sample.angle_error_roll = -angle_roll
    sample.angle_error_pitch = -angle_pitch
    sample.angle_error_yaw = 0.0
    sample.outer_loop_rate_target_roll = -0.8 * angle_roll
    sample.outer_loop_rate_target_pitch = -0.8 * angle_pitch
    sample.outer_loop_rate_target_yaw = 0.0
    sample.rate_setpoint_roll = sample.outer_loop_rate_target_roll
    sample.rate_setpoint_pitch = sample.outer_loop_rate_target_pitch
    sample.rate_setpoint_yaw = 0.0
    sample.rate_err_roll = sample.outer_loop_rate_target_roll * 0.75
    sample.rate_err_pitch = sample.outer_loop_rate_target_pitch * 0.75
    sample.rate_err_yaw = 0.0
    sample.rate_pid_p_roll = sample.rate_err_roll * 0.0007
    sample.rate_pid_p_pitch = sample.rate_err_pitch * 0.0007
    sample.rate_pid_p_yaw = 0.0
    sample.pid_out_roll = sample.rate_pid_p_roll
    sample.pid_out_pitch = sample.rate_pid_p_pitch
    sample.pid_out_yaw = 0.0
    sample.motor1 = base_duty_active + sample.pid_out_roll - sample.pid_out_pitch
    sample.motor2 = base_duty_active - sample.pid_out_roll - sample.pid_out_pitch
    sample.motor3 = base_duty_active - sample.pid_out_roll + sample.pid_out_pitch
    sample.motor4 = base_duty_active + sample.pid_out_roll + sample.pid_out_pitch
    sample.rate_meas_yaw_filtered = 1.0
    sample.baro_valid = 1
    sample.baro_altitude_m = 0.001 * index
    return sample


def test_liftoff_verify_analysis_reports_early_ramp_only_when_target_not_reached():
    samples = []
    for index in range(60):
        base = min(0.095, 0.001 + 0.08 * (index * 0.02))
        sample = make_liftoff_sample(index, base_duty_active=base, angle_roll=1.2, angle_pitch=-0.2)
        if index == 0:
            sample.inner_loop_clamp_flag = 3
            sample.motor_saturation_flag = 1
        samples.append(sample)

    result = analyze_liftoff_verify_samples(samples, base_duty=0.16)

    assert result["early_ramp_safety_pass"] is True
    assert result["target_hit_reached"] is False
    assert result["target_hit_pass"] is False
    assert result["long_ground_hold_pass"] is False
    assert result["passed"] is True

    summary = "\n".join(format_liftoff_verify_summary(result))
    assert "early_ramp_safety_pass=True" in summary
    assert "target_hit_pass=False" in summary
    assert "long_ground_hold_pass=False" in summary


def test_liftoff_verify_target_hit_pass_is_independent_from_full_log_validity():
    samples = []
    for index in range(100):
        t_s = index * 0.02
        base = min(0.16, 0.10 * t_s)
        sample = make_liftoff_sample(index, base_duty_active=base, angle_roll=0.6, angle_pitch=-0.4)
        if index > 92:
            sample.kalman_valid = 0
            sample.rate_setpoint_pitch = sample.outer_loop_rate_target_pitch + 0.4
        samples.append(sample)

    result = analyze_liftoff_verify_samples(samples, base_duty=0.16)

    assert result["target_hit_reached"] is True
    assert result["target_hit_time_s"] == pytest.approx(1.60)
    assert result["target_hit_window_duration_s"] >= 0.15
    assert result["target_hit_window_validity_ok"] is True
    assert result["target_hit_window_output_ok"] is True
    assert result["target_hit_window_attitude_ok"] is True
    assert result["pre_hit_ready_pass"] is True
    assert result["pre_hit_roll_abs_max_deg"] == pytest.approx(0.6)
    assert result["pre_hit_pitch_abs_max_deg"] == pytest.approx(0.4)
    assert result["target_hit_pass"] is True
    assert result["validity_ok"] is False
    assert result["unified_path_ok"] is False
    assert result["long_ground_hold_pass"] is False
    assert result["passed"] is True
    assert result["sample_class"] == "clean_target_hit_pass"


def test_liftoff_verify_pre_start_readiness_qualifies_clean_sample():
    samples = []
    for index in range(20):
        sample = make_liftoff_sample(
            index,
            base_duty_active=0.0,
            angle_roll=0.05,
            angle_pitch=-0.04,
            timestamp_start_us=4_600_000,
        )
        sample.control_mode = 0
        sample.control_submode = 0
        sample.filtered_gyro_x = 0.2
        sample.filtered_gyro_y = -0.1
        sample.filtered_gyro_z = 0.15
        samples.append(sample)
    for index in range(100):
        t_s = index * 0.02
        base = min(0.16, 0.10 * t_s)
        sample = make_liftoff_sample(index, base_duty_active=base, angle_roll=0.6, angle_pitch=-0.4)
        samples.append(sample)

    result = analyze_liftoff_verify_samples(samples, base_duty=0.16)

    assert result["pre_start_available"] is True
    assert result["pre_start_ready_pass"] is True
    assert result["pre_start_roll_pp_deg"] == pytest.approx(0.0)
    assert result["pre_start_pitch_pp_deg"] == pytest.approx(0.0)
    assert result["pre_start_gyro_abs_max_dps"] == pytest.approx(0.2)
    assert result["target_hit_pass"] is True
    assert result["sample_class"] == "clean_target_hit_pass"


def test_liftoff_verify_pre_start_not_ready_is_pollution_not_clean_failure():
    samples = []
    for index in range(20):
        roll = 0.12 * index
        pitch = -0.04 * index
        sample = make_liftoff_sample(
            index,
            base_duty_active=0.0,
            angle_roll=roll,
            angle_pitch=pitch,
            timestamp_start_us=4_600_000,
        )
        sample.control_mode = 0
        sample.control_submode = 0
        sample.filtered_gyro_x = 0.4
        sample.filtered_gyro_y = 0.2
        sample.filtered_gyro_z = 0.3
        samples.append(sample)
    for index in range(100):
        t_s = index * 0.02
        base = min(0.16, 0.10 * t_s)
        sample = make_liftoff_sample(index, base_duty_active=base, angle_roll=0.6, angle_pitch=-0.4)
        samples.append(sample)

    result = analyze_liftoff_verify_samples(samples, base_duty=0.16)

    assert result["pre_start_available"] is True
    assert result["pre_start_ready_pass"] is False
    assert result["pre_start_roll_pp_deg"] > 0.8
    assert result["target_hit_pass"] is True
    assert result["sample_class"] == "pre_start_not_ready_sample"


def test_current_liftoff_pre_start_readiness_requires_validity():
    samples = []
    for index in range(24):
        sample = make_liftoff_sample(
            index,
            base_duty_active=0.0,
            angle_roll=0.02,
            angle_pitch=-0.03,
            timestamp_start_us=3_000_000,
        )
        sample.control_mode = 0
        sample.control_submode = 0
        sample.filtered_gyro_x = 0.3
        sample.filtered_gyro_y = 0.2
        sample.filtered_gyro_z = 0.1
        samples.append(sample)

    clean = current_liftoff_pre_start_readiness(samples)
    assert clean["pre_start_available"] is True
    assert clean["pre_start_ready_pass"] is True

    samples[-2].kalman_valid = 0
    dirty = current_liftoff_pre_start_readiness(samples)
    assert dirty["pre_start_available"] is True
    assert dirty["pre_start_validity_ok"] is False
    assert dirty["pre_start_ready_pass"] is False


def test_liftoff_verify_pre_hit_attitude_worsening_blocks_target_hit_pass():
    samples = []
    for index in range(120):
        t_s = index * 0.02
        base = min(0.16, 0.10 * t_s)
        angle_pitch = -0.4
        if 1.40 <= t_s < 1.60:
            angle_pitch = -3.4
        sample = make_liftoff_sample(index, base_duty_active=base, angle_roll=0.6, angle_pitch=angle_pitch)
        samples.append(sample)

    result = analyze_liftoff_verify_samples(samples, base_duty=0.16)

    assert result["target_hit_reached"] is True
    assert result["target_hit_approach_ok"] is False
    assert result["pre_hit_ready_pass"] is False
    assert result["pre_hit_pitch_abs_max_deg"] == pytest.approx(3.4)
    assert result["pre_hit_validity_ok"] is True
    assert result["pre_hit_inner_clamp_seen"] is False
    assert result["pre_hit_saturation_seen"] is False
    assert result["target_hit_window_safety_ok"] is True
    assert result["target_hit_window_output_ok"] is True
    assert result["target_hit_pass"] is False
    assert result["sample_class"] == "pre_hit_polluted_sample"


def test_liftoff_verify_pre_hit_diagnosis_reports_output_edge_state():
    samples = []
    for index in range(100):
        t_s = index * 0.02
        base = min(0.16, 0.10 * t_s)
        sample = make_liftoff_sample(index, base_duty_active=base, angle_roll=0.5, angle_pitch=-0.5)
        if 1.48 <= t_s <= 1.60:
            sample.inner_loop_clamp_flag = 1
            sample.motor_saturation_flag = 1
        samples.append(sample)

    result = analyze_liftoff_verify_samples(samples, base_duty=0.16)

    assert result["target_hit_reached"] is True
    assert result["pre_hit_ready_pass"] is False
    assert result["pre_hit_inner_clamp_seen"] is True
    assert result["pre_hit_saturation_seen"] is True
    assert result["pre_hit_outer_clamp_seen"] is False


def test_near_threshold_accepts_marginal_pre_hit_if_post_hit_stays_controlled():
    samples = []
    for index in range(80):
        t_s = index * 0.02
        base = min(0.22, 0.25 * t_s)
        angle_roll = 1.2
        angle_pitch = -0.4
        if 0.70 <= t_s < 0.88:
            angle_roll = 3.5
        sample = make_liftoff_sample(index, base_duty_active=base, angle_roll=angle_roll, angle_pitch=angle_pitch)
        samples.append(sample)

    result = analyze_liftoff_verify_samples(samples, base_duty=0.22)

    assert result["target_hit_reached"] is True
    assert result["pre_hit_level"] == "marginal"
    assert result["near_threshold_window_duration_s"] >= 0.25
    assert result["near_threshold_sample_class"] == "near_threshold_marginal_pass"


def test_near_threshold_fails_on_terminal_trip_after_target_hit():
    samples = []
    for index in range(80):
        t_s = index * 0.02
        base = min(0.22, 0.25 * t_s)
        sample = make_liftoff_sample(index, base_duty_active=base, angle_roll=1.0, angle_pitch=-0.5)
        if t_s >= 0.92:
            sample.ground_trip_reason = 3
        samples.append(sample)

    result = analyze_liftoff_verify_samples(samples, base_duty=0.22)

    assert result["target_hit_reached"] is True
    assert result["pre_hit_level"] == "nominal"
    assert result["near_threshold_terminal_trip"] == 3
    assert result["near_threshold_sample_class"] == "near_threshold_fail"


def test_liftoff_verify_analysis_accepts_conservative_closed_loop_attempt():
    samples = []
    for index in range(12):
        sample = TelemetrySample.from_payload(build_telemetry_payload_v5())
        sample.timestamp_us = 1_000_000 + index * 20_000
        sample.control_mode = 6
        sample.control_submode = 2
        sample.kalman_valid = 1
        sample.attitude_valid = 1
        sample.ground_ref_valid = 1
        sample.battery_valid = 1
        sample.failsafe_reason = 0
        sample.ground_trip_reason = 0
        sample.outer_loop_clamp_flag = 0
        sample.inner_loop_clamp_flag = 2
        sample.motor_saturation_flag = 0
        sample.base_duty_active = 0.10
        sample.angle_target_roll = 0.0
        sample.angle_target_pitch = 0.0
        sample.angle_target_yaw = 0.0
        sample.angle_measured_roll = 1.0
        sample.angle_measured_pitch = -1.0
        sample.angle_error_roll = -1.0
        sample.angle_error_pitch = 1.0
        sample.angle_error_yaw = 0.0
        sample.outer_loop_rate_target_roll = -0.8
        sample.outer_loop_rate_target_pitch = 0.8
        sample.outer_loop_rate_target_yaw = 0.0
        sample.rate_setpoint_roll = -0.8
        sample.rate_setpoint_pitch = 0.8
        sample.rate_setpoint_yaw = 0.0
        sample.rate_err_roll = -0.6
        sample.rate_err_pitch = 0.6
        sample.rate_err_yaw = 0.0
        sample.rate_pid_p_roll = -0.00042
        sample.rate_pid_p_pitch = 0.00042
        sample.pid_out_roll = -0.00042
        sample.pid_out_pitch = 0.00042
        sample.pid_out_yaw = 0.0
        sample.motor1 = 0.10 - 0.00042 - 0.00042
        sample.motor2 = 0.10 + 0.00042 - 0.00042
        sample.motor3 = 0.10 + 0.00042 + 0.00042
        sample.motor4 = 0.10 - 0.00042 + 0.00042
        sample.rate_meas_yaw_filtered = 2.0
        sample.baro_valid = 1
        sample.baro_altitude_m = 0.03 * index
        samples.append(sample)

    result = analyze_liftoff_verify_samples(samples, base_duty=0.10)

    assert result["validity_ok"] is True
    assert result["safety_ok"] is True
    assert result["unified_path_ok"] is True
    assert result["chain_ok"] is True
    assert result["yaw_ok"] is True
    assert result["tilt_ok"] is True
    assert result["inner_motor_clamp_max"] == 0
    assert result["control_safe_pass"] is True
    assert result["physical_liftoff_state"] == "confirmed liftoff"
    assert result["physical_liftoff_confirmed"] is True
    assert result["free_flight_pass"] is False
    assert result["probable_liftoff"] is True
    assert result["passed"] is True

    summary = "\n".join(format_liftoff_verify_summary(result))
    assert "control_safe_pass=True" in summary
    assert "physical_liftoff=confirmed" in summary


def test_liftoff_verify_analysis_ignores_startup_idle_clamp():
    samples = []
    for index in range(6):
        sample = TelemetrySample.from_payload(build_telemetry_payload_v5())
        sample.timestamp_us = 2_000_000 + index * 20_000
        sample.control_mode = 6
        sample.control_submode = 2
        sample.kalman_valid = 1
        sample.attitude_valid = 1
        sample.ground_ref_valid = 1
        sample.battery_valid = 1
        sample.failsafe_reason = 0
        sample.ground_trip_reason = 0
        sample.outer_loop_clamp_flag = 0
        sample.inner_loop_clamp_flag = 3 if index == 0 else 0
        sample.motor_saturation_flag = 1 if index == 0 else 0
        sample.base_duty_active = 0.001 if index == 0 else 0.10
        sample.angle_target_roll = 0.0
        sample.angle_target_pitch = 0.0
        sample.angle_target_yaw = 0.0
        sample.angle_measured_roll = 0.5
        sample.angle_measured_pitch = -0.5
        sample.angle_error_roll = -0.5
        sample.angle_error_pitch = 0.5
        sample.outer_loop_rate_target_roll = -0.4
        sample.outer_loop_rate_target_pitch = 0.4
        sample.outer_loop_rate_target_yaw = 0.0
        sample.rate_setpoint_roll = -0.4
        sample.rate_setpoint_pitch = 0.4
        sample.rate_setpoint_yaw = 0.0
        sample.rate_err_roll = -0.3
        sample.rate_err_pitch = 0.3
        sample.rate_pid_p_roll = -0.00021
        sample.rate_pid_p_pitch = 0.00021
        sample.pid_out_roll = -0.00021
        sample.pid_out_pitch = 0.00021
        sample.pid_out_yaw = 0.0
        sample.motor1 = 0.10 - 0.00021 - 0.00021
        sample.motor2 = 0.10 + 0.00021 - 0.00021
        sample.motor3 = 0.10 + 0.00021 + 0.00021
        sample.motor4 = 0.10 - 0.00021 + 0.00021
        sample.rate_meas_yaw_filtered = 1.0
        sample.baro_valid = 1
        sample.baro_altitude_m = 0.01 * index
        samples.append(sample)

    result = analyze_liftoff_verify_samples(samples, base_duty=0.10)

    assert result["raw_inner_motor_clamp_max"] == 1
    assert result["raw_motor_saturation_max"] == 1
    assert result["startup_motor_clamp_count"] == 1
    assert result["inner_motor_clamp_max"] == 0
    assert result["motor_saturation_max"] == 0
    assert result["control_safe_pass"] is True
    assert result["physical_liftoff_state"] == "near liftoff / unloading"
    assert result["physical_liftoff_confirmed"] is False
    assert result["free_flight_pass"] is False
    assert result["probable_liftoff"] is False
    assert result["passed"] is True


def test_liftoff_verify_analysis_does_not_confirm_small_baro_delta_only():
    samples = []
    for index in range(8):
        sample = TelemetrySample.from_payload(build_telemetry_payload_v5())
        sample.timestamp_us = 2_500_000 + index * 20_000
        sample.control_mode = 6
        sample.control_submode = 2
        sample.kalman_valid = 1
        sample.attitude_valid = 1
        sample.ground_ref_valid = 1
        sample.battery_valid = 1
        sample.failsafe_reason = 0
        sample.ground_trip_reason = 0
        sample.outer_loop_clamp_flag = 0
        sample.inner_loop_clamp_flag = 0
        sample.motor_saturation_flag = 0
        sample.base_duty_active = 0.12
        sample.angle_target_roll = 0.0
        sample.angle_target_pitch = 0.0
        sample.angle_target_yaw = 0.0
        sample.angle_measured_roll = 0.5
        sample.angle_measured_pitch = -0.5
        sample.angle_error_roll = -0.5
        sample.angle_error_pitch = 0.5
        sample.angle_error_yaw = 0.0
        sample.outer_loop_rate_target_roll = -0.4
        sample.outer_loop_rate_target_pitch = 0.4
        sample.outer_loop_rate_target_yaw = 0.0
        sample.rate_setpoint_roll = -0.4
        sample.rate_setpoint_pitch = 0.4
        sample.rate_setpoint_yaw = 0.0
        sample.rate_err_roll = -0.3
        sample.rate_err_pitch = 0.3
        sample.rate_err_yaw = 0.0
        sample.rate_pid_p_roll = -0.00021
        sample.rate_pid_p_pitch = 0.00021
        sample.pid_out_roll = -0.00021
        sample.pid_out_pitch = 0.00021
        sample.pid_out_yaw = 0.0
        sample.motor1 = 0.12 - 0.00021 - 0.00021
        sample.motor2 = 0.12 + 0.00021 - 0.00021
        sample.motor3 = 0.12 + 0.00021 + 0.00021
        sample.motor4 = 0.12 - 0.00021 + 0.00021
        sample.rate_meas_yaw_filtered = 1.0
        sample.baro_valid = 1
        sample.baro_altitude_m = 0.015 * index
        samples.append(sample)

    result = analyze_liftoff_verify_samples(samples, base_duty=0.12)

    assert result["baro_altitude_delta_m"] == pytest.approx(0.105)
    assert result["physical_liftoff_state"] == "near liftoff / unloading"
    assert result["physical_liftoff_confirmed"] is False
    assert result["control_safe_pass"] is True


def test_liftoff_verify_analysis_separates_no_liftoff_from_control_safe_pass():
    samples = []
    for index in range(4):
        sample = TelemetrySample.from_payload(build_telemetry_payload_v5())
        sample.timestamp_us = 3_000_000 + index * 20_000
        sample.control_mode = 6
        sample.control_submode = 2
        sample.kalman_valid = 1
        sample.attitude_valid = 1
        sample.ground_ref_valid = 1
        sample.battery_valid = 1
        sample.failsafe_reason = 0
        sample.ground_trip_reason = 0
        sample.outer_loop_clamp_flag = 0
        sample.inner_loop_clamp_flag = 0
        sample.motor_saturation_flag = 0
        sample.base_duty_active = 0.10
        sample.angle_target_roll = 0.0
        sample.angle_target_pitch = 0.0
        sample.angle_target_yaw = 0.0
        sample.angle_measured_roll = 0.2
        sample.angle_measured_pitch = -0.2
        sample.angle_error_roll = 0.0
        sample.angle_error_pitch = 0.0
        sample.angle_error_yaw = 0.0
        sample.outer_loop_rate_target_roll = 0.0
        sample.outer_loop_rate_target_pitch = 0.0
        sample.outer_loop_rate_target_yaw = 0.0
        sample.rate_setpoint_roll = 0.0
        sample.rate_setpoint_pitch = 0.0
        sample.rate_setpoint_yaw = 0.0
        sample.rate_err_roll = 0.0
        sample.rate_err_pitch = 0.0
        sample.rate_err_yaw = 0.0
        sample.pid_out_roll = 0.0
        sample.pid_out_pitch = 0.0
        sample.pid_out_yaw = 0.0
        sample.motor1 = 0.10
        sample.motor2 = 0.10
        sample.motor3 = 0.10
        sample.motor4 = 0.10
        sample.rate_meas_yaw_filtered = 0.5
        sample.baro_valid = 1
        sample.baro_altitude_m = 0.003 * index
        samples.append(sample)

    result = analyze_liftoff_verify_samples(samples, base_duty=0.10)

    assert result["control_safe_pass"] is True
    assert result["physical_liftoff_state"] == "no liftoff"
    assert result["physical_liftoff_confirmed"] is False
    assert result["free_flight_pass"] is False
    assert result["passed"] is True


def test_liftoff_verify_analysis_allows_one_coherent_path_snapshot_transient():
    samples = []
    for index in range(60):
        sample = TelemetrySample.from_payload(build_telemetry_payload_v5())
        sample.timestamp_us = 4_000_000 + index * 20_000
        sample.control_mode = 6
        sample.control_submode = 2
        sample.kalman_valid = 1
        sample.attitude_valid = 1
        sample.ground_ref_valid = 1
        sample.battery_valid = 1
        sample.failsafe_reason = 0
        sample.ground_trip_reason = 0
        sample.outer_loop_clamp_flag = 0
        sample.inner_loop_clamp_flag = 0
        sample.motor_saturation_flag = 0
        sample.base_duty_active = 0.105
        sample.angle_target_roll = 0.0
        sample.angle_target_pitch = 0.0
        sample.angle_target_yaw = 0.0
        sample.angle_measured_roll = 0.5
        sample.angle_measured_pitch = -0.5
        sample.angle_error_roll = -0.5
        sample.angle_error_pitch = 0.5
        sample.angle_error_yaw = 0.0
        sample.outer_loop_rate_target_roll = -0.4
        sample.outer_loop_rate_target_pitch = 0.4
        sample.outer_loop_rate_target_yaw = 0.0
        sample.rate_setpoint_roll = -0.4
        sample.rate_setpoint_pitch = 0.4
        sample.rate_setpoint_yaw = 0.0
        sample.rate_err_roll = -0.3
        sample.rate_err_pitch = 0.3
        sample.rate_err_yaw = 0.0
        sample.rate_pid_p_roll = -0.00021
        sample.rate_pid_p_pitch = 0.00021
        sample.pid_out_roll = -0.00021
        sample.pid_out_pitch = 0.00021
        sample.pid_out_yaw = 0.0
        sample.motor1 = 0.105 - 0.00021 - 0.00021
        sample.motor2 = 0.105 + 0.00021 - 0.00021
        sample.motor3 = 0.105 + 0.00021 + 0.00021
        sample.motor4 = 0.105 - 0.00021 + 0.00021
        sample.rate_meas_yaw_filtered = 1.0
        sample.baro_valid = 1
        sample.baro_altitude_m = 0.001 * index
        if index == 30:
            sample.rate_setpoint_pitch = 0.22
        samples.append(sample)

    result = analyze_liftoff_verify_samples(samples, base_duty=0.105)

    assert result["unified_path_ratio"] == pytest.approx(59 / 60)
    assert result["unified_path_max_abs_error_dps"] == pytest.approx(0.18)
    assert result["unified_path_ok"] is True
    assert result["control_safe_pass"] is True
    assert result["physical_liftoff_state"] == "near liftoff / unloading"


def test_gui_startup_without_device_or_missing_pyqt5(monkeypatch):
    if importlib.util.find_spec("PyQt5") is None or importlib.util.find_spec("pyqtgraph") is None:
        from esp_drone_cli import gui_main

        assert gui_main.main([]) == 1
        return

    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PyQt5.QtWidgets import QApplication

    from esp_drone_cli.gui.main_window import MainWindow

    app = QApplication.instance() or QApplication([])
    window = MainWindow()
    window.close()
    app.quit()


@pytest.mark.skipif(importlib.util.find_spec("PyQt5") is None or importlib.util.find_spec("pyqtgraph") is None, reason="PyQt5/pyqtgraph not installed")
def test_gui_actions_route_through_device_session(monkeypatch, tmp_path: Path):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5.QtCore import QSettings, Qt
    from PyQt5.QtWidgets import QApplication, QFileDialog, QGroupBox, QLabel, QPushButton, QSizePolicy

    from esp_drone_cli.gui.main_window import MainWindow, QtSessionBridge

    class SyncBridge(QtSessionBridge):
        def run_async(self, label: str, callback) -> None:
            try:
                result = callback()
                self.command_finished.emit(label, result)
            except Exception as exc:  # pragma: no cover - test should not hit
                self.error_raised.emit(f"{label}: {exc}")

    export_path = tmp_path / "export.json"
    import_path = tmp_path / "import.json"
    import_path.write_text(json.dumps({"params": [{"name": "alpha", "type_id": 2, "value": 99}]}), encoding="utf-8")

    monkeypatch.setattr(QFileDialog, "getSaveFileName", staticmethod(lambda *args, **kwargs: (str(export_path), "JSON Files (*.json)")))
    monkeypatch.setattr(QFileDialog, "getOpenFileName", staticmethod(lambda *args, **kwargs: (str(import_path), "JSON Files (*.json)")))

    app = QApplication.instance() or QApplication([])
    session = FakeSession()
    settings = QSettings(str(tmp_path / "gui.ini"), QSettings.IniFormat)
    window = MainWindow(
        session=session,
        bridge_cls=SyncBridge,
        serial_port_provider=lambda: ["COM9"],
        settings=settings,
    )

    window.serial_port_combo.setCurrentText("COM9")
    window.connect_button.click()
    app.processEvents()
    assert ("connect_serial", ("COM9", 115200, 0.05, 0.75), {}) in session.calls
    assert window.connection_status_chip.text() == window._t("status.connected")
    assert window.right_tabs.currentIndex() == 0
    assert window.right_tabs.tabText(0) == window._t("tab.params")
    assert window.right_tabs.tabText(1) == window._t("tab.events")
    assert window.right_tabs.tabText(2) == window._t("tab.tools")
    assert any(window.chart_group_combo.itemData(i) == "baro" for i in range(window.chart_group_combo.count()))
    assert window.left_panel.minimumWidth() >= 320
    assert window.center_panel.minimumWidth() >= 780
    assert window.right_panel.minimumWidth() >= 360
    assert window.left_panel.sizePolicy().horizontalPolicy() != QSizePolicy.Fixed
    assert window.left_panel.verticalScrollBarPolicy() == Qt.ScrollBarAsNeeded
    assert window.connection_section.is_expanded() is True
    assert window.safety_section.is_expanded() is True
    assert window.debug_action_tabs.count() == 4
    assert window.debug_action_tabs.tabText(0) == window._t("tab.motor")
    assert window.debug_action_tabs.tabText(1) == window._t("tab.rate")
    assert window.debug_action_tabs.tabText(2) == window._t("tab.hang_attitude")
    assert window.debug_action_tabs.tabText(3) == window._t("tab.udp_control")
    assert window.debug_action_tabs.currentIndex() == 0
    assert window.params_table.rowCount() == 8
    window.link_type_combo.setCurrentIndex(window.link_type_combo.findData("udp"))
    app.processEvents()
    assert window.udp_mode_combo.currentData() == "softap"
    assert window.udp_host_edit.text() == "192.168.4.1"
    assert window.udp_port_spin.value() == 2391
    assert "ESP-DRONE" in window.udp_ap_info_label.text()
    window.udp_mode_combo.setCurrentIndex(window.udp_mode_combo.findData("sta"))
    window.udp_host_edit.setText("192.168.50.42")
    assert window.udp_mode_combo.currentData() == "sta"
    assert window.udp_host_edit.text() == "192.168.50.42"
    window.link_type_combo.setCurrentIndex(window.link_type_combo.findData("serial"))
    app.processEvents()
    session.calls.clear()

    sample = TelemetrySample.from_payload(build_telemetry_payload())
    session.emit_telemetry(sample)
    session.emit_event("imu healthy")
    app.processEvents()
    assert window.telemetry_table.item(0, 1).text() == "1.000"
    assert window.status_cards["arm_state"][1].text() == window._t("arm.disarmed")
    assert window.status_cards["baro_health"][1].text() == window._t("baro.ok")
    assert window.status_cards["baro_altitude_m"][1].text().endswith("m")
    assert "imu healthy" in window.event_log_edit.toPlainText()
    localized_texts = []
    for widget in window.findChildren((QPushButton, QLabel, QGroupBox)):
        text = widget.title() if isinstance(widget, QGroupBox) else widget.text()
        text = text.strip()
        if text:
            localized_texts.append(text)
    localized_texts.extend(
        [
            window.param_search_edit.placeholderText(),
            window.param_new_value_edit.placeholderText(),
            window.event_log_edit.placeholderText(),
            window.calib_gyro_button.toolTip(),
            window.calib_level_button.toolTip(),
            *(window.right_tabs.tabText(i) for i in range(window.right_tabs.count())),
            *(window.debug_action_tabs.tabText(i) for i in range(window.debug_action_tabs.count())),
        ]
    )
    raw_tokens = [
        text
        for text in localized_texts
        if text.startswith(("button.", "group.", "label.", "placeholder.", "tab."))
    ]
    assert raw_tokens == []

    window.stream_on_button.click()
    window.stream_off_button.click()
    window.stream_rate_spin.setValue(150)
    window.apply_stream_rate_button.click()
    window.refresh_params_button.click()
    window.arm_button.click()
    window.kill_button.click()
    window.disarm_button.click()
    window.reboot_button.click()

    window.params_table.selectRow(0)
    app.processEvents()
    window.param_new_value_edit.setText("77")
    window.set_param_button.click()
    window.save_params_button.click()
    window.reset_params_button.click()
    window.export_params_button.click()
    window.import_params_button.click()

    window.motor_combo.setCurrentIndex(2)
    window.motor_duty_spin.setValue(0.12)
    window.motor_start_button.click()
    window.motor_stop_button.click()
    window.calib_gyro_button.click()
    window.calib_level_button.click()

    window.rate_axis_combo.setCurrentIndex(1)
    window.rate_value_spin.setValue(35.0)
    window.rate_start_button.click()
    window.rate_stop_button.click()

    window.udp_max_pwm_spin.setValue(11.0)
    window.udp_enable_button.click()
    window.udp_forward_button.click()
    window.udp_yaw_right_button.click()
    window.udp_up_button.click()
    window.udp_takeoff_button.click()
    window.udp_land_button.click()
    window.udp_stop_button.click()
    window.udp_disable_button.click()

    log_path = tmp_path / "telemetry.csv"
    window.log_path_edit.setText(str(log_path))
    window.start_log_button.click()
    window.stop_log_button.click()
    window.dump_duration_spin.setValue(2.5)
    window.dump_csv_button.click()

    window.disconnect_button.click()
    app.processEvents()
    window.link_type_combo.setCurrentIndex(window.link_type_combo.findData("udp"))
    window.udp_host_edit.setText("192.168.4.1")
    window.udp_port_spin.setValue(2391)
    window.connect_button.click()
    window.disconnect_button.click()
    app.processEvents()

    call_names = [name for name, _args, _kwargs in session.calls]
    assert "start_stream" in call_names
    assert "stop_stream" in call_names
    assert "arm" in call_names
    assert "kill" in call_names
    assert "disarm" in call_names
    assert "reboot" in call_names
    assert "set_param" in call_names
    assert ("set_param", ("telemetry_usb_hz", 2, "150"), {}) in session.calls
    assert "list_params" in call_names
    assert "save_params" in call_names
    assert "reset_params" in call_names
    assert "export_params" in call_names
    assert "import_params" in call_names
    assert ("motor_test", (2, 0.12), {}) in session.calls
    assert ("motor_test", (2, 0.0), {}) in session.calls
    assert ("rate_test", (1, 35.0), {}) in session.calls
    assert ("rate_test", (1, 0.0), {}) in session.calls
    assert "udp_manual_enable" in call_names
    assert "udp_manual_disable" in call_names
    assert "udp_manual_stop" in call_names
    assert "udp_takeoff" in call_names
    assert "udp_land" in call_names
    assert "udp_manual_setpoint" in call_names
    assert ("set_param", ("udp_manual_max_pwm", 4, 0.11), {}) in session.calls
    assert ("set_param", ("udp_manual_timeout_ms", 2, 1000), {}) in session.calls
    assert "start_csv_log" in call_names
    assert "stop_csv_log" in call_names
    assert "dump_csv" in call_names
    assert ("connect_udp", ("192.168.4.1", 2391, 1.0), {}) in session.calls
    assert "disconnect" in call_names
    assert window.last_log_path_label.text().endswith("telemetry.csv")

    window.close()
    app.processEvents()


@pytest.mark.skipif(importlib.util.find_spec("PyQt5") is None or importlib.util.find_spec("pyqtgraph") is None, reason="PyQt5/pyqtgraph not installed")
def test_gui_ground_capture_ref_only_sends_command_without_applying_params(monkeypatch, tmp_path: Path):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5.QtCore import QSettings
    from PyQt5.QtWidgets import QApplication

    from esp_drone_cli.gui.main_window import MainWindow, QtSessionBridge

    class SyncBridge(QtSessionBridge):
        def run_async(self, label: str, callback) -> None:
            result = callback()
            self.command_finished.emit(label, result)

    app = QApplication.instance() or QApplication([])
    session = FakeSession()
    settings = QSettings(str(tmp_path / "gui-ground-capture.ini"), QSettings.IniFormat)
    window = MainWindow(session=session, bridge_cls=SyncBridge, serial_port_provider=lambda: ["COM9"], settings=settings)

    window.serial_port_combo.setCurrentText("COM9")
    window.connect_button.click()
    app.processEvents()
    session.calls.clear()
    window.ground_capture_button.click()
    app.processEvents()

    call_names = [name for name, _args, _kwargs in session.calls]
    assert call_names == ["ground_capture_ref"]
    assert "set_param" not in call_names
    assert "get_param" not in call_names
    assert "list_params" not in call_names

    window.close()
    app.processEvents()


@pytest.mark.skipif(importlib.util.find_spec("PyQt5") is None or importlib.util.find_spec("pyqtgraph") is None, reason="PyQt5/pyqtgraph not installed")
def test_gui_ground_record_only_dumps_csv_without_applying_params(monkeypatch, tmp_path: Path):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5.QtCore import QSettings
    from PyQt5.QtWidgets import QApplication

    from esp_drone_cli.gui.main_window import MainWindow, QtSessionBridge

    class SyncBridge(QtSessionBridge):
        def run_async(self, label: str, callback) -> None:
            result = callback()
            self.command_finished.emit(label, result)

    app = QApplication.instance() or QApplication([])
    session = FakeSession()
    settings = QSettings(str(tmp_path / "gui-ground-record.ini"), QSettings.IniFormat)
    window = MainWindow(session=session, bridge_cls=SyncBridge, serial_port_provider=lambda: ["COM9"], settings=settings)

    window.serial_port_combo.setCurrentText("COM9")
    window.connect_button.click()
    app.processEvents()
    session.calls.clear()
    window.ground_record_10_button.click()
    app.processEvents()

    call_names = [name for name, _args, _kwargs in session.calls]
    assert call_names == ["dump_csv"]
    assert "set_param" not in call_names

    window.close()
    app.processEvents()


@pytest.mark.skipif(importlib.util.find_spec("PyQt5") is None or importlib.util.find_spec("pyqtgraph") is None, reason="PyQt5/pyqtgraph not installed")
def test_gui_attitude_ground_and_liftoff_buttons_route_through_session(monkeypatch, tmp_path: Path):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    monkeypatch.chdir(tmp_path)

    from PyQt5.QtCore import QSettings
    from PyQt5.QtWidgets import QApplication

    from esp_drone_cli.gui.main_window import MainWindow, QtSessionBridge

    class SyncBridge(QtSessionBridge):
        def run_async(self, label: str, callback) -> None:
            result = callback()
            self.command_finished.emit(label, result)

    app = QApplication.instance() or QApplication([])
    session = FakeSession()
    settings = QSettings(str(tmp_path / "gui-att-ground.ini"), QSettings.IniFormat)
    window = MainWindow(session=session, bridge_cls=SyncBridge, serial_port_provider=lambda: ["COM9"], settings=settings)

    window.serial_port_combo.setCurrentText("COM9")
    window.connect_button.click()
    app.processEvents()
    session.calls.clear()

    window.att_ground_start_button.click()
    window.att_ground_stop_button.click()
    window.att_ground_log_button.click()
    window.liftoff_verify_start_button.click()
    window.liftoff_verify_stop_button.click()
    app.processEvents()

    call_names = [name for name, _args, _kwargs in session.calls]
    assert "attitude_ground_verify_start" in call_names
    assert "attitude_ground_verify_stop" in call_names
    assert "liftoff_verify_start" in call_names
    assert "liftoff_verify_stop" in call_names
    assert any(name == "dump_csv" and args[0].name.startswith(time.strftime("%Y%m%d")) for name, args, _ in session.calls)

    window.close()
    app.processEvents()


@pytest.mark.skipif(importlib.util.find_spec("PyQt5") is None or importlib.util.find_spec("pyqtgraph") is None, reason="PyQt5/pyqtgraph not installed")
def test_gui_connect_failure_shows_error_and_restores_inputs(monkeypatch, tmp_path: Path):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5.QtCore import QSettings
    from PyQt5.QtWidgets import QApplication

    from esp_drone_cli.gui.main_window import MainWindow, QtSessionBridge

    class FailingSession(FakeSession):
        def connect_serial(
            self,
            port: str,
            baudrate: int = 115200,
            timeout: float = 0.2,
            open_retry_timeout_s: float = 5.0,
        ):
            self._record("connect_serial", port, baudrate, timeout, open_retry_timeout_s)
            raise TimeoutError("HELLO timeout")

    class SyncBridge(QtSessionBridge):
        def run_async(self, label: str, callback) -> None:
            try:
                result = callback()
                self.command_finished.emit(label, result)
            except Exception as exc:
                if label in {"connect_serial", "connect_udp"}:
                    self.error_raised.emit(f"Connect failed: {exc}")
                else:
                    self.error_raised.emit(f"{label}: {exc}")

    app = QApplication.instance() or QApplication([])
    session = FailingSession()
    settings = QSettings(str(tmp_path / "gui-fail.ini"), QSettings.IniFormat)
    window = MainWindow(
        session=session,
        bridge_cls=SyncBridge,
        serial_port_provider=lambda: ["COM404"],
        settings=settings,
    )

    window.serial_port_combo.setCurrentText("COM404")
    window.connect_button.click()
    app.processEvents()

    assert ("connect_serial", ("COM404", 115200, 0.05, 0.75), {}) in session.calls
    assert window.connection_status_chip.text() == window._t("status.disconnected")
    assert window.connect_button.isEnabled()
    assert window.serial_port_combo.currentText() == "COM404"
    assert "Connect failed: HELLO timeout" in window.connection_error_detail.text()
    assert "Connect failed: HELLO timeout" in window.event_log_edit.toPlainText()

    window.close()
    app.processEvents()


@pytest.mark.skipif(importlib.util.find_spec("PyQt5") is None or importlib.util.find_spec("pyqtgraph") is None, reason="PyQt5/pyqtgraph not installed")
def test_gui_udp_empty_host_fails_without_worker(monkeypatch, tmp_path: Path):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5.QtCore import QSettings
    from PyQt5.QtWidgets import QApplication

    from esp_drone_cli.gui.main_window import MainWindow

    app = QApplication.instance() or QApplication([])
    session = FakeSession()
    settings = QSettings(str(tmp_path / "gui-empty-udp-host.ini"), QSettings.IniFormat)
    window = MainWindow(session=session, serial_port_provider=lambda: [], settings=settings)

    window.link_type_combo.setCurrentIndex(window.link_type_combo.findData("udp"))
    window.udp_host_edit.setText("")
    window.connect_button.click()
    app.processEvents()

    assert not any(name == "connect_udp" for name, _args, _kwargs in session.calls)
    assert window.connection_status_chip.text() == window._t("status.disconnected")
    assert window.connect_button.isEnabled()
    assert window._t("msg.udp_host_required") in window.connection_error_detail.text()

    window.close()
    app.processEvents()


@pytest.mark.skipif(importlib.util.find_spec("PyQt5") is None or importlib.util.find_spec("pyqtgraph") is None, reason="PyQt5/pyqtgraph not installed")
def test_gui_sta_udp_mode_connects_selected_drone_ip(monkeypatch, tmp_path: Path):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5.QtCore import QSettings
    from PyQt5.QtWidgets import QApplication

    from esp_drone_cli.gui.main_window import MainWindow, QtSessionBridge

    class SyncBridge(QtSessionBridge):
        def run_async(self, label: str, callback) -> None:
            try:
                result = callback()
                self.command_finished.emit(label, result)
            except Exception as exc:  # pragma: no cover - test should not hit
                self.error_raised.emit(f"{label}: {exc}")

    app = QApplication.instance() or QApplication([])
    session = FakeSession()
    settings = QSettings(str(tmp_path / "gui-sta-udp.ini"), QSettings.IniFormat)
    window = MainWindow(session=session, bridge_cls=SyncBridge, serial_port_provider=lambda: [], settings=settings)

    window.link_type_combo.setCurrentIndex(window.link_type_combo.findData("udp"))
    window.udp_mode_combo.setCurrentIndex(window.udp_mode_combo.findData("sta"))
    window.udp_host_edit.setText("192.168.50.42")
    window.udp_port_spin.setValue(2391)
    window.connect_button.click()
    app.processEvents()

    assert ("connect_udp", ("192.168.50.42", 2391, 1.0), {}) in session.calls
    assert "udp sta 192.168.50.42:2391" in window.connection_info_label.text()

    window.close()
    app.processEvents()


@pytest.mark.skipif(importlib.util.find_spec("PyQt5") is None or importlib.util.find_spec("pyqtgraph") is None, reason="PyQt5/pyqtgraph not installed")
def test_gui_connect_watchdog_restores_ui_when_worker_never_returns(monkeypatch, tmp_path: Path):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5.QtCore import QSettings
    from PyQt5.QtWidgets import QApplication

    from esp_drone_cli.gui.main_window import MainWindow, QtSessionBridge

    class HangingBridge(QtSessionBridge):
        def run_async(self, label: str, callback) -> None:
            self._session._record("bridge_hang", label)

    app = QApplication.instance() or QApplication([])
    session = FakeSession()
    settings = QSettings(str(tmp_path / "gui-watchdog.ini"), QSettings.IniFormat)
    window = MainWindow(
        session=session,
        bridge_cls=HangingBridge,
        serial_port_provider=lambda: ["COM55"],
        settings=settings,
    )

    window.serial_port_combo.setCurrentText("COM55")
    window.connect_button.click()
    app.processEvents()
    assert not window.connect_button.isEnabled()

    window._handle_connect_watchdog_timeout()
    app.processEvents()

    assert window.connection_status_chip.text() == window._t("status.disconnected")
    assert window.connect_button.isEnabled()
    assert window.serial_port_combo.currentText() == "COM55"
    assert window._t("msg.connect_failed", error="connection attempt timed out in GUI") in window.connection_error_detail.text()
    assert window._t("msg.connect_failed", error="connection attempt timed out in GUI") in window.event_log_edit.toPlainText()

    window.close()
    app.processEvents()


@pytest.mark.skipif(importlib.util.find_spec("PyQt5") is None or importlib.util.find_spec("pyqtgraph") is None, reason="PyQt5/pyqtgraph not installed")
def test_gui_close_disconnects_session(monkeypatch, tmp_path: Path):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PyQt5.QtCore import QSettings
    from PyQt5.QtWidgets import QApplication

    from esp_drone_cli.gui.main_window import MainWindow, QtSessionBridge

    class SyncBridge(QtSessionBridge):
        def run_async(self, label: str, callback) -> None:
            result = callback()
            self.command_finished.emit(label, result)

    app = QApplication.instance() or QApplication([])
    session = FakeSession()
    settings = QSettings(str(tmp_path / "gui-close.ini"), QSettings.IniFormat)
    window = MainWindow(session=session, bridge_cls=SyncBridge, serial_port_provider=lambda: [], settings=settings)
    window.close()
    app.processEvents()

    assert any(name == "disconnect" for name, _args, _kwargs in session.calls)


def test_cli_parser_compatibility_without_gui_dependency():
    from esp_drone_cli.cli.main import build_parser

    args = build_parser().parse_args(["--serial", "COM7", "rate-test", "yaw", "30"])
    assert args.serial == "COM7"
    assert args.command == "rate-test"
    assert args.axis == "yaw"

    baro_args = build_parser().parse_args(["--serial", "COM7", "watch-baro", "--timeout", "1"])
    assert baro_args.command == "watch-baro"

    attitude_args = build_parser().parse_args(["--serial", "COM7", "attitude-test", "start", "--base-duty", "0.05"])
    assert attitude_args.command == "attitude-test"
    assert attitude_args.action == "start"
    assert attitude_args.base_duty == pytest.approx(0.05)

    capability_args = build_parser().parse_args(["--serial", "COM7", "capabilities"])
    assert capability_args.command == "capabilities"

    udp_args = build_parser().parse_args(
        ["--udp", "192.168.4.1:2391", "udp-manual", "setpoint", "--throttle", "0.08", "--pitch", "-0.02"]
    )
    assert udp_args.command == "udp-manual"
    assert udp_args.action == "setpoint"
    assert udp_args.throttle == pytest.approx(0.08)
    assert udp_args.pitch == pytest.approx(-0.02)

    udp_default_args = build_parser().parse_args(["--udp", "connect"])
    assert udp_default_args.udp == "192.168.4.1:2391"

    sta_args = build_parser().parse_args(["--host", "192.168.50.42", "--udp-port", "2391", "connect"])
    assert sta_args.drone_ip == "192.168.50.42"
    assert sta_args.udp_port == 2391

    ground_log_args = build_parser().parse_args(["--serial", "COM7", "ground-log", "--duration", "2"])
    assert ground_log_args.command == "ground-log"
    assert ground_log_args.duration == pytest.approx(2.0)

    attitude_ground_start_args = build_parser().parse_args(
        ["--serial", "COM7", "attitude-ground-verify", "start", "--base-duty", "0.08"]
    )
    assert attitude_ground_start_args.command == "attitude-ground-verify"
    assert attitude_ground_start_args.action == "start"
    assert attitude_ground_start_args.base_duty == pytest.approx(0.08)

    attitude_ground_target_args = build_parser().parse_args(
        ["--serial", "COM7", "attitude-ground-verify", "target", "roll", "1.5"]
    )
    assert attitude_ground_target_args.command == "attitude-ground-verify"
    assert attitude_ground_target_args.action == "target"
    assert attitude_ground_target_args.axis == "roll"
    assert attitude_ground_target_args.deg == pytest.approx(1.5)

    attitude_ground_log_args = build_parser().parse_args(["--serial", "COM7", "attitude-ground-log", "--duration", "2"])
    assert attitude_ground_log_args.command == "attitude-ground-log"
    assert attitude_ground_log_args.duration == pytest.approx(2.0)

    attitude_ground_round_args = build_parser().parse_args(
        ["--serial", "COM7", "attitude-ground-round", "--target-deg", "1.0", "--auto-arm"]
    )
    assert attitude_ground_round_args.command == "attitude-ground-round"
    assert attitude_ground_round_args.target_deg == pytest.approx(1.0)
    assert attitude_ground_round_args.auto_arm is True

    liftoff_args = build_parser().parse_args(["--serial", "COM7", "liftoff-verify", "start", "--base-duty", "0.10"])
    assert liftoff_args.command == "liftoff-verify"
    assert liftoff_args.action == "start"
    assert liftoff_args.base_duty == pytest.approx(0.10)

    liftoff_round_args = build_parser().parse_args(
        ["--serial", "COM7", "liftoff-round", "--base-duty", "0.10", "--duration-s", "2.0"]
    )
    assert liftoff_round_args.command == "liftoff-round"
    assert liftoff_round_args.base_duty == pytest.approx(0.10)
    assert liftoff_round_args.duration_s == pytest.approx(2.0)

    liftoff_auto_args = build_parser().parse_args(["--serial", "COM7", "liftoff-auto16", "--attempts", "3"])
    assert liftoff_auto_args.command == "liftoff-auto16"
    assert liftoff_auto_args.attempts == 3
    assert liftoff_auto_args.ready_timeout_s == pytest.approx(12.0)
    assert liftoff_auto_args.ready_hold_s == pytest.approx(0.40)
    assert "16%" in build_parser().format_help()

    liftoff_near_args = build_parser().parse_args(
        ["--serial", "COM7", "liftoff-near-threshold", "--attempts-per-duty", "3"]
    )
    assert liftoff_near_args.command == "liftoff-near-threshold"
    assert liftoff_near_args.attempts_per_duty == 3
    assert liftoff_near_args.ready_timeout_s == pytest.approx(12.0)
    assert "22/24/25%" in build_parser().format_help()

    liftoff_near_default_args = build_parser().parse_args(["--serial", "COM7", "liftoff-near-threshold"])
    assert liftoff_near_default_args.attempts_per_duty == 2

    all_motor_args = build_parser().parse_args(
        ["--serial", "COM7", "all-motor-test", "--duty", "0.30", "--duration-s", "2.0"]
    )
    assert all_motor_args.command == "all-motor-test"
    assert all_motor_args.duty == pytest.approx(0.30)
    assert all_motor_args.duration_s == pytest.approx(2.0)
    assert all_motor_args.no_auto_arm is False
    all_motor_no_auto_args = build_parser().parse_args(
        ["--serial", "COM7", "all-motor-test", "--duty", "0.30", "--no-auto-arm"]
    )
    assert all_motor_no_auto_args.no_auto_arm is True

    motor_balance_args = build_parser().parse_args(["--serial", "COM7", "motor-thrust-balance"])
    assert motor_balance_args.command == "motor-thrust-balance"
    assert motor_balance_args.duties == "0.20,0.25,0.30,0.35"
    assert motor_balance_args.duration_s == pytest.approx(0.9)
    assert motor_balance_args.settle_s == pytest.approx(0.3)
    assert motor_balance_args.rest_s == pytest.approx(3.0)
    assert motor_balance_args.no_trim is False
    motor_balance_no_trim_args = build_parser().parse_args(
        ["--serial", "COM7", "motor-thrust-balance", "--no-trim", "--settle-s", "0"]
    )
    assert motor_balance_no_trim_args.no_trim is True
    assert motor_balance_no_trim_args.settle_s == pytest.approx(0.0)

    vibration_args = build_parser().parse_args(
        ["analyze-vibration-log", "diag.csv", "--mode", "all-motor", "--duty", "0.05"]
    )
    assert vibration_args.command == "analyze-vibration-log"
    assert vibration_args.csv == "diag.csv"
    assert vibration_args.mode == "all-motor"
    assert vibration_args.duty == pytest.approx(0.05)

    motor_trim_args = build_parser().parse_args(["--serial", "COM7", "motor-trim-estimate", "summary.csv"])
    assert motor_trim_args.command == "motor-trim-estimate"
    assert motor_trim_args.summary_csv == "summary.csv"
    assert motor_trim_args.max_adjust == pytest.approx(0.10)

    short_hop_profile_args = build_parser().parse_args(["--serial", "COM7", "apply-short-hop-tuned-profile"])
    assert short_hop_profile_args.command == "apply-short-hop-tuned-profile"


def test_cli_connect_session_uses_sta_drone_ip(monkeypatch):
    from esp_drone_cli.cli import main as cli_main

    class StubSession:
        def __init__(self) -> None:
            self.calls: list[tuple[str, tuple, dict]] = []

        def connect_udp(self, host: str, port: int = 2391, timeout: float = 1.0):
            self.calls.append(("connect_udp", (host, port, timeout), {}))
            return DeviceInfo(10, 1, 0, 0, 0)

    session = StubSession()
    monkeypatch.setattr(cli_main, "DeviceSession", lambda: session)
    args = SimpleNamespace(serial=None, baudrate=115200, udp=None, drone_ip="192.168.50.42", udp_port=2391)

    assert cli_main.connect_session_from_args(args) is session
    assert session.calls == [("connect_udp", ("192.168.50.42", 2391, 1.0), {})]


def test_cli_connect_session_reports_sta_timeout(monkeypatch):
    from esp_drone_cli.cli import main as cli_main

    class TimeoutSession:
        def connect_udp(self, host: str, port: int = 2391, timeout: float = 1.0):
            raise TimeoutError("hello timeout")

    monkeypatch.setattr(cli_main, "DeviceSession", lambda: TimeoutSession())
    args = SimpleNamespace(serial=None, baudrate=115200, udp=None, drone_ip="192.168.50.42", udp_port=2391)

    with pytest.raises(RuntimeError) as excinfo:
        cli_main.connect_session_from_args(args)

    assert "STA/custom target 192.168.50.42:2391" in str(excinfo.value)
    assert "same LAN" in str(excinfo.value)


def test_cli_import_does_not_require_pyqt5(monkeypatch):
    real_import = builtins.__import__

    def guarded_import(name, globals=None, locals=None, fromlist=(), level=0):
        if name.startswith("PyQt5") or name.startswith("pyqtgraph"):
            raise ModuleNotFoundError(name)
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", guarded_import)
    sys.modules.pop("esp_drone_cli.cli.main", None)
    module = importlib.import_module("esp_drone_cli.cli.main")
    args = module.build_parser().parse_args(["--serial", "COM7", "connect"])
    assert args.command == "connect"


def test_cli_main_runs_without_pyqt5(monkeypatch):
    real_import = builtins.__import__

    def guarded_import(name, globals=None, locals=None, fromlist=(), level=0):
        if name.startswith("PyQt5") or name.startswith("pyqtgraph"):
            raise ModuleNotFoundError(name)
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", guarded_import)
    sys.modules.pop("esp_drone_cli.cli.main", None)
    module = importlib.import_module("esp_drone_cli.cli.main")
    session = FakeSession()
    monkeypatch.setattr(module, "connect_session_from_args", lambda args: session)

    assert module.main(["--serial", "COM7", "arm"]) == 0
    call_names = [name for name, _args, _kwargs in session.calls]
    assert "arm" in call_names
    assert "close" in call_names


def test_cli_baro_command_uses_device_session(monkeypatch, capsys):
    from esp_drone_cli.cli import main as cli_main

    session = FakeSession()
    session.device_info = type("DeviceInfoStub", (), {
        "protocol_version": 3,
        "imu_mode": 1,
        "arm_state": 0,
        "stream_enabled": 0,
        "feature_bitmap": 0x3F,
    })()
    monkeypatch.setattr(cli_main, "connect_session_from_args", lambda args: session)

    real_sleep = cli_main.time.sleep

    def fake_sleep(duration: float) -> None:
        sample = TelemetrySample.from_payload(build_telemetry_payload())
        session.emit_telemetry(sample)
        real_sleep(min(duration, 0.001))

    monkeypatch.setattr(cli_main.time, "sleep", fake_sleep)
    assert cli_main.main(["--serial", "COM7", "baro", "--timeout", "0.05"]) == 0
    output = capsys.readouterr().out
    assert "pressure_pa=" in output
    assert "altitude_m=" in output


def test_cli_rate_status_command_uses_device_session(monkeypatch, capsys):
    from esp_drone_cli.cli import main as cli_main

    session = FakeSession()
    session.device_info = type("DeviceInfoStub", (), {
        "protocol_version": 3,
        "imu_mode": 1,
        "arm_state": 1,
        "stream_enabled": 0,
        "feature_bitmap": 0x3F,
    })()
    monkeypatch.setattr(cli_main, "connect_session_from_args", lambda args: session)

    real_sleep = cli_main.time.sleep

    def fake_sleep(duration: float) -> None:
        session.emit_telemetry(TelemetrySample.from_payload(build_telemetry_payload()))
        real_sleep(min(duration, 0.001))

    monkeypatch.setattr(cli_main.time, "sleep", fake_sleep)
    assert cli_main.main(["--serial", "COM7", "rate-status", "roll", "--timeout", "0.05", "--interval", "0.01"]) == 0
    output = capsys.readouterr().out
    assert "roll rate_setpoint_roll=10.000" in output
    assert "roll_rate=-2.000" in output
    assert "source_expr=-gyro_y" in output
    assert "raw_gyro_y=2.000" in output


def test_cli_rate_test_returns_firmware_status_on_error(monkeypatch, capsys):
    from esp_drone_cli.cli import main as cli_main

    session = FakeSession()
    monkeypatch.setattr(cli_main, "connect_session_from_args", lambda args: session)
    monkeypatch.setattr(session, "rate_test", lambda axis_index, value_dps: 4)

    assert cli_main.main(["--serial", "COM7", "rate-test", "yaw", "30"]) == 4
    error_text = capsys.readouterr().err
    assert "rate-test failed: device must be armed first" in error_text


def test_cli_attitude_ground_and_liftoff_commands_use_device_session(monkeypatch, tmp_path: Path):
    from esp_drone_cli.cli import main as cli_main

    session = FakeSession()
    monkeypatch.setattr(cli_main, "connect_session_from_args", lambda args: session)

    assert cli_main.main(["--serial", "COM7", "attitude-ground-verify", "start", "--base-duty", "0.08"]) == 0
    assert cli_main.main(["--serial", "COM7", "attitude-ground-verify", "target", "pitch", "-1.25"]) == 0
    assert cli_main.main(["--serial", "COM7", "attitude-ground-verify", "stop"]) == 0
    assert cli_main.main(
        ["--serial", "COM7", "attitude-ground-log", "--duration", "0.01", "--output-dir", str(tmp_path)]
    ) == 0
    assert cli_main.main(["--serial", "COM7", "liftoff-verify", "start", "--base-duty", "0.10"]) == 0
    assert cli_main.main(["--serial", "COM7", "liftoff-verify", "stop"]) == 0

    assert ("attitude_ground_verify_start", (0.08,), {}) in session.calls
    assert ("attitude_ground_set_target", (1, -1.25), {}) in session.calls
    assert ("attitude_ground_verify_stop", (), {}) in session.calls
    assert ("liftoff_verify_start", (0.10,), {}) in session.calls
    assert ("liftoff_verify_stop", (), {}) in session.calls
    assert any(name == "dump_csv" and args[0].name.endswith("_attitude_ground_verify_log.csv") for name, args, _ in session.calls)


def test_cli_liftoff_round_records_and_stops_with_fake_session(monkeypatch, tmp_path: Path):
    from esp_drone_cli.cli import main as cli_main

    session = FakeSession()
    monkeypatch.setattr(cli_main, "connect_session_from_args", lambda args: session)

    rc = cli_main.main(
        [
            "--serial",
            "COM7",
            "liftoff-round",
            "--base-duty",
            "0.10",
            "--duration-s",
            "0.01",
            "--settle-s",
            "0",
            "--output-dir",
            str(tmp_path),
        ]
    )

    assert rc == 2
    call_names = [name for name, _args, _kwargs in session.calls]
    assert "ground_capture_ref" in call_names
    assert "arm" in call_names
    assert ("liftoff_verify_start", (0.10,), {}) in session.calls
    assert "start_stream" in call_names
    assert "liftoff_verify_stop" in call_names
    assert "disarm" in call_names
    assert any(name == "start_csv_log" and args[0].name.endswith("_liftoff_verify_round.csv") for name, args, _ in session.calls)


def test_cli_all_motor_test_records_stops_and_disarms_with_fake_session(monkeypatch, tmp_path: Path):
    from esp_drone_cli.cli import main as cli_main

    session = FakeSession()
    monkeypatch.setattr(cli_main, "connect_session_from_args", lambda args: session)
    monkeypatch.setattr(cli_main.time, "sleep", lambda _duration: None)

    rc = cli_main.main(
        [
            "--serial",
            "COM7",
            "all-motor-test",
            "--duty",
            "0.30",
            "--duration-s",
            "0.1",
            "--output-dir",
            str(tmp_path),
        ]
    )

    assert rc == 0
    assert ("require_all_motor_test", (), {}) in session.calls
    assert ("arm", (), {}) in session.calls
    assert ("get_latest_telemetry", (), {}) in session.calls
    assert ("all_motor_test_start", (0.30, 0.1), {}) in session.calls
    assert ("all_motor_test_stop", (), {}) in session.calls
    assert ("disarm", (), {}) in session.calls
    call_names = [name for name, _args, _kwargs in session.calls]
    assert call_names.index("arm") < call_names.index("get_latest_telemetry")
    assert call_names.index("get_latest_telemetry") < call_names.index("all_motor_test_start")
    assert call_names.index("all_motor_test_start") < call_names.index("all_motor_test_stop")
    assert call_names.index("all_motor_test_stop") < call_names.index("disarm")
    assert any(name == "start_csv_log" and args[0].name.endswith("_all_motor_test_30pct.csv") for name, args, _ in session.calls)


def test_cli_all_motor_test_fails_when_active_window_is_too_short(monkeypatch, tmp_path: Path, capsys):
    from esp_drone_cli.cli import main as cli_main

    class ShortActiveSession(FakeSession):
        def all_motor_test_start(self, duty: float, duration_s: float) -> int:
            self._record("all_motor_test_start", duty, duration_s)
            if self._arm_state != 1:
                return CmdStatus.ARM_REQUIRED
            self._set_runtime_state(control_mode=7)
            self.emit_event("all-motor test started duty=0.300 duration_ms=2000")
            self.emit_telemetry(self._telemetry_sample(0, duty, control_mode=7))
            self.emit_telemetry(self._telemetry_sample(135_000, duty, control_mode=7))
            return CmdStatus.OK

    session = ShortActiveSession()
    monkeypatch.setattr(cli_main, "connect_session_from_args", lambda args: session)
    monkeypatch.setattr(cli_main.time, "sleep", lambda _duration: None)

    rc = cli_main.main(
        [
            "--serial",
            "COM7",
            "all-motor-test",
            "--duty",
            "0.30",
            "--duration-s",
            "2.0",
            "--output-dir",
            str(tmp_path),
        ]
    )

    output = capsys.readouterr().out
    assert rc == 2
    assert "active_duration_s=0.135" in output
    assert "active_duration_ok=False" in output
    assert "active_battery_min_v=4.000" in output
    assert "duration_ticks_100ms=20" in output
    assert ("all_motor_test_stop", (), {}) in session.calls
    assert ("disarm", (), {}) in session.calls


def test_cli_all_motor_test_no_auto_arm_sends_start_without_arm_or_disarm(monkeypatch, tmp_path: Path, capsys):
    from esp_drone_cli.cli import main as cli_main

    session = FakeSession()
    monkeypatch.setattr(cli_main, "connect_session_from_args", lambda args: session)
    monkeypatch.setattr(cli_main.time, "sleep", lambda _duration: None)

    rc = cli_main.main(
        [
            "--serial",
            "COM7",
            "all-motor-test",
            "--duty",
            "0.30",
            "--duration-s",
            "0.1",
            "--output-dir",
            str(tmp_path),
            "--no-auto-arm",
        ]
    )

    assert rc == CmdStatus.ARM_REQUIRED
    call_names = [name for name, _args, _kwargs in session.calls]
    assert "arm" not in call_names
    assert "disarm" not in call_names
    assert "get_latest_telemetry" in call_names
    assert "all_motor_test_start" in call_names
    assert "all_motor_test_stop" in call_names
    assert call_names.index("all_motor_test_start") < call_names.index("get_latest_telemetry")
    assert call_names.index("all_motor_test_start") < call_names.index("all_motor_test_stop")
    assert "CMD_STATUS_ARM_REQUIRED" in capsys.readouterr().err


def test_cli_all_motor_test_auto_arm_timeout_when_device_never_arms(monkeypatch, tmp_path: Path, capsys):
    from esp_drone_cli.cli import main as cli_main

    class NeverArmedSession(FakeSession):
        def arm(self) -> int:
            self._record("arm")
            self._set_runtime_state(arm_state=0, failsafe_reason=1)
            return CmdStatus.OK

    session = NeverArmedSession()
    monkeypatch.setattr(cli_main, "connect_session_from_args", lambda args: session)

    fake_time = [1000.0]
    monkeypatch.setattr(cli_main.time, "sleep", lambda d: fake_time.__setitem__(0, fake_time[0] + d))
    monkeypatch.setattr(cli_main.time, "monotonic", lambda: fake_time[0])

    rc = cli_main.main(
        [
            "--serial", "COM7", "all-motor-test",
            "--duty", "0.30", "--duration-s", "0.1",
            "--output-dir", str(tmp_path),
        ]
    )

    assert rc == CmdStatus.ARM_REQUIRED
    err = capsys.readouterr().err
    assert "auto-arm did not reach ARMED" in err
    assert "NOT_SENT_WAIT_ARM_TIMEOUT" in err
    call_names = [name for name, _args, _kwargs in session.calls]
    assert "arm" in call_names
    assert "all_motor_test_start" not in call_names
    assert "all_motor_test_stop" in call_names
    assert "disarm" in call_names
    assert call_names.index("arm") < call_names.index("all_motor_test_stop")
    assert call_names.index("all_motor_test_stop") < call_names.index("disarm")


def test_cli_all_motor_test_auto_arm_command_fails(monkeypatch, tmp_path: Path, capsys):
    from esp_drone_cli.cli import main as cli_main

    class ArmRejectedSession(FakeSession):
        def arm(self) -> int:
            self._record("arm")
            return CmdStatus.REJECTED

    session = ArmRejectedSession()
    monkeypatch.setattr(cli_main, "connect_session_from_args", lambda args: session)
    monkeypatch.setattr(cli_main.time, "sleep", lambda _duration: None)

    rc = cli_main.main(
        [
            "--serial", "COM7", "all-motor-test",
            "--duty", "0.30", "--duration-s", "0.1",
            "--output-dir", str(tmp_path),
        ]
    )

    assert rc == CmdStatus.REJECTED
    err = capsys.readouterr().err
    assert "auto-arm command failed" in err
    assert "NOT_SENT_ARM_COMMAND_FAILED" in err
    call_names = [name for name, _args, _kwargs in session.calls]
    assert "arm" in call_names
    assert "all_motor_test_start" not in call_names
    assert "all_motor_test_stop" in call_names
    assert "disarm" in call_names
    assert call_names.index("arm") < call_names.index("all_motor_test_stop")


def test_cli_all_motor_test_start_returns_arm_required_after_auto_arm(monkeypatch, tmp_path: Path, capsys):
    from esp_drone_cli.cli import main as cli_main

    class StartArmRequiredSession(FakeSession):
        def all_motor_test_start(self, duty: float, duration_s: float) -> int:
            self._record("all_motor_test_start", duty, duration_s)
            return CmdStatus.ARM_REQUIRED

    session = StartArmRequiredSession()
    monkeypatch.setattr(cli_main, "connect_session_from_args", lambda args: session)
    monkeypatch.setattr(cli_main.time, "sleep", lambda _duration: None)

    rc = cli_main.main(
        [
            "--serial", "COM7", "all-motor-test",
            "--duty", "0.30", "--duration-s", "0.1",
            "--output-dir", str(tmp_path),
        ]
    )

    assert rc == CmdStatus.ARM_REQUIRED
    err = capsys.readouterr().err
    assert "auto-arm did not actually enter ARMED" in err
    call_names = [name for name, _args, _kwargs in session.calls]
    assert "arm" in call_names
    assert "all_motor_test_start" in call_names
    assert call_names.index("arm") < call_names.index("all_motor_test_start")
    assert "all_motor_test_stop" in call_names
    assert "disarm" in call_names
    assert call_names.index("all_motor_test_start") < call_names.index("all_motor_test_stop")
    assert call_names.index("all_motor_test_stop") < call_names.index("disarm")


def test_cli_all_motor_test_start_returns_conflict(monkeypatch, tmp_path: Path, capsys):
    from esp_drone_cli.cli import main as cli_main

    class ConflictSession(FakeSession):
        def arm(self) -> int:
            self._record("arm")
            self._set_runtime_state(arm_state=1, failsafe_reason=0, control_mode=1)
            return CmdStatus.OK

    session = ConflictSession()
    monkeypatch.setattr(cli_main, "connect_session_from_args", lambda args: session)
    monkeypatch.setattr(cli_main.time, "sleep", lambda _duration: None)

    rc = cli_main.main(
        [
            "--serial", "COM7", "all-motor-test",
            "--duty", "0.30", "--duration-s", "0.1",
            "--output-dir", str(tmp_path),
        ]
    )

    assert rc == CmdStatus.CONFLICT
    err = capsys.readouterr().err
    assert "CMD_STATUS_CONFLICT" in err
    assert "control_mode=1" in err
    call_names = [name for name, _args, _kwargs in session.calls]
    assert "arm" in call_names
    assert "all_motor_test_start" in call_names
    assert call_names.index("arm") < call_names.index("all_motor_test_start")
    assert "all_motor_test_stop" in call_names
    assert "disarm" in call_names
    assert call_names.index("all_motor_test_start") < call_names.index("all_motor_test_stop")
    assert call_names.index("all_motor_test_stop") < call_names.index("disarm")


def test_cli_motor_thrust_balance_uses_single_motor_path_and_logs(monkeypatch, tmp_path: Path, capsys):
    from esp_drone_cli.cli import main as cli_main

    session = FakeSession()
    seed_motor_scale_offset_params(session)
    monkeypatch.setattr(cli_main, "connect_session_from_args", lambda args: session)
    monkeypatch.setattr(cli_main.time, "sleep", lambda _duration: None)

    rc = cli_main.main(
        [
            "--serial",
            "COM7",
            "motor-thrust-balance",
            "--duties",
            "0.20",
            "--duration-s",
            "0.2",
            "--rest-s",
            "0",
            "--output-dir",
            str(tmp_path),
        ]
    )

    assert rc == 0
    output = capsys.readouterr().out
    assert "trim_applied=True" in output
    assert "trim_mode=motor_scale/motor_offset active" in output
    assert "trim_path=motor-test -> motor_set_test_output -> motor_set_armed_outputs" in output
    assert "trim_params=motor_trim,motor_gamma,motor_scale,motor_offset,motor_deadband,motor_min_start" in output
    assert "score_note=score is IMU vibration/disturbance response, not thrust-stand force" in output
    assert "trim_targets:" in output
    assert "M2 input=0.200 trim_target=0.2200 scale=1.1000 offset=0.0000" in output
    assert "M3 input=0.200 trim_target=0.1825 scale=0.9126 offset=0.0000" in output
    assert "trial=2 motor=M2 input_duty=0.200 trim_target_duty=0.2200 scale=1.1000 offset=0.0000" in output
    call_names = [name for name, _args, _kwargs in session.calls]
    assert "start_csv_log" in call_names
    assert "start_stream" in call_names
    assert "stop_stream" in call_names
    assert "stop_csv_log" in call_names
    assert "arm" not in call_names
    assert "all_motor_test_start" not in call_names
    assert ("get_param", ("motor_scale_m2", 1.0), {}) in session.calls
    assert ("get_param", ("motor_offset_m2", 1.0), {}) in session.calls
    for motor_index in range(4):
        assert ("motor_test", (motor_index, 0.20), {}) in session.calls
        assert ("motor_test", (motor_index, 0.0), {}) in session.calls
    assert ("disarm", (), {}) in session.calls
    summary_paths = [path for path in tmp_path.iterdir() if path.name.endswith("_motor_thrust_balance_summary.csv")]
    assert summary_paths
    summary_text = summary_paths[0].read_text(encoding="utf-8")
    assert "input_duty,trim_scale,trim_offset,trim_target_duty" in summary_text
    assert "rate_meas_yaw_raw_rms,rate_meas_yaw_raw_peak" in summary_text
    assert "rate_meas_yaw_filtered_rms,rate_meas_yaw_filtered_peak" in summary_text
    assert "acc_norm_min,acc_norm_max,acc_norm_mean" in summary_text
    assert "0.220000" in summary_text


def test_cli_motor_thrust_balance_no_trim_temporarily_neutralizes_scale_offset(monkeypatch, tmp_path: Path, capsys):
    from esp_drone_cli.cli import main as cli_main

    session = FakeSession()
    seed_motor_scale_offset_params(session)
    monkeypatch.setattr(cli_main, "connect_session_from_args", lambda args: session)
    monkeypatch.setattr(cli_main.time, "sleep", lambda _duration: None)

    rc = cli_main.main(
        [
            "--serial",
            "COM7",
            "motor-thrust-balance",
            "--duties",
            "0.20",
            "--duration-s",
            "0.2",
            "--settle-s",
            "0",
            "--rest-s",
            "0",
            "--motors",
            "M2,M3",
            "--output-dir",
            str(tmp_path),
            "--no-trim",
        ]
    )

    assert rc == 0
    output = capsys.readouterr().out
    assert "trim_applied=False" in output
    assert "trim_mode=no-trim: motor_scale=1.0 motor_offset=0.0 temporary" in output
    assert "M2 input=0.200 trim_target=0.2000 scale=1.0000 offset=0.0000" in output
    assert "M3 input=0.200 trim_target=0.2000 scale=1.0000 offset=0.0000" in output
    assert ("set_param", ("motor_scale_m2", 4, 1.0), {}) in session.calls
    assert ("set_param", ("motor_scale_m3", 4, 1.0), {}) in session.calls
    assert ("set_param", ("motor_offset_m2", 4, 0.0), {}) in session.calls
    assert ("set_param", ("motor_scale_m2", 4, 1.1), {}) in session.calls
    assert ("set_param", ("motor_scale_m3", 4, 0.9126), {}) in session.calls
    assert ("motor_test", (1, 0.20), {}) in session.calls
    assert ("motor_test", (2, 0.20), {}) in session.calls
    assert "all_motor_test_start" not in [name for name, _args, _kwargs in session.calls]
    assert session.get_param("motor_scale_m2").value == pytest.approx(1.10)
    assert session.get_param("motor_scale_m3").value == pytest.approx(0.9126)


def test_cli_analyze_vibration_log_runs_offline_and_writes_summary(monkeypatch, tmp_path: Path, capsys):
    from esp_drone_cli.cli import main as cli_main

    def fail_connect(_args):
        raise AssertionError("analyze-vibration-log must not connect to device")

    monkeypatch.setattr(cli_main, "connect_session_from_args", fail_connect)
    source = tmp_path / "diag_static.csv"
    source.write_text(
        "\n".join(
            [
                (
                    "timestamp_us,raw_gyro_x,raw_gyro_y,raw_gyro_z,"
                    "filtered_gyro_x,filtered_gyro_y,filtered_gyro_z,"
                    "rate_meas_yaw_raw,rate_meas_yaw_filtered,"
                    "raw_acc_x,raw_acc_y,raw_acc_z,loop_dt_us,"
                    "battery_voltage,kalman_valid,ground_trip_reason,motor_saturation_flag"
                ),
                "0,1,2,-10,1,2,-5,10,5,0,0,1,1000,4.1,1,0,0",
                "1000,2,3,20,2,3,8,-20,-8,0,0,1,1100,4.0,1,0,0",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    output = tmp_path / "diag_static_summary.csv"

    rc = cli_main.main(["analyze-vibration-log", str(source), "--mode", "static", "--output", str(output)])

    assert rc == 0
    stdout = capsys.readouterr().out
    assert f"summary_csv={output}" in stdout
    assert "mode=static motor=none duty=0.000 samples=2" in stdout
    assert "yaw_raw_minmax=-20.00..10.00" in stdout
    assert "yaw_raw_peak=20.00" in stdout
    assert "warning=none" in stdout
    summary_text = output.read_text(encoding="utf-8")
    assert "rate_meas_yaw_filtered_rms" in summary_text
    assert "loop_dt_us_min,loop_dt_us_max" in summary_text


def test_cli_motor_trim_estimate_applies_conservative_ram_params(monkeypatch, tmp_path: Path, capsys):
    from esp_drone_cli.cli import main as cli_main

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
    session = FakeSession()
    monkeypatch.setattr(cli_main, "connect_session_from_args", lambda args: session)

    rc = cli_main.main(["--serial", "COM7", "motor-trim-estimate", str(summary), "--apply"])

    assert rc == 0
    assert ("set_param", ("motor_scale_m1", 4, 1.0), {}) in session.calls
    assert ("set_param", ("motor_scale_m2", 4, 0.9), {}) in session.calls
    assert ("set_param", ("motor_scale_m3", 4, 1.1), {}) in session.calls
    assert ("set_param", ("motor_offset_m3", 4, 0.0), {}) in session.calls
    assert ("get_param", ("motor_scale_m1", 1.0), {}) in session.calls
    assert ("get_param", ("motor_scale_m2", 1.0), {}) in session.calls
    assert ("get_param", ("motor_offset_m4", 1.0), {}) in session.calls
    output = capsys.readouterr().out
    assert "M2 ratio_to_M1=1.600 scale=0.9000" in output
    assert "M3 ratio_to_M1=0.200 scale=1.1000" in output
    assert "score_note=score is IMU vibration/disturbance response, not thrust-stand force" in output
    assert "ratio_to_M1 is not a real thrust ratio" in output
    assert "write_confirm:" in output
    assert "M2 scale_param=motor_scale_m2 written=0.900000 readback=0.900000" in output
    assert "offset_param=motor_offset_m3 written=0.000000 readback=0.000000" in output
    assert "motor_trim_scale_m2" not in output


def test_cli_apply_short_hop_tuned_profile_writes_ram_only(monkeypatch, capsys):
    from esp_drone_cli.cli import main as cli_main

    session = FakeSession()
    monkeypatch.setattr(cli_main, "connect_session_from_args", lambda args: session)

    rc = cli_main.main(["--serial", "COM7", "apply-short-hop-tuned-profile"])

    assert rc == 0
    call_names = [name for name, _args, _kwargs in session.calls]
    assert "require_attitude_ground_verify" in call_names
    assert ("set_param", ("rate_kp_roll", 4, 0.00077), {}) in session.calls
    assert ("set_param", ("rate_kp_pitch", 4, 0.00077), {}) in session.calls
    assert ("set_param", ("ground_att_rate_limit_roll", 4, 6.0), {}) in session.calls
    assert ("set_param", ("ground_att_trip_deg", 4, 10.0), {}) in session.calls
    assert ("set_param", ("ground_test_base_duty", 4, 0.35), {}) in session.calls
    assert ("set_param", ("ground_test_max_extra_duty", 4, 0.1), {}) in session.calls
    assert ("set_param", ("ground_test_motor_balance_limit", 4, 0.3), {}) in session.calls
    assert ("set_param", ("ground_test_ramp_duty_per_s", 4, 0.25), {}) in session.calls
    assert "save_params" not in call_names

    output = capsys.readouterr().out
    assert "profile=short_hop_tuned_ram" in output
    assert "persisted=False" in output
    assert "motor_trim=unchanged" in output
    assert "ground_att_rate_limit_roll=6.0" in output


def test_cli_attitude_start_old_firmware_fails_before_param_write(monkeypatch, capsys):
    from esp_drone_cli.cli import main as cli_main

    session = FakeSession()
    session.device_info = DeviceInfo(
        protocol_version=2,
        imu_mode=1,
        arm_state=0,
        stream_enabled=0,
        feature_bitmap=0x1F,
    )
    monkeypatch.setattr(cli_main, "connect_session_from_args", lambda args: session)

    assert cli_main.main(["--serial", "COM7", "attitude-test", "start", "--base-duty", "0.05"]) == 1
    error_text = capsys.readouterr().err
    assert "does not advertise bench-only hang-attitude support" in error_text
    call_names = [name for name, _args, _kwargs in session.calls]
    assert "set_param" not in call_names
    assert "attitude_test_start" not in call_names


def test_firmware_dispatch_registers_attitude_capture_ref():
    repo_root = Path(__file__).resolve().parents[3]
    protocol = (repo_root / "firmware" / "main" / "console" / "console_protocol.h").read_text(encoding="utf-8")
    dispatch = (repo_root / "firmware" / "main" / "console" / "console.c").read_text(encoding="utf-8")

    assert "CMD_ATTITUDE_CAPTURE_REF = 10" in protocol
    assert "case CMD_ATTITUDE_CAPTURE_REF:" in dispatch
    assert "attitude_bench_capture_reference(&sample)" in dispatch
    assert "CONSOLE_FEATURE_ATTITUDE_HANG_BENCH" in protocol


def test_firmware_udp_protocol_covers_usb_console_commands():
    repo_root = Path(__file__).resolve().parents[3]
    dispatch = (repo_root / "firmware" / "main" / "console" / "console.c").read_text(encoding="utf-8")
    udp_protocol = (repo_root / "firmware" / "main" / "udp_protocol" / "udp_protocol.c").read_text(encoding="utf-8")

    usb_cases = set(re.findall(r"case\s+(CMD_[A-Z0-9_]+)\s*:", dispatch))
    udp_cases = set(re.findall(r"case\s+(CMD_[A-Z0-9_]+)\s*:", udp_protocol))

    assert usb_cases - udp_cases == set()


def test_firmware_dispatch_registers_udp_manual_control():
    repo_root = Path(__file__).resolve().parents[3]
    protocol = (repo_root / "firmware" / "main" / "console" / "console_protocol.h").read_text(encoding="utf-8")
    dispatch = (repo_root / "firmware" / "main" / "console" / "console.c").read_text(encoding="utf-8")
    udp_manual = (repo_root / "firmware" / "main" / "udp_manual" / "udp_manual.c").read_text(encoding="utf-8")
    app_main = (repo_root / "firmware" / "main" / "app_main.c").read_text(encoding="utf-8")
    udp_protocol = (repo_root / "firmware" / "main" / "udp_protocol" / "udp_protocol.c").read_text(encoding="utf-8")

    assert "CMD_UDP_MANUAL_ENABLE = 13" in protocol
    assert "MSG_UDP_MANUAL_SETPOINT = 0x50" in protocol
    assert "CONSOLE_FEATURE_UDP_MANUAL_CONTROL" in protocol
    assert "CONSOLE_FEATURE_STABILIZE_MIN" in protocol
    assert "CONSOLE_FEATURE_PREFLIGHT_CHECK" in protocol
    assert "case CMD_UDP_MANUAL_ENABLE:" in dispatch
    assert "case MSG_UDP_MANUAL_SETPOINT:" in dispatch
    assert "CONTROL_MODE_STABILIZE_MIN" in udp_manual
    assert "preflight_check_stabilize_min" in udp_manual
    assert "stabilize-min watchdog timeout" in udp_manual
    assert "ground_tune_capture_reference(&sample, &estimator_state)" not in udp_manual
    assert "flight_control_stabilize_min_rate_setpoint" in app_main
    assert "legacy udp manual flight path disabled: use stabilize-min" in app_main
    assert "udp_protocol_task" in udp_protocol
    assert "MSG_UDP_MANUAL_SETPOINT" in udp_protocol


def test_firmware_dispatch_registers_attitude_ground_verify_and_liftoff_paths():
    repo_root = Path(__file__).resolve().parents[3]
    protocol = (repo_root / "firmware" / "main" / "console" / "console_protocol.h").read_text(encoding="utf-8")
    dispatch = (repo_root / "firmware" / "main" / "console" / "console.c").read_text(encoding="utf-8")
    app_main = (repo_root / "firmware" / "main" / "app_main.c").read_text(encoding="utf-8")
    ground_tune = (repo_root / "firmware" / "main" / "ground_tune" / "ground_tune.c").read_text(encoding="utf-8")
    udp_protocol = (repo_root / "firmware" / "main" / "udp_protocol" / "udp_protocol.c").read_text(encoding="utf-8")

    assert "CONSOLE_PROTOCOL_VERSION 0x0Au" in protocol
    assert "CONSOLE_FEATURE_ATTITUDE_GROUND_VERIFY" in protocol
    assert "CONSOLE_FEATURE_LOW_RISK_LIFTOFF_VERIFY" in protocol
    assert "CONSOLE_FEATURE_ALL_MOTOR_TEST" in protocol
    assert "CMD_ATTITUDE_GROUND_VERIFY_START = 22" in protocol
    assert "CMD_LIFTOFF_VERIFY_START = 24" in protocol
    assert "CMD_ATTITUDE_GROUND_SET_TARGET = 26" in protocol
    assert "CMD_ALL_MOTOR_TEST_START = 27" in protocol
    assert "case CMD_ATTITUDE_GROUND_VERIFY_START:" in dispatch
    assert "case CMD_LIFTOFF_VERIFY_START:" in dispatch
    assert "case CMD_ATTITUDE_GROUND_SET_TARGET:" in dispatch
    assert "case CMD_ALL_MOTOR_TEST_START:" in dispatch
    assert "case CMD_ATTITUDE_GROUND_VERIFY_START:" in udp_protocol
    assert "case CMD_LIFTOFF_VERIFY_START:" in udp_protocol
    assert "case CMD_ALL_MOTOR_TEST_START:" in udp_protocol
    assert "runtime_state_set_all_motor_test(req.arg_f32, duration_ms, 0u)" in dispatch
    assert "runtime_state_set_all_motor_test(req->arg_f32, duration_ms, 0u)" in udp_protocol
    assert "all-motor test started duty=%.3f duration_ms=%lu" in dispatch
    assert "all-motor test started duty=%.3f duration_ms=%lu" in udp_protocol
    assert "all-motor test stopped: arm_state=%d failsafe_reason=%d" in app_main
    assert "if (all_motor.start_us == 0u)" in app_main
    assert "runtime_state_set_all_motor_test(all_motor.duty, all_motor.duration_ms, now_us)" in app_main
    assert "GROUND_TUNE_SUBMODE_ATTITUDE_VERIFY" in app_main
    assert "GROUND_TUNE_SUBMODE_LOW_RISK_LIFTOFF" in app_main
    assert "CONTROL_MODE_ALL_MOTOR_TEST" in app_main
    assert "liftoff_verify_auto_disarm_ms" in app_main
    assert "ground_att_target_limit_deg" in ground_tune
    assert "outer_clamp_flags" in ground_tune


def test_gui_ground_defaults_preserve_confirmed_rate_p_only_baseline():
    repo_root = Path(__file__).resolve().parents[3]
    gui_main = (repo_root / "tools" / "esp_drone_cli" / "esp_drone_cli" / "gui" / "main_window.py").read_text(encoding="utf-8")

    assert '("rate_kp_roll", "rate_kp_roll", 0.0, 0.05, 5, 0.0001, 0.0007)' in gui_main
    assert '("rate_kp_pitch", "rate_kp_pitch", 0.0, 0.05, 5, 0.0001, 0.0007)' in gui_main
    assert '("rate_kp_yaw", "rate_kp_yaw", 0.0, 0.05, 5, 0.0001, 0.0005)' in gui_main
    assert '("rate_ki_roll", "rate_ki_roll", 0.0, 0.02, 5, 0.0001, 0.0)' in gui_main
    assert '("rate_kd_roll", "rate_kd_roll", 0.0, 0.01, 5, 0.0001, 0.0)' in gui_main
    assert '("ground_att_kp_roll", "ground_att_kp_roll", 0.0, 10.0, 2, 0.1, 0.8)' in gui_main
    assert '("ground_att_rate_limit_roll", "ground_att_rate_limit_roll", 0.0, 60.0, 1, 1.0, 4.0)' in gui_main
    assert '("ground_att_target_limit_deg", "ground_target_limit", 0.5, 10.0, 1, 0.5, 2.0)' in gui_main
    assert '("ground_test_max_extra_duty", "ground_max_extra_duty", 0.0, 0.20, 3, 0.01, 0.03)' in gui_main


def test_firmware_registers_softap_udp_transport():
    repo_root = Path(__file__).resolve().parents[3]
    app_main = (repo_root / "firmware" / "main" / "app_main.c").read_text(encoding="utf-8")
    cmake = (repo_root / "firmware" / "main" / "CMakeLists.txt").read_text(encoding="utf-8")
    params_h = (repo_root / "firmware" / "main" / "params" / "params.h").read_text(encoding="utf-8")
    params_c = (repo_root / "firmware" / "main" / "params" / "params.c").read_text(encoding="utf-8")
    wifi_ap = (repo_root / "firmware" / "main" / "network" / "wifi_ap.c").read_text(encoding="utf-8")
    udp_protocol = (repo_root / "firmware" / "main" / "udp_protocol" / "udp_protocol.c").read_text(encoding="utf-8")

    assert "wifi_ap_start();" in app_main
    assert '"network"' in cmake
    assert "esp_wifi" in cmake
    assert "esp_netif" in cmake
    assert "esp_event" in cmake
    assert "wifi_ap_enable" in params_h
    assert "wifi_ap_channel" in params_c
    assert "wifi_udp_port" in params_c
    assert "wifi_mode" in params_h
    assert "sta_ssid" in params_h
    assert "PARAM_TYPE_STRING" in params_h
    assert "store->wifi_mode" in params_c
    assert "esp_netif_create_default_wifi_ap" in wifi_ap
    assert "esp_netif_create_default_wifi_sta" in wifi_ap
    assert "WIFI_MODE_STA" in wifi_ap
    assert "WIFI_MODE_APSTA" in wifi_ap
    assert "softap started" in wifi_ap
    assert "ssid=%s" in wifi_ap
    assert "sta connected ip=" in wifi_ap
    assert "sta fallback softap started" in wifi_ap
    assert "wifi link lost:" in wifi_ap
    assert "motor_stop_all();" in wifi_ap
    assert "safety_request_disarm();" in wifi_ap
    assert "WIFI_EVENT_AP_STACONNECTED" in wifi_ap
    assert "INADDR_ANY" in udp_protocol
    assert "params_get()->wifi_udp_port" in udp_protocol


def test_docs_describe_softap_sta_udp_paths():
    repo_root = Path(__file__).resolve().parents[3]
    transport_doc = (repo_root / "docs" / "softap_udp_transport.md").read_text(encoding="utf-8")
    cli_doc = (repo_root / "docs" / "python_cli_usage.md").read_text(encoding="utf-8")
    gui_doc = (repo_root / "docs" / "python_gui_usage.md").read_text(encoding="utf-8")
    protocol_doc = (repo_root / "docs" / "udp_manual_control_protocol.md").read_text(encoding="utf-8")

    assert "`wifi_mode`: `softap`, `sta`, or `apsta`" in transport_doc
    assert "sta connected ip=" in transport_doc
    assert "SoftAP as the fallback" in transport_doc
    assert "python -m esp_drone_cli --host 192.168.50.42 connect" in cli_doc
    assert "`--host` and `--drone-ip` are aliases" in cli_doc
    assert "`Mode = STA`" in gui_doc
    assert "STA WiFi disconnect clears active motor tests/control state" in protocol_doc


def test_firmware_registers_motor_trim_params_and_applies_before_motor_clamp():
    repo_root = Path(__file__).resolve().parents[3]
    params_h = (repo_root / "firmware" / "main" / "params" / "params.h").read_text(encoding="utf-8")
    params_c = (repo_root / "firmware" / "main" / "params" / "params.c").read_text(encoding="utf-8")
    motor_c = (repo_root / "firmware" / "main" / "motor" / "motor.c").read_text(encoding="utf-8")
    mixer_c = (repo_root / "firmware" / "main" / "mixer" / "mixer.c").read_text(encoding="utf-8")
    app_main = (repo_root / "firmware" / "main" / "app_main.c").read_text(encoding="utf-8")

    assert "#define PARAMS_SCHEMA_VERSION 11u" in params_h
    assert "float motor_trim_scale[4];" in params_h
    assert "float motor_trim_offset[4];" in params_h
    assert "float motor_scale[4];" in params_h
    assert "float motor_offset[4];" in params_h
    assert "float motor_min_start[4];" in params_h
    assert "float motor_deadband[4];" in params_h
    assert "float motor_gamma[4];" in params_h
    assert "float motor_trim[4];" in params_h
    assert '"motor_trim_scale_m3"' not in params_c
    assert '"motor_trim_offset_m3"' not in params_c
    assert '"motor_scale_m3"' in params_c
    assert '"motor_offset_m3"' in params_c
    assert '"motor_deadband_m3"' in params_c
    assert '"motor_gamma_m3"' in params_c
    assert "store->motor_pwm_freq_hz = 15000;" in params_c
    assert "store->motor_idle_duty = 0.03f;" in params_c
    assert "store->motor_startup_boost_duty = 0.05f;" in params_c
    assert "store->motor_slew_limit_per_tick = 0.02f;" in params_c
    assert "store->motor_trim_scale[i] = 1.0f;" in params_c
    assert "store->motor_trim_offset[i] = 0.0f;" in params_c
    assert "store->motor_scale[i] = 1.0f;" in params_c
    assert "store->motor_gamma[i] = 1.0f;" in params_c
    assert "PARAMS_SCHEMA_VERSION_BEFORE_MOTOR_TRIM 8u" in params_c
    assert "PARAMS_SCHEMA_VERSION_BEFORE_STABILIZE_MIN 9u" in params_c
    assert "PARAMS_SCHEMA_VERSION_BEFORE_WIFI_STA 10u" in params_c
    assert "params_validate_pwm_freq_resolution" in params_c
    assert "freq_hz > 40000u" in params_c
    assert "<= 80000000ull" in params_c
    assert "duty = duty * params->motor_scale[logical_motor] + params->motor_offset[logical_motor];" in motor_c
    assert "duty = motor_clampf(duty, 0.0f, params->motor_max_duty);" in motor_c
    assert re.search(r"void motor_set_test_output\(.*?motor_set_armed_outputs\(outputs, false\);", motor_c, re.DOTALL)
    assert re.search(
        r"CONTROL_MODE_ALL_MOTOR_TEST.*?command_outputs\[i\] = duty;.*?motor_set_armed_outputs\(command_outputs, false\);",
        app_main,
        re.DOTALL,
    )
    mixer_body = re.search(r"void mixer_mix\(.*?\n}\n\nstatic bool mixer_check_direction", mixer_c, re.DOTALL)
    assert mixer_body is not None
    assert "motor_scale" not in mixer_body.group(0)
    assert "motor_offset" not in mixer_body.group(0)
    assert "motor_trim" not in mixer_body.group(0)
    write_single_body = re.search(r"static void motor_write_single\(.*?\n}", motor_c, re.DOTALL)
    assert write_single_body is not None
    assert "motor_apply_compensation" not in write_single_body.group(0)


def test_firmware_telemetry_battery_read_does_not_emit_transient_zero():
    repo_root = Path(__file__).resolve().parents[3]
    app_main = (repo_root / "firmware" / "main" / "app_main.c").read_text(encoding="utf-8")
    board_config = (repo_root / "firmware" / "main" / "board" / "board_config.c").read_text(encoding="utf-8")

    assert "s_battery_adc_mutex = xSemaphoreCreateMutex();" in board_config
    assert "xSemaphoreTake(s_battery_adc_mutex, portMAX_DELAY);" in board_config
    assert "BATTERY_ADC_SAMPLE_COUNT 5u" in board_config
    assert "board_battery_sort_raw_samples(raw_samples);" in board_config
    assert "BATTERY_ADC_OUTLIER_DELTA_V" in board_config
    assert "s_battery_outlier_streak" in board_config
    assert "board_battery_return_last_valid" in board_config
    assert "if (board_battery_read(&battery_raw, &battery_mv, &battery_v) == ESP_OK)" in app_main
    assert "last_battery_valid = true;" in app_main
    assert "battery_raw = last_battery_raw;" in app_main
    assert "float battery_v = last_battery_v;" in app_main


def test_firmware_telemetry_reports_observed_rate_error():
    repo_root = Path(__file__).resolve().parents[3]
    console_c = (repo_root / "firmware" / "main" / "console" / "console.c").read_text(encoding="utf-8")
    udp_protocol = (repo_root / "firmware" / "main" / "udp_protocol" / "udp_protocol.c").read_text(encoding="utf-8")

    for source in (console_c, udp_protocol):
        assert "rate_measured_for_error = estimator_state.filtered_rate_rpy_dps;" in source
        assert "rate_setpoint_request.roll - rate_measured_for_error.roll" in source
        assert ".rate_err_roll = rate_error_observed.roll" in source
        assert ".rate_err_pitch = rate_error_observed.pitch" in source
        assert ".rate_err_yaw = rate_error_observed.yaw" in source


def test_firmware_console_ack_path_is_not_starved_by_telemetry():
    repo_root = Path(__file__).resolve().parents[3]
    app_main = (repo_root / "firmware" / "main" / "app_main.c").read_text(encoding="utf-8")
    console_c = (repo_root / "firmware" / "main" / "console" / "console.c").read_text(encoding="utf-8")

    defines = {
        match.group(1): int(match.group(2))
        for match in re.finditer(r"^#define\s+([A-Z_]+_TASK_PRIO)\s+(\d+)\s*$", app_main, re.MULTILINE)
    }

    assert defines["SERVICE_TASK_PRIO"] > defines["TELEMETRY_TASK_PRIO"]
    assert defines["SERVICE_TASK_PRIO"] > defines["RC_UDP_TASK_PRIO"]
    assert defines["SERVICE_TASK_PRIO"] < defines["FLIGHT_CONTROL_TASK_PRIO"]
    assert re.search(r"xSemaphoreTake\(\s*s_console_tx_mutex,\s*telemetry_frame \? 0 : portMAX_DELAY\s*\)", console_c)
    assert "if (!telemetry_frame)" in console_c
    assert "fflush(stdout);" in console_c
    assert "fsync(fileno(stdout));" in console_c


def test_set_param_detects_device_rejection():
    class RejectingTransport(MockTransport):
        def send_message(self, msg_type: int, payload: bytes = b"", flags: int = 0, seq: int = 0) -> None:
            if msg_type == MsgType.PARAM_SET:
                type_id = payload[0]
                name_len = payload[1]
                name = payload[2 : 2 + name_len].decode("ascii")
                value_bytes = b"\x00\x00\x00\x00" if type_id == 4 else payload[2 + name_len :]
                self.inject(Frame(MsgType.PARAM_VALUE, flags, seq, encode_param_payload(name, type_id, value_bytes)))
                return
            super().send_message(msg_type, payload, flags=flags, seq=seq)

    session = DeviceSession()
    transport = RejectingTransport()
    session.connect_transport(transport)
    with pytest.raises(RuntimeError, match="set_param rejected by device"):
        session.set_param("rate_kp_roll", 4, "0.0035")
    session.disconnect()


def test_gui_entry_reports_missing_pyqt5_without_affecting_cli(monkeypatch):
    real_import = builtins.__import__

    def guarded_import(name, globals=None, locals=None, fromlist=(), level=0):
        if name.startswith("PyQt5") or name.startswith("pyqtgraph"):
            raise ModuleNotFoundError(name)
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", guarded_import)
    sys.modules.pop("esp_drone_cli.gui.main_window", None)
    from esp_drone_cli import gui_main

    assert gui_main.main([]) == 1

    from esp_drone_cli.cli.main import build_parser

    assert build_parser().prog == "esp-drone-cli"


@pytest.mark.skipif(importlib.util.find_spec("PyQt5") is None or importlib.util.find_spec("pyqtgraph") is None, reason="PyQt5/pyqtgraph not installed")
def test_gui_entry_direct_file_import_loads_runner(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    repo_root = Path(__file__).resolve().parents[3]
    entry_path = repo_root / "tools" / "esp_drone_cli" / "esp_drone_cli" / "gui_main.py"
    spec = importlib.util.spec_from_file_location("esp_drone_gui_direct_entry", entry_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    run_gui = module._load_run_gui()

    assert run_gui.__name__ == "run_gui"
    assert run_gui.__module__ == "esp_drone_cli.gui.main_window"


def test_compatibility_shims_resolve_to_core_owners():
    from esp_drone_cli.client import DeviceSession as ShimDeviceSession
    from esp_drone_cli.client import EspDroneClient
    from esp_drone_cli.core.device_session import DeviceSession as CoreDeviceSession
    from esp_drone_cli.protocol.messages import MsgType as ShimMsgType
    from esp_drone_cli.core.protocol.messages import MsgType as CoreMsgType
    from esp_drone_cli.transport.serial_link import SerialTransport as ShimSerialTransport
    from esp_drone_cli.core.transport.serial_link import SerialTransport as CoreSerialTransport

    assert ShimDeviceSession is CoreDeviceSession
    assert EspDroneClient is CoreDeviceSession
    assert ShimMsgType is CoreMsgType
    assert ShimSerialTransport is CoreSerialTransport


def test_serial_transport_keeps_default_control_lines(monkeypatch):
    from esp_drone_cli.core.transport import serial_link

    serial_instances = []

    class FakeSerial:
        def __init__(self, *args, **kwargs) -> None:
            self.args = args
            self.kwargs = kwargs
            self.dtr_assignments = []
            self.rts_assignments = []
            serial_instances.append(self)

        def __setattr__(self, name, value):
            if name == "dtr":
                self.__dict__.setdefault("dtr_assignments", []).append(value)
            if name == "rts":
                self.__dict__.setdefault("rts_assignments", []).append(value)
            self.__dict__[name] = value

        def reset_input_buffer(self) -> None:
            return None

        def reset_output_buffer(self) -> None:
            return None

    monkeypatch.setattr(serial_link.serial, "Serial", FakeSerial)
    monkeypatch.setattr(serial_link.time, "sleep", lambda _seconds: None)

    serial_link.SerialTransport("COM7", baudrate=115200, timeout=0.2)

    assert len(serial_instances) == 1
    assert serial_instances[0].kwargs == {
        "port": "COM7",
        "baudrate": 115200,
        "timeout": 0.2,
        "write_timeout": 1.0,
    }
    assert serial_instances[0].dtr_assignments == []
    assert serial_instances[0].rts_assignments == []


def test_serial_transport_recv_frame_skips_invalid_packets(monkeypatch):
    from esp_drone_cli.core.transport import serial_link

    class FakeSerial:
        def __init__(self, *args, **kwargs) -> None:
            self.data = bytearray()

        def reset_input_buffer(self) -> None:
            return None

        def reset_output_buffer(self) -> None:
            return None

        def read(self, _size: int) -> bytes:
            if not self.data:
                return b""
            return bytes([self.data.pop(0)])

    fake_serial = FakeSerial()
    monkeypatch.setattr(serial_link.serial, "Serial", lambda *args, **kwargs: fake_serial)
    monkeypatch.setattr(serial_link.time, "sleep", lambda _seconds: None)

    transport = serial_link.SerialTransport("COM7", settle_delay_s=0.0)
    hello_payload = HELLO_RESP_STRUCT.pack(1, 1, 0, 0, 0xF)
    fake_serial.data.extend(b"bad\x00")
    fake_serial.data.extend(encode_serial_packet(MsgType.HELLO_RESP, hello_payload))

    frame = transport.recv_frame(timeout=0.2)

    assert frame.msg_type == MsgType.HELLO_RESP
    assert frame.payload == hello_payload


def test_serial_transport_preserves_partial_packet_across_timeouts(monkeypatch):
    from esp_drone_cli.core.transport import serial_link

    class FakeSerial:
        def __init__(self, *args, **kwargs) -> None:
            self.chunks: list[bytes] = []

        def reset_input_buffer(self) -> None:
            return None

        def reset_output_buffer(self) -> None:
            return None

        def read(self, _size: int) -> bytes:
            if not self.chunks:
                return b""
            chunk = self.chunks.pop(0)
            if chunk == b"":
                time.sleep(0.01)
                return b""
            if len(chunk) > 1:
                self.chunks.insert(0, chunk[1:])
            return chunk[:1]

    fake_serial = FakeSerial()
    monkeypatch.setattr(serial_link.serial, "Serial", lambda *args, **kwargs: fake_serial)
    monkeypatch.setattr(serial_link.time, "sleep", lambda _seconds: None)

    transport = serial_link.SerialTransport("COM7", settle_delay_s=0.0)
    packet = encode_serial_packet(MsgType.STREAM_CTRL, b"\x01", seq=7)
    fake_serial.chunks.extend([packet[:4], b""])

    with pytest.raises(TimeoutError):
        transport.recv_frame(timeout=0.001)

    fake_serial.chunks.append(packet[4:])
    frame = transport.recv_frame(timeout=0.2)

    assert frame.msg_type == MsgType.STREAM_CTRL
    assert frame.payload == b"\x01"


# ---------------------------------------------------------------------------
# GUI WiFi settings tests
# ---------------------------------------------------------------------------

def test_gui_wifi_settings_qsettings_persistence(monkeypatch, tmp_path: Path):
    """WiFi settings fields round-trip through QSettings."""
monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5.QtCore import QSettings
    from PyQt5.QtWidgets import QApplication
    from esp_drone_cli.gui.main_window import MainWindow

    app = QApplication.instance() or QApplication([])
    settings = QSettings(str(tmp_path / "wifi_test.ini"), QSettings.IniFormat)
    settings.setValue("wifi/mode", "apsta")
    settings.setValue("wifi/ssid", "TestWiFi")
    settings.setValue("wifi/static_ip", "192.168.1.100")
    settings.setValue("wifi/gateway", "192.168.1.1")
    settings.setValue("wifi/netmask", "255.255.255.0")
    settings.setValue("wifi/remember_password", False)

    window = MainWindow(settings=settings)
    window._load_settings()
    assert window._wifi_mode_value() == "apsta"
    assert window.wifi_ssid_edit.text() == "TestWiFi"
    assert window.wifi_static_ip_edit.text() == "192.168.1.100"
    assert window.wifi_password_edit.text() == ""
    assert not window.remember_password_check.isChecked()

    window.wifi_ssid_edit.setText("NewWiFi")
    window._save_settings()
    assert settings.value("wifi/ssid") == "NewWiFi"
    assert settings.value("wifi/password") is None

    window.remember_password_check.setChecked(True)
    window.wifi_password_edit.setText("secret123")
    window._save_settings()
    assert settings.value("wifi/password") == "secret123"

    window.remember_password_check.setChecked(False)
    window._save_settings()
    assert settings.value("wifi/password") is None
    window.close()
    app.quit()


def test_gui_wifi_write_order_matches_parameter_list(monkeypatch, tmp_path: Path):
    """Verify _write_wifi_config writes params in the correct order."""
monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5.QtCore import QSettings
    from PyQt5.QtWidgets import QApplication
    from esp_drone_cli.gui.main_window import MainWindow
    from esp_drone_cli.core.models import ParamValue

    app = QApplication.instance() or QApplication([])
    settings = QSettings(str(tmp_path / "wifi_test2.ini"), QSettings.IniFormat)
    written_params: list[tuple[str, str]] = []

    class FakeSession:
        connected = True
        def set_param(self, name, type_id, value):
            written_params.append((name, str(value)))
            return ParamValue(name=name, type_id=type_id, value=value)

    window = MainWindow(settings=settings)
    window._session = FakeSession()
    window.wifi_mode_combo.setCurrentIndex(2)
    window.wifi_ssid_edit.setText("MyHomeWiFi")
    window.wifi_password_edit.setText("pass123")
    window.wifi_static_ip_edit.setText("10.0.0.50")
    window.wifi_gateway_edit.setText("10.0.0.1")
    window.wifi_netmask_edit.setText("255.0.0.0")

    window._write_wifi_config()
    assert len(written_params) >= 3
    assert written_params[0] == ("wifi_mode", "apsta")
    assert written_params[1] == ("sta_ssid", "MyHomeWiFi")
    assert written_params[2] == ("sta_password", "pass123")
    written_names = [p[0] for p in written_params]
    assert "sta_static_ip" in written_names
    assert "sta_gateway" in written_names
    assert "sta_netmask" in written_names

    written_params.clear()
    window.wifi_static_ip_edit.setText("")
    window.wifi_gateway_edit.setText("")
    window.wifi_netmask_edit.setText("")
    window._write_wifi_config()
    written_names = [p[0] for p in written_params]
    assert "sta_static_ip" not in written_names
    assert "sta_gateway" not in written_names
    assert "sta_netmask" not in written_names

    window.close()
    app.quit()


def test_gui_wifi_mode_defaults_to_apsta(monkeypatch, tmp_path: Path):
    """WiFi mode combo defaults to AP+STA for safety."""
monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5.QtCore import QSettings
    from PyQt5.QtWidgets import QApplication
    from esp_drone_cli.gui.main_window import MainWindow

    app = QApplication.instance() or QApplication([])
    settings = QSettings(str(tmp_path / "wifi_test3.ini"), QSettings.IniFormat)
    window = MainWindow(settings=settings)
    assert window._wifi_mode_value() == "apsta"
    assert window.wifi_mode_combo.currentIndex() == 2
    window.close()
    app.quit()


def test_gui_softap_default_connection_not_affected(monkeypatch, tmp_path: Path):
    """Confirm SoftAP default host/port are unchanged."""
monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5.QtCore import QSettings
    from PyQt5.QtWidgets import QApplication
    from esp_drone_cli.gui.main_window import MainWindow

    app = QApplication.instance() or QApplication([])
    settings = QSettings(str(tmp_path / "wifi_test4.ini"), QSettings.IniFormat)
    window = MainWindow(settings=settings)
    assert window._udp_mode_value() == "softap"
    assert window.udp_host_edit.text() == "192.168.4.1"
    assert window.udp_port_spin.value() == 2391
    window.close()
    app.quit()


def test_cli_set_string_type_accepted() -> None:
    """CLI parser accepts 'set name string value'."""
    from esp_drone_cli.cli.main import main as cli_main

    import argparse
    try:
        cli_main(["--serial", "COM99", "set", "sta_ssid", "string", "MyHotspot"])
    except SystemExit:
        pass


def test_cli_host_and_drone_ip_flags_equivalent(monkeypatch) -> None:
    """--host and --drone-ip are aliases mapping to drone_ip destination."""
    from esp_drone_cli.cli.main import build_parser, EspDroneArgumentParser

    parser = build_parser()
    args = parser.parse_args(["--host", "10.0.0.50", "connect"])
    assert args.drone_ip == "10.0.0.50"

    args2 = parser.parse_args(["--drone-ip", "10.0.0.51", "connect"])
    assert args2.drone_ip == "10.0.0.51"


def test_gui_wifi_not_connected_save_reboot_does_not_call_save(monkeypatch, tmp_path: Path):
    """When not connected, save&reboot only logs and returns; never calls save_params/reboot."""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5.QtCore import QSettings
    from PyQt5.QtWidgets import QApplication
    from esp_drone_cli.gui.main_window import MainWindow

    app = QApplication.instance() or QApplication([])
    settings = QSettings(str(tmp_path / "wifi_test_nc.ini"), QSettings.IniFormat)

    save_called = []
    reboot_called = []

    class FakeSessionNotConnected:
        connected = False

        def set_param(self, name, type_id, value):
            return type("ParamValue", (), {"name": name, "type_id": type_id, "value": value})()

        def save_params(self):
            save_called.append(True)

        def reboot(self):
            reboot_called.append(True)

        def disconnect(self):
            pass

    window = MainWindow(settings=settings)
    window._session = FakeSessionNotConnected()
    window.show_password_check.setChecked(False)
    window.remember_password_check.setChecked(False)

    window._save_and_reboot_wifi()
    assert not save_called
    assert not reboot_called

    window.close()
    app.quit()


def test_gui_wifi_write_order_correct_with_dirty_tracking(monkeypatch, tmp_path: Path):
    """Write WiFi Config sets dirty=False and last_wifi_write_ok=True on success."""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5.QtCore import QSettings
    from PyQt5.QtWidgets import QApplication
    from esp_drone_cli.gui.main_window import MainWindow
    from esp_drone_cli.core.models import ParamValue

    app = QApplication.instance() or QApplication([])
    settings = QSettings(str(tmp_path / "wifi_dirty.ini"), QSettings.IniFormat)

    class FakeSessionConnected:
        connected = True

        def set_param(self, name, type_id, value):
            return ParamValue(name=name, type_id=type_id, value=value)

    window = MainWindow(settings=settings)
    window._session = FakeSessionConnected()
    window.show_password_check.setChecked(False)
    window.remember_password_check.setChecked(False)

    window._mark_wifi_dirty()
    assert window._wifi_config_dirty
    assert not window._last_wifi_write_ok

    window._write_wifi_config()
    assert not window._wifi_config_dirty
    assert window._last_wifi_write_ok

    window.close()
    app.quit()


def test_gui_wifi_password_not_in_qsettings_by_default(monkeypatch, tmp_path: Path):
    """Default: sta_password never written to QSettings unless remember is checked."""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5.QtCore import QSettings
    from PyQt5.QtWidgets import QApplication
    from esp_drone_cli.gui.main_window import MainWindow

    app = QApplication.instance() or QApplication([])
    settings = QSettings(str(tmp_path / "wifi_nopass.ini"), QSettings.IniFormat)
    settings.clear()

    window = MainWindow(settings=settings)
    window.show_password_check.setChecked(False)
    window.remember_password_check.setChecked(False)
    window.wifi_password_edit.setText("secret123")
    window._save_settings()

    assert settings.value("wifi/password") is None

    window.remember_password_check.setChecked(True)
    window._save_settings()
    assert settings.value("wifi/password") == "secret123"

    window.remember_password_check.setChecked(False)
    window._save_settings()
    assert settings.value("wifi/password") is None

    window.close()
    app.quit()
