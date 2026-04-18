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

import pytest

from esp_drone_cli.core import DeviceSession
from esp_drone_cli.cli.main import (
    analyze_attitude_ground_verify_samples,
    analyze_liftoff_verify_samples,
    format_rate_status_line,
)
from esp_drone_cli.core.models import (
    CMD_REQ_STRUCT,
    CapabilityError,
    DeviceInfo,
    FEATURE_ATTITUDE_GROUND_VERIFY,
    FEATURE_ATTITUDE_HANG_BENCH,
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
)
from esp_drone_cli.core.protocol.framing import decode_frame, encode_frame, encode_serial_packet
from esp_drone_cli.core.protocol.messages import CmdId, Frame, MsgType


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
        self._params = [
            ParamValue("alpha", 2, 42),
            ParamValue("beta", 4, 1.5),
            ParamValue("udp_manual_max_pwm", 4, 0.12),
            ParamValue("wifi_ap_enable", 0, True),
            ParamValue("wifi_ap_channel", 1, 6),
            ParamValue("wifi_udp_port", 2, 2391),
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
        return 0

    def disarm(self) -> int:
        self._record("disarm")
        return 0

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

    def set_param(self, name: str, type_id: int, value):
        self._record("set_param", name, type_id, value)
        for index, item in enumerate(self._params):
            if item.name == name:
                cast_value = float(value) if type_id == 4 else int(value)
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

    def start_csv_log(self, output_path: Path) -> None:
        self._record("start_csv_log", output_path)
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


def test_udp_manual_setpoint_encodes_expected_payload():
    session = DeviceSession()
    transport = MockTransport()
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
                5,
                1,
                0,
                0,
                0x7F,
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
    assert info.protocol_version == 5
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


def test_liftoff_verify_analysis_accepts_conservative_closed_loop_attempt():
    samples = []
    for index in range(8):
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
        sample.baro_altitude_m = 0.02 * index
        samples.append(sample)

    result = analyze_liftoff_verify_samples(samples, base_duty=0.10)

    assert result["validity_ok"] is True
    assert result["safety_ok"] is True
    assert result["unified_path_ok"] is True
    assert result["chain_ok"] is True
    assert result["yaw_ok"] is True
    assert result["tilt_ok"] is True
    assert result["inner_motor_clamp_max"] == 0
    assert result["probable_liftoff"] is True
    assert result["passed"] is True


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
    assert result["passed"] is True


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
    assert window.params_table.rowCount() == 6
    window.link_type_combo.setCurrentIndex(window.link_type_combo.findData("udp"))
    app.processEvents()
    assert window.udp_host_edit.text() == "192.168.4.1"
    assert window.udp_port_spin.value() == 2391
    assert "ESP-DRONE" in window.udp_ap_info_label.text()
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
    assert "case CMD_UDP_MANUAL_ENABLE:" in dispatch
    assert "case MSG_UDP_MANUAL_SETPOINT:" in dispatch
    assert "CONTROL_MODE_UDP_MANUAL" in udp_manual
    assert "udp manual watchdog timeout" in udp_manual
    assert "ground_tune_capture_reference(&sample, &estimator_state)" in udp_manual
    assert "ground_tune_compute(&estimator_state, &rate_setpoint)" in app_main
    assert "udp_protocol_task" in udp_protocol
    assert "MSG_UDP_MANUAL_SETPOINT" in udp_protocol


def test_firmware_dispatch_registers_attitude_ground_verify_and_liftoff_paths():
    repo_root = Path(__file__).resolve().parents[3]
    protocol = (repo_root / "firmware" / "main" / "console" / "console_protocol.h").read_text(encoding="utf-8")
    dispatch = (repo_root / "firmware" / "main" / "console" / "console.c").read_text(encoding="utf-8")
    app_main = (repo_root / "firmware" / "main" / "app_main.c").read_text(encoding="utf-8")
    ground_tune = (repo_root / "firmware" / "main" / "ground_tune" / "ground_tune.c").read_text(encoding="utf-8")
    udp_protocol = (repo_root / "firmware" / "main" / "udp_protocol" / "udp_protocol.c").read_text(encoding="utf-8")

    assert "CONSOLE_PROTOCOL_VERSION 0x08u" in protocol
    assert "CONSOLE_FEATURE_ATTITUDE_GROUND_VERIFY" in protocol
    assert "CONSOLE_FEATURE_LOW_RISK_LIFTOFF_VERIFY" in protocol
    assert "CMD_ATTITUDE_GROUND_VERIFY_START = 22" in protocol
    assert "CMD_LIFTOFF_VERIFY_START = 24" in protocol
    assert "CMD_ATTITUDE_GROUND_SET_TARGET = 26" in protocol
    assert "case CMD_ATTITUDE_GROUND_VERIFY_START:" in dispatch
    assert "case CMD_LIFTOFF_VERIFY_START:" in dispatch
    assert "case CMD_ATTITUDE_GROUND_SET_TARGET:" in dispatch
    assert "case CMD_ATTITUDE_GROUND_VERIFY_START:" in udp_protocol
    assert "case CMD_LIFTOFF_VERIFY_START:" in udp_protocol
    assert "GROUND_TUNE_SUBMODE_ATTITUDE_VERIFY" in app_main
    assert "GROUND_TUNE_SUBMODE_LOW_RISK_LIFTOFF" in app_main
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
    assert "esp_netif_create_default_wifi_ap" in wifi_ap
    assert "esp_wifi_set_mode(WIFI_MODE_AP)" in wifi_ap
    assert "softap started ssid=" in wifi_ap
    assert "WIFI_EVENT_AP_STACONNECTED" in wifi_ap
    assert "INADDR_ANY" in udp_protocol
    assert "params_get()->wifi_udp_port" in udp_protocol


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
