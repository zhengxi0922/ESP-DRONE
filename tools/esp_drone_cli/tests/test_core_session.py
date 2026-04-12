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
import socket
import sys
import threading
import time
from pathlib import Path

import pytest

from esp_drone_cli.core import DeviceSession
from esp_drone_cli.cli.main import format_rate_status_line
from esp_drone_cli.core.models import (
    CMD_REQ_STRUCT,
    CapabilityError,
    DeviceInfo,
    FEATURE_ATTITUDE_HANG_BENCH,
    HELLO_RESP_STRUCT,
    HELLO_RESP_STRUCT_V2,
    ParamSnapshot,
    ParamValue,
    TELEMETRY_STRUCT,
    TELEMETRY_STRUCT_V1,
    TELEMETRY_STRUCT_V3,
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

    def connect_serial(self, port: str, baudrate: int = 115200, timeout: float = 0.2):
        self._record("connect_serial", port, baudrate, timeout)
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
    assert ("connect_serial", ("COM9", 115200, 0.2), {}) in session.calls
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
    assert window.debug_action_tabs.tabText(2) == "Hang Attitude"
    assert window.debug_action_tabs.tabText(3) == window._t("tab.udp_control")
    assert window.debug_action_tabs.currentIndex() == 0
    assert window.params_table.rowCount() == 3
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
    assert "start_csv_log" in call_names
    assert "stop_csv_log" in call_names
    assert "dump_csv" in call_names
    assert ("connect_udp", ("192.168.4.1", 2391, 1.0), {}) in session.calls
    assert "disconnect" in call_names
    assert window.last_log_path_label.text().endswith("telemetry.csv")

    window.close()
    app.processEvents()


@pytest.mark.skipif(importlib.util.find_spec("PyQt5") is None or importlib.util.find_spec("pyqtgraph") is None, reason="PyQt5/pyqtgraph not installed")
def test_gui_connect_failure_shows_error_and_restores_inputs(monkeypatch, tmp_path: Path):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5.QtCore import QSettings
    from PyQt5.QtWidgets import QApplication

    from esp_drone_cli.gui.main_window import MainWindow, QtSessionBridge

    class FailingSession(FakeSession):
        def connect_serial(self, port: str, baudrate: int = 115200, timeout: float = 0.2):
            self._record("connect_serial", port, baudrate, timeout)
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

    assert ("connect_serial", ("COM404", 115200, 0.2), {}) in session.calls
    assert window.connection_status_chip.text() == window._t("status.disconnected")
    assert window.connect_button.isEnabled()
    assert window.serial_port_combo.currentText() == "COM404"
    assert "Connect failed: HELLO timeout" in window.connection_error_detail.text()
    assert "Connect failed: HELLO timeout" in window.event_log_edit.toPlainText()

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
    udp_protocol = (repo_root / "firmware" / "main" / "udp_protocol" / "udp_protocol.c").read_text(encoding="utf-8")

    assert "CMD_UDP_MANUAL_ENABLE = 13" in protocol
    assert "MSG_UDP_MANUAL_SETPOINT = 0x50" in protocol
    assert "CONSOLE_FEATURE_UDP_MANUAL_CONTROL" in protocol
    assert "case CMD_UDP_MANUAL_ENABLE:" in dispatch
    assert "case MSG_UDP_MANUAL_SETPOINT:" in dispatch
    assert "CONTROL_MODE_UDP_MANUAL" in udp_manual
    assert "udp manual watchdog timeout" in udp_manual
    assert "udp_protocol_task" in udp_protocol
    assert "MSG_UDP_MANUAL_SETPOINT" in udp_protocol


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
    assert serial_instances[0].kwargs == {"port": "COM7", "baudrate": 115200, "timeout": 0.2}
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
