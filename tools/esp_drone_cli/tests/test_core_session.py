from __future__ import annotations

import importlib.util
import json
import queue
import time
from pathlib import Path

from esp_drone_cli.core import DeviceSession
from esp_drone_cli.core.models import HELLO_RESP_STRUCT, TELEMETRY_STRUCT
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
    ]
    return TELEMETRY_STRUCT.pack(*values)


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
            hello = HELLO_RESP_STRUCT.pack(1, 1, 0, 0, 0xF)
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

    def recv_frame(self, timeout: float) -> Frame:
        try:
            return self._frames.get(timeout=timeout)
        except queue.Empty as exc:
            raise TimeoutError("mock timeout") from exc

    def inject(self, frame: Frame) -> None:
        self._frames.put(frame)

    def close(self) -> None:
        self.closed = True


def test_device_session_mock_roundtrip(tmp_path: Path):
    session = DeviceSession()
    transport = MockTransport()
    info = session.connect_transport(transport)
    assert info.protocol_version == 1
    assert session.arm() == 0
    assert session.get_param("demo").value == 2.5

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


def test_gui_startup_without_device_or_missing_pyside6(monkeypatch):
    if importlib.util.find_spec("PySide6") is None:
        from esp_drone_cli import gui_main

        assert gui_main.main([]) == 1
        return

    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication

    from esp_drone_cli.gui.main_window import MainWindow

    app = QApplication.instance() or QApplication([])
    window = MainWindow()
    window.close()
    app.quit()


def test_cli_parser_compatibility_without_gui_dependency():
    from esp_drone_cli.cli.main import build_parser

    args = build_parser().parse_args(["--serial", "COM7", "rate-test", "yaw", "30"])
    assert args.serial == "COM7"
    assert args.command == "rate-test"
    assert args.axis == "yaw"
