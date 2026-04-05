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
import sys
import time
from pathlib import Path

import pytest

from esp_drone_cli.core import DeviceSession
from esp_drone_cli.core.models import HELLO_RESP_STRUCT, ParamSnapshot, ParamValue, TELEMETRY_STRUCT, TelemetrySample
from esp_drone_cli.core.protocol.framing import encode_serial_packet
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


class FakeSession:
    def __init__(self) -> None:
        self.calls: list[tuple[str, tuple, dict]] = []
        self.is_connected = False
        self.last_log_path: Path | None = None
        self._telemetry_callbacks = []
        self._event_callbacks = []
        self._connection_callbacks = []
        self._params = [
            ParamValue("alpha", 2, 42),
            ParamValue("beta", 4, 1.5),
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
        self._emit_connection()
        return "fake-device"

    def connect_udp(self, host: str, port: int = 2391, timeout: float = 1.0):
        self._record("connect_udp", host, port, timeout)
        self.is_connected = True
        self._emit_connection()
        return "fake-device"

    def disconnect(self, safe_stop_stream: bool = True) -> None:
        self._record("disconnect", safe_stop_stream)
        self.is_connected = False
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


def test_device_session_mock_roundtrip(tmp_path: Path):
    session = DeviceSession()
    transport = MockTransport()
    info = session.connect_transport(transport)
    assert info.protocol_version == 1
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
    assert window.left_panel.minimumWidth() >= 320
    assert window.center_panel.minimumWidth() >= 780
    assert window.right_panel.minimumWidth() >= 360
    assert window.left_panel.sizePolicy().horizontalPolicy() != QSizePolicy.Fixed
    assert window.left_panel.verticalScrollBarPolicy() == Qt.ScrollBarAsNeeded
    assert window.connection_section.is_expanded() is True
    assert window.safety_section.is_expanded() is True
    assert window.debug_action_tabs.count() == 2
    assert window.debug_action_tabs.tabText(0) == window._t("tab.motor")
    assert window.debug_action_tabs.tabText(1) == window._t("tab.rate")
    assert window.debug_action_tabs.currentIndex() == 0
    assert window.params_table.rowCount() == 2
    session.calls.clear()

    sample = TelemetrySample.from_payload(build_telemetry_payload())
    session.emit_telemetry(sample)
    session.emit_event("imu healthy")
    app.processEvents()
    assert window.telemetry_table.item(0, 1).text() == "1.000"
    assert window.status_cards["arm_state"][1].text() == window._t("arm.disarmed")
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
    assert "start_csv_log" in call_names
    assert "stop_csv_log" in call_names
    assert "dump_csv" in call_names
    assert ("connect_udp", ("192.168.4.1", 2391, 1.0), {}) in session.calls
    assert "disconnect" in call_names
    assert window.last_log_path_label.text().endswith("telemetry.csv")

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
