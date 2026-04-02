from __future__ import annotations

import json
import queue
import threading
import time
from pathlib import Path
from typing import Callable

from .csv_log import CsvTelemetryLogger
from .models import (
    CMD_REQ_STRUCT,
    CMD_RESP_STRUCT,
    DeviceInfo,
    ParamSnapshot,
    ParamValue,
    TelemetrySample,
    decode_device_info,
    decode_event_text,
    decode_param_value,
    encode_param_value,
)
from .protocol.messages import CmdId, Frame, MsgType
from .transport.base import Transport
from .transport.serial_link import SerialTransport
from .transport.udp_link import UdpTransport


TelemetryCallback = Callable[[TelemetrySample], None]
EventCallback = Callable[[str], None]
ConnectionCallback = Callable[[dict[str, object]], None]


class DeviceSession:
    def __init__(self, transport: Transport | None = None) -> None:
        self._transport: Transport | None = None
        self._seq = 0
        self._command_lock = threading.Lock()
        self._response_queue: queue.Queue[Frame] = queue.Queue()
        self._event_queue: queue.Queue[str] = queue.Queue()
        self._stop_event = threading.Event()
        self._reader_thread: threading.Thread | None = None
        self._connected = False
        self._device_info: DeviceInfo | None = None
        self._latest_telemetry: TelemetrySample | None = None
        self._last_error: str | None = None
        self._last_log_path: Path | None = None
        self._csv_logger: CsvTelemetryLogger | None = None
        self._callback_lock = threading.Lock()
        self._next_callback_id = 1
        self._telemetry_callbacks: dict[int, TelemetryCallback] = {}
        self._event_callbacks: dict[int, EventCallback] = {}
        self._connection_callbacks: dict[int, ConnectionCallback] = {}

        if transport is not None:
            self.connect_transport(transport)

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFFFF
        return self._seq

    def _alloc_callback_id(self) -> int:
        with self._callback_lock:
            value = self._next_callback_id
            self._next_callback_id += 1
            return value

    def _snapshot_callbacks(self, mapping: dict[int, Callable]) -> list[Callable]:
        with self._callback_lock:
            return list(mapping.values())

    def subscribe_telemetry(self, callback: TelemetryCallback) -> int:
        callback_id = self._alloc_callback_id()
        with self._callback_lock:
            self._telemetry_callbacks[callback_id] = callback
        return callback_id

    def subscribe_event_log(self, callback: EventCallback) -> int:
        callback_id = self._alloc_callback_id()
        with self._callback_lock:
            self._event_callbacks[callback_id] = callback
        return callback_id

    def subscribe_connection_state(self, callback: ConnectionCallback) -> int:
        callback_id = self._alloc_callback_id()
        with self._callback_lock:
            self._connection_callbacks[callback_id] = callback
        return callback_id

    def unsubscribe(self, callback_id: int) -> None:
        with self._callback_lock:
            self._telemetry_callbacks.pop(callback_id, None)
            self._event_callbacks.pop(callback_id, None)
            self._connection_callbacks.pop(callback_id, None)

    def _notify_connection(self, *, connected: bool, error: str | None = None) -> None:
        payload = {
            "connected": connected,
            "device_info": self._device_info,
            "error": error,
        }
        for callback in self._snapshot_callbacks(self._connection_callbacks):
            callback(payload)

    def _notify_event(self, message: str) -> None:
        self._event_queue.put(message)
        for callback in self._snapshot_callbacks(self._event_callbacks):
            callback(message)

    def _notify_telemetry(self, sample: TelemetrySample) -> None:
        self._latest_telemetry = sample
        if self._csv_logger is not None:
            self._csv_logger.write(sample)
        for callback in self._snapshot_callbacks(self._telemetry_callbacks):
            callback(sample)

    def _send_message(self, msg_type: int, payload: bytes = b"", flags: int = 0) -> None:
        if self._transport is None:
            raise RuntimeError("device session is not connected")
        self._transport.send_message(msg_type, payload, flags=flags, seq=self._next_seq())

    def _recv_until(self, *msg_types: int, timeout: float = 1.0) -> Frame:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = max(0.01, deadline - time.monotonic())
            try:
                frame = self._response_queue.get(timeout=remaining)
            except queue.Empty:
                continue
            if frame.msg_type in msg_types:
                return frame
        raise TimeoutError(f"timed out waiting for {msg_types}")

    def _reader_loop(self) -> None:
        assert self._transport is not None
        while not self._stop_event.is_set():
            try:
                frame = self._transport.recv_frame(0.1)
            except TimeoutError:
                continue
            except Exception as exc:  # pragma: no cover - depends on transport implementation
                if self._stop_event.is_set():
                    break
                self._last_error = str(exc)
                self._connected = False
                self._notify_connection(connected=False, error=self._last_error)
                break

            if frame.msg_type == MsgType.TELEMETRY_SAMPLE:
                self._notify_telemetry(TelemetrySample.from_payload(frame.payload))
            elif frame.msg_type == MsgType.EVENT_LOG_TEXT:
                self._notify_event(decode_event_text(frame.payload))
            else:
                self._response_queue.put(frame)

    def _clear_queues(self) -> None:
        while not self._response_queue.empty():
            try:
                self._response_queue.get_nowait()
            except queue.Empty:
                break
        while not self._event_queue.empty():
            try:
                self._event_queue.get_nowait()
            except queue.Empty:
                break

    def connect_transport(self, transport: Transport, hello_timeout: float = 1.0) -> DeviceInfo:
        self.disconnect()
        self._transport = transport
        self._clear_queues()
        self._stop_event.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, name="esp-drone-rx", daemon=True)
        self._reader_thread.start()
        try:
            info = self.hello(timeout=hello_timeout)
        except Exception:
            self.disconnect()
            raise
        self._connected = True
        self._notify_connection(connected=True)
        return info

    def connect_serial(self, port: str, baudrate: int = 115200, timeout: float = 0.2) -> DeviceInfo:
        return self.connect_transport(SerialTransport(port=port, baudrate=baudrate, timeout=timeout))

    def connect_udp(self, host: str, port: int = 2391, timeout: float = 1.0) -> DeviceInfo:
        return self.connect_transport(UdpTransport(host=host, port=port, timeout=timeout))

    def disconnect(self, safe_stop_stream: bool = True) -> None:
        if self._transport is None:
            return

        if safe_stop_stream and self._connected:
            try:
                self.stop_stream(timeout=0.5)
            except Exception:
                pass

        self._stop_event.set()
        try:
            self._transport.close()
        except Exception:
            pass
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
        if self._csv_logger is not None:
            self._csv_logger.close()
            self._csv_logger = None
        self._transport = None
        self._connected = False
        self._device_info = None
        self._notify_connection(connected=False, error=None)

    def close(self) -> None:
        self.disconnect()

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def last_error(self) -> str | None:
        return self._last_error

    @property
    def last_log_path(self) -> Path | None:
        return self._last_log_path

    def get_latest_telemetry(self) -> TelemetrySample | None:
        return self._latest_telemetry

    def iter_event_log(self, timeout: float = 10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = max(0.01, deadline - time.monotonic())
            try:
                yield self._event_queue.get(timeout=remaining)
            except queue.Empty:
                continue

    def hello(self, timeout: float = 1.0) -> DeviceInfo:
        with self._command_lock:
            self._send_message(MsgType.HELLO_REQ)
            frame = self._recv_until(MsgType.HELLO_RESP, timeout=timeout)
            self._device_info = decode_device_info(frame.payload)
            return self._device_info

    def command(self, cmd_id: int, arg_u8: int = 0, arg_f32: float = 0.0, timeout: float = 1.0) -> int:
        with self._command_lock:
            payload = CMD_REQ_STRUCT.pack(cmd_id, arg_u8 & 0xFF, 0, float(arg_f32))
            self._send_message(MsgType.CMD_REQ, payload)
            frame = self._recv_until(MsgType.CMD_RESP, timeout=timeout)
            _, status, _ = CMD_RESP_STRUCT.unpack(frame.payload)
            return status

    def arm(self) -> int:
        return self.command(CmdId.ARM)

    def disarm(self) -> int:
        return self.command(CmdId.DISARM)

    def kill(self) -> int:
        return self.command(CmdId.KILL)

    def reboot(self) -> int:
        return self.command(CmdId.REBOOT)

    def motor_test(self, motor_index: int, duty: float) -> int:
        return self.command(CmdId.MOTOR_TEST, arg_u8=motor_index, arg_f32=duty)

    def axis_test(self, axis_index: int, value: float) -> int:
        return self.command(CmdId.AXIS_TEST, arg_u8=axis_index, arg_f32=value)

    def rate_test(self, axis_index: int, value_dps: float) -> int:
        return self.command(CmdId.RATE_TEST, arg_u8=axis_index, arg_f32=value_dps)

    def calib_gyro(self) -> int:
        return self.command(CmdId.CALIB_GYRO)

    def calib_level(self) -> int:
        return self.command(CmdId.CALIB_LEVEL)

    def send_rc(self, *_args, **_kwargs) -> None:
        raise NotImplementedError("send_rc is reserved in core but not implemented by the current firmware protocol")

    def start_stream(self, timeout: float = 1.0) -> None:
        with self._command_lock:
            self._send_message(MsgType.STREAM_CTRL, b"\x01")
            self._recv_until(MsgType.STREAM_CTRL, timeout=timeout)

    def stop_stream(self, timeout: float = 1.0) -> None:
        with self._command_lock:
            self._send_message(MsgType.STREAM_CTRL, b"\x00")
            self._recv_until(MsgType.STREAM_CTRL, timeout=timeout)

    def get_param(self, name: str, timeout: float = 1.0) -> ParamValue:
        with self._command_lock:
            name_bytes = name.encode("ascii")
            self._send_message(MsgType.PARAM_GET, bytes([len(name_bytes)]) + name_bytes)
            frame = self._recv_until(MsgType.PARAM_VALUE, timeout=timeout)
            return decode_param_value(frame.payload)

    def set_param_raw(self, name: str, type_id: int, value_bytes: bytes, timeout: float = 1.0) -> ParamValue:
        with self._command_lock:
            name_bytes = name.encode("ascii")
            payload = bytes([type_id, len(name_bytes)]) + name_bytes + value_bytes
            self._send_message(MsgType.PARAM_SET, payload)
            frame = self._recv_until(MsgType.PARAM_VALUE, timeout=timeout)
            return decode_param_value(frame.payload)

    def set_param(self, name: str, type_id: int, value: str | bytes) -> ParamValue:
        value_bytes = value if isinstance(value, bytes) else encode_param_value(type_id, value)
        return self.set_param_raw(name, type_id, value_bytes)

    def list_params(self, timeout: float = 1.0) -> list[ParamValue]:
        with self._command_lock:
            self._send_message(MsgType.PARAM_LIST_REQ)
            items: list[ParamValue] = []
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                frame = self._recv_until(MsgType.PARAM_VALUE, MsgType.PARAM_LIST_END, timeout=max(0.1, deadline - time.monotonic()))
                if frame.msg_type == MsgType.PARAM_LIST_END:
                    return items
                items.append(decode_param_value(frame.payload))
        raise TimeoutError("timed out waiting for parameter list end")

    def save_params(self, timeout: float = 1.0) -> None:
        with self._command_lock:
            self._send_message(MsgType.PARAM_SAVE)
            self._recv_until(MsgType.PARAM_SAVE, timeout=timeout)

    def reset_params(self, timeout: float = 1.0) -> None:
        with self._command_lock:
            self._send_message(MsgType.PARAM_RESET)
            self._recv_until(MsgType.PARAM_RESET, timeout=timeout)

    def export_params(self, output_path: Path) -> ParamSnapshot:
        info = self._device_info or self.hello()
        params = self.list_params(timeout=3.0)
        snapshot = ParamSnapshot(
            schema=1,
            firmware={
                "protocol_version": info.protocol_version,
                "imu_mode": info.imu_mode,
                "feature_bitmap": info.feature_bitmap,
            },
            params=[{"name": p.name, "type_id": p.type_id, "value": p.value} for p in params],
        )
        snapshot.write_json(output_path)
        return snapshot

    def import_params(self, input_path: Path, save_after: bool = False) -> list[ParamValue]:
        data = json.loads(input_path.read_text(encoding="utf-8"))
        applied: list[ParamValue] = []
        for item in data.get("params", []):
            applied.append(self.set_param(str(item["name"]), int(item["type_id"]), str(item["value"])))
        if save_after:
            self.save_params()
        return applied

    def start_csv_log(self, output_path: Path) -> None:
        if self._csv_logger is not None:
            self._csv_logger.close()
        self._csv_logger = CsvTelemetryLogger(output_path)
        self._last_log_path = output_path

    def stop_csv_log(self) -> Path | None:
        if self._csv_logger is None:
            return self._last_log_path
        self._csv_logger.close()
        self._csv_logger = None
        return self._last_log_path

    def dump_csv(self, output_path: Path, duration_s: float = 5.0) -> int:
        self.start_stream()
        self.start_csv_log(output_path)
        deadline = time.monotonic() + duration_s
        start_rows = self._csv_logger.rows_written if self._csv_logger is not None else 0
        while time.monotonic() < deadline:
            time.sleep(0.05)
        logger = self._csv_logger
        rows = 0 if logger is None else logger.rows_written - start_rows
        self.stop_csv_log()
        return rows
