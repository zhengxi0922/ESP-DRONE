"""设备会话抽象。

`DeviceSession` 统一封装传输层、协议握手、命令访问、参数读写和
遥测/事件分发。CLI 与 GUI 都应通过该对象访问设备，避免重复维护
各自的连接状态与协议细节。
"""

from __future__ import annotations

import json
import math
import queue
import threading
import time
from collections import deque
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
    UDP_MANUAL_SETPOINT_STRUCT,
    coerce_param_value,
    decode_device_info,
    decode_event_text,
    decode_param_value,
    encode_param_value,
)
from .protocol.messages import CmdId, Frame, MsgType
from .transport.base import Transport
from .transport.serial_link import SerialTransport
from .transport.udp_link import UdpTransport

DEFAULT_RESPONSE_TIMEOUT_S = 2.5
ATTITUDE_GROUND_TARGET_TIMEOUT_S = 8.0


TelemetryCallback = Callable[[TelemetrySample], None]
EventCallback = Callable[[str], None]
ConnectionCallback = Callable[[dict[str, object]], None]


def _param_values_match(type_id: int, expected: object, actual: object) -> bool:
    if type_id == 4:
        return abs(float(expected) - float(actual)) <= 1e-6
    return expected == actual


class DeviceSession:
    """封装单设备连接、命令收发和回调分发。

    该类内部维护一个后台接收线程用于消费传输层数据，并通过命令锁保证
    请求-响应类操作按顺序执行。公开回调可能在接收线程或当前调用线程中触发，
    调用方不应直接假定其运行在线程安全的 UI 上下文中。
    """

    def __init__(self, transport: Transport | None = None) -> None:
        """初始化会话对象。

        Args:
            transport: 可选的已构造传输对象。提供时会立即调用
                `connect_transport()` 完成握手。

        Raises:
            Exception: 传输建立或握手失败时，透传底层异常。
        """

        self._transport: Transport | None = None
        self._seq = 0
        self._command_lock = threading.Lock()
        self._response_queue: queue.Queue[Frame] = queue.Queue()
        self._pending_response_frames: deque[Frame] = deque()
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
        """订阅遥测样本回调。

        Args:
            callback: 每次收到 `TelemetrySample` 时调用的回调。回调可能在接收线程中执行。

        Returns:
            用于后续 `unsubscribe()` 的订阅标识。
        """

        callback_id = self._alloc_callback_id()
        with self._callback_lock:
            self._telemetry_callbacks[callback_id] = callback
        return callback_id

    def subscribe_event_log(self, callback: EventCallback) -> int:
        """订阅事件日志回调。

        Args:
            callback: 每次收到文本事件时调用的回调。回调可能在接收线程中执行。

        Returns:
            用于后续 `unsubscribe()` 的订阅标识。
        """

        callback_id = self._alloc_callback_id()
        with self._callback_lock:
            self._event_callbacks[callback_id] = callback
        return callback_id

    def subscribe_connection_state(self, callback: ConnectionCallback) -> int:
        """订阅连接状态变化。

        Args:
            callback: 连接状态回调。回调参数为包含 `connected`、`device_info`
                和 `error` 的字典。

        Returns:
            用于后续 `unsubscribe()` 的订阅标识。
        """

        callback_id = self._alloc_callback_id()
        with self._callback_lock:
            self._connection_callbacks[callback_id] = callback
        return callback_id

    def unsubscribe(self, callback_id: int) -> None:
        """取消订阅回调。

        Args:
            callback_id: 由订阅方法返回的标识。

        Note:
            当标识不存在时，本方法静默返回。
        """

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

    def _restore_deferred_response_frames(self, frames: list[Frame]) -> None:
        for frame in reversed(frames):
            self._pending_response_frames.appendleft(frame)

    def _recv_matching_response(
        self,
        predicate: Callable[[Frame], bool],
        *,
        timeout: float,
        timeout_message: str,
        drop_unmatched: Callable[[Frame], bool] | None = None,
    ) -> Frame:
        deadline = time.monotonic() + timeout
        deferred: list[Frame] = []
        while time.monotonic() < deadline:
            while self._pending_response_frames:
                frame = self._pending_response_frames.popleft()
                if predicate(frame):
                    self._restore_deferred_response_frames(deferred)
                    return frame
                if drop_unmatched is not None and drop_unmatched(frame):
                    continue
                deferred.append(frame)

            remaining = max(0.01, deadline - time.monotonic())
            try:
                frame = self._response_queue.get(timeout=remaining)
            except queue.Empty:
                continue
            if predicate(frame):
                self._restore_deferred_response_frames(deferred)
                return frame
            if drop_unmatched is not None and drop_unmatched(frame):
                continue
            deferred.append(frame)
        self._restore_deferred_response_frames(deferred)
        raise TimeoutError(timeout_message)

    def _recv_until(self, *msg_types: int, timeout: float = DEFAULT_RESPONSE_TIMEOUT_S) -> Frame:
        return self._recv_matching_response(
            lambda frame: frame.msg_type in msg_types,
            timeout=timeout,
            timeout_message=f"timed out waiting for {msg_types}",
        )

    def _recv_command_response(self, cmd_id: int, timeout: float = DEFAULT_RESPONSE_TIMEOUT_S) -> int:
        def matches(frame: Frame) -> bool:
            if frame.msg_type != MsgType.CMD_RESP:
                return False
            if len(frame.payload) != CMD_RESP_STRUCT.size:
                raise ValueError("CMD_RESP payload has unexpected length")
            response_cmd_id, status, _ = CMD_RESP_STRUCT.unpack(frame.payload)
            return response_cmd_id == cmd_id

        def drop_stale(frame: Frame) -> bool:
            return frame.msg_type == MsgType.CMD_RESP

        frame = self._recv_matching_response(
            matches,
            timeout=timeout,
            timeout_message=f"timed out waiting for command response {cmd_id}",
            drop_unmatched=drop_stale,
        )
        _response_cmd_id, status, _ = CMD_RESP_STRUCT.unpack(frame.payload)
        return status

    def _recv_stream_ack(self, enabled: bool, timeout: float = DEFAULT_RESPONSE_TIMEOUT_S) -> None:
        expected_value = 1 if enabled else 0

        def matches(frame: Frame) -> bool:
            return frame.msg_type == MsgType.STREAM_CTRL and len(frame.payload) > 0 and frame.payload[0] == expected_value

        def drop_stale(frame: Frame) -> bool:
            return frame.msg_type == MsgType.STREAM_CTRL

        self._recv_matching_response(
            matches,
            timeout=timeout,
            timeout_message=f"timed out waiting for stream {'on' if enabled else 'off'} acknowledgement",
            drop_unmatched=drop_stale,
        )

    def _recv_param_value(self, name: str, timeout: float = DEFAULT_RESPONSE_TIMEOUT_S) -> ParamValue:
        decoded_match: ParamValue | None = None

        def matches(frame: Frame) -> bool:
            nonlocal decoded_match
            if frame.msg_type != MsgType.PARAM_VALUE:
                return False
            value = decode_param_value(frame.payload)
            if value.name != name:
                return False
            decoded_match = value
            return True

        def drop_stale(frame: Frame) -> bool:
            return frame.msg_type == MsgType.PARAM_VALUE

        self._recv_matching_response(
            matches,
            timeout=timeout,
            timeout_message=f"timed out waiting for parameter value {name!r}",
            drop_unmatched=drop_stale,
        )
        assert decoded_match is not None
        return decoded_match

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
        self._pending_response_frames.clear()
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

    def connect_transport(
        self,
        transport: Transport,
        hello_timeout: float = 1.0,
        *,
        notify_failure: bool = True,
    ) -> DeviceInfo:
        """接入一个已构造的传输对象并完成握手。

        Args:
            transport: 可直接收发协议帧的传输对象。
            hello_timeout: `HELLO` 握手超时，单位为秒。

        Returns:
            设备返回的握手信息。

        Raises:
            TimeoutError: 握手阶段在限定时间内未收到响应。
            Exception: 传输层打开、收发或解码失败时透传底层异常。
        """

        self.disconnect()
        self._transport = transport
        self._clear_queues()
        self._stop_event.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, name="esp-drone-rx", daemon=True)
        self._reader_thread.start()
        try:
            info = self.hello(timeout=hello_timeout)
        except Exception as exc:
            self._last_error = str(exc)
            self._teardown_transport(
                safe_stop_stream=False,
                notify=notify_failure,
                error=self._last_error,
            )
            raise
        self._connected = True
        self._last_error = None
        self._notify_connection(connected=True)
        return info

    def connect_serial(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 0.2,
        hello_timeout: float = 4.0,
        open_retry_timeout_s: float = 5.0,
    ) -> DeviceInfo:
        """通过串口建立连接。

        Args:
            port: 串口名，例如 `COM5`。
            baudrate: 波特率。
            timeout: 底层串口读超时，单位为秒。
            hello_timeout: 协议握手超时，单位为秒。

        Returns:
            设备返回的握手信息。

        Raises:
            Exception: 串口打开、握手或协议收发失败时透传底层异常。
        """

        last_timeout: TimeoutError | None = None
        for attempt in range(2):
            try:
                transport = SerialTransport(
                    port=port,
                    baudrate=baudrate,
                    timeout=timeout,
                    open_retry_timeout_s=open_retry_timeout_s,
                )
            except Exception as exc:
                self._last_error = str(exc)
                self._notify_connection(connected=False, error=self._last_error)
                raise

            try:
                return self.connect_transport(
                    transport,
                    hello_timeout=hello_timeout,
                    notify_failure=attempt == 1,
                )
            except TimeoutError as exc:
                last_timeout = exc
                if attempt == 0:
                    continue
                raise

        assert last_timeout is not None
        raise last_timeout

    def connect_udp(self, host: str, port: int = 2391, timeout: float = 1.0) -> DeviceInfo:
        """通过 UDP 建立连接。

        Args:
            host: 设备主机地址。
            port: 设备监听端口。
            timeout: 底层 UDP 接收超时，单位为秒。

        Returns:
            设备返回的握手信息。

        Raises:
            Exception: UDP 套接字创建、握手或协议收发失败时透传底层异常。
        """

        return self.connect_transport(UdpTransport(host=host, port=port, timeout=timeout))

    def _teardown_transport(
        self,
        *,
        safe_stop_stream: bool,
        notify: bool,
        error: str | None,
    ) -> None:
        """Close transport resources while optionally preserving a connection error."""

        if self._transport is None:
            return

        if safe_stop_stream and self._connected:
            try:
                self.stop_stream(timeout=0.5)
            except Exception:
                pass

        if self._reader_thread is not None and threading.current_thread() is not self._reader_thread:
            self._stop_event.set()
            self._reader_thread.join(timeout=0.5)
        else:
            self._stop_event.set()

        try:
            self._transport.close()
        except Exception:
            pass
        if self._reader_thread is not None and threading.current_thread() is not self._reader_thread:
            self._reader_thread.join(timeout=0.5)
        if self._csv_logger is not None:
            self._csv_logger.close()
            self._csv_logger = None
        self._transport = None
        self._reader_thread = None
        self._connected = False
        self._device_info = None
        if error is not None:
            self._last_error = error
        if notify:
            self._notify_connection(connected=False, error=error)

    def disconnect(self, safe_stop_stream: bool = True) -> None:
        """断开当前连接并释放后台资源。

        Args:
            safe_stop_stream: 为 `True` 时会在已连接状态下尽力发送一次停流命令，
                以避免设备继续高速推送遥测。

        Note:
            本方法会吞掉关闭过程中的清理异常，适合在 CLI/GUI 退出路径中调用。
        """

        self._last_error = None
        self._teardown_transport(
            safe_stop_stream=safe_stop_stream,
            notify=True,
            error=None,
        )

    def close(self) -> None:
        """`disconnect()` 的别名，便于统一释放接口。"""

        self.disconnect()

    @property
    def is_connected(self) -> bool:
        """返回当前是否已完成握手并保持连接。"""

        return self._connected

    @property
    def last_error(self) -> str | None:
        """返回最近一次连接或后台接收错误。"""

        return self._last_error

    @property
    def device_info(self) -> DeviceInfo | None:
        """返回最近一次握手得到的设备信息。"""

        return self._device_info

    @property
    def last_log_path(self) -> Path | None:
        """返回最近一次 CSV 日志输出路径。"""

        return self._last_log_path

    def get_latest_telemetry(self) -> TelemetrySample | None:
        """返回最近一次收到的遥测样本。"""

        return self._latest_telemetry

    def iter_event_log(self, timeout: float = 10.0):
        """在限定时间窗口内迭代事件日志。

        Args:
            timeout: 总等待时长，单位为秒。超时后生成器自然结束。

        Yields:
            设备上报的事件文本。
        """

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = max(0.01, deadline - time.monotonic())
            try:
                yield self._event_queue.get(timeout=remaining)
            except queue.Empty:
                continue

    def hello(self, timeout: float = DEFAULT_RESPONSE_TIMEOUT_S) -> DeviceInfo:
        """发送握手请求并刷新 `device_info`。

        Args:
            timeout: 等待响应的超时，单位为秒。

        Returns:
            设备返回的握手信息。

        Raises:
            RuntimeError: 当前尚未接入传输对象。
            TimeoutError: 在限定时间内未收到握手响应。
            ValueError: 返回帧格式不符合当前协议结构。
        """

        with self._command_lock:
            self._send_message(MsgType.HELLO_REQ)
            frame = self._recv_until(MsgType.HELLO_RESP, timeout=timeout)
            self._device_info = decode_device_info(frame.payload)
            return self._device_info

    def command(self, cmd_id: int, arg_u8: int = 0, arg_f32: float = 0.0, timeout: float = DEFAULT_RESPONSE_TIMEOUT_S) -> int:
        """发送通用命令请求并返回设备状态码。

        Args:
            cmd_id: 协议命令编号。
            arg_u8: 8 位无符号参数，仅保留最低 8 位。
            arg_f32: 浮点参数，按协议以 `float32` 编码。
            timeout: 等待响应的超时，单位为秒。

        Returns:
            固件在 `CMD_RESP` 中返回的状态码。

        Raises:
            RuntimeError: 当前尚未接入传输对象。
            TimeoutError: 在限定时间内未收到命令响应。
            ValueError: 返回帧载荷长度与协议结构不一致。
        """

        with self._command_lock:
            payload = CMD_REQ_STRUCT.pack(cmd_id, arg_u8 & 0xFF, 0, float(arg_f32))
            self._send_message(MsgType.CMD_REQ, payload)
            return self._recv_command_response(cmd_id, timeout=timeout)

    def require_attitude_hang_bench(self) -> None:
        """Fail before sending hang-attitude commands to firmware without the advertised capability."""

        info = self._device_info or self.hello()
        info.require_attitude_hang_bench()

    def require_udp_manual_control(self) -> None:
        """Fail before sending experimental UDP manual commands to unsupported firmware."""

        info = self._device_info or self.hello()
        info.require_udp_manual_control()

    def require_ground_tune(self) -> None:
        """Fail before sending flat-ground tune commands to unsupported firmware."""

        info = self._device_info or self.hello()
        info.require_ground_tune()

    def require_attitude_ground_verify(self) -> None:
        """Fail before sending flat-ground attitude verify commands to unsupported firmware."""

        info = self._device_info or self.hello()
        info.require_attitude_ground_verify()

    def require_low_risk_liftoff_verify(self) -> None:
        """Fail before sending low-risk liftoff verify commands to unsupported firmware."""

        info = self._device_info or self.hello()
        info.require_low_risk_liftoff_verify()

    def require_all_motor_test(self) -> None:
        """Fail before sending independent all-motor test commands to unsupported firmware."""

        info = self._device_info or self.hello()
        info.require_all_motor_test()

    def arm(self) -> int:
        """请求设备解锁。"""

        return self.command(CmdId.ARM)

    def disarm(self) -> int:
        """请求设备上锁。"""

        return self.command(CmdId.DISARM)

    def kill(self) -> int:
        """请求设备立即进入急停。"""

        return self.command(CmdId.KILL)

    def reboot(self) -> int:
        """请求设备重启。"""

        return self.command(CmdId.REBOOT)

    def motor_test(self, motor_index: int, duty: float) -> int:
        """执行单电机点动测试。

        Args:
            motor_index: 电机索引，按当前 CLI 约定为 0 基编号。
            duty: 传给固件的测试输出值。本层不做范围检查，最终约束由固件决定。

        Returns:
            设备返回的命令状态码。

        Note:
            该接口面向台架调试，不应在未固定机体的情况下调用。
        """

        return self.command(CmdId.MOTOR_TEST, arg_u8=motor_index, arg_f32=duty)

    def all_motor_test_start(self, duty: float, duration_s: float) -> int:
        """Start the independent equal-duty all-motor test path."""

        self.require_all_motor_test()
        duration_ticks = int(math.ceil(float(duration_s) * 10.0 - 1e-9))
        return self.command(CmdId.ALL_MOTOR_TEST_START, arg_u8=duration_ticks, arg_f32=float(duty))

    def all_motor_test_stop(self) -> int:
        """Stop the independent equal-duty all-motor test path."""

        self.require_all_motor_test()
        return self.command(CmdId.ALL_MOTOR_TEST_STOP)

    def axis_test(self, axis_index: int, value: float) -> int:
        """执行开环轴向测试。

        Args:
            axis_index: 轴编号。当前上层调用约定为 `0=roll`、`1=pitch`、`2=yaw`。
            value: 传给固件的测试值，本层不做范围检查。

        Returns:
            设备返回的命令状态码。
        """

        return self.command(CmdId.AXIS_TEST, arg_u8=axis_index, arg_f32=value)

    def rate_test(self, axis_index: int, value_dps: float) -> int:
        """执行速率环测试。

        Args:
            axis_index: 轴编号。当前上层调用约定为 `0=roll`、`1=pitch`、`2=yaw`。
            value_dps: 目标角速度，单位为度每秒。本层不做范围检查。

        Returns:
            设备返回的命令状态码。
        """

        return self.command(CmdId.RATE_TEST, arg_u8=axis_index, arg_f32=value_dps)

    def attitude_capture_ref(self) -> int:
        """Capture the current natural hanging attitude as the bench reference."""

        self.require_attitude_hang_bench()
        return self.command(CmdId.ATTITUDE_CAPTURE_REF)

    def attitude_test_start(self) -> int:
        """Start the bench-only hang-attitude outer loop."""

        self.require_attitude_hang_bench()
        return self.command(CmdId.ATTITUDE_TEST_START)

    def attitude_test_stop(self) -> int:
        """Stop the bench-only hang-attitude outer loop."""

        self.require_attitude_hang_bench()
        return self.command(CmdId.ATTITUDE_TEST_STOP)

    def ground_capture_ref(self) -> int:
        """Capture the current flat-ground attitude as the ground tune reference."""

        self.require_ground_tune()
        return self.command(CmdId.GROUND_CAPTURE_REF)

    def ground_test_start(self, base_duty: float | None = None) -> int:
        """Start the flat-ground low-throttle tune loop."""

        self.require_ground_tune()
        return self.command(CmdId.GROUND_TEST_START, arg_f32=0.0 if base_duty is None else float(base_duty))

    def ground_test_stop(self) -> int:
        """Stop the flat-ground low-throttle tune loop."""

        self.require_ground_tune()
        return self.command(CmdId.GROUND_TEST_STOP)

    def attitude_ground_verify_start(self, base_duty: float | None = None) -> int:
        """Start the flat-ground attitude outer-loop verification path."""

        self.require_attitude_ground_verify()
        return self.command(CmdId.ATTITUDE_GROUND_VERIFY_START, arg_f32=0.0 if base_duty is None else float(base_duty))

    def attitude_ground_verify_stop(self) -> int:
        """Stop the flat-ground attitude outer-loop verification path."""

        self.require_attitude_ground_verify()
        return self.command(CmdId.ATTITUDE_GROUND_VERIFY_STOP)

    def attitude_ground_set_target(
        self,
        axis_index: int,
        target_deg: float,
        timeout: float = ATTITUDE_GROUND_TARGET_TIMEOUT_S,
    ) -> int:
        """Set one small flat-ground attitude verification angle target."""

        self.require_attitude_ground_verify()
        return self.command(
            CmdId.ATTITUDE_GROUND_SET_TARGET,
            arg_u8=axis_index,
            arg_f32=float(target_deg),
            timeout=timeout,
        )

    def liftoff_verify_start(self, base_duty: float | None = None) -> int:
        """Start the explicit low-risk liftoff verification path."""

        self.require_low_risk_liftoff_verify()
        return self.command(CmdId.LIFTOFF_VERIFY_START, arg_f32=0.0 if base_duty is None else float(base_duty))

    def liftoff_verify_stop(self) -> int:
        """Stop the explicit low-risk liftoff verification path."""

        self.require_low_risk_liftoff_verify()
        return self.command(CmdId.LIFTOFF_VERIFY_STOP)

    def udp_manual_enable(self) -> int:
        """Enter experimental UDP manual mode."""

        self.require_udp_manual_control()
        return self.command(CmdId.UDP_MANUAL_ENABLE)

    def udp_manual_disable(self) -> int:
        """Disable experimental UDP manual mode and request disarm."""

        self.require_udp_manual_control()
        return self.command(CmdId.UDP_MANUAL_DISABLE)

    def udp_manual_stop(self) -> int:
        """Send the explicit experimental UDP manual stop command."""

        self.require_udp_manual_control()
        return self.command(CmdId.UDP_MANUAL_STOP)

    def udp_takeoff(self) -> int:
        """Request an experimental UDP takeoff base-duty ramp with attitude roll/pitch and rate-PID yaw."""

        self.require_udp_manual_control()
        return self.command(CmdId.UDP_TAKEOFF)

    def udp_land(self) -> int:
        """Request an experimental UDP landing base-duty ramp."""

        self.require_udp_manual_control()
        return self.command(CmdId.UDP_LAND)

    def udp_manual_setpoint(
        self,
        throttle: float,
        pitch: float,
        roll: float,
        yaw: float,
        timeout: float = 1.0,
    ) -> int:
        """Send one experimental UDP manual setpoint frame.

        Throttle is a normalized base-duty target. Pitch/roll fields remain in
        the protocol, but firmware holds roll/pitch through the attitude outer
        loop and uses yaw as the manual rate request. Firmware remains
        authoritative for clamping.
        """

        self.require_udp_manual_control()
        values = (float(throttle), float(pitch), float(roll), float(yaw))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("udp manual setpoint values must be finite")
        with self._command_lock:
            self._send_message(MsgType.UDP_MANUAL_SETPOINT, UDP_MANUAL_SETPOINT_STRUCT.pack(*values))
            return self._recv_command_response(CmdId.UDP_MANUAL_SETPOINT, timeout=timeout)

    def calib_gyro(self) -> int:
        """请求执行陀螺仪校准。"""

        return self.command(CmdId.CALIB_GYRO)

    def calib_level(self) -> int:
        """请求执行水平校准。调用前应保证机体静止且水平。"""

        return self.command(CmdId.CALIB_LEVEL)

    def send_rc(self, *_args, **_kwargs) -> None:
        """保留 RC 发送接口。

        Raises:
            NotImplementedError: 当前固件协议未实现该能力。
        """

        raise NotImplementedError("send_rc is reserved in core but not implemented by the current firmware protocol")

    def start_stream(self, timeout: float = DEFAULT_RESPONSE_TIMEOUT_S) -> None:
        """请求设备开始推送遥测流。

        Args:
            timeout: 等待设备确认的超时，单位为秒。

        Raises:
            RuntimeError: 当前尚未接入传输对象。
            TimeoutError: 在限定时间内未收到流控确认。
        """

        with self._command_lock:
            self._send_message(MsgType.STREAM_CTRL, b"\x01")
            self._recv_stream_ack(True, timeout=timeout)

    def stop_stream(self, timeout: float = DEFAULT_RESPONSE_TIMEOUT_S) -> None:
        """请求设备停止推送遥测流。

        Args:
            timeout: 等待设备确认的超时，单位为秒。

        Raises:
            RuntimeError: 当前尚未接入传输对象。
            TimeoutError: 在限定时间内未收到流控确认。
        """

        with self._command_lock:
            self._send_message(MsgType.STREAM_CTRL, b"\x00")
            self._recv_stream_ack(False, timeout=timeout)

    def get_param(self, name: str, timeout: float = DEFAULT_RESPONSE_TIMEOUT_S) -> ParamValue:
        """读取单个参数。

        Args:
            name: 参数名，必须可编码为 ASCII。
            timeout: 等待参数值响应的超时，单位为秒。

        Returns:
            固件返回的参数对象。

        Raises:
            UnicodeEncodeError: 参数名包含非 ASCII 字符时抛出。
            RuntimeError: 当前尚未接入传输对象。
            TimeoutError: 在限定时间内未收到参数响应。
            ValueError: 返回载荷不符合参数编码格式。
        """

        with self._command_lock:
            name_bytes = name.encode("ascii")
            self._send_message(MsgType.PARAM_GET, bytes([len(name_bytes)]) + name_bytes)
            return self._recv_param_value(name, timeout=timeout)

    def set_param_raw(self, name: str, type_id: int, value_bytes: bytes, timeout: float = DEFAULT_RESPONSE_TIMEOUT_S) -> ParamValue:
        """按原始字节写入参数。

        Args:
            name: 参数名，必须可编码为 ASCII。
            type_id: 协议参数类型编号。
            value_bytes: 已按协议编码的参数值字节串。
            timeout: 等待写入回显的超时，单位为秒。

        Returns:
            固件回显的参数对象。

        Raises:
            UnicodeEncodeError: 参数名包含非 ASCII 字符时抛出。
            RuntimeError: 当前尚未接入传输对象。
            TimeoutError: 在限定时间内未收到参数响应。
            ValueError: 返回载荷不符合参数编码格式。
        """

        with self._command_lock:
            name_bytes = name.encode("ascii")
            payload = bytes([type_id, len(name_bytes)]) + name_bytes + value_bytes
            self._send_message(MsgType.PARAM_SET, payload)
            return self._recv_param_value(name, timeout=timeout)

    def set_param(self, name: str, type_id: int, value: object) -> ParamValue:
        """将文本值编码后写入参数。

        Args:
            name: 参数名，必须可编码为 ASCII。
            type_id: 协议参数类型编号。
            value: 待写入的值。传入 `bytes` 时会直接透传，不再二次编码。

        Returns:
            固件回显的参数对象。

        Raises:
            ValueError: `type_id` 不受支持或 `value` 无法按目标类型编码时抛出。
            UnicodeEncodeError: 参数名包含非 ASCII 字符时抛出。
            TimeoutError: 在限定时间内未收到参数响应。
        """

        value_bytes = value if isinstance(value, bytes) else encode_param_value(type_id, str(value))
        result = self.set_param_raw(name, type_id, value_bytes)
        if isinstance(value, bytes):
            return result

        expected_value = coerce_param_value(type_id, value)
        if result.type_id != type_id or not _param_values_match(type_id, expected_value, result.value):
            raise RuntimeError(
                f"set_param rejected by device: requested {name}={expected_value!r}, got {result.value!r}"
            )
        return result

    def list_params(self, timeout: float = 3.0) -> list[ParamValue]:
        """读取设备当前全部参数。

        Args:
            timeout: 整个枚举流程的总超时，单位为秒。

        Returns:
            按设备返回顺序组成的参数列表。

        Raises:
            TimeoutError: 在限定时间内未收到 `PARAM_LIST_END`。
            RuntimeError: 当前尚未接入传输对象。
            ValueError: 某个参数项的编码格式不符合预期。
        """

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

    def save_params(self, timeout: float = DEFAULT_RESPONSE_TIMEOUT_S) -> None:
        """请求设备持久化当前参数。

        Args:
            timeout: 等待设备确认的超时，单位为秒。

        Raises:
            RuntimeError: 当前尚未接入传输对象。
            TimeoutError: 在限定时间内未收到保存确认。
        """

        with self._command_lock:
            self._send_message(MsgType.PARAM_SAVE)
            self._recv_until(MsgType.PARAM_SAVE, timeout=timeout)

    def reset_params(self, timeout: float = DEFAULT_RESPONSE_TIMEOUT_S) -> None:
        """请求设备恢复默认参数。

        Args:
            timeout: 等待设备确认的超时，单位为秒。

        Raises:
            RuntimeError: 当前尚未接入传输对象。
            TimeoutError: 在限定时间内未收到复位确认。
        """

        with self._command_lock:
            self._send_message(MsgType.PARAM_RESET)
            self._recv_until(MsgType.PARAM_RESET, timeout=timeout)

    def export_params(self, output_path: Path) -> ParamSnapshot:
        """导出当前参数快照到 JSON 文件。

        Args:
            output_path: 输出文件路径。父目录需由调用方提前准备。

        Returns:
            已写入磁盘的参数快照对象。

        Raises:
            TimeoutError: 握手或参数枚举超时。
            OSError: 输出文件无法写入时抛出。
            ValueError: 返回参数项无法按当前协议解码时抛出。
        """

        info = self._device_info or self.hello()
        params = self.list_params(timeout=3.0)
        snapshot = ParamSnapshot(
            schema=1,
            firmware={
                "protocol_version": info.protocol_version,
                "imu_mode": info.imu_mode,
                "feature_bitmap": info.feature_bitmap,
                "feature_names": info.feature_names(),
                "build_git_hash": info.build_git_hash,
                "build_time_utc": info.build_time_utc,
            },
            params=[{"name": p.name, "type_id": p.type_id, "value": p.value} for p in params],
        )
        snapshot.write_json(output_path)
        return snapshot

    def import_params(self, input_path: Path, save_after: bool = False) -> list[ParamValue]:
        """从 JSON 快照导入参数。

        Args:
            input_path: 输入文件路径。文件内容至少需要包含 `params` 列表。
            save_after: 为 `True` 时，在全部写入后额外发送一次保存命令。

        Returns:
            成功应用并由设备回显的参数列表。

        Raises:
            OSError: 输入文件无法读取时抛出。
            json.JSONDecodeError: 文件内容不是合法 JSON 时抛出。
            KeyError: 某个参数项缺少 `name`、`type_id` 或 `value` 字段时抛出。
            ValueError: 参数值无法按目标类型编码时抛出。
            TimeoutError: 写参或保存过程中设备未按时响应。
        """

        data = json.loads(input_path.read_text(encoding="utf-8"))
        applied: list[ParamValue] = []
        for item in data.get("params", []):
            applied.append(self.set_param(str(item["name"]), int(item["type_id"]), str(item["value"])))
        if save_after:
            self.save_params()
        return applied

    def start_csv_log(self, output_path: Path) -> None:
        """开始将后续遥测样本写入 CSV。

        Args:
            output_path: 输出文件路径。再次调用时会先关闭已有日志文件。

        Raises:
            OSError: 输出文件无法创建或写入时抛出。
        """

        if self._csv_logger is not None:
            self._csv_logger.close()
        self._csv_logger = CsvTelemetryLogger(output_path)
        self._last_log_path = output_path

    def stop_csv_log(self) -> Path | None:
        """停止当前 CSV 记录并返回最近一次输出路径。

        Returns:
            最近一次日志路径；若从未启动过记录则返回 `None`。
        """

        if self._csv_logger is None:
            return self._last_log_path
        self._csv_logger.close()
        self._csv_logger = None
        return self._last_log_path

    def dump_csv(self, output_path: Path, duration_s: float = 5.0) -> int:
        """在固定时长内采集遥测并写入 CSV。

        Args:
            output_path: 输出文件路径。
            duration_s: 采集时长，单位为秒。

        Returns:
            本次调用新增写入的遥测行数。

        Raises:
            TimeoutError: 启动遥测流时设备未按时响应。
            OSError: CSV 文件无法创建或写入时抛出。

        Note:
            该方法会启动遥测流并在结束时停止 CSV 记录，但不会自动调用
            `stop_stream()`。如果调用方不再需要实时流，应自行停流。
        """

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
