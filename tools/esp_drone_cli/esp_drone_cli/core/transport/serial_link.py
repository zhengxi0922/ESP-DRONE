"""基于 USB CDC/串口的传输实现。"""

from __future__ import annotations

import time

import serial

from esp_drone_cli.core.protocol.framing import decode_serial_packet, encode_serial_packet
from esp_drone_cli.core.protocol.messages import Frame


class SerialTransport:
    """使用 `pyserial` 与设备进行串口通信。"""

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 0.2,
        settle_delay_s: float = 0.25,
        open_retry_timeout_s: float = 5.0,
        open_retry_interval_s: float = 0.25,
    ) -> None:
        """创建并打开串口连接。

        Args:
            port: 串口名，例如 `COM7`。
            baudrate: 波特率。
            timeout: 底层串口读超时，单位为秒。
            settle_delay_s: 打开串口后额外等待的稳定时间，单位为秒。
            open_retry_timeout_s: 打开串口的总重试时长，单位为秒。
            open_retry_interval_s: 重试间隔，单位为秒。

        Raises:
            serial.SerialException: 在重试时限内始终无法打开串口时抛出。

        注意:
            当前实现会在连接建立后清空输入缓冲，避免把旧数据当成新会话内容。
        """

        deadline = time.monotonic() + max(0.0, open_retry_timeout_s)
        last_error: Exception | None = None
        while True:
            try:
                self._serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
                break
            except serial.SerialException as exc:
                last_error = exc
                if time.monotonic() >= deadline:
                    raise
                time.sleep(max(0.01, open_retry_interval_s))
        if last_error is not None:
            pass

        if settle_delay_s > 0:
            time.sleep(settle_delay_s)
        self._serial.reset_input_buffer()

    def send(self, data: bytes) -> None:
        """发送原始串口字节流。

        Args:
            data: 已编码好的串口数据包。
        """

        self._serial.write(data)
        self._serial.flush()

    def send_message(self, msg_type: int, payload: bytes = b"", flags: int = 0, seq: int = 0) -> None:
        """编码并发送一条协议消息。

        Args:
            msg_type: 消息类型编号。
            payload: 原始负载。
            flags: 帧标志位。
            seq: 发送序号。
        """

        self.send(encode_serial_packet(msg_type, payload, flags=flags, seq=seq))

    def recv_frame(self, timeout: float) -> Frame:
        """接收并解析一帧串口协议消息。

        Args:
            timeout: 最长等待时间，单位为秒。

        Returns:
            一帧已通过 COBS 与 CRC 校验的协议消息。

        Raises:
            TimeoutError: 超时前未收到完整合法帧时抛出。
        """

        deadline = time.monotonic() + timeout
        packet = bytearray()
        while time.monotonic() < deadline:
            chunk = self._serial.read(1)
            if not chunk:
                continue
            packet.extend(chunk)
            if len(packet) > 4096:
                packet.clear()
                continue
            if chunk == b"\x00":
                try:
                    return decode_serial_packet(bytes(packet))
                except ValueError:
                    packet.clear()
        raise TimeoutError("serial frame timeout")

    def close(self) -> None:
        """关闭串口句柄。"""

        self._serial.close()
