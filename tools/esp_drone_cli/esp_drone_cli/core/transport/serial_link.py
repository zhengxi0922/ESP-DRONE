from __future__ import annotations

import time

import serial

from esp_drone_cli.core.protocol.framing import decode_serial_packet, encode_serial_packet
from esp_drone_cli.core.protocol.messages import Frame


class SerialTransport:
    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 0.2,
        settle_delay_s: float = 0.25,
        open_retry_timeout_s: float = 5.0,
        open_retry_interval_s: float = 0.25,
    ) -> None:
        deadline = time.monotonic() + max(0.0, open_retry_timeout_s)
        last_error: Exception | None = None
        while True:
            try:
                # Keep pyserial's default control-line behavior so Windows USB CDC
                # devices see the same line state as standard terminal tools.
                self._serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
                break
            except serial.SerialException as exc:
                last_error = exc
                if time.monotonic() >= deadline:
                    raise
                time.sleep(max(0.01, open_retry_interval_s))
        if last_error is not None:
            # Opening eventually succeeded after a transient COM hold. No action needed.
            pass

        if settle_delay_s > 0:
            time.sleep(settle_delay_s)
        self._serial.reset_input_buffer()

    def send(self, data: bytes) -> None:
        self._serial.write(data)
        self._serial.flush()

    def send_message(self, msg_type: int, payload: bytes = b"", flags: int = 0, seq: int = 0) -> None:
        self.send(encode_serial_packet(msg_type, payload, flags=flags, seq=seq))

    def recv_frame(self, timeout: float) -> Frame:
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
        self._serial.close()
