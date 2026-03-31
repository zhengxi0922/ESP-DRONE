from __future__ import annotations

import time

import serial

from esp_drone_cli.protocol.framing import decode_serial_packet, encode_serial_packet
from esp_drone_cli.protocol.messages import Frame


class SerialTransport:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.2) -> None:
        self._serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)

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
            if chunk == b"\x00":
                return decode_serial_packet(bytes(packet))
        raise TimeoutError("serial frame timeout")

    def close(self) -> None:
        self._serial.close()
