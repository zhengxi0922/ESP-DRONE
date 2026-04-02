from __future__ import annotations

import socket

from esp_drone_cli.core.protocol.framing import decode_frame, encode_frame
from esp_drone_cli.core.protocol.messages import Frame


class UdpTransport:
    def __init__(self, host: str, port: int = 2391, timeout: float = 1.0) -> None:
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(timeout)

    def send(self, data: bytes) -> None:
        self._sock.sendto(data, self._addr)

    def send_message(self, msg_type: int, payload: bytes = b"", flags: int = 0, seq: int = 0) -> None:
        self.send(encode_frame(msg_type, payload, flags=flags, seq=seq))

    def recv_frame(self, timeout: float) -> Frame:
        self._sock.settimeout(timeout)
        data, _ = self._sock.recvfrom(2048)
        return decode_frame(data)

    def close(self) -> None:
        self._sock.close()
