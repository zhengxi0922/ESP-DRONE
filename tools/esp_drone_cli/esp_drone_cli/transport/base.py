from __future__ import annotations

from typing import Protocol

from esp_drone_cli.protocol.messages import Frame


class Transport(Protocol):
    def send(self, data: bytes) -> None: ...
    def recv_frame(self, timeout: float) -> Frame: ...
    def close(self) -> None: ...
