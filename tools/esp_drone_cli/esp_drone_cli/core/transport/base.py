"""设备会话使用的传输接口协议。"""

from __future__ import annotations

from typing import Protocol

from esp_drone_cli.core.protocol.messages import Frame


class Transport(Protocol):
    """会话层依赖的最小传输能力集合。"""

    def send(self, data: bytes) -> None:
        """发送原始字节流。

        Args:
            data: 已编码完成的字节串。
        """

        ...

    def send_message(self, msg_type: int, payload: bytes = b"", flags: int = 0, seq: int = 0) -> None:
        """发送一条协议消息。

        Args:
            msg_type: 消息类型编号。
            payload: 原始负载。
            flags: 帧标志位。
            seq: 发送序号。
        """

        ...

    def recv_frame(self, timeout: float) -> Frame:
        """接收一帧协议消息。

        Args:
            timeout: 最长等待时间，单位为秒。

        Returns:
            一帧已通过基础校验的协议消息。

        Raises:
            TimeoutError: 在超时前未收到完整消息时抛出。
        """

        ...

    def close(self) -> None:
        """关闭底层传输资源。"""

        ...
