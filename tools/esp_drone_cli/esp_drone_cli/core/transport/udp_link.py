"""基于 UDP 的传输实现。"""

from __future__ import annotations

import socket

from esp_drone_cli.core.protocol.framing import decode_frame, encode_frame
from esp_drone_cli.core.protocol.messages import Frame


class UdpTransport:
    """使用 UDP 与设备交换协议帧。"""

    def __init__(self, host: str, port: int = 2391, timeout: float = 1.0) -> None:
        """创建 UDP 连接对象。

        Args:
            host: 目标主机地址。
            port: 目标端口，默认为 `2391`。
            timeout: 接收超时，单位为秒。

        Raises:
            OSError: 套接字创建失败时抛出。
        """

        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(timeout)

    def send(self, data: bytes) -> None:
        """发送原始 UDP 数据报。

        Args:
            data: 已编码好的原始协议帧。

        Raises:
            OSError: 发送失败时抛出。
        """

        self._sock.sendto(data, self._addr)

    def send_message(self, msg_type: int, payload: bytes = b"", flags: int = 0, seq: int = 0) -> None:
        """编码并发送一条协议消息。

        Args:
            msg_type: 消息类型编号。
            payload: 原始负载。
            flags: 帧标志位。
            seq: 发送序号。
        """

        self.send(encode_frame(msg_type, payload, flags=flags, seq=seq))

    def recv_frame(self, timeout: float) -> Frame:
        """接收并解析一帧 UDP 协议消息。

        Args:
            timeout: 接收超时，单位为秒。

        Returns:
            解析后的协议帧。

        Raises:
            TimeoutError: 底层 socket 超时时抛出。
            OSError: 套接字接收失败时抛出。
            ValueError: 收到的原始数据不符合协议格式时抛出。
        """

        self._sock.settimeout(timeout)
        data, _ = self._sock.recvfrom(2048)
        return decode_frame(data)

    def close(self) -> None:
        """关闭 UDP 套接字。"""

        self._sock.close()
