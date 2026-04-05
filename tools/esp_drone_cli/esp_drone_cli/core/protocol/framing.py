"""协议帧与串口 COBS 包的编解码实现。"""

from __future__ import annotations

import struct

from .messages import FRAME_MAGIC, FRAME_VERSION, Frame


HEADER_STRUCT = struct.Struct("<BBBBHH")
CRC_STRUCT = struct.Struct("<H")


def crc16_ccitt(data: bytes) -> int:
    """计算 CRC16-CCITT 校验值。

    Args:
        data: 待校验的原始字节串。

    Returns:
        16 位无符号 CRC 值。
    """

    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    """对原始字节串执行 COBS 编码。

    Args:
        data: 原始负载。

    Returns:
        不包含尾部分隔零字节的 COBS 编码结果。
    """

    output = bytearray()
    code_index = 0
    output.append(0)
    code = 1

    for byte in data:
        if byte == 0:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1
        else:
            output.append(byte)
            code += 1
            if code == 0xFF:
                output[code_index] = code
                code_index = len(output)
                output.append(0)
                code = 1

    output[code_index] = code
    return bytes(output)


def cobs_decode(data: bytes) -> bytes:
    """解码 COBS 字节串。

    Args:
        data: 不含结尾分隔零字节的 COBS 数据。

    Returns:
        解码后的原始字节串。

    Raises:
        ValueError: 输入包含非法 COBS 编码或数据被截断时抛出。
    """

    output = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        if code == 0:
            raise ValueError("invalid COBS code 0")
        index += 1
        for _ in range(code - 1):
            if index >= len(data):
                raise ValueError("truncated COBS payload")
            output.append(data[index])
            index += 1
        if code != 0xFF and index < len(data):
            output.append(0)
    return bytes(output)


def encode_frame(msg_type: int, payload: bytes = b"", flags: int = 0, seq: int = 0) -> bytes:
    """编码一帧原始协议消息。

    Args:
        msg_type: 消息类型编号。
        payload: 负载字节串。
        flags: 帧标志位。
        seq: 发送序号，只保留低 16 位。

    Returns:
        带帧头和 CRC 的原始协议帧。
    """

    header = HEADER_STRUCT.pack(FRAME_MAGIC, FRAME_VERSION, msg_type, flags, seq & 0xFFFF, len(payload))
    raw = header + payload
    crc = CRC_STRUCT.pack(crc16_ccitt(raw))
    return raw + crc


def decode_frame(raw: bytes) -> Frame:
    """解析并校验一帧原始协议消息。

    Args:
        raw: 包含帧头、负载和 CRC 的原始字节串。

    Returns:
        解析后的 `Frame` 对象。

    Raises:
        ValueError: 帧头、长度或 CRC 校验失败时抛出。
    """

    if len(raw) < HEADER_STRUCT.size + CRC_STRUCT.size:
        raise ValueError("frame too short")

    magic, version, msg_type, flags, seq, payload_len = HEADER_STRUCT.unpack_from(raw, 0)
    if magic != FRAME_MAGIC or version != FRAME_VERSION:
        raise ValueError("bad frame header")

    expected_len = HEADER_STRUCT.size + payload_len + CRC_STRUCT.size
    if len(raw) != expected_len:
        raise ValueError("payload length mismatch")

    expected_crc = CRC_STRUCT.unpack_from(raw, len(raw) - CRC_STRUCT.size)[0]
    actual_crc = crc16_ccitt(raw[:-CRC_STRUCT.size])
    if expected_crc != actual_crc:
        raise ValueError("bad crc")

    payload = raw[HEADER_STRUCT.size : HEADER_STRUCT.size + payload_len]
    return Frame(msg_type=msg_type, flags=flags, seq=seq, payload=payload)


def encode_serial_packet(msg_type: int, payload: bytes = b"", flags: int = 0, seq: int = 0) -> bytes:
    """编码用于串口发送的完整数据包。

    Args:
        msg_type: 消息类型编号。
        payload: 负载字节串。
        flags: 帧标志位。
        seq: 发送序号。

    Returns:
        经过 COBS 编码并追加结尾零字节的串口包。
    """

    return cobs_encode(encode_frame(msg_type, payload, flags=flags, seq=seq)) + b"\x00"


def decode_serial_packet(packet: bytes) -> Frame:
    """解码串口接收到的完整数据包。

    Args:
        packet: 可能包含尾部分隔零字节的串口包。

    Returns:
        解析后的 `Frame` 对象。

    Raises:
        ValueError: COBS 或帧校验失败时抛出。
    """

    if packet.endswith(b"\x00"):
        packet = packet[:-1]
    return decode_frame(cobs_decode(packet))
