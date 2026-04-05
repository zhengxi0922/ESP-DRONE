# ============================================================
# @file framing.py
# @brief ESP-DRONE ???????
# @details ?????????CRC?COBS ??????????
# @author Codex
# @date 2026-04-05
# @version 1.0
# ============================================================

from __future__ import annotations

import struct

from .messages import FRAME_MAGIC, FRAME_VERSION, Frame


HEADER_STRUCT = struct.Struct("<BBBBHH")
CRC_STRUCT = struct.Struct("<H")


def crc16_ccitt(data: bytes) -> int:
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
    header = HEADER_STRUCT.pack(FRAME_MAGIC, FRAME_VERSION, msg_type, flags, seq & 0xFFFF, len(payload))
    raw = header + payload
    crc = CRC_STRUCT.pack(crc16_ccitt(raw))
    return raw + crc


def decode_frame(raw: bytes) -> Frame:
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
    return cobs_encode(encode_frame(msg_type, payload, flags=flags, seq=seq)) + b"\x00"


def decode_serial_packet(packet: bytes) -> Frame:
    if packet.endswith(b"\x00"):
        packet = packet[:-1]
    return decode_frame(cobs_decode(packet))
