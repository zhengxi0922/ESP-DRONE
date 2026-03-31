from esp_drone_cli.protocol.framing import (
    cobs_decode,
    cobs_encode,
    decode_frame,
    decode_serial_packet,
    encode_frame,
    encode_serial_packet,
)
from esp_drone_cli.protocol.messages import MsgType


def test_cobs_roundtrip():
    payload = b"\x00\x11\x22\x00\x33"
    encoded = cobs_encode(payload)
    assert cobs_decode(encoded) == payload


def test_frame_roundtrip():
    raw = encode_frame(MsgType.HELLO_REQ, b"abc", seq=7)
    frame = decode_frame(raw)
    assert frame.msg_type == MsgType.HELLO_REQ
    assert frame.seq == 7
    assert frame.payload == b"abc"


def test_serial_packet_roundtrip():
    packet = encode_serial_packet(MsgType.STREAM_CTRL, b"\x01", seq=9)
    frame = decode_serial_packet(packet)
    assert frame.msg_type == MsgType.STREAM_CTRL
    assert frame.seq == 9
    assert frame.payload == b"\x01"
