from __future__ import annotations

from dataclasses import dataclass


FRAME_MAGIC = 0xA5
FRAME_VERSION = 0x01


class MsgType:
    HELLO_REQ = 0x01
    HELLO_RESP = 0x02
    CMD_REQ = 0x10
    CMD_RESP = 0x11
    PARAM_GET = 0x20
    PARAM_VALUE = 0x21
    PARAM_SET = 0x22
    PARAM_LIST_REQ = 0x23
    PARAM_LIST_ITEM = 0x24
    PARAM_LIST_END = 0x25
    PARAM_SAVE = 0x26
    PARAM_RESET = 0x27
    STREAM_CTRL = 0x30
    TELEMETRY_SAMPLE = 0x31
    EVENT_LOG_TEXT = 0x40


class CmdId:
    ARM = 1
    DISARM = 2
    KILL = 3
    REBOOT = 4
    MOTOR_TEST = 5
    CALIB_GYRO = 6
    CALIB_LEVEL = 7


@dataclass(slots=True)
class Frame:
    msg_type: int
    flags: int
    seq: int
    payload: bytes
