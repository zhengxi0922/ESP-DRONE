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
    UDP_MANUAL_SETPOINT = 0x50


class CmdId:
    ARM = 1
    DISARM = 2
    KILL = 3
    REBOOT = 4
    MOTOR_TEST = 5
    CALIB_GYRO = 6
    CALIB_LEVEL = 7
    AXIS_TEST = 8
    RATE_TEST = 9
    ATTITUDE_CAPTURE_REF = 10
    ATTITUDE_TEST_START = 11
    ATTITUDE_TEST_STOP = 12
    UDP_MANUAL_ENABLE = 13
    UDP_MANUAL_DISABLE = 14
    UDP_MANUAL_SETPOINT = 15
    UDP_TAKEOFF = 16
    UDP_LAND = 17
    UDP_MANUAL_STOP = 18
    GROUND_CAPTURE_REF = 19
    GROUND_TEST_START = 20
    GROUND_TEST_STOP = 21


class CmdStatus:
    OK = 0
    REJECTED = 1
    UNSUPPORTED = 2
    INVALID_ARGUMENT = 3
    ARM_REQUIRED = 4
    DISARM_REQUIRED = 5
    IMU_NOT_READY = 6
    CONFLICT = 7
    STORAGE_ERROR = 8
    REF_REQUIRED = 9


CMD_NAMES = {
    CmdId.ARM: "arm",
    CmdId.DISARM: "disarm",
    CmdId.KILL: "kill",
    CmdId.REBOOT: "reboot",
    CmdId.MOTOR_TEST: "motor-test",
    CmdId.CALIB_GYRO: "calib gyro",
    CmdId.CALIB_LEVEL: "calib level",
    CmdId.AXIS_TEST: "axis-test",
    CmdId.RATE_TEST: "rate-test",
    CmdId.ATTITUDE_CAPTURE_REF: "attitude-capture-ref",
    CmdId.ATTITUDE_TEST_START: "attitude-test start",
    CmdId.ATTITUDE_TEST_STOP: "attitude-test stop",
    CmdId.UDP_MANUAL_ENABLE: "udp-manual enable",
    CmdId.UDP_MANUAL_DISABLE: "udp-manual disable",
    CmdId.UDP_MANUAL_SETPOINT: "udp-manual setpoint",
    CmdId.UDP_TAKEOFF: "udp takeoff",
    CmdId.UDP_LAND: "udp land",
    CmdId.UDP_MANUAL_STOP: "udp-manual stop",
    CmdId.GROUND_CAPTURE_REF: "ground-capture-ref",
    CmdId.GROUND_TEST_START: "ground-test start",
    CmdId.GROUND_TEST_STOP: "ground-test stop",
}

CMD_STATUS_TEXT = {
    CmdStatus.OK: "ok",
    CmdStatus.REJECTED: "command was rejected",
    CmdStatus.UNSUPPORTED: "command is not supported by this firmware",
    CmdStatus.INVALID_ARGUMENT: "argument is invalid or outside the allowed bench range",
    CmdStatus.ARM_REQUIRED: "device must be armed first",
    CmdStatus.DISARM_REQUIRED: "device must be disarmed first",
    CmdStatus.IMU_NOT_READY: "fresh IMU / quaternion / gyro feedback is not ready",
    CmdStatus.CONFLICT: "command conflicts with the current control mode or safety state",
    CmdStatus.STORAGE_ERROR: "device-side persistence or internal operation failed",
    CmdStatus.REF_REQUIRED: "attitude reference has not been captured yet",
}


class CommandError(RuntimeError):
    def __init__(self, cmd_id: int, status: int) -> None:
        self.cmd_id = cmd_id
        self.status = status
        super().__init__(describe_command_status(cmd_id, status))


def command_name(cmd_id: int) -> str:
    return CMD_NAMES.get(cmd_id, f"cmd-{cmd_id}")


def describe_command_status(cmd_id: int, status: int) -> str:
    return f"{command_name(cmd_id)} failed: {CMD_STATUS_TEXT.get(status, f'unknown status {status}')}"


def ensure_command_ok(cmd_id: int, status: int) -> int:
    if status != CmdStatus.OK:
        raise CommandError(cmd_id, status)
    return status


@dataclass(slots=True)
class Frame:
    msg_type: int
    flags: int
    seq: int
    payload: bytes
