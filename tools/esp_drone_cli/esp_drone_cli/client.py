from __future__ import annotations

import csv
import struct
import time
from dataclasses import dataclass
from pathlib import Path

from .protocol.messages import CmdId, Frame, MsgType


CMD_REQ_STRUCT = struct.Struct("<BBHf")
CMD_RESP_STRUCT = struct.Struct("<BBH")
HELLO_RESP_STRUCT = struct.Struct("<BBBBI")
TELEMETRY_STRUCT = struct.Struct("<Q" + "f" * 27 + "III8B")


@dataclass(slots=True)
class ParamValue:
    name: str
    type_id: int
    value: object


class EspDroneClient:
    def __init__(self, transport) -> None:
        self._transport = transport
        self._seq = 0

    def close(self) -> None:
        self._transport.close()

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFFFF
        return self._seq

    def _send_message(self, msg_type: int, payload: bytes = b"") -> None:
        seq = self._next_seq()
        if hasattr(self._transport, "send_message"):
            self._transport.send_message(msg_type, payload, seq=seq)
        else:
            raise RuntimeError("transport does not support send_message")

    def _recv_until(self, *msg_types: int, timeout: float = 1.0) -> Frame:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self._transport.recv_frame(max(0.01, deadline - time.monotonic()))
            if frame.msg_type in msg_types:
                return frame
        raise TimeoutError(f"timed out waiting for {msg_types}")

    def hello(self) -> dict[str, int]:
        self._send_message(MsgType.HELLO_REQ)
        frame = self._recv_until(MsgType.HELLO_RESP)
        protocol_version, imu_mode, arm_state, stream_enabled, feature_bitmap = HELLO_RESP_STRUCT.unpack(frame.payload)
        return {
            "protocol_version": protocol_version,
            "imu_mode": imu_mode,
            "arm_state": arm_state,
            "stream_enabled": stream_enabled,
            "feature_bitmap": feature_bitmap,
        }

    def command(self, cmd_id: int, arg_u8: int = 0, arg_f32: float = 0.0) -> int:
        payload = CMD_REQ_STRUCT.pack(cmd_id, arg_u8 & 0xFF, 0, float(arg_f32))
        self._send_message(MsgType.CMD_REQ, payload)
        frame = self._recv_until(MsgType.CMD_RESP)
        _, status, _ = CMD_RESP_STRUCT.unpack(frame.payload)
        return status

    def arm(self) -> int:
        return self.command(CmdId.ARM)

    def disarm(self) -> int:
        return self.command(CmdId.DISARM)

    def kill(self) -> int:
        return self.command(CmdId.KILL)

    def reboot(self) -> int:
        return self.command(CmdId.REBOOT)

    def motor_test(self, motor_index: int, duty: float) -> int:
        return self.command(CmdId.MOTOR_TEST, arg_u8=motor_index, arg_f32=duty)

    def axis_test(self, axis_index: int, value: float) -> int:
        return self.command(CmdId.AXIS_TEST, arg_u8=axis_index, arg_f32=value)

    def rate_test(self, axis_index: int, value_dps: float) -> int:
        return self.command(CmdId.RATE_TEST, arg_u8=axis_index, arg_f32=value_dps)

    def set_stream(self, enabled: bool) -> None:
        self._send_message(MsgType.STREAM_CTRL, bytes([1 if enabled else 0]))
        self._recv_until(MsgType.STREAM_CTRL)

    def get_param(self, name: str) -> ParamValue:
        name_bytes = name.encode("ascii")
        self._send_message(MsgType.PARAM_GET, bytes([len(name_bytes)]) + name_bytes)
        frame = self._recv_until(MsgType.PARAM_VALUE)
        return decode_param_value(frame.payload)

    def set_param(self, name: str, type_id: int, value_bytes: bytes) -> ParamValue:
        name_bytes = name.encode("ascii")
        payload = bytes([type_id, len(name_bytes)]) + name_bytes + value_bytes
        self._send_message(MsgType.PARAM_SET, payload)
        frame = self._recv_until(MsgType.PARAM_VALUE)
        return decode_param_value(frame.payload)

    def list_params(self) -> list[ParamValue]:
        self._send_message(MsgType.PARAM_LIST_REQ)
        items: list[ParamValue] = []
        while True:
            frame = self._recv_until(MsgType.PARAM_LIST_ITEM, MsgType.PARAM_VALUE, MsgType.PARAM_LIST_END)
            if frame.msg_type == MsgType.PARAM_LIST_END:
                return items
            items.append(decode_param_value(frame.payload))

    def save_params(self) -> None:
        self._send_message(MsgType.PARAM_SAVE)
        self._recv_until(MsgType.PARAM_SAVE)

    def reset_params(self) -> None:
        self._send_message(MsgType.PARAM_RESET)
        self._recv_until(MsgType.PARAM_RESET)

    def iter_log(self, timeout: float = 30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            yield self._transport.recv_frame(max(0.01, deadline - time.monotonic()))

    def dump_csv(self, output_path: Path, duration_s: float = 5.0) -> int:
        self.set_stream(True)
        count = 0
        with output_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow([
                "timestamp_us", "gyro_x", "gyro_y", "gyro_z",
                "acc_x", "acc_y", "acc_z", "quat_w", "quat_x", "quat_y", "quat_z",
                "roll_deg", "pitch_deg", "yaw_deg",
                "setpoint_roll", "setpoint_pitch", "setpoint_yaw",
                "rate_setpoint_roll", "rate_setpoint_pitch", "rate_setpoint_yaw",
                "pid_out_roll", "pid_out_pitch", "pid_out_yaw",
                "motor1", "motor2", "motor3", "motor4",
                "battery_voltage", "battery_adc_raw", "loop_dt_us", "imu_age_us",
                "imu_mode", "imu_health", "arm_state", "failsafe_reason", "control_mode",
            ])
            deadline = time.monotonic() + duration_s
            while time.monotonic() < deadline:
                frame = self._recv_until(MsgType.TELEMETRY_SAMPLE, MsgType.EVENT_LOG_TEXT, timeout=0.5)
                if frame.msg_type != MsgType.TELEMETRY_SAMPLE:
                    continue
                writer.writerow(TELEMETRY_STRUCT.unpack(frame.payload))
                count += 1
        return count


def decode_param_value(payload: bytes) -> ParamValue:
    type_id = payload[0]
    name_len = payload[1]
    name = payload[2 : 2 + name_len].decode("ascii")
    value_bytes = payload[2 + name_len :]
    if type_id == 0:
        value = value_bytes[0] != 0
    elif type_id == 1:
        value = value_bytes[0]
    elif type_id == 2:
        value = struct.unpack("<I", value_bytes)[0]
    elif type_id == 3:
        value = struct.unpack("<i", value_bytes)[0]
    elif type_id == 4:
        value = struct.unpack("<f", value_bytes)[0]
    else:
        value = value_bytes
    return ParamValue(name=name, type_id=type_id, value=value)
