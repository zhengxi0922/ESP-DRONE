from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

from .client import EspDroneClient
from .protocol.messages import CmdId
from .transport.serial_link import SerialTransport
from .transport.udp_link import UdpTransport


def build_transport(args):
    if args.serial:
        return SerialTransport(args.serial, baudrate=args.baudrate)
    if args.udp:
        host, _, port = args.udp.partition(":")
        return UdpTransport(host, int(port or "2391"))
    raise SystemExit("one of --serial or --udp is required")


def add_common_transport_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--serial", help="Serial port, for example COM7")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--udp", help="UDP endpoint host[:port], default port 2391")


def cmd_connect(client: EspDroneClient, _args) -> int:
    print(client.hello())
    return 0


def cmd_arm(client: EspDroneClient, _args) -> int:
    return client.arm()


def cmd_disarm(client: EspDroneClient, _args) -> int:
    return client.disarm()


def cmd_kill(client: EspDroneClient, _args) -> int:
    return client.kill()


def cmd_reboot(client: EspDroneClient, _args) -> int:
    return client.reboot()


def cmd_get(client: EspDroneClient, args) -> int:
    print(client.get_param(args.name))
    return 0


def cmd_set(client: EspDroneClient, args) -> int:
    if args.type == "bool":
        type_id, value_bytes = 0, bytes([1 if args.value.lower() in {"1", "true", "yes", "on"} else 0])
    elif args.type == "u8":
        type_id, value_bytes = 1, struct.pack("<B", int(args.value))
    elif args.type == "u32":
        type_id, value_bytes = 2, struct.pack("<I", int(args.value))
    elif args.type == "i32":
        type_id, value_bytes = 3, struct.pack("<i", int(args.value))
    elif args.type == "float":
        type_id, value_bytes = 4, struct.pack("<f", float(args.value))
    else:
        raise SystemExit(f"unsupported type {args.type}")
    print(client.set_param(args.name, type_id, value_bytes))
    return 0


def cmd_list(client: EspDroneClient, _args) -> int:
    for item in client.list_params():
        print(item)
    return 0


def cmd_save(client: EspDroneClient, _args) -> int:
    client.save_params()
    return 0


def cmd_reset(client: EspDroneClient, _args) -> int:
    client.reset_params()
    return 0


def cmd_stream(client: EspDroneClient, args) -> int:
    client.set_stream(args.state == "on")
    return 0


def cmd_log(client: EspDroneClient, args) -> int:
    for frame in client.iter_log(timeout=args.timeout):
        print(frame)
    return 0


def cmd_dump_csv(client: EspDroneClient, args) -> int:
    count = client.dump_csv(Path(args.output), duration_s=args.duration)
    print(f"wrote {count} telemetry rows to {args.output}")
    return 0


def cmd_motor_test(client: EspDroneClient, args) -> int:
    motor_index = int(args.motor[1]) - 1 if args.motor.lower().startswith("m") else int(args.motor)
    return client.motor_test(motor_index, float(args.duty))


def axis_name_to_index(name: str) -> int:
    names = {"roll": 0, "pitch": 1, "yaw": 2}
    if name not in names:
        raise SystemExit(f"unsupported axis {name}")
    return names[name]


def cmd_axis_test(client: EspDroneClient, args) -> int:
    return client.axis_test(axis_name_to_index(args.axis), float(args.value))


def cmd_rate_test(client: EspDroneClient, args) -> int:
    return client.rate_test(axis_name_to_index(args.axis), float(args.value))


def cmd_calib(client: EspDroneClient, args) -> int:
    cmd_id = CmdId.CALIB_GYRO if args.kind == "gyro" else CmdId.CALIB_LEVEL
    return client.command(cmd_id)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="esp-drone-cli")
    add_common_transport_args(parser)
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("connect")
    sub.add_parser("arm")
    sub.add_parser("disarm")
    sub.add_parser("kill")
    sub.add_parser("reboot")

    get_p = sub.add_parser("get")
    get_p.add_argument("name")

    set_p = sub.add_parser("set")
    set_p.add_argument("name")
    set_p.add_argument("type", choices=["bool", "u8", "u32", "i32", "float"])
    set_p.add_argument("value")

    sub.add_parser("list")
    sub.add_parser("save")
    sub.add_parser("reset")

    stream_p = sub.add_parser("stream")
    stream_p.add_argument("state", choices=["on", "off"])

    log_p = sub.add_parser("log")
    log_p.add_argument("--timeout", type=float, default=10.0)

    csv_p = sub.add_parser("dump-csv")
    csv_p.add_argument("output")
    csv_p.add_argument("--duration", type=float, default=5.0)

    motor_p = sub.add_parser("motor-test")
    motor_p.add_argument("motor")
    motor_p.add_argument("duty")

    axis_p = sub.add_parser("axis-test")
    axis_p.add_argument("axis", choices=["roll", "pitch", "yaw"])
    axis_p.add_argument("value")

    rate_p = sub.add_parser("rate-test")
    rate_p.add_argument("axis", choices=["roll", "pitch", "yaw"])
    rate_p.add_argument("value")

    calib_p = sub.add_parser("calib")
    calib_p.add_argument("kind", choices=["gyro", "level"])

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    transport = build_transport(args)
    client = EspDroneClient(transport)
    try:
        dispatch = {
            "connect": cmd_connect,
            "arm": cmd_arm,
            "disarm": cmd_disarm,
            "kill": cmd_kill,
            "reboot": cmd_reboot,
            "get": cmd_get,
            "set": cmd_set,
            "list": cmd_list,
            "save": cmd_save,
            "reset": cmd_reset,
            "stream": cmd_stream,
            "log": cmd_log,
            "dump-csv": cmd_dump_csv,
            "motor-test": cmd_motor_test,
            "axis-test": cmd_axis_test,
            "rate-test": cmd_rate_test,
            "calib": cmd_calib,
        }
        return int(dispatch[args.command](client, args) or 0)
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
