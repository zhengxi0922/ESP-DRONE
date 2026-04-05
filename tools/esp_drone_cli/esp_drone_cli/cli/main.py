# ============================================================
# @file main.py
# @brief ESP-DRONE CLI ????
# @details ????????????? DeviceSession ?????????????????
# @author Codex
# @date 2026-04-05
# @version 1.0
# ============================================================

from __future__ import annotations

import argparse
from collections import deque
import sys
import time
from pathlib import Path

from esp_drone_cli.core import DeviceSession, TelemetrySample


def axis_name_to_index(name: str) -> int:
    names = {"roll": 0, "pitch": 1, "yaw": 2}
    if name not in names:
        raise SystemExit(f"unsupported axis {name}")
    return names[name]


def connect_session_from_args(args) -> DeviceSession:
    session = DeviceSession()
    if args.serial:
        session.connect_serial(args.serial, baudrate=args.baudrate)
    elif args.udp:
        host, _, port = args.udp.partition(":")
        session.connect_udp(host, int(port or "2391"))
    else:
        raise SystemExit("one of --serial or --udp is required")
    return session


def add_common_transport_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--serial", help="Serial port, for example COM7")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--udp", help="UDP endpoint host[:port], default port 2391")


def cmd_connect(session: DeviceSession, _args) -> int:
    print(session.device_info or session.hello())
    return 0


def cmd_arm(session: DeviceSession, _args) -> int:
    return session.arm()


def cmd_disarm(session: DeviceSession, _args) -> int:
    return session.disarm()


def cmd_kill(session: DeviceSession, _args) -> int:
    return session.kill()


def cmd_reboot(session: DeviceSession, _args) -> int:
    return session.reboot()


def cmd_get(session: DeviceSession, args) -> int:
    print(session.get_param(args.name))
    return 0


def cmd_set(session: DeviceSession, args) -> int:
    type_id = {"bool": 0, "u8": 1, "u32": 2, "i32": 3, "float": 4}[args.type]
    print(session.set_param(args.name, type_id, args.value))
    return 0


def cmd_list(session: DeviceSession, _args) -> int:
    for item in session.list_params(timeout=3.0):
        print(item)
    return 0


def cmd_save(session: DeviceSession, _args) -> int:
    session.save_params()
    return 0


def cmd_reset(session: DeviceSession, _args) -> int:
    session.reset_params()
    return 0


def cmd_export(session: DeviceSession, args) -> int:
    session.export_params(Path(args.output))
    print(args.output)
    return 0


def cmd_import(session: DeviceSession, args) -> int:
    applied = session.import_params(Path(args.input), save_after=args.save)
    print(f"applied {len(applied)} parameters from {args.input}")
    return 0


def cmd_stream(session: DeviceSession, args) -> int:
    if args.state == "on":
        session.start_stream()
    else:
        session.stop_stream()
    return 0


def cmd_log(session: DeviceSession, args) -> int:
    def on_event(message: str) -> None:
        print(f"EVENT {message}")

    def on_telemetry(sample: TelemetrySample) -> None:
        if not args.telemetry:
            return
        print(sample.to_display_map())

    event_token = session.subscribe_event_log(on_event)
    telemetry_token = session.subscribe_telemetry(on_telemetry)
    try:
        if args.telemetry:
            session.start_stream()
        time.sleep(args.timeout)
        return 0
    finally:
        session.unsubscribe(event_token)
        session.unsubscribe(telemetry_token)


def cmd_dump_csv(session: DeviceSession, args) -> int:
    count = session.dump_csv(Path(args.output), duration_s=args.duration)
    print(f"wrote {count} telemetry rows to {args.output}")
    return 0


def cmd_baro(session: DeviceSession, args) -> int:
    info = session.device_info or session.hello()
    if info.protocol_version < 2:
        print("current firmware does not advertise baro telemetry support")
        return 1

    samples: deque[TelemetrySample] = deque()

    def on_telemetry(sample: TelemetrySample) -> None:
        samples.append(sample)

    token = session.subscribe_telemetry(on_telemetry)
    had_sample = False
    had_valid_baro = False
    try:
        session.start_stream()
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            if not samples:
                time.sleep(0.05)
                continue

            sample = samples.popleft()
            had_sample = True
            had_valid_baro = had_valid_baro or bool(sample.baro_valid)
            print(
                "pressure_pa={:.1f} temperature_c={:.2f} altitude_m={:.3f} "
                "valid={} update_age_us={}".format(
                    sample.baro_pressure_pa,
                    sample.baro_temperature_c,
                    sample.baro_altitude_m,
                    bool(sample.baro_valid),
                    sample.baro_update_age_us,
                )
            )

        if not had_sample:
            print("device stream did not produce telemetry within timeout")
            return 1
        if not had_valid_baro:
            print("current telemetry stream has no valid baro data")
            return 1
        return 0
    finally:
        try:
            session.stop_stream()
        except Exception:
            pass
        session.unsubscribe(token)


def cmd_motor_test(session: DeviceSession, args) -> int:
    motor_index = int(args.motor[1]) - 1 if args.motor.lower().startswith("m") else int(args.motor)
    return session.motor_test(motor_index, float(args.duty))


def cmd_axis_test(session: DeviceSession, args) -> int:
    return session.axis_test(axis_name_to_index(args.axis), float(args.value))


def cmd_rate_test(session: DeviceSession, args) -> int:
    return session.rate_test(axis_name_to_index(args.axis), float(args.value))


def cmd_calib(session: DeviceSession, args) -> int:
    return session.calib_gyro() if args.kind == "gyro" else session.calib_level()


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

    export_p = sub.add_parser("export")
    export_p.add_argument("output")

    import_p = sub.add_parser("import")
    import_p.add_argument("input")
    import_p.add_argument("--save", action="store_true")

    stream_p = sub.add_parser("stream")
    stream_p.add_argument("state", choices=["on", "off"])

    log_p = sub.add_parser("log")
    log_p.add_argument("--timeout", type=float, default=10.0)
    log_p.add_argument("--telemetry", action="store_true")

    baro_p = sub.add_parser("baro", aliases=["watch-baro"])
    baro_p.add_argument("--timeout", type=float, default=5.0)

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
    session = connect_session_from_args(args)
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
            "export": cmd_export,
            "import": cmd_import,
            "stream": cmd_stream,
            "log": cmd_log,
            "baro": cmd_baro,
            "watch-baro": cmd_baro,
            "dump-csv": cmd_dump_csv,
            "motor-test": cmd_motor_test,
            "axis-test": cmd_axis_test,
            "rate-test": cmd_rate_test,
            "calib": cmd_calib,
        }
        return int(dispatch[args.command](session, args) or 0)
    finally:
        session.close()


if __name__ == "__main__":
    sys.exit(main())
