"""命令行前端实现。"""

from __future__ import annotations

import argparse
from collections import deque
import sys
import time
from pathlib import Path

from esp_drone_cli.core import DeviceSession, TelemetrySample
from esp_drone_cli.core.protocol.messages import CmdId, CommandError, ensure_command_ok


def axis_name_to_index(name: str) -> int:
    """将轴名称转换为协议使用的轴编号。

    Args:
        name: 轴名称，只支持 `roll`、`pitch` 或 `yaw`。

    Returns:
        对应的轴编号，范围为 `0` 到 `2`。

    Raises:
        SystemExit: 轴名称不受支持时抛出。
    """

    names = {"roll": 0, "pitch": 1, "yaw": 2}
    if name not in names:
        raise SystemExit(f"unsupported axis {name}")
    return names[name]


def axis_index_to_name(index: int) -> str:
    names = ("roll", "pitch", "yaw")
    if index < 0 or index >= len(names):
        raise SystemExit(f"unsupported axis index {index}")
    return names[index]


def format_rate_status_line(sample: TelemetrySample, axis_name: str) -> str:
    snapshot = sample.axis_rate_debug_map(axis_name)
    motors = ", ".join(f"{value:.3f}" for value in snapshot["motor_outputs"])
    return (
        f"{axis_name} "
        f"sp={snapshot['setpoint_dps']:.3f} "
        f"fb={snapshot['feedback_dps']:.3f} "
        f"{snapshot['source_field']}={snapshot['source_value']:.3f} "
        f"p={snapshot['pid_p']:.4f} "
        f"i={snapshot['pid_i']:.4f} "
        f"d={snapshot['pid_d']:.4f} "
        f"out={snapshot['pid_out']:.4f} "
        f"motors=[{motors}] "
        f"arm={snapshot['arm_state']} "
        f"mode={snapshot['control_mode']} "
        f"imu_age_us={snapshot['imu_age_us']} "
        f"loop_dt_us={snapshot['loop_dt_us']}"
    )


def format_rate_status_line_all(sample: TelemetrySample) -> str:
    parts = [format_rate_status_line(sample, axis_name) for axis_name in ("roll", "pitch", "yaw")]
    return " | ".join(parts)


def connect_session_from_args(args) -> DeviceSession:
    """根据命令行参数创建并连接设备会话。

    Args:
        args: `argparse` 解析后的参数对象，需包含 `serial`、`udp` 和 `baudrate` 字段。

    Returns:
        已建立连接的 `DeviceSession`。

    Raises:
        SystemExit: 未提供 `--serial` 或 `--udp` 时抛出。
        ValueError: UDP 端口无法转换为整数时抛出。
        RuntimeError: 设备连接或握手失败时由下层会话抛出。
    """

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
    """为命令解析器补充通用连接参数。

    Args:
        parser: 目标解析器。

    Returns:
        None.
    """

    parser.add_argument("--serial", help="Serial port, for example COM7")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--udp", help="UDP endpoint host[:port], default port 2391")


def cmd_connect(session: DeviceSession, _args) -> int:
    """执行握手并打印设备摘要。

    Args:
        session: 已连接的设备会话。
        _args: 当前命令未使用的参数对象。

    Returns:
        固定返回 `0`。
    """

    print(session.device_info or session.hello())
    return 0


def cmd_arm(session: DeviceSession, _args) -> int:
    """发送解锁命令。

    Args:
        session: 已连接的设备会话。
        _args: 当前命令未使用的参数对象。

    Returns:
        设备返回的命令状态码。
    """

    ensure_command_ok(CmdId.ARM, session.arm())
    return 0


def cmd_disarm(session: DeviceSession, _args) -> int:
    """发送上锁命令。

    Args:
        session: 已连接的设备会话。
        _args: 当前命令未使用的参数对象。

    Returns:
        设备返回的命令状态码。
    """

    ensure_command_ok(CmdId.DISARM, session.disarm())
    return 0


def cmd_kill(session: DeviceSession, _args) -> int:
    """发送急停命令。

    Args:
        session: 已连接的设备会话。
        _args: 当前命令未使用的参数对象。

    Returns:
        设备返回的命令状态码。
    """

    ensure_command_ok(CmdId.KILL, session.kill())
    return 0


def cmd_reboot(session: DeviceSession, _args) -> int:
    """发送重启命令。

    Args:
        session: 已连接的设备会话。
        _args: 当前命令未使用的参数对象。

    Returns:
        设备返回的命令状态码。
    """

    ensure_command_ok(CmdId.REBOOT, session.reboot())
    return 0


def cmd_get(session: DeviceSession, args) -> int:
    """读取并打印单个参数。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `name`。

    Returns:
        固定返回 `0`。
    """

    print(session.get_param(args.name))
    return 0


def cmd_set(session: DeviceSession, args) -> int:
    """写入单个参数并打印设备回显值。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `name`、`type` 和 `value`。

    Returns:
        固定返回 `0`。

    Raises:
        KeyError: 参数类型不在支持集合内时抛出。
        ValueError: 文本值无法转换为目标参数类型时抛出。
    """

    type_id = {"bool": 0, "u8": 1, "u32": 2, "i32": 3, "float": 4}[args.type]
    print(session.set_param(args.name, type_id, args.value))
    return 0


def cmd_list(session: DeviceSession, _args) -> int:
    """枚举并打印全部参数。

    Args:
        session: 已连接的设备会话。
        _args: 当前命令未使用的参数对象。

    Returns:
        固定返回 `0`。
    """

    for item in session.list_params(timeout=3.0):
        print(item)
    return 0


def cmd_save(session: DeviceSession, _args) -> int:
    """请求设备持久化当前参数。

    Args:
        session: 已连接的设备会话。
        _args: 当前命令未使用的参数对象。

    Returns:
        固定返回 `0`。
    """

    session.save_params()
    return 0


def cmd_reset(session: DeviceSession, _args) -> int:
    """请求设备恢复默认参数。

    Args:
        session: 已连接的设备会话。
        _args: 当前命令未使用的参数对象。

    Returns:
        固定返回 `0`。
    """

    session.reset_params()
    return 0


def cmd_export(session: DeviceSession, args) -> int:
    """导出当前参数快照到 JSON 文件。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `output`。

    Returns:
        固定返回 `0`。

    Raises:
        OSError: 输出文件不可写时抛出。
    """

    session.export_params(Path(args.output))
    print(args.output)
    return 0


def cmd_import(session: DeviceSession, args) -> int:
    """从 JSON 文件导入参数。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `input` 和 `save`。

    Returns:
        固定返回 `0`。

    Raises:
        OSError: 输入文件不可读时抛出。
        ValueError: 文件内容无法转换为协议参数值时抛出。
    """

    applied = session.import_params(Path(args.input), save_after=args.save)
    print(f"applied {len(applied)} parameters from {args.input}")
    return 0


def cmd_stream(session: DeviceSession, args) -> int:
    """打开或关闭遥测流。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `state`，取值为 `on` 或 `off`。

    Returns:
        固定返回 `0`。
    """

    if args.state == "on":
        session.start_stream()
    else:
        session.stop_stream()
    return 0


def cmd_log(session: DeviceSession, args) -> int:
    """在给定时长内打印事件日志，可选打印遥测。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `timeout` 和 `telemetry`。

    Returns:
        固定返回 `0`。

    注意:
        若开启 `--telemetry`，该命令会主动打开遥测流，但不会在结束时显式停流。
    """

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


def cmd_rate_status(session: DeviceSession, args) -> int:
    """Print rate-loop bench telemetry with per-axis filtering."""

    samples: deque[TelemetrySample] = deque(maxlen=1)

    def on_telemetry(sample: TelemetrySample) -> None:
        samples.append(sample)

    token = session.subscribe_telemetry(on_telemetry)
    had_sample = False
    next_emit = time.monotonic()
    try:
        session.start_stream()
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            if not samples:
                time.sleep(0.02)
                continue
            if time.monotonic() < next_emit:
                time.sleep(0.02)
                continue

            sample = samples[-1]
            had_sample = True
            if args.axis == "all":
                print(format_rate_status_line_all(sample))
            else:
                print(format_rate_status_line(sample, args.axis))
            next_emit = time.monotonic() + args.interval

        if not had_sample:
            print("rate-status did not receive telemetry within timeout")
            return 1
        return 0
    finally:
        try:
            session.stop_stream()
        except Exception:
            pass
        session.unsubscribe(token)


def cmd_dump_csv(session: DeviceSession, args) -> int:
    """在指定时长内采集遥测并导出 CSV。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `output` 和 `duration`。

    Returns:
        固定返回 `0`。
    """

    count = session.dump_csv(Path(args.output), duration_s=args.duration)
    print(f"wrote {count} telemetry rows to {args.output}")
    return 0


def cmd_baro(session: DeviceSession, args) -> int:
    """在限定时间内观察气压计遥测。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `timeout`。

    Returns:
        `0` 表示收到有效气压计数据，`1` 表示协议版本不支持、超时或始终无有效数据。

    注意:
        该命令会在退出前主动停止遥测流。
    """

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
    """执行单电机点动测试。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `motor` 和 `duty`。

    Returns:
        设备返回的命令状态码。

    注意:
        `motor` 同时支持 `m1` 这类名称和纯数字索引。
    """

    motor_index = int(args.motor[1]) - 1 if args.motor.lower().startswith("m") else int(args.motor)
    ensure_command_ok(CmdId.MOTOR_TEST, session.motor_test(motor_index, float(args.duty)))
    return 0


def cmd_axis_test(session: DeviceSession, args) -> int:
    """执行开环轴向测试。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `axis` 和 `value`。

    Returns:
        设备返回的命令状态码。
    """

    ensure_command_ok(CmdId.AXIS_TEST, session.axis_test(axis_name_to_index(args.axis), float(args.value)))
    return 0


def cmd_rate_test(session: DeviceSession, args) -> int:
    """执行速率环测试。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `axis` 和 `value`。

    Returns:
        设备返回的命令状态码。
    """

    ensure_command_ok(CmdId.RATE_TEST, session.rate_test(axis_name_to_index(args.axis), float(args.value)))
    return 0


def cmd_calib(session: DeviceSession, args) -> int:
    """执行陀螺或水平校准。

    Args:
        session: 已连接的设备会话。
        args: 命令参数，需包含 `kind`，取值为 `gyro` 或 `level`。

    Returns:
        设备返回的命令状态码。
    """

    if args.kind == "gyro":
        ensure_command_ok(CmdId.CALIB_GYRO, session.calib_gyro())
    else:
        ensure_command_ok(CmdId.CALIB_LEVEL, session.calib_level())
    return 0


def build_parser() -> argparse.ArgumentParser:
    """构建 CLI 命令解析器。

    Returns:
        已配置全部子命令和连接参数的解析器对象。
    """

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

    rate_status_p = sub.add_parser(
        "rate-status",
        aliases=["watch-rate"],
        help="watch rate-loop telemetry for roll, pitch, yaw, or all axes",
        description="Print rate-loop bench telemetry with per-axis filtering.",
    )
    rate_status_p.add_argument("axis", nargs="?", choices=["roll", "pitch", "yaw", "all"], default="all")
    rate_status_p.add_argument("--timeout", type=float, default=5.0)
    rate_status_p.add_argument("--interval", type=float, default=0.2)

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
    """执行 CLI 主流程。

    Args:
        argv: 可选命令行参数列表；为 `None` 时读取进程实际参数。

    Returns:
        命令返回码。成功通常为 `0`，设备命令失败时返回设备状态码或子命令定义的错误码。

    Raises:
        SystemExit: `argparse` 参数校验失败时抛出。
        TimeoutError: 设备在规定时间内未返回预期响应时由底层会话抛出。

    注意:
        函数结束时始终会关闭设备会话。
    """

    parser = build_parser()
    args = parser.parse_args(argv)
    session: DeviceSession | None = None
    try:
        session = connect_session_from_args(args)
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
            "rate-status": cmd_rate_status,
            "watch-rate": cmd_rate_status,
            "baro": cmd_baro,
            "watch-baro": cmd_baro,
            "dump-csv": cmd_dump_csv,
            "motor-test": cmd_motor_test,
            "axis-test": cmd_axis_test,
            "rate-test": cmd_rate_test,
            "calib": cmd_calib,
        }
        return int(dispatch[args.command](session, args) or 0)
    except CommandError as exc:
        print(str(exc), file=sys.stderr)
        return int(exc.status or 1)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        if session is not None:
            session.close()


if __name__ == "__main__":
    sys.exit(main())
