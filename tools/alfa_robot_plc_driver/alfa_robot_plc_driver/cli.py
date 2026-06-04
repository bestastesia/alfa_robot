"""Command-line interface for the ALFA PLC driver."""

from __future__ import annotations

import argparse
import sys

from .config import PlcConnectionConfig, PlcProtocolConfig
from .driver import PlcDriver
from .mock import MockPlcTransport


def parse_axis_values(text: str) -> dict[int, float]:
    result: dict[int, float] = {}
    for part in text.split(","):
        axis_text, value_text = part.split(":", 1)
        result[int(axis_text.strip().removeprefix("Axis").removeprefix("axis"))] = float(value_text.strip())
    return result


def build_driver(args) -> PlcDriver:
    protocol = PlcProtocolConfig(max_axes=args.max_axes)
    if args.mock:
        return PlcDriver(protocol=protocol, transport=MockPlcTransport(protocol=protocol, active_axes=args.mock_active_axes))
    return PlcDriver(
        connection=PlcConnectionConfig(
            host=args.ip,
            port=args.port,
            unit_id=args.unit,
            timeout_s=args.timeout,
        ),
        protocol=protocol,
    )


def print_system(system) -> None:
    print(f"MB_SYS raw={list(system.raw_words)}")
    print(
        "protocol_mode={} active_axes={} max_capacity={} block_words={} position_scale={} speed_scale={} cmd_base={} sts_base={} sys_base={}".format(
            system.protocol_mode,
            system.active_axis_count,
            system.max_axis_capacity,
            system.axis_block_size,
            system.position_scale,
            system.speed_scale,
            system.cmd_base_mw,
            system.sts_base_mw,
            system.sys_base_mw,
        )
    )


def print_axis(status) -> None:
    print(
        "Axis{} cmd_base={} sts_base={} | CMD cw={} 0x{:04X}, id={}, target={:.2f}, vel={:.2f} | "
        "STS sw={} 0x{:04X}, ack={}, err={}, fb={:.2f}, single={:.2f}, last={:.2f}".format(
            status.axis,
            status.cmd_base,
            status.sts_base,
            status.control_word,
            status.control_word,
            status.command_id,
            status.command_target_deg,
            status.velocity,
            status.status_word,
            status.status_word,
            status.ack_command_id,
            status.error_code,
            status.feedback_pos_deg,
            status.feedback_single_deg,
            status.last_target_deg,
        )
    )


def confirm_write(args, message: str) -> bool:
    if not args.yes_write:
        print("未写入。确认安全后加 --yes-write。")
        return False
    input(message + "；Ctrl+C 取消...")
    return True


def command_status(args) -> None:
    driver = build_driver(args)
    print_system(driver.read_system())
    axes = [args.axis] if args.axis else None
    for status in driver.read_axes(axes):
        print_axis(status)


def command_read_angles(args) -> None:
    driver = build_driver(args)
    system = driver.read_system()
    print(f"ActiveAxisCount={system.active_axis_count}")
    print("axis,feedback_pos_deg,single_deg,last_target_deg,ack_command_id,error_code,status_word")
    for status in driver.read_axes():
        print(
            f"Axis{status.axis},{status.feedback_pos_deg:.2f},{status.feedback_single_deg:.2f},{status.last_target_deg:.2f},{status.ack_command_id},{status.error_code},0x{status.status_word:04X}"
        )


def command_move_abs(args) -> None:
    targets = parse_axis_values(args.targets)
    driver = build_driver(args)
    print("Before:")
    for status in driver.read_axes(sorted(targets)):
        print_axis(status)
    print("Plan:")
    for axis, target in targets.items():
        print(f"  Axis{axis}: target={target:.2f} deg")
    if not confirm_write(args, "确认机械臂安全、人员远离、可以执行绝对运动后按 Enter"):
        return
    result = driver.move_abs(
        targets,
        velocity=args.vel,
        acceleration=args.acc,
        deceleration=args.dec,
        emergency_deceleration=args.emergency_dec,
    )
    print("After command:")
    for command in result:
        print(f"Sent Axis{command.axis} target={command.target_deg:.2f}deg CommandID={command.command_id}")
        print_axis(command.status)


def command_move_delta(args) -> None:
    deltas = parse_axis_values(args.deltas)
    driver = build_driver(args)
    current = {status.axis: status for status in driver.read_axes(sorted(deltas))}
    print("Plan:")
    for axis, delta in deltas.items():
        target = current[axis].feedback_single_deg + delta
        print(f"  Axis{axis}: current={current[axis].feedback_single_deg:.2f} deg, delta={delta:+.2f} deg -> target={target:.2f} deg")
    if not confirm_write(args, "确认机械臂安全、人员远离、可以执行增量运动后按 Enter"):
        return
    result = driver.move_delta(
        deltas,
        velocity=args.vel,
        acceleration=args.acc,
        deceleration=args.dec,
        emergency_deceleration=args.emergency_dec,
    )
    print("After command:")
    for command in result:
        print(f"Sent Axis{command.axis} target={command.target_deg:.2f}deg CommandID={command.command_id}")
        print_axis(command.status)


def command_clear(args) -> None:
    driver = build_driver(args)
    if args.all:
        if not confirm_write(args, "确认要清所有 active axes 的 ControlWord=0 后按 Enter"):
            return
        result = driver.clear_commands()
    else:
        if args.axis is None:
            raise SystemExit("clear requires --axis or --all")
        if not confirm_write(args, f"确认要清 Axis{args.axis} ControlWord=0 后按 Enter"):
            return
        result = [driver.clear_command(args.axis)]
    for status in result:
        print_axis(status)


def add_common(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--ip", default="192.168.1.88")
    parser.add_argument("--port", type=int, default=502)
    parser.add_argument("--unit", type=int, default=255)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--max-axes", type=int, default=12)
    parser.add_argument("--mock", action="store_true", help="use in-memory mock PLC")
    parser.add_argument("--mock-active-axes", type=int, default=6)


def add_motion(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--vel", type=float, default=3.0)
    parser.add_argument("--acc", type=float, default=10.0)
    parser.add_argument("--dec", type=float, default=10.0)
    parser.add_argument("--emergency-dec", type=float, default=30.0)
    parser.add_argument("--yes-write", action="store_true")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="ALFA PLC Modbus driver CLI")
    add_common(parser)
    sub = parser.add_subparsers(dest="command", required=True)

    status = sub.add_parser("status")
    status.add_argument("--axis", type=int)
    status.set_defaults(func=command_status)

    read_angles = sub.add_parser("read-angles")
    read_angles.set_defaults(func=command_read_angles)

    move_abs = sub.add_parser("move-abs")
    move_abs.add_argument("--targets", required=True)
    add_motion(move_abs)
    move_abs.set_defaults(func=command_move_abs)

    move_delta = sub.add_parser("move-delta")
    move_delta.add_argument("--deltas", required=True)
    add_motion(move_delta)
    move_delta.set_defaults(func=command_move_delta)

    clear = sub.add_parser("clear")
    clear.add_argument("--axis", type=int)
    clear.add_argument("--all", action="store_true")
    clear.add_argument("--yes-write", action="store_true")
    clear.set_defaults(func=command_clear)

    args = parser.parse_args(argv)
    args.func(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
