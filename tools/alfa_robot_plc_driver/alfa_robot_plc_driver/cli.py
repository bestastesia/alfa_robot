"""Command-line interface for the ALFA PLC driver."""

from __future__ import annotations

import argparse
import sys
import time

from .config import PlcConnectionConfig, PlcProtocolConfig
from .driver import PlcDriver
from .mock import MockPlcTransport


def parse_axis_values(text: str) -> dict[int, float]:
    result: dict[int, float] = {}
    for part in text.split(","):
        axis_text, value_text = part.split(":", 1)
        result[int(axis_text.strip().removeprefix("Axis").removeprefix("axis"))] = float(value_text.strip())
    return result


def parse_axes(text: str) -> list[int]:
    axes: list[int] = []
    for part in text.split(","):
        item = part.strip().removeprefix("Axis").removeprefix("axis")
        if not item:
            continue
        axes.append(int(item))
    return axes


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


def command_smoke_12(args) -> None:
    driver = build_driver(args)
    system = driver.read_system()
    print_system(system)
    if system.active_axis_count < args.axes:
        raise SystemExit(f"PLC only reports {system.active_axis_count} active axes; refusing {args.axes}-axis smoke test")

    axes = list(range(1, args.axes + 1))
    before = {status.axis: status for status in driver.read_axes(axes)}
    deltas = {axis: args.delta for axis in axes}

    print("Before:")
    for axis in axes:
        print_axis(before[axis])

    print("Plan:")
    for axis in axes:
        target = before[axis].feedback_single_deg + deltas[axis]
        print(f"  Axis{axis}: current={before[axis].feedback_single_deg:.2f} deg, delta={deltas[axis]:+.2f} deg -> target={target:.2f} deg")

    if not confirm_write(args, f"确认 1-{args.axes} 轴机械安全、人员远离、可以执行小增量联动后按 Enter"):
        return

    result = driver.move_delta(
        deltas,
        velocity=args.vel,
        acceleration=args.acc,
        deceleration=args.dec,
        emergency_deceleration=args.emergency_dec,
    )
    targets = {command.axis: command.target_deg for command in result}
    print("After command:")
    for command in result:
        print(f"Sent Axis{command.axis} target={command.target_deg:.2f}deg CommandID={command.command_id}")

    deadline = time.monotonic() + args.monitor_s
    sample = 0
    while True:
        sample += 1
        statuses = driver.read_axes(axes)
        print(f"Sample {sample}:")
        for status in statuses:
            print(
                f"  Axis{status.axis}: fb={status.feedback_single_deg:.2f} deg, "
                f"target={targets[status.axis]:.2f} deg, ack={status.ack_command_id}, err={status.error_code}, sw=0x{status.status_word:04X}"
            )
        if time.monotonic() >= deadline:
            break
        time.sleep(args.interval_s)


def command_stream_abs(args) -> None:
    if args.hz <= 0:
        raise SystemExit("--hz must be > 0")
    if args.duration_s <= 0:
        raise SystemExit("--duration-s must be > 0")
    if abs(args.speed) <= 0:
        raise SystemExit("--speed must be non-zero")

    axes = parse_axes(args.axes)
    driver = build_driver(args)
    system = driver.read_system()
    print_system(system)
    if max(axes) > system.active_axis_count:
        raise SystemExit(f"PLC only reports {system.active_axis_count} active axes; refusing Axis{max(axes)}")

    start_statuses = {status.axis: status for status in driver.read_axes(axes)}
    start_positions = {axis: start_statuses[axis].feedback_single_deg for axis in axes}
    max_delta = abs(args.speed) * args.duration_s
    if max_delta > args.max_delta:
        raise SystemExit(
            f"planned max delta {max_delta:.2f} deg exceeds --max-delta {args.max_delta:.2f} deg; reduce speed/duration or raise limit explicitly"
        )

    print("Before:")
    for axis in axes:
        print_axis(start_statuses[axis])
    print("Plan:")
    print(
        f"  axes={axes}, stream_speed={args.speed:+.2f} deg/s, hz={args.hz:.2f}, "
        f"duration={args.duration_s:.2f}s, PLC profile vel={args.vel:.2f} deg/s"
    )
    for axis in axes:
        target = start_positions[axis] + args.speed * args.duration_s
        print(f"  Axis{axis}: start={start_positions[axis]:.2f} deg -> final_target={target:.2f} deg")

    if not confirm_write(args, "确认机械安全、人员远离、可以执行连续绝对位置下发后按 Enter"):
        return

    period = 1.0 / args.hz
    feedback_period = 1.0 / args.feedback_hz if args.feedback_hz > 0 else None
    start_time = time.monotonic()
    next_feedback = start_time
    next_tick = start_time
    command_count = 0
    if hasattr(driver.transport, "open"):
        driver.transport.open()
    command_ids = driver.prepare_move_abs_stream(
        axes,
        velocity=args.vel,
        acceleration=args.acc,
        deceleration=args.dec,
        emergency_deceleration=args.emergency_dec,
    )
    while True:
        now = time.monotonic()
        elapsed = min(now - start_time, args.duration_s)
        targets = {axis: start_positions[axis] + args.speed * elapsed for axis in axes}
        driver.stream_move_abs_tick(targets, command_ids)
        command_count += 1
        command_ids = driver.next_command_ids(command_ids)
        should_read_feedback = feedback_period is not None and (now >= next_feedback or elapsed >= args.duration_s)
        if should_read_feedback:
            statuses = {status.axis: status for status in driver.read_axes(axes)}
            summary = ", ".join(
                f"A{axis}:target={targets[axis]:.2f},ack={statuses[axis].ack_command_id},fb={statuses[axis].feedback_single_deg:.2f},err={statuses[axis].error_code}"
                for axis in axes
            )
            print(f"t={elapsed:.2f}s cmd#{command_count}: {summary}")
            next_feedback += feedback_period
        if elapsed >= args.duration_s:
            break
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
    if hasattr(driver.transport, "close"):
        driver.transport.close()

    print("Final readback:")
    for status in driver.read_axes(axes):
        print_axis(status)


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


def command_control_word(args) -> None:
    driver = build_driver(args)
    system = driver.read_system()
    axes = list(range(1, system.active_axis_count + 1)) if args.all else parse_axes(args.axes)
    print_system(system)
    print("Before:")
    for status in driver.read_axes(axes):
        print_axis(status)
    if not confirm_write(args, f"确认要对 axes={axes} 写 {args.label} ControlWord={args.control_word} / 0x{args.control_word:04X} 后按 Enter"):
        return
    result = driver.write_control_word(axes, args.control_word)
    print("After:")
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

    smoke_12 = sub.add_parser("smoke-12", help="read current positions, then send a small synchronous delta to 1..N axes")
    smoke_12.add_argument("--axes", type=int, default=12)
    smoke_12.add_argument("--delta", type=float, default=0.1)
    smoke_12.add_argument("--monitor-s", type=float, default=5.0)
    smoke_12.add_argument("--interval-s", type=float, default=0.5)
    add_motion(smoke_12)
    smoke_12.set_defaults(func=command_smoke_12)

    stream_abs = sub.add_parser("stream-abs", help="periodically send absolute targets to approximate velocity control")
    stream_abs.add_argument("--axes", default="1", help="comma separated axes, e.g. 1 or 1,2,3")
    stream_abs.add_argument("--speed", type=float, required=True, help="outer-loop target speed in deg/s; sign controls direction")
    stream_abs.add_argument("--duration-s", type=float, default=2.0)
    stream_abs.add_argument("--hz", type=float, default=5.0)
    stream_abs.add_argument("--feedback-hz", type=float, default=2.0, help="read/print feedback rate; set 0 to only read at the end")
    stream_abs.add_argument("--max-delta", type=float, default=5.0)
    stream_abs.add_argument("--print-every", type=int, default=1)
    add_motion(stream_abs)
    stream_abs.set_defaults(func=command_stream_abs)

    clear = sub.add_parser("clear")
    clear.add_argument("--axis", type=int)
    clear.add_argument("--all", action="store_true")
    clear.add_argument("--yes-write", action="store_true")
    clear.set_defaults(func=command_clear)

    estop = sub.add_parser("estop", help="write Enable + EmergencyStop control word")
    estop.add_argument("--axes", default="1", help="comma separated axes, e.g. 1 or 1,2,3")
    estop.add_argument("--all", action="store_true")
    estop.add_argument("--yes-write", action="store_true")
    estop.set_defaults(func=command_control_word, control_word=257, label="EmergencyStop")

    reset_estop = sub.add_parser("reset-estop", help="write Enable + ResetEmergency, then manually return to normal if needed")
    reset_estop.add_argument("--axes", default="1", help="comma separated axes, e.g. 1 or 1,2,3")
    reset_estop.add_argument("--all", action="store_true")
    reset_estop.add_argument("--yes-write", action="store_true")
    reset_estop.set_defaults(func=command_control_word, control_word=1025, label="ResetEmergency")

    reset_fault = sub.add_parser("reset-fault", help="write Enable + ResetFault")
    reset_fault.add_argument("--axes", default="1", help="comma separated axes, e.g. 1 or 1,2,3")
    reset_fault.add_argument("--all", action="store_true")
    reset_fault.add_argument("--yes-write", action="store_true")
    reset_fault.set_defaults(func=command_control_word, control_word=3, label="ResetFault")

    args = parser.parse_args(argv)
    args.func(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
