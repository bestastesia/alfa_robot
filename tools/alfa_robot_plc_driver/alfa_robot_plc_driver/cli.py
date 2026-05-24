from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .axis import StopPolicy
from .config import load_config, require_complete_addresses
from .driver import PlcDriver


def parse_targets(text: str) -> dict[int, float]:
    targets: dict[int, float] = {}
    for item in text.split(","):
        if not item.strip():
            continue
        axis_text, value_text = item.split(":", 1)
        targets[int(axis_text)] = float(value_text)
    if not targets:
        raise argparse.ArgumentTypeError("targets must contain at least one axis:value pair")
    return targets


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ALFA Robot PLC/Modbus 12-axis test driver")
    parser.add_argument("--config", default="config/plc_modbus_map.example.yaml", help="PLC Modbus map YAML")
    parser.add_argument("--mock", action="store_true", help="Use in-memory mock PLC runtime")
    parser.add_argument("--dry-run", action="store_true", help="Validate config and print planned action without connecting")

    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("status", help="Read all axis status")

    for command in ("enable", "disable", "stop", "emergency-stop", "reset-fault", "reset-emergency", "set-zero", "return-zero"):
        item = subparsers.add_parser(command, help=f"{command} one axis or all axes")
        target = item.add_mutually_exclusive_group(required=True)
        target.add_argument("--axis", type=int)
        target.add_argument("--all", action="store_true")

    move = subparsers.add_parser("move", help="Move one axis to a single-turn target degree")
    move.add_argument("--axis", type=int, required=True)
    move.add_argument("--deg", type=float, required=True)
    move.add_argument("--timeout", type=float, default=10.0)

    move_many = subparsers.add_parser("move-many", help="Move multiple axes to their own target degrees")
    move_many.add_argument("--targets", type=parse_targets, required=True, help="Comma-separated axis:deg list, e.g. 1:10,2:-20")
    move_many.add_argument("--timeout", type=float, default=10.0)
    move_many.add_argument("--stop-policy", choices=[item.value for item in StopPolicy], default=StopPolicy.STOP.value)

    return parser


def make_driver(args) -> PlcDriver:
    config = load_config(args.config)
    if args.dry_run:
        print(f"DRY-RUN config={Path(args.config)} command={args.command}")
        return PlcDriver.for_mock(config)
    if args.mock:
        return PlcDriver.for_mock(config)
    require_complete_addresses(config)
    return PlcDriver.for_real_plc(config)


def selected_axes(driver: PlcDriver, args) -> list[int]:
    if getattr(args, "all", False):
        return sorted(driver.axes)
    return [int(args.axis)]


def print_status(driver: PlcDriver) -> None:
    driver.process_mock_once()
    for axis_id in sorted(driver.axes):
        status = driver.axis(axis_id).read_status()
        print(
            f"axis{axis_id:02d} {status.joint_name:16s} "
            f"power={int(status.power_status)} emergency={int(status.emergency_latched)} "
            f"active={status.active_cmd_type} move={status.move_cmd_id} "
            f"done={status.move_cmd_done_id} error={status.move_cmd_error_id} "
            f"pos_deg={status.feedback_pos_deg:.3f}"
        )


def run_command(driver: PlcDriver, args) -> int:
    command = args.command
    if command == "status":
        print_status(driver)
        return 0

    if command == "move":
        axis = driver.axis(args.axis)
        result = axis.move_to_deg(args.deg, done_timeout_s=args.timeout)
        print(f"axis{result.axis_id} move command_id={result.command_id} ok={result.ok} reason={result.reason}")
        return 0 if result.ok else 1

    if command == "move-many":
        result = driver.move_many(args.targets, StopPolicy(args.stop_policy), done_timeout_s=args.timeout)
        for axis_id in sorted(result.results):
            item = result.results[axis_id]
            print(f"axis{axis_id} move command_id={item.command_id} ok={item.ok} reason={item.reason}")
        if result.stopped_axes:
            print(f"stopped_axes={','.join(str(axis_id) for axis_id in result.stopped_axes)}")
        return 0 if result.ok else 1

    action_map = {
        "enable": "enable",
        "disable": "disable",
        "stop": "stop",
        "emergency-stop": "emergency_stop",
        "reset-fault": "reset_fault",
        "reset-emergency": "reset_emergency",
        "set-zero": "set_zero",
        "return-zero": "return_zero",
    }
    method_name = action_map[command]
    ok = True
    for axis_id in selected_axes(driver, args):
        axis = driver.axis(axis_id)
        method = getattr(axis, method_name)
        if command == "return-zero":
            result = method()
            print(f"axis{axis_id} return-zero command_id={result.command_id} ok={result.ok} reason={result.reason}")
            ok = ok and result.ok
        else:
            method()
            print(f"axis{axis_id} {command}: sent")
    return 0 if ok else 1


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    driver = make_driver(args)
    try:
        return run_command(driver, args)
    finally:
        driver.close()


if __name__ == "__main__":
    raise SystemExit(main())
