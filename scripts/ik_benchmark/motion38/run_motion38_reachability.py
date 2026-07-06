#!/usr/bin/env python3
"""MOTION-38 current robot + tool0=0.209 reachability runner.

This is a thin Python wrapper around the existing
ros2_ws/src/alfa_robot_moveit_config/scripts/nine_orient_reachability.py.

It does not source ROS setup files. Source ROS/colcon in your terminal first:
  cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
  source install/setup.bash
"""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
ROS_WS = REPO_ROOT / "ros2_ws"
NINE_ORIENT = ROS_WS / "src/alfa_robot_moveit_config/scripts/nine_orient_reachability.py"
VISUALIZER = REPO_ROOT / "scripts/ik_benchmark/scripts/visualize_nine_orient_three_class.py"
OUT_DIR = REPO_ROOT / "data/ik_range/motion38_current_tool0209"

FRONT_CSV = OUT_DIR / "nine_orient_left_kdl_forward_updown0_x0275_0975_y-015_055_z02_14_step005_10ms_collision_classified.csv"
FRONT_RRD = OUT_DIR / "nine_orient_left_kdl_forward_updown0_x0275_0975_y-015_055_z02_14_step005_10ms_collision_classified.rrd"
DOWN_CSV = OUT_DIR / "nine_orient_left_kdl_down_updown0_x065_0925_y-015_055_z02_09_stepx0025_stepyz005_10ms_collision_classified.csv"
DOWN_RRD = OUT_DIR / "nine_orient_left_kdl_down_updown0_x065_0925_y-015_055_z02_09_stepx0025_stepyz005_10ms_collision_classified.rrd"


def ros_args(params: dict[str, str]) -> list[str]:
    args = ["--ros-args"]
    for key, value in params.items():
        args.extend(["-p", f"{key}:={value}"])
    return args


def front_command() -> list[str]:
    return [
        "/usr/bin/python3",
        str(NINE_ORIENT),
        *ros_args({
            "side": "left",
            "reference_frame": "world",
            "min_x": "0.275",
            "max_x": "0.975",
            "min_y": "-0.15",
            "max_y": "0.55",
            "min_z": "0.2",
            "max_z": "1.4",
            "step": "0.05",
            "ik_timeout": "0.01",
            "avoid_collisions": "true",
            "classify_collisions": "true",
            "fixed_updown": "0.0",
            "center_orientation_xyzw": "[0.0, 0.7071, 0.0, 0.7071]",
            "output_csv": str(FRONT_CSV),
        }),
    ]


def down_command() -> list[str]:
    return [
        "/usr/bin/python3",
        str(NINE_ORIENT),
        *ros_args({
            "side": "left",
            "reference_frame": "world",
            "min_x": "0.65",
            "max_x": "0.925",
            "min_y": "-0.15",
            "max_y": "0.55",
            "min_z": "0.2",
            "max_z": "0.9",
            "step": "0.05",
            "step_x": "0.025",
            "step_y": "0.05",
            "step_z": "0.05",
            "ik_timeout": "0.01",
            "avoid_collisions": "true",
            "classify_collisions": "true",
            "fixed_updown": "0.0",
            # tool +Z points to world -Z.
            "center_orientation_xyzw": "[0.0, 1.0, 0.0, 0.0]",
            "output_csv": str(DOWN_CSV),
        }),
    ]


def viz_command(csv_path: Path, rrd_path: Path) -> list[str]:
    return ["/usr/bin/python3", str(VISUALIZER), str(csv_path), "--save", str(rrd_path)]


def print_command(command: list[str]) -> None:
    print("+ " + " ".join(str(part) for part in command))


def run(command: list[str], *, execute: bool) -> None:
    print_command(command)
    if execute:
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        subprocess.run(command, cwd=str(ROS_WS if command[1] == str(NINE_ORIENT) else REPO_ROOT), check=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run MOTION-38 reachability scans with the existing nine_orient script.")
    parser.add_argument("mode", choices=["front", "down", "viz-front", "viz-down", "all-viz"])
    parser.add_argument("--execute", action="store_true", help="Actually run; without this only prints the command.")
    args = parser.parse_args()

    if args.mode == "front":
        run(front_command(), execute=args.execute)
    elif args.mode == "down":
        run(down_command(), execute=args.execute)
    elif args.mode == "viz-front":
        run(viz_command(FRONT_CSV, FRONT_RRD), execute=args.execute)
    elif args.mode == "viz-down":
        run(viz_command(DOWN_CSV, DOWN_RRD), execute=args.execute)
    elif args.mode == "all-viz":
        run(viz_command(FRONT_CSV, FRONT_RRD), execute=args.execute)
        run(viz_command(DOWN_CSV, DOWN_RRD), execute=args.execute)


if __name__ == "__main__":
    main()
