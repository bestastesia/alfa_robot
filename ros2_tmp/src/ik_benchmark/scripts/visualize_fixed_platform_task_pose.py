#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import rerun as rr

HELPER = Path(__file__).resolve().parents[4] / "scripts" / "ik_benchmark" / "scripts" / "visualize_rerun.py"
if not HELPER.exists():
    HELPER = Path("/mnt/mydisk/ALFA/alfa_robot/scripts/ik_benchmark/scripts/visualize_rerun.py")
spec = importlib.util.spec_from_file_location("alfa_visualize_rerun_helpers", HELPER)
if spec is None or spec.loader is None:
    raise RuntimeError(f"cannot load helper {HELPER}")
helpers = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = helpers
spec.loader.exec_module(helpers)

WORLD_TO_BASE_Z = 0.202094


def boxes(front_x: float) -> dict[int, tuple[float, float, float]]:
    rows = [[1, 3, 2, 4], [5, 7, 6, 8], [9, 11, 10, 12], [13, 15, 14, 16], [17, 19, 18, 20]]
    y_by_id = {
        1: 0.6, 3: 0.2, 2: -0.2, 4: -0.6,
        5: 0.6, 7: 0.2, 6: -0.2, 8: -0.6,
        9: 0.6, 11: 0.2, 10: -0.2, 12: -0.6,
        13: 0.6, 15: 0.2, 14: -0.2, 16: -0.6,
        17: 0.6, 19: 0.2, 18: -0.2, 20: -0.6,
    }
    out: dict[int, tuple[float, float, float]] = {}
    for row_i, row in enumerate(rows):
        z = 0.2 + 0.4 * (len(rows) - 1 - row_i)
        for box_id in row:
            out[box_id] = (front_x, y_by_id[box_id], z)
    return out


def home_positions(updown: float) -> dict[str, float]:
    arm = [0.0, math.radians(5.0), math.radians(145.0), 0.0, math.radians(120.0), 0.0]
    names = [
        "turn", "updown",
        "left_v5_joint1", "left_v5_joint2", "left_v5_joint3", "left_v5_joint4", "left_v5_joint5", "left_v5_joint6",
        "right_v5_joint1", "right_v5_joint2", "right_v5_joint3", "right_v5_joint4", "right_v5_joint5", "right_v5_joint6",
    ]
    return dict(zip(names, [0.0, updown] + arm + arm))


def log_points(path: str, pts: list[list[float]], labels: list[str], color: list[int], radius: float = 0.035) -> None:
    rr.log(path, rr.Points3D(pts, labels=labels, colors=[color] * len(pts), radii=[radius] * len(pts)))


def log_arrows(path: str, pts: list[list[float]], vectors: list[list[float]], color: list[int]) -> None:
    rr.log(path, rr.Arrows3D(origins=pts, vectors=vectors, colors=[color] * len(pts), radii=[0.012] * len(pts)))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--front-x", type=float, default=0.76)
    parser.add_argument("--left-box", type=int, default=5)
    parser.add_argument("--right-box", type=int, default=6)
    parser.add_argument("--updown", type=float, default=0.18)
    parser.add_argument("--save", type=Path, default=Path("data/ik_benchmark/fixed_platform_debug/box_5_6_pose_check.rrd"))
    parser.add_argument("--spawn", action="store_true")
    args = parser.parse_args()

    rr.init("fixed_platform_task_pose_check", spawn=args.spawn)
    if hasattr(rr, "set_time_sequence"):
        rr.set_time_sequence("frame", 0)
    elif hasattr(rr, "set_time"):
        rr.set_time("frame", sequence=0)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    urdf_text = helpers.render_current_urdf()
    robot = helpers.UrdfRobot(urdf_text)
    helpers.log_robot_static_model(robot, "world/robot", log_meshes=True)
    positions = home_positions(args.updown)
    helpers.log_robot_state(robot, positions, "world/robot")
    transforms = robot.fk(positions)

    key_links = ["base_link", "updown", "left_v5_link1", "right_v5_link1", "left_v5_tool0", "right_v5_tool0"]
    for link in key_links:
        if link in transforms:
            p = transforms[link][:3, 3].tolist()
            log_points(f"world/key_links/{link}", [p], [link], [255, 255, 255], 0.035)
            axis_x = (transforms[link][:3, :3] @ np.array([0.12, 0.0, 0.0])).tolist()
            axis_z = (transforms[link][:3, :3] @ np.array([0.0, 0.0, 0.12])).tolist()
            log_arrows(f"world/key_links/{link}_x_axis", [p], [axis_x], [255, 0, 0])
            log_arrows(f"world/key_links/{link}_z_axis", [p], [axis_z], [0, 120, 255])

    all_boxes = boxes(args.front_x)
    centers = []
    half_sizes = []
    colors = []
    labels = []
    for box_id, (x, y, z) in sorted(all_boxes.items()):
        centers.append([x + 0.15, y, z])
        half_sizes.append([0.15, 0.2, 0.2])
        labels.append(str(box_id))
        colors.append([255, 180, 40, 90] if box_id in {args.left_box, args.right_box} else [100, 100, 100, 45])
    rr.log("world/box_stack", rr.Boxes3D(centers=centers, half_sizes=half_sizes, labels=labels, colors=colors), static=True)

    left_world = list(all_boxes[args.left_box])
    right_world = list(all_boxes[args.right_box])
    left_base_hardcoded = [left_world[0], left_world[1], left_world[2] - WORLD_TO_BASE_Z]
    right_base_hardcoded = [right_world[0], right_world[1], right_world[2] - WORLD_TO_BASE_Z]

    log_points("world/targets/world_frame", [left_world, right_world], [f"L{args.left_box} world", f"R{args.right_box} world"], [0, 255, 0], 0.05)
    log_arrows("world/targets/world_frame/+x", [left_world, right_world], [[0.16, 0, 0], [0.16, 0, 0]], [0, 255, 0])
    log_points("world/targets/current_hardcoded_base_z", [left_base_hardcoded, right_base_hardcoded], [f"L{args.left_box} z-0.202", f"R{args.right_box} z-0.202"], [255, 0, 255], 0.045)
    log_arrows("world/targets/current_hardcoded_base_z/+x", [left_base_hardcoded, right_base_hardcoded], [[0.16, 0, 0], [0.16, 0, 0]], [255, 0, 255])

    rr.log(
        "world/notes",
        rr.TextDocument(
            f"fixed updown={args.updown}\n"
            f"orange boxes: selected {args.left_box}/{args.right_box}\n"
            f"green points: task coordinates as published (front_x={args.front_x})\n"
            f"magenta points: current orchestrator hardcoded z -= {WORLD_TO_BASE_Z}\n"
            "white markers: base_link/updown/link1/tool0 FK at home pose"
        ),
    )

    args.save.parent.mkdir(parents=True, exist_ok=True)
    rr.save(str(args.save))
    print(f"saved {args.save}")
    print(f"L{args.left_box} world/base-hardcoded: {left_world} / {left_base_hardcoded}")
    print(f"R{args.right_box} world/base-hardcoded: {right_world} / {right_base_hardcoded}")
    for link in key_links:
        if link in transforms:
            print(f"{link}: {transforms[link][:3, 3].tolist()}")


if __name__ == "__main__":
    main()
