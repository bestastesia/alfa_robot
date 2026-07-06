#!/usr/bin/env python3
"""Visualize box_stack_dual_ik_benchmark JSONL with Rerun."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
import rerun as rr


def load_rerun_helpers():
    helper_path = Path(__file__).with_name("visualize_rerun.py")
    spec = importlib.util.spec_from_file_location("alfa_visualize_rerun_helpers", helper_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load helpers: {helper_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def read_jsonl(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]], dict[str, Any]]:
    header: dict[str, Any] = {}
    rounds: list[dict[str, Any]] = []
    summary: dict[str, Any] = {}
    with path.open() as file:
        for line in file:
            if not line.strip():
                continue
            row = json.loads(line)
            if row.get("type") == "header":
                header = row
            elif row.get("type") == "round":
                rounds.append(row)
            elif row.get("type") == "summary":
                summary = row
    return header, rounds, summary


def all_boxes(box_x: float) -> dict[int, tuple[float, float, float]]:
    rows = [
        [(1, 0.6), (3, 0.2), (2, -0.2), (4, -0.6)],
        [(5, 0.6), (7, 0.2), (6, -0.2), (8, -0.6)],
        [(9, 0.6), (11, 0.2), (10, -0.2), (12, -0.6)],
        [(13, 0.6), (15, 0.2), (14, -0.2), (16, -0.6)],
        [(17, 0.6), (19, 0.2), (18, -0.2), (20, -0.6)],
    ]
    out: dict[int, tuple[float, float, float]] = {}
    for row_i, row in enumerate(rows):
        z = 0.2 + 0.4 * (len(rows) - 1 - row_i)
        for box_id, y in row:
            out[box_id] = (box_x, y, z)
    return out


def transform_point(transform: np.ndarray, point: list[float]) -> list[float]:
    homogeneous = transform @ np.array([point[0], point[1], point[2], 1.0], dtype=float)
    return [float(homogeneous[0]), float(homogeneous[1]), float(homogeneous[2])]


def transform_vector(transform: np.ndarray, vector: list[float]) -> list[float]:
    rotated = transform[:3, :3] @ np.array(vector, dtype=float)
    return [float(rotated[0]), float(rotated[1]), float(rotated[2])]


def log_box_stack(box_x: float, grabbed_ids: set[int]) -> None:
    centers = []
    half_sizes = []
    colors = []
    labels = []
    for box_id, (x, y, z) in sorted(all_boxes(box_x).items()):
        # The benchmark target is the front-face grasp point facing the robot.
        # Box volume extends backward from that surface by 0.3 m along +X.
        centers.append([x + 0.15, y, z])
        half_sizes.append([0.15, 0.2, 0.2])
        if box_id in grabbed_ids:
            colors.append([255, 180, 60, 160])
        else:
            colors.append([80, 80, 80, 80])
        labels.append(str(box_id))
    rr.log("scene/boxes", rr.Boxes3D(centers=centers, half_sizes=half_sizes, colors=colors, labels=labels), static=True)


def log_point(path: str, point: list[float], color: list[int], label: str, radius: float = 0.035) -> None:
    rr.log(path, rr.Points3D([point], colors=[color], radii=[radius], labels=[label]))


def log_arrow(path: str, origin: list[float], vector: list[float], color: list[int]) -> None:
    rr.log(path, rr.Arrows3D(origins=[origin], vectors=[vector], colors=[color], radii=[0.01]))


def log_tool0_extension(robot: Any, positions: dict[str, float], robot_path: str) -> None:
    transforms = robot.fk(positions)
    specs = [
        ("left", "leftjoint6", "left_tool0", [0, 220, 255]),
        ("right", "rightjoint6", "right_tool0", [255, 120, 0]),
    ]
    for side, link6, tool0, color in specs:
        if link6 not in transforms or tool0 not in transforms:
            continue
        link6_pos = transforms[link6][:3, 3].tolist()
        tool0_pos = transforms[tool0][:3, 3].tolist()
        vector = (transforms[tool0][:3, 3] - transforms[link6][:3, 3]).tolist()
        rr.log(
            f"{robot_path}/{side}_tool0_extension",
            rr.Arrows3D(origins=[link6_pos], vectors=[vector], colors=[color], radii=[0.018]),
        )
        rr.log(
            f"{robot_path}/{side}_tool0_tip_marker",
            rr.Points3D([tool0_pos], colors=[color], radii=[0.045], labels=[f"{side}_tool0"]),
        )


def joint_positions(record: dict[str, Any]) -> tuple[dict[str, float] | None, str]:
    names = record.get("selected_joint_names") or []
    values = record.get("selected_joint_values") or []
    if names and len(names) == len(values):
        return {str(name): float(value) for name, value in zip(names, values)}, "selected"

    names = record.get("best_rejected_joint_names") or []
    values = record.get("best_rejected_joint_values") or []
    if names and len(names) == len(values):
        return {str(name): float(value) for name, value in zip(names, values)}, "best_rejected"

    return None, "home"


def home_positions() -> dict[str, float]:
    arm = [0.0, math.radians(15), math.radians(135), 0.0, math.radians(60), 0.0]
    names = [
        "updown",
        "leftjoint1", "leftjoint2", "leftjoint3", "leftjoint4", "leftjoint5", "leftjoint6",
        "rightjoint1", "rightjoint2", "rightjoint3", "rightjoint4", "rightjoint5", "rightjoint6",
    ]
    return dict(zip(names, [0.45] + arm + arm))


def log_round_text(record: dict[str, Any], summary: dict[str, Any]) -> None:
    lines = [
        f"round: {record.get('round')}  boxes: {record.get('left_box')} + {record.get('right_box')}",
        f"success: {record.get('success')}  solver: {record.get('solver_mode')}",
        f"h_interval: [{record.get('h_interval_lower')}, {record.get('h_interval_upper')}]  selected_h: {record.get('selected_h')}",
        f"legal/trials: {record.get('legal_count')} / {record.get('trial_count')}",
        f"selected_score: {record.get('selected_score')}",
        f"failure_reason: {record.get('failure_reason')}",
        f"best_rejected: {record.get('best_rejected_reason')}  pos_err={record.get('best_rejected_direct_pos_error')}  h={record.get('best_rejected_h')}",
        f"rejection_summary: {record.get('rejection_summary')}",
        f"summary: {summary.get('success_rounds')} / {summary.get('rounds')} success",
    ]
    rr.log("info/round", rr.TextLog("\n".join(lines)))


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize box stack dual IK benchmark JSONL")
    parser.add_argument("jsonl", type=Path)
    parser.add_argument("--save", type=Path, default=None)
    parser.add_argument("--connect", action="store_true")
    parser.add_argument("--no-meshes", action="store_true")
    parser.add_argument("--robot-path", default="robot")
    args = parser.parse_args()

    helpers = load_rerun_helpers()
    header, rounds, summary = read_jsonl(args.jsonl)
    if not rounds:
        raise SystemExit(f"no round records in {args.jsonl}")

    if args.save:
        rr.init("box_stack_dual_ik", recording_id=f"box_stack_dual_ik_{args.jsonl.stem}")
        rr.save(str(args.save))
    elif args.connect:
        rr.init("box_stack_dual_ik", recording_id=f"box_stack_dual_ik_{args.jsonl.stem}")
        rr.connect()
    else:
        rr.init("box_stack_dual_ik", recording_id=f"box_stack_dual_ik_{args.jsonl.stem}")
        rr.spawn()

    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    robot = helpers.UrdfRobot(helpers.render_current_urdf())
    helpers.log_robot_static_model(robot, args.robot_path, log_meshes=not args.no_meshes)
    base_to_world = robot.fk({}).get("base_link", np.eye(4))

    grabbed_ids = {int(r["left_box"]) for r in rounds} | {int(r["right_box"]) for r in rounds}
    log_box_stack(float(header.get("box_x", 0.5)), grabbed_ids)

    for index, record in enumerate(rounds):
        helpers.set_sample_time(index)
        positions, pose_source = joint_positions(record)
        current_positions = positions or home_positions()
        helpers.log_robot_state(robot, current_positions, args.robot_path)
        log_tool0_extension(robot, current_positions, args.robot_path)
        rr.log("info/pose_source", rr.TextLog(f"robot pose source: {pose_source}"))

        left = [float(v) for v in record.get("left_target", [])]
        right = [float(v) for v in record.get("right_target", [])]
        grasp_mode = str(record.get("grasp_mode", "front"))
        grasp_vector_base = [0.0, 0.0, -0.18] if grasp_mode == "top_suction" else [0.18, 0.0, 0.0]
        grasp_vector = transform_vector(base_to_world, grasp_vector_base)
        if len(left) == 3:
            left_world = transform_point(base_to_world, left)
            log_point("targets/left_grasp_world", left_world, [0, 220, 255], f"L{record.get('left_box')} {grasp_mode}")
            log_arrow("targets/left_grasp_world_forward", left_world, grasp_vector, [0, 220, 255])
        if len(right) == 3:
            right_world = transform_point(base_to_world, right)
            log_point("targets/right_grasp_world", right_world, [255, 120, 0], f"R{record.get('right_box')} {grasp_mode}")
            log_arrow("targets/right_grasp_world_forward", right_world, grasp_vector, [255, 120, 0])
        log_round_text(record, summary)

    print(f"Loaded {len(rounds)} rounds from {args.jsonl}")
    if args.save:
        print(f"saved: {args.save}")


if __name__ == "__main__":
    main()
