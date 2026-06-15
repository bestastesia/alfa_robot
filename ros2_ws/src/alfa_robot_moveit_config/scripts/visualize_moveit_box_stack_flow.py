#!/usr/bin/env python3
"""Visualize dual_arm_planner MoveIt trajectory JSONL with Rerun."""

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
    candidates = [
        Path(__file__).resolve().parents[4] / "scripts" / "ik_benchmark" / "scripts" / "visualize_rerun.py",
        Path.cwd() / "scripts" / "ik_benchmark" / "scripts" / "visualize_rerun.py",
        Path.cwd().parent / "scripts" / "ik_benchmark" / "scripts" / "visualize_rerun.py",
    ]
    for helper_path in candidates:
        if helper_path.exists():
            spec = importlib.util.spec_from_file_location("alfa_visualize_rerun_helpers", helper_path)
            if spec is None or spec.loader is None:
                continue
            module = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = module
            spec.loader.exec_module(module)
            return module
    raise RuntimeError("cannot find scripts/ik_benchmark/scripts/visualize_rerun.py")


def read_jsonl(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]], dict[str, Any]]:
    header: dict[str, Any] = {}
    stages: list[dict[str, Any]] = []
    summary: dict[str, Any] = {}
    with path.open() as file:
        for line in file:
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            row_type = row.get("type")
            if row_type == "header":
                header = row
            elif row_type == "stage":
                stages.append(row)
            elif row_type == "summary":
                summary = row
    return header, stages, summary


def all_boxes(box_x: float) -> dict[int, tuple[float, float, float]]:
    rows = [
        [(1, 0.8), (2, 0.4), (3, 0.0), (4, -0.4), (5, -0.8)],
        [(6, 0.8), (7, 0.4), (8, 0.0), (9, -0.4), (10, -0.8)],
        [(11, 0.8), (12, 0.4), (13, 0.0), (14, -0.4), (15, -0.8)],
        [(16, 0.8), (17, 0.4), (18, 0.0), (19, -0.4), (20, -0.8)],
        [(21, 0.8), (22, 0.4), (23, 0.0), (24, -0.4), (25, -0.8)],
    ]
    out: dict[int, tuple[float, float, float]] = {}
    for row_i, row in enumerate(rows):
        z = 0.2 + 0.4 * (len(rows) - 1 - row_i)
        for box_id, y in row:
            out[box_id] = (box_x, y, z)
    return out


def log_box_stack(box_x: float) -> None:
    centers = []
    half_sizes = []
    colors = []
    labels = []
    for box_id, (x, y, z) in sorted(all_boxes(box_x).items()):
        centers.append([x + 0.15, y, z])
        half_sizes.append([0.15, 0.2, 0.2])
        colors.append([255, 180, 60, 90])
        labels.append(str(box_id))
    rr.log("scene/boxes", rr.Boxes3D(centers=centers, half_sizes=half_sizes, colors=colors, labels=labels), static=True)


def default_container_obstacle() -> dict[str, Any]:
    thickness = 0.02
    length = 4.0
    width = 2.2
    height = 2.4
    center_x = 0.8
    center_y = 0.0
    floor_z = 0.0
    return {
        "enabled": True,
        "frame": "world",
        "length": length,
        "width": width,
        "height": height,
        "center_x": center_x,
        "center_y": center_y,
        "floor_z": floor_z,
        "wall_thickness": thickness,
        "panels": [
            {
                "id": "container_left_wall",
                "center": [center_x, center_y + width * 0.5 + thickness * 0.5, floor_z + height * 0.5],
                "size": [length, thickness, height],
            },
            {
                "id": "container_right_wall",
                "center": [center_x, center_y - width * 0.5 - thickness * 0.5, floor_z + height * 0.5],
                "size": [length, thickness, height],
            },
            {
                "id": "container_ceiling",
                "center": [center_x, center_y, floor_z + height + thickness * 0.5],
                "size": [length, width + 2.0 * thickness, thickness],
            },
        ],
    }


def log_container_obstacle(config: dict[str, Any] | None) -> None:
    if not config:
        config = default_container_obstacle()
    if not config.get("enabled", True):
        return

    panels = config.get("panels") or default_container_obstacle()["panels"]
    centers = []
    half_sizes = []
    colors = []
    labels = []
    for panel in panels:
        center = panel.get("center", [])
        size = panel.get("size", [])
        if len(center) != 3 or len(size) != 3:
            continue
        centers.append([float(value) for value in center])
        half_sizes.append([float(value) * 0.5 for value in size])
        colors.append([80, 170, 255, 45])
        labels.append(str(panel.get("id", "container")))
    if centers:
        rr.log(
            "scene/container",
            rr.Boxes3D(centers=centers, half_sizes=half_sizes, colors=colors, labels=labels),
            static=True,
        )


def log_static_box_obstacles(config: dict[str, Any] | None) -> None:
    if not config or not config.get("enabled", False):
        rr.log("scene/static_box_obstacles", rr.Boxes3D(centers=[], half_sizes=[]))
        return
    boxes = config.get("boxes", [])
    centers = []
    half_sizes = []
    colors = []
    labels = []
    for box in boxes:
        center = box.get("center", [])
        size = box.get("size", [])
        if len(center) != 3 or len(size) != 3:
            continue
        centers.append([float(value) for value in center])
        half_sizes.append([float(value) * 0.5 for value in size])
        colors.append([170, 80, 255, 130])
        labels.append(str(box.get("id", "static_box_obstacle")))
    rr.log(
        "scene/static_box_obstacles",
        rr.Boxes3D(centers=centers, half_sizes=half_sizes, colors=colors, labels=labels),
    )


def transform_point(transform: np.ndarray, point: list[float]) -> list[float]:
    homogeneous = transform @ np.array([point[0], point[1], point[2], 1.0], dtype=float)
    return [float(homogeneous[0]), float(homogeneous[1]), float(homogeneous[2])]


def joint_dict_from_point(stage: dict[str, Any], point: dict[str, Any]) -> dict[str, float]:
    state_map = stage.get("start_state", {}).get("joint_map", {})
    out = {str(name): float(value) for name, value in state_map.items()}
    names = stage.get("trajectory", {}).get("joint_names", [])
    positions = point.get("positions", [])
    for name, value in zip(names, positions):
        out[str(name)] = float(value)
    return out


def start_point_from_stage(stage: dict[str, Any]) -> dict[str, Any] | None:
    state_map = stage.get("start_state", {}).get("joint_map", {})
    names = stage.get("trajectory", {}).get("joint_names", [])
    if not state_map or not names:
        return None
    try:
        positions = [float(state_map[str(name)]) for name in names]
    except KeyError:
        return None
    return {
        "time_from_start_sec": 0.0,
        "positions": positions,
        "velocities": [0.0 for _ in positions],
    }


def ensure_points_start_at_stage_start(stage: dict[str, Any], points: list[dict[str, Any]]) -> list[dict[str, Any]]:
    start_point = start_point_from_stage(stage)
    if start_point is None:
        return points
    if not points:
        return [start_point]

    first_positions = points[0].get("positions", [])
    start_positions = start_point["positions"]
    if len(first_positions) == len(start_positions):
        max_delta = max(
            abs(float(first) - float(start))
            for first, start in zip(first_positions, start_positions)
        )
        if max_delta < 1e-6:
            return points
    return [start_point, *points]


def interpolate_point(a: dict[str, Any], b: dict[str, Any], ratio: float) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key in ("positions", "velocities"):
        lhs = a.get(key, [])
        rhs = b.get(key, [])
        if len(lhs) == len(rhs):
            out[key] = [
                float(left) * (1.0 - ratio) + float(right) * ratio
                for left, right in zip(lhs, rhs)
            ]
    lhs_time = float(a.get("time_from_start_sec", 0.0))
    rhs_time = float(b.get("time_from_start_sec", lhs_time))
    out["time_from_start_sec"] = lhs_time * (1.0 - ratio) + rhs_time * ratio
    return out


def normalize_playback_points(points: list[dict[str, Any]], repeat_factor: int) -> list[dict[str, Any]]:
    if repeat_factor <= 1 or len(points) <= 1:
        return points
    out: list[dict[str, Any]] = []
    for index in range(len(points) - 1):
        for step in range(repeat_factor):
            out.append(interpolate_point(points[index], points[index + 1], step / repeat_factor))
    out.append(points[-1])
    return out


def log_target_pose(path: str, pose: dict[str, Any], color: list[int], label: str, base_to_world: np.ndarray) -> None:
    position = pose.get("position", [])
    if len(position) != 3:
        return
    world_position = transform_point(base_to_world, [float(v) for v in position])
    rr.log(path, rr.Points3D([world_position], colors=[color], radii=[0.035], labels=[label]))


def log_stage_text(stage: dict[str, Any], point_index: int, point_count: int) -> None:
    extra = stage.get("extra", {})
    ik = extra.get("ik", {})
    selected = ik.get("selected", {})
    lines = [
        f"stage: {stage.get('stage')}",
        f"point: {point_index + 1}/{point_count}",
        f"kind: {extra.get('stage_kind', 'joint_target')}",
        f"ik_strategy: {ik.get('strategy', '')}",
        f"selected_h: {selected.get('h', '')} score: {selected.get('score', '')}",
        f"h_index/seed_index: {selected.get('h_index', '')}/{selected.get('seed_index', '')}",
        f"legal/trials: {ik.get('legal_count', '')}/{ik.get('trial_count', '')} wall_ms={ik.get('wall_ms', '')}",
    ]
    rr.log("info/stage", rr.TextLog("\n".join(lines)))


def log_attached_boxes(
    robot: Any,
    joints: dict[str, float],
    attached_boxes: list[dict[str, Any]],
    path: str = "scene/attached_boxes",
    success: bool = False,
) -> None:
    if not attached_boxes:
        rr.log(path, rr.Clear(recursive=True))
        return

    fk = robot.fk(joints)
    centers = []
    half_sizes = []
    quaternions = []
    colors = []
    labels = []
    for box in attached_boxes:
        link_name = str(box.get("link_name", ""))
        link_tf = fk.get(link_name)
        center_in_link = box.get("center_in_link", [])
        size = box.get("size", [])
        if link_tf is None or len(center_in_link) != 3 or len(size) != 3:
            continue
        world_center = transform_point(link_tf, [float(value) for value in center_in_link])
        centers.append(world_center)
        half_sizes.append([float(value) * 0.5 for value in size])
        quaternions.append(robot_module_matrix_to_quaternion(link_tf[:3, :3]))
        colors.append([40, 220, 90, 150] if success else [255, 80, 40, 120])
        labels.append(str(box.get("id", "carried_box")))
    if centers:
        rr.log(
            path,
            rr.Boxes3D(
                centers=centers,
                half_sizes=half_sizes,
                quaternions=quaternions,
                colors=colors,
                labels=labels,
            ),
        )
    else:
        rr.log(path, rr.Clear(recursive=True))


def robot_module_matrix_to_quaternion(matrix: np.ndarray) -> list[float]:
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            w = (matrix[2, 1] - matrix[1, 2]) / scale
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            w = (matrix[0, 2] - matrix[2, 0]) / scale
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            w = (matrix[1, 0] - matrix[0, 1]) / scale
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
    return [float(x), float(y), float(z), float(w)]


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize MoveIt box-stack flow JSONL")
    parser.add_argument("jsonl", type=Path)
    parser.add_argument("--save", type=Path, default=None)
    parser.add_argument("--connect", action="store_true")
    parser.add_argument("--stride", type=int, default=1, help="Log every Nth trajectory point")
    parser.add_argument("--no-meshes", action="store_true")
    parser.add_argument("--no-container", action="store_true")
    parser.add_argument(
        "--no-playback-normalization",
        action="store_true",
        help="Do not add visual-only interpolation samples for fast velocity_scale recordings",
    )
    parser.add_argument(
        "--reference-velocity-scale",
        type=float,
        default=0.25,
        help="Rerun playback density reference; velocity_scale=1.0 defaults to 4x visual samples",
    )
    parser.add_argument("--robot-path", default="robot")
    args = parser.parse_args()

    helpers = load_rerun_helpers()
    header, stages, summary = read_jsonl(args.jsonl)
    if not stages:
        raise SystemExit(f"no stage records in {args.jsonl}")

    if args.save:
        rr.init("moveit_box_stack_flow", recording_id=f"moveit_flow_{args.jsonl.stem}")
        rr.save(str(args.save))
    elif args.connect:
        rr.init("moveit_box_stack_flow", recording_id=f"moveit_flow_{args.jsonl.stem}")
        rr.connect()
    else:
        rr.init("moveit_box_stack_flow", recording_id=f"moveit_flow_{args.jsonl.stem}")
        rr.spawn()

    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    robot = helpers.UrdfRobot(helpers.render_current_urdf())
    helpers.log_robot_static_model(robot, args.robot_path, log_meshes=not args.no_meshes)
    base_to_world = robot.fk({}).get("base_link", np.eye(4))
    log_box_stack(float(header.get("box_front_x", 0.625)))
    if not args.no_container:
        log_container_obstacle(header.get("container_obstacle"))

    velocity_scale = float(header.get("velocity_scale", args.reference_velocity_scale))
    repeat_factor = 1
    if not args.no_playback_normalization and args.reference_velocity_scale > 1e-9:
        repeat_factor = max(1, int(round(velocity_scale / args.reference_velocity_scale)))

    sample = 0
    stride = max(1, args.stride)
    for stage in stages:
        extra = stage.get("extra", {})
        log_target_pose("targets/left", extra.get("left_target", {}), [0, 220, 255], "left target", base_to_world)
        log_target_pose("targets/right", extra.get("right_target", {}), [255, 120, 0], "right target", base_to_world)
        points = stage.get("trajectory", {}).get("points", [])
        if not points:
            continue
        points = ensure_points_start_at_stage_start(stage, points)
        points = normalize_playback_points(points, repeat_factor)
        selected_indices = list(range(0, len(points), stride))
        if selected_indices[-1] != len(points) - 1:
            selected_indices.append(len(points) - 1)
        for point_index in selected_indices:
            helpers.set_sample_time(sample)
            log_static_box_obstacles(stage.get("static_box_obstacles", header.get("static_box_obstacles")))
            point = points[point_index]
            joints = joint_dict_from_point(stage, point)
            helpers.log_robot_state(robot, joints, args.robot_path)
            stage_success = bool(
                stage.get("extra", {}).get("valid", False)
                and stage.get("extra", {}).get("stage_kind") == "post_extract_loaded_plan"
            )
            log_attached_boxes(robot, joints, stage.get("attached_boxes", []), success=stage_success)
            log_stage_text(stage, point_index, len(points))
            sample += 1

    rr.log("info/summary", rr.TextLog(json.dumps(summary, ensure_ascii=False, indent=2)))
    print(f"Loaded {len(stages)} stages, logged {sample} samples from {args.jsonl}")
    print(f"Rerun playback repeat factor: {repeat_factor} (velocity_scale={velocity_scale})")
    if args.save:
        print(f"saved: {args.save}")


if __name__ == "__main__":
    main()
