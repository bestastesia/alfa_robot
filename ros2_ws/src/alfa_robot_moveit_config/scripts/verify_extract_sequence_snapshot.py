#!/usr/bin/env python3

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

try:
    from alfa_robot_rerun.visualize_rerun import UrdfRobot, render_current_urdf
except ImportError as exc:
    raise SystemExit(
        "无法导入 alfa_robot_rerun；请先 source ros2_ws/install/setup.bash"
    ) from exc

BOX_SIZE = np.array([0.3, 0.4, 0.5], dtype=float)


def box_center_z(box_id: int) -> float:
    row_from_top = (box_id - 1) // 3
    return (4 - row_from_top - 0.5) * 0.5


def transformed_box_aabb(
    robot: UrdfRobot,
    joint_map: dict[str, float],
    box: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray]:
    transforms = robot.fk(joint_map)
    link_name = str(box["link_name"])
    link_transform = transforms[link_name]
    center = np.asarray(box["center_in_link"], dtype=float)
    size = np.asarray(box["size"], dtype=float)
    corners = []
    for sx in (-0.5, 0.5):
        for sy in (-0.5, 0.5):
            for sz in (-0.5, 0.5):
                local = center + np.array([sx, sy, sz]) * size
                corners.append((link_transform @ np.append(local, 1.0))[:3])
    world_corners = np.asarray(corners)
    return world_corners.min(axis=0), world_corners.max(axis=0)


def xz_detached(
    carried_min: np.ndarray,
    carried_max: np.ndarray,
    source_min: np.ndarray,
    source_max: np.ndarray,
    tolerance: float,
) -> bool:
    return bool(
        carried_max[0] <= source_min[0] + tolerance
        or carried_min[0] >= source_max[0] - tolerance
        or carried_max[2] <= source_min[2] + tolerance
        or carried_min[2] >= source_max[2] - tolerance
    )


def aabb_overlaps(
    lhs_min: np.ndarray,
    lhs_max: np.ndarray,
    rhs_min: np.ndarray,
    rhs_max: np.ndarray,
) -> bool:
    return bool(np.all(lhs_min <= rhs_max) and np.all(lhs_max >= rhs_min))


def verify_snapshot(
    snapshot_path: Path,
    robot: UrdfRobot,
    margin: float,
    tolerance: float,
) -> list[str]:
    snapshot = json.loads(snapshot_path.read_text())
    replay_stages = list(snapshot.get("replay_stages", []))
    if not replay_stages:
        return []

    errors: list[str] = []
    extract_stages = [
        stage for stage in replay_stages
        if "selected_extract_step_" in str(stage.get("stage", ""))
    ]
    loaded_stages = [
        stage for stage in replay_stages
        if str(stage.get("stage", "")).endswith("selected_loaded_plan")
    ]

    if not extract_stages:
        errors.append("成功快照缺少 selected_extract_step")
        return errors

    final_extract = extract_stages[-1]
    goal_state = final_extract.get("goal_state", {})
    joint_map = goal_state.get("joint_map", {})
    attached_boxes = list(final_extract.get("attached_boxes", []))
    box_ids = [int(snapshot.get("left_box_id", 0)), int(snapshot.get("right_box_id", 0))]
    box_front_x = float(snapshot.get("box_front_x", 0.0))

    if len(attached_boxes) != 2 or any(box_id <= 0 for box_id in box_ids):
        errors.append("抽离终态缺少双附着箱或箱号")
    else:
        for attached_box, box_id in zip(attached_boxes, box_ids):
            carried_min, carried_max = transformed_box_aabb(robot, joint_map, attached_box)
            source_center = np.array(
                [box_front_x + 0.5 * BOX_SIZE[0], 0.0, box_center_z(box_id)],
                dtype=float,
            )
            source_min = source_center - 0.5 * BOX_SIZE - margin
            source_max = source_center + 0.5 * BOX_SIZE + margin
            if not xz_detached(
                carried_min, carried_max, source_min, source_max, tolerance
            ):
                errors.append(
                    f"{attached_box.get('id', box_id)} 未完全脱离："
                    f"carried_x=[{carried_min[0]:.6f},{carried_max[0]:.6f}] "
                    f"carried_z=[{carried_min[2]:.6f},{carried_max[2]:.6f}] "
                    f"source_x=[{source_min[0]:.6f},{source_max[0]:.6f}] "
                    f"source_z=[{source_min[2]:.6f},{source_max[2]:.6f}]"
                )

    if not loaded_stages:
        errors.append("成功快照缺少 selected_loaded_plan")
    else:
        loaded = loaded_stages[-1]
        trajectory = loaded.get("trajectory", {})
        points = list(trajectory.get("points", []))
        if len(points) < 2:
            errors.append("负重轨迹点数不足 2")
        else:
            first = [float(value) for value in points[0].get("positions", [])]
            last = [float(value) for value in points[-1].get("positions", [])]
            if len(first) != len(last) or not first:
                errors.append("负重轨迹首尾关节维度错误")
            else:
                endpoint_motion = sum(abs(end - begin) for begin, end in zip(first, last))
                recorded_motion = float(
                    loaded.get("extra", {}).get("loaded_plan_trajectory_distance", 0.0)
                )
                if endpoint_motion <= tolerance:
                    errors.append("负重轨迹是原地零运动")
                if recorded_motion <= tolerance:
                    errors.append("负重轨迹累计运动量为零")
                if not math.isfinite(recorded_motion):
                    errors.append("负重轨迹累计运动量非有限值")

            joint_names = list(trajectory.get("joint_names", []))
            loaded_boxes = list(loaded.get("attached_boxes", []))
            rear_guards = [
                obstacle
                for obstacle in loaded.get("static_box_obstacles", {}).get("boxes", [])
                if str(obstacle.get("id", "")).endswith("_rear_guard")
            ]
            for point_index, point in enumerate(points):
                joint_map = dict(zip(joint_names, point.get("positions", [])))
                for attached_box in loaded_boxes:
                    carried_min, carried_max = transformed_box_aabb(
                        robot, joint_map, attached_box
                    )
                    for rear_guard in rear_guards:
                        guard_center = np.asarray(rear_guard["center"], dtype=float)
                        guard_size = np.asarray(rear_guard["size"], dtype=float)
                        guard_min = guard_center - 0.5 * guard_size
                        guard_max = guard_center + 0.5 * guard_size
                        if aabb_overlaps(
                            carried_min, carried_max, guard_min, guard_max
                        ):
                            errors.append(
                                f"负重轨迹点 {point_index} 的 "
                                f"{attached_box.get('id', 'carried_box')} 穿入 "
                                f"{rear_guard.get('id', 'rear_guard')}"
                            )

    return errors


def main() -> int:
    parser = argparse.ArgumentParser(
        description="验证抽箱序列成功快照是否严格脱离并包含真实负重轨迹"
    )
    parser.add_argument("run_root", type=Path)
    parser.add_argument("--margin", type=float, default=0.03)
    parser.add_argument("--tolerance", type=float, default=1e-6)
    args = parser.parse_args()

    snapshot_paths = sorted(args.run_root.glob("*/stage_snapshot.json"))
    if not snapshot_paths:
        print(f"未找到快照：{args.run_root}", file=sys.stderr)
        return 2

    robot = UrdfRobot(render_current_urdf())
    checked = 0
    failed = 0
    for snapshot_path in snapshot_paths:
        snapshot = json.loads(snapshot_path.read_text())
        if not snapshot.get("replay_stages"):
            continue
        checked += 1
        errors = verify_snapshot(snapshot_path, robot, args.margin, args.tolerance)
        label = snapshot_path.parent.name
        if errors:
            failed += 1
            print(f"[FAIL] {label}")
            for error in errors:
                print(f"  - {error}")
        else:
            print(f"[PASS] {label}")

    print(f"验证完成：成功快照={checked}，失败={failed}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
