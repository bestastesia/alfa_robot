#!/usr/bin/python3
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import trimesh

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import extract_sequence_rerun as sequence
import extract_stage_monitor_console as monitor

DEFAULT_RUN_ROOT = Path(
    "/mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/"
    "updown_full_scan_joint2_90/sequence_20260711_175531"
)
DEFAULT_TASKS = "11_L16_R23,12_L21_R18,13_L21_R23"


def load_snapshot(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text())
    if not isinstance(data, dict):
        raise ValueError(f"快照格式错误：{path}")
    return data


def collision_entities(reason: str) -> list[str]:
    prefix = "robot/carried box state colliding: "
    text = reason[len(prefix):] if reason.startswith(prefix) else reason
    if " <-> " in text:
        return [part.strip() for part in text.split(" <-> ", 1)]
    if " overlaps " in text:
        return [part.strip() for part in text.split(" overlaps ", 1)]
    return [text.strip()] if text.strip() else []


def load_red_meshes(robot: Any, helpers: Any) -> dict[str, list[tuple[np.ndarray, np.ndarray, np.ndarray]]]:
    result: dict[str, list[tuple[np.ndarray, np.ndarray, np.ndarray]]] = {}
    for link_name, link in robot.links.items():
        visuals = []
        for visual in link.visuals:
            mesh_path = helpers.package_uri_to_path(visual.path)
            if not mesh_path or not Path(mesh_path).exists():
                continue
            loaded = trimesh.load(mesh_path, force="mesh", process=False)
            if not isinstance(loaded, trimesh.Trimesh):
                continue
            vertices = np.asarray(loaded.vertices, dtype=float) * np.asarray(visual.scale, dtype=float)
            faces = np.asarray(loaded.faces, dtype=np.uint32)
            visuals.append((vertices, faces, np.asarray(visual.origin, dtype=float)))
        if visuals:
            result[link_name] = visuals
    return result


def log_red_robot_links(
    rr: Any,
    robot: Any,
    joints: dict[str, float],
    entities: list[str],
    meshes: dict[str, list[tuple[np.ndarray, np.ndarray, np.ndarray]]],
) -> None:
    rr.log("monitor/diagnostics/collision_links", rr.Clear(recursive=True))
    fk = robot.fk(joints)
    for entity in entities:
        if entity not in fk or entity not in meshes:
            continue
        link_tf = fk[entity]
        for visual_index, (vertices, faces, visual_origin) in enumerate(meshes[entity]):
            tf = link_tf @ visual_origin
            world_vertices = (tf[:3, :3] @ vertices.T).T + tf[:3, 3]
            rr.log(
                f"monitor/diagnostics/collision_links/{entity}/visual_{visual_index}",
                rr.Mesh3D(
                    vertex_positions=world_vertices,
                    triangle_indices=faces,
                    albedo_factor=[255, 0, 0, 235],
                ),
            )
        center = fk[entity][:3, 3]
        rr.log(
            f"monitor/diagnostics/collision_links/{entity}/label",
            rr.Points3D(
                positions=[center],
                radii=[0.045],
                colors=[[255, 0, 0, 255]],
                labels=[entity],
            ),
        )


def obstacle_by_id(snapshot: dict[str, Any], obstacle_id: str) -> tuple[list[float], list[float]] | None:
    static = snapshot.get("static_box_obstacles", {})
    boxes = static.get("boxes", []) if isinstance(static, dict) else []
    for box in boxes:
        if str(box.get("id", "")) == obstacle_id:
            return [float(v) for v in box.get("center", [])], [float(v) for v in box.get("size", [])]
    return None


def attached_box_by_id(
    snapshot: dict[str, Any],
    robot: Any,
    joints: dict[str, float],
    obstacle_id: str,
) -> tuple[list[float], list[float], list[float]] | None:
    fk = robot.fk(joints)
    for box in snapshot.get("attached_boxes", []):
        if str(box.get("id", "")) != obstacle_id:
            continue
        link_name = str(box.get("link_name", ""))
        link_tf = fk.get(link_name)
        center_local = box.get("center_in_link", [])
        size = box.get("size", [])
        if link_tf is None or len(center_local) != 3 or len(size) != 3:
            continue
        center = monitor.transform_point(link_tf, [float(v) for v in center_local])
        quat = monitor.matrix_to_quaternion(link_tf[:3, :3])
        return center, [float(v) for v in size], quat
    return None


def log_red_collision_objects(
    rr: Any,
    snapshot: dict[str, Any],
    robot: Any,
    joints: dict[str, float],
    entities: list[str],
) -> None:
    rr.log("monitor/diagnostics/collision_objects", rr.Clear(recursive=True))
    for entity in entities:
        attached = attached_box_by_id(snapshot, robot, joints, entity)
        if attached is not None:
            center, size, quat = attached
            rr.log(
                f"monitor/diagnostics/collision_objects/{entity}",
                rr.Boxes3D(
                    centers=[center], half_sizes=[[v * 0.5 for v in size]], quaternions=[quat],
                    colors=[[255, 0, 0, 190]], labels=[entity],
                ),
            )
            continue
        obstacle = obstacle_by_id(snapshot, entity)
        if obstacle is not None:
            center, size = obstacle
            rr.log(
                f"monitor/diagnostics/collision_objects/{entity}",
                rr.Boxes3D(
                    centers=[center], half_sizes=[[v * 0.5 for v in size]],
                    colors=[[255, 0, 0, 190]], labels=[entity],
                ),
            )


def log_records(
    snapshot: dict[str, Any], records: list[dict[str, Any]], accepted: bool,
    helpers: Any, robot: Any, display_args: Any, red_meshes: Any,
    task_index: int, task_count: int, sample: int,
) -> int:
    scene_y_shift = float(snapshot.get("scene_y_shift", -0.4))
    box_front_x = float(snapshot.get("box_front_x", 0.625))
    left_id = int(snapshot.get("left_box_id", 0))
    right_id = int(snapshot.get("right_box_id", 0))
    for index, record in enumerate(records):
        joint_map = record.get("state", {}).get("joint_map", {})
        if not isinstance(joint_map, dict) or not joint_map:
            continue
        joints = {str(k): float(v) for k, v in joint_map.items()}
        reason = "collision_free" if accepted else str(record.get("scene_rejection_reason", "collision_rejected"))
        entities = [] if accepted else collision_entities(reason)
        helpers.set_sample_time(sample)
        monitor.log_default_container(scene_y_shift)
        monitor.log_box_stack(box_front_x, left_id, right_id, scene_y_shift)
        monitor.log_static_box_obstacles(snapshot.get("static_box_obstacles"))
        sequence.log_robot_state_display(helpers, robot, joints, "monitor/robot", display_args)
        sequence.log_attached_boxes_display(robot, joints, snapshot.get("attached_boxes", []), display_args)
        log_red_robot_links(monitor.rr, robot, joints, entities, red_meshes)
        log_red_collision_objects(monitor.rr, snapshot, robot, joints, entities)
        color = [40, 220, 80, 255] if accepted else [255, 0, 0, 255]
        monitor.rr.log(
            "monitor/diagnostics/result",
            monitor.rr.Points3D(
                positions=[[0.0, 0.0, 0.0]], radii=[0.08], colors=[color],
                labels=["碰撞通过" if accepted else reason],
            ),
        )
        monitor.rr.log(
            "monitor/info",
            monitor.rr.TextLog(
                f"任务 {task_index}/{task_count}: L{left_id}/R{right_id} | "
                f"{'成功' if accepted else '失败'} {index+1}/{len(records)} | "
                f"h={float(record.get('h', joints.get('updown', 0.0))):.2f}m | {reason}"
            ),
        )
        sample += 1
    return sample


def main() -> int:
    parser = argparse.ArgumentParser(description="回放指定任务的全高度碰撞通过/失败IK，碰撞连接件红色高亮")
    parser.add_argument("--run-root", type=Path, default=DEFAULT_RUN_ROOT)
    parser.add_argument("--tasks", default=DEFAULT_TASKS)
    parser.add_argument("--save", type=Path, required=True)
    args = parser.parse_args()
    task_names = [v.strip() for v in args.tasks.split(",") if v.strip()]
    snapshots = [load_snapshot(args.run_root / task / "stage_snapshot.json") for task in task_names]

    import rerun as rr
    monitor.rr = rr
    helpers = monitor.load_rerun_helpers()
    args.save.parent.mkdir(parents=True, exist_ok=True)
    rr.init("updown_scan_collision_highlights")
    rr.save(str(args.save))
    robot = helpers.UrdfRobot(helpers.render_current_urdf())
    rr.log("monitor", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    helpers.log_robot_static_model(robot, "monitor/robot", log_meshes=True)
    red_meshes = load_red_meshes(robot, helpers)
    display_args = SimpleNamespace(
        display_base_x=0.0, display_base_y=0.0,
        display_base_yaw_deg=0.0, display_turn_offset_deg=0.0,
    )
    sample = 0
    for task_index, snapshot in enumerate(snapshots, 1):
        accepted = [r for r in snapshot.get("records", []) if isinstance(r, dict)]
        rejected = [r for r in snapshot.get("scene_rejected_records", []) if isinstance(r, dict)]
        sample = log_records(snapshot, accepted, True, helpers, robot, display_args, red_meshes, task_index, len(snapshots), sample)
        sample = log_records(snapshot, rejected, False, helpers, robot, display_args, red_meshes, task_index, len(snapshots), sample)
        print(f"L{snapshot.get('left_box_id')}/R{snapshot.get('right_box_id')}: 成功={len(accepted)} 失败={len(rejected)}")
    print(f"已保存：{args.save.resolve()} frames={sample}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
