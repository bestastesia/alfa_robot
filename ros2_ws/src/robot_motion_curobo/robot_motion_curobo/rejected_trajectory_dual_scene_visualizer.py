"""Show one cuRobo candidate in its sphere scene and the rejecting production scene."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from .collision_model_visualizer import (
    Cuboid,
    Payload,
    Sphere,
    collision_map,
    cuboid_covering_spheres,
    load_robot_model,
    load_scene,
    matrix_to_quaternion_xyzw,
    transform_spheres,
)


def load_stage(path: Path, stage_index: int) -> dict[str, Any]:
    stages = []
    for line in path.read_text().splitlines():
        record = json.loads(line)
        if record.get("type") == "stage":
            stages.append(record)
    if stage_index < 0 or stage_index >= len(stages):
        raise ValueError(f"stage index {stage_index} outside [0,{len(stages)})")
    return stages[stage_index]


def payloads_from_stage(stage: dict[str, Any]) -> tuple[Payload, ...]:
    result = []
    for box in stage.get("attached_boxes", []):
        link_name = str(box.get("link_name", ""))
        side = "left" if link_name.startswith("left_") else "right" if link_name.startswith("right_") else ""
        center = box.get("center_in_link", [])
        size = box.get("size", [])
        if not side or len(center) != 3 or len(size) != 3:
            continue
        digits = "".join(character for character in str(box.get("id", "")) if character.isdigit())
        result.append(
            Payload(
                side=side,
                box_id=int(digits) if digits else 0,
                link_name=link_name,
                grasp_mode=str(box.get("grasp_mode", "top_suction")),
                center_xyz=tuple(float(value) for value in center),
                size_xyz=tuple(float(value) for value in size),
            )
        )
    return tuple(result)


def aabb_from_payload(payload: Payload, transform: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    center = np.asarray(payload.center_xyz, dtype=float)
    half = 0.5 * np.asarray(payload.size_xyz, dtype=float)
    corners = np.asarray(
        [
            transform[:3, :3] @ (center + np.asarray([sx, sy, sz]) * half) + transform[:3, 3]
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ]
    )
    return corners.min(axis=0), corners.max(axis=0)


def aabb_overlaps(
    lower: np.ndarray,
    upper: np.ndarray,
    cuboid: Cuboid,
) -> bool:
    obstacle_center = np.asarray(cuboid.center, dtype=float)
    obstacle_half = 0.5 * np.asarray(cuboid.size, dtype=float)
    return bool(
        np.all(lower <= obstacle_center + obstacle_half)
        and np.all(upper >= obstacle_center - obstacle_half)
    )


def translated_spheres(spheres: Sequence[Sphere], offset: np.ndarray) -> tuple[Sphere, ...]:
    return tuple(
        Sphere(
            sphere.link_name,
            tuple(np.asarray(sphere.center, dtype=float) + offset),
            sphere.radius,
            sphere.kind,
        )
        for sphere in spheres
    )


def log_cuboids(
    rr: Any,
    path: str,
    cuboids: Sequence[Cuboid],
    offset: np.ndarray,
    colliding_names: set[str],
    base_color: list[int],
) -> None:
    rr.log(
        path,
        rr.Boxes3D(
            centers=[(np.asarray(cuboid.center) + offset).tolist() for cuboid in cuboids],
            half_sizes=[[0.5 * value for value in cuboid.size] for cuboid in cuboids],
            quaternions=[
                [
                    cuboid.quaternion_wxyz[1],
                    cuboid.quaternion_wxyz[2],
                    cuboid.quaternion_wxyz[3],
                    cuboid.quaternion_wxyz[0],
                ]
                for cuboid in cuboids
            ],
            colors=[
                [255, 25, 25, 170] if cuboid.name in colliding_names else base_color
                for cuboid in cuboids
            ],
            labels=[cuboid.name for cuboid in cuboids],
        ),
    )


def log_sphere_layer(
    rr: Any,
    path: str,
    spheres: Sequence[Sphere],
    collisions: dict[int, tuple[str, ...]],
    clear_color: list[int],
) -> None:
    clear = [sphere for index, sphere in enumerate(spheres) if index not in collisions]
    colliding = [sphere for index, sphere in enumerate(spheres) if index in collisions]
    for suffix, selected, color in (
        ("clear", clear, clear_color),
        ("colliding", colliding, [255, 25, 25, 220]),
    ):
        rr.log(
            f"{path}/{suffix}",
            rr.Ellipsoids3D(
                centers=[sphere.center for sphere in selected],
                half_sizes=[[sphere.radius] * 3 for sphere in selected],
                colors=[color] * len(selected),
                show_labels=False,
            ),
        )


def log_production_payloads(
    rr: Any,
    path: str,
    payloads: Sequence[Payload],
    transforms: dict[str, np.ndarray],
    offset: np.ndarray,
    colliding_payloads: set[str],
) -> None:
    actual_centers = []
    actual_half_sizes = []
    actual_quaternions = []
    actual_colors = []
    actual_labels = []
    aabb_centers = []
    aabb_half_sizes = []
    aabb_colors = []
    aabb_labels = []
    for payload in payloads:
        transform = transforms[payload.link_name]
        center = transform[:3, :3] @ np.asarray(payload.center_xyz) + transform[:3, 3]
        lower, upper = aabb_from_payload(payload, transform)
        object_id = f"carried_{payload.side}_box_{payload.box_id}"
        colliding = object_id in colliding_payloads
        actual_centers.append((center + offset).tolist())
        actual_half_sizes.append([0.5 * value for value in payload.size_xyz])
        actual_quaternions.append(matrix_to_quaternion_xyzw(transform[:3, :3]))
        actual_colors.append([255, 25, 25, 210] if colliding else [40, 220, 90, 100])
        actual_labels.append(object_id)
        aabb_centers.append((0.5 * (lower + upper) + offset).tolist())
        aabb_half_sizes.append((0.5 * (upper - lower)).tolist())
        aabb_colors.append([255, 25, 25, 45] if colliding else [255, 210, 30, 25])
        aabb_labels.append(object_id + " production AABB")
    rr.log(
        f"{path}/actual_boxes",
        rr.Boxes3D(
            centers=actual_centers,
            half_sizes=actual_half_sizes,
            quaternions=actual_quaternions,
            colors=actual_colors,
            labels=actual_labels,
        ),
    )
    rr.log(
        f"{path}/production_aabbs",
        rr.Boxes3D(
            centers=aabb_centers,
            half_sizes=aabb_half_sizes,
            colors=aabb_colors,
            labels=aabb_labels,
        ),
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compare one cuRobo candidate against its planning sphere and production scenes."
    )
    parser.add_argument("--jsonl", required=True)
    parser.add_argument("--stage-index", type=int, default=0)
    parser.add_argument("--robot-config", required=True)
    parser.add_argument("--curobo-scene", required=True)
    parser.add_argument("--production-scene", required=True)
    parser.add_argument("--save", required=True)
    parser.add_argument("--summary", default="")
    parser.add_argument("--payload-grid-resolution", type=float, default=0.1)
    parser.add_argument("--activation-distance", type=float, default=0.0)
    parser.add_argument("--view-separation", type=float, default=1.8)
    parser.add_argument("--no-meshes", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    jsonl_path = Path(args.jsonl).expanduser().resolve()
    robot_path = Path(args.robot_config).expanduser().resolve()
    curobo_scene_path = Path(args.curobo_scene).expanduser().resolve()
    production_scene_path = Path(args.production_scene).expanduser().resolve()
    save_path = Path(args.save).expanduser().resolve()
    summary_path = (
        Path(args.summary).expanduser().resolve()
        if args.summary else save_path.with_suffix(".json")
    )
    save_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.parent.mkdir(parents=True, exist_ok=True)

    stage = load_stage(jsonl_path, args.stage_index)
    payloads = payloads_from_stage(stage)
    _, urdf_path, local_robot_spheres = load_robot_model(robot_path)
    curobo_cuboids = load_scene(curobo_scene_path)
    production_cuboids = load_scene(production_scene_path)
    production_rear_guards = tuple(
        cuboid for cuboid in production_cuboids if cuboid.name.endswith("_rear_guard")
    )
    if not production_rear_guards:
        raise ValueError(f"production scene has no *_rear_guard cuboid: {production_scene_path}")
    local_payload_spheres = {
        payload.side: cuboid_covering_spheres(payload, args.payload_grid_resolution)
        for payload in payloads
    }

    from alfa_robot_rerun import visualize_rerun as helpers

    rr = helpers.rr
    rr.init("alfa_curobo_rejected_trajectory_comparison", recording_id=save_path.stem, spawn=False)
    rr.save(str(save_path))
    rr.log("comparison", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log(
        "comparison/legend",
        rr.TextLog(
            "upper(+Y)=cuRobo planning scene and spheres; lower(-Y)=true production scene; "
            "red=collision in that scene; yellow transparent=carried-box production AABB; "
            "the production overlay uses the dedicated rear_guard validation gate"
        ),
        static=True,
    )

    robot = helpers.UrdfRobot(urdf_path.read_text())
    helpers.log_robot_static_model(
        robot, "comparison/curobo/robot", log_meshes=not args.no_meshes
    )
    helpers.log_robot_static_model(
        robot, "comparison/production/robot", log_meshes=not args.no_meshes
    )
    curobo_offset = np.asarray([0.0, args.view_separation, 0.0])
    production_offset = np.asarray([0.0, -args.view_separation, 0.0])
    curobo_base = np.eye(4)
    production_base = np.eye(4)
    curobo_base[:3, 3] = curobo_offset
    production_base[:3, 3] = production_offset

    base_joints = {
        str(name): float(value)
        for name, value in stage.get("start_state", {}).get("joint_map", {}).items()
    }
    base_joints.setdefault("pitch", 0.0)
    base_joints.setdefault("turn", 0.0)
    names = [str(name) for name in stage.get("trajectory", {}).get("joint_names", [])]
    points = stage.get("trajectory", {}).get("points", [])

    production_collision_frames: list[int] = []
    production_frames_by_payload: dict[str, list[int]] = {
        f"carried_{payload.side}_box_{payload.box_id}": [] for payload in payloads
    }
    production_pairs_by_frame: dict[int, list[str]] = {}
    curobo_collision_frames: list[int] = []

    for frame_index, point in enumerate(points):
        helpers.set_sample_time(frame_index)
        joints = dict(base_joints)
        joints.update(
            (name, float(value)) for name, value in zip(names, point.get("positions", []))
        )
        transforms = robot.fk(joints)
        for link_name, transform in transforms.items():
            helpers.log_transform_matrix(
                f"comparison/curobo/robot/{link_name}", curobo_base @ transform
            )
            helpers.log_transform_matrix(
                f"comparison/production/robot/{link_name}", production_base @ transform
            )

        robot_spheres = transform_spheres(local_robot_spheres, transforms)
        robot_hits, curobo_obstacle_counts = collision_map(
            robot_spheres, curobo_cuboids, args.activation_distance
        )
        robot_spheres_display = translated_spheres(robot_spheres, curobo_offset)
        log_sphere_layer(
            rr,
            "comparison/curobo/robot_spheres",
            robot_spheres_display,
            robot_hits,
            [30, 135, 255, 65],
        )
        curobo_pairs = {
            f"robot:{robot_spheres[index].link_name}<->{obstacle}"
            for index, obstacles in robot_hits.items()
            for obstacle in obstacles
        }
        payload_hits_by_side: dict[str, dict[int, tuple[str, ...]]] = {}
        for payload in payloads:
            world_spheres = transform_spheres(local_payload_spheres[payload.side], transforms)
            hits, counts = collision_map(
                world_spheres, curobo_cuboids, args.activation_distance
            )
            payload_hits_by_side[payload.side] = hits
            for obstacle, count in counts.items():
                curobo_obstacle_counts[obstacle] += count
            display_spheres = translated_spheres(world_spheres, curobo_offset)
            log_sphere_layer(
                rr,
                f"comparison/curobo/{payload.side}_payload_spheres",
                display_spheres,
                hits,
                [0, 220, 255, 95] if payload.side == "left" else [210, 70, 255, 95],
            )
            curobo_pairs.update(
                f"payload_{payload.side}:{payload.link_name}<->{obstacle}"
                for obstacles in hits.values()
                for obstacle in obstacles
            )
        if curobo_pairs:
            curobo_collision_frames.append(frame_index)
        log_cuboids(
            rr,
            "comparison/curobo/planning_scene",
            curobo_cuboids,
            curobo_offset,
            {name for name, count in curobo_obstacle_counts.items() if count},
            [240, 170, 40, 55],
        )

        production_pairs = []
        colliding_payloads = set()
        colliding_obstacles = set()
        for payload in payloads:
            lower, upper = aabb_from_payload(payload, transforms[payload.link_name])
            object_id = f"carried_{payload.side}_box_{payload.box_id}"
            # This is intentionally limited to rear_guard.  It reproduces the
            # dedicated carried_box_clear_rear_guard production gate that rejected
            # this candidate; the optional static side-wall AABB gate was disabled.
            for cuboid in production_rear_guards:
                if aabb_overlaps(lower, upper, cuboid):
                    pair = f"{object_id}<->{cuboid.name}"
                    production_pairs.append(pair)
                    colliding_payloads.add(object_id)
                    colliding_obstacles.add(cuboid.name)
                    production_frames_by_payload[object_id].append(frame_index)
        if production_pairs:
            production_collision_frames.append(frame_index)
            production_pairs_by_frame[frame_index] = sorted(set(production_pairs))
        log_cuboids(
            rr,
            "comparison/production/true_scene",
            production_cuboids,
            production_offset,
            colliding_obstacles,
            [120, 180, 230, 50],
        )
        log_production_payloads(
            rr,
            "comparison/production/payloads",
            payloads,
            transforms,
            production_offset,
            colliding_payloads,
        )
        rr.log(
            "comparison/production/collision",
            rr.Scalars([1.0 if production_pairs else 0.0]),
        )
        time_s = float(point.get("time_from_start_sec", 0.0))
        rr.log(
            "comparison/info",
            rr.TextLog(
                f"raw frame index={frame_index} (frame {frame_index + 1}/{len(points)}) "
                f"t={time_s:.3f}s | cuRobo sphere scene: "
                f"{', '.join(sorted(curobo_pairs)) if curobo_pairs else 'clear'} | "
                f"production rear_guard gate: "
                f"{', '.join(sorted(set(production_pairs))) if production_pairs else 'clear'}"
            ),
        )

    first_frame = production_collision_frames[0] if production_collision_frames else None
    last_frame = production_collision_frames[-1] if production_collision_frames else None
    summary = {
        "source_jsonl": str(jsonl_path),
        "source_stage_index": args.stage_index,
        "source_stage": stage.get("stage"),
        "trajectory_points": len(points),
        "curobo_scene": str(curobo_scene_path),
        "production_scene": str(production_scene_path),
        "rerun": str(save_path),
        "curobo_sphere_collision_frames_zero_based": curobo_collision_frames,
        "production_rear_guard_collision_frames_zero_based": production_collision_frames,
        "production_rear_guard_first_collision_zero_based": first_frame,
        "production_rear_guard_first_collision_one_based": (
            None if first_frame is None else first_frame + 1
        ),
        "production_rear_guard_first_collision_time_s": (
            None if first_frame is None else float(points[first_frame].get("time_from_start_sec", 0.0))
        ),
        "production_rear_guard_last_collision_zero_based": last_frame,
        "production_rear_guard_last_collision_one_based": (
            None if last_frame is None else last_frame + 1
        ),
        "production_rear_guard_frames_by_payload_zero_based": production_frames_by_payload,
        "production_rear_guard_pairs_by_frame_zero_based": {
            str(index): pairs for index, pairs in production_pairs_by_frame.items()
        },
        "note": (
            "Production highlighting reproduces the dedicated attached-box world-AABB versus "
            "rear_guard gate used to validate this path. Other production cuboids remain visible but "
            "do not drive this overlay; this is not a replacement for MoveIt/FCL."
        ),
    }
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
