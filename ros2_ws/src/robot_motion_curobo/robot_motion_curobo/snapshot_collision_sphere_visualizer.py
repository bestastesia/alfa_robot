"""Replay a saved production snapshot with the exact cuRobo collision spheres."""

from __future__ import annotations

import argparse
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np

from .collision_model_visualizer import (
    Cuboid,
    Payload,
    Sphere,
    collision_map,
    cuboid_covering_spheres,
    load_robot_model,
    load_scene,
    log_payload_boxes,
    log_scene,
    log_sphere_layer,
    matrix_to_quaternion_xyzw,
    quaternion_xyzw_to_matrix,
    transform_spheres,
)
from .rejected_trajectory_dual_scene_visualizer import aabb_from_payload, aabb_overlaps


@dataclass(frozen=True)
class ReplaySample:
    section: str
    stage: str
    point_index: int
    point_count: int
    joints: dict[str, float]
    attached_boxes: tuple[dict[str, Any], ...]


def _state_point(stage: dict[str, Any], state_key: str, time_s: float) -> dict[str, Any] | None:
    names = [str(name) for name in stage.get("trajectory", {}).get("joint_names", [])]
    state = stage.get(state_key, {}).get("joint_map", {})
    if not names or not isinstance(state, dict):
        return None
    try:
        positions = [float(state[name]) for name in names]
    except (KeyError, TypeError, ValueError):
        return None
    return {"time_from_start_sec": time_s, "positions": positions}


def _same_positions(left: dict[str, Any], right: dict[str, Any], tolerance: float = 1.0e-6) -> bool:
    left_values = left.get("positions", [])
    right_values = right.get("positions", [])
    return (
        len(left_values) == len(right_values)
        and bool(left_values)
        and max(abs(float(a) - float(b)) for a, b in zip(left_values, right_values)) <= tolerance
    )


def densify_points(
    points: Sequence[dict[str, Any]],
    max_step: float = 5.0 * math.pi / 180.0,
) -> list[dict[str, Any]]:
    if len(points) < 2:
        return list(points)
    result = [dict(points[0])]
    for point in points[1:]:
        previous = result[-1]
        previous_values = [float(value) for value in previous.get("positions", [])]
        next_values = [float(value) for value in point.get("positions", [])]
        if not previous_values or len(previous_values) != len(next_values):
            result.append(dict(point))
            continue
        subdivisions = max(
            1,
            int(math.ceil(max(abs(a - b) for a, b in zip(previous_values, next_values)) / max_step)),
        )
        previous_time = float(previous.get("time_from_start_sec", 0.0))
        next_time = float(point.get("time_from_start_sec", previous_time))
        for subdivision in range(1, subdivisions + 1):
            ratio = subdivision / subdivisions
            result.append(
                {
                    **point,
                    "time_from_start_sec": previous_time + (next_time - previous_time) * ratio,
                    "positions": [
                        a + (b - a) * ratio for a, b in zip(previous_values, next_values)
                    ],
                }
            )
    return result


def playback_points(stage: dict[str, Any]) -> list[dict[str, Any]]:
    points = [dict(point) for point in stage.get("trajectory", {}).get("points", [])]
    start = _state_point(stage, "start_state", 0.0)
    if start is not None and (not points or not _same_positions(start, points[0])):
        points.insert(0, start)
    end_time = float(points[-1].get("time_from_start_sec", 0.0)) if points else 0.0
    goal = _state_point(stage, "goal_state", end_time)
    if goal is not None and (not points or not _same_positions(goal, points[-1])):
        points.append(goal)
    return densify_points(points)


def joint_map_for_point(stage: dict[str, Any], point: dict[str, Any]) -> dict[str, float]:
    joints = {
        str(name): float(value)
        for name, value in stage.get("start_state", {}).get("joint_map", {}).items()
    }
    names = stage.get("trajectory", {}).get("joint_names", [])
    for name, value in zip(names, point.get("positions", [])):
        joints[str(name)] = float(value)
    return joints


def repaired_samples(snapshot: dict[str, Any], stride: int) -> list[ReplaySample]:
    samples: list[ReplaySample] = []
    previous_positions: list[float] | None = None
    previous_names: list[str] | None = None
    for stage in snapshot.get("replay_stages", []):
        if not isinstance(stage, dict):
            continue
        points = playback_points(stage)
        names = [str(name) for name in stage.get("trajectory", {}).get("joint_names", [])]
        if previous_positions is not None and previous_names == names and points:
            bridge = densify_points(
                [
                    {"time_from_start_sec": 0.0, "positions": previous_positions},
                    {"time_from_start_sec": 0.1, "positions": points[0].get("positions", [])},
                ]
            )
            if len(bridge) > 2:
                points = bridge[1:-1] + points
        indices = list(range(0, len(points), max(1, stride)))
        if points and (not indices or indices[-1] != len(points) - 1):
            indices.append(len(points) - 1)
        for index in indices:
            samples.append(
                ReplaySample(
                    section="repaired_full_flow",
                    stage=str(stage.get("stage", "stage")),
                    point_index=index,
                    point_count=len(points),
                    joints=joint_map_for_point(stage, points[index]),
                    attached_boxes=tuple(stage.get("attached_boxes", [])),
                )
            )
        if points:
            previous_positions = [float(value) for value in points[-1].get("positions", [])]
            previous_names = names
    return samples


def unrepaired_loaded_samples(snapshot: dict[str, Any], steps: int) -> list[ReplaySample]:
    loaded_stage = next(
        (
            stage for stage in reversed(snapshot.get("replay_stages", []))
            if isinstance(stage, dict) and "loaded" in str(stage.get("stage", ""))
        ),
        None,
    )
    if loaded_stage is None:
        return []
    start = loaded_stage.get("start_state", {}).get("joint_map", {})
    goal = loaded_stage.get("goal_state", {}).get("joint_map", {})
    names = [str(name) for name in loaded_stage.get("target_names", [])]
    if not start or not goal or not names:
        return []
    result = []
    for index in range(steps + 1):
        ratio = index / steps
        joints = {str(name): float(value) for name, value in start.items()}
        for name in names:
            joints[name] = float(start[name]) + (float(goal[name]) - float(start[name])) * ratio
        result.append(
            ReplaySample(
                section="unrepaired_loaded_shortcut",
                stage="direct_13d_without_local_curobo",
                point_index=index,
                point_count=steps + 1,
                joints=joints,
                attached_boxes=tuple(loaded_stage.get("attached_boxes", [])),
            )
        )
    return result


def payloads_from_attached(boxes: Iterable[dict[str, Any]]) -> tuple[Payload, ...]:
    payloads = []
    for box in boxes:
        link_name = str(box.get("link_name", ""))
        side = "left" if link_name.startswith("left_") else "right" if link_name.startswith("right_") else ""
        center = box.get("center_in_link", [])
        size = box.get("size", [])
        if not side or len(center) != 3 or len(size) != 3:
            continue
        identifier = str(box.get("id", ""))
        match = re.search(r"(\d+)$", identifier)
        payloads.append(
            Payload(
                side=side,
                box_id=int(match.group(1)) if match else 0,
                link_name=link_name,
                grasp_mode="top_suction" if abs(float(center[2]) - 0.2) < 1.0e-6 else "front",
                center_xyz=tuple(float(value) for value in center),
                size_xyz=tuple(float(value) for value in size),
            )
        )
    return tuple(payloads)


def payload_aabb_collision_map(
    payloads: Sequence[Payload],
    transforms: dict[str, np.ndarray],
    cuboids: Sequence[Cuboid],
) -> tuple[dict[str, tuple[str, ...]], dict[str, int]]:
    """Evaluate actual carried-box world AABBs against production cuboids."""
    collisions: dict[str, tuple[str, ...]] = {}
    obstacle_counts = {cuboid.name: 0 for cuboid in cuboids}
    for payload in payloads:
        transform = transforms.get(payload.link_name)
        if transform is None:
            continue
        lower, upper = aabb_from_payload(payload, transform)
        names = tuple(
            cuboid.name for cuboid in cuboids if aabb_overlaps(lower, upper, cuboid)
        )
        if names:
            collisions[payload.side] = names
            for name in names:
                obstacle_counts[name] += 1
    return collisions, obstacle_counts


def log_actual_payload_boxes(
    rr: Any,
    payloads: Sequence[Payload],
    transforms: dict[str, np.ndarray],
    collisions: dict[str, tuple[str, ...]],
) -> None:
    """Log physical payload cuboids, without the conservative sphere cover."""
    centers = []
    half_sizes = []
    quaternions = []
    colors = []
    labels = []
    for payload in payloads:
        transform = transforms[payload.link_name]
        local_rotation = quaternion_xyzw_to_matrix(payload.orientation_xyzw)
        centers.append(
            (transform[:3, :3] @ np.asarray(payload.center_xyz) + transform[:3, 3]).tolist()
        )
        half_sizes.append([0.5 * value for value in payload.size_xyz])
        quaternions.append(matrix_to_quaternion_xyzw(transform[:3, :3] @ local_rotation))
        colliding = payload.side in collisions
        colors.append(
            [255, 25, 25, 210]
            if colliding
            else ([0, 210, 255, 80] if payload.side == "left" else [210, 70, 255, 80])
        )
        labels.append(
            f"{payload.side} actual box {payload.box_id}"
            + (f" collision: {', '.join(collisions[payload.side])}" if colliding else "")
        )
    rr.log(
        "world/collision_model/payload_boxes",
        rr.Boxes3D(
            centers=centers,
            half_sizes=half_sizes,
            quaternions=quaternions,
            colors=colors,
            labels=labels,
        ),
    )


def clear_sphere_layers(rr: Any) -> None:
    """Prevent repaired-section spheres from persisting into the direct comparison."""
    for path in (
        "world/collision_model/robot_spheres",
        "world/collision_model/left_payload_spheres",
        "world/collision_model/right_payload_spheres",
    ):
        log_sphere_layer(rr, path, (), {}, [0, 0, 0, 0])


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Replay an extract-monitor snapshot with robot and payload cuRobo spheres."
    )
    parser.add_argument("--snapshot", required=True)
    parser.add_argument("--robot-config", required=True)
    parser.add_argument("--scene-config", required=True)
    parser.add_argument(
        "--unrepaired-scene-config",
        default="",
        help=(
            "Production scene used for the unrepaired actual-box comparison; "
            "defaults to --scene-config"
        ),
    )
    parser.add_argument("--save", required=True)
    parser.add_argument("--summary", default="")
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--unrepaired-steps", type=int, default=100)
    parser.add_argument("--payload-grid-resolution", type=float, default=0.1)
    parser.add_argument("--activation-distance", type=float, default=0.0)
    parser.add_argument("--no-meshes", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    snapshot_path = Path(args.snapshot).expanduser().resolve()
    robot_path = Path(args.robot_config).expanduser().resolve()
    scene_path = Path(args.scene_config).expanduser().resolve()
    unrepaired_scene_path = (
        Path(args.unrepaired_scene_config).expanduser().resolve()
        if args.unrepaired_scene_config
        else scene_path
    )
    save_path = Path(args.save).expanduser().resolve()
    summary_path = (
        Path(args.summary).expanduser().resolve()
        if args.summary else save_path.with_suffix(".json")
    )
    save_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.parent.mkdir(parents=True, exist_ok=True)

    snapshot = json.loads(snapshot_path.read_text())
    _, urdf_path, local_robot_spheres = load_robot_model(robot_path)
    cuboids = load_scene(scene_path)
    unrepaired_cuboids = load_scene(unrepaired_scene_path)
    unrepaired_collision_cuboids = tuple(
        cuboid for cuboid in unrepaired_cuboids if cuboid.name.endswith("_rear_guard")
    )
    if not unrepaired_collision_cuboids:
        raise ValueError(
            f"unrepaired production scene has no *_rear_guard cuboid: {unrepaired_scene_path}"
        )
    repaired = repaired_samples(snapshot, args.stride)
    unrepaired = unrepaired_loaded_samples(snapshot, max(2, args.unrepaired_steps))

    try:
        from alfa_robot_rerun import visualize_rerun as helpers
    except ImportError as exc:
        raise RuntimeError("source the workspace and make alfa_robot_rerun importable") from exc
    rr = helpers.rr
    rr.init("alfa_curobo_snapshot_collision_spheres", recording_id=save_path.stem, spawn=False)
    rr.save(str(save_path))
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log(
        "world/legend",
        rr.TextLog(
            "repaired flow: blue=robot spheres, cyan/purple=payload spheres, "
            "red=sphere/cuboid intersection; unrepaired direct path: robot mesh plus "
            "actual carried boxes only, red=actual-box production rear_guard overlap"
        ),
        static=True,
    )

    robot = helpers.UrdfRobot(urdf_path.read_text())
    helpers.log_robot_static_model(robot, "world/robot_visual", log_meshes=not args.no_meshes)
    section_stats: dict[str, dict[str, Any]] = {}
    samples = [*repaired, *unrepaired]
    for sample_index, sample in enumerate(samples):
        helpers.set_sample_time(sample_index)
        transforms = robot.fk(sample.joints)
        helpers.log_robot_state(robot, sample.joints, "world/robot_visual")
        payloads = payloads_from_attached(sample.attached_boxes)
        if sample.section == "unrepaired_loaded_shortcut":
            clear_sphere_layers(rr)
            payload_collisions, obstacle_counts = payload_aabb_collision_map(
                payloads, transforms, unrepaired_collision_cuboids
            )
            log_scene(
                rr,
                unrepaired_cuboids,
                {
                    cuboid.name: obstacle_counts.get(cuboid.name, 0)
                    for cuboid in unrepaired_cuboids
                },
            )
            log_actual_payload_boxes(rr, payloads, transforms, payload_collisions)
            collision_pairs = sorted(
                f"actual_payload_{side}<->{obstacle}"
                for side, obstacles in payload_collisions.items()
                for obstacle in obstacles
            )
            collision_model = "actual_payload_world_aabb_vs_production_rear_guard"
        else:
            robot_spheres = transform_spheres(local_robot_spheres, transforms)
            robot_hits, robot_counts = collision_map(
                robot_spheres, cuboids, args.activation_distance
            )
            payload_spheres: dict[str, tuple[Sphere, ...]] = {}
            payload_hits: dict[str, dict[int, tuple[str, ...]]] = {}
            obstacle_counts = dict(robot_counts)
            for payload in payloads:
                local = cuboid_covering_spheres(payload, args.payload_grid_resolution)
                world = transform_spheres(local, transforms)
                hits, counts = collision_map(world, cuboids, args.activation_distance)
                payload_spheres[payload.side] = world
                payload_hits[payload.side] = hits
                for obstacle, count in counts.items():
                    obstacle_counts[obstacle] += count

            log_scene(rr, cuboids, obstacle_counts)
            log_sphere_layer(
                rr,
                "world/collision_model/robot_spheres",
                robot_spheres,
                robot_hits,
                [30, 135, 255, 65],
            )
            for side, color in (
                ("left", [0, 220, 255, 90]),
                ("right", [210, 70, 255, 90]),
            ):
                log_sphere_layer(
                    rr,
                    f"world/collision_model/{side}_payload_spheres",
                    payload_spheres.get(side, ()),
                    payload_hits.get(side, {}),
                    color,
                )
            if payloads:
                log_payload_boxes(rr, payloads, transforms)
            else:
                rr.log(
                    "world/collision_model/payload_boxes",
                    rr.Boxes3D(centers=[], half_sizes=[]),
                )
            collision_pairs = sorted(
                {
                    f"robot:{robot_spheres[index].link_name}<->{obstacle}"
                    for index, obstacles in robot_hits.items()
                    for obstacle in obstacles
                }
                | {
                    f"payload_{side}:{payload_spheres[side][index].link_name}<->{obstacle}"
                    for side, hits in payload_hits.items()
                    for index, obstacles in hits.items()
                    for obstacle in obstacles
                }
            )
            collision_model = "curobo_sphere_vs_cuboid"
        rr.log(
            "world/info",
            rr.TextLog(
                f"{sample.section} | {sample.stage} | "
                f"point {sample.point_index + 1}/{sample.point_count} | "
                f"model={collision_model} | "
                + (", ".join(collision_pairs) if collision_pairs else "no collision")
            ),
        )
        stats = section_stats.setdefault(
            sample.section,
            {
                "samples": 0,
                "collision_samples": 0,
                "collision_pairs": set(),
                "collision_samples_by_stage": {},
                "collision_pairs_by_stage": {},
                "collision_model": collision_model,
            },
        )
        stats["samples"] += 1
        if collision_pairs:
            stats["collision_samples"] += 1
            stats["collision_pairs"].update(collision_pairs)
            stats["collision_samples_by_stage"][sample.stage] = (
                stats["collision_samples_by_stage"].get(sample.stage, 0) + 1
            )
            stats["collision_pairs_by_stage"].setdefault(sample.stage, set()).update(
                collision_pairs
            )

    serializable_stats = {
        name: {
            **values,
            "collision_pairs": sorted(values["collision_pairs"]),
            "collision_pairs_by_stage": {
                stage: sorted(pairs)
                for stage, pairs in values["collision_pairs_by_stage"].items()
            },
        }
        for name, values in section_stats.items()
    }
    summary = {
        "snapshot": str(snapshot_path),
        "robot_config": str(robot_path),
        "scene_config": str(scene_path),
        "unrepaired_scene_config": str(unrepaired_scene_path),
        "rerun": str(save_path),
        "robot_sphere_count": len(local_robot_spheres),
        "payload_grid_resolution_m": args.payload_grid_resolution,
        "payload_spheres_per_top_box": len(
            cuboid_covering_spheres(
                Payload("left", 0, "left_tool0", "top_suction", (0.0, 0.0, 0.2), (0.3, 0.4, 0.4)),
                args.payload_grid_resolution,
            )
        ),
        "scene_cuboid_count": len(cuboids),
        "sections": serializable_stats,
        "note": (
            "The repaired flow uses cuRobo sphere-versus-cuboid visualization. The unrepaired "
            "direct comparison hides every sphere layer and highlights actual carried boxes using "
            "the production world-AABB versus rear_guard gate. MoveIt/FCL and cuRobo self-collision "
            "are not recomputed by this offline viewer."
        ),
    }
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
    print(f"robot spheres: {len(local_robot_spheres)}")
    print(f"sections: {json.dumps(serializable_stats, ensure_ascii=False)}")
    print(f"saved Rerun recording: {save_path}")
    print(f"saved audit summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
