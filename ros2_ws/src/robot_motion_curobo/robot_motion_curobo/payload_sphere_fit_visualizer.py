"""Compare fitted payload collision spheres in the current Rerun scene."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from . import collision_model_visualizer as base


FIT_MODES = ("conservative_grid", "morphit", "voxel", "surface")


def fitted_spheres_for_payload(
    payload: base.Payload,
    fit_record: dict[str, Any],
) -> tuple[base.Sphere, ...]:
    centers = fit_record.get("centers", [])
    radii = fit_record.get("radii", [])
    if len(centers) != len(radii):
        raise ValueError(f"fit center/radius mismatch for {payload.side}")
    rotation = base.quaternion_xyzw_to_matrix(payload.orientation_xyzw)
    result = []
    for center, radius_value in zip(centers, radii):
        radius = float(radius_value)
        local = np.asarray(payload.center_xyz) + rotation @ np.asarray(center, dtype=float)
        if radius <= 0.0:
            raise ValueError(f"fit contains non-positive radius for {payload.side}")
        result.append(
            base.Sphere(
                link_name=payload.link_name,
                center=tuple(float(value) for value in local),
                radius=radius,
                kind=f"payload_{payload.side}",
            )
        )
    return tuple(result)


def fit_for_payload(
    document: dict[str, Any], payload: base.Payload, mode: str
) -> tuple[tuple[base.Sphere, ...], dict[str, Any]]:
    if mode == "conservative_grid":
        spheres = base.cuboid_covering_spheres(payload, 0.1)
        radius = spheres[0].radius if spheres else 0.0
        protrusion = max(0.0, radius - 0.05)
        return spheres, {
            "sphere_count": len(spheres),
            "metrics": None,
            "exact_axis_protrusion": {
                "per_axis_m": [protrusion, protrusion, protrusion],
                "maximum_m": protrusion,
            },
        }
    box_name = "top_suction" if payload.grasp_mode == "top_suction" else "front"
    try:
        fit_record = document["boxes"][box_name]["fits"][mode]
    except (KeyError, TypeError) as error:
        raise ValueError(f"fit JSON has no {box_name}/{mode} record") from error
    return fitted_spheres_for_payload(payload, fit_record), fit_record


def fit_metrics_text(mode: str, side: str, fit: dict[str, Any], hits: int) -> str:
    exact = fit.get("exact_axis_protrusion") or {}
    metrics = fit.get("metrics")
    common = (
        f"{side}: n={fit.get('sphere_count', '?')}, hits={hits}, "
        f"max_axis_out={1000.0 * float(exact.get('maximum_m', 0.0)):.2f}mm"
    )
    if mode == "conservative_grid" or not isinstance(metrics, dict):
        return common
    return (
        common
        + f", coverage={100.0 * float(metrics.get('coverage', 0.0)):.1f}%"
        + f", protruding_surface={100.0 * float(metrics.get('protrusion', 0.0)):.1f}%"
        + f", surface_gap_p95={1000.0 * float(metrics.get('surface_gap_p95', 0.0)):.1f}mm"
    )


def build_parser(repo_root: Path) -> argparse.ArgumentParser:
    config_root = repo_root / "ros2_ws/src/robot_motion_curobo/config"
    parser = argparse.ArgumentParser(
        description="Render conservative/MORPHIT/VOXEL/SURFACE payload fits in one Rerun timeline."
    )
    parser.add_argument("--tasks", default=str(config_root / "cspace_tuning_tasks.yml"))
    parser.add_argument("--task", default=base.DEFAULT_TASK)
    parser.add_argument("--fits", required=True, help="JSON from payload_sphere_fit_generator")
    parser.add_argument("--modes", default=",".join(FIT_MODES))
    parser.add_argument("--endpoint", choices=("start", "goal", "both"), default="both")
    parser.add_argument("--robot-config", default="")
    parser.add_argument("--scene-config", default="")
    parser.add_argument("--activation-distance", type=float, default=0.0)
    parser.add_argument("--no-meshes", action="store_true")
    parser.add_argument("--save", default="")
    parser.add_argument("--summary", default="")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    repo_root = base.find_repo_root()
    args = build_parser(repo_root).parse_args(argv)
    modes = tuple(value.strip().lower() for value in args.modes.split(",") if value.strip())
    unsupported = [mode for mode in modes if mode not in FIT_MODES]
    if unsupported:
        raise ValueError(f"unsupported modes: {unsupported}")

    fits_path = Path(args.fits).expanduser().resolve()
    fit_document = json.loads(fits_path.read_text())
    tasks_path = Path(args.tasks).expanduser().resolve()
    tasks_document, task = base.load_tasks(tasks_path, args.task)
    robot_value = args.robot_config or tasks_document.get("robot_config_path", "")
    scene_value = args.scene_config or task.get("scene_config_path", "")
    if not robot_value or not scene_value:
        raise ValueError("robot/scene config path is missing")
    robot_path = base._resolve_path(str(robot_value), tasks_path.parent)
    scene_path = base._resolve_path(str(scene_value), tasks_path.parent)
    kinematics, urdf_path, local_robot_spheres = base.load_robot_model(robot_path)
    cuboids = base.load_scene(scene_path)
    payloads = base.payloads_from_task(task)

    save_path = Path(args.save).expanduser().resolve() if args.save else (
        repo_root / "data/collision_visualization" / f"{args.task}_payload_sphere_fit_comparison.rrd"
    )
    summary_path = Path(args.summary).expanduser().resolve() if args.summary else save_path.with_suffix(".json")
    save_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.parent.mkdir(parents=True, exist_ok=True)

    helpers = base.load_rerun_helpers(repo_root)
    rr = helpers.rr
    rr.init("alfa_payload_sphere_fit_comparison", recording_id=args.task, spawn=False)
    rr.save(str(save_path))
    robot = helpers.UrdfRobot(urdf_path.read_text())
    helpers.log_robot_static_model(robot, "world/robot_visual", log_meshes=not args.no_meshes)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log(
        "world/legend",
        rr.TextLog(
            "timeline order: endpoint then conservative_grid/MORPHIT/VOXEL/SURFACE; "
            "robot=blue, left payload=cyan, right payload=purple, scene intersection=red"
        ),
        static=True,
    )

    endpoints = ("start", "goal") if args.endpoint == "both" else (args.endpoint,)
    audit: dict[str, Any] = {
        "task": args.task,
        "fits_path": str(fits_path),
        "robot_config_path": str(robot_path),
        "scene_config_path": str(scene_path),
        "modes": list(modes),
        "endpoints": {},
    }
    sample = 0
    for endpoint in endpoints:
        joints = base.task_joint_map(task[endpoint], kinematics)
        transforms = robot.fk(joints)
        world_robot_spheres = base.transform_spheres(local_robot_spheres, transforms)
        robot_collisions, robot_obstacle_counts = base.collision_map(
            world_robot_spheres, cuboids, args.activation_distance
        )
        endpoint_audit: dict[str, Any] = {}
        for mode in modes:
            local_payload_spheres: dict[str, tuple[base.Sphere, ...]] = {}
            fit_records: dict[str, dict[str, Any]] = {}
            for payload in payloads:
                local_payload_spheres[payload.side], fit_records[payload.side] = fit_for_payload(
                    fit_document, payload, mode
                )

            world_payload_spheres: dict[str, tuple[base.Sphere, ...]] = {}
            payload_collisions: dict[str, dict[int, tuple[str, ...]]] = {}
            obstacle_counts = dict(robot_obstacle_counts)
            for side, local_spheres in local_payload_spheres.items():
                world_spheres = base.transform_spheres(local_spheres, transforms)
                collisions, counts = base.collision_map(
                    world_spheres, cuboids, args.activation_distance
                )
                world_payload_spheres[side] = world_spheres
                payload_collisions[side] = collisions
                for name, count in counts.items():
                    obstacle_counts[name] += count

            helpers.set_sample_time(sample)
            helpers.log_robot_state(robot, joints, "world/robot_visual")
            base.log_scene(rr, cuboids, obstacle_counts)
            base.log_sphere_layer(
                rr,
                "world/collision_model/robot_spheres",
                world_robot_spheres,
                robot_collisions,
                [30, 135, 255, 65],
            )
            for side, spheres in world_payload_spheres.items():
                base.log_sphere_layer(
                    rr,
                    f"world/collision_model/{side}_payload_spheres",
                    spheres,
                    payload_collisions[side],
                    [0, 220, 255, 110] if side == "left" else [210, 70, 255, 110],
                )
            base.log_payload_boxes(rr, payloads, transforms)

            lines = [f"sample={sample} endpoint={endpoint} fit={mode}"]
            for side in ("left", "right"):
                lines.append(
                    fit_metrics_text(
                        mode, side, fit_records[side], len(payload_collisions[side])
                    )
                )
            rr.log("world/status", rr.TextLog("\n".join(lines)))
            collision_pairs = sorted(
                {
                    f"payload_{side}:{spheres[index].link_name} <-> {obstacle}"
                    for side, collisions in payload_collisions.items()
                    for index, obstacles in collisions.items()
                    for obstacle in obstacles
                    for spheres in (world_payload_spheres[side],)
                }
            )
            rr.log(
                "world/collision_pairs",
                rr.TextLog("\n".join(collision_pairs) if collision_pairs else "no payload/scene intersections"),
            )
            endpoint_audit[mode] = {
                "sample": sample,
                "robot_scene_colliding_spheres": len(robot_collisions),
                "payload_sphere_counts": {
                    side: len(spheres) for side, spheres in world_payload_spheres.items()
                },
                "payload_scene_colliding_spheres": {
                    side: len(collisions) for side, collisions in payload_collisions.items()
                },
                "obstacle_hit_sphere_counts": {
                    name: count for name, count in obstacle_counts.items() if count
                },
                "fit_metrics": fit_records,
                "collision_pairs": collision_pairs,
            }
            print(
                f"sample={sample} {endpoint}/{mode}: "
                + ", ".join(
                    f"{side}={len(world_payload_spheres[side])} spheres/"
                    f"{len(payload_collisions[side])} hits"
                    for side in ("left", "right")
                )
            )
            sample += 1
        audit["endpoints"][endpoint] = endpoint_audit

    summary_path.write_text(json.dumps(audit, indent=2, ensure_ascii=False) + "\n")
    print(f"saved Rerun comparison: {save_path}")
    print(f"saved audit summary: {summary_path}")
    print(f"open with: rerun {save_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
