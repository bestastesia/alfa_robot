"""Rerun visualization for a cuRobo world and its sphere collision model.

The implementation deliberately does not import cuRobo or Torch.  It reads the
same generated robot/scene YAML files as the planner, evaluates the robot URDF
with the requested task state, and transforms every configured sphere into the
world frame.  This keeps the visual audit usable on machines without CUDA.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np
import yaml

from .planner_core import AUTHORITY_JOINT_NAMES


DEFAULT_TASK = "loaded_L11_R8"
DEFAULT_PAYLOAD_GRID_RESOLUTION_M = 0.1


@dataclass(frozen=True)
class Cuboid:
    name: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True)
class Sphere:
    link_name: str
    center: tuple[float, float, float]
    radius: float
    kind: str = "robot"


@dataclass(frozen=True)
class Payload:
    side: str
    box_id: int
    link_name: str
    grasp_mode: str
    center_xyz: tuple[float, float, float]
    size_xyz: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


def _as_float_tuple(values: Iterable[Any], size: int, label: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != size or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} must contain {size} finite values")
    return result


def quaternion_xyzw_to_matrix(quaternion: Sequence[float]) -> np.ndarray:
    x, y, z, w = _as_float_tuple(quaternion, 4, "quaternion")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("quaternion has zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_to_quaternion_xyzw(matrix: np.ndarray) -> tuple[float, float, float, float]:
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
    return float(x), float(y), float(z), float(w)


def cuboid_rotation(cuboid: Cuboid) -> np.ndarray:
    w, x, y, z = cuboid.quaternion_wxyz
    return quaternion_xyzw_to_matrix((x, y, z, w))


def sphere_intersects_cuboid(
    sphere: Sphere,
    cuboid: Cuboid,
    activation_distance: float = 0.0,
) -> bool:
    """Exact sphere versus oriented-box intersection test."""
    local = cuboid_rotation(cuboid).T @ (
        np.asarray(sphere.center, dtype=float) - np.asarray(cuboid.center, dtype=float)
    )
    half = 0.5 * np.asarray(cuboid.size, dtype=float)
    closest = np.clip(local, -half, half)
    delta = local - closest
    radius = sphere.radius + max(0.0, float(activation_distance))
    return float(delta @ delta) <= radius * radius


def cuboid_covering_spheres(payload: Payload, nominal_cell_size: float) -> tuple[Sphere, ...]:
    """Match the conservative circumscribed-cell payload cover used by the backend."""
    if not math.isfinite(nominal_cell_size) or nominal_cell_size <= 0.0:
        raise ValueError("payload grid resolution must be positive")
    counts = tuple(max(1, int(math.ceil(size / nominal_cell_size))) for size in payload.size_xyz)
    cell = tuple(size / count for size, count in zip(payload.size_xyz, counts))
    radius = 0.5 * math.sqrt(sum(value * value for value in cell))
    rotation = quaternion_xyzw_to_matrix(payload.orientation_xyzw)
    spheres = []
    for ix in range(counts[0]):
        for iy in range(counts[1]):
            for iz in range(counts[2]):
                local = np.array(
                    [
                        -0.5 * payload.size_xyz[0] + (ix + 0.5) * cell[0],
                        -0.5 * payload.size_xyz[1] + (iy + 0.5) * cell[1],
                        -0.5 * payload.size_xyz[2] + (iz + 0.5) * cell[2],
                    ],
                    dtype=float,
                )
                center = np.asarray(payload.center_xyz) + rotation @ local
                spheres.append(
                    Sphere(
                        link_name=payload.link_name,
                        center=tuple(float(value) for value in center),
                        radius=radius,
                        kind=f"payload_{payload.side}",
                    )
                )
    return tuple(spheres)


def transform_spheres(
    local_spheres: Iterable[Sphere], transforms: dict[str, np.ndarray]
) -> tuple[Sphere, ...]:
    world_spheres = []
    for sphere in local_spheres:
        transform = transforms.get(sphere.link_name)
        if transform is None:
            raise ValueError(f"sphere link is absent from URDF: {sphere.link_name}")
        center = transform[:3, :3] @ np.asarray(sphere.center) + transform[:3, 3]
        world_spheres.append(
            Sphere(
                link_name=sphere.link_name,
                center=tuple(float(value) for value in center),
                radius=sphere.radius,
                kind=sphere.kind,
            )
        )
    return tuple(world_spheres)


def _resolve_path(value: str, base: Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (base / path).resolve()


def load_tasks(path: Path, task_name: str) -> tuple[dict[str, Any], dict[str, Any]]:
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict) or not isinstance(document.get("tasks"), dict):
        raise ValueError(f"invalid tasks YAML: {path}")
    if task_name not in document["tasks"]:
        raise ValueError(f"unknown task {task_name!r}; choices={sorted(document['tasks'])}")
    task = document["tasks"][task_name]
    if not isinstance(task, dict):
        raise ValueError(f"invalid task entry: {task_name}")
    return document, task


def load_robot_model(path: Path) -> tuple[dict[str, Any], Path, tuple[Sphere, ...]]:
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict):
        raise ValueError(f"invalid robot YAML: {path}")
    robot_cfg = document.get("robot_cfg", document)
    kinematics = robot_cfg.get("kinematics") if isinstance(robot_cfg, dict) else None
    if not isinstance(kinematics, dict):
        raise ValueError(f"robot YAML has no kinematics map: {path}")

    asset_root = _resolve_path(str(kinematics.get("asset_root_path", ".")), path.parent)
    urdf_value = kinematics.get("urdf_path")
    if not isinstance(urdf_value, str) or not urdf_value:
        raise ValueError(f"robot YAML has no urdf_path: {path}")
    urdf_path = _resolve_path(urdf_value, asset_root)
    if not urdf_path.is_file():
        raise FileNotFoundError(f"robot URDF not found: {urdf_path}")

    sphere_document: Any = kinematics.get("collision_spheres")
    if isinstance(sphere_document, str):
        sphere_path = _resolve_path(sphere_document, path.parent)
        sphere_document = yaml.safe_load(sphere_path.read_text())
        if isinstance(sphere_document, dict) and "collision_spheres" in sphere_document:
            sphere_document = sphere_document["collision_spheres"]
    if not isinstance(sphere_document, dict):
        raise ValueError(f"robot YAML has no collision sphere map: {path}")

    buffer = float(kinematics.get("collision_sphere_buffer", 0.0))
    spheres = []
    for link_name, entries in sphere_document.items():
        if not isinstance(entries, list):
            raise ValueError(f"sphere list for {link_name} is invalid")
        for entry in entries:
            if not isinstance(entry, dict):
                raise ValueError(f"sphere entry for {link_name} is invalid")
            center = _as_float_tuple(entry.get("center", ()), 3, f"{link_name}.center")
            radius = float(entry.get("radius", 0.0)) + buffer
            if not math.isfinite(radius) or radius <= 0.0:
                raise ValueError(f"sphere radius for {link_name} must be positive")
            spheres.append(Sphere(str(link_name), center, radius))
    return kinematics, urdf_path, tuple(spheres)


def load_scene(path: Path) -> tuple[Cuboid, ...]:
    document = yaml.safe_load(path.read_text())
    cuboids = document.get("cuboid") if isinstance(document, dict) else None
    if not isinstance(cuboids, dict):
        raise ValueError(f"scene YAML has no cuboid map: {path}")
    result = []
    for name, value in cuboids.items():
        if not isinstance(value, dict):
            raise ValueError(f"invalid cuboid {name}")
        dims = _as_float_tuple(value.get("dims", ()), 3, f"{name}.dims")
        pose = _as_float_tuple(value.get("pose", ()), 7, f"{name}.pose")
        if not all(dimension > 0.0 for dimension in dims):
            raise ValueError(f"cuboid {name} dimensions must be positive")
        result.append(Cuboid(str(name), pose[:3], dims, pose[3:]))
    return tuple(result)


def payloads_from_task(task: dict[str, Any]) -> tuple[Payload, ...]:
    payloads = []
    for side in ("left", "right"):
        box_id_value = task.get(f"{side}_box_id")
        if box_id_value is None:
            continue
        grasp_mode = str(task.get(f"{side}_grasp_mode", "front"))
        if grasp_mode == "top_suction":
            center = (0.0, 0.0, 0.2)
            size = (0.3, 0.4, 0.4)
        elif grasp_mode == "front":
            center = (0.0, 0.0, 0.15)
            size = (0.4, 0.4, 0.3)
        else:
            raise ValueError(f"unsupported {side} grasp mode: {grasp_mode}")
        payloads.append(
            Payload(
                side=side,
                box_id=int(box_id_value),
                link_name=f"{side}_tool0",
                grasp_mode=grasp_mode,
                center_xyz=center,
                size_xyz=size,
            )
        )
    return tuple(payloads)


def task_joint_map(
    task_positions: Sequence[float], kinematics: dict[str, Any]
) -> dict[str, float]:
    values = _as_float_tuple(task_positions, len(AUTHORITY_JOINT_NAMES), "task state")
    result = {
        str(name): float(value)
        for name, value in (kinematics.get("lock_joints") or {}).items()
    }
    result.update(zip(AUTHORITY_JOINT_NAMES, values))
    return result


def find_repo_root() -> Path:
    for start in (Path.cwd().resolve(), Path(__file__).resolve()):
        for candidate in (start, *start.parents):
            if (candidate / ".ai_teamwork" / "START.md").exists():
                return candidate
    raise FileNotFoundError("cannot find repository root")


def load_rerun_helpers(repo_root: Path):
    del repo_root
    try:
        from alfa_robot_rerun import visualize_rerun
    except ImportError as exc:
        raise RuntimeError(
            "alfa_robot_rerun is required for collision visualization; "
            "source the workspace before running this tool"
        ) from exc
    return visualize_rerun


def collision_map(
    spheres: Iterable[Sphere], cuboids: Sequence[Cuboid], activation_distance: float
) -> tuple[dict[int, tuple[str, ...]], dict[str, int]]:
    collisions: dict[int, tuple[str, ...]] = {}
    obstacle_counts = {cuboid.name: 0 for cuboid in cuboids}
    for index, sphere in enumerate(spheres):
        names = tuple(
            cuboid.name
            for cuboid in cuboids
            if sphere_intersects_cuboid(sphere, cuboid, activation_distance)
        )
        if names:
            collisions[index] = names
            for name in names:
                obstacle_counts[name] += 1
    return collisions, obstacle_counts


def _log_ellipsoids(rr: Any, path: str, spheres: Sequence[Sphere], color: list[int]) -> None:
    if spheres:
        rr.log(
            path,
            rr.Ellipsoids3D(
                centers=[sphere.center for sphere in spheres],
                half_sizes=[[sphere.radius] * 3 for sphere in spheres],
                colors=[color] * len(spheres),
                show_labels=False,
            ),
        )
    else:
        rr.log(path, rr.Ellipsoids3D(centers=[], half_sizes=[]))


def log_sphere_layer(
    rr: Any,
    path: str,
    spheres: Sequence[Sphere],
    collisions: dict[int, tuple[str, ...]],
    clear_color: list[int],
) -> None:
    clear = [sphere for index, sphere in enumerate(spheres) if index not in collisions]
    colliding = [sphere for index, sphere in enumerate(spheres) if index in collisions]
    _log_ellipsoids(rr, f"{path}/clear", clear, clear_color)
    _log_ellipsoids(rr, f"{path}/colliding", colliding, [255, 25, 25, 210])


def log_scene(
    rr: Any, cuboids: Sequence[Cuboid], obstacle_counts: dict[str, int]
) -> None:
    colors = [
        [255, 45, 45, 100] if obstacle_counts[cuboid.name] else [240, 170, 40, 55]
        for cuboid in cuboids
    ]
    rr.log(
        "world/collision_scene/cuboids",
        rr.Boxes3D(
            centers=[cuboid.center for cuboid in cuboids],
            half_sizes=[[dimension * 0.5 for dimension in cuboid.size] for cuboid in cuboids],
            quaternions=[
                [
                    cuboid.quaternion_wxyz[1],
                    cuboid.quaternion_wxyz[2],
                    cuboid.quaternion_wxyz[3],
                    cuboid.quaternion_wxyz[0],
                ]
                for cuboid in cuboids
            ],
            colors=colors,
            labels=[cuboid.name for cuboid in cuboids],
        ),
    )


def log_payload_boxes(rr: Any, payloads: Sequence[Payload], transforms: dict[str, np.ndarray]) -> None:
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
        half_sizes.append([value * 0.5 for value in payload.size_xyz])
        quaternions.append(matrix_to_quaternion_xyzw(transform[:3, :3] @ local_rotation))
        colors.append([0, 210, 255, 35] if payload.side == "left" else [210, 70, 255, 35])
        labels.append(f"{payload.side} box {payload.box_id} ({payload.grasp_mode})")
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


def build_parser(repo_root: Path) -> argparse.ArgumentParser:
    default_tasks = repo_root / "ros2_ws/src/robot_motion_curobo/config/cspace_tuning_tasks.yml"
    parser = argparse.ArgumentParser(
        description="Visualize a cuRobo cuboid scene and robot/payload collision spheres in Rerun."
    )
    parser.add_argument("--tasks", default=str(default_tasks), help="Task YAML containing state and scene paths")
    parser.add_argument("--task", default=DEFAULT_TASK, help="Task name from --tasks")
    parser.add_argument("--robot-config", default="", help="Override robot YAML from the task document")
    parser.add_argument("--scene-config", default="", help="Override scene YAML from the selected task")
    parser.add_argument("--endpoint", choices=("start", "goal", "both"), default="both")
    parser.add_argument("--payload-grid-resolution", type=float, default=DEFAULT_PAYLOAD_GRID_RESOLUTION_M)
    parser.add_argument("--activation-distance", type=float, default=0.0)
    parser.add_argument("--no-payloads", action="store_true", help="Hide attached boxes and reserved payload spheres")
    parser.add_argument("--no-meshes", action="store_true", help="Do not include robot visual STL assets")
    parser.add_argument("--connect", action="store_true", help="Also stream to an already-running Rerun viewer")
    parser.add_argument("--save", default="", help="Output .rrd; default is data/collision_visualization/<task>.rrd")
    parser.add_argument("--summary", default="", help="Output JSON audit summary next to the RRD by default")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    repo_root = find_repo_root()
    args = build_parser(repo_root).parse_args(argv)
    tasks_path = Path(args.tasks).expanduser().resolve()
    tasks_document, task = load_tasks(tasks_path, args.task)

    robot_value = args.robot_config or tasks_document.get("robot_config_path", "")
    if not robot_value:
        raise ValueError("robot config is missing; pass --robot-config")
    robot_path = _resolve_path(str(robot_value), tasks_path.parent)
    scene_value = args.scene_config or task.get("scene_config_path", "")
    if not scene_value:
        raise ValueError("scene config is missing; pass --scene-config")
    scene_path = _resolve_path(str(scene_value), tasks_path.parent)

    kinematics, urdf_path, local_robot_spheres = load_robot_model(robot_path)
    cuboids = load_scene(scene_path)
    payloads = () if args.no_payloads else payloads_from_task(task)
    local_payload_spheres = {
        payload.side: cuboid_covering_spheres(payload, args.payload_grid_resolution)
        for payload in payloads
    }

    save_path = Path(args.save).expanduser().resolve() if args.save else (
        repo_root / "data/collision_visualization" / f"{args.task}_collision_spheres.rrd"
    )
    summary_path = Path(args.summary).expanduser().resolve() if args.summary else save_path.with_suffix(".json")
    save_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.parent.mkdir(parents=True, exist_ok=True)

    helpers = load_rerun_helpers(repo_root)
    rr = helpers.rr
    rr.init("alfa_curobo_collision_model", recording_id=args.task, spawn=False)
    rr.save(str(save_path))
    if args.connect:
        rr.connect()

    robot = helpers.UrdfRobot(urdf_path.read_text())
    helpers.log_robot_static_model(robot, "world/robot_visual", log_meshes=not args.no_meshes)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log(
        "world/legend",
        rr.TextLog(
            "robot spheres=blue; left payload=cyan; right payload=purple; "
            "sphere/scene intersections=red; scene cuboids=orange"
        ),
        static=True,
    )

    endpoints = ("start", "goal") if args.endpoint == "both" else (args.endpoint,)
    audit: dict[str, Any] = {
        "task": args.task,
        "tasks_path": str(tasks_path),
        "robot_config_path": str(robot_path),
        "urdf_path": str(urdf_path),
        "scene_config_path": str(scene_path),
        "activation_distance_m": args.activation_distance,
        "payload_grid_resolution_m": args.payload_grid_resolution,
        "robot_sphere_count": len(local_robot_spheres),
        "scene_cuboid_count": len(cuboids),
        "endpoints": {},
        "note": "Red highlighting is sphere-vs-scene geometry only; self-collision is not evaluated.",
    }

    for sample, endpoint in enumerate(endpoints):
        if endpoint not in task:
            raise ValueError(f"task {args.task} has no {endpoint} state")
        joint_map = task_joint_map(task[endpoint], kinematics)
        transforms = robot.fk(joint_map)
        world_robot_spheres = transform_spheres(local_robot_spheres, transforms)
        robot_collisions, robot_obstacle_counts = collision_map(
            world_robot_spheres, cuboids, args.activation_distance
        )

        world_payload_spheres: dict[str, tuple[Sphere, ...]] = {}
        payload_collisions: dict[str, dict[int, tuple[str, ...]]] = {}
        obstacle_counts = dict(robot_obstacle_counts)
        for side, local_spheres in local_payload_spheres.items():
            world_spheres = transform_spheres(local_spheres, transforms)
            collisions, counts = collision_map(world_spheres, cuboids, args.activation_distance)
            world_payload_spheres[side] = world_spheres
            payload_collisions[side] = collisions
            for name, count in counts.items():
                obstacle_counts[name] += count

        helpers.set_sample_time(sample)
        helpers.log_robot_state(robot, joint_map, "world/robot_visual")
        log_scene(rr, cuboids, obstacle_counts)
        log_sphere_layer(
            rr,
            "world/collision_model/robot_spheres",
            world_robot_spheres,
            robot_collisions,
            [30, 135, 255, 75],
        )
        for side, spheres in world_payload_spheres.items():
            log_sphere_layer(
                rr,
                f"world/collision_model/{side}_payload_spheres",
                spheres,
                payload_collisions[side],
                [0, 220, 255, 95] if side == "left" else [210, 70, 255, 95],
            )
        if payloads:
            log_payload_boxes(rr, payloads, transforms)

        collision_pairs = sorted(
            {
                f"robot:{world_robot_spheres[index].link_name} <-> {obstacle}"
                for index, obstacles in robot_collisions.items()
                for obstacle in obstacles
            }
            | {
                f"payload_{side}:{spheres[index].link_name} <-> {obstacle}"
                for side, collisions in payload_collisions.items()
                for index, obstacles in collisions.items()
                for obstacle in obstacles
                for spheres in (world_payload_spheres[side],)
            }
        )
        status = (
            f"{endpoint}: robot={len(world_robot_spheres)} spheres, "
            f"robot-scene hits={len(robot_collisions)}, "
            + ", ".join(
                f"{side}-payload={len(world_payload_spheres[side])} spheres/"
                f"{len(payload_collisions[side])} hits"
                for side in sorted(world_payload_spheres)
            )
        )
        rr.log("world/status", rr.TextLog(status))
        if collision_pairs:
            rr.log("world/collision_pairs", rr.TextLog("\n".join(collision_pairs)))
        else:
            rr.log("world/collision_pairs", rr.TextLog("no sphere/scene intersections"))

        audit["endpoints"][endpoint] = {
            "joint_map": joint_map,
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
            "collision_pairs": collision_pairs,
        }

    summary_path.write_text(json.dumps(audit, indent=2, ensure_ascii=False) + "\n")
    print(f"task: {args.task}")
    print(f"robot spheres: {len(local_robot_spheres)}")
    print(f"scene cuboids: {len(cuboids)}")
    for endpoint, result in audit["endpoints"].items():
        payload_hits = result["payload_scene_colliding_spheres"]
        print(
            f"{endpoint}: robot hits={result['robot_scene_colliding_spheres']}, "
            f"payload hits={payload_hits}"
        )
    print(f"saved Rerun recording: {save_path}")
    print(f"saved audit summary: {summary_path}")
    print(f"open with: rerun {save_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
