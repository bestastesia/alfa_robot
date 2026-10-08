#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation



ACTIVE_JOINTS = [
    "updown",
    *[f"left_joint{i}" for i in range(1, 8)],
    *[f"right_joint{i}" for i in range(1, 8)],
]
LOCKED_JOINTS = {
    "head_joint": 0.0,
    "head_pitch_joint": 0.0,
}
CHASSIS_LINKS = {
    "base_link",
}
PLANNING_COLLISION_LINKS = {
    "base_link",
    "arm_carriage",
    "head_yaw",
    "head",
    *[f"left_link{i}" for i in range(1, 8)],
    *[f"right_link{i}" for i in range(1, 8)],
    "left_suction",
    "right_suction",
}
ROUNDS = [
    (24, 20, False),
    (23, 21, False),
    (22, None, False),
    (19, 15, False),
    (18, 16, False),
    (None, 17, False),
    (14, 10, False),
    (13, 11, False),
    (12, None, False),
    (9, 5, False),
    (8, 6, False),
    (None, 7, False),
    (4, 0, True),
    (3, 1, True),
    (2, None, True),
]


def transform(xyz: str | None, rpy: str | None) -> np.ndarray:
    result = np.eye(4)
    if xyz:
        result[:3, 3] = np.fromstring(xyz, sep=" ")
    if rpy:
        result[:3, :3] = Rotation.from_euler("xyz", np.fromstring(rpy, sep=" ")).as_matrix()
    return result


def joint_motion(joint: ET.Element, value: float) -> np.ndarray:
    result = np.eye(4)
    axis_element = joint.find("axis")
    axis = np.array([1.0, 0.0, 0.0])
    if axis_element is not None:
        axis = np.fromstring(axis_element.attrib.get("xyz", "1 0 0"), sep=" ")
    axis /= np.linalg.norm(axis)
    joint_type = joint.attrib["type"]
    if joint_type in {"revolute", "continuous"}:
        result[:3, :3] = Rotation.from_rotvec(axis * value).as_matrix()
    elif joint_type == "prismatic":
        result[:3, 3] = axis * value
    return result


def urdf_link_transforms(urdf: Path, joint_values: dict[str, float]) -> tuple[ET.Element, dict[str, np.ndarray]]:
    root = ET.parse(urdf).getroot()
    children: dict[str, list[ET.Element]] = {}
    child_links = set()
    for joint in root.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        children.setdefault(parent, []).append(joint)
        child_links.add(child)
    root_link = next(link.attrib["name"] for link in root.findall("link") if link.attrib["name"] not in child_links)
    poses = {root_link: np.eye(4)}
    stack = [root_link]
    while stack:
        parent = stack.pop()
        for joint in children.get(parent, []):
            child = joint.find("child").attrib["link"]
            origin_element = joint.find("origin")
            origin = transform(
                origin_element.attrib.get("xyz") if origin_element is not None else None,
                origin_element.attrib.get("rpy") if origin_element is not None else None,
            )
            poses[child] = poses[parent] @ origin @ joint_motion(
                joint, joint_values.get(joint.attrib["name"], 0.0)
            )
            stack.append(child)
    return root, poses


def chassis_front_x(urdf: Path, joint_values: dict[str, float]) -> float:
    import trimesh

    root, poses = urdf_link_transforms(urdf, joint_values)
    maximum = -math.inf
    for link in root.findall("link"):
        name = link.attrib["name"]
        if name not in CHASSIS_LINKS:
            continue
        for collision in link.findall("collision"):
            mesh_element = collision.find("geometry/mesh")
            if mesh_element is None:
                continue
            mesh = trimesh.load(mesh_element.attrib["filename"], force="mesh", process=False)
            origin_element = collision.find("origin")
            origin = transform(
                origin_element.attrib.get("xyz") if origin_element is not None else None,
                origin_element.attrib.get("rpy") if origin_element is not None else None,
            )
            vertices = np.column_stack((np.asarray(mesh.vertices), np.ones(len(mesh.vertices))))
            world = (poses[name] @ origin @ vertices.T).T
            maximum = max(maximum, float(world[:, 0].max()))
    if not math.isfinite(maximum):
        raise RuntimeError("could not compute chassis front")
    return maximum


def derive_robot_config(source: Path, destination: Path, home: dict[str, float]) -> None:
    import yaml

    data = yaml.safe_load(source.read_text())
    kinematics = data.get("robot_cfg", data)["kinematics"]
    kinematics.pop("load_collision_spheres", None)
    kinematics.pop("num_envs", None)
    kinematics["lock_joints"] = copy.deepcopy(LOCKED_JOINTS)
    collision_spheres = kinematics.get("collision_spheres", {})
    kinematics["collision_spheres"] = {
        name: spheres
        for name, spheres in collision_spheres.items()
        if name in PLANNING_COLLISION_LINKS
    }
    kinematics["collision_link_names"] = [
        name
        for name in kinematics.get("collision_link_names", [])
        if name in PLANNING_COLLISION_LINKS
    ]
    kinematics["mesh_link_names"] = [
        name
        for name in kinematics.get("mesh_link_names", [])
        if name in PLANNING_COLLISION_LINKS
    ]
    self_ignore = kinematics.get("self_collision_ignore", {})
    kinematics["self_collision_ignore"] = {
        name: [other for other in others if other in PLANNING_COLLISION_LINKS]
        for name, others in self_ignore.items()
        if name in PLANNING_COLLISION_LINKS
    }
    self_buffer = kinematics.get("self_collision_buffer", {})
    kinematics["self_collision_buffer"] = {
        name: value
        for name, value in self_buffer.items()
        if name in PLANNING_COLLISION_LINKS
    }
    cspace = kinematics["cspace"]
    source_names = cspace["joint_names"]
    indices = [source_names.index(name) for name in ACTIVE_JOINTS]
    for key, value in list(cspace.items()):
        if isinstance(value, list) and len(value) == len(source_names):
            cspace[key] = [value[index] for index in indices]
    cspace["joint_names"] = ACTIVE_JOINTS
    cspace["default_joint_position"] = [home[name] for name in ACTIVE_JOINTS]
    destination.write_text(yaml.safe_dump(data, sort_keys=False))


def align_tool_z(reference_quaternion_wxyz: np.ndarray, target: np.ndarray) -> np.ndarray:
    reference_xyzw = np.roll(reference_quaternion_wxyz, -1)
    reference = Rotation.from_quat(reference_xyzw)
    source = reference.as_matrix()[:, 2]
    target = target / np.linalg.norm(target)
    cross = np.cross(source, target)
    dot = float(np.clip(np.dot(source, target), -1.0, 1.0))
    if np.linalg.norm(cross) < 1e-10:
        alignment = Rotation.identity() if dot > 0.0 else Rotation.from_rotvec(np.array([0.0, math.pi, 0.0]))
    else:
        alignment = Rotation.from_rotvec(cross / np.linalg.norm(cross) * math.acos(dot))
    result_xyzw = (alignment * reference).as_quat()
    return np.roll(result_xyzw, 1)


def canonical_side_suction_quaternion_wxyz(side: str) -> np.ndarray:
    return np.array([0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5)])


def canonical_side_tool_to_box(side: str) -> np.ndarray:
    transform = np.eye(4)
    contact_rotation = Rotation.from_quat(
        np.roll(canonical_side_suction_quaternion_wxyz(side), -1)
    ).as_matrix()
    transform[:3, :3] = contact_rotation.T
    transform[2, 3] = 0.15
    return transform


def wall_center(front_x: float, wall_distance: float, box_id: int) -> np.ndarray:
    return np.array([
        front_x + wall_distance + 0.15,
        (box_id % 5 - 2) * 0.41,
        0.20 + (box_id // 5) * 0.41,
    ])


def contact_position(
    front_x: float, wall_distance: float, box_id: int, top: bool
) -> np.ndarray:
    center = wall_center(front_x, wall_distance, box_id)
    if top:
        center[2] += 0.200001
    else:
        center[0] -= 0.150001
    return center


def make_scene(front_x: float, wall_distance: float, excluded: set[int]) -> Scene:
    from curobo.scene import Cuboid, Scene

    wall_back = front_x + wall_distance + 0.30 + 1e-6
    cuboids = [
        Cuboid(name="ground", pose=[wall_back - 2.0, 0.0, -0.05, 1, 0, 0, 0], dims=[4.2, 2.6, 0.1]),
        Cuboid(name="left_wall", pose=[wall_back - 2.0, -1.25, 1.2, 1, 0, 0, 0], dims=[4.0, 0.1, 2.4]),
        Cuboid(name="right_wall", pose=[wall_back - 2.0, 1.25, 1.2, 1, 0, 0, 0], dims=[4.0, 0.1, 2.4]),
        Cuboid(name="front_wall", pose=[wall_back + 0.05, 0.0, 1.2, 1, 0, 0, 0], dims=[0.1, 2.6, 2.4]),
        Cuboid(name="ceiling", pose=[wall_back - 2.0, 0.0, 2.45, 1, 0, 0, 0], dims=[4.2, 2.6, 0.1]),
    ]
    for box_id in range(25):
        if box_id in excluded:
            continue
        center = wall_center(front_x, wall_distance, box_id)
        cuboids.append(Cuboid(
            name=f"wall_box_{box_id:02d}",
            pose=[*center.tolist(), 1, 0, 0, 0],
            dims=[0.30, 0.40, 0.40],
        ))
    return Scene(cuboid=cuboids)


def goal_for_round(
    round_pair: tuple[int | None, int | None],
    top: bool,
    home_positions: dict[str, np.ndarray],
    target_quaternions: dict[str, np.ndarray],
    front_x: float,
    wall_distance: float,
) -> GoalToolPose:
    import torch
    from curobo.types import GoalToolPose, Pose

    poses = {}
    for side, box_id in zip(("left", "right"), round_pair):
        frame = f"{side}_tool0"
        position = home_positions[frame] if box_id is None else contact_position(
            front_x, wall_distance, box_id, top
        )
        mode = "top" if top else "side"
        quaternion = home_positions[f"{frame}_quat"] if box_id is None else target_quaternions[frame][mode]
        poses[frame] = Pose(
            position=torch.tensor(position, device="cuda", dtype=torch.float32).reshape(1, 3),
            quaternion=torch.tensor(quaternion, device="cuda", dtype=torch.float32).reshape(1, 4),
        )
    return GoalToolPose.from_poses(poses, ordered_tool_frames=["left_tool0", "right_tool0"])


def solve_one(ik: InverseKinematics, goal: GoalToolPose, current_state: JointState) -> dict:
    from curobo.types import JointState

    started = time.perf_counter()
    try:
        result = ik.solve_pose(goal, current_state=current_state, return_seeds=8)
    except Exception as error:
        return {
            "success": False,
            "wall_ms": (time.perf_counter() - started) * 1000.0,
            "failure": f"{type(error).__name__}: {error}",
        }
    wall_ms = (time.perf_counter() - started) * 1000.0
    success = bool(result.success.reshape(-1)[0].item())
    output = {
        "success": success,
        "wall_ms": wall_ms,
        "solve_ms": float(result.solve_time * 1000.0),
        "position_error_mm": float(result.position_error.reshape(-1)[0].item() * 1000.0),
        "rotation_error_deg": math.degrees(float(result.rotation_error.reshape(-1)[0].item())),
    }
    if success:
        solution = result.js_solution.position.reshape(-1, len(ik.joint_names))[0]
        output["joint_names"] = list(ik.joint_names)
        output["joints"] = [float(value) for value in solution.detach().cpu()]
        fk = ik.compute_kinematics(JointState.from_position(solution.reshape(1, -1), joint_names=ik.joint_names))
        output["fk"] = {
            frame: {
                "position": [float(value) for value in pose.position.reshape(-1, 3)[0].detach().cpu()],
                "quaternion_wxyz": [float(value) for value in pose.quaternion.reshape(-1, 4)[0].detach().cpu()],
            }
            for frame, pose in fk.tool_poses.to_dict().items()
        }
    return output


def main() -> None:
    import torch
    import yaml
    from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
    from curobo.types import JointState

    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-config", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--named-poses", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--wall-distance", type=float, default=0.9)
    parser.add_argument("--num-seeds", type=int, default=128)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    named = yaml.safe_load(args.named_poses.read_text())["named_poses"]
    home = named["home"]
    pose_families = {name: named[name] for name in ("home", "second_home")}
    derived_config = args.output_dir / "alfa_v3_suction_ik_15d.yml"
    derive_robot_config(args.robot_config, derived_config, home)
    front_x = chassis_front_x(args.urdf, home)

    robot_data = yaml.safe_load(derived_config.read_text())
    kinematic_robot = copy.deepcopy(robot_data)
    kinematic_cfg = InverseKinematicsCfg.create(
        robot=kinematic_robot,
        num_seeds=args.num_seeds,
        self_collision_check=False,
        load_collision_spheres=False,
        use_cuda_graph=False,
        position_tolerance=0.002,
        orientation_tolerance=math.radians(1.0),
        override_iters_for_multi_link_ik=300,
    )
    kinematic_ik = InverseKinematics(kinematic_cfg)
    home_states = {
        family: JointState.from_position(
            torch.tensor(
                [[values[name] for name in ACTIVE_JOINTS]],
                device="cuda",
                dtype=torch.float32,
            ),
            joint_names=ACTIVE_JOINTS,
        )
        for family, values in pose_families.items()
    }
    home_poses_by_family = {}
    target_quaternions = {}
    for family, state in home_states.items():
        poses = {}
        home_fk = kinematic_ik.compute_kinematics(state).tool_poses.to_dict()
        for frame, pose in home_fk.items():
            position = pose.position.reshape(-1, 3)[0].detach().cpu().numpy()
            quaternion = pose.quaternion.reshape(-1, 4)[0].detach().cpu().numpy()
            poses[frame] = position
            poses[f"{frame}_quat"] = quaternion
            if family == "home":
                target_quaternions[frame] = {
                    "side": canonical_side_suction_quaternion_wxyz(frame.removesuffix("_tool0")),
                    "top": align_tool_z(quaternion, np.array([0.0, 0.0, -1.0])),
                }
        home_poses_by_family[family] = poses

    collision_cfg = InverseKinematicsCfg.create(
        robot=robot_data,
        scene_model=make_scene(front_x, args.wall_distance, {24, 20}),
        collision_cache={"cuboid": 40},
        num_seeds=args.num_seeds,
        self_collision_check=True,
        use_cuda_graph=False,
        position_tolerance=0.002,
        orientation_tolerance=math.radians(1.0),
        optimizer_collision_activation_distance=0.005,
        override_iters_for_multi_link_ik=300,
    )
    collision_ik = InverseKinematics(collision_cfg)

    records = []
    for index, (left_box, right_box, top) in enumerate(ROUNDS, start=1):
        pair = (left_box, right_box)
        attempts = []
        for family in ("home", "second_home"):
            goal = goal_for_round(
                pair,
                top,
                home_poses_by_family[family],
                target_quaternions,
                front_x,
                args.wall_distance,
            )
            kinematic_attempt = solve_one(
                kinematic_ik, goal, home_states[family]
            )
            collision_ik.update_world(make_scene(
                front_x, args.wall_distance, {box for box in pair if box is not None}
            ))
            collision_attempt = solve_one(
                collision_ik, goal, home_states[family]
            )
            attempts.append({
                "initial_pose": family,
                "kinematic": kinematic_attempt,
                "collision_free": collision_attempt,
            })
        selected = next(
            (attempt for attempt in attempts if attempt["collision_free"]["success"]),
            next(
                (attempt for attempt in attempts if attempt["kinematic"]["success"]),
                attempts[0],
            ),
        )
        kinematic = selected["kinematic"]
        collision = selected["collision_free"]
        record = {
            "round": index,
            "left_box": pair[0],
            "right_box": pair[1],
            "suction_mode": "top" if top else "side",
            "initial_pose": selected["initial_pose"],
            "attempts": attempts,
            "kinematic": kinematic,
            "collision_free": collision,
        }
        records.append(record)
        print(
            f"round={index:02d} pair={pair} "
            f"initial={selected['initial_pose']} "
            f"kinematic={kinematic['success']} collision={collision['success']} "
            f"kin_ms={kinematic['wall_ms']:.2f} coll_ms={collision['wall_ms']:.2f}"
        )

    summary = {
        "kind": "v3_curobo_v2_wall_ik_benchmark",
        "wall_distance_m": args.wall_distance,
        "chassis_front_x_m": front_x,
        "robot_config": str(derived_config),
        "num_seeds": args.num_seeds,
        "active_joints": ACTIVE_JOINTS,
        "locked_joints": LOCKED_JOINTS,
        "kinematic_successes": sum(
            any(attempt["kinematic"]["success"] for attempt in item["attempts"])
            for item in records
        ),
        "collision_free_successes": sum(
            any(attempt["collision_free"]["success"] for attempt in item["attempts"])
            for item in records
        ),
        "collision_model_warning": (
            "Preview collision spheres under-cover model_base and arm_carriage; "
            "collision-free results are diagnostic only."
        ),
        "rounds": records,
    }
    output = args.output_dir / "summary.json"
    output.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({key: summary[key] for key in (
        "wall_distance_m", "chassis_front_x_m", "kinematic_successes",
        "collision_free_successes", "collision_model_warning"
    )}, indent=2))


if __name__ == "__main__":
    main()
