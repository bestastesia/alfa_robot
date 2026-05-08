#!/usr/bin/env python3
"""Sample and visualize a DH-model reachable workspace.

This is an offline kinematics sanity-check tool. It does not depend on ROS.
It can use Robotics Toolbox when installed, and falls back to a small built-in
standard/modified DH FK implementation otherwise. It can also read URDF-style
origin/axis chains for exact FK of exported URDF joints, and filter samples by a
single end-effector axis direction, which checks fixed-facing-direction
reachability without constraining full roll/pitch/yaw.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml


def load_config(path: Path) -> dict[str, Any]:
    with path.open() as config_file:
        return yaml.safe_load(config_file)


def dh_transform(theta: float, d: float, a: float, alpha: float, convention: str) -> np.ndarray:
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)

    if convention == "standard":
        return np.array(
            [
                [ct, -st * ca, st * sa, a * ct],
                [st, ct * ca, -ct * sa, a * st],
                [0.0, sa, ca, d],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
    if convention == "modified":
        return np.array(
            [
                [ct, -st, 0.0, a],
                [st * ca, ct * ca, -sa, -d * sa],
                [st * sa, ct * sa, ca, d * ca],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
    raise ValueError(f"unsupported DH convention: {convention}")


def rpy_matrix(rpy: list[float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation_x = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    rotation_y = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rotation_z = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rotation_z @ rotation_y @ rotation_x


def homogeneous(rotation: np.ndarray | None = None, translation: list[float] | np.ndarray | None = None) -> np.ndarray:
    transform = np.eye(4)
    if rotation is not None:
        transform[:3, :3] = rotation
    if translation is not None:
        transform[:3, 3] = np.asarray(translation, dtype=float)
    return transform


def axis_angle_matrix(axis: list[float], angle: float) -> np.ndarray:
    axis_array = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis_array)
    if norm == 0.0:
        raise ValueError("joint axis must be non-zero")
    x, y, z = axis_array / norm
    c, s = math.cos(angle), math.sin(angle)
    one_minus_c = 1.0 - c
    return np.array(
        [
            [c + x * x * one_minus_c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s],
            [y * x * one_minus_c + z * s, c + y * y * one_minus_c, y * z * one_minus_c - x * s],
            [z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, c + z * z * one_minus_c],
        ],
        dtype=float,
    )


def urdf_origin_axis_transform(link: dict[str, Any], joint_value: float) -> np.ndarray:
    origin = link.get("origin", {})
    origin_transform = homogeneous(
        rotation=rpy_matrix([float(value) for value in origin.get("rpy", [0.0, 0.0, 0.0])]),
        translation=[float(value) for value in origin.get("xyz", [0.0, 0.0, 0.0])],
    )

    joint_type = link.get("type", "revolute")
    axis = [float(value) for value in link.get("axis", [0.0, 0.0, 1.0])]
    if joint_type in {"revolute", "continuous"}:
        motion = homogeneous(rotation=axis_angle_matrix(axis, float(joint_value)))
    elif joint_type == "prismatic":
        axis_array = np.asarray(axis, dtype=float)
        norm = np.linalg.norm(axis_array)
        if norm == 0.0:
            raise ValueError("joint axis must be non-zero")
        motion = homogeneous(translation=(axis_array / norm) * float(joint_value))
    elif joint_type == "fixed":
        motion = np.eye(4)
    else:
        raise ValueError(f"unsupported joint type: {joint_type}")
    return origin_transform @ motion


def sample_joint_values(config: dict[str, Any], samples: int, seed: int | None) -> np.ndarray:
    rng = np.random.default_rng(seed)
    columns = []
    for link in config["links"]:
        lower, upper = link["qlim"]
        columns.append(rng.uniform(float(lower), float(upper), samples))
    return np.column_stack(columns)


def fk_builtin_transforms(config: dict[str, Any], q_values: np.ndarray) -> np.ndarray:
    convention = config.get("convention", "standard")
    transforms = np.empty((q_values.shape[0], 4, 4), dtype=float)

    for row_index, q_row in enumerate(q_values):
        transform = np.eye(4)
        for joint_value, link in zip(q_row, config["links"]):
            if convention == "urdf_origin_axis":
                transform = transform @ urdf_origin_axis_transform(link, float(joint_value))
                continue

            joint_type = link.get("type", "revolute")
            theta_offset = float(link.get("theta", 0.0))
            d_offset = float(link.get("d", 0.0))
            if joint_type in {"revolute", "continuous"}:
                theta = theta_offset + float(joint_value)
                d = d_offset
            elif joint_type == "prismatic":
                theta = theta_offset
                d = d_offset + float(joint_value)
            else:
                raise ValueError(f"unsupported joint type: {joint_type}")

            transform = transform @ dh_transform(
                theta=theta,
                d=d,
                a=float(link.get("a", 0.0)),
                alpha=float(link.get("alpha", 0.0)),
                convention=convention,
            )
        transforms[row_index] = transform
    return transforms


def fk_builtin(config: dict[str, Any], q_values: np.ndarray) -> np.ndarray:
    transforms = fk_builtin_transforms(config, q_values)
    return transforms[:, :3, 3]


def axis_vector(axis_name: str) -> np.ndarray:
    axis_name = axis_name.strip().lower()
    sign = -1.0 if axis_name.startswith("-") else 1.0
    base_name = axis_name[1:] if axis_name.startswith("-") else axis_name
    axes = {
        "x": np.array([1.0, 0.0, 0.0]),
        "y": np.array([0.0, 1.0, 0.0]),
        "z": np.array([0.0, 0.0, 1.0]),
    }
    if base_name not in axes:
        raise ValueError(f"unsupported axis '{axis_name}', expected x/y/z or -x/-y/-z")
    return sign * axes[base_name]


def filter_axis_alignment(
    config: dict[str, Any],
    q_values: np.ndarray,
    ee_axis: str,
    target_axis: str,
    angle_deg: float,
    allow_opposite: bool,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    transforms = fk_builtin_transforms(config, q_values)
    rotations = transforms[:, :3, :3]
    local_axis = axis_vector(ee_axis)
    target = axis_vector(target_axis)

    world_axes = rotations @ local_axis
    dot_products = np.clip(world_axes @ target, -1.0, 1.0)
    if allow_opposite:
        dot_products = np.abs(dot_products)

    alignment_angles = np.degrees(np.arccos(dot_products))
    keep_mask = alignment_angles <= angle_deg
    return transforms[keep_mask, :3, 3], q_values[keep_mask], alignment_angles[keep_mask]


def fk_roboticstoolbox(config: dict[str, Any], q_values: np.ndarray) -> np.ndarray:
    import roboticstoolbox as rtb

    if config.get("convention", "standard") != "standard":
        raise ValueError("Robotics Toolbox path currently supports standard DH configs only")

    rtb_links = []
    for link in config["links"]:
        joint_type = link.get("type", "revolute")
        if joint_type in {"revolute", "continuous"}:
            rtb_links.append(
                rtb.RevoluteDH(
                    d=float(link.get("d", 0.0)),
                    a=float(link.get("a", 0.0)),
                    alpha=float(link.get("alpha", 0.0)),
                    qlim=link.get("qlim"),
                )
            )
        elif joint_type == "prismatic":
            rtb_links.append(
                rtb.PrismaticDH(
                    theta=float(link.get("theta", 0.0)),
                    a=float(link.get("a", 0.0)),
                    alpha=float(link.get("alpha", 0.0)),
                    qlim=link.get("qlim"),
                )
            )
        else:
            raise ValueError(f"unsupported joint type: {joint_type}")

    robot = rtb.DHRobot(rtb_links, name=config.get("name", "DH Robot"))
    print(robot)
    transforms = robot.fkine(q_values)
    return np.asarray(transforms.t)


def save_csv(
    path: Path,
    points: np.ndarray,
    q_values: np.ndarray,
    joint_names: list[str],
    alignment_angles: np.ndarray | None = None,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as csv_file:
        fieldnames = ["x", "y", "z", *joint_names]
        if alignment_angles is not None:
            fieldnames.append("alignment_angle_deg")
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for row_index, (point, q_row) in enumerate(zip(points, q_values)):
            row = {"x": point[0], "y": point[1], "z": point[2]}
            row.update({name: value for name, value in zip(joint_names, q_row)})
            if alignment_angles is not None:
                row["alignment_angle_deg"] = alignment_angles[row_index]
            writer.writerow(row)


def visualize_matplotlib(points: np.ndarray) -> None:
    import matplotlib.pyplot as plt

    if points.size == 0:
        print("No points to visualize.")
        return

    fig = plt.figure()
    axis = fig.add_subplot(111, projection="3d")
    axis.scatter(points[:, 0], points[:, 1], points[:, 2], c="b", s=0.5)
    axis.set_xlabel("X")
    axis.set_ylabel("Y")
    axis.set_zlabel("Z")
    axis.set_title("Reachable Workspace")
    axis.set_box_aspect(np.maximum(np.ptp(points, axis=0), 1e-6))
    plt.show()


def visualize_open3d(points: np.ndarray, point_size: float) -> None:
    import open3d as o3d

    if points.size == 0:
        print("No points to visualize.")
        return

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.paint_uniform_color([0.0, 0.45, 1.0])
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.25, origin=[0, 0, 0])

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="DH Reachable Workspace")
    vis.add_geometry(cloud)
    vis.add_geometry(axes)
    render_option = vis.get_render_option()
    render_option.point_size = point_size
    render_option.background_color = np.asarray([0.02, 0.02, 0.02])
    vis.run()
    vis.destroy_window()


def main() -> None:
    default_config = Path(__file__).resolve().parent / "configs" / "example_4dof.yaml"
    parser = argparse.ArgumentParser(description="Sample a DH workspace and visualize/save points")
    parser.add_argument("--config", type=Path, default=default_config)
    parser.add_argument("--samples", type=int, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--backend", choices=["auto", "rtb", "builtin"], default="auto")
    parser.add_argument("--visualizer", choices=["matplotlib", "open3d", "none"], default="matplotlib")
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--point-size", type=float, default=3.0)
    parser.add_argument(
        "--require-axis-alignment",
        action="store_true",
        help="keep only samples whose selected end-effector local axis matches the target direction",
    )
    parser.add_argument(
        "--align-ee-axis",
        default="z",
        help="end-effector local axis to align: x/y/z or -x/-y/-z (default: z)",
    )
    parser.add_argument(
        "--align-target",
        default="z",
        help="target direction in world/base frame: x/y/z or -x/-y/-z (default: z)",
    )
    parser.add_argument(
        "--align-angle-deg",
        type=float,
        default=5.0,
        help="maximum allowed axis alignment angle in degrees (default: 5)",
    )
    parser.add_argument(
        "--allow-opposite",
        action="store_true",
        help="treat anti-parallel axes as valid too",
    )
    args = parser.parse_args()

    config = load_config(args.config)
    samples = args.samples or int(config.get("sampling", {}).get("samples", 30000))
    seed = args.seed if args.seed is not None else config.get("sampling", {}).get("seed")
    joint_names = [link.get("name", f"q{index + 1}") for index, link in enumerate(config["links"])]

    q_values = sample_joint_values(config, samples=samples, seed=seed)
    print(f"Sampling {samples} joint states for {config.get('name', 'DH Robot')}...")

    alignment_angles = None
    if args.backend in {"auto", "rtb"} and not args.require_axis_alignment:
        try:
            points = fk_roboticstoolbox(config, q_values)
        except Exception as exc:
            if args.backend == "rtb":
                raise
            print(f"Robotics Toolbox unavailable/failed ({exc}); using built-in FK.")
            points = fk_builtin(config, q_values)
    else:
        points = fk_builtin(config, q_values)

    if args.require_axis_alignment:
        points, q_values, alignment_angles = filter_axis_alignment(
            config=config,
            q_values=q_values,
            ee_axis=args.align_ee_axis,
            target_axis=args.align_target,
            angle_deg=args.align_angle_deg,
            allow_opposite=args.allow_opposite,
        )
        print(
            "Axis alignment filter: "
            f"EE {args.align_ee_axis} -> target {args.align_target}, "
            f"tolerance {args.align_angle_deg:g} deg"
        )
        print(f"  kept: {len(points)} / {samples} samples")

    if points.size:
        print("Workspace bounds:")
        print(f"  min xyz: {points.min(axis=0)}")
        print(f"  max xyz: {points.max(axis=0)}")
    else:
        print("Workspace bounds: no samples passed the current filters")

    output = args.output or Path(config.get("output", {}).get("csv", "dh_workspace_points.csv"))
    save_csv(output, points, q_values, joint_names, alignment_angles=alignment_angles)
    print(f"Saved CSV: {output}")

    if args.visualizer == "matplotlib":
        visualize_matplotlib(points)
    elif args.visualizer == "open3d":
        visualize_open3d(points, point_size=args.point_size)


if __name__ == "__main__":
    main()
