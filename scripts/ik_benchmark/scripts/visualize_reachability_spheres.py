#!/usr/bin/env python3
"""Visualize fitted arm reachability spheres with the current robot model."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import rerun as rr

from visualize_rerun import UrdfRobot, log_robot_static_model, log_robot_state, render_current_urdf


def parse_vec3(text: str) -> list[float]:
    values = [float(part.strip()) for part in text.split(",")]
    if len(values) != 3:
        raise argparse.ArgumentTypeError("expected x,y,z")
    return values


def log_sphere(path: str, center: list[float], radius: float, color: list[int], label: str) -> None:
    rr.log(
        path,
        rr.Ellipsoids3D(
            centers=[center],
            radii=[radius],
            colors=[color],
            fill_mode=rr.components.FillMode.TransparentFillMajorWireframe,
            labels=[label],
            show_labels=True,
        ),
    )
    rr.log(
        f"{path}/center",
        rr.Points3D(
            positions=[center],
            colors=[color[:3]],
            radii=0.035,
            labels=[f"{label}\ncenter={tuple(round(v, 3) for v in center)}\nr={radius:.3f}"],
        ),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize left/right reachability spheres in Rerun")
    parser.add_argument("--left-center", type=parse_vec3, default=[-0.065, 0.2, 1.025], help="Left sphere center x,y,z")
    parser.add_argument("--right-center", type=parse_vec3, default=[-0.065, -0.2, 1.025], help="Right sphere center x,y,z")
    parser.add_argument("--radius", type=float, default=0.815, help="Sphere radius")
    parser.add_argument("--save", default="", help="Save Rerun recording to .rrd")
    parser.add_argument("--connect", action="store_true", help="Connect to existing Rerun viewer")
    parser.add_argument("--no-robot", action="store_true", help="Do not show robot model")
    parser.add_argument("--robot-path", default="world/robot", help="Rerun robot path")
    parser.add_argument("--no-meshes", action="store_true", help="Only show robot link transforms")
    args = parser.parse_args()

    rr.init("reachability_spheres", recording_id="current_reachability_spheres")
    if args.save:
        rr.save(args.save)
    elif args.connect:
        rr.connect()
    else:
        rr.spawn()

    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    if not args.no_robot:
        robot = UrdfRobot(render_current_urdf())
        log_robot_static_model(robot, args.robot_path, log_meshes=not args.no_meshes)
        log_robot_state(robot, {}, args.robot_path)

    log_sphere("world/reachability/left_sphere", args.left_center, args.radius, [0, 180, 255, 90], "left reach sphere")
    log_sphere("world/reachability/right_sphere", args.right_center, args.radius, [255, 120, 0, 90], "right reach sphere")

    centers = np.array([args.left_center, args.right_center], dtype=np.float32)
    rr.log(
        "world/reachability/center_line",
        rr.LineStrips3D(
            strips=[centers.tolist()],
            colors=[[220, 220, 220]],
            radii=0.008,
        ),
    )
    rr.log(
        "summary",
        rr.TextLog(
            "\n".join([
                f"left_center={args.left_center}",
                f"right_center={args.right_center}",
                f"radius={args.radius}",
                "right sphere is y-mirrored from left by default",
            ])
        ),
    )

    print(f"left_center={args.left_center}")
    print(f"right_center={args.right_center}")
    print(f"radius={args.radius}")
    if args.save:
        print(f"saved: {args.save}")


if __name__ == "__main__":
    main()
