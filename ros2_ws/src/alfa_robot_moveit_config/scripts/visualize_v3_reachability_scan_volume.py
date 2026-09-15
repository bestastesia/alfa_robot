#!/usr/bin/python3
"""Preview the active V3.1.1 reachability scan volume without running IK."""

from __future__ import annotations

import argparse
from pathlib import Path
import warnings

import numpy as np
import rerun as rr
from rerun.error_utils import RerunWarning

from alfa_robot_rerun.visualize_rerun import (
    UrdfRobot,
    log_robot_state,
    log_robot_static_model,
    render_current_urdf,
)


DEFAULT_JOINT1 = np.array([0.181, -0.33054221, 1.3032993])
GRAY = [145, 150, 160, 145]
BLUE = [55, 145, 245, 240]
AXIS_COLORS = [[235, 70, 70, 240], [70, 210, 95, 240], [70, 130, 245, 240]]


def inclusive_axis(start: float, stop: float, step: float) -> np.ndarray:
    count = int(round((stop - start) / step)) + 1
    return start + np.arange(count, dtype=np.float64) * step


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="预览 V3.1.1 左臂连续可达性测试将采用的三维点云范围"
    )
    parser.add_argument(
        "--save",
        default=(
            "/mnt/mydisk/ALFA/alfa_robot_v3/data/ik_benchmark/"
            "v3_0_9_scan_volume_preview/left_joint1_centered_scan_volume_preview.rrd"
        ),
    )
    parser.add_argument("--spawn", action="store_true")
    parser.add_argument("--x-min", type=float, default=0.15)
    parser.add_argument("--x-max", type=float, default=1.35)
    parser.add_argument("--x-step", type=float, default=0.01)
    parser.add_argument("--y-min", type=float, default=-1.18054221)
    parser.add_argument("--y-max", type=float, default=0.51945779)
    parser.add_argument("--y-step", type=float, default=0.05)
    parser.add_argument("--z-min", type=float, default=0.3032993)
    parser.add_argument("--z-max", type=float, default=2.3032993)
    parser.add_argument("--z-step", type=float, default=0.05)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    warnings.filterwarnings("ignore", category=RerunWarning)
    axes = (
        inclusive_axis(args.x_min, args.x_max, args.x_step),
        inclusive_axis(args.y_min, args.y_max, args.y_step),
        inclusive_axis(args.z_min, args.z_max, args.z_step),
    )
    x_grid, y_grid, z_grid = np.meshgrid(*axes, indexing="ij")
    points = np.column_stack((x_grid.ravel(), y_grid.ravel(), z_grid.ravel()))

    output_path = Path(args.save).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    rr.init("v3_0_9_reachability_scan_volume", spawn=args.spawn)
    rr.save(str(output_path))

    robot = UrdfRobot(render_current_urdf())
    log_robot_static_model(
        robot, "world/robot", log_meshes=True, log_view_coordinates=False
    )
    log_robot_state(robot, {}, "world/robot")

    center = np.array(
        [
            (args.x_min + args.x_max) * 0.5,
            (args.y_min + args.y_max) * 0.5,
            (args.z_min + args.z_max) * 0.5,
        ]
    )
    size = np.array(
        [args.x_max - args.x_min, args.y_max - args.y_min, args.z_max - args.z_min]
    )
    rr.log(
        "world/planned_scan/points",
        rr.Points3D(points, colors=[GRAY], radii=0.004),
        static=True,
    )
    rr.log(
        "world/planned_scan/bounds",
        rr.Boxes3D(
            centers=[center],
            sizes=[size],
            colors=[[95, 150, 220, 90]],
            labels=["planned scan volume"],
            show_labels=True,
        ),
        static=True,
    )
    rr.log(
        "world/planned_scan/left_joint1",
        rr.Points3D(
            [DEFAULT_JOINT1],
            colors=[BLUE],
            radii=0.022,
            labels=["left_joint1"],
            show_labels=True,
        ),
        static=True,
    )
    rr.log(
        "world/planned_scan/world_axes",
        rr.Arrows3D(
            origins=[[0.0, 0.0, 0.0]] * 3,
            vectors=[[0.25, 0.0, 0.0], [0.0, 0.25, 0.0], [0.0, 0.0, 0.25]],
            colors=AXIS_COLORS,
            radii=0.008,
            labels=["+X", "+Y", "+Z"],
            show_labels=True,
        ),
        static=True,
    )
    rr.log(
        "summary",
        rr.TextLog(
            f"V3.1.1 planned scan volume: {len(points)} points; "
            f"X=[{args.x_min:.3f},{args.x_max:.3f}]m, "
            f"Y=[{args.y_min:.3f},{args.y_max:.3f}]m, "
            f"Z=[{args.z_min:.3f},{args.z_max:.3f}]m. Gray means untested."
        ),
        static=True,
    )

    print(f"Rerun: {output_path}")
    print(f"points={len(points)} shape={tuple(len(axis) for axis in axes)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
