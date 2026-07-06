#!/usr/bin/env python3
"""Visualize reachable point-cloud edges from nine-orientation IK CSV.

Green points are all reachable points. The six axis extrema among reachable
points are highlighted with distinct colors and labels: x-/x+/y-/y+/z-/z+.
"""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rerun as rr

from visualize_rerun import UrdfRobot, log_robot_static_model, log_robot_state, render_current_urdf


EDGE_COLORS: dict[str, list[int]] = {
    "x_min": [255, 0, 0],
    "x_max": [255, 128, 0],
    "y_min": [0, 80, 255],
    "y_max": [0, 220, 255],
    "z_min": [180, 0, 255],
    "z_max": [255, 0, 180],
}

EDGE_LABELS: dict[str, str] = {
    "x_min": "x- min",
    "x_max": "x+ max",
    "y_min": "y- min",
    "y_max": "y+ max",
    "z_min": "z- min",
    "z_max": "z+ max",
}


@dataclass(frozen=True)
class PointStats:
    side: str
    x: float
    y: float
    z: float
    n_success: int
    n_total: int

    @property
    def position(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)


def load_reachable_points(csv_path: Path, side: str) -> list[PointStats]:
    grouped: dict[tuple[str, float, float, float], list[int]] = defaultdict(lambda: [0, 0])
    with csv_path.open(newline="") as file:
        reader = csv.DictReader(file)
        for row in reader:
            if side != "all" and row.get("side") != side:
                continue
            if row.get("orient_label") == "SUMMARY":
                continue
            key = (
                row.get("side", ""),
                round(float(row["x"]), 6),
                round(float(row["y"]), 6),
                round(float(row["z"]), 6),
            )
            grouped[key][1] += 1
            if row.get("success") == "1":
                grouped[key][0] += 1

    points: list[PointStats] = []
    for (point_side, x, y, z), (n_success, n_total) in grouped.items():
        if n_success <= 0:
            continue
        points.append(PointStats(point_side, x, y, z, n_success, n_total))
    return points


def edge_points(points: list[PointStats]) -> dict[str, list[PointStats]]:
    if not points:
        return {}
    values = {
        "x_min": min(p.x for p in points),
        "x_max": max(p.x for p in points),
        "y_min": min(p.y for p in points),
        "y_max": max(p.y for p in points),
        "z_min": min(p.z for p in points),
        "z_max": max(p.z for p in points),
    }
    return {
        name: [p for p in points if abs(getattr(p, axis_name(name)) - value) < 1e-9]
        for name, value in values.items()
    }


def axis_name(edge_name: str) -> str:
    return edge_name[0]


def log_text_label(path: str, point: PointStats, label: str, color: list[int]) -> None:
    rr.log(
        path,
        rr.Points3D(
            positions=[point.position],
            colors=[color],
            radii=0.035,
            labels=[label],
        ),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize reachable point-cloud axis extrema")
    parser.add_argument(
        "csv_path",
        nargs="?",
        default="ros2_ws/nine_orient_reachability_current_nocollisions.csv",
        help="Reachability CSV path",
    )
    parser.add_argument("--side", default="left", choices=["left", "right", "all"], help="Side to visualize")
    parser.add_argument("--save", default="", help="Save Rerun recording to .rrd")
    parser.add_argument("--connect", action="store_true", help="Connect to an existing Rerun viewer")
    parser.add_argument("--point-radius", type=float, default=0.01, help="Radius for regular reachable points")
    parser.add_argument("--no-robot", action="store_true", help="Do not show the robot model")
    parser.add_argument("--robot-path", default="world/robot", help="Rerun path for the robot model")
    parser.add_argument("--no-meshes", action="store_true", help="Only show robot link transforms, not STL meshes")
    args = parser.parse_args()

    csv_path = Path(args.csv_path)
    points = load_reachable_points(csv_path, args.side)
    if not points:
        raise SystemExit(f"No reachable points found in {csv_path} for side={args.side}")

    edges = edge_points(points)
    positions = np.array([p.position for p in points], dtype=np.float32)
    colors = np.tile(np.array([[0, 180, 80]], dtype=np.uint8), (len(points), 1))

    rr.init("reachability_edges", recording_id=csv_path.stem)
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

    rr.log(
        "world/reachable_points",
        rr.Points3D(positions=positions, colors=colors, radii=args.point_radius),
    )

    summary_lines = [
        f"csv: {csv_path}",
        f"side: {args.side}",
        f"reachable_points: {len(points)}",
    ]
    for edge_name, edge_group in edges.items():
        color = EDGE_COLORS[edge_name]
        label = EDGE_LABELS[edge_name]
        edge_positions = np.array([p.position for p in edge_group], dtype=np.float32)
        rr.log(
            f"world/edges/{edge_name}",
            rr.Points3D(
                positions=edge_positions,
                colors=np.tile(np.array([color], dtype=np.uint8), (len(edge_group), 1)),
                radii=0.035,
            ),
        )

        representative = edge_group[0]
        value = getattr(representative, axis_name(edge_name))
        summary_lines.append(f"{label}: {value:.3f}, points={len(edge_group)}")
        log_text_label(
            f"world/edge_labels/{edge_name}",
            representative,
            f"{label}\n{representative.position}\npoints={len(edge_group)}",
            color,
        )

    rr.log("summary", rr.TextLog("\n".join(summary_lines)))

    print("\n".join(summary_lines))
    if args.save:
        print(f"saved: {args.save}")


if __name__ == "__main__":
    main()
