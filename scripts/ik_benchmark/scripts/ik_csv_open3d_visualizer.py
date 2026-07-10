#!/usr/bin/env python3
"""Visualize IK grid CSV results as an Open3D point cloud.

CSV columns expected: x,y,z,is_success,time_ms,error_code
"""

import argparse
import csv
from pathlib import Path

import numpy as np


def parse_bool(value: str) -> bool:
    return str(value).strip().lower() in {"true", "1", "yes", "y"}


def load_points(csv_path: Path, show_failed: bool, max_points: int | None):
    success_points = []
    failed_points = []
    success_times = []

    with csv_path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        required = {"x", "y", "z", "is_success", "time_ms", "error_code"}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"CSV missing columns: {sorted(missing)}")

        for row in reader:
            point = [float(row["x"]), float(row["y"]), float(row["z"])]
            if parse_bool(row["is_success"]):
                success_points.append(point)
                success_times.append(float(row["time_ms"]))
            elif show_failed:
                failed_points.append(point)

    def downsample(points):
        if max_points is None or len(points) <= max_points:
            return np.asarray(points, dtype=np.float64)
        indices = np.linspace(0, len(points) - 1, max_points).astype(int)
        return np.asarray(points, dtype=np.float64)[indices]

    return downsample(success_points), downsample(failed_points), np.asarray(success_times)


def make_cloud(points: np.ndarray, color):
    import open3d as o3d

    cloud = o3d.geometry.PointCloud()
    if len(points) > 0:
        cloud.points = o3d.utility.Vector3dVector(points)
        cloud.paint_uniform_color(color)
    return cloud


def main():
    parser = argparse.ArgumentParser(description="Visualize IK CSV as Open3D point cloud")
    parser.add_argument("csv", type=Path, help="IK result CSV path")
    parser.add_argument("--show-failed", action="store_true", help="Also show failed points in red")
    parser.add_argument("--max-points", type=int, default=None, help="Downsample each class for display")
    parser.add_argument("--point-size", type=float, default=4.0, help="Open3D point size")
    args = parser.parse_args()

    success_points, failed_points, success_times = load_points(args.csv, args.show_failed, args.max_points)
    total_display = len(success_points) + len(failed_points)

    print(f"CSV: {args.csv}")
    print(f"successful display points: {len(success_points)}")
    if args.show_failed:
        print(f"failed display points: {len(failed_points)}")
    if len(success_times) > 0:
        print(f"success solve time ms: avg={success_times.mean():.3f}, min={success_times.min():.3f}, max={success_times.max():.3f}")
    if total_display == 0:
        print("No points to display.")
        return

    import open3d as o3d

    geometries = []
    geometries.append(make_cloud(success_points, [0.0, 0.85, 0.15]))
    if args.show_failed:
        geometries.append(make_cloud(failed_points, [0.9, 0.05, 0.05]))

    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.25, origin=[0, 0, 0])
    geometries.append(axes)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=f"IK reachable points: {args.csv.name}")
    for geometry in geometries:
        vis.add_geometry(geometry)
    render_option = vis.get_render_option()
    render_option.point_size = args.point_size
    render_option.background_color = np.asarray([0.02, 0.02, 0.02])
    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()
