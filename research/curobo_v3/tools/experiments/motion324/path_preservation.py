#!/usr/bin/env python3
"""MOTION-324: check that TOPP-RA timing keeps the Cartesian transport path.

For each cached dev path, the timed samples are compared with the dense geometric path in tool
space: the largest distance to the path polyline and the largest orientation difference to the
nearest path sample, plus the joint-space spline-to-polyline deviation reported by the parameterizer.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from curobo_core.contracts import PlannerAssets
from v3_cartesian_transport_planner import CartesianTransportPlanner
from v3_path_timing import time_parameterize_toppra
from v3_timed_wall_sequence import WALL_SEQUENCE
from v3_wall_ik_benchmark import ACTIVE_JOINTS


def tool_poses(planner, rows, side):
    poses = []
    for row in rows:
        planner.fk(row)
        poses.append(planner.fk_robot.get_transform(f"{side}_tool0", planner.fk_robot.base_link).copy())
    return np.asarray(poses)


def segment_distance(points, polyline):
    """Distance from each point to the polyline, checking the two segments around its nearest vertex."""
    nearest = np.linalg.norm(points[:, None] - polyline[None], axis=2).argmin(axis=1)
    best = np.full(len(points), np.inf)
    for offset in (-1, 0):
        start = np.clip(nearest + offset, 0, len(polyline) - 2)
        a, b = polyline[start], polyline[start + 1]
        span = b - a
        fraction = np.clip(np.einsum("ij,ij->i", points - a, span) / np.maximum(np.einsum("ij,ij->i", span, span), 1e-18), 0, 1)
        best = np.minimum(best, np.linalg.norm(points - (a + fraction[:, None] * span), axis=1))
    return best, nearest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--paths", type=Path, required=True, help="cartesian-paths.npz from timing_prototype.py")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    planner = CartesianTransportPlanner(PlannerAssets(), transport_search_mode="cartesian", transport_timing="toppra")
    data = np.load(args.paths)
    names = planner._timed_joint_names
    index = [names.index(name) for name in ACTIVE_JOINTS]
    report = []
    for boxes in [task for task in WALL_SEQUENCE if len(task) == 1]:
        side, box = next(iter(boxes.items()))
        name = f"{side}{box:02d}"
        path, template = data[f"{name}/path"], data[f"{name}/template"]
        rows = np.repeat(template[None], len(path), axis=0)
        rows[:, index] = path
        trajectory, validation = time_parameterize_toppra(rows, names, *planner._timed_limits,
                                                          sample_dt=planner.trajectory_sample_dt)
        planner.configure_task(boxes)
        reference = tool_poses(planner, path, side)
        timed = tool_poses(planner, np.asarray(validation, dtype=np.float64)[:, index], side)
        distance, nearest = segment_distance(timed[:, :3, 3], reference[:, :3, 3])
        angle = (Rotation.from_matrix(timed[:, :3, :3]).inv() * Rotation.from_matrix(reference[nearest, :3, :3])).magnitude()
        entry = dict(task=name, duration_s=trajectory["duration_s"], samples=len(timed), path_rows=len(path),
                     max_tool_deviation_mm=float(distance.max() * 1000),
                     max_orientation_deviation_deg=float(np.degrees(angle.max())),
                     joint_spline_vs_polyline=trajectory["maximum_linear_deviation"])
        report.append(entry)
        print(json.dumps(entry), flush=True)
    args.output.write_text(json.dumps(report, indent=1) + "\n")


if __name__ == "__main__":
    main()
