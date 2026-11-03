#!/usr/bin/env python3
"""MOTION-324 prototype: compare transport timing methods on the dev single-arm tasks.

Plans each task once in Cartesian mode (paths are cached), then times the Cartesian transport
path with the pipeline cubic and with TOPP-RA at several acceleration scales, and times the
baseline geometric path (the stop waypoints recovered from the frozen timed result) with the
same TOPP-RA settings, segment by segment between stops, as a like-for-like reference.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from curobo_core.contracts import PlannerAssets
from curobo_core.trajectory import time_parameterize_path
from v3_cartesian_transport_planner import CartesianTransportPlanner
from v3_path_timing import time_parameterize_toppra
from v3_timed_wall_sequence import WALL_SEQUENCE
from v3_wall_ik_benchmark import ACTIVE_JOINTS

DEV_SINGLE = [task for task in WALL_SEQUENCE if len(task) == 1]


def cache_paths(planner, cache):
    rows = {}
    for boxes in DEV_SINGLE:
        planner.configure_task(boxes)
        result = planner.full_cycle(boxes)
        if not result.get("success"):
            raise RuntimeError(f"Cartesian cycle failed for {boxes}: {result.get('blocked_stage')}")
        template = np.asarray(result["frames"])[[i for i, phase in enumerate(result["phases"]) if phase == "transport"][0]]
        name = "-".join(f"{side}{box:02d}" for side, box in boxes.items())
        rows[f"{name}/path"], rows[f"{name}/template"] = np.asarray(planner.last_transport_path), template
    np.savez(cache, **rows)


def stop_waypoints(frozen):
    phases = frozen["phases"]
    indices = [i for i, phase in enumerate(phases) if phase == "transport"]
    frames = np.asarray(frozen["frames"])[indices]
    speed = np.abs(np.asarray(frozen["velocities"])[indices]).max(axis=1)
    stops = sorted({0, len(indices) - 1} | {i for i in range(1, len(indices) - 1) if speed[i] < 1e-6})
    return frames[stops]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frozen", type=Path, required=True, help="frozen baseline-wall directory")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scales", default="1.0,0.8,0.65")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    planner = CartesianTransportPlanner(PlannerAssets(), transport_search_mode="cartesian")
    planner.warmup_resources()
    cache = args.output / "cartesian-paths.npz"
    if not cache.exists():
        cache_paths(planner, cache)
    data = np.load(cache)
    names = planner._timed_joint_names
    index = [names.index(name) for name in ACTIVE_JOINTS]
    limits = planner._timed_limits
    scales = [float(item) for item in args.scales.split(",")]
    report = []
    for boxes in DEV_SINGLE:
        name = "-".join(f"{side}{box:02d}" for side, box in boxes.items())
        path, template = data[f"{name}/path"], data[f"{name}/template"]
        rows = np.repeat(template[None], len(path), axis=0)
        rows[:, index] = path
        planner.configure_task(boxes)
        snapshot = planner.snapshot
        entry = dict(task=name, cartesian=dict(
            bound_s=float((np.abs(np.diff(path, axis=0)).sum(axis=0) / limits[0][index]).max()),
            pipeline_s=time_parameterize_path(rows, names, *limits, sample_dt=planner.trajectory_sample_dt)[0]["duration_s"]))
        for scale in scales:
            try:
                trajectory, validation = time_parameterize_toppra(rows, names, *limits, sample_dt=planner.trajectory_sample_dt,
                                                                  acceleration_scale=scale)
                sampled = trajectory["sampled_trajectory"]
                entry["cartesian"][f"toppra_{scale}"] = dict(
                    duration_s=trajectory["duration_s"], valid=planner._timed_phase_valid("transport", validation, boxes, snapshot),
                    **trajectory["toppra"], max_linear_deviation=trajectory["maximum_linear_deviation"],
                    ratios=[float(np.max(np.abs(np.asarray(sampled[field])) / limit))
                            for field, limit in zip(("velocities", "accelerations", "jerks"), limits)])
            except RuntimeError as error:
                entry["cartesian"][f"toppra_{scale}"] = dict(error=str(error))
        frozen = json.loads((args.frozen / f"round-{WALL_SEQUENCE.index(boxes) + 1:02d}.json").read_text())
        stops = stop_waypoints(frozen)
        segment = next(item for item in frozen["timed_segments"] if item["name"] == "transport")
        baseline = dict(pipeline_s=segment["duration_s"], parameterizer=segment["parameterizer"], stop_waypoints=len(stops),
                        bound_s=float((np.abs(np.diff(stops[:, index], axis=0)).sum(axis=0) / limits[0][index]).max()),
                        cubic_per_stop_segment_s=sum(time_parameterize_path(stops[i:i + 2], names, *limits,
                                                                            sample_dt=planner.trajectory_sample_dt)[0]["duration_s"]
                                                     for i in range(len(stops) - 1)))
        for scale in scales:
            try:
                baseline[f"toppra_{scale}"] = sum(time_parameterize_toppra(
                    stops[i:i + 2], names, *limits, sample_dt=planner.trajectory_sample_dt, acceleration_scale=scale)[0]["duration_s"]
                    for i in range(len(stops) - 1))
            except RuntimeError as error:
                baseline[f"toppra_{scale}"] = str(error)
        entry["baseline"] = baseline
        report.append(entry)
        print(json.dumps(entry, ensure_ascii=False, default=float), flush=True)
    (args.output / "timing-prototype.json").write_text(json.dumps(report, ensure_ascii=False, indent=1, default=float) + "\n")


if __name__ == "__main__":
    main()
