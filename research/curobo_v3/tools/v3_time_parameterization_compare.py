#!/usr/bin/env python3
"""Compare Mentor retiming and cuRobo TrajOpt on the same stage; simulation only."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from curobo_core.contracts import ACTIVE_JOINTS, PlannerAssets
from curobo_core.rolling_backend import RollingBackend
from curobo_core.rolling_curobo import SegmentReplanner
from curobo_core.trajectory import TimedTrajectory, time_parameterize_path
from v3_rolling_replan import stage_input


def stats(values):
    values = np.asarray(values, dtype=float)
    return {"count": len(values), "median": float(np.median(values)),
            "p95": float(np.percentile(values, 95)), "min": float(values.min()),
            "max": float(values.max()), "samples": values.tolist()}


def weighted_arc(positions, weights):
    return float(np.linalg.norm(np.diff(np.asarray(positions), axis=0) * weights, axis=1).sum())


def nearest_path_distance(left, right, weights, block=128):
    left, right = np.asarray(left), np.asarray(right)
    nearest = []
    for start in range(0, len(left), block):
        delta = (left[start:start + block, None] - right[None]) * weights
        nearest.extend(np.linalg.norm(delta, axis=2).min(axis=1).tolist())
    return {"mean": float(np.mean(nearest)), "max": float(np.max(nearest))}


def tool_paths(backend, positions):
    paths = {"left": [], "right": []}
    for row in positions:
        backend.fk(row)
        for side in paths:
            paths[side].append(backend.fk_robot.get_transform(
                f"{side}_tool0", backend.fk_robot.base_link)[:3, 3].copy())
    return {side: np.asarray(points) for side, points in paths.items()}


def trajectory_metrics(document, validation, reference, weights, backend, reference_tools):
    curve = TimedTrajectory(document)
    positions = np.asarray(document["positions"])
    tools = tool_paths(backend, positions)
    return {
        "duration_s": curve.duration,
        "frames": len(curve.times),
        "weighted_joint_arc": weighted_arc(positions, weights),
        "directed_distance_to_reference": nearest_path_distance(positions, reference, weights),
        "tool_paths": {side: {
            "length_m": weighted_arc(points, np.ones(3)),
            "directed_distance_to_reference_m": nearest_path_distance(
                points, reference_tools[side], np.ones(3)),
        } for side, points in tools.items()},
        "limit_ratios": validation["limit_ratios"],
        "collision_samples": validation["collision_samples"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--initial-cycle", type=Path, nargs="+", required=True)
    parser.add_argument("--stage", choices=("approach", "transport", "conveyor_place", "return_home"),
                        default="approach")
    parser.add_argument("--repeats", type=int, default=10)
    parser.add_argument("--cut-fraction", type=float, default=.125)
    parser.add_argument("--seed", type=int, default=20261003)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.repeats < 1 or not 0 < args.cut_fraction < 1:
        parser.error("positive repeats and 0 < cut-fraction < 1 required")

    records = [json.loads(path.read_text()) for path in args.initial_cycle]
    initial, snapshot, boxes, payload = stage_input(records[0], args.stage)
    for record in records[1:]:
        other, other_snapshot, other_boxes, other_payload = stage_input(record, args.stage)
        if (other_boxes != boxes or other_payload != payload or other_snapshot.geometry_key != snapshot.geometry_key or
                any(not np.array_equal(np.asarray(other[key]), np.asarray(initial[key]))
                    for key in ("time_from_start_s", "positions", "velocities", "accelerations", "jerks"))):
            raise ValueError("comparison inputs must contain the exact same stage trajectory and scene geometry")
    reference = np.asarray(initial["positions"], dtype=float)
    curve = TimedTrajectory(initial)
    mentor_phase_times = []
    for record in records:
        result = record["result"] if "result" in record else record["results"][-1]
        detail = result.get("trajectory_timing_breakdown_ms", {}).get("phases", {}).get(args.stage)
        if detail is None:
            raise ValueError("initial cycle lacks trajectory_timing_breakdown_ms; regenerate it")
        mentor_phase_times.append(sum(detail.values()))

    np.random.seed(args.seed)
    backend = RollingBackend(PlannerAssets())
    backend.configure_task(boxes)
    adapter = SegmentReplanner(backend, boxes, payload=payload, phase=args.stage)
    validity = adapter.scene(snapshot)
    weights = validity.weights.detach().cpu().numpy()
    initial_validation = adapter.validate(initial, validity)
    reference_tools = tool_paths(backend, reference)

    # Warm the cached optimizer outside measurements.
    backend.optimize_rrt_path(reference, validity, boxes, payload=payload)
    full_runs, full_document, full_validation = [], None, None
    for _ in range(args.repeats):
        began = time.perf_counter()
        _, telemetry = backend.optimize_rrt_path(reference, validity, boxes, payload=payload)
        generated = time.perf_counter()
        document = telemetry.pop("trajectory", None)
        if document is None:
            raise RuntimeError(f"full-stage TrajOpt failed: {telemetry}")
        validation = adapter.validate(document, validity)
        full_runs.append({"generation_ms": (generated - began) * 1000,
                          "validation_ms": (time.perf_counter() - generated) * 1000,
                          "sdk_solve_ms": telemetry.get("solve_time_ms"),
                          "telemetry": telemetry})
        full_document, full_validation = document, validation

    cut_s = curve.duration * args.cut_fraction
    start_state = curve.sample(cut_s)
    suffix = curve.suffix(cut_s)
    suffix_positions = np.asarray(suffix["positions"])
    rolling_runs, rolling_document, rolling_validation = [], None, None
    for _ in range(args.repeats):
        began = time.perf_counter()
        _, telemetry = backend.optimize_rrt_path(
            suffix_positions, validity, boxes, payload=payload, start_state=start_state)
        generated = time.perf_counter()
        document = telemetry.pop("trajectory", None)
        if document is None:
            raise RuntimeError(f"rolling TrajOpt failed: {telemetry}")
        validation = adapter.validate(document, validity)
        rolling_runs.append({"generation_ms": (generated - began) * 1000,
                             "validation_ms": (time.perf_counter() - generated) * 1000,
                             "sdk_solve_ms": telemetry.get("solve_time_ms"),
                             "splice_velocity_error": telemetry.get("splice_velocities_error"),
                             "splice_acceleration_error": telemetry.get("splice_accelerations_error")})
        rolling_document, rolling_validation = document, validation

    indices = [backend._timed_joint_names.index(name) for name in ACTIVE_JOINTS]
    limits = [np.asarray(values)[indices] for values in backend._timed_limits]
    mentor_suffix, _ = time_parameterize_path(
        suffix_positions, ACTIVE_JOINTS, *limits, sample_dt=adapter.sample_dt)
    mentor_splice_error = {
        key: float(np.max(np.abs(np.asarray(mentor_suffix[key][0]) - start_state[key])))
        for key in ("positions", "velocities", "accelerations")
    }

    report = {
        "schema_version": 1,
        "source": "same_stage_time_parameterization_comparison",
        "hardware_commands": False,
        "stage": args.stage,
        "task_boxes": boxes,
        "input_cycles": [{"path": str(path.resolve()),
                          "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                         for path in args.initial_cycle],
        "methods": {
            "mentor_rest_to_rest": {
                "scope": "geometry-preserving shortcut + rest-to-rest timing + validation",
                "phase_wall_ms": stats(mentor_phase_times),
                "metrics": trajectory_metrics(initial, initial_validation, reference, weights, backend, reference_tools),
                "supports_nonzero_splice": False,
                "nonzero_splice_errors_if_retimed": mentor_splice_error,
            },
            "curobo_trajopt_full_stage": {
                "scope": "seed resampling + geometry/timing TrajOpt + common final validation",
                "generation_ms": stats([row["generation_ms"] for row in full_runs]),
                "validation_ms": stats([row["validation_ms"] for row in full_runs]),
                "total_ms": stats([row["generation_ms"] + row["validation_ms"] for row in full_runs]),
                "sdk_solve_ms": stats([row["sdk_solve_ms"] for row in full_runs]),
                "metrics": trajectory_metrics(full_document, full_validation, reference, weights, backend, reference_tools),
                "supports_nonzero_splice": True,
            },
            "curobo_trajopt_rolling_suffix": {
                "scope": "nonzero q/v/a boundary TrajOpt + common final validation",
                "cut_s": cut_s,
                "generation_ms": stats([row["generation_ms"] for row in rolling_runs]),
                "validation_ms": stats([row["validation_ms"] for row in rolling_runs]),
                "total_ms": stats([row["generation_ms"] + row["validation_ms"] for row in rolling_runs]),
                "metrics": trajectory_metrics(rolling_document, rolling_validation, suffix_positions, weights, backend, tool_paths(backend, suffix_positions)),
                "splice_velocity_error": stats([row["splice_velocity_error"] for row in rolling_runs]),
                "splice_acceleration_error": stats([row["splice_acceleration_error"] for row in rolling_runs]),
            },
        },
        "limitations": [
            "Mentor phase timing is measured from complete fresh baseline runs; TrajOpt repeats reuse warmed GPU resources.",
            "The two methods solve different problems: Mentor preserves a geometric path and assumes rest endpoints; TrajOpt may change geometry and supports nonzero splice derivatives.",
            "This single free-space stage cannot justify replacing constrained contact/extract timing.",
            "Dynamics and torque constraints remain disabled in both methods.",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"success": True, "stage": args.stage,
                      "mentor_ms": report["methods"]["mentor_rest_to_rest"]["phase_wall_ms"],
                      "trajopt_ms": report["methods"]["curobo_trajopt_full_stage"]["total_ms"],
                      "output": str(args.output)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
