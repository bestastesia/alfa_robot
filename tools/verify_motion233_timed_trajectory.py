#!/usr/bin/env python3
"""Verify a saved cuRobo cycle or sequence has a complete bounded time trajectory."""

import argparse
import json
import statistics
from pathlib import Path

import numpy as np
import yaml
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--mobile-robot-config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    document = json.loads(args.result.read_text())
    result = document.get("results", [document])[-1]
    config = yaml.safe_load(args.mobile_robot_config.read_text())
    cspace = config.get("robot_cfg", config)["kinematics"]["cspace"]
    joint_names = cspace["joint_names"]
    urdf = ET.parse(config.get("robot_cfg", config)["kinematics"]["urdf_path"]).getroot()
    velocity_by_name = {
        joint.attrib["name"]: float(joint.find("limit").attrib["velocity"])
        for joint in urdf.findall("joint") if joint.find("limit") is not None
    }
    limits = {
        "velocities": np.asarray([velocity_by_name[name] for name in joint_names], dtype=float),
        "accelerations": np.asarray(cspace["max_acceleration"], dtype=float),
        "jerks": np.asarray(cspace["max_jerk"], dtype=float),
    }
    if not result.get("success"):
        raise SystemExit(f"planning result failed: {result.get('error')}")
    if result.get("joint_names") != joint_names:
        raise SystemExit("joint order does not match the mobile robot configuration")

    frame_count = len(result["frames"])
    time_values = np.asarray(result.get("time_from_start_s", ()), dtype=float)
    if len(time_values) != frame_count or np.any(np.diff(time_values) < -1e-12):
        raise SystemExit("time_from_start_s is missing, misaligned or moves backwards")
    ratios = {}
    for field, limit in limits.items():
        values = np.asarray(result.get(field, ()), dtype=float)
        if values.shape != (frame_count, len(joint_names)) or not np.isfinite(values).all():
            raise SystemExit(f"{field} is missing, non-finite or misaligned")
        ratios[field] = float(np.max(np.abs(values) / limit))
        if ratios[field] > 1.0001:
            raise SystemExit(f"{field} exceeds configured limits: {ratios[field]:.6f}")

    rounds = result.get("rounds") or [{"result": result, "status": "completed"}]
    if any(record.get("status") != "completed" for record in rounds):
        raise SystemExit("not every sequence round completed")
    if any(not record["result"].get("timed_validation", {}).get("success") for record in rounds):
        raise SystemExit("a cycle timed-validation gate failed")
    event_indices = [index for index, phase in enumerate(result["phases"])
                     if phase in ("home", "attach", "release")]
    velocities = np.asarray(result["velocities"])
    accelerations = np.asarray(result["accelerations"])
    if event_indices and (np.max(np.abs(velocities[event_indices])) > 1e-7 or
                          np.max(np.abs(accelerations[event_indices])) > 1e-7):
        raise SystemExit("lifecycle event frames are not stationary")

    optimizations = []
    for record in rounds:
        cycle = record["result"]
        selected = next((attempt.get("approach", {}) for attempt in cycle.get("attempts", ())
                         if attempt.get("candidate") == cycle.get("selected_candidate")), {})
        for name, stats in (("approach", selected),
                            ("transport", cycle.get("transport_search_stats", {})),
                            ("return_home", cycle.get("home_search_stats", {}))):
            optimization = stats.get("trajectory_optimization")
            if optimization:
                optimizations.append((name, optimization))
    optimization_summary = {}
    for name in ("approach", "transport", "return_home"):
        entries = [item for segment, item in optimizations if segment == name]
        if entries:
            walls = [float(item.get("wall_ms", 0.0)) for item in entries]
            optimization_summary[name] = {
                "attempts": len(entries), "accepted": sum(bool(item.get("accepted")) for item in entries),
                "wall_sum_s": sum(walls) / 1000.0, "wall_median_ms": statistics.median(walls),
                "wall_max_ms": max(walls),
            }

    summary = {
        "success": True,
        "sequence": bool(result.get("sequence")),
        "rounds": len(rounds),
        "completed_box_ids": result.get("completed_box_ids"),
        "frames": frame_count,
        "motion_duration_s": float(time_values[-1]),
        "planning_wall_s": float(result.get("total_ms", 0.0)) / 1000.0,
        "maximum_limit_ratios": ratios,
        "event_frames_stationary": True,
        "trajopt_wall_s": sum(float(item.get("wall_ms", 0.0)) for _, item in optimizations) / 1000.0,
        "trajopt_segments": optimization_summary,
        "dynamics_enabled": False,
        "torque_constraints_enabled": False,
        "source": str(args.result),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
