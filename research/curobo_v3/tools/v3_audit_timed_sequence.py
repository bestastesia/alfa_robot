#!/usr/bin/env python3
"""Audit timing consistency, stops, fallbacks and configured limits in a saved sequence."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from curobo_core.contracts import PlannerAssets


def true_runs(mask):
    edges = np.diff(np.pad(np.asarray(mask, dtype=np.int8), (1, 1)))
    return list(zip(np.flatnonzero(edges == 1).tolist(), (np.flatnonzero(edges == -1) - 1).tolist()))


def main():
    defaults = PlannerAssets()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--mobile-robot-config", type=Path, default=defaults.mobile_robot_config)
    parser.add_argument("--sample-dt", type=float, default=.025)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    document = json.loads(args.input.read_text())
    config = yaml.safe_load(args.mobile_robot_config.read_text())
    kinematics = config.get("robot_cfg", config)["kinematics"]
    names = list(kinematics["cspace"]["joint_names"])
    urdf = ET.parse(kinematics["urdf_path"]).getroot()
    joints = {joint.attrib["name"]: joint for joint in urdf.findall("joint")}
    limits = {
        "velocity": dict(zip(names, (float(joints[name].find("limit").attrib["velocity"])
                                     for name in names))),
        "acceleration": dict(zip(names, kinematics["cspace"]["max_acceleration"])),
        "jerk": dict(zip(names, kinematics["cspace"]["max_jerk"])),
    }

    hard_failures, warnings, segments, boundaries = [], [], [], []
    frame_count = len(document.get("frames", []))
    for field in ("phases", "payload", "frame_rounds", "time_from_start_s",
                  "velocities", "accelerations", "jerks"):
        if len(document.get(field, [])) != frame_count:
            hard_failures.append(f"{field} length does not match frames")

    time_values = np.asarray(document["time_from_start_s"], dtype=np.float64)
    positions = np.asarray(document["frames"], dtype=np.float64)
    dt = np.diff(time_values)
    positive_dt = dt[dt > 1e-12]
    if np.any(dt < -1e-12):
        hard_failures.append("global time moves backwards")
    if len(positive_dt) and positive_dt.max() > args.sample_dt * 1.05:
        hard_failures.append(f"maximum sample gap {positive_dt.max():.6f}s exceeds configured cadence")
    zero_time = np.flatnonzero(np.abs(dt) <= 1e-12)
    max_zero_time_motion = max((float(np.max(np.abs(positions[index + 1] - positions[index])))
                                for index in zero_time), default=0.0)
    if max_zero_time_motion > 1e-10:
        hard_failures.append("joint motion occurs across a zero-duration edge")

    phase_durations = {}
    fallback_reasons = {}
    structural_stop_candidates = 0
    for round_number, record in enumerate(document["rounds"], 1):
        result = record["result"]
        validation = result.get("timed_validation", {})
        if not validation.get("success"):
            hard_failures.append(f"round {round_number} timed validation failed")
        stats_by_name = {
            "approach": result["attempts"][result["selected_candidate"] - 1]["approach"],
            "transport": result["transport_search_stats"],
            "return_home": result["home_search_stats"],
        }
        timed = sorted(result["timed_segments"], key=lambda item: item["frame_start"])
        for left, right in zip(timed, timed[1:]):
            left_map = dict(zip(left["joint_names"], left["velocities"][-1]))
            right_map = dict(zip(right["joint_names"], right["velocities"][0]))
            common = set(left_map) & set(right_map)
            jump = max((abs(right_map[name] - left_map[name]) / limits["velocity"][name]
                        for name in common), default=0.0)
            boundaries.append({"round": round_number, "left": left["name"], "right": right["name"],
                               "velocity_jump_ratio": jump})
            if jump > .02:
                hard_failures.append(
                    f"round {round_number} {left['name']}->{right['name']} velocity discontinuity {jump:.3f}")
            if (left["name"], right["name"]) in {
                    ("extract", "transport"), ("transport", "turn_loaded"),
                    ("turn_empty", "return_home")}:
                structural_stop_candidates += 1

        for segment in timed:
            joint_names = segment["joint_names"]
            values = {field: np.asarray(segment[field], dtype=np.float64)
                      for field in ("positions", "velocities", "accelerations", "jerks")}
            local_time = np.asarray(segment["time_from_start_s"], dtype=np.float64)
            ratio_arrays = []
            peaks = {}
            for field, key in (("velocities", "velocity"), ("accelerations", "acceleration"),
                               ("jerks", "jerk")):
                limit = np.asarray([limits[key][name] for name in joint_names])
                ratio = np.abs(values[field]) / limit
                ratio_arrays.append(ratio)
                index = np.unravel_index(np.argmax(ratio), ratio.shape)
                peaks[key] = {"ratio": float(ratio[index]), "joint": joint_names[index[1]],
                              "sample": int(index[0])}
                if ratio[index] > 1.0001:
                    hard_failures.append(
                        f"round {round_number} {segment['name']} exceeds {key} limit")
            speed = np.max(ratio_arrays[0], axis=1)
            zero_runs = true_runs(speed < 1e-5)
            internal_stops = [run for run in zero_runs if run[0] > 0 and run[1] < len(speed) - 1]
            stationary = np.all(np.abs(np.diff(values["positions"], axis=0)) < 1e-10, axis=1)
            dwell = max((float(local_time[end + 1] - local_time[start])
                         for start, end in true_runs(stationary)), default=0.0)
            stats = stats_by_name.get(segment["name"], {})
            optimization = stats.get("trajectory_optimization", {})
            reason = optimization.get("fallback_reason")
            if reason:
                fallback_reasons[reason] = fallback_reasons.get(reason, 0) + 1
                if reason == "optimized samples could not be aligned to cycle frames":
                    hard_failures.append(f"round {round_number} {segment['name']} alignment fallback")
            expected_internal = max(0, int(stats.get("shortcut_waypoints", 2)) - 2)
            if not optimization.get("accepted") and len(internal_stops) > expected_internal:
                hard_failures.append(
                    f"round {round_number} {segment['name']} has excess internal stops")
            duration = float(local_time[-1])
            phase_durations[segment["name"]] = phase_durations.get(segment["name"], 0.0) + duration
            segments.append({
                "round": round_number, "name": segment["name"], "duration_s": duration,
                "frames": len(local_time), "internal_stops": len(internal_stops),
                "expected_internal_stops": expected_internal, "max_stationary_dwell_s": dwell,
                "peaks": peaks, "optimization_accepted": optimization.get("accepted"),
                "fallback_reason": reason, "raw_waypoints": stats.get("raw_waypoints"),
                "shortcut_waypoints": stats.get("shortcut_waypoints"),
            })

    max_ratios = {key: max(segment["peaks"][key]["ratio"] for segment in segments)
                  for key in ("velocity", "acceleration", "jerk")}
    for key, ratio in max_ratios.items():
        if ratio > .9:
            warnings.append(f"peak {key} utilization is {ratio:.1%}")
    if structural_stop_candidates:
        warnings.append(f"{structural_stop_candidates} phase boundaries are rest-to-rest merge candidates")
    internal_stop_count = sum(segment["internal_stops"] for segment in segments)
    if internal_stop_count:
        warnings.append(f"{internal_stop_count} conservative internal RRT waypoint stops remain")

    output = {
        "success": not hard_failures,
        "input": str(args.input),
        "summary": {
            "rounds": len(document["rounds"]), "frames": frame_count,
            "duration_s": document.get("motion_duration_s"), "segment_count": len(segments),
            "positive_sample_dt_min_s": float(positive_dt.min()) if len(positive_dt) else None,
            "positive_sample_dt_max_s": float(positive_dt.max()) if len(positive_dt) else None,
            "zero_duration_edges": len(zero_time), "max_zero_duration_motion": max_zero_time_motion,
            "max_limit_ratios": max_ratios, "internal_stop_count": internal_stop_count,
            "structural_stop_candidates": structural_stop_candidates,
        },
        "hard_failures": hard_failures, "warnings": warnings,
        "phase_durations_s": phase_durations, "fallback_reasons": fallback_reasons,
        "segments": segments, "boundaries": boundaries,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"success": output["success"], **output["summary"],
                      "hard_failures": hard_failures, "warnings": warnings},
                     ensure_ascii=False, indent=2))
    raise SystemExit(0 if output["success"] else 1)


if __name__ == "__main__":
    main()
