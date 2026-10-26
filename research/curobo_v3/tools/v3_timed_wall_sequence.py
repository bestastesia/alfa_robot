#!/usr/bin/env python3
"""Plan the established 15-round/25-box mentor sequence and add timed trajectories."""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from curobo_core.contracts import PlannerAssets
from v3_timed_single_arm_planner import TimedSingleArmPlanner


WALL_SEQUENCE = (
    {"left": 24, "right": 20}, {"left": 23, "right": 21}, {"right": 22},
    {"left": 19, "right": 15}, {"left": 18, "right": 16}, {"right": 17},
    {"left": 14, "right": 10}, {"left": 13, "right": 11}, {"right": 12},
    {"left": 9, "right": 5}, {"left": 8, "right": 2}, {"right": 7},
    {"left": 4, "right": 0}, {"left": 3, "right": 1}, {"right": 6},
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--trajopt-interpolation-dt", type=float, default=.025)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    assets = PlannerAssets()
    planner = TimedSingleArmPlanner(assets, args.trajopt_interpolation_dt)
    planner.warmup_resources()
    report = {"success": False, "sequence": True, "rounds": [], "frames": [],
              "phases": [], "payload": [], "frame_rounds": [],
              "joint_names": list(planner.mobile_robot["kinematics"]["cspace"]["joint_names"]),
              "time_from_start_s": [], "velocities": [], "accelerations": [], "jerks": [],
              "completed_box_ids": []}
    began = time.perf_counter()
    for number, boxes in enumerate(WALL_SEQUENCE):
        print(f"整墙第{number+1}/15轮 {boxes}", flush=True)
        planner.configure_task(boxes)
        result = planner.full_cycle(boxes, lambda text: print(f"第{number+1}/15轮 · {text}", flush=True))
        path = args.output_dir / f"round-{number+1:02d}.json"
        path.write_text(json.dumps(result, ensure_ascii=False) + "\n")
        record = {"task": boxes, "status": "completed" if result.get("success") else "failed",
                  "result": result, "path": str(path)}
        report["rounds"].append(record)
        if not result.get("success"):
            report["error"] = result.get("error") or {
                "stage": result.get("blocked_stage"), "message": result.get("blocked_details")}
            report["blocked_round"] = number + 1
            break
        if report["frames"] and not np.allclose(report["frames"][-1], result["frames"][0], atol=1e-6, rtol=0):
            report.setdefault("boundary_navigation_resets", []).append({
                "before_round": number + 1,
                "maximum_joint_delta": float(np.max(np.abs(
                    np.asarray(report["frames"][-1]) - np.asarray(result["frames"][0])))),
                "policy": "mentor navigation boundary; excluded from arm motion time",
            })
        offset = report["time_from_start_s"][-1] if report["time_from_start_s"] else 0.0
        for key in ("frames", "phases", "payload", "velocities", "accelerations", "jerks"):
            report[key].extend(result[key])
        report["time_from_start_s"].extend(offset + value for value in result["time_from_start_s"])
        report["frame_rounds"].extend([number] * len(result["frames"]))
        report["completed_box_ids"].extend(boxes.values())
    else:
        report["success"] = True
        report["error"] = None
    report["remaining_box_ids"] = sorted(set(range(25)) - set(report["completed_box_ids"]))
    report["motion_duration_s"] = report["time_from_start_s"][-1] if report["time_from_start_s"] else 0.0
    report["total_ms"] = (time.perf_counter() - began) * 1000
    output = args.output_dir / "full-wall.json"
    output.write_text(json.dumps(report, ensure_ascii=False) + "\n")
    print(json.dumps({key: report.get(key) for key in
                      ("success", "completed_box_ids", "remaining_box_ids", "total_ms", "motion_duration_s", "error")},
                     ensure_ascii=False), flush=True)
    raise SystemExit(0 if report["success"] else 1)


if __name__ == "__main__":
    main()
