"""Build a paired v2b/B2 smoke report from same-environment result JSON files."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from v3_candidate_ranker import ACTIVE_JOINTS, Kinematics


def read_pair(spec):
    mode, task, b1_path, b2_path = spec.split("=", 3)
    b1, b2 = json.loads(Path(b1_path).read_text()), json.loads(Path(b2_path).read_text())
    if b1["instance"] != task or b2["instance"] != task:
        raise ValueError("pair task differs from result instance")
    b2_plan = sum(item["wall_ms"] for item in b2.get("outcomes", []))
    b2_last = b2.get("outcomes", [{}])[-1].get("info", {})
    row = {
        "mode": mode, "instance": task, "v2b_success": bool(b1.get("success")), "b2_success": bool(b2.get("success")),
        "v2b_selection_ms": b1.get("selection_total_ms"), "b2_selection_ms": b2.get("selection_ms"),
        "v2b_planning_ms": b1.get("planning_wall_ms"), "b2_planning_ms": b2_plan,
        "v2b_total_wall_ms": (b1.get("selection_total_ms") or 0.) + (b1.get("planning_wall_ms") or 0.),
        "b2_total_wall_ms": (b2.get("selection_ms") or 0.) + b2_plan,
        "v2b_executed_t_lb_s": b1.get("executed_t_path_s"), "b2_executed_t_lb_s": b2_last.get("executed_t_path_s"),
        "v2b_motion_duration_s": b1.get("motion_duration_s"), "b2_motion_duration_s": b2.get("motion_duration_s"),
        "v2b_acceleration_ratio": b1.get("report", {}).get("timed_validation", {}).get("acceleration_limit_ratio"),
        "b2_acceleration_ratio": b2.get("timed_validation", {}).get("acceleration_limit_ratio"),
        "v2b_jerk_ratio": b1.get("report", {}).get("timed_validation", {}).get("jerk_limit_ratio"),
        "b2_jerk_ratio": b2.get("timed_validation", {}).get("jerk_limit_ratio"),
        "b2_selection_budget_exceeded": bool(b2.get("selection_budget_exceeded")),
        "b2_indices": "/".join(map(str, b2.get("choice", {}).get("indices", []))),
    }
    for metric in ("selection_ms", "planning_ms", "total_wall_ms", "executed_t_lb_s", "motion_duration_s",
                   "acceleration_ratio", "jerk_ratio"):
        row["delta_" + metric] = row["b2_" + metric] - row["v2b_" + metric]
    return row


def trajectory_metrics(path, kinematics):
    document = np.load(path)
    names = document["joint_names"].astype(str).tolist()
    indices = [names.index(name) for name in ACTIVE_JOINTS]
    positions = np.asarray(document["frames"], dtype=np.float64)[:, indices]
    delta = np.diff(positions, axis=0) / (kinematics.upper - kinematics.lower)
    reversals = 0
    for joint in range(delta.shape[1]):
        movement = delta[:, joint]
        signs = np.sign(movement[np.abs(movement) > 1e-5])
        reversals += int(np.sum(signs[1:] != signs[:-1])) if len(signs) > 1 else 0
    return {"normalized_joint_variation": float(np.abs(delta).sum()), "joint_direction_reversals": reversals}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pair", action="append", required=True, help="MODE=INSTANCE=B1_JSON=B2_JSON")
    parser.add_argument("--trajectory-pair", action="append", required=True,
                        help="MODE=INSTANCE=B1_NPZ=B2_NPZ")
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    rows = [read_pair(spec) for spec in args.pair]
    kinematics = Kinematics(args.urdf)
    trajectories = {}
    for spec in args.trajectory_pair:
        mode, task, b1_path, b2_path = spec.split("=", 3)
        trajectories[mode, task] = (trajectory_metrics(b1_path, kinematics), trajectory_metrics(b2_path, kinematics))
    for row in rows:
        if (row["mode"], row["instance"]) not in trajectories:
            raise ValueError("missing trajectory pair for result pair")
        left, right = trajectories[row["mode"], row["instance"]]
        for metric in left:
            row["v2b_" + metric] = left[metric]
            row["b2_" + metric] = right[metric]
            row["delta_" + metric] = right[metric] - left[metric]
    delta_metrics = [key.removeprefix("delta_") for key in rows[0] if key.startswith("delta_")]
    summary = {
        "schema": "motion386-b2-paired-smoke-v1",
        "pairs": len(rows),
        "both_success": sum(row["v2b_success"] and row["b2_success"] for row in rows),
        "b2_selection_budget_exceeded": sum(row["b2_selection_budget_exceeded"] for row in rows),
        "median_delta_b2_minus_v2b": {metric: float(np.median([row["delta_" + metric] for row in rows]))
                                       for metric in delta_metrics},
        "b2_better_count": {metric: sum(row["delta_" + metric] < 0. for row in rows) for metric in delta_metrics},
        "decision": "do not promote or expand yet: group selection is feasible, but endpoint smoothness does not consistently improve timed jerk",
        "rows": rows,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "report.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    with (args.output_dir / "pairs.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    print(json.dumps({key: summary[key] for key in ("schema", "pairs", "both_success",
                                                     "b2_selection_budget_exceeded", "decision")}, ensure_ascii=False))


if __name__ == "__main__":
    main()
