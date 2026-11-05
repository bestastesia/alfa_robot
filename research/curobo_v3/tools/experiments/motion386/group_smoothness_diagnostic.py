"""Compare the already-selected three IK endpoints with the B2 kinematic smoothness proxy.

This is deliberately not the complete B2 group score: historical MOTION-386 rows
do not contain the extraction endpoint or collision fractions for every group edge.
"""
import argparse
import csv
from dataclasses import replace
import json
from pathlib import Path

import numpy as np

from motion306_learning.group_score import GroupScoreConfig, score_groups
from v3_candidate_ranker import Kinematics


def percentile(values, q):
    return float(np.percentile(np.asarray(values, dtype=np.float64), q)) if values else None


def selected_group(rows):
    stages = {row["stage"]: row for row in rows if row["stage"] < 3 and row.get("reached")}
    if set(stages) != {0, 1, 2}:
        return None
    points, redundant = [], []
    for stage in range(3):
        row = stages[stage]
        q = [row.get(f"q{i}") for i in range(15)]
        values = [row.get("updown_m"), row.get("left_swivel_deg"), row.get("right_swivel_deg")]
        if any(value is None for value in q + values):
            return None
        points.append(q)
        redundant.append(values)
    return np.asarray(points, dtype=np.float64), np.asarray(redundant, dtype=np.float64)


def build_diagnostic(data, config, kinematics):
    labels = {row["id"]: row["label"] for row in data["functions"]}
    grouped = {}
    for row in data["rows"]:
        grouped.setdefault((row["function_id"], row["instance"]), []).append(row)
    functions = [row["id"] for row in data["functions"]]
    instances = sorted({instance for _, instance in grouped})
    available = {(function, instance): selected_group(grouped.get((function, instance), []))
                 for function in functions for instance in instances}
    common = [instance for instance in instances if all(available[function, instance] is not None
                                                         for function in functions)]
    proxy_config = replace(config, motion_weight=0., collision_penalty_s=0.)
    output = []
    for instance in common:
        points = np.stack([available[function, instance][0] for function in functions])
        redundant = np.stack([available[function, instance][1] for function in functions])
        result = score_groups(points, kinematics.velocity, kinematics.lower, kinematics.upper, redundant,
                              np.zeros((len(functions), 2)), proxy_config)
        smoothness = (result["terms"]["turn_s"] + result["terms"]["reversal_s"] +
                      result["terms"]["redundancy_s"])
        robustness = result["terms"]["joint_limit_s"]
        best_smoothness = smoothness.min()
        smoothness_score = 100. * np.exp(-(smoothness - best_smoothness) / config.score_temperature_s)
        order = np.argsort(smoothness, kind="stable")
        ranks = np.empty(len(functions), dtype=int)
        ranks[order] = np.arange(1, len(functions) + 1)
        for index, function in enumerate(functions):
            episode_rows = grouped[function, instance]
            final = next(row for row in episode_rows if row["stage"] == 3)
            output.append({
                "instance": instance,
                "function_id": function,
                "function_label": labels[function],
                "final_success": bool(final["episode_success"]),
                "smoothness_rank": int(ranks[index]),
                "smoothness_score": float(smoothness_score[index]),
                "smoothness_cost_s": float(smoothness[index]),
                "robustness_cost_s": float(robustness[index]),
                "partial_group_cost_s": float(result["total_s"][index]),
                "turn_fraction": float(result["turn_fraction"][index]),
                "reversal_fraction": float(result["reversal_fraction"][index]),
                "redundancy_curvature": float(result["redundancy_curvature"][index]),
                "joint_limit_margin_fraction": float(result["joint_limit_margin_fraction"][index]),
                "turn_s": float(result["terms"]["turn_s"][index]),
                "reversal_s": float(result["terms"]["reversal_s"][index]),
                "redundancy_s": float(result["terms"]["redundancy_s"][index]),
                "joint_limit_s": float(result["terms"]["joint_limit_s"][index]),
            })
    summary = []
    for function in functions:
        rows = [row for row in output if row["function_id"] == function]
        success = [row for row in rows if row["final_success"]]
        failed = [row for row in rows if not row["final_success"]]
        summary.append({
            "function_id": function,
            "function_label": labels[function],
            "common_tasks": len(rows),
            "final_success_common": len(success),
            "final_success_rate_common": len(success) / len(rows) if rows else None,
            "smoothest_or_tied_count": sum(abs(row["smoothness_score"] - 100.) < 1e-9 for row in rows),
            "smoothness_rank_mean": float(np.mean([row["smoothness_rank"] for row in rows])) if rows else None,
            "smoothness_score_p50": percentile([row["smoothness_score"] for row in rows], 50),
            "smoothness_cost_s_p50": percentile([row["smoothness_cost_s"] for row in rows], 50),
            "robustness_cost_s_p50": percentile([row["robustness_cost_s"] for row in rows], 50),
            "partial_group_cost_s_p50": percentile([row["partial_group_cost_s"] for row in rows], 50),
            "turn_fraction_p50": percentile([row["turn_fraction"] for row in rows], 50),
            "reversal_fraction_p50": percentile([row["reversal_fraction"] for row in rows], 50),
            "redundancy_curvature_p50": percentile([row["redundancy_curvature"] for row in rows], 50),
            "joint_limit_margin_fraction_p50": percentile([row["joint_limit_margin_fraction"] for row in rows], 50),
            "success_smoothness_score_mean": float(np.mean([row["smoothness_score"] for row in success])) if success else None,
            "failure_smoothness_score_mean": float(np.mean([row["smoothness_score"] for row in failed])) if failed else None,
        })
    return {
        "schema": "motion386-b2-selected-triple-diagnostic-v1",
        "scope": "selected contact/carry/place endpoints only; excludes extraction, full chain motion and edge collision",
        "interpretation": "lower smoothness_cost_s and higher within-task smoothness_score mean smoother selected IK endpoints; robustness is reported separately",
        "complete_b2_claim": False,
        "functions": len(functions),
        "input_tasks": len(instances),
        "common_tasks": len(common),
        "excluded_tasks": len(instances) - len(common),
        "config": proxy_config.to_dict(),
        "summary": summary,
        "rows": output,
    }


def write_csv(path, rows):
    if not rows:
        raise ValueError("no diagnostic rows")
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(args.input.read_text())
    report = build_diagnostic(data, GroupScoreConfig.load(args.config), Kinematics(args.urdf))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    write_csv(args.output_dir / "summary.csv", report["summary"])
    write_csv(args.output_dir / "selected_triples.csv", report["rows"])
    print(json.dumps({key: report[key] for key in ("schema", "scope", "functions", "input_tasks",
                                                    "common_tasks", "excluded_tasks")}, ensure_ascii=False))


if __name__ == "__main__":
    main()
