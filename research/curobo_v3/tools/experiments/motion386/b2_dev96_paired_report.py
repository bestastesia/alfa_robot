"""Build the same-source default-size v2b versus B2-time paired report."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.stats import binomtest


def load(directory):
    rows = [json.loads(path.read_text()) for path in sorted(Path(directory).glob("*.json"))]
    mapping = {row["instance"]: row for row in rows}
    if len(rows) != 96 or len(mapping) != 96:
        raise ValueError(f"expected 96 unique instances in {directory}, found {len(rows)}/{len(mapping)}")
    return mapping


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def b1_timed(row, name):
    return row.get("report", {}).get("timed_validation", {}).get(name)


def b2_timed(row, name):
    return row.get("timed_validation", {}).get(name)


def metric(row, arm, name):
    if name == "selection_ms":
        return row.get("selection_total_ms" if arm == "v2b" else "selection_ms")
    if name == "total_wall_ms":
        selection = metric(row, arm, "selection_ms") or 0.
        return selection + (row.get("planning_wall_ms") or 0.)
    if name == "acceleration_ratio":
        return b1_timed(row, "acceleration_limit_ratio") if arm == "v2b" else b2_timed(row, "acceleration_limit_ratio")
    if name == "jerk_ratio":
        return b1_timed(row, "jerk_limit_ratio") if arm == "v2b" else b2_timed(row, "jerk_limit_ratio")
    return row.get(name)


def distribution(values):
    values = [float(value) for value in values if value is not None and np.isfinite(float(value))]
    return {"n": len(values), "mean": float(np.mean(values)) if values else None,
            "p50": float(np.percentile(values, 50)) if values else None,
            "p95": float(np.percentile(values, 95)) if values else None}


def failure_counts(rows):
    counts = {}
    for row in rows.values():
        if row.get("success"):
            continue
        key = f"stage{row.get('failure_stage')}:{row.get('reason')}"
        counts[key] = counts.get(key, 0) + 1
    return counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--v2b-instances", type=Path, required=True)
    parser.add_argument("--v2b-config", type=Path, required=True)
    parser.add_argument("--b2-instances", type=Path, required=True)
    parser.add_argument("--b2-config", type=Path, required=True)
    parser.add_argument("--registry", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    v2b, b2 = load(args.v2b_instances), load(args.b2_instances)
    if set(v2b) != set(b2):
        raise ValueError("paired arms contain different instances")
    instances = sorted(v2b)
    v2b_only = [key for key in instances if v2b[key]["success"] and not b2[key]["success"]]
    b2_only = [key for key in instances if b2[key]["success"] and not v2b[key]["success"]]
    both = [key for key in instances if v2b[key]["success"] and b2[key]["success"]]
    neither = [key for key in instances if not v2b[key]["success"] and not b2[key]["success"]]
    names = ("selection_ms", "planning_wall_ms", "total_wall_ms", "raw_t_path_s", "executed_t_path_s",
             "motion_duration_s", "acceleration_ratio", "jerk_ratio")
    paired = {}
    for name in names:
        values = []
        for key in both:
            left, right = metric(v2b[key], "v2b", name), metric(b2[key], "b2", name)
            if left is not None and right is not None:
                values.append(float(right) - float(left))
        paired[name] = {**distribution(values), "better": sum(value < 0. for value in values),
                        "same": sum(abs(value) <= 1e-12 for value in values), "worse": sum(value > 0. for value in values)}

    per_mode = []
    for mode in sorted({row["mode"] for row in v2b.values()}):
        keys = [key for key in instances if v2b[key]["mode"] == mode]
        per_mode.append({"mode": mode, "instances": len(keys),
                         "v2b_success": sum(v2b[key]["success"] for key in keys),
                         "b2_success": sum(b2[key]["success"] for key in keys)})
    detail = []
    for key in instances:
        left, right = v2b[key], b2[key]
        row = {"instance": key, "mode": left["mode"], "v2b_success": bool(left["success"]),
               "b2_success": bool(right["success"]), "v2b_failure_stage": left.get("failure_stage"),
               "v2b_reason": left.get("reason"), "b2_failure_stage": right.get("failure_stage"),
               "b2_reason": right.get("reason")}
        for name in names:
            row["v2b_" + name] = metric(left, "v2b", name)
            row["b2_" + name] = metric(right, "b2", name)
            row["delta_" + name] = (None if row["v2b_" + name] is None or row["b2_" + name] is None else
                                    row["b2_" + name] - row["v2b_" + name])
        detail.append(row)

    discordant = len(v2b_only) + len(b2_only)
    report = {
        "schema": "motion386-b2-time-dev96-paired-v1",
        "dataset": "S30x40x40 dev96 seed101 strict single selection/group",
        "registry_sha256": sha256(args.registry),
        "v2b": {"success": sum(row["success"] for row in v2b.values()), "instances": 96,
                 "config": str(args.v2b_config), "config_sha256": sha256(args.v2b_config),
                 "selection": distribution(metric(row, "v2b", "selection_ms") for row in v2b.values()),
                 "success_metrics": {name: distribution(metric(row, "v2b", name) for row in v2b.values() if row["success"])
                                     for name in names if name != "selection_ms"},
                 "selection_budget_exceeded": sum(bool(row.get("selection_budget_exceeded")) for row in v2b.values()),
                 "failures": failure_counts(v2b)},
        "b2_time_beam8": {"success": sum(row["success"] for row in b2.values()), "instances": 96,
                           "config": str(args.b2_config), "config_sha256": sha256(args.b2_config),
                           "selection": distribution(metric(row, "b2", "selection_ms") for row in b2.values()),
                           "success_metrics": {name: distribution(metric(row, "b2", name) for row in b2.values() if row["success"])
                                               for name in names if name != "selection_ms"},
                           "selection_budget_exceeded": sum(bool(row.get("selection_budget_exceeded")) for row in b2.values()),
                           "failures": failure_counts(b2)},
        "paired_outcomes": {"v2b_only_success": v2b_only, "b2_only_success": b2_only,
                            "both_success": len(both), "neither_success": len(neither),
                            "discordant_exact_two_sided_p": (float(binomtest(len(b2_only), discordant, .5).pvalue)
                                                             if discordant else 1.)},
        "paired_b2_minus_v2b": paired,
        "per_mode": per_mode,
        "decision": "specific dual-side development improvement only; keep v2b selected because net success is +1/96 with five outcome swaps, paired trajectory medians do not improve, and total wall time increases",
        "independent_test_opened": False,
        "rows": detail,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    with (args.output_dir / "pairs.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(detail[0]), lineterminator="\n")
        writer.writeheader(); writer.writerows(detail)
    print(json.dumps({"v2b": report["v2b"]["success"], "b2": report["b2_time_beam8"]["success"],
                      "v2b_only": len(v2b_only), "b2_only": len(b2_only), "both": len(both),
                      "decision": report["decision"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
