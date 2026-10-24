#!/usr/bin/env python3
"""MOTION-311 G1p: place-stage headroom from the D3 labels (per mode, per carry IK).

For every carry IK g the D3 labels hold the baseline place rule (cuRobo IK nearest to g, first connected) and
forced enumerated place IKs: 4 best analytic T_lb(g->p) + T_lb(p->Home), 2 nearest to the baseline placement, 2
random. Realized place+return time is the planned-path bound t_path (sum over frames of max_j |dq_j| / v_j).
  headroom   (baseline t - best labelled feasible t) / baseline t           (hindsight over the labelled set)
  b0_gain    (baseline t - t of the analytic-best choice, if it succeeds) / baseline t; b0 first-try success
The independence check compares classes/times of the same pair under other tasks and offsets.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def place_return(entry):
    times = (entry.get("cost") or {}).get("t_path_by_phase") or {}
    return times.get("conveyor_place", 0.) + times.get("return_home", 0.) if times else None


def analyse(folder):
    rows = [json.loads(line) for line in (folder / "labels.jsonl").read_text().splitlines()]
    by_carry = {}
    for row in rows:
        by_carry.setdefault(row["carry_index"], []).append(row)
    carries = []
    for index, entries in sorted(by_carry.items()):
        baseline = next((entry for entry in entries if entry["place_index"] is None), None)
        forced = [entry for entry in entries if entry["place_index"] is not None]
        if baseline is None or len(forced) < 8:
            continue
        analytic, near, random = forced[:4], forced[4:6], forced[6:]
        feasible = [entry for entry in forced if entry["class_"] == "success"]
        base_t = place_return(baseline) if baseline["class_"] == "success" else None
        best_t = min((place_return(entry) for entry in feasible), default=None)
        b0 = analytic[0]
        carries.append(dict(carry=index, baseline_class=baseline["class_"], baseline_t=base_t, best_t=best_t,
                            headroom=None if base_t is None or best_t is None else (base_t - best_t) / base_t,
                            b0_success=b0["class_"] == "success",
                            b0_gain=None if base_t is None or b0["class_"] != "success" else (base_t - place_return(b0)) / base_t,
                            analytic_success=float(np.mean([entry["class_"] == "success" for entry in analytic])),
                            near_success=float(np.mean([entry["class_"] == "success" for entry in near])),
                            random_success=float(np.mean([entry["class_"] == "success" for entry in random])),
                            rescue=baseline["class_"] != "success" and bool(feasible)))
    output = dict(carries=len(carries), labels=len(rows),
                  classes={label: sum(row["class_"] == label for row in rows) for label in sorted({row["class_"] for row in rows})})
    if carries:
        def median(key):
            values = [item[key] for item in carries if item[key] is not None]
            return (float(np.median(values)), float(np.quantile(values, .9))) if values else None
        output.update(
            baseline_success=float(np.mean([item["baseline_class"] == "success" for item in carries])),
            baseline_failures_rescuable=sum(item["rescue"] for item in carries),
            headroom_median_q90=median("headroom"), share_headroom_ge_5pct=float(np.mean([item["headroom"] >= .05 for item in carries if item["headroom"] is not None])),
            b0_first_try_success=float(np.mean([item["b0_success"] for item in carries])), b0_gain_median_q90=median("b0_gain"),
            success_rate=dict(analytic=float(np.mean([item["analytic_success"] for item in carries])),
                              near_baseline=float(np.mean([item["near_success"] for item in carries])),
                              random=float(np.mean([item["random_success"] for item in carries]))))
    check = folder / "independence.jsonl"
    if check.exists():
        rows = [json.loads(line) for line in check.read_text().splitlines()]
        same = [row["class_"] == row["reference_class"] for row in rows]
        delta = [abs(sum(row["t"].values()) - sum(row["reference_t"].values())) / sum(row["reference_t"].values())
                 for row in rows if row["t"] and row["reference_t"]]
        output["independence"] = dict(runs=len(rows), class_agreement=float(np.mean(same)),
                                      time_delta_median=float(np.median(delta)) if delta else None,
                                      time_delta_q90=float(np.quantile(delta, .9)) if delta else None)
    output["rows"] = carries
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--d3", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = {folder.name: analyse(folder) for folder in sorted(args.d3.iterdir()) if (folder / "labels.jsonl").exists()}
    if args.output:
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=1, default=float) + "\n")
    print(json.dumps({mode: {key: value for key, value in item.items() if key != "rows"} for mode, item in report.items()},
                     ensure_ascii=False, indent=1, default=float))


if __name__ == "__main__":
    main()
