#!/usr/bin/env python3
"""MOTION-311 G1 report: per-subset headroom, label noise, censoring and the J_arc / motion-time relation.

Subsets are {dual, single} x {side, top}. Per pre-registration v1 a subset passes G1 for S1 when
  (median cost headroom >= 5 % or median time headroom >= 150 ms) and median seed-noise Spearman >= 0.5;
for S2 when the median S2 cost headroom >= 5 % (same noise gate). Cost headroom is reported twice:
  single-seed   (J_A - min_i J_i) / J_A on the primary labels (optimistic: the minimum of noisy values)
  cross-seed    the candidate chosen on seed offset 0, costed with its seed-offset-1 label (noise subset)
T_lb = sum over motion phases of max_j |dq_j| / v_j (URDF velocity limits) between the phase endpoints,
compared with time-parameterized motion durations on the dev diagnostic runs.
"""
import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import label_analysis as analysis

ROOT = Path(__file__).resolve().parents[3]
URDF = ROOT / "generated/v3_analytic_071cb95/model/robot_meter_meshes.urdf"
ACTIVE_JOINTS = ["updown", *[f"left_joint{i}" for i in range(1, 8)], *[f"right_joint{i}" for i in range(1, 8)]]
SUBSETS = ("dual-side", "dual-top", "single-side", "single-top")


def velocities():
    joints = {joint.attrib["name"]: joint for joint in ET.parse(URDF).getroot().findall("joint")}
    return np.array([float(joints[name].find("limit").attrib["velocity"]) for name in ACTIVE_JOINTS])


def time_bound(cost, velocity):
    ends = (cost or {}).get("phase_ends")
    if not ends:
        return None
    return float(sum(np.max(np.abs(np.asarray(last) - np.asarray(first)) / velocity) for first, last in ends.values()))


def median(values):
    values = [value for value in values if value is not None and not (isinstance(value, float) and math.isnan(value))]
    return float(np.median(values)) if values else None


def s2_headroom(record):
    """Per S2 root: (J of the root's S1 run with the baseline goals - best labelled single goal) / that J."""
    output = []
    for root in record.get("s2_pools", {}):
        base = next((entry for entry in record["s1"] if entry["index"] == int(root)), None)
        labelled = [entry for entry in record["s2"] if entry["root"] == int(root) and entry["class_"] == "success"]
        if base is None or base["class_"] != "success" or not labelled:
            continue
        base_j = base["cost"]["j_arc"]
        best = min(entry["cost"]["j_arc"] for entry in labelled)
        output.append(dict(root=int(root), base_j=base_j, best_j=best, headroom=(base_j - best) / base_j,
                           best_in_baseline=next(entry for entry in labelled if entry["cost"]["j_arc"] == best)["in_baseline"]))
    return output


def cross_seed(primary, repeat):
    """Realisable cost headroom: argmin on seed offset 0, costed on seed offset 1 (both baselines from offset 1)."""
    other = {record["instance"]: record for record in repeat}
    rows = []
    for record in primary:
        twin = other.get(record["instance"])
        if twin is None or twin["baseline"]["class_"] != "success":
            continue
        mine = {entry["index"]: entry["cost"]["j_arc"] for entry in record["s1"] if entry["class_"] == "success"}
        theirs = {entry["index"]: entry["cost"]["j_arc"] for entry in twin["s1"] if entry["class_"] == "success"}
        if not mine:
            continue
        chosen = min(mine, key=mine.get)
        base = twin["baseline"]["cost"]["j_arc"]
        rows.append(dict(instance=record["instance"], subset=analysis.subset(record), chosen=chosen,
                         chosen_succeeds_on_k1=chosen in theirs,
                         realised_headroom=None if chosen not in theirs else (base - theirs[chosen]) / base,
                         single_seed_headroom_k1=(base - min(theirs.values())) / base if theirs else None))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--primary", type=Path, required=True)
    parser.add_argument("--noise", type=Path, required=True)
    parser.add_argument("--censor", type=Path)
    parser.add_argument("--budget-ms", type=float, default=350.)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    primary, repeat = analysis.load(args.primary), analysis.load(args.noise)
    velocity = velocities()
    rows, summary = analysis.g1(primary, args.budget_ms)
    noise = analysis.noise(primary, repeat)
    crossed = cross_seed(primary, repeat)
    by_instance = {record["instance"]: record for record in primary}
    report = dict(budget_ms=args.budget_ms, instances=len(primary), noise_instances=len(repeat), subsets={})
    for name in SUBSETS:
        records = [record for record in primary if analysis.subset(record) == name]
        labels = {}
        for record in records:
            for entry in record["s1"]:
                labels[entry["class_"]] = labels.get(entry["class_"], 0) + 1
        baseline = {}
        for record in records:
            baseline[record["baseline"]["class_"]] = baseline.get(record["baseline"]["class_"], 0) + 1
        s2 = [item for record in records for item in s2_headroom(record)]
        noise_values = [item["spearman"] for key, item in noise.items() if analysis.subset(by_instance[key]) == name]
        cross_rows = [row for row in crossed if row["subset"] == name]
        entry = dict(summary.get(name, {}), baseline_classes=baseline, s1_classes=labels,
                     noise_median_spearman=median(noise_values), noise_instances=len(noise_values),
                     cross_seed_instances=len(cross_rows), cross_seed_median_headroom=median([row["realised_headroom"] for row in cross_rows]),
                     cross_seed_chosen_still_succeeds=float(np.mean([row["chosen_succeeds_on_k1"] for row in cross_rows])) if cross_rows else None,
                     s2_roots=len(s2), s2_median_headroom=median([item["headroom"] for item in s2]),
                     s2_best_outside_baseline_goals=float(np.mean([not item["best_in_baseline"] for item in s2])) if s2 else None)
        cost_ok = (entry.get("median_cost_headroom") or 0) >= .05
        time_ok = (entry.get("median_time_headroom_ms") or 0) >= 150
        noise_ok = entry["noise_median_spearman"] is not None and entry["noise_median_spearman"] >= .5
        entry["g1_s1"] = dict(cost=cost_ok, time=time_ok, noise=noise_ok, passed=bool((cost_ok or time_ok) and noise_ok))
        entry["g1_s2"] = dict(cost=(entry["s2_median_headroom"] or 0) >= .05, noise=noise_ok,
                              passed=bool((entry["s2_median_headroom"] or 0) >= .05 and noise_ok))
        report["subsets"][name] = entry
    timed_pairs = []
    for record in primary:
        points = [(entry["cost"]["j_arc"], time_bound(entry["cost"], velocity), entry["timed"]["motion_duration_s"])
                  for entry in record["s1"] if entry.get("timed") and entry["timed"].get("motion_duration_s") is not None
                  and entry.get("cost")]
        if len(points) >= 3:
            timed_pairs.append(dict(instance=record["instance"], subset=analysis.subset(record),
                                    j_vs_t=analysis.spearman([p[0] for p in points], [p[2] for p in points]),
                                    tlb_vs_t=analysis.spearman([p[1] for p in points], [p[2] for p in points]),
                                    points=points))
    report["time_relation"] = dict(instances=len(timed_pairs), median_spearman_j_arc_vs_motion=median([row["j_vs_t"] for row in timed_pairs]),
                                   median_spearman_t_lb_vs_motion=median([row["tlb_vs_t"] for row in timed_pairs]),
                                   by_subset={name: dict(j=median([row["j_vs_t"] for row in timed_pairs if row["subset"] == name]),
                                                         t_lb=median([row["tlb_vs_t"] for row in timed_pairs if row["subset"] == name]))
                                              for name in SUBSETS})
    t_rows = []
    for record in primary:
        if record["baseline"]["class_"] != "success" or not record["s1"]:
            continue
        base = time_bound(record["baseline"]["cost"], velocity)
        bounds = [time_bound(entry["cost"], velocity) for entry in record["s1"] if entry["class_"] == "success"]
        bounds = [value for value in bounds if value is not None]
        if base and bounds:
            t_rows.append(dict(subset=analysis.subset(record), headroom=(base - min(bounds)) / base))
    report["t_lb_headroom"] = {name: median([row["headroom"] for row in t_rows if row["subset"] == name]) for name in SUBSETS}
    if args.censor:
        censored = analysis.load(args.censor)
        rescued = total = 0
        for record in censored:
            for entry in record["s1"]:
                total += 1
                rescued += entry["class_"] == "success"
        report["censoring_probe"] = dict(candidates=total, succeeded_with_3x_budget=rescued,
                                         fraction=rescued / total if total else None)
    report["rows"] = rows
    report["cross_seed"] = crossed
    report["timed"] = timed_pairs
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=1, default=float) + "\n")
    brief = {key: value for key, value in report.items() if key not in ("rows", "cross_seed", "timed")}
    print(json.dumps(brief, ensure_ascii=False, indent=1, default=float))


if __name__ == "__main__":
    main()
