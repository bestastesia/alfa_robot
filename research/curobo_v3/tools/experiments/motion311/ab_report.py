#!/usr/bin/env python3
"""MOTION-311 v2 A/B report (MOTION-337): paired arm comparison per mode and the G3' consistency checks.

Per arm: success (first try = no fallback round; overall) and failure classes; paired relative differences against
A (and B against B0) of T_lb_path (primary), J_arc, motion duration and planning wall time, over the instances where
both arms succeed, with the median and its cluster-bootstrap 95 % interval (clusters = box cells, 2000 resamples,
seed 311); the stage selection latency (p50/p95 per round); stage outcomes (abstention, carry/place choices,
place attempts, fallbacks). G3': a B/B0 round whose selection abstained or fell back to a full baseline round must
reproduce arm A's frames; online contact choices are compared with the offline G2' prediction when given.
The proposed decision rules (prereg decision_rules_proposed) are evaluated per mode.
"""
import argparse
import json
from pathlib import Path

import numpy as np

METRICS = {"t_lb_path": lambda entry: entry["cost"]["t_path"], "j_arc": lambda entry: entry["cost"]["j_arc"],
           "motion_s": lambda entry: entry.get("motion_duration_s"), "plan_ms": lambda entry: entry["wall_ms"]}


def cells(record):
    return "|".join(sorted(f"{int(box) // 5}:{min(int(box) % 5, 4 - int(box) % 5)}" for box in record["task"].values()))


def bootstrap_median(values, clusters, rounds=2000, seed=311):
    values, clusters = np.asarray(values), np.asarray(clusters)
    if not len(values):
        return None
    groups = [np.flatnonzero(clusters == key) for key in sorted(set(clusters))]
    rng = np.random.default_rng(seed)
    medians = [np.median(values[np.concatenate([groups[index] for index in rng.integers(len(groups), size=len(groups))])]) for _ in range(rounds)]
    return dict(n=len(values), median=float(np.median(values)), ci95=[float(np.quantile(medians, .025)), float(np.quantile(medians, .975))])


def paired(records, arm, reference):
    output = {}
    for name, metric in METRICS.items():
        values, clusters = [], []
        for record in records:
            first, second = record["arms"].get(arm), record["arms"].get(reference)
            if not first or not second or first["class_"] != "success" or second["class_"] != "success":
                continue
            a, b = metric(first), metric(second)
            if a is None or b is None or not b:
                continue
            values.append((a - b) / b)
            clusters.append(cells(record))
        output[name] = bootstrap_median(values, clusters)
    return output


def selection_ms(entry):
    selection = entry.get("selection") or {}
    parts = [(selection.get("contact") or {}).get("ms", 0.)] + [item.get("ms", 0.) for item in selection.get("carry", []) + selection.get("place", [])]
    return float(sum(parts))


def arm_summary(records, arm):
    entries = [record["arms"][arm] for record in records if arm in record["arms"]]
    if not entries:
        return None
    classes = {}
    for entry in entries:
        classes[entry["class_"]] = classes.get(entry["class_"], 0) + 1
    selections = [entry.get("selection") or {} for entry in entries]
    fallback = [bool(selection.get("fallback")) for selection in selections]
    output = dict(n=len(entries), success=float(np.mean([entry["class_"] == "success" for entry in entries])),
                  first_try=float(np.mean([entry["class_"] == "success" and not back for entry, back in zip(entries, fallback)])),
                  classes=classes, fallback_rounds=int(sum(fallback)))
    if arm != "A":
        latency = [selection_ms(entry) for entry in entries]
        places = [item for selection in selections for item in selection.get("place", [])]
        carries = [item for selection in selections for item in selection.get("carry", [])]
        output.update(abstained=float(np.mean([bool(selection.get("abstained")) for selection in selections])),
                      selection_ms_p50_p95=[float(np.quantile(latency, .5)), float(np.quantile(latency, .95))],
                      carry=dict(n=len(carries), baseline_rule=float(np.mean([item["choice"] is None for item in carries])) if carries else None),
                      place=dict(n=len(places), outcomes={key: sum(item["outcomes"].count(key) for item in places)
                                                          for key in sorted({value for item in places for value in item["outcomes"]})}))
    return output


def g3(records, offline=None):
    """B/B0 rounds that abstained or fell back must reproduce arm A's frames; online contact choice vs offline."""
    checks, agree = [], []
    choices = {}
    if offline is not None:
        for row in offline["rows"]["contact"]:
            for arm in ("B", "Bx", "B0"):
                if arm in row and row[arm].get("choice") is not None:
                    choices[row["instance"], arm] = row[arm]["choice"]
    for record in records:
        reference = record["arms"].get("A")
        for arm, entry in record["arms"].items():
            if arm == "A" or reference is None:
                continue
            selection = entry.get("selection") or {}
            if selection.get("abstained") or selection.get("fallback"):
                checks.append(dict(instance=record["instance"], arm=arm, kind="abstained" if selection.get("abstained") else "fallback",
                                   equal=entry.get("frames_digest") == reference.get("frames_digest")))
            online = ((selection.get("contact") or {}).get("order") or [None])[0]
            if (record["instance"], arm) in choices and online is not None:
                agree.append(online == choices[record["instance"], arm])
    return dict(frames_equal=float(np.mean([item["equal"] for item in checks])) if checks else None, checks=len(checks),
                mismatches=[item for item in checks if not item["equal"]],
                contact_choice_equal_offline=float(np.mean(agree)) if agree else None, compared=len(agree))


def verdict(records, arm="B"):
    both = [record for record in records if "A" in record["arms"] and arm in record["arms"]]
    a_only = sum(record["arms"]["A"]["class_"] == "success" and record["arms"][arm]["class_"] != "success" for record in both)
    success_ok = np.mean([record["arms"][arm]["class_"] == "success" for record in both]) >= np.mean([record["arms"]["A"]["class_"] == "success" for record in both]) and a_only <= 1
    cost = paired(records, arm, "A")["t_lb_path"]
    time = paired(records, arm, "A")["plan_ms"]
    return dict(success_ok=bool(success_ok), a_success_b_fail=int(a_only),
                cost=None if cost is None else ("improved" if cost["median"] <= -.05 and cost["ci95"][1] < 0 else
                                                "non_inferior" if cost["median"] <= .02 else "worse"),
                time=None if time is None else ("improved" if time["median"] <= -.15 else "non_inferior" if time["median"] <= .10 else "worse"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ab", type=Path, required=True)
    parser.add_argument("--offline", type=Path, help="g2v2_report.py output (contact choice agreement)")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    records = [json.loads(path.read_text()) for path in sorted((args.ab / "instances").glob("*.json"))]
    offline = json.loads(args.offline.read_text()) if args.offline else None
    arms = sorted({arm for record in records for arm in record["arms"]})
    report = dict(instances=len(records), arms=arms, by_mode={})
    for mode in sorted({record["mode"] for record in records}) + ["dual", "single", "all"]:
        mine = [record for record in records if mode in ("all", record["mode"], record["kind"])]
        if not mine:
            continue
        report["by_mode"][mode] = dict(n=len(mine), arms={arm: arm_summary(mine, arm) for arm in arms},
                                       paired={f"{arm}_vs_A": paired(mine, arm, "A") for arm in arms if arm != "A"},
                                       verdict={arm: verdict(mine, arm) for arm in arms if arm != "A"})
        if "B" in arms and "B0" in arms:
            report["by_mode"][mode]["paired"]["B_vs_B0"] = paired(mine, "B", "B0")
    report["g3"] = g3(records, offline)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=1, default=float) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=1, default=float))


if __name__ == "__main__":
    main()
