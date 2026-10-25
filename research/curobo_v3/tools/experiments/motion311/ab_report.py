#!/usr/bin/env python3
"""MOTION-311 v2 A/B report (MOTION-337): paired arm comparison per mode and the G3' consistency checks.

Per arm: success, success without a full-round fallback, and verified no-retry single-triple success (unknown or
not applicable kept separate); failure classes; paired relative differences against
A (and B against B0) of T_lb_path (primary), J_arc, motion duration, planning time (the planner's total_ms: selection,
search, verification and any fallback round), the trajectory time-parameterization that follows it and the wall time
of both, over the instances where both arms succeed, with the median and its cluster-bootstrap 95 % interval (clusters = box cells, 2000 resamples,
seed 311); the stage selection latency (p50/p95 per round); stage outcomes (abstention, carry/place choices,
place attempts, fallbacks). G3': a B/B0 round whose selection abstained or fell back to a full baseline round must
reproduce arm A's frames; online contact choices are compared with the offline G2' prediction when given.
The proposed decision rules (prereg decision_rules_proposed) are evaluated per mode. Instances whose scene is infeasible
for the baseline (no contact IK) are listed and left out.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from v3_label_classes import SCENE_MARKERS

METRICS = {"t_lb_path": lambda entry: entry["cost"]["t_path"], "j_arc": lambda entry: entry["cost"]["j_arc"],
           "motion_s": lambda entry: entry.get("motion_duration_s"), "plan_ms": lambda entry: entry.get("total_ms"),
           "timing_ms": lambda entry: entry.get("trajectory_timing_ms"), "wall_ms": lambda entry: entry["wall_ms"]}


def outcome(entry):
    """The arm's class; exceptions are split by their message (scene, CUDA memory, time-parameterization gate)."""
    if entry["class_"] != "exception":
        return entry["class_"]
    error = entry.get("error") or ""
    if any(marker in error for marker in SCENE_MARKERS):
        return "scene_infeasible"
    if "out of memory" in error.lower():
        return "exception:cuda_oom"
    if "timed samples failed" in error:
        return "exception:timing_gate"
    return "exception"


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


def first_selected_triple(entry):
    """True only with evidence of three single-candidate choices succeeding without a retry; False for a failed
    recommendation/retry; None when no triple was recommended or the recorded evidence is insufficient. In
    particular the baseline's parallel pool is not evidence of single-triple success."""
    selection = entry.get("selection")
    if not selection or selection.get("abstained"):
        return None
    if entry["class_"] != "success" or selection.get("fallback") or selection.get("baseline_ranking"):
        return False
    order = (selection.get("contact") or {}).get("order") or []
    if not order:
        return None
    batches = ((entry.get("timings") or {}).get("lazy") or {}).get("batches")
    if batches is not None:
        if len(batches) != 1 or batches[0].get("kind") != "selection" or not batches[0].get("success"):
            return False
        if batches[0].get("allowed_contacts") != [order[0]]:
            return False
        selected = entry.get("selected_contact")  # dual is zero-based
    else:
        attempts = [call for call in entry.get("trace", []) if call.get("call") == "cartesian_single" and not call.get("along_normal")]
        if not attempts:
            return None
        if len(attempts) != 1 or not attempts[0].get("ok"):
            return False
        selected = entry.get("selected_contact")
        selected = selected - 1 if selected is not None else None  # single is one-based
    if selected is None:
        return None
    if selected != order[0]:
        return False
    carries, places = selection.get("carry", []), selection.get("place", [])
    if not carries or not places:
        return None
    if len(carries) != 1 or len(places) != 1:
        return False
    outcomes = places[0].get("outcomes", [])
    if len(outcomes) > 1 or any(value not in ("success", "baseline_rule") for value in outcomes):
        return False
    if carries[0].get("choice") is None or outcomes != ["success"]:
        return None
    attempt_ranks = places[0].get("attempt_ranks")
    if not attempt_ranks:
        return None  # old logs did not record silent IK-resolution skips
    if attempt_ranks[0] != 0:
        return False
    searches = entry.get("searches") or []
    if not searches:
        return None
    if any(not call.get("success") for call in searches):
        return False
    if any(call.get("starts") != 1 or call.get("goals") != 1 for call in searches):
        return None
    return True


def arm_summary(records, arm):
    entries = [record["arms"][arm] for record in records if arm in record["arms"]]
    if not entries:
        return None
    classes = {}
    for entry in entries:
        classes[outcome(entry)] = classes.get(outcome(entry), 0) + 1
    selections = [entry.get("selection") or {} for entry in entries]
    fallback = [bool(selection.get("fallback")) for selection in selections]
    first = [first_selected_triple(entry) for entry in entries]
    known = [value for value in first if value is not None]
    output = dict(n=len(entries), success=float(np.mean([entry["class_"] == "success" for entry in entries])),
                  success_without_round_fallback=float(np.mean([entry["class_"] == "success" and not back for entry, back in zip(entries, fallback)])),
                  first_selected_triple=dict(verified_success=sum(value is True for value in first),
                                             failed_or_retried=sum(value is False for value in first),
                                             not_applicable_or_unknown=len(first) - len(known),
                                             rate_among_known=float(np.mean(known)) if known else None,
                                             verified_share_all=sum(value is True for value in first) / len(entries)),
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
    checks, partial, unverifiable, agree = [], [], [], []
    choices = {}
    if offline is not None:
        for row in offline["rows"]["contact"]:
            for arm in ("B", "Bx", "B0"):
                if arm in row and "choice" in row[arm]:
                    choices[row["instance"], arm] = row[arm]["choice"]
    for record in records:
        reference = record["arms"].get("A")
        for arm, entry in record["arms"].items():
            if arm == "A":
                continue
            selection = entry.get("selection") or {}
            if reference is not None and (selection.get("abstained") or selection.get("fallback")):
                item = dict(instance=record["instance"], arm=arm, kind="abstained" if selection.get("abstained") else "fallback")
                digest, baseline = entry.get("frames_digest"), reference.get("frames_digest")
                if not digest or not baseline:
                    unverifiable.append(dict(item, reason="missing frames digest"))
                else:
                    item["equal"] = digest == baseline and outcome(entry) == outcome(reference)
                    (checks if entry["class_"] == reference["class_"] == "success" else partial).append(item)
            online = ((selection.get("contact") or {}).get("order") or [None])[0]
            policy = "Bx" if arm == "B" and entry.get("contact_value") == "explicit" else arm
            if (record["instance"], policy) in choices and entry.get("selection") is not None:
                agree.append(online == choices[record["instance"], policy])
    return dict(frames_equal=float(np.mean([item["equal"] for item in checks])) if checks else None, checks=len(checks),
                partial_frames_equal=float(np.mean([item["equal"] for item in partial])) if partial else None,
                partial_checks=len(partial), unverifiable=unverifiable,
                mismatches=[item for item in checks + partial if not item["equal"]],
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
    loaded = [json.loads(path.read_text()) for path in sorted((args.ab / "instances").glob("*.json"))]
    records = [record for record in loaded if outcome(record["arms"]["A"]) != "scene_infeasible"]
    offline = json.loads(args.offline.read_text()) if args.offline else None
    arms = sorted({arm for record in records for arm in record["arms"]})
    report = dict(instances=len(records), scene_infeasible=[record["instance"] for record in loaded if record not in records], arms=arms, by_mode={})
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
