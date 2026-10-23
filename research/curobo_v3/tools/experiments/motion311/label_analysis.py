#!/usr/bin/env python3
"""MOTION-311 offline analysis of label instances: candidate tables, policy simulation and the G1 headroom gate.

Policy simulation (exact for single-candidate attempts): candidates are attempted alone in the given order,
at most m times, each with a transport allocation of B ms; an attempt succeeds iff its forced label
succeeded with first_verified_solution_ms <= B. A failed attempt costs its forced wall time up to the
failure (a transport timeout costs the time before the transport search plus B). Dual arm fails over to
the next attempt unless the failure came after selection (conveyor suffix), single arm unless it came at
or after the transport search; those, and m exhausted attempts, fall back to the recorded baseline round
(its wall time is added). Costs: J_arc of the forced run, or of the baseline round on fallback.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

SINGLE_BUDGET_MS = 2000.  # single-arm attempts keep the 2 s searches; only their order changes
AFTER_SELECTION = {"dual": {"反算放置底盘位", "龙头车放置目标IK", "龙头车放置起点", "龙头车放置RRT", "龙头车放置RRT细验", "保持转身姿态回Home"},
                   "single": {"运输终点", "运输RRT", "运输终点圆柱验收", "反算放置底盘位", "龙头车放置目标IK", "龙头车放置起点",
                              "龙头车放置RRT", "龙头车放置RRT细验", "保持转身姿态回Home"}}


def load(folder):
    return [json.loads(path.read_text()) for path in sorted(Path(folder, "instances").glob("*.json"))]


def subset(record):
    return f"{record['kind']}-{'top' if record['context']['top_grasp'] else 'side'}"


def transport_call(record, entry):
    if record["kind"] == "dual":
        calls = [call for call in entry["searches"] if call["module"] == "multiroot"]
    else:
        flags = [call for call in entry.get("trace", []) if call.get("call") == "single_path"]
        arm = [call for call in entry["searches"] if call["module"] == "single_arm"]
        calls = [call for call, flag in zip(arm, flags) if flag.get("rotation")]
    return calls[0] if calls else None


def candidate(record, entry):
    """Per-candidate facts used by the simulator."""
    suffix = entry.get("suffix") or {}
    wall = entry["wall_ms"] + (suffix.get("ms", 0.) if suffix.get("cached") else 0.)
    call = transport_call(record, entry)
    prep = None if call is None else call["start_ms"] - entry["start_ms"]
    verified = None if call is None else call.get("first_verified_solution_ms")
    cost = entry.get("cost") or {}
    return dict(index=entry.get("index"), label=entry["class_"], stage=entry["stage"], wall_ms=wall, prep_ms=prep,
                transport_verified_ms=verified, transport_wall_ms=None if call is None else call["wall_ms"],
                j_arc=cost.get("j_arc"), arc_by_phase=cost.get("arc_by_phase"), rotation_cost=cost.get("rotation_cost"))


def attempt(kind, facts, budget_ms):
    """(outcome, cost ms) of one single-candidate attempt; outcome is 'success', 'next' or 'fallback'."""
    verified, prep = facts["transport_verified_ms"], facts["prep_ms"]
    if prep is not None and (verified is None or verified > budget_ms):
        waited = budget_ms if verified is not None else min(budget_ms, facts["transport_wall_ms"])
        return ("next" if kind == "dual" else "fallback"), prep + waited
    if facts["label"] == "success":
        return "success", facts["wall_ms"]
    if facts["stage"] in AFTER_SELECTION[kind]:
        return "fallback", facts["wall_ms"]
    return "next", facts["wall_ms"]


def simulate(record, order, m, budget_ms, extra_ms=0.):
    kind = record["kind"]
    table = {entry["index"]: candidate(record, entry) for entry in record["s1"]}
    baseline = record["baseline"]
    spent, attempts = extra_ms, 0
    for index in order[:m]:
        outcome, cost = attempt(kind, table[index], budget_ms)
        spent += cost
        attempts += 1
        if outcome == "success":
            return dict(success=True, j_arc=table[index]["j_arc"], plan_ms=spent, attempts=attempts, fallback=False, chosen=index)
        if outcome == "fallback":
            break
    cost = (baseline.get("cost") or {}).get("j_arc")
    return dict(success=baseline["class_"] == "success", j_arc=cost, plan_ms=spent + baseline["wall_ms"], attempts=attempts,
                fallback=True, chosen=baseline.get("selected_index"))


def oracle_order(record, budget_ms):
    facts = [candidate(record, entry) for entry in record["s1"]]
    good = [item for item in facts if item["label"] == "success" and item["transport_verified_ms"] is not None
            and item["transport_verified_ms"] <= budget_ms]
    return [item["index"] for item in sorted(good, key=lambda item: item["j_arc"])]


def ranking_ms(record):
    lazy = ((record["baseline"].get("timings") or {}).get("lazy") or {}).get("timings_ms") or {}
    return lazy.get("endpoint_prediction_ms", 0.) + lazy.get("prediction_ranking_ms", 0.)


def g1(records, budget_ms=350., m=1):
    rows = []
    for record in records:
        baseline = record["baseline"]
        if baseline["class_"] != "success" or not record["s1"]:
            rescue = oracle_order(record, budget_ms if record["kind"] == "dual" else SINGLE_BUDGET_MS) if record["s1"] else []
            rows.append(dict(instance=record["instance"], subset=subset(record), usable=False, baseline=baseline["class_"],
                             baseline_stage=baseline.get("stage"), rescuable=bool(rescue), rescue_candidates=len(rescue)))
            continue
        budget = budget_ms if record["kind"] == "dual" else SINGLE_BUDGET_MS
        best = oracle_order(record, budget)
        base_j = baseline["cost"]["j_arc"]
        if record["kind"] == "dual":
            a_prime = simulate(record, baseline.get("ranking") or list(range(len(record["s1"]))), m, budget, ranking_ms(record))
        else:
            a_prime = simulate(record, list(range(len(record["s1"]))), len(record["s1"]), budget)
        best_first = simulate(record, best, 1, budget) if best else None
        facts = [candidate(record, entry) for entry in record["s1"]]
        rows.append(dict(
            instance=record["instance"], subset=subset(record), usable=True, baseline_j=base_j, baseline_ms=baseline["wall_ms"],
            best_j=None if not best else best_first["j_arc"], best_index=best[0] if best else None,
            cost_headroom=None if not best else (base_j - best_first["j_arc"]) / base_j,
            a_prime_ms=None if a_prime is None else a_prime["plan_ms"], best_first_ms=None if not best else best_first["plan_ms"],
            time_headroom_ms=None if (a_prime is None or not best) else a_prime["plan_ms"] - best_first["plan_ms"],
            successes=sum(item["label"] == "success" for item in facts), candidates=len(facts),
            labels={label: sum(item["label"] == label for item in facts) for label in sorted({item["label"] for item in facts})}))
    summary = {}
    for name in sorted({row["subset"] for row in rows}):
        group = [row for row in rows if row["subset"] == name and row["usable"]]
        cost = [row["cost_headroom"] for row in group if row["cost_headroom"] is not None]
        time_gain = [row["time_headroom_ms"] for row in group if row["time_headroom_ms"] is not None]
        failed = [row for row in rows if row["subset"] == name and not row["usable"]]
        summary[name] = dict(instances=len([row for row in rows if row["subset"] == name]), usable=len(group),
                             baseline_failed=len(failed), rescuable=sum(bool(row.get("rescuable")) for row in failed),
                             median_cost_headroom=float(np.median(cost)) if cost else None,
                             median_time_headroom_ms=float(np.median(time_gain)) if time_gain else None,
                             median_success_fraction=float(np.median([row["successes"] / row["candidates"] for row in group])) if group else None)
    return rows, summary


def spearman(first, second):
    first, second = np.argsort(np.argsort(first)), np.argsort(np.argsort(second))
    if len(first) < 3 or np.std(first) == 0 or np.std(second) == 0:
        return None
    return float(np.corrcoef(first, second)[0, 1])


def noise(primary, repeat):
    """Within-instance Spearman of J_arc between two label runs (e.g. seed offsets 0 and 1)."""
    other = {record["instance"]: record for record in repeat}
    output = {}
    for record in primary:
        if record["instance"] not in other:
            continue
        mine = {entry["index"]: (entry.get("cost") or {}).get("j_arc") for entry in record["s1"] if entry["class_"] == "success"}
        theirs = {entry["index"]: (entry.get("cost") or {}).get("j_arc") for entry in other[record["instance"]]["s1"] if entry["class_"] == "success"}
        common = sorted(set(mine) & set(theirs))
        output[record["instance"]] = dict(common=len(common), spearman=spearman([mine[i] for i in common], [theirs[i] for i in common]),
                                          label_agreement=float(np.mean([entry["class_"] == other_entry["class_"] for entry, other_entry
                                                                         in zip(record["s1"], other[record["instance"]]["s1"])])) if record["s1"] else None)
    return output


def determinism(first, second):
    """Fields that must be identical between two runs of the same configuration."""
    other = {record["instance"]: record for record in second}
    differences = []
    for record in first:
        twin = other.get(record["instance"])
        if twin is None:
            continue
        for part in ("s1", "s2"):
            for mine, theirs in zip(record[part], twin[part]):
                for key in ("class_", "stage"):
                    if mine.get(key) != theirs.get(key):
                        differences.append((record["instance"], part, mine.get("index", mine.get("goal")), key, mine.get(key), theirs.get(key)))
                cost, twin_cost = mine.get("cost") or {}, theirs.get("cost") or {}
                if cost.get("j_arc") != twin_cost.get("j_arc"):
                    differences.append((record["instance"], part, mine.get("index", mine.get("goal")), "j_arc", cost.get("j_arc"), twin_cost.get("j_arc")))
        if (record["baseline"].get("cost") or {}).get("j_arc") != (twin["baseline"].get("cost") or {}).get("j_arc"):
            differences.append((record["instance"], "baseline", None, "j_arc", None, None))
    return differences


def timed_correlation(records):
    """Within-instance Spearman(J_arc, motion duration) over the time-parameterized S1 successes."""
    output = {}
    for record in records:
        pairs = [((entry.get("cost") or {}).get("j_arc"), entry["timed"].get("motion_duration_s")) for entry in record["s1"]
                 if entry.get("timed") and entry["timed"].get("motion_duration_s") is not None]
        if len(pairs) >= 3:
            output[record["instance"]] = dict(pairs=len(pairs), spearman=spearman([pair[0] for pair in pairs], [pair[1] for pair in pairs]))
    values = [item["spearman"] for item in output.values() if item["spearman"] is not None]
    return dict(per_instance=output, median=float(np.median(values)) if values else None)


def pilot(args):
    first = load(args.first)
    report = dict(instances={record["instance"]: dict(subset=subset(record), wall_s=round(record["wall_s"], 1), baseline=record["baseline"]["class_"],
                                                      s1={label: sum(entry["class_"] == label for entry in record["s1"]) for label in
                                                          sorted({entry["class_"] for entry in record["s1"]})},
                                                      s2={label: sum(entry["class_"] == label for entry in record["s2"]) for label in
                                                          sorted({entry["class_"] for entry in record["s2"]})},
                                                      suffix_cached=sum(bool((entry.get("suffix") or {}).get("cached")) for entry in record["s1"] + record["s2"]))
                                  for record in first})
    if args.second:
        report["determinism_differences"] = determinism(first, load(args.second))
    if args.no_memo:
        report["memo_differences"] = determinism(load(args.no_memo), first)
    if args.repeat:
        values = noise(first, load(args.repeat))
        report["noise"] = values
        report["noise_median_spearman"] = float(np.median([item["spearman"] for item in values.values() if item["spearman"] is not None] or [np.nan]))
    report["timed_correlation"] = timed_correlation(first)
    text = json.dumps(report, ensure_ascii=False, indent=1, default=float)
    if args.output:
        args.output.write_text(text + "\n")
    print(text[:6000])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    check = commands.add_parser("pilot")
    check.add_argument("--first", type=Path, required=True)
    check.add_argument("--second", type=Path)
    check.add_argument("--no-memo", type=Path)
    check.add_argument("--repeat", type=Path)
    check.add_argument("--output", type=Path)
    gate = commands.add_parser("g1")
    gate.add_argument("--labels", type=Path, required=True)
    gate.add_argument("--repeat", type=Path, help="labels of the same instances with another seed offset (noise gate)")
    gate.add_argument("--budget-ms", type=float, default=350.)
    gate.add_argument("--output", type=Path)
    same = commands.add_parser("determinism")
    same.add_argument("--first", type=Path, required=True)
    same.add_argument("--second", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "pilot":
        return pilot(args)
    if args.command == "determinism":
        differences = determinism(load(args.first), load(args.second))
        print(json.dumps(dict(differences=len(differences), first=differences[:20]), ensure_ascii=False, indent=1))
        raise SystemExit(0 if not differences else 1)
    records = load(args.labels)
    rows, summary = g1(records, args.budget_ms)
    output = dict(budget_ms=args.budget_ms, summary=summary, rows=rows)
    if args.repeat:
        output["noise"] = noise(records, load(args.repeat))
        values = [item["spearman"] for item in output["noise"].values() if item["spearman"] is not None]
        output["noise_median_spearman"] = float(np.median(values)) if values else None
    text = json.dumps(output, ensure_ascii=False, indent=1, default=float)
    if args.output:
        args.output.write_text(text + "\n")
    print(json.dumps(dict(summary=summary, noise_median_spearman=output.get("noise_median_spearman")), ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
