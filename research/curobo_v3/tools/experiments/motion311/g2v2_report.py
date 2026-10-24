#!/usr/bin/env python3
"""MOTION-311 G2' (v2, offline on the dev split): stage choices of B (networks), Bx (explicit V2) and B0 (T_lb only).

contact  every dev instance of P3: the arm's first choice among the 64 contacts as one single-candidate attempt (dual:
         singleton lazy batch within 350 ms, then the baseline schedule; single: first in the order, then the baseline
         order), realized from the forced S1 runs, i.e. with the baseline carry and place rules downstream. First-try
         success, T_lb of the round (phase ends; P3 has no path-wise bound) and estimated planning time against the
         baseline round of the same instance.
carry    the S2 roots of the dev instances: the arm's choice among the labelled forced goals and the baseline's own
         goal; realized T_lb of transport + place + return against the baseline's (a failed goal costs the baseline
         plus C_fail).
place    the out-of-fold N3 restricted choice on D3 (train_v2_report.json), per mode.
Also the distillation check: N1's V2 against the explicit V2 (log error, within-instance rank agreement of the pick).
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import v3_triple_ranker as triple
from experiments.motion311 import label_analysis as analysis
from experiments.motion311 import train_ranker as base
from experiments.motion311.train_triple import (URDF, load_tables, mode_of, planner_home, record_context, velocities)

FRONT = ("approach", "contact_approach", "extract")
DOWN = ("transport", "conveyor_place", "return_home")
VELOCITY = velocities()


def relative(values):
    values = [value for value in values if value is not None]
    return dict(n=len(values), median=float(np.median(values)), mean=float(np.mean(values)),
                q10=float(np.quantile(values, .1)), q90=float(np.quantile(values, .9))) if values else None


def t_lb(entry, phases=FRONT + DOWN):
    """Endpoint T_lb of the round's motion phases (P3 labels hold phase ends, not the path-wise bound)."""
    ends = ((entry or {}).get("cost") or {}).get("phase_ends")
    if not ends:
        return None
    return sum(float(triple.time_bound(ends[phase][0], ends[phase][1], VELOCITY)) for phase in phases if phase in ends)


def contact_stage(records, models, tables, velocity, theta=.5):
    from curobo_core.planner import AnalyticBridge
    home = planner_home().astype(np.float64)
    atlases = {mode: np.load(path, allow_pickle=False)["joints"] for mode, path in base.ATLASES.items()}
    kinematics = triple.ranker.Kinematics(URDF, analytic=AnalyticBridge(), carriage=triple.ranker.CarriageModel(triple.ranker.UrdfFk(URDF), home),
                                          atlases=atlases)
    rows, distillation = [], []
    for record in records:
        mode = mode_of(record)
        if record["contacts"] is None or not record["s1"] or record["baseline"]["class_"] != "success" or mode not in tables["b0"]:
            continue
        context = record_context(record)
        contacts = np.asarray(record["contacts"], dtype=np.float64)
        features, ends, eligible = triple.contact_features(contacts, context, kinematics, velocity)
        probability, distilled, max_z = models["n1"].predict(features)
        distilled = distilled * triple.V2_SCALE_S
        masked = np.where(eligible[:, None], ends, np.nan)
        explicit = {name: triple.contact_downstream(masked, *tables["b0"][mode], velocity, context, model)
                    for name, model in (("b", models["n2"]), ("b0", None))}
        front = triple.time_bound(home, contacts, velocity) + triple.time_bound(contacts, ends, velocity)
        arms = dict(B=(front + distilled + (1 - probability) * triple.C_FAIL_S, np.where(max_z <= triple.Z_LIMIT, probability, 0.)),
                    Bx=(front + explicit["b"] + (1 - probability) * triple.C_FAIL_S, np.where(max_z <= triple.Z_LIMIT, probability, 0.)),
                    B0=(front + explicit["b0"], np.ones(len(contacts))))
        finite = eligible & np.isfinite(explicit["b"])
        if finite.any():
            distillation.append(dict(log_error=float(np.median(np.abs(np.log(distilled[finite]) - np.log(explicit["b"][finite])))),
                                     same_pick=int(np.argmin(np.where(finite, arms["B"][0], np.inf))) == int(np.argmin(np.where(finite, arms["Bx"][0], np.inf)))))
        base_t, base_ms = t_lb(record["baseline"]), record["baseline"]["wall_ms"]
        lazy = record["baseline"].get("ranking") or list(range(len(contacts)))
        output = dict(instance=record["instance"], mode=mode)
        for arm, (values, chance) in arms.items():
            order = triple.ranked(np.where(eligible, values, np.inf), np.where(eligible, chance, 0.), theta)
            if record["kind"] == "dual":
                outcome = analysis.simulate(record, order[:1] if order else lazy, 1, 350.)
            else:
                full = order[:1] + [index for index in range(len(contacts)) if index not in order[:1]]
                outcome = analysis.simulate(record, full, len(contacts), analysis.SINGLE_BUDGET_MS)
            chosen = next((entry for entry in record["s1"] if entry["index"] == outcome["chosen"]), None) if not outcome["fallback"] else record["baseline"]
            realized = t_lb(chosen) if outcome["success"] else None
            output[arm] = dict(abstained=not order, choice=order[0] if order else None, first_try=not outcome["fallback"] and outcome["attempts"] == 1 and outcome["success"],
                               t_lb_vs_baseline=None if realized is None or not base_t else (realized - base_t) / base_t,
                               plan_vs_baseline=(outcome["plan_ms"] - base_ms) / base_ms if base_ms else None,
                               kept_baseline=outcome["chosen"] == record["baseline"].get("selected_index"))
        rows.append(output)
    return rows, distillation


def carry_stage(records, models, tables, velocity, theta=.5):
    rows = []
    for record in records:
        mode = mode_of(record)
        if mode not in tables["b0"]:
            continue
        context = record_context(record)
        for root, pool in (record.get("s2_pools") or {}).items():
            reference = next((entry for entry in record["s1"] if entry["index"] == int(root)), None)
            forced = [entry for entry in record["s2"] if entry["root"] == int(root)]
            if reference is None or reference["class_"] != "success" or not forced or reference.get("reached_goal") is None:
                continue
            down = lambda entry: t_lb(entry, DOWN)
            base_t = down(reference)
            goals = np.asarray(pool["pool"], dtype=np.float64)
            options = [(entry["goal"], down(entry) if entry["class_"] == "success" else None) for entry in forced]
            options.append((reference["reached_goal"], base_t))
            candidates = goals[[goal for goal, _ in options]]
            output = dict(instance=record["instance"], mode=mode, root=int(root), best=(min(t for _, t in options if t is not None) - base_t) / base_t)
            for arm, model in (("B", models["n2"]), ("B0", None)):
                rows_table, values_table = tables["b0"][mode]
                lookup = {row.astype(np.float32).tobytes(): value for row, value in zip(rows_table, values_table)}
                downstream = np.array([lookup.get(row.astype(np.float32).tobytes(), np.nan) for row in candidates])
                if np.isnan(downstream).any():
                    places = np.load(Path(tables["folder"]) / mode / "place_set.npz")["rows"]
                    missing = np.isnan(downstream)
                    downstream[missing], _ = triple.downstream_values(candidates[missing], places, planner_home().astype(np.float64), velocity, context)
                values, probability = triple.carry_values(np.asarray(pool["root"]), candidates, downstream, velocity, context, model)
                order = triple.ranked(values, probability, theta)
                pick = order[0] if order else len(options) - 1
                realized = options[pick][1] if options[pick][1] is not None else base_t + triple.C_FAIL_S
                output[arm] = dict(vs_baseline=(realized - base_t) / base_t, success=options[pick][1] is not None, kept_baseline=pick == len(options) - 1)
            rows.append(output)
    return rows


def summarise(rows, arms, keys):
    output = {}
    for mode in sorted({row["mode"] for row in rows}) + ["all"]:
        mine = [row for row in rows if mode == "all" or row["mode"] == mode]
        output[mode] = dict(n=len(mine), **{arm: {key: (relative([row[arm][key] for row in mine]) if key.endswith("baseline") and key != "kept_baseline"
                                                         else float(np.mean([row[arm][key] for row in mine])))
                                                  for key in keys} for arm in arms})
        if "best" in mine[0]:
            output[mode]["best"] = relative([row["best"] for row in mine])
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--primary", type=Path, required=True)
    parser.add_argument("--models", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    velocity = VELOCITY
    report = json.loads((args.models / "train_v2_report.json").read_text())
    models = {name: triple.ranker.Ensemble.load(args.models / file, report[stage]["model_sha256"], names)
              for name, file, names, stage in (("n1", "n1.npz", triple.CONTACT_NAMES, "n1"), ("n2", "n2.npz", triple.TRANSPORT_NAMES, "n2"),
                                               ("n3", "n3_place.npz", triple.PLACE_NAMES, "n3"))}
    tables = dict(b0=load_tables(args.models, "b0"), folder=args.models / "assets")
    records = [json.loads(path.read_text()) for path in sorted((args.primary / "instances").glob("*.json"))]
    dev = [record for record in records if record["split"] == "dev"]
    contact, distillation = contact_stage(dev, models, tables, velocity)
    carry = carry_stage(dev, models, tables, velocity)
    output = dict(dev_instances=len(dev),
                  contact=summarise(contact, ("B", "Bx", "B0"), ("abstained", "first_try", "t_lb_vs_baseline", "plan_vs_baseline", "kept_baseline")),
                  carry=summarise(carry, ("B", "B0"), ("vs_baseline", "success", "kept_baseline")) if carry else None,
                  place={mode: value for mode, value in ((report.get("n3") or {}).get("choice_by_mode") or {}).items()},
                  distillation=dict(n=len(distillation), log_error_median=float(np.median([row["log_error"] for row in distillation])) if distillation else None,
                                    same_pick=float(np.mean([row["same_pick"] for row in distillation])) if distillation else None),
                  rows=dict(contact=contact, carry=carry))
    args.output.write_text(json.dumps(output, ensure_ascii=False, indent=1, default=float) + "\n")
    print(json.dumps({key: value for key, value in output.items() if key != "rows"}, ensure_ascii=False, indent=1, default=float))


if __name__ == "__main__":
    main()
