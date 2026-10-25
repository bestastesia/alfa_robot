#!/usr/bin/env python3
"""MOTION-311 explicit-V2 equivalence/latency on dev contact sets (GPU lock required).

Uses recorded P3 contacts with the live planner context. Alternates exhaustive top-32 N2 evaluation and exact
lower-bound pruning on identical contacts; neither policy nor model changes. This measures contact selection,
not the entire round: carry/place costs must still be measured online. No held-out test contacts are loaded.
"""
import argparse
import json
import sys
import time
from pathlib import Path
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import v3_triple_ranker as triple
import v3_triple_selection_planner as selection
from v3_goal_pool import direct_goal_pool, exact_goal_pool
from curobo_core.contracts import PlannerAssets
from v3_motion311_ab import ABPlanner, load_models


def exhaustive(ends, rows, downstream, velocity, context, model=None, pool=512, keep=32, near=None):
    ends, rows = np.atleast_2d(np.asarray(ends, dtype=np.float64)), np.asarray(rows, dtype=np.float64)
    downstream = np.asarray(downstream, dtype=np.float64)
    output = np.full(len(ends), np.inf)
    valid = np.flatnonzero(np.isfinite(ends).all(axis=1))
    if not len(valid):
        return output
    near = triple.goal_pool(ends[valid], rows, pool) if near is None else np.asarray(near)
    values = np.max(np.abs(rows[near] - ends[valid][:, None]) / velocity, axis=-1) + downstream[near]
    if model is None:
        output[valid] = values.min(axis=1)
        return output
    keep = min(keep, near.shape[1])
    best = np.take_along_axis(near, np.argsort(values, axis=1, kind="stable")[:, :keep], axis=1)
    q, _ = triple.carry_values(np.repeat(ends[valid], keep, axis=0), rows[best.reshape(-1)], downstream[best.reshape(-1)], velocity, context, model)
    output[valid] = q.reshape(len(valid), keep).min(axis=1)
    return output


def value_delta(first, second):
    """Maximum error over all contacts, including matching infinities for ineligible rows."""
    first, second = np.asarray(first), np.asarray(second)
    if first.shape != second.shape or np.isnan(first).any() or np.isnan(second).any():
        return float("inf")
    finite = np.isfinite(first)
    if not np.array_equal(finite, np.isfinite(second)) or not np.array_equal(first[~finite], second[~finite]):
        return float("inf")
    return float(np.abs(first[finite] - second[finite]).max()) if finite.any() else 0.


class CountModel:
    def __init__(self, model):
        self.model, self.rows = model, 0

    def predict(self, features):
        self.rows += len(features)
        return self.model.predict(features)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--primary", type=Path, required=True)
    parser.add_argument("--models", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=10)
    args = parser.parse_args()
    records = [json.loads(path.read_text()) for path in sorted((args.primary / "instances").glob("*.json"))]
    records = [r for r in records if r["split"] == "dev" and r.get("contacts")]
    planner = ABPlanner(PlannerAssets(), .025)
    models = load_models(args.models)
    counter = CountModel(models["n2"])
    models["n2"] = counter
    planner.setup_selection(args.models / "assets", models)
    planner.arm, planner.contact_value = "B", "explicit"
    pruned = triple.contact_downstream
    rows = []
    for record in records:
        planner.scene_offset = tuple(record["offset_m"])
        planner.configure_task(record["task"])
        contacts = np.asarray(record["contacts"], dtype=np.float64)
        samples, counts, orders, values = {key: [] for key in ("exhaustive", "pruned")}, {}, {}, {}
        same, delta = True, 0.
        for repeat in range(args.repeats + 1):  # first pair is warm-up, excluded from timings
            methods = [("exhaustive", exhaustive, direct_goal_pool), ("pruned", pruned, exact_goal_pool)]
            for name, function, goal_query in methods[::1 if repeat % 2 else -1]:
                counter.rows = 0
                def capture(*positional, **keywords):
                    output = function(*positional, **keywords)
                    values[name] = output.copy()
                    return output
                with patch.object(triple, "contact_downstream", capture), patch.object(selection, "exact_goal_pool", goal_query):
                    order = planner.contact_order(record["task"], contacts)
                report = planner.selection_report["contact"]
                if repeat:
                    samples[name].append(report["ms"])
                counts[name], orders[name] = counter.rows, order
            same = same and orders["exhaustive"] == orders["pruned"]
            difference = value_delta(values["exhaustive"], values["pruned"])
            delta = max(delta, difference)
        row = dict(instance=record["instance"], mode=planner.mode(record["task"]), same_order=same, max_value_delta=delta,
                   evaluated=counts, ms=samples)
        rows.append(row)
        print(json.dumps({key: value for key, value in row.items() if key != "ms"}), flush=True)
    summary = {}
    for mode in sorted({row["mode"] for row in rows}) + ["all"]:
        mine = [r for r in rows if mode == "all" or r["mode"] == mode]
        summary[mode] = {name: dict(ms_p50_p95=np.quantile([t for r in mine for t in r["ms"][name]], [.5, .95]).tolist(),
                                   n2_rows=sum(r["evaluated"][name] for r in mine)) for name in ("exhaustive", "pruned")}
    output = dict(instances=len(rows), same_order=all(r["same_order"] for r in rows),
                  max_value_delta=max(r["max_value_delta"] for r in rows), summary=summary, rows=rows)
    args.output.write_text(json.dumps(output, indent=1) + "\n")
    print(json.dumps({key: value for key, value in output.items() if key != "rows"}), flush=True)
    raise SystemExit(0 if output["same_order"] and output["max_value_delta"] < 1e-10 else 1)


if __name__ == "__main__":
    main()
