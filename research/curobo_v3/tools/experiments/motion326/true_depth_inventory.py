#!/usr/bin/env python3
"""MOTION-326 step 2c: per-run true penetration depths for every audited run with contacts.

Frames inside each run's reported contact ranges are sampled with a stride; every colliding pair
found by the all-pairs FCL check is measured with exact signed distances (FCL BVH depths are not
true depths). Output: one record per (run, pair) with the largest true depth, plus a summary per
set, arm and task kind at several severity thresholds.
"""
import argparse
import collections
import json
from pathlib import Path
import sys

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from curobo_core.scene import SceneSnapshot
from experiments.motion326.contact_points import AllContactsChecker
from experiments.motion326.miss_inventory import contact_class, task_kind
from experiments.motion326.verify_depth import penetration, shape_mesh
from v3_research_assets import ROOT

THRESHOLDS_MM = (0., 2., 5., 10.)


def run_depths(checker, names, advance, frames, ranges, stride, samples, cache):
    best = {}
    wanted = sorted({index for first, last in ranges for index in list(range(first, last + 1, stride)) + [last]})
    for index in wanted:
        positions = dict(zip(names, frames[index]))
        positions["base_x"] -= advance
        found = checker.contacts(positions, attached=True, payload_enabled=True, max_contacts=1)
        if not found:
            continue
        links = {name: checker.robot.get_transform(name, checker.robot.base_link) for name in {s["link"] for s in checker.shapes if s["link"]}}
        for pair in {" | ".join(sorted(side["name"] for side in contact["sides"])) for contact in found}:
            indices = [next(i for i, s in enumerate(checker.shapes) if f"{s['kind']}:{s['name']}" == name) for name in pair.split(" | ")]
            meshes = [shape_mesh(checker, i, checker.shapes[i]["origin"] if checker.shapes[i]["link"] is None
                                 else links[checker.shapes[i]["link"]] @ checker.shapes[i]["origin"], cache) for i in indices]
            depth = penetration(*meshes, samples=samples) * 1000
            if depth > best.get(pair, (-1, None))[0]:
                best[pair] = (depth, index)
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--set", nargs=2, action="append", metavar=("LABEL", "RUN_DIR"), required=True)
    parser.add_argument("--stride", type=int, default=8)
    parser.add_argument("--samples", type=int, default=1500)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = yaml.safe_load((ROOT / "generated/v3_analytic_071cb95/mobile18/alfa_v322_suction_mobile18.yml").read_text())["kinematics"]
    args.output.mkdir(parents=True, exist_ok=True)
    records_path = args.output / "true-depth-records.jsonl"
    done = {json.loads(line)["key"] for line in records_path.read_text().splitlines()} if records_path.exists() else set()
    cache = {}
    with records_path.open("a") as sink:
        for label, directory in args.set:
            directory = Path(directory)
            for run in json.loads((directory / "fcl-audit.json").read_text())["runs"]:
                key = f"{label}/{run['run']}"
                if key in done:
                    continue
                record = dict(key=key, set=label, run=run["run"], arm=run["run"].split("-")[-1], kind=task_kind(run["run"]), pairs={})
                if run["contacts"]:
                    result = json.loads((directory / "full" / f"{run['run']}.json").read_text())
                    event = next(item for item in result["predicted_scene_events"] if item["phase"] == "attach")
                    snapshot = SceneSnapshot.from_dict(event["snapshot"])
                    checker = AllContactsChecker(config, snapshot, dict(result["task"]), snapshot.attachments)
                    frames = [row for row, phase in zip(result["frames"], result["phases"]) if phase == "transport"]
                    ranges = [(c["first_frame"], c["last_frame"]) for c in run["contacts"].values()]
                    best = run_depths(checker, result["joint_names"], float(result.get("base_advance_m", 0.)), frames,
                                      ranges, args.stride, args.samples, cache)
                    record["pairs"] = {pair: dict(true_depth_mm=depth, frame=frame, cls=contact_class(pair))
                                       for pair, (depth, frame) in best.items()}
                sink.write(json.dumps(record) + "\n")
                sink.flush()
                print(json.dumps({"key": key, "pairs": {p: round(v["true_depth_mm"], 1) for p, v in record["pairs"].items()}}), flush=True)
    records = [json.loads(line) for line in records_path.read_text().splitlines() if line.strip()]
    summary = collections.defaultdict(lambda: dict(runs=0, depths=[], classes=collections.defaultdict(list)))
    for record in records:
        group = summary[(record["set"], record["arm"], record["kind"])]
        group["runs"] += 1
        group["depths"].append(max((p["true_depth_mm"] for p in record["pairs"].values()), default=0.))
        for pair, value in record["pairs"].items():
            group["classes"][value["cls"]].append(value["true_depth_mm"])
    table = []
    for (label, arm, kind), group in sorted(summary.items()):
        depths = np.asarray(group["depths"])
        table.append(dict(set=label, arm=arm, kind=kind, runs=group["runs"],
                          runs_over_mm={str(t): int((depths > t).sum()) for t in THRESHOLDS_MM},
                          max_true_depth_mm=float(depths.max()) if len(depths) else 0.,
                          classes={c: dict(n=len(v), max=float(max(v))) for c, v in group["classes"].items()}))
    (args.output / "true-depth-summary.json").write_text(json.dumps(table, ensure_ascii=False, indent=1) + "\n")
    for row in table:
        print(f"{row['set']:8s} {row['arm']:20s} {row['kind']:4s} runs {row['runs']:3d} | >0/2/5/10 mm: "
              f"{'/'.join(str(row['runs_over_mm'][str(t)]) for t in THRESHOLDS_MM)} | max {row['max_true_depth_mm']:.1f} mm | "
              + ", ".join(f"{c} {v['n']}×≤{v['max']:.1f}" for c, v in row["classes"].items()))


if __name__ == "__main__":
    main()
