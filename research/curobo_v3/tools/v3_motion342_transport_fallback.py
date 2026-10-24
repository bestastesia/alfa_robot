#!/usr/bin/env python3
"""MOTION-342 A/B: the v2 Cartesian transport search with the joint-RRT fallback, with and without
routing branch-changing stages to the joint RRT, against the baseline on a new docking-offset test set.

Arms share one planner instance (pre-registered in config/motion342_transport_fallback.json):
  joint_rrt              baseline
  cartesian_v21_toppra   v2 + fallback_joint_rrt, TOPP-RA timing
  cartesian_v21r_toppra  v2 + fallback_joint_rrt + route_branch_change, TOPP-RA timing
`summarize` runs the MOTION-323 summary (pre-registered rules per variant) and adds the trade-off
report: execution-duration tail against the paired baseline, wrist branch flips and singularity
margins, extra rotation, and the tier that produced each transport.
"""
import argparse
import collections
import json
from pathlib import Path

import numpy as np
import yourdfpy

from curobo_core.contracts import PlannerAssets
from v3_motion312_transport_ab import audit, run_once, summarize as summarize_ab, task_name
from v3_motion327_transport_v2 import FROZEN_V1, V2_FLAGS, DockingOffsetPlanner
from v3_research_assets import ROOT

CONFIG = ROOT / "config/motion342_transport_fallback.json"
ARMS = {"joint_rrt": ("joint_rrt", None),
        "cartesian_v21_toppra": ("cartesian_toppra", dict(fallback_joint_rrt=True)),
        "cartesian_v21r_toppra": ("cartesian_toppra", dict(fallback_joint_rrt=True, route_branch_change=True))}


def instances(config):
    rows, counts = [], {}
    for item in config["test_set"]["instances"]:
        name = task_name(item["task"])
        counts[name] = counts.get(name, -1) + 1
        rows.append((f"{name}-o{counts[name]}", item["task"], tuple(item["offset_m"])))
    return rows, config["test_set"]["seeds"]


def run(args):
    config = json.loads(CONFIG.read_text())
    v1_options = json.loads(FROZEN_V1.read_text())["cartesian_options_frozen"]
    rows, seeds = instances(config)
    seeds = [int(seed) for seed in args.seeds.split(",")] if args.seeds else seeds
    arms = args.arms.split(",")
    runs, full = args.output / "runs", args.output / "full"
    runs.mkdir(parents=True, exist_ok=True)
    full.mkdir(exist_ok=True)
    assets = PlannerAssets()
    planner = DockingOffsetPlanner(assets, cartesian_options={**v1_options, **V2_FLAGS})
    planner.warmup_resources()
    model = yourdfpy.URDF.load(assets.mobile_robot_config.parent / "model/robot_meter_meshes_mobile18.urdf", load_meshes=False)
    for index, (name, boxes, offset) in enumerate(rows):
        planner.scene_offset = offset
        for seed_index, seed in enumerate(seeds):
            order = arms if (index + seed_index) % 2 == 0 else arms[::-1]
            for arm in order:
                path = runs / f"{name}-s{seed}-{arm}.json"
                if path.exists():
                    continue
                base_arm, extra = ARMS[arm]
                planner.cartesian_options = {**v1_options, **V2_FLAGS, **extra} if extra is not None else v1_options
                record, result = run_once(planner, model, boxes, base_arm, seed)
                record.update(arm=arm, task=name, base_task=task_name(boxes), scene_offset_m=list(offset), order=order,
                              cartesian_options=planner.cartesian_options)
                path.write_text(json.dumps(record, ensure_ascii=False, default=float) + "\n")
                if result is not None and (args.keep_full == "all" or (args.keep_full == "first" and seed_index == 0)):
                    (full / path.name).write_text(json.dumps(result, ensure_ascii=False) + "\n")
                print(json.dumps({key: record.get(key) for key in ("task", "arm", "seed", "success", "failure_class")}
                                 | {"tier": (record.get("planning") or {}).get("tier"),
                                    "search_ms": (record.get("planning") or {}).get("search_ms"),
                                    "extra_rot": (record.get("executed") or {}).get("extra_rotation_deg"),
                                    "duration": (record.get("executed") or {}).get("transport_duration_s"),
                                    "valid": (record.get("safety") or {}).get("validated")}, ensure_ascii=False, default=float),
                      flush=True)


def tradeoff(records):
    by_key = {(record["task"], record["seed"], record["arm"]): record for record in records}
    report = {}
    for arm in sorted({record["arm"] for record in records}):
        rows = [record for record in records if record["arm"] == arm and record.get("success")]
        executed = [record["executed"] for record in rows]
        ratios = [record["executed"]["transport_duration_s"] / base["executed"]["transport_duration_s"]
                  for record in rows for base in [by_key.get((record["task"], record["seed"], "joint_rrt"))]
                  if arm != "joint_rrt" and base and base.get("success")]
        margins = [item["min_singularity_margin_deg"] for item in executed if item.get("min_singularity_margin_deg") is not None]
        report[arm] = dict(
            successful_runs=len(rows),
            tiers=dict(collections.Counter(str((record.get("planning") or {}).get("tier")) for record in rows)),
            runs_with_branch_flip=sum(bool(item.get("branch_flips")) for item in executed),
            singularity_margin_deg=dict(median=float(np.median(margins)), min=float(np.min(margins))) if margins else None,
            extra_rotation_deg_median={kind: float(np.median(values)) if values else None for kind, values in (
                ("side", [item["extra_rotation_deg"] for item in executed if not item["top_grasp"]]),
                ("top", [item["extra_rotation_deg"] for item in executed if item["top_grasp"]]))},
            duration_vs_baseline=dict(pairs=len(ratios), median=float(np.median(ratios)), p90=float(np.percentile(ratios, 90)),
                                      max=float(np.max(ratios)), over_2x=sum(ratio > 2 for ratio in ratios),
                                      over_3x=sum(ratio > 3 for ratio in ratios)) if ratios else None)
    return report


def summarize(args):
    summarize_ab(args)
    output = args.output or args.input / "summary.json"
    summary = json.loads(output.read_text())
    summary["tradeoff"] = tradeoff([json.loads(path.read_text()) for path in sorted((args.input / "runs").glob("*.json"))])
    output.write_text(json.dumps(summary, ensure_ascii=False, indent=1, default=float) + "\n")
    print(json.dumps(summary["tradeoff"], ensure_ascii=False, indent=1, default=float))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    runner = commands.add_parser("run")
    runner.add_argument("--arms", default=",".join(ARMS))
    runner.add_argument("--seeds", help="comma separated; default: pre-registered seeds")
    runner.add_argument("--keep-full", choices=("none", "first", "all"), default="first")
    runner.add_argument("--output", type=Path, required=True)
    for command in ("summarize", "audit"):
        sub = commands.add_parser(command)
        sub.add_argument("--input", type=Path, required=True)
        if command == "summarize":
            sub.add_argument("--output", type=Path)
    args = parser.parse_args()
    {"run": run, "summarize": summarize, "audit": audit}[args.command](args)


if __name__ == "__main__":
    main()
