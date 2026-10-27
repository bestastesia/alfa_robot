#!/usr/bin/env python3
"""MOTION-363 A/B: the MOTION-342 B form with the controlled wrist flip, against B and the baseline.

Arms share one planner instance (pre-registered in config/motion363_wrist_flip.json):
  joint_rrt              baseline
  cartesian_v21r_toppra  MOTION-342 B: v2 + fallback_joint_rrt + route_branch_change, TOPP-RA timing
  cartesian_v22_toppra   B + wrist_flip
Splits: `dev` = the MOTION-342 test scenes that B routed (seeds 342001-3, design data); `test` = the
new docking-offset instances of this config. `summarize` runs the MOTION-323 summary (pre-registered
rules per variant), the MOTION-342 trade-off report, and classifies every wrist branch flip of the
executed transport as controlled (tool at rest) or moving.
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
from v3_motion342_transport_fallback import CONFIG as CONFIG_342, instances as instances_342, tradeoff
from v3_research_assets import ROOT

CONFIG = ROOT / "config/motion363_wrist_flip.json"
B_OPTIONS = dict(fallback_joint_rrt=True, route_branch_change=True)
ARMS = {"joint_rrt": ("joint_rrt", None),
        "cartesian_v21r_toppra": ("cartesian_toppra", B_OPTIONS),
        "cartesian_v22_toppra": ("cartesian_toppra", dict(B_OPTIONS, wrist_flip=True))}
REST_LINEAR_MM_S, REST_ANGULAR_DEG_S = 5., 2.  # tool speeds below these count as "at rest" at a flip


def split_rows(config, split):
    if split == "test":
        rows, counts = [], {}
        for item in config["test_set"]["instances"]:
            name = task_name(item["task"])
            counts[name] = counts.get(name, -1) + 1
            rows.append((f"{name}-o{counts[name]}", item["task"], tuple(item["offset_m"])))
        return rows, config["test_set"]["seeds"]
    rows, seeds = instances_342(json.loads(CONFIG_342.read_text()))
    wanted = set(config["dev_set"]["scenes"])
    return [row for row in rows if row[0] in wanted], config["dev_set"]["seeds"]


def run(args):
    config = json.loads(CONFIG.read_text())
    v1_options = json.loads(FROZEN_V1.read_text())["cartesian_options_frozen"]
    rows, seeds = split_rows(config, args.split)
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
                stats = planner.last_transport_stats or {}
                record.update(arm=arm, task=name, base_task=task_name(boxes), scene_offset_m=list(offset), order=order,
                              cartesian_options=planner.cartesian_options,
                              wrist_flip={key: stats.get(key) for key in ("swing_q5_deg", "swing_q7_deg", "progress", "swivel_deg",
                                                                         "lift_offset_m", "position_offset_m", "offsets", "goal",
                                                                         "budget_s", "attempts", "timing_ms", "search_ms")}
                              if stats.get("tier") == "wrist_flip"
                              else (stats.get("wrist_flip") or None))
                path.write_text(json.dumps(record, ensure_ascii=False, default=float) + "\n")
                if result is not None and (args.keep_full == "all" or (args.keep_full == "first" and seed_index == 0)):
                    (full / path.name).write_text(json.dumps(result, ensure_ascii=False) + "\n")
                print(json.dumps({key: record.get(key) for key in ("task", "arm", "seed", "success", "failure_class")}
                                 | {"tier": (record.get("planning") or {}).get("tier"),
                                    "search_ms": (record.get("planning") or {}).get("search_ms"),
                                    "extra_rot": (record.get("executed") or {}).get("extra_rotation_deg"),
                                    "duration": (record.get("executed") or {}).get("transport_duration_s"),
                                    "timing": (record.get("executed") or {}).get("transport_parameterizer"),
                                    "valid": (record.get("safety") or {}).get("validated")}, ensure_ascii=False, default=float),
                      flush=True)


def flips(result, model):
    """Wrist branch flips (q6 sign changes) of the executed transport, with the tool speed at each flip."""
    names, phases = result["joint_names"], result["phases"]
    index = [i for i, phase in enumerate(phases) if phase == "transport"]
    side = next(iter(result["task"]))
    q6 = np.asarray(result["frames"])[index, names.index(f"{side}_joint6")]
    times = np.asarray(result.get("time_from_start_s") or np.arange(len(phases)) * .025)[index]
    found = []
    for k in np.flatnonzero(np.sign(q6[:-1]) * np.sign(q6[1:]) < 0):
        poses = []
        for frame in (index[k], index[k + 1]):
            model.update_cfg({name: value for name, value in zip(names, result["frames"][frame]) if name in model.actuated_joint_names})
            poses.append(model.get_transform(f"{side}_tool0", model.base_link))
        dt = max(float(times[k + 1] - times[k]), 1e-6)
        relative = poses[0][:3, :3].T @ poses[1][:3, :3]
        angle = np.degrees(np.arccos(np.clip((np.trace(relative) - 1) / 2, -1., 1.)))
        found.append(dict(frame=int(k), linear_mm_s=float(np.linalg.norm(poses[1][:3, 3] - poses[0][:3, 3]) / dt * 1000),
                          angular_deg_s=float(angle / dt)))
    for item in found:
        item["at_rest"] = item["linear_mm_s"] <= REST_LINEAR_MM_S and item["angular_deg_s"] <= REST_ANGULAR_DEG_S
    return found


def flip_report(directory, records):
    model = yourdfpy.URDF.load(PlannerAssets().mobile_robot_config.parent / "model/robot_meter_meshes_mobile18.urdf",
                               load_meshes=False)
    report = {}
    for arm in sorted({record["arm"] for record in records}):
        counts, routed = collections.Counter(), collections.Counter()
        for record in records:
            if record["arm"] != arm or not record.get("success"):
                continue
            path = directory / "full" / f"{record['task']}-s{record['seed']}-{arm}.json"
            if not path.exists():
                counts["no_full_result"] += 1
                continue
            items = flips(json.loads(path.read_text()), model)
            counts["runs"] += 1
            counts["runs_with_flip"] += bool(items)
            counts["flips_at_rest"] += sum(item["at_rest"] for item in items)
            counts["flips_moving"] += sum(not item["at_rest"] for item in items)
            tier = str((record.get("planning") or {}).get("tier"))
            if tier in ("wrist_flip", "joint_rrt_routed"):
                routed[tier] += 1
        report[arm] = dict(counts, branch_change_tiers=dict(routed))
    return report


def summarize(args):
    summarize_ab(args)
    output = args.output or args.input / "summary.json"
    summary = json.loads(output.read_text())
    records = [json.loads(path.read_text()) for path in sorted((args.input / "runs").glob("*.json"))]
    summary["tradeoff"] = tradeoff(records)
    summary["wrist_flips"] = flip_report(args.input, records)
    output.write_text(json.dumps(summary, ensure_ascii=False, indent=1, default=float) + "\n")
    print(json.dumps({key: summary[key] for key in ("tradeoff", "wrist_flips")}, ensure_ascii=False, indent=1, default=float))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    runner = commands.add_parser("run")
    runner.add_argument("--split", choices=("dev", "test"), required=True)
    runner.add_argument("--arms", default=",".join(ARMS))
    runner.add_argument("--seeds", help="comma separated; default: the split's seeds")
    runner.add_argument("--keep-full", choices=("none", "first", "all"), default="all")
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
