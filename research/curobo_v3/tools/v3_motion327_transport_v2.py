#!/usr/bin/env python3
"""MOTION-327 A/B: Cartesian transport search v2 vs the joint RRT baseline on docking-offset scenes.

Reuses the MOTION-323 harness (run_once / summarize / audit). Arms share one planner instance:
  joint_rrt            baseline
  cartesian_v2_toppra  v2 search (shortcut, monotonic progress, strict budget) + fixed TOPP-RA (primary)
  cartesian_v2         v2 search with the pipeline timing (secondary)
Scenes: the dev set uses the 25 nominal single-arm scenes; the independent test set shifts the
container, fillers and target boxes by a pre-registered (dx, dy) relative to the robot.
"""
import argparse
import json
from dataclasses import replace
from pathlib import Path

import yourdfpy

from curobo_core.contracts import PlannerAssets
from v3_motion312_transport_ab import HarnessPlanner, audit, run_once, summarize, task_name
from v3_research_assets import ROOT

CONFIG = ROOT / "config/motion327_transport_v2.json"
FROZEN_V1 = ROOT / "config/motion312_transport_ab.json"
V2_FLAGS = dict(shortcut=True, monotonic_progress=True, strict_budget=True)
ARMS = {"joint_rrt": ("joint_rrt", False), "cartesian_v2_toppra": ("cartesian_toppra", True),
        "cartesian_v2": ("cartesian", True)}
SHIFTED_IDS = {"ground", "left_wall", "right_wall", "front_wall", "ceiling"}


class DockingOffsetPlanner(HarnessPlanner):
    """Shifts the pick-side scene by scene_offset (m) after every task configuration."""

    scene_offset = (0.0, 0.0)

    def configure_task(self, boxes):
        result = super().configure_task(boxes)
        dx, dy = self.scene_offset
        if dx or dy:
            objects = tuple(
                replace(item, pose=replace(item.pose, position=(item.pose.position[0] + dx, item.pose.position[1] + dy,
                                                                item.pose.position[2])))
                if item.object_id in SHIFTED_IDS or item.object_id.startswith(("wall_box_", "task_fill_")) else item
                for item in self.snapshot.objects)
            self.snapshot = replace(self.snapshot, objects=objects, revision=self.snapshot.revision + 1)
        return result


def instances(config, split):
    if split == "dev":
        return [(task_name(task), task, (0.0, 0.0)) for task in config["dev_set"]["tasks"]], config["dev_set"]["seeds"]
    rows, counts = [], {}
    for item in config["test_set"]["instances"]:
        name = task_name(item["task"])
        counts[name] = counts.get(name, -1) + 1
        rows.append((f"{name}-o{counts[name]}", item["task"], tuple(item["offset_m"])))
    return rows, config["test_set"]["seeds"]


def run(args):
    config = json.loads(CONFIG.read_text())
    v1_options = json.loads(FROZEN_V1.read_text())["cartesian_options_frozen"]
    v2_options = {**v1_options, **V2_FLAGS, **json.loads(args.cartesian_options or "{}")}
    rows, seeds = instances(config, args.split)
    seeds = [int(seed) for seed in args.seeds.split(",")] if args.seeds else seeds
    arms = args.arms.split(",")
    runs, full = args.output / "runs", args.output / "full"
    runs.mkdir(parents=True, exist_ok=True)
    full.mkdir(exist_ok=True)
    assets = PlannerAssets()
    planner = DockingOffsetPlanner(assets, cartesian_options=v2_options)
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
                base_arm, v2 = ARMS[arm]
                planner.cartesian_options = v2_options if v2 else v1_options
                record, result = run_once(planner, model, boxes, base_arm, seed)
                record.update(arm=arm, task=name, base_task=task_name(boxes), scene_offset_m=list(offset), order=order,
                              cartesian_options=planner.cartesian_options)
                path.write_text(json.dumps(record, ensure_ascii=False, default=float) + "\n")
                if result is not None and (args.keep_full == "all" or (args.keep_full == "first" and seed_index == 0)):
                    (full / path.name).write_text(json.dumps(result, ensure_ascii=False) + "\n")
                print(json.dumps({key: record.get(key) for key in ("task", "arm", "seed", "success", "failure_class")}
                                 | {"search_ms": (record.get("planning") or {}).get("search_ms"),
                                    "variant": ((planner.last_transport_stats or {}).get("path_variant")),
                                    "extra_rot": (record.get("executed") or {}).get("extra_rotation_deg"),
                                    "duration": (record.get("executed") or {}).get("transport_duration_s"),
                                    "valid": (record.get("safety") or {}).get("validated")}, ensure_ascii=False, default=float),
                      flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    runner = commands.add_parser("run")
    runner.add_argument("--split", choices=("dev", "test"), required=True)
    runner.add_argument("--arms", default=",".join(ARMS))
    runner.add_argument("--seeds", help="comma separated; default: pre-registered seeds of the split")
    runner.add_argument("--keep-full", choices=("none", "first", "all"), default="first")
    runner.add_argument("--cartesian-options", help="JSON overrides applied on top of the v2 options (dev tuning only)")
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
