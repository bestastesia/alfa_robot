#!/usr/bin/env python3
"""MOTION-326 step 3c: are the frames a sphere patch rejects really in mesh contact?

For runs where appended spheres reject frames (patch_eval.py check/check-cpu output), evenly spaced
rejected frames are re-checked: the CPU sphere replica names the pairs that overlap in that frame
(patched link vs link, payload box or world object), and each pair is checked against the exact
meshes. A mesh contact counts as a true rejection and its true depth is measured with signed
distances; otherwise the mesh clearance shows how conservative the patch is.
"""
import argparse
import collections
import json
from pathlib import Path
import sys

import fcl
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from curobo_core.contracts import PlannerAssets
from curobo_core.scene import SceneSnapshot
from experiments.motion326.contact_points import AllContactsChecker
from experiments.motion326.cpu_spheres import ChainFK, appended_collisions, sphere_groups, world, world_cuboids
from experiments.motion326.verify_depth import penetration, shape_mesh
from v3_research_assets import ROOT

PAYLOAD_PHASES = {"transport"}


def shape_indices(checker, label):
    """Shape indices of an attribution label: robot link, 'payload:<side>' or 'world:<object>'."""
    if label.startswith("payload:"):
        return [i for i, s in enumerate(checker.shapes) if s["kind"] == "payload" and s["link"].startswith(label[8:])]
    if label.startswith("world:"):
        return [i for i, s in enumerate(checker.shapes) if s["kind"] == "world" and s["name"] == label[6:]]
    return [i for i, s in enumerate(checker.shapes) if s["kind"] == "robot" and s["link"] == label]


def measure(checker, cache, first, second):
    """Largest true depth over colliding mesh pairs, or the smallest clearance (negative) if none collide."""
    links = {name: checker.robot.get_transform(name, checker.robot.base_link) for name in {s["link"] for s in checker.shapes if s["link"]}}
    frame = lambda i: checker.shapes[i]["origin"] if checker.shapes[i]["link"] is None else links[checker.shapes[i]["link"]] @ checker.shapes[i]["origin"]
    depth, clearance = None, np.inf
    for i in first:
        for j in second:
            request = fcl.CollisionRequest(num_max_contacts=1, enable_contact=True)
            result = fcl.CollisionResult()
            if fcl.collide(checker.shapes[i]["object"], checker.shapes[j]["object"], request, result):
                value = penetration(shape_mesh(checker, i, frame(i), cache), shape_mesh(checker, j, frame(j), cache), samples=1500)
                depth = max(depth or 0., value)
            else:
                distance = fcl.DistanceResult()
                fcl.distance(checker.shapes[i]["object"], checker.shapes[j]["object"], fcl.DistanceRequest(), distance)
                clearance = min(clearance, distance.min_distance)
    return (depth * 1000, None) if depth is not None else (None, clearance * 1000)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--set", nargs=2, action="append", metavar=("LABEL", "RUN_DIR"), required=True)
    parser.add_argument("--checked", type=Path, required=True, help="patched check JSONL")
    parser.add_argument("--patch", type=Path, required=True, help="patch.json the check used")
    parser.add_argument("--per-run", type=int, default=4, help="rejected frames re-checked per run and phase")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    directories = {label: Path(path) for label, path in args.set}
    config = yaml.safe_load((ROOT / "generated/v3_analytic_071cb95/mobile18/alfa_v322_suction_mobile18.yml").read_text())["kinematics"]
    fixed = yaml.safe_load(PlannerAssets().robot_config.read_text())["kinematics"]
    box_fit = json.loads(PlannerAssets().box_fit.read_text())
    added = {link: value["added_spheres"] for link, value in json.loads(args.patch.read_text())["links"].items()}
    ignored = {frozenset((a, b)) for a, links in fixed["self_collision_ignore"].items() for b in links}
    records, cache = [], {}
    for entry in map(json.loads, args.checked.read_text().splitlines()):
        flagged = {phase: [i for a, b in value["invalid"] for i in range(a, b + 1)] for phase, value in entry["phases"].items()}
        if not any(flagged.values()):
            continue
        result = json.loads((directories[entry["set"]] / "full" / f"{entry['run']}.json").read_text())
        event = next(item for item in result["predicted_scene_events"] if item["phase"] == "attach")
        snapshot = SceneSnapshot.from_dict(event["snapshot"])
        checker = AllContactsChecker(config, snapshot, dict(result["task"]), snapshot.attachments)
        fk = ChainFK(checker.robot)
        names, advance = result["joint_names"], float(result.get("base_advance_m", 0.))
        base = {**checker.locked, **dict(zip(snapshot.state.joint_names, snapshot.state.positions))}
        for phase, frames in flagged.items():
            if not frames:
                continue
            rows = [row for row, name in zip(result["frames"], result["phases"]) if name == phase]
            payload = phase in PAYLOAD_PHASES
            groups = sphere_groups(fixed, box_fit, snapshot.attachments if payload else (), added)
            for index in sorted({frames[k] for k in np.linspace(0, len(frames) - 1, min(args.per_run, len(frames))).round().astype(int)}):
                positions = dict(zip(names, rows[index]))
                positions["base_x"] -= advance
                values = {**base, **positions}
                _, pairs = appended_collisions(groups, world(groups, fk.transforms(values, 1)), ignored, world_cuboids(snapshot),
                                               snapshot.policy.ground_z_m)
                checker.contacts(positions, attached=payload, payload_enabled=payload, max_contacts=1)  # sets transforms
                for pair, overlap in pairs.items():
                    link, other = pair.split(" | ")
                    depth, clearance = measure(checker, cache, shape_indices(checker, link), shape_indices(checker, other))
                    records.append(dict(set=entry["set"], run=entry["run"], arm=entry["arm"], kind=entry["kind"], phase=phase,
                                        frame=int(index), pair=pair, sphere_overlap_mm=overlap["max_overlap_mm"],
                                        true_depth_mm=depth, clearance_mm=clearance))
        print(entry["set"], entry["run"], {p: len(f) for p, f in flagged.items() if f}, flush=True)
    args.output.write_text("\n".join(json.dumps(record) for record in records) + "\n")
    summary = collections.defaultdict(lambda: dict(contact=0, clear=0, depths=[], clearances=[]))
    for record in records:
        group = summary[(record["phase"], "joint_rrt" if record["arm"] == "joint_rrt" else "cartesian", record["kind"], record["pair"])]
        if record["true_depth_mm"] is not None:
            group["contact"] += 1
            group["depths"].append(record["true_depth_mm"])
        else:
            group["clear"] += 1
            group["clearances"].append(record["clearance_mm"])
    for key, group in sorted(summary.items()):
        depths, clearances = np.asarray(group["depths"]), np.asarray(group["clearances"])
        print(f"{key[0]:16s} {key[1]:10s} {key[2]:4s} {key[3]:34s} | mesh contact {group['contact']:3d}"
              + (f" (true depth median {np.median(depths):5.1f} max {depths.max():5.1f} mm)" if len(depths) else "")
              + f" | clear {group['clear']:3d}" + (f" (clearance median {np.median(clearances):5.1f} max {clearances.max():5.1f} mm)" if len(clearances) else ""))


if __name__ == "__main__":
    main()
