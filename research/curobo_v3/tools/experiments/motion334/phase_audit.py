#!/usr/bin/env python3
"""MOTION-334 step 1: collision-mesh audit of the executed frames (by default outside transport).

For every audited successful cycle of the given sets (the runs in each set's fcl-audit.json), the
frames of the non-transport phases are checked against the collision meshes with the all-pairs FCL
check of MOTION-326 (contact_points.AllContactsChecker). Everything is evaluated in the attach
snapshot (map frame, fixed-base contract): the base joints of each frame carry the base pose, minus
the top-suction base advance, so the conveyor phases (base turned round and moved back) share the
same world. Payload per phase as the planner checks it: none in approach, contact_approach and
return_home; attached in extract and conveyor_place. In extract the planner checks the box only at
the end of the extraction (defer_payload_until_extract_end), so payload contacts there are reported
apart from the model misses. Before contact the planner leaves the task boxes out of its world but
checks them on their own (TargetContactValidity: no sphere inside the box in approach; in
contact_approach only the suction spheres may touch it at the contact), so approach and
contact_approach add the task boxes as obstacles at their pick poses (the box pose at the attach
frame) and class those contacts as robot-target. For
every phase and pair with contacts, frames inside the contact ranges are sampled with a stride and
measured with exact signed distances (FCL BVH depths are not penetration depths on non-convex
meshes). Phase frames repeated across arms (same task, offset and seed) are evaluated once. Sets
without fcl-audit.json (MOTION-334 re-runs) use every successful run in full/; --phases can add
transport (payload attached).

  run        audit one shard of the runs (shards are split by scene, so repeated frames meet)
  summarize  merge the shards: per phase, contact class, baseline/Cartesian (or set label, --by-set)
             and task kind, the runs over each true-depth threshold, the deepest pairs, and the base
             contacts as extra frames for base_attribution.py
"""
import argparse
import collections
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import zlib

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from curobo_core.adapter import matrix_pose, pose_matrix
from curobo_core.scene import SceneObject, SceneSnapshot
from experiments.motion326.contact_points import AllContactsChecker
from experiments.motion326.miss_inventory import contact_class, task_kind
from experiments.motion326.verify_depth import penetration, shape_mesh
from v3_research_assets import ROOT

# phase: (box attached, payload checked, task boxes as obstacles at their pick poses)
PHASES = {"approach": (False, False, True), "contact_approach": (False, False, True), "extract": (True, True, False),
          "transport": (True, True, False), "conveyor_place": (True, True, False), "return_home": (False, False, False)}
DEFAULT_PHASES = ("approach", "contact_approach", "extract", "conveyor_place", "return_home")
THRESHOLDS_MM = (0., 2., 5., 10.)
BASELINE_ARMS = ("joint_rrt",)


def scene_key(run):
    """Run name without the arm: task[-offset]-seed."""
    return run.rsplit("-", 1)[0]


def run_names(directory):
    if (directory / "fcl-audit.json").exists():
        return [run["run"] for run in json.loads((directory / "fcl-audit.json").read_text())["runs"]]
    return [path.stem for path in sorted((directory / "runs").glob("*.json"))
            if json.loads(path.read_text()).get("success") and (directory / "full" / path.name).exists()]


def jobs(sets, shard, shards):
    out = []
    for label, directory in sets:
        directory = Path(directory)
        for name in run_names(directory):
            key = f"{label}/{scene_key(name)}"
            if zlib.crc32(key.encode()) % shards == shard:
                out.append((label, directory, name))
    return sorted(out, key=lambda job: (job[0], scene_key(job[2]), job[2]))


def shape_index(checker, name):
    return next(i for i, shape in enumerate(checker.shapes) if f"{shape['kind']}:{shape['name']}" == name)


def true_depth(checker, rows, pair, attached, payload, samples, cache):
    """Largest exact penetration (mm) of one pair over the given joint states, and the row where it occurs."""
    indices = [shape_index(checker, name) for name in pair.split(" | ")]
    best = (0., None)
    for row, positions in rows:
        checker.contacts(positions, attached=attached, payload_enabled=payload, max_contacts=1)  # updates the poses
        meshes = []
        for index in indices:
            shape = checker.shapes[index]
            transform = shape["origin"] if shape["link"] is None else (
                checker.robot.get_transform(shape["link"], checker.robot.base_link) @ shape["origin"])
            meshes.append(shape_mesh(checker, index, transform, cache))
        depth = penetration(*meshes, samples=samples) * 1000
        if depth > best[0]:
            best = (depth, row)
    return best


def audit_phase(checker, result, indices, attached, payload, args, cache):
    names, frames = result["joint_names"], result["frames"]
    advance = float(result.get("base_advance_m", 0.))

    def positions(local):
        values = dict(zip(names, frames[indices[local]]))
        values["base_x"] -= advance
        return values

    hits, fcl_depth = collections.defaultdict(list), collections.defaultdict(float)
    for local in range(0, len(indices), args.stride):
        for contact in checker.contacts(positions(local), attached=attached, payload_enabled=payload, max_contacts=1):
            pair = " | ".join(sorted(side["name"] for side in contact["sides"]))
            if not hits[pair] or hits[pair][-1] != local:
                hits[pair].append(local)
            fcl_depth[pair] = max(fcl_depth[pair], contact["depth_mm"])
    pairs = {}
    for pair, locals_ in hits.items():
        sampled = sorted(set(locals_[::args.depth_stride]) | {locals_[-1]})
        depth, at = true_depth(checker, [(local, positions(local)) for local in sampled], pair, attached, payload,
                               args.samples, cache)
        target = any(side in {f"world:{name}" for name in checker.tasks} for side in pair.split(" | "))
        pairs[pair] = dict(frames=len(locals_), first=locals_[0], last=locals_[-1], fcl_depth_mm=fcl_depth[pair],
                           true_depth_mm=depth, true_frame=at, cls="robot-target" if target else contact_class(pair))
    return dict(frames=int(len(indices)), checked=len(range(0, len(indices), args.stride)), pairs=pairs)


def precontact_checker(config, result, snapshot):
    """Checker for the world before contact: the attached boxes become world objects at their pose in the attach frame."""
    checker = AllContactsChecker(config, snapshot, dict(result["task"]), snapshot.attachments)
    event = next(item for item in result["predicted_scene_events"] if item["phase"] == "attach")
    values = dict(zip(result["joint_names"], result["frames"][event["frame_index"]]))
    values["base_x"] -= float(result.get("base_advance_m", 0.))
    state = {**checker.locked, **dict(zip(snapshot.state.joint_names, snapshot.state.positions)), **values}
    checker.robot.update_cfg({name: state[name] for name in checker.robot.actuated_joint_names})
    boxes = tuple(SceneObject(item.object_id, item.dimensions_m, matrix_pose(
        checker.robot.get_transform(item.parent_link, checker.robot.base_link) @ pose_matrix(item.tool_to_object)))
        for item in snapshot.attachments)
    scene = replace(snapshot, objects=snapshot.objects + boxes, attachments=(),
                    policy=replace(snapshot.policy, exclude_task_objects_before_contact=False))
    return AllContactsChecker(config, scene, dict(result["task"]), ())


def run(args):
    config = yaml.safe_load((ROOT / "generated/v3_analytic_071cb95/mobile18/alfa_v322_suction_mobile18.yml").read_text())["kinematics"]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    done = set()
    if args.output.exists():
        done = {(item["set"], item["run"]) for item in map(json.loads, args.output.read_text().splitlines())}
    memo, cache = {}, {}
    with args.output.open("a") as sink:
        for label, directory, name in jobs(args.set, args.shard, args.shards):
            if (label, name) in done:
                continue
            result = json.loads((directory / "full" / f"{name}.json").read_text())
            event = next(item for item in result["predicted_scene_events"] if item["phase"] == "attach")
            snapshot = SceneSnapshot.from_dict(event["snapshot"])
            checkers = {False: AllContactsChecker(config, snapshot, dict(result["task"]), snapshot.attachments),
                        True: precontact_checker(config, result, snapshot)}
            phases = np.asarray(result["phases"])
            entry = dict(set=label, run=name, arm=name.split("-")[-1], kind=task_kind(name), phases={}, reused=[])
            for phase in args.phases:
                attached, payload, targets = PHASES[phase]
                indices = np.flatnonzero(phases == phase)
                digest = hashlib.sha1(np.round(np.asarray(result["frames"])[indices], 9).tobytes()).hexdigest()
                key = (label, scene_key(name), phase, digest)
                if key in memo:
                    entry["phases"][phase] = memo[key]
                    entry["reused"].append(phase)
                    continue
                memo[key] = entry["phases"][phase] = audit_phase(checkers[targets], result, indices, attached, payload, args, cache)
            sink.write(json.dumps(entry) + "\n")
            sink.flush()
            print(json.dumps({"set": label, "run": name, "reused": entry["reused"],
                              "pairs": {p: {k: round(v["true_depth_mm"], 1) for k, v in d["pairs"].items()}
                                        for p, d in entry["phases"].items() if d["pairs"]}}), flush=True)


def summarize(args):
    records = [json.loads(line) for path in args.input for line in path.read_text().splitlines() if line.strip()]
    keys = {(r["set"], r["run"]) for r in records}
    if len(keys) != len(records):
        raise SystemExit("duplicate runs across inputs")
    table = collections.defaultdict(lambda: dict(runs=set(), depths=collections.defaultdict(float), pairs=collections.Counter(),
                                                 deepest=collections.defaultdict(float)))
    groups = collections.Counter()
    extra = []
    for record in records:
        group = record["set"] if args.by_set else "baseline" if record["arm"] in BASELINE_ARMS else "cartesian"
        groups[(group, record["kind"])] += 1
        for phase, data in record["phases"].items():
            for pair, value in data["pairs"].items():
                cls = value["cls"]
                if phase == "extract" and "payload:" in pair:
                    cls = "payload (extract, outside the planner check)"
                elif "robot:base_link" in pair:
                    cls = "base"
                row = table[(phase, cls, group, record["kind"])]
                key = (record["set"], record["run"])
                row["runs"].add(key)
                row["depths"][key] = max(row["depths"][key], value["true_depth_mm"])
                links = " | ".join(sorted(item.split(":")[1] for item in pair.split(" | ")))
                row["pairs"][links] += 1
                row["deepest"][links] = max(row["deepest"][links], value["true_depth_mm"])
                if cls == "base" and value["true_depth_mm"] > 0. and value["true_frame"] is not None:
                    extra.append(dict(set=record["set"], run=record["run"], arm=record["arm"], kind=record["kind"], phase=phase,
                                      frame=int(value["true_frame"]), pair=pair, true_depth_mm=value["true_depth_mm"]))
    rows = []
    for (phase, cls, group, kind), row in sorted(table.items()):
        depths = np.asarray(list(row["depths"].values()))
        rows.append(dict(phase=phase, cls=cls, group=group, kind=kind, runs_total=groups[(group, kind)],
                         runs_with_contact=len(row["runs"]),
                         runs_over_mm={str(t): int((depths > t).sum()) for t in THRESHOLDS_MM},
                         max_true_depth_mm=float(depths.max()) if len(depths) else 0.,
                         pairs={links: dict(runs=n, max_true_depth_mm=row["deepest"][links])
                                for links, n in row["pairs"].most_common(6)}))
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "phase-summary.json").write_text(json.dumps(dict(
        cycles=len(records), groups={f"{g}/{k}": n for (g, k), n in sorted(groups.items())}, rows=rows),
        ensure_ascii=False, indent=1) + "\n")
    (args.output / "base-extra-frames.jsonl").write_text("".join(json.dumps(item) + "\n" for item in extra))
    print(f"cycles {len(records)}; groups {dict(groups)}; base contact records {len(extra)}")
    for row in rows:
        over = "/".join(str(row["runs_over_mm"][str(t)]) for t in THRESHOLDS_MM)
        print(f"{row['phase']:16s} {row['cls']:44s} {row['group']:9s} {row['kind']:4s} | >0/2/5/10 mm {over:13s} of {row['runs_total']:3d}"
              f" | max {row['max_true_depth_mm']:5.1f} | " + ", ".join(f"{k} {v['runs']}x<={v['max_true_depth_mm']:.1f}"
                                                                   for k, v in list(row["pairs"].items())[:3]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    runner = commands.add_parser("run")
    runner.add_argument("--set", nargs=2, action="append", metavar=("LABEL", "RUN_DIR"), required=True)
    runner.add_argument("--shard", type=int, default=0)
    runner.add_argument("--shards", type=int, default=1)
    runner.add_argument("--stride", type=int, default=1)
    runner.add_argument("--depth-stride", type=int, default=8)
    runner.add_argument("--samples", type=int, default=1500)
    runner.add_argument("--phases", type=lambda text: text.split(","), default=DEFAULT_PHASES,
                        help=f"comma separated, of {', '.join(PHASES)}")
    runner.add_argument("--output", type=Path, required=True)
    merger = commands.add_parser("summarize")
    merger.add_argument("--input", type=Path, nargs="+", required=True)
    merger.add_argument("--output", type=Path, required=True)
    merger.add_argument("--by-set", action="store_true", help="group by set label instead of baseline/Cartesian (A/B re-runs)")
    args = parser.parse_args()
    {"run": run, "summarize": summarize}[args.command](args)


if __name__ == "__main__":
    main()
