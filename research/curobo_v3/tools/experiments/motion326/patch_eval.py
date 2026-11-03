#!/usr/bin/env python3
"""MOTION-326 step 3b: would a sphere model have rejected the audited frames?

`check` re-evaluates the executed frames of every audited successful run with the GPU validity of
the single-arm validate() (target validity for approach, ground-checked GpuValidity otherwise,
payload attached in transport) under one robot config, and writes the invalid frame ranges per
phase. For invalid transport frames it also lists overlapping sphere pairs that involve spheres
appended by a patch. `compare` joins a frozen and a patched check with the true-depth records and
the FCL audits: detection of runs with real penetration, and new rejections of mesh-clean runs.
`agree` checks the CPU replica (`check-cpu`) frame by frame against the GPU check of the same patch.
"""
import argparse
import collections
from dataclasses import replace
import json
from pathlib import Path
import sys

import numpy as np
import torch
import yaml
import yourdfpy

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from curobo.types import JointState
from curobo_core.contracts import PlannerAssets
from curobo_core.scene import SceneSnapshot
from experiments.motion326.cpu_spheres import (ChainFK, added_base_cuboids, appended_collisions, base_cuboids, planner_ignored,
                                               sphere_groups, world, world_cuboids)
from experiments.motion326.miss_inventory import task_kind
from v3_batched_loaded_search import GpuValidity
from v3_motion327_transport_v2 import FROZEN_V1, DockingOffsetPlanner
from v3_wall_ik_benchmark import ACTIVE_JOINTS

PHASES = {"approach": False, "contact_approach": False, "extract": False, "transport": True}
THRESHOLDS_MM = (0., 2., 5., 10.)


def ranges(indices):
    result = []
    for index in indices.tolist():
        if result and index == result[-1][1] + 1:
            result[-1][1] = index
        else:
            result.append([index, index])
    return result


def expand(spans):
    return {index for first, last in spans for index in range(first, last + 1)}


def sphere_labels(checker, robot, added_counts):
    """Label every sphere index: link name, '<link>+' for spheres a patch appended, 'payload:<side>' for box spheres."""
    config = checker.kinematics.config.kinematics_config
    labels = {}
    for link, balls in robot["kinematics"]["collision_spheres"].items():
        indices = config.get_sphere_index_from_link_name(link).cpu().tolist()
        configured, added = len(balls), added_counts.get(link, 0)
        for order, index in enumerate(indices):
            if order < configured - added:
                labels[index] = link
            elif order < configured:
                labels[index] = link + "+"
            else:
                labels[index] = "payload:" + link.split("_")[0]
    return labels


def patch_overlaps(checker, values, labels, ignored):
    """Overlapping (appended sphere, other sphere) pairs per frame, by link label."""
    added = [index for index, label in labels.items() if label.endswith("+")]
    if not added or not len(values):
        return {}
    checker.setup_batch_tensors(1, len(values))
    state = checker.kinematics.compute_kinematics(JointState.from_position(
        torch.tensor(values, device="cuda", dtype=torch.float32)[None], joint_names=ACTIVE_JOINTS))
    spheres = state.robot_spheres.reshape(len(values), -1, 4)
    found = collections.defaultdict(lambda: [0, 0.])
    others = [index for index, label in labels.items() if not label.endswith("+")]
    for index in added:
        link = labels[index][:-1]
        partners = [other for other in others if labels[other] != link and frozenset((link, labels[other].replace("payload:", "") + "_link7"
                    if labels[other].startswith("payload:") else labels[other])) not in ignored]
        if not partners:
            continue
        a, b = spheres[:, index], spheres[:, partners]
        valid = (a[:, 3:4] > 0) & (b[..., 3] > 0)
        overlap = (a[:, None, 3] + b[..., 3] - torch.linalg.vector_norm(a[:, None, :3] - b[..., :3], dim=-1)) * valid
        best, which = overlap.max(dim=1)
        for frame in torch.nonzero(best > 0).reshape(-1).tolist():
            key = f"{link} | {labels[partners[int(which[frame])]]}"
            found[key][0] += 1
            found[key][1] = max(found[key][1], float(best[frame]) * 1000)
    return {key: dict(frames=value[0], max_overlap_mm=value[1]) for key, value in found.items()}


def check(args):
    assets = replace(PlannerAssets(), **{key: getattr(args, key) for key in ("robot_config", "mobile_robot_config")
                                         if getattr(args, key)})
    planner = DockingOffsetPlanner(assets, cartesian_options=json.loads(FROZEN_V1.read_text())["cartesian_options_frozen"])
    robot = yaml.safe_load(assets.robot_config.read_text())
    ignored = planner_ignored(robot["kinematics"])
    added_counts = {}
    if args.patch:
        added_counts = {link: len(value["added_spheres"]) for link, value in json.loads(args.patch.read_text())["links"].items()}
    jobs = []
    for label, directory in args.set:
        directory = Path(directory)
        for run in json.loads((directory / "fcl-audit.json").read_text())["runs"]:
            record = json.loads((directory / "runs" / f"{run['run']}.json").read_text())
            jobs.append((label, directory, run["run"], tuple(record.get("scene_offset_m", (0., 0.)))))
    done = set()
    if args.output.exists():
        done = {(item["set"], item["run"]) for item in map(json.loads, args.output.read_text().splitlines())}
    with args.output.open("a") as sink:
        for label, directory, name, offset in sorted(jobs, key=lambda job: (job[0], job[2].split("-s")[0], job[3])):
            if (label, name) in done:
                continue
            result = json.loads((directory / "full" / f"{name}.json").read_text())
            boxes = {side: int(box) for side, box in result["task"].items()}
            planner.scene_offset = offset
            planner.configure_task(boxes)
            names = result["joint_names"]
            values = np.asarray(result["frames"])[:, [names.index(joint) for joint in ACTIVE_JOINTS]]
            phases = np.asarray(result["phases"])
            entry = dict(set=label, run=name, arm=name.split("-")[-1], kind=task_kind(name), phases={})
            for phase, payload in PHASES.items():
                indices = np.flatnonzero(phases == phase)
                checker = planner.checker(boxes, payload=payload)
                validity = planner.target_validity(checker) if phase == "approach" else GpuValidity(checker, check_ground=True)
                mask = validity.mask(torch.tensor(values[indices], device="cuda", dtype=torch.float32)).cpu().numpy()
                entry["phases"][phase] = dict(frames=int(len(indices)), invalid=ranges(np.flatnonzero(~mask)))
                if phase == "transport" and added_counts:
                    labels = sphere_labels(checker, robot, added_counts)
                    invalid = np.flatnonzero(~mask)[:args.max_attributed]
                    entry["transport_patch_overlaps"] = patch_overlaps(checker, values[indices][invalid], labels, ignored)
            sink.write(json.dumps(entry) + "\n")
            sink.flush()
            print(json.dumps({"set": label, "run": name, "invalid": {p: sum(b - a + 1 for a, b in v["invalid"])
                                                                     for p, v in entry["phases"].items()},
                              "overlaps": {k: v["frames"] for k, v in entry.get("transport_patch_overlaps", {}).items()}}),
                  flush=True)


def check_cpu(args):
    """CPU replica of `check` for a patched model: only appended spheres or cuboids can reject a frame (see cpu_spheres)."""
    assets = PlannerAssets()
    fixed = yaml.safe_load(assets.robot_config.read_text())["kinematics"]
    mobile = yaml.safe_load(assets.mobile_robot_config.read_text())["kinematics"]
    fk = ChainFK(yourdfpy.URDF.load(mobile["urdf_path"], load_meshes=False))
    box_fit = json.loads(assets.box_fit.read_text())
    added = {link: value["added_spheres"] for link, value in json.loads(args.patch.read_text())["links"].items()} if args.patch else {}
    ignored = planner_ignored(fixed)
    boxes = base_cuboids()
    extra = added_base_cuboids(args.cuboids) if args.cuboids else []
    locked = dict(mobile.get("lock_joints", {}))
    done = set()
    if args.output.exists():
        done = {(item["set"], item["run"]) for item in map(json.loads, args.output.read_text().splitlines())}
    with args.output.open("a") as sink:
        for label, directory in args.set:
            directory = Path(directory)
            for run in json.loads((directory / "fcl-audit.json").read_text())["runs"]:
                if (label, run["run"]) in done:
                    continue
                result = json.loads((directory / "full" / f"{run['run']}.json").read_text())
                event = next(item for item in result["predicted_scene_events"] if item["phase"] == "attach")
                snapshot = SceneSnapshot.from_dict(event["snapshot"])
                rows, phases = np.asarray(result["frames"]), np.asarray(result["phases"])
                base = {**locked, **dict(zip(snapshot.state.joint_names, snapshot.state.positions))}
                advance = float(result.get("base_advance_m", 0.))
                cuboids = world_cuboids(snapshot)
                entry = dict(set=label, run=run["run"], arm=run["run"].split("-")[-1], kind=task_kind(run["run"]),
                             phases={}, patch_overlaps={})
                for phase, payload in PHASES.items():
                    indices = np.flatnonzero(phases == phase)
                    groups = sphere_groups(fixed, box_fit, snapshot.attachments if payload else (), added)
                    invalid, overlaps = [], {}
                    for chunk in np.array_split(indices, max(1, len(indices) // args.chunk)):
                        values = {**base, **{name: rows[chunk, column] for column, name in enumerate(result["joint_names"])}}
                        values["base_x"] = values["base_x"] - advance
                        poses = fk.transforms(values, len(chunk))
                        hit, found = appended_collisions(groups, world(groups, poses), ignored, cuboids,
                                                         snapshot.policy.ground_z_m, base=(poses["base_link"], boxes),
                                                         added_base=(poses["base_link"], extra) if extra else None)
                        invalid.extend(np.searchsorted(indices, chunk[hit]).tolist())
                        for key, value in found.items():
                            merged = overlaps.setdefault(key, dict(frames=0, max_overlap_mm=0.))
                            merged["frames"] += value["frames"]
                            merged["max_overlap_mm"] = max(merged["max_overlap_mm"], value["max_overlap_mm"])
                    entry["phases"][phase] = dict(frames=int(len(indices)), invalid=ranges(np.asarray(invalid, dtype=int)))
                    entry["patch_overlaps"][phase] = overlaps
                entry["transport_patch_overlaps"] = entry["patch_overlaps"]["transport"]
                sink.write(json.dumps(entry) + "\n")
                sink.flush()
                print(json.dumps({"set": label, "run": run["run"], "invalid": {p: sum(b - a + 1 for a, b in v["invalid"])
                                                                               for p, v in entry["phases"].items()},
                                  "overlaps": {k: v["frames"] for k, v in entry["transport_patch_overlaps"].items()}}),
                      flush=True)


def compare(args):
    load = lambda path: {(item["set"], item["run"]): item for item in map(json.loads, path.read_text().splitlines())}
    frozen, patched = load(args.frozen), load(args.patched)
    truth = {(item["set"], item["run"]): item for item in map(json.loads, args.true_depth.read_text().splitlines())}
    rows = []
    for key, after in patched.items():
        before, depth = frozen[key], truth[key]
        new = {phase: sorted(expand(after["phases"][phase]["invalid"]) - expand(before["phases"][phase]["invalid"]))
               for phase in PHASES}
        if args.links:  # credit only rejections attributed to appended spheres of these links (CPU check output)
            attributed = after.get("patch_overlaps", {"transport": after.get("transport_patch_overlaps", {})})
            new = {phase: value if any(pair.split(" | ")[0] in args.links for pair in attributed.get(phase, {})) else []
                   for phase, value in new.items()}
        flagged = expand(after["phases"]["transport"]["invalid"])
        pairs = depth["pairs"]
        rows.append(dict(set=key[0], run=key[1], group="joint_rrt" if after["arm"] == "joint_rrt" else "cartesian",
                         arm=after["arm"], kind=after["kind"],
                         true_depth_mm=max((p["true_depth_mm"] for p in pairs.values()), default=0.),
                         classes=sorted({p["cls"] for p in pairs.values() if p["true_depth_mm"] > 2.}),
                         carriage=any("arm_carriage" in pair and p["true_depth_mm"] > 2. for pair, p in pairs.items()),
                         base=any("base_link" in pair and p["true_depth_mm"] > 2. for pair, p in pairs.items()),
                         deepest_flagged=any(any(abs(f - p["frame"]) <= 2 for f in flagged) for p in pairs.values()
                                             if p["true_depth_mm"] == max(q["true_depth_mm"] for q in pairs.values())),
                         frozen_transport_invalid=sum(b - a + 1 for a, b in before["phases"]["transport"]["invalid"]),
                         new_invalid={phase: len(value) for phase, value in new.items()},
                         overlaps=after.get("transport_patch_overlaps", {})))
    summary, grouped = [], collections.defaultdict(list)
    for row in rows:
        grouped[(row["set"], row["group"], row["kind"])].append(row)
        grouped[("all", row["group"], row["kind"])].append(row)
    for (label, group, kind), items in sorted(grouped.items()):
        detect = {}
        for threshold in THRESHOLDS_MM:
            hit = [row for row in items if row["true_depth_mm"] > threshold]
            detect[str(threshold)] = [sum(row["new_invalid"]["transport"] > 0 for row in hit), len(hit)]
        target = [row for row in items if row["carriage"] or row["base"]]
        clean = [row for row in items if row["true_depth_mm"] <= 0.]
        summary.append(dict(set=label, group=group, kind=kind, runs=len(items), detected_over_mm=detect,
                            patch_target_runs=[sum(row["new_invalid"]["transport"] > 0 for row in target), len(target)],
                            deepest_frame_flagged=[sum(row["deepest_flagged"] for row in target), len(target)],
                            clean_runs_flagged=[sum(row["new_invalid"]["transport"] > 0 for row in clean), len(clean)],
                            other_phases_new_invalid_runs=sum(any(row["new_invalid"][p] for p in PHASES if p != "transport")
                                                              for row in items),
                            frozen_transport_invalid_runs=sum(row["frozen_transport_invalid"] > 0 for row in items)))
    args.output.write_text(json.dumps(dict(summary=summary, runs=rows), ensure_ascii=False, indent=1) + "\n")
    for row in summary:
        print(f"{row['set']:8s} {row['group']:10s} {row['kind']:4s} runs {row['runs']:3d} | detected >0/2/5/10 mm "
              + " ".join(f"{a}/{b}" for a, b in row["detected_over_mm"].values())
              + f" | carriage/base targets {row['patch_target_runs'][0]}/{row['patch_target_runs'][1]}"
              f" (deepest frame {row['deepest_frame_flagged'][0]}) | clean flagged {row['clean_runs_flagged'][0]}/{row['clean_runs_flagged'][1]}"
              f" | other phases {row['other_phases_new_invalid_runs']} | frozen invalid {row['frozen_transport_invalid_runs']}")


def agree(args):
    """Frame-level agreement of a CPU replica check with the GPU check of the same patch."""
    load = lambda path: {(item["set"], item["run"]): item for item in map(json.loads, path.read_text().splitlines())}
    reference, candidate = load(args.gpu), load(args.cpu)
    counts, differing = collections.Counter(), []
    for key, entry in candidate.items():
        if key not in reference:
            continue
        for phase in PHASES:
            gpu = expand(reference[key]["phases"][phase]["invalid"])
            cpu = expand(entry["phases"][phase]["invalid"])
            counts[f"{phase}:both"] += len(gpu & cpu)
            counts[f"{phase}:gpu_only"] += len(gpu - cpu)
            counts[f"{phase}:cpu_only"] += len(cpu - gpu)
            if gpu != cpu:
                differing.append(dict(set=key[0], run=key[1], phase=phase, gpu_only=len(gpu - cpu), cpu_only=len(cpu - gpu)))
    report = dict(runs=len(candidate), missing_in_gpu=len(set(candidate) - set(reference)), frames=dict(counts),
                  differing_run_phases=differing)
    args.output.write_text(json.dumps(report, indent=1) + "\n")
    print(json.dumps({key: value for key, value in report.items() if key != "differing_run_phases"})
          + f" | differing run-phases {len(differing)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    checker = commands.add_parser("check")
    checker.add_argument("--set", nargs=2, action="append", metavar=("LABEL", "RUN_DIR"), required=True)
    checker.add_argument("--robot-config", type=Path)
    checker.add_argument("--mobile-robot-config", type=Path)
    checker.add_argument("--patch", type=Path, help="patch.json from sphere_patch.py, to attribute new rejections")
    checker.add_argument("--max-attributed", type=int, default=2000, help="invalid transport frames attributed per run")
    checker.add_argument("--output", type=Path, required=True, help="JSONL, appended and resumable")
    cpu = commands.add_parser("check-cpu")
    cpu.add_argument("--set", nargs=2, action="append", metavar=("LABEL", "RUN_DIR"), required=True)
    cpu.add_argument("--patch", type=Path, help="patch.json from sphere_patch.py")
    cpu.add_argument("--cuboids", type=Path, help="cuboids.json from cuboid_patch.py (frozen + added base cuboids)")
    cpu.add_argument("--chunk", type=int, default=400, help="frames per vectorized chunk")
    cpu.add_argument("--output", type=Path, required=True, help="JSONL, appended and resumable")
    comparer = commands.add_parser("compare")
    comparer.add_argument("--frozen", type=Path, required=True)
    comparer.add_argument("--patched", type=Path, required=True)
    comparer.add_argument("--true-depth", type=Path, required=True, help="true-depth-records.jsonl")
    comparer.add_argument("--links", type=lambda text: text.split(","), help="only count rejections caused by spheres "
                          "appended to these links (e.g. arm_carriage); approximate per run: a run counts if any such pair overlaps")
    comparer.add_argument("--output", type=Path, required=True)
    agreement = commands.add_parser("agree")
    agreement.add_argument("--gpu", type=Path, required=True, help="`check` output with the patched configs")
    agreement.add_argument("--cpu", type=Path, required=True, help="`check-cpu` output for the same patch")
    agreement.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "check-cpu" and not (args.patch or args.cuboids):
        parser.error("check-cpu needs --patch and/or --cuboids")
    {"check": check, "check-cpu": check_cpu, "compare": compare, "agree": agree}[args.command](args)


if __name__ == "__main__":
    main()
