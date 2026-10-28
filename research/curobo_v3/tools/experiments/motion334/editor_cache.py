#!/usr/bin/env python3
"""MOTION-334: frame cache for the patch editor (patch_editor.py).

For every audited run of the given sets it stores, at every --stride-th frame, the poses of every
link with collision spheres (and base_link) in the attach snapshot's world (base joints minus the
top-suction base advance, as patch_check.py evaluates them), with the phase and the frame index in
the phase. Baseline runs (joint_rrt) keep all checked phases; Cartesian runs keep transport only,
the one checked phase where their frames differ from the baseline's in a way the patch targets.
Per run it also keeps what the patched checks test against: the world cuboids of the attach
snapshot (ground excluded), the task box as TargetContactValidity takes it in approach (axis-aligned
at its pick pose), the payload spheres (on link7, as loaded_robot builds them) and the ground height.

  python editor_cache.py --set dev DIR --set holdout DIR --set v2test DIR --shard K --shards N --output DIR
"""
import argparse
import json
from pathlib import Path
import sys
import zlib

import numpy as np
import yaml
import yourdfpy

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from curobo_core.adapter import pose_matrix
from curobo_core.contracts import PlannerAssets
from curobo_core.scene import SceneSnapshot
from experiments.motion326.cpu_spheres import ChainFK, sphere_groups, world_cuboids
from experiments.motion326.miss_inventory import task_kind

PHASES = ("approach", "contact_approach", "extract", "transport", "conveyor_place", "return_home")
CARTESIAN_PHASES = ("transport",)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--set", nargs=2, action="append", metavar=("LABEL", "RUN_DIR"), required=True)
    parser.add_argument("--stride", type=int, default=2)
    parser.add_argument("--shard", type=int, default=0)
    parser.add_argument("--shards", type=int, default=1)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    assets = PlannerAssets()
    fixed = yaml.safe_load(assets.robot_config.read_text())["kinematics"]
    mobile = yaml.safe_load(assets.mobile_robot_config.read_text())["kinematics"]
    fk = ChainFK(yourdfpy.URDF.load(mobile["urdf_path"], load_meshes=False))
    box_fit = json.loads(assets.box_fit.read_text())
    links = sorted(set(fixed["collision_spheres"]) | {"base_link"})
    locked = dict(mobile.get("lock_joints", {}))
    for label, directory in args.set:
        directory = Path(directory)
        for run in json.loads((directory / "fcl-audit.json").read_text())["runs"]:
            name = run["run"]
            if zlib.crc32(f"{label}/{name}".encode()) % args.shards != args.shard:
                continue
            target = args.output / label / f"{name}.npz"
            if target.exists():
                continue
            arm = name.split("-")[-1]
            result = json.loads((directory / "full" / f"{name}.json").read_text())
            event = next(item for item in result["predicted_scene_events"] if item["phase"] == "attach")
            snapshot = SceneSnapshot.from_dict(event["snapshot"])
            names, advance = result["joint_names"], float(result.get("base_advance_m", 0.))
            base = {**locked, **dict(zip(snapshot.state.joint_names, snapshot.state.positions))}
            frames, phases = np.asarray(result["frames"]), np.asarray(result["phases"])
            kept = PHASES if arm == "joint_rrt" else CARTESIAN_PHASES
            rows, codes, local = [], [], []
            for code, phase in enumerate(PHASES):
                if phase not in kept:
                    continue
                indices = np.flatnonzero(phases == phase)[::args.stride]
                rows.append(indices)
                codes.append(np.full(len(indices), code, dtype=np.int8))
                local.append(np.arange(len(np.flatnonzero(phases == phase)))[::args.stride].astype(np.int32))
            rows = np.concatenate(rows)
            values = {**base, **{joint: frames[rows, column] for column, joint in enumerate(names)}}
            values["base_x"] = values["base_x"] - advance
            poses = fk.transforms(values, len(rows))
            attach = {**base, **dict(zip(names, frames[event["frame_index"]]))}
            attach["base_x"] -= advance
            attach_poses = fk.transforms(attach, 1)
            tasks = [(attach_poses[item.parent_link][0] @ pose_matrix(item.tool_to_object))[:3, 3].tolist() + list(np.asarray(item.dimensions_m) / 2)
                     for item in snapshot.attachments]
            payload = [(group[1], group[2].tolist(), group[3].tolist())
                       for group in sphere_groups(fixed, box_fit, snapshot.attachments, {}) if group[0].startswith("payload:")]
            target.parent.mkdir(parents=True, exist_ok=True)
            np.savez(target, poses=np.stack([poses[link][:, :3, :] for link in links], axis=1).astype(np.float32),
                     phase=np.concatenate(codes), local=np.concatenate(local))
            (target.with_suffix(".json")).write_text(json.dumps(dict(
                set=label, run=name, arm=arm, kind=task_kind(name), links=links, frames=int(len(rows)),
                world=[(item_name, pose.tolist(), half.tolist()) for item_name, pose, half in world_cuboids(snapshot)],
                tasks=tasks, payload=payload, ground_z=snapshot.policy.ground_z_m)) + "\n")
            print(label, name, len(rows), flush=True)


if __name__ == "__main__":
    main()
