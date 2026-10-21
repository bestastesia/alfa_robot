#!/usr/bin/env python3
"""MOTION-326 step 2b: verify FCL contact depths with exact signed distances.

FCL reports BVH (non-convex mesh) penetration depths that are not true depths. For the deepest
recorded contacts of each shape pair, both shapes are rebuilt in world coordinates and the true
penetration is measured as the largest signed distance of either surface (sampled) inside the
other watertight shape.
"""
import argparse
import collections
import json
from pathlib import Path
import sys

import numpy as np
import trimesh
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from curobo_core.scene import SceneSnapshot
from experiments.motion326.contact_points import AllContactsChecker
from v3_research_assets import ROOT


def shape_mesh(checker, index, transform, cache):
    shape = checker.shapes[index]
    if shape["kind"] == "robot":
        if shape["name"] not in cache:
            link_name, mesh_index = shape["name"].rsplit(":", 1)
            collision = checker.robot.link_map[link_name].collisions[int(mesh_index)]
            mesh = trimesh.load(collision.geometry.mesh.filename, force="mesh", process=False)
            if collision.geometry.mesh.scale is not None:
                mesh.apply_scale(collision.geometry.mesh.scale)
            vertices, inverse = np.unique(mesh.vertices, axis=0, return_inverse=True)
            cache[shape["name"]] = trimesh.Trimesh(vertices=vertices, faces=inverse[mesh.faces], process=False)
        mesh = cache[shape["name"]].copy()
    else:
        mesh = trimesh.creation.box(extents=2 * np.asarray(shape["half"]))
    mesh.apply_transform(transform)
    return mesh


def penetration(mesh_a, mesh_b, samples=3000):
    depth = 0.
    for source, target in ((mesh_a, mesh_b), (mesh_b, mesh_a)):
        points = np.vstack([source.vertices, trimesh.sample.sample_surface(source, samples, seed=7)[0]])
        inside = trimesh.proximity.signed_distance(target, points)
        depth = max(depth, float(inside.max()))
    return depth


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contacts", type=Path, required=True, help="contact-points.jsonl from contact_points.py")
    parser.add_argument("--set", nargs=2, action="append", metavar=("LABEL", "RUN_DIR"), required=True)
    parser.add_argument("--per-pair", type=int, default=4)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    directories = {label: Path(path) for label, path in args.set}
    config = yaml.safe_load((ROOT / "generated/v3_analytic_071cb95/mobile18/alfa_v322_suction_mobile18.yml").read_text())["kinematics"]
    records = [json.loads(line) for line in args.contacts.read_text().splitlines() if line.strip()]
    best = collections.defaultdict(dict)
    for record in records:
        pair = " | ".join(sorted(side["name"] for side in record["sides"]))
        key = (record["set"], record["run"], record["frame"])
        if record["depth_mm"] > best[pair].get(key, -1):
            best[pair][key] = record["depth_mm"]
    selected = [(pair, key, depth) for pair, frames in best.items()
                for key, depth in sorted(frames.items(), key=lambda kv: -kv[1])[:args.per_pair]]
    checkers, results, cache = {}, [], {}
    for pair, (label, run, frame), fcl_depth in selected:
        if (label, run) not in checkers:
            result = json.loads((directories[label] / "full" / f"{run}.json").read_text())
            event = next(item for item in result["predicted_scene_events"] if item["phase"] == "attach")
            snapshot = SceneSnapshot.from_dict(event["snapshot"])
            frames = [row for row, phase in zip(result["frames"], result["phases"]) if phase == "transport"]
            checkers[(label, run)] = (AllContactsChecker(config, snapshot, dict(result["task"]), snapshot.attachments),
                                      result["joint_names"], float(result.get("base_advance_m", 0.)), frames)
        checker, names, advance, frames = checkers[(label, run)]
        positions = dict(zip(names, frames[frame]))
        positions["base_x"] -= advance
        checker.contacts(positions, attached=True, payload_enabled=True, max_contacts=1)  # sets the shape transforms
        state_links = {name: checker.robot.get_transform(name, checker.robot.base_link) for name in {s["link"] for s in checker.shapes if s["link"]}}
        names_in_pair = pair.split(" | ")
        indices = [next(i for i, s in enumerate(checker.shapes) if f"{s['kind']}:{s['name']}" == name) for name in names_in_pair]
        meshes = [shape_mesh(checker, index, checker.shapes[index]["origin"] if checker.shapes[index]["link"] is None
                             else state_links[checker.shapes[index]["link"]] @ checker.shapes[index]["origin"], cache)
                  for index in indices]
        true_depth = penetration(*meshes) * 1000
        results.append(dict(pair=pair, set=label, run=run, frame=frame, fcl_depth_mm=fcl_depth, true_depth_mm=true_depth))
        print(f"{pair:48s} {label:8s} {run:40s} f{frame:5d} | FCL {fcl_depth:6.1f} mm | signed-distance {true_depth:6.1f} mm", flush=True)
    args.output.write_text(json.dumps(results, indent=1) + "\n")


if __name__ == "__main__":
    main()
