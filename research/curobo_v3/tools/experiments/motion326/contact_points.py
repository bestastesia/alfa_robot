#!/usr/bin/env python3
"""MOTION-326 step 2: locate the mesh contacts that the GPU sphere model accepted.

For the deepest audited runs, every frame inside the reported contact ranges is re-checked against
all collision pairs (not only the first one), and each FCL contact point is expressed in the local
frame of the robot link or payload box involved. Payload contacts are labelled corner / edge / face
by how many box axes they lie on, to test whether inscribed payload spheres leave box edges open.
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

from curobo_core.mesh_validation import MeshStateChecker
from curobo_core.scene import SceneSnapshot
from v3_research_assets import ROOT


class AllContactsChecker(MeshStateChecker):
    def contacts(self, positions, *, attached, payload_enabled, max_contacts=4):
        state = {**self.locked, **dict(zip(self.snapshot.state.joint_names, self.snapshot.state.positions)), **positions}
        self.robot.update_cfg({name: state[name] for name in self.robot.actuated_joint_names})
        links = {name: self.robot.get_transform(name, self.robot.base_link) for name in {s["link"] for s in self.shapes if s["link"]}}
        frames, enabled = [], []
        for item in self.shapes:
            active = not (item["kind"] == "payload" and not payload_enabled)
            if item["kind"] == "world" and item["name"] in self.tasks:
                active = not (attached or self.snapshot.policy.exclude_task_objects_before_contact)
            transform = item["origin"] if item["link"] is None else links[item["link"]] @ item["origin"]
            item["object"].setTransform(fcl.Transform(transform[:3, :3], transform[:3, 3]))
            frames.append(transform)
            enabled.append(active)
        request = fcl.CollisionRequest(num_max_contacts=max_contacts, enable_contact=True)
        found = []
        for i, j in self.pairs:
            if not (enabled[i] and enabled[j]):
                continue
            result = fcl.CollisionResult()
            if not fcl.collide(self.shapes[i]["object"], self.shapes[j]["object"], request, result):
                continue
            for contact in result.contacts:
                point = np.append(np.asarray(contact.pos, dtype=float), 1.)
                sides = []
                for index in (i, j):
                    shape = self.shapes[index]
                    local = (np.linalg.inv(frames[index]) @ point)[:3]
                    entry = dict(name=f"{shape['kind']}:{shape['name']}", local=local.round(4).tolist())
                    if shape["kind"] == "payload":
                        on_face = np.abs(np.abs(local) - shape["half"]) <= 0.01
                        entry["box_feature"] = {3: "corner", 2: "edge", 1: "face"}.get(int(on_face.sum()), "interior")
                    sides.append(entry)
                found.append(dict(depth_mm=float(contact.penetration_depth) * 1000, world=point[:3].round(4).tolist(), sides=sides))
        return found


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--set", nargs=2, action="append", metavar=("LABEL", "RUN_DIR"), required=True,
                        help="A/B output directory containing fcl-audit.json and full/")
    parser.add_argument("--runs-per-arm", type=int, default=5)
    parser.add_argument("--stride", type=int, default=2)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = yaml.safe_load((ROOT / "generated/v3_analytic_071cb95/mobile18/alfa_v322_suction_mobile18.yml").read_text())["kinematics"]
    records = []
    for label, directory in args.set:
        directory = Path(directory)
        runs = json.loads((directory / "fcl-audit.json").read_text())["runs"]
        by_arm = collections.defaultdict(list)
        for run in runs:
            if run["contacts"]:
                by_arm[run["run"].split("-")[-1]].append(run)
        for arm, items in sorted(by_arm.items()):
            for run in sorted(items, key=lambda r: -max(c["max_depth_mm"] for c in r["contacts"].values()))[:args.runs_per_arm]:
                result = json.loads((directory / "full" / f"{run['run']}.json").read_text())
                event = next(item for item in result["predicted_scene_events"] if item["phase"] == "attach")
                snapshot = SceneSnapshot.from_dict(event["snapshot"])
                checker = AllContactsChecker(config, snapshot, dict(result["task"]), snapshot.attachments)
                names, advance = result["joint_names"], float(result.get("base_advance_m", 0.))
                frames = [frame for frame, phase in zip(result["frames"], result["phases"]) if phase == "transport"]
                wanted = sorted({index for contact in run["contacts"].values()
                                 for index in range(contact["first_frame"], contact["last_frame"] + 1, args.stride)})
                for index in wanted:
                    positions = dict(zip(names, frames[index]))
                    positions["base_x"] -= advance
                    for contact in checker.contacts(positions, attached=True, payload_enabled=True):
                        records.append(dict(set=label, run=run["run"], arm=arm, frame=index, **contact))
                print(f"{label} {run['run']}: {len(wanted)} frames re-checked", flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(json.dumps(record) for record in records) + "\n")
    summary = collections.defaultdict(lambda: dict(n=0, max_depth=0., features=collections.Counter(), local=[], partners=collections.Counter()))
    for record in records:
        for side, other in (record["sides"], record["sides"][::-1]):
            entry = summary[(record["arm"], side["name"])]
            entry["n"] += 1
            entry["max_depth"] = max(entry["max_depth"], record["depth_mm"])
            entry["local"].append(side["local"])
            entry["partners"][other["name"]] += 1
            if "box_feature" in side:
                entry["features"][side["box_feature"]] += 1
    for (arm, name), entry in sorted(summary.items(), key=lambda kv: -kv[1]["max_depth"]):
        if entry["n"] < 3 and not name.startswith("payload"):
            continue
        local = np.asarray(entry["local"])
        features = f" | box {dict(entry['features'])}" if entry["features"] else ""
        print(f"{arm:20s} {name:28s} n {entry['n']:5d} max {entry['max_depth']:5.1f} mm | local x {local[:, 0].min():+.3f}..{local[:, 0].max():+.3f} "
              f"y {local[:, 1].min():+.3f}..{local[:, 1].max():+.3f} z {local[:, 2].min():+.3f}..{local[:, 2].max():+.3f}{features} | "
              f"with {entry['partners'].most_common(2)}")


if __name__ == "__main__":
    main()
