#!/usr/bin/env python3
"""MOTION-334 step 3: what a patched collision model detects and what it rejects, over the whole cycle.

Joins three inputs per run and phase:
  - the mesh audit of the executed frames: phase_audit.py records (approach, contact_approach, extract,
    conveyor_place, return_home; contact frame ranges and exact depths per pair; a later file replaces
    the phases it covers) and, for transport, the MOTION-326 true-depth records (deepest frame per pair);
  - patched checks (patch_eval.py check on the GPU, patch_check.py patch-eval check-cpu): the frames the
    patched model rejects (all of them were valid in the frozen model). With several files, a phase
    comes from the first file that has it: the GPU check is exact but covers only the fixed-base phases
    (approach to transport), the CPU replica adds conveyor_place and return_home but leaves out the
    task-box check of approach;
  - optionally the rejected-frame audit (patch_check.py flag-audit): sampled rejected frames, each
    overlapping pair measured against the meshes (contact depth or clearance).

detection   run-phases with a mesh contact deeper than each threshold, and how many of them the patch
            rejects: anywhere in the phase, and at the contact itself (a rejected frame inside a
            contact range, or at the deepest frame for transport). Extract payload contacts are listed
            apart: the planner checks the box only at the end of extraction, so no model can catch them.
rejections  run-phases the patch rejects, with and without mesh contact (> 0 mm) in the phase, and the
            rejected frames inside a contact range.
flags       rejected frames re-checked against the meshes: contact, or clearance in bands.
The touch of the suction cup on the task box at the end of contact_approach is the grasp, not a
miss; it is classed robot-target (grasp).
Baseline (joint_rrt) and Cartesian runs are counted apart; approach to extract are planned before
transport and repeat across the arms of a scene.
"""
import argparse
import collections
import json
from pathlib import Path

import numpy as np

THRESHOLDS_MM = (0., 2., 5., 10.)
BANDS_MM = (2., 5., 10.)
EXTRACT_PAYLOAD = "payload (extract, outside the planner check)"
GRASP = "robot-target (grasp)"
EXEMPT = (EXTRACT_PAYLOAD, GRASP)


def contact_class(phase, pair, cls, last=None, frames=None):
    if phase == "extract" and "payload:" in pair:
        return EXTRACT_PAYLOAD
    if phase == "contact_approach" and cls == "robot-target" and last == frames - 1:
        return GRASP
    return "base" if "robot:base_link" in pair else cls


def expand(spans):
    return {index for first, last in spans for index in range(first, last + 1)}


def load_contacts(audit_paths, transport_path):
    """(set, run) -> phase -> [(class, depth mm, contact frames)]; a later audit file replaces the phases it covers."""
    contacts = collections.defaultdict(dict)
    for path in audit_paths:
        for record in map(json.loads, path.read_text().splitlines()):
            for phase, data in record["phases"].items():
                contacts[(record["set"], record["run"])][phase] = [
                    (contact_class(phase, pair, value["cls"], value["last"], data["frames"]), value["true_depth_mm"],
                     range(value["first"], value["last"] + 1)) for pair, value in data["pairs"].items()]
    if transport_path:
        for record in map(json.loads, transport_path.read_text().splitlines()):
            contacts[(record["set"], record["run"])]["transport"] = [
                (contact_class("transport", pair, value["cls"]), value["true_depth_mm"], range(value["frame"], value["frame"] + 1))
                for pair, value in record["pairs"].items()]
    return contacts


def load_checked(paths):
    """Patched checks by run; a phase comes from the first file that has it (GPU check first, CPU replica after)."""
    merged = {}
    for path in paths:
        for entry in map(json.loads, path.read_text().splitlines()):
            if not entry:
                continue
            item = merged.setdefault((entry["set"], entry["run"]), dict(entry, phases={}, source={}))
            for phase, value in entry["phases"].items():
                if phase not in item["phases"]:
                    item["phases"][phase], item["source"][phase] = value, path.name
    return list(merged.values())


def group_of(arm):
    return "baseline" if arm == "joint_rrt" else "cartesian"


def analyse(checked, contacts, flags):
    detection = collections.defaultdict(lambda: collections.Counter())
    rejections = collections.defaultdict(lambda: collections.Counter())
    totals = collections.Counter()
    for entry in checked:
        key, group = (entry["set"], entry["run"]), group_of(entry["arm"])
        totals[(group, entry["kind"])] += 1
        for phase, value in entry["phases"].items():
            invalid = expand(value["invalid"])
            found = contacts.get(key, {}).get(phase, [])
            touching = any(depth > 0. for cls, depth, _ in found if cls not in EXEMPT)
            row = rejections[(phase, group, entry["kind"])]
            row["rejected"] += bool(invalid)
            row["rejected_with_contact"] += bool(invalid) and touching
            row["rejected_mesh_clean"] += bool(invalid) and not touching
            row["rejected_frames"] += len(invalid)
            row["rejected_frames_in_contact"] += len(invalid & {index for cls, depth, frames in found
                                                                if depth > 0. and cls not in EXEMPT for index in frames})
            for cls in sorted({cls for cls, _, _ in found}):
                depths = [(depth, frames) for c, depth, frames in found if c == cls]
                for threshold in THRESHOLDS_MM:
                    deep = [frames for depth, frames in depths if depth > threshold]
                    if not deep:
                        continue
                    cell = detection[(phase, cls, group, entry["kind"], threshold)]
                    cell["runs"] += 1
                    cell["phase_rejected"] += bool(invalid)
                    cell["contact_rejected"] += any(index in invalid for frames in deep for index in frames)
    flag_rows = collections.defaultdict(lambda: dict(frames=0, contact=0, depths=[], clearances=[]))
    for record in flags:
        row = flag_rows[(record["phase"], group_of(record["arm"]), record["kind"], record["pair"].split(" | ")[1].split(":")[0]
                         if ":" in record["pair"].split(" | ")[1] else "robot")]
        row["frames"] += 1
        if record["true_depth_mm"] is not None:
            row["contact"] += 1
            row["depths"].append(record["true_depth_mm"])
        else:
            row["clearances"].append(record["clearance_mm"])
    return totals, detection, rejections, flag_rows


def report(args):
    contacts = load_contacts(args.audit, args.transport_depth)
    checked = load_checked(args.checked)
    flags = [json.loads(line) for line in args.flags.read_text().splitlines() if line.strip()] if args.flags else []
    totals, detection, rejections, flag_rows = analyse(checked, contacts, flags)
    out = dict(checked=[str(path) for path in args.checked], runs=len(checked), groups={f"{g}/{k}": n for (g, k), n in sorted(totals.items())},
               detection=[dict(phase=p, cls=c, group=g, kind=k, over_mm=t, **cell)
                          for (p, c, g, k, t), cell in sorted(detection.items())],
               rejections=[dict(phase=p, group=g, kind=k, runs_total=totals[(g, k)], **row)
                           for (p, g, k), row in sorted(rejections.items())],
               flags=[dict(phase=p, group=g, kind=k, partner=partner, frames=row["frames"], contact=row["contact"],
                           max_depth_mm=max(row["depths"], default=None),
                           clearance_bands_mm={f"<={b:g}": int(sum(c <= b for c in row["clearances"])) for b in BANDS_MM}
                           | {f">{BANDS_MM[-1]:g}": int(sum(c > BANDS_MM[-1] for c in row["clearances"]))},
                           clearance_median_mm=float(np.median(row["clearances"])) if row["clearances"] else None)
                      for (p, g, k, partner), row in sorted(flag_rows.items())])
    args.output.write_text(json.dumps(out, ensure_ascii=False, indent=1) + "\n")
    print(f"{' + '.join(path.name for path in args.checked)}: {len(checked)} runs {out['groups']}")
    print("detection (runs with contact > t: rejected in phase / at the contact)")
    for row in out["detection"]:
        if row["group"] == "baseline" or args.all_groups:
            print(f"  {row['phase']:16s} {row['cls']:44s} {row['group']:9s} {row['kind']:4s} >{row['over_mm']:4.1f} mm "
                  f"{row['runs']:4d} | phase {row['phase_rejected']:4d} | contact {row['contact_rejected']:4d}")
    print("rejections (run-phases rejected: with mesh contact / mesh-clean, of runs)")
    for row in out["rejections"]:
        if row["rejected"] and (row["group"] == "baseline" or args.all_groups):
            print(f"  {row['phase']:16s} {row['group']:9s} {row['kind']:4s} {row['rejected']:4d} = {row['rejected_with_contact']:4d}"
                  f" + {row['rejected_mesh_clean']:4d} of {row['runs_total']:4d} | frames {row['rejected_frames']}"
                  f" ({row['rejected_frames_in_contact']} inside a contact range)")
    print("flags (re-checked rejected frames: contact | clearance bands)")
    for row in out["flags"]:
        if row["group"] == "baseline" or args.all_groups:
            print(f"  {row['phase']:16s} {row['group']:9s} {row['kind']:4s} vs {row['partner']:8s} {row['frames']:5d} | contact {row['contact']:4d}"
                  + (f" (max {row['max_depth_mm']:.1f})" if row["max_depth_mm"] is not None else "")
                  + f" | clear {row['clearance_bands_mm']} median {row['clearance_median_mm']}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit", type=Path, nargs="+", required=True, help="phase_audit.py run outputs")
    parser.add_argument("--transport-depth", type=Path, help="MOTION-326 true-depth-records.jsonl (transport)")
    parser.add_argument("--checked", type=Path, nargs="+", required=True, help="patched check JSONL files, GPU check first")
    parser.add_argument("--flags", type=Path, help="flag-audit JSONL of the same check")
    parser.add_argument("--all-groups", action="store_true", help="print Cartesian rows too")
    parser.add_argument("--output", type=Path, required=True)
    report(parser.parse_args())


if __name__ == "__main__":
    main()
