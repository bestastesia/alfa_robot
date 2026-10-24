#!/usr/bin/env python3
"""MOTION-311 G1' (v2): per-mode headroom of the stage-wise three-IK selection, before any v2 network is trusted.

Per mode (dual/single-left/single-right x side/top):
  place    D3 realized place+return time: baseline rule vs best labelled option vs the pure-T_lb choice (B0)
  carry    S2 forced carry IKs of one root: downstream T_lb (transport + place + return, baseline place rule)
  triple   optimistic analytic bound: min over successful contacts, raw carry candidates and the enumerated place set
           of the whole-round T_lb, against the baseline round's T_lb (straight, collision-free motions assumed)
  success  baseline failures and how many have a successful forced contact (P3)
"""
import argparse
import glob
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import g1_place_report

ROOT = Path(__file__).resolve().parents[3]
NAMES = ["updown"] + [f"left_joint{index}" for index in range(1, 8)] + [f"right_joint{index}" for index in range(1, 8)]
FRONT = ("approach", "contact_approach", "extract")
DOWN = ("transport", "conveyor_place", "return_home")
ATLASES = {"side": ROOT / "generated/selected_placement_20261013/all_solutions.npz", "top": ROOT / "generated/placement_analytic_atlas/top.npz"}


def velocities():
    joints = {joint.attrib["name"]: joint for joint in ET.parse(ROOT / "generated/v3_analytic_071cb95/model/robot_meter_meshes.urdf").getroot().findall("joint")}
    return np.array([float(joints[name].find("limit").attrib["velocity"]) for name in NAMES])


def bound(first, second, velocity):
    return np.max(np.abs(np.asarray(second, dtype=np.float64) - np.asarray(first, dtype=np.float64)) / velocity, axis=-1)


def phase_bound(cost, phases, velocity):
    ends = cost.get("phase_ends") or {}
    return sum(float(bound(ends[phase][0], ends[phase][1], velocity)) for phase in phases if phase in ends)


def mode_of(record):
    grasp = "top" if record["context"]["top_grasp"] else "side"
    return f"dual-{grasp}" if record["kind"] == "dual" else f"single-{next(iter(record['task']))}-{grasp}"


def idle_variants(planner_home, contact, side):
    """The planner's six idle-arm variants (reference = extraction end, Home, four shoulder/elbow offsets)."""
    idle = slice(8, 15) if side == "left" else slice(1, 8)
    variants = [np.asarray(contact)[idle], planner_home[idle]]
    for shoulder, elbow in [(-20, 15), (20, -15), (-35, 30), (35, -30)]:
        arm = planner_home[idle].copy()
        arm[1] += np.radians(shoulder)
        arm[3] += np.radians(elbow)
        variants.append(arm)
    return idle, variants


def downstream_table(carries, places, home, velocity, chunk=512):
    """V3(g) = min_p T_lb(g, p) + T_lb(p, Home) for every carry candidate."""
    back = bound(places, home, velocity)
    output = np.empty(len(carries))
    for offset in range(0, len(carries), chunk):
        block = carries[offset:offset + chunk]
        output[offset:offset + chunk] = np.min(np.max(np.abs(block[:, None, :] - places[None]) / velocity, axis=-1) + back[None], axis=1)
    return output


def triple_bound(records, places, velocity, atlas):
    gains = []
    home = np.asarray(records[0]["context"]["home"], dtype=np.float64)
    single = records[0]["kind"] == "single"
    side = next(iter(records[0]["task"])) if single else None
    table = None if single else downstream_table(atlas, places, home, velocity)
    for record in records:
        if record["baseline"]["class_"] != "success":
            continue
        base = phase_bound(record["baseline"]["cost"], FRONT + DOWN, velocity)
        best = np.inf
        for entry in record["s1"]:
            if entry["class_"] != "success":
                continue
            ends = entry["cost"]["phase_ends"]
            root = np.asarray(ends["extract"][1], dtype=np.float64)
            front = phase_bound(entry["cost"], FRONT, velocity)
            if single:
                idle, variants = idle_variants(home, root, side)
                candidates = []
                for arm in variants:
                    values = atlas.copy()
                    values[:, idle] = arm
                    candidates.append(values)
                carries = np.concatenate(candidates)
                values = bound(root[None], carries, velocity) + downstream_table(carries, places, home, velocity)
            else:
                values = bound(root[None], atlas, velocity) + table
            best = min(best, front + float(values.min()))
        if np.isfinite(best):
            gains.append((base - best) / base)
    return gains


def carry_headroom(records, velocity):
    output = []
    for record in records:
        for root in record.get("s2_pools", {}):
            reference = next((entry for entry in record["s1"] if entry["index"] == int(root)), None)
            forced = [entry for entry in record["s2"] if entry["root"] == int(root) and entry["class_"] == "success"]
            if reference is None or reference["class_"] != "success" or not forced:
                continue
            base = phase_bound(reference["cost"], DOWN, velocity)
            output.append((base - min(phase_bound(entry["cost"], DOWN, velocity) for entry in forced)) / base)
    return output


def quantiles(values):
    values = [value for value in values if value is not None]
    if not values:
        return None
    return dict(n=len(values), median=float(np.median(values)), q10=float(np.quantile(values, .1)), q90=float(np.quantile(values, .9)),
                share_ge_5pct=float(np.mean(np.asarray(values) >= .05)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--primary", type=Path, required=True)
    parser.add_argument("--d3", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    velocity = velocities()
    records = [json.loads(Path(path).read_text()) for path in sorted(glob.glob(str(args.primary / "instances/*.json")))]
    report = {}
    for folder in sorted(path for path in args.d3.iterdir() if (path / "place_set.npz").exists()):
        mode = folder.name
        mine = [record for record in records if mode_of(record) == mode]
        places = np.load(folder / "place_set.npz")["rows"].astype(np.float64)
        atlas = np.load(ATLASES["top" if mode.endswith("top") else "side"])["joints"].astype(np.float64)
        place = g1_place_report.analyse(folder) if (folder / "labels.jsonl").exists() else {}
        failures = [record for record in mine if record["baseline"]["class_"] != "success"]
        rescuable = [record for record in failures if any(entry["class_"] == "success" for entry in record["s1"])]
        report[mode] = dict(instances=len(mine),
                            place={key: value for key, value in place.items() if key != "rows"},
                            carry=quantiles(carry_headroom(mine, velocity)),
                            triple_bound=quantiles(triple_bound(mine, places, velocity, atlas)) if mine else None,
                            success=dict(baseline_failures=len(failures), rescuable=len(rescuable),
                                         stages=sorted({record["baseline"].get("stage") or "none" for record in failures})))
        print(json.dumps({mode: report[mode]}, ensure_ascii=False, default=float), flush=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=1, default=float) + "\n")


if __name__ == "__main__":
    main()
