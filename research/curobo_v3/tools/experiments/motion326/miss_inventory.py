#!/usr/bin/env python3
"""MOTION-326 step 1: inventory of FCL mesh contacts that the GPU sphere model accepted.

Reads the transport-frame audits of the MOTION-323/327 A/B runs (every audited run was valid for
the GPU sphere model) and groups the reported contacts by set, arm, task kind and contact class.
The audits report the first colliding pair per frame, so pair frame counts depend on pair order;
per-run maximum depths do not.
"""
import argparse
import collections
import json
from pathlib import Path

import numpy as np

SIDE_FIRST_BOX = 10  # boxes 0-9 are top suction, 10-24 side suction


def task_kind(run):
    task = run.split("-")[0]
    return "top" if int(task[-2:]) < SIDE_FIRST_BOX else "side"


def contact_class(pair):
    kinds = sorted(item.split(":")[0] for item in pair.split(" | "))
    if "ground" in pair:
        return "ground"
    return {("robot", "robot"): "self", ("payload", "robot"): "robot-payload", ("payload", "world"): "payload-world",
            ("robot", "world"): "robot-world"}.get(tuple(kinds), "other")


def link_pair(pair):
    return " | ".join(sorted(item.split(":")[1] for item in pair.split(" | ")))


def stats(values):
    if not values:
        return dict(n=0)
    values = np.asarray(values)
    return dict(n=int(len(values)), median=float(np.median(values)), p95=float(np.percentile(values, 95)), max=float(values.max()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit", nargs=2, action="append", metavar=("LABEL", "FCL_AUDIT_JSON"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    groups = collections.defaultdict(lambda: dict(runs=0, passed=0, run_max_depth=[], classes=collections.defaultdict(list),
                                                  pairs=collections.defaultdict(list)))
    for label, path in args.audit:
        for run in json.loads(Path(path).read_text())["runs"]:
            arm = run["run"].split("-")[-1]
            group = groups[(label, arm, task_kind(run["run"]))]
            group["runs"] += 1
            group["passed"] += bool(run["passed"])
            depths = [contact["max_depth_mm"] for contact in run["contacts"].values()]
            if depths:
                group["run_max_depth"].append(max(depths))
            for pair, contact in run["contacts"].items():
                group["classes"][contact_class(pair)].append(contact["max_depth_mm"])
                group["pairs"][link_pair(pair)].append(contact["max_depth_mm"])
    report = []
    for (label, arm, kind), group in sorted(groups.items()):
        report.append(dict(
            set=label, arm=arm, kind=kind, runs=group["runs"], passed=group["passed"],
            run_max_depth_mm=stats(group["run_max_depth"]),
            classes={name: stats(values) for name, values in sorted(group["classes"].items())},
            top_pairs=sorted(({"pair": pair, **stats(values)} for pair, values in group["pairs"].items()),
                             key=lambda item: -item["max"])[:8]))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=1) + "\n")
    for row in report:
        classes = ", ".join(f"{name} {value['n']}×≤{value['max']:.1f}" for name, value in row["classes"].items())
        depth = row["run_max_depth_mm"]
        print(f"{row['set']:8s} {row['arm']:20s} {row['kind']:4s} passed {row['passed']:3d}/{row['runs']:3d} | "
              f"run max depth median {depth.get('median', 0):5.1f} p95 {depth.get('p95', 0):5.1f} max {depth.get('max', 0):5.1f} mm | {classes}")


if __name__ == "__main__":
    main()
