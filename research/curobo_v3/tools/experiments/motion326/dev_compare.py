#!/usr/bin/env python3
"""MOTION-326 step 3d: planning impact of a sphere patch on the dev set (item 3).

Runs of the same task, seed and arm are paired across variants (e.g. frozen-a, patched, carriage,
frozen-b). Per variant and arm: success, single-arm validate(), which executed transports differ from
the reference (frame by frame), paired median changes of the transport stage, search, round planning
time and transport duration (over all pairs and over the changed transports), and, when present, the
mesh audit (fcl-audit.json) and true depths.
"""
import argparse
import collections
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from experiments.motion326.miss_inventory import task_kind

METRICS = {"transport_stage_ms": ("planning", "transport_stage_ms"), "search_ms": ("planning", "search_ms"),
           "round_total_ms": ("planning", "round_total_ms"), "transport_duration_s": ("executed", "transport_duration_s")}
THRESHOLDS_MM = (2., 5., 10.)


def value(record, path):
    section, key = path
    return (record.get(section) or {}).get(key)


def transport_frames(directory, name):
    result = json.loads((Path(directory) / "full" / f"{name}.json").read_text())
    return np.asarray([row for row, phase in zip(result["frames"], result["phases"]) if phase == "transport"])


def same_frames(a, b):
    return a.shape == b.shape and bool(np.allclose(a, b, atol=1e-9))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", nargs=2, action="append", metavar=("LABEL", "RUN_DIR"), required=True)
    parser.add_argument("--reference", required=True, help="variant label the others are paired against")
    parser.add_argument("--true-depth", type=Path, help="true-depth-records.jsonl with set labels = variant labels")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    directories = dict(args.variant)
    runs = {label: {path.stem: json.loads(path.read_text()) for path in (Path(directory) / "runs").glob("*.json")}
            for label, directory in args.variant}
    audits = {}
    for label, directory in args.variant:
        path = Path(directory) / "fcl-audit.json"
        if path.exists():
            audits[label] = {item["run"]: item for item in json.loads(path.read_text())["runs"]}
    depths = collections.defaultdict(dict)
    if args.true_depth:
        for record in map(json.loads, args.true_depth.read_text().splitlines()):
            depths[record["set"]][record["run"]] = max((p["true_depth_mm"] for p in record["pairs"].values()), default=0.)
    reference = runs[args.reference]
    table = []
    for label, _ in args.variant:
        for arm in sorted({record["arm"] for record in runs[label].values()}):
            items = {name: record for name, record in runs[label].items() if record["arm"] == arm}
            row = dict(variant=label, arm=arm, runs=len(items), success=sum(r["success"] for r in items.values()),
                       validated=sum(bool((r.get("safety") or {}).get("validated")) for r in items.values()),
                       failures=collections.Counter(f"{r.get('failure_class')}:{r.get('blocked_stage')}" for r in items.values()
                                                    if not r["success"]),
                       changed_transports=[], paired={}, paired_changed={}, medians={})
            names = [name for name, record in items.items()
                     if name in reference and record["success"] and reference[name]["success"]]
            row["changed_transports"] = sorted(name for name in names if not same_frames(
                transport_frames(directories[label], name), transport_frames(directories[args.reference], name)))
            for metric, path in METRICS.items():
                for key, subset in (("paired", names), ("paired_changed", row["changed_transports"])):
                    ratios = [value(items[name], path) / value(reference[name], path) - 1 for name in subset
                              if value(items[name], path) and value(reference[name], path)]
                    row[key][metric] = float(np.median(ratios)) if ratios else None
                values = [value(r, path) for r in items.values() if r["success"] and value(r, path) is not None]
                row["medians"][metric] = [float(np.median(values)), float(np.percentile(values, 95))] if values else None
            if label in audits:
                audited = [audits[label][name] for name in items if name in audits[label]]
                row["mesh_audit"] = {kind: [sum(a["passed"] for a in audited if task_kind(a["run"]) == kind),
                                            sum(task_kind(a["run"]) == kind for a in audited)] for kind in ("side", "top")}
            if label in depths:
                values = [depths[label][name] for name in items if name in depths[label]]
                row["true_depth_runs_over_mm"] = {str(t): [sum(v > t for v in values), len(values)] for t in THRESHOLDS_MM}
                row["true_depth_max_mm"] = max(values, default=0.)
            row["failures"] = dict(row["failures"])
            table.append(row)
    args.output.write_text(json.dumps(table, ensure_ascii=False, indent=1) + "\n")
    for row in table:
        paired = " ".join(f"{k} {v * 100:+.1f}%" for k, v in row["paired"].items() if v is not None)
        audit = f" | mesh pass side {row['mesh_audit']['side'][0]}/{row['mesh_audit']['side'][1]} top {row['mesh_audit']['top'][0]}/{row['mesh_audit']['top'][1]}" if "mesh_audit" in row else ""
        depth = (" | true depth >2/5/10 mm " + " ".join(f"{a}/{b}" for a, b in row["true_depth_runs_over_mm"].values())
                 + f" max {row['true_depth_max_mm']:.1f}") if "true_depth_runs_over_mm" in row else ""
        changed = row["paired_changed"]["transport_duration_s"]
        print(f"{row['variant']:9s} {row['arm']:17s} success {row['success']}/{row['runs']} validated {row['validated']} "
              f"changed {len(row['changed_transports'])}" + (f" (duration {changed * 100:+.1f}%)" if changed is not None else "")
              + f" | stage median/P95 {row['medians']['transport_stage_ms'][0]:.0f}/"
              f"{row['medians']['transport_stage_ms'][1]:.0f} ms | paired vs {args.reference}: {paired}{audit}{depth}"
              + (f" | failures {row['failures']}" if row["failures"] else ""))


if __name__ == "__main__":
    main()
