#!/usr/bin/env python3
"""MOTION-334 step 3: A/B of patched collision models against the frozen one (v3_motion327_transport_v2.py runs).

Every variant is paired run by run (same task, docking offset, seed and arm) with the reference run:
success and failure classes, planning time (round total and the planner's stages, paired relative
change), the round's executed motion time, and per phase whether the executed frames changed. A second
frozen run gives the frozen-vs-frozen drift of the same numbers.

  python ab_compare.py --reference frozen-a DIR --variant full-p3 DIR --variant frozen-b DIR ... --output report.json
"""
import argparse
import collections
import json
from pathlib import Path

import numpy as np

STAGES = {"round_total_ms": lambda result: result.get("total_ms"),
          "contact_candidates_ms": lambda result: result.get("timing_ms", {}).get("contact_candidates_ms"),
          "extraction_and_approach_ms": lambda result: result.get("timing_ms", {}).get("extraction_and_approach_ms"),
          "transport_ms": lambda result: result.get("timing_ms", {}).get("transport_ms"),
          "conveyor_suffix_ms": lambda result: result.get("conveyor_suffix_ms")}
PHASES = ("approach", "contact_approach", "extract", "transport", "conveyor_place", "return_home")


def load(directory):
    runs = {}
    for path in sorted((directory / "runs").glob("*.json")):
        record = json.loads(path.read_text())
        runs[(record["task"], record["seed"], record["arm"])] = (record, directory / "full" / path.name)
    return runs


def phase_frames(path):
    result = json.loads(path.read_text())
    frames, phases = np.asarray(result["frames"]), np.asarray(result["phases"])
    return result, {phase: frames[phases == phase] for phase in PHASES}


def compare(reference, variant):
    out = dict(runs=len(variant), success=sum(bool(record.get("success")) for record, _ in variant.values()),
               failures=collections.Counter(record.get("failure_class") for record, _ in variant.values() if not record.get("success")),
               lost=[], gained=[], changes={stage: [] for stage in STAGES}, round_motion=[], changed_phases=collections.Counter(),
               changed_runs=0, kinds=collections.Counter())
    for key, (record, path) in sorted(variant.items()):
        if key not in reference:
            continue
        base, base_path = reference[key]
        if base.get("success") and not record.get("success"):
            out["lost"].append(f"{key[0]}-s{key[1]}: {record.get('failure_class')} {record.get('blocked_details', '')[:120]}")
        if record.get("success") and not base.get("success"):
            out["gained"].append(f"{key[0]}-s{key[1]}")
        if not (base.get("success") and record.get("success")):
            continue
        out["round_motion"].append((record["executed"]["round_motion_s"] / base["executed"]["round_motion_s"] - 1) * 100)
        if not (path.exists() and base_path.exists()):
            continue
        (result, frames), (base_result, base_frames) = phase_frames(path), phase_frames(base_path)
        for stage, get in STAGES.items():
            if get(result) is not None and get(base_result):
                out["changes"][stage].append((get(result) / get(base_result) - 1) * 100)
        changed = [phase for phase in PHASES if frames[phase].shape != base_frames[phase].shape
                   or not np.allclose(frames[phase], base_frames[phase], atol=1e-9)]
        out["changed_phases"].update(changed)
        out["changed_runs"] += bool(changed)
        out["kinds"]["top" if record["executed"]["top_grasp"] else "side"] += bool(changed)
    out["paired"] = len(out["round_motion"])
    out["planning_paired_median_pct"] = {stage: float(np.median(values)) if values else None for stage, values in out.pop("changes").items()}
    out["round_motion_paired_median_pct"] = float(np.median(out["round_motion"])) if out["round_motion"] else None
    out["round_motion_paired_max_pct"] = float(np.max(out.pop("round_motion"))) if out["paired"] else None
    out["failures"], out["changed_phases"], out["kinds"] = dict(out["failures"]), dict(out["changed_phases"]), dict(out["kinds"])
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", nargs=2, metavar=("LABEL", "RUN_DIR"), required=True)
    parser.add_argument("--variant", nargs=2, action="append", metavar=("LABEL", "RUN_DIR"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    reference = load(Path(args.reference[1]))
    report = dict(reference=args.reference[0], reference_runs=len(reference),
                  reference_success=sum(bool(record.get("success")) for record, _ in reference.values()), variants={})
    for label, directory in args.variant:
        report["variants"][label] = compare(reference, load(Path(directory)))
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=1) + "\n")
    print(f"reference {args.reference[0]}: success {report['reference_success']}/{report['reference_runs']}")
    for label, row in report["variants"].items():
        planning = ", ".join(f"{stage} {value:+.1f}%" for stage, value in row["planning_paired_median_pct"].items() if value is not None)
        print(f"{label}: success {row['success']}/{row['runs']} (lost {len(row['lost'])}, gained {len(row['gained'])}) | paired {row['paired']}"
              f" | planning median {planning} | round motion median {row['round_motion_paired_median_pct']:+.1f}%"
              f" (max {row['round_motion_paired_max_pct']:+.1f}%) | runs with changed frames {row['changed_runs']} {row['kinds']}"
              f" phases {row['changed_phases']}")
        for line in row["lost"]:
            print("   lost", line)


if __name__ == "__main__":
    main()
