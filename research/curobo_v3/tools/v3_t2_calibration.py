#!/usr/bin/env python3
"""Derive per-stage experimental t2 candidates from frozen rolling evidence."""
import argparse
from collections import defaultdict
import glob
import json
import math
from pathlib import Path

import numpy as np


STAGES = ("approach", "contact_approach", "extract", "transport", "conveyor_place", "return_home")
CURRENT_T2 = {"approach": 1.8, "contact_approach": .2, "extract": .2,
              "transport": 1.8, "conveyor_place": .85, "return_home": 1.8}


def round_up(value, quantum=.05):
    return math.ceil(value / quantum - 1e-12) * quantum


def collect_paths(root):
    report = json.loads((root / "acceptance-report-current.json").read_text())
    paths = set()
    for case in report["cases"]:
        if case["fault"] != "none":
            continue
        path = Path(case["matrix"]).parent / f"{case['name']}.json"
        if path.is_file():
            paths.add(path)
    for directory in ("all-rounds-free-clear", "all-rounds-contact-clear", "all-rounds-extract-clear"):
        paths.update(Path(path) for path in glob.glob(str(root / directory / "r*-*.json")))
    return sorted(paths)


def stage_rows(paths):
    stages = defaultdict(list)
    for path in paths:
        document = json.loads(path.read_text())
        stage = document.get("stage")
        if stage not in STAGES:
            continue
        for solve in document.get("solves", []):
            if "compute_wall_s" not in solve or solve.get("terminal"):
                continue
            spans = {name: end - start for name, (start, end) in solve.get("spans", {}).items()}
            residual = max(0., solve["compute_wall_s"] - sum(spans.values()))
            stages[stage].append({"wall_s": solve["compute_wall_s"], "spans": spans,
                                  "residual_s": residual, "scenario": document.get("scenario"),
                                  "strategy": solve.get("strategy"), "source": str(path)})
    return stages


def calibrate(rows, margin=.25, quantum=.05):
    walls = np.asarray([row["wall_s"] for row in rows])
    signatures = defaultdict(list)
    for row in rows:
        signatures[tuple(sorted(row["spans"]))].append(row)
    signature_bounds = []
    for signature, group in signatures.items():
        bound = sum(max(row["spans"][name] for row in group) for name in signature)
        bound += max(row["residual_s"] for row in group)
        signature_bounds.append({"signature": signature, "samples": len(group), "bound_s": bound,
                                 "observed_max_s": max(row["wall_s"] for row in group)})
    modeled = max(signature_bounds, key=lambda item: item["bound_s"])
    candidate = round_up(modeled["bound_s"] * (1 + margin), quantum)
    return {
        "samples": len(rows), "median_s": float(np.median(walls)),
        "p95_s": float(np.percentile(walls, 95)), "p99_s": float(np.percentile(walls, 99)),
        "observed_max_s": float(walls.max()),
        "modeled_branch_bound_s": modeled["bound_s"],
        "modeled_branch_samples": modeled["samples"],
        "modeled_branch_signature": list(modeled["signature"]),
        "margin_ratio": margin, "rounding_quantum_s": quantum,
        "candidate_t2_s": candidate,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--margin", type=float, default=.25)
    parser.add_argument("--quantum-s", type=float, default=.05)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.margin < 0 or args.quantum_s <= 0:
        parser.error("nonnegative margin and positive quantum required")
    stages = stage_rows(collect_paths(args.evidence_root))
    result = {"schema_version": 1, "evidence_root": str(args.evidence_root.resolve()),
              "method": "max composable control-flow signature span + margin, rounded upward",
              "stages": {stage: {"current_t2_s": CURRENT_T2[stage],
                                 **calibrate(stages[stage], args.margin, args.quantum_s)}
                         for stage in STAGES}}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({stage: values["candidate_t2_s"] for stage, values in result["stages"].items()},
                     ensure_ascii=False))


if __name__ == "__main__":
    main()
