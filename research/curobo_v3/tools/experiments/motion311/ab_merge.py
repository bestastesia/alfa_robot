#!/usr/bin/env python3
"""Merge arms of the same instances from several A/B runs (e.g. A/B0 from one run, re-run B/Bx from another).

Later folders override earlier ones per arm; every merged record lists the source folder of each arm. Planning results
are deterministic per arm, so arms planned in separate runs remain paired; timing comparisons across runs carry the
run-to-run load difference and are flagged in the record.
"""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=Path, nargs="+", required=True, help="A/B output folders, later ones win per arm")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    merged = {}
    for run in args.runs:
        for path in sorted((run / "instances").glob("*.json")):
            record = json.loads(path.read_text())
            target = merged.setdefault(record["instance"], dict(record, arms={}, sources={}))
            for arm, entry in record["arms"].items():
                target["arms"][arm] = entry
                target["sources"][arm] = str(run)
    folder = args.output / "instances"
    folder.mkdir(parents=True, exist_ok=True)
    for instance, record in merged.items():
        record["timing_across_runs"] = len(set(record["sources"].values())) > 1
        (folder / f"{instance}.json").write_text(json.dumps(record, ensure_ascii=False) + "\n")
    print(json.dumps(dict(instances=len(merged), runs=[str(run) for run in args.runs]), ensure_ascii=False))


if __name__ == "__main__":
    main()
