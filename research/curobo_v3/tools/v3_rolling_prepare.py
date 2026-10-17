#!/usr/bin/env python3
"""Generate fresh PR65 timed baseline + exact online phase snapshots, no hardware IO."""
import argparse
import json
from pathlib import Path
import time
from curobo_core.contracts import PlannerAssets
from curobo_core.rolling_backend import RollingBackend
from v3_timed_wall_sequence import WALL_SEQUENCE


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--round', type=int, choices=range(len(WALL_SEQUENCE)))
    parser.add_argument('--sequence', action='store_true', help='fresh 15-round/25-box Mentor baseline with phase snapshots')
    args = parser.parse_args()
    if args.sequence and args.round is not None:
        parser.error('--sequence owns rounds; do not combine with --round')
    started = time.perf_counter()
    planner = RollingBackend(PlannerAssets())
    planner.warmup_resources()
    initialization = time.perf_counter()-started
    began = time.perf_counter()
    if args.sequence:
        result = dict(sequence=True, success=False, baseline="PR65@541fdbc", rounds=[], completed_box_ids=[])
        for index, boxes in enumerate(WALL_SEQUENCE):
            print(f"round {index+1}/{len(WALL_SEQUENCE)}: {boxes}", flush=True)
            record = planner.export_cycle(boxes, lambda text: print(text, flush=True))
            record.update(task=boxes, status="completed" if record["result"].get("success") else "failed")
            result["rounds"].append(record)
            if not record["result"].get("success"):
                result["error"] = record["result"].get("error")
                break
            result["completed_box_ids"].extend(boxes.values())
        else:
            result["success"] = True
        result["remaining_box_ids"] = sorted(set(range(25))-set(result["completed_box_ids"]))
        success = result["success"]
        baseline_ms = sum(r["result"].get("total_ms", 0.) for r in result["rounds"])
        timing_ms = sum(r["result"].get("trajectory_timing_ms", 0.) for r in result["rounds"])
    else:
        result = planner.export_cycle(WALL_SEQUENCE[args.round or 0], lambda text: print(text, flush=True))
        success = bool(result["result"].get("success"))
        baseline_ms = result["result"].get("total_ms")
        timing_ms = result["result"].get("trajectory_timing_ms")
    result.update(initialization_s=initialization, preparation_wall_s=time.perf_counter()-began)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False)+'\n')
    print(json.dumps({'success': success,
                      'baseline_planning_ms': baseline_ms,
                      'trajectory_timing_ms': timing_ms,
                      'preparation_wall_s': result['preparation_wall_s']}), flush=True)
    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
