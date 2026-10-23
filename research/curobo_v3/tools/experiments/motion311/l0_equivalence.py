#!/usr/bin/env python3
"""MOTION-311 L0: the 15-round wall sequence with the new hooks must equal the frozen baseline frame for frame.

Modes:
  plain         TimedSingleArmPlanner as shipped (hooks present, default behaviour)
  instrumented  DockingOffsetMixin at (0, 0) + SearchRecorder(k=0) + every hook overridden by an explicit
                pass-through, so the hook call paths themselves are exercised
Frames, selected candidates, lazy rankings, and the non-timing fields of the search statistics are compared
with the frozen round JSONs; any difference exits with status 1.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from curobo_core.contracts import PlannerAssets
from v3_scene_variants import DockingOffsetMixin
from v3_search_recorder import SearchRecorder
from v3_timed_single_arm_planner import TimedSingleArmPlanner
from v3_timed_wall_sequence import WALL_SEQUENCE

STATS = ("transport_search_stats", "conveyor_search_stats", "conveyor_home_search_stats", "conveyor_place_search_stats")


class PassThroughPlanner(DockingOffsetMixin, TimedSingleArmPlanner):
    hook_calls = None

    def count(self, name):
        self.hook_calls[name] = self.hook_calls.get(name, 0) + 1

    def rank_contacts(self, boxes):
        self.count("rank_contacts")
        return self.baseline_ranking(boxes, self.generate_contacts(boxes))

    def lazy_batches(self, boxes, ranked):
        self.count("lazy_batches")
        yield from super().lazy_batches(boxes, ranked)

    def single_contact_order(self, boxes, contacts):
        self.count("single_contact_order")
        return list(super().single_contact_order(boxes, contacts))

    def order_transport_goals(self, start, candidates):
        self.count("order_transport_goals")
        return super().order_transport_goals(start, candidates)

    def placement_goal_order(self, root, shortlisted, distances, baseline_scores):
        self.count("placement_goal_order")
        return None


def untimed_stats(stats):
    return {key: value for key, value in (stats or {}).items() if not key.endswith("_ms")}


def compare(result, frozen):
    frames = np.asarray(result.get("frames", []), dtype=np.float64)
    reference = np.asarray(frozen.get("frames", []), dtype=np.float64)
    report = dict(success=result.get("success"), frozen_success=frozen.get("success"), frames=len(frames),
                  frozen_frames=len(reference))
    report["frame_max_abs_diff"] = float(np.abs(frames - reference).max()) if frames.shape == reference.shape and len(frames) else None
    report["selected_equal"] = (result.get("selected_candidate"), result.get("selected_original_contact_index")) == \
        (frozen.get("selected_candidate"), frozen.get("selected_original_contact_index"))
    report["ranking_equal"] = (result.get("lazy_extraction") or {}).get("ranking") == (frozen.get("lazy_extraction") or {}).get("ranking")
    report["stats_equal"] = all(untimed_stats(result.get(key)) == untimed_stats(frozen.get(key)) for key in STATS)
    report["motion_duration_equal"] = result.get("motion_duration_s") == frozen.get("motion_duration_s")
    report["equal"] = bool(report["frame_max_abs_diff"] == 0.0 and report["selected_equal"] and report["ranking_equal"]
                           and report["stats_equal"] and report["motion_duration_equal"] and report["success"] == report["frozen_success"])
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frozen", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mode", choices=("plain", "instrumented"), required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    planner_class = PassThroughPlanner if args.mode == "instrumented" else TimedSingleArmPlanner
    planner = planner_class(PlannerAssets(), .025)
    if args.mode == "instrumented":
        planner.hook_calls = {}
    planner.warmup_resources()
    rounds, began = [], time.perf_counter()
    recorder = SearchRecorder(0) if args.mode == "instrumented" else None
    if recorder:
        recorder.__enter__()
    try:
        for number, boxes in enumerate(WALL_SEQUENCE, start=1):
            planner.configure_task(boxes)
            result = planner.full_cycle(boxes)
            frozen = json.loads((args.frozen / f"round-{number:02d}.json").read_text())
            row = dict(round=number, task=boxes, **compare(result, frozen))
            rounds.append(row)
            print(json.dumps(row, ensure_ascii=False), flush=True)
    finally:
        if recorder:
            recorder.__exit__(None, None, None)
    summary = dict(mode=args.mode, rounds=rounds, all_equal=all(row["equal"] for row in rounds),
                   hook_calls=getattr(planner, "hook_calls", None), recorded_searches=len(recorder.calls) if recorder else None,
                   wall_s=time.perf_counter() - began)
    (args.output / f"l0-{args.mode}.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1) + "\n")
    print(json.dumps({key: summary[key] for key in ("mode", "all_equal", "hook_calls", "recorded_searches", "wall_s")}, ensure_ascii=False))
    raise SystemExit(0 if summary["all_equal"] else 1)


if __name__ == "__main__":
    main()
