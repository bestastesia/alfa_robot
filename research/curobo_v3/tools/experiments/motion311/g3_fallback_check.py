#!/usr/bin/env python3
"""G3 success/abstention/full-fallback regression on dev (GPU lock required).

Inject one CycleBlocked after a complete selected suffix, so the real full-cycle fallback replans from Home.
Compare successful complete frames with arm A; also test explicit abstention with an unattainable threshold.
This is a fault-injection check, not an efficacy or success-rate sample.
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from curobo_core.contracts import PlannerAssets
from curobo_core.planner import CycleBlocked
from v3_motion311_ab import ABPlanner, load_models, run_arm
from v3_search_recorder import SearchRecorder


class FaultPlanner(ABPlanner):
    inject_failure = False
    injected = 0

    def conveyor_suffix(self, boxes, report, append, progress):
        super().conveyor_suffix(boxes, report, append, progress)
        if self.inject_failure and self.arm != "A" and self.selection_active():
            self.inject_failure = False
            self.injected += 1
            raise CycleBlocked("龙头车放置RRT", "G3 fault injection after completed suffix", report)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--models", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    planner = FaultPlanner(PlannerAssets(), .025)
    planner.setup_selection(args.models / "assets", load_models(args.models))
    planner.contact_value = "explicit"
    cases = []
    for boxes in ({"left": 17, "right": 21}, {"left": 23}):
        kind = "dual" if len(boxes) == 2 else "single"
        with SearchRecorder(0) as recorder:
            planner.theta = .5
            baseline = run_arm(planner, boxes, kind, "A", recorder)
            planner.inject_failure = True
            before = planner.injected
            fallback = run_arm(planner, boxes, kind, "B", recorder)
            planner.inject_failure = False
            planner.theta = 2.
            abstained = run_arm(planner, boxes, kind, "B", recorder)
        passed = (baseline["class_"] == fallback["class_"] == abstained["class_"] == "success"
                  and baseline["frames_digest"] == fallback["frames_digest"] == abstained["frames_digest"]
                  and planner.injected == before + 1 and planner.arm == "B"
                  and bool(fallback["selection"].get("fallback")) and abstained["selection"]["abstained"]
                  and fallback["total_ms"] >= fallback["selection"]["fallback"]["first_total_ms"])
        cases.append(dict(task=boxes, passed=passed, baseline=baseline, fallback=fallback, abstained=abstained))
    output = dict(all_equal=all(row["passed"] for row in cases), cases=cases)
    args.output.write_text(json.dumps(output, ensure_ascii=False, indent=1) + "\n")
    print(json.dumps(dict(all_equal=output["all_equal"], cases=[dict(task=r["task"], passed=r["passed"]) for r in cases])), flush=True)
    raise SystemExit(0 if output["all_equal"] else 1)


if __name__ == "__main__":
    main()
