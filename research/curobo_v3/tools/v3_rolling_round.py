#!/usr/bin/env python3
"""Run all six rolling stages through one prewarmed CUDA worker; simulation only."""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace

from v3_rolling_replan import (close_persistent_worker, persistent_stage_config,
                               prime_persistent_worker, progress_line, run,
                               stage_input, start_persistent_worker)


STAGES = ("approach", "contact_approach", "extract", "transport", "conveyor_place", "return_home")
STAGE_BUDGETS = {
    "approach": (1., .15, .05), "contact_approach": (.1, .15, .05),
    "extract": (.35, .15, .05), "transport": (.75, .15, .05),
    "conveyor_place": (.9, .15, .05), "return_home": (1., .15, .05),
}


def stage_args(args, stage):
    t2, t3, t4 = STAGE_BUDGETS[stage]
    scenario = args.scenario if stage == args.obstacle_stage else "clear"
    return SimpleNamespace(
        initial_cycle=args.initial_cycle, round=0, stage=stage, scenario=scenario,
        obstacle_center=args.obstacle_center if scenario != "clear" else None,
        obstacle_velocity=args.obstacle_velocity if scenario != "clear" else None,
        obstacle_size=args.obstacle_size, fault="none", compute_bound_s=t2, link_bound_s=t3,
        admission_bound_s=t4, link_delay_s=min(args.link_delay_s, t3), slices=None,
        execution_timeout_s=args.execution_timeout_s, disable_braking_fallback=False,
        braking_duration_s=args.braking_duration_s, max_feedback_age_s=.1,
        seed=args.seed, output=args.output_dir / f"{stage}.json", stream_progress=args.stream_progress)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--initial-cycle", type=Path, required=True)
    parser.add_argument("--obstacle-stage", choices=STAGES, default="transport")
    parser.add_argument("--scenario", choices=("static", "moving", "appear", "depart"), default="static")
    parser.add_argument("--obstacle-center", type=float, nargs=3, required=True)
    parser.add_argument("--obstacle-size", type=float, nargs=3, default=(.08, .08, .08))
    parser.add_argument("--obstacle-velocity", type=float, nargs=3, default=(0., 0., 0.))
    parser.add_argument("--link-delay-s", type=float, default=.02)
    parser.add_argument("--admission-bound-s", type=float, default=.1,
                        help="t4 admission/contract-validation budget")
    parser.add_argument("--braking-duration-s", type=float, default=.5)
    parser.add_argument("--execution-timeout-s", type=float, default=120.)
    parser.add_argument("--seed", type=int, default=20261003)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--stream-progress", action="store_true")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    record = json.loads(args.initial_cycle.read_text())
    stage_options = [stage_args(args, stage) for stage in STAGES]
    shared = start_persistent_worker()
    reports = []
    try:
        configs = []
        for options in stage_options:
            initial, snapshot, boxes, payload = stage_input(record, options.stage)
            configs.append(persistent_stage_config(initial, snapshot, boxes, payload, options))
        prime = prime_persistent_worker(shared, configs)
        if args.stream_progress:
            print(progress_line("rolling_service_ready",
                                service_initialization_s=shared["service"]["initialization_s"],
                                warmup_s=prime["wall_s"],
                                warmed_topologies=prime["warmed_topologies"]), flush=True)
        for options in stage_options:
            reports.append(run(options, shared=shared))
    finally:
        close_persistent_worker(shared)
    summary = {
        "success": len(reports) == len(STAGES) and all(report["success"] for report in reports),
        "worker_initialization_s": shared["service"]["initialization_s"],
        "worker_warmup_s": prime["wall_s"],
        "warmed_topologies": prime["warmed_topologies"],
        "stages": [{"stage": report["stage"], "success": report["success"],
                    "splice_count": report["splice_count"],
                    "motion_wall_s": report["completed_at"] - report["started_at"],
                    "worker_stage_initialization_s": report["startup"]["initialization_s"],
                    "warmup": report["startup"]["warmup"]} for report in reports],
    }
    (args.output_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    if args.stream_progress:
        print(progress_line("round_finished", **summary), flush=True)
    print(json.dumps(summary, ensure_ascii=False), flush=True)
    if not summary["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
