"""Headless planning demo. Optionally records timed TrajOpt segments; never sends hardware commands."""

import argparse
import json
from pathlib import Path
import sys

from curobo_core.contracts import PlanRequest, PlannerAssets
from curobo_core.planner import FullCyclePlanner
from curobo_core.fixtures import WallLayout, add_wall_arguments, wall_layout_from_args


def main():
    defaults = PlannerAssets()
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("robot_config", "mobile_robot_config", "urdf", "named_poses", "box_fit", "target_poses"):
        parser.add_argument("--" + name.replace("_", "-"), type=Path, default=getattr(defaults, name))
    parser.add_argument("--request", type=Path)
    parser.add_argument("--search-seed", type=int, help="RRT sampling seed; IK and search budgets are unchanged")
    parser.add_argument("--trajopt-rrt", action="store_true",
                        help="seed cuRobo TrajOpt from 15-DoF RRT segments; dynamics/torque stay disabled")
    parser.add_argument("--trajopt-interpolation-dt", type=float, default=0.025,
                        help="seconds between optimized q/qdot/qddot/jerk samples")
    parser.add_argument("--suction-mode", choices=("auto", "side", "top"), default=None)
    parser.add_argument("--left-box", type=int, default=24)
    parser.add_argument("--right-box", type=int, default=20)
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sequence", action="store_true", help="plan every occupied wall cell, stopping at the first failed round")
    add_wall_arguments(parser)
    args = parser.parse_args()
    if args.search_seed is not None and not 0 <= args.search_seed < 2**32:
        parser.error("--search-seed must be an unsigned 32-bit integer")
    if args.trajopt_interpolation_dt <= 0:
        parser.error("--trajopt-interpolation-dt must be positive")
    try:
        args.wall_layout = wall_layout_from_args(args)
    except (ValueError, TypeError) as error:
        parser.error(str(error))
    if args.request and args.wall_layout != WallLayout():
        parser.error("--request already owns scene geometry; do not combine it with wall overrides")
    if args.runs < 1:
        parser.error("runs must be positive")
    if args.sequence and args.suction_mode not in (None, "auto"):
        parser.error("wall sequence selects suction per round; use --request to force a mode")
    if args.sequence and (args.request or args.runs != 1 or args.left_box != 24 or args.right_box != 20):
        parser.error("--sequence owns its rounds; do not combine it with --request, repeated --runs or box selection")
    planner = FullCyclePlanner(args)
    if args.sequence:
        result = planner.plan_sequence(planner.snapshot, lambda message: print(message, file=sys.stderr, flush=True),
                                       search_seed=args.search_seed)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
        print(json.dumps({k: result.get(k) for k in ('success', 'completed_box_ids', 'remaining_box_ids', 'total_ms', 'motion_duration_s', 'error')}, ensure_ascii=False))
        raise SystemExit(0 if result['success'] else 1)
    request = PlanRequest.from_dict(json.loads(args.request.read_text())) if args.request else planner.demo_request(
        {"left": args.left_box, "right": args.right_box})
    from dataclasses import replace
    if args.suction_mode is not None:
        request = replace(request, suction_mode=args.suction_mode)
    if args.search_seed is not None:
        request = replace(request, search_seed=args.search_seed)
    results = []
    for _ in range(args.runs):
        result = planner.plan_request(request, lambda message: print(message, file=sys.stderr, flush=True))
        results.append(result)
        if not result["success"]:
            break
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({"request": request.to_dict(), "results": results},
                                      ensure_ascii=False, indent=2) + "\n")
    print(json.dumps([{key: result.get(key) for key in (
        "success", "total_ms", "selected_candidate", "final_home_error", "error", "scene_id")}
        for result in results], ensure_ascii=False), flush=True)
    if not results[-1]["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
