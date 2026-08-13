#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

from armmotion_demo.common import (
    parse_task_code,
    planning_task_from_suction_surface_poses,
    suction_surface_poses_for_task,
)
from armmotion_demo.planner_adapter import PlannerAdapter


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="生成十一档五排预抓取后段轨迹缓存")
    parser.add_argument("--source-ws", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--cache-output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--distance-cm", nargs="+", type=int, default=list(range(70, 81)))
    parser.add_argument("--rows", nargs="+", type=int, default=list(range(1, 6)))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_root = args.output_root / time.strftime("run_%Y%m%d_%H%M%S")
    source_root = run_root / "snapshots"
    planner_root = run_root / "planner"
    source_root.mkdir(parents=True, exist_ok=False)
    planner_root.mkdir(parents=True, exist_ok=True)
    planner = PlannerAdapter(
        source_ws=args.source_ws,
        output_root=planner_root,
        rate_hz=30.0,
        max_joint_speed_deg_s=10.0,
        max_joint_acceleration_deg_s2=60.0,
        max_updown_speed_m_s=0.15,
        max_updown_acceleration_m_s2=0.05,
        speed_scale=3.0,
        timeout_s=args.timeout,
        trajectory_cache_enabled=False,
    )
    results = []
    try:
        for distance_cm in args.distance_cm:
            distance_m = distance_cm / 100.0
            sequence_root = source_root / f"x_{distance_cm:02d}cm" / "sequence_generated"
            for row in args.rows:
                fixture = parse_task_code(f"B{row}", distance_m, distance_m)
                left, right = suction_surface_poses_for_task(fixture)
                task = planning_task_from_suction_surface_poses(
                    f"cache_x{distance_cm:02d}_row{row}",
                    left,
                    right,
                    fixture.left_grasp_mode,
                    fixture.right_grasp_mode,
                )
                started = time.monotonic()
                try:
                    plan = planner.compute(task)
                    elapsed_s = time.monotonic() - started
                    task_root = sequence_root / (
                        f"{row:02d}_L{fixture.left_box_id}_R{fixture.right_box_id}"
                    )
                    task_root.mkdir(parents=True, exist_ok=False)
                    shutil.copy2(plan.snapshot_path, task_root / "stage_snapshot.json")
                    results.append(
                        {
                            "distance_cm": distance_cm,
                            "row": row,
                            "success": True,
                            "wall_s": elapsed_s,
                            "snapshot": str(task_root / "stage_snapshot.json"),
                        }
                    )
                    print(
                        f"CACHE_GRID x={distance_m:.2f}m row={row} "
                        f"success wall={elapsed_s:.3f}s",
                        flush=True,
                    )
                except Exception as exc:
                    results.append(
                        {
                            "distance_cm": distance_cm,
                            "row": row,
                            "success": False,
                            "wall_s": time.monotonic() - started,
                            "reason": str(exc),
                        }
                    )
                    print(
                        f"CACHE_GRID x={distance_m:.2f}m row={row} failed: {exc}",
                        flush=True,
                    )
    finally:
        planner.close()

    (run_root / "results.json").write_text(
        json.dumps(results, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    failures = [item for item in results if not item["success"]]
    if failures:
        print(
            f"缓存规划中有 {len(failures)} 条失败，仅写入成功轨迹",
            flush=True,
        )

    generator = Path(__file__).with_name("generate_trajectory_cache.py")
    subprocess.run(
        [
            sys.executable,
            str(generator),
            str(source_root),
            str(args.cache_output),
        ],
        check=True,
    )
    print(f"缓存生成完成: {args.cache_output}", flush=True)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
