#!/usr/bin/env python3
"""Run the 15-stage parallel IK speed-limit benchmark variants.

This is a thin orchestrator around the existing `parallel_ik_benchmark` C++
executable. It writes one JSONL per variant plus a manifest file.
"""

from __future__ import annotations

import argparse
import json
import subprocess
from datetime import datetime
from pathlib import Path


VARIANTS = [
    {
        "id": "serial_50ms",
        "label": "对照组1：串行IK / 50ms",
        "workers": "1",
        "timeout": "0.05",
    },
    {
        "id": "parallel8_50ms",
        "label": "对照组2：并行宽时域IK / 50ms",
        "workers": "8",
        "timeout": "0.05",
    },
    {
        "id": "parallel8_10ms",
        "label": "实验组：并行窄时域IK / 10ms",
        "workers": "8",
        "timeout": "0.01",
    },
]


def main() -> int:
    parser = argparse.ArgumentParser(description="Run parallel IK speed-limit benchmark variants")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("/mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/parallel_ik_speed_limit"),
        help="Directory for JSONL results",
    )
    parser.add_argument("--profile", default="success", help="parallel_ik_benchmark profile, default success")
    parser.add_argument("--limit-episodes", type=int, default=3, help="3 episodes × 5 stages = 15 stages")
    parser.add_argument("--h-candidates", type=int, default=8)
    parser.add_argument("--seed-attempts", type=int, default=32, help="8 × 32 = 256 trials per stage")
    parser.add_argument("--dataset-prefilter", default="sphere")
    parser.add_argument("--base-only-dataset", action="store_true", default=True)
    parser.add_argument("--allow-collision-solutions", action="store_true", help="Pass through to benchmark")
    parser.add_argument("--include-candidates", action="store_true", help="Emit per-trial JSON; much larger files")
    parser.add_argument("--extra-arg", action="append", default=[], help="Extra argument passed to parallel_ik_benchmark")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    manifest = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "intent": "比较每阶段 256 次 IK 求解耗时：串行IK、并行宽时域IK、并行窄时域IK",
        "profile": args.profile,
        "limit_episodes": args.limit_episodes,
        "h_candidates": args.h_candidates,
        "seed_attempts": args.seed_attempts,
        "trial_budget_per_stage": args.h_candidates * args.seed_attempts,
        "variants": [],
    }

    for variant in VARIANTS:
        output = args.output_dir / f"{variant['id']}.jsonl"
        cmd = [
            "ros2", "run", "alfa_robot_benchmarks", "parallel_ik_benchmark",
            "--profile", args.profile,
            "--limit-episodes", str(args.limit_episodes),
            "--base-only-dataset",
            "--dataset-prefilter", args.dataset_prefilter,
            "--workers-list", variant["workers"],
            "--timeout", variant["timeout"],
            "--h-candidates", str(args.h_candidates),
            "--seed-attempts", str(args.seed_attempts),
            "--output", str(output),
        ]
        if args.allow_collision_solutions:
            cmd.append("--allow-collision-solutions")
        if args.include_candidates:
            cmd.append("--include-candidates")
        cmd.extend(args.extra_arg)

        print("\n===", variant["label"], "===")
        print(" ".join(cmd))
        manifest["variants"].append({**variant, "output": str(output), "command": cmd})
        if not args.dry_run:
            subprocess.run(cmd, check=True)

    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    print(f"\nmanifest: {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
