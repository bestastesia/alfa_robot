#!/usr/bin/env python3
"""IK Benchmark — Python wrapper for batch testing multiple solvers/groups."""

import argparse
import json
import subprocess
import sys
from pathlib import Path

SOLVERS = {
    "kdl":    "kdl_kinematics_plugin/KDLKinematicsPlugin",
    "bio_ik": "bio_ik/BioIKKinematicsPlugin",
}

GROUPS = ["left_arm", "right_arm", "dual_arm_with_base"]


def run_benchmark(group: str, solver: str, samples: int = 50,
                  timeout: float = 2.0, perturb: float = 0.1,
                  free_joint6: bool = False, jsonl_path: str = "") -> dict:
    cmd = [
        "ros2", "run", "alfa_robot_benchmarks", "ik_benchmark",
        "--group", group,
        "--solver", solver,
        "--samples", str(samples),
        "--timeout", str(timeout),
        "--perturb", str(perturb),
    ]
    if free_joint6:
        cmd.append("--free-joint6")
    if jsonl_path:
        cmd += ["--jsonl", jsonl_path]

    result = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    if result.returncode != 0:
        print(f"ERROR: {solver} on {group} failed", file=sys.stderr)
        print(result.stderr, file=sys.stderr)
        return {"solver": solver, "group": group, "error": True}

    # Parse output for summary
    lines = result.stdout.strip().split("\n")
    summary = {"solver": solver, "group": group, "error": False}
    for line in lines:
        if "Success rate:" in line:
            parts = line.split()
            summary["success"] = int(parts[2].split("/")[0])
            summary["total"] = int(parts[2].split("/")[1])
            summary["success_rate"] = float(parts[3].strip("()%"))
        elif "Avg time:" in line:
            summary["avg_ms"] = float(line.split()[2].strip("ms"))
        elif "Avg pos err:" in line:
            summary["avg_pos_err"] = float(line.split()[2].strip("m"))
        elif "Avg ori err:" in line:
            summary["avg_ori_err"] = float(line.split()[2].strip("rad"))

    return summary


def main():
    parser = argparse.ArgumentParser(description="IK Benchmark — batch test multiple solvers")
    parser.add_argument("--groups", nargs="+", default=["left_arm"],
                        choices=GROUPS, help="Groups to test")
    parser.add_argument("--solvers", nargs="+", default=["kdl"],
                        choices=list(SOLVERS.keys()), help="Solvers to test")
    parser.add_argument("--samples", type=int, default=50)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--perturb", type=float, default=0.1)
    parser.add_argument("--free-joint6", action="store_true")
    parser.add_argument("--output", type=str, default="", help="Output JSON path")
    args = parser.parse_args()

    results = []
    for group in args.groups:
        for solver_key in args.solvers:
            solver = SOLVERS[solver_key]
            jsonl_path = ""
            if args.output:
                jsonl_path = str(Path(args.output).with_suffix(
                    f".{group}_{solver_key}.jsonl"))

            print(f"\n--- {group} + {solver_key} ---")
            r = run_benchmark(group, solver, args.samples, args.timeout,
                              args.perturb, args.free_joint6, jsonl_path)
            results.append(r)

    # Summary table
    print("\n" + "=" * 70)
    print(f"{'Group':<22} {'Solver':<10} {'Rate':>8} {'Avg ms':>8} {'Pos err':>10} {'Ori err':>10}")
    print("-" * 70)
    for r in results:
        if r.get("error"):
            print(f"{r['group']:<22} {r['solver']:<10}  ERROR")
        else:
            print(f"{r['group']:<22} {r['solver']:<10} "
                  f"{r.get('success_rate', 0):>7.1f}% "
                  f"{r.get('avg_ms', 0):>7.1f} "
                  f"{r.get('avg_pos_err', 0):>9.4f} "
                  f"{r.get('avg_ori_err', 0):>9.4f}")

    if args.output:
        with open(args.output, "w") as f:
            json.dump(results, f, indent=2)
        print(f"\nResults saved to {args.output}")


if __name__ == "__main__":
    main()