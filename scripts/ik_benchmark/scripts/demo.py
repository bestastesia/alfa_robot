#!/usr/bin/env python3
"""IK Demo — Quick single-solver demonstration."""

import argparse
import subprocess
import sys

SOLVERS = {
    "kdl":     "kdl_kinematics_plugin/KDLKinematicsPlugin",
    "bio_ik": "bio_ik/BioIKKinematicsPlugin",
}

GROUPS = ["left_arm", "right_arm", "dual_arm_with_base"]


def main():
    parser = argparse.ArgumentParser(description="IK Demo — quick demonstration")
    parser.add_argument("--group", type=str, default="left_arm",
                        choices=GROUPS, help="Planning group")
    parser.add_argument("--solver", type=str, default="kdl",
                        choices=list(SOLVERS.keys()), help="IK solver")
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--free-joint6", action="store_true",
                        help="Don't fix joint6 to 0")
    parser.add_argument("--no-perturb", action="store_true",
                        help="FK → IK directly (no perturbation)")
    args = parser.parse_args()

    solver_plugin = SOLVERS[args.solver]
    cmd = [
        "ros2", "run", "alfa_robot_benchmarks", "ik_demo",
        "--group", args.group,
        "--solver", solver_plugin,
        "--trials", str(args.trials),
        "--timeout", str(args.timeout),
    ]
    if args.free_joint6:
        cmd.append("--free-joint6")
    if args.no_perturb:
        cmd.append("--no-perturb")

    result = subprocess.run(cmd)
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()