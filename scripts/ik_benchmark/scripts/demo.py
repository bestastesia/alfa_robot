#!/usr/bin/env python3
"""
ALFA Robot IK Demo — 单次求解演示

流程: 随机关节 → FK → 加噪 → IK → 对比误差
展示如何调用 IK 求解器进行单次求解。

用法:
  python3 demo.py --group left_arm --solver trac_ik
  python3 demo.py --group right_arm --solver kdl
  python3 demo.py --group dual_arm_with_base --solver pick_ik
"""

import argparse
import json
import subprocess
import sys


SOLVER_SHORTCUTS = {
    "kdl":     "kdl_kinematics_plugin/KDLKinematicsPlugin",
    "trac_ik": "trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin",
    "pick_ik": "pick_ik/PickIkPlugin",
    "bio_ik":  "bio_ik/BioIKKinematicsPlugin",
}


def find_executable() -> str:
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "alfa_robot_benchmarks"],
            capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            prefix = result.stdout.strip()
            from pathlib import Path
            exe = Path(prefix) / "lib/alfa_robot_benchmarks/ik_demo"
            if exe.exists():
                return str(exe)
    except Exception:
        pass
    return "ros2 run alfa_robot_benchmarks ik_demo"


def main():
    parser = argparse.ArgumentParser(description="ALFA Robot IK 单次求解 Demo")
    parser.add_argument("--group", default="left_arm",
                        help="规划组: left_arm / right_arm / dual_arm_with_base")
    parser.add_argument("--solver", default="trac_ik",
                        help="求解器: kdl / trac_ik / pick_ik / bio_ik")
    parser.add_argument("--timeout", type=float, default=2.0, help="超时(秒)")
    parser.add_argument("--seed", type=int, default=None, help="随机种子")
    args = parser.parse_args()

    exe = find_executable()
    cmd = [exe, "--group", args.group, "--solver", args.solver,
           "--timeout", str(args.timeout)]
    if args.seed is not None:
        cmd += ["--seed", str(args.seed)]

    print(f"调用: {args.solver} [{args.group}]")
    print(f"命令: {cmd}")

    result = subprocess.run(cmd, capture_output=True, text=True, timeout=args.timeout + 10)

    if result.returncode != 0:
        print(f"错误: {result.stderr}")
        sys.exit(1)

    try:
        data = json.loads(result.stdout.strip())
    except json.JSONDecodeError:
        print(result.stdout)
        sys.exit(1)

    # 打印结果
    print(f"\n{'='*50}")
    print(f"  IK 求解结果")
    print(f"{'='*50}")
    print(f"  求解器 : {args.solver}")
    print(f"  规划组 : {args.group}")
    print(f"  成功   : {'YES' if data['success'] else 'NO'}")

    if data["success"]:
        print(f"  耗时   : {data['solve_ms']:.2f} ms")
        print(f"  位置误差 : {data['pos_error']:.6f} m")
        print(f"  姿态误差 : {data['ori_error']:.6f} rad "
              f"({data['ori_error'] * 180 / 3.14159:.2f}°)")
        print(f"  关节解 :")
        for name, val in zip(data["joint_names"], data["joint_values"]):
            print(f"    {name} = {val:.6f}")

    # FK 原始位姿
    fk = data.get("fk_left")
    if fk:
        print(f"\n  FK 原始左臂位姿:")
        print(f"    pos: {fk['pos']}")
        print(f"    quat: {fk['quat']}")

    # IK 目标位姿
    tgt = data.get("target_left")
    if tgt:
        print(f"  IK 目标左臂位姿:")
        print(f"    pos: {tgt['pos']}")
        print(f"    quat: {tgt['quat']}")

    if data.get("fk_right"):
        print(f"\n  FK 原始右臂位姿: {data['fk_right']['pos']}")
        print(f"  IK 目标右臂位姿: {data['target_right']['pos']}")

    print(f"\n  原始 JSON:")
    print(f"  {json.dumps(data, indent=2)[:500]}...")


if __name__ == "__main__":
    main()