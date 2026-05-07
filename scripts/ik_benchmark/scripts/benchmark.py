#!/usr/bin/env python3
"""
ALFA Robot IK Benchmark — Python 调用脚本

用法:
  # 单臂 TRAC-IK
  python3 benchmark.py --group left_arm --solver trac_ik --samples 20

  # 双臂 pick_ik
  python3 benchmark.py --group dual_arm_with_base --solver pick_ik --samples 20

  # 对比所有求解器
  python3 benchmark.py --mode compare --samples 20
"""

import argparse
import json
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# ── 求解器配置 ─────────────────────────────────────────────────────────────

SOLVER_SHORTCUTS = {
    "kdl":     "kdl_kinematics_plugin/KDLKinematicsPlugin",
    "trac_ik": "trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin",
    "pick_ik": "pick_ik/PickIkPlugin",
    "bio_ik":  "bio_ik/BioIKKinematicsPlugin",
}

# 单臂求解器配置
SINGLE_ARM_CONFIGS = [
    ("left_arm",  "kdl"),
    ("left_arm",  "trac_ik"),
    ("right_arm", "kdl"),
    ("right_arm", "trac_ik"),
]

# 双臂求解器配置
DUAL_ARM_CONFIGS = [
    ("dual_arm_with_base", "pick_ik"),
    ("dual_arm_with_base", "bio_ik"),
]


@dataclass
class SampleResult:
    index: int
    status: str  # success / large_error / failed
    solve_ms: float
    pos_error: float = 0.0
    ori_error: float = 0.0


@dataclass
class BenchmarkResult:
    solver: str
    group: str
    samples: list[SampleResult] = field(default_factory=list)

    @property
    def total(self): return len(self.samples)

    @property
    def success_count(self): return sum(1 for s in self.samples if s.status == "success")

    @property
    def avg_ms(self): return sum(s.solve_ms for s in self.samples) / max(self.total, 1)

    @property
    def avg_pos_error(self):
        solved = [s for s in self.samples if s.status != "failed"]
        return sum(s.pos_error for s in solved) / max(len(solved), 1)

    @property
    def success_rate(self):
        return self.success_count / max(self.total, 1) * 100

    def print_summary(self):
        print(f"\n{'='*60}")
        print(f"  {self.solver}  [{self.group}]")
        print(f"{'='*60}")
        print(f"  样本数     : {self.total}")
        print(f"  成功率     : {self.success_rate:.1f}%")
        print(f"  平均耗时   : {self.avg_ms:.2f} ms")
        print(f"  平均位置误差: {self.avg_pos_error:.6f} m")


# ── 调用 C++ 可执行 ───────────────────────────────────────────────────────

def find_executable() -> str:
    """查找 ik_demo 可执行文件路径"""
    # 优先用 ros2 run
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "alfa_robot_benchmarks"],
            capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            prefix = result.stdout.strip()
            exe = Path(prefix) / "lib/alfa_robot_benchmarks/ik_demo"
            if exe.exists():
                return str(exe)
    except Exception:
        pass
    # fallback
    return "ros2 run alfa_robot_benchmarks ik_demo"


def run_demo(group: str, solver: str, samples: int = 1,
             timeout: float = 2.0, perturb_pos: float = 0.05,
             perturb_ori: float = 0.25, seed: int = 42) -> list[dict]:
    """调用 C++ ik_demo 多次，返回 JSON 结果列表"""
    exe = find_executable()
    results = []

    cmd_base = [
        exe, "--group", group, "--solver", solver,
        "--timeout", str(timeout),
        "--perturb-pos", str(perturb_pos),
        "--perturb-ori", str(perturb_ori),
    ]

    for i in range(samples):
        cmd = cmd_base + ["--seed", str(seed + i)]
        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=timeout + 5)
            if result.returncode == 0 and result.stdout.strip():
                data = json.loads(result.stdout.strip())
                results.append(data)
            else:
                results.append({
                    "success": False, "solve_ms": 0,
                    "pos_error": 0, "ori_error": 0,
                    "error": result.stderr.strip()[:200]
                })
        except subprocess.TimeoutExpired:
            results.append({
                "success": False, "solve_ms": timeout * 1000,
                "pos_error": 0, "ori_error": 0, "error": "timeout"
            })
        except json.JSONDecodeError as e:
            results.append({
                "success": False, "solve_ms": 0,
                "pos_error": 0, "ori_error": 0, "error": f"json parse: {e}"
            })

    return results


def run_benchmark(group: str, solver: str, samples: int = 20,
                  timeout: float = 2.0) -> BenchmarkResult:
    """运行一次 benchmark"""
    print(f"  运行 {solver} [{group}] × {samples} ...", end=" ", flush=True)

    results = run_demo(group, solver, samples, timeout)
    br = BenchmarkResult(solver=solver, group=group)

    for i, r in enumerate(results):
        status = "success" if r.get("success") else "failed"
        if status == "success" and r.get("pos_error", 0) > 0.005:
            status = "large_error"
        br.samples.append(SampleResult(
            index=i, status=status,
            solve_ms=r.get("solve_ms", 0),
            pos_error=r.get("pos_error", 0),
            ori_error=r.get("ori_error", 0),
        ))

    print(f"完成 (成功率 {br.success_rate:.0f}%)")
    return br


# ── main ───────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="ALFA Robot IK Benchmark")
    parser.add_argument("--mode", choices=["single", "dual", "compare", "all"],
                        default="single", help="测试模式")
    parser.add_argument("--group", default="left_arm", help="规划组名")
    parser.add_argument("--solver", default="kdl", help="求解器 (kdl/trac_ik/pick_ik/bio_ik)")
    parser.add_argument("--samples", type=int, default=20, help="样本数")
    parser.add_argument("--timeout", type=float, default=2.0, help="IK 超时(秒)")
    args = parser.parse_args()

    results = []

    if args.mode == "single":
        solver_full = SOLVER_SHORTCUTS.get(args.solver, args.solver)
        br = run_benchmark(args.group, args.solver, args.samples, args.timeout)
        results.append(br)

    elif args.mode == "dual":
        br = run_benchmark(args.group, args.solver, args.samples, args.timeout)
        results.append(br)

    elif args.mode in ("compare", "all"):
        # 单臂对比
        print("\n── 单臂求解器对比 ──")
        for group, solver in SINGLE_ARM_CONFIGS:
            br = run_benchmark(group, solver, args.samples, args.timeout)
            results.append(br)

        # 双臂对比
        print("\n── 双臂求解器对比 ──")
        for group, solver in DUAL_ARM_CONFIGS:
            br = run_benchmark(group, solver, args.samples, args.timeout)
            results.append(br)

    # 打印汇总
    for br in results:
        br.print_summary()

    # 对比表
    if len(results) > 1:
        print(f"\n{'='*60}")
        print(f"  对比汇总")
        print(f"{'='*60}")
        print(f"  {'求解器':<30} {'组':<25} {'成功率':>6} {'平均耗时':>10} {'平均误差':>10}")
        print(f"  {'-'*30} {'-'*25} {'-'*6} {'-'*10} {'-'*10}")
        for br in results:
            print(f"  {br.solver:<30} {br.group:<25} {br.success_rate:>5.1f}% "
                  f"{br.avg_ms:>8.2f}ms {br.avg_pos_error:>8.5f}m")


if __name__ == "__main__":
    main()
