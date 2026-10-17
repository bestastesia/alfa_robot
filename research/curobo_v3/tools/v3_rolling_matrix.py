#!/usr/bin/env python3
"""Serial, same-runtime GPU acceptance matrix. Every run keeps its raw evidence."""
import argparse
import itertools
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

import numpy as np

from v3_verify_rolling import audit
from curobo_core.contracts import PlannerAssets

FAULTS = {
    "compute_delay": ("deadline",), "link_delay": ("delivered", "communication deadline"),
    "stale_scene": ("scene is stale",), "feedback_error": ("execution positions",),
    "scene_changed": ("scene changed",), "worker_failure": ("worker exited", "EOFError"),
    "solver_failure": ("injected solver failure",), "admission_delay": ("admission deadline",),
}


def invoke(command, log, timeout):
    with log.open("w") as stream:
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            return process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            return -1



def summarize_reports(reports):
    """Pool raw spans within each stage/scenario, never average run percentiles."""
    groups = {}
    for report in reports:
        key = f"{report['stage']}/{report['scenario']}/{report.get('fault', 'none')}"
        group = groups.setdefault(key, {"runs": 0, "completed": 0, "rejected": 0,
                                       "triggers": 0, "splices": 0, "deadline_misses": 0,
                                       "optimized": 0, "revalidated": 0, "censored_computes": 0,
                                       "raw": {}})
        group["runs"] += 1
        group["completed" if report["success"] else "rejected"] += 1
        for target, source in (("triggers", "trigger_count"), ("splices", "splice_count"),
                               ("deadline_misses", "deadline_miss_count"),
                               ("optimized", "optimization_success_count"),
                               ("revalidated", "revalidated_fallback_count"),
                               ("censored_computes", "censored_compute_count")):
            group[target] += report.get(source, 0)
        def record(name, value):
            if not np.isfinite(value) or value < 0:
                raise ValueError(f"invalid recorded duration: {name}")
            group["raw"].setdefault(name, []).append(value)
        for solve in report.get("solves", []):
            if "compute_wall_s" in solve:
                record("compute_wall_s", solve["compute_wall_s"])
            for name, (start, end) in solve.get("spans", {}).items():
                record(f"stage_{name}_s", end-start)
        for event in report.get("events", []):
            for name in ("available_latency_s", "accepted_latency_s", "feedback_latency_s"):
                if name in event:
                    record(name, event[name])
    for group in groups.values():
        group["completion_rate"] = group["completed"] / group["runs"]
        group["latencies"] = {name: {"count": len(values), "median": float(np.median(values)),
                                    "p95": float(np.percentile(values, 95)), "max": max(values)}
                              for name, values in group.pop("raw").items()}
    return groups


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--initial-cycle", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--stages", nargs="+", choices=("approach", "transport", "return_home"),
                        default=["approach", "transport", "return_home"])
    parser.add_argument("--scenarios", nargs="+", choices=("clear", "static", "moving", "appear", "depart"),
                        default=["clear", "static", "moving", "appear", "depart"])
    parser.add_argument("--seeds", nargs="+", type=int, default=[20261003, 20261004, 20261005])
    parser.add_argument("--rounds", nargs="+", type=int, default=[0])
    parser.add_argument("--slices", type=int, help="explicit n; each stage must satisfy n<=t1/(t2+t3)")
    parser.add_argument("--compute-bound-s", type=float, required=True)
    parser.add_argument("--link-bound-s", type=float, required=True)
    parser.add_argument("--link-delay-s", type=float, default=0.)
    parser.add_argument("--execution-timeout-s", type=float, default=120.)
    parser.add_argument("--faults", nargs="*", choices=tuple(FAULTS), default=[])
    parser.add_argument("--gpu-audit", action="store_true")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    tools = Path(__file__).parent
    matrix = {"runs": [], "complete": False, "configuration": {**vars(args),
               "initial_cycle": str(args.initial_cycle), "output_dir": str(args.output_dir)}}
    cases = [(*case, "none") for case in itertools.product(args.rounds, args.stages, args.scenarios, args.seeds)]
    cases += [(args.rounds[0], args.stages[0], "clear", args.seeds[0], fault) for fault in args.faults]
    for number, (round_index, stage, scenario, seed, fault) in enumerate(cases, 1):
        name = f"r{round_index}-{stage}-{scenario}-{seed}-{fault}"
        result = args.output_dir / f"{name}.json"
        if result.exists():
            raise FileExistsError(f"refusing to overwrite acceptance evidence: {result}")
        command = [sys.executable, str(tools / "v3_rolling_replan.py"),
                   "--initial-cycle", str(args.initial_cycle), "--stage", stage,
                   "--scenario", scenario, "--seed", str(seed), "--round", str(round_index), "--fault", fault,
                   "--compute-bound-s", str(args.compute_bound_s), "--link-bound-s", str(args.link_bound_s),
                   "--link-delay-s", str(args.link_delay_s), "--execution-timeout-s", str(args.execution_timeout_s),
                   "--output", str(result)]
        if args.slices is not None:
            command.extend(("--slices", str(args.slices)))
        row = {"name": name, "command": command, "verified": False}
        row["exit_code"] = invoke(command, result.with_suffix(".log"), 210 + args.execution_timeout_s)
        try:
            report = json.loads(result.read_text())
            row["audit"] = audit(report, PlannerAssets().robot_config, fault == "none", scenario != "clear")
            if fault != "none" and not any(token in report.get("error", "") for token in FAULTS[fault]):
                raise ValueError("rejected for a different reason than the injected fault")
            if args.gpu_audit and fault == "none":
                verify = [sys.executable, str(tools / "v3_verify_rolling.py"), "--result", str(result), "--gpu"]
                if scenario != "clear":
                    verify.append("--require-obstacle-response")
                if invoke(verify, result.with_suffix(".audit.log"), 180) != 0:
                    raise ValueError("independent GPU audit failed; see .audit.log")
            row["verified"] = True
            row["statistics"] = report["statistics"]
        except (ValueError, KeyError, OSError) as error:
            row["error"] = str(error)
        matrix["runs"].append(row)
        matrix["complete"] = number == len(cases)
        matrix["passed"] = sum(r["verified"] for r in matrix["runs"])
        temporary = args.output_dir / "matrix.tmp"
        temporary.write_text(json.dumps(matrix, ensure_ascii=False, indent=2) + "\n")
        temporary.replace(args.output_dir / "matrix.json")
        print(f"{number}/{len(cases)} {name}: {'PASS' if row['verified'] else row.get('error')}", flush=True)
    matrix["summary"] = summarize_reports(
        json.loads((args.output_dir / f"{row['name']}.json").read_text())
        for row in matrix["runs"] if (args.output_dir / f"{row['name']}.json").exists())
    temporary = args.output_dir / "matrix.tmp"
    temporary.write_text(json.dumps(matrix, ensure_ascii=False, indent=2) + "\n")
    temporary.replace(args.output_dir / "matrix.json")
    if matrix["passed"] != len(cases):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
