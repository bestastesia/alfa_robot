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
    "solver_failure": ("injected solver failure",),
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
    if matrix["passed"] != len(cases):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
