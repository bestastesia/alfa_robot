"""Run one strict B1 instance and persist the full timed trajectory for paired replay."""
import argparse
import json
from pathlib import Path

import numpy as np

from motion306_learning.env import LookaheadEnv
from motion306_learning.evaluate import evaluate_instance
from motion306_learning.rankers import B1CostConfig


def find_item(path, instance):
    matches = [item for item in json.loads(path.read_text())["instances"] if item["instance"] == instance]
    if len(matches) != 1:
        raise ValueError(f"expected exactly one instance, found {len(matches)}")
    return matches[0]


def save_trajectory(path, report):
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, frames=np.asarray(report["frames"], dtype=np.float64),
                        phases=np.asarray(report["phases"]), payload=np.asarray(report["payload"], dtype=bool),
                        time_from_start_s=np.asarray(report.get("time_from_start_s", []), dtype=np.float64),
                        joint_names=np.asarray(report["joint_names"]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, required=True)
    parser.add_argument("--instance", required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--trajectory-output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=101)
    args = parser.parse_args()
    item = find_item(args.registry, args.instance)
    env = LookaheadEnv(timed=True, physical=False, contact_pool="capped", place_gate=True)
    record = evaluate_instance(env, item, "b1", B1CostConfig.load(args.config), seed=args.seed, screen=True)
    if record.get("success"):
        save_trajectory(args.trajectory_output, env.report)
        record["trajectory"] = str(args.trajectory_output.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({key: record.get(key) for key in ("instance", "success", "status", "reason",
                                                       "selection_total_ms", "planning_wall_ms",
                                                       "executed_t_path_s", "motion_duration_s")}, ensure_ascii=False))
    return 0 if record.get("success") else 2


if __name__ == "__main__":
    raise SystemExit(main())
