"""Select and optionally execute one strict B2 IK group for one registry instance."""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from motion306_learning.env import LookaheadEnv
from motion306_learning.group_backend import LookaheadGroupBackend
from motion306_learning.group_score import GroupScoreConfig
from motion306_learning.group_selector import GroupBeamConfig, select_group
from motion311_learning.evaluate import instance_seed


def jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(item) for item in value]
    return value


def find_item(path, instance):
    document = json.loads(path.read_text())
    matches = [item for item in document["instances"] if item["instance"] == instance]
    if len(matches) != 1:
        raise ValueError(f"expected exactly one registry instance, found {len(matches)}")
    return matches[0]


def execute_choice(env, choice):
    outcomes = []
    expected_rows = choice.rows[[1, 3, 4]]
    selected_indices = (choice.contact_index, choice.carry_index, choice.place_index)
    for stage, (index, expected) in enumerate(zip(selected_indices, expected_rows)):
        observation = env.observation
        if observation.stage != stage or index < 0 or index >= len(observation.candidates):
            raise RuntimeError(f"preview index does not exist in executed stage {stage} pool: index={index}, count={len(observation.candidates)}")
        if not np.allclose(observation.candidates[index], expected, atol=2e-6, rtol=0.):
            difference = float(np.max(np.abs(observation.candidates[index] - expected)))
            nearest = int(np.argmin(np.max(np.abs(observation.candidates - expected[None]), axis=1)))
            nearest_difference = float(np.max(np.abs(observation.candidates[nearest] - expected)))
            raise RuntimeError(f"previewed group row differs at stage {stage}: index={index}, max_abs={difference}, "
                               f"nearest={nearest}, nearest_max_abs={nearest_difference}")
        began = time.perf_counter()
        result = env.step(index)
        outcomes.append({"stage": stage, "index": index, "wall_ms": (time.perf_counter() - began) * 1000,
                         "terminated": result.terminated, "truncated": result.truncated,
                         "reward": result.reward, "info": result.info})
        if result.terminated or result.truncated:
            break
    success = (len(outcomes) == 3 and outcomes[-1]["info"].get("success") and env.report.get("success") and
               env.report.get("timed_validation", {}).get("success"))
    return bool(success), outcomes


def save_trajectory(path, env):
    report = env.report
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
    parser.add_argument("--trajectory-output", type=Path)
    parser.add_argument("--seed", type=int, default=101)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    item = find_item(args.registry, args.instance)
    score_config = GroupScoreConfig.load(args.config)
    beam_config = GroupBeamConfig.load(args.config)
    env = LookaheadEnv(timed=True, physical=False, contact_pool="capped", place_gate=True)
    reset_began = time.perf_counter()
    planner_seed = instance_seed(item, args.seed)
    env.reset(item, seed=planner_seed)
    reset_ms = (time.perf_counter() - reset_began) * 1000
    backend = LookaheadGroupBackend(env, score_config.edge_collision_samples)
    selected_began = time.perf_counter()
    choice = select_group(backend, score_config, beam_config)
    selection_ms = (time.perf_counter() - selected_began) * 1000 + env.generation_ms + env.feature_ms + env.lookahead_ms
    record = {"schema": "motion386-b2-group-smoke-v1", "instance": args.instance, "seed": args.seed,
              "planner_seed": planner_seed,
              "config": str(args.config.resolve()), "reset_ms": reset_ms, "selection_ms": selection_ms,
              "selection_budget_ms": 2000., "selection_budget_exceeded": selection_ms > 2000.,
              "strict_single_group": True, "executed": bool(args.execute), "success": False}
    if choice is None:
        record.update(status="no_valid_group", backend=backend.audit())
    else:
        record.update(status="selected", choice={"indices": [choice.contact_index, choice.carry_index, choice.place_index],
                                                  "total_s": choice.total_s, "score": choice.score,
                                                  "terms_s": choice.terms_s, "audit": choice.audit,
                                                  "rows": choice.rows, "redundancy": choice.redundancy})
        if args.execute:
            try:
                success, outcomes = execute_choice(env, choice)
                record.update(success=success, status="success" if success else "task_failure", outcomes=outcomes,
                              timed_validation=env.report.get("timed_validation"),
                              motion_duration_s=env.report.get("motion_duration_s"))
                if success and args.trajectory_output:
                    save_trajectory(args.trajectory_output, env)
                    record["trajectory"] = str(args.trajectory_output.resolve())
            except Exception as error:
                record.update(status="infrastructure", success=False, error_type=type(error).__name__, error=str(error))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(jsonable(record), ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({key: record.get(key) for key in ("schema", "instance", "status", "success",
                                                       "selection_ms", "selection_budget_exceeded")}, ensure_ascii=False))
    return 0 if record["status"] in ("selected", "success") else 2


if __name__ == "__main__":
    raise SystemExit(main())
