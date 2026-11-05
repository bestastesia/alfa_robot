"""Batch-evaluate one frozen B2 group selector with strict single-group execution."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import signal
import time

import numpy as np
import torch

from motion306_learning.env import LookaheadEnv
from motion306_learning.evaluate import select_items
from motion306_learning.group_backend import LookaheadGroupBackend
from motion306_learning.group_score import GroupScoreConfig
from motion306_learning.group_selector import GroupBeamConfig, select_group
from motion311_learning.evaluate import (_atomic_json, compact_report, instance_seed, mode, sha256_file)
from v3_wall_ik_benchmark import ACTIVE_JOINTS


SCHEMA = "motion386-b2-group-evaluation-v1"


def finite(value):
    if value is None:
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def execute_choice(env, choice):
    outcomes = []
    for stage, (index, expected) in enumerate(zip(
            (choice.contact_index, choice.carry_index, choice.place_index), choice.rows[[1, 3, 4]])):
        observation = env.observation
        if observation.stage != stage or index < 0 or index >= len(observation.candidates):
            raise RuntimeError("selected index does not exist in executed stage pool")
        # Analytic branches are rebuilt through float32 GPU state; the same stable
        # index can differ by about 1.4e-6 rad while remaining the same candidate.
        if not np.allclose(observation.candidates[index], expected, atol=2e-6, rtol=0.):
            difference = float(np.max(np.abs(observation.candidates[index] - expected)))
            raise RuntimeError(f"selected row differs from executed stage pool: stage={stage}, max_abs={difference}")
        began = time.perf_counter()
        result = env.step(index)
        outcomes.append({"stage": stage, "index": index, "wall_ms": (time.perf_counter() - began) * 1000.,
                         "terminated": bool(result.terminated), "truncated": bool(result.truncated),
                         "reward": float(result.reward), "info": result.info})
        if result.terminated or result.truncated:
            break
    success = (len(outcomes) == 3 and outcomes[-1]["info"].get("success") and env.report.get("success") and
               env.report.get("timed_validation", {}).get("success"))
    return bool(success), outcomes


def trajectory_metrics(env):
    report = env.report
    if not report.get("frames"):
        return {}
    indices = [report["joint_names"].index(name) for name in ACTIVE_JOINTS]
    positions = np.asarray(report["frames"], dtype=np.float64)[:, indices]
    if len(positions) < 2:
        return {"normalized_joint_variation": 0., "joint_direction_reversals": 0}
    span = np.asarray(env.decoder.upper) - np.asarray(env.decoder.lower)
    delta = np.diff(positions, axis=0) / span
    reversals = 0
    for joint in range(delta.shape[1]):
        values = delta[:, joint]
        signs = np.sign(values[np.abs(values) > 1e-5])
        reversals += int(np.sum(signs[1:] != signs[:-1])) if len(signs) > 1 else 0
    return {"normalized_joint_variation": float(np.abs(delta).sum()),
            "joint_direction_reversals": reversals}


def evaluate(env, item, score_config, beam_config, seed, budget_ms):
    began = time.perf_counter()
    planner_seed = instance_seed(item, seed)
    record = {"schema": SCHEMA, "instance": item["instance"], "base_instance": item.get("base_instance"),
              "split": item["split"], "task": item["task"], "mode": mode(item),
              "box_level": item.get("box_level"), "box_dims": item.get("box_dims"),
              "size_split": item.get("size_split"), "seed": seed, "planner_seed": planner_seed,
              "strict_single_group": True, "success": False, "status": "infrastructure"}
    try:
        reset_began = time.perf_counter()
        env.reset(item, seed=planner_seed)
        record["reset_ms"] = (time.perf_counter() - reset_began) * 1000.
        backend = LookaheadGroupBackend(env, score_config.edge_collision_samples)
        selection_began = time.perf_counter()
        choice = select_group(backend, score_config, beam_config)
        record["selection_ms"] = ((time.perf_counter() - selection_began) * 1000. + env.generation_ms +
                                  env.feature_ms + env.lookahead_ms)
        record["selection_budget_ms"] = budget_ms
        record["selection_budget_exceeded"] = record["selection_ms"] > budget_ms
        if choice is None:
            record.update(status="task_failure", failure_stage=0, reason="no_valid_group", backend=backend.audit())
        else:
            record["choice"] = {"indices": [choice.contact_index, choice.carry_index, choice.place_index],
                                "total_s": choice.total_s, "score": choice.score,
                                "terms_s": choice.terms_s, "audit": choice.audit,
                                "redundancy": choice.redundancy.tolist()}
            success, outcomes = execute_choice(env, choice)
            record["outcomes"] = outcomes
            record["planning_wall_ms"] = float(sum(row["wall_ms"] for row in outcomes))
            record["success"] = success
            if success:
                last = outcomes[-1]["info"]
                record.update(status="success", raw_t_path_s=finite(last.get("cumulative_cost_s")),
                              executed_t_path_s=finite(last.get("executed_t_path_s")),
                              motion_duration_s=finite(last.get("motion_duration_s")),
                              timed_validation=env.report.get("timed_validation"),
                              trajectory_metrics=trajectory_metrics(env), report=compact_report(env.report))
            else:
                last = outcomes[-1] if outcomes else {"stage": 0, "info": {}}
                info = last["info"]
                record.update(status="infrastructure" if last.get("truncated") else "task_failure",
                              failure_stage=last["stage"], reason=info.get("reason", info.get("class_")),
                              error=info.get("error"), report=compact_report(env.report))
    except Exception as error:
        record.update(status="infrastructure", success=False, reason=str(error), error_type=type(error).__name__)
    record["wall_s"] = time.perf_counter() - began
    record["total_selection_planning_ms"] = ((record.get("selection_ms") or 0.) +
                                              (record.get("planning_wall_ms") or 0.))
    return record


def percentile(rows, name, q, successful=False):
    values = [row.get(name) for row in rows if (not successful or row.get("success")) and row.get(name) is not None]
    return float(np.percentile(values, q)) if values else None


def build_report(rows, config, complete):
    failures = {}
    for row in rows:
        if not row.get("success"):
            key = f"stage{row.get('failure_stage')}:{row.get('reason')}"
            failures[key] = failures.get(key, 0) + 1
    metrics = ("selection_ms", "planning_wall_ms", "total_selection_planning_ms", "raw_t_path_s",
               "executed_t_path_s", "motion_duration_s")
    summary = {name: {"p50": percentile(rows, name, 50, name not in ("selection_ms",)),
                      "p95": percentile(rows, name, 95, name not in ("selection_ms",))} for name in metrics}
    timed = [row["timed_validation"] for row in rows if row.get("success") and row.get("timed_validation")]
    trajectory = [row["trajectory_metrics"] for row in rows if row.get("success") and row.get("trajectory_metrics")]
    return {"schema": SCHEMA, "configuration": config, "complete": complete, "instances": len(rows),
            "strict_success": sum(bool(row.get("success")) for row in rows),
            "selection_budget_exceeded": sum(bool(row.get("selection_budget_exceeded")) for row in rows),
            "status_counts": {status: sum(row.get("status") == status for row in rows)
                              for status in sorted({row.get("status") for row in rows})},
            "failures": failures, "metrics": summary,
            "timed_validation": {name: {"p50": float(np.median([row[name] for row in timed])) if timed else None,
                                         "p95": float(np.percentile([row[name] for row in timed], 95)) if timed else None}
                                 for name in ("velocity_limit_ratio", "acceleration_limit_ratio", "jerk_limit_ratio")},
            "trajectory_metrics": {name: {"p50": float(np.median([row[name] for row in trajectory])) if trajectory else None,
                                           "p95": float(np.percentile([row[name] for row in trajectory], 95)) if trajectory else None}
                                   for name in ("normalized_joint_variation", "joint_direction_reversals")}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--group", choices=("dev", "train"), default="dev")
    parser.add_argument("--levels", default="S30x40x40")
    parser.add_argument("--seed", type=int, default=101)
    parser.add_argument("--max-instances", type=int)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--max-minutes", type=float, default=60.)
    args = parser.parse_args()
    document = json.loads(args.registry.read_text())
    config_document = json.loads(args.config.read_text())
    budget_ms = float(config_document["selection_contract"]["budget_ms"])
    score_config, beam_config = GroupScoreConfig.load(args.config), GroupBeamConfig.load(args.config)
    items = select_items(document, args.group, set(filter(None, args.levels.split(","))),
                         args.max_instances, args.stride)
    configuration = {"schema": SCHEMA, "registry": str(args.registry.resolve()),
                     "registry_sha256": sha256_file(args.registry), "config": str(args.config.resolve()),
                     "config_sha256": sha256_file(args.config), "group": args.group,
                     "levels": sorted(set(filter(None, args.levels.split(",")))), "seed": args.seed,
                     "max_instances": args.max_instances, "stride": args.stride,
                     "strict_single_group": True, "test_split_opened": False}
    args.output.mkdir(parents=True, exist_ok=True)
    config_path = args.output / "config.json"
    if config_path.exists() and json.loads(config_path.read_text()) != configuration:
        raise ValueError("resume configuration differs; use a new output directory")
    _atomic_json(config_path, configuration)
    stopped = [False]
    signal.signal(signal.SIGTERM, lambda *_: stopped.__setitem__(0, True))
    signal.signal(signal.SIGINT, lambda *_: stopped.__setitem__(0, True))
    env, started, complete = None, time.perf_counter(), True
    try:
        for item in items:
            target = args.output / "instances" / (hashlib.sha256(item["instance"].encode()).hexdigest()[:24] + ".json")
            if target.exists():
                continue
            if stopped[0] or time.perf_counter() - started >= args.max_minutes * 60.:
                complete = False
                break
            if env is None:
                env = LookaheadEnv(timed=True, physical=False, contact_pool="capped", place_gate=True)
            row = evaluate(env, item, score_config, beam_config, args.seed, budget_ms)
            row["config_sha256"] = configuration["config_sha256"]
            _atomic_json(target, row)
            print(json.dumps({"instance": row["instance"], "success": row["success"], "status": row["status"],
                              "reason": row.get("reason"), "selection_ms": round(row.get("selection_ms") or 0., 1),
                              "wall_s": round(row["wall_s"], 1)}, ensure_ascii=False), flush=True)
            if row["status"] == "infrastructure":
                env = None
    finally:
        rows = [json.loads(path.read_text()) for path in sorted((args.output / "instances").glob("*.json"))]
        _atomic_json(args.output / "report.json", build_report(rows, configuration, complete and len(rows) == len(items)))
    return 0 if complete else 3


if __name__ == "__main__":
    raise SystemExit(main())
