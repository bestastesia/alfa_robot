"""Strict single-action, resumable evaluation; no fitting or weight updates.

Real StageEnv construction is lazy and must be launched under the project's GPU
flock. Unit tests use CPU stubs. The test split requires --allow-test. Reports
retain infrastructure errors and scene failures in all-instance denominators.
"""
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import signal
import time

import numpy as np
import torch

from motion311_learning.contracts import config_digest, reward
from motion311_learning.dataset import normalized_actions, normalized_state
from motion311_learning.features import PHYSICS_DIM
from motion311_learning.networks import DQNAgent, SACAgent
from motion311_learning.supervised import SupervisedPolicy


ROUTES = ("r1", "r2", "r3", "r4", "heuristic", "uniform")
CONTINUOUS = ("r4", "uniform")
SCHEMA = "motion311-strict-evaluation-v1"


def _jsonable(value):
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, np.generic):
        return _jsonable(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _atomic_json(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(_jsonable(value), ensure_ascii=False, allow_nan=False, indent=2) + "\n")
    temporary.replace(path)


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def mode(item):
    task = item["task"]
    arm = "dual" if len(task) == 2 else "single-" + next(iter(task))
    return arm + ("-top" if item["top_grasp"] else "-side")


def instance_seed(item, seed):
    # Same planner seed across routes; independent of traversal/resume order.
    content = json.dumps({"instance": item["instance"], "seed": int(seed)}, sort_keys=True).encode()
    return int.from_bytes(hashlib.sha256(content).digest()[:4], "little") % (2**31 - 1)


def _numeric(value):
    if value is None:
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def compact_report(report):
    """Hash complete frame bytes while retaining metadata, not huge timed arrays."""
    report = report or {}
    frames = np.asarray(report.get("frames", []), dtype="<f8")
    frame_hash = hashlib.sha256(np.ascontiguousarray(frames).tobytes()).hexdigest()
    large = {"frames", "timed_segments", "velocities", "accelerations", "jerks", "time_from_start_s",
             "phases", "payload", "attempts"}
    metadata = {key: value for key, value in report.items() if key not in large}
    phase_counts = {}
    for phase in report.get("phases", []):
        phase_counts[phase] = phase_counts.get(phase, 0) + 1
    segments = [{key: value for key, value in segment.items()
                 if key in ("name", "frame_start", "frame_end", "duration_s", "parameterizer",
                            "maximum_linear_deviation", "raw_waypoints", "shortcut_waypoints")}
                for segment in report.get("timed_segments", [])]
    metadata.update(frames_sha256=frame_hash, frames_hash_encoding="contiguous-little-endian-float64",
                    frames_shape=list(frames.shape), frame_count=len(frames), phase_counts=phase_counts,
                    segment_summary=segments)
    # Hash the full report in canonical JSON, without storing its large arrays twice.
    encoded = json.dumps(_jsonable(report), ensure_ascii=False, allow_nan=False, sort_keys=True,
                         separators=(",", ":")).encode()
    metadata["full_report_sha256"] = hashlib.sha256(encoded).hexdigest()
    return _jsonable(metadata)


def choose_action(route, policy, observation, item, velocity, rng):
    """One current-stage action. No future candidates or fallback candidates."""
    started = time.perf_counter()
    if route not in ROUTES:
        raise ValueError("unknown route")
    if route in ("r1", "r2"):
        state = observation.state
        candidates = observation.candidate_features(physical=route == "r2")
    elif route == "r3":
        state = normalized_state(observation.state)
        candidates = normalized_actions(observation.candidates)
    elif route == "r4":
        state, candidates = normalized_state(observation.state), None
    else:
        state, candidates = observation.state, observation.candidates
    preprocess_ms = (time.perf_counter() - started) * 1000
    inference_started = time.perf_counter()
    if route in ("r1", "r2"):
        action = policy.select(state, candidates, observation.stage)
    elif route == "r3":
        action = policy.select(state, candidates, observation.stage, epsilon=0.0)
    elif route == "r4":
        action = np.asarray(policy.select(state, observation.stage, deterministic=True), dtype=float).copy()
    elif route == "uniform":
        action = rng.uniform(-1.0, 1.0, size=3)
    else:
        if not len(candidates):
            raise ValueError("empty current candidate pool")
        velocity = np.asarray(velocity, dtype=float)
        if velocity.shape != (15,) or not np.isfinite(velocity).all() or np.any(velocity <= 0):
            raise ValueError("heuristic requires 15 positive joint velocity limits")
        previous, home = state[7:22], state[22:37]
        costs = np.max(np.abs(candidates - previous) / velocity, axis=1)
        if observation.stage == 2:
            costs += np.max(np.abs(candidates - home) / velocity, axis=1)
        action = int(np.argmin(costs))
    if route in CONTINUOUS:
        action = np.asarray(action, dtype=float).copy()
        if action.shape != (3,) or not np.isfinite(action).all() or np.any(np.abs(action) > 1):
            raise ValueError("continuous policy returned an invalid normalized action")
        if len(item["task"]) == 1:
            action[2 if "left" in item["task"] else 1] = 0.0
    else:
        if int(action) != action or not 0 <= int(action) < len(observation.candidates):
            raise ValueError("policy selected outside current candidate pool")
        action = int(action)
    return action, {"preprocess_ms": preprocess_ms,
                    "inference_ms": (time.perf_counter() - inference_started) * 1000,
                    "inference_includes_internal_normalization": route in ("r1", "r2")}


def evaluate_instance(env, item, route, policy=None, seed=0):
    """Never retry an action, a failed stage, a reset, or an infrastructure error."""
    began = time.perf_counter()
    planner_seed = instance_seed(item, seed)
    rng = np.random.default_rng(planner_seed)
    record = dict(schema=SCHEMA, instance=item["instance"], split=item["split"], task=item["task"],
                  mode=mode(item), offset_m=item.get("offset_m"), route=route, seed=int(seed),
                  planner_seed=planner_seed, success=False, status="infrastructure", failure_stage=None,
                  terminated=False, truncated=False, outcomes=[], rewards=[], third_stage_reached=False,
                  third_stage_failed=False)
    current_stage, reset_ok = None, False
    last_info, selection = {}, None
    try:
        reset_started = time.perf_counter()
        observation = env.reset(item, seed=planner_seed, continuous=route in CONTINUOUS)
        reset_ok = True
        record["reset_ms"] = (time.perf_counter() - reset_started) * 1000
        if observation.stage != 0:
            raise ValueError("reset did not return stage 0")
        if route not in CONTINUOUS and not len(observation.candidates):
            record.update(status="empty_initial_candidates", reason="scene_no_contact_candidates",
                          failure_stage=0, terminated=True, rewards=[reward(0.0, 0.0, False)])
        else:
            for expected_stage in range(3):
                current_stage = int(observation.stage)
                if current_stage != expected_stage:
                    raise ValueError("environment skipped or repeated a decision stage")
                record["third_stage_reached"] |= current_stage == 2
                generation_ms = _numeric(getattr(env, "generation_ms", None))
                features_ms = _numeric(getattr(env, "feature_ms", None))
                selection = dict(stage=current_stage, candidate_count=len(observation.candidates),
                                 generation_ms=generation_ms, feature_ms=features_ms)
                action, timing = choose_action(route, policy, observation, item, env.velocity, rng)
                selection.update(timing, action_index=None if route in CONTINUOUS else action,
                                 action_params=action.tolist() if route in CONTINUOUS else observation.actions[action].tolist())
                step_started = time.perf_counter()
                result = env.step(action)
                last_info = dict(result.info)
                selection.update(reward=float(result.reward), terminated=bool(result.terminated),
                                 truncated=bool(result.truncated), info=_jsonable(last_info),
                                 decode_ms=_numeric(last_info.get("decode_ms")),
                                 step_wall_ms=(time.perf_counter() - step_started) * 1000)
                if last_info.get("feature_ms") is not None:
                    selection["feature_ms"] = _numeric(last_info["feature_ms"])
                record["outcomes"].append(selection)
                selection = None
                record["rewards"].append(float(result.reward))
                record["terminated"], record["truncated"] = bool(result.terminated), bool(result.truncated)
                if result.truncated:
                    record.update(status="infrastructure", failure_stage=current_stage,
                                  reason=last_info.get("reason", last_info.get("class_", "truncated")))
                    break
                if result.terminated and result.observation is not None:
                    raise ValueError("terminal transition unexpectedly exposes a next observation")
                if result.terminated:
                    accepted = (current_stage == 2 and bool(last_info.get("success"))
                                and len(record["outcomes"]) == 3
                                and bool(getattr(env, "report", {}).get("success"))
                                and bool(getattr(env, "report", {}).get("timed_validation", {}).get("success")))
                    if accepted:
                        record.update(success=True, status="success")
                    else:
                        reason = ("strict_terminal_validation" if last_info.get("success")
                                  else last_info.get("reason", last_info.get("class_", "task_failure")))
                        failed_stage = current_stage + 1 if last_info.get("class_") == "empty_next_candidates" else current_stage
                        record.update(status="task_failure", failure_stage=failed_stage, reason=reason)
                        if failed_stage == 2:
                            record["third_stage_reached"] = True
                    break
                if result.observation is None:
                    raise ValueError("continuing transition lacks a next observation")
                observation = result.observation
            else:
                raise ValueError("three decisions completed without termination")
    except Exception as error:
        # EpisodeFailure is recognized without importing the CUDA environment.
        scene_failure = not reset_ok and hasattr(error, "reason")
        record.update(status=("scene_infeasible" if scene_failure else "infrastructure"), success=False,
                      terminated=scene_failure, truncated=not scene_failure,
                      failure_stage=current_stage if current_stage is not None else (0 if scene_failure else None),
                      reason=str(getattr(error, "reason", error)), error_type=type(error).__name__)
        if scene_failure:
            record["rewards"] = [reward(0.0, 0.0, False)]
        if selection is not None:
            selection.update(error_type=type(error).__name__, error=str(error), truncated=True)
            record["outcomes"].append(selection)
    report = getattr(env, "report", {}) if reset_ok else {}
    record["third_stage_failed"] = (record["third_stage_reached"] and record["status"] == "task_failure"
                                     and record["failure_stage"] == 2)
    record["episode_return"] = float(sum(record["rewards"]))
    record["raw_t_path_s"] = _numeric(last_info.get("cumulative_cost_s", getattr(env, "cost_s", None))) if reset_ok else None
    record["executed_t_path_s"] = _numeric(last_info.get("executed_t_path_s"))
    record["motion_duration_s"] = _numeric(last_info.get("motion_duration_s", report.get("motion_duration_s")))
    record["reward_cost_kind"] = last_info.get("reward_cost_kind", "raw_planned_path_t_lb")
    record["report"] = compact_report(report)
    record["collision_acceptance"] = {"timed_validation": _jsonable(report.get("timed_validation")),
                                      "strict_three_decisions": len(record["outcomes"]) == 3,
                                      "no_action_retry": True,
                                      "scope": "existing StageEnv gates; not a new continuous-geometry certificate"}
    record["wall_s"] = time.perf_counter() - began
    return _jsonable(record)


def wilson(successes, total, z=1.959963984540054):
    if total < 0 or successes < 0 or successes > total:
        raise ValueError("invalid binomial counts")
    if not total:
        return {"successes": 0, "total": 0, "rate": None, "low": None, "high": None}
    p = successes / total
    denominator = 1 + z * z / total
    center = (p + z * z / (2 * total)) / denominator
    half = z * math.sqrt(p * (1 - p) / total + z * z / (4 * total * total)) / denominator
    return {"successes": successes, "total": total, "rate": p,
            "low": max(0.0, center - half), "high": min(1.0, center + half)}


def _summary(rows):
    counts = {}
    for row in rows:
        counts[row["status"]] = counts.get(row["status"], 0) + 1
    success_rows = [row for row in rows if row["success"]]
    reached = sum(bool(row["third_stage_reached"]) for row in rows)
    failed = sum(bool(row["third_stage_failed"]) for row in rows)
    metrics = {}
    for name in ("raw_t_path_s", "executed_t_path_s", "motion_duration_s", "wall_s"):
        values = [row[name] for row in success_rows if row.get(name) is not None]
        metrics[name] = {"n": len(values), "mean": float(np.mean(values)) if values else None,
                         "median": float(np.median(values)) if values else None}
    timing = {}
    for name in ("generation_ms", "feature_ms", "preprocess_ms", "inference_ms", "decode_ms"):
        values = [outcome[name] for row in rows for outcome in row["outcomes"] if outcome.get(name) is not None]
        timing[name] = {"n_measured": len(values), "mean_ms": float(np.mean(values)) if values else None,
                        "p95_ms": float(np.percentile(values, 95)) if values else None}
    return {"n": len(rows), "success": wilson(len(success_rows), len(rows)), "statuses": counts,
            "third_stage_reached": reached, "third_stage_failed": failed,
            "third_stage_failure_given_reached": wilson(failed, reached),
            "third_stage_failure_per_episode": wilson(failed, len(rows)),
            "third_stage_infrastructure": sum(row["third_stage_reached"] and row["status"] == "infrastructure" for row in rows),
            "success_only_costs": metrics, "selection_timing": timing}


def build_report(records):
    """Paired comparisons use common (instance, seed, split); costs require both success."""
    groups, seen, fingerprints, split_registries = {}, set(), {}, {}
    for row in records:
        provenance = row.get("provenance", {})
        registry_hash = provenance.get("registry_sha256")
        if row["split"] in split_registries and split_registries[row["split"]] != registry_hash:
            raise ValueError("paired routes must use the same registry provenance")
        split_registries[row["split"]] = registry_hash
        run_key = (row["split"], row["route"])
        fingerprint = (provenance.get("registry_sha256"), provenance.get("checkpoint_sha256"),
                       provenance.get("source_sha256"))
        if run_key in fingerprints and fingerprints[run_key] != fingerprint:
            raise ValueError("cannot pool one route across differing registry/model/source provenance")
        fingerprints[run_key] = fingerprint
        identity = (row["route"], row["instance"], row["seed"], row["split"])
        if identity in seen:
            raise ValueError("duplicate evaluation identity")
        seen.add(identity)
        groups.setdefault((row["split"], row["mode"], row["route"]), []).append(row)
        groups.setdefault((row["split"], "all", row["route"]), []).append(row)
    summaries, paired = {}, {}
    for (split, task_mode, route), rows in sorted(groups.items()):
        summaries.setdefault(split, {}).setdefault(task_mode, {})[route] = _summary(rows)
    for split, task_mode in sorted({(key[0], key[1]) for key in groups}):
        routes = sorted(key[2] for key in groups if key[:2] == (split, task_mode))
        for a, b in itertools.combinations(routes, 2):
            left = {(row["instance"], row["seed"]): row for row in groups[(split, task_mode, a)]}
            right = {(row["instance"], row["seed"]): row for row in groups[(split, task_mode, b)]}
            keys = sorted(left.keys() & right.keys())
            success_delta = [int(right[key]["success"]) - int(left[key]["success"]) for key in keys]
            costs = {}
            for name in ("raw_t_path_s", "executed_t_path_s", "motion_duration_s"):
                differences = [right[key][name] - left[key][name] for key in keys
                               if left[key]["success"] and right[key]["success"]
                               and left[key].get(name) is not None and right[key].get(name) is not None]
                costs[name] = {"n_both_success": len(differences),
                               "mean_b_minus_a": float(np.mean(differences)) if differences else None,
                               "median_b_minus_a": float(np.median(differences)) if differences else None}
            comparison = {"a": a, "b": b, "n_paired": len(keys), "n_a_only": len(left.keys() - right.keys()),
                          "n_b_only": len(right.keys() - left.keys()),
                          "success_delta_b_minus_a": float(np.mean(success_delta)) if keys else None,
                          "a_only_success": sum(left[key]["success"] and not right[key]["success"] for key in keys),
                          "b_only_success": sum(right[key]["success"] and not left[key]["success"] for key in keys),
                          "both_success": sum(left[key]["success"] and right[key]["success"] for key in keys),
                          "paired_success_costs": costs}
            paired.setdefault(split, {}).setdefault(task_mode, {})[a + "__" + b] = comparison
    return {"schema": SCHEMA, "summaries": summaries, "paired": paired,
            "denominator_policy": "all persisted instances including infrastructure, empty pools and scene failures",
            "third_stage_reached_definition": "stage-2 observation/attempt or failure to generate its current candidate pool",
            "confidence": "95% Wilson marginal success intervals; paired differences are descriptive",
            "strict_single_action": True, "training_updates": 0}


def _load_policy(route, models):
    if route in ("heuristic", "uniform"):
        return None, None
    if models is None:
        raise ValueError("learned routes require --models")
    path = Path(models)
    if path.is_dir():
        path = path / ("checkpoint.pt" if route in ("r1", "r2") else "agent.pt")
    if route in ("r1", "r2"):
        policy = SupervisedPolicy.load(path)
        if policy.physical != (route == "r2"):
            raise ValueError("supervised checkpoint physical mode does not match route")
    else:
        policy = (DQNAgent if route == "r3" else SACAgent).load(path)
    return policy, {"path": str(path.resolve()), "sha256": sha256_file(path)}


def _source_fingerprint():
    # Changes to policy/environment semantics must not silently resume old runs.
    root = Path(__file__).resolve().parent
    modules = ("evaluate.py", "stage_env.py", "redundancy.py", "features.py", "contracts.py",
               "dataset.py", "networks.py", "supervised.py")
    hashes = {name: sha256_file(root / name) for name in modules if (root / name).is_file()}
    return config_digest(hashes)


def _initialization_failure(item, route, seed, error):
    return dict(schema=SCHEMA, instance=item["instance"], split=item["split"], task=item["task"],
                mode=mode(item), offset_m=item.get("offset_m"), route=route, seed=int(seed),
                planner_seed=instance_seed(item, seed), success=False, status="infrastructure",
                failure_stage=None, terminated=False, truncated=True, outcomes=[], rewards=[],
                episode_return=0.0, third_stage_reached=False, third_stage_failed=False,
                reason="environment_initialization:" + str(error), error_type=type(error).__name__,
                raw_t_path_s=None, executed_t_path_s=None, motion_duration_s=None,
                report=compact_report({}), wall_s=0.0, collision_acceptance={"timed_validation": None})


def run_evaluation(registry, output, route, models=None, group="dev", seed=0, max_minutes=24.0,
                   allow_test=False, env_factory=None, policy=None, stop_requested=None):
    if route not in ROUTES or group not in ("train", "dev", "test"):
        raise ValueError("unknown route/group")
    if group == "test" and not allow_test:
        raise ValueError("test split requires explicit --allow-test; this run is not authorized")
    if not math.isfinite(max_minutes) or max_minutes <= 0:
        raise ValueError("max_minutes must be positive")
    registry, output = Path(registry), Path(output)
    document = json.loads(registry.read_text())
    items = [item for item in document["instances"] if item["split"] == group and item.get("group", item["split"]) == group]
    if not items:
        raise ValueError("registry has no selected instances")
    if len({item["instance"] for item in items}) != len(items):
        raise ValueError("registry has duplicate instances")
    if policy is None:
        policy, checkpoint = _load_policy(route, models)
    else:
        checkpoint = {"injected_policy": type(policy).__name__}
    configuration = {"schema": SCHEMA, "route": route, "group": group, "seed": int(seed),
                     "registry_sha256": sha256_file(registry), "checkpoint": checkpoint,
                     "source_sha256": _source_fingerprint(),
                     "physical_dim": PHYSICS_DIM, "timed": True, "no_training": True,
                     "strict_single_action": True, "allow_test": bool(allow_test)}
    run_dir = output / route / (group + "-s" + str(seed))
    config_file = run_dir / "config.json"
    if config_file.exists() and json.loads(config_file.read_text()) != configuration:
        raise ValueError("resume configuration/checkpoint differs; use a new output directory")
    _atomic_json(config_file, configuration)
    config_hash = config_digest(configuration)
    if env_factory is None:
        from motion311_learning.stage_env import StageEnv
        env_factory = StageEnv
    started, env, completed = time.perf_counter(), None, True
    initialization_error = None
    try:
        for item in items:
            identity = hashlib.sha256(item["instance"].encode()).hexdigest()[:24]
            target = run_dir / "instances" / (identity + ".json")
            if target.exists():
                saved = json.loads(target.read_text())
                if saved.get("config_sha256") != config_hash or saved.get("instance") != item["instance"]:
                    raise ValueError("existing instance provenance does not match this run")
                continue
            if time.perf_counter() - started >= max_minutes * 60 or (stop_requested and stop_requested()):
                completed = False
                break
            if env is None and initialization_error is None:
                try:
                    env = env_factory(timed=True, physical=route == "r2")
                except Exception as error:
                    initialization_error = error
            row = (_initialization_failure(item, route, seed, initialization_error)
                   if initialization_error is not None else evaluate_instance(env, item, route, policy, seed))
            row["config_sha256"] = config_hash
            row["provenance"] = dict(registry_sha256=configuration["registry_sha256"],
                                     source_sha256=configuration["source_sha256"],
                                     checkpoint_sha256=None if checkpoint is None else checkpoint.get("sha256"))
            _atomic_json(target, row)
            print(json.dumps({"route": route, "instance": item["instance"], "success": row["success"],
                              "status": row["status"], "wall_s": row["wall_s"]}), flush=True)
            # Do not reuse a potentially corrupted runtime after infrastructure faults.
            if row["status"] == "infrastructure":
                env = None
    finally:
        records = []
        for path in sorted(output.glob("*/*/instances/*.json")):
            row = json.loads(path.read_text())
            if row.get("schema") == SCHEMA:
                records.append(row)
        _atomic_json(output / "report.json", build_report(records))
    return completed


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, required=True)
    parser.add_argument("--models", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--route", choices=ROUTES, required=True)
    parser.add_argument("--group", choices=("train", "dev", "test"), default="dev")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-minutes", type=float, default=24.0)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--allow-test", action="store_true")
    parser.add_argument("--report-only", action="store_true")
    args = parser.parse_args(argv)
    if args.threads < 1:
        parser.error("--threads must be positive")
    if args.group == "test" and not args.allow_test:
        parser.error("test split requires explicit --allow-test")
    torch.set_num_threads(args.threads)
    if args.report_only:
        records = [json.loads(path.read_text()) for path in sorted(args.output.glob("*/*/instances/*.json"))]
        _atomic_json(args.output / "report.json", build_report(records))
        return 0
    stop = [False]
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    completed = run_evaluation(args.registry, args.output, args.route, args.models, args.group, args.seed,
                               args.max_minutes, args.allow_test, stop_requested=lambda: stop[0])
    return 0 if completed else 3


if __name__ == "__main__":
    raise SystemExit(main())
