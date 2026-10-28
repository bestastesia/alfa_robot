"""Strict single-shot evaluation with selector-side screening, reported per box size; no fitting or updates.

Each stage ranks the current candidates, screens them with the environment's deterministic gates and executes
exactly one action; any failure ends the episode. When no candidate passes, the top-ranked one executes and its
own gate reports the failure. Denominators keep every instance. Seen and held-out box sizes are reported
separately; the test split needs --allow-test and stays sealed for MOTION-364. Launch under the GPU flock.
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
from motion311_learning.evaluate import _atomic_json, _jsonable, _numeric, _summary, compact_report, instance_seed, mode, sha256_file
from motion311_learning.networks import DQNAgent
from motion306_learning.rankers import dqn_order, heuristic_order, lookahead_heuristic_order, uniform_order
from motion306_learning.screening import screened_choice

ROUTES = ("heuristic", "lookahead_heuristic", "uniform", "r3_311", "r3_plus")
SCHEMA = "motion364-screened-evaluation-v1"


def rank(route, policy, observation, velocity, rng):
    if route == "heuristic":
        return heuristic_order(observation, velocity)
    if route == "uniform":
        return uniform_order(observation, rng)
    if route == "lookahead_heuristic":
        return lookahead_heuristic_order(observation)
    if route == "r3_311":
        return dqn_order(policy, observation)
    if route == "r3_plus":
        return policy.order(observation)
    raise ValueError("unknown route")


def route_label(route, screen):
    return route + ("+screen" if screen else "")


def evaluate_instance(env, item, route, policy=None, seed=0, screen=True):
    """Never retry an action, a failed stage, a reset, or an infrastructure error."""
    began = time.perf_counter()
    planner_seed = instance_seed(item, seed)
    rng = np.random.default_rng(planner_seed)
    record = dict(schema=SCHEMA, instance=item["instance"], base_instance=item.get("base_instance"), split=item["split"],
                  task=item["task"], mode=mode(item), offset_m=item.get("offset_m"), box_level=item.get("box_level"),
                  box_dims=item.get("box_dims"), size_split=item.get("size_split"), route=route, screen=bool(screen),
                  seed=int(seed), planner_seed=planner_seed, success=False, status="infrastructure", failure_stage=None,
                  terminated=False, truncated=False, outcomes=[], rewards=[], third_stage_reached=False,
                  third_stage_failed=False)
    current_stage, reset_ok = None, False
    last_info, selection = {}, None
    try:
        reset_started = time.perf_counter()
        observation = env.reset(item, seed=planner_seed)
        reset_ok = True
        record["reset_ms"] = (time.perf_counter() - reset_started) * 1000
        if observation.stage != 0:
            raise ValueError("reset did not return stage 0")
        if not len(observation.candidates):
            record.update(status="empty_initial_candidates", reason="scene_no_contact_candidates",
                          failure_stage=0, terminated=True, rewards=[reward(0.0, 0.0, False)])
        else:
            for expected_stage in range(3):
                current_stage = int(observation.stage)
                if current_stage != expected_stage:
                    raise ValueError("environment skipped or repeated a decision stage")
                record["third_stage_reached"] |= current_stage == 2
                selection = dict(stage=current_stage, candidate_count=len(observation.candidates),
                                 generation_ms=_numeric(getattr(env, "generation_ms", None)),
                                 feature_ms=_numeric(getattr(env, "feature_ms", None)),
                                 lookahead_ms=_numeric(getattr(env, "lookahead_ms", None)))
                ranked = time.perf_counter()
                order = np.asarray(rank(route, policy, observation, env.velocity, rng), dtype=int)
                selection["rank_ms"] = (time.perf_counter() - ranked) * 1000
                if sorted(order.tolist()) != list(range(len(observation.candidates))):
                    raise ValueError("ranking is not a permutation of the current candidate pool")
                index = int(order[0])
                if screen:
                    chosen, selection["screening"] = screened_choice(env, order)
                    selection["screening_exhausted"] = chosen is None
                    index = index if chosen is None else chosen
                selection.update(action_index=index, action_rank=int(np.flatnonzero(order == index)[0]),
                                 action_params=observation.actions[index].tolist())
                step_started = time.perf_counter()
                result = env.step(index)
                last_info = dict(result.info)
                selection.update(reward=float(result.reward), terminated=bool(result.terminated),
                                 truncated=bool(result.truncated), info=_jsonable(last_info),
                                 step_wall_ms=(time.perf_counter() - step_started) * 1000)
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
    record["selection_overhead_ms"] = float(sum((outcome.get("generation_ms") or 0.) + (outcome.get("lookahead_ms") or 0.)
                                                + (outcome.get("rank_ms") or 0.) + (outcome.get("screening") or {}).get("screen_ms", 0.)
                                                for outcome in record["outcomes"]))
    record["report"] = compact_report(report)
    record["wall_s"] = time.perf_counter() - began
    return _jsonable(record)


def _screening_summary(rows):
    stages = {}
    for stage in range(3):
        records = [outcome for row in rows for outcome in row["outcomes"]
                   if outcome.get("stage") == stage and outcome.get("screening") is not None]
        if not records:
            continue
        rejected = {}
        for outcome in records:
            for reason, count in outcome["screening"]["rejected"].items():
                rejected[reason] = rejected.get(reason, 0) + count
        times = [outcome["screening"]["screen_ms"] for outcome in records]
        stages[str(stage)] = dict(n=len(records), checks_mean=float(np.mean([o["screening"]["checks"] for o in records])),
                                  screen_ms_median=float(np.median(times)), screen_ms_p95=float(np.percentile(times, 95)),
                                  exhausted=sum(bool(o.get("screening_exhausted")) for o in records),
                                  chosen_rank_mean=float(np.mean([o["action_rank"] for o in records])), rejected=rejected)
    overhead = [row["selection_overhead_ms"] for row in rows if row["outcomes"]]
    return dict(stages=stages, selection_overhead_ms=dict(
        n=len(overhead), median=float(np.median(overhead)) if overhead else None,
        p95=float(np.percentile(overhead, 95)) if overhead else None))


def _keys(row):
    level = row.get("box_level") or "legacy"
    return [("all",), ("size_split", row.get("size_split") or "legacy"), ("level", level), ("mode", row["mode"]),
            ("level_mode", level, row["mode"])]


def build_report(records):
    """Per route (with/without screening): overall, seen/held-out sizes, per level, per mode. Paired by (instance, seed)."""
    groups, seen = {}, set()
    for row in records:
        label = route_label(row["route"], row["screen"])
        identity = (label, row["instance"], row["seed"], row["split"])
        if identity in seen:
            raise ValueError("duplicate evaluation identity")
        seen.add(identity)
        for key in _keys(row):
            groups.setdefault((row["split"], key, label), []).append(row)
    summaries, paired = {}, {}
    for (split, key, label), rows in sorted(groups.items()):
        summary = _summary(rows)
        summary["screening"] = _screening_summary(rows)
        summaries.setdefault(split, {}).setdefault("/".join(key), {})[label] = summary
    for split, key in sorted({(k[0], k[1]) for k in groups if k[1][0] in ("all", "size_split", "level")}):
        labels = sorted(k[2] for k in groups if k[:2] == (split, key))
        for a, b in itertools.combinations(labels, 2):
            left = {(row["instance"], row["seed"]): row for row in groups[(split, key, a)]}
            right = {(row["instance"], row["seed"]): row for row in groups[(split, key, b)]}
            common = sorted(left.keys() & right.keys())
            differences = {}
            for name in ("raw_t_path_s", "executed_t_path_s", "motion_duration_s"):
                values = [right[k][name] - left[k][name] for k in common if left[k]["success"] and right[k]["success"]
                          and left[k].get(name) is not None and right[k].get(name) is not None]
                differences[name] = dict(n_both_success=len(values), median_b_minus_a=float(np.median(values)) if values else None,
                                         mean_b_minus_a=float(np.mean(values)) if values else None)
            paired.setdefault(split, {}).setdefault("/".join(key), {})[a + "__" + b] = dict(
                n_paired=len(common), a_only_success=sum(left[k]["success"] and not right[k]["success"] for k in common),
                b_only_success=sum(right[k]["success"] and not left[k]["success"] for k in common),
                both_success=sum(left[k]["success"] and right[k]["success"] for k in common),
                paired_success_costs=differences)
    return {"schema": SCHEMA, "summaries": summaries, "paired": paired,
            "denominator_policy": "all persisted instances including infrastructure, empty pools and scene failures",
            "screening": "selector-side deterministic gates of the environment; one executed action per stage",
            "strict_single_action": True, "training_updates": 0}


def _load_policy(route, models):
    if route in ("heuristic", "lookahead_heuristic", "uniform"):
        return None, None
    path = Path(models)
    path = path / "agent.pt" if path.is_dir() else path
    if route == "r3_plus":
        from motion306_learning.agent import LookaheadAgent
        return LookaheadAgent.load(path), {"path": str(path.resolve()), "sha256": sha256_file(path)}
    return DQNAgent.load(path), {"path": str(path.resolve()), "sha256": sha256_file(path)}


def _source_fingerprint():
    tools = Path(__file__).resolve().parents[1]
    files = [tools / "motion311_learning" / name for name in ("stage_env.py", "redundancy.py", "features.py", "contracts.py",
                                                              "dataset.py", "networks.py", "evaluate.py")]
    files += sorted((tools / "motion306_learning").glob("*.py")) + [tools / "v3_box_geometry.py"]
    return config_digest({str(path.relative_to(tools)): sha256_file(path) for path in files})


def select_items(document, group, levels, limit=None, stride=1):
    items = [item for item in document["instances"] if item["split"] == group and item.get("group", item["split"]) == group
             and (not levels or item.get("box_level") in levels)]
    if len({item["instance"] for item in items}) != len(items):
        raise ValueError("registry has duplicate instances")
    items = sorted(items, key=lambda item: (str(item.get("box_level")), item["instance"]))
    per_level = {}
    for item in items:
        per_level.setdefault(item.get("box_level"), []).append(item)
    # every `stride`-th instance of each level spreads a subset over all modes; `limit` then caps it
    return [item for rows in per_level.values() for item in rows[::stride][:limit]]


def run_evaluation(registry, output, route, models=None, group="dev", levels=(), seed=0, screen=True, max_minutes=60.,
                   allow_test=False, env_factory=None, policy=None, stop_requested=None, lookahead_env=True, limit=None,
                   stride=1, contact_pool="capped", place_gate=False):
    if route not in ROUTES or group not in ("train", "dev", "test"):
        raise ValueError("unknown route/group")
    if group == "test" and not allow_test:
        raise ValueError("test split requires explicit --allow-test; MOTION-364 keeps it sealed")
    registry, output = Path(registry), Path(output)
    items = select_items(json.loads(registry.read_text()), group, set(levels), limit, stride)
    if not items:
        raise ValueError("registry has no selected instances")
    if policy is None:
        policy, checkpoint = _load_policy(route, models)
    else:
        checkpoint = {"injected_policy": type(policy).__name__}
    lookahead_env = lookahead_env or route in ("r3_plus", "lookahead_heuristic") or contact_pool != "capped" or place_gate
    configuration = {"schema": SCHEMA, "route": route, "screen": bool(screen), "group": group, "levels": sorted(levels),
                     "environment": "LookaheadEnv" if lookahead_env else "StageEnv", "limit_per_level": limit, "stride": stride, "contact_pool": contact_pool, "place_gate": bool(place_gate),
                     "seed": int(seed), "registry_sha256": sha256_file(registry), "checkpoint": checkpoint,
                     "source_sha256": _source_fingerprint(), "timed": True, "no_training": True,
                     "strict_single_action": True, "allow_test": bool(allow_test)}
    run_dir = output / route_label(route, screen) / f"{group}-s{seed}"
    config_file = run_dir / "config.json"
    if config_file.exists() and json.loads(config_file.read_text()) != configuration:
        raise ValueError("resume configuration/checkpoint differs; use a new output directory")
    _atomic_json(config_file, configuration)
    config_hash = config_digest(configuration)
    if env_factory is None and lookahead_env:
        from motion306_learning.env import LookaheadEnv as env_factory
    elif env_factory is None:
        from motion311_learning.stage_env import StageEnv as env_factory
    started, env, completed, level = time.perf_counter(), None, True, None
    try:
        for item in items:
            target = run_dir / "instances" / (hashlib.sha256(item["instance"].encode()).hexdigest()[:24] + ".json")
            if target.exists():
                saved = json.loads(target.read_text())
                if saved.get("config_sha256") != config_hash or saved.get("instance") != item["instance"]:
                    raise ValueError("existing instance provenance does not match this run")
                continue
            if time.perf_counter() - started >= max_minutes * 60 or (stop_requested and stop_requested()):
                completed = False
                break
            if env is None:
                env = env_factory(timed=True, physical=False,
                                  **(dict(contact_pool=contact_pool, place_gate=place_gate) if lookahead_env else {}))
            if level is not None and item.get("box_level") != level and hasattr(env, "planner"):
                env.planner.persistent_checkers.clear()  # one size at a time on the 8 GB card
                torch.cuda.empty_cache()
            level = item.get("box_level")
            row = evaluate_instance(env, item, route, policy, seed, screen)
            row["config_sha256"] = config_hash
            row["provenance"] = dict(registry_sha256=configuration["registry_sha256"], source_sha256=configuration["source_sha256"],
                                     checkpoint_sha256=None if checkpoint is None else checkpoint.get("sha256"))
            _atomic_json(target, row)
            print(json.dumps({"route": route_label(route, screen), "instance": item["instance"], "success": row["success"],
                              "status": row["status"], "reason": row.get("reason"), "wall_s": round(row["wall_s"], 1)},
                             ensure_ascii=False), flush=True)
            if row["status"] == "infrastructure":
                env = None
    finally:
        records = [json.loads(path.read_text()) for path in sorted(output.glob("*/*/instances/*.json"))]
        _atomic_json(output / "report.json", build_report([row for row in records if row.get("schema") == SCHEMA]))
    return completed


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, required=True)
    parser.add_argument("--models", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--route", choices=ROUTES, required=True)
    parser.add_argument("--group", choices=("train", "dev", "test"), default="dev")
    parser.add_argument("--levels", default="", help="comma-separated box levels; empty = all")
    parser.add_argument("--seed", type=int, default=101)
    parser.add_argument("--no-screen", action="store_true")
    parser.add_argument("--stage-env", action="store_true", help="plain MOTION-311 StageEnv (no lookahead features)")
    parser.add_argument("--max-instances", type=int, help="first N instances of every level (instance order)")
    parser.add_argument("--stride", type=int, default=1, help="every k-th instance of every level")
    parser.add_argument("--contact-pool", choices=("capped", "gated"), default="capped")
    parser.add_argument("--place-gate", action="store_true", help="gate placement rows at the conveyor pose")
    parser.add_argument("--max-minutes", type=float, default=60.)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--allow-test", action="store_true")
    args = parser.parse_args(argv)
    torch.set_num_threads(args.threads)
    stop = [False]
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    levels = [level for level in args.levels.split(",") if level]
    completed = run_evaluation(args.registry, args.output, args.route, args.models, args.group, levels, args.seed,
                               not args.no_screen, args.max_minutes, args.allow_test, stop_requested=lambda: stop[0],
                               lookahead_env=not args.stage_env, limit=args.max_instances, stride=args.stride,
                               contact_pool=args.contact_pool, place_gate=args.place_gate)
    return 0 if completed else 3


if __name__ == "__main__":
    raise SystemExit(main())
