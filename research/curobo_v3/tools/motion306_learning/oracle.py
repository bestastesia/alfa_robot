"""Offline future-optimal reference search for MOTION-364: training labels and evaluation, never a runtime policy.

From Home, each stage expands up to K gate-passing candidates with real planning through StageEnv
checkpoint/restore under the strict evaluation's planner seed: the heuristic's and the analytic lookahead's first
gate-passing choices, then half of the remaining slots in heuristic order and the rest seeded-random among the
other gate-passing rows. Every leaf is a complete episode. The reference optimum is the successful leaf with the
shortest time-parameterized motion duration: the best within the searched tree, not a proven global optimum.
The heuristic's own path is always in the tree, so its regret against that reference is exact. Replicates re-run
the heuristic policy and the best triple under other planner seeds, separating choice-dependent path failures
from RRT randomness. Dev only; launch under the GPU flock.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import time

import numpy as np
import torch

from motion311_learning.contracts import config_digest
from motion311_learning.evaluate import _atomic_json, _jsonable, instance_seed, mode, sha256_file
from motion311_learning.stage_env import EpisodeFailure
from motion306_learning.evaluate import _source_fingerprint
from motion306_learning.rankers import heuristic_order, lookahead_heuristic_order

SCHEMA = "motion364-future-optimum-v1"
ADVANCED = ("stage_success", "success", "empty_next_candidates")


def passing_rows(env, observation):
    """Indices of the current pool passing the stage's deterministic gates (the selector-side screening gates)."""
    rows = observation.candidates
    if observation.stage == 0:
        passing = []
        for index in np.flatnonzero(env.contact_collision_free(rows)):
            try:
                env.contact_checks(rows[int(index)])
            except EpisodeFailure:
                continue
            passing.append(int(index))
        return passing
    reasons = getattr(env, "gate_reasons", None)
    if reasons is None:
        reasons = env.carry_checks(rows) if observation.stage == 1 else [None] * len(rows)
    return [index for index, reason in enumerate(reasons) if reason is None]


def branches(observation, passing, velocity, k, rng):
    """Up to k distinct gate-passing (index, roles): the heuristic's and the analytic lookahead's first choices,
    then half of the remaining slots in heuristic order and the rest seeded-random among the other rows."""
    allowed = set(passing)
    local = [int(index) for index in heuristic_order(observation, velocity) if int(index) in allowed]
    if not local:
        return []
    roles = {local[0]: ["heuristic"]}
    lookahead = next(int(index) for index in lookahead_heuristic_order(observation) if int(index) in allowed)
    roles.setdefault(lookahead, []).append("lookahead")
    for index in [index for index in local if index not in roles][:(max(0, k - len(roles)) + 1) // 2]:
        roles[index] = ["local"]
    rest = [index for index in local if index not in roles]
    for index in (rng.permutation(rest)[:max(0, k - len(roles))] if rest else []):
        roles[int(index)] = ["random"]
    return list(roles.items())[:k]


def edge_record(env, observation, path, roles, result):
    info = result.info
    block = env.timed_blocks[-1] if info.get("class_") in ADVANCED and getattr(env, "timed", False) and env.timed_blocks else None
    return dict(stage=int(observation.stage), path=[int(index) for index in path], roles=list(roles),
                row=np.asarray(observation.candidates[path[-1]]).tolist(), class_=info.get("class_"),
                reason=info.get("reason"), success=bool(info.get("success")), terminated=bool(result.terminated),
                truncated=bool(result.truncated), stage_cost_s=info.get("stage_cost_s"),
                cumulative_cost_s=info.get("cumulative_cost_s"),
                stage_duration_s=None if block is None else block.get("motion_duration_s"),
                motion_duration_s=info.get("motion_duration_s"), wall_ms=info.get("wall_ms"))


def search(env, root, ks, velocity, seed_root):
    """Depth-first expansion from a stage-0 checkpoint: one edge per executed step; siblings restore their node."""
    edges, nodes = [], []

    def expand(checkpoint, path):
        observation = env.restore(checkpoint)
        passing = passing_rows(env, observation)
        rng = np.random.default_rng(np.random.SeedSequence([int(seed_root), int(observation.stage), *path]))
        picks = branches(observation, passing, velocity, ks[observation.stage], rng)
        nodes.append(dict(stage=int(observation.stage), path=list(path), pool=len(observation.candidates),
                          passing=len(passing), picks=[index for index, _ in picks]))
        for index, roles in picks:
            observation = env.restore(checkpoint)
            result = env.step(index)
            edges.append(edge_record(env, observation, path + [index], roles, result))
            if not result.terminated and not result.truncated:
                expand(env.checkpoint(), path + [index])

    expand(root, [])
    return edges, nodes


def follow(edges, role):
    """Last edge of the path a single-choice policy takes through the tree (the edge carrying `role` per node)."""
    path, last = [], None
    for stage in range(3):
        edge = next((edge for edge in edges if edge["stage"] == stage and edge["path"][:-1] == path
                     and role in edge["roles"]), None)
        if edge is None:
            break
        path, last = edge["path"], edge
        if edge["terminated"] or edge["truncated"]:
            break
    return last


def best_leaf(edges, key="motion_duration_s"):
    leaves = [edge for edge in edges if edge["stage"] == 2 and edge["success"] and edge.get(key) is not None]
    return min(leaves, key=lambda edge: (edge[key], edge["cumulative_cost_s"])) if leaves else None


def path_summary(edges, last):
    if last is None:
        return None
    along = [next(edge for edge in edges if edge["path"] == last["path"][:depth]) for depth in range(1, len(last["path"]) + 1)]
    return dict(path=last["path"], rows=[edge["row"] for edge in along], roles=[edge["roles"] for edge in along],
                success=bool(last["stage"] == 2 and last["success"]), stage=last["stage"], reason=last["reason"],
                motion_duration_s=last["motion_duration_s"], cumulative_cost_s=last["cumulative_cost_s"])


def run_policy(env, item, planner_seed, choose):
    """One strict episode; choose(env, observation, passing) names the executed index (None stops the episode)."""
    outcome = dict(planner_seed=int(planner_seed), success=False, path=[], stage=None, reason=None, motion_duration_s=None)
    try:
        observation = env.reset(item, seed=planner_seed)
    except EpisodeFailure as failure:
        return dict(outcome, stage=0, reason=failure.reason)
    for _ in range(3):
        if not len(observation.candidates):
            return dict(outcome, stage=int(observation.stage), reason="no_candidates")
        index = choose(env, observation, passing_rows(env, observation))
        if index is None:
            return dict(outcome, stage=int(observation.stage), reason="no_choice")
        result = env.step(index)
        info = result.info
        outcome["path"].append(int(index))
        outcome.update(stage=int(observation.stage), class_=info.get("class_"), reason=info.get("reason"),
                       cumulative_cost_s=info.get("cumulative_cost_s"))
        if result.terminated or result.truncated:
            return dict(outcome, success=bool(info.get("success")), motion_duration_s=info.get("motion_duration_s"))
        observation = result.observation
    raise ValueError("three decisions completed without termination")


def heuristic_choice(env, observation, passing):
    allowed = set(passing)
    return next((int(index) for index in heuristic_order(observation, env.velocity) if int(index) in allowed), None)


def fixed_choice(summary):
    """Replays a searched path; a row that differs from the searched one stops the episode (no silent remap)."""
    def choose(env, observation, passing):
        index, row = summary["path"][observation.stage], summary["rows"][observation.stage]
        same = index < len(observation.candidates) and np.allclose(observation.candidates[index], row, atol=1e-9)
        return index if same else None
    return choose


def search_instance(env, item, seed, ks, replicate_seeds):
    began = time.perf_counter()
    planner_seed = instance_seed(item, seed)
    record = dict(schema=SCHEMA, instance=item["instance"], base_instance=item.get("base_instance"), split=item["split"],
                  mode=mode(item), box_level=item.get("box_level"), size_split=item.get("size_split"), seed=int(seed),
                  planner_seed=planner_seed, branches=list(ks), load_average_start=list(os.getloadavg()))
    try:
        observation = env.reset(item, seed=planner_seed)
    except EpisodeFailure as failure:
        return dict(record, status="scene_infeasible", reason=failure.reason, wall_s=time.perf_counter() - began)
    if not len(observation.candidates):
        return dict(record, status="scene_infeasible", reason="scene_no_contact_candidates", wall_s=time.perf_counter() - began)
    edges, nodes = search(env, env.checkpoint(), ks, env.velocity, planner_seed)
    searched = time.perf_counter()
    best = path_summary(edges, best_leaf(edges))
    replicates = []
    for other in replicate_seeds:
        other_seed = instance_seed(item, other)
        entry = dict(seed=int(other), heuristic=run_policy(env, item, other_seed, heuristic_choice))
        if best is not None:
            entry["best"] = run_policy(env, item, other_seed, fixed_choice(best))
        replicates.append(entry)
    return dict(record, status="searched", edges=edges, nodes=nodes, heuristic=path_summary(edges, follow(edges, "heuristic")),
                lookahead=path_summary(edges, follow(edges, "lookahead")), best=best,
                leaves=sum(edge["stage"] == 2 and edge["success"] for edge in edges), steps=len(edges), replicates=replicates,
                search_s=searched - began, replicate_s=time.perf_counter() - searched, wall_s=time.perf_counter() - began,
                load_average_end=list(os.getloadavg()))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, required=True)
    parser.add_argument("--instances", type=Path, required=True, help="file with one dev instance name per line, run in order")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=101, help="evaluation seed; the planner seed is instance_seed(item, seed)")
    parser.add_argument("--branches", default="5,4,3", help="gate-passing candidates expanded per stage")
    parser.add_argument("--replicates", default="102,103,104,105", help="other evaluation seeds re-running heuristic and best")
    parser.add_argument("--max-minutes", type=float, default=120.)
    parser.add_argument("--threads", type=int, default=4)
    args = parser.parse_args(argv)
    torch.set_num_threads(args.threads)
    ks = [int(value) for value in args.branches.split(",")]
    replicate_seeds = [int(value) for value in args.replicates.split(",") if value]
    if len(ks) != 3 or min(ks) < 1:
        parser.error("--branches needs three positive counts")
    names = args.instances.read_text().split()
    items = {item["instance"]: item for item in json.loads(args.registry.read_text())["instances"] if item["instance"] in set(names)}
    missing = [name for name in names if name not in items]
    if missing or any(items[name]["split"] != "dev" or items[name].get("group") != "dev" for name in names):
        parser.error(f"only dev instances of the registry are searched: {missing or 'non-dev entry'}")
    configuration = dict(schema=SCHEMA, seed=args.seed, branches=ks, replicates=replicate_seeds, place_gate=True,
                         contact_pool="capped", registry_sha256=sha256_file(args.registry), source_sha256=_source_fingerprint())
    config_file = args.output / "config.json"
    if config_file.exists() and json.loads(config_file.read_text()) != configuration:
        raise SystemExit("resume configuration differs; use a new output directory")
    _atomic_json(config_file, configuration)
    config_hash = config_digest(configuration)
    stop = [False]
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    from motion306_learning.env import LookaheadEnv
    started, env, level = time.perf_counter(), None, None
    for name in names:
        item = items[name]
        target = args.output / "instances" / (hashlib.sha256(name.encode()).hexdigest()[:24] + ".json")
        if target.exists():
            continue
        if stop[0] or time.perf_counter() - started >= args.max_minutes * 60:
            return 3
        if env is None:
            env = LookaheadEnv(timed=True, physical=False, contact_pool="capped", place_gate=True)
        if level is not None and item.get("box_level") != level:
            env.planner.persistent_checkers.clear()  # one size at a time on the 8 GB card
            torch.cuda.empty_cache()
        level = item.get("box_level")
        try:
            record = search_instance(env, item, args.seed, ks, replicate_seeds)
        except Exception as error:  # infrastructure: keep the record, rebuild the runtime
            record = dict(schema=SCHEMA, instance=name, status="infrastructure", error_type=type(error).__name__, error=str(error)[:500])
            env = None
        record["config_sha256"] = config_hash
        _atomic_json(target, _jsonable(record))
        summary = {key: record.get(key) for key in ("instance", "status", "steps", "leaves", "wall_s")}
        for key in ("heuristic", "best"):
            summary[key] = None if not record.get(key) else {k: record[key][k] for k in ("success", "motion_duration_s", "path")}
        print(json.dumps(_jsonable(summary), ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
