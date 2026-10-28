#!/usr/bin/env python3
"""Online screened lookahead DQN over the seen box sizes, with a crash-consistent per-episode journal.

Episodes start at Home on group=train, split=train items of the training size levels, balanced over
(level, mode). Each stage ranks by Q (a random order with probability epsilon), executes the first
gate-passing candidate once and records the transition; next-stage pools keep only gate-passing rows.
An episode record is durable before its updates; agent.pt commits weights with the episode watermark, so a
restart replays updates and never reruns the environment. Real environment calls need the outer GPU flock.
Held-out size levels and dev/test items are refused.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import signal
import time

import numpy as np
import torch

from motion311_learning.contracts import COST_WEIGHT, config_digest, potential
from motion311_learning.evaluate import mode
from motion311_learning.networks import DQNAgent
from motion311_learning.train import atomic_json, file_sha256
from motion306_learning.agent import LookaheadAgent, candidate_inputs, feasible, state_input
from motion306_learning.screening import screened_choice

RESUME_VERSION = 1
INFO_KEYS = ("stage", "class_", "reason", "success", "stage_cost_s", "cumulative_cost_s", "wall_ms", "generation_ms",
             "executed_t_path_s", "motion_duration_s")


def train_items(registry, levels):
    heldout = set(registry.get("heldout_levels", ()))
    if heldout & set(levels):
        raise ValueError("held-out box levels never enter training")
    items = [item for item in registry["instances"] if item["group"] == "train" and item["split"] == "train"
             and item.get("box_level") in levels]
    if not items:
        raise ValueError("no group=train, split=train items for the requested levels")
    groups = {}
    for item in sorted(items, key=lambda row: row["instance"]):
        task = json.dumps(item["task"], sort_keys=True, separators=(",", ":"))
        groups.setdefault((item["box_level"], mode(item)), {}).setdefault(task, []).append(item)
    return [[groups[key][task] for task in sorted(groups[key])] for key in sorted(groups)]


def sample_item(groups, rng):
    """Uniform (level, mode) -> task -> instance, as MOTION-311 sampled mode -> task -> instance."""
    tasks = groups[int(rng.integers(len(groups)))]
    instances = tasks[int(rng.integers(len(tasks)))]
    return copy.deepcopy(instances[int(rng.integers(len(instances)))])


def epsilon_at(index, episodes, start, end):
    return float(end + (start - end) * max(0., 1. - index / max(1, episodes - 1)))


def transition(observation, index, result, screening, action_rank):
    info = {key: result.info.get(key) for key in INFO_KEYS if key in result.info}
    following = result.observation
    return dict(stage=int(observation.stage), state=state_input(observation).tolist(),
                action=candidate_inputs(observation, [index])[0].tolist(), action_index=int(index),
                action_rank=int(action_rank), reward=float(result.reward), terminated=bool(result.terminated),
                truncated=bool(result.truncated), feasible=feasible(result.info), info=info, screening=screening,
                next_state=None if following is None else state_input(following).tolist(),
                next_candidates=None if following is None else candidate_inputs(following, following.valid).tolist())


def shaped_rewards(rows, cost_weight):
    """The environment reward with another cost weight: success +/-1 - w * delta raw_T/(raw_T+40)."""
    rewards, before = [], 0.
    for row in rows:
        after = float(row["info"].get("cumulative_cost_s", before))
        terminal = 0. if not row["terminated"] else (1. if row["info"].get("success") else -1.)
        rewards.append(terminal - cost_weight * (potential(after) - potential(before)))
        before = after
    return rewards


def learnable(record, cost_weight=COST_WEIGHT, success_target="stage"):
    """Replay rows; the logit label is the executed stage's own outcome ("stage") or the episode's ("episode")."""
    rows = record.get("transitions", [])
    if not rows or any(row["truncated"] for row in rows):
        return []
    rewards = [row["reward"] for row in rows] if cost_weight == COST_WEIGHT else shaped_rewards(rows, cost_weight)
    episode = float(bool(rows[-1]["info"].get("success")))
    return [dict(stage=row["stage"], state=np.asarray(row["state"], dtype=np.float32),
                 action=np.asarray(row["action"], dtype=np.float32), reward=reward, terminated=row["terminated"],
                 feasible=row["feasible"] if success_target == "stage" else episode, next_state=None if row["next_state"] is None else np.asarray(row["next_state"], dtype=np.float32),
                 next_candidates=None if row["next_candidates"] is None else np.asarray(row["next_candidates"], dtype=np.float32))
            for row, reward in zip(rows, rewards)]


def run_episode(env, agent, item, planner_seed, epsilon, rng):
    """One Home-start episode; returns (transitions, scene_failure_reason)."""
    from motion311_learning.stage_env import EpisodeFailure
    try:
        observation = env.reset(item, seed=planner_seed)
    except EpisodeFailure as failure:
        return [], failure.reason
    if not len(observation.candidates):
        return [], "scene_no_contact_candidates"
    transitions = []
    for _ in range(3):
        order = agent.order(observation, epsilon, rng)
        chosen, screening = screened_choice(env, order)
        index = int(order[0]) if chosen is None else chosen
        result = env.step(index)
        transitions.append(transition(observation, index, result, screening, int(np.flatnonzero(order == index)[0])))
        if result.terminated or result.truncated:
            break
        observation = result.observation
    return transitions, None


def commit(agent, replay, state, record, path, updates, checkpoint, cost_weight=COST_WEIGHT, success_target="stage"):
    index = record["episode_index"]
    if index != state["last_committed_episode"] + 1:
        raise ValueError("episodes must commit in contiguous order")
    rows = learnable(record, cost_weight, success_target)
    replay.extend(rows)
    rng = np.random.default_rng(np.random.SeedSequence([record["seed"], index, 7]))
    count = 0
    for _ in range(updates * len(rows)):
        if len(replay) >= 32:
            ids = rng.choice(len(replay), min(64, len(replay)), replace=False)
            agent.update([replay[i] for i in ids])
            count += 1
    state = copy.deepcopy(state)
    state["last_committed_episode"] = index
    state["committed_records"].append(file_sha256(path))
    state["gradient_updates"] += count
    agent.save(checkpoint, state)
    return state


def source_fingerprint():
    tools = Path(__file__).resolve().parents[1]
    files = [tools / "motion311_learning" / name for name in ("stage_env.py", "redundancy.py", "features.py", "contracts.py", "dataset.py")]
    files += sorted((tools / "motion306_learning").glob("*.py")) + [tools / "v3_box_geometry.py"]
    return config_digest({str(path.relative_to(tools)): file_sha256(path) for path in files})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--init-311", type=Path, help="MOTION-311 R3 agent.pt to widen (function-preserving)")
    parser.add_argument("--levels", default="", help="comma-separated training levels; empty = all non-held-out")
    parser.add_argument("--episodes", type=int, default=3000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-minutes", type=float, default=60.)
    parser.add_argument("--updates", type=int, default=4)
    parser.add_argument("--epsilon-start", type=float, default=.3)
    parser.add_argument("--epsilon-end", type=float, default=.05)
    parser.add_argument("--feasibility-weight", type=float, default=.5)
    parser.add_argument("--cost-weight", type=float, default=COST_WEIGHT, help="reward weight of the raw_T potential")
    parser.add_argument("--prior", choices=("none", "heuristic", "lookahead"), default="none",
                        help="fixed analytic prior added to Q (agent.PRIOR_COLUMNS)")
    parser.add_argument("--prior-weight", type=float, default=0., help="prior per second of analytic stage cost")
    parser.add_argument("--residual", action="store_true", help="widen 311 but start its Q output at a constant")
    parser.add_argument("--contact-pool", choices=("capped", "gated"), default="capped")
    parser.add_argument("--place-gate", action="store_true", help="gate placement rows at the conveyor pose")
    parser.add_argument("--selection", choices=("q", "veto"), default="q")
    parser.add_argument("--veto-margin", type=float, default=.15)
    parser.add_argument("--success-target", choices=("stage", "episode"), default="stage")
    parser.add_argument("--threads", type=int, default=4)
    args = parser.parse_args(argv)
    if args.episodes < 1 or args.updates < 0 or not math.isfinite(args.max_minutes) or args.max_minutes <= 0:
        parser.error("invalid episode/update/time budget")
    torch.set_num_threads(args.threads)
    registry = json.loads(args.registry.read_text())
    levels = [level for level in args.levels.split(",") if level] or [
        level for level in registry["size_levels"] if level not in registry.get("heldout_levels", ())]
    groups = train_items(registry, levels)
    configuration = dict(schema="motion364-lookahead-dqn-v1", resume_version=RESUME_VERSION, seed=args.seed,
                         episodes=args.episodes, updates=args.updates, levels=sorted(levels),
                         epsilon=[args.epsilon_start, args.epsilon_end], feasibility_weight=args.feasibility_weight,
                         prior=args.prior, prior_weight=args.prior_weight, residual=args.residual, contact_pool=args.contact_pool,
                         place_gate=args.place_gate, selection=args.selection, veto_margin=args.veto_margin,
                         success_target=args.success_target,
                         registry_sha256=file_sha256(args.registry), source_sha256=source_fingerprint(),
                         init_311=None if args.init_311 is None else dict(path=str(args.init_311.resolve()), sha256=file_sha256(args.init_311)),
                         reward=f"success +/-1 -{args.cost_weight:g} delta raw_T/(raw_T+40)", screening="selector-side exact gates")
    args.output.mkdir(parents=True, exist_ok=True)
    config_file = args.output / "config.json"
    if config_file.exists() and json.loads(config_file.read_text()) != configuration:
        raise RuntimeError("cannot resume a different training configuration")
    atomic_json(config_file, configuration)
    checkpoint, episodes_dir = args.output / "agent.pt", args.output / "episodes"
    replay = []
    if checkpoint.exists():
        agent, state = LookaheadAgent.load(checkpoint, with_training_state=True)
        if state is None or state.get("configuration_sha256") != config_digest(configuration):
            raise ValueError("checkpoint does not belong to this configuration")
        for index in range(state["last_committed_episode"] + 1):
            path = episodes_dir / f"e{index:06d}.json"
            if file_sha256(path) != state["committed_records"][index]:
                raise ValueError("committed record is missing or modified")
            replay.extend(learnable(json.loads(path.read_text()), args.cost_weight, args.success_target))
    else:
        if episodes_dir.is_dir() and any(episodes_dir.glob("*.json")):
            raise ValueError("records exist without their initial checkpoint")
        kwargs = dict(feasibility_weight=args.feasibility_weight, seed=args.seed, prior=args.prior,
                      prior_weight=args.prior_weight, selection=args.selection, veto_margin=args.veto_margin)
        if args.residual and args.init_311 is None:
            parser.error("--residual widens a MOTION-311 agent; pass --init-311")
        agent = (LookaheadAgent.from_311(DQNAgent.load(args.init_311), residual=args.residual, **kwargs)
                 if args.init_311 is not None else LookaheadAgent(**kwargs))
        state = dict(version=RESUME_VERSION, configuration_sha256=config_digest(configuration), last_committed_episode=-1,
                     committed_records=[], gradient_updates=0)
        agent.save(checkpoint, state)
    stop = [False]
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    started, env = time.perf_counter(), None
    for index in range(state["last_committed_episode"] + 1, args.episodes):
        if stop[0] or time.perf_counter() - started > args.max_minutes * 60:
            return 3
        path = episodes_dir / f"e{index:06d}.json"
        if path.exists():
            record = json.loads(path.read_text())
            if record.get("config_sha256") != config_digest(configuration) or record.get("episode_index") != index:
                raise ValueError("pending record provenance differs")
        else:
            if env is None:
                from motion306_learning.env import LookaheadEnv
                env = LookaheadEnv(timed=True, physical=False, contact_pool=args.contact_pool, place_gate=args.place_gate)
            rng = np.random.default_rng(np.random.SeedSequence([args.seed, index, 364]))
            item = sample_item(groups, rng)
            epsilon = epsilon_at(index, args.episodes, args.epsilon_start, args.epsilon_end)
            began = time.perf_counter()
            transitions, scene = run_episode(env, agent, item, args.seed * 1000003 + index, epsilon, rng)
            record = dict(schema=configuration["schema"], config_sha256=config_digest(configuration), episode_index=index,
                          seed=args.seed, instance=item["instance"], box_level=item["box_level"], mode=mode(item),
                          group="train", split="train", epsilon=epsilon, scene_failure=scene, transitions=transitions,
                          wall_s=time.perf_counter() - began)
            atomic_json(path, record)
            if any(row["truncated"] for row in transitions):
                env = None  # do not reuse a runtime after an infrastructure fault
        state = commit(agent, replay, state, record, path, args.updates, checkpoint, args.cost_weight, args.success_target)
        if index % 100 == 0:
            agent.save(args.output / f"checkpoint-{index:06d}.pt", state)
        final = record["transitions"][-1]["info"] if record["transitions"] else {}
        print(json.dumps(dict(episode=index, instance=record["instance"], level=record["box_level"],
                              steps=len(record["transitions"]), success=bool(final.get("success")),
                              final=record["scene_failure"] or final.get("reason"), replay=len(replay),
                              updates=state["gradient_updates"], wall_s=round(record["wall_s"], 1)), ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
