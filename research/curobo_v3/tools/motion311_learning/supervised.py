"""Train-only, task-group-held-out supervised three-stage policies (CPU only).

Targets are observed-return means, optionally mixed with one-step teachers from
already frozen later-stage networks. They are NOT globally optimal Q labels.
Only fit/label construction reads recorded future candidates. Policy inference
accepts known state and the current candidate matrix, never a replay transition.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
from typing import Sequence

import numpy as np
import torch
from torch.nn import functional as F

from motion311_learning.contracts import C_REF_S, Q_DIM, RETURN_MIN, RETURN_MAX
from motion311_learning.dataset import load_episodes, normalized_actions, normalized_state
from motion311_learning.features import PHYSICS_DIM, STATE_DIM
from motion311_learning.networks import StageSupervised


TARGET_VERSION = "weighted-observed-mean-plus-bounded-frozen-next-v2"


def _key(values) -> bytes:
    return np.ascontiguousarray(values, dtype=np.float64).tobytes()


def _task_key(episode) -> str:
    # Offsets, repeat indices and trajectory identities must NOT split one task.
    return json.dumps(episode["task"], sort_keys=True, separators=(",", ":"))


def _finite(values, name):
    values = np.asarray(values, dtype=np.float64)
    if not np.isfinite(values).all():
        raise ValueError(f"nonfinite {name}")
    return values


def validate_train_episodes(episodes) -> None:
    if not episodes:
        raise ValueError("no train episodes")
    for episode in episodes:
        if episode.get("split") != "train" or episode.get("group") != "train":
            raise ValueError("supervised fitting accepts train only; dev/test are forbidden")
        transitions = episode.get("transitions", [])
        if not transitions or not transitions[-1].get("terminated"):
            raise ValueError("complete terminated training episodes are required")
        if any(row.get("truncated") or not row.get("trainable", False) for row in transitions):
            raise ValueError("truncated or untrainable episodes cannot supply supervised labels")


def split_task_groups(episodes, seed=311, fraction=0.2):
    """Fixed train-only task split, before aggregation or label construction."""
    validate_train_episodes(episodes)
    if not 0.0 < fraction < 1.0:
        raise ValueError("holdout fraction must be in (0, 1)")
    groups = sorted({_task_key(episode) for episode in episodes})
    if len(groups) < 2:
        raise ValueError("at least two complete task groups are needed for internal holdout")
    rng = np.random.default_rng(seed)
    order = rng.permutation(len(groups))
    count = min(len(groups) - 1, max(1, int(math.ceil(len(groups) * fraction))))
    heldout = {groups[index] for index in order[:count]}
    train = [episode for episode in episodes if _task_key(episode) not in heldout]
    validation = [episode for episode in episodes if _task_key(episode) in heldout]
    return train, validation, {"train_tasks": sorted(set(groups) - heldout), "holdout_tasks": sorted(heldout),
                               "seed": int(seed), "fraction": float(fraction)}


def _suffix_cost(episode, row):
    if not row["episode_success"]:
        return None
    total = float(episode["transitions"][-1]["info"]["cumulative_cost_s"])
    potential = float(row["state"][-1])
    if not math.isfinite(total) or not 0 <= potential < 1:
        raise ValueError("successful suffix requires finite cost and a valid cost potential")
    before = C_REF_S * potential / (1.0 - potential)
    if total + 1e-6 < before:
        raise ValueError("successful cumulative cost precedes the current state")
    return max(0.0, total - before)


def aggregate_stage(episodes, stage: int, physical=False):
    """Arithmetic means for exact (state, action_q); no best-of-leaf labels."""
    validate_train_episodes(episodes)
    if stage not in range(3):
        raise ValueError("stage must be 0, 1 or 2")
    groups = {}
    for episode in episodes:
        for row in episode["transitions"]:
            if row["stage"] != stage:
                continue
            state = _finite(row["state"], "state")
            action = _finite(row["action_q"], "action_q")
            if state.shape != (STATE_DIM,) or action.shape != (Q_DIM,):
                raise ValueError("unexpected state/action shape")
            body = None
            if physical:
                body = _finite(row["action_physics"], "action_physics")
                if body.shape != (PHYSICS_DIM,):
                    raise ValueError("unexpected physical feature shape")
            key = (_key(state), _key(action))
            if key not in groups:
                groups[key] = dict(state=state, action_q=action, physics=body, rows=[], returns=[], successes=[], log_costs=[], weights=[], cost_weights=[])
            group = groups[key]
            if physical and not np.array_equal(group["physics"], body):
                raise ValueError("identical state/action has inconsistent physical features")
            group["rows"].append(row)
            value = float(row["return"])
            if not math.isfinite(value):
                raise ValueError("nonfinite observed return")
            group["returns"].append(value)
            weight = float(row.get("suffix_weight", 1.))
            if not math.isfinite(weight) or weight <= 0:
                raise ValueError("suffix_weight must be positive and finite")
            group["weights"].append(weight)
            group["successes"].append(float(row["episode_success"]))
            cost = _suffix_cost(episode, row)
            if cost is not None:
                group["log_costs"].append(math.log(max(cost, 1e-6)))
                group["cost_weights"].append(weight)
    records = []
    for key in sorted(groups):
        group = groups[key]
        group["state_key"] = key[0]
        group["count"] = len(group["rows"])
        weights = group.pop("weights")
        group["row_weights"] = weights
        group["effective_weight"] = float(sum(weights))
        group["value"] = float(np.average(group.pop("returns"), weights=weights))
        group["success_probability"] = float(np.average(group.pop("successes"), weights=weights))
        costs = group.pop("log_costs")
        group["has_success_cost"] = bool(costs)
        group["log_success_cost"] = float(np.average(costs, weights=group["cost_weights"])) if costs else 0.0
        group.pop("cost_weights")
        records.append(group)
    return records


class SupervisedPolicy:
    """Raw-state/current-candidate wrapper; input normalization is internal.

    current_candidates is [N,15], or [N,15+PHYSICS_DIM] in physical mode.
    It may be obtained from Observation.candidate_features(physical=...).
    Predictions are ensemble-averaged; select returns a current row index.
    """

    def __init__(self, models: Sequence[Sequence[StageSupervised]], config: dict):
        self.models = [list(stage) for stage in models]
        self.config = copy.deepcopy(config)
        self.physical = bool(config["physical"])
        self.action_dim = Q_DIM + (PHYSICS_DIM if self.physical else 0)
        if len(self.models) != 3:
            raise ValueError("policy requires three stage slots")
        for members in self.models:
            for model in members:
                if any(parameter.device.type != "cpu" for parameter in model.parameters()):
                    raise ValueError("supervised policy is CPU-only")
                model.requires_grad_(False).train(False)

    @torch.no_grad()
    def predict(self, state, current_candidates, stage: int) -> np.ndarray:
        if stage not in range(3) or not self.models[stage]:
            raise ValueError("requested stage is not fitted")
        state = _finite(state, "state")
        rows = _finite(current_candidates, "current_candidates")
        if state.shape != (STATE_DIM,) or rows.ndim != 2 or rows.shape[1] != self.action_dim:
            raise ValueError("incorrect state or current candidate shape")
        if not len(rows):
            raise ValueError("cannot infer from an empty current candidate pool")
        states = torch.as_tensor(normalized_state(state), dtype=torch.float32, device="cpu").expand(len(rows), -1)
        actions = torch.as_tensor(normalized_actions(rows[:, :Q_DIM], rows[:, Q_DIM:] if self.physical else None),
                                  dtype=torch.float32, device="cpu")
        output = torch.stack([model(states, actions) for model in self.models[stage]]).mean(0)
        if not torch.isfinite(output).all():
            raise FloatingPointError("nonfinite supervised prediction")
        return output.numpy()

    def predict_values(self, state, current_candidates, stage: int) -> np.ndarray:
        return self.predict(state, current_candidates, stage)[:, 0]

    def select(self, state, current_candidates, stage: int) -> int:
        return int(np.argmax(self.predict_values(state, current_candidates, stage)))

    def save(self, path, report=None) -> None:
        if any(not stage for stage in self.models):
            raise ValueError("all stages must be fitted before saving")
        torch.save({"version": 1, "kind": "motion311-supervised", "config": self.config,
                    "state_dict": [[model.state_dict() for model in stage] for stage in self.models],
                    "report": report}, path)

    @classmethod
    def load(cls, path):
        saved = torch.load(path, map_location="cpu", weights_only=True)
        if saved.get("version") != 1 or saved.get("kind") != "motion311-supervised":
            raise ValueError("incompatible supervised checkpoint")
        config = saved["config"]
        if config["state_dim"] != STATE_DIM or config["physics_dim"] != PHYSICS_DIM:
            raise ValueError("checkpoint feature schema differs from this code")
        action_dim = Q_DIM + (PHYSICS_DIM if config["physical"] else 0)
        models = []
        with torch.random.fork_rng(devices=[]):
            for stage, width in enumerate(config["widths"]):
                members = []
                for state in saved["state_dict"][stage]:
                    model = StageSupervised(STATE_DIM, action_dim, width)
                    model.load_state_dict(state)
                    members.append(model)
                models.append(members)
        return cls(models, config)


def teacher_targets(records, stage, policy: SupervisedPolicy | None, mc_weight=0.5):
    """Mean observed return blended with frozen one-step estimates, training only.

    Missing future data is not synthesized. Terminal rows use immediate reward;
    an explicitly empty recorded pool is terminal. No teacher is used at stage 2.
    """
    if not 0.0 <= mc_weight <= 1.0:
        raise ValueError("mc_weight must be in [0, 1]")
    targets = []
    cache = {}
    for group in records:
        estimates, weights = [], []
        for row in group["rows"]:
            if row["terminated"] or stage == 2:
                estimates.append(float(row["reward"]))
                weights.append(float(row.get("suffix_weight", 1.)))
                continue
            future = row.get("next")
            if future is None or policy is None:
                continue
            if future["stage"] != stage + 1:
                raise ValueError("teacher transition must advance exactly one stage")
            candidates = np.asarray(future["candidates"], dtype=np.float64).reshape(-1, Q_DIM)
            if not len(candidates):
                estimates.append(float(row["reward"]))
                weights.append(float(row.get("suffix_weight", 1.)))
                continue
            if policy.physical:
                body = np.asarray(future["physics"], dtype=np.float64).reshape(len(candidates), PHYSICS_DIM)
                candidates = np.concatenate([candidates, body], axis=1)
            key = (_key(future["state"]), _key(candidates))
            if key not in cache:
                raw = float(np.max(policy.predict_values(future["state"], candidates, stage + 1)))
                cache[key] = float(np.clip(raw, RETURN_MIN, RETURN_MAX))
            estimates.append(float(row["reward"]) + cache[key])
            weights.append(float(row.get("suffix_weight", 1.)))
        target = group["value"]
        if estimates and stage != 2:
            target = mc_weight * target + (1.0 - mc_weight) * float(np.average(estimates, weights=weights))
        targets.append(float(np.clip(target, RETURN_MIN, RETURN_MAX)))
    return _finite(targets, "teacher targets").astype(np.float32)


def _tensors(records, physical):
    states = torch.as_tensor(np.stack([normalized_state(row["state"]) for row in records]), dtype=torch.float32)
    actions = torch.as_tensor(normalized_actions([row["action_q"] for row in records],
                                                [row["physics"] for row in records] if physical else None), dtype=torch.float32)
    values = torch.tensor([row["value"] for row in records], dtype=torch.float32)
    success = torch.tensor([row["success_probability"] for row in records], dtype=torch.float32)
    costs = torch.tensor([row["log_success_cost"] for row in records], dtype=torch.float32)
    mask = torch.tensor([row["has_success_cost"] for row in records], dtype=torch.bool)
    # Pairwise ranking only compares alternatives for the exact same state.
    states_to_rows = {}
    for index, row in enumerate(records):
        states_to_rows.setdefault(row["state_key"], []).append(index)
    pairs, seen = [], 0
    pair_rng = np.random.default_rng(311)
    # Bounded reservoir: large alternative pools must not allocate O(N^2) rows.
    for indices in states_to_rows.values():
        for k, i in enumerate(indices):
            for j in indices[k + 1:]:
                if abs(records[i]["value"] - records[j]["value"]) <= 1e-6:
                    continue
                seen += 1
                if len(pairs) < 20000:
                    pairs.append((i, j))
                else:
                    replace = int(pair_rng.integers(seen))
                    if replace < len(pairs):
                        pairs[replace] = (i, j)
    pairs = torch.tensor(pairs, dtype=torch.long).reshape(-1, 2)
    return dict(states=states, actions=actions, values=values, success=success, costs=costs, mask=mask, pairs=pairs)


def _rank_loss(predicted, target, pairs):
    if not len(pairs):
        return predicted.sum() * 0.0
    difference = target[pairs[:, 0]] - target[pairs[:, 1]]
    margin = predicted[pairs[:, 0]] - predicted[pairs[:, 1]]
    return (F.softplus(-difference.sign() * margin) * difference.abs().clamp(max=2.0)).mean()


def _metrics(output, data):
    value = F.smooth_l1_loss(output[:, 0], data["values"])
    ranking = _rank_loss(output[:, 0], data["values"], data["pairs"])
    return {"selection_loss": float(value + 0.25 * ranking), "value_huber": float(value),
            "ranking_loss": float(ranking),
            "success_bce": float(F.binary_cross_entropy_with_logits(output[:, 1], data["success"]))}


def _fit_member(train, validation, targets, width, seed, epochs, patience, lr):
    with torch.random.fork_rng(devices=[]):
        torch.random.default_generator.manual_seed(seed)
        model = StageSupervised(STATE_DIM, train["actions"].shape[1], width)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    best, best_epoch, best_state, curve = float("inf"), -1, None, []
    targets = torch.as_tensor(targets, dtype=torch.float32)
    for epoch in range(epochs):
        model.train(True)
        prediction = model(train["states"], train["actions"])
        value_loss = F.smooth_l1_loss(prediction[:, 0], targets)
        # Value and pairwise ranking optimize the same MC-anchored improvement target.
        rank_loss = _rank_loss(prediction[:, 0], targets, train["pairs"])
        classification = F.binary_cross_entropy_with_logits(prediction[:, 1], train["success"])
        cost_loss = (F.smooth_l1_loss(prediction[train["mask"], 2], train["costs"][train["mask"]])
                     if train["mask"].any() else prediction[:, 2].sum() * 0.0)
        loss = value_loss + 0.25 * rank_loss + 0.1 * classification + 0.05 * cost_loss
        if not torch.isfinite(loss):
            raise FloatingPointError("nonfinite supervised training loss")
        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 10.0, error_if_nonfinite=True)
        optimizer.step()
        model.train(False)
        with torch.no_grad():
            metrics = _metrics(model(validation["states"], validation["actions"]), validation)
        if not all(math.isfinite(value) for value in metrics.values()):
            raise FloatingPointError("nonfinite supervised holdout metric")
        curve.append(dict(epoch=epoch, train_loss=float(loss.detach()), **metrics))
        if metrics["selection_loss"] < best - 1e-8:
            best, best_epoch = metrics["selection_loss"], epoch
            best_state = copy.deepcopy(model.state_dict())
        elif epoch - best_epoch >= patience:
            break
    model.load_state_dict(best_state)
    model.requires_grad_(False).train(False)
    return model, dict(seed=int(seed), width=int(width), best_epoch=best_epoch,
                       best_selection_loss=best, curve=curve)


def fit_episodes(episodes, physical=False, seed=311, epochs=200, threads=4,
                 widths=(32, 64), seeds=None, patience=20, holdout_fraction=0.2,
                 mc_weight=0.5, backward=True, lr=1e-3):
    """Fit reversed stages and select width by mean internal-holdout value/rank loss.

    Three independent seeds are retained as an ensemble for each chosen width.
    No external dev/test row is allowed, including rows used only for selection.
    """
    episodes = list(episodes)
    validate_train_episodes(episodes)
    if epochs < 1 or patience < 1 or threads < 1 or not math.isfinite(lr) or lr <= 0:
        raise ValueError("epochs, patience, threads and lr must be positive")
    widths = tuple(int(width) for width in widths)
    if not widths or any(width < 1 for width in widths):
        raise ValueError("width grid must contain positive widths")
    seeds = tuple(int(value) for value in (seeds if seeds is not None else (seed, seed + 1, seed + 2)))
    if len(seeds) != 3 or len(set(seeds)) != 3:
        raise ValueError("exactly three distinct training seeds are required")
    if not 0.0 <= mc_weight <= 1.0:
        raise ValueError("mc_weight must be in [0, 1]")
    train_episodes, heldout_episodes, split = split_task_groups(episodes, seed, holdout_fraction)
    old_threads = torch.get_num_threads()
    torch.set_num_threads(threads)
    config = dict(physical=bool(physical), state_dim=STATE_DIM, physics_dim=PHYSICS_DIM,
                  widths=[None, None, None], seeds=list(seeds), hidden_layers=2,
                  device="cpu", target_version=TARGET_VERSION if backward else "observed-return-mean-v1",
                  mc_weight=float(mc_weight), gamma=1.0, backward=bool(backward),
                  normalization="fixed-physical-units-dataset-v1", input="raw-state-and-current-candidates",
                  selection="internal-task-holdout-value-huber+0.25*pairwise-ranking", split=split)
    report = {"config": config, "stages": {}, "external_dev_used": False,
              "label_semantics": "observed-return mean blended with frozen one-step teacher; not global optimal Q",
              "cost_label": "mean log of observed successful suffix cumulative_cost_s difference"}
    policy = SupervisedPolicy([[], [], []], config)
    try:
        for stage in (2, 1, 0):
            train_records = aggregate_stage(train_episodes, stage, physical)
            heldout_records = aggregate_stage(heldout_episodes, stage, physical)
            if not train_records or not heldout_records:
                raise ValueError(f"stage {stage} lacks train/internal-holdout samples after complete-task split")
            targets = (teacher_targets(train_records, stage, policy, mc_weight) if backward
                       else np.array([row["value"] for row in train_records], dtype=np.float32))
            train = _tensors(train_records, physical)
            validation = _tensors(heldout_records, physical)
            trials, selected_models, selected_score, selected_width = [], None, float("inf"), None
            for width in widths:
                members, summaries = [], []
                for member_seed in seeds:
                    model, summary = _fit_member(train, validation, targets, width, member_seed, epochs, patience, lr)
                    members.append(model)
                    summaries.append(summary)
                score = float(np.mean([summary["best_selection_loss"] for summary in summaries]))
                trials.append(dict(width=width, selection_loss_mean=score, members=summaries))
                if score < selected_score:
                    selected_models, selected_score, selected_width = members, score, width
            policy.models[stage] = selected_models
            policy.config["widths"][stage] = selected_width
            config["widths"][stage] = selected_width
            target_hash = hashlib.sha256(np.ascontiguousarray(targets).tobytes()).hexdigest()
            report["stages"][str(stage)] = dict(train_aggregates=len(train_records), holdout_aggregates=len(heldout_records),
                                                observed_train_rows=sum(row["count"] for row in train_records),
                                                ranking_pairs=len(train["pairs"]), target_sha256=target_hash,
                                                target_min=float(targets.min()), target_max=float(targets.max()),
                                                selected_width=selected_width, selection_loss=selected_score, trials=trials)
    finally:
        torch.set_num_threads(old_threads)
    return policy, report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--physical", action="store_true")
    parser.add_argument("--seed", type=int, default=311)
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--threads", type=int, default=4)
    args = parser.parse_args(argv)
    source = args.data / "episodes" if (args.data / "episodes").is_dir() else args.data
    if args.output.exists() and any(args.output.iterdir()):
        raise FileExistsError("output must be new or empty; do not overwrite trained evidence")
    episodes = list(load_episodes(source, split="train"))
    policy, report = fit_episodes(episodes, physical=args.physical, seed=args.seed,
                                  epochs=args.epochs, threads=args.threads)
    args.output.mkdir(parents=True, exist_ok=True)
    policy.save(args.output / "checkpoint.pt", report=report)
    (args.output / "training_report.json").write_text(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2) + "\n")
    print(json.dumps({"checkpoint": str(args.output / "checkpoint.pt"), "widths": policy.config["widths"],
                      "target_version": policy.config["target_version"], "train_episodes": len(episodes)}))


if __name__ == "__main__":
    main()
