"""Screened candidate-conditioned Double DQN with a feasibility head (MOTION-364 R3 evolution), CPU only.

Input per candidate: MOTION-311 normalized state, box dims, normalized joint row and the stage's lookahead
features. Each stage network outputs [Q, feasibility logit]; the logit is trained on the real gate/planning
outcome of the executed action (BCE), Q on Double-DQN targets over the next stage's gate-passing pool only.
`from_311` widens a MOTION-311 DQNAgent with zero weights on the new inputs, so the initial Q is identical.
An optional fixed analytic prior makes Q = network + prior, prior = -weight * (the stage's analytic cost in s);
with `residual=True` the widened network's Q output starts at a constant, so the initial ranking is the prior's.
selection="veto" ranks by the prior's analytic cost alone and only demotes candidates whose predicted success
probability (the logit head) is more than `veto_margin` below the pool's best; an untrained head vetoes nothing.
"""
import copy
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.nn import functional as F

from motion311_learning.dataset import normalized_actions, normalized_state
from motion311_learning.features import STATE_DIM
from motion306_learning.env import EXTRA_DIM

STATES = 3
DIM_SCALE = .4
EXTRA_SCALE = np.array([[10., 10., 1., 10., 10., 1.], [10., 1., 10., 1., 1., 1.], [10., 10., 20., 1., 1., 1.]],
                       dtype=np.float32)
STATE_IN = STATE_DIM + 3
ACTION_IN = 15 + EXTRA_DIM
FEASIBLE_CLASSES = ("stage_success", "success", "empty_next_candidates")
DEAD_END_VALUE = -1.  # a continuing transition whose next pool has no gate-passing row can only fail next
# Lookahead columns (s) summed by the prior per stage: "heuristic" = MOTION-311 local movement (+ Home at placement),
# "lookahead" adds extraction and the nearest gated carry goal at contact (rankers.lookahead_heuristic_order).
PRIOR_COLUMNS = {"heuristic": ((0,), (0,), (0, 1)), "lookahead": ((0, 1, 3), (0,), (0, 1))}
RESIDUAL_OFFSET = .5


def state_input(observation):
    return np.concatenate([normalized_state(observation.state), np.asarray(observation.dims, dtype=np.float32) / DIM_SCALE])


def candidate_inputs(observation, rows=None):
    rows = slice(None) if rows is None else rows
    extra = np.asarray(observation.extra, dtype=np.float32)[rows] / EXTRA_SCALE[observation.stage]
    return np.concatenate([normalized_actions(observation.candidates[rows]), extra], axis=1).astype(np.float32)


def feasible(info):
    return float(info.get("class_") in FEASIBLE_CLASSES)


def prior_values(stage, inputs, kind, weight):
    """-weight * analytic stage cost (s) from candidate inputs [normalized row, scaled lookahead]; zero for "none"."""
    inputs = torch.as_tensor(inputs)
    if kind == "none" or weight == 0:
        return torch.zeros(len(inputs))
    seconds = inputs[:, 15:] * torch.as_tensor(EXTRA_SCALE[stage])
    return -weight * seconds[:, list(PRIOR_COLUMNS[kind][stage])].sum(dim=1)


def _mlp(width):
    return nn.Sequential(nn.Linear(STATE_IN + ACTION_IN, width), nn.ReLU(), nn.Linear(width, width), nn.ReLU(),
                         nn.Linear(width, 2))


class LookaheadAgent:
    def __init__(self, width=64, lr=3e-4, gamma=1., tau=.005, feasibility_weight=.5, seed=0, prior="none", prior_weight=0.,
                 selection="q", veto_margin=.15, value_weight=1.):
        if prior != "none" and prior not in PRIOR_COLUMNS:
            raise ValueError("unknown prior")
        if selection not in ("q", "veto") or (selection == "veto" and prior == "none"):
            raise ValueError("selection is q or veto; veto needs an analytic prior")
        self.config = dict(width=int(width), lr=float(lr), gamma=float(gamma), tau=float(tau),
                           feasibility_weight=float(feasibility_weight), seed=int(seed), prior=prior,
                           prior_weight=float(prior_weight), selection=selection, veto_margin=float(veto_margin),
                           value_weight=float(value_weight))
        self.gamma, self.tau, self.feasibility_weight = float(gamma), float(tau), float(feasibility_weight)
        self.prior, self.prior_weight = prior, float(prior_weight)
        self.selection, self.veto_margin, self.value_weight = selection, float(veto_margin), float(value_weight)
        self.rng = np.random.default_rng(seed)
        with torch.random.fork_rng(devices=[]):
            torch.random.default_generator.manual_seed(seed)
            self.online = nn.ModuleList([_mlp(width) for _ in range(STATES)])
        self.target = copy.deepcopy(self.online).requires_grad_(False)
        self.optimizers = [torch.optim.Adam(model.parameters(), lr=lr) for model in self.online]

    @classmethod
    def from_311(cls, dqn, residual=False, **kwargs):
        """Copy a MOTION-311 DQNAgent's stage networks; new inputs start at zero weight, the logit row is fresh.
        residual=True keeps the 311 hidden features but zeroes the Q output, leaving the prior's ranking."""
        agent = cls(**{**dict(width=dqn.config["width"], gamma=dqn.gamma, tau=dqn.tau, seed=dqn.config["seed"]), **kwargs})
        for old, new in zip(dqn.online, agent.online):
            with torch.no_grad():
                first, middle, last = old.net[0], old.net[2], old.net[4]
                new[0].weight.zero_()
                new[0].weight[:, :STATE_DIM] = first.weight[:, :STATE_DIM]
                new[0].weight[:, STATE_IN:STATE_IN + 15] = first.weight[:, STATE_DIM:]
                new[0].bias.copy_(first.bias)
                new[2].weight.copy_(middle.weight)
                new[2].bias.copy_(middle.bias)
                new[4].weight[0] = 0. if residual else last.weight[0]
                new[4].bias[0] = RESIDUAL_OFFSET if residual else last.bias[0]
                if residual:
                    new[4].weight[1] = 0.  # constant initial success prediction: a veto starts with no effect
                new[4].bias[1] = 2.
        agent.target = copy.deepcopy(agent.online).requires_grad_(False)
        agent.optimizers = [torch.optim.Adam(model.parameters(), lr=agent.config["lr"]) for model in agent.online]
        return agent

    @torch.no_grad()
    def scores(self, observation):
        """(Q, feasibility probability) for every current candidate."""
        rows = torch.as_tensor(candidate_inputs(observation))
        state = torch.as_tensor(state_input(observation)).expand(len(rows), -1)
        output = self.online[observation.stage](torch.cat((state, rows), dim=1))
        q = output[:, 0] + prior_values(observation.stage, rows, self.prior, self.prior_weight)
        return q.numpy(), torch.sigmoid(output[:, 1]).numpy()

    def order(self, observation, epsilon=0., rng=None):
        """Descending Q, or a random permutation with probability epsilon (exploration among gate-passing rows)."""
        rng = self.rng if rng is None else rng
        if epsilon > 0 and rng.random() < epsilon:
            return rng.permutation(len(observation.candidates))
        q, probability = self.scores(observation)
        if self.selection == "q":
            return np.argsort(-q, kind="stable")
        cost = -prior_values(observation.stage, candidate_inputs(observation), self.prior, 1.).numpy()
        vetoed = probability < probability.max() - self.veto_margin
        return np.lexsort((cost, vetoed))  # cheapest first; vetoed rows keep their cost order at the end

    @torch.no_grad()
    def compute_targets(self, batch):
        targets = torch.tensor([float(row["reward"]) for row in batch])
        for index, row in enumerate(batch):
            if row["terminated"] or row["next_state"] is None:
                continue
            if not len(row["next_candidates"]):
                targets[index] += self.gamma * DEAD_END_VALUE
                continue
            stage = row["stage"] + 1
            candidates = torch.as_tensor(np.asarray(row["next_candidates"], dtype=np.float32))
            inputs = torch.cat((torch.as_tensor(np.asarray(row["next_state"], dtype=np.float32)).expand(len(candidates), -1),
                                candidates), dim=1)
            prior = prior_values(stage, candidates, self.prior, self.prior_weight)
            chosen = int((self.online[stage](inputs)[:, 0] + prior).argmax())
            targets[index] += self.gamma * (self.target[stage](inputs[chosen:chosen + 1])[0, 0] + prior[chosen])
        return targets

    def update(self, batch):
        targets = self.compute_targets(batch)
        total, feasibility_loss = 0., 0.
        for stage in range(STATES):
            indices = [i for i, row in enumerate(batch) if row["stage"] == stage]
            if not indices:
                continue
            inputs = torch.as_tensor(np.stack([np.concatenate([batch[i]["state"], batch[i]["action"]]) for i in indices]),
                                     dtype=torch.float32)
            output = self.online[stage](inputs)
            labels = torch.tensor([float(batch[i]["feasible"]) for i in indices])
            prior = prior_values(stage, inputs[:, STATE_IN:], self.prior, self.prior_weight)
            value_loss = F.smooth_l1_loss(output[:, 0] + prior, targets[indices])
            logit_loss = F.binary_cross_entropy_with_logits(output[:, 1], labels)
            loss = self.value_weight * value_loss + self.feasibility_weight * logit_loss
            if not torch.isfinite(loss):
                raise FloatingPointError("nonfinite loss")
            self.optimizers[stage].zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_(self.online[stage].parameters(), 10., error_if_nonfinite=True)
            self.optimizers[stage].step()
            total += float(value_loss.detach()) * len(indices)
            feasibility_loss += float(logit_loss.detach()) * len(indices)
        with torch.no_grad():
            for source, destination in zip(self.online.parameters(), self.target.parameters()):
                destination.lerp_(source, self.tau)
        return dict(value_loss=total / len(batch), feasibility_loss=feasibility_loss / len(batch),
                    target_mean=float(targets.mean()))

    def state_dict(self):
        return dict(online=self.online.state_dict(), target=self.target.state_dict(),
                    optimizers=[optimizer.state_dict() for optimizer in self.optimizers],
                    numpy_rng=copy.deepcopy(self.rng.bit_generator.state))

    def load_state_dict(self, state):
        self.online.load_state_dict(state["online"])
        self.target.load_state_dict(state["target"])
        for optimizer, saved in zip(self.optimizers, state["optimizers"]):
            optimizer.load_state_dict(saved)
        self.rng.bit_generator.state = state["numpy_rng"]

    def save(self, path, training_state=None):
        content = dict(version=1, agent=type(self).__name__, config=self.config, state_dict=self.state_dict())
        if training_state is not None:
            content["training_state"] = copy.deepcopy(training_state)
        path = Path(path)
        temporary = path.with_suffix(path.suffix + ".tmp")
        torch.save(content, temporary)
        temporary.replace(path)

    @classmethod
    def load(cls, path, with_training_state=False):
        content = torch.load(path, map_location="cpu", weights_only=True)
        if content.get("version") != 1 or content.get("agent") != cls.__name__:
            raise ValueError("incompatible lookahead-agent checkpoint")
        agent = cls(**content["config"])
        agent.load_state_dict(content["state_dict"])
        return (agent, content.get("training_state")) if with_training_state else agent
