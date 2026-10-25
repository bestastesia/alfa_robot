"""CPU-only, three-stage learning networks for standardized motion features.

Stages are 0/1/2. A transition bootstraps from stage + 1, never from the
same stage, and stage 2 is always terminal. DQN scores only supplied
candidate rows; future candidate pools belong to training transitions only.
SAC uses a fixed entropy temperature (default 0.05), not automatic tuning.
Feature normalization and replay sampling are intentionally external.
"""
from __future__ import annotations

import copy
import math
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np
import torch
from torch import nn
from torch.nn import functional as F


_STAGES = 3
_CPU = torch.device("cpu")


def _dimension(value: int, name: str) -> int:
    if isinstance(value, bool) or int(value) != value or int(value) <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return int(value)


def _stage(value: int) -> int:
    if isinstance(value, bool) or int(value) != value or not 0 <= int(value) < _STAGES:
        raise ValueError("stage must be 0, 1, or 2")
    return int(value)


def _tensor(value) -> torch.Tensor:
    if isinstance(value, torch.Tensor) and value.device.type != "cpu":
        raise ValueError("learning networks accept CPU tensors only")
    tensor = torch.as_tensor(value, dtype=torch.float32, device=_CPU)
    if not torch.isfinite(tensor).all():
        raise ValueError("features and rewards must be finite")
    return tensor.detach()


def _vector(value, size: int, name: str) -> torch.Tensor:
    tensor = _tensor(value)
    if tensor.shape != (size,):
        raise ValueError(f"{name} must have shape ({size},), got {tuple(tensor.shape)}")
    return tensor


def _candidates(value, size: int) -> torch.Tensor:
    tensor = _tensor(value)
    if tensor.shape == (0,):
        tensor = tensor.reshape(0, size)
    if tensor.ndim != 2 or tensor.shape[1] != size:
        raise ValueError(f"candidate_features must have shape (N, {size})")
    return tensor


def _check_matrix(value: torch.Tensor, size: int, name: str) -> None:
    if value.device.type != "cpu":
        raise ValueError("learning networks accept CPU tensors only")
    if value.ndim != 2 or value.shape[1] != size:
        raise ValueError(f"{name} must have shape (B, {size})")


def _mlp(input_dim: int, output_dim: int, width: int) -> nn.Sequential:
    return nn.Sequential(
        nn.Linear(input_dim, width, device=_CPU), nn.ReLU(),
        nn.Linear(width, width, device=_CPU), nn.ReLU(),
        nn.Linear(width, output_dim, device=_CPU),
    )


class StageQ(nn.Module):
    """Candidate-conditioned Q(s, a), returning one scalar per batch row."""

    def __init__(self, state_dim: int, action_dim: int, width: int = 64):
        super().__init__()
        self.state_dim = _dimension(state_dim, "state_dim")
        self.action_dim = _dimension(action_dim, "action_dim")
        self.net = _mlp(self.state_dim + self.action_dim, 1, _dimension(width, "width"))

    def forward(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        _check_matrix(state, self.state_dim, "state")
        _check_matrix(action, self.action_dim, "action")
        if state.shape[0] != action.shape[0]:
            raise ValueError("state and action batch sizes must match")
        return self.net(torch.cat((state, action), dim=-1)).squeeze(-1)


class StageSupervised(StageQ):
    """Shared trunk with [value, success_logit, log_success_cost] outputs."""

    def __init__(self, state_dim: int, action_dim: int, width: int = 64):
        super().__init__(state_dim, action_dim, width)
        self.net[-1] = nn.Linear(_dimension(width, "width"), 3, device=_CPU)


class GaussianActor(nn.Module):
    """Reparameterized tanh Gaussian; forward/sample return (action, logprob).

    logprob has shape [B] and includes the tanh Jacobian. Its numerically
    stable pre-tanh expression stays finite even when tanh rounds to +/-1.
    """

    def __init__(self, state_dim: int, action_dim: int = 3, width: int = 64):
        super().__init__()
        self.state_dim = _dimension(state_dim, "state_dim")
        self.action_dim = _dimension(action_dim, "action_dim")
        self.net = _mlp(self.state_dim, 2 * self.action_dim, _dimension(width, "width"))

    def _parameters_for(self, state: torch.Tensor):
        _check_matrix(state, self.state_dim, "state")
        mean, log_std = self.net(state).chunk(2, dim=-1)
        return mean, log_std.clamp(-20.0, 2.0)

    def sample(self, state: torch.Tensor, generator: torch.Generator | None = None,
               action_mask: torch.Tensor | None = None):
        mean, log_std = self._parameters_for(state)
        noise = torch.randn(mean.shape, dtype=mean.dtype, device=_CPU, generator=generator)
        pre_tanh = mean + log_std.exp() * noise
        # Use sampled noise to avoid cancellation in (pre_tanh - mean) / std.
        normal_logprob = -0.5 * (noise.square() + 2.0 * log_std + math.log(2.0 * math.pi))
        log_jacobian = 2.0 * (math.log(2.0) - pre_tanh - F.softplus(-2.0 * pre_tanh))
        action = pre_tanh.tanh()
        per_dimension = normal_logprob - log_jacobian
        if action_mask is not None:
            mask = self._validated_mask(action_mask, action)
            action, per_dimension = action * mask, per_dimension * mask
        return action, per_dimension.sum(dim=-1)

    @staticmethod
    def _validated_mask(mask: torch.Tensor, reference: torch.Tensor) -> torch.Tensor:
        if mask.device.type != "cpu" or mask.shape != reference.shape:
            raise ValueError("action_mask must be a CPU tensor with shape [B, action_dim]")
        if not torch.isfinite(mask).all() or not ((mask == 0) | (mask == 1)).all():
            raise ValueError("action_mask entries must be zero or one")
        return mask.detach().to(dtype=reference.dtype)

    def forward(self, state: torch.Tensor, generator: torch.Generator | None = None,
                action_mask: torch.Tensor | None = None):
        return self.sample(state, generator=generator, action_mask=action_mask)

    def deterministic(self, state: torch.Tensor, action_mask: torch.Tensor | None = None) -> torch.Tensor:
        mean, _ = self._parameters_for(state)
        action = mean.tanh()
        return action if action_mask is None else action * self._validated_mask(action_mask, action)


class TwinCritic(nn.Module):
    def __init__(self, state_dim: int, action_dim: int = 3, width: int = 64):
        super().__init__()
        self.q1 = StageQ(state_dim, action_dim, width)
        self.q2 = StageQ(state_dim, action_dim, width)

    def forward(self, state: torch.Tensor, action: torch.Tensor):
        return self.q1(state, action), self.q2(state, action)


def _config(state_dim, action_dim, width, lr, gamma, tau, seed, device):
    if torch.device(device).type != "cpu":
        raise ValueError("these learning agents are CPU-only")
    if not math.isfinite(lr) or lr <= 0:
        raise ValueError("lr must be positive and finite")
    if not 0.0 <= gamma <= 1.0 or not 0.0 < tau <= 1.0:
        raise ValueError("gamma must be in [0, 1] and tau in (0, 1]")
    return dict(state_dim=_dimension(state_dim, "state_dim"),
                action_dim=_dimension(action_dim, "action_dim"),
                width=_dimension(width, "width"), lr=float(lr), gamma=float(gamma),
                tau=float(tau), seed=int(seed), device="cpu")


def _batch(batch: Sequence[Mapping], state_dim: int, action_dim: int):
    if not len(batch):
        raise ValueError("batch must not be empty")
    stages = [_stage(row["stage"]) for row in batch]
    states = torch.stack([_vector(row["state"], state_dim, "state") for row in batch])
    actions = torch.stack([_vector(row["action"], action_dim, "action") for row in batch])
    rewards = _tensor([float(row["reward"]) for row in batch])
    return stages, states, actions, rewards


def _finite(value: torch.Tensor, name: str) -> torch.Tensor:
    if not torch.isfinite(value).all():
        raise FloatingPointError(f"nonfinite {name}")
    return value


@torch.no_grad()
def _soft_update(online: nn.Module, target: nn.Module, tau: float) -> None:
    for source, destination in zip(online.parameters(), target.parameters()):
        destination.lerp_(source, tau)


class _Checkpoint:
    def save(self, path: str | Path) -> None:
        torch.save({"version": 1, "agent": type(self).__name__, "config": self.config,
                    "state_dict": self.state_dict()}, path)

    @classmethod
    def load(cls, path: str | Path):
        checkpoint = torch.load(path, map_location="cpu", weights_only=True)
        if checkpoint.get("version") != 1 or checkpoint.get("agent") != cls.__name__:
            raise ValueError("incompatible learning-agent checkpoint")
        agent = cls(**checkpoint["config"])
        agent.load_state_dict(checkpoint["state_dict"])
        return agent


class DQNAgent(_Checkpoint):
    """Three independent candidate-conditioned Double-DQN stage networks."""

    def __init__(self, state_dim: int, action_dim: int, width: int = 64,
                 lr: float = 3e-4, gamma: float = 1.0, tau: float = 0.005,
                 seed: int = 0, device: str = "cpu"):
        self.config = _config(state_dim, action_dim, width, lr, gamma, tau, seed, device)
        self.state_dim, self.action_dim = self.config["state_dim"], self.config["action_dim"]
        self.gamma, self.tau = self.config["gamma"], self.config["tau"]
        self.device = _CPU
        self.rng = np.random.default_rng(seed)
        # Seed only the CPU generator and restore the caller's global RNG.
        with torch.random.fork_rng(devices=[]):
            torch.random.default_generator.manual_seed(seed)
            self.online = nn.ModuleList([StageQ(state_dim, action_dim, width) for _ in range(_STAGES)])
        self.target = copy.deepcopy(self.online).requires_grad_(False).train(False)
        self.optimizers = [torch.optim.Adam(model.parameters(), lr=lr) for model in self.online]

    @torch.no_grad()
    def select(self, state, candidate_features, stage: int, epsilon: float = 0.0,
               rng: np.random.Generator | None = None) -> int:
        """Return an index in the *current* supplied candidate matrix."""
        stage = _stage(stage)
        if not 0.0 <= epsilon <= 1.0:
            raise ValueError("epsilon must be in [0, 1]")
        candidates = _candidates(candidate_features, self.action_dim)
        if not len(candidates):
            raise ValueError("cannot select from an empty current candidate pool")
        state = _vector(state, self.state_dim, "state")
        rng = self.rng if rng is None else rng
        if epsilon > 0.0 and rng.random() < epsilon:
            return int(rng.integers(len(candidates)))
        scores = _finite(self.online[stage](state.expand(len(candidates), -1), candidates), "Q scores")
        return int(scores.argmax().item())

    @torch.no_grad()
    def compute_targets(self, batch: Sequence[Mapping]) -> torch.Tensor:
        stages, _, _, rewards = _batch(batch, self.state_dim, self.action_dim)
        targets = rewards.clone()
        for stage in range(_STAGES - 1):
            indices, states, pools = [], [], []
            for index, row in enumerate(batch):
                if (stages[index] != stage or row["terminated"] or row["next_state"] is None
                        or row["next_candidates"] is None):
                    continue
                candidates = _candidates(row["next_candidates"], self.action_dim)
                if not len(candidates):
                    continue
                indices.append(index)
                states.append(_vector(row["next_state"], self.state_dim, "next_state"))
                pools.append(candidates)
            if not indices:
                continue
            # Pack variable-length pools for one forward per next-stage model.
            repeated = torch.cat([state.expand(len(pool), -1) for state, pool in zip(states, pools)])
            candidates = torch.cat(pools)
            online_q = _finite(self.online[stage + 1](repeated, candidates), "next online Q")
            selected, offset = [], 0
            for pool in pools:
                selected.append(offset + int(online_q[offset:offset + len(pool)].argmax()))
                offset += len(pool)
            # Online chooses; target evaluates those identical selected rows.
            future = self.target[stage + 1](repeated[selected], candidates[selected])
            targets[indices] += self.gamma * future
        return _finite(targets, "DQN targets")

    def update(self, batch: Sequence[Mapping]) -> dict[str, float]:
        stages, states, actions, _ = _batch(batch, self.state_dim, self.action_dim)
        targets = self.compute_targets(batch)
        loss_sum, td_sum = 0.0, 0.0
        for stage in range(_STAGES):
            indices = [index for index, value in enumerate(stages) if value == stage]
            if not indices:
                continue
            optimizer = self.optimizers[stage]
            predicted = self.online[stage](states[indices], actions[indices])
            loss = _finite(F.smooth_l1_loss(predicted, targets[indices]), "DQN loss")
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_(self.online[stage].parameters(), 10.0, error_if_nonfinite=True)
            optimizer.step()
            loss_sum += float(loss.detach()) * len(indices)
            td_sum += float((predicted.detach() - targets[indices]).abs().sum())
        _soft_update(self.online, self.target, self.tau)
        return {"loss": loss_sum / len(batch), "td_abs_mean": td_sum / len(batch),
                "target_mean": float(targets.mean())}

    def state_dict(self) -> dict:
        return {"online": self.online.state_dict(), "target": self.target.state_dict(),
                "optimizers": [optimizer.state_dict() for optimizer in self.optimizers],
                "numpy_rng": copy.deepcopy(self.rng.bit_generator.state)}

    def load_state_dict(self, state: Mapping) -> None:
        self.online.load_state_dict(state["online"])
        self.target.load_state_dict(state["target"])
        if len(state["optimizers"]) != _STAGES:
            raise ValueError("checkpoint must contain three optimizers")
        for optimizer, saved in zip(self.optimizers, state["optimizers"]):
            optimizer.load_state_dict(saved)
        self.rng.bit_generator.state = state["numpy_rng"]


class SACAgent(_Checkpoint):
    """Three-stage continuous SAC, fixed alpha=0.05 and gamma=1 by default.

    None next_candidates is permitted for continuous-action transitions. An
    explicitly empty candidate pool means the environment has no legal next
    action and suppresses bootstrapping, as do termination and the final stage.
    """

    def __init__(self, state_dim: int, action_dim: int = 3, width: int = 64,
                 lr: float = 3e-4, gamma: float = 1.0, tau: float = 0.005,
                 alpha: float = 0.05, seed: int = 0, device: str = "cpu",
                 action_mask_state_indices: Sequence[int] | None = None):
        self.config = _config(state_dim, action_dim, width, lr, gamma, tau, seed, device)
        self.action_mask_state_indices = None
        if action_mask_state_indices is not None:
            indices = tuple(action_mask_state_indices)
            if (action_dim != 3 or len(indices) != 2 or len(set(indices)) != 2
                    or any(isinstance(index, bool) or int(index) != index or not 0 <= index < state_dim
                           for index in indices)):
                raise ValueError("action mask requires action_dim=3 and two distinct valid state indices")
            self.action_mask_state_indices = tuple(int(index) for index in indices)
        self.config["action_mask_state_indices"] = (None if self.action_mask_state_indices is None
                                                   else list(self.action_mask_state_indices))
        if not math.isfinite(alpha) or alpha < 0:
            raise ValueError("fixed alpha must be nonnegative and finite")
        self.config["alpha"] = float(alpha)
        self.state_dim, self.action_dim = self.config["state_dim"], self.config["action_dim"]
        self.gamma, self.tau, self.alpha = self.config["gamma"], self.config["tau"], float(alpha)
        self.device = _CPU
        self.generator = torch.Generator(device=_CPU).manual_seed(seed)
        with torch.random.fork_rng(devices=[]):
            torch.random.default_generator.manual_seed(seed)
            self.actors = nn.ModuleList([GaussianActor(state_dim, action_dim, width) for _ in range(_STAGES)])
            self.critics = nn.ModuleList([TwinCritic(state_dim, action_dim, width) for _ in range(_STAGES)])
        self.target_critics = copy.deepcopy(self.critics).requires_grad_(False).train(False)
        self.actor_optimizers = [torch.optim.Adam(model.parameters(), lr=lr) for model in self.actors]
        self.critic_optimizers = [torch.optim.Adam(model.parameters(), lr=lr) for model in self.critics]

    def action_mask(self, states: torch.Tensor) -> torch.Tensor | None:
        """Optional explicit contract: two state flags encode left/right activity.

        No state layout is assumed unless action_mask_state_indices is provided.
        The updown dimension is always active; inactive arm psi is exactly zero.
        """
        if self.action_mask_state_indices is None:
            return None
        _check_matrix(states, self.state_dim, "states")
        flags = states[:, list(self.action_mask_state_indices)]
        if not torch.isfinite(flags).all() or not ((flags == 0) | (flags == 1)).all() or not (flags.sum(-1) >= 1).all():
            raise ValueError("mask state flags must be binary with at least one active arm")
        return torch.cat((torch.ones((len(states), 1), dtype=states.dtype, device=_CPU), flags.detach()), dim=-1)

    def _sample(self, actor, states):
        mask = self.action_mask(states)
        if mask is None:
            return actor.sample(states, generator=self.generator)
        return actor.sample(states, generator=self.generator, action_mask=mask)

    @torch.no_grad()
    def select(self, state, stage: int, deterministic: bool = False) -> np.ndarray:
        actor = self.actors[_stage(stage)]
        state = _vector(state, self.state_dim, "state").unsqueeze(0)
        action = (actor.deterministic(state, action_mask=self.action_mask(state)) if deterministic
                  else self._sample(actor, state)[0])
        return _finite(action[0], "SAC action").numpy().copy()

    @torch.no_grad()
    def compute_targets(self, batch: Sequence[Mapping]) -> torch.Tensor:
        stages, _, _, rewards = _batch(batch, self.state_dim, self.action_dim)
        targets = rewards.clone()
        for stage in range(_STAGES - 1):
            indices, states = [], []
            for index, row in enumerate(batch):
                if stages[index] != stage or row["terminated"] or row["next_state"] is None:
                    continue
                candidates = row.get("next_candidates")
                if candidates is not None and len(candidates) == 0:
                    continue
                indices.append(index)
                states.append(_vector(row["next_state"], self.state_dim, "next_state"))
            if not indices:
                continue
            state = torch.stack(states)
            action, logprob = self._sample(self.actors[stage + 1], state)
            q1, q2 = self.target_critics[stage + 1](state, action)
            targets[indices] += self.gamma * (torch.minimum(q1, q2) - self.alpha * logprob)
        return _finite(targets, "SAC targets")

    def update(self, batch: Sequence[Mapping]) -> dict[str, float]:
        stages, states, actions, _ = _batch(batch, self.state_dim, self.action_dim)
        if (actions.abs() > 1.0).any():
            raise ValueError("SAC replay actions must be in [-1, 1]")
        mask = self.action_mask(states)
        if mask is not None:
            actions = actions * mask
        targets = self.compute_targets(batch)
        critic_sum, actor_sum, entropy_sum = 0.0, 0.0, 0.0
        for stage in range(_STAGES):
            indices = [index for index, value in enumerate(stages) if value == stage]
            if not indices:
                continue
            state, action, target = states[indices], actions[indices], targets[indices]
            critic, actor = self.critics[stage], self.actors[stage]
            critic_optimizer, actor_optimizer = self.critic_optimizers[stage], self.actor_optimizers[stage]
            q1, q2 = critic(state, action)
            critic_loss = _finite(F.mse_loss(q1, target) + F.mse_loss(q2, target), "SAC critic loss")
            critic_optimizer.zero_grad(set_to_none=True)
            critic_loss.backward()
            nn.utils.clip_grad_norm_(critic.parameters(), 10.0, error_if_nonfinite=True)
            critic_optimizer.step()
            critic_optimizer.zero_grad(set_to_none=True)

            # Freeze weights, not the action derivative. Actor gradients must
            # flow through Q(s, actor(s)); restore the critic on exceptions too.
            critic.requires_grad_(False)
            try:
                sampled, logprob = self._sample(actor, state)
                policy_q1, policy_q2 = critic(state, sampled)
                actor_loss = _finite((self.alpha * logprob - torch.minimum(policy_q1, policy_q2)).mean(),
                                     "SAC actor loss")
                actor_optimizer.zero_grad(set_to_none=True)
                actor_loss.backward()
                nn.utils.clip_grad_norm_(actor.parameters(), 10.0, error_if_nonfinite=True)
                actor_optimizer.step()
            finally:
                critic.requires_grad_(True)
            critic_sum += float(critic_loss.detach()) * len(indices)
            actor_sum += float(actor_loss.detach()) * len(indices)
            entropy_sum += float(-logprob.detach().sum())
        _soft_update(self.critics, self.target_critics, self.tau)
        return {"critic_loss": critic_sum / len(batch), "actor_loss": actor_sum / len(batch),
                "loss": (critic_sum + actor_sum) / len(batch), "entropy": entropy_sum / len(batch),
                "target_mean": float(targets.mean()), "alpha": self.alpha}

    def state_dict(self) -> dict:
        return {"actors": self.actors.state_dict(), "critics": self.critics.state_dict(),
                "target_critics": self.target_critics.state_dict(),
                "actor_optimizers": [optimizer.state_dict() for optimizer in self.actor_optimizers],
                "critic_optimizers": [optimizer.state_dict() for optimizer in self.critic_optimizers],
                "torch_rng": self.generator.get_state()}

    def load_state_dict(self, state: Mapping) -> None:
        self.actors.load_state_dict(state["actors"])
        self.critics.load_state_dict(state["critics"])
        self.target_critics.load_state_dict(state["target_critics"])
        for key in ("actor_optimizers", "critic_optimizers"):
            if len(state[key]) != _STAGES:
                raise ValueError("checkpoint must contain three optimizers of each kind")
            for optimizer, saved in zip(getattr(self, key), state[key]):
                optimizer.load_state_dict(saved)
        self.generator.set_state(state["torch_rng"])
