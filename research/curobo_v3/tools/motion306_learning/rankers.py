"""Candidate orderings for the screened selector; the first candidate passing the gates is executed."""
from dataclasses import asdict, dataclass
import json
from pathlib import Path

import numpy as np
import torch

from motion311_learning.dataset import normalized_actions, normalized_state


@dataclass(frozen=True)
class B1CostConfig:
    """MOTION-365 B1 soft ranking terms. Every non-dimensionless contribution is in seconds."""

    selection_budget_ms: float = 2000.
    contact_approach_weight: float = 1.
    contact_extraction_weight: float = 1.
    contact_nearest_carry_weight: float = 1.
    contact_carry_mean_excess_weight: float = 0.
    contact_scarcity_penalty_s: float = 0.
    carry_transport_weight: float = 1.
    carry_home_proxy_weight: float = 0.
    carry_updown_penalty_s_per_m: float = 0.
    place_motion_weight: float = 1.
    place_home_weight: float = 1.
    joint_limit_margin_fraction: float = .05
    joint_limit_penalty_s: float = 0.
    direct_path_samples: int = 9
    approach_direct_risk_penalty_s: float = 0.
    carry_direct_risk_penalty_s: float = 0.
    place_direct_risk_penalty_s: float = 0.
    home_direct_risk_penalty_s: float = 0.

    def __post_init__(self):
        values = asdict(self)
        if not all(np.isfinite(value) and value >= 0. for value in values.values()):
            raise ValueError("B1 cost values must be finite and nonnegative")
        if not 0. < self.selection_budget_ms:
            raise ValueError("B1 selection budget must be positive")
        if not 0. < self.joint_limit_margin_fraction <= .5:
            raise ValueError("B1 joint-limit margin fraction must be in (0, 0.5]")
        if isinstance(self.direct_path_samples, bool) or int(self.direct_path_samples) != self.direct_path_samples or self.direct_path_samples < 2:
            raise ValueError("B1 direct-path samples must be an integer >= 2")

    @classmethod
    def load(cls, path):
        document = json.loads(Path(path).read_text())
        if document.get("schema") != "motion365-b1-cost-v1" or document.get("units") != "seconds":
            raise ValueError("unsupported B1 cost config")
        return cls(**document["cost"])

    def to_dict(self):
        return asdict(self)

    def direct_risk_enabled(self, stage):
        return bool((self.approach_direct_risk_penalty_s,
                     self.carry_direct_risk_penalty_s,
                     self.place_direct_risk_penalty_s + self.home_direct_risk_penalty_s)[stage])


def heuristic_order(observation, velocity):
    """MOTION-311 local heuristic as a full ordering: current-stage movement time, plus return-home at placement."""
    state, rows = observation.state, observation.candidates
    previous, home = state[7:22], state[22:37]
    costs = np.max(np.abs(rows - previous) / velocity, axis=1)
    if observation.stage == 2:
        costs = costs + np.max(np.abs(rows - home) / velocity, axis=1)
    return np.argsort(costs, kind='stable')


def lookahead_heuristic_order(observation):
    """Analytic lookahead baseline on the same features: at contact, approach + extraction + nearest gated carry
    goal from the solved extraction endpoint; later stages equal the local heuristic (move, plus Home at placement)."""
    extra = np.asarray(observation.extra)
    if observation.stage == 0:
        costs = extra[:, 0] + extra[:, 1] + extra[:, 3]
    elif observation.stage == 1:
        costs = extra[:, 0]
    else:
        costs = extra[:, 0] + extra[:, 1]
    return np.argsort(costs, kind='stable')


def _joint_limit_penalty(rows, lower, upper, config):
    rows = np.asarray(rows, dtype=np.float64).reshape(-1, 15)
    lower, upper = np.asarray(lower, dtype=np.float64), np.asarray(upper, dtype=np.float64)
    span = upper - lower
    if rows.shape[1] != len(lower) or np.any(span <= 0.):
        raise ValueError("B1 joint limits do not match candidate rows")
    margin = np.minimum((rows - lower) / span, (upper - rows) / span).min(axis=1)
    shortfall = np.clip((config.joint_limit_margin_fraction - margin) /
                        config.joint_limit_margin_fraction, 0., 1.)
    return config.joint_limit_penalty_s * shortfall, margin


def b1_cost_terms(observation, velocity, lower, upper, config, risk=None):
    """Return auditable B1 terms and total cost; hard gates remain in selector screening."""
    if not isinstance(config, B1CostConfig):
        raise TypeError("B1 ranking requires B1CostConfig")
    extra = np.asarray(observation.extra, dtype=np.float64)
    count = len(observation.candidates)
    if config.direct_risk_enabled(observation.stage) and risk is None:
        raise ValueError("enabled B1 direct-risk term is missing")
    terms = {}
    if observation.stage == 0:
        terms["approach_s"] = config.contact_approach_weight * extra[:, 0]
        terms["extraction_s"] = config.contact_extraction_weight * extra[:, 1]
        terms["nearest_carry_s"] = config.contact_nearest_carry_weight * extra[:, 3]
        terms["carry_mean_excess_s"] = config.contact_carry_mean_excess_weight * np.maximum(extra[:, 4] - extra[:, 3], 0.)
        terms["carry_scarcity_s"] = config.contact_scarcity_penalty_s * (1. - np.clip(extra[:, 5], 0., 1.))
        if config.approach_direct_risk_penalty_s:
            terms["approach_direct_risk_s"] = config.approach_direct_risk_penalty_s * np.asarray(risk["approach"], dtype=np.float64)
    elif observation.stage == 1:
        terms["transport_s"] = config.carry_transport_weight * extra[:, 0]
        terms["home_proxy_s"] = config.carry_home_proxy_weight * extra[:, 2]
        terms["updown_soft_s"] = config.carry_updown_penalty_s_per_m * extra[:, 3]
        if config.carry_direct_risk_penalty_s:
            terms["carry_direct_risk_s"] = config.carry_direct_risk_penalty_s * np.asarray(risk["carry"], dtype=np.float64)
    elif observation.stage == 2:
        terms["placement_s"] = config.place_motion_weight * extra[:, 0]
        terms["return_home_s"] = config.place_home_weight * extra[:, 1]
        if config.place_direct_risk_penalty_s:
            terms["placement_direct_risk_s"] = config.place_direct_risk_penalty_s * np.asarray(risk["place"], dtype=np.float64)
        if config.home_direct_risk_penalty_s:
            terms["home_direct_risk_s"] = config.home_direct_risk_penalty_s * np.asarray(risk["home"], dtype=np.float64)
    else:
        raise ValueError("B1 stage must be 0, 1 or 2")
    limit_penalty, margin = _joint_limit_penalty(observation.candidates, lower, upper, config)
    terms["joint_limit_soft_s"] = limit_penalty
    total = np.sum(np.stack(list(terms.values())), axis=0) if count else np.empty(0)
    valid = np.array(getattr(observation, "valid", np.ones(count, dtype=bool)), dtype=bool, copy=True)
    valid &= margin >= -1e-9  # candidate generation should guarantee this; retain an explicit hard limit gate
    if observation.stage == 0:
        valid &= extra[:, 2] >= .5  # an unsolved extraction endpoint cannot have a finite downstream value
    total = np.where(valid, total, np.inf)
    return dict(terms=terms, total_s=total, valid=valid, joint_limit_margin_fraction=margin)


def b1_order(observation, velocity, lower, upper, config, risk=None):
    """Stable analytic B1 ordering. Candidate index is the final deterministic tie-break."""
    costs = b1_cost_terms(observation, velocity, lower, upper, config, risk)["total_s"]
    return np.argsort(costs, kind="stable")


def uniform_order(observation, rng):
    return rng.permutation(len(observation.candidates))


@torch.no_grad()
def dqn_scores(agent, observation):
    rows = torch.as_tensor(normalized_actions(observation.candidates))
    state = torch.as_tensor(normalized_state(observation.state)).expand(len(rows), -1)
    return agent.online[observation.stage](state, rows).numpy()


def dqn_order(agent, observation):
    """Descending Q; the stable sort keeps DQNAgent.select's first-maximum tie rule for the head."""
    return np.argsort(-dqn_scores(agent, observation), kind='stable')
