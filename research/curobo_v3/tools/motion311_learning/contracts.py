"""Shared finite-horizon contract; deliberately contains no planner or learning imports."""
from dataclasses import dataclass
import hashlib
import json

import numpy as np

SCHEMA = 3
STAGES = ("contact", "carry", "place")
Q_DIM = 15
ACTION_DIM = 3
C_REF_S = 40.
COST_WEIGHT = .2
RETURN_MIN = -1. - COST_WEIGHT
RETURN_MAX = 1.


def potential(cost_s):
    if not np.isfinite(cost_s) or cost_s < 0:
        raise ValueError("cost must be finite and nonnegative")
    return cost_s / (cost_s + C_REF_S)


def reward(before_s, after_s, terminal=None):
    """Terminal True=successful full cycle, False=task failure, None=nonterminal."""
    if after_s < before_s:
        raise ValueError("cumulative cost cannot decrease")
    result = -COST_WEIGHT * (potential(after_s) - potential(before_s))
    return result + (0. if terminal is None else (1. if terminal else -1.))


def row_digest(values):
    return hashlib.sha256(np.ascontiguousarray(values, dtype=np.float64).tobytes()).hexdigest()


def config_digest(config):
    return hashlib.sha256(json.dumps(config, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


@dataclass(frozen=True)
class Observation:
    stage: int
    state: np.ndarray
    candidates: np.ndarray
    actions: np.ndarray
    physics: np.ndarray
    targets: np.ndarray

    def __post_init__(self):
        if self.stage not in range(3):
            raise ValueError("stage must be 0, 1 or 2")
        shapes = {"state": (None,), "candidates": (None, Q_DIM), "actions": (None, ACTION_DIM),
                  "physics": (None, None), "targets": (2, 4, 4)}
        for name, shape in shapes.items():
            array = np.asarray(getattr(self, name), dtype=np.float64)
            if array.ndim != len(shape) or any(size is not None and size != actual for size, actual in zip(shape, array.shape)):
                raise ValueError(f"bad {name} shape {array.shape}")
            if not np.isfinite(array).all():
                raise ValueError(f"nonfinite {name}")
            array = array.copy()
            array.setflags(write=False)
            object.__setattr__(self, name, array)
        if not (len(self.candidates) == len(self.actions) == len(self.physics)):
            raise ValueError("candidate/action/physics lengths differ")

    def candidate_features(self, physical=False):
        rows = self.candidates
        if physical:
            rows = np.concatenate([rows, self.physics], axis=1)
        return rows.copy()


@dataclass
class StepResult:
    observation: Observation | None
    reward: float
    terminated: bool
    truncated: bool
    info: dict

    def __post_init__(self):
        if self.terminated and self.truncated:
            raise ValueError("terminal task failure is not infrastructure truncation")
        if not np.isfinite(self.reward):
            raise ValueError("nonfinite reward")
        if not self.terminated and not self.truncated and self.observation is None:
            raise ValueError("continuing transition requires next observation")
