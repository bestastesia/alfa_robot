"""Explainable whole-group scoring for contact, carry and placement IK choices."""
from dataclasses import asdict, dataclass
import json
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class GroupScoreConfig:
    """All weights are equivalent seconds; bounded proxy terms are dimensionless."""

    motion_weight: float = 1.
    turn_penalty_s: float = .75
    reversal_penalty_s: float = 1.
    redundancy_penalty_s: float = .5
    collision_penalty_s: float = 2.
    joint_limit_margin_fraction: float = .05
    joint_limit_penalty_s: float = 1.
    score_temperature_s: float = 5.
    edge_collision_samples: int = 9
    updown_curvature_scale_m: float = .1
    swivel_curvature_scale_deg: float = 30.

    def __post_init__(self):
        values = asdict(self)
        if not all(np.isfinite(value) and value >= 0. for value in values.values()):
            raise ValueError("group score values must be finite and nonnegative")
        if not 0. < self.joint_limit_margin_fraction <= .5:
            raise ValueError("joint-limit margin fraction must be in (0, 0.5]")
        if self.score_temperature_s <= 0.:
            raise ValueError("score temperature must be positive")
        if isinstance(self.edge_collision_samples, bool) or int(self.edge_collision_samples) != self.edge_collision_samples:
            raise ValueError("edge collision samples must be an integer")
        if self.edge_collision_samples < 2:
            raise ValueError("edge collision samples must be at least two")
        if self.updown_curvature_scale_m <= 0. or self.swivel_curvature_scale_deg <= 0.:
            raise ValueError("redundancy curvature scales must be positive")

    @classmethod
    def load(cls, path):
        document = json.loads(Path(path).read_text())
        if document.get("schema") != "motion386-b2-group-score-v1" or document.get("units") != "seconds":
            raise ValueError("unsupported group-score config")
        return cls(**document["cost"])

    def to_dict(self):
        return asdict(self)


def _groups(points):
    values = np.asarray(points, dtype=np.float64)
    scalar = values.ndim == 2
    if scalar:
        values = values[None]
    if values.ndim != 3 or values.shape[1] < 3 or values.shape[2] != 15:
        raise ValueError("points must have shape (groups, points>=3, 15) or (points>=3, 15)")
    return values, scalar


def _limits(values, velocity, lower, upper):
    velocity = np.asarray(velocity, dtype=np.float64).reshape(15)
    lower = np.asarray(lower, dtype=np.float64).reshape(15)
    upper = np.asarray(upper, dtype=np.float64).reshape(15)
    if (not np.isfinite(velocity).all() or np.any(velocity <= 0.) or not np.isfinite(lower).all() or
            not np.isfinite(upper).all() or np.any(upper <= lower)):
        raise ValueError("velocity and joint limits must be finite, positive and ordered")
    return velocity, lower, upper, upper - lower


def _redundancy_curvature(redundancy, count, config):
    if redundancy is None:
        raise ValueError("group scoring requires Updown and left/right swivel values for all three selected IKs")
    values = np.asarray(redundancy, dtype=np.float64)
    if values.ndim == 2:
        values = values[None]
    if values.shape != (count, 3, 3) or not np.isfinite(values).all():
        raise ValueError("redundancy must have shape (groups, 3, 3): IK x [Updown m, left psi deg, right psi deg]")
    curvature = np.abs(values[:, 0] - 2. * values[:, 1] + values[:, 2])
    scales = np.array([config.updown_curvature_scale_m,
                       config.swivel_curvature_scale_deg,
                       config.swivel_curvature_scale_deg])
    return np.clip(curvature / scales, 0., 1.).mean(axis=1)


def group_score_terms(points, velocity, lower, upper, redundancy, edge_collision_fraction, config=None,
                      upstream_valid=None):
    """Return auditable, vectorized terms for complete candidate groups.

    ``points`` is normally Home/contact/extraction/carry/place/Home. Collision
    fractions must cover every adjacent edge. Hard validity remains separate from
    the soft score and is combined through ``upstream_valid``.
    """
    config = GroupScoreConfig() if config is None else config
    if not isinstance(config, GroupScoreConfig):
        raise TypeError("config must be GroupScoreConfig")
    values, scalar = _groups(points)
    count, point_count, _ = values.shape
    velocity, lower, upper, span = _limits(values, velocity, lower, upper)
    risks = np.asarray(edge_collision_fraction, dtype=np.float64)
    if risks.ndim == 1:
        risks = risks[None]
    if risks.shape != (count, point_count - 1) or not np.isfinite(risks).all():
        raise ValueError("edge collision fractions must match every adjacent group edge")
    if np.any((risks < 0.) | (risks > 1.)):
        raise ValueError("edge collision fractions must be in [0, 1]")

    delta = np.diff(values, axis=1)
    edge_time_s = np.max(np.abs(delta) / velocity[None, None], axis=2)
    denominator = velocity[None, None] * edge_time_s[:, :, None]
    usage = np.divide(delta, denominator, out=np.zeros_like(delta), where=denominator > 1e-12)
    turn_fraction = np.mean(np.abs(np.diff(usage, axis=1)) / 2., axis=(1, 2))

    before, after = delta[:, :-1], delta[:, 1:]
    reverses = before * after < 0.
    reversal_fraction = np.mean(np.where(reverses, np.minimum(np.abs(before), np.abs(after)) /
                                         span[None, None], 0.), axis=(1, 2))
    redundancy_curvature = _redundancy_curvature(redundancy, count, config)

    normalized_margin = np.minimum((values - lower[None, None]) / span[None, None],
                                   (upper[None, None] - values) / span[None, None])
    group_margin = normalized_margin.min(axis=(1, 2))
    limit_shortfall = np.clip((config.joint_limit_margin_fraction - group_margin) /
                              config.joint_limit_margin_fraction, 0., 1.)
    collision_fraction = risks.max(axis=1)

    terms = {
        "motion_s": config.motion_weight * edge_time_s.sum(axis=1),
        "turn_s": config.turn_penalty_s * turn_fraction,
        "reversal_s": config.reversal_penalty_s * reversal_fraction,
        "redundancy_s": config.redundancy_penalty_s * redundancy_curvature,
        "collision_s": config.collision_penalty_s * collision_fraction,
        "joint_limit_s": config.joint_limit_penalty_s * limit_shortfall,
    }
    total = np.sum(np.stack(list(terms.values())), axis=0)
    valid = np.isfinite(values).all(axis=(1, 2)) & (group_margin >= -1e-9)
    if upstream_valid is not None:
        upstream = np.asarray(upstream_valid, dtype=bool).reshape(count)
        valid &= upstream
    total = np.where(valid, total, np.inf)
    result = dict(terms=terms, total_s=total, valid=valid, edge_time_s=edge_time_s,
                  turn_fraction=turn_fraction, reversal_fraction=reversal_fraction,
                  redundancy_curvature=redundancy_curvature, collision_fraction=collision_fraction,
                  joint_limit_margin_fraction=group_margin)
    if scalar:
        return {name: ({key: float(item[0]) for key, item in value.items()} if isinstance(value, dict) else
                       bool(value[0]) if value.dtype == bool else
                       value[0].copy() if value.ndim > 1 else float(value[0]))
                for name, value in result.items()}
    return result


def score_groups(points, velocity, lower, upper, redundancy, edge_collision_fraction, config=None,
                 upstream_valid=None):
    """Add a 0-100 within-task score; minimum valid equivalent-second cost is 100."""
    config = GroupScoreConfig() if config is None else config
    result = group_score_terms(points, velocity, lower, upper, redundancy, edge_collision_fraction,
                               config, upstream_valid)
    total = np.atleast_1d(result["total_s"])
    valid = np.atleast_1d(result["valid"])
    score = np.zeros(len(total), dtype=np.float64)
    if valid.any():
        best = total[valid].min()
        score[valid] = 100. * np.exp(-(total[valid] - best) / config.score_temperature_s)
    result["score"] = float(score[0]) if np.ndim(result["total_s"]) == 0 else score
    return result
