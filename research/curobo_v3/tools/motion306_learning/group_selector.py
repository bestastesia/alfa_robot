"""Hierarchical beam selection for one contact/carry/placement IK group."""
from dataclasses import dataclass, replace
import json
from pathlib import Path

import numpy as np

from motion306_learning.group_score import GroupScoreConfig, score_groups


@dataclass(frozen=True)
class GroupBeamConfig:
    contact_beam: int = 4
    pair_beam: int = 16
    place_cap: int = 64
    full_group_limit: int = 1024
    selection_rule: str = "weighted_total"
    primary_cost_slack_s: float = 0.

    def __post_init__(self):
        for name in ("contact_beam", "pair_beam", "place_cap", "full_group_limit"):
            value = getattr(self, name)
            if isinstance(value, bool) or int(value) != value or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if self.selection_rule not in ("weighted_total", "near_optimal_smooth"):
            raise ValueError("unsupported group selection rule")
        if not np.isfinite(self.primary_cost_slack_s) or self.primary_cost_slack_s < 0.:
            raise ValueError("primary cost slack must be finite and nonnegative")
        if self.selection_rule == "weighted_total" and self.primary_cost_slack_s != 0.:
            raise ValueError("weighted-total selection does not use primary cost slack")
        if self.pair_beam * self.place_cap > self.full_group_limit:
            raise ValueError("pair beam times place cap exceeds the full-group limit")

    @classmethod
    def load(cls, path):
        document = json.loads(Path(path).read_text())
        if document.get("schema") != "motion386-b2-group-score-v1":
            raise ValueError("unsupported group-selector config")
        contract = document["selection_contract"]
        return cls(**{name: contract[name] for name in cls.__dataclass_fields__})


@dataclass(frozen=True)
class CandidatePool:
    rows: np.ndarray
    redundancy: np.ndarray
    edge_collision_fraction: np.ndarray
    indices: np.ndarray
    valid: np.ndarray

    def __post_init__(self):
        rows = np.asarray(self.rows, dtype=np.float64).reshape(-1, 15)
        count = len(rows)
        redundancy = np.asarray(self.redundancy, dtype=np.float64).reshape(count, 3)
        risk = np.asarray(self.edge_collision_fraction, dtype=np.float64).reshape(count)
        indices = np.asarray(self.indices, dtype=int).reshape(count)
        valid = np.asarray(self.valid, dtype=bool).reshape(count)
        if not np.isfinite(rows).all() or not np.isfinite(redundancy).all() or not np.isfinite(risk).all():
            raise ValueError("candidate pools must be finite")
        if np.any((risk < 0.) | (risk > 1.)):
            raise ValueError("edge collision fractions must be in [0, 1]")
        if len(set(indices.tolist())) != count:
            raise ValueError("candidate indices must be unique within a pool")
        for name, value in (("rows", rows), ("redundancy", redundancy),
                            ("edge_collision_fraction", risk), ("indices", indices), ("valid", valid)):
            value = value.copy()
            value.setflags(write=False)
            object.__setattr__(self, name, value)


@dataclass(frozen=True)
class ContactPool(CandidatePool):
    extraction: np.ndarray

    def __post_init__(self):
        super().__post_init__()
        extraction = np.asarray(self.extraction, dtype=np.float64).reshape(len(self.rows), 15).copy()
        extraction.setflags(write=False)
        object.__setattr__(self, "extraction", extraction)


@dataclass(frozen=True)
class PlacePool(CandidatePool):
    home_collision_fraction: np.ndarray

    def __post_init__(self):
        super().__post_init__()
        risk = np.asarray(self.home_collision_fraction, dtype=np.float64).reshape(len(self.rows))
        if not np.isfinite(risk).all() or np.any((risk < 0.) | (risk > 1.)):
            raise ValueError("Home collision fractions must be finite and in [0, 1]")
        risk = risk.copy()
        risk.setflags(write=False)
        object.__setattr__(self, "home_collision_fraction", risk)


@dataclass(frozen=True)
class GroupChoice:
    contact_index: int
    carry_index: int
    place_index: int
    rows: np.ndarray
    redundancy: np.ndarray
    total_s: float
    score: float
    terms_s: dict
    audit: dict


def _partial_scores(points, risks, velocity, lower, upper, score_config):
    """Score an incomplete chain without the three-IK redundancy-curvature term."""
    config = replace(score_config, redundancy_penalty_s=0.)
    dummy = np.zeros((len(points), 3, 3), dtype=np.float64)
    return score_groups(points, velocity, lower, upper, dummy, risks, config)["total_s"]


def _select_group(backend, score_config=None, beam_config=None):
    """Select one deterministic group from a backend that exposes hierarchical pools.

    Backend contract:
      - ``home``, ``velocity``, ``lower``, ``upper`` arrays;
      - ``contact_pool() -> ContactPool``;
      - ``carry_pool(contact_index, extraction) -> CandidatePool``;
      - ``place_pool(contact_index, carry_index, carry_row) -> PlacePool``.
    """
    score_config = GroupScoreConfig() if score_config is None else score_config
    beam_config = GroupBeamConfig() if beam_config is None else beam_config
    home = np.asarray(backend.home, dtype=np.float64).reshape(15)
    velocity = np.asarray(backend.velocity, dtype=np.float64).reshape(15)
    lower = np.asarray(backend.lower, dtype=np.float64).reshape(15)
    upper = np.asarray(backend.upper, dtype=np.float64).reshape(15)

    contact = backend.contact_pool()
    contact_valid = np.flatnonzero(contact.valid)
    if not len(contact_valid):
        return None
    contact_points = np.stack([np.repeat(home[None], len(contact_valid), axis=0),
                               contact.rows[contact_valid], contact.extraction[contact_valid]], axis=1)
    contact_risks = np.stack([contact.edge_collision_fraction[contact_valid],
                              np.zeros(len(contact_valid))], axis=1)
    contact_cost = _partial_scores(contact_points, contact_risks, velocity, lower, upper, score_config)
    contact_order = contact_valid[np.argsort(contact_cost, kind="stable")]
    chosen_contacts = []
    for contact_slot in contact_order:
        extraction = contact.extraction[contact_slot]
        if hasattr(backend, "validate_contact"):
            extraction = backend.validate_contact(int(contact.indices[contact_slot]),
                                                  contact.rows[contact_slot], extraction)
        if extraction is not None:
            chosen_contacts.append((int(contact_slot), np.asarray(extraction, dtype=np.float64).reshape(15)))
        if len(chosen_contacts) >= beam_config.contact_beam:
            break
    if not chosen_contacts:
        return None

    pairs = []
    for contact_slot, extraction in chosen_contacts:
        contact_index = int(contact.indices[contact_slot])
        carry = backend.carry_pool(contact_index, extraction)
        carry_valid = np.flatnonzero(carry.valid)
        if not len(carry_valid):
            continue
        count = len(carry_valid)
        points = np.stack([np.repeat(home[None], count, axis=0),
                           np.repeat(contact.rows[contact_slot][None], count, axis=0),
                           np.repeat(extraction[None], count, axis=0),
                           carry.rows[carry_valid]], axis=1)
        risks = np.stack([np.repeat(contact.edge_collision_fraction[contact_slot], count),
                          np.zeros(count), carry.edge_collision_fraction[carry_valid]], axis=1)
        costs = _partial_scores(points, risks, velocity, lower, upper, score_config)
        for local, cost in zip(carry_valid, costs):
            pairs.append((float(cost), int(contact_slot), extraction, carry, int(local)))
    if not pairs:
        return None
    pair_candidate_count = len(pairs)
    pairs.sort(key=lambda item: item[0])  # Python sort is stable; pool order is the deterministic tie-break.
    pairs = pairs[:beam_config.pair_beam]

    groups, redundant, risks, identities = [], [], [], []
    for partial_cost, contact_slot, extraction, carry, carry_slot in pairs:
        contact_index = int(contact.indices[contact_slot])
        carry_index = int(carry.indices[carry_slot])
        place = backend.place_pool(contact_index, carry_index, carry.rows[carry_slot])
        place_valid = np.flatnonzero(place.valid)[:beam_config.place_cap]
        for place_slot in place_valid:
            groups.append(np.stack([home, contact.rows[contact_slot], extraction,
                                    carry.rows[carry_slot], place.rows[place_slot], home]))
            redundant.append(np.stack([contact.redundancy[contact_slot], carry.redundancy[carry_slot],
                                       place.redundancy[place_slot]]))
            risks.append([contact.edge_collision_fraction[contact_slot], 0.,
                          carry.edge_collision_fraction[carry_slot], place.edge_collision_fraction[place_slot],
                          place.home_collision_fraction[place_slot]])
            identities.append((contact_index, carry_index, int(place.indices[place_slot]), partial_cost))
    if not groups:
        return None
    if len(groups) > beam_config.full_group_limit:
        raise RuntimeError("group construction exceeded configured limit")
    scored = score_groups(np.asarray(groups), velocity, lower, upper, np.asarray(redundant), np.asarray(risks), score_config)
    weighted_order = np.argsort(scored["total_s"], kind="stable")
    valid_indices = np.flatnonzero(scored["valid"])
    if beam_config.selection_rule == "weighted_total":
        order = weighted_order
    else:
        primary = (scored["terms"]["motion_s"] + scored["terms"]["collision_s"] +
                   scored["terms"]["joint_limit_s"])
        smoothness = (scored["terms"]["turn_s"] + scored["terms"]["reversal_s"] +
                      scored["terms"]["redundancy_s"])
        best_primary = primary[valid_indices].min() if len(valid_indices) else np.inf
        eligible = valid_indices[primary[valid_indices] <= best_primary + beam_config.primary_cost_slack_s + 1e-12]
        order = np.asarray(sorted(eligible.tolist(), key=lambda index: (smoothness[index], primary[index], index)), dtype=int)
    selected = next((int(index) for index in order if scored["valid"][index]), None)
    if selected is None:
        return None
    contact_index, carry_index, place_index, partial_cost = identities[selected]
    terms = {name: float(values[selected]) for name, values in scored["terms"].items()}
    audit = {
        "contact_pool": len(contact.rows), "contact_valid": int(contact.valid.sum()),
        "contact_beam": len(chosen_contacts), "pair_candidates": pair_candidate_count,
        "pair_beam": len(pairs), "full_groups": len(groups),
        "selection_rule": beam_config.selection_rule,
        "primary_cost_slack_s": beam_config.primary_cost_slack_s,
        "selected_rank": int(np.flatnonzero(order == selected)[0]),
        "selected_weighted_rank": int(np.flatnonzero(weighted_order == selected)[0]),
        "selected_partial_cost_s": partial_cost,
        "selected_turn_fraction": float(scored["turn_fraction"][selected]),
        "selected_reversal_fraction": float(scored["reversal_fraction"][selected]),
        "selected_redundancy_curvature": float(scored["redundancy_curvature"][selected]),
        "selected_collision_fraction": float(scored["collision_fraction"][selected]),
        "selected_joint_limit_margin_fraction": float(scored["joint_limit_margin_fraction"][selected]),
    }
    if hasattr(backend, "audit"):
        audit["backend"] = backend.audit()
    return GroupChoice(contact_index, carry_index, place_index, np.asarray(groups[selected]).copy(),
                       np.asarray(redundant[selected]).copy(), float(scored["total_s"][selected]),
                       float(scored["score"][selected]), terms, audit)


def select_group(backend, score_config=None, beam_config=None):
    """Select one group and always restore any mutable preview backend state."""
    choice = None
    try:
        choice = _select_group(backend, score_config, beam_config)
    finally:
        if hasattr(backend, "finalize"):
            backend.finalize()
    if choice is not None and hasattr(backend, "audit"):
        choice.audit["backend"] = backend.audit()
    return choice
