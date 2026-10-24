"""MOTION-311 v2: stage-wise IK selection (contact -> carry -> place) with time-valued costs.

Costs are seconds. T_lb(a, b) = max_j |b_j - a_j| / v_j (URDF velocity limits) is the straight-line time bound of
a joint motion. Each stage network returns, per candidate, a success probability within the stage's budget and,
where trained, a cost correction; the composition below turns them into the value of a candidate:
  N3 (carry g, place p):   Q3 = (T_lb(g,p) + T_lb(p,Home)) * exp(r3) + (1 - P3) * C_fail ;  V3(g) = min_p Q3
  N2 (root r, carry g):    Q2 = T_lb(r,g) + (1 - P2) * C_fail + V3(g)
  N1 (contact c):          Q1 = T_lb(Home,c) + T_lb(c,r(c)) + (1 - P1) * C_fail + V2(c), V2 predicted by N1 directly
C_fail is the planning time one failed attempt costs (pre-registered 2 s). B0 is the same composition with
P = 1 and no correction (pure T_lb). Inference is numpy on the CPU.
"""
import numpy as np

import v3_candidate_ranker as ranker

C_FAIL_S = 2.0
FLAG_NAMES = ["dual", "top_grasp", "left_active", "right_active"]


def time_bound(first, second, velocity):
    return np.max(np.abs(np.asarray(second, dtype=np.float64) - np.asarray(first, dtype=np.float64)) / velocity, axis=-1)


def flags(context):
    return np.array([float(context["kind"] == "dual"), float(context["top_grasp"]),
                     float("left" in context["sides"]), float("right" in context["sides"])])


def place_features(carry, places, home, velocity, context):
    """(P, 66) features of place IKs `places` (P x 15) for one carry IK `carry` (15)."""
    carry, places, home = (np.asarray(value, dtype=np.float64) for value in (carry, places, home))
    places = np.atleast_2d(places)
    step = (places - carry) / velocity
    back = (home - places) / velocity
    return np.concatenate([np.repeat(carry[None], len(places), 0), places, step, back,
                           np.abs(step).max(1, keepdims=True), np.abs(back).max(1, keepdims=True),
                           np.repeat(flags(context)[None], len(places), 0)], axis=1)


PLACE_NAMES = ([f"carry_{name}" for name in ranker.ACTIVE_JOINTS] + [f"place_{name}" for name in ranker.ACTIVE_JOINTS]
               + [f"step_{name}" for name in ranker.ACTIVE_JOINTS] + [f"back_{name}" for name in ranker.ACTIVE_JOINTS]
               + ["t_lb_place", "t_lb_return"] + FLAG_NAMES)


def transport_features(root, carries, velocity, context):
    """(G, 56) features of carry IKs `carries` (G x 15) for one extraction end `root` (15)."""
    root, carries = np.asarray(root, dtype=np.float64), np.atleast_2d(np.asarray(carries, dtype=np.float64))
    step = (carries - root) / velocity
    boxes = np.concatenate([np.asarray(context["box_centers"].get(side, (0., 0., 0.)), dtype=np.float64) for side in ("left", "right")])
    return np.concatenate([np.repeat(root[None], len(carries), 0), carries, step, np.abs(step).max(1, keepdims=True),
                           np.repeat(boxes[None], len(carries), 0), np.repeat(flags(context)[None], len(carries), 0)], axis=1)


TRANSPORT_NAMES = ([f"root_{name}" for name in ranker.ACTIVE_JOINTS] + [f"carry_{name}" for name in ranker.ACTIVE_JOINTS]
                   + [f"step_{name}" for name in ranker.ACTIVE_JOINTS] + ["t_lb_transport"]
                   + [f"box_{side}_{axis}" for side in ("left", "right") for axis in "xyz"] + FLAG_NAMES)


def contact_features(contacts, context, kinematics, velocity):
    """(N, 73) = S1 T0 + T1 features + T_lb(Home, c) and T_lb(c, r(c)); also returns the endpoints and eligibility."""
    contacts = np.asarray(contacts, dtype=np.float64)
    ends = ranker.endpoints(contacts, context, kinematics)
    extra, eligible = ranker.s1_t1(contacts, context, kinematics, ends)
    filled = np.where(np.isfinite(ends), ends, contacts)
    home = np.asarray(context["home"], dtype=np.float64)
    times = np.column_stack([time_bound(home, contacts, velocity), time_bound(contacts, filled, velocity)])
    return np.concatenate([ranker.s1_t0(contacts, context, kinematics), extra, times], axis=1), filled, eligible


CONTACT_NAMES = ranker.S1_T0_NAMES + ranker.S1_T1_NAMES + ["t_lb_approach", "t_lb_extract"]


def place_values(carry, places, home, velocity, context, model=None):
    """Q3 for every place IK of one carry IK; model=None is B0 (P = 1, no correction)."""
    bound = time_bound(carry, places, velocity) + time_bound(places, home, velocity)
    if model is None:
        return bound, np.ones(len(bound))
    probability, ratio, _ = model.predict(place_features(carry, places, home, velocity, context))
    return bound * ratio + (1. - probability) * C_FAIL_S, probability


def carry_values(root, carries, downstream, velocity, context, model=None):
    """Q2 for every carry IK of one extraction end, given their downstream values V3; model=None is B0."""
    bound = time_bound(root, carries, velocity) + np.asarray(downstream, dtype=np.float64)
    if model is None:
        return bound, np.ones(len(bound))
    probability, _, _ = model.predict(transport_features(root, carries, velocity, context))
    return bound + (1. - probability) * C_FAIL_S, probability


def ranked(values, probability, theta=.5):
    """Candidate indices by ascending value among those with P >= theta (empty: abstain)."""
    keep = np.flatnonzero(np.asarray(probability) >= theta)
    return [int(index) for index in keep[np.argsort(np.asarray(values)[keep], kind="stable")]]
