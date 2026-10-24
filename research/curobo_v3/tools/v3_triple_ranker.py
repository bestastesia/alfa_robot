"""MOTION-311 v2: stage-wise IK selection (contact -> carry -> place) with time-valued costs.

Costs are seconds. T_lb(a, b) = max_j |b_j - a_j| / v_j (URDF velocity limits) is the straight-line time bound of
a joint motion. Each stage network returns, per candidate, a success probability within the stage's budget and,
where trained, a cost correction; the composition below turns them into the value of a candidate:
  N3 (carry g, place p):   Q3 = (T_lb(g,p) + T_lb(p,Home)) * exp(r3) + (1 - P3) * C_fail ;  V3(g) = min_p Q3
  N2 (root r, carry g):    Q2 = T_lb(r,g) + (1 - P2) * C_fail + V3(g)
  N1 (contact c):          Q1 = T_lb(Home,c) + T_lb(c,r(c)) + (1 - P1) * C_fail + V2(c), V2 predicted by N1 directly
C_fail is the planning time one failed attempt costs (pre-registered 2 s). B0 is the same composition with
P = 1 and no correction (pure T_lb). The explicit V2(c) = min over the baseline's goal pool of Q2 (contact_downstream)
is B0's contact value, the ablation of the N1 prediction and N1's distillation target. The V3 used by the carry and
contact stages is the analytic one (model=None) in every arm (A10: N3 extrapolates beyond its D3 carries); N3's Q3
ranks the places of a realized carry. Inference is numpy on the CPU.
"""
import numpy as np

import v3_candidate_ranker as ranker

C_FAIL_S = 2.0
Z_LIMIT = 6.  # a candidate with a feature beyond 6 training standard deviations is never recommended (P := 0)
PLACE_KEEP = 64  # V3 and the place choice consider the places of lowest T_lb(g,p) + T_lb(p,Home) only
FLAG_NAMES = ["dual", "top_grasp", "left_active", "right_active"]


def time_bound(first, second, velocity):
    return np.max(np.abs(np.asarray(second, dtype=np.float64) - np.asarray(first, dtype=np.float64)) / velocity, axis=-1)


def flags(context):
    return np.array([float(context["kind"] == "dual"), float(context["top_grasp"]),
                     float("left" in context["sides"]), float("right" in context["sides"])])


def place_features(carry, places, home, velocity, context):
    """(P, 66) features of place IKs `places` (P x 15) for one carry IK `carry` (15) or row-wise carries (P x 15)."""
    carry, places, home = (np.asarray(value, dtype=np.float64) for value in (carry, places, home))
    places = np.atleast_2d(places)
    carry = np.broadcast_to(carry, places.shape)
    step = (places - carry) / velocity
    back = (home - places) / velocity
    return np.concatenate([carry, places, step, back,
                           np.abs(step).max(1, keepdims=True), np.abs(back).max(1, keepdims=True),
                           np.repeat(flags(context)[None], len(places), 0)], axis=1)


PLACE_NAMES = ([f"carry_{name}" for name in ranker.ACTIVE_JOINTS] + [f"place_{name}" for name in ranker.ACTIVE_JOINTS]
               + [f"step_{name}" for name in ranker.ACTIVE_JOINTS] + [f"back_{name}" for name in ranker.ACTIVE_JOINTS]
               + ["t_lb_place", "t_lb_return"] + FLAG_NAMES)


def transport_features(root, carries, velocity, context):
    """(G, 56) features of carry IKs `carries` (G x 15) for one extraction end `root` (15) or row-wise roots (G x 15)."""
    carries = np.atleast_2d(np.asarray(carries, dtype=np.float64))
    root = np.broadcast_to(np.asarray(root, dtype=np.float64), carries.shape)
    step = (carries - root) / velocity
    boxes = np.concatenate([np.asarray(context["box_centers"].get(side, (0., 0., 0.)), dtype=np.float64) for side in ("left", "right")])
    return np.concatenate([root, carries, step, np.abs(step).max(1, keepdims=True),
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
    probability, ratio, max_z = model.predict(place_features(carry, places, home, velocity, context))
    probability = np.where(max_z > Z_LIMIT, 0., probability)
    return bound * ratio + (1. - probability) * C_FAIL_S, probability


def carry_values(root, carries, downstream, velocity, context, model=None):
    """Q2 for every carry IK of one extraction end, given their downstream values V3; model=None is B0."""
    bound = time_bound(root, carries, velocity) + np.asarray(downstream, dtype=np.float64)
    if model is None:
        return bound, np.ones(len(bound))
    probability, _, max_z = model.predict(transport_features(root, carries, velocity, context))
    probability = np.where(max_z > Z_LIMIT, 0., probability)
    return bound + (1. - probability) * C_FAIL_S, probability


def downstream_values(carries, places, home, velocity, context, model=None, keep=PLACE_KEEP, chunk=256):
    """V3(g) = min_p Q3(g, p) over the `keep` places of lowest T_lb(g,p) + T_lb(p,Home), and the argmin place index."""
    carries = np.atleast_2d(np.asarray(carries, dtype=np.float64))
    places = np.asarray(places, dtype=np.float64)
    back = time_bound(places, home, velocity)
    keep = min(keep, len(places))
    values, choice = np.empty(len(carries)), np.empty(len(carries), dtype=int)
    for offset in range(0, len(carries), chunk):
        block = carries[offset:offset + chunk]
        bound = np.max(np.abs(places[None] - block[:, None]) / velocity, axis=-1) + back[None]
        near = np.argpartition(bound, keep - 1, axis=1)[:, :keep] if keep < len(places) else np.broadcast_to(np.arange(len(places)), bound.shape)
        pairs = np.repeat(block, keep, axis=0)
        q, _ = place_values(pairs, places[near.reshape(-1)], home, velocity, context, model)
        q = q.reshape(len(block), keep)
        best = np.argmin(q, axis=1)
        values[offset:offset + len(block)] = q[np.arange(len(block)), best]
        choice[offset:offset + len(block)] = near[np.arange(len(block)), best]
    return values, choice


def goal_pool(ends, rows, pool=512):
    """(N, pool) indices of the table rows nearest to each end in the baseline's weighted joint distance."""
    scaled = np.asarray(rows, dtype=np.float64) * ranker.WEIGHTS
    distance = (scaled ** 2).sum(1)[None] - 2. * (np.atleast_2d(ends) * ranker.WEIGHTS) @ scaled.T
    if pool >= len(scaled):
        return np.broadcast_to(np.arange(len(scaled)), distance.shape)
    return np.argpartition(distance, pool - 1, axis=1)[:, :pool]


def contact_downstream(ends, rows, downstream, velocity, context, model=None, pool=512, keep=32, near=None):
    """V2(c) = min_g Q2(r(c), g) over the `pool` table rows nearest to each extraction end r(c) (the baseline's goal
    pool; `near` may hold it precomputed for the finite ends, e.g. on the GPU); with a model the N2 risk is added on
    the `keep` best of them. NaN ends: inf."""
    ends, rows = np.atleast_2d(np.asarray(ends, dtype=np.float64)), np.asarray(rows, dtype=np.float64)
    downstream = np.asarray(downstream, dtype=np.float64)
    output = np.full(len(ends), np.inf)
    valid = np.flatnonzero(np.isfinite(ends).all(axis=1))
    if not len(valid):
        return output
    near = goal_pool(ends[valid], rows, pool) if near is None else np.asarray(near)
    values = np.max(np.abs(rows[near] - ends[valid][:, None]) / velocity, axis=-1) + downstream[near]
    if model is None:
        output[valid] = values.min(axis=1)
        return output
    keep = min(keep, near.shape[1])
    best = np.take_along_axis(near, np.argsort(values, axis=1, kind="stable")[:, :keep], axis=1)
    q, _ = carry_values(np.repeat(ends[valid], keep, axis=0), rows[best.reshape(-1)], downstream[best.reshape(-1)], velocity, context, model)
    output[valid] = q.reshape(len(valid), keep).min(axis=1)
    return output


MULTIPLIERS = np.random.default_rng(311).integers(1, 2 ** 63, 15, dtype=np.uint64) | np.uint64(1)


def row_hashes(rows):
    return (np.ascontiguousarray(rows, dtype=np.float32).view(np.uint32).astype(np.uint64) * MULTIPLIERS).sum(1)


class RowTable:
    """Exact float32-row lookup of a V3 table (sorted 64-bit row hashes, verified by row equality)."""

    def __init__(self, rows, values):
        self.rows, self.values = np.ascontiguousarray(rows, dtype=np.float32), np.asarray(values, dtype=np.float64)
        hashes = row_hashes(self.rows)
        self.order = np.argsort(hashes, kind="stable")
        self.sorted = hashes[self.order]

    def lookup(self, rows):
        rows = np.ascontiguousarray(rows, dtype=np.float32)
        position = np.minimum(np.searchsorted(self.sorted, row_hashes(rows)), len(self.sorted) - 1)
        index = self.order[position]
        return np.where(np.all(self.rows[index] == rows, axis=1), index, -1)


def ranked(values, probability, theta=.5):
    """Candidate indices by ascending value among those with P >= theta (empty: abstain)."""
    keep = np.flatnonzero(np.asarray(probability) >= theta)
    return [int(index) for index in keep[np.argsort(np.asarray(values)[keep], kind="stable")]]
