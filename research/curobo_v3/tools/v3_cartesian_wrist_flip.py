"""Controlled wrist flip for the Cartesian transport search.

When the nearest goal (in the baseline's goal order) lies on the wrist-flipped branch (q2, q4 as at the
start, q6 opposite), the branch-fixed search can only reach a same-branch goal by a long detour, because
q5/q7 would have to pass their +-180 deg limits. Instead, the transport keeps a straight tool path to a
flipped-branch goal and switches the wrist branch once, at a point X* of that path where q6 = 0:
the arm reaches X* on the start branch, the box stops while q5 and q7 counter-rotate (at q6 = 0 the
tool pose depends only on q5 + q7, so this is a self-motion), and the arm continues on the flipped
branch. X* is searched over progress s, swivel psi and lift u, which are null-space (the tool path
does not change). q6 = 0 is a curve in (s, psi, u); Gauss-Newton steps from points of the straight
start -> goal line land on it near the line, in the coordinates q6 (cos q5, sin q5), which are smooth
through the singularity and equal on both branches. The points are tried in the order of an estimated
execution time (velocity-limited joint travel plus the rest-to-rest swing), because the slow shoulder
and elbow joints, not the search, decide how long the flip takes. A small position offset of X* is
tried only after every candidate goal failed on the exact line. The search stops at its deadline, so
a failed attempt leaves the rest of the stage budget to the joint RRT. The path is checked like a
final transport path, and the swing indices are returned so the timing can stop the box there.
"""
import math
import time

import numpy as np
from scipy.optimize import minimize

from v3_batched_loaded_search import densify
from v3_cartesian_math import branch, refine, wrap

DEFAULTS = dict(wrist_flip_goals=3, wrist_flip_offset_m=.05, wrist_flip_swing_tolerance_deg=2., wrist_flip_budget_s=.6,
                wrist_flip_paths=3)
S_GRID = np.linspace(.04, .96, 16)
PSI_GRID = np.radians(np.arange(-180., 180., 16.))
U_OFFSETS = np.array([0., -.075, .075, -.15, .15])  # smallest lift change first
EARLY_DEG = 1.  # stop the grid once a cell this close to q6 = 0 is found
FINE_PARTS = 12  # extra subdivisions of the last step before X* on each side
NEWTON_ROUNDS = 12
NEWTON_STEP = np.array([.1, .35, .05])  # per-round caps on (s, psi, du)
NEWTON_SCALE = np.array([1., 1., .3])  # minimal-norm metric; a lift change moves the elbow most
NEWTON_PROBE = 1e-5
COST_POINTS = 12  # samples per piece for the execution-time estimate
ACCEPT_DEG = .005


class _Deadline(Exception):
    pass


def flipped(arm_branch):
    return arm_branch[0], arm_branch[1], -arm_branch[2]


def wrist_only_change(start_arm, goal_arm):
    return branch(goal_arm) == flipped(branch(start_arm))


def rest_to_rest_s(distance, velocity, acceleration):
    """Duration of a rest-to-rest move under velocity and acceleration limits."""
    distance = np.abs(distance)
    return np.where(distance >= velocity ** 2 / acceleration, distance / velocity + velocity / acceleration,
                    2. * np.sqrt(distance / acceleration))


class WristFlip:
    """X* search and path construction for one flipped-branch goal (wraps a CartesianTransportSpace)."""

    def __init__(self, space, bounds, options):
        self.space, self.bounds, self.options = space, bounds, options
        self.branch_a = space.arm_branch
        self.branch_b = flipped(self.branch_a)
        self.seeds = None  # grid minima of |q6|, for the offset search and as Newton starts off the line
        planner = space.planner
        names = list(planner._timed_joint_names)
        index = [names.index(f"{planner.active_side}_joint{i}") for i in range(1, 8)]
        self.velocity, self.acceleration = (np.asarray(limits)[index] for limits in planner._timed_limits[:2])

    def state(self, x):
        """x = (s, psi, du[, dx, dy, dz]) -> state on the straight start -> goal line, offset if given."""
        space = self.space
        state = space.start_state + np.clip(x[0], 0., 1.) * (space.goal_states[0] - space.start_state)
        if len(x) > 3:
            state[:3] += x[3:6]
        state[4] = np.clip(state[4] + x[2], *self.bounds)
        state[5] = x[1]
        return state

    def states(self, xs):
        """Vectorized `state` for x = (s, psi, du) rows on the exact line."""
        xs = np.asarray(xs, dtype=np.float64).reshape(-1, 3)
        space = self.space
        states = space.start_state + np.clip(xs[:, :1], 0., 1.) * (space.goal_states[0] - space.start_state)
        states[:, 4] = np.clip(states[:, 4] + xs[:, 2], *self.bounds)
        states[:, 5] = xs[:, 1]
        return states

    def joints(self, states, arm_branch):
        space = self.space
        saved, space.arm_branch = space.arm_branch, arm_branch
        try:
            return space.joints(np.asarray(states, dtype=np.float64).reshape(-1, 6))
        finally:
            space.arm_branch = saved

    def q6_start_branch(self, states):
        """|q6| in degrees on the start branch (inf where IK fails); the flipped branch has the same |q6|."""
        rows, found = self.joints(states, self.branch_a)
        value = np.abs(np.degrees(rows[:, self.space.arm][:, 5]))
        value[~found] = np.inf
        return value

    def q6(self, states):
        values = []
        for arm_branch in (self.branch_a, self.branch_b):
            rows, found = self.joints(states, arm_branch)
            value = np.abs(np.degrees(rows[:, self.space.arm][:, 5]))
            value[~found] = np.inf
            values.append(value)
        return np.minimum(*values)

    def wrist(self, states):
        """q6 (cos q5, sin q5) per state, nan where neither branch has an IK solution."""
        values = np.full((len(states), 2), np.nan)
        for arm_branch in (self.branch_b, self.branch_a):  # the start branch wins where both solve
            rows, found = self.joints(states, arm_branch)
            q5, q6 = rows[found][:, self.space.arm][:, 4], rows[found][:, self.space.arm][:, 5]
            values[found] = q6[:, None] * np.stack([np.cos(q5), np.sin(q5)], axis=1)
        return values

    def crossings(self, deadline):
        """Exact-line X* candidates [(estimated s, x)] with |q6| <= ACCEPT_DEG, cheapest first."""
        line = np.stack([S_GRID, S_GRID * self.space.goal_states[0, 5], np.zeros_like(S_GRID)], axis=1)
        xs = self._newton(line, deadline)
        if not len(xs) and self._seeds(deadline):
            xs = self._newton(np.array([seed[1:] for seed in self.seeds]), deadline)
        if not len(xs):
            return []
        xs = xs[np.unique(np.round(xs / [.01, .02, .005]), axis=0, return_index=True)[1]]
        costs = self.cost(xs)
        order = [i for i in np.argsort(costs) if np.isfinite(costs[i])]
        return [(float(costs[i]), xs[i]) for i in order]

    def _newton(self, xs, deadline):
        """Gauss-Newton with minimal-norm steps from each x = (s, psi, du) onto the q6 = 0 curve; converged rows."""
        xs = np.asarray(xs, dtype=np.float64).reshape(-1, 3)
        probes = np.vstack([np.zeros(3), np.eye(3) * NEWTON_PROBE])
        for _ in range(NEWTON_ROUNDS):
            if not len(xs) or time.perf_counter() > deadline:
                break
            values = self.wrist(self.states((xs[None] + probes[:, None]).reshape(-1, 3))).reshape(4, len(xs), 2)
            keep = np.isfinite(values).all(axis=(0, 2))
            xs, values = xs[keep], values[:, keep]
            jacobian = (values[1:] - values[0]).transpose(1, 2, 0) / NEWTON_PROBE * NEWTON_SCALE
            step = -np.einsum("nij,nj->ni", np.linalg.pinv(jacobian), values[0]) * NEWTON_SCALE
            xs = xs + np.clip(step, -NEWTON_STEP, NEWTON_STEP)
        # At q6 = 0 exactly the IK may solve only one branch; the path itself never solves at X*.
        return xs[self.q6(self.states(xs)) <= ACCEPT_DEG] if len(xs) else xs

    def cost(self, xs):
        """Estimated execution time of the path through each X*: velocity-limited joint travel of both pieces plus
        the rest-to-rest swing; inf without IK or with an impure swing."""
        space, count = self.space, len(xs)
        x_star = self.states(xs)
        x_a, x_b = x_star.copy(), x_star.copy()
        x_a[:, 5] = space.start_state[5] + wrap(x_star[:, 5] - space.start_state[5])
        x_b[:, 5] = space.goal_states[0, 5] + wrap(x_star[:, 5] - space.goal_states[0, 5])
        fractions = np.r_[np.linspace(0., 1., COST_POINTS)[:-1], 1. - 1e-4][None, :, None]  # the last just before X*
        travel, found, ends = 0., np.ones(count, dtype=bool), []
        for origin, end, arm_branch in ((space.start_state, x_a, self.branch_a), (space.goal_states[0], x_b, self.branch_b)):
            rows, ok = self.joints((origin + fractions * (end - origin)[:, None]).reshape(-1, 6), arm_branch)
            arms = rows[:, space.arm].reshape(count, COST_POINTS, 7)
            travel = travel + (np.abs(np.diff(arms, axis=1)) / self.velocity).max(axis=2).sum(axis=1)
            found &= ok.reshape(count, COST_POINTS).all(axis=1)
            ends.append(arms[:, -1])
        swing = ends[1] - ends[0]
        tolerance = math.radians(self.options["wrist_flip_swing_tolerance_deg"])
        pure = (np.abs(np.delete(swing, [4, 6], axis=1)).max(axis=1) <= tolerance) & (np.abs(swing[:, 4] + swing[:, 6]) <= tolerance)
        cost = travel + rest_to_rest_s(swing[:, 4], self.velocity[4], self.acceleration[4])
        return np.where(found & pure, cost, np.inf)

    def _seeds(self, deadline):
        """Grid minima of |q6| on the start branch per lift offset (the smallest offsets first), cached."""
        if self.seeds is None:
            seeds = []
            for du in U_OFFSETS:
                values = self.q6_start_branch(self.states([(s, psi, du) for s in S_GRID for psi in PSI_GRID]))
                values = values.reshape(len(S_GRID), len(PSI_GRID))
                i, j = np.unravel_index(np.argmin(values), values.shape)
                if np.isfinite(values[i, j]):
                    seeds.append((values[i, j], S_GRID[i], PSI_GRID[j], du))
                if time.perf_counter() > deadline:
                    return []
                if seeds and min(seeds)[0] < EARLY_DEG:
                    break
            self.seeds = sorted(seeds)[:3]
        return self.seeds

    def offset_crossing(self, deadline):
        """(|q6| deg at X*, x) of the best q6 = 0 point with a position offset of X*, or None."""
        best = None
        limit = self.options["wrist_flip_offset_m"]
        for _, s, psi, du in self._seeds(deadline):
            def objective(x):
                if time.perf_counter() > deadline:
                    raise _Deadline
                if np.linalg.norm(x[3:6]) > limit:
                    return 90. + float(np.linalg.norm(x[3:6]))
                return float(self.q6(self.state(x)[None])[0])

            try:
                result = minimize(objective, [s, psi, du, 0., 0., 0.], method="Nelder-Mead",
                                  options=dict(xatol=1e-8, fatol=1e-7, maxiter=1500))
            except _Deadline:
                break
            if best is None or result.fun < best[0]:
                best = (float(result.fun), np.asarray(result.x))
        return best

    def path(self, x, goal_row):
        """Dense joint path start -> X* (start branch) -> swing -> goal (flipped branch), or (None, reason)."""
        space, resolution = self.space, self.options["path_resolution"]
        x_star = self.state(x)
        x_a, x_b = x_star.copy(), x_star.copy()  # psi is periodic: write X* closest to each segment's other end
        x_a[5] = space.start_state[5] + wrap(x_star[5] - space.start_state[5])
        x_b[5] = space.goal_states[0, 5] + wrap(x_star[5] - space.goal_states[0, 5])
        first = _fine_end(np.stack([space.start_state, x_a]), space.weights_np, resolution)[:-1]
        second = _fine_end(np.stack([space.goal_states[0], x_b]), space.weights_np, resolution)[:-1][::-1]
        rows_a, found_a = self.joints(first, self.branch_a)
        rows_b, found_b = self.joints(second, self.branch_b)
        if not (found_a.all() and found_b.all()):
            return None, "ik"
        arm = space.arm
        if max(np.abs(np.diff(rows_a[:, arm], axis=0)).max(initial=0.),
               np.abs(np.diff(rows_b[:, arm], axis=0)).max(initial=0.)) > space.max_jump:
            return None, "continuity"
        swing = rows_b[0, arm] - rows_a[-1, arm]
        tolerance = math.radians(self.options["wrist_flip_swing_tolerance_deg"])
        if np.abs(np.delete(swing, [4, 6])).max() > tolerance or abs(swing[4] + swing[6]) > tolerance:
            return None, "impure_swing"
        if np.abs(rows_a[0] - space.start).max() > 1e-4 or np.abs(rows_b[-1] - goal_row).max() > 1e-4:
            return None, "endpoint"
        rows_a[0], rows_b[-1] = space.start, goal_row
        swing_start = len(densify(rows_a)) - 1
        dense = np.asarray(densify(np.vstack([rows_a, rows_b])))
        swing_end = swing_start + len(densify(np.stack([rows_a[-1], rows_b[0]]))) - 1
        if not space._collision_free(dense).all():
            return None, "collision"
        return dense, dict(swing_indices=[int(swing_start), int(swing_end)], swing_q5_deg=float(np.degrees(swing[4])),
                           swing_q7_deg=float(np.degrees(swing[6])), progress=float(np.clip(x[0], 0., 1.)),
                           swivel_deg=float(np.degrees(wrap(x_star[5]))), lift_offset_m=float(x[2]),
                           position_offset_m=float(np.linalg.norm(x[3:6])) if len(x) > 3 else 0.)


def _fine_end(states, weights, resolution):
    """Refined segment whose last step is subdivided further (its end is X*, dropped by the caller)."""
    dense = refine(states, weights, resolution)
    tail = dense[-2] + np.linspace(0., 1., FINE_PARTS + 1)[1:, None] * (dense[-1] - dense[-2])
    return np.vstack([dense[:-1], tail])


def plan_wrist_flip(planner, start, candidates, checker, ik, options, deadline):
    """Return (dense joint path or None, stats) for a wrist-flipping transport; see the module docstring."""
    from v3_cartesian_transport import GOAL_WEIGHTS, CartesianTransportSpace

    began = time.perf_counter()
    options = {**DEFAULTS, **options}
    start = np.asarray(start, dtype=np.float64)
    arm = slice(1, 8) if planner.active_side == "left" else slice(8, 15)
    target = flipped(branch(start[arm]))
    pool = np.asarray(candidates, dtype=np.float64)
    pool = pool[[branch(row[arm]) == target for row in pool]]
    pool = pool[np.argsort(np.linalg.norm((pool - start) * GOAL_WEIGHTS, axis=1))][:options["wrist_flip_goals"]]
    lower, upper = (value[0].item() for value in checker.kinematics.get_joint_limits().position)
    stats = dict(planner="cartesian_wrist_flip", flipped_branch_goals=int(len(pool)), attempts=[],
                 budget_s=max(deadline - began, 0.), timing_ms=dict(setup=0., crossing=0., path=0.))
    clock = lambda key, since: stats["timing_ms"].__setitem__(key, stats["timing_ms"][key] + (time.perf_counter() - since) * 1000)
    flips = {}
    for offsets in (False, True):  # every goal on the exact line first; offsets are the slower search
        for index, goal_row in enumerate(pool):
            if time.perf_counter() > deadline:
                stats.update(search_ms=(time.perf_counter() - began) * 1000, failure="wrist_flip_deadline")
                return None, stats
            if index not in flips:
                tick = time.perf_counter()
                space = CartesianTransportSpace(planner, start, goal_row[None], checker, ik, options)
                flips[index] = WristFlip(space, (lower + 1e-5, upper - 1e-5), options)
                clock("setup", tick)
            tick = time.perf_counter()
            if offsets:
                cross = flips[index].offset_crossing(deadline)
                found = [] if cross is None or cross[0] > .01 else [(None, cross[1])]
            else:
                found = flips[index].crossings(deadline)[:options["wrist_flip_paths"]]
            clock("crossing", tick)
            if not found:
                stats["attempts"].append(dict(goal=index, offsets=offsets, result="no_crossing"))
                continue
            for estimate, x in found:
                tick = time.perf_counter()
                path, info = flips[index].path(x, goal_row)
                clock("path", tick)
                if path is not None:
                    stats.update(info, search_ms=(time.perf_counter() - began) * 1000, offsets=offsets, goal=index,
                                 estimated_s=estimate)
                    stats["attempts"].append(dict(goal=index, offsets=offsets, result="ok"))
                    return path, stats
                stats["attempts"].append(dict(goal=index, offsets=offsets, result=info))
    stats.update(search_ms=(time.perf_counter() - began) * 1000, failure="no_wrist_flip_path")
    return None, stats
