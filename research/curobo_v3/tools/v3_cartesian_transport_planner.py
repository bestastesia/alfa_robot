"""Timed single-arm planner whose transport stage can use the Cartesian direct search (MOTION-323).

`joint_rrt` keeps the baseline transport search (optionally with another seed); `cartesian` runs
`plan_cartesian_transport` on the same filtered goal pool and keeps its task-space path through
time parameterization instead of replacing it with a joint-space shortcut. With
`transport_timing="toppra"` (MOTION-324) the Cartesian transport segment also gets a
path-preserving TOPP-RA candidate, which competes with the pipeline candidates under the same
collision gate. MOTION-342 options in `cartesian_options` (both off by default):
- `fallback_joint_rrt`: a Cartesian search that finds no path hands the rest of the same 2 s transport
  budget to the baseline joint RRT (same goals and seed);
- `route_branch_change`: if the nearest goal (the baseline's goal order) lies on another wrist branch
  sign(q2, q4, q6) than the start, the stage goes to the baseline joint RRT directly, because the
  branch-fixed Cartesian search then needs a long detour around the wrist singularity.
A joint-space transport path from either option is shortcut and timed like the baseline's.
With `wrist_flip` (also off by default, used together with `route_branch_change`), a stage whose nearest
goal differs from the start only in the wrist branch first tries a straight tool path with one
controlled wrist flip (v3_cartesian_wrist_flip); the box stops at the flip, so that path is timed in
three rest-to-rest TOPP-RA pieces. That search is capped at `wrist_flip_budget_s`; if it finds no path,
the stage goes to the joint RRT with the rest of the budget.
"""
import time

import numpy as np

import v3_cartesian_transport as cartesian
from v3_cartesian_math import branch
from v3_cartesian_transport import plan_cartesian_transport
from v3_cartesian_wrist_flip import DEFAULTS as WRIST_FLIP_DEFAULTS, plan_wrist_flip, wrist_only_change
from v3_path_timing import time_parameterize_toppra, time_parameterize_toppra_pieces
from v3_timed_single_arm_planner import TimedSingleArmPlanner

TRANSPORT_SEARCH_MODES = ("joint_rrt", "cartesian")
TRANSPORT_TIMINGS = ("pipeline", "toppra")
TRANSPORT_BUDGET_S = 2.


class CartesianTransportPlanner(TimedSingleArmPlanner):
    def __init__(self, args, sample_dt=.025, transport_search_mode="joint_rrt", transport_seed=20261015,
                 cartesian_options=None, transport_timing="pipeline"):
        super().__init__(args, sample_dt)
        if transport_search_mode not in TRANSPORT_SEARCH_MODES:
            raise ValueError(f"transport_search_mode must be one of {TRANSPORT_SEARCH_MODES}")
        if transport_timing not in TRANSPORT_TIMINGS:
            raise ValueError(f"transport_timing must be one of {TRANSPORT_TIMINGS}")
        self.transport_search_mode = transport_search_mode
        self.transport_seed = int(transport_seed)
        self.cartesian_options = dict(cartesian_options or {})
        self.transport_timing = transport_timing
        self.last_toppra_error = None
        self.transport_joint_path_used = False
        self.last_wrist_flip = None

    def _branch_change(self, start, goals):
        arm = slice(1, 8) if self.active_side == "left" else slice(8, 15)
        return branch(np.asarray(goals)[0][arm]) != branch(np.asarray(start)[arm])

    def transport_path(self, start, goals, checker, candidates):
        self.transport_joint_path_used = False
        self.last_wrist_flip = None
        if self.transport_search_mode == "joint_rrt":
            path, stats = self.single_path(start, goals, checker, True, seed=self.transport_seed)
        elif self.cartesian_options.get("route_branch_change") and self._branch_change(start, goals):
            began, path, flip = time.perf_counter(), None, None
            arm = slice(1, 8) if self.active_side == "left" else slice(8, 15)
            if self.cartesian_options.get("wrist_flip") and wrist_only_change(np.asarray(start)[arm], np.asarray(goals)[0][arm]):
                cartesian._IK = cartesian._IK or cartesian.BatchAnalyticIk()
                options = {**cartesian.DEFAULTS, **WRIST_FLIP_DEFAULTS, **self.cartesian_options}
                path, flip = plan_wrist_flip(self, start, candidates, checker, cartesian._IK, options,
                                             began + min(options["wrist_flip_budget_s"], TRANSPORT_BUDGET_S))
            if path is not None:
                stats = dict(flip, tier="wrist_flip", first_verified_solution_ms=flip["search_ms"])
                self.last_wrist_flip = dict(rows=len(path), swing=flip["swing_indices"])
            else:
                remaining = TRANSPORT_BUDGET_S - (time.perf_counter() - began) if flip else TRANSPORT_BUDGET_S
                path, stats = self.single_path(start, goals, checker, True, budget=remaining, seed=self.transport_seed)
                elapsed = (time.perf_counter() - began) * 1000
                stats = dict(stats or {}, tier="joint_rrt_routed", wrist_flip=flip, search_ms=elapsed,
                             first_verified_solution_ms=elapsed)
                self.transport_joint_path_used = path is not None
        else:
            began = time.perf_counter()
            path, stats = plan_cartesian_transport(self, start, candidates, checker, budget_s=TRANSPORT_BUDGET_S,
                                                   seed=self.transport_seed, options=self.cartesian_options)
            remaining = TRANSPORT_BUDGET_S - (time.perf_counter() - began)
            if path is None and self.cartesian_options.get("fallback_joint_rrt") and remaining > 0:
                path, fallback = self.single_path(start, goals, checker, True, budget=remaining, seed=self.transport_seed)
                elapsed = (time.perf_counter() - began) * 1000
                stats = dict(stats or {}, fallback=dict(fallback or {}, budget_s=remaining, success=path is not None))
                if path is not None:
                    self.transport_joint_path_used = True
                    stats.update(tier="joint_rrt_fallback", cartesian_failure=stats.get("failure"), failure=None,
                                 search_ms=elapsed, first_verified_solution_ms=elapsed)
        # Kept because a failed transport raises CycleBlocked before the stats reach the report.
        self.last_transport_stats = dict(stats or {}, mode=self.transport_search_mode, success=path is not None)
        self.last_transport_path = path
        return path, stats

    def _shortcut_phase(self, phase, rows, boxes, original_snapshot):
        if phase == "transport" and self.transport_search_mode == "cartesian" and not self.transport_joint_path_used:
            return rows, {"raw_waypoints": len(rows), "shortcut_waypoints": len(rows),
                          "shortcut_skipped": "cartesian transport keeps its task-space path"}
        return super()._shortcut_phase(phase, rows, boxes, original_snapshot)

    def _extra_timing_candidates(self, phase, rows, names, sample_dt, boxes, original_snapshot):
        if not (phase == "transport" and self.transport_search_mode == "cartesian" and self.transport_timing == "toppra"
                and not self.transport_joint_path_used):
            return super()._extra_timing_candidates(phase, rows, names, sample_dt, boxes, original_snapshot)
        candidates = []
        timings = [("validated_toppra", lambda: time_parameterize_toppra(rows, names, *self._timed_limits, sample_dt=sample_dt))]
        if self.last_wrist_flip and self.last_wrist_flip["rows"] == len(rows):
            first, last = self.last_wrist_flip["swing"]
            pieces = [rows[:first + 1], rows[first:last + 1], rows[last:]]
            timings.append(("validated_toppra_wrist_flip", lambda: time_parameterize_toppra_pieces(
                pieces, names, *self._timed_limits, sample_dt=sample_dt)))
        for name, timing in timings:
            try:
                trajectory, validation = timing()
            except RuntimeError as error:
                self.last_toppra_error = str(error)
                continue
            if not self._timed_phase_valid(phase, validation, boxes, original_snapshot):
                self.last_toppra_error = f"{name}: timed samples failed the collision gate"
                continue
            candidates.append((trajectory["duration_s"], name, trajectory, validation))
        return candidates
