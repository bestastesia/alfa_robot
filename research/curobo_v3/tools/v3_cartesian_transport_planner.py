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
from curobo_core.trajectory import time_parameterize_path, time_parameterize_polyline

TRANSPORT_SEARCH_MODES = ("joint_rrt", "cartesian")
TRANSPORT_TIMINGS = ("pipeline", "toppra")
TRANSPORT_BUDGET_S = 2.


class CartesianTransportPlanner(TimedSingleArmPlanner):
    def __init__(self, args, sample_dt=.025, transport_search_mode="joint_rrt", transport_seed=20261015,
                 cartesian_options=None, transport_timing="pipeline", torque_model=None, use_trajopt=False):
        super().__init__(args, sample_dt, torque_model=torque_model, use_trajopt=use_trajopt)
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

    def plan_geometric_cycle(self, boxes, progress):
        self.last_transport_stats = None
        self.last_transport_path = None
        self.last_wrist_flip = None
        self.last_toppra_error = None
        self.transport_joint_path_used = False
        return super().plan_geometric_cycle(boxes, progress)

    def _branch_change(self, start, goals):
        arm = slice(1, 8) if self.active_side == "left" else slice(8, 15)
        return branch(np.asarray(goals)[0][arm]) != branch(np.asarray(start)[arm])

    def transport_path(self, start, goals, checker, candidates):
        self.last_toppra_error = None
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
                    stats.update(trajectory_fallback_waypoints=(fallback or {}).get("trajectory_fallback_waypoints", []),
                                 tier="joint_rrt_fallback", cartesian_failure=stats.get("failure"), failure=None,
                                 search_ms=elapsed, first_verified_solution_ms=elapsed)
        source = ("joint" if self.transport_search_mode == "joint_rrt" or self.transport_joint_path_used
                  else "wrist_flip" if self.last_wrist_flip else "cartesian")
        stats = dict(stats or {}, path_source=source, success=path is not None)
        # Kept because a failed transport raises CycleBlocked before the stats reach the report.
        self.last_transport_stats = dict(stats or {}, mode=self.transport_search_mode, success=path is not None)
        self.last_transport_path = path
        return path, stats

    @staticmethod
    def _transport_source(result):
        if len(result.get("task", {})) == 2:
            return "joint"
        stats = result.get("transport_search_stats", {})
        source = stats.get("path_source")
        if source not in ("joint", "cartesian", "wrist_flip"):
            raise RuntimeError("transport has no recorded actual path source")
        return source

    def _baseline_waypoints(self, result, phase, rows, names, boxes, original_snapshot):
        if phase == "transport" and self._transport_source(result) != "joint":
            return rows
        return super()._baseline_waypoints(result, phase, rows, names, boxes, original_snapshot)

    def _time_phase(self, result, phase, rows, names, boxes, original_snapshot):
        if phase != "transport" or self._transport_source(result) == "joint":
            return super()._time_phase(result, phase, rows, names, boxes, original_snapshot)
        source = self._transport_source(result)
        candidates, attempts = [], []
        if source == "wrist_flip":
            bounds = result["transport_search_stats"].get("swing_indices", ())
            if len(bounds) != 2 or not 0 < bounds[0] < bounds[1] < len(rows) - 1:
                raise RuntimeError("wrist_flip has invalid stopping boundaries")
            first, last = bounds
            pieces = [rows[:first + 1], rows[first:last + 1], rows[last:]]
            timings = [("validated_toppra_wrist_flip", lambda: time_parameterize_toppra_pieces(
                pieces, names, *self._timed_limits, sample_dt=self.trajectory_sample_dt))]
        else:
            timings = [("validated_cartesian_cubic", lambda: time_parameterize_path(
                rows, names, *self._timed_limits, sample_dt=self.trajectory_sample_dt))]
            if self.transport_timing == "toppra":
                timings.append(("validated_toppra", lambda: time_parameterize_toppra(
                    rows, names, *self._timed_limits, sample_dt=self.trajectory_sample_dt)))
        for name, timing in timings:
            began = time.perf_counter()
            attempt = {"parameterizer": name, "success": False}
            try:
                trajectory, validation = timing()
                attempt["parameterization_ms"] = (time.perf_counter() - began) * 1000
                check_started = time.perf_counter()
                valid = self._timed_phase_valid(phase, validation, boxes, original_snapshot)
                attempt["collision_validation_ms"] = (time.perf_counter() - check_started) * 1000
                if not valid:
                    raise RuntimeError(f"{name}: timed samples failed the collision gate")
                attempt["success"] = True
                candidates.append((trajectory["duration_s"], name, trajectory))
            except RuntimeError as error:
                self.last_toppra_error = attempt["error"] = str(error)
            finally:
                attempt["wall_ms"] = (time.perf_counter() - began) * 1000
                attempts.append(attempt)
                result["transport_timing_attempts"] = attempts
        if not candidates and source != "wrist_flip":
            began = time.perf_counter()
            trajectory, validation = time_parameterize_polyline(
                rows, names, *self._timed_limits, sample_dt=self.trajectory_sample_dt)
            valid = self._timed_phase_valid(phase, validation, boxes, original_snapshot)
            attempts.append({"parameterizer": "validated_cartesian_polyline_fallback", "success": valid,
                             "wall_ms": (time.perf_counter() - began) * 1000})
            if valid:
                candidates.append((trajectory["duration_s"], "validated_cartesian_polyline_fallback", trajectory))
        if not candidates:
            raise RuntimeError(f"{source} time parameterization failed: {attempts}")
        _, name, trajectory = min(candidates, key=lambda item: item[0])
        return trajectory, name, {"trajopt_attempted": False, "path_source": source,
                                  "timing_attempts": attempts}

    def _baseline_contract(self, result, geometric_positions):
        contract = super()._baseline_contract(result, geometric_positions)
        contract["geometry_policy"] = "validated_source_specific_timing"
        contract["phase_parameterizers"] = [segment["parameterizer"] for segment in result["timed_segments"]]
        return contract
