"""Timed single-arm planner whose transport stage can use the Cartesian direct search (MOTION-323).

`joint_rrt` keeps the baseline transport search (optionally with another seed); `cartesian` runs
`plan_cartesian_transport` on the same filtered goal pool and keeps its task-space path through
time parameterization instead of replacing it with a joint-space shortcut. With
`transport_timing="toppra"` (MOTION-324) the Cartesian transport segment also gets a
path-preserving TOPP-RA candidate, which competes with the pipeline candidates under the same
collision gate.
"""
from v3_cartesian_transport import plan_cartesian_transport
from v3_path_timing import time_parameterize_toppra
from v3_timed_single_arm_planner import TimedSingleArmPlanner

TRANSPORT_SEARCH_MODES = ("joint_rrt", "cartesian")
TRANSPORT_TIMINGS = ("pipeline", "toppra")


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

    def transport_path(self, start, goals, checker, candidates):
        if self.transport_search_mode == "joint_rrt":
            path, stats = self.single_path(start, goals, checker, True, seed=self.transport_seed)
        else:
            path, stats = plan_cartesian_transport(self, start, candidates, checker, seed=self.transport_seed,
                                                   options=self.cartesian_options)
        # Kept because a failed transport raises CycleBlocked before the stats reach the report.
        self.last_transport_stats = dict(stats or {}, mode=self.transport_search_mode, success=path is not None)
        self.last_transport_path = path
        return path, stats

    def _shortcut_phase(self, phase, rows, boxes, original_snapshot):
        if phase == "transport" and self.transport_search_mode == "cartesian":
            return rows, {"raw_waypoints": len(rows), "shortcut_waypoints": len(rows),
                          "shortcut_skipped": "cartesian transport keeps its task-space path"}
        return super()._shortcut_phase(phase, rows, boxes, original_snapshot)

    def _extra_timing_candidates(self, phase, rows, names, sample_dt, boxes, original_snapshot):
        if not (phase == "transport" and self.transport_search_mode == "cartesian" and self.transport_timing == "toppra"):
            return super()._extra_timing_candidates(phase, rows, names, sample_dt, boxes, original_snapshot)
        try:
            trajectory, validation = time_parameterize_toppra(rows, names, *self._timed_limits, sample_dt=sample_dt)
        except RuntimeError as error:
            self.last_toppra_error = str(error)
            return []
        if not self._timed_phase_valid(phase, validation, boxes, original_snapshot):
            self.last_toppra_error = "TOPP-RA timed samples failed the collision gate"
            return []
        return [(trajectory["duration_s"], "validated_toppra", trajectory, validation)]
