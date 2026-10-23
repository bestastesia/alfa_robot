"""Harness-only RRT seed offsets and search-call recording (MOTION-311).

The planner hard-codes its RRT seeds (approach 20261003+index, multi-root 621648851+attempts, conveyor
20261013, home 20261014, single-arm 20261015). SearchRecorder rebinds batched_rrt_connect_multi_goal in
the planner modules for the duration of a `with` block: offset k adds k*1000003 to every seed (k=0 keeps
the seeds), budget_scale multiplies every search budget (1.0 keeps them; used only to probe censored
outcomes), and every call is recorded with its seed, budget and compact statistics. Production code
is untouched; cuRobo's numerical place IK is not affected.
"""
import time

SEED_STRIDE = 1000003
STAT_KEYS = ("iterations", "tree_nodes", "goal_count", "selected_goal", "first_solution_ms", "first_verified_solution_ms",
             "search_ms", "best_weighted_length", "raw_waypoints", "fine_rejections", "solutions_found", "failure")


def planner_modules():
    import curobo_core.backend as backend
    import v3_multiroot_full_cycle_planner as multiroot
    import v3_single_arm_planner as single_arm
    return {"multiroot": multiroot, "backend": backend, "single_arm": single_arm}


class SearchRecorder:
    def __init__(self, seed_offset=0, modules=None, budget_scale=1.0):
        self.seed_offset = int(seed_offset)
        self.budget_scale = float(budget_scale)
        self.modules = modules
        self.calls = []
        self._originals = {}

    def __enter__(self):
        if self.modules is None:
            self.modules = planner_modules()
        self.t0 = time.perf_counter()
        for name, module in self.modules.items():
            self._originals[name] = module.batched_rrt_connect_multi_goal
            module.batched_rrt_connect_multi_goal = self._wrap(name, self._originals[name])
        return self

    def __exit__(self, *exc):
        for name, module in self.modules.items():
            module.batched_rrt_connect_multi_goal = self._originals[name]
        self._originals = {}
        return False

    def mark(self):
        return len(self.calls)

    def since(self, mark):
        return self.calls[mark:]

    def _wrap(self, name, original):
        def search(start, goals, lower, upper, validity, budget_s, seed, *args, **kwargs):
            seed = int(seed) + self.seed_offset * SEED_STRIDE
            budget_s = budget_s * self.budget_scale if self.budget_scale != 1.0 else budget_s
            began = time.perf_counter()
            path, stats = original(start, goals, lower, upper, validity, budget_s, seed, *args, **kwargs)
            self.calls.append(dict(module=name, seed=seed, budget_s=float(budget_s), start_ms=(began - self.t0) * 1000,
                                   starts=1 if start.ndim == 1 else int(start.shape[0]), goals=int(goals.shape[0]),
                                   success=path is not None, wall_ms=(time.perf_counter() - began) * 1000,
                                   stats={key: stats.get(key) for key in STAT_KEYS if key in (stats or {})}))
            return path, stats
        return search
