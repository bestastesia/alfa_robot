"""Cartesian direct search for the single-arm transport stage (MOTION-323).

A state is (tool position, orientation progress s, updown, swivel psi relative to the start).
The active arm follows the branch-fixed analytic IK, so every state maps to one joint row; the
idle arm is joint-interpolated with s towards the selected goal's idle-arm variant. The existing
RRT-Connect searches this space through `CartesianTransportSpace`, and every result is
re-checked in joint space with the same loaded validity as the baseline.
"""
import ctypes
import math
import time

import numpy as np
import torch
from scipy.spatial.transform import Rotation, Slerp

from curobo_core.backend import WORKSPACE_ROOT
from v3_batched_loaded_search import GpuValidity, densify
from v3_cartesian_math import METRE, CarriageModel, branch, edge_points, refine, wrap
from v3_multiroot_search import batched_rrt_connect_multi_goal

STATE_NAMES = ["tool_x", "tool_y", "tool_z", "progress", "updown", "psi"]
GOAL_WEIGHTS = np.array([METRE] + [1.] * 14)
DEFAULTS = dict(goals_per_group=32, max_jump_deg=8., edge_resolution=.05, path_resolution=.01,
                workspace_margin_m=.35, batch=256, connect_candidates=64, step=.28,
                # MOTION-327 v2 options; all off keeps the MOTION-323 (v1) behaviour.
                shortcut=False, monotonic_progress=False, strict_budget=False, budget_reserve_s=.25)


class BatchAnalyticIk:
    def __init__(self):
        self.library = ctypes.CDLL(str(WORKSPACE_ROOT / "generated/v3_analytic_071cb95/libv3_analytic_bridge.so"))
        values = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
        flags = np.ctypeslib.ndpointer(dtype=np.int32, flags="C_CONTIGUOUS")
        self.library.v3_solve_batch.argtypes = [ctypes.c_int, ctypes.c_int, values, values, values, flags, values, flags]
        self.library.v3_solve_batch.restype = ctypes.c_int
        self.calls = 0
        self.seconds = 0.

    def solve(self, side, targets, swivels, seeds, arm_branch):
        began = time.perf_counter()
        count = len(targets)
        output = np.zeros((count, 7))
        found = np.zeros(count, dtype=np.int32)
        self.library.v3_solve_batch(side, count, np.ascontiguousarray(targets, dtype=np.float64).reshape(-1),
                                    np.ascontiguousarray(swivels, dtype=np.float64),
                                    np.ascontiguousarray(seeds, dtype=np.float64).reshape(-1),
                                    np.asarray(arm_branch, dtype=np.int32), output, found)
        self.calls += count
        self.seconds += time.perf_counter() - began
        return output, found.astype(bool)


class CartesianTransportSpace:
    """Validity adapter that lets the existing RRT-Connect search the Cartesian transport state."""

    joint_names = STATE_NAMES
    stability_weight = 0.

    def __init__(self, planner, start, goal_rows, checker, ik, options):
        self.planner, self.ik, self.options = planner, ik, options
        side = planner.active_side
        self.side_index = 0 if side == "left" else 1
        self.arm = slice(1 + 7 * self.side_index, 8 + 7 * self.side_index)
        self.idle = planner.idle_indices
        self.start = np.asarray(start, dtype=np.float64)
        self.goal_rows = np.asarray(goal_rows, dtype=np.float64)
        self.arm_branch = branch(self.start[self.arm])
        self.carriage = CarriageModel(planner, self.start)
        tools = []
        for row in (self.start, self.goal_rows[0]):
            planner.fk(row)
            tools.append(planner.fk_robot.get_transform(f"{side}_tool0", planner.fk_robot.base_link).copy())
        rotations = Rotation.from_matrix(np.stack([tool[:3, :3] for tool in tools]))
        self.slerp = Slerp([0., 1.], rotations)
        self.turn = (rotations[0].inv() * rotations[1]).magnitude()
        self.psi_start = planner.analytic.swivel(self.side_index, self.start[self.arm])
        self.validity = GpuValidity(checker, check_ground=True)
        self.weights_np = np.array([METRE] * 3 + [max(self.turn, 1e-3), METRE, 1.])
        self.weights = torch.tensor(self.weights_np, device="cuda", dtype=torch.float32)
        self.max_jump = math.radians(options["max_jump_deg"])
        self.start_state = np.r_[tools[0][:3, 3], 0., self.start[0], 0.]
        self.goal_states = np.stack([np.r_[tools[1][:3, 3], 1., row[0],
                                           wrap(planner.analytic.swivel(self.side_index, row[self.arm]) - self.psi_start)]
                                     for row in self.goal_rows])

    def bounds(self, lower_updown, upper_updown):
        positions = np.stack([self.start_state[:3], self.goal_states[0, :3]])
        margin = self.options["workspace_margin_m"]
        lower = np.r_[positions.min(axis=0) - margin, 0., lower_updown, -math.pi]
        upper = np.r_[positions.max(axis=0) + margin, 1., upper_updown, math.pi]
        return (torch.tensor(lower, device="cuda", dtype=torch.float32),
                torch.tensor(upper, device="cuda", dtype=torch.float32))

    def joints(self, states):
        states = np.asarray(states, dtype=np.float64).reshape(-1, 6)
        progress = np.clip(states[:, 3], 0., 1.)
        targets = np.repeat(np.eye(4)[None], len(states), axis=0)
        targets[:, :3, :3] = self.slerp(progress).as_matrix()
        targets[:, :3, 3] = states[:, :3]
        seeds = np.repeat(self.start[self.arm][None], len(states), axis=0)
        arms, found = self.ik.solve(self.side_index, self.carriage.inverse(states[:, 4]) @ targets,
                                    self.psi_start + states[:, 5], seeds, self.arm_branch)
        rows = np.empty((len(states), 15))
        rows[:, 0] = states[:, 4]
        rows[:, self.arm] = arms
        rows[:, self.idle] = self.start[self.idle] + progress[:, None] * (self.goal_rows[0][self.idle] - self.start[self.idle])
        return rows, found

    def _collision_free(self, rows):
        return self.validity.mask(torch.tensor(rows, device="cuda", dtype=torch.float32)).cpu().numpy()

    def edges(self, starts, goals, resolution=None, return_stability=False):
        points = edge_points(starts.detach().cpu().numpy(), goals.detach().cpu().numpy(), self.weights_np,
                             resolution or self.options["edge_resolution"])
        count, horizon = points.shape[:2]
        rows, found = self.joints(points.reshape(-1, 6))
        rows = rows.reshape(count, horizon, 15)
        valid = found.reshape(count, horizon).all(axis=1)
        if horizon > 1:
            valid &= np.abs(np.diff(rows[:, :, self.arm], axis=1)).max(axis=(1, 2)) <= self.max_jump
        candidates = np.nonzero(valid)[0]
        if len(candidates):
            valid[candidates] = self._collision_free(rows[candidates].reshape(-1, 15)).reshape(len(candidates), horizon).all(axis=1)
        result = torch.tensor(valid, device="cuda")
        return (result, torch.zeros(count, device="cuda")) if return_stability else result

    def mask(self, states):
        rows, valid = self.joints(states.detach().cpu().numpy())
        if len(rows) > 1:
            valid[1:] &= np.abs(np.diff(rows[:, self.arm], axis=0)).max(axis=1) <= self.max_jump
        if valid.any():
            valid[np.nonzero(valid)[0]] = self._collision_free(rows[valid])
        return torch.tensor(valid, device=states.device)

    def shortcut(self, waypoints):
        """Greedy state-space shortcut: from each kept waypoint jump to the farthest one with a valid edge."""
        points = np.asarray(waypoints, dtype=np.float64)
        kept = [0]
        while kept[-1] < len(points) - 1:
            source = kept[-1]
            targets = np.arange(len(points) - 1, source, -1)
            starts = torch.tensor(np.repeat(points[source][None], len(targets), axis=0), dtype=torch.float32)
            valid = self.edges(starts, torch.tensor(points[targets], dtype=torch.float32)).cpu().numpy()
            kept.append(int(targets[np.argmax(valid)]) if valid.any() else source + 1)
        return points[kept]

    def monotonic(self, waypoints):
        """Waypoints with non-decreasing orientation progress, or None if a changed edge is invalid."""
        points = np.array(waypoints, dtype=np.float64)
        points[:, 3] = np.maximum.accumulate(points[:, 3])
        if np.allclose(points, waypoints):
            return points
        valid = self.edges(torch.tensor(points[:-1], dtype=torch.float32), torch.tensor(points[1:], dtype=torch.float32))
        return points if bool(valid.all().item()) else None

    def joint_path(self, waypoints, goal_row):
        """Dense joint path for state waypoints, snapped to the exact start and goal rows, or a failure reason."""
        rows, found = self.joints(refine(waypoints, self.weights_np, self.options["path_resolution"]))
        if not found.all():
            return None, "ik"
        if np.abs(np.diff(rows[:, self.arm], axis=0)).max() > self.max_jump:
            return None, "continuity"
        if np.abs(rows[0] - self.start).max() > 1e-4 or np.abs(rows[-1] - goal_row).max() > 1e-4:
            return None, "endpoint"
        rows[0], rows[-1] = self.start, goal_row
        dense = np.asarray(densify(rows))
        if not self._collision_free(dense).all():
            return None, "collision"
        return dense, None


_IK = None


def plan_cartesian_transport(planner, start, candidates, checker, budget_s=2., seed=20261015, options=None):
    """Return (dense joint path or None, stats) for the single-arm transport stage."""
    global _IK
    began = time.perf_counter()
    options = {**DEFAULTS, **(options or {})}
    _IK = _IK or BatchAnalyticIk()
    calls, seconds = _IK.calls, _IK.seconds
    start = np.asarray(start, dtype=np.float64)
    rows = np.asarray(candidates, dtype=np.float64)
    arm = slice(1, 8) if planner.active_side == "left" else slice(8, 15)
    wanted = branch(start[arm])
    same = rows[[branch(row[arm]) == wanted for row in rows]]
    stats = dict(planner="cartesian", branch=list(wanted), candidate_count=len(rows), same_branch_goals=len(same),
                 search_seed=seed, groups_tried=0, iterations=0, tree_nodes=0, solutions_found=0,
                 first_solution_ms=None, first_verified_solution_ms=None, shortcut_applied=False, attempts=[])

    def finish(path, **extra):
        elapsed = (time.perf_counter() - began) * 1000
        stats.update(extra, search_ms=elapsed, ik_calls=_IK.calls - calls, ik_ms=(_IK.seconds - seconds) * 1000)
        if path is not None:
            stats.update(first_solution_ms=stats["first_solution_ms"] or elapsed, first_verified_solution_ms=elapsed)
        return path, stats

    if not len(same):
        return finish(None, failure="no_same_branch_goal")
    groups = {}
    for row in same:
        groups.setdefault(tuple(np.round(row[planner.idle_indices], 6)), []).append(row)
    ordered = sorted((np.asarray(group) for group in groups.values()),
                     key=lambda group: np.linalg.norm((group - start) * GOAL_WEIGHTS, axis=1).min())
    lower, upper = (values[0].item() for values in checker.kinematics.get_joint_limits().position)
    strict = options["strict_budget"]
    over_budget = lambda: time.perf_counter() - began >= budget_s
    for group in ordered:
        if over_budget():
            break
        goals = group[np.argsort(np.linalg.norm((group - start) * GOAL_WEIGHTS, axis=1))][:options["goals_per_group"]]
        space = CartesianTransportSpace(planner, start, goals, checker, _IK, options)
        stats["groups_tried"] += 1
        attempt = dict(idle_group=stats["groups_tried"], goals=len(goals))
        stats["attempts"].append(attempt)
        start_states = torch.tensor(np.repeat(space.start_state[None], len(goals), axis=0), device="cuda", dtype=torch.float32)
        line_ok = space.edges(start_states, torch.tensor(space.goal_states, device="cuda", dtype=torch.float32))
        attempt["direct_lines_coarse_valid"] = int(line_ok.sum().item())
        for index in torch.nonzero(line_ok).reshape(-1).tolist():
            if strict and over_budget():
                break
            stats["first_solution_ms"] = stats["first_solution_ms"] or (time.perf_counter() - began) * 1000
            path, failure = space.joint_path([space.start_state, space.goal_states[index]], goals[index])
            if path is not None and not (strict and over_budget()):
                return finish(path, tier="direct_line", selected_goal=index, raw_waypoints=2)
            attempt.setdefault("direct_line_fine_failures", []).append(failure or "over_budget")
        remaining = budget_s - (time.perf_counter() - began) - (options["budget_reserve_s"] if strict else 0.)
        if remaining <= 0:
            break
        bound_low, bound_high = space.bounds(lower + 1e-5, upper - 1e-5)
        waypoints, search = batched_rrt_connect_multi_goal(
            torch.tensor(space.start_state, device="cuda", dtype=torch.float32),
            torch.tensor(space.goal_states, device="cuda", dtype=torch.float32), bound_low, bound_high, space,
            remaining, seed, stop_at_first_valid=True, batch=options["batch"],
            connect_candidates=options["connect_candidates"], step=options["step"])
        attempt["rrt"] = {key: value for key, value in search.items() if not isinstance(value, (list, dict))}
        stats["iterations"] += search.get("iterations", 0)
        stats["tree_nodes"] += search.get("tree_nodes", search.get("start_nodes", 0) + search.get("goal_nodes", 0))
        stats["solutions_found"] += search.get("solutions_found", 0)
        if search.get("first_solution_ms") is not None and stats["first_solution_ms"] is None:
            stats["first_solution_ms"] = (time.perf_counter() - began) * 1000 - search["search_ms"] + search["first_solution_ms"]
        if waypoints is None:
            continue
        variants = [("raw", np.asarray(waypoints, dtype=np.float64))]
        if options["shortcut"]:
            variants.insert(0, ("shortcut", space.shortcut(waypoints)))
        if options["monotonic_progress"]:
            monotonic = space.monotonic(variants[0][1])
            if monotonic is not None:
                variants.insert(0, (f"{variants[0][0]}_monotonic", monotonic))
        for label, points in variants:
            if strict and over_budget():
                attempt.setdefault("rrt_fine_failures", {})[label] = "over_budget"
                break
            path, failure = space.joint_path(points, goals[search["selected_goal"]])
            if path is not None and not (strict and over_budget()):
                return finish(path, tier="rrt", path_variant=label, selected_goal=search["selected_goal"],
                              raw_waypoints=len(waypoints), final_waypoints=len(points),
                              state_waypoints=points.round(6).tolist())
            attempt.setdefault("rrt_fine_failures", {})[label] = failure or "over_budget"
    return finish(None, failure="budget_exhausted" if over_budget() else "no_valid_path")
