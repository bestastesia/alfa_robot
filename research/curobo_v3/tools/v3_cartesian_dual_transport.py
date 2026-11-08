"""Branch-fixed, multi-root dual Cartesian transport (one payload per arm).

States are [pL(3), pR(3), s, updown, psiL, psiR]. Both swivels use the
first root's reference, not a different zero for each root. Positions are free;
only the two orientation interpolations share progress. The loaded checker is
responsible for both payloads and all whole-robot collision/limit constraints.

The deadline is absolute and cooperative: an in-flight FK/IK/CUDA call cannot
be preempted, but a late result is never returned as a successful path.
"""
import math
import time
from types import SimpleNamespace

import numpy as np
import torch
from scipy.spatial.transform import Rotation, Slerp

from v3_batched_loaded_search import GpuValidity, densify
from v3_cartesian_math import METRE, CarriageModel, branch, edge_points, refine, wrap
from v3_cartesian_transport import (
    BatchAnalyticIk, CartesianTransportSpace, DEFAULTS as SINGLE_DEFAULTS,
    GOAL_WEIGHTS,
)
from v3_multiroot_search import batched_rrt_connect_multi_goal

STATE_NAMES = ["left_x", "left_y", "left_z", "right_x", "right_y", "right_z",
               "progress", "updown", "psi_left", "psi_right"]
DEFAULTS = {**SINGLE_DEFAULTS, "batch": 128, "connect_candidates": 32,
            "edge_resolution": .08, "dual_direct_pairs": 32,
            "dual_direct_share": .5, "dual_postprocess_reserve_s": .01,
            "dual_ik_batch": 1024, "max_updown_step_m": .05,
            "reference_position_tolerance_m": 1e-4,
            "reference_rotation_tolerance_rad": 1e-4,
            # Float32 atlas/FK/analytic reconstruction is not bit-exact.
            "endpoint_joint_tolerance_rad": 2e-4,
            "endpoint_updown_tolerance_m": 1e-6,
            "endpoint_state_tolerance": 2e-6}
ARMS = (slice(1, 8), slice(8, 15))
_IK = None


class _Deadline(Exception):
    """Not a candidate failure; must propagate out of all search/postprocessing."""


class _CandidateMismatch(ValueError):
    pass


def _check_deadline(deadline):
    if time.perf_counter() >= deadline:
        raise _Deadline


def _numpy(values):
    if isinstance(values, torch.Tensor):
        values = values.detach().to(device="cpu", dtype=torch.float64).numpy()
    return np.asarray(values, dtype=np.float64)


def _rows(values):
    rows = _numpy(values)
    if rows.size == 0:
        return rows.reshape(0, 15)
    if rows.ndim != 2 or rows.shape[1] != 15 or not np.isfinite(rows).all():
        raise ValueError("dual transport candidates must be finite [N, 15] joint rows")
    return rows


def _json_safe(value):
    if isinstance(value, torch.Tensor):
        return _json_safe(value.detach().cpu().tolist())
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, np.generic):
        return _json_safe(value.item())
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def dual_branch(row):
    """The left and right (q2, q4, q6) sign triples, in that order."""
    row = _numpy(row)
    if row.shape != (15,) or not np.isfinite(row).all():
        raise ValueError("dual_branch requires one finite 15-joint row")
    return branch(row[ARMS[0]]), branch(row[ARMS[1]])


def group_dual_candidates(roots, goals):
    """Retain every root/goal of each matching branch pair, with original indices."""
    roots, goals = _rows(roots), _rows(goals)
    groups = {}
    for index, row in enumerate(roots):
        signature = dual_branch(row)
        groups.setdefault(signature, dict(branch=signature, root_indices=[], goal_indices=[]))[
            "root_indices"].append(index)
    for index, row in enumerate(goals):
        signature = dual_branch(row)
        if signature in groups:
            groups[signature]["goal_indices"].append(index)
    return [group for group in groups.values() if group["goal_indices"]]


def _pair_distances(roots, goals):
    return np.linalg.norm((roots[:, None] - goals[None]) * GOAL_WEIGHTS, axis=2)


def nearest_branch_change(roots, goals):
    """Route iff the baseline-weighted nearest pair changes either arm's branch."""
    roots, goals = _rows(roots), _rows(goals)
    if not len(roots) or not len(goals):
        return dict(route=False, root_index=None, goal_index=None, reason="no_candidates")
    distances = _pair_distances(roots, goals)
    root, goal = np.unravel_index(np.argmin(distances), distances.shape)
    first, last = dual_branch(roots[root]), dual_branch(goals[goal])
    return dict(route=first != last, root_index=int(root), goal_index=int(goal),
                distance=float(distances[root, goal]),
                root_branch=[list(item) for item in first], goal_branch=[list(item) for item in last])


class DualCartesianTransportSpace(CartesianTransportSpace):
    """Ten-dimensional adapter for the existing dimension-agnostic RRT-Connect.

    Only the dimension-independent state shortcut is inherited. All methods
    assuming the single-arm six-dimensional layout are replaced.
    """

    joint_names = STATE_NAMES
    stability_weight = 0.

    def __init__(self, planner, roots, goals, checker, ik, options):
        self.options = {**DEFAULTS, **(options or {})}
        self.deadline = self.options.get("_deadline", math.inf)
        self._check()
        self.planner, self.ik = planner, ik
        self.root_rows, self.goal_rows = _rows(roots).copy(), _rows(goals).copy()
        if not len(self.root_rows) or not len(self.goal_rows):
            raise _CandidateMismatch("no_candidates")
        self.arm_branches = dual_branch(self.root_rows[0])
        if any(dual_branch(row) != self.arm_branches for row in np.vstack([self.root_rows, self.goal_rows])):
            raise _CandidateMismatch("branch_mismatch")
        self.max_jump = math.radians(self.options["max_jump_deg"])
        self.max_updown_step = self.options["max_updown_step_m"]
        self.lower_joints, self.upper_joints = (
            _numpy(limit).reshape(15) for limit in checker.kinematics.get_joint_limits().position)

        def checked_fk(row):
            self._check()
            planner.fk(row)
            self._check()

        # CarriageModel probes FK several times internally. Check between
        # those probes too, rather than spending the budget on all of them.
        self.carriage = CarriageModel(SimpleNamespace(fk=checked_fk, fk_robot=planner.fk_robot),
                                      self.root_rows[0])
        self._check()
        poses = []
        for pool in (self.root_rows, self.goal_rows):
            tools = []
            for row in pool:
                self._check()
                planner.fk(row)
                tools.append(np.stack([
                    planner.fk_robot.get_transform(f"{side}_tool0", planner.fk_robot.base_link).copy()
                    for side in ("left", "right")]))
                self._check()
            tools = np.asarray(tools)
            for side in range(2):
                positions = np.linalg.norm(tools[:, side, :3, 3] - tools[0, side, :3, 3], axis=1)
                rotations = Rotation.from_matrix(tools[:, side, :3, :3])
                angles = (rotations[0].inv() * rotations).magnitude()
                if (np.any(positions > self.options["reference_position_tolerance_m"])
                        or np.any(angles > self.options["reference_rotation_tolerance_rad"])):
                    raise _CandidateMismatch("reference_pose_mismatch")
            poses.append(tools)
        self.slerps, turns = [], []
        for side in range(2):
            rotations = Rotation.from_matrix(np.stack([pose[0, side, :3, :3] for pose in poses]))
            self.slerps.append(Slerp([0., 1.], rotations))
            turns.append((rotations[0].inv() * rotations[1]).magnitude())
        self.psi_reference = np.array([
            planner.analytic.swivel(side, self.root_rows[0, arm]) for side, arm in enumerate(ARMS)])
        states = []
        for progress, (pool, tools) in enumerate(zip((self.root_rows, self.goal_rows), poses)):
            values = np.empty((len(pool), 10))
            values[:, :6] = tools[:, :, :3, 3].reshape(-1, 6)
            values[:, 6], values[:, 7] = progress, pool[:, 0]
            for index, row in enumerate(pool):
                self._check()
                for side, arm in enumerate(ARMS):
                    values[index, 8 + side] = wrap(planner.analytic.swivel(side, row[arm]) - self.psi_reference[side])
            states.append(values)
        self.start_states, self.goal_states = states
        self.weights_np = np.array([METRE] * 6 + [max(np.linalg.norm(turns), 1e-3), METRE, 1., 1.])
        # Reject an incompatible group instead of silently losing atlas candidates
        # or snapping an unrelated solution onto an endpoint.
        reconstructed, found = self.joints(np.vstack(states))
        endpoints = np.vstack([self.root_rows, self.goal_rows])
        if not (found.all() and self._endpoint_close(reconstructed, endpoints).all()):
            raise _CandidateMismatch("endpoint_round_trip")
        self._check()
        self.validity = GpuValidity(checker, check_ground=True)
        self.device = self.validity.weights.device
        self.weights = torch.tensor(self.weights_np, device=self.device, dtype=torch.float32)
        self._check()

    def _check(self):
        _check_deadline(getattr(self, "deadline", math.inf))

    def _endpoint_close(self, first, last):
        delta = np.abs(first - last)
        return ((delta[..., 0] <= self.options["endpoint_updown_tolerance_m"])
                & (delta[..., 1:].max(axis=-1) <= self.options["endpoint_joint_tolerance_rad"]))

    def bounds(self, lower_updown, upper_updown):
        self._check()
        positions = np.vstack([self.start_states[:, :6], self.goal_states[:, :6]])
        margin = self.options["workspace_margin_m"]
        lower = np.r_[positions.min(axis=0) - margin, 0., lower_updown, -math.pi, -math.pi]
        upper = np.r_[positions.max(axis=0) + margin, 1., upper_updown, math.pi, math.pi]
        return (torch.tensor(lower, device=self.device, dtype=torch.float32),
                torch.tensor(upper, device=self.device, dtype=torch.float32))

    def joints(self, states):
        """Sequential batch IK per arm, one shared lift and fixed seed/branch per arm."""
        self._check()
        states = _numpy(states).reshape(-1, 10)
        rows = np.full((len(states), 15), np.nan)
        rows[:, 0] = states[:, 7]
        found = (np.isfinite(states).all(axis=1) & (states[:, 6] >= -1e-7)
                 & (states[:, 6] <= 1. + 1e-7)
                 & (states[:, 7] >= self.lower_joints[0]) & (states[:, 7] <= self.upper_joints[0]))
        indices = np.flatnonzero(found)
        batch = max(1, int(self.options["dual_ik_batch"]))
        for offset in range(0, len(indices), batch):
            self._check()
            selected = indices[offset:offset + batch]
            chunk = states[selected]
            inverse = self.carriage.inverse(chunk[:, 7])
            for side, arm in enumerate(ARMS):
                self._check()
                targets = np.repeat(np.eye(4)[None], len(chunk), axis=0)
                targets[:, :3, :3] = self.slerps[side](np.clip(chunk[:, 6], 0., 1.)).as_matrix()
                targets[:, :3, 3] = chunk[:, 3 * side:3 * side + 3]
                seeds = np.repeat(self.root_rows[0, arm][None], len(chunk), axis=0)
                arms, solved = self.ik.solve(side, inverse @ targets,
                                             self.psi_reference[side] + chunk[:, 8 + side],
                                             seeds, self.arm_branches[side])
                self._check()
                rows[selected, arm] = arms
                signature = np.where(arms[:, [1, 3, 5]] > 0, 1, -1)
                found[selected] &= solved & (signature == self.arm_branches[side]).all(axis=1)
        found &= (np.isfinite(rows).all(axis=1) & (rows >= self.lower_joints).all(axis=1)
                  & (rows <= self.upper_joints).all(axis=1))
        self._check()
        return rows, found

    def _collision_free(self, rows):
        self._check()
        result = np.empty(len(rows), dtype=bool)
        for offset in range(0, len(rows), 4096):
            self._check()
            chunk = torch.tensor(rows[offset:offset + 4096], device=self.device, dtype=torch.float32)
            result[offset:offset + len(chunk)] = self.validity.mask(chunk).detach().cpu().numpy()
            self._check()
        return result

    def _continuous(self, rows):
        """Per-step tests in radians for all 14 arm axes, metres for updown."""
        delta = np.abs(np.diff(rows, axis=-2))
        return ((delta[..., 1:].max(axis=-1) <= self.max_jump)
                & (delta[..., 0] <= self.max_updown_step))

    def edges(self, starts, goals, resolution=None, return_stability=False):
        self._check()
        first, last = _numpy(starts).reshape(-1, 10), _numpy(goals).reshape(-1, 10)
        device = starts.device if isinstance(starts, torch.Tensor) else self.device
        if len(first):
            points = edge_points(first, last, self.weights_np,
                                 self.options["edge_resolution"] if resolution is None else resolution)
            count, horizon = points.shape[:2]
            rows, found = self.joints(points.reshape(-1, 10))
            rows = rows.reshape(count, horizon, 15)
            valid = found.reshape(count, horizon).all(axis=1) & self._continuous(rows).all(axis=1)
            candidates = np.flatnonzero(valid)
            if len(candidates):
                valid[candidates] = self._collision_free(rows[candidates].reshape(-1, 15)).reshape(
                    len(candidates), horizon).all(axis=1)
        else:
            valid = np.zeros(0, dtype=bool)
        self._check()
        result = torch.tensor(valid, device=device)
        return (result, torch.zeros(len(valid), device=device)) if return_stability else result

    def mask(self, states):
        """Validate an ordered dense path (the RRT's final-verification contract)."""
        rows, valid = self.joints(states)
        valid[1:] &= self._continuous(rows)
        candidates = np.flatnonzero(valid)
        if len(candidates):
            valid[candidates] = self._collision_free(rows[candidates])
        self._check()
        device = states.device if isinstance(states, torch.Tensor) else self.device
        return torch.tensor(valid, device=device)

    def monotonic(self, waypoints):
        self._check()
        points = _numpy(waypoints).copy()
        points[:, 6] = np.maximum.accumulate(points[:, 6])
        if np.array_equal(points, _numpy(waypoints)):
            return points
        valid = self.edges(torch.tensor(points[:-1], dtype=torch.float32),
                           torch.tensor(points[1:], dtype=torch.float32))
        self._check()
        return points if bool(valid.all().item()) else None

    def root_index(self, state):
        """Identify the original root from an RRT endpoint, allowing float32 transport."""
        self._check()
        state = _numpy(state)
        # RRT stores float32 states. Match that exact representation before
        # using a tolerance; otherwise two close roots can be confused. If
        # distinct joint rows collapse to the same state, do not guess one.
        exact = np.flatnonzero(np.all(self.start_states == state, axis=1))
        rounded = np.flatnonzero(np.all(self.start_states.astype(np.float32) == state, axis=1))
        matches = rounded if np.array_equal(state, state.astype(np.float32)) else exact
        if len(matches):
            rows = self.root_rows[matches]
            return int(matches[0]) if np.all(rows == rows[0]) else None
        distance = np.max(np.abs(self.start_states - state), axis=1)
        matches = np.flatnonzero(distance <= self.options["endpoint_state_tolerance"])
        if len(matches) != 1:
            return None
        return int(matches[0])

    def joint_path(self, states, goal_row, start_row=None):
        """Reconstruct, snap exact selected endpoints, then recheck continuity/collision."""
        self._check()
        states = _numpy(states).reshape(-1, 10)
        if len(states) < 2 or not np.isfinite(states).all():
            return None, "endpoint"
        if start_row is None:
            root = self.root_index(states[0])
            if root is None:
                return None, "endpoint"
            start_row = self.root_rows[root]
        else:
            start_row = _numpy(start_row)
            matches = np.flatnonzero(np.all(self.root_rows == start_row, axis=1))
            if not len(matches) or not np.any(np.max(np.abs(self.start_states[matches] - states[0]), axis=1)
                                             <= self.options["endpoint_state_tolerance"]):
                return None, "endpoint"
        goal_row = _numpy(goal_row)
        matches = np.flatnonzero(np.all(self.goal_rows == goal_row, axis=1))
        if not len(matches) or not np.any(np.max(np.abs(self.goal_states[matches] - states[-1]), axis=1)
                                         <= self.options["endpoint_state_tolerance"]):
            return None, "endpoint"
        rows, found = self.joints(refine(states, self.weights_np, self.options["path_resolution"]))
        if not found.all():
            return None, "ik"
        if not self._continuous(rows).all():
            return None, "continuity"
        if not self._endpoint_close(rows[[0, -1]], np.stack([start_row, goal_row])).all():
            return None, "endpoint"
        rows[0], rows[-1] = start_row, goal_row
        # Snapping can introduce a jump even if the unsnapped sequence passed.
        if not self._continuous(rows).all():
            return None, "continuity"
        self._check()
        dense = np.asarray(densify(rows), dtype=np.float64)
        dense[0], dense[-1] = start_row, goal_row
        self._check()
        if not self._continuous(dense).all():
            return None, "continuity"
        if not self._collision_free(dense).all():
            return None, "collision"
        self._check()
        return dense, None


def plan_dual_cartesian_transport(planner, roots, goals, checker, budget_s=.175,
                                  seed=20261015, options=None):
    """Return (dense 15-axis path or None, JSON-safe stats), under one fixed deadline.

    Direct pairs are distance-sorted and checked in chunks, not a truncated
    candidate pool. Direct attempts get at most half the group's remaining
    time by default; the rest is available to multi-root/multi-goal RRT and
    final reconstruction. Joint-space fallback is the caller's responsibility.
    """
    global _IK
    began = time.perf_counter()
    if not math.isfinite(budget_s) or budget_s < 0:
        raise ValueError("dual transport budget_s must be finite and nonnegative")
    deadline = began + budget_s
    options = {**DEFAULTS, **(options or {}), "_deadline": deadline}
    chunk_size = int(options["dual_direct_pairs"])
    if chunk_size < 1 or not 0. <= options["dual_direct_share"] <= 1.:
        raise ValueError("dual_direct_pairs must be positive and dual_direct_share in [0, 1]")
    stats = dict(planner="cartesian_dual", budget_s=float(budget_s), search_seed=int(seed),
                 root_count=0, goal_count=0, groups_tried=0, iterations=0, tree_nodes=0,
                 solutions_found=0, first_solution_ms=None, first_verified_solution_ms=None,
                 shortcut_applied=False, attempts=[])
    ik = None
    calls, seconds = 0, 0.

    def finish(path, **extra):
        stats.update(extra)
        stats.update(ik_calls=0 if ik is None else ik.calls - calls,
                     ik_ms=0. if ik is None else (ik.seconds - seconds) * 1000)
        result = _json_safe(stats)
        now = time.perf_counter()
        if now >= deadline:
            path = None
            result.update(failure="budget_exhausted")
            result.pop("tier", None)
        result.update(search_ms=(now - began) * 1000, success=path is not None)
        if path is not None:
            result["first_verified_solution_ms"] = result["search_ms"]
        return path, result

    try:
        _check_deadline(deadline)
        roots, goals = _rows(roots), _rows(goals)
        stats.update(root_count=len(roots), goal_count=len(goals), candidate_count=len(goals))
        groups = group_dual_candidates(roots, goals)
        stats.update(group_count=len(groups),
                     same_branch_roots=sum(len(group["root_indices"]) for group in groups),
                     same_branch_goals=sum(len(group["goal_indices"]) for group in groups),
                     same_branch_pairs=sum(len(group["root_indices"]) * len(group["goal_indices"]) for group in groups))
        stats["branch_pair_coverage"] = stats["same_branch_pairs"] / max(1, len(roots) * len(goals))
        _check_deadline(deadline)
        if not groups:
            return finish(None, failure="no_same_branch_pair")
        ordered = []
        for group in groups:
            _check_deadline(deadline)
            distances = _pair_distances(roots[group["root_indices"]], goals[group["goal_indices"]])
            ordered.append((float(distances.min()), group, np.argsort(distances, axis=None, kind="stable")))
        ordered.sort(key=lambda item: item[0])
        _check_deadline(deadline)
        _IK = _IK or BatchAnalyticIk()
        ik = _IK
        calls, seconds = ik.calls, ik.seconds
        _check_deadline(deadline)
        for group_number, (_, group, pairs) in enumerate(ordered):
            _check_deadline(deadline)
            # A soft fair-share boundary. The hard deadline never changes.
            now = time.perf_counter()
            group_end = now + (deadline - now) / (len(ordered) - group_number)
            attempt = dict(**group, direct_pairs_total=len(pairs), direct_pairs_checked=0,
                           direct_chunks=0, direct_lines_coarse_valid=0)
            stats["attempts"].append(attempt)
            stats["groups_tried"] += 1
            group_roots, group_goals = roots[group["root_indices"]], goals[group["goal_indices"]]
            try:
                space = DualCartesianTransportSpace(planner, group_roots, group_goals, checker, ik, options)
            except _CandidateMismatch as error:
                _check_deadline(deadline)
                attempt["failure"] = str(error)
                continue
            _check_deadline(deadline)
            now = time.perf_counter()
            reserve = max(0., options["dual_postprocess_reserve_s"])
            direct_end = now + max(0., group_end - now - reserve) * options["dual_direct_share"]
            for offset in range(0, len(pairs), chunk_size):
                _check_deadline(deadline)
                if time.perf_counter() >= direct_end:
                    break
                root_indices, goal_indices = np.unravel_index(pairs[offset:offset + chunk_size],
                                                              (len(group_roots), len(group_goals)))
                first = torch.tensor(space.start_states[root_indices], device=space.device, dtype=torch.float32)
                last = torch.tensor(space.goal_states[goal_indices], device=space.device, dtype=torch.float32)
                valid = space.edges(first, last).detach().cpu().numpy()
                _check_deadline(deadline)
                attempt["direct_pairs_checked"] += len(root_indices)
                attempt["direct_chunks"] += 1
                attempt["direct_lines_coarse_valid"] += int(valid.sum())
                for index in np.flatnonzero(valid):
                    if time.perf_counter() >= direct_end:
                        break
                    if stats["first_solution_ms"] is None:
                        stats["first_solution_ms"] = (time.perf_counter() - began) * 1000
                    root, goal = int(root_indices[index]), int(goal_indices[index])
                    path, failure = space.joint_path([space.start_states[root], space.goal_states[goal]],
                                                     group_goals[goal], start_row=group_roots[root])
                    _check_deadline(deadline)
                    if path is not None:
                        stats["solutions_found"] += 1
                        return finish(path, tier="direct_line", selected_root=group["root_indices"][root],
                                      selected_goal=group["goal_indices"][goal], raw_waypoints=2)
                    attempt.setdefault("direct_line_fine_failures", []).append(failure)
            _check_deadline(deadline)
            lower, upper = space.bounds(float(space.lower_joints[0]) + 1e-5,
                                         float(space.upper_joints[0]) - 1e-5)
            first = torch.tensor(space.start_states, device=space.device, dtype=torch.float32)
            last = torch.tensor(space.goal_states, device=space.device, dtype=torch.float32)
            remaining = group_end - time.perf_counter() - reserve
            if remaining <= 0:
                attempt["rrt_skipped"] = "no_remaining_group_budget"
                continue
            waypoints, search = batched_rrt_connect_multi_goal(
                first, last, lower, upper, space, remaining, int(seed), stop_at_first_valid=True,
                batch=options["batch"], connect_candidates=options["connect_candidates"], step=options["step"])
            attempt["rrt"] = search
            _check_deadline(deadline)
            stats["iterations"] += search.get("iterations", 0)
            stats["tree_nodes"] += search.get("tree_nodes", search.get("start_nodes", 0) + search.get("goal_nodes", 0))
            stats["solutions_found"] += search.get("solutions_found", 0)
            if waypoints is None:
                continue
            if stats["first_solution_ms"] is None:
                stats["first_solution_ms"] = (time.perf_counter() - began) * 1000
            points = _numpy(waypoints)
            root = space.root_index(points[0])
            goal = search.get("selected_goal")
            if root is None or goal is None or not 0 <= int(goal) < len(group_goals):
                attempt["failure"] = "rrt_endpoint"
                continue
            goal = int(goal)
            variants = [("raw", points)]
            if options["shortcut"]:
                variants.insert(0, ("shortcut", space.shortcut(points)))
                _check_deadline(deadline)
            if options["monotonic_progress"]:
                revised = space.monotonic(variants[0][1])
                _check_deadline(deadline)
                if revised is not None:
                    variants.insert(0, (variants[0][0] + "_monotonic", revised))
            for label, points in variants:
                _check_deadline(deadline)
                path, failure = space.joint_path(points, group_goals[goal], start_row=group_roots[root])
                _check_deadline(deadline)
                if path is not None:
                    return finish(path, tier="rrt", path_variant=label, shortcut_applied=label.startswith("shortcut"),
                                  selected_root=group["root_indices"][root], selected_goal=group["goal_indices"][goal],
                                  raw_waypoints=len(waypoints), final_waypoints=len(points), state_waypoints=points.tolist())
                attempt.setdefault("rrt_fine_failures", {})[label] = failure
    except _Deadline:
        return finish(None, failure="budget_exhausted")
    return finish(None, failure="no_valid_path")
