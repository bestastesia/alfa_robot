"""StageEnv with box size in the state and cheap per-candidate lookahead features; actions and gates unchanged.

Lookahead uses only quantities known before acting: the current pool, the next stage's goal poses and the
analytic IK. Stage 0 solves each contact's extraction endpoint directly (one analytic solve per arm, the chained
gate stays in screening) and measures it against one shared carry pool generated from Home and payload-gated.
Stage 1 carries the batched carry gates as a validity mask. Times are joint-space lower bounds max|dq|/v in s.
contact_pool="gated" builds the stage-0 pool from the full analytic grid after one batched collision pass, then
applies the same 64-row rule (3/4 nearest to the current state, 1/4 spread); "capped" is the MOTION-311 pool.
place_gate=True also gates stage-2 rows with the suffix's own payload collision check at the conveyor parking pose.
"""
from dataclasses import dataclass, field, replace
import math
import time

import numpy as np
from scipy.spatial.transform import Rotation
import torch

from curobo_core.adapter import matrix_pose
from motion311_learning.contracts import Observation
from motion311_learning.stage_env import StageEnv
from v3_batched_loaded_search import GpuValidity
from v3_box_geometry import extraction_distance_m
from v3_task_scene import grasp_transform
from v3_wall_ik_benchmark import ACTIVE_JOINTS

EXTRA_DIM = 6
FAR_S = 10.
NEAR_WINDOW_S = .5


@dataclass(frozen=True)
class LookaheadObservation(Observation):
    extra: np.ndarray = field(default=None)
    valid: np.ndarray = field(default=None)
    dims: np.ndarray = field(default=None)

    def __post_init__(self):
        super().__post_init__()
        extra = np.asarray(self.extra, dtype=np.float64).reshape(len(self.candidates), EXTRA_DIM)
        valid = np.asarray(self.valid, dtype=bool).reshape(len(self.candidates))
        dims = np.asarray(self.dims, dtype=np.float64).reshape(3)
        if not np.isfinite(extra).all() or not np.isfinite(dims).all():
            raise ValueError("nonfinite lookahead features")
        for name, value in (("extra", extra), ("valid", valid), ("dims", dims)):
            value = value.copy()
            value.setflags(write=False)
            object.__setattr__(self, name, value)


def movement_s(rows, reference, velocity):
    """Joint-space lower-bound time from one reference (15,) to each row (N, 15)."""
    rows = np.asarray(rows, dtype=np.float64).reshape(-1, 15)
    return np.max(np.abs(rows - np.asarray(reference, dtype=np.float64)) / velocity, axis=1)


def pairwise_s(starts, goals, velocity):
    starts = np.asarray(starts, dtype=np.float64).reshape(-1, 15)
    goals = np.asarray(goals, dtype=np.float64).reshape(-1, 15)
    return np.max(np.abs(starts[:, None, :] - goals[None, :, :]) / velocity, axis=2)


def cap_ordered(rows, actions, cap):
    """MOTION-311 pool rule on rows already in the generator's movement-score order: 3/4 head, 1/4 evenly spread."""
    cap = min(cap, len(rows))
    head = min(cap, max(1, cap * 3 // 4))
    chosen = list(range(head))
    rest = np.arange(head, len(rows))
    if len(chosen) < cap:
        chosen.extend(rest[np.linspace(0, len(rest) - 1, cap - head, dtype=int)].tolist())
    return rows[chosen], actions[chosen]


def contact_features(rows, endpoints, home, carry_goals, velocity):
    """[approach, extraction, endpoint solved, nearest carry goal, mean of 8 nearest, share within +0.5 s]."""
    rows = np.asarray(rows, dtype=np.float64).reshape(-1, 15)
    features = np.zeros((len(rows), EXTRA_DIM))
    features[:, 0] = movement_s(rows, home, velocity)
    solved = np.array([endpoint is not None for endpoint in endpoints], dtype=bool)
    features[:, 2] = solved
    features[:, 3:5] = FAR_S
    if solved.any():
        ends = np.stack([endpoint for endpoint in endpoints if endpoint is not None])
        features[solved, 1] = np.max(np.abs(ends - rows[solved]) / velocity, axis=1)
        if len(carry_goals):
            times = np.sort(pairwise_s(ends, carry_goals, velocity), axis=1)
            features[solved, 3] = times[:, 0]
            features[solved, 4] = times[:, :8].mean(axis=1)
            features[solved, 5] = (times <= times[:, :1] + NEAR_WINDOW_S).mean(axis=1)
    return np.minimum(features, FAR_S)


def carry_features(rows, previous, home, reasons, velocity):
    """[transport, passes gates, Home distance, updown change, 0, 0]."""
    rows = np.asarray(rows, dtype=np.float64).reshape(-1, 15)
    features = np.zeros((len(rows), EXTRA_DIM))
    features[:, 0] = movement_s(rows, previous, velocity)
    features[:, 1] = [reason is None for reason in reasons]
    features[:, 2] = movement_s(rows, home, velocity)
    features[:, 3] = np.abs(rows[:, 0] - np.asarray(previous)[0])
    return np.minimum(features, FAR_S)


def place_features(rows, previous, home, velocity):
    """[placement move, return Home, their sum, 0, 0, 0]."""
    rows = np.asarray(rows, dtype=np.float64).reshape(-1, 15)
    features = np.zeros((len(rows), EXTRA_DIM))
    features[:, 0] = movement_s(rows, previous, velocity)
    features[:, 1] = movement_s(rows, home, velocity)
    features[:, 2] = features[:, 0] + features[:, 1]
    return np.minimum(features, FAR_S)


class LookaheadEnv(StageEnv):
    carry_lookahead_cap = 512
    full_grid = 10 ** 9

    def __init__(self, *args, contact_pool="capped", place_gate=False, **kwargs):
        if contact_pool not in ("capped", "gated"):
            raise ValueError("contact_pool must be capped or gated")
        self.contact_pool, self.place_gate = contact_pool, bool(place_gate)
        super().__init__(*args, **kwargs)

    def candidate_pool(self):
        if self.stage != 0 or self.contact_pool == "capped":
            return super().candidate_pool()
        rows, actions = self.decoder.candidate_set(self.targets, self.previous, self.full_grid, 0)
        if not len(rows):
            return rows, actions
        free = self.contact_collision_free(rows)
        return cap_ordered(rows[free], actions[free], self.caps[0])

    def reset(self, item, seed=None, continuous=False):
        self.carry_lookahead = None
        return super().reset(item, seed, continuous)

    def restore(self, state):
        self.carry_lookahead = None
        return super().restore(state)

    def extraction_endpoint(self, row):
        """Direct analytic solve at the full extraction distance with the contact's swivel; None if unsolvable."""
        p = self.planner
        row = np.asarray(row, dtype=np.float64)
        p.fk(row)
        carriage = np.linalg.inv(p.fk_robot.get_transform('arm_carriage', p.fk_robot.base_link))
        tools = {side: p.fk_robot.get_transform(side + '_tool0', p.fk_robot.base_link).copy() for side in self.boxes}
        end = row.copy()
        for side, tool in tools.items():
            index = 0 if side == 'left' else 1
            columns = slice(1 + 7 * index, 8 + 7 * index)
            tool[0, 3] -= extraction_distance_m(p.box_fit['dimensions_m'])
            solutions = np.asarray(p.analytic.solve(index, carriage @ tool, p.analytic.swivel(index, row[columns]), row[columns]))
            if not len(solutions):
                return None
            end[columns] = solutions[np.argmin(np.sum((solutions - row[columns]) ** 2, axis=1))]
        return end

    def place_checks(self, rows):
        """conveyor_suffix's placement gate for a batch: payload collision at the conveyor parking pose (same
        parking computation as the suffix). The snapshot is restored; the suffix re-checks its executed row."""
        p, names = self.planner, self.names
        endpoint = np.asarray(self.report['frames'][-1], dtype=np.float64).copy()
        start = endpoint[[names.index(name) for name in ACTIVE_JOINTS]]
        rotation = Rotation.from_euler('z', math.pi).as_matrix()
        reference = p.conveyor_reference_targets()
        required = []
        for side, tool in p.tool_matrices(start).items():
            box = tool @ grasp_transform(side, p.top_grasp, p.box_fit['dimensions_m'])
            box[0, 3] += p.conveyor_push_m
            goal = np.asarray(reference[side]['position'], dtype=np.float64).copy()
            goal[0] -= p.base_advance
            required.append((goal - rotation @ box[:3, 3])[:2])
        parking = np.mean(required, axis=0)
        if np.max(np.linalg.norm(np.asarray(required) - parking, axis=1)) > .002:
            return ['placement_parking'] * len(rows)
        endpoint[[names.index('base_x'), names.index('base_y')]] = parking
        endpoint[names.index('base_yaw')] = math.pi
        base = np.eye(4)
        base[:3, :3] = rotation
        base[:2, 3] = parking
        original = p.snapshot
        p.snapshot = replace(original, state=original.state.with_positions(dict(zip(names, endpoint)), matrix_pose(base)))
        try:
            validity = GpuValidity(p.checker(self.boxes, payload=True), check_ground=True)
            valid = validity.mask(torch.tensor(np.asarray(rows, dtype=np.float32).reshape(-1, 15), device='cuda')).cpu().numpy()
        finally:
            p.snapshot = original
        return [None if ok else 'place_collision' for ok in valid]

    def _carry_lookahead(self):
        if self.carry_lookahead is None:
            rows, _ = self.decoder.candidate_set(self.carry_targets, self.home, self.carry_lookahead_cap, 1)
            reasons = self.carry_checks(rows)
            self.carry_lookahead = np.asarray([row for row, reason in zip(rows, reasons) if reason is None]).reshape(-1, 15)
        return self.carry_lookahead

    def observe(self):
        base = super().observe()
        began = time.perf_counter()
        rows, reasons = base.candidates, None
        if base.stage == 0:
            endpoints = [self.extraction_endpoint(row) for row in rows]
            extra = contact_features(rows, endpoints, self.home, self._carry_lookahead(), self.velocity)
            valid = np.ones(len(rows), dtype=bool)
        elif base.stage == 1:
            reasons = self.carry_checks(rows)
            extra = carry_features(rows, self.previous, self.home, reasons, self.velocity)
            valid = np.array([reason is None for reason in reasons], dtype=bool)
        else:
            extra = place_features(rows, self.previous, self.home, self.velocity)
            if self.place_gate and len(rows):
                reasons = self.place_checks(rows)
                valid = np.array([reason is None for reason in reasons], dtype=bool)
            else:
                valid = np.ones(len(rows), dtype=bool)
        self.gate_reasons = reasons
        self.lookahead_ms = (time.perf_counter() - began) * 1000
        self.observation = LookaheadObservation(base.stage, base.state, base.candidates, base.actions, base.physics,
                                                base.targets, extra=extra, valid=valid,
                                                dims=np.asarray(self.planner.box_fit['dimensions_m'], dtype=np.float64))
        return self.observation
