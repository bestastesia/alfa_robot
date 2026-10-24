"""MOTION-311 placement (conveyor) IK candidates for the three-IK selection.

Every carry IK reaches the same carry TCP pose, so the conveyor place target derived from it is the same in
every round of a mode. Dual arm: per updown level (5 cm) the `per_arm` fastest-to-Home analytic solutions of
each arm over the 10 deg swivel grid are paired, collision-checked in the exact suffix context (parked base,
loaded checker) and the `limit` fastest valid pairs are kept, with their analytic parameters (updown, swivel
per arm) so a chosen candidate can be re-solved for a run's own target. Single arm: the planner's own analytic
enumeration (idle-arm variants included) ordered from Home. Time to Home is the straight-line bound
max_j |q_j - home_j| / v_j with the URDF velocity limits.
"""
import math

import numpy as np
import torch

from v3_wall_ik_benchmark import ACTIVE_JOINTS

ARMS = {"left": slice(1, 8), "right": slice(8, 15)}


def active_velocities(planner):
    return planner._timed_limits[0][[planner._timed_joint_names.index(name) for name in ACTIVE_JOINTS]]


def time_to(rows, reference, velocity):
    return np.max(np.abs(np.asarray(rows) - np.asarray(reference)) / velocity, axis=-1)


def dual_place_set(planner, targets, validity, per_arm=24, limit=2048):
    """(rows N x 15, params N x 3 [updown, swivel_left_deg, swivel_right_deg]) of valid dual placement IKs."""
    home = np.asarray(planner.home_values, dtype=np.float64)
    velocity = active_velocities(planner)
    lower, upper = [values.cpu().numpy() for values in validity.checker.kinematics.get_joint_limits().position]
    rows, params = [], []
    for updown in np.round(np.arange(-1., .0001, .05), 2):
        if updown < lower[0] - 1e-7 or updown > upper[0] + 1e-7:
            continue
        reference = home.copy()
        reference[0] = updown
        planner.fk(reference)
        carriage = np.linalg.inv(planner.fk_robot.get_transform("arm_carriage", planner.fk_robot.base_link))
        arms = []
        for arm, side in enumerate(ARMS):
            columns = ARMS[side]
            values, swivels = [], []
            for swivel in range(-180, 180, 10):
                for solution in planner.analytic.solve(arm, carriage @ targets[side], math.radians(swivel), reference[columns]):
                    if np.all(solution >= lower[columns] - 1e-7) and np.all(solution <= upper[columns] + 1e-7):
                        values.append(solution)
                        swivels.append(swivel)
            values = np.asarray(values).reshape(-1, 7)
            order = np.argsort(time_to(values, home[columns], velocity[columns]), kind="stable")[:per_arm]
            arms.append((values[order], np.asarray(swivels)[order]))
        for left, left_swivel in zip(*arms[0]):
            for right, right_swivel in zip(*arms[1]):
                rows.append(np.concatenate([[updown], left, right]))
                params.append((updown, left_swivel, right_swivel))
    if not rows:
        return np.empty((0, 15)), np.empty((0, 3))
    rows, params = np.asarray(rows), np.asarray(params, dtype=np.float64)
    valid = []
    for offset in range(0, len(rows), 2048):
        valid.append(validity.mask(torch.tensor(rows[offset:offset + 2048], device="cuda", dtype=torch.float32)).cpu().numpy())
    valid = np.concatenate(valid)
    rows, params = rows[valid], params[valid]
    order = np.argsort(time_to(rows, home, velocity), kind="stable")[:limit]
    return rows[order], params[order]


def single_place_set(planner, targets, checker, limit=512):
    """Valid single-arm placement IKs (idle-arm variants included), ordered by the planner's Home cost."""
    rows = np.asarray(planner.enumerate_single(targets[planner.active_side], planner.home_values, checker, limit), dtype=np.float64)
    side = planner.active_side
    swivel = [math.degrees(planner.analytic.swivel(0 if side == "left" else 1, row[ARMS[side]])) for row in rows]
    params = np.column_stack([rows[:, 0], swivel, swivel]) if len(rows) else np.empty((0, 3))
    return rows, params


def resolve(planner, targets, row, params):
    """Re-solve a stored candidate for this run's own targets with its updown, swivels and nearest branch."""
    row = np.asarray(row, dtype=np.float64).copy()
    planner.fk(row)
    carriage = np.linalg.inv(planner.fk_robot.get_transform("arm_carriage", planner.fk_robot.base_link))
    for arm, side in enumerate(ARMS):
        if side not in targets:
            continue
        columns = ARMS[side]
        solutions = planner.analytic.solve(arm, carriage @ targets[side], math.radians(params[1 + arm]), row[columns])
        if not len(solutions):
            return None
        row[columns] = solutions[np.argmin(np.sum((solutions - row[columns]) ** 2, axis=1))]
    return row
