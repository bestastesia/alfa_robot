"""Continuous redundancy actions decoded by the frozen analytic IK (no neural joint output).

u is normalized to [-1, 1] from the URDF interval; psi is periodic, normalized by pi. The idle arm stays at the
previous physical state. Every arm uses the nearest valid analytic branch to the previous state, with a stable
lexicographic tie-break; branch choice never probes future stages or RRT results.
"""
import math

import numpy as np

from v3_candidate_ranker import ARMS, CarriageModel, WEIGHTS


def parameters(action, lower_u=-1., upper_u=0.):
    action = np.asarray(action, dtype=np.float64)
    if action.shape != (3,) or not np.isfinite(action).all() or np.any(np.abs(action) > 1. + 1e-8):
        raise ValueError("redundancy action must be a finite 3-vector in [-1, 1]")
    return np.array([lower_u + (float(np.clip(action[0], -1., 1.)) + 1.) * .5 * (upper_u - lower_u),
                     ((action[1] + 1.) % 2. - 1.) * math.pi,
                     ((action[2] + 1.) % 2. - 1.) * math.pi])


def normalized(updown, swivels, lower_u=-1., upper_u=0.):
    return np.array([2. * (updown - lower_u) / (upper_u - lower_u) - 1.,
                     ((swivels[0] + math.pi) % (2 * math.pi) - math.pi) / math.pi,
                     ((swivels[1] + math.pi) % (2 * math.pi) - math.pi) / math.pi])


def nearest_branch(solutions, seed, lower, upper):
    values = np.asarray(solutions, dtype=np.float64).reshape(-1, 7)
    values = values[np.isfinite(values).all(axis=1) & (values >= lower - 1e-7).all(axis=1) & (values <= upper + 1e-7).all(axis=1)]
    if not len(values):
        return None
    order = sorted(range(len(values)), key=lambda i: (float(np.sum((values[i] - seed) ** 2)), tuple(values[i])))
    return values[order[0]].copy()


class RedundancyDecoder:
    def __init__(self, planner):
        from v3_candidate_ranker import Kinematics
        limits = Kinematics(planner.robot["kinematics"]["urdf_path"])
        self.lower, self.upper = limits.lower, limits.upper
        self.planner = planner
        self.carriage = CarriageModel(planner, planner.home_values)

    def encode(self, row):
        psi = [self.planner.analytic.swivel(i, np.asarray(row)[ARMS[side]]) for i, side in enumerate(ARMS)]
        return normalized(row[0], psi, self.lower[0], self.upper[0])

    def decode(self, action, targets, previous, verify=True):
        values = np.asarray(previous, dtype=np.float64).copy()
        params = parameters(action, self.lower[0], self.upper[0])
        values[0] = params[0]
        inverse = self.carriage.inverse(np.array([values[0]]))[0]
        for index, side in enumerate(ARMS):
            if side not in targets:
                continue
            columns = ARMS[side]
            solutions = self.planner.analytic.solve(index, inverse @ targets[side], params[index + 1], values[columns])
            if len(solutions) >= 16:
                raise RuntimeError("analytic bridge capacity reached; incomplete branch set")
            branch = nearest_branch(solutions, values[columns], self.lower[columns], self.upper[columns])
            if branch is None:
                return None
            values[columns] = branch
        if verify:
            self.planner.fk(values)
            for side, target in targets.items():
                actual = self.planner.fk_robot.get_transform(side + '_tool0', self.planner.fk_robot.base_link)
                angle = math.acos(float(np.clip((np.trace(actual[:3, :3].T @ target[:3, :3]) - 1.) / 2., -1., 1.)))
                if np.linalg.norm(actual[:3, 3] - target[:3, 3]) > .002 or angle > math.radians(1.):
                    return None
        return values

    def candidate_set(self, targets, previous, cap=64, stage=0):
        """Enumerate the same normalized grid as the continuous actor can reach, no collision or future checks.
        Order only by current movement time proxy. Keep 3/4 nearest and 1/4 deterministic diverse grid rows."""
        single = len(targets) == 1
        rows, actions = [], []
        u_grid = np.linspace(-1., 1., 21)
        psi_grid = np.linspace(-1., 1., 36, endpoint=False)
        for u in u_grid:
            params = parameters([u, 0., 0.], self.lower[0], self.upper[0])
            inverse = self.carriage.inverse(np.array([params[0]]))[0]
            arms = {}
            for index, side in enumerate(ARMS):
                if side not in targets:
                    continue
                solutions = []
                for psi in psi_grid:
                    found = self.planner.analytic.solve(index, inverse @ targets[side], psi * math.pi, np.asarray(previous)[ARMS[side]])
                    if len(found) >= 16:
                        raise RuntimeError("analytic bridge capacity reached")
                    branch = nearest_branch(found, np.asarray(previous)[ARMS[side]], self.lower[ARMS[side]], self.upper[ARMS[side]])
                    if branch is not None:
                        solutions.append((psi, branch))
                arms[side] = solutions
            if any(not entries for entries in arms.values()):
                continue
            if single:
                side = next(iter(targets))
                for psi, branch in arms[side]:
                    row = np.asarray(previous, dtype=np.float64).copy()
                    row[0], row[ARMS[side]] = params[0], branch
                    action = [u, psi if side == 'left' else 0., psi if side == 'right' else 0.]
                    rows.append(row); actions.append(action)
            else:
                for left_psi, left in arms['left']:
                    for right_psi, right in arms['right']:
                        rows.append(np.concatenate([[params[0]], left, right]))
                        actions.append([u, left_psi, right_psi])
        if not rows:
            return np.empty((0, 15)), np.empty((0, 3))
        rows, actions = np.asarray(rows), np.asarray(actions)
        score = np.linalg.norm((rows - previous) * WEIGHTS, axis=1)
        order = np.argsort(score, kind='stable')
        cap = min(cap, len(rows))
        head = min(cap, max(1, cap * 3 // 4))
        chosen = list(order[:head])
        rest = order[head:]
        if len(chosen) < cap:
            chosen.extend(rest[np.linspace(0, len(rest) - 1, cap - head, dtype=int)].tolist())
        return rows[chosen], actions[chosen]
