"""NumPy-only geometry for the MOTION-323 Cartesian transport search (unit-tested without CUDA)."""
import math

import numpy as np

METRE = math.radians(15) / .1  # 10 cm of tool or carriage motion weighs like 15 degrees, as in the baseline


def branch(arm):
    """Analytic IK branch of a 7-joint arm: signs of q2, q4 and q6."""
    return tuple(1 if arm[index] > 0 else -1 for index in (1, 3, 5))


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class JointFrame:
    """Planner joint coordinates relative to the analytic IK library's own (071cb95) ones:
    q_library = wrap(offsets[side] + signs*q) as in v3_zero_model_candidate; no offsets is the identity.

    Branches are the signs of the library's q2, q4 and q6. Taken on the library angles they keep
    naming the same continuous solution family whatever the planner's joint coordinates."""

    def __init__(self, offsets=None, signs=None):
        self.identity = offsets is None
        if not self.identity:
            self.offsets = np.asarray(offsets, dtype=np.float64).reshape(2, 7)
            self.signs = np.asarray(signs, dtype=np.float64).reshape(7)
            if not np.all(np.abs(self.signs) == 1.):
                raise ValueError("joint signs must be +1 or -1")

    def legacy(self, side, arm):
        """Library angles of 7-joint arm values of side 0 (left) or 1 (right)."""
        if self.identity:
            return np.asarray(arm, dtype=np.float64)
        return wrap(self.offsets[side] + self.signs * np.asarray(arm, dtype=np.float64))

    def from_legacy(self, side, arm):
        if self.identity:
            return np.asarray(arm, dtype=np.float64)
        return wrap((np.asarray(arm, dtype=np.float64) - self.offsets[side]) * self.signs)

    def branch(self, side, arm):
        return branch(arm if self.identity else self.legacy(side, arm))


LEGACY_FRAME = JointFrame()


def edge_points(first, last, weights, resolution):
    """Points along straight state edges; every edge gets the same horizon, padded with its end."""
    first, last = np.asarray(first, dtype=np.float64), np.asarray(last, dtype=np.float64)
    steps = np.maximum(np.ceil(np.max(np.abs((last - first) * weights), axis=1) / resolution), 1).astype(int)
    fractions = np.minimum(np.arange(int(steps.max()) + 1)[None, :] / steps[:, None], 1.)
    return first[:, None, :] + fractions[:, :, None] * (last - first)[:, None, :]


def refine(waypoints, weights, resolution):
    """Insert states so that no weighted step along the waypoint polyline exceeds `resolution`."""
    states = [np.asarray(waypoints[0], dtype=np.float64)]
    for target in np.asarray(waypoints[1:], dtype=np.float64):
        base = states[-1]
        count = max(1, math.ceil(np.max(np.abs((target - base) * weights)) / resolution))
        states.extend(base + (target - base) * alpha for alpha in np.linspace(1 / count, 1, count))
    return np.asarray(states)


class CarriageModel:
    """arm_carriage pose as a closed-form translation of updown, checked against the planner's URDF FK."""

    def __init__(self, planner, reference):
        row = np.asarray(reference, dtype=np.float64).copy()

        def carriage(updown):
            row[0] = updown
            planner.fk(row)
            return np.asarray(planner.fk_robot.get_transform("arm_carriage", planner.fk_robot.base_link), dtype=np.float64)

        zero, low = carriage(0.), carriage(-.5)
        self.rotation, self.origin = zero[:3, :3], zero[:3, 3]
        self.axis = (low[:3, 3] - zero[:3, 3]) / -.5
        for updown in (-1., -.3, -.05):
            if np.max(np.abs(carriage(updown) - self.matrix(np.array([updown]))[0])) > 1e-9:
                raise RuntimeError("arm_carriage is not a pure translation of updown")

    def matrix(self, updown):
        output = np.repeat(np.eye(4)[None], len(updown), axis=0)
        output[:, :3, :3] = self.rotation
        output[:, :3, 3] = self.origin + np.asarray(updown)[:, None] * self.axis
        return output

    def inverse(self, updown):
        output = np.repeat(np.eye(4)[None], len(updown), axis=0)
        output[:, :3, :3] = self.rotation.T
        output[:, :3, 3] = -(self.origin + np.asarray(updown)[:, None] * self.axis) @ self.rotation
        return output
