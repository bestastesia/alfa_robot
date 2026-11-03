"""NumPy-only geometry for the MOTION-323 Cartesian transport search (unit-tested without CUDA)."""
import math

import numpy as np

METRE = math.radians(15) / .1  # 10 cm of tool or carriage motion weighs like 15 degrees, as in the baseline


def branch(arm):
    """Analytic IK branch of a 7-joint arm: signs of q2, q4 and q6."""
    return tuple(1 if arm[index] > 0 else -1 for index in (1, 3, 5))


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


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
