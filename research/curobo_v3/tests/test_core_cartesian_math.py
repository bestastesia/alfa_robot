import math
import sys
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from v3_cartesian_math import METRE, CarriageModel, branch, edge_points, refine, wrap


class StubRobot:
    base_link = 'base_link'

    def __init__(self, tilt_per_metre=0.):
        self.updown = 0.
        self.tilt_per_metre = tilt_per_metre

    def get_transform(self, frame, base):
        matrix = np.eye(4)
        angle = self.tilt_per_metre * self.updown
        matrix[:3, :3] = [[1, 0, 0], [0, math.cos(angle), -math.sin(angle)], [0, math.sin(angle), math.cos(angle)]]
        matrix[:3, :3] = matrix[:3, :3] @ [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        matrix[:3, 3] = [.174, 0., 1.68 - self.updown]
        return matrix


class StubPlanner:
    def __init__(self, tilt_per_metre=0.):
        self.fk_robot = StubRobot(tilt_per_metre)

    def fk(self, row):
        self.fk_robot.updown = float(row[0])


class CartesianMathTest(unittest.TestCase):
    def test_branch_uses_signs_of_q2_q4_q6(self):
        self.assertEqual(branch([9, .2, 9, -.3, 9, .4, 9]), (1, -1, 1))
        self.assertEqual(branch([0, -.2, 0, .3, 0, -.4, 0]), (-1, 1, -1))

    def test_wrap_returns_the_shortest_equivalent_angle(self):
        self.assertAlmostEqual(wrap(math.radians(200)), math.radians(-160))
        self.assertAlmostEqual(wrap(math.radians(-190)), math.radians(170))
        self.assertAlmostEqual(wrap(.3), .3)

    def test_edge_points_hit_both_ends_and_respect_the_weighted_resolution(self):
        weights = np.array([METRE] * 3 + [.8, METRE, 1.])
        first = np.array([[0, 0, 0, 0, -.1, 0], [.1, .2, .3, .5, -.2, 1.]])
        last = np.array([[.5, 0, 0, 1, -.3, 2.], [.1, .2, .3, .5, -.2, 1.]])
        points = edge_points(first, last, weights, .05)
        np.testing.assert_allclose(points[:, 0], first)
        np.testing.assert_allclose(points[:, -1], last)
        steps = np.abs(np.diff(points, axis=1) * weights).max(axis=2)
        self.assertLessEqual(float(steps.max()), .05 + 1e-12)

    def test_refine_keeps_waypoints_and_bounds_each_step(self):
        weights = np.array([METRE] * 3 + [1., METRE, 1.])
        waypoints = [np.zeros(6), np.r_[.2, 0, 0, .5, 0, 1.], np.r_[.2, .3, 0, 1., -.1, 2.]]
        states = refine(waypoints, weights, .01)
        np.testing.assert_allclose(states[0], waypoints[0])
        np.testing.assert_allclose(states[-1], waypoints[-1])
        self.assertTrue(any(np.allclose(state, waypoints[1]) for state in states))
        self.assertLessEqual(float(np.abs(np.diff(states, axis=0) * weights).max()), .01 + 1e-12)

    def test_carriage_model_matches_fk_and_inverts(self):
        model = CarriageModel(StubPlanner(), np.zeros(15))
        planner = StubPlanner()
        for updown in (-.8, -.25, 0.):
            planner.fk(np.r_[updown, np.zeros(14)])
            np.testing.assert_allclose(model.matrix(np.array([updown]))[0],
                                       planner.fk_robot.get_transform('arm_carriage', 'base_link'), atol=1e-12)
        updown = np.array([-.9, -.4, -.05])
        np.testing.assert_allclose(model.inverse(updown) @ model.matrix(updown), np.repeat(np.eye(4)[None], 3, axis=0),
                                   atol=1e-12)

    def test_carriage_model_rejects_a_rotating_carriage(self):
        with self.assertRaises(RuntimeError):
            CarriageModel(StubPlanner(tilt_per_metre=.2), np.zeros(15))


if __name__ == '__main__':
    unittest.main()
