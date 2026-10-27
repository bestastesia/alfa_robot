import sys
from pathlib import Path
import time
import types
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import v3_cartesian_wrist_flip as wrist_flip


class CurveSpace:
    """Stand-in for CartesianTransportSpace whose wrist is singular (q6 = 0) on the curve
    s = 0.4 - 0.5 du, psi = 0.3 + 0.2 s; q6 grows with the distance from it. With `strict`, the
    start branch (q6 > 0) has no IK solution within 1e-9 rad of q6 = 0, as the left arm's analytic IK."""

    arm = slice(8, 15)

    def __init__(self, strict=False):
        self.strict = strict
        self.arm_branch = (1, 1, 1)
        self.start_state = np.array([0., 0., 0., 0., .5, 0.])
        self.goal_states = np.array([[.5, .2, 0., 1., .5, 1.]])
        self.planner = types.SimpleNamespace(
            active_side='right', _timed_joint_names=[f'right_joint{i}' for i in range(1, 8)],
            _timed_limits=(np.r_[[.26] * 4, [.52] * 3], np.r_[[.52] * 4, [1.05] * 3]))

    def joints(self, states):
        s, lift, psi = states[:, 3], states[:, 4], states[:, 5]
        w = np.stack([s - .4 + .5 * (lift - .5), psi - .3 - .2 * s], axis=1)
        radius = np.linalg.norm(w, axis=1)
        q6 = radius * self.arm_branch[2]
        q5 = np.arctan2(w[:, 1], w[:, 0]) + (0. if self.arm_branch[2] > 0 else np.pi)
        q5 = (q5 + np.pi) % (2 * np.pi) - np.pi
        rows = np.zeros((len(states), 15))
        rows[:, 0] = lift
        rows[:, self.arm] = np.stack([psi, s, lift, s * psi, q5, q6, -q5], axis=1)
        found = radius > 1e-9 if self.strict and self.arm_branch[2] > 0 else np.ones(len(states), dtype=bool)
        return rows, found


class WristFlipCrossingTest(unittest.TestCase):
    def crossings(self, strict):
        flip = wrist_flip.WristFlip(CurveSpace(strict), (0., 1.), dict(wrist_flip.DEFAULTS))
        return flip.crossings(time.perf_counter() + 10.)

    def assert_on_curve(self, found):
        self.assertTrue(found)
        for _, (s, psi, du) in found:
            self.assertAlmostEqual(s, .4 - .5 * du, places=6)
            self.assertAlmostEqual(psi, .3 + .2 * s, places=6)

    def test_newton_from_the_line_lands_on_the_singular_curve_cheapest_first(self):
        found = self.crossings(strict=False)
        self.assert_on_curve(found)
        costs = [cost for cost, _ in found]
        self.assertEqual(costs, sorted(costs))

    def test_a_crossing_that_only_the_flipped_branch_solves_exactly_is_kept(self):
        self.assert_on_curve(self.crossings(strict=True))


class RestToRestTest(unittest.TestCase):
    def test_velocity_and_acceleration_limited_moves(self):
        self.assertAlmostEqual(float(wrist_flip.rest_to_rest_s(1., .5, 1.)), 2.5)
        self.assertAlmostEqual(float(wrist_flip.rest_to_rest_s(-.16, .5, 1.)), .8)


if __name__ == '__main__':
    unittest.main()
