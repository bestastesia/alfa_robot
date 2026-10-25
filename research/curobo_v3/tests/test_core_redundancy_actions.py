from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from motion311_learning.redundancy import parameters, normalized, nearest_branch


class RedundancyTest(unittest.TestCase):
    def test_action_roundtrip_and_periodicity(self):
        for action in ([0., -.4, .9], [-1., 0., 0.], [1., .8, -.6]):
            values = parameters(action)
            np.testing.assert_allclose(normalized(values[0], values[1:]), action, atol=1e-12)
        np.testing.assert_allclose(parameters([0., 1., -1.]), parameters([0., -1., 1.]))

    def test_invalid_actions_fail_explicitly(self):
        for action in ([0., 1.1, 0.], [0., 0.], [0., np.nan, 0.]):
            with self.assertRaises(ValueError):
                parameters(action)

    def test_branch_choice_is_independent_of_solution_order(self):
        solutions = np.array([[1.] * 7, [-1.] * 7, [3.] * 7])
        lo, hi = np.full(7, -2.), np.full(7, 2.)
        expected = nearest_branch(solutions, np.zeros(7), lo, hi)
        np.testing.assert_array_equal(expected, -np.ones(7))
        np.testing.assert_array_equal(expected, nearest_branch(solutions[::-1], np.zeros(7), lo, hi))
        self.assertIsNone(nearest_branch([], np.zeros(7), lo, hi))


if __name__ == '__main__':
    unittest.main()
