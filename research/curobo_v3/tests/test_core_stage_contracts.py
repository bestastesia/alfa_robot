from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from motion311_learning.contracts import Observation, StepResult, potential, reward


class RewardTest(unittest.TestCase):
    def test_complete_return_is_independent_of_segmentation(self):
        value = reward(0., 10.) + reward(10., 25.) + reward(25., 40., True)
        self.assertAlmostEqual(value, 1. - .2 * potential(40.))
        self.assertGreater(reward(0., 1e9, True), reward(0., 0., False))

    def test_failure_is_negative_even_when_early(self):
        self.assertEqual(reward(0., 0., False), -1.)
        with self.assertRaises(ValueError):
            reward(10., 9.)


class ObservationTest(unittest.TestCase):
    def observation(self):
        return Observation(0, np.zeros(50), np.zeros((4, 15)), np.zeros((4, 3)), np.zeros((4, 9)),
                           np.repeat(np.eye(4)[None], 2, 0))

    def test_no_future_fields_and_no_mutation(self):
        obs = self.observation()
        self.assertFalse(hasattr(obs, "next_candidates"))
        with self.assertRaises(ValueError):
            obs.state[0] = 1.
        self.assertEqual(obs.candidate_features(True).shape, (4, 24))

    def test_bad_shapes_are_rejected(self):
        with self.assertRaises(ValueError):
            Observation(0, np.zeros(50), np.zeros((4, 15)), np.zeros((3, 3)), np.zeros((4, 9)),
                        np.repeat(np.eye(4)[None], 2, 0))

    def test_no_bootstrap_state_for_truncation_is_explicit(self):
        StepResult(None, 0., False, True, {"cause": "infrastructure"})
        with self.assertRaises(ValueError):
            StepResult(None, 0., False, False, {})
        with self.assertRaises(ValueError):
            StepResult(None, -1., True, True, {})


if __name__ == "__main__":
    unittest.main()
