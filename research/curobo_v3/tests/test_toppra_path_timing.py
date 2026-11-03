import importlib.util
import math
import sys
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from curobo_core.trajectory import time_parameterize_path
from v3_path_timing import time_parameterize_toppra

NAMES = ['joint_a', 'joint_b']
LIMITS = (np.array([1., 1.]), np.array([2., 2.]), np.array([20., 20.]))


def curved_path():
    angle = np.linspace(0, math.pi, 200)
    return np.column_stack([np.cos(angle) - 1, np.sin(angle)])


@unittest.skipUnless(importlib.util.find_spec('toppra'), 'requires toppra')
class ToppraPathTimingTest(unittest.TestCase):
    def test_limits_endpoints_and_monotonic_time(self):
        path = curved_path()
        trajectory, validation = time_parameterize_toppra(path, NAMES, *LIMITS, sample_dt=.01)
        sampled = trajectory['sampled_trajectory']
        for field, limit in zip(('velocities', 'accelerations', 'jerks'), LIMITS):
            self.assertLessEqual(float(np.max(np.abs(np.asarray(sampled[field])) / limit)), 1.0001, field)
        time = np.asarray(sampled['time_from_start_s'])
        self.assertTrue(bool(np.all(np.diff(time) > 0)))
        self.assertAlmostEqual(time[-1], trajectory['duration_s'])
        positions = np.asarray(validation, dtype=np.float64)
        np.testing.assert_allclose(positions[[0, -1]], path[[0, -1]], atol=1e-5)
        np.testing.assert_allclose(np.asarray(sampled['velocities'])[[0, -1]], 0., atol=1e-6)

    def test_samples_stay_on_the_geometric_path(self):
        path = curved_path()
        _, validation = time_parameterize_toppra(path, NAMES, *LIMITS, sample_dt=.01)
        radius = np.linalg.norm(np.asarray(validation, dtype=np.float64) - [-1., 0.], axis=1)
        self.assertLess(float(np.max(np.abs(radius - 1.))), 1e-3)

    def test_local_curvature_no_longer_slows_the_whole_path(self):
        # Long straight runs joined by one tight turn: the single global clock must crawl along the
        # whole path to respect the turn, while TOPP-RA only slows down near it.
        arc = np.linspace(-math.pi / 2, 0, 40)
        path = np.vstack([np.column_stack([np.linspace(0, 2.9, 120), np.zeros(120)]),
                          np.column_stack([2.9 + .1 * np.cos(arc), .1 + .1 * np.sin(arc)])[1:],
                          np.column_stack([np.full(120, 3.), np.linspace(.1, 3., 120)])[1:]])
        toppra, _ = time_parameterize_toppra(path, NAMES, *LIMITS, sample_dt=.01)
        cubic, _ = time_parameterize_path(path, NAMES, *LIMITS, sample_dt=.01)
        self.assertLess(toppra['duration_s'], cubic['duration_s'])


if __name__ == '__main__':
    unittest.main()
