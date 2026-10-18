"""CPU-only checks for the time-parameterization comparison report helpers."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from v3_time_parameterization_compare import nearest_path_distance, stats, weighted_arc


class TimeParameterizationCompareTest(unittest.TestCase):
    def test_summary_and_path_metrics_are_deterministic(self):
        self.assertEqual(stats([3., 1., 2.])["median"], 2.)
        path = np.array([[0., 0.], [1., 0.], [1., 1.]])
        self.assertAlmostEqual(weighted_arc(path, np.ones(2)), 2.)
        distance = nearest_path_distance(path, path, np.ones(2), block=1)
        self.assertEqual(distance, {"mean": 0., "max": 0.})


if __name__ == "__main__":
    unittest.main()
