"""CPU-only tests for per-stage t2 calibration math."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from v3_t2_calibration import calibrate, round_up


class T2CalibrationTest(unittest.TestCase):
    def test_signature_bound_and_margin_round_up(self):
        rows = [
            {"wall_s": .30, "spans": {"a": .10, "b": .15}, "residual_s": .05},
            {"wall_s": .28, "spans": {"a": .12, "b": .13}, "residual_s": .03},
            {"wall_s": .20, "spans": {"a": .18}, "residual_s": .02},
        ]
        result = calibrate(rows, margin=.25, quantum=.05)
        self.assertAlmostEqual(result["modeled_branch_bound_s"], .32)
        self.assertAlmostEqual(result["candidate_t2_s"], .4)
        self.assertAlmostEqual(round_up(.401, .05), .45)


if __name__ == "__main__":
    unittest.main()
