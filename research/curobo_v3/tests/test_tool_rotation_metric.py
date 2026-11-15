import sys
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from v3_tool_rotation_metric import rotation_metrics


class RotationMetricTest(unittest.TestCase):
    def score(self,degrees,phases):
        xyzw=Rotation.from_euler('z',degrees,degrees=True).as_quat()
        return rotation_metrics({'left':xyzw[:,[3,0,1,2]]},phases)['left']

    def test_monotonic_necessary_turn_has_no_extra(self):
        self.assertAlmostEqual(self.score([0,20,40],['home','transport','transport'])['extra_deg'],0.)

    def test_backtracking_counts_extra_and_quaternion_sign_is_invariant(self):
        result=self.score([0,40,20],['home','transport','transport'])
        self.assertAlmostEqual(result['extra_deg'],40.)
        self.assertAlmostEqual(result['total_deg'],60.)

    def test_navigation_is_not_counted_as_arm_extra(self):
        self.assertAlmostEqual(self.score([0,180,180],['home','base_reposition','release'])['extra_deg'],0.)


if __name__=='__main__':unittest.main()
