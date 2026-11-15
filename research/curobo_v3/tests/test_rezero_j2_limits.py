from pathlib import Path
import sys
import unittest
import json
import yaml
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from v3_zero_model_candidate import OFFSETS_DEG,J2_LIMITS_DEG,legacy_angles,ROOT,OLD


class RezeroJointLimitsTest(unittest.TestCase):
    def test_full_legacy_j2_range_preserved_by_each_arm(self):
        for side in ('left','right'):
            for old_j2 in np.linspace(-105.,105.,101):
                new_j2=OFFSETS_DEG[side][1]-old_j2
                lower,upper=J2_LIMITS_DEG[side]
                self.assertGreaterEqual(new_j2,lower-1e-10)
                self.assertLessEqual(new_j2,upper+1e-10)
                pose=np.zeros(7);pose[1]=np.deg2rad(new_j2)
                self.assertAlmostEqual(np.rad2deg(legacy_angles(side,pose)[1]),old_j2)

    def test_home_and_unloading_references_in_limits(self):
        for side,new_angles in [('left',[105.,-65.,115.,-85.,120.]),('right',[-105.,65.,-115.,85.,-120.])]:
            lower,upper=J2_LIMITS_DEG[side]
            self.assertTrue(all(lower<=value<=upper for value in new_angles))

    def test_selected_home_matches_user_numbers_and_preserved_height(self):
        root=Path(__file__).resolve().parents[1]
        policy=json.loads((root/'config/rezero_home.json').read_text())
        pose=yaml.safe_load((root/'generated/zero_model_candidate_20261018/named_poses.yaml').read_text())['named_poses']['home']
        self.assertEqual(pose['updown'],-.3)
        for side,expected in [('left',[65,-75,20,-90,-180,40,90]),('right',[-65,75,-20,90,0,-40,-90])]:
            self.assertEqual(policy[side+'_deg'],expected)
            np.testing.assert_allclose(np.rad2deg([pose[f'{side}_joint{index}'] for index in range(1,8)]),expected,atol=1e-9)

    def test_both_j6_axes_reverse_without_moving_joint_zero(self):
        old=ET.parse(OLD).getroot()
        new=ET.parse(ROOT/'generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf').getroot()
        for side in ('left','right'):
            before=old.find(f"joint[@name='{side}_joint6']")
            after=new.find(f"joint[@name='{side}_joint6']")
            np.testing.assert_allclose(np.fromstring(after.find('axis').get('xyz'),sep=' '),
                                       -np.fromstring(before.find('axis').get('xyz'),sep=' '))
            for attribute in ('xyz','rpy'):
                np.testing.assert_allclose(np.fromstring(after.find('origin').get(attribute),sep=' '),
                                           np.fromstring(before.find('origin').get(attribute),sep=' '),atol=1e-12)
            np.testing.assert_allclose(legacy_angles(side,np.zeros(7)),np.deg2rad(OFFSETS_DEG[side]))


if __name__=='__main__':unittest.main()
