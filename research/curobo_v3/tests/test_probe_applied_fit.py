import json
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from fit_probe_contacts import ContactKinematics, scores
from probe_touch_ik import ROOT
from ipc_left_prefix.coordinate_mapping import POLICY, motor_to_model, model_to_motor


class AppliedContactFitTest(unittest.TestCase):
    def test_deployed_policy_matches_offline_candidate_a(self):
        data=json.loads((ROOT/'generated/probe_contact_fit_20261020/fit_results.json').read_text())
        selected=data['fits']['zero_only']
        np.testing.assert_allclose(POLICY['motor_minus_model_deg'],selected['recommended_motor_minus_model_deg'],atol=1e-12)
        raw=np.array(data['input_motor_deg'])
        corrected=np.rad2deg(motor_to_model(np.deg2rad(raw)))
        expected=np.array(data['input_model_deg'])+selected['additional_model_angle_correction_deg']
        np.testing.assert_allclose(corrected,expected,atol=1e-12)
        model=ContactKinematics(ROOT/'generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf',-.3)
        actual=scores(model.positions(corrected,np.zeros(7),[0,0,70]))
        self.assertAlmostEqual(actual['rms_mm'],selected['training']['rms_mm'],places=7)
        np.testing.assert_allclose(np.rad2deg(model_to_motor(np.deg2rad(corrected))),raw,atol=1e-12)


if __name__=='__main__':unittest.main()
