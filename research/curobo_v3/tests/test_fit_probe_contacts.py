from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from fit_probe_contacts import ContactKinematics, ROOT, fit, scores


class ContactFitTest(unittest.TestCase):
    def test_linear_tcp_recovery_with_synthetic_same_point_poses(self):
        from scipy.spatial.transform import Rotation
        rng = np.random.default_rng(45)
        rotations = Rotation.random(15, random_state=rng).as_matrix()
        tool = np.array([1.2,-2.3,69.5])
        center = np.array([1250.,970.,1400.])
        frames = np.broadcast_to(np.eye(4),(15,4,4)).copy()
        frames[:,:3,:3] = rotations
        frames[:,:3,3] = (center - rotations @ tool)/1000
        class Model:
            def transforms(self, _degrees):
                return frames
        _offset, actual, _diagnostics = fit(Model(),np.zeros((15,7)),'tcp_only')
        np.testing.assert_allclose(actual,tool,atol=1e-9)

    def test_j1_same_point_cost_invariance(self):
        model = ContactKinematics(ROOT/'generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf',-.3)
        rng = np.random.default_rng(12)
        angles = rng.uniform(-50,50,(10,7))
        initial = scores(model.positions(angles,np.zeros(7),[0,0,70]))['rms_mm']
        offset = np.zeros(7)
        offset[0] = 10
        shifted = scores(model.positions(angles,offset,[0,0,70]))['rms_mm']
        self.assertAlmostEqual(initial,shifted,places=8)


if __name__ == '__main__':
    unittest.main()
