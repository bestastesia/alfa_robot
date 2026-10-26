import sys
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from v3_contact_roll_policy import TOOL_ROLL
from v3_tcp_scene_contract import GraspInput, reconstruct_scene, tool_to_box


class RezeroContractTest(unittest.TestCase):
    def test_aligned_tcp_keeps_real_box_pose_and_scene(self):
        rng=np.random.default_rng(20261020)
        for mode in ('side','top'):
            for _trial in range(30):
                base=np.eye(4)
                base[:3,:3]=Rotation.from_euler('z',rng.uniform(-5,5),degrees=True).as_matrix()
                base[:2,3]=rng.uniform(-.03,.03,2)
                box=np.eye(4)
                box[:3,:3]=Rotation.from_euler('xyz',rng.uniform(-2,2,3),degrees=True).as_matrix()
                box[:3,3]=[1.2,.2,1.4]
                attachment=TOOL_ROLL@tool_to_box(mode)
                tcp=np.linalg.inv(base)@box@np.linalg.inv(attachment)
                np.testing.assert_allclose(base@tcp@attachment,box,atol=1e-12)
                generated=reconstruct_scene((GraspInput('left',mode,tcp@TOOL_ROLL),),base)
                np.testing.assert_allclose(generated['boxes']['left'],box,atol=1e-12)
                np.testing.assert_allclose(base[:3,:3]@generated['outward_footprint'],[-1,0,0],atol=1e-12)


if __name__=='__main__':unittest.main()
