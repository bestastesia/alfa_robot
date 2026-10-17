from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from v3_tcp_scene_contract import GraspInput, reconstruct_scene, random_parking, tool_to_box


class TcpSceneContractTest(unittest.TestCase):
    def test_rotated_mixed_grasps_and_parking(self):
        rng=np.random.default_rng(18)
        for sample in range(500):
            base,yaw=random_parking(rng)
            self.assertLessEqual(np.linalg.norm(base[:2,3]),.05+1e-12)
            self.assertLessEqual(abs(yaw),5.)
            inputs=[]
            truth=[]
            for side,lateral,mode in [('left',.41,'side'),('right',-.41,'top')]:
                box=np.eye(4)
                box[:3,:3]=Rotation.from_euler('xyz',[rng.uniform(-2,2),rng.uniform(-2,2),rng.uniform(-5,5)],degrees=True).as_matrix()
                box[:3,3]=[1.55,lateral,1.02]
                truth.append(box)
                inputs.append(GraspInput(side,mode,np.linalg.inv(base)@box@np.linalg.inv(tool_to_box(mode))))
            scene=reconstruct_scene(inputs,base)
            for grasp,box in zip(inputs,truth):
                np.testing.assert_allclose(scene['boxes'][grasp.side],box,atol=1e-10)
            for fill in scene['fills']:
                lower=np.asarray(fill['center'])-np.asarray(fill['dimensions'])/2
                upper=np.asarray(fill['center'])+np.asarray(fill['dimensions'])/2
                for bounds in scene['bounds'].values():
                    self.assertTrue(np.any(upper<=bounds[0]+1e-10) or np.any(lower>=bounds[1]-1e-10))
            np.testing.assert_allclose(base[:3,:3]@scene['outward_footprint'],[-1,0,0],atol=1e-12)

    def test_two_mm_lateral_opening(self):
        box=np.eye(4)
        box[:3,3]=[1.55,0.,1.]
        grasp=GraspInput('left','side',box@np.linalg.inv(tool_to_box('side')))
        scene=reconstruct_scene([grasp],np.eye(4))
        edges=[]
        for fill in scene['fills']:
            if fill['name'].startswith('working_'):
                edges.extend([fill['center'][1]-fill['dimensions'][1]/2,fill['center'][1]+fill['dimensions'][1]/2])
        self.assertTrue(any(abs(value+.202)<1e-12 for value in edges))
        self.assertTrue(any(abs(value-.202)<1e-12 for value in edges))


if __name__=='__main__':unittest.main()
