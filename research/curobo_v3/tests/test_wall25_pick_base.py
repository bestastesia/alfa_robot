import sys
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from v3_wall25_demo import pick_base_pose
from v3_tcp_rezero_pipeline import tool_to_box


class WallPickBaseTest(unittest.TestCase):
    def test_existing_top_grasp_advance_and_world_target_invariance(self):
        for task,mode,expected in [({'left':24,'right':20},'side',0.),({'left':9,'right':5},'top',.3),({'left':4,'right':7},'top',.3)]:
            base=pick_base_pose(task)
            self.assertEqual(base[0,3],expected)
            box=np.eye(4);box[:3,3]=[1.5,.82,.61]
            tcp=np.linalg.inv(base)@box@np.linalg.inv(tool_to_box(mode))
            np.testing.assert_allclose(base@tcp@tool_to_box(mode),box,atol=1e-12)


if __name__=='__main__':unittest.main()
