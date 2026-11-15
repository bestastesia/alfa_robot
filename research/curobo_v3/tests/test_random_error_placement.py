import sys
from pathlib import Path
import unittest
from types import SimpleNamespace

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from v3_random_error_placement import navigation_and_box_goals


class PlacementTest(unittest.TestCase):
    def test_turn_preserves_arm_side_with_canonical_references(self):
        from v3_conveyor_cycle_planner import ConveyorCyclePlanner
        source=SimpleNamespace(conveyor_center_x_m=-2.9,conveyor_marker_x_m=.6,
            conveyor_height_m=.732,box_fit=dict(dimensions_m=[.3,.4,.4]),placement_clearance_m=.15)
        references=ConveyorCyclePlanner.conveyor_reference_targets(source)
        held={}
        for side in ('left','right'):
            pose=np.eye(4);pose[:3,3]=[.55,.425 if side=='left' else -.425,1.2]
            held[side]=pose
        base,goals=navigation_and_box_goals(held,references)
        for side,sign in [('left',1),('right',-1)]:
            self.assertAlmostEqual(goals[side][1,3],sign*.425)
            np.testing.assert_allclose((base@goals[side])[:3,3],references[side]['position'],atol=1e-12)
            self.assertAlmostEqual(goals[side][0,3]-.55,.8)

    def test_inconsistent_parking_is_rejected(self):
        held={side:np.eye(4) for side in ('left','right')}
        references={'left':dict(position=[0,1,1]),'right':dict(position=[0,-1,1])}
        with self.assertRaises(ValueError):navigation_and_box_goals(held,references)


if __name__=='__main__':unittest.main()
