from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation
import yourdfpy
import torch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from v3_rezero_task_planner import RezeroTaskPlanner, FOLDER
from v3_single_arm_planner import SingleArmPlanner
from v3_task_scene import grasp_transform
from v3_contact_roll_policy import aligned_grasp_transform,aligned_atlas
from v3_box_rotation_cost import BoxRotationCost
import tempfile


class ContactRollTest(unittest.TestCase):
    def test_orientation_comparison_normalizes_float_fk_quaternions(self):
        metric=BoxRotationCost.__new__(BoxRotationCost)
        metric.joint_names=['left_joint7']
        metric.attachments={'left':torch.tensor([1.,0.,0.,0.])}
        poses={'left_tool0':SimpleNamespace(quaternion=torch.tensor([[.9999998,0.,0.,0.],[1.,0.,0.,0.]]))}
        metric.checker=SimpleNamespace(kinematics=SimpleNamespace(compute_kinematics=lambda _state:SimpleNamespace(tool_poses=SimpleNamespace(to_dict=lambda:poses))))
        actual=metric.orientations(torch.zeros((2,1)))['left']
        torch.testing.assert_close(torch.linalg.vector_norm(actual,dim=-1),torch.ones(2),atol=1e-8,rtol=0.)

    def test_carry_atlas_and_attachment_preserve_actual_box_pose(self):
        model=yourdfpy.URDF.load(FOLDER/'robot_rezero_candidate.urdf',load_meshes=False)
        for top,mode in [(False,'side'),(True,'top')]:
            with tempfile.TemporaryDirectory() as temporary:
                path=aligned_atlas(FOLDER/(mode+'_atlas.npz'),Path(temporary)/'aligned.npz')
                with np.load(path,allow_pickle=False) as stored:
                    aligned=stored['joints']
                    self.assertTrue(all(len(stored[key])==len(aligned) for key in stored.files))
                with np.load(FOLDER/(mode+'_atlas.npz'),allow_pickle=False) as stored:
                    old=stored['joints']
                for row in np.linspace(0,len(old)-1,5,dtype=int):
                    boxes=[]
                    for values,new in [(old[row],False),(aligned[row],True)]:
                        names=['updown']+[f'{side}_joint{index}' for side in ('left','right') for index in range(1,8)]
                        mapping=dict(zip(names,values));model.update_cfg({name:mapping.get(name,0.) for name in model.actuated_joint_names})
                        boxes.append({side:model.get_transform(side+'_tool0',model.base_link)@(aligned_grasp_transform(side,top) if new else grasp_transform(side,top)) for side in ('left','right')})
                    for side in ('left','right'):
                        np.testing.assert_allclose(boxes[0][side],boxes[1][side],atol=1e-6)

    def test_actual_contact_method_does_not_request_half_turn_from_home(self):
        model=yourdfpy.URDF.load(FOLDER/'robot_rezero_candidate.urdf',load_meshes=False)
        for top,limit in [(False,35.),(True,70.)]:
            planner=RezeroTaskPlanner.__new__(RezeroTaskPlanner)
            planner.top_grasp=top
            old=grasp_transform('left',top)
            target={side+'_tool0':dict(position=[1.,0.,1.],quaternion=np.roll(Rotation.from_matrix(old[:3,:3].T).as_quat(),1)) for side in ('left','right')}
            with patch.object(SingleArmPlanner,'contact_poses_for_task',return_value=target):
                actual=planner.contact_poses_for_task(dict(left=18,right=22))
            for side,angles in [('left',[65,-75,20,-90,-180,40,90]),('right',[-65,75,-20,90,0,-40,-90])]:
                values={name:0. for name in model.actuated_joint_names}
                values.update({f'{side}_joint{index}':np.deg2rad(angle) for index,angle in enumerate(angles,1)})
                model.update_cfg(values)
                home=model.get_transform(side+'_tool0',model.base_link)[:3,:3]
                goal=Rotation.from_quat(np.roll(actual[side+'_tool0']['quaternion'],-1)).as_matrix()
                angle=np.rad2deg(Rotation.from_matrix(home.T@goal).magnitude())
                self.assertLess(angle,limit,(side,top,angle))
                np.testing.assert_allclose(goal[:,2],old[:3,:3].T[:,2],atol=1e-10)


if __name__=='__main__':
    unittest.main()
