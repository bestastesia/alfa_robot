"""Contact refusal must have a bound, never rely on finite IK search failure."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

import numpy as np
import yourdfpy

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from curobo_core.contact_reach import urdf_reach_model, contact_reach_witnesses
from v3_zero_model_candidate import OFFSETS_DEG, JOINT_SIGNS


def pick(matrix):return NS(contact_tool=lambda:matrix)


class ContactReachTest(unittest.TestCase):
    def test_actual_urdf_witness_never_refuses_known_joint_solutions_and_restores_fk(self):
        robot=yourdfpy.URDF.load(ROOT/'generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf',load_meshes=False)
        rng=np.random.default_rng(20261122)
        original=robot.cfg.copy()
        offsets=np.deg2rad([OFFSETS_DEG['left'],OFFSETS_DEG['right']])
        model=urdf_reach_model(robot,offsets,JOINT_SIGNS)
        np.testing.assert_array_equal(robot.cfg,original)
        for side in ('left','right'):
            self.assertAlmostEqual(model[side]['upper']+model[side]['fore'],.98,places=6)
            self.assertAlmostEqual(np.linalg.norm(model[side]['tool_offset']),.2342,places=6)
        for _ in range(100):
            cfg={name:0. for name in robot.actuated_joint_names}
            cfg['updown']=float(rng.uniform(-1.,0.))
            for side in ('left','right'):
                for i in range(1,8):
                    name=side+'_joint'+str(i);limit=robot.joint_map[name].limit
                    cfg[name]=float(rng.uniform(limit.lower,limit.upper))
            robot.update_cfg(cfg)
            targets={s:pick(robot.get_transform(s+'_tool0',robot.base_link).copy()) for s in ('left','right')}
            self.assertEqual(contact_reach_witnesses(targets,model,(-1.,0.)),[])

    def test_known_asymmetric_contact_targets_are_not_refused(self):
        from experiments.box_layout_20261119.check_asymmetric_layouts import cases
        from v3_box_layout import parse_layout
        robot=yourdfpy.URDF.load(ROOT/'generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf',load_meshes=False)
        model=urdf_reach_model(robot,np.deg2rad([OFFSETS_DEG['left'],OFFSETS_DEG['right']]),JOINT_SIGNS)
        for name,document in cases():
            self.assertEqual(contact_reach_witnesses(parse_layout(document),model,(-1.,0.)),[],name)

    def test_convex_cap_bound_and_pose_envelope_not_a_workspace_cutoff(self):
        arm=dict(shoulder=np.zeros(3),upper=1.,fore=1.,direction=np.array([1.,0.,0.]),
                 tool_offset=np.zeros(3),cosine=0.,geometry_error_m=1e-5)
        model=dict(left=arm,lift_axis=np.array([0.,0.,1.]))
        target=np.eye(4);target[0,3]=-1.5
        witnesses=contact_reach_witnesses({'left':pick(target)},model,(-1.,1.))
        self.assertEqual(witnesses[0]['kind'],'assigned_arm_joint2_reach_bound')
        self.assertGreater(witnesses[0]['unreachable_gap_m'],.4)
        target[0,3]=1.5
        self.assertEqual(contact_reach_witnesses({'left':pick(target)},model,(-1.,1.)),[])
        # A slight nominal excess within the generous error envelope is not a proof.
        target[0,3]=2.001
        self.assertEqual(contact_reach_witnesses({'left':pick(target)},model,(-1.,1.)),[])


if __name__=='__main__':unittest.main()
