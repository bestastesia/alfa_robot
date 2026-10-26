import io
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yourdfpy

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from v3_dual_zero_review import ZERO_DEG,load_review,old_to_review
from v3_zero_model_candidate import OLD,legacy_angles,COORDINATE_REVISION
from v3_rezero_analytic import RezeroAnalytic


class DualZeroReviewTest(unittest.TestCase):
    def test_zero_geometry_and_new_positive_j6_axes(self):
        path=Path(__file__).resolve().parents[1]/'generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf'
        definition=ET.parse(OLD)
        for side in ('left','right'):definition.find(f"joint[@name='{side}_tool0_fixed']/origin").set('xyz','0 0 0.191')
        old=yourdfpy.URDF.load(io.BytesIO(ET.tostring(definition.getroot())),load_meshes=False)
        new=load_review(path,False)
        zero={f'{side}_joint{i}':float(value) for side in ('left','right') for i,value in enumerate(legacy_angles(side,np.zeros(7)),1)}
        old.update_cfg(zero);new.update_cfg({name:0. for name in new.actuated_joint_names})
        for link in old.link_map:
            np.testing.assert_allclose(old.get_transform(link,old.base_link),new.get_transform(link,new.base_link),atol=1e-8)
        for side in ('left','right'):
            name=side+'_joint6';joint=new.joint_map[name]
            before=new.get_transform(joint.child,new.base_link).copy()
            axis=before[:3,:3]@joint.axis
            self.assertGreater(axis[2],.999)
            new.update_cfg({name:.001})
            after=new.get_transform(joint.child,new.base_link)
            increment=Rotation.from_matrix(after[:3,:3]@before[:3,:3].T).as_rotvec()
            self.assertAlmostEqual(float(increment@axis),.001,places=9)
            new.update_cfg({name:0.})
        rng=np.random.default_rng(24)
        for _trial in range(20):
            values={name:float(rng.uniform(-.4,.4)) for name in new.actuated_joint_names}
            legacy=dict(values)
            for side in ('left','right'):
                converted=legacy_angles(side,[values[f'{side}_joint{i}'] for i in range(1,8)])
                legacy.update({f'{side}_joint{i}':float(value) for i,value in enumerate(converted,1)})
            old.update_cfg(legacy);new.update_cfg(values)
            for side in ('left','right'):
                np.testing.assert_allclose(old.get_transform(side+'_tool0',old.base_link),new.get_transform(side+'_tool0',new.base_link),atol=1e-8)

    def test_active_new_tcp_analytic_ik_both_arms(self):
        root=Path(__file__).resolve().parents[1]
        model=yourdfpy.URDF.load(root/'generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf',load_meshes=False)
        analytic=RezeroAnalytic();rng=np.random.default_rng(191)
        for arm,side in enumerate(('left','right')):
            for _trial in range(100):
                q=analytic.lower[arm]+rng.uniform(.15,.85,7)*(analytic.upper[arm]-analytic.lower[arm])
                values={name:0. for name in model.actuated_joint_names}
                values.update({f'{side}_joint{i}':float(value) for i,value in enumerate(q,1)})
                model.update_cfg(values)
                inverse=np.linalg.inv(model.get_transform('arm_carriage',model.base_link))
                target=inverse@model.get_transform(side+'_tool0',model.base_link)
                candidates=analytic.solve(arm,target,analytic.swivel(arm,q),q)
                self.assertGreater(len(candidates),0)
                values.update({f'{side}_joint{i}':float(value) for i,value in enumerate(candidates[0],1)})
                model.update_cfg(values)
                actual=inverse@model.get_transform(side+'_tool0',model.base_link)
                self.assertLess(np.linalg.norm(actual[:3,3]-target[:3,3]),2e-6)
                self.assertLess(Rotation.from_matrix(actual[:3,:3].T@target[:3,:3]).magnitude(),2e-6)

    def test_no_duplicate_zero_application(self):
        path=Path(__file__).resolve().parents[1]/'generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf'
        from v3_dual_zero_review import review_definition
        self.assertEqual(ET.tostring(ET.parse(path).getroot()),ET.tostring(review_definition(path).getroot()))

    def test_discrete_offsets_and_equal_heights(self):
        from v3_zero_model_candidate import OFFSETS_DEG
        for side,offsets in OFFSETS_DEG.items():
            for index,angle in enumerate(offsets):
                self.assertAlmostEqual(angle/90,round(angle/90)) if index!=1 else self.assertEqual(abs(angle),15.)
        path=Path(__file__).resolve().parents[1]/'generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf'
        model=yourdfpy.URDF.load(path,load_meshes=False)
        values={name:0. for name in model.actuated_joint_names};values['updown']=-.3
        for side in ('left','right'):values.update({side+'_joint3':-np.pi/2,side+'_joint6':np.pi/2})
        model.update_cfg(values)
        left=model.get_transform('left_tool0',model.base_link)[2,3]
        right=model.get_transform('right_tool0',model.base_link)[2,3]
        self.assertLess(abs(left-right),1e-6)
        for side in ('left','right'):
            joint=model.joint_map[side+'_joint4']
            np.testing.assert_allclose([joint.limit.lower,joint.limit.upper],np.deg2rad([-145,145]),atol=1e-8)


if __name__=='__main__':unittest.main()
