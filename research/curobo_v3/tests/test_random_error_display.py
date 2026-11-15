from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
import yourdfpy

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from v3_tcp_scene_preview import display_robot_urdf


class DisplayBaseTest(unittest.TestCase):
    def test_renderer_applies_navigation_base_exactly_once(self):
        root=Path(__file__).resolve().parents[1]
        fixed=yourdfpy.URDF.load(display_robot_urdf(root),load_meshes=False)
        mobile=yourdfpy.URDF.load(root/'generated/zero_model_candidate_20261018/mobile/robot_rezero_candidate.urdf',load_meshes=False)
        self.assertFalse({'base_x','base_y','base_yaw'} & set(fixed.actuated_joint_names))
        for yaw in (0.,.047,np.pi):
            values={name:0. for name in mobile.actuated_joint_names}
            values.update(base_x=-.1,base_y=.04,base_yaw=yaw,updown=-.3,left_joint2=-.4,left_joint4=-1.)
            mobile.update_cfg(values)
            fixed.update_cfg({name:values[name] for name in fixed.actuated_joint_names})
            base=np.eye(4);base[:3,:3]=Rotation.from_euler('z',yaw).as_matrix();base[:2,3]=[-.1,.04]
            for link in ('base_footprint','left_tool0','right_tool0'):
                np.testing.assert_allclose(base@fixed.get_transform(link,fixed.base_link),mobile.get_transform(link,mobile.base_link),atol=1e-7)


if __name__=='__main__':unittest.main()
