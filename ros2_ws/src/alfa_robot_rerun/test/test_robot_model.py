import math

import numpy as np

from alfa_robot_rerun.visualize_rerun import UrdfRobot


URDF = """
<robot name="fixture">
  <link name="world"/>
  <link name="tool"/>
  <joint name="joint1" type="revolute">
    <parent link="world"/>
    <child link="tool"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
"""


def test_urdf_robot_fk_uses_joint_origin_and_axis():
    robot = UrdfRobot(URDF)
    transforms = robot.fk({"joint1": math.pi / 2.0})

    assert robot.root_link == "world"
    assert np.allclose(transforms["tool"][:3, 3], [1.0, 0.0, 0.0])
    assert np.allclose(
        transforms["tool"][:3, :3],
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
        atol=1e-9,
    )
