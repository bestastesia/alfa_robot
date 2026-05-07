"""Launch MuJoCo interactive viewer for the ALFA robot scene."""
import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path("scene_robot_only.xml")
d = mujoco.MjData(m)
mujoco.mj_resetData(m, d)
mujoco.mj_forward(m, d)

with mujoco.viewer.launch_passive(m, d) as viewer:
    viewer.cam.distance = 4.0
    viewer.cam.elevation = -20
    viewer.cam.azimuth = 135
    viewer.cam.lookat[:] = [2.0, 0.0, 0.8]

    while viewer.is_running():
        mujoco.mj_step(m, d)
        viewer.sync()
