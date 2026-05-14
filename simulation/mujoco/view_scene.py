"""Launch MuJoCo interactive viewer for the ALFA robot-only scene."""
import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

import mujoco
import mujoco.viewer
from alfa_env import AlfaEnv


env = AlfaEnv(model_path="scene_robot_only.xml")
env.reset()
m = env.model
d = env.data

with mujoco.viewer.launch_passive(m, d) as viewer:
    viewer.cam.distance = 2.8
    viewer.cam.elevation = -20
    viewer.cam.azimuth = 135
    viewer.cam.lookat[:] = [0.45, 0.0, 0.45]

    while viewer.is_running():
        mujoco.mj_step(m, d)
        viewer.sync()
