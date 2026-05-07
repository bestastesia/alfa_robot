import mujoco
import mujoco.viewer
from alfa_interface import AlfaRobotInterface

class AlfaEnv:
    def __init__(self, model_path="scene.xml", sim_dt=0.002, frame_skip=10):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.model.opt.timestep = sim_dt
        self.data = mujoco.MjData(self.model)

        self.sim_dt = sim_dt
        self.frame_skip = frame_skip
        self.control_dt = sim_dt * frame_skip

        self.robot = AlfaRobotInterface(self.model, self.data)
        self.viewer = None

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        return self._get_obs()

    def step(self, ctrl_cmds: dict):
        self.robot.apply_hybrid_target(ctrl_cmds)
        return self._get_obs()

    def _get_obs(self):
        return self.robot.get_all_joint_positions()

    def render(self):
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.cam.distance = 3.5
            self.viewer.cam.elevation = -15
            self.viewer.cam.lookat[:] = [0.0, 0, 0.5]
        if self.viewer.is_running():
            self.viewer.sync()
            return True
        return False

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
