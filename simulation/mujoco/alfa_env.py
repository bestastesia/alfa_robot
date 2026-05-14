from pathlib import Path

import mujoco
import mujoco.viewer
try:
    from alfa_interface import AlfaRobotInterface
except ImportError:
    from .alfa_interface import AlfaRobotInterface


MOVEIT_INITIAL_POSITIONS = (
    Path(__file__).resolve().parents[2]
    / "ros2_ws/src/alfa_robot_moveit_config/config/initial_positions.yaml"
)
BASE_INITIAL_POSITIONS = {
    "base_x": 0.0,
    "base_y": 0.0,
    "base_yaw": 0.0,
}


def load_moveit_initial_positions(path=None):
    initial_positions = dict(BASE_INITIAL_POSITIONS)
    initial_positions_path = Path(path) if path is not None else MOVEIT_INITIAL_POSITIONS
    if not initial_positions_path.exists():
        return initial_positions

    in_initial_positions = False
    for raw_line in initial_positions_path.read_text().splitlines():
        line_without_comment = raw_line.split("#", 1)[0].rstrip()
        if not line_without_comment.strip():
            continue

        stripped = line_without_comment.strip()
        if stripped == "initial_positions:":
            in_initial_positions = True
            continue

        if not in_initial_positions:
            continue
        if raw_line and not raw_line[0].isspace():
            break
        if ":" not in stripped:
            continue

        name, value = stripped.split(":", 1)
        value = value.strip()
        if value:
            initial_positions[name.strip()] = float(value)

    return initial_positions


class AlfaEnv:
    def __init__(self, model_path="scene.xml", sim_dt=0.002, frame_skip=10, initial_positions_path=None):
        model_path = Path(model_path)
        if not model_path.is_absolute():
            model_path = Path(__file__).resolve().parent / model_path

        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.model.opt.timestep = sim_dt
        self.data = mujoco.MjData(self.model)

        self.sim_dt = sim_dt
        self.frame_skip = frame_skip
        self.control_dt = sim_dt * frame_skip
        self.initial_positions = load_moveit_initial_positions(initial_positions_path)

        self.robot = AlfaRobotInterface(self.model, self.data)
        self.viewer = None

    def reset(self, initial_positions=None):
        mujoco.mj_resetData(self.model, self.data)
        target_positions = dict(self.initial_positions)
        if initial_positions:
            target_positions.update(initial_positions)
        self.robot.apply_hybrid_target(target_positions)
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
