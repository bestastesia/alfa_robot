import numpy as np
import mujoco

class AlfaRobotInterface:
    """
    Alfa 机器人混合控制底层接口 (current v5 MuJoCo)
    基座: pitch, turn, updown
    双臂: left_v5_joint1-6 / right_v5_joint1-6
    """
    def __init__(self, model, data):
        self.model = model
        self.data = data

        self.joint_names = [
            "base_x", "base_y", "base_yaw",
            "pitch", "turn", "updown",
            "left_v5_joint1", "left_v5_joint2", "left_v5_joint3",
            "left_v5_joint4", "left_v5_joint5", "left_v5_joint6",
            "right_v5_joint1", "right_v5_joint2", "right_v5_joint3",
            "right_v5_joint4", "right_v5_joint5", "right_v5_joint6",
        ]
        self.actuator_ids = {}
        self.jnt_qpos_adrs = {}
        self.jnt_qvel_adrs = {}

        for jname in self.joint_names:
            j_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if j_id >= 0:
                self.jnt_qpos_adrs[jname] = self.model.jnt_qposadr[j_id]
                self.jnt_qvel_adrs[jname] = self.model.jnt_dofadr[j_id]

        for i in range(self.model.nu):
            act_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if act_name:
                self.actuator_ids[act_name] = i
                if act_name.startswith("act_"):
                    self.actuator_ids[act_name[4:]] = i

    def apply_hybrid_target(self, target_dict: dict):
        for name, target in target_dict.items():
            act_id = self.actuator_ids.get(name, -1)
            q_adr  = self.jnt_qpos_adrs.get(name, -1)
            v_adr  = self.jnt_qvel_adrs.get(name, -1)

            if q_adr >= 0: self.data.qpos[q_adr] = target
            if v_adr >= 0: self.data.qvel[v_adr] = 0.0
            if act_id >= 0: self.data.ctrl[act_id] = target

    def get_all_joint_positions(self) -> dict:
        obs = {}
        for jname in self.joint_names:
            adr = self.jnt_qpos_adrs.get(jname, -1)
            if adr >= 0:
                obs[jname] = self.data.qpos[adr]
        return obs
