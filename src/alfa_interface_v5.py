import numpy as np
import mujoco

class AlfaRobotInterfaceV5:
    """
    Alfa 机器人混合控制底层接口 (v5 — 新臂结构适配)
    躯干: base_link → pitch (Y hinge) → turn (Z hinge) → updown (Z slide)
    左臂: leftjoint1-6 (6 DOF, 全部铰链)
    右臂: rightjoint1-6 (6 DOF, 全部铰链)
    v5 移除: plate, leftarmbase, rightarmbase, camera, lidar
    v5 新增: pitch 关节
    """
    def __init__(self, model, data):
        self.model = model
        self.data = data

        self.joint_names = [
            # 躯干
            "pitch", "turn", "updown",
            # 右臂 6 DOF
            "rightjoint1", "rightjoint2", "rightjoint3",
            "rightjoint4", "rightjoint5", "rightjoint6",
            # 左臂 6 DOF
            "leftjoint1", "leftjoint2", "leftjoint3",
            "leftjoint4", "leftjoint5", "leftjoint6",
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

        # 传感器地址缓存
        self._sensor_adrs = {}
        self._sensor_dims = {}
        sensor_names = [
            "s_left_force", "s_left_torque",
            "s_right_force", "s_right_torque",
            "s_left_ee_pos", "s_left_suction_normal", "s_left_suction_touch",
        ]
        for sname in sensor_names:
            sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, sname)
            if sid >= 0:
                self._sensor_adrs[sname] = self.model.sensor_adr[sid]
                self._sensor_dims[sname] = self.model.sensor_dim[sid]

    def apply_hybrid_target(self, target_dict: dict):
        for name, target in target_dict.items():
            act_id = self.actuator_ids.get(name, -1)
            q_adr = self.jnt_qpos_adrs.get(name, -1)
            v_adr = self.jnt_qvel_adrs.get(name, -1)

            if q_adr >= 0:
                self.data.qpos[q_adr] = target
            if v_adr >= 0:
                self.data.qvel[v_adr] = 0.0
            if act_id >= 0:
                self.data.ctrl[act_id] = target

    def get_all_joint_positions(self) -> dict:
        obs = {}
        for jname in self.joint_names:
            adr = self.jnt_qpos_adrs.get(jname, -1)
            if adr >= 0:
                obs[jname] = self.data.qpos[adr]
        return obs

    def _read_sensor(self, name):
        adr = self._sensor_adrs.get(name, -1)
        dim = self._sensor_dims.get(name, 0)
        if adr < 0:
            return None
        return self.data.sensordata[adr:adr + dim].copy()

    def get_left_force(self) -> np.ndarray:
        return self._read_sensor("s_left_force")

    def get_left_torque(self) -> np.ndarray:
        return self._read_sensor("s_left_torque")

    def get_left_ft(self) -> dict:
        return {
            "force": self.get_left_force(),
            "torque": self.get_left_torque(),
        }

    def get_left_ee_pos(self) -> np.ndarray:
        return self._read_sensor("s_left_ee_pos")

    def get_left_suction_touch(self) -> float:
        val = self._read_sensor("s_left_suction_touch")
        return float(val[0]) if val is not None else 0.0
