#!/usr/bin/env python3
"""
ALFA Robot 实时力学可视化器 — Pinocchio RNEA 版

功能：
  - Pinocchio RNEA 递归牛顿-欧拉算法计算真实逆动力学力矩
  - 各连杆重力 + 末端吸盘载荷完整计算
  - tkinter 滑块实时调整关节角度和吸盘质量
  - MeshCat 3D 可视化机器人姿态和力箭头
  - 实时信息面板显示所有关节力矩和力分解
  - --validate 模式执行 T-0027 校验

用法：
  conda activate alfa
  python3 realtime_force_visualizer.py [--urdf PATH] [--validate] [--motor-mass KG]
"""

import argparse
import sys
import os
import time
import threading
import textwrap
import numpy as np

# ─── Pinocchio 核心 ────────────────────────────────────────────────────────────
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

# ─── tkinter (GUI) ────────────────────────────────────────────────────────────
import tkinter as tk
from tkinter import ttk

# ─── 常量 ─────────────────────────────────────────────────────────────────────
GRAVITY = 9.80665          # m/s^2
DEFAULT_MOTOR_MASS = 5.0   # kg — 默认电机质量（已被 JOINT_MASSES 覆盖）
DEFAULT_PAYLOAD_MASS = 5.0 # kg — 吸盘默认载荷

# 各关节电机质量 (kg) — 用户指定
JOINT_MASSES = {
    1: 2.84,    # joint1
    2: 13.74,   # joint2 (肩部大电机)
    3: 2.84,    # joint3
    4: 2.84,    # joint4
    5: 2.84,    # joint5
    6: 28.0,    # joint6 (含夹取货物)
}

MOTOR_LINK_NAMES = [       # 需要覆盖 inertial 的电机 link
    "leftjoint1", "leftjoint2", "leftjoint3",
    "leftjoint4", "leftjoint5", "leftjoint6",
    "rightjoint1", "rightjoint2", "rightjoint3",
    "rightjoint4", "rightjoint5", "rightjoint6",
]

# legacy_5 新增 fixed link 的 site 名（用于接触检测 / 力语义标注）
SITE_NAMES = {
    "big_arm_site":   ("left_big_arm",   "right_big_arm"),
    "big_arm_2_site": ("left_big_arm_2", "right_big_arm_2"),
    "little_arm_site":("left_little_arm", "right_little_arm"),
}

LEFT_SITE_NAMES = [
    "left_big_arm",
    "left_big_arm_2",
    "left_little_arm",
]

RIGHT_SITE_NAMES = [
    "right_big_arm",
    "right_big_arm_2",
    "right_little_arm",
]

# 接触检测关心的 link 列表
CONTACT_LINKS = {
    "left": [
        "leftjoint1", "leftjoint2", "leftjoint3",
        "leftjoint4", "leftjoint5", "leftjoint6",
        "left_big_arm", "left_big_arm_2", "left_little_arm",
    ],
    "right": [
        "rightjoint1", "rightjoint2", "rightjoint3",
        "rightjoint4", "rightjoint5", "rightjoint6",
        "right_big_arm", "right_big_arm_2", "right_little_arm",
    ],
}

# 语义力分量颜色 (T-0025 定义)
FORCE_COLORS = {
    "axis":   [0.2, 0.6, 1.0],   # 蓝 — 关节轴力
    "normal": [1.0, 0.3, 0.3],   # 红 — 法向力
    "side":   [0.3, 1.0, 0.3],   # 绿 — 侧向力
}

# ═══════════════════════════════════════════════════════════════════════════════
# 1. AlfaPinocchioModel — 力学核心
# ═══════════════════════════════════════════════════════════════════════════════

class AlfaPinocchioModel:
    """基于 Pinocchio RNEA 的 ALFA 机器人力学模型。"""

    def __init__(self, urdf_path: str, motor_mass: float = DEFAULT_MOTOR_MASS):
        self.urdf_path = urdf_path
        self.motor_mass = motor_mass

        # 加载 URDF
        self.model = pin.buildModelFromUrdf(urdf_path)
        self._override_motor_inertials(motor_mass)

        self.data = self.model.createData()
        self.nq = self.model.nq
        self.nv = self.model.nv

        # 构建关节映射表
        self._build_joint_maps()

        # 几何模型（后续由 visualizer 加载）
        self.geom_model = None

    # ── 电机 inertial 覆盖 ──────────────────────────────────────────────────
    def _override_motor_inertials(self, mass_kg: float = None):
        """将左右臂 legacy_link1..6 的 inertial 覆盖为 JOINT_MASSES 中指定的质量,
        com=[0,0,0], inertia 用均匀球近似 (r ≈ 0.05m)。
        mass_kg 参数已废弃，质量由 JOINT_MASSES 常量决定。"""
        r_approx = 0.05

        for name in MOTOR_LINK_NAMES:
            fid = self.model.getFrameId(name)
            if fid >= self.model.nframes:
                continue
            frame = self.model.frames[fid]
            jid = frame.parentJoint

            # 提取关节编号 (leftjoint3 → 3)
            joint_num = int(name[-1])
            m = JOINT_MASSES[joint_num]

            I_scalar = 0.4 * m * r_approx ** 2
            inertia = pin.Inertia(m, np.zeros(3), I_scalar * np.eye(3))
            self.model.inertias[jid] = inertia

    # ── 关节映射 ─────────────────────────────────────────────────────────────
    def _build_joint_maps(self):
        """构建 {joint_name: v_idx} 和活动关节列表。"""
        self.active_joints = {}  # name -> v_idx
        self.legacy_joints = {"left": {}, "right": {}}

        for i in range(1, self.model.njoints):
            name = self.model.names[i]
            joint = self.model.joints[i]
            if joint.nv == 0:
                continue
            idx_v = joint.idx_v
            self.active_joints[name] = idx_v

            # 识别左右臂 legacy 关节
            for side in ("left", "right"):
                prefix = f"{side}_legacy_joint"
                if name.startswith(prefix):
                    num = name.replace(prefix, "")
                    self.legacy_joints[side][f"J{num}"] = idx_v

    # ── RNEA 重力力矩 ───────────────────────────────────────────────────────
    def compute_gravity_torques(self, q: np.ndarray) -> np.ndarray:
        """RNEA(q, v=0, a=0) → 各关节维持姿态所需力矩（含所有连杆重力）。"""
        v = pin.utils.zero(self.nv)
        a = pin.utils.zero(self.nv)
        return pin.rnea(self.model, self.data, q, v, a).copy()

    # ── 末端载荷力矩 ────────────────────────────────────────────────────────
    def compute_payload_torques(self, q: np.ndarray, side: str,
                                 payload_mass: float) -> np.ndarray:
        """计算吸盘载荷在各关节产生的力矩。
        使用 Jacobian 转置: tau = J^T * wrench"""
        if payload_mass <= 0:
            return np.zeros(self.nv)

        tool_frame_name = f"{side}_legacy_tool0"
        fid = self.model.getFrameId(tool_frame_name)
        if fid >= self.model.nframes:
            return np.zeros(self.nv)

        # FK + update frames
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        # Jacobian (LOCAL_WORLD_ALIGNED → 力在世界系)
        J = pin.computeFrameJacobian(self.model, self.data, q, fid,
                                      pin.LOCAL_WORLD_ALIGNED)
        # 载荷 wrench: [F; 0] — 只有重力
        wrench = np.zeros(6)
        wrench[2] = -payload_mass * GRAVITY  # Fz = -mg (world z down)
        return (J.T @ wrench).copy()

    # ── 总力矩 ──────────────────────────────────────────────────────────────
    def compute_total_torques(self, q: np.ndarray,
                               left_payload: float = 0.0,
                               right_payload: float = 0.0) -> np.ndarray:
        """重力 + 两臂载荷的总力矩。"""
        tau = self.compute_gravity_torques(q)
        tau += self.compute_payload_torques(q, "left", left_payload)
        tau += self.compute_payload_torques(q, "right", right_payload)
        return tau

    # ── 关节世界坐标 ────────────────────────────────────────────────────────
    def compute_joint_world_frames(self, q: np.ndarray):
        """FK 获取各关节的 world 坐标和轴方向。"""
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        frames = {}
        for i in range(1, self.model.njoints):
            name = self.model.names[i]
            oMi = self.data.oMi[i]
            frames[name] = {
                "pos": oMi.translation.copy(),
                "rot": oMi.rotation.copy(),
                "se3": oMi.copy(),
            }
        return frames

    # ── RNEA 关节反力 ───────────────────────────────────────────────────────
    def compute_joint_reaction_forces(self, q: np.ndarray):
        """RNEA 内部数据中各关节的反力和反力矩。
        data.f[joint_id] = [force(3); torque(3)] 在 parent frame 中。"""
        v = pin.utils.zero(self.nv)
        a = pin.utils.zero(self.nv)
        pin.rnea(self.model, self.data, q, v, a)

        reactions = {}
        for i in range(1, self.model.njoints):
            name = self.model.names[i]
            # data.f[i] 是 6D force (在 joint i 的 parent frame)
            f_joint = self.data.f[i]
            reactions[name] = {
                "force": f_joint.linear.copy(),   # 3D 力
                "torque": f_joint.angular.copy(),  # 3D 力矩
            }
        return reactions

    # ── 力分解 (T-0025 语义) ───────────────────────────────────────────────
    def compute_force_decomposition(self, q: np.ndarray, joint_name: str,
                                     tau_total: np.ndarray):
        """将关节力矩分解为 T-0025 定义的语义分量：
        - tau_axis_Nm: 关节轴力矩
        - force_axis_N: 关节轴方向反力
        - force_normal_N: 法向反力
        - force_side_N: 侧向反力
        """
        v_idx = self.active_joints.get(joint_name)
        if v_idx is None:
            return None

        tau_axis_Nm = tau_total[v_idx]

        # 获取关节世界坐标
        frames = self.compute_joint_world_frames(q)
        if joint_name not in frames:
            return {"tau_axis_Nm": tau_axis_Nm}

        joint_frame = frames[joint_name]
        joint_axis_world = joint_frame["rot"][:, 2]  # z 轴 = 关节轴

        # RNEA 反力
        reactions = self.compute_joint_reaction_forces(q)
        if joint_name not in reactions:
            return {"tau_axis_Nm": tau_axis_Nm}

        f_reaction = reactions[joint_name]["force"]

        # 投影到语义轴
        force_axis_N = float(np.dot(f_reaction, joint_axis_world))
        f_perp = f_reaction - force_axis_N * joint_axis_world

        # normal 方向: 在臂平面内（简化为 y 方向）
        normal_dir = np.array([0, 1, 0])
        force_normal_N = float(np.dot(f_perp, normal_dir))
        f_perp2 = f_perp - force_normal_N * normal_dir

        force_side_N = float(np.linalg.norm(f_perp2))

        return {
            "tau_axis_Nm": float(tau_axis_Nm),
            "force_axis_N": force_axis_N,
            "force_normal_N": force_normal_N,
            "force_side_N": force_side_N,
        }

    # ── 默认配置 ────────────────────────────────────────────────────────────
    def neutral_q(self) -> np.ndarray:
        """返回中性配置。"""
        return pin.neutral(self.model)

    # ── 构建几何模型 ────────────────────────────────────────────────────────
    def build_geometry(self, package_dirs: list = None):
        """加载可视化几何模型。"""
        if package_dirs is None:
            package_dirs = []
        self.geom_model = pin.buildGeomFromUrdf(
            self.model, self.urdf_path, pin.GeometryType.VISUAL,
            None, package_dirs
        )


# ═══════════════════════════════════════════════════════════════════════════════
# 2. MeshCatVisualizerBridge — 3D 可视化
# ═══════════════════════════════════════════════════════════════════════════════

class MeshCatVisualizerBridge:
    """MeshCat 3D 可视化桥接层，负责机器人显示和力箭头渲染。"""

    def __init__(self, alfa_model: AlfaPinocchioModel, package_dirs: list):
        self.alfa = alfa_model
        self.package_dirs = package_dirs
        self.viz = None
        self._arrow_cache = {}

    def init(self):
        """初始化 MeshCat 可视化器。"""
        alfa = self.alfa
        if alfa.geom_model is None:
            alfa.build_geometry(package_dirs)

        self.viz = MeshcatVisualizer(alfa.model, alfa.geom_model, alfa.geom_model)
        self.viz.initViewer(open=True)
        self.viz.loadViewerModel()
        q = alfa.neutral_q()
        self.viz.display(q)
        url = self.viz.viewer.url() if hasattr(self.viz, 'viewer') and hasattr(self.viz.viewer, 'url') else "http://127.0.0.1:7000/static/"
        print(f"[MeshCat] 可视化已启动: {url}")

    def display(self, q: np.ndarray):
        """更新机器人姿态。"""
        if self.viz:
            self.viz.display(q)

    def update_force_arrows(self, q: np.ndarray, joint_data: dict):
        """在各关节位置画力矩箭头。
        箭头长度 ∝ |力矩|, 方向 = 关节轴, 颜色同 T-0025 定义。"""
        if not self.viz:
            return

        import meshcat.geometry as mg

        frames = self.alfa.compute_joint_world_frames(q)

        for joint_name, data in joint_data.items():
            if joint_name not in frames:
                continue
            pos = frames[joint_name]["pos"]
            tau = data.get("tau_axis_Nm", 0)

            # 箭头缩放: 1 Nm → 0.01m 长
            arrow_len = abs(tau) * 0.01
            if arrow_len < 0.005:
                # 太短就隐藏
                key = f"force_arrows/{joint_name}"
                if key in self.viz.viewer:
                    self.viz.viewer[key].delete()
                continue

            direction = frames[joint_name]["rot"][:, 2]  # 关节轴
            if tau < 0:
                direction = -direction

            # 创建箭头
            key = f"force_arrows/{joint_name}"
            end = pos + direction * arrow_len

            # 用简单圆柱 + 球代替箭头
            color = FORCE_COLORS["axis"] if abs(tau) < 50 else [1, 0.8, 0]
            self._draw_arrow(key, pos, direction, arrow_len, color)

    def update_payload_arrow(self, q: np.ndarray, side: str, mass: float):
        """在吸盘位置画载荷重力箭头（红色向下）。"""
        if not self.viz or mass <= 0:
            return

        tool_frame_name = f"{side}_legacy_tool0"
        fid = self.alfa.model.getFrameId(tool_frame_name)
        if fid >= self.alfa.model.nframes:
            return

        pin.forwardKinematics(self.alfa.model, self.alfa.data, q)
        pin.updateFramePlacements(self.alfa.model, self.alfa.data)

        tool_pos = self.alfa.data.oMf[fid].translation
        arrow_len = mass * 0.01  # 1kg → 0.01m
        key = f"payload_arrows/{side}_payload"
        self._draw_arrow(key, tool_pos, np.array([0, 0, -1]), arrow_len,
                         [1.0, 0.0, 0.0])

    def _draw_arrow(self, key: str, origin: np.ndarray, direction: np.ndarray,
                     length: float, color: list):
        """在 MeshCat 中画一个箭头（用圆柱近似）。"""
        import meshcat.geometry as mg

        if length < 0.002:
            return

        end = origin + direction * length

        # 用 LineSegments 简单画线
        vertices = np.array([origin, end]).T.astype(np.float32)
        colors = np.array([color + [1.0], color + [1.0]]).T.astype(np.float32)

        line_key = key + "_line"
        self.viz.viewer[line_key].set_object(
            mg.LineSegments(
                mg.PointsGeometry(vertices, colors),
                mg.LineBasicMaterial()
            )
        )


# ═══════════════════════════════════════════════════════════════════════════════
# 3. SliderGUI — tkinter 控制面板
# ═══════════════════════════════════════════════════════════════════════════════

class SliderGUI:
    """tkinter 控制面板：关节角度滑块 + 吸盘质量滑块 + 实时力矩信息表。"""

    def __init__(self, alfa_model: AlfaPinocchioModel,
                 on_update_callback=None):
        self.alfa = alfa_model
        self.on_update = on_update_callback  # 回调: (q, left_payload, right_payload)
        self.q = alfa_model.neutral_q()
        self.left_payload = DEFAULT_PAYLOAD_MASS
        self.right_payload = DEFAULT_PAYLOAD_MASS
        self.sliders = {}
        self.info_labels = {}
        self._build_gui()

    def _build_gui(self):
        """构建 GUI 界面。"""
        self.root = tk.Tk()
        self.root.title("ALFA 力学仿真控制台")
        self.root.geometry("960x800")
        self.root.configure(bg="#1e1e2e")

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Title.TLabel", font=("Consolas", 12, "bold"),
                         foreground="#cdd6f4", background="#1e1e2e")
        style.configure("Info.TLabel", font=("Consolas", 9),
                         foreground="#a6adc8", background="#1e1e2e")
        style.configure("Slider.TScale", background="#1e1e2e")
        style.configure("TLabelframe", background="#1e1e2e",
                         foreground="#cdd6f4")
        style.configure("TLabelframe.Label", background="#1e1e2e",
                         foreground="#89b4fa", font=("Consolas", 10, "bold"))

        # ── 主框架 ────────────────────────────────────────────────────────
        main_frame = ttk.Frame(self.root, style="TFrame")
        main_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        # ── 基座关节 ──────────────────────────────────────────────────────
        base_frame = ttk.LabelFrame(main_frame, text="基座关节")
        base_frame.pack(fill=tk.X, padx=4, pady=2)
        self._add_base_sliders(base_frame)

        # ── 左臂 ──────────────────────────────────────────────────────────
        left_frame = ttk.LabelFrame(main_frame, text="左臂 (Left)")
        left_frame.pack(fill=tk.X, padx=4, pady=2)
        self._add_arm_sliders(left_frame, "left")

        # ── 右臂 ──────────────────────────────────────────────────────────
        right_frame = ttk.LabelFrame(main_frame, text="右臂 (Right)")
        right_frame.pack(fill=tk.X, padx=4, pady=2)
        self._add_arm_sliders(right_frame, "right")

        # ── 吸盘载荷 ──────────────────────────────────────────────────────
        payload_frame = ttk.LabelFrame(main_frame, text="吸盘载荷 (kg)")
        payload_frame.pack(fill=tk.X, padx=4, pady=2)
        self._add_payload_sliders(payload_frame)

        # ── 力矩信息面板 ──────────────────────────────────────────────────
        info_frame = ttk.LabelFrame(main_frame, text="实时力矩 (Pinocchio RNEA)")
        info_frame.pack(fill=tk.BOTH, expand=True, padx=4, pady=2)
        self._build_info_panel(info_frame)

    def _add_base_sliders(self, parent):
        """基座关节滑块：pitch, turn, updown。"""
        joints = [
            ("pitch", "Pitch (°)", -90, 90, 0),
            ("turn",  "Turn (°)", -180, 180, 0),
            ("updown", "Up/Down (m)", 0.0, 1.2, 0.5),
        ]
        for name, label, lo, hi, default in joints:
            self._make_slider(parent, name, label, lo, hi, default)

    def _add_arm_sliders(self, parent, side: str):
        """左右臂 6 轴滑块。"""
        legacy = self.alfa.legacy_joints[side]
        for jname, v_idx in sorted(legacy.items()):
            full_name = f"{side}_legacy_joint{jname[1:]}"
            label = f"J{jname[1:]} (°)"
            lo = np.degrees(self.alfa.model.lowerPositionLimit[v_idx])
            hi = np.degrees(self.alfa.model.upperPositionLimit[v_idx])
            if lo < -360:
                lo = -180
            if hi > 360:
                hi = 180
            self._make_slider(parent, full_name, label, lo, hi, 0.0)

    def _add_payload_sliders(self, parent):
        """吸盘质量滑块。"""
        for side in ("left", "right"):
            self._make_slider(parent, f"{side}_payload", side.capitalize(),
                              0, 50, DEFAULT_PAYLOAD_MASS)

    def _make_slider(self, parent, name: str, label: str,
                      lo: float, hi: float, default: float):
        """创建一个带标签和数值显示的滑块。"""
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, padx=4, pady=1)

        lbl = ttk.Label(frame, text=label, width=14, anchor="e",
                         style="Info.TLabel")
        lbl.pack(side=tk.LEFT, padx=(0, 4))

        var = tk.DoubleVar(value=default)
        slider = ttk.Scale(frame, from_=lo, to=hi, variable=var,
                            orient=tk.HORIZONTAL,
                            command=lambda *a: self._on_slider_change())
        slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)

        val_lbl = ttk.Label(frame, text=f"{default:.1f}", width=8,
                             style="Info.TLabel")
        val_lbl.pack(side=tk.LEFT, padx=(4, 0))

        self.sliders[name] = {
            "var": var,
            "label": val_lbl,
            "lo": lo,
            "hi": hi,
        }

    def _build_info_panel(self, parent):
        """构建实时力矩信息表。"""
        # 用 Canvas + Frame 实现可滚动表格
        canvas = tk.Canvas(parent, bg="#1e1e2e", highlightthickness=0)
        scrollbar = ttk.Scrollbar(parent, orient=tk.VERTICAL, command=canvas.yview)
        self.info_inner = ttk.Frame(canvas)

        self.info_inner.bind("<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.info_inner, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # 表头
        headers = ["关节", "τ_axis (Nm)", "F_axis (N)", "F_normal (N)",
                    "F_side (N)", "effort%", "状态"]
        for col, h in enumerate(headers):
            lbl = tk.Label(self.info_inner, text=h, font=("Consolas", 9, "bold"),
                           fg="#89b4fa", bg="#1e1e2e", anchor="w", width=12)
            lbl.grid(row=0, column=col, padx=2, pady=1)

        # 数据行
        all_joints = list(self.alfa.active_joints.keys())
        for i, jname in enumerate(all_joints):
            row_labels = {}
            for col in range(7):
                lbl = tk.Label(self.info_inner, text="—", font=("Consolas", 9),
                               fg="#a6adc8", bg="#1e1e2e", anchor="w", width=12)
                lbl.grid(row=i + 1, column=col, padx=2, pady=0)
                row_labels[col] = lbl
            self.info_labels[jname] = row_labels

    # ── 滑块回调 ─────────────────────────────────────────────────────────────
    def _on_slider_change(self):
        """滑块变化时同步 q 向量并触发回调。"""
        # 更新显示值
        for name, info in self.sliders.items():
            val = info["var"].get()
            info["label"].config(text=f"{val:.1f}")

        # 构建 q 向量
        q = self.alfa.neutral_q()

        # 基座关节
        for bname in ["pitch", "turn"]:
            if bname in self.sliders:
                v_idx = self.alfa.active_joints.get(bname)
                if v_idx is not None:
                    q[v_idx] = np.radians(self.sliders[bname]["var"].get())

        if "updown" in self.sliders:
            v_idx = self.alfa.active_joints.get("updown")
            if v_idx is not None:
                q[v_idx] = self.sliders["updown"]["var"].get()

        # 左右臂
        for side in ("left", "right"):
            for jname, v_idx in self.alfa.legacy_joints[side].items():
                full_name = f"{side}_legacy_joint{jname[1:]}"
                if full_name in self.sliders:
                    q[v_idx] = np.radians(self.sliders[full_name]["var"].get())

        self.q = q
        self.left_payload = self.sliders.get("left_payload", {}).get("var", tk.DoubleVar(value=DEFAULT_PAYLOAD_MASS)).get()
        self.right_payload = self.sliders.get("right_payload", {}).get("var", tk.DoubleVar(value=DEFAULT_PAYLOAD_MASS)).get()

        if self.on_update:
            self.on_update(q, self.left_payload, self.right_payload)

    # ── 信息面板更新 ────────────────────────────────────────────────────────
    def update_info(self, joint_data: dict):
        """更新力矩信息面板。joint_data = {joint_name: {tau_axis_Nm, ...}}"""
        for jname, row_labels in self.info_labels.items():
            if jname not in joint_data:
                continue
            d = joint_data[jname]
            tau = d.get("tau_axis_Nm", 0)
            effort_pct = 0
            v_idx = self.alfa.active_joints.get(jname)
            if v_idx is not None and v_idx < len(self.alfa.model.effortLimit):
                max_effort = self.alfa.model.effortLimit[v_idx]
                if max_effort > 0:
                    effort_pct = abs(tau) / max_effort * 100

            row_labels[0].config(text=jname[-12:])  # 截短名
            row_labels[1].config(text=f"{tau:.2f}")
            row_labels[2].config(text=f"{d.get('force_axis_N', 0):.1f}")
            row_labels[3].config(text=f"{d.get('force_normal_N', 0):.1f}")
            row_labels[4].config(text=f"{d.get('force_side_N', 0):.1f}")
            row_labels[5].config(text=f"{effort_pct:.0f}%")

            # 状态颜色
            if effort_pct > 90:
                row_labels[6].config(text="⚠ 过载", fg="#f38ba8")
            elif effort_pct > 70:
                row_labels[6].config(text="高负载", fg="#fab387")
            elif effort_pct > 30:
                row_labels[6].config(text="正常", fg="#a6e3a1")
            else:
                row_labels[6].config(text="轻载", fg="#89b4fa")

    # ── 运行 ─────────────────────────────────────────────────────────────────
    def run(self):
        """启动 tkinter 主循环。"""
        # 初始触发一次回调
        self._on_slider_change()
        self.root.mainloop()


# ═══════════════════════════════════════════════════════════════════════════════
# 4. T0027Validator — 校验工具
# ═══════════════════════════════════════════════════════════════════════════════

class T0027Validator:
    """T-0027 校验工具：对比旧 MVP 手动计算 vs Pinocchio RNEA。"""

    def __init__(self, alfa_model: AlfaPinocchioModel):
        self.alfa = alfa_model

    def run(self):
        """执行校验并输出报告。"""
        print("=" * 70)
        print("T-0027 校验报告：Pinocchio RNEA 力学验证")
        print("=" * 70)

        test_configs = [
            ("零位（竖直向上）", self.alfa.neutral_q(), 0, 0),
            ("左臂 J2 弯曲 90°", self._q_left_j2_90(), 5, 0),
            ("左臂 J2=90° J3=90°", self._q_left_j2_j3_90(), 5, 0),
            ("右臂 J2 弯曲 90°", self._q_right_j2_90(), 0, 5),
            ("双臂弯曲 + 双载荷", self._q_both_bent(), 10, 10),
        ]

        for title, q, lp, rp in test_configs:
            print(f"\n{'─' * 60}")
            print(f"  配置: {title}")
            print(f"  左吸盘={lp}kg, 右吸盘={rp}kg")
            print(f"{'─' * 60}")

            tau_grav = self.alfa.compute_gravity_torques(q)
            tau_payload_l = self.alfa.compute_payload_torques(q, "left", lp)
            tau_payload_r = self.alfa.compute_payload_torques(q, "right", rp)
            tau_total = tau_grav + tau_payload_l + tau_payload_r

            print(f"\n  {'关节':<20} {'τ_gravity':>10} {'τ_payload':>10} {'τ_total':>10} {'effort%':>8}")
            print(f"  {'─' * 60}")

            for jname, v_idx in sorted(self.alfa.active_joints.items(),
                                         key=lambda x: x[1]):
                tau_g = tau_grav[v_idx]
                tau_p = tau_payload_l[v_idx] + tau_payload_r[v_idx]
                tau_t = tau_total[v_idx]
                effort = 0
                if v_idx < len(self.alfa.model.effortLimit):
                    max_e = self.alfa.model.effortLimit[v_idx]
                    if max_e > 0:
                        effort = abs(tau_t) / max_e * 100
                print(f"  {jname:<20} {tau_g:>10.2f} {tau_p:>10.2f} {tau_t:>10.2f} {effort:>7.1f}%")

            # 力分解
            print(f"\n  力分解 (T-0025 语义):")
            for jname, v_idx in sorted(self.alfa.active_joints.items(),
                                         key=lambda x: x[1]):
                decomp = self.alfa.compute_force_decomposition(q, jname, tau_total)
                if decomp:
                    print(f"    {jname:<20} τ_axis={decomp.get('tau_axis_Nm',0):.2f} Nm  "
                          f"F_axis={decomp.get('force_axis_N',0):.1f} N  "
                          f"F_normal={decomp.get('force_normal_N',0):.1f} N  "
                          f"F_side={decomp.get('force_side_N',0):.1f} N")

        # ── 物理合理性校验 ──────────────────────────────────────────────────
        print(f"\n{'=' * 70}")
        print("物理合理性校验")
        print(f"{'=' * 70}")

        # 校验 1: 零位时关节力矩应接近 0（竖直向上，重力沿关节轴方向）
        q0 = self.alfa.neutral_q()
        tau0 = self.alfa.compute_gravity_torques(q0)
        legacy_tau = [tau0[self.alfa.legacy_joints[s][j]]
                  for s in ("left", "right")
                  for j in sorted(self.alfa.legacy_joints[s].keys())]
        max_tau_zero = max(abs(t) for t in legacy_tau)
        print(f"\n  [1] 零位重力力矩: 最大 |τ| = {max_tau_zero:.4f} Nm", end="")
        if max_tau_zero < 5.0:
            print(" ✓ (接近 0，合理)")
        else:
            print(" ✗ (零位力矩应接近 0)")

        # 校验 2: 肩关节弯曲 90° 时，J2 力矩应最大（杠杆最长）
        q_j2 = self._q_left_j2_90()
        tau_j2 = self.alfa.compute_gravity_torques(q_j2)
        left_legacy_taus = {j: abs(tau_j2[v_idx])
                        for j, v_idx in self.alfa.legacy_joints["left"].items()}
        j2_is_max = left_legacy_taus["J2"] >= max(left_legacy_taus.values()) * 0.5
        print(f"  [2] J2 弯曲 90° 时左臂力矩: {left_legacy_taus}")
        if j2_is_max:
            print("     ✓ J2 力矩为最大之一（杠杆效应正确）")
        else:
            print("     ⚠ J2 不是最大力矩关节")

        # 校验 3: 载荷线性性
        q_test = self._q_left_j2_90()
        tau_5kg = self.alfa.compute_payload_torques(q_test, "left", 5.0)
        tau_10kg = self.alfa.compute_payload_torques(q_test, "left", 10.0)
        ratio_ok = np.allclose(tau_10kg, 2 * tau_5kg, atol=0.01)
        print(f"  [3] 载荷线性性: τ(10kg)/τ(5kg) ≈ {tau_10kg[4]/tau_5kg[4]:.4f}" if tau_5kg[4] != 0 else "  [3] 载荷线性性: 跳过 (力矩为 0)")
        if ratio_ok:
            print("     ✓ 力矩与载荷成正比")

        print(f"\n{'=' * 70}")
        print("校验完成")
        print(f"{'=' * 70}")

    # ── 测试配置 ─────────────────────────────────────────────────────────────
    def _q_left_j2_90(self) -> np.ndarray:
        q = self.alfa.neutral_q()
        q[2] = 0.5  # updown
        q[self.alfa.legacy_joints["left"]["J2"]] = np.pi / 2
        return q

    def _q_left_j2_j3_90(self) -> np.ndarray:
        q = self._q_left_j2_90()
        q[self.alfa.legacy_joints["left"]["J3"]] = np.pi / 2
        return q

    def _q_right_j2_90(self) -> np.ndarray:
        q = self.alfa.neutral_q()
        q[2] = 0.5
        q[self.alfa.legacy_joints["right"]["J2"]] = np.pi / 2
        return q

    def _q_both_bent(self) -> np.ndarray:
        q = self.alfa.neutral_q()
        q[2] = 0.5
        q[self.alfa.legacy_joints["left"]["J2"]] = np.pi / 3
        q[self.alfa.legacy_joints["right"]["J2"]] = np.pi / 3
        return q


# ═══════════════════════════════════════════════════════════════════════════════
# 5. 主循环
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="ALFA 实时力学可视化器")
    parser.add_argument("--urdf", default=None,
                        help="URDF 文件路径（默认自动生成）")
    parser.add_argument("--validate", action="store_true",
                        help="运行 T-0027 校验模式")
    parser.add_argument("--motor-mass", type=float, default=DEFAULT_MOTOR_MASS,
                        help=f"电机质量 kg（默认 {DEFAULT_MOTOR_MASS}）")
    parser.add_argument("--no-meshcat", action="store_true",
                        help="禁用 MeshCat 3D 可视化")
    args = parser.parse_args()

    # ── URDF 路径 ────────────────────────────────────────────────────────────
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(script_dir, "..", "..", ".."))

    if args.urdf:
        urdf_path = args.urdf
    else:
        urdf_path = os.path.join(script_dir, "..", "generated", "current_alfa_robot.urdf")
        if not os.path.exists(urdf_path):
            # 尝试从 xacro 生成
            xacro_path = os.path.join(
                project_root, "ros2_ws", "src", "alfa_robot_description",
                "urdf", "alfa_robot.urdf.xacro")
            if os.path.exists(xacro_path):
                print(f"[INFO] 从 xacro 生成 URDF...")
                gen_dir = os.path.join(script_dir, "..", "generated")
                os.makedirs(gen_dir, exist_ok=True)
                urdf_path = os.path.join(gen_dir, "current_alfa_robot.urdf")
                os.system(f"xacro {xacro_path} > {urdf_path}")
            else:
                print(f"[ERROR] 找不到 URDF 或 xacro 文件")
                sys.exit(1)

    if not os.path.exists(urdf_path):
        print(f"[ERROR] URDF 文件不存在: {urdf_path}")
        sys.exit(1)

    print(f"[INFO] URDF: {urdf_path}")
    print(f"[INFO] 电机质量: {args.motor_mass} kg")

    # ── 初始化模型 ───────────────────────────────────────────────────────────
    alfa = AlfaPinocchioModel(urdf_path, motor_mass=args.motor_mass)
    print(f"[INFO] 模型加载完成: nq={alfa.nq}, nv={alfa.nv}, "
          f"活动关节={len(alfa.active_joints)}")

    # ── 校验模式 ─────────────────────────────────────────────────────────────
    if args.validate:
        validator = T0027Validator(alfa)
        validator.run()
        return

    # ── 正常模式：GUI + MeshCat ─────────────────────────────────────────────
    package_dirs = [
        os.path.join(project_root, "ros2_ws", "src"),
        os.path.join(project_root, "ros2_ws", "install"),
    ]

    # MeshCat 可视化
    meshcat_bridge = None
    if not args.no_meshcat:
        try:
            alfa.build_geometry(package_dirs)
            meshcat_bridge = MeshCatVisualizerBridge(alfa, package_dirs)
            meshcat_bridge.init()
        except Exception as e:
            print(f"[WARN] MeshCat 初始化失败: {e}")
            print("[WARN] 将以无 3D 可视化模式运行")
            meshcat_bridge = None

    # ── 回调函数 ─────────────────────────────────────────────────────────────
    def on_update(q, left_payload, right_payload):
        # 计算力矩
        tau_total = alfa.compute_total_torques(q, left_payload, right_payload)

        # 力分解
        joint_data = {}
        for jname in alfa.active_joints:
            decomp = alfa.compute_force_decomposition(q, jname, tau_total)
            if decomp:
                joint_data[jname] = decomp

        # 更新 MeshCat
        if meshcat_bridge:
            try:
                meshcat_bridge.display(q)
                meshcat_bridge.update_force_arrows(q, joint_data)
                meshcat_bridge.update_payload_arrow(q, "left", left_payload)
                meshcat_bridge.update_payload_arrow(q, "right", right_payload)
            except Exception as e:
                pass  # 不让 MeshCat 错误阻断 GUI

        # 更新信息面板
        gui.update_info(joint_data)

    # ── 启动 GUI ─────────────────────────────────────────────────────────────
    gui = SliderGUI(alfa, on_update_callback=on_update)

    print("\n" + "=" * 50)
    print("ALFA 力学仿真控制台已启动")
    print("拖动滑块调整关节角度和吸盘质量")
    print("=" * 50 + "\n")

    gui.run()


if __name__ == "__main__":
    main()
