"""
可视化碰撞体积: 红色半透=碰撞体, 灰色半透=视觉体
"""
import numpy as np, mujoco, mujoco.viewer
from src.alfa_interface_v5 import AlfaRobotInterfaceV5

m = mujoco.MjModel.from_xml_path("scene_v5.xml")
d = mujoco.MjData(m)
mujoco.mj_resetData(m, d)
r = AlfaRobotInterfaceV5(m, d)

# 保存原始rgba
orig_rgba = {}
for i in range(m.ngeom):
    orig_rgba[i] = m.geom_rgba[i].copy()

# 碰撞体(contype=1) → 红色半透, 视觉体(contype=0) → 灰色半透
for i in range(m.ngeom):
    if m.geom_contype[i] == 1:
        m.geom_rgba[i] = [1.0, 0.2, 0.2, 0.6]  # 红色半透 = 碰撞体
    else:
        m.geom_rgba[i][3] = 0.2  # 其他半透

# 高亮吸盘碰撞体
for side in ["left", "right"]:
    gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{side}_suction_col")
    m.geom_rgba[gid] = [1.0, 0.0, 1.0, 0.9]  # 品红 = 吸盘碰撞体

# 箱子碰撞体用黄色
for box_prefix in ["box_r0", "box_r1", "box_r2"]:
    for geom_name in [f"{box_prefix}_c0_l0", f"{box_prefix}_c1_l0",
                      f"{box_prefix}_c0_l1", f"{box_prefix}_c1_l1",
                      f"{box_prefix}_c0_l2", f"{box_prefix}_c1_l2"]:
        gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, geom_name) if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, geom_name) >= 0 else -1
        # 直接用body的geom
    # 用body来找geom
for i in range(m.ngeom):
    body_id = m.geom_bodyid[i]
    body_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
    if body_name.startswith("box_") and m.geom_contype[i] == 1:
        m.geom_rgba[i] = [1.0, 0.8, 0.0, 0.7]  # 黄色半透 = 箱子碰撞体

# 容器碰撞体用青色
for i in range(m.ngeom):
    body_id = m.geom_bodyid[i]
    body_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
    if body_name == "container" and m.geom_contype[i] == 1:
        m.geom_rgba[i] = [0.0, 1.0, 1.0, 0.4]  # 青色半透

# Init
r.apply_hybrid_target(dict(
    base_x=0, base_y=0, base_yaw=0, pitch=0, turn=0, updown=0.3,
    leftjoint1=0, leftjoint2=0, leftjoint3=0, leftjoint4=-np.pi/2, leftjoint5=0, leftjoint6=0,
    rightjoint1=0, rightjoint2=0, rightjoint3=0, rightjoint4=-np.pi/2, rightjoint5=0, rightjoint6=0,
    right_suction=0, left_suction=0))
mujoco.mj_forward(m, d)
for _ in range(200): mujoco.mj_step(m, d)
mujoco.mj_forward(m, d)

print("图例:")
print("  品红 = 吸盘碰撞体 (left/right_suction_col)")
print("  黄色 = 箱子碰撞体")
print("  红色 = 机械臂碰撞体")
print("  青色 = 集装箱碰撞体")
print("  灰色透明 = 视觉/无碰撞体")
print("\n关闭窗口退出...")

with mujoco.viewer.launch_passive(m, d) as viewer:
    viewer.cam.distance = 4.5
    viewer.cam.elevation = -20
    viewer.cam.lookat[:] = [1.5, 0, 0.8]
    while viewer.is_running():
        viewer.sync()
