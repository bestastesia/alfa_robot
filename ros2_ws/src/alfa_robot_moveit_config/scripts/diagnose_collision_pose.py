#!/usr/bin/python3
"""渲染碰撞误报前 MoveIt 实际收到的关节姿态,并数值分析右臂 link 间距。

用途:诊断 right_joint1/2/3<->right_joint6 自碰撞误报到底是
(a) 姿态本身折叠导致网格真的贴近,还是 (b) 关节正负号认知错误导致姿态是镜像/反向的。
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path("/mnt/mydisk/ALFA/alfa_robot")
RERUN_PKG = REPO / "ros2_ws/src/alfa_robot_rerun"
if str(RERUN_PKG) not in sys.path:
    sys.path.insert(0, str(RERUN_PKG))

import numpy as np
from alfa_robot_rerun import visualize_rerun as vr

# MoveIt 碰撞节点实际收到的姿态(/robot_motion/model_joint_states,报自碰撞的那一帧)
POSE = {
    "updown": 0.23289, "turn": -0.00048, "pitch": 0.0,
    "left_joint1": 0.00038, "left_joint2": -0.78748, "left_joint3": -2.11524,
    "left_joint4": -1.30916, "left_joint5": -0.00018, "left_joint6": 7e-05,
    "right_joint1": -0.00042, "right_joint2": 0.78708, "right_joint3": 2.11644,
    "right_joint4": 1.30811, "right_joint5": -0.00026, "right_joint6": 0.0023,
}


def link_origin(fk: dict, name: str):
    tf = fk.get(name)
    if tf is None:
        return None
    return np.array([tf[0, 3], tf[1, 3], tf[2, 3]])


def main() -> int:
    save_path = Path(sys.argv[1]) if len(sys.argv) > 1 else REPO / "data/test_reports/collision_pose.rrd"
    save_path.parent.mkdir(parents=True, exist_ok=True)

    robot = vr.UrdfRobot(vr.render_current_urdf())
    fk = robot.fk(POSE)

    # 数值分析:右臂各 link 原点两两距离,重点看 right_joint6 与 right_joint1/2/3
    print("=== 右臂 link 原点(base 系, m) ===")
    right_links = ["right_joint1", "right_joint2", "right_joint3", "right_joint4",
                   "right_joint5", "right_joint6", "right_tool0"]
    origins = {}
    for ln in right_links:
        o = link_origin(fk, ln)
        origins[ln] = o
        if o is not None:
            print(f"  {ln:14s} = [{o[0]:+.4f}, {o[1]:+.4f}, {o[2]:+.4f}]")
    print("\n=== right_joint6 与 right_joint1/2/3 原点间距(m) ===")
    j6 = origins.get("right_joint6")
    for ln in ["right_joint1", "right_joint2", "right_joint3"]:
        o = origins.get(ln)
        if j6 is not None and o is not None:
            print(f"  |right_joint6 - {ln}| = {np.linalg.norm(j6 - o):.4f}")

    # 对称对照:左臂(镜像姿态)同样的间距,看是否一致(判断是否单纯折叠)
    print("\n=== 左臂对照 left_joint6 与 left_joint1/2/3 间距(m) ===")
    l6 = link_origin(fk, "left_joint6")
    for ln in ["left_joint1", "left_joint2", "left_joint3"]:
        o = link_origin(fk, ln)
        if l6 is not None and o is not None:
            print(f"  |left_joint6 - {ln}| = {np.linalg.norm(l6 - o):.4f}")

    # 渲染到 rrd
    import rerun as rr
    rr.init("collision_pose_diagnosis")
    rr.save(str(save_path))
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    vr.log_robot_static_model(robot, "world/robot", log_meshes=True)
    vr.log_robot_state(robot, POSE, "world/robot")
    # 标出被误报的 link 原点
    for ln in ["right_joint1", "right_joint2", "right_joint3", "right_joint6"]:
        o = origins.get(ln)
        if o is not None:
            rr.log(f"world/markers/{ln}",
                   rr.Points3D(positions=[o.tolist()], radii=[0.02],
                               colors=[[255, 60, 60, 255]], labels=[ln]))
    print(f"\n渲染已保存: {save_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
