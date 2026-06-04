#!/usr/bin/env python3
"""
读取机器人当前状态（末端位姿 + 关节角度），自动写入 JSON 配置文件。

与 set_joints.py / move_to_pose.py 共用同一 JSON 格式，
读取后自动写入，无需手动复制粘贴。

用法:
  python3 get_ee_pose.py                  # 打印 + 写入 robot_config.json
  python3 get_ee_pose.py my_pose.json      # 指定输出文件
  python3 get_ee_pose.py --no-write        # 只打印不写文件
  python3 get_ee_pose.py --append          # 向已有 JSON追加（保留已有字段）

前置条件:
  ros2 launch alfa_robot_moveit_config demo.launch.py
"""

import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tf2_ros

LEFT_TIP = "left_v5_tool0"
RIGHT_TIP = "right_v5_tool0"
BASE_FRAME = "base_link"

ALL_JOINTS = [
    "pitch", "turn", "updown",
    "left_v5_joint1", "left_v5_joint2", "left_v5_joint3",
    "left_v5_joint4", "left_v5_joint5", "left_v5_joint6",
    "right_v5_joint1", "right_v5_joint2", "right_v5_joint3",
    "right_v5_joint4", "right_v5_joint5", "right_v5_joint6",
]


class EEPoseReader(Node):

    def __init__(self):
        super().__init__('ee_pose_reader')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.current_js: dict = {}
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

    def _js_cb(self, msg: JointState):
        for name, val in zip(msg.name, msg.position):
            self.current_js[name] = val

    def get_ee_pose(self, link: str) -> dict | None:
        try:
            t = self.tf_buffer.lookup_transform(
                BASE_FRAME, link, rclpy.time.Time(),
                rclpy.duration.Duration(seconds=2.0))
            return {
                "x": round(t.transform.translation.x, 4),
                "y": round(t.transform.translation.y, 4),
                "z": round(t.transform.translation.z, 4),
                "qx": round(t.transform.rotation.x, 4),
                "qy": round(t.transform.rotation.y, 4),
                "qz": round(t.transform.rotation.z, 4),
                "qw": round(t.transform.rotation.w, 4),
            }
        except Exception as e:
            self.get_logger().warn(f"TF {BASE_FRAME}->{link} 失败: {e}")
            return None

    def get_joints(self) -> dict:
        joints = {}
        for j in ALL_JOINTS:
            if j in self.current_js:
                joints[j] = round(self.current_js[j], 4)
        return joints

    def rpy_from_quat(self, qx, qy, qz, qw):
        sinr = 2 * (qw * qx + qy * qz)
        cosr = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr, cosr)
        sinp = 2 * (qw * qy - qz * qx)
        pitch = math.asin(max(-1, min(1, sinp)))
        siny = 2 * (qw * qz + qx * qy)
        cosy = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny, cosy)
        return [round(math.degrees(a), 2) for a in (roll, pitch, yaw)]


def main():
    output_file = "robot_config.json"
    no_write = "--no-write" in sys.argv
    append = "--append" in sys.argv

    # 解析输出文件名（第一个非 flag 参数）
    for arg in sys.argv[1:]:
        if not arg.startswith("--") and arg.endswith(".json"):
            output_file = arg

    rclpy.init()
    node = EEPoseReader()

    # Spin 让 TF buffer 和 joint_states 填充
    print("等待数据 …")
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.2)

    left = node.get_ee_pose(LEFT_TIP)
    right = node.get_ee_pose(RIGHT_TIP)
    joints = node.get_joints()

    node.destroy_node()
    rclpy.shutdown()

    # 打印
    print("\n" + "=" * 50)
    print("机器人当前状态")
    print("=" * 50)

    if left:
        rpy = node.rpy_from_quat(left["qx"], left["qy"], left["qz"], left["qw"])
        print(f"  左臂末端 ({LEFT_TIP}):")
        print(f"    位置: x={left['x']}  y={left['y']}  z={left['z']}")
        print(f"    姿态: R={rpy[0]}°  P={rpy[1]}°  Y={rpy[2]}°")
        print(f"    四元数: qx={left['qx']} qy={left['qy']} qz={left['qz']} qw={left['qw']}")
    else:
        print(f"  左臂末端: 获取失败")

    if right:
        rpy = node.rpy_from_quat(right["qx"], right["qy"], right["qz"], right["qw"])
        print(f"  右臂末端 ({RIGHT_TIP}):")
        print(f"    位置: x={right['x']}  y={right['y']}  z={right['z']}")
        print(f"    姿态: R={rpy[0]}°  P={rpy[1]}°  Y={rpy[2]}°")
        print(f"    四元数: qx={right['qx']} qy={right['qy']} qz={right['qz']} qw={right['qw']}")
    else:
        print(f"  右臂末端: 获取失败")

    if joints:
        print(f"\n  关节角度 ({len(joints)}/{len(ALL_JOINTS)}):")
        for j in ALL_JOINTS:
            if j in joints:
                v = joints[j]
                unit = "m" if j == "updown" else "rad"
                deg = f"({math.degrees(v):.2f}°)" if j != "updown" else f"({v:.4f}m)"
                print(f"    {j:20s}: {v:.4f} {deg}")

    print("=" * 50)

    if no_write:
        print("  (未写入文件)")
        return

    # 构建输出 JSON
    new_data = {
        "joints": joints,
        "ee_pose": {
            "left": left or {},
            "right": right or {},
        }
    }

    if append and os.path.exists(output_file):
        with open(output_file) as f:
            existing = json.load(f)
        # 更新 joints 和 ee_pose，保留其他字段
        existing["joints"] = new_data["joints"]
        existing["ee_pose"] = new_data["ee_pose"]
        data = existing
    else:
        # 新建完整格式
        data = {
            "description": "机器人位姿配置 — set_joints.py 和 move_to_pose.py 共用",
            "version": "1.0",
            "joints": joints,
            "ee_pose": {
                "left": left or {},
                "right": right or {},
            },
            "meta": {
                "exec_time": 2.0,
                "ik_timeout": 10.0,
                "ik_solver": "kdl"
            }
        }

    with open(output_file, 'w') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

    print(f"\n  已写入: {output_file}")
    print(f"  包含: joints({len(joints)}) + ee_pose")


if __name__ == "__main__":
    main()