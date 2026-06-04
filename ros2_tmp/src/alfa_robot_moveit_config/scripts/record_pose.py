#!/usr/bin/env python3
"""
末端位姿记录工具

在 MoveIt demo 环境下运行，拖拽 + Execute 后，按 s 记录当前机器人位姿。
输出 JSON 文件，可直接用于 pick_place_demo.py。

用法:
  ros2 launch alfa_robot_moveit_config demo.launch.py
  python3 record_pose.py

交互菜单:
  s - 保存当前位姿
  p - 打印当前关节角度
  d - 打印已保存的所有记录
  q - 退出并写入 JSON 文件
"""

import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import tf2_ros


ALL_JOINTS = [
    "pitch", "turn", "updown",
    "left_v5_joint1", "left_v5_joint2", "left_v5_joint3",
    "left_v5_joint4", "left_v5_joint5", "left_v5_joint6",
    "right_v5_joint1", "right_v5_joint2", "right_v5_joint3",
    "right_v5_joint4", "right_v5_joint5", "right_v5_joint6",
]

LEFT_TIP = "left_v5_tool0"
RIGHT_TIP = "right_v5_tool0"
BASE_FRAME = "base_link"


def rad_to_deg(v: float) -> float:
    return round(math.degrees(v), 2)


def pose_from_transform(t: TransformStamped) -> dict:
    return {
        "x": t.transform.translation.x,
        "y": t.transform.translation.y,
        "z": t.transform.translation.z,
        "qx": t.transform.rotation.x,
        "qy": t.transform.rotation.y,
        "qz": t.transform.rotation.z,
        "qw": t.transform.rotation.w,
    }


class PoseRecorder(Node):

    def __init__(self):
        super().__init__('pose_recorder')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.current_js: dict = {}
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        self.records: list[dict] = []

        self.get_logger().info("等待关节状态和 TF …")
        deadline = time.time() + 10.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.current_js) >= 14:
                break
        if len(self.current_js) < 14:
            self.get_logger().warn(f"仅收到 {len(self.current_js)} 个关节状态")

    def _js_cb(self, msg: JointState):
        for name, val in zip(msg.name, msg.position):
            self.current_js[name] = val

    def get_ee_pose(self, tip: str) -> dict | None:
        try:
            t = self.tf_buffer.lookup_transform(
                BASE_FRAME, tip, rclpy.time.Time(),
                rclpy.duration.Duration(seconds=2.0))
            return pose_from_transform(t)
        except Exception as e:
            self.get_logger().warn(f"TF {BASE_FRAME}->{tip} 失败: {e}")
            return None

    def capture(self) -> dict | None:
        """捕获当前关节角度 + 双末端位姿。"""
        rclpy.spin_once(self, timeout_sec=0.3)

        joints = {}
        for j in ALL_JOINTS:
            if j in self.current_js:
                joints[j] = round(self.current_js[j], 6)

        left_pose = self.get_ee_pose(LEFT_TIP)
        right_pose = self.get_ee_pose(RIGHT_TIP)

        if left_pose is None or right_pose is None:
            self.get_logger().error("无法获取末端 TF，请确认 MoveIt demo 正在运行")
            return None

        record = {
            "left_ee_pose": left_pose,
            "right_ee_pose": right_pose,
            "joint_values": joints,
            "joint_values_deg": {k: rad_to_deg(v) for k, v in joints.items()},
        }

        missing = [j for j in ALL_JOINTS if j not in joints]
        if missing:
            self.get_logger().warn(f"缺少关节: {missing}")

        return record

    def save_to_file(self, filepath: str):
        output = {
            "description": "双臂末端位姿记录 - 由 record_pose.py 生成",
            "frames": [BASE_FRAME],
            "left_tip": LEFT_TIP,
            "right_tip": RIGHT_TIP,
            "all_joints_order": ALL_JOINTS,
            "poses": self.records,
        }
        with open(filepath, 'w') as f:
            json.dump(output, f, indent=2, ensure_ascii=False)
        self.get_logger().info(f"已保存 {len(self.records)} 条记录到 {filepath}")


def main():
    rclpy.init()
    recorder = PoseRecorder()

    output_file = os.path.join(os.path.dirname(__file__), "recorded_poses.json")
    if len(sys.argv) > 1:
        output_file = sys.argv[1]

    print("\n" + "=" * 60)
    print("双臂末端位姿记录工具")
    print("=" * 60)
    print("在 RViz 中拖拽 + Plan & Execute，到位后按 s 记录：")
    print("  s - 保存当前位姿")
    print("  p - 打印当前关节角度")
    print("  d - 打印已保存的所有记录")
    print("  q - 退出并写入 JSON 文件")
    print("=" * 60)

    try:
        while True:
            cmd = input("\n> ").strip().lower()

            if cmd == 'q':
                if recorder.records:
                    recorder.save_to_file(output_file)
                    print(f"\n输出文件: {output_file}")
                else:
                    print("\n没有保存任何记录")
                break

            elif cmd == 's':
                record = recorder.capture()
                if record:
                    idx = len(recorder.records) + 1
                    recorder.records.append(record)
                    lp = record["left_ee_pose"]
                    rp = record["right_ee_pose"]
                    print(f"\n  [{idx}] 左臂=({lp['x']:.3f}, {lp['y']:.3f}, {lp['z']:.3f})"
                          f"  右臂=({rp['x']:.3f}, {rp['y']:.3f}, {rp['z']:.3f})")
                    jvd = record["joint_values_deg"]
                    print(f"       turn={jvd.get('turn', '?')}°  updown={jvd.get('updown', '?')}°")
                    print(f"       左臂: {', '.join(str(jvd.get(f'left_v5_joint{k}','?')) for k in range(1,7))}")
                    print(f"       右臂: {', '.join(str(jvd.get(f'right_v5_joint{k}','?')) for k in range(1,7))}")

            elif cmd == 'p':
                rclpy.spin_once(recorder, timeout_sec=0.3)
                print("\n  当前关节角度 (°):")
                for j in ALL_JOINTS:
                    if j in recorder.current_js:
                        print(f"    {j}: {rad_to_deg(recorder.current_js[j])}")
                    else:
                        print(f"    {j}: ---")

            elif cmd == 'd':
                if not recorder.records:
                    print("\n  暂无保存记录")
                else:
                    for i, r in enumerate(recorder.records):
                        lp = r["left_ee_pose"]
                        rp = r["right_ee_pose"]
                        print(f"\n  [{i+1}] 左({lp['x']:.3f},{lp['y']:.3f},{lp['z']:.3f})"
                              f" 右({rp['x']:.3f},{rp['y']:.3f},{rp['z']:.3f})")

            else:
                print("未知命令，可用: s / p / d / q")

    except KeyboardInterrupt:
        print("\n用户中断")
        if recorder.records:
            recorder.save_to_file(output_file)
            print(f"输出文件: {output_file}")

    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()