#!/usr/bin/env python3
# Copyright (c) 2025, b»robotized
#
# Licensed under the Apache License, Version 2.0 (the "License");

"""
精确关节控制工具：通过命令行输入精确数值控制机器人关节。

使用方式:
  ros2 run alfa_robot_bringup precise_joint_control

命令:
  list              - 列出所有关节及其当前值
  set <joint> <val> - 设置指定关节到目标值 (高精度，支持 6+ 小数位)
  move <j1> <j2> ... - 批量设置所有关节值 (按顺序)
  home              - 所有关节归零
  save <name>       - 保存当前姿态到文件
  load <name>       - 加载保存的姿态
  poses             - 列出所有保存的姿态
  help              - 显示帮助
  quit              - 退出

示例:
  set leftjoint2 0.123456789    # 设置单个关节
  move 0 0 0.1 0.2 0 0 0 0 0 0  # 批量设置 (按关节顺序)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import os
import json
import readline  # 支持命令历史和编辑
from pathlib import Path


DEFAULT_JOINT_ORDER = [
    "turn",
    "updown",
    "leftarmbase",
    "leftjoint1",
    "leftjoint2",
    "leftjoint3",
    "leftjoint4",
    "leftjoint5",
    "rightarmbase",
    "rightjoint1",
    "rightjoint2",
    "rightjoint3",
    "rightjoint4",
]

JOINT_LIMITS = {
    "turn": (-3.14159, 3.14159),
    "updown": (0.0, 1.0),
    "leftarmbase": (0.0, 0.14),
    "leftjoint1": (0.0, 0.42),
    "leftjoint2": (-3.14159, 3.14159),
    "leftjoint3": (-3.14159, 3.14159),
    "leftjoint4": (0.0, 0.22),
    "leftjoint5": (-3.14159, 3.14159),
    "rightarmbase": (0.0, 0.16),
    "rightjoint1": (0.0, 0.5),
    "rightjoint2": (-3.14159, 3.14159),
    "rightjoint3": (-3.14159, 3.14159),
    "rightjoint4": (-3.14159, 3.14159),
}


class PreciseJointControl(Node):
    def __init__(self):
        super().__init__("precise_joint_control")

        self.declare_parameter("controller_name", "all_position_controller")
        self.declare_parameter("joint_names", DEFAULT_JOINT_ORDER)

        controller_name = self.get_parameter("controller_name").value
        self._joint_names = list(self.get_parameter("joint_names").value)

        # 发布位置命令
        self._cmd_pub = self.create_publisher(
            Float64MultiArray, f"/{controller_name}/commands", 10
        )

        # 订阅关节状态
        self._state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )

        self._current_positions = {name: 0.0 for name in self._joint_names}
        self._positions_received = False

        # 姿态保存目录
        self._poses_dir = Path.home() / ".alfa_robot_poses"
        self._poses_dir.mkdir(exist_ok=True)

        self.get_logger().info(f"精确关节控制工具已启动")
        self.get_logger().info(f"控制器: {controller_name}")
        self.get_logger().info(f"关节: {self._joint_names}")
        self.get_logger().info("输入 'help' 查看命令列表")

    def _joint_state_callback(self, msg: JointState):
        """更新当前关节位置。"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position) and name in self._current_positions:
                self._current_positions[name] = msg.position[i]
        self._positions_received = True

    def send_positions(self, positions: dict):
        """发送位置命令到控制器。"""
        cmd = Float64MultiArray()
        cmd.data = [positions.get(name, 0.0) for name in self._joint_names]
        self._cmd_pub.publish(cmd)

    def print_status(self):
        """打印所有关节当前值。"""
        print("\n" + "=" * 70)
        print(f"{'关节':<15} {'当前值':>20} {'最小':>12} {'最大':>12}")
        print("=" * 70)
        for name in self._joint_names:
            val = self._current_positions.get(name, 0.0)
            lo, hi = JOINT_LIMITS.get(name, (-float('inf'), float('inf')))
            print(f"{name:<15} {val:>20.8f} {lo:>12.5f} {hi:>12.5f}")
        print("=" * 70)

    def set_joint(self, joint_name: str, value: float):
        """设置单个关节值。"""
        if joint_name not in self._joint_names:
            print(f"错误: 未知关节 '{joint_name}'")
            print(f"可用关节: {', '.join(self._joint_names)}")
            return False

        lo, hi = JOINT_LIMITS.get(joint_name, (-float('inf'), float('inf')))
        if not (lo <= value <= hi):
            print(f"警告: 值 {value:.8f} 超出范围 [{lo:.5f}, {hi:.5f}]")
            confirm = input("仍要继续? (y/n): ").strip().lower()
            if confirm != 'y':
                return False

        # 更新并发送
        self._current_positions[joint_name] = value
        self.send_positions(self._current_positions)
        print(f"设置 {joint_name} = {value:.8f}")
        return True

    def move_all(self, values: list):
        """批量设置所有关节值。"""
        if len(values) != len(self._joint_names):
            print(f"错误: 需要 {len(self._joint_names)} 个值，收到 {len(values)} 个")
            return False

        try:
            values = [float(v) for v in values]
        except ValueError as e:
            print(f"错误: 无法解析数值 - {e}")
            return False

        # 检查限位
        for name, val in zip(self._joint_names, values):
            lo, hi = JOINT_LIMITS.get(name, (-float('inf'), float('inf')))
            if not (lo <= val <= hi):
                print(f"警告: {name}={val:.8f} 超出 [{lo:.5f}, {hi:.5f}]")

        # 更新并发送
        for name, val in zip(self._joint_names, values):
            self._current_positions[name] = val
        self.send_positions(self._current_positions)

        print("已设置所有关节:")
        for name, val in zip(self._joint_names, values):
            print(f"  {name} = {val:.8f}")
        return True

    def go_home(self):
        """所有关节归零。"""
        for name in self._joint_names:
            self._current_positions[name] = 0.0
        self.send_positions(self._current_positions)
        print("所有关节已归零")

    def save_pose(self, name: str):
        """保存当前姿态。"""
        pose_file = self._poses_dir / f"{name}.json"
        data = {
            "positions": self._current_positions.copy(),
            "description": f"Saved pose: {name}"
        }
        with open(pose_file, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"姿态 '{name}' 已保存到 {pose_file}")

    def load_pose(self, name: str):
        """加载保存的姿态。"""
        pose_file = self._poses_dir / f"{name}.json"
        if not pose_file.exists():
            print(f"错误: 找不到姿态 '{name}'")
            return False

        with open(pose_file, 'r') as f:
            data = json.load(f)

        positions = data.get("positions", {})
        for name_key in self._joint_names:
            if name_key in positions:
                self._current_positions[name_key] = positions[name_key]

        self.send_positions(self._current_positions)
        print(f"已加载姿态 '{name}':")
        for name_key in self._joint_names:
            print(f"  {name_key} = {self._current_positions[name_key]:.8f}")
        return True

    def list_poses(self):
        """列出所有保存的姿态。"""
        poses = list(self._poses_dir.glob("*.json"))
        if not poses:
            print("没有保存的姿态")
            return

        print("\n保存的姿态:")
        for pose_file in poses:
            name = pose_file.stem
            with open(pose_file, 'r') as f:
                data = json.load(f)
            desc = data.get("description", "")
            print(f"  {name}: {desc}")

    def print_help(self):
        """打印帮助信息。"""
        print("""
命令帮助:
  list              - 列出所有关节及其当前值 (8位精度)
  set <joint> <val> - 设置指定关节到目标值 (支持任意精度)
                      示例: set leftjoint2 0.123456789
  move <v1> <v2> ...- 批量设置所有关节 (按顺序，14个值)
                      示例: move 0 0 0.1 0.2 0 0 0 0 0 0 0 0 0 0
  home              - 所有关节归零
  save <name>       - 保存当前姿态
                      示例: save work_position_1
  load <name>       - 加载保存的姿态
                      示例: load work_position_1
  poses             - 列出所有保存的姿态
  help              - 显示此帮助
  quit / q          - 退出程序

关节列表 (按顺序):
  0: turn          - 底座旋转
  1: updown        - 升降
  2: leftarmbase   - 左臂底座滑动
  3: leftjoint1    - 左臂伸缩1
  4: leftjoint2    - 左臂旋转2
  5: leftjoint3    - 左臂旋转3
  6: leftjoint4    - 左臂伸缩4
  7: leftjoint5    - 左臂旋转5
  8: rightarmbase  - 右臂底座滑动
  9: rightjoint1   - 右臂伸缩1
  10: rightjoint2  - 右臂旋转2
  11: rightjoint3  - 右臂旋转3
  12: rightjoint4  - 右臂旋转4
""")


def main(args=None):
    rclpy.init(args=args)
    node = PreciseJointControl()

    print("\n等待关节状态...")
    import time
    for _ in range(50):  # 等待最多5秒
        rclpy.spin_once(node, timeout_sec=0.1)
        if node._positions_received:
            break
    else:
        print("警告: 未收到关节状态，将使用默认值")

    node.print_status()

    try:
        while rclpy.ok():
            try:
                cmd_input = input("\n> ").strip()
            except EOFError:
                break

            if not cmd_input:
                continue

            parts = cmd_input.split()
            cmd = parts[0].lower()
            args_list = parts[1:]

            if cmd == "quit" or cmd == "q":
                print("退出...")
                break
            elif cmd == "help":
                node.print_help()
            elif cmd == "list":
                rclpy.spin_once(node, timeout_sec=0.1)
                node.print_status()
            elif cmd == "set":
                if len(args_list) >= 2:
                    try:
                        value = float(args_list[1])
                        node.set_joint(args_list[0], value)
                    except ValueError:
                        print(f"错误: 无法解析数值 '{args_list[1]}'")
                else:
                    print("用法: set <关节名> <值>")
            elif cmd == "move":
                if len(args_list) >= 1:
                    node.move_all(args_list)
                else:
                    print("用法: move <值1> <值2> ... (共14个值)")
            elif cmd == "home":
                node.go_home()
            elif cmd == "save":
                if args_list:
                    node.save_pose(args_list[0])
                else:
                    print("用法: save <姿态名称>")
            elif cmd == "load":
                if args_list:
                    node.load_pose(args_list[0])
                else:
                    print("用法: load <姿态名称>")
            elif cmd == "poses":
                node.list_poses()
            else:
                print(f"未知命令: {cmd}。输入 'help' 查看可用命令。")

            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
