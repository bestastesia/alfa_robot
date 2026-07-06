#!/usr/bin/python3
"""
关节角度写入工具 — 直接通过 follow_joint_trajectory action 控制

直接调用 ros2_control 的 FollowJointTrajectory action，
绕过 MoveIt 的状态管理问题，确保关节真正移动。

用法:
  # 从 JSON 文件写入
  python3 set_joints.py joints.json

  # 命令行指定
  python3 set_joints.py --turn 0.5 --updown 0.3

  # 重置所有关节到 0
  python3 set_joints.py --zero

  # 指定执行时间（秒）
  python3 set_joints.py joints.json --time 3.0

前置条件:
  ros2 launch alfa_robot_moveit_config demo.launch.py
"""

import json
import math
import os
import sys
import time
import argparse

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, RobotState

# 控制器 → 关节映射
TORSO_JOINTS = ["pitch", "turn"]
ARM_JOINTS = [
    "updown",
    "leftjoint1", "leftjoint2", "leftjoint3",
    "leftjoint4", "leftjoint5", "leftjoint6",
    "rightjoint1", "rightjoint2", "rightjoint3",
    "rightjoint4", "rightjoint5", "rightjoint6",
]
ALL_JOINTS = TORSO_JOINTS + ARM_JOINTS


class JointSetter(Node):

    def __init__(self):
        super().__init__('set_joints')

        self.current_js: dict = {}
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        # 尝试连接所有可能的控制器，只等可用的
        self.controllers: dict[str, ActionClient] = {}
        self.controller_joints: dict[str, list] = {}

        # 按当前 ros2_controllers.yaml 的配置列出候选
        candidates = {
            "torso_controller": {
                "ns": "/torso_controller/follow_joint_trajectory",
                "joints": ["pitch", "turn"],
            },
            "dual_arm_controller": {
                "ns": "/dual_arm_controller/follow_joint_trajectory",
                "joints": [
                    "updown",
                    "leftjoint1", "leftjoint2", "leftjoint3",
                    "leftjoint4", "leftjoint5", "leftjoint6",
                    "rightjoint1", "rightjoint2", "rightjoint3",
                    "rightjoint4", "rightjoint5", "rightjoint6",
                ],
            },
        }

        self.get_logger().info("检测控制器 …")
        for name, info in candidates.items():
            client = ActionClient(self, FollowJointTrajectory, info["ns"])
            if client.wait_for_server(timeout_sec=3.0):
                self.controllers[name] = client
                self.controller_joints[name] = info["joints"]
                self.get_logger().info(f"  {name}: 就绪 ({len(info['joints'])} 关节)")
            else:
                self.get_logger().warn(f"  {name}: 不可用，跳过")
                client.destroy()

        if not self.controllers:
            self.get_logger().error("没有可用的控制器！请确认 demo.launch.py 已启动")
            return

        self.get_logger().info(f"可用控制器: {list(self.controllers.keys())}")

        # 等关节状态到达
        deadline = time.time() + 3.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.current_js) >= 10:
                break

        # 连接 ApplyPlanningScene 服务，用于同步 move_group 状态
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.scene_client.wait_for_service(timeout_sec=3.0)

    def _js_cb(self, msg: JointState):
        for name, val in zip(msg.name, msg.position):
            self.current_js[name] = val

    def sync_move_group(self):
        """将当前关节状态推送到 move_group 和 RViz，使两者同步。"""
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

        # 刷新当前关节状态
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state = RobotState()
        scene_msg.robot_state.joint_state = JointState()
        scene_msg.robot_state.joint_state.name = list(self.current_js.keys())
        scene_msg.robot_state.joint_state.position = list(self.current_js.values())

        # 1. 调用服务更新 move_group 内部状态
        req = ApplyPlanningScene.Request()
        req.scene = scene_msg
        future = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        # 2. 发布到 /planning_scene，触发 SceneMonitor 广播给 RViz
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        pub = self.create_publisher(PlanningScene, '/planning_scene', qos)
        # 等 publisher 建立
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        pub.publish(scene_msg)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_publisher(pub)

        if future.done() and future.result() is not None and future.result().success:
            self.get_logger().info("move_group 状态已同步")
        else:
            self.get_logger().warn("move_group 状态同步失败")

    def _send_trajectory(self, joint_names: list, target_positions: list,
                          client: ActionClient,
                          duration_sec: float = 2.0) -> bool:
        """发送轨迹到指定控制器。"""
        # 起点: 当前位置
        current = [self.current_js.get(n, 0.0) for n in joint_names]

        start_point = JointTrajectoryPoint()
        start_point.positions = current
        start_point.time_from_start.sec = 0

        end_point = JointTrajectoryPoint()
        end_point.positions = target_positions
        end_point.time_from_start.sec = int(duration_sec)
        end_point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        traj = JointTrajectory()
        traj.joint_names = joint_names
        traj.points.append(start_point)
        traj.points.append(end_point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info(
            f"发送轨迹: {joint_names} → "
            f"[{', '.join(f'{v:.3f}' for v in target_positions)}] "
            f"耗时 {duration_sec:.1f}s")

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)

        if not send_future.done() or send_future.result() is None:
            self.get_logger().error("Goal 发送失败")
            return False

        gh = send_future.result()
        if not gh.accepted:
            self.get_logger().error("Goal 被拒绝")
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 5.0)

        if not result_future.done() or result_future.result() is None:
            self.get_logger().error("执行超时")
            return False

        result = result_future.result()
        if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("执行成功 ✓")
            return True

        self.get_logger().error(f"执行失败: error_code={result.result.error_code}")
        return False

    def set_joints(self, targets: dict, duration_sec: float = 2.0) -> bool:
        """设置关节角度。自动分配到对应控制器。"""
        ok = True
        covered = set()

        for ctrl_name, ctrl_joints in self.controller_joints.items():
            ctrl_targets = {n: targets[n] for n in ctrl_joints if n in targets}
            if not ctrl_targets:
                continue
            covered.update(ctrl_targets.keys())
            names = list(ctrl_joints)
            positions = [
                ctrl_targets[name] if name in ctrl_targets else self.current_js.get(name, 0.0)
                for name in names
            ]
            if not self._send_trajectory(names, positions, self.controllers[ctrl_name], duration_sec):
                ok = False

        # 检查有没有没被任何控制器覆盖的关节
        uncovered = set(targets.keys()) - covered
        if uncovered:
            self.get_logger().warn(
                f"以下关节没有控制器覆盖，无法执行: {sorted(uncovered)}")

        if not covered:
            self.get_logger().warn("没有指定任何可执行的关节目标")
            return False

        # 执行完后同步 move_group 的内部状态
        if ok and self.scene_client.service_is_ready():
            self.sync_move_group()

        return ok

    def set_zero(self, duration_sec: float = 2.0) -> bool:
        """重置所有关节到 0。"""
        targets = {n: 0.0 for n in ALL_JOINTS}
        return self.set_joints(targets, duration_sec)


def main():
    parser = argparse.ArgumentParser(description="关节角度写入工具")
    parser.add_argument(
        "json_file", nargs="?", default=None,
        help="关节角度 JSON 文件")
    parser.add_argument(
        "--zero", action="store_true",
        help="重置所有关节到 0")
    parser.add_argument(
        "--time", type=float, default=2.0,
        help="执行时间 (秒, 默认 2.0)")
    parser.add_argument(
        "--pitch", type=float, default=None)
    parser.add_argument(
        "--turn", type=float, default=None)
    parser.add_argument(
        "--updown", type=float, default=None)
    parser.add_argument(
        "--left1", "--leftjoint1", type=float, default=None)
    parser.add_argument(
        "--left2", "--leftjoint2", type=float, default=None)
    parser.add_argument(
        "--left3", "--leftjoint3", type=float, default=None)
    parser.add_argument(
        "--left4", "--leftjoint4", type=float, default=None)
    parser.add_argument(
        "--left5", "--leftjoint5", type=float, default=None)
    parser.add_argument(
        "--left6", "--leftjoint6", type=float, default=None)
    parser.add_argument(
        "--right1", "--rightjoint1", type=float, default=None)
    parser.add_argument(
        "--right2", "--rightjoint2", type=float, default=None)
    parser.add_argument(
        "--right3", "--rightjoint3", type=float, default=None)
    parser.add_argument(
        "--right4", "--rightjoint4", type=float, default=None)
    parser.add_argument(
        "--right5", "--rightjoint5", type=float, default=None)
    parser.add_argument(
        "--right6", "--rightjoint6", type=float, default=None)
    args = parser.parse_args()

    targets = {}

    # 从 JSON 文件读取
    if args.json_file:
        if not os.path.exists(args.json_file):
            print(f"错误: 文件不存在 {args.json_file}")
            sys.exit(1)
        with open(args.json_file) as f:
            data = json.load(f)

        # 统一格式: 顶层 "joints" 字段
        if "joints" in data:
            targets = data["joints"]
        elif "joint_values" in data:
            targets = data["joint_values"]
        elif "poses" in data:
            # record_pose.py 输出的多条记录，取第一条
            targets = data["poses"][0].get("joint_values", {})
        else:
            # 纯关节角度 dict
            targets = data

    # 命令行覆盖
    cli_args = {
        "pitch": args.pitch, "turn": args.turn, "updown": args.updown,
        "leftjoint1": args.left1, "leftjoint2": args.left2,
        "leftjoint3": args.left3, "leftjoint4": args.left4,
        "leftjoint5": args.left5, "leftjoint6": args.left6,
        "rightjoint1": args.right1, "rightjoint2": args.right2,
        "rightjoint3": args.right3, "rightjoint4": args.right4,
        "rightjoint5": args.right5, "rightjoint6": args.right6,
    }
    for name, val in cli_args.items():
        if val is not None:
            targets[name] = val

    # --zero 模式
    if args.zero:
        targets = {n: 0.0 for n in ALL_JOINTS}

    if not targets:
        print("用法: python3 set_joints.py joints.json")
        print("      python3 set_joints.py --turn 0.5 --updown 0.3")
        print("      python3 set_joints.py --zero")
        sys.exit(1)

    # 打印目标
    print("\n" + "=" * 50)
    print("关节角度写入工具")
    print("=" * 50)
    for name in ALL_JOINTS:
        if name in targets:
            current = "—"
            target = targets[name]
            deg = math.degrees(target) if name != "updown" else f"{target:.3f}m"
            print(f"  {name:20s}: → {target:.4f} ({deg})")
    print(f"  执行时间: {args.time:.1f}s")
    print("=" * 50)

    rclpy.init()
    setter = JointSetter()
    try:
        ok = setter.set_joints(targets, duration_sec=args.time)
        if not ok:
            print("执行失败")
        else:
            print("执行成功")
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        setter.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
