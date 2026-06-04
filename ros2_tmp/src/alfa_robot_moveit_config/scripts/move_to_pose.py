#!/usr/bin/env python3
"""
末端位姿 → IK 求解 → 执行

输入末端位姿（左/右），通过 MoveIt IK 服务求解关节角，
再用 follow_joint_trajectory 直接执行，确保关节真正移动。

用法:
  # 从 JSON 文件读取位姿并执行
  python3 move_to_pose.py pose.json

  # 指定左臂位姿 (x y z qx qy qz qw)
  python3 move_to_pose.py --left 0.5 0.3 0.8 0 0 0 1

  # 指定双臂位姿
  python3 move_to_pose.py --left 0.5 0.3 0.8 0 0 0 1 --right 0.5 -0.3 0.8 0 0 0 1

  # 只求解不执行
  python3 move_to_pose.py --left 0.5 0.3 0.8 0 0 0 1 --ik-only

  # 指定 IK 求解器
  python3 move_to_pose.py --left 0.5 0.3 0.8 0 0 0 1 --solver pick_ik

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

from moveit_msgs.srv import GetPositionIK, ApplyPlanningScene
from moveit_msgs.msg import PositionIKRequest, RobotState, MoveItErrorCodes, PlanningScene
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState

LEFT_TIP = "left_v5_tool0"
RIGHT_TIP = "right_v5_tool0"
BASE_FRAME = "base_link"

LEFT_ARM_GROUP = "left_v5_arm"
RIGHT_ARM_GROUP = "right_v5_arm"
DUAL_ARM_GROUP = "dual_v5_arm_with_base"

TORSO_JOINTS = ["pitch", "turn"]
ARM_JOINTS = [
    "updown",
    "left_v5_joint1", "left_v5_joint2", "left_v5_joint3",
    "left_v5_joint4", "left_v5_joint5", "left_v5_joint6",
    "right_v5_joint1", "right_v5_joint2", "right_v5_joint3",
    "right_v5_joint4", "right_v5_joint5", "right_v5_joint6",
]

# IK 求解器配置
SOLVER_MAP = {
    "kdl": "kdl_kinematics_plugin/KDLKinematicsPlugin",
    "trac_ik": "trac_ik_kinematics_plugin/TRACIKKinematicsPlugin",
    "pick_ik": "pick_ik/PickIKPlugin",
    "bio_ik": "bio_ik/BioIKKinematicsPlugin",
}


def parse_pose_args(vals: list) -> Pose:
    """从 [x y z qx qy qz qw] 解析 Pose。"""
    p = Pose()
    p.position.x = float(vals[0])
    p.position.y = float(vals[1])
    p.position.z = float(vals[2])
    p.orientation.x = float(vals[3])
    p.orientation.y = float(vals[4])
    p.orientation.z = float(vals[5])
    p.orientation.w = float(vals[6])
    return p


def dict_to_pose(d: dict) -> Pose:
    p = Pose()
    p.position.x = d["x"]
    p.position.y = d["y"]
    p.position.z = d["z"]
    p.orientation.x = d["qx"]
    p.orientation.y = d["qy"]
    p.orientation.z = d["qz"]
    p.orientation.w = d["qw"]
    return p


def pose_to_dict(p: Pose) -> dict:
    return {
        "x": round(p.position.x, 4),
        "y": round(p.position.y, 4),
        "z": round(p.position.z, 4),
        "qx": round(p.orientation.x, 4),
        "qy": round(p.orientation.y, 4),
        "qz": round(p.orientation.z, 4),
        "qw": round(p.orientation.w, 4),
    }


class MoveToPose(Node):

    def __init__(self):
        super().__init__('move_to_pose')

        self.current_js: dict = {}
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        # IK 服务
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # 动态检测控制器
        self.controllers: dict[str, ActionClient] = {}
        self.controller_joints: dict[str, list] = {}

        candidates = {
            "torso_controller": {
                "ns": "/torso_controller/follow_joint_trajectory",
                "joints": ["pitch", "turn"],
            },
            "dual_v5_arm_controller": {
                "ns": "/dual_v5_arm_controller/follow_joint_trajectory",
                "joints": [
                    "updown",
                    "left_v5_joint1", "left_v5_joint2", "left_v5_joint3",
                    "left_v5_joint4", "left_v5_joint5", "left_v5_joint6",
                    "right_v5_joint1", "right_v5_joint2", "right_v5_joint3",
                    "right_v5_joint4", "right_v5_joint5", "right_v5_joint6",
                ],
            },
        }

        self.get_logger().info("等待 IK 服务和控制器 …")
        self.ik_client.wait_for_service(timeout_sec=10.0)

        for name, info in candidates.items():
            client = ActionClient(self, FollowJointTrajectory, info["ns"])
            if client.wait_for_server(timeout_sec=3.0):
                self.controllers[name] = client
                self.controller_joints[name] = info["joints"]
                self.get_logger().info(f"  {name}: 就绪")
            else:
                self.get_logger().warn(f"  {name}: 不可用")
                client.destroy()

        self.get_logger().info("服务就绪")

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

    # ── IK 求解 ────────────────────────────────────────────────

    def solve_ik(self, group_name: str, tip_link: str,
                  target_pose: Pose,
                  timeout: float = 5.0,
                  solver: str = None) -> dict | None:
        """求解 IK，返回 {joint_name: value} 或 None。"""
        req = GetPositionIK.Request()
        req.ik_request.group_name = group_name
        req.ik_request.pose_stamped = PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = BASE_FRAME
        req.ik_request.pose_stamped.pose = target_pose
        req.ik_request.ik_link_name = tip_link
        req.ik_request.timeout.sec = int(timeout)
        req.ik_request.timeout.nanosec = int((timeout - int(timeout)) * 1e9)

        # 设置当前关节状态作为 seed
        req.ik_request.robot_state = RobotState()
        for j in TORSO_JOINTS + ARM_JOINTS:
            if j in self.current_js:
                req.ik_request.robot_state.joint_state.name.append(j)
                req.ik_request.robot_state.joint_state.position.append(self.current_js[j])

        if solver and solver in SOLVER_MAP:
            req.ik_request.kinematics_plugin = SOLVER_MAP[solver]

        self.get_logger().info(
            f"IK 求解: group={group_name} tip={tip_link} "
            f"pos=({target_pose.position.x:.3f},{target_pose.position.y:.3f},{target_pose.position.z:.3f})")

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout + 2.0)

        if not future.done() or future.result() is None:
            self.get_logger().error("IK 服务调用失败/超时")
            return None

        res = future.result()
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"IK 求解失败: error_code={res.error_code.val}")
            return None

        # 提取关节角度
        solution = {}
        js = res.solution.joint_state
        for name, val in zip(js.name, js.position):
            solution[name] = val

        self.get_logger().info(
            f"IK 求解成功: {len(solution)} 个关节")
        return solution

    def solve_dual_ik(self, left_pose: Pose, right_pose: Pose,
                       timeout: float = 10.0,
                       solver: str = None) -> dict | None:
        """双臂一起 IK 求解，使用 dual_v5_arm_with_base group。"""
        req = GetPositionIK.Request()
        req.ik_request.group_name = DUAL_ARM_GROUP
        req.ik_request.ik_link_names = [LEFT_TIP, RIGHT_TIP]
        req.ik_request.pose_stamped_vector = []
        for pose in [left_pose, right_pose]:
            ps = PoseStamped()
            ps.header.frame_id = BASE_FRAME
            ps.pose = pose
            req.ik_request.pose_stamped_vector.append(ps)
        req.ik_request.timeout.sec = int(timeout)
        req.ik_request.timeout.nanosec = int((timeout - int(timeout)) * 1e9)

        # seed state
        req.ik_request.robot_state = RobotState()
        for j in ["pitch", "turn", "updown"] + [
            "left_v5_joint1", "left_v5_joint2", "left_v5_joint3",
            "left_v5_joint4", "left_v5_joint5", "left_v5_joint6",
            "right_v5_joint1", "right_v5_joint2", "right_v5_joint3",
            "right_v5_joint4", "right_v5_joint5", "right_v5_joint6",
        ]:
            if j in self.current_js:
                req.ik_request.robot_state.joint_state.name.append(j)
                req.ik_request.robot_state.joint_state.position.append(self.current_js[j])

        if solver and solver in SOLVER_MAP:
            req.ik_request.kinematics_plugin = SOLVER_MAP[solver]

        self.get_logger().info(
            f"IK 求解: group={DUAL_ARM_GROUP} tips=[{LEFT_TIP}, {RIGHT_TIP}]")

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout + 2.0)

        if not future.done() or future.result() is None:
            self.get_logger().error("IK 服务调用失败/超时")
            return None

        res = future.result()
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"IK 求解失败: error_code={res.error_code.val}")
            return None

        solution = {}
        js = res.solution.joint_state
        for name, val in zip(js.name, js.position):
            solution[name] = val

        self.get_logger().info(f"IK 求解成功: {len(solution)} 个关节")
        return solution

    # ── 执行 ───────────────────────────────────────────────────

    def _send_trajectory(self, joint_names: list, target_positions: list,
                          client: ActionClient,
                          duration_sec: float = 3.0) -> bool:
        current = [self.current_js.get(n, 0.0) for n in joint_names]

        start_point = JointTrajectoryPoint()
        start_point.positions = current

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

        self.get_logger().info(f"执行轨迹: {len(joint_names)} 关节, {duration_sec:.1f}s")

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

    def execute_joints(self, targets: dict, duration_sec: float = 3.0) -> bool:
        """执行关节角度目标。自动分配到可用控制器。"""
        ok = True
        covered = set()

        for ctrl_name, ctrl_joints in self.controller_joints.items():
            ctrl_targets = {n: targets[n] for n in ctrl_joints if n in targets}
            if not ctrl_targets:
                continue
            covered.update(ctrl_targets.keys())
            names = list(ctrl_targets.keys())
            positions = list(ctrl_targets.values())
            if not self._send_trajectory(names, positions, self.controllers[ctrl_name], duration_sec):
                ok = False

        uncovered = set(targets.keys()) - covered
        if uncovered:
            self.get_logger().warn(f"以下关节没有控制器: {sorted(uncovered)}")
        return ok

    # ── 一步到位: 位姿 → IK → 执行 ──────────────────────────────

    def move_to_pose(self, left_pose: Pose = None, right_pose: Pose = None,
                      ik_timeout: float = 10.0,
                      exec_duration: float = 3.0,
                      solver: str = None,
                      ik_only: bool = False) -> dict | None:
        """末端位姿 → IK 求解 → 执行。返回求解结果或 None。"""
        if left_pose is None and right_pose is None:
            self.get_logger().error("需要指定至少一个末端位姿")
            return None

        solution = {}

        if left_pose and right_pose:
            # 双臂: 分别求解
            solution = self.solve_dual_ik(left_pose, right_pose, ik_timeout, solver)
        elif left_pose:
            solution = self.solve_ik(
                LEFT_ARM_GROUP, LEFT_TIP, left_pose, ik_timeout, solver=solver)
        elif right_pose:
            solution = self.solve_ik(
                RIGHT_ARM_GROUP, RIGHT_TIP, right_pose, ik_timeout, solver=solver)

        if solution is None:
            self.get_logger().error("IK 求解失败")
            return None

        # 打印求解结果
        print("\n  IK 求解结果:")
        for name in TORSO_JOINTS + ARM_JOINTS:
            if name in solution:
                v = solution[name]
                unit = "m" if name == "updown" else "°"
                display = f"{v:.4f} ({math.degrees(v):.2f}{unit})" if name != "updown" else f"{v:.4f}m"
                print(f"    {name:20s}: {display}")

        if ik_only:
            self.get_logger().info("仅求解模式，不执行")
            return solution

        # 执行
        ok = self.execute_joints(solution, exec_duration)
        if ok:
            self.sync_move_group()
            return solution
        return None


def main():
    parser = argparse.ArgumentParser(description="末端位姿 → IK → 执行")
    parser.add_argument(
        "json_file", nargs="?", default=None,
        help="位姿 JSON 文件 (record_pose.py 输出)")
    parser.add_argument(
        "--left", nargs=7, type=float, metavar=("X","Y","Z","QX","QY","QZ","QW"),
        help="左臂末端位姿")
    parser.add_argument(
        "--right", nargs=7, type=float, metavar=("X","Y","Z","QX","QY","QZ","QW"),
        help="右臂末端位姿")
    parser.add_argument(
        "--ik-only", action="store_true",
        help="仅求解 IK，不执行")
    parser.add_argument(
        "--ik-timeout", type=float, default=10.0,
        help="IK 求解超时 (秒, 默认 10)")
    parser.add_argument(
        "--exec-time", type=float, default=3.0,
        help="执行时间 (秒, 默认 3)")
    parser.add_argument(
        "--solver", choices=["kdl", "trac_ik", "pick_ik", "bio_ik"], default=None,
        help="IK 求解器 (默认使用 MoveIt 配置)")
    args = parser.parse_args()

    left_pose = None
    right_pose = None

    # 从 JSON 读取 (优先 "ee_pose" 字段，兼容旧格式)
    if args.json_file:
        if not os.path.exists(args.json_file):
            print(f"错误: 文件不存在 {args.json_file}")
            sys.exit(1)
        with open(args.json_file) as f:
            data = json.load(f)

        if "ee_pose" in data:
            # 统一格式: 顶层 "ee_pose" 字段
            if "left" in data["ee_pose"]:
                left_pose = dict_to_pose(data["ee_pose"]["left"])
            if "right" in data["ee_pose"]:
                right_pose = dict_to_pose(data["ee_pose"]["right"])
        elif "poses" in data:
            # record_pose.py 输出
            pose_data = data["poses"][0]
            if "left_ee_pose" in pose_data:
                left_pose = dict_to_pose(pose_data["left_ee_pose"])
            if "right_ee_pose" in pose_data:
                right_pose = dict_to_pose(pose_data["right_ee_pose"])
        else:
            if "left" in data:
                left_pose = dict_to_pose(data["left"])
            if "right" in data:
                right_pose = dict_to_pose(data["right"])

    # 命令行覆盖
    if args.left:
        left_pose = parse_pose_args(args.left)
    if args.right:
        right_pose = parse_pose_args(args.right)

    if left_pose is None and right_pose is None:
        print("用法: python3 move_to_pose.py --left X Y Z QX QY QZ QW")
        print("      python3 move_to_pose.py --left ... --right ...")
        print("      python3 move_to_pose.py pose.json")
        print("      python3 move_to_pose.py --left ... --ik-only")
        sys.exit(1)

    # 打印目标
    print("\n" + "=" * 50)
    print("末端位姿 → IK → 执行")
    print("=" * 50)
    if left_pose:
        d = pose_to_dict(left_pose)
        print(f"  左臂: ({d['x']:.3f}, {d['y']:.3f}, {d['z']:.3f})")
    if right_pose:
        d = pose_to_dict(right_pose)
        print(f"  右臂: ({d['x']:.3f}, {d['y']:.3f}, {d['z']:.3f})")
    if args.ik_only:
        print("  ** 仅求解模式 **")
    print("=" * 50)

    rclpy.init()
    mover = MoveToPose()
    try:
        result = mover.move_to_pose(
            left_pose=left_pose,
            right_pose=right_pose,
            ik_timeout=args.ik_timeout,
            exec_duration=args.exec_time,
            solver=args.solver,
            ik_only=args.ik_only,
        )
        if result is None and not args.ik_only:
            print("执行失败")
        elif result is None:
            print("IK 求解失败")
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        mover.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()