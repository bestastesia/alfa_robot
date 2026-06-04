#!/usr/bin/env python3
"""
双臂 Pick-Place 流程 Demo

5 阶段: 预抓取 → 前伸抓取(附箱子) → 后退 → 放置(卸箱子) → 回位

核心改进: 使用 MoveGroup action (/move_action) 而不是 split pipeline，
确保 move_group 内部状态更新，解决 RViz 中关节状态不变的问题。

用法:
  ros2 launch alfa_robot_moveit_config demo.launch.py
  python3 pick_place_flow.py poses.json

  # 也可用 record_pose.py 记录的位姿
  python3 pick_place_flow.py recorded_poses.json --pre-grasp 0 --place 1

  # 纯规划不执行
  python3 pick_place_flow.py poses.json --dry-run
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

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, PositionConstraint,
    OrientationConstraint, JointConstraint, MoveItErrorCodes,
    PlanningScene, CollisionObject, AttachedCollisionObject,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

LEFT_TIP = "left_v5_tool0"
RIGHT_TIP = "right_v5_tool0"
BASE_FRAME = "base_link"

DUAL_ARM_GROUP = "dual_arm_with_base"
TORSO_GROUP = "base"

TORSO_JOINTS = ["pitch", "turn", "updown"]
ARM_JOINTS = [
    "updown",
    "left_v5_joint1", "left_v5_joint2", "left_v5_joint3",
    "left_v5_joint4", "left_v5_joint5", "left_v5_joint6",
    "right_v5_joint1", "right_v5_joint2", "right_v5_joint3",
    "right_v5_joint4", "right_v5_joint5", "right_v5_joint6",
]

LEFT_TOUCH_LINKS = ["left_v5_link6", "left_v5_tool0"]
RIGHT_TOUCH_LINKS = ["right_v5_link6", "right_v5_tool0"]

BOX_DIMS = [0.06, 0.06, 0.06]  # 6cm cube


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


def offset_pose_along_z(pose: Pose, distance: float) -> Pose:
    """沿 pose 局部 Z 轴偏移位置（吸盘方向）。

    distance > 0: 向后退; distance < 0: 向前伸
    """
    q = pose.orientation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w

    # 旋转矩阵的 Z 列 = 局部 Z 在世界坐标系中的方向
    rz_x = 2 * (qx * qz + qw * qy)
    rz_y = 2 * (qy * qz - qw * qx)
    rz_z = 1 - 2 * (qx * qx + qy * qy)

    result = Pose()
    result.position.x = pose.position.x + distance * rz_x
    result.position.y = pose.position.y + distance * rz_y
    result.position.z = pose.position.z + distance * rz_z
    result.orientation = pose.orientation
    return result


def make_pose_constraints(link_name: str, pose: Pose,
                          pos_tol: float = 0.005,
                          ori_tol: float = 0.05) -> tuple:
    pos = PositionConstraint()
    pos.header.frame_id = BASE_FRAME
    pos.link_name = link_name
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [pos_tol]
    pos.constraint_region.primitives.append(sphere)
    region_pose = Pose()
    region_pose.position = pose.position
    region_pose.orientation.w = 1.0
    pos.constraint_region.primitive_poses.append(region_pose)
    pos.weight = 1.0

    ori = OrientationConstraint()
    ori.header.frame_id = BASE_FRAME
    ori.link_name = link_name
    ori.orientation = pose.orientation
    ori.absolute_x_axis_tolerance = ori_tol
    ori.absolute_y_axis_tolerance = ori_tol
    ori.absolute_z_axis_tolerance = ori_tol
    ori.weight = 1.0

    return pos, ori


class PickPlaceFlow(Node):

    def __init__(self, velocity_scale=0.3, accel_scale=0.2, dry_run=False):
        super().__init__('pick_place_flow')

        self.velocity_scale = velocity_scale
        self.accel_scale = accel_scale
        self.dry_run = dry_run

        # MoveGroup action 客户端（核心: 同 RViz "Plan & Execute"）
        self.move_client = ActionClient(self, MoveGroup, '/move_action')

        # 规划场景服务（附着/卸载箱子）
        self.scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene')

        # 关节状态订阅
        self.current_js: dict = {}
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        self.get_logger().info("等待 MoveGroup action 服务 …")
        self.move_client.wait_for_server(timeout_sec=15.0)
        self.get_logger().info("等待 ApplyPlanningScene 服务 …")
        self.scene_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("服务就绪")

        # 等第一帧关节状态
        deadline = time.time() + 5.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.current_js) >= 10:
                break

    def _js_cb(self, msg: JointState):
        for name, val in zip(msg.name, msg.position):
            self.current_js[name] = val

    # ── MoveGroup Action 调用 ──────────────────────────────────

    def _call_move_group(self, group_name: str,
                          goal_constraints: Constraints,
                          planning_time: float = 10.0,
                          attempts: int = 20,
                          plan_only: bool = False) -> bool:
        """通过 MoveGroup action 规划+执行。"""
        mpr = MotionPlanRequest()
        mpr.group_name = group_name
        mpr.allowed_planning_time = planning_time
        mpr.num_planning_attempts = attempts
        mpr.max_velocity_scaling_factor = self.velocity_scale
        mpr.max_acceleration_scaling_factor = self.accel_scale
        mpr.goal_constraints.append(goal_constraints)

        goal = MoveGroup.Goal()
        goal.request = mpr
        goal.planning_options.plan_only = plan_only or self.dry_run
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self.get_logger().info(
            f"[{group_name}] 发送 MoveGroup goal (plan_only={goal.planning_options.plan_only})")

        send_future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)

        if not send_future.done() or send_future.result() is None:
            self.get_logger().error("MoveGroup goal 发送失败")
            return False

        gh = send_future.result()
        if not gh.accepted:
            self.get_logger().error("MoveGroup goal 被拒绝")
            return False

        self.get_logger().info("Goal 已接受，等待结果 …")
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)

        if not result_future.done() or result_future.result() is None:
            self.get_logger().error("MoveGroup 执行超时")
            return False

        result = result_future.result()
        code = result.result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("MoveGroup 执行成功")
            return True

        self.get_logger().error(f"MoveGroup 失败: error_code={code}")
        return False

    # ── 躯干移动 ───────────────────────────────────────────────

    def move_torso(self, pitch: float, turn: float) -> bool:
        goal = Constraints()
        for name, val in [("pitch", pitch), ("turn", turn)]:
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal.joint_constraints.append(jc)
        return self._call_move_group(TORSO_GROUP, goal, planning_time=5.0, attempts=5)

    # ── 双臂末端位姿移动 ──────────────────────────────────────

    def move_dual_arms(self, left_pose: Pose, right_pose: Pose,
                        pos_tol: float = 0.005,
                        ori_tol: float = 0.05) -> bool:
        goal = Constraints()
        goal.name = "dual_ee_goal"
        lpos, lori = make_pose_constraints(LEFT_TIP, left_pose, pos_tol, ori_tol)
        rpos, rori = make_pose_constraints(RIGHT_TIP, right_pose, pos_tol, ori_tol)
        goal.position_constraints.extend([lpos, rpos])
        goal.orientation_constraints.extend([lori, rori])
        return self._call_move_group(DUAL_ARM_GROUP, goal)

    # ── 双臂关节目标移动 ──────────────────────────────────────

    def move_dual_arms_joints(self, joint_targets: dict) -> bool:
        goal = Constraints()
        for name, val in joint_targets.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal.joint_constraints.append(jc)
        return self._call_move_group(DUAL_ARM_GROUP, goal, planning_time=5.0, attempts=5)

    # ── 箱子附着/卸载 ──────────────────────────────────────────

    def _apply_scene(self, scene: PlanningScene) -> bool:
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done() and future.result() is not None:
            return future.result().success
        return False

    def attach_box(self, object_id: str, link_name: str,
                   box_dims: list = BOX_DIMS,
                   touch_links: list = None,
                   box_offset: Pose = None) -> bool:
        """附着箱子到末端 link。"""
        co = CollisionObject()
        co.header.frame_id = link_name
        co.id = object_id
        co.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(box_dims)
        co.primitives.append(box)

        pose = box_offset if box_offset else Pose()
        if not box_offset:
            pose.orientation.w = 1.0
        co.primitive_poses.append(pose)

        aco = AttachedCollisionObject()
        aco.object = co
        aco.link_name = link_name
        aco.touch_links = touch_links or [link_name]
        aco.weight = 0.1

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)

        ok = self._apply_scene(scene)
        if ok:
            self.get_logger().info(f"附着 '{object_id}' 到 {link_name}")
        else:
            self.get_logger().error(f"附着 '{object_id}' 失败")
        return ok

    def detach_box(self, object_id: str, link_name: str) -> bool:
        """卸载箱子，留在当前位置。"""
        aco = AttachedCollisionObject()
        aco.object = CollisionObject()
        aco.object.id = object_id
        aco.object.operation = CollisionObject.REMOVE
        aco.link_name = link_name

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)

        ok = self._apply_scene(scene)
        if ok:
            self.get_logger().info(f"卸载 '{object_id}' 从 {link_name}")
        else:
            self.get_logger().error(f"卸载 '{object_id}' 失败")
        return ok

    def remove_box_from_world(self, object_id: str) -> bool:
        """从规划场景中完全移除箱子。"""
        co = CollisionObject()
        co.header.frame_id = BASE_FRAME
        co.id = object_id
        co.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)

        ok = self._apply_scene(scene)
        if ok:
            self.get_logger().info(f"移除 '{object_id}' 从场景")
        return ok


# ── 位姿配置加载 ──────────────────────────────────────────────

def load_config(filepath: str) -> dict:
    """加载位姿 JSON 配置。"""
    with open(filepath) as f:
        data = json.load(f)

    # 支持 record_pose.py 输出格式 (poses 数组)
    if "poses" in data and "pre_grasp" not in data:
        poses = data["poses"]
        if len(poses) < 2:
            print(f"错误: 需要至少 2 条记录 (预抓取 + 放置)")
            sys.exit(1)
        config = {
            "pre_grasp": {
                "left": poses[0].get("left_ee_pose"),
                "right": poses[0].get("right_ee_pose"),
            },
            "place": {
                "left": poses[1].get("left_ee_pose"),
                "right": poses[1].get("right_ee_pose"),
            },
            "approach_distance": 0.08,
            "retreat_height": 0.05,
            "turn_deg_at_place": 90,
        }
        return config

    return data


# ── 主流程 ────────────────────────────────────────────────────

def run_flow(demo: PickPlaceFlow, config: dict):
    approach_dist = config.get("approach_distance", 0.08)
    retreat_height = config.get("retreat_height", 0.05)
    turn_deg = config.get("turn_deg_at_place", 90)

    # ── 阶段1: 预抓取 ────────────────────────────────
    demo.get_logger().info("=" * 40)
    demo.get_logger().info("阶段1: 预抓取")
    demo.get_logger().info("=" * 40)

    pre_grasp = config["pre_grasp"]
    left_pre = dict_to_pose(pre_grasp["left"])
    right_pre = dict_to_pose(pre_grasp["right"])

    # 先移躯干 (turn=0)
    ok = demo.move_torso(pitch=0.0, turn=0.0)
    if not ok:
        demo.get_logger().warn("躯干移动失败，继续尝试双臂")

    ok = demo.move_dual_arms(left_pre, right_pre)
    if not ok:
        demo.get_logger().error("预抓取失败，终止")
        return
    time.sleep(0.5)

    # ── 阶段2: 前伸抓取 ──────────────────────────────
    demo.get_logger().info("=" * 40)
    demo.get_logger().info("阶段2: 前伸抓取 (沿吸盘方向前伸 {:.0f}cm)".format(approach_dist * 100))
    demo.get_logger().info("=" * 40)

    left_grasp = offset_pose_along_z(left_pre, -approach_dist)
    right_grasp = offset_pose_along_z(right_pre, -approach_dist)

    ok = demo.move_dual_arms(left_grasp, right_grasp)
    if not ok:
        demo.get_logger().error("前伸抓取失败，终止")
        return
    time.sleep(0.3)

    # 附着箱子到双臂末端
    demo.attach_box("left_box", LEFT_TIP, touch_links=LEFT_TOUCH_LINKS)
    demo.attach_box("right_box", RIGHT_TIP, touch_links=RIGHT_TOUCH_LINKS)
    time.sleep(0.5)

    # ── 阶段3: 后退到安全位 ──────────────────────────
    demo.get_logger().info("=" * 40)
    demo.get_logger().info("阶段3: 后退到安全位")
    demo.get_logger().info("=" * 40)

    # 后退: 回到预抓取位姿 + 向上偏移 retreat_height
    left_retreat = offset_pose_along_z(left_grasp, approach_dist + retreat_height)
    right_retreat = offset_pose_along_z(right_grasp, approach_dist + retreat_height)

    ok = demo.move_dual_arms(left_retreat, right_retreat)
    if not ok:
        demo.get_logger().error("后退失败，终止")
        return
    time.sleep(0.5)

    # ── 阶段4: 放置到侧边 ────────────────────────────
    demo.get_logger().info("=" * 40)
    demo.get_logger().info("阶段4: 放置 (turn 旋转 {:.0f}°)".format(turn_deg))
    demo.get_logger().info("=" * 40)

    # 躯干旋转到放置方向
    ok = demo.move_torso(pitch=0.0, turn=math.radians(turn_deg))
    if not ok:
        demo.get_logger().warn("躯干旋转失败")

    place = config["place"]
    left_place = dict_to_pose(place["left"])
    right_place = dict_to_pose(place["right"])

    ok = demo.move_dual_arms(left_place, right_place)
    if not ok:
        demo.get_logger().error("放置失败，终止")
        return
    time.sleep(0.3)

    # 卸载箱子
    demo.detach_box("left_box", LEFT_TIP)
    demo.detach_box("right_box", RIGHT_TIP)
    time.sleep(0.3)

    # 从场景中移除箱子
    demo.remove_box_from_world("left_box")
    demo.remove_box_from_world("right_box")

    # ── 阶段5: 回位 ──────────────────────────────────
    demo.get_logger().info("=" * 40)
    demo.get_logger().info("阶段5: 回位")
    demo.get_logger().info("=" * 40)

    # 躯干回正
    ok = demo.move_torso(pitch=0.0, turn=0.0)
    if not ok:
        demo.get_logger().warn("躯干回位失败")

    # 双臂回 home (所有关节 = 0)
    home_targets = {j: 0.0 for j in ARM_JOINTS}
    ok = demo.move_dual_arms_joints(home_targets)
    if not ok:
        demo.get_logger().warn("双臂回位失败")

    demo.get_logger().info("Pick-Place 流程完成!")


def main():
    parser = argparse.ArgumentParser(description="双臂 Pick-Place 流程 Demo")
    parser.add_argument(
        "poses_file",
        help="位姿 JSON 文件 (pick_place_config.json 或 record_pose.py 输出)")
    parser.add_argument(
        "--velocity", type=float, default=0.3,
        help="速度缩放 0.0~1.0 (默认 0.3)")
    parser.add_argument(
        "--accel", type=float, default=0.2,
        help="加速度缩放 0.0~1.0 (默认 0.2)")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="仅规划不执行")
    parser.add_argument(
        "--approach", type=float, default=None,
        help="前伸距离 (米), 覆盖配置文件")
    parser.add_argument(
        "--retreat", type=float, default=None,
        help="后退额外高度 (米), 覆盖配置文件")
    parser.add_argument(
        "--turn", type=float, default=None,
        help="放置时 turn 角度 (度), 覆盖配置文件")
    args = parser.parse_args()

    filepath = args.poses_file
    if not os.path.exists(filepath):
        print(f"错误: 文件不存在 {filepath}")
        sys.exit(1)

    config = load_config(filepath)

    # 命令行覆盖
    if args.approach is not None:
        config["approach_distance"] = args.approach
    if args.retreat is not None:
        config["retreat_height"] = args.retreat
    if args.turn is not None:
        config["turn_deg_at_place"] = args.turn

    # 打印概要
    print("\n" + "=" * 60)
    print("双臂 Pick-Place 流程 Demo")
    print("=" * 60)
    pg = config["pre_grasp"]
    pl = config["place"]
    print(f"  预抓取: 左({pg['left']['x']:.2f},{pg['left']['y']:.2f},{pg['left']['z']:.2f})"
          f"  右({pg['right']['x']:.2f},{pg['right']['y']:.2f},{pg['right']['z']:.2f})")
    print(f"  放置:   左({pl['left']['x']:.2f},{pl['left']['y']:.2f},{pl['left']['z']:.2f})"
          f"  右({pl['right']['x']:.2f},{pl['right']['y']:.2f},{pl['right']['z']:.2f})")
    print(f"  前伸: {config.get('approach_distance', 0.08)*100:.0f}cm"
          f"  后退高度: {config.get('retreat_height', 0.05)*100:.0f}cm"
          f"  turn: {config.get('turn_deg_at_place', 90)}°")
    print(f"  速度/加速度: {args.velocity} / {args.accel}")
    if args.dry_run:
        print("  ** 纯规划模式 (不执行) **")
    print("=" * 60)

    input("\n按 Enter 开始 …")

    rclpy.init()
    demo = PickPlaceFlow(
        velocity_scale=args.velocity,
        accel_scale=args.accel,
        dry_run=args.dry_run,
    )
    try:
        run_flow(demo, config)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()