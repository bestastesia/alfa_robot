#!/usr/bin/env python3
"""
测试 left_arm_full / right_arm_full 规划组的脚本
验证规划时 turn 和 updown 是否参与

功能：
- 显示当前末端位姿（而非关节角度）
- 左右臂切换规划
- 一臂规划时另一臂保持不动（被动）

使用方法：
1. 先启动 MoveIt: ros2 launch alfa_robot_moveit_config demo.launch.py
2. 运行此脚本: python3 test_arm_full_planning.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
from moveit_msgs.srv import GetMotionPlan, GetPositionIK, GetPlanningScene
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from moveit_msgs.action import MoveGroup
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
import tf2_ros
import time


class ArmFullPlanner(Node):
    def __init__(self):
        super().__init__('test_arm_full_planner')

        # TF buffer for getting end effector poses
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 规划服务客户端
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')

        # MoveGroup action 客户端
        self.move_action_client = ActionClient(self, MoveGroup, '/move_action')

        # 关节状态订阅
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info("等待 MoveIt 服务...")
        # 使用异步方式等待服务
        start_time = time.time()
        timeout = 10.0

        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.plan_client.service_is_ready() and self.ik_client.service_is_ready():
                break
            self.get_logger().info('等待规划服务...')

        if not (self.plan_client.service_is_ready() and self.ik_client.service_is_ready()):
            self.get_logger().error("MoveIt 服务不可用，请确保 MoveIt 已启动")
            raise RuntimeError("MoveIt 服务不可用")

        self.get_logger().info("MoveIt 服务已就绪")

        # 等待获取关节状态
        self.get_logger().info("等待关节状态...")
        timeout = 5.0
        start = time.time()
        while self.current_joint_state is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_joint_state:
            self.get_logger().info("关节状态已接收")
        else:
            self.get_logger().warn("未收到关节状态，将使用默认值")

        # 当前选择的臂
        self.current_arm = "left"

    def joint_state_callback(self, msg):
        """存储当前关节状态"""
        self.current_joint_state = msg

    def get_current_joint_values(self, arm="left"):
        """获取指定臂的所有关节值（包括躯干）"""
        if self.current_joint_state is None:
            return {}

        # 定义关节顺序
        joint_names = ["turn", "updown"]
        if arm == "left":
            joint_names.extend(["left_joint1", "left_joint2",
                               "left_joint3", "left_joint4", "left_joint5", "left_joint6"])
        else:
            joint_names.extend(["right_joint1", "right_joint2",
                               "right_joint3", "right_joint4", "right_joint5", "right_joint6"])

        result = {}
        for name in joint_names:
            try:
                idx = self.current_joint_state.name.index(name)
                result[name] = self.current_joint_state.position[idx]
            except ValueError:
                result[name] = 0.0

        return result

    def get_all_joint_values(self):
        """获取所有关节值"""
        if self.current_joint_state is None:
            return {}
        result = {}
        for i, name in enumerate(self.current_joint_state.name):
            result[name] = self.current_joint_state.position[i]
        return result

    def get_end_effector_pose(self, arm="left"):
        """获取末端执行器位姿"""
        ee_link = "left_joint6" if arm == "left" else "right_joint6"

        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link", ee_link, rclpy.time.Time()
            )

            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            return pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"无法获取 {ee_link} 位姿: {e}")
            return None

    def print_current_state(self):
        """打印当前状态"""
        self.get_logger().info("=" * 60)

        # 显示左臂末端位姿
        left_pose = self.get_end_effector_pose("left")
        if left_pose:
            self.get_logger().info("左臂末端位姿 (left_joint6):")
            self.get_logger().info(f"  位置: x={left_pose.position.x:.4f}, "
                                   f"y={left_pose.position.y:.4f}, "
                                   f"z={left_pose.position.z:.4f}")
            self.get_logger().info(f"  姿态: x={left_pose.orientation.x:.4f}, "
                                   f"y={left_pose.orientation.y:.4f}, "
                                   f"z={left_pose.orientation.z:.4f}, "
                                   f"w={left_pose.orientation.w:.4f}")

        # 显示右臂末端位姿
        right_pose = self.get_end_effector_pose("right")
        if right_pose:
            self.get_logger().info("-" * 60)
            self.get_logger().info("右臂末端位姿 (right_joint6):")
            self.get_logger().info(f"  位置: x={right_pose.position.x:.4f}, "
                                   f"y={right_pose.position.y:.4f}, "
                                   f"z={right_pose.position.z:.4f}")
            self.get_logger().info(f"  姿态: x={right_pose.orientation.x:.4f}, "
                                   f"y={right_pose.orientation.y:.4f}, "
                                   f"z={right_pose.orientation.z:.4f}, "
                                   f"w={right_pose.orientation.w:.4f}")

        self.get_logger().info("=" * 60)

    def get_group_info(self, arm="left"):
        """获取规划组信息"""
        if arm == "left":
            return {
                "group_name": "left_arm_with_base",
                "ee_link": "left_joint6",
                "controller": "left_arm_with_base_controller",
                "joints": ["updown", "left_joint1", "left_joint2", "left_joint3",
                          "left_joint4", "left_joint5", "left_joint6"]
            }
        else:
            return {
                "group_name": "right_arm_with_base",
                "ee_link": "right_joint6",
                "controller": "right_arm_with_base_controller",
                "joints": ["updown", "right_joint1", "right_joint2", "right_joint3",
                          "right_joint4", "right_joint5", "right_joint6"]
            }

    def plan_to_joint_values(self, joint_values, arm="left"):
        """
        规划到目标关节值

        Args:
            joint_values: 字典，关节名到目标值的映射
            arm: "left" 或 "right"

        Returns:
            规划结果轨迹，失败返回 None
        """
        group_info = self.get_group_info(arm)
        group_name = group_info["group_name"]

        self.get_logger().info(f"规划到目标关节值 (组: {group_name})...")

        # 创建请求
        request = GetMotionPlan.Request()
        motion_plan_request = MotionPlanRequest()

        # 设置规划组
        motion_plan_request.group_name = group_name

        # 设置起始状态为当前状态
        if self.current_joint_state:
            motion_plan_request.start_state.joint_state = self.current_joint_state

        # 设置目标约束 - 使用关节约束
        goal_constraints = Constraints()

        for joint_name in group_info["joints"]:
            if joint_name in joint_values:
                jc = JointConstraint()
                jc.joint_name = joint_name
                jc.position = float(joint_values[joint_name])
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                goal_constraints.joint_constraints.append(jc)

        motion_plan_request.goal_constraints.append(goal_constraints)

        # 设置规划参数
        motion_plan_request.allowed_planning_time = 5.0
        motion_plan_request.num_planning_attempts = 5
        motion_plan_request.max_velocity_scaling_factor = 0.5
        motion_plan_request.max_acceleration_scaling_factor = 0.5

        request.motion_plan_request = motion_plan_request

        # 调用规划服务
        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("规划服务调用失败")
            return None

        response = future.result()

        if response.motion_plan_response.error_code.val != 1:
            self.get_logger().error(f"规划失败，错误码: {response.motion_plan_response.error_code.val}")
            return None

        trajectory = response.motion_plan_response.trajectory
        self.get_logger().info(f"规划成功! 轨迹点数: {len(trajectory.joint_trajectory.points)}")

        # 打印轨迹中的关节名，验证 turn 和 updown 是否参与
        self.get_logger().info("轨迹包含的关节:")
        for jn in trajectory.joint_trajectory.joint_names:
            self.get_logger().info(f"  - {jn}")

        return trajectory

    def plan_to_pose(self, target_pose, arm="left"):
        """
        规划到目标位姿
        使用位姿约束直接规划，让规划器自动调整所有关节（包括躯干）

        Args:
            target_pose: geometry_msgs/Pose 或 PoseStamped
            arm: "left" 或 "right"

        Returns:
            规划结果轨迹，失败返回 None
        """
        from moveit_msgs.msg import PositionConstraint, OrientationConstraint, BoundingVolume
        from shape_msgs.msg import SolidPrimitive

        group_info = self.get_group_info(arm)
        group_name = group_info["group_name"]
        ee_link = group_info["ee_link"]

        self.get_logger().info(f"规划到目标位姿 (组: {group_name}, 末端: {ee_link})...")

        # 处理输入位姿
        if isinstance(target_pose, PoseStamped):
            pose = target_pose.pose
            frame_id = target_pose.header.frame_id
        else:
            pose = target_pose
            frame_id = "base_link"

        # 创建规划请求
        request = GetMotionPlan.Request()
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = group_name

        # 设置起始状态为当前状态
        if self.current_joint_state:
            motion_plan_request.start_state.joint_state = self.current_joint_state

        # 设置目标约束 - 使用位姿约束
        goal_constraints = Constraints()

        # 位置约束
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = frame_id
        pos_constraint.link_name = ee_link
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0

        # 使用小球作为约束区域
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]  # 1cm 半径的球
        pos_constraint.constraint_region.primitives.append(primitive)

        # 约束区域的位置 = 目标位置
        region_pose = Pose()
        region_pose.position.x = pose.position.x
        region_pose.position.y = pose.position.y
        region_pose.position.z = pose.position.z
        region_pose.orientation.w = 1.0
        pos_constraint.constraint_region.primitive_poses.append(region_pose)
        pos_constraint.weight = 1.0

        goal_constraints.position_constraints.append(pos_constraint)

        # 姿态约束
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = frame_id
        ori_constraint.orientation = pose.orientation
        ori_constraint.link_name = ee_link
        ori_constraint.absolute_x_axis_tolerance = 0.1  # 放宽容差
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0

        goal_constraints.orientation_constraints.append(ori_constraint)

        motion_plan_request.goal_constraints.append(goal_constraints)

        # 设置规划参数
        motion_plan_request.allowed_planning_time = 10.0
        motion_plan_request.num_planning_attempts = 10
        motion_plan_request.max_velocity_scaling_factor = 0.3
        motion_plan_request.max_acceleration_scaling_factor = 0.3

        request.motion_plan_request = motion_plan_request

        self.get_logger().info("开始规划（位姿约束，包含躯干关节）...")

        # 调用规划服务
        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("规划服务调用失败")
            return None

        response = future.result()

        if response.motion_plan_response.error_code.val != 1:
            self.get_logger().error(f"规划失败，错误码: {response.motion_plan_response.error_code.val}")
            return None

        trajectory = response.motion_plan_response.trajectory
        self.get_logger().info(f"规划成功! 轨迹点数: {len(trajectory.joint_trajectory.points)}")

        # 打印轨迹中的关节名，验证 turn 和 updown 是否参与
        self.get_logger().info("轨迹包含的关节:")
        for jn in trajectory.joint_trajectory.joint_names:
            self.get_logger().info(f"  - {jn}")

        # 打印起始和终点的关节值变化
        if len(trajectory.joint_trajectory.points) > 0:
            start_point = trajectory.joint_trajectory.points[0]
            end_point = trajectory.joint_trajectory.points[-1]

            self.get_logger().info("关节值变化:")
            for i, jn in enumerate(trajectory.joint_trajectory.joint_names):
                start_val = start_point.positions[i] if i < len(start_point.positions) else 0
                end_val = end_point.positions[i] if i < len(end_point.positions) else 0
                delta = end_val - start_val
                if abs(delta) > 0.001:
                    self.get_logger().info(f"  {jn}: {start_val:.4f} -> {end_val:.4f} (变化: {delta:.4f})")

        return trajectory

    def plan_relative(self, arm="left", turn_delta=0.0, updown_delta=0.0):
        """
        规划相对运动（只改变躯干，臂保持当前）

        Args:
            arm: "left" 或 "right"
            turn_delta: turn 关节增量
            updown_delta: updown 关节增量

        Returns:
            规划结果轨迹
        """
        group_info = self.get_group_info(arm)

        self.get_logger().info(f"规划相对运动: turn+{turn_delta:.4f}, updown+{updown_delta:.4f}")

        # 获取当前所有关节值
        current = self.get_all_joint_values()

        # 构建目标：只改变 turn 和 updown，其他保持
        target = current.copy()

        if turn_delta != 0.0:
            target["turn"] = current.get("turn", 0.0) + turn_delta
            # 限制范围
            target["turn"] = max(-3.14159, min(3.14159, target["turn"]))

        if updown_delta != 0.0:
            target["updown"] = current.get("updown", 0.0) + updown_delta
            # 限制范围
            target["updown"] = max(0.0, min(1.0, target["updown"]))

        return self.plan_to_joint_values(target, arm)

    def execute_trajectory(self, trajectory, arm="left"):
        """
        执行轨迹

        Args:
            trajectory: RobotTrajectory 或 JointTrajectory
            arm: "left" 或 "right"

        Returns:
            是否成功
        """
        group_info = self.get_group_info(arm)

        self.get_logger().info(f"执行轨迹 (控制器: {group_info['controller']})...")

        # 提取 JointTrajectory
        if hasattr(trajectory, 'joint_trajectory'):
            joint_traj = trajectory.joint_trajectory
        else:
            joint_traj = trajectory

        # 使用 MoveGroup action 执行
        goal = MoveGroup.Goal()
        goal.request.group_name = group_info["group_name"]

        if self.current_joint_state:
            goal.request.start_state.joint_state = self.current_joint_state

        # 设置轨迹目标
        goal.trajectory = joint_traj

        # 设置规划选项
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        # 发送目标
        self.move_action_client.wait_for_server()
        send_goal_future = self.move_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("执行请求被拒绝")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code.val == 1:
            self.get_logger().info("执行成功!")
            return True
        else:
            self.get_logger().error(f"执行失败，错误码: {result.error_code.val}")
            return False


def main():
    rclpy.init()

    node = ArmFullPlanner()

    try:
        # 打印当前状态
        rclpy.spin_once(node, timeout_sec=0.5)
        node.print_current_state()

        print("\n" + "=" * 60)
        print("测试 arm_full 规划组 (包含 turn, updown 躯干关节)")
        print("规划时另一臂保持不动（被动）")
        print("=" * 60)

        current_arm = "left"

        while True:
            print(f"\n当前选择: 【{current_arm}臂】")
            print("\n选择操作:")
            print("  0. 切换左/右臂")
            print("  1. 规划到目标位姿 (绝对位置)")
            print("  2. 规划到目标关节值")
            print("  3. 打印当前末端位姿")
            print("  4. 相对平移 (xyz增量，姿态不变)")
            print("  5. 测试: updown + 0.1m (躯干升降)")
            print("  6. 测试: turn + 0.2rad (躯干旋转)")
            print("  7. 退出")

            choice = input("\n请输入选项 (0-7): ").strip()

            if choice == "0":
                current_arm = "right" if current_arm == "left" else "left"
                print(f"已切换到 {current_arm}臂")

            elif choice == "1":
                try:
                    print(f"\n输入目标位姿 (相对于 base_link) - {current_arm}臂:")
                    print("提示: 位置必填，姿态可跳过（保持当前姿态）")

                    x = float(input("  x (m): "))
                    y = float(input("  y (m): "))
                    z = float(input("  z (m): "))

                    # 获取当前姿态作为默认值
                    current_pose = node.get_end_effector_pose(current_arm)

                    target_pose = PoseStamped()
                    target_pose.header.frame_id = "base_link"
                    target_pose.pose.position.x = x
                    target_pose.pose.position.y = y
                    target_pose.pose.position.z = z

                    # 姿态输入（可选）
                    print("  姿态 (四元数)，回车使用当前姿态:")
                    ox_input = input(f"    ox (当前: {current_pose.orientation.x:.4f}): ").strip()
                    oy_input = input(f"    oy (当前: {current_pose.orientation.y:.4f}): ").strip()
                    oz_input = input(f"    oz (当前: {current_pose.orientation.z:.4f}): ").strip()
                    ow_input = input(f"    ow (当前: {current_pose.orientation.w:.4f}): ").strip()

                    if ox_input or oy_input or oz_input or ow_input:
                        target_pose.pose.orientation.x = float(ox_input) if ox_input else 0.0
                        target_pose.pose.orientation.y = float(oy_input) if oy_input else 0.0
                        target_pose.pose.orientation.z = float(oz_input) if oz_input else 0.0
                        target_pose.pose.orientation.w = float(ow_input) if ow_input else 1.0
                    else:
                        # 使用当前姿态
                        target_pose.pose.orientation = current_pose.orientation

                    trajectory = node.plan_to_pose(target_pose, current_arm)
                    if trajectory:
                        execute = input("执行轨迹? (y/n): ").strip().lower()
                        if execute == 'y':
                            node.execute_trajectory(trajectory, current_arm)
                            rclpy.spin_once(node, timeout_sec=0.5)
                            node.print_current_state()

                except ValueError as e:
                    print(f"输入错误: {e}")

            elif choice == "2":
                try:
                    group_info = node.get_group_info(current_arm)
                    joint_names = group_info["joints"]

                    print(f"\n输入目标关节值 - {current_arm}臂:")
                    print("提示: 只输入需要改变的关节，其他保持当前值")

                    current = node.get_all_joint_values()
                    target = current.copy()

                    for joint_name in joint_names:
                        val = input(f"  {joint_name} (当前: {current.get(joint_name, 0.0):.4f}, 回车跳过): ").strip()
                        if val:
                            target[joint_name] = float(val)

                    trajectory = node.plan_to_joint_values(target, current_arm)
                    if trajectory:
                        execute = input("执行轨迹? (y/n): ").strip().lower()
                        if execute == 'y':
                            node.execute_trajectory(trajectory, current_arm)
                            rclpy.spin_once(node, timeout_sec=0.5)
                            node.print_current_state()

                except ValueError as e:
                    print(f"输入错误: {e}")

            elif choice == "3":
                rclpy.spin_once(node, timeout_sec=0.5)
                node.print_current_state()

            elif choice == "4":
                # 相对平移功能
                try:
                    print(f"\n相对平移 - {current_arm}臂:")
                    print("提示: 输入xyz增量，姿态保持不变")

                    dx = input("  dx (m, 回车跳过): ").strip()
                    dy = input("  dy (m, 回车跳过): ").strip()
                    dz = input("  dz (m, 回车跳过): ").strip()

                    if not (dx or dy or dz):
                        print("未输入任何增量")
                        continue

                    dx_val = float(dx) if dx else 0.0
                    dy_val = float(dy) if dy else 0.0
                    dz_val = float(dz) if dz else 0.0

                    # 获取当前位姿
                    rclpy.spin_once(node, timeout_sec=0.5)
                    current_pose = node.get_end_effector_pose(current_arm)

                    if current_pose is None:
                        print("无法获取当前位姿")
                        continue

                    # 构建目标位姿：当前位置 + 增量，姿态不变
                    target_pose = PoseStamped()
                    target_pose.header.frame_id = "base_link"
                    target_pose.pose.position.x = current_pose.position.x + dx_val
                    target_pose.pose.position.y = current_pose.position.y + dy_val
                    target_pose.pose.position.z = current_pose.position.z + dz_val
                    target_pose.pose.orientation = current_pose.orientation

                    print(f"\n目标位姿:")
                    print(f"  位置: x={target_pose.pose.position.x:.4f}, "
                          f"y={target_pose.pose.position.y:.4f}, "
                          f"z={target_pose.pose.position.z:.4f}")

                    trajectory = node.plan_to_pose(target_pose, current_arm)
                    if trajectory:
                        execute = input("执行轨迹? (y/n): ").strip().lower()
                        if execute == 'y':
                            node.execute_trajectory(trajectory, current_arm)
                            rclpy.spin_once(node, timeout_sec=0.5)
                            node.print_current_state()

                except ValueError as e:
                    print(f"输入错误: {e}")

            elif choice == "5":
                print(f"\n测试: 将 updown 增加 0.1m - {current_arm}臂")
                trajectory = node.plan_relative(current_arm, turn_delta=0.0, updown_delta=0.1)
                if trajectory:
                    execute = input("执行轨迹? (y/n): ").strip().lower()
                    if execute == 'y':
                        node.execute_trajectory(trajectory, current_arm)
                        rclpy.spin_once(node, timeout_sec=0.5)
                        node.print_current_state()

            elif choice == "6":
                print(f"\n测试: 将 turn 增加 0.2 rad - {current_arm}臂")
                trajectory = node.plan_relative(current_arm, turn_delta=0.2, updown_delta=0.0)
                if trajectory:
                    execute = input("执行轨迹? (y/n): ").strip().lower()
                    if execute == 'y':
                        node.execute_trajectory(trajectory, current_arm)
                        rclpy.spin_once(node, timeout_sec=0.5)
                        node.print_current_state()

            elif choice == "7":
                print("退出")
                break

            else:
                print("无效选项")

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
