#!/usr/bin/env python3

import math
import sys
from typing import List

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PositionIKRequest,
    RobotState,
)
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node


def normalize_quaternion(qx: float, qy: float, qz: float, qw: float) -> List[float]:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        return [0.0, 0.0, 0.0, 1.0]
    return [qx / norm, qy / norm, qz / norm, qw / norm]


class MoveItPoseGoalTester(Node):
    def __init__(self) -> None:
        super().__init__("moveit_pose_goal_tester")

        self.declare_parameter("group_name", "left_arm")
        self.declare_parameter("ee_link", "leftjoint6")
        self.declare_parameter("reference_frame", "base_link")
        self.declare_parameter("target_x", 0.357)
        self.declare_parameter("target_y", -0.705)
        self.declare_parameter("target_z", 0.684)
        self.declare_parameter("target_qx", -0.502)
        self.declare_parameter("target_qy", 0.502)
        self.declare_parameter("target_qz", -0.498)
        self.declare_parameter("target_qw", 0.498)
        self.declare_parameter("joint_tolerance", 0.01)
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("num_planning_attempts", 5)
        self.declare_parameter("max_velocity_scaling_factor", 0.2)
        self.declare_parameter("max_acceleration_scaling_factor", 0.2)
        self.declare_parameter("plan_only", False)
        self.declare_parameter("wait_for_server_sec", 20.0)

        self._action_client = ActionClient(self, MoveGroup, "move_action")
        self._ik_client = self.create_client(GetPositionIK, "compute_ik")

    def _solve_ik(self, group_name: str, ee_link: str, reference_frame: str,
                  target_pose: Pose) -> List[str] | None:
        """Call /compute_ik to convert a Cartesian pose to joint values.
        Returns (joint_names, joint_positions) or None on failure."""
        if not self._ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/compute_ik service not available")
            return None

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = reference_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = target_pose

        ik_request = PositionIKRequest()
        ik_request.group_name = group_name
        ik_request.ik_link_name = ee_link
        ik_request.pose_stamped = pose_stamped
        ik_request.robot_state = RobotState()
        ik_request.robot_state.is_diff = True
        ik_request.avoid_collisions = True

        srv_request = GetPositionIK.Request()
        srv_request.ik_request = ik_request

        future = self._ik_client.call_async(srv_request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response is None or response.error_code.val != 1:
            code = response.error_code.val if response else "None"
            self.get_logger().error(f"IK solve failed: error_code={code}")
            return None

        joint_state = response.solution.joint_state
        self.get_logger().info(
            f"IK solved: {list(zip(joint_state.name, [f'{v:.4f}' for v in joint_state.position]))}"
        )
        return joint_state

    def _build_goal(self) -> MoveGroup.Goal | None:
        group_name = self.get_parameter("group_name").get_parameter_value().string_value
        ee_link = self.get_parameter("ee_link").get_parameter_value().string_value
        reference_frame = self.get_parameter("reference_frame").get_parameter_value().string_value

        qx, qy, qz, qw = normalize_quaternion(
            self.get_parameter("target_qx").get_parameter_value().double_value,
            self.get_parameter("target_qy").get_parameter_value().double_value,
            self.get_parameter("target_qz").get_parameter_value().double_value,
            self.get_parameter("target_qw").get_parameter_value().double_value,
        )

        target_pose = Pose()
        target_pose.position.x = self.get_parameter("target_x").get_parameter_value().double_value
        target_pose.position.y = self.get_parameter("target_y").get_parameter_value().double_value
        target_pose.position.z = self.get_parameter("target_z").get_parameter_value().double_value
        target_pose.orientation.x = qx
        target_pose.orientation.y = qy
        target_pose.orientation.z = qz
        target_pose.orientation.w = qw

        self.get_logger().info(
            f"Target pose: position=({target_pose.position.x:.3f}, "
            f"{target_pose.position.y:.3f}, {target_pose.position.z:.3f}) "
            f"orientation=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})"
        )

        # Step 1: solve IK to get joint values
        joint_state = self._solve_ik(group_name, ee_link, reference_frame, target_pose)
        if joint_state is None:
            return None

        # Step 2: build JointConstraint goal (like setApproximateJointValueTarget)
        joint_tolerance = self.get_parameter("joint_tolerance").get_parameter_value().double_value
        goal_constraints = Constraints()
        for name, position in zip(joint_state.name, joint_state.position):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = joint_tolerance
            jc.tolerance_below = joint_tolerance
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)

        request = MotionPlanRequest()
        request.group_name = group_name
        request.num_planning_attempts = (
            self.get_parameter("num_planning_attempts").get_parameter_value().integer_value
        )
        request.allowed_planning_time = (
            self.get_parameter("allowed_planning_time").get_parameter_value().double_value
        )
        request.max_velocity_scaling_factor = (
            self.get_parameter("max_velocity_scaling_factor").get_parameter_value().double_value
        )
        request.max_acceleration_scaling_factor = (
            self.get_parameter("max_acceleration_scaling_factor").get_parameter_value().double_value
        )
        request.start_state.is_diff = True
        request.goal_constraints = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = self.get_parameter("plan_only").get_parameter_value().bool_value
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        return goal

    def run(self) -> int:
        wait_timeout = self.get_parameter("wait_for_server_sec").get_parameter_value().double_value
        if not self._action_client.wait_for_server(timeout_sec=wait_timeout):
            self.get_logger().error("MoveIt move_action server not available")
            return 1

        goal = self._build_goal()
        if goal is None:
            return 5

        self.get_logger().info(
            f"Sending MoveIt goal: group={goal.request.group_name}"
            f" joints={[jc.joint_name for jc in goal.request.goal_constraints[0].joint_constraints]}"
        )

        send_goal_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("MoveIt goal rejected")
            return 2

        self.get_logger().info("MoveIt goal accepted, waiting for result")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error("MoveIt result missing")
            return 3

        result = wrapped_result.result
        error_code = result.error_code.val
        planned_points = len(result.planned_trajectory.joint_trajectory.points)
        executed_points = len(result.executed_trajectory.joint_trajectory.points)
        self.get_logger().info(
            f"MoveIt finished: error_code={error_code}"
            f" planned_points={planned_points}"
            f" executed_points={executed_points}"
        )
        return 0 if error_code == 1 else 4


def main() -> None:
    rclpy.init(args=sys.argv)
    node = MoveItPoseGoalTester()
    exit_code = node.run()
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
