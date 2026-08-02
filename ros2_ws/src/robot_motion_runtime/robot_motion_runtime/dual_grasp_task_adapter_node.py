from __future__ import annotations

import math
import re
import threading
import time
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robot_motion_interfaces.msg import (
    ArmExtractPolicy,
    AttachedBox,
    DualGraspStrategy,
    RobotMotionState,
    TaskReceipt,
)
from robot_motion_interfaces.srv import RunDualArmPoseTask, RunDualGraspTask
from robot_motion_runtime.box_pair_task_adapter_node import (
    default_loaded_goal,
    make_pose,
    seed_or_default,
)
from robot_motion_runtime.common import RuntimeStatusPublisher
from robot_motion_runtime.dual_grasp_strategy import (
    ArmExtractPolicyValue,
    BOX_DEPTH_M,
    BOX_HEIGHT_M,
    BOX_ROW_COUNT,
    BOX_WIDTH_M,
    BOTTOM_ROW_FRONT_CENTER_Z_M,
    OUTER_BOX_GRASP_LATERAL_OFFSET_M,
    ROW_MATCH_TOLERANCE_M,
    TOP_SUCTION_FIRST_ROW,
    DualGraspStrategyValue,
    pose6d_value,
    resolve_front_face_dual_grasp_strategy,
)


def safe_id(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_]+", "_", value.strip())
    return cleaned.strip("_") or "task"


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    half_roll = 0.5 * float(roll)
    half_pitch = 0.5 * float(pitch)
    half_yaw = 0.5 * float(yaw)
    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)
    out = Quaternion()
    out.w = cr * cp * cy + sr * sp * sy
    out.x = sr * cp * cy - cr * sp * sy
    out.y = cr * sp * cy + sr * cp * sy
    out.z = cr * cp * sy - sr * sp * cy
    return out


def make_attached_box_for_task(side: str, task_id: str, mode: str) -> AttachedBox:
    top_suction = mode == "top_suction"
    out = AttachedBox()
    out.id = f"carried_{side}_{safe_id(task_id)}"
    out.box_id = 0
    out.side = side
    out.grasp_mode = mode
    out.link_name = f"{side}_tool0"
    out.center_in_link.orientation.w = 1.0
    out.center_in_link.position.y = (
        -OUTER_BOX_GRASP_LATERAL_OFFSET_M if side == "left"
        else OUTER_BOX_GRASP_LATERAL_OFFSET_M
    )
    out.size = Vector3()
    if top_suction:
        out.center_in_link.position.z = BOX_HEIGHT_M * 0.5
        out.size.x = BOX_DEPTH_M
        out.size.y = BOX_WIDTH_M
        out.size.z = BOX_HEIGHT_M
    else:
        out.center_in_link.position.z = BOX_DEPTH_M * 0.5
        out.size.x = BOX_HEIGHT_M
        out.size.y = BOX_WIDTH_M
        out.size.z = BOX_DEPTH_M
    return out


def arm_policy_message(value: ArmExtractPolicyValue) -> ArmExtractPolicy:
    out = ArmExtractPolicy()
    out.grasp_mode = value.grasp_mode
    out.require_full_detachment = value.require_full_detachment
    out.front_clearance_levels = value.front_clearance_levels
    out.retreat_priority = value.retreat_priority
    out.lift_priority = value.lift_priority
    out.pitch_priority = value.pitch_priority
    return out


def strategy_message(value: DualGraspStrategyValue) -> DualGraspStrategy:
    out = DualGraspStrategy()
    out.task_type = value.task_type
    out.name = value.name
    out.left = arm_policy_message(value.left)
    out.right = arm_policy_message(value.right)
    out.height_difference_m = value.height_difference_m
    return out


class DualGraspTaskAdapterNode(Node):
    """External task adapter for whole-machine dual-grasp requests.

    Public contract:
      RunDualGraspTask: two box front-face centre 6D poses.
      TaskReceipt: accepted/running/succeeded/failed state for the whole machine.

    Internal details remain hidden behind RunDualArmPoseTask.
    """

    def __init__(self) -> None:
        super().__init__("dual_grasp_task_adapter")
        self.declare_parameter("service_name", "/robot_motion/run_dual_grasp_task")
        self.declare_parameter("pose_task_service", "/robot_motion/run_dual_arm_pose_task")
        self.declare_parameter("receipt_topic", "/robot_motion/task_receipt")
        self.declare_parameter("state_topic", "/robot_motion/state")
        self.declare_parameter("service_timeout_s", 60.0)
        self.declare_parameter("default_frame_id", "base_link")
        self.declare_parameter("default_fixed_updown", 0.0)
        # updown 逻辑/URDF 与电机物理规划范围均为 [0, 0.7]。fixed_updown
        # 取当前车体状态,可能落在旧范围或越界,这里在"喂给规划前"就夹紧,不把越界值当 IK 种子。
        self.declare_parameter("updown_logical_lower_m", 0.0)
        self.declare_parameter("updown_logical_upper_m", 0.7)
        self.declare_parameter("default_candidate_limit", 8)
        self.declare_parameter("default_planning_mode", "shortcut")
        self.declare_parameter("default_velocity_scale", 1.0)
        self.declare_parameter("default_acceleration_scale", 1.0)
        self.declare_parameter("box_row_count", BOX_ROW_COUNT)
        self.declare_parameter("box_height_m", BOX_HEIGHT_M)
        self.declare_parameter("box_depth_m", BOX_DEPTH_M)
        self.declare_parameter("bottom_row_front_center_z_m", BOTTOM_ROW_FRONT_CENTER_Z_M)
        self.declare_parameter("row_match_tolerance_m", ROW_MATCH_TOLERANCE_M)
        self.declare_parameter("top_suction_first_row", TOP_SUCTION_FIRST_ROW)

        self.service_name = str(self.get_parameter("service_name").value)
        self.pose_task_service = str(self.get_parameter("pose_task_service").value)
        self.receipt_topic = str(self.get_parameter("receipt_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.service_timeout_s = float(self.get_parameter("service_timeout_s").value)
        self.default_frame_id = str(self.get_parameter("default_frame_id").value)
        self.default_fixed_updown = float(self.get_parameter("default_fixed_updown").value)
        self.updown_logical_lower_m = float(self.get_parameter("updown_logical_lower_m").value)
        self.updown_logical_upper_m = float(self.get_parameter("updown_logical_upper_m").value)
        self.default_candidate_limit = int(self.get_parameter("default_candidate_limit").value)
        self.default_planning_mode = str(self.get_parameter("default_planning_mode").value)
        self.default_velocity_scale = float(self.get_parameter("default_velocity_scale").value)
        self.default_acceleration_scale = float(self.get_parameter("default_acceleration_scale").value)
        self.box_row_count = int(self.get_parameter("box_row_count").value)
        self.box_height_m = float(self.get_parameter("box_height_m").value)
        self.box_depth_m = float(self.get_parameter("box_depth_m").value)
        self.bottom_row_front_center_z_m = float(
            self.get_parameter("bottom_row_front_center_z_m").value
        )
        self.row_match_tolerance_m = float(
            self.get_parameter("row_match_tolerance_m").value
        )
        self.top_suction_first_row = int(
            self.get_parameter("top_suction_first_row").value
        )

        self.callback_group = ReentrantCallbackGroup()
        self.latest_state: RobotMotionState | None = None
        self.state_sub = self.create_subscription(
            RobotMotionState,
            self.state_topic,
            self.on_state,
            10,
            callback_group=self.callback_group,
        )
        self.receipt_pub = self.create_publisher(TaskReceipt, self.receipt_topic, 10)
        self.pose_task_client = self.create_client(
            RunDualArmPoseTask,
            self.pose_task_service,
            callback_group=self.callback_group,
        )
        self.service = self.create_service(
            RunDualGraspTask,
            self.service_name,
            self.on_run_dual_grasp_task,
            callback_group=self.callback_group,
        )
        self.status = RuntimeStatusPublisher(
            self,
            self.service_name,
            "front-face 6D task adapter; derives rows/modes and calls RunDualArmPoseTask",
        )
        self.status.mark_ready(
            f"pose_task={self.pose_task_service}, receipt={self.receipt_topic}"
        )
        self.get_logger().info(
            f"DualGraspTask adapter ready: service={self.service_name} "
            f"pose_task={self.pose_task_service} receipt={self.receipt_topic}"
        )

    def on_state(self, state: RobotMotionState) -> None:
        if state.authoritative:
            self.latest_state = state

    def publish_receipt(self, task_id: str, state: str, message: str) -> None:
        receipt = TaskReceipt()
        receipt.task_id = task_id
        receipt.state = state
        receipt.stamp = self.get_clock().now().to_msg()
        receipt.message = message
        self.receipt_pub.publish(receipt)

    def call_pose_task(self, request: RunDualArmPoseTask.Request) -> RunDualArmPoseTask.Response:
        if not self.pose_task_client.wait_for_service(timeout_sec=self.service_timeout_s):
            raise TimeoutError(f"RunDualArmPoseTask service not available: {self.pose_task_service}")
        event = threading.Event()
        holder: dict[str, Any] = {}
        future = self.pose_task_client.call_async(request)

        def on_done(done_future):
            try:
                holder["response"] = done_future.result()
            except Exception as exc:  # pragma: no cover - defensive runtime path
                holder["error"] = exc
            event.set()

        future.add_done_callback(on_done)
        if not event.wait(timeout=self.service_timeout_s):
            raise TimeoutError("RunDualArmPoseTask call timed out")
        if "error" in holder:
            raise RuntimeError(f"RunDualArmPoseTask call failed: {holder['error']}")
        return holder["response"]

    def pose_stamped(self, pose_6d) -> PoseStamped:
        out = PoseStamped()
        out.header.frame_id = self.default_frame_id
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose = make_pose(
            pose_6d.x,
            pose_6d.y,
            pose_6d.z,
            quaternion_from_rpy(pose_6d.roll, pose_6d.pitch, pose_6d.yaw),
        )
        return out

    def seed_state(self):
        if self.latest_state is not None and self.latest_state.joint_state.name:
            return self.latest_state.joint_state
        return seed_or_default(JointState(), self.default_fixed_updown)

    def clamp_updown_logical(self, value: float) -> float:
        """把 updown 逻辑值夹紧到规划范围 [0, 0.7]；越界(如旧范围遗留值或车体
        当前停在范围外)时告警并夹紧，绝不把越界值当 IK 种子/规划输入。"""
        clamped = max(self.updown_logical_lower_m, min(self.updown_logical_upper_m, float(value)))
        if abs(clamped - float(value)) > 1e-6:
            self.get_logger().warn(
                f"updown 逻辑值 {float(value):.4f}m 超出规划范围 "
                f"[{self.updown_logical_lower_m}, {self.updown_logical_upper_m}]m，已夹紧到 {clamped:.4f}m"
            )
        return clamped

    def fixed_updown(self) -> float:
        if self.latest_state is None:
            return self.clamp_updown_logical(self.default_fixed_updown)
        joint_state = self.latest_state.joint_state
        if "updown" not in joint_state.name:
            return self.clamp_updown_logical(self.default_fixed_updown)
        index = joint_state.name.index("updown")
        if index >= len(joint_state.position):
            return self.clamp_updown_logical(self.default_fixed_updown)
        return self.clamp_updown_logical(float(joint_state.position[index]))

    def on_run_dual_grasp_task(self, request, response):
        started = time.monotonic()
        task_id = request.request_id.strip() or f"dual_grasp_{int(started * 1000.0)}"
        response.request_id = task_id
        response.state = "accepted"
        self.publish_receipt(task_id, "accepted", "task accepted")
        self.status.mark_running(f"{task_id} execute={request.execute}")
        try:
            left_front_face_pose = pose6d_value(request.left.pose_6d)
            right_front_face_pose = pose6d_value(request.right.pose_6d)
            resolution = resolve_front_face_dual_grasp_strategy(
                left_front_face_pose,
                right_front_face_pose,
                row_count=self.box_row_count,
                box_height_m=self.box_height_m,
                box_depth_m=self.box_depth_m,
                bottom_row_center_z_m=self.bottom_row_front_center_z_m,
                row_match_tolerance_m=self.row_match_tolerance_m,
                top_suction_first_row=self.top_suction_first_row,
            )
            strategy = resolution.strategy
            effective_left_pose = resolution.left_tool_pose
            effective_right_pose = resolution.right_tool_pose
            effective_left_mode = strategy.left.grasp_mode
            effective_right_mode = strategy.right.grasp_mode

            pose_request = RunDualArmPoseTask.Request()
            pose_request.context.request_id = task_id
            pose_request.context.frame_id = self.default_frame_id
            pose_request.context.stamp = self.get_clock().now().to_msg()
            pose_request.seed_state = self.seed_state()
            fixed_updown = self.fixed_updown()
            pose_request.left_target = self.pose_stamped(effective_left_pose)
            pose_request.right_target = self.pose_stamped(effective_right_pose)
            pose_request.attached_boxes = [
                make_attached_box_for_task("left", task_id, effective_left_mode),
                make_attached_box_for_task("right", task_id, effective_right_mode),
            ]
            pose_request.loaded_goal_family = [default_loaded_goal(fixed_updown)]
            pose_request.fixed_updown = fixed_updown
            pose_request.left_top_suction = effective_left_mode == "top_suction"
            pose_request.right_top_suction = effective_right_mode == "top_suction"
            pose_request.strategy = strategy_message(strategy)
            pose_request.candidate_limit = self.default_candidate_limit
            pose_request.planning_mode = self.default_planning_mode
            pose_request.execute = bool(request.execute)
            pose_request.dry_run = not bool(request.execute)
            pose_request.velocity_scale = self.default_velocity_scale
            pose_request.acceleration_scale = self.default_acceleration_scale

            self.publish_receipt(
                task_id,
                "running",
                f"task planning/execution started; strategy={strategy.name} "
                f"rows=({resolution.left_row.row_from_top},"
                f"{resolution.right_row.row_from_top}) "
                f"row_residuals=({resolution.left_row.residual_m:+.3f},"
                f"{resolution.right_row.residual_m:+.3f})m",
            )
            pose_response = self.call_pose_task(pose_request)
            response.success = bool(pose_response.success)
            response.state = "succeeded" if response.success else "failed"
            response.message = (
                f"{pose_response.message}; elapsed={(time.monotonic() - started) * 1000.0:.2f}ms"
            )
            self.publish_receipt(task_id, response.state, response.message)
            self.status.mark_done(response.success, response.message)
            return response
        except Exception as exc:  # pragma: no cover - runtime safety path
            response.success = False
            response.state = "failed"
            response.message = str(exc)
            self.publish_receipt(task_id, "failed", response.message)
            self.status.mark_done(False, response.message)
            return response


def main() -> None:
    rclpy.init()
    node = DualGraspTaskAdapterNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
