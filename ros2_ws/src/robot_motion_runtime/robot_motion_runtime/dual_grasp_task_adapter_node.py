from __future__ import annotations

import re
import threading
import time
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robot_motion_interfaces.msg import AttachedBox, RobotMotionState, TaskReceipt
from robot_motion_interfaces.srv import RunDualArmPoseTask, RunDualGraspTask
from robot_motion_runtime.box_pair_task_adapter_node import (
    default_loaded_goal,
    forward_x_orientation,
    make_pose,
    seed_or_default,
    top_suction_orientation,
)
from robot_motion_runtime.common import RuntimeStatusPublisher, clamp_motion_scale


def normalize_grasp_mode(value: str) -> str:
    normalized = value.strip().lower()
    aliases = {
        "": "front",
        "front": "front",
        "side": "front",
        "side_suction": "front",
        "top": "top_suction",
        "top_suction": "top_suction",
        "down": "top_suction",
    }
    if normalized not in aliases:
        raise ValueError(f"unsupported grasp mode: {value}")
    return aliases[normalized]


def safe_id(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_]+", "_", value.strip())
    return cleaned.strip("_") or "task"


def make_attached_box_for_task(side: str, task_id: str, mode: str) -> AttachedBox:
    top_suction = mode == "top_suction"
    out = AttachedBox()
    out.id = f"carried_{side}_{safe_id(task_id)}"
    out.box_id = 0
    out.side = side
    out.grasp_mode = mode
    out.link_name = f"{side}_tool0"
    out.center_in_link.orientation.w = 1.0
    out.size = Vector3()
    if top_suction:
        out.center_in_link.position.z = 0.4 * 0.5
        out.size.x = 0.3
        out.size.y = 0.4
        out.size.z = 0.4
    else:
        out.center_in_link.position.z = 0.3 * 0.5
        out.size.x = 0.4
        out.size.y = 0.4
        out.size.z = 0.3
    return out


class DualGraspTaskAdapterNode(Node):
    """External task adapter for whole-machine dual-grasp requests.

    Public contract:
      RunDualGraspTask: two end-effector positions + two grasp modes.
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
        self.declare_parameter("default_candidate_limit", 8)
        self.declare_parameter("default_planning_mode", "shortcut")
        self.declare_parameter("default_velocity_scale", 1.0)
        self.declare_parameter("default_acceleration_scale", 1.0)

        self.service_name = str(self.get_parameter("service_name").value)
        self.pose_task_service = str(self.get_parameter("pose_task_service").value)
        self.receipt_topic = str(self.get_parameter("receipt_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.service_timeout_s = float(self.get_parameter("service_timeout_s").value)
        self.default_frame_id = str(self.get_parameter("default_frame_id").value)
        self.default_fixed_updown = float(self.get_parameter("default_fixed_updown").value)
        self.default_candidate_limit = int(self.get_parameter("default_candidate_limit").value)
        self.default_planning_mode = str(self.get_parameter("default_planning_mode").value)
        self.default_velocity_scale = float(self.get_parameter("default_velocity_scale").value)
        self.default_acceleration_scale = float(self.get_parameter("default_acceleration_scale").value)

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
            "external dual-grasp task adapter; emits TaskReceipt and calls RunDualArmPoseTask",
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

    def pose_stamped(self, frame_id: str, x: float, y: float, z: float, mode: str) -> PoseStamped:
        orientation = top_suction_orientation() if mode == "top_suction" else forward_x_orientation()
        out = PoseStamped()
        out.header.frame_id = frame_id
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose = make_pose(x, y, z, orientation)
        return out

    def seed_state(self):
        if self.latest_state is not None and self.latest_state.joint_state.name:
            return self.latest_state.joint_state
        return seed_or_default(JointState(), self.default_fixed_updown)

    def fixed_updown(self) -> float:
        if self.latest_state is None:
            return self.default_fixed_updown
        joint_state = self.latest_state.joint_state
        if "updown" not in joint_state.name:
            return self.default_fixed_updown
        index = joint_state.name.index("updown")
        if index >= len(joint_state.position):
            return self.default_fixed_updown
        return float(joint_state.position[index])

    def on_run_dual_grasp_task(self, request, response):
        started = time.monotonic()
        task_id = (
            request.task_id.strip()
            or request.context.request_id.strip()
            or f"dual_grasp_{int(started * 1000.0)}"
        )
        response.task_id = task_id
        response.state = "accepted"
        self.publish_receipt(task_id, "accepted", "task accepted")
        self.status.mark_running(f"{task_id} execute={request.execute} dry_run={request.dry_run}")
        try:
            left_mode = normalize_grasp_mode(request.left_grasp_mode)
            right_mode = normalize_grasp_mode(request.right_grasp_mode)
            frame_id = (
                request.frame_id.strip()
                or request.context.frame_id.strip()
                or self.default_frame_id
            )

            pose_request = RunDualArmPoseTask.Request()
            pose_request.context = request.context
            pose_request.context.request_id = task_id
            pose_request.context.frame_id = frame_id
            pose_request.context.stamp = self.get_clock().now().to_msg()
            pose_request.seed_state = self.seed_state()
            fixed_updown = self.fixed_updown()
            pose_request.left_target = self.pose_stamped(
                frame_id,
                request.left_position.x,
                request.left_position.y,
                request.left_position.z,
                left_mode,
            )
            pose_request.right_target = self.pose_stamped(
                frame_id,
                request.right_position.x,
                request.right_position.y,
                request.right_position.z,
                right_mode,
            )
            pose_request.attached_boxes = [
                make_attached_box_for_task("left", task_id, left_mode),
                make_attached_box_for_task("right", task_id, right_mode),
            ]
            pose_request.loaded_goal_family = [default_loaded_goal(fixed_updown)]
            pose_request.fixed_updown = fixed_updown
            pose_request.left_top_suction = left_mode == "top_suction"
            pose_request.right_top_suction = right_mode == "top_suction"
            pose_request.candidate_limit = self.default_candidate_limit
            pose_request.planning_mode = self.default_planning_mode
            pose_request.execute = bool(request.execute)
            pose_request.dry_run = bool(request.dry_run)
            pose_request.velocity_scale = clamp_motion_scale(
                request.velocity_scale, self.default_velocity_scale
            )
            pose_request.acceleration_scale = clamp_motion_scale(
                request.acceleration_scale, self.default_acceleration_scale
            )

            self.publish_receipt(task_id, "running", "task planning/execution started")
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
