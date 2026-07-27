from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Vector3
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robot_motion_interfaces.msg import AttachedBox, RobotMotionState
from robot_motion_interfaces.srv import RunBoxPairTask, RunDualArmPoseTask
from robot_motion_runtime.common import RuntimeStatusPublisher, clamp_motion_scale
from robot_motion_runtime.dual_grasp_strategy import (
    BOX_DEPTH_M,
    BOX_HEIGHT_M,
    BOX_WIDTH_M,
    OUTER_BOX_GRASP_LATERAL_OFFSET_M,
)


DEFAULT_JOINT_NAMES = [
    "updown",
    "turn",
    "pitch",
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
]

FRONT_SUCTION_BOX_IDS = {1, 3, 4, 6}


@dataclass(frozen=True)
class BoxSpec:
    box_id: int
    x: float
    y: float
    z: float


def make_boxes(front_x: float, y_shift: float) -> dict[int, BoxSpec]:
    rows_top_to_bottom = [
        [(1, 0.5), (2, 0.0), (3, -0.5)],
        [(4, 0.5), (5, 0.0), (6, -0.5)],
        [(7, 0.5), (8, 0.0), (9, -0.5)],
        [(10, 0.5), (11, 0.0), (12, -0.5)],
        [(13, 0.5), (14, 0.0), (15, -0.5)],
    ]
    row_count = len(rows_top_to_bottom)
    boxes: dict[int, BoxSpec] = {}
    for row_index, row in enumerate(rows_top_to_bottom):
        z = (float(row_count - row_index) - 0.5) * BOX_HEIGHT_M
        for box_id, y in row:
            boxes[int(box_id)] = BoxSpec(int(box_id), float(front_x), float(y) + float(y_shift), z)
    return boxes


def quat_multiply(lhs: tuple[float, float, float, float], rhs: tuple[float, float, float, float]):
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quat_from_axis_angle(axis: str, angle: float) -> tuple[float, float, float, float]:
    half = 0.5 * angle
    s = math.sin(half)
    c = math.cos(half)
    if axis == "x":
        return (s, 0.0, 0.0, c)
    if axis == "y":
        return (0.0, s, 0.0, c)
    if axis == "z":
        return (0.0, 0.0, s, c)
    raise ValueError(f"unsupported axis: {axis}")


def normalize_quat(value: tuple[float, float, float, float]) -> Quaternion:
    x, y, z, w = value
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    out = Quaternion()
    out.x = x / norm
    out.y = y / norm
    out.z = z / norm
    out.w = w / norm
    return out


def forward_x_orientation() -> Quaternion:
    return normalize_quat(
        quat_multiply(
            (0.0, 0.70710678, 0.0, 0.70710678),
            quat_from_axis_angle("z", math.pi),
        )
    )


def top_suction_orientation() -> Quaternion:
    return normalize_quat(
        quat_multiply(
            (0.0, 1.0, 0.0, 0.0),
            quat_from_axis_angle("z", math.pi),
        )
    )


def make_pose(x: float, y: float, z: float, orientation: Quaternion) -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation = orientation
    return pose


def make_front_grasp_pose(box: BoxSpec, world_to_base_z: float) -> Pose:
    return make_pose(box.x, box.y, box.z - world_to_base_z, forward_x_orientation())


def make_top_suction_pose(
    box: BoxSpec,
    world_to_base_z: float,
    x_offset: float,
    z_offset: float,
) -> Pose:
    return make_pose(
        box.x + x_offset,
        box.y,
        box.z + z_offset - world_to_base_z,
        top_suction_orientation(),
    )


def normalize_grasp_mode(value: str, box_id: int) -> str:
    normalized = value.strip().lower()
    if normalized in ("", "auto"):
        return "front" if int(box_id) in FRONT_SUCTION_BOX_IDS else "top_suction"
    aliases = {
        "front": "front",
        "side": "front",
        "side_suction": "front",
        "top": "top_suction",
        "top_suction": "top_suction",
        "down": "top_suction",
    }
    if normalized not in aliases:
        raise ValueError(f"unsupported grasp mode '{value}' for box {box_id}")
    return aliases[normalized]


def seed_or_default(seed: JointState, fixed_updown: float) -> JointState:
    if seed.name:
        return seed
    out = JointState()
    out.name = list(DEFAULT_JOINT_NAMES)
    out.position = [float(fixed_updown)] + [0.0] * (len(out.name) - 1)
    return out


def default_loaded_goal(fixed_updown: float) -> JointState:
    out = JointState()
    out.name = list(DEFAULT_JOINT_NAMES)
    out.position = [float(fixed_updown)] + [0.0] * (len(out.name) - 1)
    return out


def make_attached_box(side: str, box_id: int, top_suction: bool) -> AttachedBox:
    out = AttachedBox()
    out.id = f"carried_{side}_box_{int(box_id)}"
    out.box_id = int(box_id)
    out.side = side
    out.grasp_mode = "top_suction" if top_suction else "front"
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


class BoxPairTaskAdapterNode(Node):
    """Semantic box-pair task adapter.

    It is intentionally thin: box ids and scene geometry become explicit target poses and
    attached box specs, then the existing service chain handles IK, extract, loaded planning,
    and execution.
    """

    def __init__(self) -> None:
        super().__init__("box_pair_task_adapter")
        self.declare_parameter("service_name", "/robot_motion/run_box_pair_task")
        self.declare_parameter("pose_task_service", "/robot_motion/run_dual_arm_pose_task")
        self.declare_parameter("state_topic", "/robot_motion/state")
        self.declare_parameter("service_timeout_s", 10.0)
        self.declare_parameter("default_box_front_x", 0.925)
        self.declare_parameter("default_scene_y_shift", 0.0)
        self.declare_parameter("default_world_to_base_z", 0.202094)
        self.declare_parameter("default_fixed_updown", 0.0)
        # updown 逻辑/URDF 与电机物理规划范围均为 [0, 0.7]。
        self.declare_parameter("updown_logical_lower_m", 0.0)
        self.declare_parameter("updown_logical_upper_m", 0.7)
        self.declare_parameter("default_top_suction_x_offset", 0.15)
        self.declare_parameter("default_top_suction_z_offset", 0.2)
        self.declare_parameter("default_candidate_limit", 8)
        self.declare_parameter("default_planning_mode", "shortcut")

        self.service_name = str(self.get_parameter("service_name").value)
        self.pose_task_service = str(self.get_parameter("pose_task_service").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.service_timeout_s = float(self.get_parameter("service_timeout_s").value)
        self.default_box_front_x = float(self.get_parameter("default_box_front_x").value)
        self.default_scene_y_shift = float(self.get_parameter("default_scene_y_shift").value)
        self.default_world_to_base_z = float(self.get_parameter("default_world_to_base_z").value)
        self.default_fixed_updown = float(self.get_parameter("default_fixed_updown").value)
        self.updown_logical_lower_m = float(self.get_parameter("updown_logical_lower_m").value)
        self.updown_logical_upper_m = float(self.get_parameter("updown_logical_upper_m").value)
        self.default_top_suction_x_offset = float(self.get_parameter("default_top_suction_x_offset").value)
        self.default_top_suction_z_offset = float(self.get_parameter("default_top_suction_z_offset").value)
        self.default_candidate_limit = int(self.get_parameter("default_candidate_limit").value)
        self.default_planning_mode = str(self.get_parameter("default_planning_mode").value)

        self.callback_group = ReentrantCallbackGroup()
        self.latest_state: RobotMotionState | None = None
        self.state_sub = self.create_subscription(
            RobotMotionState,
            self.state_topic,
            self.on_state,
            10,
            callback_group=self.callback_group,
        )
        self.pose_task_client = self.create_client(
            RunDualArmPoseTask,
            self.pose_task_service,
            callback_group=self.callback_group,
        )
        self.service = self.create_service(
            RunBoxPairTask,
            self.service_name,
            self.on_run_box_pair_task,
            callback_group=self.callback_group,
        )
        self.status = RuntimeStatusPublisher(
            self,
            self.service_name,
            "box-pair task adapter; converts box ids to target poses then calls RunDualArmPoseTask",
        )
        self.status.mark_ready(f"waiting for box ids; pose_task={self.pose_task_service}")
        self.get_logger().info(
            f"BoxPairTask adapter ready: service={self.service_name} pose_task={self.pose_task_service}"
        )

    def on_state(self, state: RobotMotionState) -> None:
        if state.authoritative:
            self.latest_state = state

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

    def pose_for_box(
        self,
        side: str,
        box: BoxSpec,
        mode: str,
        world_to_base_z: float,
        top_x_offset: float,
        top_z_offset: float,
    ) -> Pose:
        if mode == "top_suction":
            pose = make_top_suction_pose(box, world_to_base_z, top_x_offset, top_z_offset)
        else:
            pose = make_front_grasp_pose(box, world_to_base_z)
        pose.position.y = box.y + (
            -OUTER_BOX_GRASP_LATERAL_OFFSET_M if side == "left"
            else OUTER_BOX_GRASP_LATERAL_OFFSET_M
        )
        return pose

    @staticmethod
    def pose_stamped(pose: Pose) -> PoseStamped:
        out = PoseStamped()
        out.header.frame_id = "base_link"
        out.pose = pose
        return out

    def fill_response_from_pose_task(self, response, pose_response) -> None:
        response.success = bool(pose_response.success)
        response.message = pose_response.message
        response.ik_candidate_states = list(pose_response.ik_candidate_states)
        response.selected_extract = pose_response.selected_extract
        response.selected_loaded = pose_response.selected_loaded
        response.extract_candidates = list(pose_response.extract_candidates)
        response.loaded_candidates = list(pose_response.loaded_candidates)

    def seed_state_for_request(self, request, fixed_updown: float) -> JointState:
        if request.seed_state.name:
            return request.seed_state
        if self.latest_state is not None and self.latest_state.joint_state.name:
            return self.latest_state.joint_state
        return seed_or_default(request.seed_state, fixed_updown)

    def on_run_box_pair_task(self, request, response):
        started = time.monotonic()
        self.status.mark_running(
            f"L{request.left_box_id}/R{request.right_box_id} execute={request.execute} dry_run={request.dry_run}"
        )
        try:
            box_front_x = request.box_front_x if request.box_front_x > 0.0 else self.default_box_front_x
            scene_y_shift = request.scene_y_shift if request.scene_y_shift != 0.0 else self.default_scene_y_shift
            world_to_base_z = request.world_to_base_z if request.world_to_base_z > 0.0 else self.default_world_to_base_z
            # updown 逻辑值合法范围是 [0, 0.7]；0.0 现在是合法的最低位置。
            fixed_updown = (
                request.fixed_updown
                if request.fixed_updown >= self.updown_logical_lower_m
                else self.default_fixed_updown
            )
            # 显式给了但越界时仍夹紧到 [0, 0.7] 并告警。
            updown_clamped = max(
                self.updown_logical_lower_m, min(self.updown_logical_upper_m, float(fixed_updown))
            )
            if abs(updown_clamped - float(fixed_updown)) > 1e-6:
                self.get_logger().warn(
                    f"updown 逻辑值 {float(fixed_updown):.4f}m 超出规划范围 "
                    f"[{self.updown_logical_lower_m}, {self.updown_logical_upper_m}]m，已夹紧到 {updown_clamped:.4f}m"
                )
            fixed_updown = updown_clamped
            top_x_offset = (
                request.top_suction_x_offset
                if request.top_suction_x_offset != 0.0
                else self.default_top_suction_x_offset
            )
            top_z_offset = (
                request.top_suction_z_offset
                if request.top_suction_z_offset != 0.0
                else self.default_top_suction_z_offset
            )
            boxes = make_boxes(box_front_x, scene_y_shift)
            if request.left_box_id not in boxes:
                raise ValueError(f"unknown left_box_id: {request.left_box_id}")
            if request.right_box_id not in boxes:
                raise ValueError(f"unknown right_box_id: {request.right_box_id}")

            left_mode = normalize_grasp_mode(request.left_grasp_mode, request.left_box_id)
            right_mode = normalize_grasp_mode(request.right_grasp_mode, request.right_box_id)
            left_pose = self.pose_for_box("left", boxes[request.left_box_id], left_mode, world_to_base_z, top_x_offset, top_z_offset)
            right_pose = self.pose_for_box("right", boxes[request.right_box_id], right_mode, world_to_base_z, top_x_offset, top_z_offset)
            response.left_target = self.pose_stamped(left_pose)
            response.right_target = self.pose_stamped(right_pose)
            response.attached_boxes = [
                make_attached_box("left", request.left_box_id, left_mode == "top_suction"),
                make_attached_box("right", request.right_box_id, right_mode == "top_suction"),
            ]

            pose_request = RunDualArmPoseTask.Request()
            pose_request.context = request.context
            pose_request.seed_state = self.seed_state_for_request(request, fixed_updown)
            pose_request.left_target = response.left_target
            pose_request.right_target = response.right_target
            pose_request.attached_boxes = list(response.attached_boxes)
            pose_request.scene_objects = list(request.scene_objects)
            pose_request.attached_collision_objects = list(request.attached_collision_objects)
            pose_request.loaded_goal_family = list(request.loaded_goal_family) or [default_loaded_goal(fixed_updown)]
            pose_request.fixed_updown = fixed_updown
            pose_request.left_top_suction = left_mode == "top_suction"
            pose_request.right_top_suction = right_mode == "top_suction"
            pose_request.position_tolerance = request.position_tolerance if request.position_tolerance >= 1e-6 else 0.0
            pose_request.orientation_tolerance = (
                request.orientation_tolerance if request.orientation_tolerance >= 1e-6 else 0.0
            )
            pose_request.max_solutions_per_arm = request.max_solutions_per_arm
            pose_request.candidate_limit = int(request.candidate_limit or self.default_candidate_limit)
            pose_request.planning_mode = request.planning_mode or self.default_planning_mode
            pose_request.execute = bool(request.execute)
            pose_request.dry_run = bool(request.dry_run)
            pose_request.velocity_scale = clamp_motion_scale(request.velocity_scale, 1.0)
            pose_request.acceleration_scale = clamp_motion_scale(request.acceleration_scale, 1.0)

            pose_response = self.call_pose_task(pose_request)
            self.fill_response_from_pose_task(response, pose_response)
            response.message = (
                f"L{request.left_box_id}/R{request.right_box_id} modes=({left_mode},{right_mode}) "
                f"targets=({response.left_target.pose.position.x:.3f},{response.left_target.pose.position.y:.3f},"
                f"{response.left_target.pose.position.z:.3f})/"
                f"({response.right_target.pose.position.x:.3f},{response.right_target.pose.position.y:.3f},"
                f"{response.right_target.pose.position.z:.3f}); "
                f"{response.message}; adapter={(time.monotonic() - started) * 1000.0:.2f}ms"
            )
            self.status.mark_done(response.success, response.message)
            return response
        except Exception as exc:  # pragma: no cover - runtime safety path
            response.success = False
            response.message = str(exc)
            self.status.mark_done(False, response.message)
            return response


def main() -> None:
    rclpy.init()
    node = BoxPairTaskAdapterNode()
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
