from __future__ import annotations

import argparse
import math
import threading
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from alfa_task_interfaces.action import PlanAndExecutePick, PlanAndExecutePlace
from alfa_task_interfaces.msg import BoxGeometry, PickTarget, PlaceTarget
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from .common import TASK_LAYOUTS, parse_task_code


WORLD_TO_BASE_Z_M = 0.202094
BOX_SIZE_M = (0.3, 0.4, 0.4)
FRONT_ORIENTATION_XYZW = (0.70710678, 0.0, 0.70710678, 0.0)
TOP_ORIENTATION_XYZW = (1.0, 0.0, 0.0, 0.0)
LAYOUT_Y_SHIFT_M = {"A": 0.05, "B": 0.0}


def _wait_future(future, timeout_s: float, label: str):
    deadline = time.monotonic() + timeout_s
    while not future.done():
        if time.monotonic() >= deadline:
            raise TimeoutError(f"{label}超时")
        time.sleep(0.02)
    return future.result()


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, ...]:
    half_roll = 0.5 * roll
    half_pitch = 0.5 * pitch
    half_yaw = 0.5 * yaw
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class ManualDomainTask(Node):
    def __init__(self) -> None:
        super().__init__("manual_motion_domain_task")
        self.pick_client = ActionClient(
            self,
            PlanAndExecutePick,
            "/motion/plan_and_execute_pick",
        )
        self.place_client = ActionClient(
            self,
            PlanAndExecutePlace,
            "/motion/plan_and_execute_place",
        )
        self.initialize_client = self.create_client(
            Trigger,
            "/motion/dev/initialize_loaded_pose",
        )

    def initialize(self, timeout_s: float) -> None:
        if not self.initialize_client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError("Motion 初始化服务不可用")
        response = _wait_future(
            self.initialize_client.call_async(Trigger.Request()),
            timeout_s,
            "初始化负重位",
        )
        if response is None or not response.success:
            raise RuntimeError("初始化失败: " + ("无响应" if response is None else response.message))
        print(response.message, flush=True)

    @staticmethod
    def _geometry(box_id: int, row: int, column: int, distance: float, layout: str, mode: str, stamp) -> BoxGeometry:
        geometry = BoxGeometry()
        y_shift = LAYOUT_Y_SHIFT_M[layout]
        box_y = (2 - column) * BOX_SIZE_M[1] + y_shift
        box_z_world = (5 - row + 0.5) * BOX_SIZE_M[2]
        box_z_base = box_z_world - WORLD_TO_BASE_Z_M
        geometry.body_pose.header.stamp = stamp
        geometry.body_pose.header.frame_id = "base_link"
        geometry.body_pose.pose.pose.position.x = distance
        geometry.body_pose.pose.pose.position.y = box_y
        geometry.body_pose.pose.pose.position.z = box_z_base
        geometry.body_pose.pose.pose.orientation.w = 1.0
        geometry.suction_surface_pose.header.stamp = stamp
        geometry.suction_surface_pose.header.frame_id = "base_link"
        pose = geometry.suction_surface_pose.pose.pose
        if mode == "top_suction":
            pose.position.x = distance + 0.15
            pose.position.z = box_z_base + 0.20
            orientation = TOP_ORIENTATION_XYZW
        else:
            pose.position.x = distance
            pose.position.z = box_z_base
            orientation = FRONT_ORIENTATION_XYZW
        pose.position.y = box_y
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
        geometry.size_m.x, geometry.size_m.y, geometry.size_m.z = BOX_SIZE_M
        geometry.size_valid = [True, True, True]
        geometry.size_source = [1, 1, 1]
        return geometry

    @staticmethod
    def _geometry_from_suction_pose(
        pose_6d: list[float],
        mode: str,
        stamp,
    ) -> BoxGeometry:
        x, y, z, roll, pitch, yaw = pose_6d
        orientation = _quaternion_from_rpy(roll, pitch, yaw)
        geometry = BoxGeometry()
        geometry.suction_surface_pose.header.stamp = stamp
        geometry.suction_surface_pose.header.frame_id = "base_link"
        suction_pose = geometry.suction_surface_pose.pose.pose
        suction_pose.position.x = x
        suction_pose.position.y = y
        suction_pose.position.z = z
        (
            suction_pose.orientation.x,
            suction_pose.orientation.y,
            suction_pose.orientation.z,
            suction_pose.orientation.w,
        ) = orientation

        geometry.body_pose.header.stamp = stamp
        geometry.body_pose.header.frame_id = "base_link"
        body_pose = geometry.body_pose.pose.pose
        body_pose.position.x = x - (0.15 if mode == "top_suction" else 0.0)
        body_pose.position.y = y
        body_pose.position.z = z - (0.20 if mode == "top_suction" else 0.0)
        body_pose.orientation.w = 1.0
        geometry.size_m.x, geometry.size_m.y, geometry.size_m.z = BOX_SIZE_M
        geometry.size_valid = [True, True, True]
        geometry.size_source = [1, 1, 1]
        return geometry

    @staticmethod
    def _base_pick_goal(request_id: str, sequence_id: int):
        goal = PlanAndExecutePick.Goal()
        goal.context.request_id = request_id
        goal.context.task_id = request_id
        goal.context.sequence_id = sequence_id
        goal.motion_profile_id = "motion_demo_current"
        goal.grip_profile_id = "current_plc_default"
        goal.expected_map_version = "manual_box_stack"
        goal.max_pose_age.sec = 10
        return goal

    def make_pick_goal(self, task_code: str, front: float, top: float, request_id: str):
        task = parse_task_code(task_code, front, top)
        mode = task.grasp_family
        distance = task.effective_distance_m
        stamp = self.get_clock().now().to_msg()
        goal = self._base_pick_goal(request_id, task.index)
        for box_id, arm_id in (
            (task.left_box_id, PickTarget.ARM_LEFT),
            (task.right_box_id, PickTarget.ARM_RIGHT),
        ):
            target = PickTarget()
            target.box_id = box_id
            target.row = (box_id - 1) // 3 + 1
            target.column = (box_id - 1) % 3 + 1
            target.arm_id = arm_id
            target.suction_mode = mode
            target.refined_geometry = self._geometry(
                box_id,
                target.row,
                target.column,
                distance,
                task.layout,
                mode,
                stamp,
            )
            goal.targets.append(target)
        return task, goal

    def make_direct_pick_goal(self, options, request_id: str):
        stamp = self.get_clock().now().to_msg()
        goal = self._base_pick_goal(request_id, options.sequence_id)
        for pose_6d, box_id, row, column, arm_id, mode in (
            (
                options.left,
                options.left_box_id,
                options.left_row,
                options.left_column,
                PickTarget.ARM_LEFT,
                options.left_mode,
            ),
            (
                options.right,
                options.right_box_id,
                options.right_row,
                options.right_column,
                PickTarget.ARM_RIGHT,
                options.right_mode,
            ),
        ):
            target = PickTarget()
            target.box_id = box_id
            target.row = row
            target.column = column
            target.arm_id = arm_id
            target.suction_mode = mode
            target.refined_geometry = self._geometry_from_suction_pose(
                pose_6d,
                mode,
                stamp,
            )
            goal.targets.append(target)
        return goal

    @staticmethod
    def make_place_goal(pick_goal, request_id: str):
        goal = PlanAndExecutePlace.Goal()
        goal.context.request_id = request_id
        goal.context.task_id = pick_goal.context.task_id
        goal.context.sequence_id = pick_goal.context.sequence_id
        goal.motion_profile_id = pick_goal.motion_profile_id
        goal.placement_profile_id = "fixed_demo_place"
        goal.grip_profile_id = pick_goal.grip_profile_id
        for pick_target in pick_goal.targets:
            target = PlaceTarget()
            target.box_id = pick_target.box_id
            target.row = pick_target.row
            target.column = pick_target.column
            target.arm_id = pick_target.arm_id
            goal.targets.append(target)
        return goal

    def run_action(self, client, goal, label: str, timeout_s: float):
        if not client.wait_for_server(timeout_sec=timeout_s):
            raise RuntimeError(f"{label} Action 不可用")

        def feedback(message) -> None:
            print(
                f"{label}: {message.feedback.stage} "
                f"{100.0 * message.feedback.progress_0_to_1:.0f}%",
                flush=True,
            )

        goal_handle = _wait_future(
            client.send_goal_async(goal, feedback_callback=feedback),
            timeout_s,
            f"{label} Goal 应答",
        )
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"{label} Goal 被拒绝")
        wrapped = _wait_future(goal_handle.get_result_async(), timeout_s, f"{label} Result")
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"{label} 失败 status={wrapped.status}: {wrapped.result.error.message}"
            )
        return wrapped.result


def parse_args():
    parser = argparse.ArgumentParser(description="五域 Motion Action 手工任务发布器")
    parser.add_argument("--task", help="测试映射 A1..A5/B1..B5，仅发布器使用")
    parser.add_argument(
        "--left",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="左吸盘 base_link 下的6D吸附位姿，角度单位rad",
    )
    parser.add_argument(
        "--right",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="右吸盘 base_link 下的6D吸附位姿，角度单位rad",
    )
    parser.add_argument("--left-box-id", type=int, default=1)
    parser.add_argument("--right-box-id", type=int, default=3)
    parser.add_argument("--left-row", type=int, default=1)
    parser.add_argument("--right-row", type=int, default=1)
    parser.add_argument("--left-column", type=int, default=1)
    parser.add_argument("--right-column", type=int, default=3)
    parser.add_argument("--left-mode", choices=("front", "top_suction"), default="front")
    parser.add_argument("--right-mode", choices=("front", "top_suction"), default="front")
    parser.add_argument("--sequence-id", type=int, default=1)
    parser.add_argument("--front-distance", type=float, default=0.9)
    parser.add_argument("--top-distance", type=float, default=0.7)
    parser.add_argument("--initialize", action="store_true")
    parser.add_argument("--skip-place", action="store_true")
    parser.add_argument("--yes-execute", action="store_true")
    parser.add_argument("--timeout", type=float, default=300.0)
    options = parser.parse_args()
    direct_pose = options.left is not None or options.right is not None
    if options.task and direct_pose:
        parser.error("--task 与 --left/--right 不能同时使用")
    if not options.task and (options.left is None or options.right is None):
        parser.error("必须提供 --task，或同时提供 --left 与 --right")
    return options


def main(args=None) -> None:
    options = parse_args()
    if not options.yes_execute:
        raise SystemExit("拒绝发送：必须显式增加 --yes-execute")
    rclpy.init(args=args)
    node = ManualDomainTask()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        if options.initialize:
            input("确认人员远离且 rt-control 已 READY，回车初始化到负重位：")
            node.initialize(options.timeout)
        label = options.task.upper() if options.task else "6D"
        request_id = f"manual-{label}-{uuid.uuid4().hex[:8]}"
        if options.task:
            task, pick_goal = node.make_pick_goal(
                options.task,
                options.front_distance,
                options.top_distance,
                request_id,
            )
            description = (
                f"L{task.left_box_id}/R{task.right_box_id} "
                f"mode={task.grasp_family} distance={task.effective_distance_m:.3f}m"
            )
        else:
            pick_goal = node.make_direct_pick_goal(options, request_id)
            description = (
                f"L{options.left_box_id}/R{options.right_box_id} "
                f"modes=({options.left_mode},{options.right_mode}) source=direct_6d"
            )
        print(f"发布 M-02: request={request_id} {description}", flush=True)
        pick_result = node.run_action(node.pick_client, pick_goal, "M-02", options.timeout)
        print(
            f"M-02 完成 verification={pick_result.overall_verification_level}",
            flush=True,
        )
        if not options.skip_place:
            place_goal = node.make_place_goal(pick_goal, request_id + "-place")
            print("发布 M-03", flush=True)
            place_result = node.run_action(node.place_client, place_goal, "M-03", options.timeout)
            print(
                f"M-03 完成 verification={place_result.overall_verification_level}",
                flush=True,
            )
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
