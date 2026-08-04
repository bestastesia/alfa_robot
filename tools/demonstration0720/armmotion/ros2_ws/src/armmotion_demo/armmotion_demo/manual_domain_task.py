from __future__ import annotations

import argparse
import math
import threading
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from alfa_motion_interfaces.action import ExecuteMotionStage
from alfa_motion_interfaces.msg import MotionPoseTarget
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from .common import (
    Pose6DValue,
    front_face_poses_for_task,
    parse_task_code,
    pose6d_from_dict,
)


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


def _pose_stamped(value: Pose6DValue, stamp) -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = "base_link"
    message.header.stamp = stamp
    message.pose.position.x = value.x
    message.pose.position.y = value.y
    message.pose.position.z = value.z
    quaternion = _quaternion_from_rpy(value.roll, value.pitch, value.yaw)
    (
        message.pose.orientation.x,
        message.pose.orientation.y,
        message.pose.orientation.z,
        message.pose.orientation.w,
    ) = quaternion
    return message


TARGET_MODES = {
    "no_move": MotionPoseTarget.NO_MOVE,
    "front": MotionPoseTarget.SIDE_SUCTION,
    "side_suction": MotionPoseTarget.SIDE_SUCTION,
    "top_suction": MotionPoseTarget.TOP_SUCTION,
}


class ManualDomainTask(Node):
    def __init__(self) -> None:
        super().__init__("manual_motion_domain_task")
        self.stage_client = ActionClient(
            self,
            ExecuteMotionStage,
            "/motion/execute_stage",
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

    def make_goal(
        self,
        *,
        request_id: str,
        task_id: str,
        sequence_id: int,
        stage: int,
        left: Pose6DValue | None = None,
        right: Pose6DValue | None = None,
        left_mode: str = "no_move",
        right_mode: str = "no_move",
    ):
        goal = ExecuteMotionStage.Goal()
        goal.context.request_id = request_id
        goal.context.task_id = task_id
        goal.context.sequence_id = int(sequence_id)
        goal.stage = int(stage)
        if left is not None and right is not None:
            stamp = self.get_clock().now().to_msg()
            goal.left_target.pose = _pose_stamped(left, stamp)
            goal.right_target.pose = _pose_stamped(right, stamp)
            goal.left_target.grasp_mode = TARGET_MODES[left_mode]
            goal.right_target.grasp_mode = TARGET_MODES[right_mode]
        return goal

    def run_stage(self, goal, label: str, timeout_s: float):
        if not self.stage_client.wait_for_server(timeout_sec=timeout_s):
            raise RuntimeError("/motion/execute_stage Action 不可用")

        def feedback(message) -> None:
            value = message.feedback
            print(
                f"{label}: {value.state} {100.0 * value.progress_0_to_1:.0f}%",
                flush=True,
            )

        goal_handle = _wait_future(
            self.stage_client.send_goal_async(goal, feedback_callback=feedback),
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
        print(
            f"{label} 完成 planning={wrapped.result.planning_time_s:.3f}s "
            f"execution={wrapped.result.execution_time_s:.3f}s",
            flush=True,
        )
        return wrapped.result


def parse_args():
    parser = argparse.ArgumentParser(description="Motion 单阶段 Action 手工任务发布器")
    parser.add_argument("--task", help="测试映射 A1..A5/B1..B5，仅测试客户端使用")
    parser.add_argument(
        "--recapture-left",
        nargs=6,
        type=float,
        required=True,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="第一次发送的左重拍末端 base_link 6D位姿，角度单位rad",
    )
    parser.add_argument(
        "--recapture-right",
        nargs=6,
        type=float,
        required=True,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="第一次发送的右重拍末端 base_link 6D位姿，角度单位rad",
    )
    parser.add_argument(
        "--left",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="左箱正面中心 base_link 6D位姿，角度单位rad",
    )
    parser.add_argument(
        "--right",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="右箱正面中心 base_link 6D位姿，角度单位rad",
    )
    parser.add_argument("--sequence-id", type=int, default=1)
    parser.add_argument("--front-distance", type=float, default=0.9)
    parser.add_argument("--top-distance", type=float, default=0.7)
    parser.add_argument("--initialize", action="store_true")
    for prefix, default in (
        ("recapture-left", "no_move"),
        ("recapture-right", "no_move"),
        ("left", "front"),
        ("right", "front"),
    ):
        parser.add_argument(
            f"--{prefix}-mode",
            choices=tuple(TARGET_MODES),
            default=default,
        )
    parser.add_argument("--stop-after", choices=("pregrasp", "approach", "place", "return"))
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
    if options.task:
        task = parse_task_code(
            options.task,
            options.front_distance,
            options.top_distance,
        )
        left, right = front_face_poses_for_task(task)
        label = task.code
    else:
        left = pose6d_from_dict(
            dict(zip(("x", "y", "z", "roll", "pitch", "yaw"), options.left)),
            "left",
        )
        right = pose6d_from_dict(
            dict(zip(("x", "y", "z", "roll", "pitch", "yaw"), options.right)),
            "right",
        )
        label = "6D"
    recapture_left = pose6d_from_dict(
        dict(zip(("x", "y", "z", "roll", "pitch", "yaw"), options.recapture_left)),
        "recapture_left",
    )
    recapture_right = pose6d_from_dict(
        dict(zip(("x", "y", "z", "roll", "pitch", "yaw"), options.recapture_right)),
        "recapture_right",
    )

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
        task_id = f"manual-{label}-{uuid.uuid4().hex[:8]}"
        sequence_id = options.sequence_id
        stages = [
            (
                ExecuteMotionStage.Goal.MOVE_TO_RECAPTURE,
                "重拍位",
                recapture_left,
                recapture_right,
                options.recapture_left_mode,
                options.recapture_right_mode,
            ),
            (
                ExecuteMotionStage.Goal.MOVE_TO_PREGRASP,
                "预抓取",
                left,
                right,
                options.left_mode,
                options.right_mode,
            ),
            (ExecuteMotionStage.Goal.APPROACH_SUCTION, "靠近吸附", None, None, "no_move", "no_move"),
            (ExecuteMotionStage.Goal.MOVE_TO_PLACE, "放置", None, None, "no_move", "no_move"),
            (ExecuteMotionStage.Goal.RETURN_INITIAL, "返回初始位", None, None, "no_move", "no_move"),
        ]
        stop_stage = {
            "pregrasp": ExecuteMotionStage.Goal.MOVE_TO_PREGRASP,
            "approach": ExecuteMotionStage.Goal.APPROACH_SUCTION,
            "place": ExecuteMotionStage.Goal.MOVE_TO_PLACE,
            "return": ExecuteMotionStage.Goal.RETURN_INITIAL,
        }.get(options.stop_after)
        for index, (
            stage,
            stage_label,
            stage_left,
            stage_right,
            stage_left_mode,
            stage_right_mode,
        ) in enumerate(stages, start=1):
            request_id = f"{task_id}-{index}"
            print(f"发布阶段：{stage_label} request={request_id}", flush=True)
            goal = node.make_goal(
                request_id=request_id,
                task_id=task_id,
                sequence_id=sequence_id,
                stage=stage,
                left=stage_left,
                right=stage_right,
                left_mode=stage_left_mode,
                right_mode=stage_right_mode,
            )
            node.run_stage(goal, stage_label, options.timeout)
            if stage == ExecuteMotionStage.Goal.APPROACH_SUCTION:
                print("靠近完成；吸附通路与真空确认由 Autonomy/RT-Control 负责。", flush=True)
            elif stage == ExecuteMotionStage.Goal.MOVE_TO_PLACE:
                print("放置完成；释放通路与真空确认由 Autonomy/RT-Control 负责。", flush=True)
            if stop_stage == stage:
                break
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
