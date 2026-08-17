from __future__ import annotations

import argparse
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_motion_interfaces.action import ExecuteMotionStage
from robot_motion_interfaces.msg import DualArmPoseTargets
from std_srvs.srv import Trigger

from .common import (
    Pose6DValue,
    camera_view_poses_for_task,
    parse_task_code,
    pose6d_from_dict,
    suction_surface_poses_for_task,
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


def _pose(value: Pose6DValue) -> Pose:
    message = Pose()
    message.position.x = value.x
    message.position.y = value.y
    message.position.z = value.z
    quaternion = _quaternion_from_rpy(value.roll, value.pitch, value.yaw)
    (
        message.orientation.x,
        message.orientation.y,
        message.orientation.z,
        message.orientation.w,
    ) = quaternion
    return message


TARGET_MODES = {
    "no_move": DualArmPoseTargets.GRASP_MODE_NO_MOVE,
    "front": DualArmPoseTargets.GRASP_MODE_SIDE_SUCTION,
    "side_suction": DualArmPoseTargets.GRASP_MODE_SIDE_SUCTION,
    "top_suction": DualArmPoseTargets.GRASP_MODE_TOP_SUCTION,
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
        stage: int,
        left: Pose6DValue | None = None,
        right: Pose6DValue | None = None,
        left_mode: str = "no_move",
        right_mode: str = "no_move",
    ):
        goal = ExecuteMotionStage.Goal()
        goal.execution_stage = int(stage)
        goal.targets.left_grasp_mode = TARGET_MODES[left_mode]
        goal.targets.right_grasp_mode = TARGET_MODES[right_mode]
        if left is not None:
            goal.targets.left_pose = _pose(left)
        if right is not None:
            goal.targets.right_pose = _pose(right)
        return goal

    def run_stage(self, goal, label: str, timeout_s: float):
        if not self.stage_client.wait_for_server(timeout_sec=timeout_s):
            raise RuntimeError("/motion/execute_stage Action 不可用")

        def feedback(message) -> None:
            value = message.feedback
            state = {
                ExecuteMotionStage.Feedback.MOTION_STATE_PLANNING: "PLANNING",
                ExecuteMotionStage.Feedback.MOTION_STATE_EXECUTING: "EXECUTING",
                ExecuteMotionStage.Feedback.MOTION_STATE_SETTLING: "SETTLING",
            }.get(int(value.motion_state), "UNSPECIFIED")
            print(
                f"{label}: {state}",
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
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.ok:
            error = wrapped.result.error
            raise RuntimeError(
                f"{label} 失败 status={wrapped.status} code={error.code} "
                f"retryable={error.retryable}: {error.message or wrapped.result.diagnostic}"
            )
        print(f"{label} 完成: {wrapped.result.diagnostic}", flush=True)
        return wrapped.result


def parse_args():
    parser = argparse.ArgumentParser(description="Motion 单阶段 Action 手工任务发布器")
    parser.add_argument("--task", help="测试映射 A1..A5/B1..B5，仅测试客户端使用")
    parser.add_argument(
        "--recapture-left",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="第一次发送的左重拍末端 base_link 6D位姿，角度单位rad",
    )
    parser.add_argument(
        "--recapture-right",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="第一次发送的右重拍末端 base_link 6D位姿，角度单位rad",
    )
    parser.add_argument(
        "--left",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="左吸附面中心 base_link 6D位姿，角度单位rad",
    )
    parser.add_argument(
        "--right",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="右吸附面中心 base_link 6D位姿，角度单位rad",
    )
    parser.add_argument("--front-distance", type=float, default=0.9)
    parser.add_argument("--top-distance", type=float, default=0.7)
    parser.add_argument("--initialize", action="store_true")
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="每个阶段发送前等待回车，便于手工协调吸附与释放",
    )
    for prefix in ("recapture-left", "recapture-right", "left", "right"):
        parser.add_argument(
            f"--{prefix}-mode",
            choices=tuple(TARGET_MODES),
            default=None,
        )
    parser.add_argument("--stop-after", choices=("pregrasp", "approach", "place", "return"))
    parser.add_argument("--yes-execute", action="store_true")
    parser.add_argument("--timeout", type=float, default=300.0)
    options = parser.parse_args()
    direct_pose = options.left is not None or options.right is not None
    if options.task and direct_pose:
        parser.error("--task 与 --left/--right 不能同时使用")
    if not options.task:
        for prefix, default_mode in (
            ("recapture_left", "side_suction"),
            ("recapture_right", "side_suction"),
            ("left", "front"),
            ("right", "front"),
        ):
            pose = getattr(options, prefix)
            mode_name = f"{prefix}_mode"
            mode = getattr(options, mode_name)
            if mode is None:
                mode = default_mode if pose is not None else "no_move"
                setattr(options, mode_name, mode)
            if mode != "no_move" and pose is None:
                parser.error(f"--{prefix.replace('_', '-')} 缺失，但对应 mode 不是 no_move")
        if options.left_mode == "no_move" and options.right_mode == "no_move":
            parser.error("抓取目标左右臂不能同时为 no_move")
        if (
            options.recapture_left_mode == "no_move"
            and options.recapture_right_mode == "no_move"
        ):
            parser.error("重拍目标左右臂不能同时为 no_move")
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
        left, right = suction_surface_poses_for_task(task)
        recapture_left, recapture_right = camera_view_poses_for_task(task)
        left_mode = task.left_grasp_mode
        right_mode = task.right_grasp_mode
        recapture_left_mode = left_mode
        recapture_right_mode = right_mode
        label = task.code
    else:
        fields = ("x", "y", "z", "roll", "pitch", "yaw")

        def optional_pose(values, label):
            if values is None:
                return None
            return pose6d_from_dict(dict(zip(fields, values)), label)

        left = optional_pose(options.left, "left")
        right = optional_pose(options.right, "right")
        recapture_left = optional_pose(options.recapture_left, "recapture_left")
        recapture_right = optional_pose(options.recapture_right, "recapture_right")
        left_mode = options.left_mode
        right_mode = options.right_mode
        recapture_left_mode = options.recapture_left_mode
        recapture_right_mode = options.recapture_right_mode
        label = "6D"

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
        stages = [
            (
                ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW,
                "重拍位",
                recapture_left,
                recapture_right,
                recapture_left_mode,
                recapture_right_mode,
            ),
            (
                ExecuteMotionStage.Goal.EXECUTION_STAGE_PREGRASP,
                "预抓取",
                left,
                right,
                left_mode,
                right_mode,
            ),
            (ExecuteMotionStage.Goal.EXECUTION_STAGE_APPROACH, "靠近吸附", None, None, "no_move", "no_move"),
            (ExecuteMotionStage.Goal.EXECUTION_STAGE_PLACE, "放置", None, None, "no_move", "no_move"),
            (ExecuteMotionStage.Goal.EXECUTION_STAGE_HOME, "返回初始位", None, None, "no_move", "no_move"),
        ]
        stop_stage = {
            "pregrasp": ExecuteMotionStage.Goal.EXECUTION_STAGE_PREGRASP,
            "approach": ExecuteMotionStage.Goal.EXECUTION_STAGE_APPROACH,
            "place": ExecuteMotionStage.Goal.EXECUTION_STAGE_PLACE,
            "return": ExecuteMotionStage.Goal.EXECUTION_STAGE_HOME,
        }.get(options.stop_after)
        for index, (
            stage,
            stage_label,
            stage_left,
            stage_right,
            stage_left_mode,
            stage_right_mode,
        ) in enumerate(stages, start=1):
            if options.interactive:
                input(f"确认外部条件满足，回车发送阶段：{stage_label}；Ctrl-C取消：")
            print(f"发布阶段：{stage_label}", flush=True)
            goal = node.make_goal(
                stage=stage,
                left=stage_left,
                right=stage_right,
                left_mode=stage_left_mode,
                right_mode=stage_right_mode,
            )
            node.run_stage(goal, stage_label, options.timeout)
            if stage == ExecuteMotionStage.Goal.EXECUTION_STAGE_APPROACH:
                print(
                    "靠近完成；请由 Autonomy/RT-Control 打开吸附通路并确认真空，"
                    "确认前不要发送放置阶段。",
                    flush=True,
                )
            elif stage == ExecuteMotionStage.Goal.EXECUTION_STAGE_PLACE:
                print(
                    "放置完成；请由 Autonomy/RT-Control 关闭吸附通路并确认释放，"
                    "确认前不要发送返回阶段。",
                    flush=True,
                )
            if stop_stage == stage:
                break
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
