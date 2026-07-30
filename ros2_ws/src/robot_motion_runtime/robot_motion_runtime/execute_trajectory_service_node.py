from __future__ import annotations

import math
import threading
from copy import deepcopy

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robot_motion_interfaces.srv import ExecuteTrajectory
from robot_motion_runtime.common import (
    REAL_ARM_JOINT_NAMES,
    RuntimeStatusPublisher,
    canonical_joint_name,
    duration_seconds,
    hardware_joint_name,
    resample_trajectory,
)
from alfa_robot_execution_bridge.joints import RT_CONTROL_ACTION_NAME


class ExecuteTrajectoryServiceNode(Node):
    """Service facade for trajectory execution.

    It gives the service-oriented runtime graph a stable /robot_motion/execute_trajectory
    entry while the actual backend can remain the existing FollowJointTrajectory action.
    """

    def __init__(self) -> None:
        super().__init__("execute_trajectory_service")
        self.declare_parameter("service_name", "/robot_motion/execute_trajectory")
        self.declare_parameter(
            "action_name",
            RT_CONTROL_ACTION_NAME,
        )
        self.declare_parameter("forward_action", True)
        self.declare_parameter("wait_for_action_timeout_s", 2.0)
        self.declare_parameter("wait_for_goal_acceptance", False)
        self.declare_parameter("wait_for_result", False)
        self.declare_parameter("wait_for_result_timeout_s", 120.0)
        self.declare_parameter("resample_before_forward", True)
        self.declare_parameter("resample_rate_hz", 10.0)
        self.declare_parameter("adapt_to_hardware_joint_order", True)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("hold_missing_from_joint_states", True)

        self.service_name = str(self.get_parameter("service_name").value)
        self.action_name = str(self.get_parameter("action_name").value)
        self.forward_action = bool(self.get_parameter("forward_action").value)
        self.wait_for_action_timeout_s = float(self.get_parameter("wait_for_action_timeout_s").value)
        self.wait_for_goal_acceptance = bool(self.get_parameter("wait_for_goal_acceptance").value)
        self.wait_for_result = bool(self.get_parameter("wait_for_result").value)
        self.wait_for_result_timeout_s = float(self.get_parameter("wait_for_result_timeout_s").value)
        self.resample_before_forward = bool(self.get_parameter("resample_before_forward").value)
        self.resample_rate_hz = float(self.get_parameter("resample_rate_hz").value)
        if self.resample_before_forward and self.resample_rate_hz <= 0.0:
            raise ValueError(
                f"resample_rate_hz must be > 0 when resample_before_forward=True, "
                f"got {self.resample_rate_hz}"
            )
        # 10Hz 契约：real-direct 路径（execute_l6_r8_mock_live.py 的 --hz 硬上限）
        # 与本服务的重采样频率必须口径一致，否则轨迹在两条路径上的实际执行节奏会不一致。
        if self.resample_before_forward and abs(self.resample_rate_hz - 10.0) > 1e-6:
            self.get_logger().warn(
                f"resample_rate_hz={self.resample_rate_hz:.2f} deviates from the 10Hz contract "
                "shared with execute_l6_r8_mock_live.py's --hz safety cap. Confirm this is "
                "intentional before running against real hardware."
            )
        self.adapt_to_hardware_joint_order = bool(
            self.get_parameter("adapt_to_hardware_joint_order").value
        )
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.hold_missing_from_joint_states = bool(
            self.get_parameter("hold_missing_from_joint_states").value
        )

        self.callback_group = ReentrantCallbackGroup()
        self.latest_joint_positions: dict[str, float] = {}
        self.joint_state_lock = threading.RLock()
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.on_joint_state,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.action_name,
            callback_group=self.callback_group,
        )
        self.service = self.create_service(
            ExecuteTrajectory,
            self.service_name,
            self.on_execute,
            callback_group=self.callback_group,
        )
        self.status = RuntimeStatusPublisher(
            self,
            self.service_name,
            "ExecuteTrajectory service; optionally forwards to FollowJointTrajectory action backend",
        )
        self.status.mark_ready(f"action={self.action_name}, forward_action={self.forward_action}")
        self.get_logger().info(
            f"ExecuteTrajectory service ready: service={self.service_name} "
            f"action={self.action_name} forward={self.forward_action} "
            f"wait_for_goal_acceptance={self.wait_for_goal_acceptance} "
            f"wait_for_result={self.wait_for_result} "
            f"resample={self.resample_before_forward}@{self.resample_rate_hz:.1f}Hz "
            f"hardware_order={self.adapt_to_hardware_joint_order}"
        )

    def on_joint_state(self, msg: JointState) -> None:
        values = {}
        for index, name in enumerate(msg.name):
            if index >= len(msg.position):
                continue
            values[str(name)] = float(msg.position[index])
        with self.joint_state_lock:
            self.latest_joint_positions = values

    @staticmethod
    def trajectory_summary(trajectory: JointTrajectory) -> str:
        points = list(trajectory.points)
        if not points:
            return "points=0 duration=0.000s"

        times = [duration_seconds(point.time_from_start) for point in points]
        duration_s = times[-1]
        min_dt = math.inf
        max_dt = 0.0
        max_step_rad = 0.0
        max_step_joint = ""
        max_speed_rad_s = 0.0
        max_speed_joint = ""

        for point_index in range(1, len(points)):
            dt = max(0.0, times[point_index] - times[point_index - 1])
            min_dt = min(min_dt, dt)
            max_dt = max(max_dt, dt)
            previous = points[point_index - 1]
            current = points[point_index]
            joint_count = min(
                len(trajectory.joint_names),
                len(previous.positions),
                len(current.positions),
            )
            for joint_index in range(joint_count):
                delta = abs(float(current.positions[joint_index]) - float(previous.positions[joint_index]))
                joint_name = str(trajectory.joint_names[joint_index])
                if delta > max_step_rad:
                    max_step_rad = delta
                    max_step_joint = joint_name
                if dt > 1e-9:
                    speed = delta / dt
                    if speed > max_speed_rad_s:
                        max_speed_rad_s = speed
                        max_speed_joint = joint_name

        if not math.isfinite(min_dt):
            min_dt = 0.0
        return (
            f"points={len(points)} duration={duration_s:.3f}s "
            f"dt=[{min_dt:.3f},{max_dt:.3f}]s "
            f"max_step={math.degrees(max_step_rad):.2f}deg@{max_step_joint or '-'} "
            f"max_speed={math.degrees(max_speed_rad_s):.2f}deg/s@{max_speed_joint or '-'}"
        )

    @staticmethod
    def canonical_source_names(trajectory: JointTrajectory) -> list[str]:
        return [hardware_joint_name(str(name)) for name in trajectory.joint_names]

    @staticmethod
    def joint_changes(trajectory: JointTrajectory, index: int, tolerance: float = 1e-9) -> bool:
        first_value = None
        for point in trajectory.points:
            if index >= len(point.positions):
                continue
            value = float(point.positions[index])
            if first_value is None:
                first_value = value
                continue
            if abs(value - first_value) > tolerance:
                return True
        return False

    def hold_value_for(self, real_name: str) -> float | None:
        model_name = canonical_joint_name(real_name)
        with self.joint_state_lock:
            if real_name in self.latest_joint_positions:
                return self.latest_joint_positions[real_name]
            if model_name in self.latest_joint_positions:
                return self.latest_joint_positions[model_name]
        return None

    def adapt_trajectory_for_hardware(self, trajectory: JointTrajectory) -> JointTrajectory:
        if not self.adapt_to_hardware_joint_order:
            return trajectory

        canonical_names = self.canonical_source_names(trajectory)
        duplicates = sorted({name for name in canonical_names if canonical_names.count(name) > 1})
        if duplicates:
            raise ValueError(f"duplicate joints after alias mapping: {duplicates}")
        name_to_index = {name: index for index, name in enumerate(canonical_names)}
        target_indices: list[int | None] = []
        hold_values: list[float] = []
        for real_name in REAL_ARM_JOINT_NAMES:
            index = name_to_index.get(real_name)
            target_indices.append(index)
            if index is None:
                hold_value = self.hold_value_for(real_name)
                if hold_value is None or not self.hold_missing_from_joint_states:
                    raise ValueError(
                        f"missing required hardware joint {real_name}; "
                        f"trajectory names={list(trajectory.joint_names)}"
                    )
                hold_values.append(float(hold_value))
            else:
                hold_values.append(0.0)

        mapped_targets = set(REAL_ARM_JOINT_NAMES)
        for source_index, canonical_name in enumerate(canonical_names):
            if canonical_name in mapped_targets:
                continue
            if self.joint_changes(trajectory, source_index):
                raise ValueError(
                    f"planned joint {trajectory.joint_names[source_index]} changes but cannot be "
                    "forwarded to the fixed 14-axis rt-control action"
                )

        out = JointTrajectory()
        out.header = trajectory.header
        out.joint_names = list(REAL_ARM_JOINT_NAMES)
        for point_index, source_point in enumerate(trajectory.points):
            if len(source_point.positions) != len(trajectory.joint_names):
                raise ValueError(
                    f"point {point_index} has {len(source_point.positions)} positions for "
                    f"{len(trajectory.joint_names)} joints"
                )
            point = JointTrajectoryPoint()
            point.time_from_start = source_point.time_from_start
            for target_index, hold_value in zip(target_indices, hold_values):
                if target_index is None:
                    point.positions.append(float(hold_value))
                else:
                    point.positions.append(float(source_point.positions[target_index]))
            if source_point.velocities:
                for target_index, _ in zip(target_indices, hold_values):
                    point.velocities.append(
                        float(source_point.velocities[target_index])
                        if target_index is not None and target_index < len(source_point.velocities)
                        else 0.0
                    )
            if source_point.accelerations:
                for target_index, _ in zip(target_indices, hold_values):
                    point.accelerations.append(
                        float(source_point.accelerations[target_index])
                        if target_index is not None and target_index < len(source_point.accelerations)
                        else 0.0
                    )
            if source_point.effort:
                for target_index, _ in zip(target_indices, hold_values):
                    point.effort.append(
                        float(source_point.effort[target_index])
                        if target_index is not None and target_index < len(source_point.effort)
                        else 0.0
                    )
            out.points.append(point)
        return out

    def on_execute(self, request, response):
        self.status.mark_running(
            f"points={len(request.trajectory.points)} dry_run={request.dry_run}"
        )
        if not request.trajectory.joint_names:
            response.accepted = False
            response.message = "trajectory.joint_names is empty"
            self.status.mark_done(False, response.message)
            return response
        if not request.trajectory.points:
            response.accepted = False
            response.message = "trajectory.points is empty"
            self.status.mark_done(False, response.message)
            return response
        if request.velocity_scale < 0.0 or request.velocity_scale > 1.0:
            response.accepted = False
            response.message = (
                f"velocity_scale out of range [0.0,1.0]: {request.velocity_scale}"
            )
            self.status.mark_done(False, response.message)
            return response
        if request.acceleration_scale < 0.0 or request.acceleration_scale > 1.0:
            response.accepted = False
            response.message = (
                f"acceleration_scale out of range [0.0,1.0]: {request.acceleration_scale}"
            )
            self.status.mark_done(False, response.message)
            return response
        if request.velocity_scale != 0.0 or request.acceleration_scale != 0.0:
            self.get_logger().warn(
                "ExecuteTrajectory received velocity_scale="
                f"{request.velocity_scale} acceleration_scale={request.acceleration_scale}, "
                "but this node's forward path does not apply either field to the outgoing "
                "trajectory (only resample_rate_hz reshapes timing). These values are "
                "currently accepted but silently ignored downstream."
            )
        if request.dry_run or not self.forward_action:
            response.accepted = True
            response.message = (
                f"dry_run accepted: joints={len(request.trajectory.joint_names)} "
                f"points={len(request.trajectory.points)}"
            )
            self.status.mark_done(True, response.message)
            return response

        if not self.action_client.wait_for_server(timeout_sec=self.wait_for_action_timeout_s):
            response.accepted = False
            response.message = f"action server not available: {self.action_name}"
            self.status.mark_done(False, response.message)
            return response

        try:
            outgoing_trajectory = (
                resample_trajectory(request.trajectory, self.resample_rate_hz)
                if self.resample_before_forward
                else deepcopy(request.trajectory)
            )
            outgoing_trajectory = self.adapt_trajectory_for_hardware(outgoing_trajectory)
        except ValueError as exc:
            response.accepted = False
            response.message = f"trajectory adaptation failed: {exc}"
            self.status.mark_done(False, response.message)
            return response

        self.get_logger().info(
            "Forwarding trajectory to FollowJointTrajectory: "
            + self.trajectory_summary(outgoing_trajectory)
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = outgoing_trajectory
        future = self.action_client.send_goal_async(goal)
        if not self.wait_for_goal_acceptance and not self.wait_for_result:
            response.accepted = True
            response.message = "action goal sent"
            self.status.mark_done(True, response.message)
            return response

        event = threading.Event()
        holder = {}

        def done_callback(done_future):
            holder["goal_handle"] = done_future.result()
            event.set()

        future.add_done_callback(done_callback)
        if not event.wait(timeout=self.wait_for_action_timeout_s):
            response.accepted = False
            response.message = "timed out waiting for action goal acceptance"
            self.status.mark_done(False, response.message)
            return response

        goal_handle = holder.get("goal_handle")
        response.accepted = bool(goal_handle and goal_handle.accepted)
        if not response.accepted:
            response.message = "action goal rejected"
            self.status.mark_done(False, response.message)
            return response
        if not self.wait_for_result:
            response.message = "action goal accepted"
            self.status.mark_done(True, response.message)
            return response

        result_event = threading.Event()
        result_holder = {}

        def result_callback(done_future):
            try:
                result_holder["response"] = done_future.result()
            except Exception as exc:  # pragma: no cover - defensive runtime path
                result_holder["error"] = exc
            result_event.set()

        goal_handle.get_result_async().add_done_callback(result_callback)
        if not result_event.wait(timeout=self.wait_for_result_timeout_s):
            response.accepted = False
            response.message = "timed out waiting for action result"
            self.status.mark_done(False, response.message)
            return response
        if "error" in result_holder:
            response.accepted = False
            response.message = f"action result failed: {result_holder['error']}"
            self.status.mark_done(False, response.message)
            return response

        result_response = result_holder.get("response")
        result = getattr(result_response, "result", None)
        error_code = getattr(result, "error_code", 0)
        error_string = getattr(result, "error_string", "")
        response.accepted = int(error_code) == int(FollowJointTrajectory.Result.SUCCESSFUL)
        response.message = (
            "action result received: "
            f"error_code={int(error_code)} error_string='{str(error_string)}'"
        )
        self.status.mark_done(response.accepted, response.message)
        return response


def main() -> None:
    rclpy.init()
    node = ExecuteTrajectoryServiceNode()
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
