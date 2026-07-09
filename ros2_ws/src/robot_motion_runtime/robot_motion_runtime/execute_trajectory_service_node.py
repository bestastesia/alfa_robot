from __future__ import annotations

import threading

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_motion_interfaces.srv import ExecuteTrajectory
from robot_motion_runtime.common import RuntimeStatusPublisher, resample_trajectory


class ExecuteTrajectoryServiceNode(Node):
    """Service facade for trajectory execution.

    It gives the service-oriented runtime graph a stable /robot_motion/execute_trajectory
    entry while the actual backend can remain the existing FollowJointTrajectory action.
    """

    def __init__(self) -> None:
        super().__init__("execute_trajectory_service")
        self.declare_parameter("service_name", "/robot_motion/execute_trajectory")
        self.declare_parameter("action_name", "/alfa_execution/execute_joint_trajectory")
        self.declare_parameter("forward_action", True)
        self.declare_parameter("wait_for_action_timeout_s", 2.0)
        self.declare_parameter("wait_for_goal_acceptance", False)
        self.declare_parameter("wait_for_result", False)
        self.declare_parameter("wait_for_result_timeout_s", 120.0)
        self.declare_parameter("resample_before_forward", True)
        self.declare_parameter("resample_rate_hz", 20.0)

        self.service_name = str(self.get_parameter("service_name").value)
        self.action_name = str(self.get_parameter("action_name").value)
        self.forward_action = bool(self.get_parameter("forward_action").value)
        self.wait_for_action_timeout_s = float(self.get_parameter("wait_for_action_timeout_s").value)
        self.wait_for_goal_acceptance = bool(self.get_parameter("wait_for_goal_acceptance").value)
        self.wait_for_result = bool(self.get_parameter("wait_for_result").value)
        self.wait_for_result_timeout_s = float(self.get_parameter("wait_for_result_timeout_s").value)
        self.resample_before_forward = bool(self.get_parameter("resample_before_forward").value)
        self.resample_rate_hz = float(self.get_parameter("resample_rate_hz").value)

        self.callback_group = ReentrantCallbackGroup()
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
            f"resample={self.resample_before_forward}@{self.resample_rate_hz:.1f}Hz"
        )

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

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = (
            resample_trajectory(request.trajectory, self.resample_rate_hz)
            if self.resample_before_forward
            else request.trajectory
        )
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
