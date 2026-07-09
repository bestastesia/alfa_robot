from __future__ import annotations

import threading

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_motion_interfaces.srv import ExecuteTrajectory
from robot_motion_runtime.common import RuntimeStatusPublisher


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

        self.service_name = str(self.get_parameter("service_name").value)
        self.action_name = str(self.get_parameter("action_name").value)
        self.forward_action = bool(self.get_parameter("forward_action").value)
        self.wait_for_action_timeout_s = float(self.get_parameter("wait_for_action_timeout_s").value)

        self.action_client = ActionClient(self, FollowJointTrajectory, self.action_name)
        self.service = self.create_service(ExecuteTrajectory, self.service_name, self.on_execute)
        self.status = RuntimeStatusPublisher(
            self,
            self.service_name,
            "ExecuteTrajectory service; optionally forwards to FollowJointTrajectory action backend",
        )
        self.status.mark_ready(f"action={self.action_name}, forward_action={self.forward_action}")
        self.get_logger().info(
            f"ExecuteTrajectory service ready: service={self.service_name} "
            f"action={self.action_name} forward={self.forward_action}"
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
        goal.trajectory = request.trajectory
        future = self.action_client.send_goal_async(goal)
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
        response.message = "action goal accepted" if response.accepted else "action goal rejected"
        self.status.mark_done(response.accepted, response.message)
        return response


def main() -> None:
    rclpy.init()
    node = ExecuteTrajectoryServiceNode()
    executor = MultiThreadedExecutor(num_threads=2)
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
