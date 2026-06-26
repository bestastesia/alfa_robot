from __future__ import annotations

import argparse
import math
import sys

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from alfa_robot_execution_bridge.joints import DEFAULT_JOINT_NAMES


def seconds_to_duration(seconds: float):
    msg = JointTrajectoryPoint().time_from_start
    whole = math.floor(max(0.0, seconds))
    msg.sec = int(whole)
    msg.nanosec = int(round((max(0.0, seconds) - whole) * 1e9))
    if msg.nanosec >= 1_000_000_000:
        msg.sec += 1
        msg.nanosec -= 1_000_000_000
    return msg


class MockTrajectoryClient(Node):
    def __init__(self, action_name: str) -> None:
        super().__init__('alfa_send_mock_trajectory')
        self._client = ActionClient(self, FollowJointTrajectory, action_name)

    def send(self, duration_s: float, amplitude_deg: float) -> bool:
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('execution action server is not available')
            return False

        amplitude = math.radians(amplitude_deg)
        trajectory = JointTrajectory()
        trajectory.joint_names = list(DEFAULT_JOINT_NAMES)
        trajectory.points = [
            self._point(0.0, [0.0] * len(DEFAULT_JOINT_NAMES)),
            self._point(duration_s * 0.5, self._wave_positions(amplitude)),
            self._point(duration_s, [0.0] * len(DEFAULT_JOINT_NAMES)),
        ]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('trajectory goal was rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f'trajectory failed: {result.error_code} {result.error_string}')
            return False
        self.get_logger().info('trajectory finished successfully')
        return True

    def _point(self, time_s: float, positions: list[float]) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = seconds_to_duration(time_s)
        return point

    def _wave_positions(self, amplitude: float) -> list[float]:
        values = [0.0] * len(DEFAULT_JOINT_NAMES)
        for name, factor in {
            'right_joint2': -0.8,
            'right_joint5': 0.5,
            'left_joint2': 0.8,
            'left_joint5': -0.5,
            'turn': 0.4,
        }.items():
            values[DEFAULT_JOINT_NAMES.index(name)] = amplitude * factor
        return values


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Send a simple 13-joint trajectory to the ALFA mock execution bridge.')
    parser.add_argument('--action', default='/alfa_execution/execute_joint_trajectory')
    parser.add_argument('--duration-s', type=float, default=4.0)
    parser.add_argument('--amplitude-deg', type=float, default=15.0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init()
    node = MockTrajectoryClient(args.action)
    try:
        return 0 if node.send(args.duration_s, args.amplitude_deg) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
