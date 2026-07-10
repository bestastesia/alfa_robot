#!/usr/bin/env python3
"""Read-only Rerun viewer for the authoritative joint-state stream."""

from __future__ import annotations

import sys

import rclpy
import rerun as rr
from rclpy.node import Node
from sensor_msgs.msg import JointState

from alfa_robot_rerun.visualize_rerun import (
    UrdfRobot,
    log_robot_state,
    log_robot_static_model,
    render_current_urdf,
)


class BasicRerunRobotViewer(Node):
    def __init__(self) -> None:
        super().__init__("alfa_rerun_basic_robot_viewer")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("application_id", "alfa_robot_basic_viewer")
        self.declare_parameter("spawn", True)
        self.declare_parameter("recording_path", "")
        self.declare_parameter("world_path", "world")
        self.declare_parameter("log_meshes", True)

        self.world_path = str(self.get_parameter("world_path").value)
        application_id = str(self.get_parameter("application_id").value)
        spawn = bool(self.get_parameter("spawn").value)
        recording_path = str(self.get_parameter("recording_path").value)
        log_meshes = bool(self.get_parameter("log_meshes").value)

        rr.init(application_id, spawn=spawn)
        if recording_path:
            rr.save(recording_path)
            self.get_logger().info(f"Rerun recording will be saved to: {recording_path}")

        urdf_text = str(self.get_parameter("robot_description").value) or render_current_urdf()
        self.robot = UrdfRobot(urdf_text)
        self.joint_positions: dict[str, float] = {}
        log_robot_static_model(self.robot, self.world_path, log_meshes=log_meshes)
        log_robot_state(self.robot, self.joint_positions, self.world_path)

        topic = str(self.get_parameter("joint_states_topic").value)
        self.subscription = self.create_subscription(JointState, topic, self.on_joint_state, 10)
        self.get_logger().info(
            f"Rerun basic robot viewer ready: root={self.robot.root_link}, joint_states={topic}"
        )

    def on_joint_state(self, msg: JointState) -> None:
        self.joint_positions.update(
            {name: float(position) for name, position in zip(msg.name, msg.position)}
        )
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp > 0.0:
            rr.set_time("ros_time", timestamp=stamp)
        log_robot_state(self.robot, self.joint_positions, self.world_path)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BasicRerunRobotViewer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
