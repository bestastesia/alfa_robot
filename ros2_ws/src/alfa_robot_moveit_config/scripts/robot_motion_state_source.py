#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from robot_motion_interfaces.msg import RobotMotionState


def stamp_is_zero(stamp) -> bool:
    return stamp.sec == 0 and stamp.nanosec == 0


class RobotMotionStateSource(Node):
    def __init__(self) -> None:
        super().__init__("robot_motion_state_source")
        self.declare_parameter("input_joint_states", "/joint_states")
        self.declare_parameter("output_state_topic", "/robot_motion/state")
        self.declare_parameter("source", "joint_states")
        self.declare_parameter("authoritative", True)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("scene_id", "")
        self.declare_parameter("state_id_prefix", "")
        self.declare_parameter("publish_period_s", 0.0)

        self.input_topic = str(self.get_parameter("input_joint_states").value)
        self.output_topic = str(self.get_parameter("output_state_topic").value)
        self.source = str(self.get_parameter("source").value)
        self.authoritative = bool(self.get_parameter("authoritative").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.scene_id = str(self.get_parameter("scene_id").value)
        self.state_id_prefix = str(self.get_parameter("state_id_prefix").value)
        self.publish_period_s = float(self.get_parameter("publish_period_s").value)

        self.publisher = self.create_publisher(RobotMotionState, self.output_topic, 10)
        self.subscription = self.create_subscription(
            JointState,
            self.input_topic,
            self.on_joint_state,
            qos_profile_sensor_data,
        )
        self.latest_state: Optional[RobotMotionState] = None
        self.timer = None
        if self.publish_period_s > 0.0 and math.isfinite(self.publish_period_s):
            self.timer = self.create_timer(self.publish_period_s, self.publish_latest)

        self.get_logger().info(
            "Robot motion state source ready: "
            f"{self.input_topic} -> {self.output_topic}, "
            f"source={self.source}, authoritative={self.authoritative}"
        )

    def make_state_id(self, stamp) -> str:
        prefix = self.state_id_prefix or self.source
        return f"{prefix}:{stamp.sec}.{stamp.nanosec:09d}"

    def on_joint_state(self, joint_state: JointState) -> None:
        now = self.get_clock().now().to_msg()
        stamp = now if stamp_is_zero(joint_state.header.stamp) else joint_state.header.stamp

        state = RobotMotionState()
        state.context.request_id = ""
        state.context.stamp = stamp
        state.context.frame_id = self.frame_id
        state.context.scene_id = self.scene_id
        state.context.state_id = self.make_state_id(stamp)
        state.source = self.source
        state.authoritative = self.authoritative
        state.joint_state = joint_state
        state.joint_state.header.stamp = stamp
        self.latest_state = state
        self.publisher.publish(state)

    def publish_latest(self) -> None:
        if self.latest_state is not None:
            self.publisher.publish(self.latest_state)


def main() -> None:
    rclpy.init()
    node = RobotMotionStateSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
