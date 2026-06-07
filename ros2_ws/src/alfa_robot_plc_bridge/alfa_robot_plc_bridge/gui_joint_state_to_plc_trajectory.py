"""Convert joint_state_publisher_gui output to small PLC JointTrajectory commands."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


DEFAULT_JOINT_NAMES = [
    'left_v5_joint1',
    'left_v5_joint2',
    'left_v5_joint3',
    'left_v5_joint4',
    'left_v5_joint5',
    'left_v5_joint6',
    'right_v5_joint1',
    'right_v5_joint2',
    'right_v5_joint3',
    'right_v5_joint4',
    'right_v5_joint5',
    'right_v5_joint6',
]


class GuiJointStateToPlcTrajectory(Node):
    def __init__(self) -> None:
        super().__init__('gui_joint_state_to_plc_trajectory')
        self.declare_parameter('source_topic', '/joint_states_gui')
        self.declare_parameter('target_topic', '/plc_joint_trajectory')
        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)
        self.declare_parameter('command_duration_s', 0.2)
        self.declare_parameter('min_publish_period_s', 0.05)
        self.declare_parameter('min_position_delta_rad', 0.0005)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.command_duration_s = float(self.get_parameter('command_duration_s').value)
        self.min_publish_period_s = float(self.get_parameter('min_publish_period_s').value)
        self.min_position_delta_rad = float(self.get_parameter('min_position_delta_rad').value)
        self.last_positions: dict[str, float] | None = None
        self.last_publish_time = self.get_clock().now()

        self.publisher = self.create_publisher(JointTrajectory, str(self.get_parameter('target_topic').value), 10)
        self.subscription = self.create_subscription(
            JointState,
            str(self.get_parameter('source_topic').value),
            self._on_joint_state,
            10,
        )
        self.get_logger().info(
            f'GUI bridge: {self.get_parameter("source_topic").value} -> {self.get_parameter("target_topic").value}, joints={self.joint_names}'
        )

    def _on_joint_state(self, msg: JointState) -> None:
        positions = {
            name: msg.position[index]
            for index, name in enumerate(msg.name)
            if index < len(msg.position)
        }
        if any(name not in positions for name in self.joint_names):
            return
        target_positions = {name: positions[name] for name in self.joint_names}
        now = self.get_clock().now()
        if self.last_positions is None:
            self.last_positions = target_positions
            self.last_publish_time = now
            self.get_logger().info('GUI initial joint state captured; waiting for slider movement')
            return
        elapsed = (now - self.last_publish_time).nanoseconds * 1e-9
        max_delta = max(abs(target_positions[name] - self.last_positions[name]) for name in self.joint_names)
        if max_delta < self.min_position_delta_rad:
            return
        if elapsed < self.min_publish_period_s:
            return
        self.last_positions = target_positions
        self.last_publish_time = now

        trajectory = JointTrajectory()
        trajectory.header.stamp = now.to_msg()
        trajectory.joint_names = list(self.joint_names)
        point = JointTrajectoryPoint()
        point.positions = [target_positions[name] for name in self.joint_names]
        point.time_from_start.sec = int(self.command_duration_s)
        point.time_from_start.nanosec = int((self.command_duration_s % 1.0) * 1e9)
        trajectory.points = [point]
        self.publisher.publish(trajectory)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GuiJointStateToPlcTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
