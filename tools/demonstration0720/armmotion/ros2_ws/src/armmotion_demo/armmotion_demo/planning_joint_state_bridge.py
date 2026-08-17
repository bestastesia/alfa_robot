from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def with_fixed_pitch(message: JointState, pitch: float = 0.0) -> JointState:
    output = JointState()
    output.header = message.header
    output.name = list(message.name)
    output.position = list(message.position)
    output.velocity = list(message.velocity)
    output.effort = list(message.effort)
    if "pitch" in output.name:
        return output

    output.name.append("pitch")
    output.position.append(float(pitch))
    if output.velocity:
        output.velocity.append(0.0)
    if output.effort:
        output.effort.append(0.0)
    return output


class PlanningJointStateBridge(Node):
    def __init__(self) -> None:
        super().__init__("motion_planning_joint_state_bridge")
        self.declare_parameter("input_topic", "/joint_states")
        self.declare_parameter("output_topic", "/motion/internal/model_joint_states")
        self.declare_parameter("fixed_pitch", 0.0)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.fixed_pitch = float(self.get_parameter("fixed_pitch").value)
        self.publisher = self.create_publisher(
            JointState,
            self.output_topic,
            qos_profile_sensor_data,
        )
        input_topic = str(self.get_parameter("input_topic").value)
        self.subscription = self.create_subscription(
            JointState,
            input_topic,
            self.on_joint_state,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"规划状态适配已就绪：{input_topic} -> {self.output_topic}; "
            f"固定补齐 pitch={self.fixed_pitch:.3f}，turn 原样透传"
        )

    def on_joint_state(self, message: JointState) -> None:
        self.publisher.publish(with_fixed_pitch(message, self.fixed_pitch))


def main() -> None:
    rclpy.init()
    node = PlanningJointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
