#!/usr/bin/env python3
# Copyright (c) 2025, b»robotized
#
# Licensed under the Apache License, Version 2.0 (the "License");

"""
桥接节点：将 joint_state_publisher_gui 的滑块输出转发到位置控制器。
订阅 /joint_states_gui，提取指定关节位置，发布到控制器 commands topic。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class JointStatesToControllerBridge(Node):
    """将 GUI 关节状态转发到位置控制器。"""

    def __init__(self):
        super().__init__("joint_states_to_controller_bridge")

        self.declare_parameter(
            "joint_names",
            [
                "turn", "updown",
                "leftarmbase", "leftjoint1", "leftjoint2", "leftjoint3", "leftjoint4",
                "rightarmbase", "rightjoint1", "rightjoint2", "rightjoint3", "rightjoint4",
            ],
        )
        self.declare_parameter("command_topic", "/all_position_controller/commands")
        self.declare_parameter("joint_states_topic", "/joint_states_gui")

        self.joint_names = self.get_parameter("joint_names").get_parameter_value().string_array_value
        command_topic = self.get_parameter("command_topic").get_parameter_value().string_value
        joint_states_topic = self.get_parameter("joint_states_topic").get_parameter_value().string_value

        self.subscription = self.create_subscription(
            JointState,
            joint_states_topic,
            self.joint_states_callback,
            10,
        )
        self.publisher = self.create_publisher(Float64MultiArray, command_topic, 10)

        self.get_logger().info(
            f"Bridge: {joint_states_topic} -> {command_topic} (joints: {self.joint_names})"
        )

    def joint_states_callback(self, msg: JointState):
        positions = []
        for name in self.joint_names:
            try:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
            except ValueError:
                self.get_logger().warn(f"Joint '{name}' not found in JointState")
                return

        out = Float64MultiArray()
        out.data = positions
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointStatesToControllerBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
