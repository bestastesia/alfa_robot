#!/usr/bin/env python3
# Copyright (c) 2025, b»robotized
#
# Licensed under the Apache License, Version 2.0 (the "License");

"""
桥接节点：将 joint_state_publisher_gui 的滑块输出直接转发为
forward_command_controller 的位置命令 (Float64MultiArray)。

不再走 JointTrajectoryController —— 滑块是连续重复发布的，
轨迹控制器会被持续抢占导致电机不动 / 关 GUI 后才执行最后一帧。
直接前向位置命令时，硬件层会把每个值当作即时目标位置下发到电机。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


_DEFAULT_JOINT_ORDER = [
    "pitch",
    "turn",
    "updown",
    "leftjoint1",
    "leftjoint2",
    "leftjoint3",
    "leftjoint4",
    "leftjoint5",
    "leftjoint6",
    "rightjoint1",
    "rightjoint2",
    "rightjoint3",
    "rightjoint4",
    "rightjoint5",
    "rightjoint6",
]

class JointStatesToControllerBridge(Node):
    """将 GUI 关节状态转发到 forward_command_controller。"""

    def __init__(self):
        super().__init__("joint_states_to_controller_bridge")

        self.declare_parameter("joint_states_topic", "/joint_states_gui")
        self.declare_parameter("controller_name", "all_position_controller")
        self.declare_parameter("joint_names", _DEFAULT_JOINT_ORDER)

        joint_states_topic = self.get_parameter("joint_states_topic").value
        controller_name = self.get_parameter("controller_name").value
        self._joint_names = list(self.get_parameter("joint_names").value)

        self._publisher = self.create_publisher(
            Float64MultiArray, f"/{controller_name}/commands", 10
        )

        self._warned_missing = False
        self.subscription = self.create_subscription(
            JointState,
            joint_states_topic,
            self.joint_states_callback,
            10,
        )

        self.get_logger().info(
            f"Bridge: {joint_states_topic} -> /{controller_name}/commands "
            f"joints={self._joint_names}"
        )

    def joint_states_callback(self, msg: JointState):
        positions = {
            name: msg.position[index]
            for index, name in enumerate(msg.name)
            if index < len(msg.position)
        }
        try:
            ordered = [positions[name] for name in self._joint_names]
        except KeyError:
            if not self._warned_missing:
                self.get_logger().warn(
                    f"Waiting for joints {self._joint_names} in {msg.name}"
                )
                self._warned_missing = True
            return

        cmd = Float64MultiArray()
        cmd.data = ordered
        self._publisher.publish(cmd)


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
