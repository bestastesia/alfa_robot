#!/usr/bin/env python3
"""
零差云控电机（can0 Node 1,2）滑块测试
仅控制 leftjoint2 和 leftjoint3 两个关节
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class ZeroerrSliderBridge(Node):
    """将 GUI 关节状态转发到零差云控电机控制器。"""

    def __init__(self):
        super().__init__("zeroerr_slider_bridge")

        self.declare_parameter("joint_states_topic", "/joint_states_gui")
        self.declare_parameter("controller_name", "zeroerr_slider_controller")
        # 只控制 leftjoint2 和 leftjoint3
        self._joint_names = ["leftjoint2", "leftjoint3"]

        joint_states_topic = self.get_parameter("joint_states_topic").value
        controller_name = self.get_parameter("controller_name").value

        self._publisher = self.create_publisher(
            Float64MultiArray, f"/{controller_name}/commands", 10
        )

        self.subscription = self.create_subscription(
            JointState,
            joint_states_topic,
            self.joint_states_callback,
            10,
        )

        self.get_logger().info(
            f"ZeroErr Bridge: {joint_states_topic} -> /{controller_name}/commands "
            f"joints={self._joint_names}"
        )

    def joint_states_callback(self, msg: JointState):
        positions = {
            name: msg.position[index]
            for index, name in enumerate(msg.name)
            if index < len(msg.position)
        }

        # 只提取 leftjoint2 和 leftjoint3
        try:
            ordered = [positions[name] for name in self._joint_names]
        except KeyError:
            # 关节名称不匹配时自动填充 0
            ordered = [0.0, 0.0]

        cmd = Float64MultiArray()
        cmd.data = ordered
        self._publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ZeroerrSliderBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
