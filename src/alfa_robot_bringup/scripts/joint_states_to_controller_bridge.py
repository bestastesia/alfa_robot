#!/usr/bin/env python3
# Copyright (c) 2025, b»robotized
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
桥接节点：将 joint_state_publisher_gui 的滑块输出转发到 JointTrajectoryController。
订阅 /joint_states_gui，提取关节位置，向四个 JTC 控制器各自发布 JointTrajectory。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


_CONTROLLER_JOINTS = {
    "torso_group_controller": ["turn", "updown"],
    "left_arm_controller": ["leftarmbase", "leftjoint1", "leftjoint2", "leftjoint3", "leftjoint4"],
    "right_arm_controller": ["rightarmbase", "rightjoint1", "rightjoint2", "rightjoint3", "rightjoint4"],
    "plate_controller": ["plate"],
}


class JointStatesToControllerBridge(Node):
    """将 GUI 关节状态转发到 JointTrajectoryController。"""

    def __init__(self):
        super().__init__("joint_states_to_controller_bridge")

        self.declare_parameter("joint_states_topic", "/joint_states_gui")
        joint_states_topic = self.get_parameter("joint_states_topic").get_parameter_value().string_value

        self._publishers = {}
        for controller, joints in _CONTROLLER_JOINTS.items():
            topic = f"/{controller}/joint_trajectory"
            self._publishers[controller] = (
                self.create_publisher(JointTrajectory, topic, 10),
                joints,
            )

        self.subscription = self.create_subscription(
            JointState,
            joint_states_topic,
            self._joint_states_callback,
            10,
        )
        self.get_logger().info(f"Bridge: {joint_states_topic} -> 4x JointTrajectory topics")

    def _joint_states_callback(self, msg: JointState):
        for controller, (pub, joints) in self._publishers.items():
            positions = []
            for name in joints:
                try:
                    idx = msg.name.index(name)
                    positions.append(msg.position[idx])
                except ValueError:
                    return  # joint not yet available, skip this cycle

            traj = JointTrajectory()
            traj.joint_names = list(joints)
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = Duration(sec=0, nanosec=100_000_000)  # 100ms
            traj.points = [point]
            pub.publish(traj)


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
