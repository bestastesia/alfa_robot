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
启动自动回零位节点。

等待四个 JTC 控制器全部进入 active 状态，然后向每个控制器发布一次全零位
JointTrajectory 指令，之后节点自动退出。

参数:
  publish_count    (int)    -- 重复发布次数，防止首包丢失，默认 5
  poll_interval    (double) -- 轮询控制器状态的间隔(s)，默认 1.0
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers


_CONTROLLERS = {
    "torso_group_controller": ["turn", "updown"],
    "left_arm_controller": ["leftarmbase", "leftjoint1", "leftjoint2", "leftjoint3", "leftjoint4"],
    "right_arm_controller": ["rightarmbase", "rightjoint1", "rightjoint2", "rightjoint3", "rightjoint4"],
    "plate_controller": ["plate"],
}


class HomingNode(Node):
    def __init__(self) -> None:
        super().__init__('homing_node')

        self.declare_parameter('publish_count', 5)
        self.declare_parameter('poll_interval', 1.0)

        self._publish_count: int = self.get_parameter('publish_count').value
        poll_interval: float = self.get_parameter('poll_interval').value

        self._publishers = {}
        for controller, joints in _CONTROLLERS.items():
            topic = f'/{controller}/joint_trajectory'
            self._publishers[controller] = (
                self.create_publisher(JointTrajectory, topic, 10),
                joints,
            )

        self._cli = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self._pending: bool = False
        self._done: bool = False

        self._timer = self.create_timer(poll_interval, self._poll)
        self.get_logger().info(
            f'HomingNode: waiting for all JTC controllers to become active '
            f'(poll every {poll_interval:.1f}s)...'
        )

    def _poll(self) -> None:
        if self._done or self._pending:
            return
        if not self._cli.service_is_ready():
            self.get_logger().debug('controller_manager not ready yet, retrying...')
            return
        self._pending = True
        future = self._cli.call_async(ListControllers.Request())
        future.add_done_callback(self._on_list)

    def _on_list(self, future) -> None:
        self._pending = False
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(f'ListControllers failed: {exc}')
            return

        active = {ctrl.name for ctrl in result.controller if ctrl.state == 'active'}
        required = set(_CONTROLLERS.keys())
        if required.issubset(active):
            self._send_home()

    def _send_home(self) -> None:
        self._done = True
        self._timer.cancel()

        for controller, (pub, joints) in self._publishers.items():
            traj = JointTrajectory()
            traj.joint_names = list(joints)
            point = JointTrajectoryPoint()
            point.positions = [0.0] * len(joints)
            point.time_from_start = Duration(sec=2, nanosec=0)
            traj.points = [point]
            for _ in range(self._publish_count):
                pub.publish(traj)

        self.get_logger().info('HomingNode: sent home positions to all controllers. Shutting down.')
        raise SystemExit(0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HomingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
