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

等待指定控制器进入 active 状态，然后向其 commands 话题发布一次全零位指令（或
自定义 home_positions 参数），之后节点自动退出。

参数:
  controller_name  (str)          -- 控制器名称，默认 all_position_controller
  home_positions   (double[])     -- 零位目标，默认全 0.0（12 个关节）
  publish_count    (int)          -- 重复发布次数，防止首包丢失，默认 5
  poll_interval    (double)       -- 轮询控制器状态的间隔(s)，默认 1.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import ListControllers


_DEFAULT_HOME = [0.0] * 12  # turn, updown, leftarmbase, leftjoint1..4, rightarmbase, rightjoint1..4


class HomingNode(Node):
    def __init__(self) -> None:
        super().__init__('homing_node')

        self.declare_parameter('controller_name', 'all_position_controller')
        self.declare_parameter('home_positions', _DEFAULT_HOME)
        self.declare_parameter('publish_count', 5)
        self.declare_parameter('poll_interval', 1.0)

        self._controller_name: str = self.get_parameter('controller_name').value
        self._home_positions: list[float] = list(self.get_parameter('home_positions').value)
        self._publish_count: int = self.get_parameter('publish_count').value
        poll_interval: float = self.get_parameter('poll_interval').value

        self._publisher = self.create_publisher(
            Float64MultiArray,
            f'/{self._controller_name}/commands',
            10,
        )
        self._cli = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self._pending: bool = False
        self._done: bool = False

        self._timer = self.create_timer(poll_interval, self._poll)
        self.get_logger().info(
            f'HomingNode: waiting for "{self._controller_name}" to become active '
            f'(poll every {poll_interval:.1f}s)...'
        )

    # ── polling ──────────────────────────────────────────────────────────────

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

        for ctrl in result.controller:
            if ctrl.name == self._controller_name and ctrl.state == 'active':
                self._send_home()
                return

    # ── homing ───────────────────────────────────────────────────────────────

    def _send_home(self) -> None:
        self._done = True
        self._timer.cancel()

        msg = Float64MultiArray()
        msg.data = self._home_positions

        for _ in range(self._publish_count):
            self._publisher.publish(msg)

        self.get_logger().info(
            f'HomingNode: sent home positions {self._home_positions} '
            f'({self._publish_count}x). Shutting down.'
        )
        # Request clean shutdown via executor
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
