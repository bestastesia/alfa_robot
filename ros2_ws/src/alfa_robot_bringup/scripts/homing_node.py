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

等待指定控制器全部进入 active 状态，然后向各自的 joint_trajectory 话题发布一次
单点回零轨迹（或自定义 home_positions 参数），之后节点自动退出。

参数:
  controller_names (str[])        -- 控制器名称列表
  home_positions   (double[])     -- 零位目标，默认全 0.0（12 个关节）
  publish_count    (int)          -- 重复发布次数，防止首包丢失，默认 5
  poll_interval    (double)       -- 轮询控制器状态的间隔(s)，默认 1.0
  trajectory_time  (double)       -- 单点轨迹到达时间(s)，默认 1.0
"""

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


_CONTROLLER_JOINTS = {
    "torso_group_controller": ["turn", "updown"],
    "left_arm_controller": ["leftarmbase", "leftjoint1", "leftjoint2", "leftjoint3", "leftjoint4"],
    "right_arm_controller": ["rightarmbase", "rightjoint1", "rightjoint2", "rightjoint3", "rightjoint4"],
}
_DEFAULT_CONTROLLER_NAMES = list(_CONTROLLER_JOINTS.keys())
_DEFAULT_HOME = [0.0] * sum(len(joints) for joints in _CONTROLLER_JOINTS.values())


class HomingNode(Node):
    def __init__(self) -> None:
        super().__init__("homing_node")

        self.declare_parameter("controller_names", _DEFAULT_CONTROLLER_NAMES)
        self.declare_parameter("home_positions", _DEFAULT_HOME)
        self.declare_parameter("publish_count", 5)
        self.declare_parameter("poll_interval", 1.0)
        self.declare_parameter("trajectory_time", 1.0)

        self._controller_names: list[str] = list(self.get_parameter("controller_names").value)
        self._home_positions: list[float] = list(self.get_parameter("home_positions").value)
        self._publish_count: int = self.get_parameter("publish_count").value
        poll_interval: float = self.get_parameter("poll_interval").value
        self._time_from_start = self._duration_from_seconds(
            float(self.get_parameter("trajectory_time").value)
        )
        self._controller_groups = self._build_controller_groups()

        self._cli = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self._pending = False
        self._done = False

        self._timer = self.create_timer(poll_interval, self._poll)
        self.get_logger().info(
            f"HomingNode: waiting for {self._controller_names} to become active "
            f"(poll every {poll_interval:.1f}s)..."
        )

    def _poll(self) -> None:
        if self._done or self._pending:
            return
        if not self._cli.service_is_ready():
            self.get_logger().debug("controller_manager not ready yet, retrying...")
            return
        self._pending = True
        future = self._cli.call_async(ListControllers.Request())
        future.add_done_callback(self._on_list)

    def _on_list(self, future) -> None:
        self._pending = False
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(f"ListControllers failed: {exc}")
            return

        active_controllers = {ctrl.name for ctrl in result.controller if ctrl.state == "active"}
        if all(name in active_controllers for name in self._controller_names):
            self._send_home()

    def _send_home(self) -> None:
        self._done = True
        self._timer.cancel()

        for group in self._controller_groups:
            trajectory = JointTrajectory()
            trajectory.joint_names = group["joint_names"]

            point = JointTrajectoryPoint()
            point.positions = group["positions"]
            point.time_from_start = self._time_from_start
            trajectory.points = [point]

            for _ in range(self._publish_count):
                group["publisher"].publish(trajectory)

        self.get_logger().info(
            f"HomingNode: sent home positions {self._home_positions} "
            f"to {self._controller_names} ({self._publish_count}x). Shutting down."
        )
        raise SystemExit(0)

    def _build_controller_groups(self):
        expected_joint_count = sum(len(_CONTROLLER_JOINTS[name]) for name in self._controller_names)
        if len(self._home_positions) != expected_joint_count:
            raise ValueError(
                f"home_positions length {len(self._home_positions)} does not match "
                f"configured controllers {self._controller_names} ({expected_joint_count} joints)"
            )

        groups = []
        cursor = 0
        for controller_name in self._controller_names:
            if controller_name not in _CONTROLLER_JOINTS:
                raise ValueError(f"Unsupported controller_name: {controller_name}")
            joint_names = _CONTROLLER_JOINTS[controller_name]
            next_cursor = cursor + len(joint_names)
            groups.append(
                {
                    "controller_name": controller_name,
                    "joint_names": joint_names,
                    "positions": self._home_positions[cursor:next_cursor],
                    "publisher": self.create_publisher(
                        JointTrajectory,
                        f"/{controller_name}/joint_trajectory",
                        10,
                    ),
                }
            )
            cursor = next_cursor
        return groups

    @staticmethod
    def _duration_from_seconds(seconds: float) -> Duration:
        sec = int(seconds)
        nanosec = int((seconds - sec) * 1_000_000_000)
        return Duration(sec=sec, nanosec=nanosec)


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


if __name__ == "__main__":
    main()
