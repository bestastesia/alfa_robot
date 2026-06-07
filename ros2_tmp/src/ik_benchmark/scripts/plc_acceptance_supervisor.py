#!/usr/bin/python3
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from alfa_robot_benchmarks.msg import TaskStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


@dataclass
class PlcState:
    mode: str = "unknown"
    state: str = "unknown"
    executing: bool = False
    last_error: str = ""
    last_command_count: int = 0
    last_trajectory_duration_s: float = 0.0


def parse_state(text: str) -> PlcState:
    data: Dict[str, str] = {}
    for item in text.split(';'):
        if '=' not in item:
            continue
        key, value = item.split('=', 1)
        data[key.strip()] = value.strip()
    out = PlcState()
    out.mode = data.get('mode', out.mode)
    out.state = data.get('state', out.state)
    out.executing = data.get('executing', 'false').lower() == 'true'
    out.last_error = data.get('last_error', '')
    try:
        out.last_command_count = int(data.get('last_command_count', '0'))
    except ValueError:
        out.last_command_count = 0
    try:
        out.last_trajectory_duration_s = float(data.get('last_trajectory_duration_s', '0'))
    except ValueError:
        out.last_trajectory_duration_s = 0.0
    return out


class PlcAcceptanceSupervisor(Node):
    def __init__(self) -> None:
        super().__init__('plc_acceptance_supervisor')
        self.state_topic = self.declare_parameter('state_topic', '/plc_bridge_state').value
        self.trajectory_topic = self.declare_parameter('trajectory_topic', '/plc_joint_trajectory').value
        self.task_status_topic = self.declare_parameter('task_status_topic', '/alfa_task/status').value
        self.enable_home_hold = bool(self.declare_parameter('enable_home_hold', False).value)
        self.home_period_s = float(self.declare_parameter('home_period_s', 1.0).value)
        self.home_duration_s = float(self.declare_parameter('home_duration_s', 0.5).value)
        self.idle_required_s = float(self.declare_parameter('idle_required_s', 1.0).value)
        self.max_home_commands = int(self.declare_parameter('max_home_commands', 0).value)
        self.log_period_s = float(self.declare_parameter('log_period_s', 1.0).value)
        self.stop_home_hold_on_task = bool(self.declare_parameter('stop_home_hold_on_task', True).value)
        self.fixed_updown = float(self.declare_parameter('fixed_updown', 0.18).value)
        self.turn = float(self.declare_parameter('turn', 0.0).value)
        self.include_unmapped_base_joints = bool(self.declare_parameter('include_unmapped_base_joints', False).value)
        self.home_deg = [float(v) for v in self.declare_parameter('home_arm_deg', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).value]
        self.arm_joint_names: List[str] = [
            'left_v5_joint1', 'left_v5_joint2', 'left_v5_joint3', 'left_v5_joint4', 'left_v5_joint5', 'left_v5_joint6',
            'right_v5_joint1', 'right_v5_joint2', 'right_v5_joint3', 'right_v5_joint4', 'right_v5_joint5', 'right_v5_joint6',
        ]
        self.latest_state = PlcState()
        self.have_state = False
        self.last_state_text = ''
        self.last_change_time = self.get_clock().now()
        self.last_home_time = self.get_clock().now()
        self.home_command_count = 0
        self.task_active = False
        self.home_hold_stopped_by_task = False
        self.last_task_id = ''
        self.last_task_state = ''

        self.create_subscription(String, self.state_topic, self._on_state, 10)
        self.create_subscription(TaskStatus, self.task_status_topic, self._on_task_status, 10)
        self.trajectory_pub = self.create_publisher(JointTrajectory, self.trajectory_topic, 10)
        self.create_timer(self.log_period_s, self._log_summary)
        self.create_timer(0.1, self._maybe_publish_home)
        self.get_logger().info(
            f'PLC acceptance supervisor ready: state={self.state_topic} trajectory={self.trajectory_topic} '
            f'home_hold={self.enable_home_hold} period={self.home_period_s:.2f}s home_deg={self.home_deg} '
            f'stop_on_task={self.stop_home_hold_on_task}'
        )

    def _on_state(self, msg: String) -> None:
        self.latest_state = parse_state(msg.data)
        self.have_state = True
        if msg.data != self.last_state_text:
            self.last_state_text = msg.data
            self.last_change_time = self.get_clock().now()
            self.get_logger().info(
                'PLC state changed: '
                f'mode={self.latest_state.mode} state={self.latest_state.state} executing={self.latest_state.executing} '
                f'commands={self.latest_state.last_command_count} duration={self.latest_state.last_trajectory_duration_s:.3f}s '
                f'error={self.latest_state.last_error}'
            )

    def _on_task_status(self, msg: TaskStatus) -> None:
        self.last_task_id = msg.task_id
        self.last_task_state = msg.state
        if msg.state in {'accepted', 'running'}:
            self.task_active = True
            if self.stop_home_hold_on_task and not self.home_hold_stopped_by_task:
                self.home_hold_stopped_by_task = True
                self.get_logger().warn(
                    f'home hold stopped because task started: task={msg.task_id} state={msg.state}'
                )
        elif msg.state in {'done', 'failed', 'rejected'}:
            self.task_active = False

    def _log_summary(self) -> None:
        if not self.have_state:
            self.get_logger().warn(f'No PLC state received yet from {self.state_topic}')
            return
        self.get_logger().info(
            'PLC monitor: '
            f'mode={self.latest_state.mode} state={self.latest_state.state} executing={self.latest_state.executing} '
            f'last_commands={self.latest_state.last_command_count} home_commands={self.home_command_count} '
            f'task={self.last_task_id}:{self.last_task_state} home_stopped={self.home_hold_stopped_by_task}'
        )

    def _maybe_publish_home(self) -> None:
        if not self.enable_home_hold:
            return
        if self.home_hold_stopped_by_task:
            return
        if self.task_active:
            return
        if self.max_home_commands > 0 and self.home_command_count >= self.max_home_commands:
            return
        if not self.have_state:
            return
        if self.latest_state.executing or self.latest_state.state != 'normal':
            return
        now = self.get_clock().now()
        idle_s = (now - self.last_change_time).nanoseconds * 1e-9
        since_home_s = (now - self.last_home_time).nanoseconds * 1e-9
        if idle_s < self.idle_required_s or since_home_s < self.home_period_s:
            return
        subscribers = self.trajectory_pub.get_subscription_count()
        if subscribers == 0:
            self.get_logger().warn(f'home hold skipped: no subscribers on {self.trajectory_topic}')
            self.last_home_time = now
            return
        self.trajectory_pub.publish(self._make_home_trajectory())
        self.last_home_time = now
        self.home_command_count += 1
        self.get_logger().info(
            f'home hold command published #{self.home_command_count}: topic={self.trajectory_topic} subscribers={subscribers}'
        )

    def _make_home_trajectory(self) -> JointTrajectory:
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.include_unmapped_base_joints:
            msg.joint_names = ['turn', 'updown'] + self.arm_joint_names
            positions = [self.turn, self.fixed_updown]
        else:
            msg.joint_names = list(self.arm_joint_names)
            positions = []
        arm_rad = [math.radians(v) for v in self.home_deg]
        positions.extend(arm_rad)
        positions.extend(arm_rad)
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(self.home_duration_s)
        point.time_from_start.nanosec = int((self.home_duration_s - int(self.home_duration_s)) * 1e9)
        msg.points.append(point)
        return msg


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PlcAcceptanceSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
