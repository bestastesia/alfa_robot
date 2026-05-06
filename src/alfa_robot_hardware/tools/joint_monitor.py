#!/usr/bin/env python3
"""
监听 JTC 状态和关节反馈，只在位置发生显著变化时输出。
用法: python3 joint_monitor.py
"""

import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState


class JointMonitor(Node):
    def __init__(self):
        super().__init__('joint_monitor')

        # 每个关节的上次记录位置，变化超过阈值才输出
        self.threshold = 0.005  # rad 或 m
        self.last_state = {}    # joint_name -> position
        self.last_cmd = {}      # joint_name -> position
        self.start_time = time.time()
        self.has_cmd_data = False

        self.state_sub = self.create_subscription(
            JointState, '/joint_states', self.on_state, 10)
        self.cmd_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/left_arm_controller/state', self.on_cmd, 10)

        self.log("=== Joint Monitor Started ===")
        self.log("Monitoring /joint_states and /left_arm_controller/state")
        self.log("Only prints when position changes > %.4f" % self.threshold)
        self.log("")

    def elapsed(self):
        return time.time() - self.start_time

    def log(self, msg):
        print("[%.3f] %s" % (self.elapsed(), msg), flush=True)

    def on_state(self, msg):
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else 0.0
            if name not in self.last_state or abs(pos - self.last_state[name]) > self.threshold:
                self.last_state[name] = pos
                self.log("STATE %s = %.4f" % (name, pos))

    def on_cmd(self, msg):
        # JTC state 包含 reference (命令) 和 feedback (实际)
        names = list(msg.joint_names)
        refs = list(msg.reference.positions) if msg.reference.positions else []
        actuals = list(msg.feedback.positions) if msg.feedback.positions else []
        errors = list(msg.error.positions) if msg.error.positions else []

        for i, name in enumerate(names):
            ref = refs[i] if i < len(refs) else 0.0
            actual = actuals[i] if i < len(actuals) else 0.0
            err = errors[i] if i < len(errors) else 0.0

            # 命令变化时输出
            if name not in self.last_cmd or abs(ref - self.last_cmd[name]) > self.threshold:
                self.last_cmd[name] = ref
                self.log("CMD %s ref=%.4f actual=%.4f err=%.4f" % (name, ref, actual, err))


def main():
    rclpy.init()
    node = JointMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.log("=== Monitor Stopped ===")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
