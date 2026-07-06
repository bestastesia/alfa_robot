#!/usr/bin/python3
"""Small joint-motion commander for the RViz + MuJoCo sync demo.

Run after `demo.launch.py` and `mujoco_sync_view.launch.py` are up. It sends a
few safe FollowJointTrajectory goals to the current ros2_control controllers, so
RViz and the MuJoCo viewer can be checked moving together.
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


TORSO_JOINTS = ["pitch", "turn"]
ARM_JOINTS = [
    "updown",
    "leftjoint1", "leftjoint2", "leftjoint3",
    "leftjoint4", "leftjoint5", "leftjoint6",
    "rightjoint1", "rightjoint2", "rightjoint3",
    "rightjoint4", "rightjoint5", "rightjoint6",
]
ALL_JOINTS = TORSO_JOINTS + ARM_JOINTS


class JointDemoCommander(Node):
    def __init__(self, duration: float):
        super().__init__("mujoco_joint_demo_commander")
        self.duration = duration
        self.current_js: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.controllers = {
            "torso_controller": (
                ActionClient(self, FollowJointTrajectory, "/torso_controller/follow_joint_trajectory"),
                TORSO_JOINTS,
            ),
            "dual_arm_controller": (
                ActionClient(self, FollowJointTrajectory, "/dual_arm_controller/follow_joint_trajectory"),
                ARM_JOINTS,
            ),
        }

    def _on_joint_state(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            self.current_js[name] = float(position)

    def wait_ready(self) -> bool:
        deadline = time.time() + 8.0
        while time.time() < deadline and len(self.current_js) < 8:
            rclpy.spin_once(self, timeout_sec=0.1)

        ok = True
        for controller_name, (client, joints) in self.controllers.items():
            if client.wait_for_server(timeout_sec=5.0):
                self.get_logger().info(f"{controller_name} ready: {len(joints)} joints")
            else:
                self.get_logger().error(f"{controller_name} action unavailable")
                ok = False
        return ok

    def _send_controller_goal(self, controller_name: str, targets: dict[str, float]) -> bool:
        client, controller_joints = self.controllers[controller_name]
        if not any(joint in targets for joint in controller_joints):
            return True
        joint_names = list(controller_joints)

        start = JointTrajectoryPoint()
        start.positions = [self.current_js.get(joint, 0.0) for joint in joint_names]
        start.time_from_start.sec = 0

        end = JointTrajectoryPoint()
        end.positions = [float(targets.get(joint, self.current_js.get(joint, 0.0))) for joint in joint_names]
        end.time_from_start.sec = int(self.duration)
        end.time_from_start.nanosec = int((self.duration - int(self.duration)) * 1e9)

        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        trajectory.points = [start, end]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(
            f"send {controller_name}: "
            + ", ".join(f"{j}={targets[j]:.3f}" for j in joint_names if j in targets)
        )
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        goal_handle = send_future.result() if send_future.done() else None
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{controller_name} rejected goal")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.duration + 5.0)
        result = result_future.result() if result_future.done() else None
        if result is None or result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            code = result.result.error_code if result else "timeout"
            self.get_logger().error(f"{controller_name} failed: {code}")
            return False
        return True

    def send_targets(self, targets: dict[str, float]) -> bool:
        ok = True
        for controller_name in self.controllers:
            ok = self._send_controller_goal(controller_name, targets) and ok
        return ok

    def run_sequence(self, loops: int):
        poses = [
            {
                "pitch": 0.04,
                "turn": 0.15,
                "updown": 0.12,
                "leftjoint1": 0.20,
                "rightjoint1": -0.20,
                "leftjoint2": math.pi / 2 - 0.15,
                "rightjoint2": math.pi / 2 - 0.15,
            },
            {
                "pitch": 0.02,
                "turn": -0.15,
                "updown": 0.04,
                "leftjoint1": -0.20,
                "rightjoint1": 0.20,
                "leftjoint2": math.pi / 2 + 0.10,
                "rightjoint2": math.pi / 2 + 0.10,
            },
            {joint: 0.0 for joint in [
                "pitch", "turn", "updown", "leftjoint1", "rightjoint1",
            ]},
        ]
        for loop_index in range(loops):
            self.get_logger().info(f"demo loop {loop_index + 1}/{loops}")
            for pose in poses:
                if not self.send_targets(pose):
                    return False
                for _ in range(5):
                    rclpy.spin_once(self, timeout_sec=0.1)
        return True


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Send a small joint motion demo to current controllers")
    parser.add_argument("--loops", type=int, default=1)
    parser.add_argument("--duration", type=float, default=2.0)
    args, _ros_args = parser.parse_known_args(argv)
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    rclpy.init()
    node = JointDemoCommander(duration=args.duration)
    try:
        if not node.wait_ready():
            return 2
        return 0 if node.run_sequence(args.loops) else 1
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
