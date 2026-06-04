#!/usr/bin/python3
"""MuJoCo-backed digital twin controller for MoveIt.

This node makes MuJoCo the simulated robot state authority:
- exposes FollowJointTrajectory action servers matching MoveIt controllers;
- applies trajectory targets to MuJoCo position actuators;
- publishes /joint_states from MuJoCo qpos;
- publishes compact semantic scene obstacles in the same base_link frame.
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import mujoco.viewer
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from alfa_robot_moveit_config.msg import SemanticCargo, SemanticScene
from mujoco_sync_bridge import (
    DEFAULT_FRAME_ID,
    JOINT_NAMES,
    MujocoJointStateViewer,
    find_default_initial_positions,
    find_default_scene,
)


DEFAULT_DUAL_CONTROLLER = "/dual_v5_arm_controller/follow_joint_trajectory"
DEFAULT_TORSO_CONTROLLER = "/torso_controller/follow_joint_trajectory"


@dataclass
class ActiveTrajectory:
    joints: list[str]
    points: list
    start_time: float
    duration: float

    def sample(self, now: float) -> tuple[dict[str, float], bool]:
        if not self.points:
            return {}, True
        elapsed = max(0.0, now - self.start_time)
        if elapsed >= self.duration:
            return dict(zip(self.joints, self.points[-1].positions)), True

        previous_time = 0.0
        previous_positions = list(self.points[0].positions)
        for point in self.points:
            point_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            if elapsed <= point_time:
                if point_time <= previous_time:
                    return dict(zip(self.joints, point.positions)), False
                alpha = (elapsed - previous_time) / (point_time - previous_time)
                positions = [
                    a + (b - a) * alpha
                    for a, b in zip(previous_positions, point.positions)
                ]
                return dict(zip(self.joints, positions)), False
            previous_time = point_time
            previous_positions = list(point.positions)
        return dict(zip(self.joints, self.points[-1].positions)), True


class MujocoDigitalTwin(Node):
    def __init__(self, args):
        super().__init__("mujoco_digital_twin")
        self.args = args
        self.viewer_sync = MujocoJointStateViewer(
            Path(args.xml),
            args.joint_states,
            Path(args.initial_positions),
            args.robot_mode,
        )
        self.joint_state_pub = self.create_publisher(JointState, args.joint_states, 10)
        self.semantic_scene_pub = self.create_publisher(SemanticScene, args.semantic_scene_topic, 10) if args.semantic_rate > 0.0 else None
        self.active_trajectories: dict[str, ActiveTrajectory] = {}
        self.trajectory_lock = threading.RLock()
        self.goal_results: dict[str, object] = {}

        self.dual_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            args.dual_action,
            execute_callback=lambda goal_handle: self.execute_trajectory("dual", goal_handle),
            goal_callback=self.accept_goal,
            cancel_callback=self.cancel_goal,
        )
        self.torso_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            args.torso_action,
            execute_callback=lambda goal_handle: self.execute_trajectory("torso", goal_handle),
            goal_callback=self.accept_goal,
            cancel_callback=self.cancel_goal,
        )
        self.get_logger().info(
            "MuJoCo digital twin ready: "
            f"xml={args.xml}, joint_states={args.joint_states}, "
            f"dual_action={args.dual_action}, torso_action={args.torso_action}, "
            f"joints={len(self.viewer_sync.qpos_adrs)}"
        )

    def accept_goal(self, goal_request):
        names = list(goal_request.trajectory.joint_names)
        unknown = [name for name in names if name not in self.viewer_sync.qpos_adrs]
        if unknown:
            self.get_logger().error(f"Rejecting trajectory with unknown joints: {unknown}")
            return GoalResponse.REJECT
        if not goal_request.trajectory.points:
            self.get_logger().error("Rejecting empty trajectory")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_goal(self, goal_handle):
        goal_id = self.goal_key(goal_handle)
        with self.trajectory_lock:
            self.active_trajectories.pop(goal_id, None)
        return CancelResponse.ACCEPT

    @staticmethod
    def goal_key(goal_handle) -> str:
        return bytes(goal_handle.goal_id.uuid).hex()

    def execute_trajectory(self, namespace: str, goal_handle):
        trajectory = goal_handle.request.trajectory
        points = list(trajectory.points)
        duration = max(
            point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            for point in points
        )
        if duration <= 0.0:
            duration = 0.01
        goal_id = self.goal_key(goal_handle)
        with self.trajectory_lock:
            self.active_trajectories[goal_id] = ActiveTrajectory(
                joints=list(trajectory.joint_names),
                points=points,
                start_time=time.monotonic(),
                duration=duration,
            )
        self.get_logger().info(
            f"Accepted {namespace} trajectory: joints={list(trajectory.joint_names)}, "
            f"points={len(points)}, duration={duration:.3f}s"
        )
        while rclpy.ok():
            with self.trajectory_lock:
                still_active = goal_id in self.active_trajectories
            if not still_active:
                break
            if goal_handle.is_cancel_requested:
                with self.trajectory_lock:
                    self.active_trajectories.pop(goal_id, None)
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result
            with self.trajectory_lock:
                active = self.active_trajectories.get(goal_id)
                elapsed = time.monotonic() - active.start_time if active is not None else duration
            if elapsed >= duration:
                break
            time.sleep(0.01)
        with self.trajectory_lock:
            self.active_trajectories.pop(goal_id, None)
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed()
        return result

    def apply_active_trajectories(self):
        now = time.monotonic()
        completed = []
        with self.trajectory_lock:
            active_items = list(self.active_trajectories.items())
        for goal_id, trajectory in active_items:
            targets, done = trajectory.sample(now)
            for name, value in targets.items():
                self.viewer_sync.joint_targets[name] = float(value)
            if done:
                completed.append(goal_id)
        if completed:
            with self.trajectory_lock:
                for goal_id in completed:
                    self.active_trajectories.pop(goal_id, None)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        names = [name for name in JOINT_NAMES if name in self.viewer_sync.qpos_adrs and not name.startswith("base_")]
        msg.name = names
        msg.position = [float(self.viewer_sync.data.qpos[self.viewer_sync.qpos_adrs[name]]) for name in names]
        msg.velocity = [0.0 for _ in names]
        self.joint_state_pub.publish(msg)

    def publish_semantic_scene(self):
        if self.semantic_scene_pub is None:
            return
        self.semantic_scene_pub.publish(
            self.viewer_sync.make_semantic_scene(self.args.frame_id, self, SemanticScene, SemanticCargo)
        )


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="MuJoCo digital twin controlled by MoveIt trajectories")
    parser.add_argument("--xml", default=str(find_default_scene()))
    parser.add_argument("--initial-positions", default=str(find_default_initial_positions()))
    parser.add_argument("--joint-states", default="/joint_states")
    parser.add_argument("--robot-mode", choices=["kinematic", "actuator"], default="actuator")
    parser.add_argument("--dual-action", default=DEFAULT_DUAL_CONTROLLER)
    parser.add_argument("--torso-action", default=DEFAULT_TORSO_CONTROLLER)
    parser.add_argument("--semantic-scene-topic", default="/mujoco_semantic_scene")
    parser.add_argument("--semantic-rate", type=float, default=1.0)
    parser.add_argument("--joint-state-rate", type=float, default=50.0)
    parser.add_argument("--viewer-rate", type=float, default=30.0)
    parser.add_argument("--physics-rate", type=float, default=200.0)
    parser.add_argument("--frame-id", default=DEFAULT_FRAME_ID)
    parser.add_argument("--camera-distance", type=float, default=4.0)
    parser.add_argument("--camera-elevation", type=float, default=-18.0)
    parser.add_argument("--camera-azimuth", type=float, default=135.0)
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    rclpy.init()
    node = MujocoDigitalTwin(args)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    physics_period = 1.0 / max(args.physics_rate, 1.0)
    viewer_period = 1.0 / max(args.viewer_rate, 1.0)
    joint_state_period = 1.0 / max(args.joint_state_rate, 1.0)
    semantic_period = 1.0 / args.semantic_rate if args.semantic_rate > 0.0 else 0.0
    node.viewer_sync.model.opt.timestep = physics_period
    next_physics_tick = time.monotonic()
    next_viewer_tick = next_physics_tick
    next_joint_state_tick = next_physics_tick
    next_semantic_tick = next_physics_tick

    try:
        with mujoco.viewer.launch_passive(node.viewer_sync.model, node.viewer_sync.data) as viewer:
            viewer.cam.distance = args.camera_distance
            viewer.cam.elevation = args.camera_elevation
            viewer.cam.azimuth = args.camera_azimuth
            viewer.cam.lookat[:] = [1.4, 0.0, 0.9]
            while viewer.is_running() and rclpy.ok():
                now = time.monotonic()
                while next_physics_tick <= now:
                    node.apply_active_trajectories()
                    node.viewer_sync.apply_targets()
                    mujoco.mj_step(node.viewer_sync.model, node.viewer_sync.data)
                    next_physics_tick += physics_period
                if now >= next_joint_state_tick:
                    node.publish_joint_states()
                    next_joint_state_tick += joint_state_period
                if node.semantic_scene_pub is not None and now >= next_semantic_tick:
                    node.publish_semantic_scene()
                    next_semantic_tick += semantic_period
                if now >= next_viewer_tick:
                    viewer.sync()
                    next_viewer_tick += viewer_period
                sleep_time = min(next_physics_tick, next_viewer_tick, next_joint_state_tick) - time.monotonic()
                if sleep_time > 0.0:
                    time.sleep(min(sleep_time, 0.01))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        executor_thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
