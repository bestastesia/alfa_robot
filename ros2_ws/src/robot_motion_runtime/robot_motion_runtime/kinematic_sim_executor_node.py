from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

from alfa_robot_execution_bridge.trajectory_interpolation import (
    InterpolatedState,
    TrajectorySample,
    sample_trajectory,
)
from alfa_robot_execution_bridge.updown import validate_updown_command_data
from robot_motion_runtime.common import (
    DEFAULT_MOTION_JOINTS,
    MODEL_TO_HARDWARE_JOINT_ALIASES,
    canonical_joint_name,
)


@dataclass
class ActiveTrajectory:
    token: int
    joint_names: list[str]
    requested_joint_names: list[str]
    samples: list[TrajectorySample]
    start_time: float


def duration_s(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


class KinematicSimExecutorNode(Node):
    """Kinematic digital-twin execution backend.

    The node intentionally exposes the same high-level ROS contract as the
    current real hardware stack for arms/turn/updown:

    - FollowJointTrajectory action:
      /dual_arm_trajectory_controller/follow_joint_trajectory
    - JointTrajectory topic:
      /dual_arm_trajectory_controller/joint_trajectory
    - Updown absolute-position topic:
      /canopen/updown_position_controller/commands

    Internally it keeps the repository model joint names (left_joint1...) so
    robot_state_publisher and Rerun remain compatible, while also publishing
    hardware aliases (left_joint1...) on /joint_states for client-side tests.
    """

    def __init__(self) -> None:
        super().__init__("kinematic_sim_executor")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "action_name",
            "/dual_arm_trajectory_controller/follow_joint_trajectory",
        )
        self.declare_parameter("legacy_action_name", "/alfa_execution/execute_joint_trajectory")
        self.declare_parameter(
            "trajectory_topic",
            "/dual_arm_trajectory_controller/joint_trajectory",
        )
        self.declare_parameter(
            "updown_command_topic",
            "/canopen/updown_position_controller/commands",
        )
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("control_rate_hz", 250.0)
        self.declare_parameter("publish_alias_joint_states", True)
        self.declare_parameter("initial_updown", 0.0)

        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.action_name = str(self.get_parameter("action_name").value)
        self.legacy_action_name = str(self.get_parameter("legacy_action_name").value)
        self.trajectory_topic = str(self.get_parameter("trajectory_topic").value)
        self.updown_command_topic = str(self.get_parameter("updown_command_topic").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.publish_alias_joint_states = bool(
            self.get_parameter("publish_alias_joint_states").value
        )
        self.initial_updown = float(self.get_parameter("initial_updown").value)

        self.joint_names = list(DEFAULT_MOTION_JOINTS)
        self.positions = {name: 0.0 for name in self.joint_names}
        self.positions["updown"] = self.initial_updown
        self.velocities = {name: 0.0 for name in self.joint_names}
        self.lock = threading.RLock()
        self.active_trajectory: ActiveTrajectory | None = None
        self.active_token = 0
        self.cancel_requested_tokens: set[int] = set()

        self.callback_group = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(JointState, self.joint_state_topic, 10)
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            self.trajectory_topic,
            self.on_trajectory_topic,
            10,
            callback_group=self.callback_group,
        )
        self.updown_sub = self.create_subscription(
            Float64MultiArray,
            self.updown_command_topic,
            self.on_updown_command,
            10,
            callback_group=self.callback_group,
        )
        self.action_servers: list[ActionServer] = []
        for name in self.unique_action_names():
            self.action_servers.append(
                ActionServer(
                    self,
                    FollowJointTrajectory,
                    name,
                    execute_callback=self.execute_goal,
                    goal_callback=self.accept_goal,
                    cancel_callback=self.cancel_goal,
                    callback_group=self.callback_group,
                )
            )
        self.control_timer = self.create_timer(
            1.0 / max(self.control_rate_hz, 1.0),
            self.on_control_timer,
            callback_group=self.callback_group,
        )
        self.publish_timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 1.0),
            self.on_publish_timer,
            callback_group=self.callback_group,
        )
        self.get_logger().info(
            "Kinematic sim executor ready: "
            f"joint_states={self.joint_state_topic}, "
            f"actions={self.unique_action_names()}, "
            f"trajectory_topic={self.trajectory_topic}, "
            f"updown_topic={self.updown_command_topic}, "
            f"control={self.control_rate_hz:.1f}Hz, "
            f"joint_state={self.publish_rate_hz:.1f}Hz, "
            f"initial_updown={self.initial_updown:.3f}"
        )

    def unique_action_names(self) -> list[str]:
        names: list[str] = []
        for name in [self.action_name, self.legacy_action_name]:
            if name and name not in names:
                names.append(name)
        return names

    def accept_goal(self, goal_request):
        try:
            self.normalize_trajectory(goal_request.trajectory)
        except ValueError as exc:
            self.get_logger().error(f"Rejecting trajectory: {exc}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_goal(self, goal_handle):
        with self.lock:
            if self.active_trajectory is not None:
                self.cancel_requested_tokens.add(self.active_trajectory.token)
                self.active_trajectory = None
        return CancelResponse.ACCEPT

    def current_positions_for(self, joint_names: list[str]) -> list[float]:
        return [float(self.positions.get(name, 0.0)) for name in joint_names]

    def set_positions_for(self, joint_names: list[str], positions: list[float]) -> None:
        for name, value in zip(joint_names, positions):
            self.positions[name] = float(value)

    @staticmethod
    def max_abs_delta(lhs: list[float], rhs: list[float]) -> float:
        if len(lhs) != len(rhs):
            return float("inf")
        return max((abs(float(a) - float(b)) for a, b in zip(lhs, rhs)), default=0.0)

    def normalize_trajectory(self, trajectory: JointTrajectory) -> tuple[list[str], list[str], list[TrajectorySample]]:
        if not trajectory.joint_names:
            raise ValueError("trajectory.joint_names is empty")
        if not trajectory.points:
            raise ValueError("trajectory.points is empty")

        requested_names = [str(name) for name in trajectory.joint_names]
        joint_names = [canonical_joint_name(name) for name in requested_names]
        unknown = [name for name in joint_names if name not in self.positions]
        if unknown:
            raise ValueError(f"unknown joints after alias mapping: {unknown}")
        duplicates = sorted({name for name in joint_names if joint_names.count(name) > 1})
        if duplicates:
            raise ValueError(f"duplicate joints after alias mapping: {duplicates}")

        samples: list[TrajectorySample] = []
        previous_time = -1e-9
        for index, point in enumerate(trajectory.points):
            if len(point.positions) != len(joint_names):
                raise ValueError(
                    f"point {index} has {len(point.positions)} positions for "
                    f"{len(joint_names)} joints"
                )
            if point.velocities and len(point.velocities) != len(joint_names):
                raise ValueError(
                    f"point {index} has {len(point.velocities)} velocities for "
                    f"{len(joint_names)} joints"
                )
            if point.accelerations and len(point.accelerations) != len(joint_names):
                raise ValueError(
                    f"point {index} has {len(point.accelerations)} accelerations for "
                    f"{len(joint_names)} joints"
                )
            point_time = duration_s(point.time_from_start)
            if point_time <= previous_time:
                raise ValueError(
                    f"point {index} time_from_start={point_time:.9f}s is not strictly increasing"
                )
            previous_time = point_time
            samples.append(
                TrajectorySample(
                    time_from_start=point_time,
                    positions=[float(value) for value in point.positions],
                    velocities=(
                        [float(value) for value in point.velocities]
                        if point.velocities
                        else None
                    ),
                    accelerations=(
                        [float(value) for value in point.accelerations]
                        if point.accelerations
                        else None
                    ),
                )
            )

        if samples[0].time_from_start > 1e-9:
            samples.insert(
                0,
                TrajectorySample(
                    time_from_start=0.0,
                    positions=self.current_positions_for(joint_names),
                    velocities=(
                        [0.0] * len(joint_names)
                        if samples[0].velocities is not None
                        else None
                    ),
                    accelerations=(
                        [0.0] * len(joint_names)
                        if samples[0].accelerations is not None
                        else None
                    ),
                ),
            )
        return joint_names, requested_names, samples

    def start_trajectory(self, trajectory: JointTrajectory, source: str) -> tuple[int, float, float]:
        joint_names, requested_names, samples = self.normalize_trajectory(trajectory)
        first_positions = samples[0].positions
        last_positions = samples[-1].positions
        current_positions = self.current_positions_for(joint_names)
        initial_delta = self.max_abs_delta(current_positions, first_positions)
        if initial_delta > 1e-3:
            self.get_logger().warning(
                f"{source} first point differs from simulated state by {initial_delta:.6f} rad/m; "
                "real hardware should normally start from fresh /joint_states"
            )

        with self.lock:
            self.active_token += 1
            token = self.active_token
            self.cancel_requested_tokens.discard(token)
            self.active_trajectory = ActiveTrajectory(
                token=token,
                joint_names=joint_names,
                requested_joint_names=requested_names,
                samples=samples,
                start_time=time.monotonic(),
            )
        self.get_logger().info(
            f"Started simulated trajectory from {source}: "
            f"joints={len(joint_names)} points={len(samples)} "
            f"duration={samples[-1].time_from_start:.3f}s initial_delta={initial_delta:.6f} "
            f"first={self.brief_positions(requested_names, first_positions)} "
            f"last={self.brief_positions(requested_names, last_positions)}"
        )
        return token, samples[-1].time_from_start, initial_delta

    @staticmethod
    def brief_positions(joint_names: list[str], positions: list[float]) -> str:
        pairs = list(zip(joint_names, positions))
        if len(pairs) > 4:
            pairs = pairs[:3] + pairs[-1:]
        return "{" + ", ".join(f"{name}={value:.3f}" for name, value in pairs) + "}"

    @staticmethod
    def sample_state(samples: list[TrajectorySample], elapsed_s: float) -> InterpolatedState:
        return sample_trajectory(samples, elapsed_s)

    @staticmethod
    def sample_positions(samples: list[TrajectorySample], elapsed_s: float) -> list[float]:
        return KinematicSimExecutorNode.sample_state(samples, elapsed_s).positions

    def on_control_timer(self) -> None:
        now = time.monotonic()
        with self.lock:
            active = self.active_trajectory
            if active is None:
                return
            elapsed = now - active.start_time
            state = self.sample_state(active.samples, elapsed)
            self.set_positions_for(active.joint_names, state.positions)
            for name, velocity in zip(active.joint_names, state.velocities):
                self.velocities[name] = float(velocity)
            if elapsed >= active.samples[-1].time_from_start:
                self.set_positions_for(active.joint_names, active.samples[-1].positions)
                self.active_trajectory = None

    def publish_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        names = list(self.joint_names)
        if self.publish_alias_joint_states:
            names.extend(
                real for model, real in MODEL_TO_HARDWARE_JOINT_ALIASES.items() if model in self.positions
            )
        msg.name = names
        msg.position = [
            float(self.positions[canonical_joint_name(name)])
            if canonical_joint_name(name) in self.positions
            else 0.0
            for name in names
        ]
        msg.velocity = [
            float(self.velocities.get(canonical_joint_name(name), 0.0))
            for name in names
        ]
        self.publisher.publish(msg)

    def on_publish_timer(self) -> None:
        with self.lock:
            self.publish_joint_state()

    def on_trajectory_topic(self, msg: JointTrajectory) -> None:
        try:
            self.start_trajectory(msg, self.trajectory_topic)
        except ValueError as exc:
            self.get_logger().error(f"Rejecting trajectory topic message: {exc}")

    def on_updown_command(self, msg: Float64MultiArray) -> None:
        try:
            target, velocity, acceleration, deceleration = validate_updown_command_data(msg.data)
        except ValueError as exc:
            self.get_logger().error(f"Rejecting updown command: {exc}")
            return
        with self.lock:
            self.positions["updown"] = target
        self.get_logger().info(
            "Simulated updown profile target accepted: "
            f"position={target:.4f}m velocity={velocity:.4f}m/s "
            f"acceleration={acceleration:.4f}m/s^2 deceleration={deceleration:.4f}m/s^2"
        )

    def execute_goal(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        try:
            token, duration, _ = self.start_trajectory(trajectory, "FollowJointTrajectory action")
        except ValueError as exc:
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            return result

        deadline = time.monotonic() + duration
        feedback_period_s = 0.05
        next_feedback = time.monotonic()
        while rclpy.ok():
            now = time.monotonic()
            if goal_handle.is_cancel_requested:
                with self.lock:
                    self.cancel_requested_tokens.add(token)
                    if self.active_trajectory is not None and self.active_trajectory.token == token:
                        self.active_trajectory = None
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "simulated trajectory canceled"
                return result

            with self.lock:
                canceled = token in self.cancel_requested_tokens
                active = self.active_trajectory
                finished = active is None or active.token != token
                current_joint_names = list(trajectory.joint_names)
                current_positions = [
                    self.positions.get(canonical_joint_name(name), 0.0)
                    for name in current_joint_names
                ]
            if canceled:
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "simulated trajectory canceled"
                return result
            if finished and now >= deadline:
                break
            if now >= next_feedback:
                feedback = FollowJointTrajectory.Feedback()
                feedback.joint_names = current_joint_names
                feedback.actual.positions = [float(value) for value in current_positions]
                goal_handle.publish_feedback(feedback)
                next_feedback = now + feedback_period_s
            time.sleep(0.002)

        with self.lock:
            self.publish_joint_state()
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "simulated trajectory reached"
        return result


def main() -> None:
    rclpy.init()
    node = KinematicSimExecutorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
