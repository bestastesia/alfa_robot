from __future__ import annotations

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


DEFAULT_JOINT_NAMES = [
    "updown",
    "turn",
    "pitch",
    "leftjoint1",
    "leftjoint2",
    "leftjoint3",
    "leftjoint4",
    "leftjoint5",
    "leftjoint6",
    "rightjoint1",
    "rightjoint2",
    "rightjoint3",
    "rightjoint4",
    "rightjoint5",
    "rightjoint6",
]


@dataclass
class ActiveSegment:
    joint_names: list[str]
    start_positions: list[float]
    target_positions: list[float]
    start_time: float
    duration_s: float


class KinematicSimExecutorNode(Node):
    """Kinematic simulated execution backend for manual full-flow tests."""

    def __init__(self) -> None:
        super().__init__("kinematic_sim_executor")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("action_name", "/alfa_execution/execute_joint_trajectory")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("initial_updown", 0.0)

        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.action_name = str(self.get_parameter("action_name").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.initial_updown = float(self.get_parameter("initial_updown").value)

        self.joint_names = list(DEFAULT_JOINT_NAMES)
        self.positions = {name: 0.0 for name in self.joint_names}
        self.positions["updown"] = self.initial_updown
        self.lock = threading.RLock()
        self.active_segment: ActiveSegment | None = None
        self.cancel_requested = False

        self.callback_group = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(JointState, self.joint_state_topic, 10)
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.action_name,
            execute_callback=self.execute_goal,
            goal_callback=self.accept_goal,
            cancel_callback=self.cancel_goal,
            callback_group=self.callback_group,
        )
        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 1.0),
            self.on_publish_timer,
            callback_group=self.callback_group,
        )
        self.get_logger().info(
            "Kinematic sim executor ready: "
            f"joint_states={self.joint_state_topic}, action={self.action_name}, "
            f"rate={self.publish_rate_hz:.1f}Hz, initial_updown={self.initial_updown:.3f}"
        )

    def accept_goal(self, goal_request):
        trajectory = goal_request.trajectory
        unknown = [name for name in trajectory.joint_names if name not in self.positions]
        if unknown:
            self.get_logger().error(f"Rejecting trajectory with unknown joints: {unknown}")
            return GoalResponse.REJECT
        if not trajectory.points:
            self.get_logger().error("Rejecting empty trajectory")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_goal(self, goal_handle):
        with self.lock:
            self.cancel_requested = True
            self.active_segment = None
        return CancelResponse.ACCEPT

    @staticmethod
    def point_time_s(point) -> float:
        return float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9

    def current_positions_for(self, joint_names: list[str]) -> list[float]:
        return [float(self.positions.get(name, 0.0)) for name in joint_names]

    def set_positions_for(self, joint_names: list[str], positions: list[float]) -> None:
        for name, value in zip(joint_names, positions):
            self.positions[name] = float(value)

    @staticmethod
    def interpolate(start: list[float], target: list[float], alpha: float) -> list[float]:
        alpha = min(1.0, max(0.0, alpha))
        return [a + (b - a) * alpha for a, b in zip(start, target)]

    @staticmethod
    def max_abs_delta(lhs: list[float], rhs: list[float]) -> float:
        if len(lhs) != len(rhs):
            return float("inf")
        return max((abs(float(a) - float(b)) for a, b in zip(lhs, rhs)), default=0.0)

    def publish_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = [float(self.positions[name]) for name in self.joint_names]
        msg.velocity = [0.0] * len(msg.name)
        self.publisher.publish(msg)

    def on_publish_timer(self) -> None:
        now = time.monotonic()
        with self.lock:
            if self.active_segment is not None:
                segment = self.active_segment
                elapsed = now - segment.start_time
                alpha = 1.0 if segment.duration_s <= 1e-6 else elapsed / segment.duration_s
                self.set_positions_for(
                    segment.joint_names,
                    self.interpolate(segment.start_positions, segment.target_positions, alpha),
                )
                if alpha >= 1.0:
                    self.active_segment = None
            self.publish_joint_state()

    def execute_goal(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        joint_names = list(trajectory.joint_names)
        points = list(trajectory.points)
        current_at_accept = self.current_positions_for(joint_names)
        first_positions = list(points[0].positions)
        last_positions = list(points[-1].positions)
        initial_delta = self.max_abs_delta(current_at_accept, first_positions)
        self.get_logger().info(
            f"Executing simulated trajectory: joints={len(joint_names)} "
            f"points={len(points)} duration={self.point_time_s(points[-1]):.3f}s "
            f"initial_delta={initial_delta:.6f} "
            f"first_updown={first_positions[joint_names.index('updown')] if 'updown' in joint_names else 0.0:.3f} "
            f"last_updown={last_positions[joint_names.index('updown')] if 'updown' in joint_names else 0.0:.3f}"
        )
        with self.lock:
            self.cancel_requested = False

        previous_time = 0.0
        previous_positions = self.current_positions_for(joint_names)
        for point_index, point in enumerate(points):
            if goal_handle.is_cancel_requested:
                with self.lock:
                    self.cancel_requested = True
                    self.active_segment = None
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            target_positions = list(point.positions)
            if len(target_positions) != len(joint_names):
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = (
                    f"point {point_index} has {len(target_positions)} positions "
                    f"for {len(joint_names)} joints"
                )
                return result

            point_time = self.point_time_s(point)
            if point_index == 0 and point_time <= 1e-9:
                if self.max_abs_delta(previous_positions, target_positions) > 1e-6:
                    self.get_logger().warning(
                        "trajectory first point differs from current state; "
                        "skipping zero-time point to avoid simulated jump"
                    )
                    previous_time = 0.0
                    previous_positions = self.current_positions_for(joint_names)
                    continue
            segment_duration = max(0.0, point_time - previous_time)
            with self.lock:
                self.active_segment = ActiveSegment(
                    joint_names=joint_names,
                    start_positions=list(previous_positions),
                    target_positions=list(target_positions),
                    start_time=time.monotonic(),
                    duration_s=segment_duration,
                )

            deadline = time.monotonic() + segment_duration
            while time.monotonic() < deadline and rclpy.ok():
                with self.lock:
                    if self.cancel_requested:
                        goal_handle.canceled()
                        result = FollowJointTrajectory.Result()
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        return result
                time.sleep(0.002)

            with self.lock:
                self.set_positions_for(joint_names, target_positions)
                self.active_segment = None
            previous_time = point_time
            previous_positions = list(target_positions)

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
    executor = MultiThreadedExecutor(num_threads=3)
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
