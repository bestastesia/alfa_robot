from __future__ import annotations

import contextlib
import copy
import math
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from alfa_robot_execution_bridge.joints import DEFAULT_JOINT_NAMES, direction_signs_for
from alfa_robot_execution_bridge.trajectory_interpolation import (
    TrajectorySample,
    sample_trajectory,
)


def duration_to_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def make_duration(seconds: float):
    msg = JointTrajectoryPoint().time_from_start
    whole = math.floor(max(0.0, seconds))
    msg.sec = int(whole)
    msg.nanosec = int(round((max(0.0, seconds) - whole) * 1e9))
    if msg.nanosec >= 1_000_000_000:
        msg.sec += 1
        msg.nanosec -= 1_000_000_000
    return msg


def make_result(error_code: int, error_string: str = ''):
    result = FollowJointTrajectory.Result()
    result.error_code = int(error_code)
    result.error_string = error_string
    return result


class ExecutionBridgeNode(Node):
    """Unified ALFA trajectory execution bridge.

    `mock` mode simulates execution and publishes joint states.
    `ros2_control` mode forwards goals to an existing FollowJointTrajectory controller.
    """

    def __init__(self) -> None:
        super().__init__('alfa_execution_bridge')

        self.declare_parameter('mode', 'mock')
        self.declare_parameter('action_name', '/alfa_execution/execute_joint_trajectory')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('publish_joint_states', True)
        self.declare_parameter('update_hz', 250.0)
        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)
        self.declare_parameter('direction_signs', [])
        self.declare_parameter('apply_direction_signs', False)
        self.declare_parameter('initial_positions', [0.0] * len(DEFAULT_JOINT_NAMES))
        self.declare_parameter('downstream_action_name', '/dual_arm_trajectory_controller/follow_joint_trajectory')
        self.declare_parameter('downstream_wait_timeout_s', 5.0)

        self.mode = str(self.get_parameter('mode').value)
        self.action_name = str(self.get_parameter('action_name').value)
        self.joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        self.publish_joint_states = bool(self.get_parameter('publish_joint_states').value)
        self.update_hz = float(self.get_parameter('update_hz').value)
        self.joint_names = [str(name) for name in self.get_parameter('joint_names').value]
        configured_direction_signs = [float(value) for value in self.get_parameter('direction_signs').value]
        self.direction_signs = configured_direction_signs or direction_signs_for(self.joint_names)
        self.apply_direction_signs = bool(self.get_parameter('apply_direction_signs').value)
        self.initial_positions = [float(value) for value in self.get_parameter('initial_positions').value]
        self.downstream_action_name = str(self.get_parameter('downstream_action_name').value)
        self.downstream_wait_timeout_s = float(self.get_parameter('downstream_wait_timeout_s').value)

        self._validate_parameters()
        self._lock = threading.Lock()
        self._positions = list(self.initial_positions)
        self._velocities = [0.0] * len(self.joint_names)
        self._active_goal_id: str | None = None
        self._active_downstream_goal = None

        self._joint_state_pub = None
        if self.mode == 'mock':
            self._joint_state_pub = self.create_publisher(JointState, self.joint_state_topic, 10)
            if self.publish_joint_states:
                period = 1.0 / max(1.0, self.update_hz)
                self.create_timer(period, self._publish_joint_state)
        elif self.mode == 'ros2_control':
            self._downstream_client = ActionClient(
                self,
                FollowJointTrajectory,
                self.downstream_action_name,
            )
        else:
            raise RuntimeError(f'unsupported mode={self.mode!r}; expected mock or ros2_control')

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self.get_logger().info(
            f'ALFA execution bridge started: mode={self.mode}, action={self.action_name}, '
            f'joints={len(self.joint_names)}, apply_direction_signs={self.apply_direction_signs}'
        )

    def _validate_parameters(self) -> None:
        if len(self.joint_names) == 0:
            raise ValueError('joint_names must not be empty')
        if len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError('joint_names must be unique')
        if len(self.direction_signs) != len(self.joint_names):
            raise ValueError('direction_signs length must match joint_names length')
        if len(self.initial_positions) != len(self.joint_names):
            raise ValueError('initial_positions length must match joint_names length')
        if self.update_hz <= 0.0:
            raise ValueError('update_hz must be positive')
        if self.downstream_wait_timeout_s <= 0.0:
            raise ValueError('downstream_wait_timeout_s must be positive')

    def _publish_joint_state(self) -> None:
        if self._joint_state_pub is None:
            return
        with self._lock:
            positions = list(self._positions)
            velocities = list(self._velocities)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = positions
        msg.velocity = velocities
        msg.effort = [0.0] * len(self.joint_names)
        self._joint_state_pub.publish(msg)

    def _goal_callback(self, goal_request) -> GoalResponse:
        valid, reason = self._validate_goal(goal_request.trajectory)
        if not valid:
            self.get_logger().warn(f'reject trajectory goal: {reason}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        active_downstream_goal = self._active_downstream_goal
        if active_downstream_goal is not None:
            active_downstream_goal.cancel_goal_async()
        return CancelResponse.ACCEPT

    def _validate_goal(self, trajectory) -> tuple[bool, str]:
        if not trajectory.points:
            return False, 'trajectory.points is empty'
        if len(set(trajectory.joint_names)) != len(trajectory.joint_names):
            return False, 'trajectory.joint_names has duplicates'
        missing = [name for name in self.joint_names if name not in trajectory.joint_names]
        if missing:
            return False, f'missing required joints: {missing}'
        previous_time = -1.0
        joint_count = len(trajectory.joint_names)
        for index, point in enumerate(trajectory.points):
            if len(point.positions) != joint_count:
                return False, f'point {index} positions length does not match trajectory.joint_names'
            if point.velocities and len(point.velocities) != joint_count:
                return False, f'point {index} velocities length does not match trajectory.joint_names'
            if point.accelerations and len(point.accelerations) != joint_count:
                return False, f'point {index} accelerations length does not match trajectory.joint_names'
            if point.effort and len(point.effort) != joint_count:
                return False, f'point {index} effort length does not match trajectory.joint_names'
            point_time = duration_to_seconds(point.time_from_start)
            if point_time < previous_time:
                return False, f'point {index} time_from_start is not monotonic'
            previous_time = point_time
        return True, ''

    def _canonicalize_samples(self, trajectory) -> list[TrajectorySample]:
        name_to_index = {name: index for index, name in enumerate(trajectory.joint_names)}
        canonical_indices = [name_to_index[name] for name in self.joint_names]
        samples: list[TrajectorySample] = []
        for point in trajectory.points:
            positions = [float(point.positions[index]) for index in canonical_indices]
            if len(point.velocities) == len(trajectory.joint_names):
                velocities = [float(point.velocities[index]) for index in canonical_indices]
            else:
                velocities = None
            if len(point.accelerations) == len(trajectory.joint_names):
                accelerations = [
                    float(point.accelerations[index]) for index in canonical_indices
                ]
            else:
                accelerations = None
            samples.append(
                TrajectorySample(
                    time_from_start=duration_to_seconds(point.time_from_start),
                    positions=positions,
                    velocities=velocities,
                    accelerations=accelerations,
                )
            )
        return samples

    def _execute_callback(self, goal_handle):
        with self._lock:
            if self._active_goal_id is not None:
                goal_handle.abort()
                return make_result(
                    FollowJointTrajectory.Result.INVALID_GOAL,
                    'another trajectory is already executing',
                )
            self._active_goal_id = str(id(goal_handle))

        try:
            if self.mode == 'mock':
                return self._execute_mock(goal_handle)
            return self._execute_ros2_control(goal_handle)
        except Exception as exc:  # pragma: no cover - protects the action server boundary
            self.get_logger().exception(f'trajectory execution failed: {exc}')
            goal_handle.abort()
            return make_result(FollowJointTrajectory.Result.INVALID_GOAL, str(exc))
        finally:
            with self._lock:
                self._active_goal_id = None
                self._active_downstream_goal = None

    def _execute_mock(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        samples = self._canonicalize_samples(trajectory)
        with self._lock:
            current_positions = list(self._positions)

        if samples[0].time_from_start > 0.0:
            samples.insert(
                0,
                TrajectorySample(
                    time_from_start=0.0,
                    positions=current_positions,
                    velocities=(
                        [0.0] * len(self.joint_names)
                        if samples[0].velocities is not None
                        else None
                    ),
                    accelerations=(
                        [0.0] * len(self.joint_names)
                        if samples[0].accelerations is not None
                        else None
                    ),
                ),
            )

        final_time = samples[-1].time_from_start
        start = time.monotonic()
        next_sleep = 1.0 / max(1.0, self.update_hz)
        self.get_logger().info(
            f'execute mock trajectory with JTC-compatible spline: '
            f'points={len(samples)}, duration={final_time:.3f}s, control={self.update_hz:.1f}Hz'
        )

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                with self._lock:
                    self._velocities = [0.0] * len(self.joint_names)
                goal_handle.canceled()
                return make_result(FollowJointTrajectory.Result.SUCCESSFUL, 'canceled')

            elapsed = min(final_time, time.monotonic() - start)
            state = sample_trajectory(samples, elapsed)
            positions = state.positions
            velocities = state.velocities

            with self._lock:
                self._positions = positions
                self._velocities = velocities

            self._publish_feedback(goal_handle, elapsed, positions, velocities)

            if elapsed >= final_time:
                break
            time.sleep(next_sleep)

        with self._lock:
            self._positions = list(samples[-1].positions)
            self._velocities = [0.0] * len(self.joint_names)
        goal_handle.succeed()
        return make_result(FollowJointTrajectory.Result.SUCCESSFUL)

    def _execute_ros2_control(self, goal_handle):
        if not self._downstream_client.wait_for_server(timeout_sec=self.downstream_wait_timeout_s):
            goal_handle.abort()
            return make_result(
                FollowJointTrajectory.Result.INVALID_GOAL,
                f'downstream action server unavailable: {self.downstream_action_name}',
            )

        downstream_goal = copy.deepcopy(goal_handle.request)
        self._canonicalize_goal_request(downstream_goal)
        if self.apply_direction_signs:
            self._apply_direction_signs(downstream_goal.trajectory)

        self.get_logger().info(
            f'forward trajectory to {self.downstream_action_name}: '
            f'points={len(downstream_goal.trajectory.points)}, joints={len(downstream_goal.trajectory.joint_names)}'
        )
        send_future = self._downstream_client.send_goal_async(
            downstream_goal,
            feedback_callback=lambda feedback: self._forward_downstream_feedback(goal_handle, feedback),
        )
        self._wait_for_future(send_future)
        downstream_goal_handle = send_future.result()
        if not downstream_goal_handle or not downstream_goal_handle.accepted:
            goal_handle.abort()
            return make_result(FollowJointTrajectory.Result.INVALID_GOAL, 'downstream rejected trajectory goal')

        self._active_downstream_goal = downstream_goal_handle
        result_future = downstream_goal_handle.get_result_async()

        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                cancel_future = downstream_goal_handle.cancel_goal_async()
                self._wait_for_future(cancel_future, timeout_s=1.0)
                goal_handle.canceled()
                return make_result(FollowJointTrajectory.Result.SUCCESSFUL, 'canceled')
            self._wait_for_future(result_future, timeout_s=0.1)

        if not result_future.done():
            goal_handle.abort()
            return make_result(FollowJointTrajectory.Result.INVALID_GOAL, 'executor stopped before downstream result')

        wrapped_result = result_future.result()
        result = wrapped_result.result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _wait_for_future(self, future, timeout_s: float | None = None) -> bool:
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            if timeout_s is not None and time.monotonic() - start >= timeout_s:
                return False
            time.sleep(0.01)
        return future.done()

    def _canonicalize_goal_request(self, goal) -> None:
        trajectory = goal.trajectory
        if trajectory.joint_names == self.joint_names:
            return

        name_to_index = {name: index for index, name in enumerate(trajectory.joint_names)}
        indices = [name_to_index[name] for name in self.joint_names]
        trajectory.joint_names = list(self.joint_names)
        for point in trajectory.points:
            point.positions = [point.positions[index] for index in indices]
            if point.velocities:
                point.velocities = [point.velocities[index] for index in indices]
            if point.accelerations:
                point.accelerations = [point.accelerations[index] for index in indices]
            if point.effort:
                point.effort = [point.effort[index] for index in indices]

    def _apply_direction_signs(self, trajectory) -> None:
        name_to_sign = dict(zip(self.joint_names, self.direction_signs))
        signs = [name_to_sign[name] for name in trajectory.joint_names]
        for point in trajectory.points:
            point.positions = [value * sign for value, sign in zip(point.positions, signs)]
            if point.velocities:
                point.velocities = [value * sign for value, sign in zip(point.velocities, signs)]
            if point.accelerations:
                point.accelerations = [value * sign for value, sign in zip(point.accelerations, signs)]

    def _forward_downstream_feedback(self, goal_handle, feedback) -> None:
        if goal_handle.is_active:
            goal_handle.publish_feedback(feedback.feedback)

    def _publish_feedback(self, goal_handle, elapsed: float, positions: list[float], velocities: list[float]) -> None:
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = list(self.joint_names)
        feedback.desired = JointTrajectoryPoint()
        feedback.desired.positions = positions
        feedback.desired.velocities = velocities
        feedback.desired.time_from_start = make_duration(elapsed)
        feedback.actual = JointTrajectoryPoint()
        feedback.actual.positions = positions
        feedback.actual.velocities = velocities
        feedback.actual.time_from_start = make_duration(elapsed)
        feedback.error = JointTrajectoryPoint()
        feedback.error.positions = [0.0] * len(self.joint_names)
        feedback.error.velocities = [0.0] * len(self.joint_names)
        feedback.error.time_from_start = make_duration(elapsed)
        goal_handle.publish_feedback(feedback)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ExecutionBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        with contextlib.suppress(Exception):
            rclpy.shutdown()
