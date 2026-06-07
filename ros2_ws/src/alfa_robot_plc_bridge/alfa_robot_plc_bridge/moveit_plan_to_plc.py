"""Plan with MoveIt and execute the resulting trajectory through the PLC bridge."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Iterable

import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


DEFAULT_ARM_JOINTS = [
    'left_v5_joint1',
    'left_v5_joint2',
    'left_v5_joint3',
    'left_v5_joint4',
    'left_v5_joint5',
    'left_v5_joint6',
    'right_v5_joint1',
    'right_v5_joint2',
    'right_v5_joint3',
    'right_v5_joint4',
    'right_v5_joint5',
    'right_v5_joint6',
]


def _duration_to_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


@dataclass(frozen=True)
class ParsedTargets:
    absolute_rad: dict[str, float]
    delta_rad: dict[str, float]


class MoveItPlanToPlc(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('moveit_plan_to_plc')
        self.args = args
        self.plan_client = self.create_client(GetMotionPlan, args.plan_service)
        self.action_client = ActionClient(self, FollowJointTrajectory, args.action)
        self._joint_state: JointState | None = None
        self._joint_state_sub = self.create_subscription(JointState, args.joint_state_topic, self._on_joint_state, 10)

    def _on_joint_state(self, msg: JointState) -> None:
        if msg.name and msg.position:
            self._joint_state = msg

    def wait_for_joint_state(self) -> JointState:
        deadline = self.get_clock().now().nanoseconds + int(self.args.timeout_s * 1e9)
        while rclpy.ok() and self._joint_state is None:
            if self.get_clock().now().nanoseconds > deadline:
                raise RuntimeError(f'timed out waiting for {self.args.joint_state_topic}')
            rclpy.spin_once(self, timeout_sec=0.1)
        assert self._joint_state is not None
        return self._joint_state

    def plan(self, current: JointState, targets: ParsedTargets) -> JointTrajectory:
        if not self.plan_client.wait_for_service(timeout_sec=self.args.timeout_s):
            raise RuntimeError(f'MoveIt planning service is not available: {self.args.plan_service}')

        current_positions = self._positions_by_name(current)
        goal_positions = self._goal_positions(current_positions, targets)

        request = GetMotionPlan.Request()
        plan_request = request.motion_plan_request
        plan_request.group_name = self.args.group
        plan_request.num_planning_attempts = self.args.planning_attempts
        plan_request.allowed_planning_time = self.args.allowed_planning_time_s
        plan_request.max_velocity_scaling_factor = self.args.velocity_scaling
        plan_request.max_acceleration_scaling_factor = self.args.acceleration_scaling
        plan_request.start_state = RobotState(joint_state=current)

        constraints = Constraints()
        for joint_name, position in goal_positions.items():
            constraint = JointConstraint()
            constraint.joint_name = joint_name
            constraint.position = position
            constraint.tolerance_above = self.args.tolerance_rad
            constraint.tolerance_below = self.args.tolerance_rad
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        plan_request.goal_constraints.append(constraints)

        self.get_logger().info(
            f'calling MoveIt plan: group={self.args.group}, joints={list(goal_positions)}, execute={self.args.execute}'
        )
        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.timeout_s + self.args.allowed_planning_time_s)
        if not future.done():
            raise RuntimeError('timed out waiting for MoveIt planning response')
        response = future.result()
        if response is None:
            raise RuntimeError('MoveIt planning service returned no response')

        plan_response = response.motion_plan_response
        if plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'MoveIt planning failed, error_code={plan_response.error_code.val}')
        trajectory = plan_response.trajectory.joint_trajectory
        if not trajectory.joint_names or not trajectory.points:
            raise RuntimeError('MoveIt returned an empty joint trajectory')
        self._print_plan_summary(trajectory)
        return trajectory

    def execute(self, trajectory: JointTrajectory) -> None:
        if not self.action_client.wait_for_server(timeout_sec=self.args.timeout_s):
            raise RuntimeError(f'PLC FollowJointTrajectory action is not available: {self.args.action}')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        self.get_logger().info(f'sending trajectory to PLC bridge action: {self.args.action}')
        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=self.args.timeout_s)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('PLC bridge rejected the trajectory goal')
        result_future = goal_handle.get_result_async()
        timeout_s = max(self.args.timeout_s, self._trajectory_duration_s(trajectory) + self.args.timeout_s)
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_s)
        if not result_future.done():
            raise RuntimeError('timed out waiting for PLC trajectory execution result')
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f'PLC trajectory failed: code={result.error_code}, message={result.error_string}')
        self.get_logger().info(f'PLC execution succeeded: {result.error_string}')

    def _positions_by_name(self, msg: JointState) -> dict[str, float]:
        positions: dict[str, float] = {}
        for index, name in enumerate(msg.name):
            if index < len(msg.position):
                positions[name] = float(msg.position[index])
        return positions

    def _goal_positions(self, current_positions: dict[str, float], targets: ParsedTargets) -> dict[str, float]:
        requested = set(targets.absolute_rad) | set(targets.delta_rad)
        if not requested:
            raise ValueError('no target joints were specified; use --delta-deg or --target-deg')
        missing = sorted(name for name in requested if name not in current_positions)
        if missing:
            raise ValueError(f'current joint_states does not contain target joints: {missing}')
        goal_positions = {name: current_positions[name] for name in requested}
        for name, position in targets.absolute_rad.items():
            goal_positions[name] = position
        for name, delta in targets.delta_rad.items():
            goal_positions[name] = current_positions[name] + delta
        return dict(sorted(goal_positions.items()))

    def _print_plan_summary(self, trajectory: JointTrajectory) -> None:
        duration_s = self._trajectory_duration_s(trajectory)
        self.get_logger().info(
            f'MoveIt plan ok: points={len(trajectory.points)}, duration={duration_s:.3f}s, joints={list(trajectory.joint_names)}'
        )
        final = trajectory.points[-1]
        for name, position in zip(trajectory.joint_names, final.positions):
            self.get_logger().info(f'  final {name} = {math.degrees(position):.3f} deg')

    def _trajectory_duration_s(self, trajectory: JointTrajectory) -> float:
        if not trajectory.points:
            return 0.0
        return _duration_to_seconds(trajectory.points[-1].time_from_start)


def parse_joint_value_map(values: Iterable[str], *, degrees: bool) -> dict[str, float]:
    parsed: dict[str, float] = {}
    for item in values:
        if not item:
            continue
        for entry in item.split(','):
            stripped = entry.strip()
            if not stripped:
                continue
            if ':' not in stripped:
                raise ValueError(f'expected JOINT:VALUE, got {stripped!r}')
            name, raw_value = stripped.split(':', 1)
            value = float(raw_value)
            parsed[name.strip()] = math.radians(value) if degrees else value
    return parsed


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Use MoveIt /plan_kinematic_path to plan a joint target and optionally execute it through the PLC bridge.'
    )
    parser.add_argument('--group', default='left_v5_arm', help='MoveIt planning group, e.g. left_v5_arm/right_v5_arm/dual_v5_arm')
    parser.add_argument('--joint-state-topic', default='/joint_states', help='JointState topic used as MoveIt start state')
    parser.add_argument('--plan-service', default='/plan_kinematic_path', help='MoveIt GetMotionPlan service')
    parser.add_argument('--action', default='/dual_v5_arm_controller/follow_joint_trajectory', help='PLC bridge FollowJointTrajectory action')
    parser.add_argument('--delta-deg', action='append', default=[], metavar='JOINT:DEG[,JOINT:DEG]', help='Relative joint target in degrees')
    parser.add_argument('--target-deg', action='append', default=[], metavar='JOINT:DEG[,JOINT:DEG]', help='Absolute joint target in degrees')
    parser.add_argument('--delta-rad', action='append', default=[], metavar='JOINT:RAD[,JOINT:RAD]', help='Relative joint target in radians')
    parser.add_argument('--target-rad', action='append', default=[], metavar='JOINT:RAD[,JOINT:RAD]', help='Absolute joint target in radians')
    parser.add_argument('--execute', action='store_true', help='Actually send the planned trajectory to PLC bridge')
    parser.add_argument('--timeout-s', type=float, default=10.0)
    parser.add_argument('--planning-attempts', type=int, default=5)
    parser.add_argument('--allowed-planning-time-s', type=float, default=5.0)
    parser.add_argument('--velocity-scaling', type=float, default=0.1)
    parser.add_argument('--acceleration-scaling', type=float, default=0.1)
    parser.add_argument('--tolerance-rad', type=float, default=0.001)
    return parser


def parse_targets(args: argparse.Namespace) -> ParsedTargets:
    absolute = parse_joint_value_map(args.target_deg, degrees=True)
    absolute.update(parse_joint_value_map(args.target_rad, degrees=False))
    delta = parse_joint_value_map(args.delta_deg, degrees=True)
    delta.update(parse_joint_value_map(args.delta_rad, degrees=False))
    overlap = sorted(set(absolute) & set(delta))
    if overlap:
        raise ValueError(f'joints cannot use both absolute and delta targets: {overlap}')
    return ParsedTargets(absolute_rad=absolute, delta_rad=delta)


def main(argv: list[str] | None = None) -> None:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    targets = parse_targets(args)

    rclpy.init(args=None)
    node = MoveItPlanToPlc(args)
    try:
        current = node.wait_for_joint_state()
        trajectory = node.plan(current, targets)
        if args.execute:
            node.execute(trajectory)
        else:
            node.get_logger().info('plan-only mode; add --execute to send this trajectory to the PLC bridge')
    except Exception as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(1) from exc
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
