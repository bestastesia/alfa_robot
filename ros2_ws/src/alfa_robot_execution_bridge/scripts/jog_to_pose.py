#!/usr/bin/env python3
"""Jog/teaching tool for the rt-control public 14-axis trajectory action.

rt-control owns encoder zero offsets and motor direction calibration. All
``/joint_states`` feedback and ``/whole_body_jtc`` commands pass through the shared
``joints.py`` contract, whose public-boundary direction signs are all positive.
"""
from __future__ import annotations

import argparse
import math
import select
import sys
import time
from dataclasses import dataclass

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from alfa_robot_execution_bridge.joints import (
    EXECUTION_JOINT_NAMES,
    RT_CONTROL_ACTION_NAME,
    RT_CONTROL_JOINT_NAMES,
    UPDOWN_LOGICAL_LOWER_M,
    UPDOWN_LOGICAL_UPPER_M,
    model_to_rt_control_acceleration,
    model_to_rt_control_position,
    model_to_rt_control_velocity,
    require_arm_joint_in_range,
    require_updown_logical_in_range,
    rt_control_to_model_position,
)
from alfa_robot_execution_bridge.updown import (
    DEFAULT_UPDOWN_ACCELERATION_MPS2,
    DEFAULT_UPDOWN_VELOCITY_MPS,
)

SMOOTHSTEP_MAX_VELOCITY_GAIN = 1.875
SMOOTHSTEP_MAX_ACCELERATION_GAIN = 10.0 / math.sqrt(3.0)


def seconds_to_duration(seconds: float):
    point = JointTrajectoryPoint()
    duration = point.time_from_start
    seconds = max(0.0, float(seconds))
    whole = math.floor(seconds)
    duration.sec = int(whole)
    duration.nanosec = int(round((seconds - whole) * 1e9))
    if duration.nanosec >= 1_000_000_000:
        duration.sec += 1
        duration.nanosec -= 1_000_000_000
    return duration


def duration_to_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def smoothstep_state(phase: float) -> tuple[float, float, float]:
    phase = min(1.0, max(0.0, phase))
    position_scale = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
    velocity_scale = 30.0 * phase**2 - 60.0 * phase**3 + 30.0 * phase**4
    acceleration_scale = 60.0 * phase - 180.0 * phase**2 + 120.0 * phase**3
    return position_scale, velocity_scale, acceleration_scale


@dataclass
class TargetPlan:
    current: dict[str, float]
    target: dict[str, float]
    updown_current: float | None
    updown_target: float | None


PLC_OUTPUT_ARGUMENTS = (
    ("left_solenoid", "left solenoid"),
    ("right_solenoid", "right solenoid"),
    ("vacuum_pump", "vacuum pump"),
)


def requested_plc_outputs(args: argparse.Namespace) -> list[tuple[str, bool]]:
    outputs: list[tuple[str, bool]] = []
    for attribute, label in PLC_OUTPUT_ARGUMENTS:
        value = str(getattr(args, attribute)).strip().lower()
        if value == "keep":
            continue
        outputs.append((label, value == "on"))
    return outputs


class JogToPose(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('lhy_jog_to_pose')
        self.args = args
        self.action_client = ActionClient(self, FollowJointTrajectory, args.action_name)
        self.plc_clients = {
            "left solenoid": self.create_client(SetBool, args.left_solenoid_service),
            "right solenoid": self.create_client(SetBool, args.right_solenoid_service),
            "vacuum pump": self.create_client(SetBool, args.vacuum_pump_service),
        }
        self._latest_joint_state: JointState | None = None
        self.create_subscription(
            JointState, args.joint_state_topic, self._on_joint_state, qos_profile_sensor_data,
        )
        self.rerun_ctx = None

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joint_state = msg
        if self.rerun_ctx is not None:
            self.rerun_ctx.on_joint_state(msg)

    def wait_joint_state(self) -> JointState:
        deadline = time.monotonic() + self.args.joint_state_timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            msg = self._latest_joint_state
            if msg is None:
                continue
            names = set(str(name) for name in msg.name)
            if all(name in names for name in RT_CONTROL_JOINT_NAMES):
                return msg
        raise TimeoutError(f'timed out waiting for full 14-axis /joint_states on {self.args.joint_state_topic}')

    def spin_for(self, seconds: float) -> None:
        """Keep processing /joint_states (and driving Rerun) for a fixed duration.

        rclpy only advances subscription callbacks while something calls
        spin/spin_once; without this, Rerun would only ever show the single
        frame captured by wait_joint_state() and then freeze for the rest of
        the process lifetime (dry-run print, the --send confirmation prompt,
        etc. do not spin on their own).
        """
        deadline = time.monotonic() + max(0.0, seconds)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    @staticmethod
    def ros_positions_from_joint_state(msg: JointState) -> tuple[dict[str, float], bool]:
        """Read the rt-control public /joint_states in ROS/URDF semantics.

        Returns (values, updown_out_of_contract). The physical-to-logical
        contract conversion always applies (the current calibrated offset is zero), but if the
        updown falls outside [UPDOWN_LOGICAL_LOWER_M, UPDOWN_LOGICAL_UPPER_M],
        the value is still reported so the operator can see current state and
        jog back into range, and updown_out_of_contract is set so callers can
        warn and refuse to use it as an implicit jog target.
        """
        values: dict[str, float] = {}
        updown_out_of_contract = False
        for index, name in enumerate(msg.name):
            name = str(name)
            if index >= len(msg.position):
                continue
            if name not in RT_CONTROL_JOINT_NAMES:
                continue
            value = rt_control_to_model_position(name, msg.position[index])
            if name in EXECUTION_JOINT_NAMES:
                values[name] = value
            elif name == 'updown':
                logical_m = value
                values[name] = logical_m
                if not (UPDOWN_LOGICAL_LOWER_M - 1e-9 <= logical_m <= UPDOWN_LOGICAL_UPPER_M + 1e-9):
                    updown_out_of_contract = True
        return values, updown_out_of_contract

    def build_target_plan(self) -> TargetPlan:
        msg = self.wait_joint_state()
        values, updown_out_of_contract = self.ros_positions_from_joint_state(msg)
        current = {name: values[name] for name in EXECUTION_JOINT_NAMES}
        updown_current = values.get('updown')
        if updown_out_of_contract and updown_current is not None:
            print(
                f'WARNING: current updown feedback converts to {updown_current:.4f} m logical, '
                f'outside the valid [{UPDOWN_LOGICAL_LOWER_M}, {UPDOWN_LOGICAL_UPPER_M}] m range. '
                'This means updown is not currently in the new logical contract. '
                'Fix/verify with the electrical engineer before commanding updown.',
                file=sys.stderr,
            )
        target = dict(current)
        specified = self._target_args_deg()
        for name, deg_value in specified.items():
            if deg_value is not None:
                target_rad = math.radians(float(deg_value))
                require_arm_joint_in_range(name, target_rad)
                target[name] = target_rad
        updown_target = self.args.updown_m
        if updown_target is not None:
            require_updown_logical_in_range(updown_target)
            if updown_current is None:
                raise RuntimeError(
                    "requested --updown-m but no 'updown' feedback was found in "
                    f'{self.args.joint_state_topic}; refusing to command updown blind'
                )
        return TargetPlan(current=current, target=target, updown_current=updown_current, updown_target=updown_target)

    def _target_args_deg(self) -> dict[str, float | None]:
        return {
            'right_joint1': self.args.right_joint1_deg,
            'right_joint2': self.args.right_joint2_deg,
            'right_joint3': self.args.right_joint3_deg,
            'right_joint4': self.args.right_joint4_deg,
            'right_joint5': self.args.right_joint5_deg,
            'right_joint6': self.args.right_joint6_deg,
            'left_joint1': self.args.left_joint1_deg,
            'left_joint2': self.args.left_joint2_deg,
            'left_joint3': self.args.left_joint3_deg,
            'left_joint4': self.args.left_joint4_deg,
            'left_joint5': self.args.left_joint5_deg,
            'left_joint6': self.args.left_joint6_deg,
            'turn': self.args.turn_deg,
        }

    def make_trajectory(self, plan: TargetPlan) -> JointTrajectory:
        """Build one complete 14-axis trajectory in rt-control ROS semantics."""
        trajectory = JointTrajectory()
        trajectory.joint_names = list(RT_CONTROL_JOINT_NAMES)
        duration_s = self.trajectory_duration_s(plan)
        steps = max(1, int(math.ceil(duration_s * self.args.hz)))
        for step in range(steps + 1):
            phase = step / steps
            if self.args.profile == 'smoothstep':
                ratio, velocity_scale, acceleration_scale = smoothstep_state(phase)
            else:
                ratio, velocity_scale, acceleration_scale = phase, 1.0, 0.0
            point = JointTrajectoryPoint()
            point.time_from_start = seconds_to_duration(duration_s * phase)
            positions = []
            velocities = []
            accelerations = []
            for name in RT_CONTROL_JOINT_NAMES:
                if name == 'updown':
                    if plan.updown_current is None:
                        raise RuntimeError("完整14轴轨迹缺少 updown 反馈")
                    current_value = plan.updown_current
                    target_value = (
                        plan.updown_current
                        if plan.updown_target is None
                        else plan.updown_target
                    )
                else:
                    current_value = plan.current[name]
                    target_value = plan.target[name]
                delta = target_value - current_value
                positions.append(
                    model_to_rt_control_position(
                        name, current_value + delta * ratio
                    )
                )
                velocities.append(
                    model_to_rt_control_velocity(
                        name, delta * velocity_scale / duration_s
                    )
                )
                accelerations.append(
                    model_to_rt_control_acceleration(
                        name,
                        delta * acceleration_scale / (duration_s * duration_s),
                    )
                )
            point.positions = positions
            if self.args.profile == 'smoothstep':
                point.velocities = velocities
                point.accelerations = accelerations
            trajectory.points.append(point)
        return trajectory

    def trajectory_duration_s(self, plan: TargetPlan) -> float:
        duration_s = float(self.args.duration_s)
        if self.args.profile != 'smoothstep' or not self.args.auto_extend_duration:
            return duration_s

        for name in EXECUTION_JOINT_NAMES:
            delta_deg = abs(math.degrees(plan.target[name] - plan.current[name]))
            if delta_deg <= 0.0:
                continue
            if self.args.max_vel_deg_s > 0.0:
                duration_s = max(duration_s, delta_deg * SMOOTHSTEP_MAX_VELOCITY_GAIN / self.args.max_vel_deg_s)
            if self.args.max_accel_deg_s2 > 0.0:
                duration_s = max(
                    duration_s,
                    math.sqrt(delta_deg * SMOOTHSTEP_MAX_ACCELERATION_GAIN / self.args.max_accel_deg_s2),
                )
        if plan.updown_current is not None and plan.updown_target is not None:
            updown_delta_m = abs(plan.updown_target - plan.updown_current)
            duration_s = max(
                duration_s,
                updown_delta_m * SMOOTHSTEP_MAX_VELOCITY_GAIN / self.args.updown_speed_mps,
                math.sqrt(
                    updown_delta_m
                    * SMOOTHSTEP_MAX_ACCELERATION_GAIN
                    / self.args.updown_acceleration_mps2
                ),
            )
        return duration_s

    def print_plan(self, plan: TargetPlan, trajectory: JointTrajectory) -> None:
        print('Current -> Target (ROS/URDF semantics, matches MoveIt/Rerun):')
        for name in EXECUTION_JOINT_NAMES:
            print(f'  {name:12s} {math.degrees(plan.current[name]):9.3f} deg -> {math.degrees(plan.target[name]):9.3f} deg')
        if plan.updown_target is not None:
            current_text = 'UNKNOWN' if plan.updown_current is None else f'{plan.updown_current:.4f} m'
            print(f'  {"updown":12s} {current_text:>13s} -> {plan.updown_target:9.4f} m (logical)')
            print(
                f'  updown rt-control target={plan.updown_target:.4f} m'
            )
        else:
            current_text = 'UNKNOWN' if plan.updown_current is None else f'{plan.updown_current:.4f} m'
            print(f'  {"updown":12s} {current_text:>13s} -> hold/no command')
        max_delta = 0.0
        max_joint = '-'
        for name in EXECUTION_JOINT_NAMES:
            delta = abs(plan.target[name] - plan.current[name])
            if delta > max_delta:
                max_delta = delta
                max_joint = name
        print('\nTrajectory:')
        print(f'  action={self.args.action_name}')
        actual_duration_s = duration_to_seconds(trajectory.points[-1].time_from_start)
        print(f'  points={len(trajectory.points)} hz={self.args.hz:.1f} duration={actual_duration_s:.3f}s requested={self.args.duration_s:.3f}s')
        print(f'  profile={self.args.profile} auto_extend_duration={self.args.auto_extend_duration}')
        if self.args.profile == 'smoothstep':
            print(f'  max_vel_limit={self.args.max_vel_deg_s:.3f}deg/s max_accel_limit={self.args.max_accel_deg_s2:.3f}deg/s^2')
        print(f'  max_arm_delta={math.degrees(max_delta):.3f}deg@{max_joint}')
        print('  rt-control boundary: shared joints.py direction contract; no raw EtherCAT offset conversion')
        if plan.updown_target is not None:
            print(
                f'  updown limits=[velocity={self.args.updown_speed_mps:.4f}m/s, '
                f'acceleration={self.args.updown_acceleration_mps2:.4f}m/s^2]'
            )

    def print_plc_plan(self) -> None:
        outputs = requested_plc_outputs(self.args)
        if not outputs:
            print("\nPLC outputs: keep all outputs unchanged")
            return
        print(f"\nPLC outputs ({self.args.plc_when}):")
        for label, enabled in outputs:
            print(f"  {label:14s} -> {'on' if enabled else 'off'}")

    def apply_plc_outputs(self) -> bool:
        outputs = requested_plc_outputs(self.args)
        for label, enabled in outputs:
            client = self.plc_clients[label]
            if not client.wait_for_service(timeout_sec=self.args.plc_service_timeout_s):
                print(f"PLC service unavailable for {label}: {client.srv_name}", file=sys.stderr)
                return False
            request = SetBool.Request()
            request.data = enabled
            future = client.call_async(request)
            deadline = time.monotonic() + self.args.plc_service_timeout_s
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            if not future.done():
                print(f"PLC service timed out for {label}: {client.srv_name}", file=sys.stderr)
                return False
            response = future.result()
            if response is None or not response.success:
                message = "no response" if response is None else response.message
                print(f"PLC command failed for {label}: {message}", file=sys.stderr)
                return False
            print(f"PLC {label} -> {'on' if enabled else 'off'}: {response.message}")
        return True

    def send(self, plan: TargetPlan, trajectory: JointTrajectory) -> int:
        if not self.action_client.wait_for_server(timeout_sec=self.args.action_timeout_s):
            raise TimeoutError(f'action server not available: {self.args.action_name}')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        print('Sending FollowJointTrajectory goal...')
        send_future = self.action_client.send_goal_async(goal, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print('Goal rejected.', file=sys.stderr)
            return 1
        print('Goal accepted.')

        if not self.args.wait_result:
            return 0
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + self.args.result_timeout_s
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not result_future.done():
            print('Timed out waiting for action result.', file=sys.stderr)
            return 2
        result = result_future.result().result
        print(f'Result: error_code={result.error_code} error_string={result.error_string!r}')
        return 0 if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL else 3

    def _feedback(self, msg) -> None:
        if not self.args.verbose_feedback:
            return
        actual = msg.feedback.actual
        print(f'  feedback actual_time={duration_to_seconds(actual.time_from_start):.3f}s')


class RerunLiveContext:
    """Optional live Rerun view: actual robot (from /joint_states) plus a ghost target pose.

    Reuses alfa_robot_rerun.visualize_rerun's existing public API. All joint
    values logged here are ROS/URDF semantics; the conversion from raw
    /joint_states already happened in JogToPose.ros_positions_from_joint_state
    before this class ever sees a value.
    """

    ACTUAL_PATH = 'world/robot'
    TARGET_PATH = 'world/robot_target'

    def __init__(self, app_id: str = 'jog_to_pose_live') -> None:
        try:
            import rerun as rerun_module
        except ImportError as exc:
            raise RuntimeError('rerun-sdk is not installed; omit --rerun or install rerun-sdk') from exc
        from alfa_robot_rerun import visualize_rerun as rerun_helpers

        self._helpers = rerun_helpers
        self._rr = rerun_module
        self.robot = rerun_helpers.UrdfRobot(rerun_helpers.render_current_urdf())
        self._rr.init(app_id)
        self._rr.spawn()
        rerun_helpers.log_robot_static_model(self.robot, self.ACTUAL_PATH, log_meshes=True)
        rerun_helpers.log_robot_static_model(self.robot, self.TARGET_PATH, log_meshes=True)
        self._sample = 0

    def on_joint_state(self, msg: JointState) -> None:
        values, _ = JogToPose.ros_positions_from_joint_state(msg)
        if not values:
            return
        self._helpers.set_sample_time(self._sample)
        self._helpers.log_robot_state(self.robot, values, self.ACTUAL_PATH)
        self._sample += 1

    def log_target(self, target: dict[str, float], updown_target: float | None) -> None:
        joint_positions = dict(target)
        if updown_target is not None:
            joint_positions['updown'] = updown_target
        self._helpers.log_robot_state(self.robot, joint_positions, self.TARGET_PATH)


def add_joint_args(parser: argparse.ArgumentParser) -> None:
    for prefix in ('right', 'left'):
        for i in range(1, 7):
            parser.add_argument(
                f'--{prefix}-joint{i}-deg', type=float, default=None,
                help='Target angle in ROS/URDF degrees (same value you would see in MoveIt/Rerun).',
            )
    parser.add_argument('--turn-deg', type=float, default=None, help='Target turn angle in ROS/URDF degrees.')
    parser.add_argument(
        '--updown-m', type=float, default=None,
        help=f'Target updown in logical meters, range [{UPDOWN_LOGICAL_LOWER_M}, {UPDOWN_LOGICAL_UPPER_M}] '
             '(matches MoveIt/Rerun updown joint value, NOT the raw physical controller command).',
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Jog/teaching tool: send a complete rt-control 14-axis pose in URDF/ROS semantics. '
                    'Replaces run_move_all_joints_abs.sh / move_all_joints_abs.py. '
                    'The fixed controller order is imported from alfa_robot_execution_bridge.joints.',
    )
    parser.add_argument('--action-name', default=RT_CONTROL_ACTION_NAME)
    parser.add_argument('--joint-state-topic', default='/joint_states')
    parser.add_argument('--updown-speed-mps', type=float, default=DEFAULT_UPDOWN_VELOCITY_MPS)
    parser.add_argument(
        '--updown-acceleration-mps2',
        type=float,
        default=DEFAULT_UPDOWN_ACCELERATION_MPS2,
    )
    parser.add_argument('--duration-s', type=float, default=5.0)
    parser.add_argument('--hz', type=float, default=10.0)
    parser.add_argument('--profile', choices=['smoothstep', 'linear'], default='smoothstep')
    parser.add_argument('--max-vel-deg-s', type=float, default=20.0, help='Smoothstep arm/turn peak velocity limit. Auto-extends duration when needed.')
    parser.add_argument('--max-accel-deg-s2', type=float, default=20.0, help='Smoothstep arm/turn peak acceleration limit. Lower value gives softer start/stop.')
    parser.add_argument('--auto-extend-duration', action=argparse.BooleanOptionalAction, default=True, help='Extend duration to satisfy max velocity/acceleration limits.')
    parser.add_argument('--joint-state-timeout-s', type=float, default=5.0)
    parser.add_argument('--action-timeout-s', type=float, default=5.0)
    parser.add_argument('--result-timeout-s', type=float, default=60.0)
    parser.add_argument('--wait-result', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--send', action='store_true', help='Actually send commands. Without this, only prints the plan (dry run).')
    parser.add_argument('--left-solenoid', choices=['keep', 'on', 'off'], default='keep')
    parser.add_argument('--right-solenoid', choices=['keep', 'on', 'off'], default='keep')
    parser.add_argument('--vacuum-pump', choices=['keep', 'on', 'off'], default='keep')
    parser.add_argument('--plc-when', choices=['before', 'after'], default='after')
    parser.add_argument('--plc-only', action='store_true')
    parser.add_argument('--left-solenoid-service', default='/plc/left_solenoid')
    parser.add_argument('--right-solenoid-service', default='/plc/right_solenoid')
    parser.add_argument('--vacuum-pump-service', default='/plc/vacuum_pump')
    parser.add_argument('--plc-service-timeout-s', type=float, default=3.0)
    parser.add_argument('--verbose-feedback', action='store_true')
    parser.add_argument('--rerun', dest='rerun', action='store_true', default=False, help='Open a live Rerun view of actual + target robot state.')
    parser.add_argument('--no-rerun', dest='rerun', action='store_false')
    parser.add_argument(
        '--rerun-preview-s', type=float, default=20.0,
        help='On a dry run (no --send) with --rerun, keep spinning /joint_states into the '
             'live view for this many seconds before exiting, so the actual robot pose is '
             'visibly live instead of a single frozen frame. Ctrl+C to stop earlier.',
    )
    add_joint_args(parser)
    return parser.parse_args()


def wait_for_enter_while_spinning(node: 'JogToPose', prompt: str) -> None:
    """Block for Enter on stdin while still spinning the node.

    Plain input() blocks the whole process, so any open --rerun view would
    freeze for the entire confirmation pause. This polls stdin with select()
    and spins the node in between, so live /joint_states keep flowing into
    Rerun while the operator is deciding whether to send.
    """
    print(prompt, end='', flush=True)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if ready:
            sys.stdin.readline()
            print()
            return


def main() -> int:
    args = parse_args()
    if args.duration_s <= 0.0:
        raise SystemExit('--duration-s must be > 0')
    if args.hz <= 0.0:
        raise SystemExit('--hz must be > 0')
    if args.max_vel_deg_s <= 0.0:
        raise SystemExit('--max-vel-deg-s must be > 0')
    if args.max_accel_deg_s2 <= 0.0:
        raise SystemExit('--max-accel-deg-s2 must be > 0')
    if args.updown_speed_mps <= 0.0:
        raise SystemExit('--updown-speed-mps must be > 0')
    if args.updown_acceleration_mps2 <= 0.0:
        raise SystemExit('--updown-acceleration-mps2 must be > 0')
    if args.plc_service_timeout_s <= 0.0:
        raise SystemExit('--plc-service-timeout-s must be > 0')
    if args.plc_only and not requested_plc_outputs(args):
        raise SystemExit('--plc-only requires at least one PLC output set to on/off')
    if args.plc_only and args.rerun:
        raise SystemExit('--plc-only cannot be combined with --rerun')
    if requested_plc_outputs(args) and args.plc_when == 'after' and not args.wait_result:
        raise SystemExit('--plc-when after requires --wait-result')
    rclpy.init()
    node = JogToPose(args)
    rerun_ctx = None
    try:
        if args.plc_only:
            node.print_plc_plan()
            if not args.send:
                print('\nDry run only. Add --send to write PLC outputs.')
                return 0
            wait_for_enter_while_spinning(
                node, '\n确认 PLC 输出切换安全后按 Enter 发送；Ctrl+C 取消...',
            )
            return 0 if node.apply_plc_outputs() else 4
        if args.rerun:
            rerun_ctx = RerunLiveContext()
            node.rerun_ctx = rerun_ctx
        plan = node.build_target_plan()
        if rerun_ctx is not None:
            rerun_ctx.log_target(plan.target, plan.updown_target)
        trajectory = node.make_trajectory(plan)
        node.print_plan(plan, trajectory)
        node.print_plc_plan()
        if not args.send:
            print('\nDry run only. Add --send after confirming hardware safety.')
            if rerun_ctx is not None:
                print(f'Watching live Rerun for {args.rerun_preview_s:.1f}s (Ctrl+C to stop early)...')
                node.spin_for(args.rerun_preview_s)
            return 0
        wait_for_enter_while_spinning(
            node, '\n确认机械安全、人员远离、目标位置正确后按 Enter 发送；Ctrl+C 取消...',
        )
        if requested_plc_outputs(args) and args.plc_when == 'before':
            if not node.apply_plc_outputs():
                return 4
        result = node.send(plan, trajectory)
        if result != 0:
            return result
        if requested_plc_outputs(args) and args.plc_when == 'after':
            if not node.apply_plc_outputs():
                return 4
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
