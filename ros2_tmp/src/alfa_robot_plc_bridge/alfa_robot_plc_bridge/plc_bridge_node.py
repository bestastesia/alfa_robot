"""ROS2 node bridging JointTrajectory commands to the ALFA PLC."""

from __future__ import annotations

import math
import threading
import time
from enum import Enum
from typing import Iterable

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import CancelResponse, GoalResponse
from rclpy.action.server import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

from .plc_core import (
    PlcConnectionConfig,
    PlcDriver,
    PositionOverwriteTrajectoryExecutor,
    TrajectoryCancelledError,
    TrajectoryExecutionConfig,
    TrajectoryPoint,
)
from .plc_core.config import PlcProtocolConfig
from .plc_core.mock import MockPlcTransport


DEFAULT_JOINT_NAMES = [
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


class BridgeState(str, Enum):
    NORMAL = 'normal'
    EXECUTING = 'executing'
    SOFT_STOPPED = 'soft_stopped'
    EMERGENCY_STOPPED = 'emergency_stopped'
    FAULT = 'fault'
    PLC_COMM_ERROR = 'plc_comm_error'


class PlcBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('plc_bridge_node')
        self._declare_parameters()
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.axis_ids = [int(axis) for axis in self.get_parameter('axis_ids').value]
        if len(self.joint_names) != len(self.axis_ids):
            raise ValueError('joint_names and axis_ids must have the same length')
        self.joint_to_axis = dict(zip(self.joint_names, self.axis_ids))
        self.axis_to_joint = dict(zip(self.axis_ids, self.joint_names))
        self.inverted_axes = {int(axis) for axis in self.get_parameter('inverted_axes').value}
        for axis in self.inverted_axes:
            if axis not in self.axis_to_joint:
                raise ValueError(f'inverted_axes contains unmapped axis: {axis}')
        self.ignore_unmapped_joints = bool(self.get_parameter('ignore_unmapped_joints').value)
        self._reported_unmapped_joints: set[str] = set()
        self.axes = tuple(self.axis_ids)
        self._plc_lock = threading.Lock()
        self._execution_lock = threading.Lock()
        self._trajectory_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._state = BridgeState.NORMAL
        self._last_error = ''
        self._last_command_count = 0
        self._last_trajectory_duration_s = 0.0
        self._fault_stop_written = False
        self._callback_group = ReentrantCallbackGroup()

        self.driver = self._build_driver()
        self.trajectory_executor = PositionOverwriteTrajectoryExecutor(
            self.driver,
            self._trajectory_config(),
            operation_lock=self._plc_lock,
        )
        self.publish_joint_states = bool(self.get_parameter('publish_joint_states').value)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10) if self.publish_joint_states else None
        self.bridge_state_pub = self.create_publisher(String, 'plc_bridge_state', 10)
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            'plc_joint_trajectory',
            self._on_trajectory,
            10,
            callback_group=self._callback_group,
        )
        self.trajectory_action = ActionServer(
            self,
            FollowJointTrajectory,
            str(self.get_parameter('follow_joint_trajectory_action').value),
            execute_callback=self._execute_action_goal,
            goal_callback=self._accept_action_goal,
            cancel_callback=self._cancel_action_goal,
            callback_group=self._callback_group,
        )
        self.soft_stop_srv = self.create_service(Trigger, 'plc_soft_stop', self._on_soft_stop, callback_group=self._callback_group)
        self.estop_srv = self.create_service(Trigger, 'plc_emergency_stop', self._on_emergency_stop, callback_group=self._callback_group)
        self.reset_estop_srv = self.create_service(Trigger, 'plc_reset_emergency', self._on_reset_emergency, callback_group=self._callback_group)
        self.reset_fault_srv = self.create_service(Trigger, 'plc_reset_fault', self._on_reset_fault, callback_group=self._callback_group)
        self.clear_commands_srv = self.create_service(Trigger, 'plc_clear_commands', self._on_clear_commands, callback_group=self._callback_group)
        feedback_hz = float(self.get_parameter('feedback_hz').value)
        self.feedback_timer = self.create_timer(1.0 / feedback_hz, self._publish_feedback_and_state, callback_group=self._callback_group)
        mode = 'mock virtual PLC' if bool(self.get_parameter('mock').value) else 'real PLC'
        self.get_logger().info(
            f'PLC bridge started in {mode}: axes={self.axes}, command_hz={self.get_parameter("command_hz").value}, plc_execution_mode={self.get_parameter("plc_execution_mode").value}, publish_joint_states={self.publish_joint_states}'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('plc_ip', '192.168.1.88')
        self.declare_parameter('plc_port', 502)
        self.declare_parameter('unit_id', 255)
        self.declare_parameter('connect_timeout_s', 3.0)
        self.declare_parameter('mock', False)
        self.declare_parameter('mock_active_axes', 12)
        self.declare_parameter('command_hz', 20.0)
        self.declare_parameter('feedback_hz', 20.0)
        self.declare_parameter('publish_joint_states', False)
        self.declare_parameter('trajectory_feedback_hz', 2.0)
        self.declare_parameter('follow_joint_trajectory_action', '/dual_v5_arm_controller/follow_joint_trajectory')
        self.declare_parameter('ignore_unmapped_joints', True)
        self.declare_parameter('velocity_limit_deg_s', 50.0)
        self.declare_parameter('acceleration_limit_deg_s2', 100.0)
        self.declare_parameter('deceleration_limit_deg_s2', 100.0)
        self.declare_parameter('emergency_deceleration_deg_s2', 120.0)
        self.declare_parameter('max_position_step_deg', 5.0)
        self.declare_parameter('plc_execution_mode', 'stream')
        self.declare_parameter('reset_recover_delay_s', 0.2)
        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)
        self.declare_parameter('axis_ids', list(range(1, 13)))
        self.declare_parameter('inverted_axes', [3, 5, 8])
        self.declare_parameter('mock_initial_positions_deg', [
            0.0, 5.0, 145.0, 0.0, 120.0, 0.0,
            0.0, 5.0, 145.0, 0.0, 120.0, 0.0,
        ])

    def _build_driver(self) -> PlcDriver:
        protocol = PlcProtocolConfig(max_axes=12)
        if bool(self.get_parameter('mock').value):
            driver = PlcDriver(
                protocol=protocol,
                transport=MockPlcTransport(protocol=protocol, active_axes=int(self.get_parameter('mock_active_axes').value)),
            )
            initial_positions = list(self.get_parameter('mock_initial_positions_deg').value)
            for index, axis in enumerate(self.axis_ids[:len(initial_positions)]):
                driver.transport.set_axis_feedback(axis, self._ros_deg_to_plc_deg(axis, float(initial_positions[index])))
            return driver
        return PlcDriver(
            connection=PlcConnectionConfig(
                host=str(self.get_parameter('plc_ip').value),
                port=int(self.get_parameter('plc_port').value),
                unit_id=int(self.get_parameter('unit_id').value),
                timeout_s=float(self.get_parameter('connect_timeout_s').value),
            ),
            protocol=protocol,
        )

    def _trajectory_config(self) -> TrajectoryExecutionConfig:
        return TrajectoryExecutionConfig(
            velocity_limit_deg_s=float(self.get_parameter('velocity_limit_deg_s').value),
            acceleration_limit_deg_s2=float(self.get_parameter('acceleration_limit_deg_s2').value),
            deceleration_limit_deg_s2=float(self.get_parameter('deceleration_limit_deg_s2').value),
            emergency_deceleration_deg_s2=float(self.get_parameter('emergency_deceleration_deg_s2').value),
            command_hz=float(self.get_parameter('command_hz').value),
            feedback_hz=float(self.get_parameter('trajectory_feedback_hz').value),
            max_position_step_deg=float(self.get_parameter('max_position_step_deg').value),
            execution_mode=str(self.get_parameter('plc_execution_mode').value),
        )

    def _publish_feedback_and_state(self) -> None:
        try:
            with self._plc_lock:
                statuses = self.driver.read_axes(self.axes)
        except Exception as exc:
            self._stop_event.set()
            self._set_state(BridgeState.PLC_COMM_ERROR, str(exc))
            self.get_logger().warning(f'failed to read PLC axes: {exc}')
            self._publish_bridge_state()
            return

        if self.publish_joint_states and self.joint_state_pub is not None:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            for status in statuses:
                msg.name.append(self.axis_to_joint[status.axis])
                msg.position.append(math.radians(self._plc_deg_to_ros_deg(status.axis, status.feedback_single_deg)))
                msg.velocity.append(0.0)
                msg.effort.append(0.0)
            self.joint_state_pub.publish(msg)

        fault_axes = [status for status in statuses if status.has_error]
        if fault_axes:
            self._stop_event.set()
            axes_text = ','.join(f'Axis{status.axis}:E{status.error_code}' for status in fault_axes)
            self._set_state(BridgeState.FAULT, axes_text)
            self._write_soft_stop_once_for_fault()
        elif self._state == BridgeState.PLC_COMM_ERROR:
            self._set_state(BridgeState.NORMAL)
        self._publish_bridge_state()

    def _on_trajectory(self, msg: JointTrajectory) -> None:
        try:
            points = self._convert_trajectory(msg)
        except Exception as exc:
            self.get_logger().error(f'rejected trajectory: {exc}')
            return
        if not self._try_start_execution():
            self.get_logger().error('trajectory already running or blocked; ignoring new trajectory')
            return
        self._publish_bridge_state()
        self._trajectory_thread = threading.Thread(target=self._execute_topic_trajectory, args=(points,), daemon=True)
        self._trajectory_thread.start()

    def _accept_action_goal(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        if self._is_motion_blocked():
            self.get_logger().error(f'rejecting action goal while bridge state={self._state.value}')
            return GoalResponse.REJECT
        if self._is_executing():
            self.get_logger().error('trajectory already running; rejecting action goal')
            return GoalResponse.REJECT
        try:
            self._convert_trajectory(goal_request.trajectory)
        except Exception as exc:
            self.get_logger().error(f'rejected action trajectory: {exc}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_action_goal(self, goal_handle) -> CancelResponse:
        self.get_logger().warning('trajectory action cancel requested; stopping PLC trajectory stream')
        self._stop_event.set()
        return CancelResponse.ACCEPT

    def _execute_action_goal(self, goal_handle) -> FollowJointTrajectory.Result:
        result = FollowJointTrajectory.Result()
        if not self._try_start_execution():
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = f'bridge busy or blocked: {self._state.value}'
            goal_handle.abort()
            return result
        try:
            points = self._convert_trajectory(goal_handle.request.trajectory)
            report = self._run_trajectory(points, cancel_checker=lambda: goal_handle.is_cancel_requested)
        except TrajectoryCancelledError as exc:
            self._safe_soft_stop_after_interrupt()
            self._set_state(BridgeState.SOFT_STOPPED, str(exc))
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            goal_handle.canceled()
            return result
        except Exception as exc:
            self._set_state(BridgeState.FAULT, str(exc))
            self.get_logger().error(f'action trajectory execution failed: {exc}')
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            goal_handle.abort()
            return result
        finally:
            self._finish_execution_if_not_blocked()
        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = f'commands={report.command_count}, duration={report.duration_s:.3f}s'
        return result

    def _convert_trajectory(self, msg: JointTrajectory) -> list[TrajectoryPoint]:
        if not msg.joint_names:
            raise ValueError('trajectory has no joint_names')
        if not msg.points:
            raise ValueError('trajectory has no points')
        unknown = [name for name in msg.joint_names if name not in self.joint_to_axis]
        if unknown and not self.ignore_unmapped_joints:
            raise ValueError(f'unknown joints: {unknown}')
        newly_reported = [name for name in unknown if name not in self._reported_unmapped_joints]
        if newly_reported:
            self.get_logger().warning(f'ignoring unmapped trajectory joints: {newly_reported}')
            self._reported_unmapped_joints.update(newly_reported)
        active_entries = [(index, self.joint_to_axis[name]) for index, name in enumerate(msg.joint_names) if name in self.joint_to_axis]
        if not active_entries:
            raise ValueError('trajectory has no PLC-mapped joints')
        points: list[TrajectoryPoint] = []
        for point in msg.points:
            if len(point.positions) != len(msg.joint_names):
                raise ValueError('point position length does not match joint_names')
            time_s = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9
            positions = {
                axis: self._ros_deg_to_plc_deg(axis, math.degrees(point.positions[index]))
                for index, axis in active_entries
            }
            points.append(TrajectoryPoint(time_from_start_s=time_s, positions_deg=positions))
        return points

    def _axis_direction(self, axis: int) -> int:
        return -1 if axis in self.inverted_axes else 1

    def _ros_deg_to_plc_deg(self, axis: int, position_deg: float) -> float:
        return position_deg * self._axis_direction(axis)

    def _plc_deg_to_ros_deg(self, axis: int, position_deg: float) -> float:
        return position_deg * self._axis_direction(axis)

    def _execute_topic_trajectory(self, points: Iterable[TrajectoryPoint]) -> None:
        try:
            report = self._run_trajectory(points)
        except TrajectoryCancelledError as exc:
            self._safe_soft_stop_after_interrupt()
            self._set_state(BridgeState.SOFT_STOPPED, str(exc))
            self.get_logger().warning(f'trajectory interrupted: {exc}')
            return
        except Exception as exc:
            self._set_state(BridgeState.FAULT, str(exc))
            self.get_logger().error(f'trajectory execution failed: {exc}')
            return
        finally:
            self._finish_execution_if_not_blocked()
            self._publish_bridge_state()
        self.get_logger().info(f'trajectory finished: commands={report.command_count}, duration={report.duration_s:.3f}s')

    def _run_trajectory(self, points: Iterable[TrajectoryPoint], cancel_checker=None):
        report = self.trajectory_executor.execute(
            points,
            stop_event=self._stop_event,
            cancel_checker=cancel_checker,
        )
        self._last_command_count = report.command_count
        self._last_trajectory_duration_s = report.duration_s
        return report

    def _on_soft_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._stop_event.set()
        try:
            with self._plc_lock:
                self.driver.soft_stop(self.axes)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._set_state(BridgeState.PLC_COMM_ERROR, str(exc))
            return response
        self._set_state(BridgeState.SOFT_STOPPED, 'PLC soft stop written')
        response.success = True
        response.message = 'PLC soft stop written'
        return response

    def _on_emergency_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._stop_event.set()
        try:
            with self._plc_lock:
                self.driver.emergency_stop(self.axes)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._set_state(BridgeState.PLC_COMM_ERROR, str(exc))
            return response
        self._set_state(BridgeState.EMERGENCY_STOPPED, 'PLC emergency stop written')
        response.success = True
        response.message = 'PLC emergency stop written'
        return response

    def _on_reset_emergency(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            with self._plc_lock:
                self.driver.reset_emergency(self.axes)
                time.sleep(float(self.get_parameter('reset_recover_delay_s').value))
                self.driver.hold_move_abs(self.axes)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._set_state(BridgeState.PLC_COMM_ERROR, str(exc))
            return response
        self._stop_event.clear()
        self._fault_stop_written = False
        self._set_state(BridgeState.NORMAL, 'PLC emergency reset written')
        response.success = True
        response.message = 'PLC emergency reset written'
        return response

    def _on_reset_fault(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            with self._plc_lock:
                self.driver.reset_fault(self.axes)
                time.sleep(float(self.get_parameter('reset_recover_delay_s').value))
                self.driver.hold_move_abs(self.axes)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._set_state(BridgeState.PLC_COMM_ERROR, str(exc))
            return response
        self._stop_event.clear()
        self._fault_stop_written = False
        self._set_state(BridgeState.NORMAL, 'PLC fault reset written')
        response.success = True
        response.message = 'PLC fault reset written'
        return response

    def _on_clear_commands(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._stop_event.set()
        try:
            with self._plc_lock:
                self.driver.clear_commands(self.axes)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._set_state(BridgeState.PLC_COMM_ERROR, str(exc))
            return response
        self._set_state(BridgeState.SOFT_STOPPED, 'PLC commands cleared')
        response.success = True
        response.message = 'PLC commands cleared'
        return response

    def _try_start_execution(self) -> bool:
        with self._execution_lock:
            if self._is_motion_blocked_locked() or self._is_executing_locked():
                return False
            self._stop_event.clear()
            self._last_command_count = 0
            self._last_trajectory_duration_s = 0.0
            self._set_state_locked(BridgeState.EXECUTING)
            return True

    def _finish_execution_if_not_blocked(self) -> None:
        with self._execution_lock:
            if self._state == BridgeState.EXECUTING:
                self._set_state_locked(BridgeState.NORMAL)

    def _is_executing(self) -> bool:
        with self._execution_lock:
            return self._is_executing_locked()

    def _is_executing_locked(self) -> bool:
        return self._state == BridgeState.EXECUTING

    def _is_motion_blocked(self) -> bool:
        with self._execution_lock:
            return self._is_motion_blocked_locked()

    def _is_motion_blocked_locked(self) -> bool:
        return self._state in {BridgeState.EMERGENCY_STOPPED, BridgeState.FAULT, BridgeState.PLC_COMM_ERROR}

    def _set_state(self, state: BridgeState, error: str = '') -> None:
        with self._execution_lock:
            self._set_state_locked(state, error)

    def _set_state_locked(self, state: BridgeState, error: str = '') -> None:
        self._state = state
        self._last_error = error

    def _publish_bridge_state(self) -> None:
        msg = String()
        msg.data = (
            f'mode={"mock" if bool(self.get_parameter("mock").value) else "real"};'
            f'state={self._state.value};'
            f'executing={str(self._is_executing()).lower()};'
            f'last_error={self._last_error};'
            f'last_command_count={self._last_command_count};'
            f'last_trajectory_duration_s={self._last_trajectory_duration_s:.3f}'
        )
        self.bridge_state_pub.publish(msg)

    def _write_soft_stop_once_for_fault(self) -> None:
        if self._fault_stop_written:
            return
        try:
            with self._plc_lock:
                self.driver.soft_stop(self.axes)
        except Exception as exc:
            self._set_state(BridgeState.PLC_COMM_ERROR, str(exc))
            return
        self._fault_stop_written = True

    def _safe_soft_stop_after_interrupt(self) -> None:
        try:
            with self._plc_lock:
                self.driver.soft_stop(self.axes)
        except Exception as exc:
            self._set_state(BridgeState.PLC_COMM_ERROR, str(exc))
            self.get_logger().error(f'failed to write soft stop after interrupt: {exc}')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PlcBridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
