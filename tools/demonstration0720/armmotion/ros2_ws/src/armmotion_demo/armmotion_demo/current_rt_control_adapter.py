from __future__ import annotations

import threading
import time

import rclpy
from alfa_control_interfaces.action import VacuumGrip
from alfa_control_interfaces.msg import VacuumChannelResult, VacuumChannelSample
from alfa_system_interfaces.msg import ErrorInfo
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_interfaces.msg import PlcIoState
from std_srvs.srv import SetBool


UNVERIFIED = 0
ATTACHED_VERIFIED = 1


class CurrentRtControlAdapter(Node):
    """Expose target M-04/M-05 while bridging the currently deployed rt-control wire."""

    def __init__(self) -> None:
        super().__init__("motion_current_rt_control_adapter")
        self.declare_parameter("target_trajectory_action", "/whole_body_jtc/follow_joint_trajectory")
        self.declare_parameter("current_trajectory_action", "/dual_arm_jtc/follow_joint_trajectory")
        self.declare_parameter("target_vacuum_action", "/vacuum/grip")
        self.declare_parameter("plc_state_topic", "/plc/io_state")
        self.declare_parameter("left_solenoid_service", "/plc/left_solenoid")
        self.declare_parameter("right_solenoid_service", "/plc/right_solenoid")
        self.declare_parameter("vacuum_pump_service", "/plc/vacuum_pump")
        self.declare_parameter("interface_timeout_s", 5.0)
        self.declare_parameter("grip_verification_timeout_s", 5.0)
        self.declare_parameter("plc_state_max_age_s", 0.5)

        self._callback_group = ReentrantCallbackGroup()
        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("current_trajectory_action").value),
            callback_group=self._callback_group,
        )
        self._trajectory_server = ActionServer(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("target_trajectory_action").value),
            execute_callback=self._execute_trajectory,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self._callback_group,
        )
        self._vacuum_server = ActionServer(
            self,
            VacuumGrip,
            str(self.get_parameter("target_vacuum_action").value),
            execute_callback=self._execute_vacuum,
            goal_callback=self._validate_vacuum_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self._callback_group,
        )
        self._left_solenoid = self.create_client(
            SetBool,
            str(self.get_parameter("left_solenoid_service").value),
            callback_group=self._callback_group,
        )
        self._right_solenoid = self.create_client(
            SetBool,
            str(self.get_parameter("right_solenoid_service").value),
            callback_group=self._callback_group,
        )
        self._vacuum_pump = self.create_client(
            SetBool,
            str(self.get_parameter("vacuum_pump_service").value),
            callback_group=self._callback_group,
        )
        self._plc_lock = threading.Lock()
        self._plc_state: PlcIoState | None = None
        self._plc_state_received = 0.0
        self.create_subscription(
            PlcIoState,
            str(self.get_parameter("plc_state_topic").value),
            self._on_plc_state,
            10,
            callback_group=self._callback_group,
        )
        self.get_logger().warning(
            "启用现行 rt-control 迁移桥：M-04 转发到 /dual_arm_jtc，"
            "M-05 转换为 PLC SetBool；这不是最终五域 wire"
        )

    @staticmethod
    def _accept_goal(_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    @staticmethod
    def _accept_cancel(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    @staticmethod
    def _error(code: int, message: str = "", origin: str = "motion_rt_adapter") -> ErrorInfo:
        error = ErrorInfo()
        error.code = int(code)
        error.retryable = False
        error.message = message
        error.origin = origin
        return error

    def _wait_future(self, future, timeout_s: float, label: str):
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError(f"{label}超时")
            time.sleep(0.01)
        return future.result()

    def _execute_trajectory(self, goal_handle):
        result = FollowJointTrajectory.Result()
        timeout = float(self.get_parameter("interface_timeout_s").value)
        if not self._trajectory_client.wait_for_server(timeout_sec=timeout):
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "现行 /dual_arm_jtc action 不可用"
            return result
        forwarded = FollowJointTrajectory.Goal()
        forwarded.trajectory = goal_handle.request.trajectory
        forwarded.path_tolerance = goal_handle.request.path_tolerance
        forwarded.goal_tolerance = goal_handle.request.goal_tolerance
        forwarded.goal_time_tolerance = goal_handle.request.goal_time_tolerance

        def feedback_callback(message) -> None:
            goal_handle.publish_feedback(message.feedback)

        try:
            send_future = self._trajectory_client.send_goal_async(
                forwarded,
                feedback_callback=feedback_callback,
            )
            downstream = self._wait_future(send_future, timeout, "转发轨迹 Goal")
            if downstream is None or not downstream.accepted:
                raise RuntimeError("现行控制器拒绝轨迹")
            result_future = downstream.get_result_async()
            cancel_sent = False
            while not result_future.done():
                if goal_handle.is_cancel_requested and not cancel_sent:
                    cancel_sent = True
                    self._wait_future(downstream.cancel_goal_async(), timeout, "取消轨迹")
                time.sleep(0.01)
            wrapped = result_future.result()
            result.error_code = wrapped.result.error_code
            result.error_string = wrapped.result.error_string
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            elif result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                goal_handle.succeed()
            else:
                goal_handle.abort()
        except Exception as exc:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            goal_handle.abort()
        return result

    def _validate_vacuum_goal(self, request) -> GoalResponse:
        channels = list(request.channels)
        if request.command not in (VacuumGrip.Goal.GRIP, VacuumGrip.Goal.RELEASE):
            return GoalResponse.REJECT
        if not channels or len(channels) > 2 or len(set(channels)) != len(channels):
            return GoalResponse.REJECT
        if any(
            channel not in (VacuumGrip.Goal.CHANNEL_LEFT, VacuumGrip.Goal.CHANNEL_RIGHT)
            for channel in channels
        ):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute_vacuum(self, goal_handle):
        request = goal_handle.request
        enabled = request.command == VacuumGrip.Goal.GRIP
        results = [self._channel_result(channel) for channel in request.channels]
        result = VacuumGrip.Result()
        try:
            if enabled:
                self._set_output("真空泵", self._vacuum_pump, True)
            for channel in request.channels:
                self._set_output(
                    "左电磁阀" if channel == VacuumGrip.Goal.CHANNEL_LEFT else "右电磁阀",
                    self._left_solenoid if channel == VacuumGrip.Goal.CHANNEL_LEFT else self._right_solenoid,
                    enabled,
                )
            for item in results:
                item.command_accepted = True
                item.valve_actuation_completed = True
            if enabled:
                self._wait_for_vacuum(request.channels, goal_handle)
                for item in results:
                    item.verification_level = ATTACHED_VERIFIED
                result.overall_verification_level = ATTACHED_VERIFIED
            else:
                result.overall_verification_level = UNVERIFIED
            result.channels = results
            result.error = self._error(ErrorInfo.SUCCESS)
            goal_handle.succeed()
        except Exception as exc:
            for item in results:
                if item.error.code == ErrorInfo.SUCCESS:
                    item.error = self._error(ErrorInfo.EXECUTION_FAILED, str(exc))
            result.channels = results
            result.overall_verification_level = UNVERIFIED
            result.error = self._error(ErrorInfo.EXECUTION_FAILED, str(exc))
            goal_handle.abort()
        return result

    def _channel_result(self, channel: int) -> VacuumChannelResult:
        result = VacuumChannelResult()
        result.channel = int(channel)
        result.verification_level = UNVERIFIED
        result.error = self._error(ErrorInfo.SUCCESS)
        return result

    def _set_output(self, label: str, client, enabled: bool) -> None:
        timeout = float(self.get_parameter("interface_timeout_s").value)
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"{label}服务不可用")
        request = SetBool.Request()
        request.data = bool(enabled)
        response = self._wait_future(client.call_async(request), timeout, label)
        if response is None or not response.success:
            detail = "无响应" if response is None else response.message
            raise RuntimeError(f"{label}命令失败: {detail}")

    def _wait_for_vacuum(self, channels, goal_handle) -> None:
        timeout = float(self.get_parameter("grip_verification_timeout_s").value)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                raise RuntimeError("真空验证被取消；不会自动发送 RELEASE")
            state = self._fresh_plc_state()
            if state is not None:
                sample_feedback = VacuumGrip.Feedback()
                sample_feedback.header = state.header
                for channel in channels:
                    sample = VacuumChannelSample()
                    sample.channel = int(channel)
                    sample.pressure_pa = float("nan")
                    sample.pump_enabled = state.vacuum_pump_on
                    sample.valve_commanded_open = (
                        state.left_solenoid_on
                        if channel == VacuumGrip.Goal.CHANNEL_LEFT
                        else state.right_solenoid_on
                    )
                    established = (
                        state.left_vacuum_established
                        if channel == VacuumGrip.Goal.CHANNEL_LEFT
                        else state.right_vacuum_established
                    )
                    sample.observed_state = "ATTACHED" if established else "WAITING"
                    sample_feedback.channels.append(sample)
                goal_handle.publish_feedback(sample_feedback)
                if state.connected and state.data_fresh and all(
                    state.left_vacuum_established
                    if channel == VacuumGrip.Goal.CHANNEL_LEFT
                    else state.right_vacuum_established
                    for channel in channels
                ):
                    return
            time.sleep(0.05)
        raise TimeoutError("现行 PLC 未在时限内给出全部通道 vacuum_established")

    def _fresh_plc_state(self) -> PlcIoState | None:
        max_age = float(self.get_parameter("plc_state_max_age_s").value)
        with self._plc_lock:
            if self._plc_state is None or time.monotonic() - self._plc_state_received > max_age:
                return None
            return self._plc_state

    def _on_plc_state(self, message: PlcIoState) -> None:
        with self._plc_lock:
            self._plc_state = message
            self._plc_state_received = time.monotonic()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CurrentRtControlAdapter()
    executor = MultiThreadedExecutor(num_threads=6)
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
