from __future__ import annotations

import math
import os
import threading
import time
from typing import Any

from alfa_robot_execution_bridge.joints import (
    ARM_JOINT_POSITION_LIMITS_RAD,
    RT_CONTROL_JOINT_NAMES,
    model_to_rt_control_acceleration,
    model_to_rt_control_position,
    model_to_rt_control_velocity,
    require_arm_joint_in_range,
    rt_control_to_model_position,
)
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .common import MotionSample


JOINT_STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
ARM_JOINT_NAMES = tuple(
    name for name in RT_CONTROL_JOINT_NAMES if name not in ("turn", "updown")
)


def _duration_message(seconds: float):
    message = JointTrajectoryPoint().time_from_start
    seconds = max(0.0, float(seconds))
    message.sec = int(seconds)
    message.nanosec = int(round((seconds - message.sec) * 1e9))
    if message.nanosec >= 1_000_000_000:
        message.sec += 1
        message.nanosec -= 1_000_000_000
    return message


class HardwareExecutor:
    def __init__(
        self,
        node,
        *,
        dry_run: bool,
        action_name: str,
        left_solenoid_service: str,
        right_solenoid_service: str,
        vacuum_pump_service: str,
        wait_timeout_s: float,
        joint_state_topic: str = "/joint_states",
        start_joint_tolerance_deg: float = 5.0,
        start_updown_tolerance_m: float = 0.015,
        manage_grasp_io: bool = True,
    ) -> None:
        self.node = node
        self.dry_run = bool(dry_run)
        self.action_name = str(action_name)
        self.action_client = ActionClient(node, FollowJointTrajectory, action_name)
        self.manage_grasp_io = bool(manage_grasp_io)
        self.left_solenoid = (
            node.create_client(SetBool, left_solenoid_service)
            if self.manage_grasp_io
            else None
        )
        self.right_solenoid = (
            node.create_client(SetBool, right_solenoid_service)
            if self.manage_grasp_io
            else None
        )
        self.vacuum_pump = (
            node.create_client(SetBool, vacuum_pump_service)
            if self.manage_grasp_io
            else None
        )
        self.wait_timeout_s = float(wait_timeout_s)
        self.start_joint_tolerance_rad = math.radians(float(start_joint_tolerance_deg))
        self.start_updown_tolerance_m = float(start_updown_tolerance_m)
        self._state_lock = threading.Lock()
        self._latest_joints: dict[str, float] = {}
        self._latest_updown_m: float | None = None
        self._joint_state_subscription = node.create_subscription(
            JointState,
            joint_state_topic,
            self._on_joint_state,
            JOINT_STATE_QOS,
        )

    def verify_interfaces(self) -> None:
        if self.dry_run:
            self.node.get_logger().warning("dry_run=true：不会向真实控制器或 PLC 下发命令")
            return
        if not self.action_client.wait_for_server(timeout_sec=self.wait_timeout_s):
            raise RuntimeError(
                f"双臂 FollowJointTrajectory action 不可用: {self.action_name}; "
                f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', 'unset')}"
            )
        if self.manage_grasp_io:
            for label, client in (
                ("左电磁阀", self.left_solenoid),
                ("右电磁阀", self.right_solenoid),
                ("真空泵", self.vacuum_pump),
            ):
                if not client.wait_for_service(timeout_sec=self.wait_timeout_s):
                    raise RuntimeError(f"{label}服务不可用")

    def execute_segment(
        self,
        samples: list[MotionSample],
        label: str,
    ) -> dict[str, float]:
        if len(samples) < 2:
            raise ValueError(f"{label}: 轨迹段采样点不足")
        start_time = samples[0].time_s
        duration_s = max(0.1, samples[-1].time_s - start_time)
        peak_updown_velocity_mps = max(
            abs(sample.updown_velocity_m_s) for sample in samples
        )
        if self.dry_run:
            self.node.get_logger().info(
                f"[DRY-RUN] {label}: points={len(samples)} duration={duration_s:.3f}s "
                f"updown={samples[0].updown_m:.3f}->{samples[-1].updown_m:.3f}m "
                f"updown_peak_velocity={peak_updown_velocity_mps:.4f}m/s"
            )
            return {
                "duration_s": duration_s,
                "updown_velocity_m_s": peak_updown_velocity_mps,
            }
        self._wait_for_start_state(samples[0], label)
        trajectory = self._make_trajectory(samples)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal_future = self.action_client.send_goal_async(goal)
        self._wait_future(goal_future, f"{label}: 等待 action 接受")
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"{label}: 控制器拒绝轨迹")
        result_future = goal_handle.get_result_async()
        self._wait_future(result_future, f"{label}: 等待轨迹完成", duration_s + self.wait_timeout_s)
        wrapped_result = result_future.result()
        result = wrapped_result.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f"{label}: 执行失败 code={result.error_code} {result.error_string}")
        self._wait_for_start_state(samples[-1], f"{label}: 等待末态反馈")
        return {
            "duration_s": duration_s,
            "updown_velocity_m_s": peak_updown_velocity_mps,
        }

    def set_grasp_solenoids(self, enabled: bool) -> None:
        if not self.manage_grasp_io:
            raise RuntimeError("当前 Motion 入口不拥有吸附通路控制权")
        state = "on" if enabled else "off"
        if self.dry_run:
            self.node.get_logger().info(f"[DRY-RUN] 双侧电磁阀与真空泵同步 {state}")
            return
        futures: list[tuple[str, Any]] = []
        for label, client in (
            ("左电磁阀", self.left_solenoid),
            ("右电磁阀", self.right_solenoid),
            ("真空泵", self.vacuum_pump),
        ):
            request = SetBool.Request()
            request.data = bool(enabled)
            futures.append((label, client.call_async(request)))
        for label, future in futures:
            self._wait_future(future, f"等待{label}{state}")
            response = future.result()
            if response is None or not response.success:
                message = "无响应" if response is None else response.message
                raise RuntimeError(f"{label}{state}失败: {message}")
        self.node.get_logger().info(f"双侧电磁阀与真空泵同步 {state} 完成")

    def current_sample(self, timeout_s: float | None = None) -> MotionSample:
        if self.dry_run:
            raise RuntimeError("dry-run 没有真实当前状态")
        deadline = time.monotonic() + (self.wait_timeout_s if timeout_s is None else timeout_s)
        while time.monotonic() < deadline:
            with self._state_lock:
                joints = dict(self._latest_joints)
                updown_m = self._latest_updown_m
            if all(name in joints for name in RT_CONTROL_JOINT_NAMES if name != "updown") and updown_m is not None:
                return MotionSample(
                    time_s=0.0,
                    joints=joints,
                    updown_m=updown_m,
                    context={"stage": "initialization/current", "updown": updown_m},
                )
            time.sleep(0.05)
        raise TimeoutError("初始化前未收到完整 /joint_states")

    def _make_trajectory(
        self,
        samples: list[MotionSample],
    ) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.joint_names = list(RT_CONTROL_JOINT_NAMES)
        with self._state_lock:
            held_turn = self._latest_joints.get("turn", samples[0].joints.get("turn", 0.0))
        start_time = samples[0].time_s
        previous_time = -1.0
        for index, sample in enumerate(samples):
            relative_time = sample.time_s - start_time
            if index == 0:
                relative_time = 0.001
            elif relative_time <= previous_time:
                relative_time = previous_time + 0.001
            point = JointTrajectoryPoint()
            point.positions = []
            point.velocities = []
            point.accelerations = []
            for name in RT_CONTROL_JOINT_NAMES:
                model_position = (
                    sample.updown_m
                    if name == "updown"
                    else sample.joints[name]
                    if name != "turn"
                    else held_turn
                )
                model_velocity = (
                    sample.updown_velocity_m_s
                    if name == "updown"
                    else sample.joint_velocities.get(name, 0.0)
                    if name != "turn"
                    else 0.0
                )
                model_acceleration = (
                    sample.updown_acceleration_m_s2
                    if name == "updown"
                    else sample.joint_accelerations.get(name, 0.0)
                    if name != "turn"
                    else 0.0
                )
                if name in ARM_JOINT_POSITION_LIMITS_RAD:
                    require_arm_joint_in_range(name, model_position)
                point.positions.append(
                    model_to_rt_control_position(name, model_position)
                )
                point.velocities.append(
                    model_to_rt_control_velocity(name, model_velocity)
                )
                point.accelerations.append(
                    model_to_rt_control_acceleration(name, model_acceleration)
                )
            point.time_from_start = _duration_message(relative_time)
            trajectory.points.append(point)
            previous_time = relative_time
        return trajectory

    def _on_joint_state(self, message: JointState) -> None:
        joints: dict[str, float] = {}
        updown_m: float | None = None
        for index, name in enumerate(message.name):
            if index >= len(message.position):
                continue
            if name not in RT_CONTROL_JOINT_NAMES:
                continue
            value = rt_control_to_model_position(name, message.position[index])
            if name != "updown":
                joints[name] = value
            else:
                updown_m = value
        with self._state_lock:
            self._latest_joints.update(joints)
            if updown_m is not None:
                self._latest_updown_m = updown_m

    def _wait_for_start_state(
        self,
        expected: MotionSample,
        label: str,
    ) -> None:
        deadline = time.monotonic() + self.wait_timeout_s
        last_error = "尚未收到完整 /joint_states"
        while time.monotonic() < deadline:
            with self._state_lock:
                joints = dict(self._latest_joints)
                updown_m = self._latest_updown_m
            if (
                all(name in joints for name in (*ARM_JOINT_NAMES, "turn"))
                and updown_m is not None
            ):
                joint_errors = {
                    name: abs(joints[name] - expected.joints[name])
                    for name in ARM_JOINT_NAMES
                }
                worst_name = max(joint_errors, key=joint_errors.get)
                worst_joint_error = joint_errors[worst_name]
                updown_error = abs(updown_m - expected.updown_m)
                if (
                    worst_joint_error <= self.start_joint_tolerance_rad
                    and updown_error <= self.start_updown_tolerance_m
                ):
                    return
                last_error = (
                    f"{worst_name}偏差={math.degrees(worst_joint_error):.2f}deg，"
                    f"updown偏差={updown_error:.4f}m"
                )
            time.sleep(0.05)
        raise RuntimeError(f"{label}: 起点状态未对齐（{last_error}）")

    def _wait_future(self, future, label: str, timeout_s: float | None = None) -> None:
        deadline = time.monotonic() + (self.wait_timeout_s if timeout_s is None else timeout_s)
        while not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError(f"{label}超时")
            time.sleep(0.01)
