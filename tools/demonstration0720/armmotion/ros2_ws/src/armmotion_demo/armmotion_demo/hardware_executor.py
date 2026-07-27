from __future__ import annotations

import math
import threading
import time
from typing import Any

from alfa_robot_execution_bridge.joints import (
    REAL_CONTROLLER_JOINT_NAMES,
    ethercat_to_ros_position,
    physical_to_logical_updown,
    ros_to_ethercat_position,
    ros_to_ethercat_velocity,
)
from alfa_robot_execution_bridge.updown import (
    make_updown_command_data,
    synchronized_updown_velocity_mps,
)
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .common import MotionSample


JOINT_STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
ARM_JOINT_NAMES = tuple(name for name in REAL_CONTROLLER_JOINT_NAMES if name != "turn")


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
        updown_topic: str,
        left_solenoid_service: str,
        right_solenoid_service: str,
        max_updown_speed_m_s: float,
        updown_acceleration_m_s2: float,
        updown_deceleration_m_s2: float,
        wait_timeout_s: float,
        joint_state_topic: str = "/joint_states",
        start_joint_tolerance_deg: float = 5.0,
        start_updown_tolerance_m: float = 0.015,
    ) -> None:
        self.node = node
        self.dry_run = bool(dry_run)
        self.action_client = ActionClient(node, FollowJointTrajectory, action_name)
        self.updown_publisher = node.create_publisher(Float64MultiArray, updown_topic, 10)
        self.left_solenoid = node.create_client(SetBool, left_solenoid_service)
        self.right_solenoid = node.create_client(SetBool, right_solenoid_service)
        self.max_updown_speed_m_s = float(max_updown_speed_m_s)
        self.updown_acceleration_m_s2 = float(updown_acceleration_m_s2)
        self.updown_deceleration_m_s2 = float(updown_deceleration_m_s2)
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
            raise RuntimeError("双臂 FollowJointTrajectory action 不可用")
        for label, client in (
            ("左电磁阀", self.left_solenoid),
            ("右电磁阀", self.right_solenoid),
        ):
            if not client.wait_for_service(timeout_sec=self.wait_timeout_s):
                raise RuntimeError(f"{label}服务不可用")

    def execute_segment(self, samples: list[MotionSample], label: str) -> dict[str, float]:
        if len(samples) < 2:
            raise ValueError(f"{label}: 轨迹段采样点不足")
        start_time = samples[0].time_s
        duration_s = max(0.1, samples[-1].time_s - start_time)
        updown_samples = [
            (sample.time_s - start_time, sample.updown_m)
            for sample in samples
        ]
        velocity_mps = synchronized_updown_velocity_mps(
            updown_samples,
            self.max_updown_speed_m_s,
        )
        if self.dry_run:
            self.node.get_logger().info(
                f"[DRY-RUN] {label}: points={len(samples)} duration={duration_s:.3f}s "
                f"updown={samples[0].updown_m:.3f}->{samples[-1].updown_m:.3f}m "
                f"pp_velocity={velocity_mps if velocity_mps is not None else 0.0:.4f}m/s"
            )
            return {
                "duration_s": duration_s,
                "updown_velocity_m_s": velocity_mps or 0.0,
            }
        self._wait_for_start_state(samples[0], label)
        if velocity_mps is not None:
            message = Float64MultiArray()
            message.data = make_updown_command_data(
                samples[-1].updown_m,
                velocity_mps,
                self.updown_acceleration_m_s2,
                self.updown_deceleration_m_s2,
            )
            self.updown_publisher.publish(message)
            self.node.get_logger().info(
                f"{label}: updown target={samples[-1].updown_m:.3f}m "
                f"velocity={velocity_mps:.4f}m/s"
            )
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
        return {
            "duration_s": duration_s,
            "updown_velocity_m_s": velocity_mps or 0.0,
        }

    def set_grasp_solenoids(self, enabled: bool) -> None:
        state = "on" if enabled else "off"
        if self.dry_run:
            self.node.get_logger().info(f"[DRY-RUN] 双侧电磁阀同步 {state}")
            return
        futures: list[tuple[str, Any]] = []
        for label, client in (
            ("左电磁阀", self.left_solenoid),
            ("右电磁阀", self.right_solenoid),
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
        self.node.get_logger().info(f"双侧电磁阀同步 {state} 完成")

    def current_sample(self, timeout_s: float | None = None) -> MotionSample:
        if self.dry_run:
            raise RuntimeError("dry-run 没有真实当前状态")
        deadline = time.monotonic() + (self.wait_timeout_s if timeout_s is None else timeout_s)
        while time.monotonic() < deadline:
            with self._state_lock:
                joints = dict(self._latest_joints)
                updown_m = self._latest_updown_m
            if all(name in joints for name in REAL_CONTROLLER_JOINT_NAMES) and updown_m is not None:
                return MotionSample(
                    time_s=0.0,
                    joints=joints,
                    updown_m=updown_m,
                    context={"stage": "initialization/current", "updown": updown_m},
                )
            time.sleep(0.05)
        raise TimeoutError("初始化前未收到完整 /joint_states")

    def _make_trajectory(self, samples: list[MotionSample]) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.joint_names = list(REAL_CONTROLLER_JOINT_NAMES)
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
            point.positions = [
                ros_to_ethercat_position(
                    name,
                    held_turn if name == "turn" else sample.joints[name],
                )
                for name in REAL_CONTROLLER_JOINT_NAMES
            ]
            point.velocities = [
                ros_to_ethercat_velocity(
                    name,
                    0.0 if name == "turn" else sample.joint_velocities.get(name, 0.0),
                )
                for name in REAL_CONTROLLER_JOINT_NAMES
            ]
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
            value = float(message.position[index])
            if name in REAL_CONTROLLER_JOINT_NAMES:
                joints[name] = ethercat_to_ros_position(name, value)
            elif name == "updown":
                updown_m = physical_to_logical_updown(value)
        with self._state_lock:
            self._latest_joints.update(joints)
            if updown_m is not None:
                self._latest_updown_m = updown_m

    def _wait_for_start_state(self, expected: MotionSample, label: str) -> None:
        deadline = time.monotonic() + self.wait_timeout_s
        last_error = "尚未收到完整 /joint_states"
        while time.monotonic() < deadline:
            with self._state_lock:
                joints = dict(self._latest_joints)
                updown_m = self._latest_updown_m
            if all(name in joints for name in ARM_JOINT_NAMES) and updown_m is not None:
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
