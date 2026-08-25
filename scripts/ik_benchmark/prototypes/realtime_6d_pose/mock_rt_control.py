#!/usr/bin/python3
"""PROTOTYPE latest-target rt-control simulator for the interactive 6D-pose demo."""

from __future__ import annotations

import json
import math
import time

import rclpy
from alfa_robot_execution_bridge.joints import RT_CONTROL_JOINT_NAMES as CONTRACT_JOINT_NAMES
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory


# Import the execution package's single source of truth. Commands with any
# other order are rejected so this prototype exercises the real boundary.
RT_CONTROL_JOINT_NAMES = list(CONTRACT_JOINT_NAMES)


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


class MockRtControl(Node):
    def __init__(self) -> None:
        super().__init__("mock_rt_control")
        self.declare_parameter("control_rate_hz", 250.0)
        self.declare_parameter("feedback_rate_hz", 100.0)
        self.declare_parameter("status_rate_hz", 10.0)
        self.declare_parameter("command_timeout_s", 0.10)
        self.declare_parameter("position_gain", 18.0)
        self.declare_parameter("max_arm_velocity_rad_s", 1.5)
        self.declare_parameter("max_arm_acceleration_rad_s2", 8.0)
        self.declare_parameter("max_updown_velocity_m_s", 0.08)
        self.declare_parameter("max_updown_acceleration_m_s2", 0.4)
        self.declare_parameter(
            "initial_arm_joints_deg", [0.0, -45.0, 120.0, -75.0, 0.0, 0.0]
        )
        self.declare_parameter("fixed_updown", 0.3)

        control_rate = float(self.get_parameter("control_rate_hz").value)
        feedback_rate = float(self.get_parameter("feedback_rate_hz").value)
        status_rate = float(self.get_parameter("status_rate_hz").value)
        initial_arm = [
            math.radians(float(value))
            for value in self.get_parameter("initial_arm_joints_deg").value
        ]
        if len(initial_arm) != 6:
            raise ValueError("initial_arm_joints_deg must contain exactly 6 values")

        initial = initial_arm + initial_arm + [0.0, float(self.get_parameter("fixed_updown").value)]
        self.position = dict(zip(RT_CONTROL_JOINT_NAMES, initial))
        self.velocity = {name: 0.0 for name in RT_CONTROL_JOINT_NAMES}
        self.target: dict[str, float] | None = None
        self.last_command_monotonic: float | None = None
        self.last_control_monotonic = time.monotonic()
        self.last_error = "waiting_for_first_command"
        self.control_ticks = 0
        self.control_window_start = self.last_control_monotonic
        self.measured_control_hz = 0.0
        self.max_tracking_error = 0.0

        self.feedback_pub = self.create_publisher(
            JointState, "/realtime_6d_pose/joint_states", qos_profile_sensor_data
        )
        self.status_pub = self.create_publisher(
            String, "/realtime_6d_pose/rt_status", qos_profile_sensor_data
        )
        self.create_subscription(
            JointTrajectory,
            "/realtime_6d_pose/rt_command",
            self.on_command,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / max(control_rate, 1.0), self.control_step)
        self.create_timer(1.0 / max(feedback_rate, 1.0), self.publish_feedback)
        self.create_timer(1.0 / max(status_rate, 1.0), self.publish_status)
        self.get_logger().info(
            f"PROTOTYPE Mock rt-control ready: control={control_rate:.1f}Hz "
            f"feedback={feedback_rate:.1f}Hz timeout="
            f"{float(self.get_parameter('command_timeout_s').value):.3f}s"
        )

    def on_command(self, message: JointTrajectory) -> None:
        if list(message.joint_names) != RT_CONTROL_JOINT_NAMES:
            self.last_error = "joint_contract_mismatch"
            self.get_logger().error(
                f"reject command joint order: expected={RT_CONTROL_JOINT_NAMES}, "
                f"got={list(message.joint_names)}"
            )
            return
        if not message.points or len(message.points[-1].positions) != len(RT_CONTROL_JOINT_NAMES):
            self.last_error = "invalid_command_point"
            return
        positions = [float(value) for value in message.points[-1].positions]
        if not all(math.isfinite(value) for value in positions):
            self.last_error = "non_finite_command"
            return
        self.target = dict(zip(RT_CONTROL_JOINT_NAMES, positions))
        self.last_command_monotonic = time.monotonic()
        self.last_error = "ok"

    def control_step(self) -> None:
        now = time.monotonic()
        nominal_dt = 1.0 / max(float(self.get_parameter("control_rate_hz").value), 1.0)
        dt = clamp(now - self.last_control_monotonic, nominal_dt * 0.25, nominal_dt * 4.0)
        self.last_control_monotonic = now
        self.control_ticks += 1

        timeout = float(self.get_parameter("command_timeout_s").value)
        command_fresh = (
            self.target is not None
            and self.last_command_monotonic is not None
            and now - self.last_command_monotonic <= timeout
        )
        gain = float(self.get_parameter("position_gain").value)
        max_arm_velocity = float(self.get_parameter("max_arm_velocity_rad_s").value)
        max_arm_acceleration = float(
            self.get_parameter("max_arm_acceleration_rad_s2").value
        )
        max_updown_velocity = float(
            self.get_parameter("max_updown_velocity_m_s").value
        )
        max_updown_acceleration = float(
            self.get_parameter("max_updown_acceleration_m_s2").value
        )

        self.max_tracking_error = 0.0
        for name in RT_CONTROL_JOINT_NAMES:
            max_velocity = max_updown_velocity if name == "updown" else max_arm_velocity
            max_acceleration = (
                max_updown_acceleration if name == "updown" else max_arm_acceleration
            )
            if command_fresh and self.target is not None:
                error = self.target[name] - self.position[name]
                desired_velocity = clamp(gain * error, -max_velocity, max_velocity)
                self.max_tracking_error = max(self.max_tracking_error, abs(error))
            else:
                desired_velocity = 0.0

            velocity_delta = clamp(
                desired_velocity - self.velocity[name],
                -max_acceleration * dt,
                max_acceleration * dt,
            )
            next_velocity = self.velocity[name] + velocity_delta
            if command_fresh and self.target is not None:
                error = self.target[name] - self.position[name]
                step = next_velocity * dt
                if abs(step) > abs(error) and step * error > 0.0:
                    step = error
                    next_velocity = 0.0
                self.position[name] += step
            else:
                self.position[name] += next_velocity * dt
            self.velocity[name] = next_velocity

        window_s = now - self.control_window_start
        if window_s >= 1.0:
            self.measured_control_hz = self.control_ticks / window_s
            self.control_ticks = 0
            self.control_window_start = now

    def publish_feedback(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(RT_CONTROL_JOINT_NAMES)
        message.position = [self.position[name] for name in RT_CONTROL_JOINT_NAMES]
        message.velocity = [self.velocity[name] for name in RT_CONTROL_JOINT_NAMES]
        self.feedback_pub.publish(message)

    def publish_status(self) -> None:
        now = time.monotonic()
        timeout = float(self.get_parameter("command_timeout_s").value)
        command_age = (
            now - self.last_command_monotonic
            if self.last_command_monotonic is not None
            else math.inf
        )
        status = {
            "control_hz": self.measured_control_hz,
            "command_age_ms": command_age * 1000.0 if math.isfinite(command_age) else None,
            "watchdog_hold": command_age > timeout,
            "max_tracking_error": self.max_tracking_error,
            "status": self.last_error,
            "interface_joint_count": len(RT_CONTROL_JOINT_NAMES),
        }
        message = String()
        message.data = json.dumps(status, separators=(",", ":"))
        self.status_pub.publish(message)


def main() -> None:
    rclpy.init()
    node = MockRtControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
