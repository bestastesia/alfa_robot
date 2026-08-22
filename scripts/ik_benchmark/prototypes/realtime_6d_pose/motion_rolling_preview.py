#!/usr/bin/python3
"""PROTOTYPE: render the rolling batch at the Motion -> RT-Control boundary.

Question: after a dragged 6D pose passes Motion's IK/jump/collision gates, what
exact 10/30 Hz RollingJointTargetBatch would Motion publish?  This node only
builds and exposes that message.  It creates no /rt endpoint, consumes no
RT-Control state, and does not simulate controller tracking or ACKs.
"""

from __future__ import annotations

import json
import math
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from unique_identifier_msgs.msg import UUID

from robot_motion_interfaces.msg import RollingJointPoint, RollingJointTargetBatch

from rolling_suffix import (
    AXIS_NAMES,
    MOTION_PREVIEW_LIMITS,
    PROTOCOL_MAJOR,
    PROTOCOL_MINOR,
    PlanDiagnostics,
    RollingTargetPlanner,
    RollingSuffix,
    SuffixPlanningError,
    build_hold_suffix,
)


TX_TOPIC = "/realtime_6d_pose/motion_tx_preview"
TX_JOINT_TOPIC = "/realtime_6d_pose/motion_tx_preview_joint_states"
TX_STATUS_TOPIC = "/realtime_6d_pose/motion_tx_status"


def uuid_message(value: uuid.UUID) -> UUID:
    message = UUID()
    message.uuid = list(value.bytes)
    return message


class MotionRollingPreview(Node):
    """Generate real rolling-message shapes without an RT-Control backend."""

    def __init__(self) -> None:
        super().__init__("realtime_6d_pose_motion_tx_preview")
        arm = str(self.declare_parameter("arm", "left").value)
        self.batch_rate_hz = float(self.declare_parameter("batch_rate_hz", 30.0).value)
        self.knot_ns = int(
            float(self.declare_parameter("knot_period_ms", 100.0).value) * 1e6
        )
        self.horizon_ns = int(
            float(self.declare_parameter("planned_horizon_ms", 500.0).value) * 1e6
        )
        self.replace_guard_ns = int(
            float(self.declare_parameter("replace_guard_ms", 24.0).value) * 1e6
        )
        self.limit_fraction = float(
            self.declare_parameter("limit_fraction", 1.0).value
        )
        initial_arm_deg = [
            float(value)
            for value in self.declare_parameter(
                "initial_arm_joints_deg", [0.0, -45.0, 120.0, -75.0, 0.0, 0.0]
            ).value
        ]
        fixed_updown = float(self.declare_parameter("fixed_updown", 0.3).value)
        if self.batch_rate_hz <= 0.0:
            raise ValueError("batch_rate_hz must be positive")
        if self.knot_ns <= 0 or self.horizon_ns < self.knot_ns:
            raise ValueError("rolling horizon must contain at least one knot interval")
        if len(initial_arm_deg) != 6:
            raise ValueError("initial_arm_joints_deg must contain six values")
        if arm == "right":
            self.active_indices = tuple(range(0, 6))
        elif arm == "left":
            self.active_indices = tuple(range(6, 12))
        else:
            raise ValueError("arm must be left or right")

        initial_arm = [math.radians(value) for value in initial_arm_deg]
        self.initial_positions = tuple(initial_arm + initial_arm + [0.0, fixed_updown])
        self.arm = arm
        self.target_planner = RollingTargetPlanner(
            active_indices=self.active_indices,
            limits=MOTION_PREVIEW_LIMITS,
            knot_ns=self.knot_ns,
            horizon_ns=self.horizon_ns,
            limit_fraction=self.limit_fraction,
        )
        self.latest_target: tuple[float, ...] | None = None
        self.latest_target_at: float | None = None
        self.last_tx: RollingSuffix | None = None
        self.next_sequence = 1
        self.local_reject_count = 0
        self.last_error = "none"
        self.session_started_ns = time.monotonic_ns()
        self.window_started = time.monotonic()
        self.window_batches = 0
        self.measured_batch_hz = 0.0
        self.last_diagnostics = PlanDiagnostics(0.0, 0.0, 0.0)
        # These identities only make the preview message structurally complete.
        # They are never negotiated with RT-Control and never leave the preview topic.
        namespace = uuid.UUID("5de74476-23de-4f8c-8f1d-48705a637418")
        self.controller_boot_id = uuid.uuid5(namespace, "motion-tx-preview-controller")
        self.session_id = uuid.uuid5(namespace, "motion-tx-preview-session")
        self.client_id = uuid.uuid5(namespace, f"motion-tx-preview-{arm}")

        self.batch_pub = self.create_publisher(
            RollingJointTargetBatch, TX_TOPIC, qos_profile_sensor_data
        )
        self.preview_joint_pub = self.create_publisher(
            JointState, TX_JOINT_TOPIC, qos_profile_sensor_data
        )
        self.status_pub = self.create_publisher(
            String, TX_STATUS_TOPIC, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState,
            "/realtime_6d_pose/commanded_joint_states",
            self.on_motion_target,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / self.batch_rate_hz, self.on_batch_tick)
        self.get_logger().info(
            "PROTOTYPE Motion TX only: "
            f"arm={arm} requested_batch={self.batch_rate_hz:.1f}Hz "
            f"topic={TX_TOPIC}; no /rt endpoints and no controller simulation"
        )

    def on_motion_target(self, message: JointState) -> None:
        values = {name: float(value) for name, value in zip(message.name, message.position)}
        if any(name not in values or not math.isfinite(values[name]) for name in AXIS_NAMES):
            self.last_error = "invalid_motion_target"
            return
        self.latest_target = tuple(values[name] for name in AXIS_NAMES)
        self.latest_target_at = time.monotonic()

    def relative_now_ns(self) -> int:
        return max(0, time.monotonic_ns() - self.session_started_ns)

    def on_batch_tick(self) -> None:
        replace_from_ns = self.relative_now_ns() + self.replace_guard_ns
        sequence = self.next_sequence
        self.next_sequence += 1
        try:
            if self.last_tx is None:
                candidate = build_hold_suffix(
                    sequence=sequence,
                    replace_from_ns=replace_from_ns,
                    hold_positions=self.initial_positions,
                    knot_ns=self.knot_ns,
                    horizon_ns=self.horizon_ns,
                )
                diagnostics = PlanDiagnostics(0.0, 0.0, 0.0)
                source = "initial_hold"
            else:
                replace_from_ns = max(replace_from_ns, self.last_tx.replace_from_ns)
                if self.latest_target is None:
                    target, _ = self.last_tx.sample(replace_from_ns)
                    source = "last_tx_hold"
                else:
                    target = self.latest_target
                    source = "motion_safe_target"
                candidate, diagnostics = self.target_planner.plan(
                    accepted=self.last_tx,
                    sequence=sequence,
                    replace_from_ns=replace_from_ns,
                    target_positions=target,
                )
        except (SuffixPlanningError, ValueError) as error:
            self.local_reject_count += 1
            self.last_error = f"local_plan_reject:{error}"
            self.publish_status(None, None, "rejected")
            return

        # Chaining from the last transmitted suffix is a Motion-side preview
        # assumption.  It is deliberately not labelled as an RT ACK.
        self.last_tx = candidate
        self.last_diagnostics = diagnostics
        self.last_error = "none"
        self.publish_batch(candidate)
        self.publish_preview_joint_state(candidate)
        self.update_measured_rate()
        self.publish_status(candidate, diagnostics, source)
        log_every = max(1, int(round(self.batch_rate_hz / 2.0)))
        if candidate.sequence == 1 or candidate.sequence % log_every == 0:
            self.get_logger().info(
                "Motion TX "
                f"seq={candidate.sequence} points={len(candidate.points)} "
                f"replace={candidate.replace_from_ns * 1e-6:.1f}ms "
                f"horizon={(candidate.buffered_until_ns - candidate.replace_from_ns) * 1e-6:.1f}ms "
                f"scale={diagnostics.target_scale:.3f} measured={self.measured_batch_hz:.1f}Hz"
            )

    def publish_batch(self, suffix: RollingSuffix) -> None:
        message = RollingJointTargetBatch()
        message.protocol_major = PROTOCOL_MAJOR
        message.protocol_minor = PROTOCOL_MINOR
        message.controller_boot_id = uuid_message(self.controller_boot_id)
        message.session_id = uuid_message(self.session_id)
        message.client_instance_id = uuid_message(self.client_id)
        message.sequence = suffix.sequence
        message.replace_from_ns = suffix.replace_from_ns
        for source in suffix.points:
            point = RollingJointPoint()
            point.time_from_session_start_ns = source.time_ns
            point.positions = list(source.positions)
            point.velocities = list(source.velocities)
            message.points.append(point)
        self.batch_pub.publish(message)

    def publish_preview_joint_state(self, suffix: RollingSuffix) -> None:
        endpoint = suffix.points[-1]
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(AXIS_NAMES)
        message.position = list(endpoint.positions)
        message.velocity = list(endpoint.velocities)
        self.preview_joint_pub.publish(message)

    def update_measured_rate(self) -> None:
        self.window_batches += 1
        now = time.monotonic()
        elapsed = now - self.window_started
        if elapsed >= 1.0:
            self.measured_batch_hz = self.window_batches / elapsed
            self.window_batches = 0
            self.window_started = now

    def publish_status(
        self,
        suffix: RollingSuffix | None,
        diagnostics: PlanDiagnostics | None,
        source: str,
    ) -> None:
        command_age_ms = (
            (time.monotonic() - self.latest_target_at) * 1000.0
            if self.latest_target_at is not None
            else None
        )
        points = []
        if suffix is not None:
            for point in suffix.points:
                points.append(
                    {
                        "time_ms": point.time_ns * 1e-6,
                        "active_positions_deg": [
                            math.degrees(point.positions[index])
                            for index in self.active_indices
                        ],
                        "active_velocities_deg_s": [
                            math.degrees(point.velocities[index])
                            for index in self.active_indices
                        ],
                    }
                )
        payload = {
            "stage": "motion_tx_preview",
            "rt_control_present": False,
            "topic": TX_TOPIC,
            "message_type": "robot_motion_interfaces/msg/RollingJointTargetBatch",
            "identity_source": "preview_placeholders_not_negotiated",
            "limits_source": "motion_moveit_preview_not_rt_negotiated",
            "source": source,
            "requested_batch_hz": self.batch_rate_hz,
            "measured_batch_hz": self.measured_batch_hz,
            "sequence": suffix.sequence if suffix is not None else self.next_sequence - 1,
            "replace_from_ns": suffix.replace_from_ns if suffix is not None else None,
            "buffered_until_ns": suffix.buffered_until_ns if suffix is not None else None,
            "point_count": len(suffix.points) if suffix is not None else 0,
            "knot_period_ms": self.knot_ns * 1e-6,
            "horizon_ms": self.horizon_ns * 1e-6,
            "command_age_ms": command_age_ms,
            "target_scale": diagnostics.target_scale if diagnostics else 0.0,
            "planned_peak_velocity": diagnostics.peak_velocity if diagnostics else 0.0,
            "planned_peak_acceleration": diagnostics.peak_acceleration if diagnostics else 0.0,
            "profile_duration_s": diagnostics.profile_duration_s if diagnostics else 0.0,
            "profile_replanned": diagnostics.replanned if diagnostics else False,
            "local_reject_count": self.local_reject_count,
            "last_error": self.last_error,
            "active_joint_names": [AXIS_NAMES[index] for index in self.active_indices],
            "points": points,
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.status_pub.publish(message)


def main() -> None:
    rclpy.init()
    node = MotionRollingPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
