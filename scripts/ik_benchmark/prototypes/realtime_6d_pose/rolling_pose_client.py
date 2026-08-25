#!/usr/bin/python3
"""PROTOTYPE ELECTRI-102 rolling client with mock and guarded real backends.

The Motion C++ node owns pose -> IK -> jump/collision checking.  This process
owns only the accepted-future protocol: 30 Hz replacement planning, ACK-based
baseline promotion, mock 250 Hz sampling, and the public RT-Control adapter.
"""

from __future__ import annotations

from dataclasses import dataclass
from collections import deque
import json
import math
import os
import signal
import time
import uuid
from typing import Any, Sequence

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from rolling_suffix import (
    AXIS_COUNT,
    AXIS_NAMES,
    AXIS_SET_SHA256,
    EXPECTED_LIMITS_SHA256,
    PROTOCOL_MAJOR,
    PROTOCOL_MINOR,
    PROVISIONAL_LIMITS,
    PlanDiagnostics,
    RollingPoint,
    RollingSuffix,
    RollingTargetPlanner,
    SuffixPlanningError,
    build_hold_suffix,
    choose_replace_from_ns,
    target_tracking_allowed,
)


REAL_CONFIRMATION = "ELECTRI_102_SUPERVISED_ROLLING"


@dataclass
class OpenSettings:
    hold_positions: tuple[float, ...]
    hold_velocities: tuple[float, ...]
    initial_replaceable_from_ns: int
    required_horizon_ns: int
    max_horizon_ns: int
    replace_lead_ns: int
    update_timeout_ns: int
    controller_period_ns: int
    buffer_capacity: int
    transport_max_points: int
    limits_source: str
    test_only_limits: bool
    limits_version_hex: str


@dataclass
class BackendView:
    mode: str = "FJT_READY"
    session_state: str = "NONE"
    has_session: bool = False
    execution_time_ns: int = 0
    replaceable_from_ns: int = 0
    buffered_until_ns: int = 0
    available_horizon_ns: int = 0
    last_seen_sequence: int = 0
    last_accepted_sequence: int = 0
    last_rejected_sequence: int = 0
    last_reject: str = "NONE"
    stop_reason: str = "NONE"
    accepted_count: int = 0
    rejected_count: int = 0
    timeout_count: int = 0
    buffer_point_count: int = 0
    status: str = "starting"


SESSION_NAMES = {
    0: "NONE",
    1: "PRIMING",
    2: "RUNNING",
    3: "STOPPING",
    4: "HOLDING",
    5: "TERMINATED",
}
MODE_NAMES = {0: "DISABLED", 1: "FJT_READY", 2: "ROLLING_READY", 3: "RESTART_REQUIRED"}
REJECT_NAMES = {
    0: "NONE",
    1: "WRONG_PROTOCOL",
    2: "WRONG_BOOT",
    3: "WRONG_SESSION",
    4: "WRONG_CLIENT",
    5: "STALE_SEQUENCE",
    6: "INVALID_SHAPE",
    7: "NON_FINITE",
    8: "NON_MONOTONIC_TIME",
    9: "LATE_REPLACE",
    10: "TIME_GAP",
    11: "CAPACITY_EXCEEDED",
    12: "INSUFFICIENT_HORIZON",
    13: "POSITION_DISCONTINUITY",
    14: "VELOCITY_DISCONTINUITY",
    15: "POSITION_LIMIT",
    16: "VELOCITY_LIMIT",
    17: "ACCELERATION_LIMIT",
    18: "NOT_STOPPING_VIABLE",
    19: "SESSION_NOT_ACCEPTING",
    20: "HORIZON_EXCEEDED",
}
STOP_NAMES = {
    0: "NONE",
    1: "GRACEFUL_CLOSE",
    2: "PRIME_TIMEOUT",
    3: "UPDATE_TIMEOUT",
    4: "LOW_WATER",
    5: "CLOCK_ANOMALY",
    6: "INTERNAL_INVARIANT",
    7: "CONTROLLER_DEACTIVATED",
    8: "DISABLE",
    9: "GROUP_FAULT",
    10: "CONTROLLER_RESTART",
}


def _uuid_message(message_type: type, value: uuid.UUID) -> Any:
    message = message_type()
    message.uuid = list(value.bytes)
    return message


def _uuid_hex(message: Any) -> str:
    return bytes(message.uuid).hex()


class MockRollingBackend:
    """In-process semantic mock; no public /rt endpoint is created."""

    def __init__(self, initial_positions: Sequence[float]) -> None:
        self.settings = OpenSettings(
            hold_positions=tuple(float(value) for value in initial_positions),
            hold_velocities=(0.0,) * AXIS_COUNT,
            initial_replaceable_from_ns=16_000_000,
            required_horizon_ns=500_000_000,
            max_horizon_ns=600_000_000,
            replace_lead_ns=16_000_000,
            update_timeout_ns=200_000_000,
            controller_period_ns=4_000_000,
            buffer_capacity=64,
            transport_max_points=256,
            limits_source="PROVISIONAL",
            test_only_limits=False,
            limits_version_hex=EXPECTED_LIMITS_SHA256,
        )
        self.view = BackendView(
            mode="ROLLING_READY",
            session_state="PRIMING",
            has_session=True,
            replaceable_from_ns=self.settings.initial_replaceable_from_ns,
            status="mock_open",
        )
        self.started_at = time.monotonic()
        self.last_update_at: float | None = None
        self.replacements: list[RollingSuffix] = []
        self.actual_positions = list(self.settings.hold_positions)
        self.actual_velocities = [0.0] * AXIS_COUNT
        self.desired_positions = list(self.settings.hold_positions)
        self.desired_velocities = [0.0] * AXIS_COUNT
        self.max_tracking_error = 0.0
        self.last_tick = time.monotonic()
        self.control_ticks = 0
        self.control_window_at = self.last_tick
        self.control_hz = 0.0
        self.closing = False
        self.closed = False

    def mark_actual_ready(self) -> None:
        pass

    def public_state_age_ns(self) -> int:
        return 0

    def poll(self) -> None:
        now = time.monotonic()
        execution_ns = max(0, int((now - self.started_at) * 1e9))
        self.view.execution_time_ns = execution_ns
        self.view.replaceable_from_ns = execution_ns + self.settings.replace_lead_ns
        if self.replacements:
            latest = self.replacements[-1]
            self.view.buffered_until_ns = latest.buffered_until_ns
            self.view.available_horizon_ns = max(0, latest.buffered_until_ns - execution_ns)
            self.view.buffer_point_count = len(latest.points)
        if (
            self.view.session_state == "RUNNING"
            and self.last_update_at is not None
            and now - self.last_update_at > self.settings.update_timeout_ns * 1e-9
        ):
            self.view.session_state = "HOLDING"
            self.view.stop_reason = "UPDATE_TIMEOUT"
            self.view.timeout_count += 1
            self.view.status = "mock_update_timeout"

    def submit(self, suffix: RollingSuffix) -> bool:
        self.poll()
        previous_last_seen = self.view.last_seen_sequence
        reason = self._validate(suffix, previous_last_seen)
        self.view.last_seen_sequence = max(self.view.last_seen_sequence, suffix.sequence)
        if reason != "NONE":
            self.view.last_rejected_sequence = suffix.sequence
            self.view.last_reject = reason
            self.view.rejected_count += 1
            self.view.status = f"mock_reject:{reason}"
            return False
        self.replacements.append(suffix)
        self.view.last_accepted_sequence = suffix.sequence
        self.view.last_reject = "NONE"
        self.view.accepted_count += 1
        self.view.session_state = "RUNNING"
        self.view.status = "mock_running"
        self.last_update_at = time.monotonic()
        self.poll()
        return True

    def _validate(self, suffix: RollingSuffix, previous_last_seen: int) -> str:
        if self.view.session_state not in {"PRIMING", "RUNNING"}:
            return "SESSION_NOT_ACCEPTING"
        if suffix.sequence <= previous_last_seen:
            return "STALE_SEQUENCE"
        if len(suffix.points) > min(self.settings.buffer_capacity, self.settings.transport_max_points):
            return "CAPACITY_EXCEEDED"
        if (
            self.view.session_state == "PRIMING"
            and suffix.replace_from_ns != self.settings.initial_replaceable_from_ns
        ):
            return "LATE_REPLACE"
        if (
            self.view.session_state == "RUNNING"
            and suffix.replace_from_ns < self.view.replaceable_from_ns
        ):
            return "LATE_REPLACE"
        horizon_from_execution = suffix.buffered_until_ns - self.view.execution_time_ns
        if horizon_from_execution > self.settings.max_horizon_ns:
            return "HORIZON_EXCEEDED"
        if not self.replacements and suffix.buffered_until_ns - suffix.replace_from_ns < self.settings.required_horizon_ns:
            return "INSUFFICIENT_HORIZON"
        if self.replacements:
            expected_q, expected_v = self.replacements[-1].sample(suffix.replace_from_ns)
            if max(abs(a - b) for a, b in zip(expected_q, suffix.points[0].positions)) > 1e-8:
                return "POSITION_DISCONTINUITY"
            if max(abs(a - b) for a, b in zip(expected_v, suffix.points[0].velocities)) > 1e-8:
                return "VELOCITY_DISCONTINUITY"
        return "NONE"

    def control_tick(self) -> None:
        self.poll()
        now = time.monotonic()
        nominal_dt = self.settings.controller_period_ns * 1e-9
        dt = min(max(now - self.last_tick, nominal_dt * 0.25), nominal_dt * 4.0)
        self.last_tick = now
        self.control_ticks += 1
        selected: RollingSuffix | None = None
        for candidate in self.replacements:
            if candidate.replace_from_ns <= self.view.execution_time_ns:
                selected = candidate
            else:
                break
        if selected is not None and self.view.session_state in {"RUNNING", "STOPPING"}:
            desired_q, desired_v = selected.sample(self.view.execution_time_ns)
            self.desired_positions = list(desired_q)
            self.desired_velocities = list(desired_v)
        elif self.view.session_state == "HOLDING":
            self.desired_positions = list(self.actual_positions)
            self.desired_velocities = [0.0] * AXIS_COUNT

        self.max_tracking_error = 0.0
        for axis in range(AXIS_COUNT):
            limit = PROVISIONAL_LIMITS[axis]
            error = self.desired_positions[axis] - self.actual_positions[axis]
            requested_velocity = self.desired_velocities[axis] + 20.0 * error
            requested_velocity = min(max(requested_velocity, -limit.velocity), limit.velocity)
            dv = min(
                max(requested_velocity - self.actual_velocities[axis], -limit.acceleration * dt),
                limit.acceleration * dt,
            )
            self.actual_velocities[axis] += dv
            self.actual_positions[axis] += self.actual_velocities[axis] * dt
            self.max_tracking_error = max(self.max_tracking_error, abs(error))
        window = now - self.control_window_at
        if window >= 1.0:
            self.control_hz = self.control_ticks / window
            self.control_ticks = 0
            self.control_window_at = now

    def begin_close(self) -> None:
        if self.closed:
            return
        self.closing = True
        self.view.session_state = "HOLDING"
        self.view.stop_reason = "GRACEFUL_CLOSE"
        self.view.status = "mock_holding"

    def close_poll(self) -> None:
        if not self.closing or self.closed:
            return
        self.view.session_state = "TERMINATED"
        self.view.has_session = False
        self.view.mode = "FJT_READY"
        self.view.status = "mock_closed"
        self.closed = True


class RealRollingBackend:
    """Public-IDL adapter.  Command publication is gated twice and off by default."""

    def __init__(
        self,
        node: Node,
        *,
        allow_command: bool,
        allow_provisional: bool,
        expected_limits_sha256: str,
    ) -> None:
        try:
            from robot_interfaces_qos import rolling_command, rolling_state
            from robot_motion_interfaces.msg import RollingJointPoint, RollingJointTargetBatch
            from robot_rt_control_interfaces.msg import RollingJointControlState
            from robot_rt_control_interfaces.srv import (
                CloseRollingJointSession,
                OpenRollingJointSession,
                SetJointControlMode,
            )
            from unique_identifier_msgs.msg import UUID
        except ImportError as error:
            raise RuntimeError(
                "real backend requires robot_interfaces commit "
                "9cc937970736cd19fd3bf5283de8cc5c15926967 in the sourced overlay"
            ) from error

        self.node = node
        self.RollingJointPoint = RollingJointPoint
        self.RollingJointTargetBatch = RollingJointTargetBatch
        self.RollingJointControlState = RollingJointControlState
        self.CloseRollingJointSession = CloseRollingJointSession
        self.OpenRollingJointSession = OpenRollingJointSession
        self.SetJointControlMode = SetJointControlMode
        self.UUID = UUID
        self.allow_command = allow_command
        self.allow_provisional = allow_provisional
        self.expected_limits_sha256 = expected_limits_sha256.lower()
        self.client_uuid = uuid.uuid4()
        self.controller_boot_id: Any | None = None
        self.session_id: Any | None = None
        self.settings: OpenSettings | None = None
        self.view = BackendView(status="real_observe_only" if not allow_command else "real_waiting")
        self.actual_ready = False
        self.latest_actual_positions: tuple[float, ...] | None = None
        self.latest_actual_at: float | None = None
        self.takeover_positions: tuple[float, ...] | None = None
        self.feedback_stale_s = 0.100
        self.rotary_takeover_tolerance = math.radians(0.25)
        self.updown_takeover_tolerance = 0.001
        self.fatal_reason: str | None = None
        self.phase = "OBSERVE" if not allow_command else "WAIT_SERVICES"
        self.future: Any | None = None
        self.closed = False
        self.closing = False
        self.raw_state: Any | None = None
        self.state_received_at: float | None = None
        self.observed_matching_session = False

        self.mode_client = node.create_client(
            SetJointControlMode, "/rt/joint_control/set_mode"
        )
        self.open_client = node.create_client(
            OpenRollingJointSession, "/rt/rolling_joint_control/open"
        )
        self.close_client = node.create_client(
            CloseRollingJointSession, "/rt/rolling_joint_control/close"
        )
        self.command_pub = (
            node.create_publisher(
                RollingJointTargetBatch,
                "/rt/rolling_joint_control/update",
                rolling_command(),
            )
            if allow_command
            else None
        )
        self.state_sub = node.create_subscription(
            RollingJointControlState,
            "/rt/rolling_joint_control/state",
            self._on_state,
            rolling_state(),
        )

    def update_actual(
        self,
        positions: Sequence[float],
        *,
        stable: bool,
    ) -> None:
        self.latest_actual_positions = tuple(float(value) for value in positions)
        self.latest_actual_at = time.monotonic()
        self.actual_ready = bool(stable)

    def mark_actual_ready(self) -> None:
        # Kept for the common mock/real backend interface.  Real readiness is
        # established by update_actual() after a stable 14-axis sample window.
        pass

    def _actual_is_fresh(self) -> bool:
        return bool(
            self.latest_actual_at is not None
            and time.monotonic() - self.latest_actual_at <= self.feedback_stale_s
        )

    def public_state_age_ns(self) -> int:
        if self.state_received_at is None:
            raise SuffixPlanningError("rt_public_state_unavailable")
        return max(0, int((time.monotonic() - self.state_received_at) * 1e9))

    def poll(self) -> None:
        if self.phase == "OBSERVE" or self.closed:
            return
        if self.closing:
            self.close_poll()
            return
        if self.phase == "WAIT_SERVICES":
            if not self.actual_ready or not self._actual_is_fresh():
                self.view.status = "real_waiting_stable_joint_states"
                return
            if not all(
                client.service_is_ready()
                for client in (self.mode_client, self.open_client, self.close_client)
            ):
                self.view.status = "real_waiting_services"
                return
            assert self.latest_actual_positions is not None
            self.takeover_positions = self.latest_actual_positions
            request = self.SetJointControlMode.Request()
            request.protocol_major = PROTOCOL_MAJOR
            request.protocol_minor = PROTOCOL_MINOR
            request.client_instance_id = _uuid_message(self.UUID, self.client_uuid)
            request.request_id = _uuid_message(self.UUID, uuid.uuid4())
            request.expected_mode.value = request.expected_mode.FJT_READY
            request.target_mode.value = request.target_mode.ROLLING_READY
            self.future = self.mode_client.call_async(request)
            self.phase = "SET_MODE"
            self.view.status = "real_set_mode_pending"
            return
        if self.phase == "SET_MODE" and self.future is not None and self.future.done():
            response = self.future.result()
            if response is None or not response.accepted or response.result.value != response.result.NONE:
                result = "no_response" if response is None else str(response.result.value)
                self._fail(f"set_mode_rejected:{result}")
                return
            if (
                response.mode.value != response.mode.ROLLING_READY
                or not response.source_controller_deactivated
                or not response.target_controller_activated
                or response.restart_required
            ):
                self._fail("set_mode_postcondition_failed")
                return
            self.controller_boot_id = response.controller_boot_id
            request = self.OpenRollingJointSession.Request()
            request.protocol_major = PROTOCOL_MAJOR
            request.protocol_minor = PROTOCOL_MINOR
            request.client_instance_id = _uuid_message(self.UUID, self.client_uuid)
            request.request_id = _uuid_message(self.UUID, uuid.uuid4())
            request.expected_controller_boot_id = self.controller_boot_id
            request.axis_set_hash = list(bytes.fromhex(AXIS_SET_SHA256))
            self.future = self.open_client.call_async(request)
            self.phase = "OPEN"
            self.view.status = "real_open_pending"
            return
        if self.phase == "OPEN" and self.future is not None and self.future.done():
            response = self.future.result()
            if response is None or not response.accepted or response.result.value != response.result.NONE:
                result = "no_response" if response is None else str(response.result.value)
                self._fail(f"open_rejected:{result}")
                return
            if (
                response.protocol_major != PROTOCOL_MAJOR
                or response.protocol_minor != PROTOCOL_MINOR
                or response.session_state.value != response.session_state.PRIMING
                or bytes(response.axis_set_hash).hex() != AXIS_SET_SHA256
                or _uuid_hex(response.client_instance_id) != self.client_uuid.hex
            ):
                self._fail("open_identity_mismatch")
                return
            limits_hex = bytes(response.limits_version).hex()
            limits_source_value = int(response.limits_source.value)
            limits_source = {1: "PRODUCTION", 2: "TEST_ONLY", 3: "PROVISIONAL"}.get(
                limits_source_value, "UNSPECIFIED"
            )
            if limits_hex != self.expected_limits_sha256:
                self._fail(f"limits_hash_mismatch:{limits_hex}")
                return
            if (response.test_only_limits or limits_source != "PRODUCTION") and not self.allow_provisional:
                self._fail(f"non_production_limits_require_explicit_allow:{limits_source}")
                return
            self.controller_boot_id = response.controller_boot_id
            self.session_id = response.session_id
            self.settings = OpenSettings(
                hold_positions=tuple(float(value) for value in response.hold_positions),
                hold_velocities=tuple(float(value) for value in response.hold_velocities),
                initial_replaceable_from_ns=int(response.initial_replaceable_from_ns),
                required_horizon_ns=int(response.required_initial_horizon_ns),
                max_horizon_ns=int(response.max_horizon_ns),
                replace_lead_ns=int(response.replace_lead_ns),
                update_timeout_ns=int(response.update_timeout_ns),
                controller_period_ns=int(response.nominal_controller_period_ns),
                buffer_capacity=int(response.buffer_capacity),
                transport_max_points=int(response.transport_max_points),
                limits_source=limits_source,
                test_only_limits=bool(response.test_only_limits),
                limits_version_hex=limits_hex,
            )
            if self.takeover_positions is None or not self._actual_is_fresh():
                self._fail("takeover_feedback_stale_after_open")
                return
            latest = self.latest_actual_positions
            assert latest is not None
            rotary_error = max(
                abs(expected - actual)
                for expected, actual in zip(
                    self.settings.hold_positions[:13], latest[:13]
                )
            )
            updown_error = abs(
                self.settings.hold_positions[13] - latest[13]
            )
            captured_rotary_error = max(
                abs(expected - actual)
                for expected, actual in zip(
                    self.settings.hold_positions[:13], self.takeover_positions[:13]
                )
            )
            captured_updown_error = abs(
                self.settings.hold_positions[13] - self.takeover_positions[13]
            )
            hold_speed = max(abs(value) for value in self.settings.hold_velocities)
            if (
                rotary_error > self.rotary_takeover_tolerance
                or captured_rotary_error > self.rotary_takeover_tolerance
                or updown_error > self.updown_takeover_tolerance
                or captured_updown_error > self.updown_takeover_tolerance
                or hold_speed > 1e-6
            ):
                self._fail(
                    "takeover_pose_mismatch:"
                    f"latest_rotary={math.degrees(rotary_error):.4f}deg,"
                    f"captured_rotary={math.degrees(captured_rotary_error):.4f}deg,"
                    f"latest_updown={updown_error * 1000.0:.3f}mm,"
                    f"captured_updown={captured_updown_error * 1000.0:.3f}mm,"
                    f"hold_speed={hold_speed:.6g}"
                )
                return
            self.view.mode = "ROLLING_READY"
            self.view.session_state = "PRIMING"
            self.view.has_session = True
            self.view.replaceable_from_ns = self.settings.initial_replaceable_from_ns
            self.view.status = "real_open"
            self.phase = "READY"

    def submit(self, suffix: RollingSuffix) -> bool:
        if self.phase != "READY" or self.settings is None or self.command_pub is None:
            return False
        message = self.RollingJointTargetBatch()
        message.protocol_major = PROTOCOL_MAJOR
        message.protocol_minor = PROTOCOL_MINOR
        message.controller_boot_id = self.controller_boot_id
        message.session_id = self.session_id
        message.client_instance_id = _uuid_message(self.UUID, self.client_uuid)
        message.sequence = suffix.sequence
        message.replace_from_ns = suffix.replace_from_ns
        for source in suffix.points:
            point = self.RollingJointPoint()
            point.time_from_session_start_ns = source.time_ns
            point.positions = list(source.positions)
            point.velocities = list(source.velocities)
            message.points.append(point)
        self.command_pub.publish(message)
        self.view.status = "real_batch_published_waiting_ack"
        return True

    def _on_state(self, message: Any) -> None:
        if message.protocol_major != PROTOCOL_MAJOR or message.protocol_minor != PROTOCOL_MINOR:
            self._fail("state_protocol_mismatch")
            return
        if (
            self.controller_boot_id is not None
            and _uuid_hex(message.controller_boot_id) != _uuid_hex(self.controller_boot_id)
        ):
            self._fail("state_boot_identity_mismatch")
            return
        if self.session_id is not None and bool(message.has_session):
            if _uuid_hex(message.session_id) != _uuid_hex(self.session_id):
                self._fail("state_session_identity_mismatch")
                return
            if _uuid_hex(message.client_instance_id) != self.client_uuid.hex:
                self._fail("state_client_identity_mismatch")
                return
            self.observed_matching_session = True
        elif (
            self.session_id is not None
            and self.phase == "READY"
            and self.observed_matching_session
        ):
            self._fail("rolling_session_disappeared")
            return
        if self.settings is not None and bool(message.has_session):
            if bytes(message.axis_set_hash).hex() != AXIS_SET_SHA256:
                self._fail("state_axis_hash_mismatch")
                return
            if bytes(message.limits_version).hex() != self.settings.limits_version_hex:
                self._fail("state_limits_hash_mismatch")
                return
        self.state_received_at = time.monotonic()
        self.raw_state = message
        self.view = BackendView(
            mode=MODE_NAMES.get(int(message.control_mode.value), f"UNKNOWN_{message.control_mode.value}"),
            session_state=SESSION_NAMES.get(
                int(message.session_state.value), f"UNKNOWN_{message.session_state.value}"
            ),
            has_session=bool(message.has_session),
            execution_time_ns=int(message.execution_time_ns),
            replaceable_from_ns=int(message.replaceable_from_ns),
            buffered_until_ns=int(message.buffered_until_ns),
            available_horizon_ns=int(message.available_horizon_ns),
            last_seen_sequence=int(message.last_seen_sequence),
            last_accepted_sequence=int(message.last_accepted_sequence),
            last_rejected_sequence=int(message.last_rejected_sequence),
            last_reject=REJECT_NAMES.get(int(message.last_reject.value), str(message.last_reject.value)),
            stop_reason=STOP_NAMES.get(int(message.stop_reason.value), str(message.stop_reason.value)),
            accepted_count=int(message.accepted_count),
            rejected_count=int(message.rejected_count),
            timeout_count=int(message.timeout_count),
            buffer_point_count=int(message.buffer_point_count),
            status=self.view.status,
        )

    def _fail(self, reason: str) -> None:
        self.fatal_reason = reason
        self.view.status = f"real_error:{reason}"
        self.node.get_logger().error(self.view.status)
        if self.allow_command and not self.closing:
            if self.session_id is not None:
                self.begin_close()
                return
            if self.controller_boot_id is not None and self.phase in {"OPEN", "READY"}:
                self.closing = True
                self._request_mode_back()
                return
        self.phase = "FAILED"

    def begin_close(self) -> None:
        if self.closed or self.closing:
            return
        self.closing = True
        if self.phase == "OBSERVE":
            self.closed = True
            return
        if self.session_id is None:
            self._request_mode_back()
            return
        request = self.CloseRollingJointSession.Request()
        request.protocol_major = PROTOCOL_MAJOR
        request.protocol_minor = PROTOCOL_MINOR
        request.controller_boot_id = self.controller_boot_id
        request.session_id = self.session_id
        request.client_instance_id = _uuid_message(self.UUID, self.client_uuid)
        request.request_id = _uuid_message(self.UUID, uuid.uuid4())
        request.operation = request.REQUEST_STOP
        self.future = self.close_client.call_async(request)
        self.phase = "REQUEST_STOP"
        self.view.status = "real_request_stop_pending"

    def close_poll(self) -> None:
        if self.closed:
            return
        if self.phase == "REQUEST_STOP" and self.future is not None and self.future.done():
            response = self.future.result()
            if response is None or not response.accepted:
                self._fail("request_stop_failed")
                return
            self.phase = "WAIT_HOLDING"
            self.view.status = "real_wait_holding"
            return
        if self.phase == "WAIT_HOLDING" and self.view.session_state == "HOLDING":
            request = self.CloseRollingJointSession.Request()
            request.protocol_major = PROTOCOL_MAJOR
            request.protocol_minor = PROTOCOL_MINOR
            request.controller_boot_id = self.controller_boot_id
            request.session_id = self.session_id
            request.client_instance_id = _uuid_message(self.UUID, self.client_uuid)
            request.request_id = _uuid_message(self.UUID, uuid.uuid4())
            request.operation = request.FINALIZE
            self.future = self.close_client.call_async(request)
            self.phase = "FINALIZE"
            self.view.status = "real_finalize_pending"
            return
        if self.phase == "FINALIZE" and self.future is not None and self.future.done():
            response = self.future.result()
            if response is None or not response.accepted or not response.completed:
                self._fail("finalize_failed")
                return
            self._request_mode_back()
            return
        if self.phase == "MODE_BACK" and self.future is not None and self.future.done():
            response = self.future.result()
            if response is None or not response.accepted or response.mode.value != response.mode.FJT_READY:
                self._fail("mode_back_failed")
                return
            self.phase = "CLOSED"
            self.view.status = (
                f"real_closed_after_error:{self.fatal_reason}"
                if self.fatal_reason is not None
                else "real_closed"
            )
            self.closed = True

    def _request_mode_back(self) -> None:
        request = self.SetJointControlMode.Request()
        request.protocol_major = PROTOCOL_MAJOR
        request.protocol_minor = PROTOCOL_MINOR
        request.client_instance_id = _uuid_message(self.UUID, self.client_uuid)
        request.request_id = _uuid_message(self.UUID, uuid.uuid4())
        request.expected_mode.value = request.expected_mode.ROLLING_READY
        request.target_mode.value = request.target_mode.FJT_READY
        self.future = self.mode_client.call_async(request)
        self.phase = "MODE_BACK"
        self.view.status = "real_mode_back_pending"


class RollingPoseClient(Node):
    def __init__(self) -> None:
        super().__init__("realtime_6d_pose_rolling_client")
        backend_name = str(self.declare_parameter("backend", "mock").value)
        arm = str(self.declare_parameter("arm", "left").value)
        self.batch_rate_hz = float(self.declare_parameter("batch_rate_hz", 30.0).value)
        self.knot_ns = int(float(self.declare_parameter("knot_period_ms", 100.0).value) * 1e6)
        self.horizon_ns = int(float(self.declare_parameter("planned_horizon_ms", 500.0).value) * 1e6)
        # The state topic is nominally 20 ms.  Do not publish at the exact
        # reported frontier: it may have advanced before the batch arrives.
        self.replace_guard_ns = int(
            float(self.declare_parameter("replace_guard_ms", 24.0).value) * 1e6
        )
        self.limit_fraction = float(self.declare_parameter("limit_fraction", 0.60).value)
        self.command_stale_s = float(self.declare_parameter("command_stale_ms", 120.0).value) * 1e-3
        self.stable_sample_count = int(
            self.declare_parameter("stable_sample_count", 25).value
        )
        self.stable_rotary_span = math.radians(
            float(self.declare_parameter("stable_rotary_span_deg", 0.05).value)
        )
        self.stable_updown_span = float(
            self.declare_parameter("stable_updown_span_m", 0.0002).value
        )
        self.enable_target_tracking = bool(
            self.declare_parameter("enable_target_tracking", False).value
        )
        if self.stable_sample_count < 2:
            raise ValueError("stable_sample_count must be at least two")
        initial_arm_deg = [
            float(value)
            for value in self.declare_parameter(
                "initial_arm_joints_deg", [0.0, -45.0, 120.0, -75.0, 0.0, 0.0]
            ).value
        ]
        fixed_updown = float(self.declare_parameter("fixed_updown", 0.3).value)
        if len(initial_arm_deg) != 6:
            raise ValueError("initial_arm_joints_deg must contain six values")
        if arm == "right":
            self.active_indices = tuple(range(0, 6))
        elif arm == "left":
            self.active_indices = tuple(range(6, 12))
        else:
            raise ValueError("arm must be left or right")
        initial_arm = [math.radians(value) for value in initial_arm_deg]
        initial_positions = tuple(initial_arm + initial_arm + [0.0, fixed_updown])

        self.backend_name = backend_name
        allow_real_command = bool(self.declare_parameter("allow_real_command", False).value)
        allow_provisional = bool(self.declare_parameter("allow_provisional_limits", False).value)
        expected_limits = str(
            self.declare_parameter("expected_limits_sha256", EXPECTED_LIMITS_SHA256).value
        )
        if backend_name == "mock":
            self.backend: MockRollingBackend | RealRollingBackend = MockRollingBackend(
                initial_positions
            )
        elif backend_name == "real":
            environment_authorized = os.environ.get("ALFA_REAL_ROLLING_CONFIRM", "") == REAL_CONFIRMATION
            if allow_real_command and not environment_authorized:
                raise RuntimeError(
                    "allow_real_command also requires "
                    f"ALFA_REAL_ROLLING_CONFIRM={REAL_CONFIRMATION}"
                )
            self.backend = RealRollingBackend(
                self,
                allow_command=allow_real_command and environment_authorized,
                allow_provisional=allow_provisional,
                expected_limits_sha256=expected_limits,
            )
        else:
            raise ValueError("backend must be mock or real")

        self.allow_real_command = allow_real_command
        self.latest_target: tuple[float, ...] | None = None
        self.latest_target_at: float | None = None
        self.tracking_armed = False
        self.actual_positions: tuple[float, ...] | None = None
        self.actual_position_samples: deque[tuple[float, ...]] = deque(
            maxlen=self.stable_sample_count
        )
        self.actual_stable = False
        self.actual_stable_logged = False
        self.accepted: RollingSuffix | None = None
        self.pending: RollingSuffix | None = None
        self.pending_diagnostics: PlanDiagnostics | None = None
        self.next_sequence = 1
        self.published_count = 0
        self.promoted_count = 0
        self.local_reject_count = 0
        self.planner_error = "none"
        self.last_batch_at: float | None = None
        self.batch_ticks = 0
        self.batch_window_at = time.monotonic()
        self.measured_batch_hz = 0.0
        self.shutdown_requested = False
        self.shutdown_started_at: float | None = None
        self.planner = RollingTargetPlanner(
            active_indices=self.active_indices,
            limits=PROVISIONAL_LIMITS,
            knot_ns=self.knot_ns,
            horizon_ns=self.horizon_ns,
            limit_fraction=self.limit_fraction,
        )

        self.feedback_pub = self.create_publisher(
            JointState, "/realtime_6d_pose/joint_states", qos_profile_sensor_data
        )
        self.accepted_reference_pub = self.create_publisher(
            JointState,
            "/realtime_6d_pose/accepted_reference_joint_states",
            qos_profile_sensor_data,
        )
        self.status_pub = self.create_publisher(
            String, "/realtime_6d_pose/rt_status", qos_profile_sensor_data
        )
        self.create_subscription(
            JointState,
            "/realtime_6d_pose/commanded_joint_states",
            self._on_motion_target,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            "/realtime_6d_pose/target_pose_cmd",
            self._on_target_pose_command,
            qos_profile_sensor_data,
        )
        if backend_name == "real":
            try:
                from robot_interfaces_qos import fast_state

                joint_qos = fast_state()
            except ImportError:
                joint_qos = qos_profile_sensor_data
            self.real_joint_sub = self.create_subscription(
                JointState, "/joint_states", self._on_real_joint_state, joint_qos
            )
        else:
            self.real_joint_sub = None

        if backend_name == "mock":
            controller_period = 0.004
            self.create_timer(controller_period, self._control_tick)
            self.create_timer(1.0 / 125.0, self._feedback_tick)
        self.create_timer(0.01, self._session_tick)
        self.create_timer(1.0 / max(1.0, self.batch_rate_hz), self._batch_tick)
        self.create_timer(0.1, self._status_tick)
        self.get_logger().info(
            "PROTOTYPE rolling client: "
            f"backend={backend_name} arm={arm} batch={self.batch_rate_hz:.1f}Hz "
            f"knot={self.knot_ns * 1e-6:.0f}ms "
            f"horizon={self.horizon_ns * 1e-6:.0f}ms "
            f"real_command={str(allow_real_command).lower()} "
            f"target_tracking={str(self.enable_target_tracking).lower()}"
        )

    def _on_motion_target(self, message: JointState) -> None:
        values = {name: float(value) for name, value in zip(message.name, message.position)}
        if any(name not in values or not math.isfinite(values[name]) for name in AXIS_NAMES):
            self.planner_error = "invalid_motion_target"
            return
        self.latest_target = tuple(values[name] for name in AXIS_NAMES)
        self.latest_target_at = time.monotonic()

    def _on_target_pose_command(self, message: PoseStamped) -> None:
        if message.header.frame_id not in ("", "base_link"):
            return
        if not self.tracking_armed:
            self.tracking_armed = True
            self.get_logger().info(
                "first external 6D target received; joint target tracking armed"
            )

    def _on_real_joint_state(self, message: JointState) -> None:
        values = {name: float(value) for name, value in zip(message.name, message.position)}
        if any(name not in values or not math.isfinite(values[name]) for name in AXIS_NAMES):
            return
        self.actual_positions = tuple(values[name] for name in AXIS_NAMES)
        self.actual_position_samples.append(self.actual_positions)
        self.actual_stable = self._actual_window_is_stable()
        if self.actual_stable and not self.actual_stable_logged:
            ordered = ", ".join(
                f"{name}={value:.8f}"
                for name, value in zip(AXIS_NAMES, self.actual_positions)
            )
            self.get_logger().info(
                f"stable 14-axis startup pose captured: {ordered}"
            )
            self.actual_stable_logged = True
        elif not self.actual_stable:
            self.actual_stable_logged = False
        if isinstance(self.backend, RealRollingBackend):
            self.backend.update_actual(
                self.actual_positions,
                stable=self.actual_stable,
            )
        relayed = JointState()
        relayed.header = message.header
        relayed.name = list(AXIS_NAMES)
        relayed.position = list(self.actual_positions)
        if len(message.velocity) == len(message.name):
            velocities = {name: float(value) for name, value in zip(message.name, message.velocity)}
            relayed.velocity = [velocities.get(name, 0.0) for name in AXIS_NAMES]
        self.feedback_pub.publish(relayed)

    def _actual_window_is_stable(self) -> bool:
        if len(self.actual_position_samples) < self.stable_sample_count:
            return False
        for axis in range(13):
            values = [sample[axis] for sample in self.actual_position_samples]
            if max(values) - min(values) > self.stable_rotary_span:
                return False
        updown = [sample[13] for sample in self.actual_position_samples]
        return max(updown) - min(updown) <= self.stable_updown_span

    def _control_tick(self) -> None:
        if isinstance(self.backend, MockRollingBackend):
            self.backend.control_tick()

    def _feedback_tick(self) -> None:
        if not isinstance(self.backend, MockRollingBackend):
            return
        self.actual_positions = tuple(self.backend.actual_positions)
        self.backend.mark_actual_ready()
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(AXIS_NAMES)
        message.position = list(self.backend.actual_positions)
        message.velocity = list(self.backend.actual_velocities)
        self.feedback_pub.publish(message)

    def _session_tick(self) -> None:
        self.backend.poll()
        view = self.backend.view
        if self.pending is not None:
            if view.last_accepted_sequence >= self.pending.sequence:
                self.accepted = self.pending
                self.pending = None
                self.promoted_count += 1
                self._publish_accepted_reference()
                self.planner_error = "none"
            elif view.last_rejected_sequence >= self.pending.sequence:
                self.pending = None
                self.local_reject_count += 1
                self.planner_error = f"rt_reject:{view.last_reject}"
        if self.shutdown_requested:
            self.backend.begin_close()
            self.backend.close_poll()

    def _batch_tick(self) -> None:
        if self.shutdown_requested or self.pending is not None:
            return
        settings = self.backend.settings
        if settings is None:
            return
        view = self.backend.view
        if self.accepted is None:
            sequence = self._take_sequence()
            candidate = build_hold_suffix(
                sequence=sequence,
                replace_from_ns=settings.initial_replaceable_from_ns,
                hold_positions=settings.hold_positions,
                knot_ns=self.knot_ns,
                horizon_ns=max(self.horizon_ns, settings.required_horizon_ns),
            )
            self._submit(candidate, PlanDiagnostics(0.0, 0.0, 0.0))
            return
        if view.session_state != "RUNNING":
            return
        try:
            replace_from_ns = choose_replace_from_ns(
                public_replaceable_from_ns=view.replaceable_from_ns,
                accepted_replace_from_ns=self.accepted.replace_from_ns,
                public_state_age_ns=self.backend.public_state_age_ns(),
                replace_guard_ns=self.replace_guard_ns,
                horizon_ns=self.horizon_ns,
                max_horizon_ns=settings.max_horizon_ns,
                replace_lead_ns=settings.replace_lead_ns,
                controller_period_ns=settings.controller_period_ns,
            )
        except (SuffixPlanningError, ValueError) as error:
            self.local_reject_count += 1
            self.planner_error = f"local_plan_reject:{error}"
            return
        target_is_fresh = (
            self.latest_target is not None
            and self.latest_target_at is not None
            and time.monotonic() - self.latest_target_at <= self.command_stale_s
        )
        if target_tracking_allowed(
            enabled=self.enable_target_tracking,
            armed=self.tracking_armed,
            target_is_fresh=target_is_fresh,
        ):
            target = self.latest_target
        else:
            target, _ = self.accepted.sample(replace_from_ns)
        sequence = self._take_sequence()
        try:
            candidate, diagnostics = self.planner.plan(
                accepted=self.accepted,
                sequence=sequence,
                replace_from_ns=replace_from_ns,
                target_positions=target,
            )
        except (SuffixPlanningError, ValueError) as error:
            self.local_reject_count += 1
            self.planner_error = f"local_plan_reject:{error}"
            return
        self._submit(candidate, diagnostics)

    def _take_sequence(self) -> int:
        value = self.next_sequence
        self.next_sequence += 1
        return value

    def _submit(self, candidate: RollingSuffix, diagnostics: PlanDiagnostics) -> None:
        self.pending = candidate
        self.pending_diagnostics = diagnostics
        if not self.backend.submit(candidate):
            self.pending = None
            self.local_reject_count += 1
            self.planner_error = f"submit_failed:{self.backend.view.last_reject}"
            return
        self.published_count += 1
        self.batch_ticks += 1
        self.last_batch_at = time.monotonic()
        window = self.last_batch_at - self.batch_window_at
        if window >= 1.0:
            self.measured_batch_hz = self.batch_ticks / window
            self.batch_ticks = 0
            self.batch_window_at = self.last_batch_at

    def _publish_accepted_reference(self) -> None:
        if self.accepted is None:
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(AXIS_NAMES)
        message.position = list(self.accepted.points[-1].positions)
        message.velocity = list(self.accepted.points[-1].velocities)
        self.accepted_reference_pub.publish(message)

    def _status_tick(self) -> None:
        view = self.backend.view
        try:
            public_state_age_ms = self.backend.public_state_age_ns() * 1e-6
        except SuffixPlanningError:
            public_state_age_ms = None
        command_age_ms = None
        if self.latest_target_at is not None:
            command_age_ms = (time.monotonic() - self.latest_target_at) * 1000.0
        diagnostics = self.pending_diagnostics
        status = {
            "backend": self.backend_name,
            "status": view.status,
            "mode": view.mode,
            "session_state": view.session_state,
            "has_session": view.has_session,
            "real_command_authorized": bool(
                self.backend_name == "real" and self.allow_real_command
            ),
            "target_tracking_enabled": self.enable_target_tracking,
            "target_tracking_armed": self.tracking_armed,
            "actual_pose_stable": self.actual_stable,
            "actual_stable_samples": len(self.actual_position_samples),
            "control_hz": self.backend.control_hz
            if isinstance(self.backend, MockRollingBackend)
            else 250.0,
            "batch_hz": self.measured_batch_hz,
            "command_age_ms": command_age_ms,
            "watchdog_hold": view.session_state in {"HOLDING", "STOPPING", "TERMINATED"},
            "max_tracking_error": self.backend.max_tracking_error
            if isinstance(self.backend, MockRollingBackend)
            else 0.0,
            "last_seen_sequence": view.last_seen_sequence,
            "last_accepted_sequence": view.last_accepted_sequence,
            "last_rejected_sequence": view.last_rejected_sequence,
            "last_reject": view.last_reject,
            "stop_reason": view.stop_reason,
            "available_horizon_ms": view.available_horizon_ns * 1e-6,
            "replaceable_from_ms": view.replaceable_from_ns * 1e-6,
            "public_state_age_ms": public_state_age_ms,
            "replace_guard_ms": self.replace_guard_ns * 1e-6,
            "buffer_point_count": view.buffer_point_count,
            "published_count": self.published_count,
            "promoted_count": self.promoted_count,
            "local_reject_count": self.local_reject_count,
            "planner_error": self.planner_error,
            "target_scale": diagnostics.target_scale if diagnostics else 0.0,
            "planned_peak_velocity": diagnostics.peak_velocity if diagnostics else 0.0,
            "planned_peak_acceleration": diagnostics.peak_acceleration if diagnostics else 0.0,
            "interface_joint_count": AXIS_COUNT,
            "axis_set_sha256": AXIS_SET_SHA256,
            "limits_sha256": self.backend.settings.limits_version_hex
            if self.backend.settings is not None
            else None,
            "limits_source": self.backend.settings.limits_source
            if self.backend.settings is not None
            else None,
        }
        message = String()
        message.data = json.dumps(status, separators=(",", ":"))
        self.status_pub.publish(message)

    def begin_shutdown(self) -> None:
        if self.shutdown_requested:
            return
        self.shutdown_requested = True
        self.shutdown_started_at = time.monotonic()
        self.get_logger().warning("shutdown requested; closing rolling session before exit")

    def ready_to_exit(self) -> bool:
        return bool(self.backend.closed)


def main() -> None:
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = RollingPoseClient()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop_requested = False

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal stop_requested
        stop_requested = True
        node.begin_shutdown()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
            if stop_requested and node.ready_to_exit():
                break
            if (
                stop_requested
                and node.shutdown_started_at is not None
                and time.monotonic() - node.shutdown_started_at > 8.0
            ):
                node.get_logger().error(
                    "safe close did not finish within 8s; RT watchdog must own the stop"
                )
                break
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
