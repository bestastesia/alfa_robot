#!/usr/bin/python3
"""9-orientation reachability tester for ALFA arms.

T-0029 implementation: at each spatial point, test 9 orientations
(center + yaw/pitch ±15° combinations). A point is "reachable" only
when ALL 9 orientations have IK solutions.

Orientation set (3×3 grid in yaw/pitch, roll=0):
  ┌────────────────────┐
  │ yaw-15 pitch-15    │ yaw+0  pitch-15    │ yaw+15 pitch-15    │
  │ yaw-15 pitch+0     │ yaw+0  pitch+0     │ yaw+15 pitch+0     │  ← center
  │ yaw-15 pitch+15    │ yaw+0  pitch+15    │ yaw+15 pitch+15    │
  └────────────────────┘

The "center" orientation is the default forward-facing pose for each arm.
Roll is excluded because joint6 can compensate for roll offsets.

This script extends reachability_tester.py with multi-orientation testing
and richer CSV output for T-0030/T-0031 visualization.
"""

from __future__ import annotations

import csv
import itertools
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


# ── Constants ────────────────────────────────────────────────────────

YAW_PITCH_DELTA = 15.0  # degrees

# 9 orientation offsets: (yaw_deg, pitch_deg)
ORIENT_SET = [
    (-YAW_PITCH_DELTA, -YAW_PITCH_DELTA),
    (0.0,              -YAW_PITCH_DELTA),
    ( YAW_PITCH_DELTA, -YAW_PITCH_DELTA),
    (-YAW_PITCH_DELTA,  0.0),
    (0.0,               0.0),              # center
    ( YAW_PITCH_DELTA,  0.0),
    (-YAW_PITCH_DELTA,  YAW_PITCH_DELTA),
    (0.0,               YAW_PITCH_DELTA),
    ( YAW_PITCH_DELTA,  YAW_PITCH_DELTA),
]

ORIENT_LABELS = [
    "yaw-15_pitch-15",
    "yaw0_pitch-15",
    "yaw+15_pitch-15",
    "yaw-15_pitch0",
    "center",
    "yaw+15_pitch0",
    "yaw-15_pitch+15",
    "yaw0_pitch+15",
    "yaw+15_pitch+15",
]


@dataclass(frozen=True)
class ArmConfig:
    group_name: str
    ik_link_name: str
    label: str
    center_quat: tuple[float, float, float, float]  # (qx, qy, qz, qw) default orientation


# Default arm configs: use arm-only groups (no turn/updown) for fixed-mount reachability
ARM_CONFIGS = {
    "left": ArmConfig(
        group_name="left_arm",
        ik_link_name="left_tool0",
        label="left",
        center_quat=(0.0, 0.7071, 0.0, 0.7071),  # pitch=90° forward
    ),
    "right": ArmConfig(
        group_name="right_arm",
        ik_link_name="right_tool0",
        label="right",
        center_quat=(0.0, 0.7071, 0.0, 0.7071),
    ),
}


def normalize_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, float, float, float]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Convert Euler angles ( radians ) to quaternion (qx, qy, qz, qw)."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return normalize_quaternion(qx, qy, qz, qw)


def quat_multiply(q1: tuple, q2: tuple) -> tuple[float, float, float, float]:
    """Hamilton product of two quaternions."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    qw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    qx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    qy = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    qz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return normalize_quaternion(qx, qy, qz, qw)


def apply_yaw_pitch_offset(center_quat: tuple, yaw_deg: float, pitch_deg: float) -> tuple[float, float, float, float]:
    """Apply yaw/pitch offset to center orientation, keeping roll=0."""
    yaw_rad = math.radians(yaw_deg)
    pitch_rad = math.radians(pitch_deg)
    offset_quat = euler_to_quaternion(0.0, pitch_rad, yaw_rad)
    return quat_multiply(center_quat, offset_quat)


def float_range(min_value: float, max_value: float, step: float) -> Iterable[float]:
    if step <= 0.0:
        raise ValueError("sample step must be > 0")
    count = int(math.floor((max_value - min_value) / step + 1e-9))
    for index in range(count + 1):
        yield min_value + step * index
    last = min_value + step * count
    if max_value - last > step * 0.5:
        yield max_value


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def as_float(value) -> float:
    return float(value)


# ── Node ─────────────────────────────────────────────────────────────

class NineOrientationReachabilityTester(Node):
    def __init__(self) -> None:
        super().__init__("nine_orient_reachability")

        # Mode and arm selection
        self.declare_parameter("side", "left")
        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("output_csv", "nine_orient_reachability.csv")
        self.declare_parameter("append", False)

        # IK parameters (avoid_collisions=False for pure reachability testing)
        self.declare_parameter("avoid_collisions", False)
        self.declare_parameter("classify_collisions", False)
        self.declare_parameter("ik_timeout", 0.05)
        self.declare_parameter("service_timeout", 10.0)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("seed_wait_sec", 2.0)
        # Optional: force base lift/updown value in the IK seed.
        # Negative value means: use current /joint_states as-is.
        self.declare_parameter("fixed_updown", -1.0)

        # Override center orientation (optional)
        self.declare_parameter("center_orientation_xyzw", [0.0, 0.7071, 0.0, 0.7071])

        # Yaw/pitch delta (degrees)
        self.declare_parameter("yaw_pitch_delta", 15.0)

        # Spatial sampling range (robot front = +X, left = +Y, up = +Z)
        self.declare_parameter("min_x", -1.2)
        self.declare_parameter("max_x", 1.6)
        self.declare_parameter("min_y", -0.3)
        self.declare_parameter("max_y", 1.2)
        self.declare_parameter("min_z", 0.0)
        self.declare_parameter("max_z", 2.0)
        self.declare_parameter("step", 0.1)
        self.declare_parameter("step_x", 0.0)
        self.declare_parameter("step_y", 0.0)
        self.declare_parameter("step_z", 0.0)

        # Group override (arm-only groups by default)
        self.declare_parameter("left_group", "left_arm")
        self.declare_parameter("right_group", "right_arm")
        self.declare_parameter("left_tip", "left_tool0")
        self.declare_parameter("right_tip", "right_tool0")

        self._latest_joint_state: Optional[JointState] = None
        self._joint_state_sub = self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._joint_state_cb,
            10,
        )

        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")

    def _joint_state_cb(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def _get_arm_config(self) -> ArmConfig:
        side = str(self.get_parameter("side").value).lower()
        base = ARM_CONFIGS[side]

        # Allow override
        group_key = f"{side}_group"
        tip_key = f"{side}_tip"
        group_override = str(self.get_parameter(group_key).value)
        tip_override = str(self.get_parameter(tip_key).value)
        center_override = list(self.get_parameter("center_orientation_xyzw").value)

        return ArmConfig(
            group_name=group_override,
            ik_link_name=tip_override,
            label=base.label,
            center_quat=normalize_quaternion(*[float(v) for v in center_override]),
        )

    def _build_orient_set(self) -> list[tuple[str, tuple[float, float, float, float]]]:
        """Build 9 orientation poses from center + yaw/pitch offsets."""
        arm = self._get_arm_config()
        delta = as_float(self.get_parameter("yaw_pitch_delta").value)
        orient_set = [
            (-delta, -delta),
            (0.0, -delta),
            (delta, -delta),
            (-delta, 0.0),
            (0.0, 0.0),
            (delta, 0.0),
            (-delta, delta),
            (0.0, delta),
            (delta, delta),
        ]
        labels = [
            f"yaw-{delta:.0f}_pitch-{delta:.0f}",
            f"yaw0_pitch-{delta:.0f}",
            f"yaw+{delta:.0f}_pitch-{delta:.0f}",
            f"yaw-{delta:.0f}_pitch0",
            "center",
            f"yaw+{delta:.0f}_pitch0",
            f"yaw-{delta:.0f}_pitch+{delta:.0f}",
            f"yaw0_pitch+{delta:.0f}",
            f"yaw+{delta:.0f}_pitch+{delta:.0f}",
        ]
        result = []
        for label, (yaw, pitch) in zip(labels, orient_set):
            quat = apply_yaw_pitch_offset(arm.center_quat, yaw, pitch)
            result.append((label, quat))
        return result

    def _seed_state(self) -> JointState:
        timeout = as_float(self.get_parameter("seed_wait_sec").value)
        deadline = self.get_clock().now() + Duration(seconds=timeout)
        while rclpy.ok() and self._latest_joint_state is None and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        if self._latest_joint_state is not None:
            seed = JointState()
            seed.header = self._latest_joint_state.header
            seed.name = list(self._latest_joint_state.name)
            seed.position = list(self._latest_joint_state.position)
            seed.velocity = list(self._latest_joint_state.velocity)
            seed.effort = list(self._latest_joint_state.effort)
        else:
            self.get_logger().warn("No /joint_states received; using diff RobotState without explicit seed")
            seed = JointState()

        fixed_updown = as_float(self.get_parameter("fixed_updown").value)
        if fixed_updown >= 0.0:
            if "updown" in seed.name:
                index = seed.name.index("updown")
                while len(seed.position) <= index:
                    seed.position.append(0.0)
                seed.position[index] = fixed_updown
            else:
                seed.name.append("updown")
                seed.position.append(fixed_updown)
            self.get_logger().info(f"Using fixed updown seed: {fixed_updown:.3f} m")
        elif "updown" in seed.name and len(seed.position) > seed.name.index("updown"):
            self.get_logger().info(f"Using current /joint_states updown seed: {seed.position[seed.name.index('updown')]:.3f} m")
        else:
            self.get_logger().info("No updown seed value available; MoveIt will use default/current state")

        return seed

    def _solve_ik(self, arm: ArmConfig, pose: PoseStamped, seed_state: JointState,
                  avoid_collisions: bool | None = None) -> tuple[bool, int, str, float]:
        """Solve IK. Returns (success, error_code, reason, solve_time_ms)."""
        request = GetPositionIK.Request()
        request.ik_request.group_name = arm.group_name
        request.ik_request.ik_link_name = arm.ik_link_name
        request.ik_request.pose_stamped = pose
        request.ik_request.robot_state.joint_state = seed_state
        request.ik_request.robot_state.is_diff = True
        if avoid_collisions is None:
            avoid_collisions = as_bool(self.get_parameter("avoid_collisions").value)
        request.ik_request.avoid_collisions = avoid_collisions

        timeout_sec = as_float(self.get_parameter("ik_timeout").value)
        request.ik_request.timeout.sec = int(timeout_sec)
        request.ik_request.timeout.nanosec = int((timeout_sec - int(timeout_sec)) * 1e9)

        import time
        t0 = time.perf_counter()
        future = self._ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0

        response = future.result()
        if response is None:
            return False, MoveItErrorCodes.FAILURE, "no_response", elapsed_ms
        success = response.error_code.val == MoveItErrorCodes.SUCCESS
        reason = "success" if success else "ik_failed"
        return success, int(response.error_code.val), reason, elapsed_ms

    def _solve_ik_classified(self, arm: ArmConfig, pose: PoseStamped,
                             seed_state: JointState) -> tuple[bool, int, str, float]:
        """Classify IK as success, collision, or unreachable.

        First solves without collision checking. If no solution exists, the pose is
        unreachable/failed. If a no-collision solution exists, solve again with
        collision checking; failure in the second pass is classified as collision.
        """
        raw_success, raw_error, raw_reason, raw_ms = self._solve_ik(
            arm, pose, seed_state, avoid_collisions=False)
        if not raw_success:
            return False, raw_error, raw_reason, raw_ms

        safe_success, safe_error, safe_reason, safe_ms = self._solve_ik(
            arm, pose, seed_state, avoid_collisions=True)
        total_ms = raw_ms + safe_ms
        if safe_success:
            return True, safe_error, "success", total_ms
        return False, safe_error, "collision", total_ms

    def run(self) -> int:
        service_timeout = as_float(self.get_parameter("service_timeout").value)
        if not self._ik_client.wait_for_service(timeout_sec=service_timeout):
            self.get_logger().error("/compute_ik service not available")
            return 3

        arm = self._get_arm_config()
        orient_set = self._build_orient_set()
        seed_state = self._seed_state()
        step = as_float(self.get_parameter("step").value)
        step_x = as_float(self.get_parameter("step_x").value) or step
        step_y = as_float(self.get_parameter("step_y").value) or step
        step_z = as_float(self.get_parameter("step_z").value) or step

        xs = list(float_range(as_float(self.get_parameter("min_x").value),
                              as_float(self.get_parameter("max_x").value), step_x))
        ys = list(float_range(as_float(self.get_parameter("min_y").value),
                              as_float(self.get_parameter("max_y").value), step_y))
        zs = list(float_range(as_float(self.get_parameter("min_z").value),
                              as_float(self.get_parameter("max_z").value), step_z))

        frame = str(self.get_parameter("reference_frame").value)
        classify_collisions = as_bool(self.get_parameter("classify_collisions").value)

        # CSV fields
        fieldnames = [
            "side", "group", "ik_link", "frame",
            "x", "y", "z",
            "orient_idx", "orient_label",
            "qx", "qy", "qz", "qw",
            "success", "error_code", "reason", "time_ms",
            # point-level summary (filled at end of each point)
            "n_success", "n_total", "point_reachable",
        ]

        output_path = Path(str(self.get_parameter("output_csv").value)).expanduser()
        if output_path.parent and str(output_path.parent) != ".":
            output_path.parent.mkdir(parents=True, exist_ok=True)
        append = as_bool(self.get_parameter("append").value)
        file_exists = output_path.exists() and output_path.stat().st_size > 0
        csv_file = output_path.open("a" if append else "w", newline="")
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        if not append or not file_exists:
            writer.writeheader()

        total_points = len(xs) * len(ys) * len(zs)
        total_tests = total_points * len(orient_set)
        self.get_logger().info(
            f"Starting 9-orient reachability: {total_points} points × {len(orient_set)} orientations = {total_tests} IK calls"
        )
        self.get_logger().info(f"  arm={arm.label} group={arm.group_name} tip={arm.ik_link_name}")
        self.get_logger().info(
            f"  range: x[{xs[0]:.3f}~{xs[-1]:.3f}] step_x={step_x} "
            f"y[{ys[0]:.3f}~{ys[-1]:.3f}] step_y={step_y} "
            f"z[{zs[0]:.3f}~{zs[-1]:.3f}] step_z={step_z}")
        self.get_logger().info(
            f"  IK timeout={as_float(self.get_parameter('ik_timeout').value)}s "
            f"avoid_collisions={as_bool(self.get_parameter('avoid_collisions').value)} "
            f"classify_collisions={classify_collisions} "
            f"fixed_updown={as_float(self.get_parameter('fixed_updown').value)}")

        point_idx = 0
        reachable_count = 0

        try:
            for x in xs:
                for y in ys:
                    for z in zs:
                        point_idx += 1
                        n_success = 0

                        for oidx, (olabel, oquat) in enumerate(orient_set):
                            pose = PoseStamped()
                            pose.header.frame_id = frame
                            pose.header.stamp = self.get_clock().now().to_msg()
                            pose.pose.position.x = x
                            pose.pose.position.y = y
                            pose.pose.position.z = z
                            pose.pose.orientation.x = oquat[0]
                            pose.pose.orientation.y = oquat[1]
                            pose.pose.orientation.z = oquat[2]
                            pose.pose.orientation.w = oquat[3]

                            if classify_collisions:
                                success, error_code, reason, time_ms = self._solve_ik_classified(
                                    arm, pose, seed_state)
                            else:
                                success, error_code, reason, time_ms = self._solve_ik(arm, pose, seed_state)
                            n_success += int(success)

                            writer.writerow({
                                "side": arm.label,
                                "group": arm.group_name,
                                "ik_link": arm.ik_link_name,
                                "frame": frame,
                                "x": f"{x:.6f}",
                                "y": f"{y:.6f}",
                                "z": f"{z:.6f}",
                                "orient_idx": oidx,
                                "orient_label": olabel,
                                "qx": f"{oquat[0]:.8f}",
                                "qy": f"{oquat[1]:.8f}",
                                "qz": f"{oquat[2]:.8f}",
                                "qw": f"{oquat[3]:.8f}",
                                "success": int(success),
                                "error_code": error_code,
                                "reason": reason,
                                "time_ms": f"{time_ms:.1f}",
                                "n_success": "",  # filled at point end
                                "n_total": "",
                                "point_reachable": "",
                            })

                        # Write point-level summary row
                        is_reachable = n_success == len(orient_set)
                        reachable_count += int(is_reachable)
                        writer.writerow({
                            "side": arm.label,
                            "group": arm.group_name,
                            "ik_link": arm.ik_link_name,
                            "frame": frame,
                            "x": f"{x:.6f}",
                            "y": f"{y:.6f}",
                            "z": f"{z:.6f}",
                            "orient_idx": -1,  # summary marker
                            "orient_label": "SUMMARY",
                            "qx": "",
                            "qy": "",
                            "qz": "",
                            "qw": "",
                            "success": "",
                            "error_code": "",
                            "reason": "",
                            "time_ms": "",
                            "n_success": n_success,
                            "n_total": len(orient_set),
                            "point_reachable": int(is_reachable),
                        })

                        if point_idx % 20 == 0:
                            csv_file.flush()
                            ratio = reachable_count / point_idx if point_idx else 0
                            self.get_logger().info(
                                f"progress: {point_idx}/{total_points} points, "
                                f"reachable={reachable_count} ({ratio:.2%})"
                            )
        finally:
            csv_file.close()

        ratio = reachable_count / total_points if total_points else 0
        self.get_logger().info(
            f"9-orient reachability done: total_points={total_points} "
            f"reachable={reachable_count} ratio={ratio:.3%} "
            f"(n_total={total_tests} IK calls)"
        )
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NineOrientationReachabilityTester()
    try:
        exit_code = node.run()
    except Exception as exc:
        node.get_logger().error(f"nine_orient_reachability failed: {exc}")
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main(sys.argv)
