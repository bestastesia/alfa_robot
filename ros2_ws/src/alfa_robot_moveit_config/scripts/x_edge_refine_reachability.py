#!/usr/bin/python3
"""High-resolution reachability scan for x-axis edge bands.

This is a focused follow-up to nine_orient_reachability.py. It scans the two
current x extrema bands with 1 cm resolution by default:
  - x_plus:  0.70 .. 0.80
  - x_minus: -0.80 .. -0.90

The output CSV keeps the same core schema as nine_orient_reachability.py and
adds a `region` column so existing visualization tools can reuse it.
"""

from __future__ import annotations

import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState


@dataclass(frozen=True)
class ArmConfig:
    group_name: str
    ik_link_name: str
    label: str
    center_quat: tuple[float, float, float, float]


ARM_CONFIGS = {
    "left": ArmConfig("left_arm", "left_tool0", "left", (0.0, 0.7071, 0.0, 0.7071)),
    "right": ArmConfig("right_arm", "right_tool0", "right", (0.0, 0.7071, 0.0, 0.7071)),
}


def normalize_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, float, float, float]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
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


def quat_multiply(q1: tuple[float, float, float, float],
                  q2: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    qw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    qx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    qy = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    qz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return normalize_quaternion(qx, qy, qz, qw)


def apply_yaw_pitch_offset(center_quat: tuple[float, float, float, float],
                           yaw_deg: float,
                           pitch_deg: float) -> tuple[float, float, float, float]:
    return quat_multiply(center_quat, euler_to_quaternion(0.0, math.radians(pitch_deg), math.radians(yaw_deg)))


def float_range(min_value: float, max_value: float, step: float) -> Iterable[float]:
    if step <= 0.0:
        raise ValueError("step must be > 0")
    direction = 1.0 if max_value >= min_value else -1.0
    step = abs(step) * direction
    count = int(math.floor(abs(max_value - min_value) / abs(step) + 1e-9))
    for index in range(count + 1):
        yield min_value + step * index
    last = min_value + step * count
    if abs(max_value - last) > abs(step) * 0.5:
        yield max_value


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def as_float(value) -> float:
    return float(value)


def parse_range_spec(spec: str) -> list[tuple[str, float, float, float, float, float, float]]:
    """Parse `name:x0:x1:y0:y1:z0:z1` entries separated by semicolons."""
    regions = []
    for item in spec.split(";"):
        item = item.strip()
        if not item:
            continue
        parts = item.split(":")
        if len(parts) != 7:
            raise ValueError(f"Invalid region spec: {item}")
        name = parts[0]
        values = [float(value) for value in parts[1:]]
        regions.append((name, *values))
    if not regions:
        raise ValueError("No x edge regions configured")
    return regions


class XEdgeRefineReachability(Node):
    def __init__(self) -> None:
        super().__init__("x_edge_refine_reachability")

        self.declare_parameter("side", "left")
        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("output_csv", "x_edge_refine_reachability.csv")
        self.declare_parameter("append", False)
        self.declare_parameter("avoid_collisions", False)
        self.declare_parameter("ik_timeout", 0.05)
        self.declare_parameter("service_timeout", 10.0)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("seed_wait_sec", 2.0)
        self.declare_parameter("center_orientation_xyzw", [0.0, 0.7071, 0.0, 0.7071])
        self.declare_parameter("yaw_pitch_delta", 15.0)
        self.declare_parameter("orientation_mode", "all9")  # all9 or center
        self.declare_parameter("step", 0.01)
        self.declare_parameter("x_step", 0.01)
        self.declare_parameter("y_step", 0.025)
        self.declare_parameter("z_step", 0.025)

        # Defaults are tight boxes around the previous x extrema edge points.
        self.declare_parameter(
            "regions",
            "x_plus:0.80:0.85:0.2:0.45:0.55:0.75;"
            "x_minus:-0.80:-0.85:0.2:0.45:0.55:0.75",
        )
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
        group_override = str(self.get_parameter(f"{side}_group").value)
        tip_override = str(self.get_parameter(f"{side}_tip").value)
        center_override = list(self.get_parameter("center_orientation_xyzw").value)
        return ArmConfig(
            group_name=group_override,
            ik_link_name=tip_override,
            label=base.label,
            center_quat=normalize_quaternion(*[float(value) for value in center_override]),
        )

    def _build_orient_set(self) -> list[tuple[str, tuple[float, float, float, float]]]:
        arm = self._get_arm_config()
        orientation_mode = str(self.get_parameter("orientation_mode").value).lower()
        if orientation_mode == "center":
            return [("center", arm.center_quat)]
        if orientation_mode != "all9":
            raise ValueError("orientation_mode must be 'all9' or 'center'")

        delta = as_float(self.get_parameter("yaw_pitch_delta").value)
        orient_set = [
            (-delta, -delta), (0.0, -delta), (delta, -delta),
            (-delta, 0.0), (0.0, 0.0), (delta, 0.0),
            (-delta, delta), (0.0, delta), (delta, delta),
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
        return [(label, apply_yaw_pitch_offset(arm.center_quat, yaw, pitch)) for label, (yaw, pitch) in zip(labels, orient_set)]

    def _seed_state(self) -> JointState:
        timeout = as_float(self.get_parameter("seed_wait_sec").value)
        deadline = self.get_clock().now() + Duration(seconds=timeout)
        while rclpy.ok() and self._latest_joint_state is None and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self._latest_joint_state is not None:
            return self._latest_joint_state
        self.get_logger().warn("No /joint_states received; using diff RobotState without explicit seed")
        return JointState()

    def _solve_ik(self, arm: ArmConfig, pose: PoseStamped, seed_state: JointState) -> tuple[bool, int, str, float]:
        request = GetPositionIK.Request()
        request.ik_request.group_name = arm.group_name
        request.ik_request.ik_link_name = arm.ik_link_name
        request.ik_request.pose_stamped = pose
        request.ik_request.robot_state.joint_state = seed_state
        request.ik_request.robot_state.is_diff = True
        request.ik_request.avoid_collisions = as_bool(self.get_parameter("avoid_collisions").value)

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
        return success, int(response.error_code.val), "success" if success else "ik_failed", elapsed_ms

    def run(self) -> int:
        service_timeout = as_float(self.get_parameter("service_timeout").value)
        if not self._ik_client.wait_for_service(timeout_sec=service_timeout):
            self.get_logger().error("/compute_ik service not available")
            return 3

        arm = self._get_arm_config()
        orient_set = self._build_orient_set()
        seed_state = self._seed_state()
        fallback_step = as_float(self.get_parameter("step").value)
        x_step = as_float(self.get_parameter("x_step").value) or fallback_step
        y_step = as_float(self.get_parameter("y_step").value) or fallback_step
        z_step = as_float(self.get_parameter("z_step").value) or fallback_step
        frame = str(self.get_parameter("reference_frame").value)
        regions = parse_range_spec(str(self.get_parameter("regions").value))

        fieldnames = [
            "region", "side", "group", "ik_link", "frame",
            "x", "y", "z", "orient_idx", "orient_label",
            "qx", "qy", "qz", "qw",
            "success", "error_code", "reason", "time_ms",
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

        total_points = 0
        for _, x0, x1, y0, y1, z0, z1 in regions:
            total_points += len(list(float_range(x0, x1, x_step))) * len(list(float_range(y0, y1, y_step))) * len(list(float_range(z0, z1, z_step)))

        self.get_logger().info(f"Starting x-edge refine scan: regions={len(regions)} x_step={x_step} y_step={y_step} z_step={z_step} points={total_points} tests={total_points * len(orient_set)}")
        self.get_logger().info(f"  arm={arm.label} group={arm.group_name} tip={arm.ik_link_name} frame={frame}")

        point_idx = 0
        reachable_count = 0
        try:
            for region, x0, x1, y0, y1, z0, z1 in regions:
                xs = list(float_range(x0, x1, x_step))
                ys = list(float_range(y0, y1, y_step))
                zs = list(float_range(z0, z1, z_step))
                self.get_logger().info(f"  region={region} x[{xs[0]:.3f}->{xs[-1]:.3f}] y[{ys[0]:.3f}->{ys[-1]:.3f}] z[{zs[0]:.3f}->{zs[-1]:.3f}]")
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

                                success, error_code, reason, time_ms = self._solve_ik(arm, pose, seed_state)
                                n_success += int(success)
                                writer.writerow({
                                    "region": region,
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
                                    "n_success": "",
                                    "n_total": "",
                                    "point_reachable": "",
                                })

                            point_reachable = n_success == len(orient_set)
                            reachable_count += int(point_reachable)
                            writer.writerow({
                                "region": region,
                                "side": arm.label,
                                "group": arm.group_name,
                                "ik_link": arm.ik_link_name,
                                "frame": frame,
                                "x": f"{x:.6f}",
                                "y": f"{y:.6f}",
                                "z": f"{z:.6f}",
                                "orient_idx": -1,
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
                                "point_reachable": int(point_reachable),
                            })

                            if point_idx % 100 == 0:
                                csv_file.flush()
                                self.get_logger().info(f"progress: {point_idx}/{total_points}, reachable={reachable_count}")
        finally:
            csv_file.close()

        ratio = reachable_count / total_points if total_points else 0.0
        self.get_logger().info(f"x-edge refine done: total_points={total_points} reachable={reachable_count} ratio={ratio:.3%}")
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = XEdgeRefineReachability()
    try:
        exit_code = node.run()
    except Exception as exc:
        node.get_logger().error(f"x_edge_refine_reachability failed: {exc}")
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main(sys.argv)
