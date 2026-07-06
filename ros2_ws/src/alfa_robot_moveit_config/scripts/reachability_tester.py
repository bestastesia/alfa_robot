#!/usr/bin/python3
"""Reachability test utility for ALFA arms.

Modes:
- manual: sample current TF pose(s) of the requested end-effector links and append to CSV.
- auto: sample a Cartesian box and call MoveIt's /compute_ik for each pose.

The node intentionally focuses on the motion-control core and writes simple CSV
artifacts. RViz Marker/3D visualization is handled by the simulation task.
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

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class ArmConfig:
    group_name: str
    ik_link_name: str
    label: str


def normalize_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, float, float, float]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


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


class ReachabilityTester(Node):
    def __init__(self) -> None:
        super().__init__("reachability_tester")

        self.declare_parameter("mode", "auto")
        self.declare_parameter("side", "left")
        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("output_csv", "reachability_results.csv")
        self.declare_parameter("append", False)

        self.declare_parameter("left_group", "left_arm_with_base")
        self.declare_parameter("right_group", "right_arm_with_base")
        self.declare_parameter("dual_group", "dual_arm_with_base")
        self.declare_parameter("left_tip", "left_tool0")
        self.declare_parameter("right_tip", "right_tool0")

        self.declare_parameter("avoid_collisions", True)
        self.declare_parameter("ik_timeout", 0.2)
        self.declare_parameter("service_timeout", 10.0)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("seed_wait_sec", 2.0)

        self.declare_parameter("manual_period", 0.2)
        self.declare_parameter("manual_duration", 0.0)

        self.declare_parameter("min_x", 0.0)
        self.declare_parameter("max_x", 0.8)
        self.declare_parameter("min_y", 0.0)
        self.declare_parameter("max_y", 0.6)
        self.declare_parameter("min_z", 0.1)
        self.declare_parameter("max_z", 0.8)
        self.declare_parameter("step", 0.1)
        self.declare_parameter("orientation_xyzw", [0.0, 0.0, 0.0, 1.0])

        self._latest_joint_state: Optional[JointState] = None
        self._joint_state_sub = self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._joint_state_cb,
            10,
        )

        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._manual_rows_written = 0
        self._manual_start_time = self.get_clock().now()
        self._manual_file = None
        self._manual_writer = None
        self._manual_timer = None
        self._manual_stop_requested = False

    def _joint_state_cb(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def run(self) -> int:
        mode = str(self.get_parameter("mode").value).lower()
        if mode == "auto":
            return self.run_auto()
        if mode == "manual":
            return self.run_manual()
        self.get_logger().error(f"Unsupported mode={mode!r}; expected 'auto' or 'manual'")
        return 2

    def _side_configs(self) -> list[ArmConfig]:
        side = str(self.get_parameter("side").value).lower()
        left = ArmConfig(
            str(self.get_parameter("left_group").value),
            str(self.get_parameter("left_tip").value),
            "left",
        )
        right = ArmConfig(
            str(self.get_parameter("right_group").value),
            str(self.get_parameter("right_tip").value),
            "right",
        )
        if side == "left":
            return [left]
        if side == "right":
            return [right]
        if side == "both":
            return [left, right]
        raise ValueError("side must be left, right, or both")

    def _open_csv(self, fieldnames: list[str]):
        output_path = Path(str(self.get_parameter("output_csv").value)).expanduser()
        if output_path.parent and str(output_path.parent) != ".":
            output_path.parent.mkdir(parents=True, exist_ok=True)
        append = as_bool(self.get_parameter("append").value)
        file_exists = output_path.exists() and output_path.stat().st_size > 0
        csv_file = output_path.open("a" if append else "w", newline="")
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        if not append or not file_exists:
            writer.writeheader()
        self.get_logger().info(f"Writing reachability CSV: {output_path}")
        return csv_file, writer

    def _seed_state(self) -> JointState:
        timeout = as_float(self.get_parameter("seed_wait_sec").value)
        deadline = self.get_clock().now() + Duration(seconds=timeout)
        while rclpy.ok() and self._latest_joint_state is None and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self._latest_joint_state is not None:
            return self._latest_joint_state
        self.get_logger().warn("No /joint_states received; using diff RobotState without explicit seed")
        return JointState()

    def _make_pose(self, x: float, y: float, z: float) -> PoseStamped:
        orientation = list(self.get_parameter("orientation_xyzw").value)
        if len(orientation) != 4:
            raise ValueError("orientation_xyzw must contain 4 numbers: [x, y, z, w]")
        qx, qy, qz, qw = normalize_quaternion(*(float(value) for value in orientation))

        pose = PoseStamped()
        pose.header.frame_id = str(self.get_parameter("reference_frame").value)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _solve_ik(self, arm: ArmConfig, pose: PoseStamped, seed_state: JointState) -> tuple[bool, int, str]:
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

        future = self._ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            return False, MoveItErrorCodes.FAILURE, "no_response"
        success = response.error_code.val == MoveItErrorCodes.SUCCESS
        return success, int(response.error_code.val), "success" if success else "ik_failed"

    def run_auto(self) -> int:
        service_timeout = as_float(self.get_parameter("service_timeout").value)
        if not self._ik_client.wait_for_service(timeout_sec=service_timeout):
            self.get_logger().error("/compute_ik service not available")
            return 3

        arms = self._side_configs()
        seed_state = self._seed_state()
        step = as_float(self.get_parameter("step").value)
        xs = list(float_range(as_float(self.get_parameter("min_x").value), as_float(self.get_parameter("max_x").value), step))
        ys = list(float_range(as_float(self.get_parameter("min_y").value), as_float(self.get_parameter("max_y").value), step))
        zs = list(float_range(as_float(self.get_parameter("min_z").value), as_float(self.get_parameter("max_z").value), step))

        fieldnames = [
            "mode", "side", "group", "ik_link", "frame", "x", "y", "z",
            "qx", "qy", "qz", "qw", "success", "error_code", "reason",
        ]
        csv_file, writer = self._open_csv(fieldnames)

        total = 0
        success_count = 0
        try:
            for arm, x, y, z in itertools.product(arms, xs, ys, zs):
                pose = self._make_pose(x, y, z)
                success, error_code, reason = self._solve_ik(arm, pose, seed_state)
                total += 1
                success_count += int(success)
                writer.writerow({
                    "mode": "auto",
                    "side": arm.label,
                    "group": arm.group_name,
                    "ik_link": arm.ik_link_name,
                    "frame": pose.header.frame_id,
                    "x": f"{pose.pose.position.x:.6f}",
                    "y": f"{pose.pose.position.y:.6f}",
                    "z": f"{pose.pose.position.z:.6f}",
                    "qx": f"{pose.pose.orientation.x:.8f}",
                    "qy": f"{pose.pose.orientation.y:.8f}",
                    "qz": f"{pose.pose.orientation.z:.8f}",
                    "qw": f"{pose.pose.orientation.w:.8f}",
                    "success": int(success),
                    "error_code": error_code,
                    "reason": reason,
                })
                if total % 50 == 0:
                    csv_file.flush()
                    self.get_logger().info(f"sampled={total} success={success_count}")
        finally:
            csv_file.close()

        ratio = success_count / total if total else 0.0
        self.get_logger().info(f"Auto reachability done: total={total} success={success_count} ratio={ratio:.3f}")
        return 0

    def run_manual(self) -> int:
        arms = self._side_configs()
        fieldnames = [
            "mode", "side", "frame", "stamp_sec", "x", "y", "z", "qx", "qy", "qz", "qw",
        ]
        self._manual_file, self._manual_writer = self._open_csv(fieldnames)
        period = as_float(self.get_parameter("manual_period").value)
        duration = as_float(self.get_parameter("manual_duration").value)

        def timer_cb() -> None:
            assert self._manual_writer is not None
            reference_frame = str(self.get_parameter("reference_frame").value)
            for arm in arms:
                try:
                    transform = self._tf_buffer.lookup_transform(
                        reference_frame,
                        arm.ik_link_name,
                        rclpy.time.Time(),
                        timeout=Duration(seconds=0.05),
                    )
                except TransformException as exc:
                    self.get_logger().warn(f"TF lookup failed for {arm.ik_link_name}: {exc}")
                    continue

                t = transform.transform.translation
                q = transform.transform.rotation
                self._manual_writer.writerow({
                    "mode": "manual",
                    "side": arm.label,
                    "frame": reference_frame,
                    "stamp_sec": f"{self.get_clock().now().nanoseconds / 1e9:.6f}",
                    "x": f"{t.x:.6f}",
                    "y": f"{t.y:.6f}",
                    "z": f"{t.z:.6f}",
                    "qx": f"{q.x:.8f}",
                    "qy": f"{q.y:.8f}",
                    "qz": f"{q.z:.8f}",
                    "qw": f"{q.w:.8f}",
                })
                self._manual_rows_written += 1
            self._manual_file.flush()
            if duration > 0.0:
                elapsed = (self.get_clock().now() - self._manual_start_time).nanoseconds / 1e9
                if elapsed >= duration:
                    self.get_logger().info("Manual duration reached; stopping recorder")
                    self._manual_stop_requested = True

        self._manual_timer = self.create_timer(period, timer_cb)
        self.get_logger().info(
            f"Manual recording started: side={str(self.get_parameter('side').value)} period={period}s duration={duration}s"
        )
        try:
            while rclpy.ok() and not self._manual_stop_requested:
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        finally:
            if self._manual_timer is not None:
                self.destroy_timer(self._manual_timer)
            self._manual_file.close()
        self.get_logger().info(f"Manual recording done: rows={self._manual_rows_written}")
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReachabilityTester()
    try:
        exit_code = node.run()
    except Exception as exc:  # keep CLI failures explicit in ROS logs
        node.get_logger().error(f"reachability_tester failed: {exc}")
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main(sys.argv)
