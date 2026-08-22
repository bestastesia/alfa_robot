#!/usr/bin/python3
"""PROTOTYPE Rerun view for target -> Motion command -> Motion TX batch.

There is intentionally no RT-Control state or simulated tracking in this view.
"""

from __future__ import annotations

import json

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from alfa_robot_rerun import visualize_rerun as helpers


def quaternion_matrix_xyzw(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion / max(float(np.linalg.norm(quaternion)), 1e-12)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def pose_matrix(message: PoseStamped) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = [
        message.pose.position.x,
        message.pose.position.y,
        message.pose.position.z,
    ]
    transform[:3, :3] = quaternion_matrix_xyzw(
        np.array(
            [
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            ],
            dtype=float,
        )
    )
    return transform


class Realtime6dPoseRerun(Node):
    def __init__(self) -> None:
        super().__init__("realtime_6d_pose_rerun_prototype")
        self.declare_parameter("spawn_viewer", True)
        self.declare_parameter("log_rate_hz", 30.0)
        self.declare_parameter("log_meshes", True)
        self.declare_parameter("arm", "left")

        try:
            import rerun as rr
        except ImportError as exc:
            raise RuntimeError("rerun-sdk is missing; run with enable_rerun:=false") from exc

        self.rr = rr
        self.rr.init("alfa_realtime_6d_pose_prototype")
        if bool(self.get_parameter("spawn_viewer").value):
            self.rr.spawn()

        self.robot = helpers.UrdfRobot(helpers.render_current_urdf())
        self.robot_path = "realtime/robot"
        helpers.log_robot_static_model(
            self.robot,
            self.robot_path,
            log_meshes=bool(self.get_parameter("log_meshes").value),
        )
        self.base_world = self.robot.fk({}).get("base_link", np.eye(4))
        arm = str(self.get_parameter("arm").value)
        if arm not in {"left", "right"}:
            raise ValueError("arm must be 'left' or 'right'")
        self.tool_link = f"{arm}_tool0"

        self.tx_preview_joints: dict[str, float] = {}
        self.commanded_joints: dict[str, float] = {}
        self.target: PoseStamped | None = None
        self.commanded_pose: PoseStamped | None = None
        self.motion_status: dict = {}
        self.tx_status: dict = {}
        self.rendered_once = False
        self.create_subscription(
            JointState,
            "/realtime_6d_pose/motion_tx_preview_joint_states",
            self.on_tx_preview_joints,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            "/realtime_6d_pose/commanded_joint_states",
            self.on_commanded_joints,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            "/realtime_6d_pose/target_pose",
            lambda message: setattr(self, "target", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            "/realtime_6d_pose/commanded_pose",
            lambda message: setattr(self, "commanded_pose", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            "/realtime_6d_pose/status",
            self.on_motion_status,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            "/realtime_6d_pose/motion_tx_status",
            self.on_tx_status,
            qos_profile_sensor_data,
        )
        rate = float(self.get_parameter("log_rate_hz").value)
        self.create_timer(1.0 / max(1.0, rate), self.render)
        self.get_logger().info(f"Rerun prototype ready: log_rate={rate:.1f}Hz")

    def on_tx_preview_joints(self, message: JointState) -> None:
        self.tx_preview_joints = {
            name: float(value) for name, value in zip(message.name, message.position)
        }

    def on_commanded_joints(self, message: JointState) -> None:
        self.commanded_joints = {
            name: float(value) for name, value in zip(message.name, message.position)
        }

    @staticmethod
    def parse_status(message: String) -> dict:
        try:
            return json.loads(message.data)
        except json.JSONDecodeError:
            return {"status": "invalid_status_json", "raw": message.data}

    def on_motion_status(self, message: String) -> None:
        self.motion_status = self.parse_status(message)

    def on_tx_status(self, message: String) -> None:
        self.tx_status = self.parse_status(message)

    def world_pose(self, message: PoseStamped) -> np.ndarray:
        if message.header.frame_id in {"", "base_link"}:
            return self.base_world @ pose_matrix(message)
        return pose_matrix(message)

    def log_pose(self, path: str, message: PoseStamped, label: str, color: list[int]) -> np.ndarray:
        transform = self.world_pose(message)
        return self.log_matrix_pose(path, transform, label, color)

    def log_matrix_pose(
        self, path: str, transform: np.ndarray, label: str, color: list[int]
    ) -> np.ndarray:
        quaternion = np.array(helpers.matrix_to_quaternion_xyzw(transform[:3, :3]))
        helpers.log_transform(path, transform[:3, 3], quaternion)
        self.rr.log(
            f"{path}/point",
            self.rr.Points3D([transform[:3, 3]], colors=color, radii=0.018, labels=[label]),
        )
        return transform[:3, 3]

    def render(self) -> None:
        if not self.motion_status:
            return
        sequence = int(self.tx_status.get("sequence", 0))
        helpers.set_sample_time(sequence)
        if self.tx_preview_joints:
            helpers.log_robot_state(self.robot, self.tx_preview_joints, self.robot_path)

        target_point = None
        commanded_point = None
        tx_endpoint = None
        if self.target is not None:
            target_point = self.log_pose("realtime/target", self.target, "6D target", [255, 165, 0])
        if self.commanded_pose is not None:
            commanded_point = self.log_pose(
                "realtime/motion_command",
                self.commanded_pose,
                "Motion accepted",
                [80, 220, 255],
            )
        if self.tx_preview_joints:
            tx_transform = self.robot.fk(self.tx_preview_joints).get(self.tool_link)
            if tx_transform is not None:
                tx_endpoint = self.log_matrix_pose(
                    "realtime/motion_tx_endpoint",
                    tx_transform,
                    "Motion TX suffix endpoint",
                    [210, 100, 255],
                )
        if target_point is not None and commanded_point is not None:
            self.rr.log(
                "realtime/target_to_command",
                self.rr.LineStrips3D([[target_point, commanded_point]], colors=[255, 190, 50]),
            )
        if commanded_point is not None and tx_endpoint is not None:
            self.rr.log(
                "realtime/command_to_tx_endpoint",
                self.rr.LineStrips3D(
                    [[commanded_point, tx_endpoint]], colors=[210, 100, 255]
                ),
            )

        active_points = self.tx_status.get("points", [])
        point_lines = []
        for index, point in enumerate(active_points):
            positions = point.get("active_positions_deg", [])
            velocities = point.get("active_velocities_deg_s", [])
            position_text = " ".join(f"{float(value):+6.1f}" for value in positions)
            velocity_text = " ".join(f"{float(value):+5.1f}" for value in velocities)
            point_lines.append(
                f"  p{index} t={float(point.get('time_ms', 0.0)):.1f}ms "
                f"q_deg=[{position_text}] v_deg_s=[{velocity_text}]"
            )
        lines = [
            "PROTOTYPE — Motion TX rolling-message preview",
            "RT-Control: NOT STARTED / NOT SIMULATED",
            f"Motion: {self.motion_status.get('status', '-')} / "
            f"accepted={self.motion_status.get('accepted', False)} / "
            f"{float(self.motion_status.get('loop_hz', 0.0)):.1f} Hz",
            f"Motion total / IK / collision: "
            f"{float(self.motion_status.get('total_ms', 0.0)):.3f} / "
            f"{float(self.motion_status.get('ik_ms', 0.0)):.3f} / "
            f"{float(self.motion_status.get('collision_ms', 0.0)):.3f} ms",
            f"IK solutions / max jump: {self.motion_status.get('solutions', 0)} / "
            f"{float(self.motion_status.get('max_joint_delta_deg', 0.0)):.3f} deg",
            f"Motion TX topic: {self.tx_status.get('topic', '-')}",
            f"message: {self.tx_status.get('message_type', '-')}",
            f"TX sequence / source: {sequence} / {self.tx_status.get('source', '-')}",
            f"requested / measured TX: "
            f"{float(self.tx_status.get('requested_batch_hz', 0.0)):.1f} / "
            f"{float(self.tx_status.get('measured_batch_hz', 0.0)):.1f} Hz",
            f"replace_from / buffered_until: "
            f"{float(self.tx_status.get('replace_from_ns') or 0) * 1e-6:.1f} / "
            f"{float(self.tx_status.get('buffered_until_ns') or 0) * 1e-6:.1f} ms",
            f"points / knot / horizon: {self.tx_status.get('point_count', 0)} / "
            f"{float(self.tx_status.get('knot_period_ms', 0.0)):.1f} / "
            f"{float(self.tx_status.get('horizon_ms', 0.0)):.1f} ms",
            f"target scale / local rejects: "
            f"{float(self.tx_status.get('target_scale', 0.0)):.3f} / "
            f"{self.tx_status.get('local_reject_count', 0)}",
            f"continuous profile duration / replanned: "
            f"{float(self.tx_status.get('profile_duration_s', 0.0)):.3f}s / "
            f"{self.tx_status.get('profile_replanned', False)}",
            f"limits source: {self.tx_status.get('limits_source', '-')}",
            f"last Motion TX error: {self.tx_status.get('last_error', '-')}",
            "active-arm points in the current outgoing batch:",
            *point_lines,
            f"fixed updown: {float(self.motion_status.get('fixed_updown', 0.0)):.3f} m",
        ]
        self.rr.log("realtime/status", self.rr.TextLog("\n".join(lines)))
        if not self.rendered_once:
            self.get_logger().info(
                f"first Rerun frame received: sequence={sequence} "
                f"tx_preview_joints={len(self.tx_preview_joints)} "
                f"commanded_joints={len(self.commanded_joints)} "
                f"target={self.target is not None} "
                f"commanded={self.commanded_pose is not None} "
                f"tx_status={bool(self.tx_status)}"
            )
            self.rendered_once = True


def main() -> None:
    rclpy.init()
    node = Realtime6dPoseRerun()
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
