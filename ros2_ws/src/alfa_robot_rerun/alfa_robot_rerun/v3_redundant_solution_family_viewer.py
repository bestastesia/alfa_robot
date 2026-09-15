#!/usr/bin/env python3
"""Live Rerun playback for every sampled V3 redundant analytic-IK family."""

from __future__ import annotations

from alfa_robot_rerun.demo_failure import log_failure

import json
import math
import time
from dataclasses import dataclass

import rclpy
import rerun as rr
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from alfa_robot_rerun.visualize_rerun import (
    UrdfRobot,
    log_robot_state,
    log_robot_static_model,
    prefer_matching_rerun_cli,
    render_current_urdf,
)


@dataclass(frozen=True)
class PlaybackFrame:
    segment_id: int
    branch_label: str
    psi_deg: float
    joints: tuple[float, ...]
    limit_margin_deg: float


def wrapped_joint_delta_degrees(
    previous: tuple[float, ...], current: tuple[float, ...]
) -> float:
    return math.degrees(
        max(
            abs(math.atan2(math.sin(after - before), math.cos(after - before)))
            for before, after in zip(previous, current)
        )
    )


class V3RedundantSolutionFamilyViewer(Node):
    def __init__(self) -> None:
        super().__init__("v3_redundant_solution_family_viewer")
        self.declare_parameter(
            "solution_family_topic",
            "/v3_redundant_ik_interactive_demo/solution_family_json",
        )
        self.declare_parameter("spawn_viewer", True)
        self.declare_parameter("recording_path", "")
        self.declare_parameter("log_meshes", True)
        self.declare_parameter("loop_playback", True)
        self.declare_parameter("playback_joint_speed_deg_s", 30.0)
        self.declare_parameter("maximum_frame_rate_hz", 60.0)
        self.declare_parameter("segment_pause_s", 0.5)

        topic = str(self.get_parameter("solution_family_topic").value)
        spawn_viewer = bool(self.get_parameter("spawn_viewer").value)
        recording_path = str(self.get_parameter("recording_path").value)
        log_meshes = bool(self.get_parameter("log_meshes").value)
        self.loop_playback = bool(self.get_parameter("loop_playback").value)
        self.playback_joint_speed_deg_s = max(
            1.0, float(self.get_parameter("playback_joint_speed_deg_s").value)
        )
        self.minimum_frame_period = 1.0 / max(
            1.0, float(self.get_parameter("maximum_frame_rate_hz").value)
        )
        self.segment_pause_s = max(
            0.0, float(self.get_parameter("segment_pause_s").value)
        )

        prefer_matching_rerun_cli()
        rr.init("v3_redundant_solution_family", spawn=spawn_viewer)
        if recording_path:
            rr.save(recording_path)
            self.get_logger().info(f"Rerun recording: {recording_path}")
        self.robot = UrdfRobot(render_current_urdf())
        log_robot_static_model(self.robot, "world/robot", log_meshes=log_meshes)
        rr.set_time("solution_frame", sequence=0)
        log_robot_state(self.robot, {}, "world/robot")

        self.frames: list[PlaybackFrame] = []
        self.side = "left"
        self.target_pose: dict = {}
        self.generation = 0
        self.frame_index = 0
        self.global_frame = 1
        self.next_frame_time = time.monotonic()
        self.last_frame: PlaybackFrame | None = None

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.subscription = self.create_subscription(String, topic, self.on_family, qos)
        self.timer = self.create_timer(0.01, self.on_timer)
        self.get_logger().info(
            f"Rerun冗余解族播放器已就绪: topic={topic} "
            f"speed={self.playback_joint_speed_deg_s:.1f}deg/s"
        )

    def on_family(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            frames: list[PlaybackFrame] = []
            for segment in payload.get("segments", []):
                branch = segment.get("branch", {})
                branch_label = (
                    f"S{branch.get('shoulder', 0):+d}/"
                    f"E{branch.get('elbow', 0):+d}/"
                    f"W{branch.get('wrist', 0):+d}"
                )
                segment_id = int(segment.get("segment_id", len(frames)))
                for sample in segment.get("samples", []):
                    joints = tuple(float(value) for value in sample["joints"])
                    if len(joints) != 7:
                        continue
                    frames.append(
                        PlaybackFrame(
                            segment_id=segment_id,
                            branch_label=branch_label,
                            psi_deg=float(sample["psi_deg"]),
                            joints=joints,
                            limit_margin_deg=float(
                                sample.get("minimum_joint_limit_margin_deg", 0.0)
                            ),
                        )
                    )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exception:
            self.get_logger().error(f"非法冗余解族消息: {exception}")
            return

        rr.log("world/failure", rr.Clear(recursive=True))
        rr.log("summary/failure", rr.Clear(recursive=True))
        log_failure(payload.get("diagnostic", {}))
        self.frames = frames
        self.side = str(payload.get("side", "left"))
        self.target_pose = dict(payload.get("target_pose", {}))
        self.generation = int(payload.get("generation", self.generation + 1))
        self.frame_index = 0
        self.last_frame = None
        self.next_frame_time = time.monotonic()
        summary = (
            f"generation={self.generation} side={self.side} "
            f"solutions={payload.get('valid_solution_count', len(frames))} "
            f"segments={payload.get('segment_count', 0)} "
            f"solve={float(payload.get('solve_ms', 0.0)):.2f}ms"
        )
        rr.log("diagnostics/family_summary", rr.TextLog(summary))
        self.log_target()
        self.get_logger().info(summary)

    def log_target(self) -> None:
        position = self.target_pose.get("position", [])
        orientation = self.target_pose.get("orientation_xyzw", [])
        if len(position) != 3 or len(orientation) != 4:
            return
        rr.log(
            "world/target",
            rr.Transform3D(translation=position, quaternion=orientation),
        )
        rr.log(
            "world/target/point",
            rr.Points3D(
                positions=[[0.0, 0.0, 0.0]],
                colors=[[30, 145, 255] if self.frames else [255, 30, 30]],
                radii=[0.035],
                labels=[f"target generation {self.generation}"],
            ),
        )

    def on_timer(self) -> None:
        if not self.frames or time.monotonic() < self.next_frame_time:
            return
        if self.frame_index >= len(self.frames):
            if not self.loop_playback:
                return
            self.frame_index = 0
            self.last_frame = None
            self.next_frame_time = time.monotonic() + self.segment_pause_s
            return

        frame = self.frames[self.frame_index]
        rr.set_time("solution_frame", sequence=self.global_frame)
        joint_positions = {
            f"{self.side}_joint{index + 1}": value
            for index, value in enumerate(frame.joints)
        }
        log_robot_state(self.robot, joint_positions, "world/robot")
        rr.log(
            "diagnostics/current_solution",
            rr.TextLog(
                f"generation={self.generation} segment={frame.segment_id} "
                f"branch={frame.branch_label} psi={frame.psi_deg:+.1f}deg "
                f"limit_margin={frame.limit_margin_deg:.1f}deg"
            ),
        )
        self.log_target()

        delay = self.minimum_frame_period
        if self.last_frame is not None:
            if self.last_frame.segment_id != frame.segment_id:
                delay = self.segment_pause_s
            else:
                delta = wrapped_joint_delta_degrees(
                    self.last_frame.joints, frame.joints
                )
                delay = max(
                    self.minimum_frame_period,
                    min(0.25, delta / self.playback_joint_speed_deg_s),
                )
        self.last_frame = frame
        self.frame_index += 1
        self.global_frame += 1
        self.next_frame_time = time.monotonic() + delay


def main() -> None:
    rclpy.init()
    node = V3RedundantSolutionFamilyViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
