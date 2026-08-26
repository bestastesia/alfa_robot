#!/usr/bin/env python3
"""Rerun viewer for the synchronous V3 dual-arm Cartesian box demo."""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass

import rclpy
import rerun as rr
import rerun.blueprint as rrb
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
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
    stage: str
    joints: tuple[float, ...]
    box_center: tuple[float, float, float]


def maximum_joint_delta_degrees(
    previous: tuple[float, ...], current: tuple[float, ...]
) -> float:
    if len(previous) != len(current):
        return 0.0
    return math.degrees(
        max(
            abs(math.atan2(math.sin(after - before), math.cos(after - before)))
            for before, after in zip(previous, current)
        )
    )


class V3DualArmCartesianBoxViewer(Node):
    def __init__(self) -> None:
        super().__init__("v3_dual_arm_cartesian_box_viewer")
        self.declare_parameter(
            "task_topic", "/v3_dual_arm_cartesian_box_demo/task_json"
        )
        self.declare_parameter("spawn_viewer", True)
        self.declare_parameter("recording_path", "")
        self.declare_parameter("log_meshes", True)
        self.declare_parameter("playback_joint_speed_deg_s", 25.0)
        self.declare_parameter("maximum_frame_rate_hz", 30.0)

        topic = str(self.get_parameter("task_topic").value)
        spawn_viewer = bool(self.get_parameter("spawn_viewer").value)
        recording_path = str(self.get_parameter("recording_path").value)
        self.playback_joint_speed_deg_s = max(
            1.0, float(self.get_parameter("playback_joint_speed_deg_s").value)
        )
        self.minimum_frame_period = 1.0 / max(
            1.0, float(self.get_parameter("maximum_frame_rate_hz").value)
        )

        prefer_matching_rerun_cli()
        rr.init("v3_dual_arm_cartesian_box_demo", spawn=spawn_viewer)
        if recording_path:
            rr.save(recording_path)
            self.get_logger().info(f"Rerun recording: {recording_path}")
        self.robot = UrdfRobot(render_current_urdf())
        log_robot_static_model(
            self.robot,
            "world/robot",
            log_meshes=bool(self.get_parameter("log_meshes").value),
        )
        rr.set_time("task_frame", sequence=0)
        log_robot_state(self.robot, {}, "world/robot")
        rr.send_blueprint(
            rrb.Blueprint(
                rrb.Horizontal(
                    rrb.Spatial3DView(
                        origin="/world",
                        contents=["/world/**"],
                        name="V3 dual-arm synchronous Cartesian box",
                    ),
                    rrb.TextDocumentView(origin="/summary", name="Task status"),
                    column_shares=[0.78, 0.22],
                ),
                collapse_panels=True,
            )
        )

        self.frames: list[PlaybackFrame] = []
        self.joint_names: tuple[str, ...] = ()
        self.current_box_center = (0.73, 0.0, 0.55)
        self.target_box_center = (0.61, 0.0, 0.55)
        self.box_size = 0.40
        self.frame_index = 0
        self.global_frame = 1
        self.next_frame_time = time.monotonic()
        self.last_frame: PlaybackFrame | None = None

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.subscription = self.create_subscription(String, topic, self.on_task, qos)
        self.timer = self.create_timer(0.01, self.on_timer)
        self.get_logger().info(f"双臂同步笛卡尔Rerun播放器已就绪: topic={topic}")

    def on_task(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError as exception:
            self.get_logger().error(f"非法任务JSON: {exception}")
            return
        self.update_scene(payload)
        kind = str(payload.get("kind", "preview"))
        if kind == "preview":
            joint_names = tuple(str(name) for name in payload.get("joint_names", []))
            current_joints = tuple(
                float(value) for value in payload.get("current_joints", [])
            )
            if joint_names and len(joint_names) == len(current_joints):
                rr.set_time("task_frame", sequence=self.global_frame)
                self.global_frame += 1
                log_robot_state(
                    self.robot,
                    dict(zip(joint_names, current_joints)),
                    "world/robot",
                )
            self.log_summary(str(payload.get("status", "调整目标箱中心")))
            return
        if kind == "planning":
            self.frames = []
            self.log_summary("计算中……")
            return
        if kind != "result":
            return
        self.joint_names = tuple(str(name) for name in payload.get("joint_names", []))
        parsed: list[PlaybackFrame] = []
        for frame in payload.get("frames", []):
            joints = tuple(float(value) for value in frame.get("joints", []))
            center = tuple(float(value) for value in frame.get("box_center", []))
            if len(joints) != len(self.joint_names) or len(center) != 3:
                continue
            parsed.append(
                PlaybackFrame(
                    stage=str(frame.get("stage", "synchronized_cartesian")),
                    joints=joints,
                    box_center=center,
                )
            )
        self.frames = parsed
        self.frame_index = 0
        self.last_frame = None
        self.next_frame_time = time.monotonic()
        success = bool(payload.get("success", False))
        total_ms = float(payload.get("total_ms", 0.0))
        metrics = dict(payload.get("metrics", {}))
        failure = str(payload.get("failure_reason", ""))
        outcome = "成功" if success else f"失败：{failure}"
        self.log_summary(
            f"{outcome}\n\n总计算 {total_ms:.2f}ms；"
            f"解析IK {float(metrics.get('ik_ms', 0.0)):.2f}ms；"
            f"碰撞检测 {float(metrics.get('collision_ms', 0.0)):.2f}ms；"
            f"轨迹帧 {len(self.frames)}"
        )

    def update_scene(self, payload: dict) -> None:
        current = payload.get("current_box_center", [])
        target = payload.get("target_box_center", [])
        if len(current) == 3:
            self.current_box_center = tuple(float(value) for value in current)
        if len(target) == 3:
            self.target_box_center = tuple(float(value) for value in target)
        self.box_size = float(payload.get("box_size", self.box_size))
        self.log_boxes(self.current_box_center)

    def log_boxes(self, displayed_center: tuple[float, float, float]) -> None:
        half_size = [self.box_size * 0.5] * 3
        rr.log(
            "world/task/current_box",
            rr.Boxes3D(
                centers=[list(displayed_center)],
                half_sizes=[half_size],
                colors=[[40, 120, 255, 190]],
                labels=["40cm rigidly held cube"],
            ),
        )
        rr.log(
            "world/task/target_box",
            rr.Boxes3D(
                centers=[list(self.target_box_center)],
                half_sizes=[half_size],
                colors=[[55, 245, 80, 75]],
                labels=["target cube center"],
            ),
        )
        rr.log(
            "world/task/straight_line",
            rr.LineStrips3D(
                strips=[[list(self.current_box_center), list(self.target_box_center)]],
                colors=[[245, 50, 235]],
                radii=[0.008],
                labels=["synchronous analytic Cartesian line"],
            ),
        )

    def log_summary(self, text: str) -> None:
        rr.log(
            "summary",
            rr.TextDocument(
                "# V3双臂同步笛卡尔箱体Demo\n\n" + text,
                media_type=rr.MediaType.MARKDOWN,
            ),
        )

    def on_timer(self) -> None:
        if self.frame_index >= len(self.frames) or time.monotonic() < self.next_frame_time:
            return
        frame = self.frames[self.frame_index]
        rr.set_time("task_frame", sequence=self.global_frame)
        self.global_frame += 1
        joints = dict(zip(self.joint_names, frame.joints))
        log_robot_state(self.robot, joints, "world/robot")
        self.log_boxes(frame.box_center)
        rr.log("summary/current_stage", rr.TextLog(frame.stage))

        delay = self.minimum_frame_period
        if self.last_frame is not None:
            delay = max(
                delay,
                maximum_joint_delta_degrees(self.last_frame.joints, frame.joints)
                / self.playback_joint_speed_deg_s,
            )
        self.last_frame = frame
        self.frame_index += 1
        self.next_frame_time = time.monotonic() + delay


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V3DualArmCartesianBoxViewer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
