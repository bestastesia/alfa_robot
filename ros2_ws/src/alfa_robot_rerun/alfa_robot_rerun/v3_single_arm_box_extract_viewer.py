#!/usr/bin/env python3
"""Live Rerun scene preview and trajectory playback for the V3 box-extract demo."""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass

import numpy as np
import rclpy
import rerun as rr
import rerun.blueprint as rrb
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
    stage: str
    joints: tuple[float, ...]
    box_attached: bool


def matrix_to_quaternion(matrix: np.ndarray) -> list[float]:
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            w = (matrix[2, 1] - matrix[1, 2]) / scale
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            w = (matrix[0, 2] - matrix[2, 0]) / scale
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            w = (matrix[1, 0] - matrix[0, 1]) / scale
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
    return [float(x), float(y), float(z), float(w)]


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


class V3SingleArmBoxExtractViewer(Node):
    def __init__(self) -> None:
        super().__init__("v3_single_arm_box_extract_viewer")
        self.declare_parameter(
            "task_topic", "/v3_single_arm_box_extract_demo/task_json"
        )
        self.declare_parameter("spawn_viewer", True)
        self.declare_parameter("recording_path", "")
        self.declare_parameter("log_meshes", True)
        self.declare_parameter("playback_joint_speed_deg_s", 25.0)
        self.declare_parameter("maximum_frame_rate_hz", 30.0)
        self.declare_parameter("stage_pause_s", 0.35)

        topic = str(self.get_parameter("task_topic").value)
        spawn_viewer = bool(self.get_parameter("spawn_viewer").value)
        recording_path = str(self.get_parameter("recording_path").value)
        log_meshes = bool(self.get_parameter("log_meshes").value)
        self.playback_joint_speed_deg_s = max(
            1.0, float(self.get_parameter("playback_joint_speed_deg_s").value)
        )
        self.minimum_frame_period = 1.0 / max(
            1.0, float(self.get_parameter("maximum_frame_rate_hz").value)
        )
        self.stage_pause_s = max(0.0, float(self.get_parameter("stage_pause_s").value))

        prefer_matching_rerun_cli()
        rr.init("v3_single_arm_box_extract_demo", spawn=spawn_viewer)
        if recording_path:
            rr.save(recording_path)
            self.get_logger().info(f"Rerun recording: {recording_path}")
        self.robot = UrdfRobot(render_current_urdf())
        log_robot_static_model(self.robot, "world/robot", log_meshes=log_meshes)
        rr.set_time("task_frame", sequence=0)
        log_robot_state(self.robot, {}, "world/robot")
        rr.send_blueprint(
            rrb.Blueprint(
                rrb.Horizontal(
                    rrb.Spatial3DView(
                        origin="/world",
                        contents=["/world/**"],
                        name="V3 single-arm box extraction",
                    ),
                    rrb.TextDocumentView(origin="/summary", name="Task status"),
                    column_shares=[0.78, 0.22],
                ),
                collapse_panels=True,
            )
        )

        self.frames: list[PlaybackFrame] = []
        self.joint_names: tuple[str, ...] = ()
        self.box_center = np.zeros(3)
        self.box_size = np.array([0.30, 0.40, 0.40])
        self.neighbor_centers: list[list[float]] = []
        self.tool_to_box_center = np.array([0.0, 0.0, 0.15])
        self.tool_link = "left_tool0"
        self.generation = 0
        self.frame_index = 0
        self.global_frame = 1
        self.next_frame_time = time.monotonic()
        self.last_frame: PlaybackFrame | None = None
        self.success = False
        self.failure_stage = ""
        self.failure_reason = ""
        self.total_ms = 0.0
        self.metrics: dict = {}

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.subscription = self.create_subscription(String, topic, self.on_task, qos)
        self.timer = self.create_timer(0.01, self.on_timer)
        self.get_logger().info(f"Rerun抽箱播放器已就绪: topic={topic}")

    def on_task(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError as exception:
            self.get_logger().error(f"非法抽箱JSON: {exception}")
            return

        self.update_scene(payload)
        kind = str(payload.get("kind", "preview"))
        if kind == "preview":
            self.log_summary(str(payload.get("status", "调整箱体位置")), planning=False)
            return
        if kind == "planning":
            self.generation = int(payload.get("generation", self.generation + 1))
            self.frames = []
            self.log_summary("计算中……", planning=True)
            self.get_logger().info(
                f"generation={self.generation} 计算开始 box={self.box_center.tolist()}"
            )
            return
        if kind != "result":
            return

        parsed_frames: list[PlaybackFrame] = []
        joint_names = tuple(str(name) for name in payload.get("joint_names", []))
        for frame in payload.get("frames", []):
            joints = tuple(float(value) for value in frame.get("joints", []))
            if not joint_names or len(joints) != len(joint_names):
                continue
            parsed_frames.append(
                PlaybackFrame(
                    stage=str(frame.get("stage", "unknown")),
                    joints=joints,
                    box_attached=bool(frame.get("box_attached", False)),
                )
            )
        self.joint_names = joint_names
        self.frames = parsed_frames
        self.generation = int(payload.get("generation", self.generation + 1))
        self.success = bool(payload.get("success", False))
        self.failure_stage = str(payload.get("failure_stage", ""))
        self.failure_reason = str(payload.get("failure_reason", ""))
        self.total_ms = float(payload.get("total_ms", 0.0))
        self.metrics = dict(payload.get("metrics", {}))
        self.tool_link = str(payload.get("tool_link", self.tool_link))
        self.frame_index = 0
        self.last_frame = None
        self.next_frame_time = time.monotonic()
        self.log_summary("计算完成，开始播放", planning=False)
        outcome = "SUCCESS" if self.success else "FAILED"
        self.get_logger().info(
            f"generation={self.generation} {outcome} total={self.total_ms:.2f}ms "
            f"frames={len(self.frames)} stage={self.failure_stage or '-'} "
            f"reason={self.failure_reason or '-'}"
        )

    def update_scene(self, payload: dict) -> None:
        center = payload.get("box_center", [])
        size = payload.get("box_size", [])
        if len(center) == 3:
            self.box_center = np.asarray(center, dtype=float)
        if len(size) == 3:
            self.box_size = np.asarray(size, dtype=float)
        self.neighbor_centers = [
            [float(value) for value in item]
            for item in payload.get("neighbor_centers", [])
            if len(item) == 3
        ]
        tool_offset = payload.get("tool_to_box_center", [])
        if len(tool_offset) == 3:
            self.tool_to_box_center = np.asarray(tool_offset, dtype=float)
        self.log_scene_points(payload)
        self.log_boxes(None, attached=False)

    def log_scene_points(self, payload: dict) -> None:
        positions = []
        labels = []
        colors = []
        for key, label, rgb in (
            ("precontact", "pre-contact", [45, 145, 255]),
            ("contact", "contact", [45, 235, 70]),
            ("retreat", "retreat 35cm", [235, 65, 225]),
        ):
            point = payload.get(key, [])
            if len(point) != 3:
                continue
            positions.append([float(value) for value in point])
            labels.append(label)
            colors.append(rgb)
        if positions:
            rr.log(
                "world/task_points",
                rr.Points3D(
                    positions=positions,
                    colors=colors,
                    radii=[0.025],
                    labels=labels,
                ),
            )
            rr.log(
                "world/cartesian_retreat_axis",
                rr.LineStrips3D(
                    strips=[[positions[1], positions[2]]],
                    colors=[[235, 65, 225]],
                    radii=[0.008],
                    labels=["35cm analytic Cartesian retreat"],
                ),
            )

    def log_boxes(
        self, joint_positions: dict[str, float] | None, *, attached: bool
    ) -> None:
        if self.neighbor_centers:
            rr.log(
                "world/boxes/neighbors",
                rr.Boxes3D(
                    centers=self.neighbor_centers,
                    half_sizes=[(self.box_size * 0.5).tolist()],
                    colors=[[255, 125, 25, 125]],
                    labels=[f"neighbor {index + 1}" for index in range(len(self.neighbor_centers))],
                ),
            )
        if not attached or joint_positions is None:
            rr.log("world/boxes/carried", rr.Clear(recursive=True))
            rr.log(
                "world/boxes/target",
                rr.Boxes3D(
                    centers=[self.box_center.tolist()],
                    half_sizes=[(self.box_size * 0.5).tolist()],
                    colors=[[45, 220, 75, 180]],
                    labels=["target box"],
                ),
            )
            return

        transforms = self.robot.fk(joint_positions)
        tool_transform = transforms.get(self.tool_link)
        if tool_transform is None:
            return
        box_transform = tool_transform.copy()
        box_transform[:3, 3] = (
            tool_transform[:3, 3]
            + tool_transform[:3, :3] @ self.tool_to_box_center
        )
        rr.log("world/boxes/target", rr.Clear(recursive=True))
        rr.log(
            "world/boxes/carried",
            rr.Boxes3D(
                centers=[box_transform[:3, 3].tolist()],
                half_sizes=[
                    [
                        float(self.box_size[2] * 0.5),
                        float(self.box_size[1] * 0.5),
                        float(self.box_size[0] * 0.5),
                    ]
                ],
                quaternions=[matrix_to_quaternion(box_transform[:3, :3])],
                colors=[[45, 225, 100, 190]],
                labels=["carried target box"],
            ),
        )

    def log_summary(self, status: str, *, planning: bool) -> None:
        if planning:
            body = (
                "# V3单臂抽箱Demo\n\n"
                f"- 状态：**{status}**\n"
                f"- 箱体中心：`{self.box_center.tolist()}`\n"
                "- 正在计算：RRT到预接触 → 解析直线接触 → 解析直线抽出 → 携箱RRT返回"
            )
        elif self.metrics:
            result_text = "成功" if self.success else "失败"
            failure = ""
            if not self.success:
                failure = (
                    f"\n- 失败阶段：`{self.failure_stage}`"
                    f"\n- 失败原因：`{self.failure_reason}`"
                )
            body = (
                "# V3单臂抽箱Demo\n\n"
                f"- 状态：**{result_text}**（{status}）\n"
                f"- 总计算：**{self.total_ms:.2f} ms**\n"
                f"- 解析路径：{float(self.metrics.get('analytic_path_ms', 0.0)):.2f} ms\n"
                f"- RRT到预接触：{float(self.metrics.get('rrt_approach_ms', 0.0)):.2f} ms\n"
                f"- 携箱RRT返回：{float(self.metrics.get('rrt_return_ms', 0.0)):.2f} ms\n"
                f"- 解析IK：{int(self.metrics.get('ik_calls', 0))}次 / "
                f"{float(self.metrics.get('ik_ms', 0.0)):.2f} ms\n"
                f"- 碰撞检测：{int(self.metrics.get('collision_checks', 0))}次 / "
                f"{float(self.metrics.get('collision_ms', 0.0)):.2f} ms"
                f"{failure}"
            )
        else:
            body = (
                "# V3单臂抽箱Demo\n\n"
                f"- 状态：**{status}**\n"
                f"- 箱体中心：`{self.box_center.tolist()}`\n"
                "- 在RViz拖动绿色箱体XYZ；右键箱体确认后才开始计算。"
            )
        rr.log(
            "summary",
            rr.TextDocument(body, media_type=rr.MediaType.MARKDOWN),
        )

    def on_timer(self) -> None:
        if not self.frames or time.monotonic() < self.next_frame_time:
            return
        if self.frame_index >= len(self.frames):
            self.frames = []
            self.log_summary("播放完成", planning=False)
            return

        frame = self.frames[self.frame_index]
        rr.set_time("task_frame", sequence=self.global_frame)
        joint_positions = dict(zip(self.joint_names, frame.joints))
        log_robot_state(self.robot, joint_positions, "world/robot")
        self.log_boxes(joint_positions, attached=frame.box_attached)
        rr.log(
            "world/current_stage",
            rr.TextLog(
                f"generation={self.generation} frame={self.frame_index + 1}/{len(self.frames)} "
                f"stage={frame.stage} attached={frame.box_attached}"
            ),
        )

        delay = self.minimum_frame_period
        if self.last_frame is not None:
            if self.last_frame.stage != frame.stage:
                delay = self.stage_pause_s
            else:
                delta = maximum_joint_delta_degrees(self.last_frame.joints, frame.joints)
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
    node = V3SingleArmBoxExtractViewer()
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
