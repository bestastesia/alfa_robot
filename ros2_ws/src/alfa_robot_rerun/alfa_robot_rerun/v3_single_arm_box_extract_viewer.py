#!/usr/bin/env python3
"""Append checked V3 box trajectories to Rerun; the viewer controls playback."""

from __future__ import annotations

from alfa_robot_rerun.demo_failure import replay_frames, log_failure
from alfa_robot_rerun.sequence_timeline import SequenceTimeline

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
    box_visible: bool = True
    scene_index: int = 0
    carried_boxes: tuple[dict, ...] = ()


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


class V3SingleArmBoxExtractViewer(Node):
    def __init__(self) -> None:
        super().__init__("v3_single_arm_box_extract_viewer")
        self.declare_parameter(
            "task_topic", "/v3_single_arm_box_extract_demo/task_json"
        )
        self.declare_parameter("robot_description", "")
        self.declare_parameter("spawn_viewer", True)
        self.declare_parameter("recording_path", "")
        self.declare_parameter("log_meshes", True)
        topic = str(self.get_parameter("task_topic").value)
        spawn_viewer = bool(self.get_parameter("spawn_viewer").value)
        recording_path = str(self.get_parameter("recording_path").value)
        log_meshes = bool(self.get_parameter("log_meshes").value)
        prefer_matching_rerun_cli()
        rr.init("v3_single_arm_box_extract_demo", spawn=spawn_viewer)
        if recording_path:
            rr.save(recording_path)
            self.get_logger().info(f"Rerun recording: {recording_path}")
        self.robot = UrdfRobot(str(self.get_parameter("robot_description").value) or render_current_urdf())
        log_robot_static_model(self.robot, "world/robot", log_meshes=log_meshes)
        rr.set_time("task_frame", sequence=0)
        # Wait for the planner's initial_joints; do not insert an all-zero preview frame.
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
                rrb.TimePanel(timeline="task_frame", expanded=True, fps=20,
                              play_state="Paused", loop_mode="Off"),
                collapse_panels=False,
            )
        )

        self.sequence_timeline = SequenceTimeline()
        self.sequence_started = time.perf_counter()
        self.scenes: list[dict] = []
        self.scene_index = -1
        self.tool_to_box_rotation = np.eye(3)
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
        self.last_frame: PlaybackFrame | None = None
        self.success = False
        self.failure_stage = ""
        self.failure_reason = ""
        self.total_ms = 0.0
        self.metrics: dict = {}
        self.wall_request: dict = {}

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.subscription = self.create_subscription(String, topic, self.on_task, qos)
        segment_qos = QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.segment_subscription = self.create_subscription(
            String, topic + "_segments", self.on_task, segment_qos)
        self.get_logger().info(f"Rerun轨迹时间轴已就绪（逐箱批量追加，界面控制播放）: topic={topic}")

    def on_task(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError as exception:
            self.get_logger().error(f"非法抽箱JSON: {exception}")
            return

        if not isinstance(payload, dict) or (payload.get("kind") == "segment" and not payload.get("task_id")):
            self.get_logger().error("非法任务JSON或分段缺少task_id")
            return
        if payload.get("task_id"):
            previous = self.sequence_timeline.task_id
            try:
                if not self.sequence_timeline.select(payload):
                    return
            except (KeyError, TypeError, ValueError) as error:
                self.get_logger().error(f"非法任务标识: {error}")
                return
            if previous != self.sequence_timeline.task_id:
                self.sequence_started = time.perf_counter()
            if payload.get("kind") == "planning" and (self.sequence_timeline.frames or self.sequence_timeline.pending):
                return  # Cross-topic delivery may put planning after the first segment.
        if payload.get("sequence") and payload.get("kind") in ("segment", "result") and "frame_end" in payload:
            try:
                # Validate before retaining a range, not after advancing the append cursor.
                names = payload["joint_names"]
                for frame in replay_frames(payload):
                    joints = frame["joints"]
                    if not names or len(joints) != len(names) or not all(map(math.isfinite, joints)):
                        raise ValueError("invalid trajectory joints")
                ready = self.sequence_timeline.append(payload)
            except (KeyError, TypeError, ValueError) as error:
                self.get_logger().error(f"拒绝序列分段（等待完整结果补齐）: {error}")
                return
            for segment in ready:
                self.write_payload(segment)
            if self.sequence_timeline.pending:
                self.get_logger().warning(f"RERUN_SEQUENCE_GAP next_frame={len(self.sequence_timeline.frames)} "
                                          f"pending={sorted(self.sequence_timeline.pending)}")
            if payload["kind"] == "result":
                self.get_logger().info(f"RERUN_SEQUENCE_READY task_id={payload['task_id']} "
                    f"frames={len(self.sequence_timeline.frames)} appended={sum(len(p['frames']) for p in ready)} "
                    f"receive_to_ready_s={time.perf_counter() - self.sequence_started:.3f} "
                    f"planning_ms={payload.get('total_ms', 0):.3f}")
            return
        self.write_payload(payload)

    def write_payload(self, payload: dict) -> None:
        started = time.perf_counter()
        # A new message must not overwrite the previous result's final frame.
        rr.set_time("task_frame", sequence=self.global_frame)
        self.wall_request = payload if payload.get("distance_demo") else {}
        self.scenes = payload.get("scenes", [])
        self.scene_index = -1
        if payload.get("kind") in ("preview", "planning"):
            self.frames = []
            self.metrics = {}
            self.last_frame = None
            log_robot_state(self.robot, dict(zip(payload.get("joint_names", []),
                                                payload.get("initial_joints", []))), "world/robot")
        self.tool_link = str(payload.get("tool_link", self.tool_link))
        self.diagnostic = payload.get("diagnostic", {}) if not payload.get("success", False) else {}
        rr.log("world/failure", rr.Clear(recursive=True))
        rr.log("summary/failure", rr.Clear(recursive=True))
        # Segment scenes become visible only at their first trajectory tick.
        if not payload.get("stream_segment"):
            self.update_scene(payload)
        kind = str(payload.get("kind", "preview"))
        if kind == "preview":
            self.log_summary(str(payload.get("status", "调整箱体位置")), planning=False)
            self.global_frame += 1
            return
        if kind == "planning":
            self.generation = int(payload.get("generation", self.generation + 1))
            self.frames = []
            self.log_summary("计算中……", planning=True)
            self.get_logger().info(
                f"generation={self.generation} 计算开始 box={self.box_center.tolist()}"
            )
            self.global_frame += 1
            return
        if kind != "result":
            return

        parsed_frames: list[PlaybackFrame] = []
        joint_names = tuple(str(name) for name in payload.get("joint_names", []))
        for frame in replay_frames(payload):
            joints = tuple(float(value) for value in frame.get("joints", []))
            if not joint_names or len(joints) != len(joint_names) or not all(map(math.isfinite, joints)):
                self.frames = []
                self.get_logger().error("非法轨迹关节帧：拒绝整段，不丢帧播放")
                return
            scene_index = int(frame.get("scene_index", 0))
            if self.scenes and not 0 <= scene_index < len(self.scenes):
                self.frames = []
                self.get_logger().error(f"非法场景索引: {scene_index}")
                return
            parsed_frames.append(
                PlaybackFrame(
                    stage=str(frame.get("stage", "unknown")),
                    joints=joints,
                    box_attached=bool(frame.get("box_attached", False)),
                    box_visible=bool(frame.get("box_visible", True)),
                    scene_index=scene_index,
                    carried_boxes=tuple(dict(box) for box in frame.get("carried_boxes", [])),
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
        self.stream_segment = bool(payload.get("stream_segment"))
        self.log_summary("当前箱子规划完成，正在追加轨迹；后台继续规划" if self.stream_segment else
                         "计算完成，正在写入完整时间轴", planning=False)
        outcome = "SUCCESS" if self.success else "FAILED"
        self.get_logger().info(
            f"generation={self.generation} {outcome} total={self.total_ms:.2f}ms "
            f"frames={len(self.frames)} stage={self.failure_stage or '-'} "
            f"reason={self.failure_reason or '-'}"
        )
        self.write_trajectory()
        self.write_elapsed_s = time.perf_counter() - started
        self.get_logger().info(
            f"RERUN_TIMELINE_READY generation={self.generation} frames={len(self.frames)} "
            f"write_elapsed_s={self.write_elapsed_s:.3f} task_id={payload.get('task_id', '-')} "
            f"range=[{payload.get('frame_begin', 0)},{payload.get('frame_end', len(self.frames))}) "
            "(含FK/写入/flush，不含规划；界面控制播放)")

    def update_scene(self, payload: dict, *, draw_boxes: bool = True) -> None:
        self.tool_link = str(payload.get("tool_link", self.tool_link))
        # Old recordings stored permuted dimensions in TCP axes; preserve their geometry.
        self.tool_to_box_rotation = np.asarray(payload.get("tool_to_box_rotation",
            [[0, 0, -1], [0, 1, 0], [1, 0, 0]]), dtype=float)
        # Clear first: a restarted planner may supply a different scene (or disabled regression mode).
        rr.log("world/environment", rr.Clear(recursive=True))
        for box in payload.get("environment", {}).get("boxes", []):
            rr.log(f"world/environment/{box['id']}", rr.Boxes3D(
                centers=[box["center"]], half_sizes=[np.asarray(box["size"]) / 2.0],
                colors=[[115, 140, 166, 50]], labels=[box["id"]]))
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
        rr.log("world/boxes/neighbors", rr.Clear(recursive=True))
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
        if draw_boxes:
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
        self, joint_positions: dict[str, float] | None, *, attached: bool, visible: bool = True,
        transforms: dict[str, np.ndarray] | None = None, carried_boxes: tuple[dict, ...] = ()
    ) -> None:
        if carried_boxes:
            rr.log("world/boxes/target", rr.Clear(recursive=True))
            rr.log("world/boxes/carried", rr.Clear(recursive=True))
            if transforms is None and joint_positions is not None:
                transforms = self.robot.fk(joint_positions)
            for box in carried_boxes:
                box_id = int(box.get("box_id", -1))
                if not bool(box.get("visible", True)):
                    continue
                center = np.asarray(box.get("box_center", self.box_center), dtype=float)
                if not bool(box.get("attached", False)) or transforms is None:
                    rr.log(f"world/boxes/target/{box_id}", rr.Boxes3D(
                        centers=[center.tolist()], half_sizes=[(self.box_size * 0.5).tolist()],
                        colors=[[45, 220, 75, 180]], labels=[f"target box {box_id}"]))
                    continue
                tool_transform = transforms.get(str(box.get("tool_link", "")))
                if tool_transform is None:
                    continue
                rotation = np.asarray(box.get("tool_to_box_rotation", np.eye(3)), dtype=float)
                offset = np.asarray(box.get("tool_to_box_center", [0.0, 0.0, 0.0]), dtype=float)
                box_transform = tool_transform.copy()
                box_transform[:3, :3] = tool_transform[:3, :3] @ rotation
                box_transform[:3, 3] = tool_transform[:3, 3] + tool_transform[:3, :3] @ offset
                rr.log(f"world/boxes/carried/{box_id}", rr.Boxes3D(
                    centers=[box_transform[:3, 3].tolist()],
                    half_sizes=[(self.box_size * 0.5).tolist()],
                    quaternions=[matrix_to_quaternion(box_transform[:3, :3])],
                    colors=[[45, 225, 100, 190]], labels=[f"carried box {box_id}"]))
            return

        if not visible:
            rr.log("world/boxes/target", rr.Clear(recursive=True))
            rr.log("world/boxes/carried", rr.Clear(recursive=True))
            return
        if not attached or joint_positions is None:
            rr.log("world/boxes/carried", rr.Clear(recursive=True))
            rr.log("world/boxes/target", rr.Boxes3D(
                centers=[self.box_center.tolist()], half_sizes=[(self.box_size * 0.5).tolist()],
                colors=[[45, 220, 75, 180]], labels=["target box"]))
            return
        if transforms is None:
            transforms = self.robot.fk(joint_positions)
        tool_transform = transforms.get(self.tool_link)
        if tool_transform is None:
            return
        box_transform = tool_transform.copy()
        box_transform[:3, :3] = tool_transform[:3, :3] @ self.tool_to_box_rotation
        box_transform[:3, 3] = tool_transform[:3, 3] + tool_transform[:3, :3] @ self.tool_to_box_center
        rr.log("world/boxes/target", rr.Clear(recursive=True))
        rr.log("world/boxes/carried", rr.Boxes3D(
            centers=[box_transform[:3, 3].tolist()], half_sizes=[(self.box_size * 0.5).tolist()],
            quaternions=[matrix_to_quaternion(box_transform[:3, :3])],
            colors=[[45, 225, 100, 190]], labels=["carried target box"]))

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
        if self.wall_request:
            body = body.replace("在RViz拖动绿色箱体XYZ；右键箱体确认后才开始计算。",
                                "通过 plan_wall_box 服务选择 x、box_id 和 arm。")
            body += (f"\n- 请求：x={self.wall_request['x']:.3f}m，box_id={self.wall_request['box_id']}"
                     f"，arm={self.wall_request.get('side', '?')}"
                     f"\n- 车头基准 X={self.wall_request['chassis_front_x']:.6f}m"
                     "\n- 仅仿真：未找到路径不等于绝对不可抓取；失败回放仅供诊断。")
        if self.wall_request.get("stream_segment") and "segment_index" in self.wall_request:
            body += (f"\n- 已收到第 {self.wall_request['segment_index'] + 1} 箱完整分段；"
                     "后续规划不等待播放，箱体按轨迹帧释放消失。")
        elif self.wall_request.get("sequence"):
            body += (f"\n- 整墙结果：{self.wall_request['completed_count']}/25；"
                     f"失败箱 {self.wall_request['failed_box_id']}；逐帧释放消失，不跳箱。")
        environment = self.wall_request.get("environment", {})
        if environment:
            body += (f"\n- 环境碰撞：{'开启' if environment['enabled'] else '关闭（仅回归）'}；"
                     f"{len(environment['boxes'])}个障碍；{environment.get('description', '')}")
        alignment = self.wall_request.get("height_alignment", {})
        if alignment.get("strategy") == "comfort_radius":
            body += (f"\n- 单高度距离策略：{alignment['arm']}臂，xy={alignment['xy']:.3f} m，"
                     f"臂长={alignment['arm_length']:.3f} m"
                     f"\n- 比例 {alignment['actual_ratio']:.3f}；区间 "
                     f"[{alignment['ratio_min']:.3f}, {alignment['ratio_max']:.3f}]；"
                     f"分支 {alignment['branch']}；目标updown={alignment['target_updown']:.3f} m"
                     f"\n- 区间外原因：{alignment['outside_reason'] or '无'}；不换高重试"
                     + ("\n- 升降 → 抓取抽出 → 后放 → 释放消失 → 保持末姿态接续下一箱（仅仿真）"
                        if self.wall_request.get("release_after_transfer") else
                        "\n- 升降 → 抓取抽出 → 携箱返回home → 携箱升降归零（仅仿真）"))
        elif alignment.get("strategy") == "top_wrist_alignment":
            body += (f"\n- 顶吸按腕心选高：水平距离 {alignment['xy']:.3f} m / 臂长 {alignment['arm_length']:.3f} m；"
                     f"updown={alignment['target_updown']:.3f} m；{alignment['outside_reason'] or '水平可达'}"
                     f"；肩高于腕心 {alignment.get('shoulder_above_wrist', 0.):.3f} m（独立于正吸比例）"
                     "\n- 后放 → 释放消失，保持末姿态；失败仅回放实际连续前缀。")
        elif alignment:
            body += (f"\n- 高度调整：{'开启' if alignment['enabled'] else '关闭'}；"
                     f"肩部中心比箱中心高 {alignment['shoulder_box_offset']:.3f} m"
                     f"\n- 计划下降 {alignment['descent']:.3f} m；"
                     f"updown 目标 {alignment['target_updown']:.3f} m（不是实机反馈）"
                     "\n- lower_to_box_height → 预接触 → 接触 → 附着 → 抽出 → 后放释放消失（箱墙搬运）")
        rr.log(
            "summary",
            rr.TextDocument(body, media_type=rr.MediaType.MARKDOWN),
        )

    def write_trajectory(self) -> None:
        # Bounded batches reduce SDK overhead, never decimate the planner's frames.
        for begin in range(0, len(self.frames), 1024):
            batch = self.frames[begin:begin + 1024]
            transforms = [self.robot.fk(dict(zip(self.joint_names, f.joints))) for f in batch]
            times = [rr.TimeColumn("task_frame", sequence=np.arange(
                self.global_frame, self.global_frame + len(batch)))]
            for link in transforms[0]:
                poses = np.asarray([tf[link] for tf in transforms])
                rr.send_columns(f"world/robot/{link}", indexes=times,
                    columns=rr.Transform3D.columns(translation=poses[:, :3, 3],
                        quaternion=[matrix_to_quaternion(p[:3, :3]) for p in poses]))
            for tf in transforms:
                self.log_frame(tf)
        self.log_summary("诊断轨迹已写入：末帧为失败点（非可执行轨迹）" if self.diagnostic else
                         ("当前分段已写入；后续轨迹自动追加，使用时间轴播放、暂停和拖动"
                          if self.stream_segment else "完整轨迹已写入：使用时间轴播放、暂停和拖动"), planning=False)
        recording = rr.get_global_data_recording()
        if recording is not None:
            recording.flush()

    def log_frame(self, transforms: dict[str, np.ndarray]) -> None:
        frame = self.frames[self.frame_index]
        rr.set_time("task_frame", sequence=self.global_frame)
        joint_positions = dict(zip(self.joint_names, frame.joints))
        if self.scenes and self.scene_index != frame.scene_index:
            self.update_scene(self.scenes[frame.scene_index], draw_boxes=False)
            self.scene_index = frame.scene_index
        self.log_boxes(joint_positions, attached=frame.box_attached, visible=frame.box_visible,
                       transforms=transforms, carried_boxes=frame.carried_boxes)
        rr.log(
            "world/current_stage",
            rr.TextLog(
                f"generation={self.generation} frame={self.frame_index + 1}/{len(self.frames)} "
                f"stage={frame.stage} attached={frame.box_attached} "
                f"visible={frame.box_visible} scene={frame.scene_index}"
            ),
        )

        if self.frame_index + 1 == len(self.frames):
            log_failure(getattr(self, "diagnostic", {}))
        self.last_frame = frame
        self.frame_index += 1
        self.global_frame += 1


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
