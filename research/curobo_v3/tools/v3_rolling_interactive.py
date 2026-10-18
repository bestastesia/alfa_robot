#!/usr/bin/env python3
"""Interactive Viser editor/replay for MOTION-292 simulation; never sends hardware commands."""
import argparse
import bisect
import json
from pathlib import Path
import queue
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np
import viser
import yourdfpy
from viser.extras import ViserUrdf

from curobo_core.adapter import matrix_pose, pose_matrix
from curobo_core.contracts import PlannerAssets
from curobo_core.scene import SceneSnapshot
from curobo_core.trajectory import TimedTrajectory


def command_for(args, controls, output):
    command = [sys.executable, str(args.runner),
               "--initial-cycle", str(args.initial_cycle), "--round", str(args.round),
               "--stage", controls["stage"], "--scenario", controls["scenario"],
               "--obstacle-center", *map(str, controls["center"]),
               "--obstacle-size", *map(str, controls["size"]),
               "--obstacle-velocity", *map(str, controls["velocity"]),
               "--compute-bound-s", str(controls["t2"]), "--link-bound-s", str(controls["t3"]),
               "--link-delay-s", str(controls["link_delay"]), "--slices", str(controls["slices"]),
               "--braking-duration-s", str(controls["braking"]), "--seed", str(controls["seed"]),
               "--execution-timeout-s", str(args.execution_timeout_s), "--output", str(output)]
    return command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--initial-cycle", type=Path, required=True)
    parser.add_argument("--initial-result", type=Path, required=True)
    parser.add_argument("--runner", type=Path, default=Path(__file__).with_name("v3_rolling_replan.py"),
                        help="prepared isolated-runtime v3_rolling_replan.py")
    parser.add_argument("--round", type=int, default=0)
    parser.add_argument("--urdf", type=Path, default=PlannerAssets().urdf)
    parser.add_argument("--port", type=int, default=8104)
    parser.add_argument("--execution-timeout-s", type=float, default=120.)
    args = parser.parse_args()
    report = json.loads(args.initial_result.read_text())
    cycle = json.loads(args.initial_cycle.read_text())
    if cycle.get("sequence"):
        cycle = cycle["rounds"][args.round]
    cycle_result = cycle["result"] if "result" in cycle else cycle["results"][-1]
    stage_durations = {item["name"]: item["time_from_start_s"][-1] for item in cycle_result["timed_segments"]}
    if report.get("source") != "wall_clock_simulation":
        parser.error("--initial-result must be a rolling simulation JSON")

    mesh_dir = args.urdf.parent / "meshes"
    urdf = yourdfpy.URDF.load(args.urdf, load_meshes=True, build_scene_graph=True,
                              filename_handler=lambda fname: str(mesh_dir / Path(fname).name))
    server = viser.ViserServer(host="127.0.0.1", port=args.port, label="MOTION-292 动态避障")
    robot = ViserUrdf(server, urdf, root_node_name="/robot")
    robot_frame = server.scene.add_frame("/robot", show_axes=False)
    original_path = server.scene.add_line_segments("/path/original", points=np.zeros((1, 2, 3)),
                                                   colors=(160, 160, 160), line_width=3.)
    active_path = server.scene.add_line_segments("/path/updated", points=np.zeros((1, 2, 3)),
                                                 colors=(35, 220, 130), line_width=5.)
    obstacle = [None]
    scene_nodes, payload_nodes = [], []
    state = {}
    preview = [False]
    inbox = queue.Queue()
    busy = threading.Event()

    server.gui.add_markdown("## MOTION-292 · 动态避障\n**灰：原轨迹　绿：更新后轨迹　红：障碍物**\n\n仅仿真，不发送实机指令。")
    with server.gui.add_folder("添加障碍", expand_by_default=True):
        stage = server.gui.add_dropdown("路径段", options=("approach", "contact_approach", "extract", "transport", "conveyor_place", "return_home"), initial_value=report["stage"])
        scenario = server.gui.add_dropdown("障碍模式", options=("static", "moving", "appear", "depart"), initial_value=report["scenario"] if report["scenario"] != "clear" else "static")
        center = server.gui.add_vector3("中心 XYZ / m", initial_value=report.get("obstacle_center_override_map_m") or (-1.0486, .6355, 1.2774), step=.005)
        size = server.gui.add_vector3("尺寸 XYZ / m", initial_value=report.get("obstacle_size_m", (.08, .08, .08)), min=(.005, .005, .005), step=.005)
        velocity = server.gui.add_vector3("速度 XYZ / m/s", initial_value=report.get("obstacle_velocity_override_map_m_s") or (.025, 0., 0.), step=.005)
        preview_button = server.gui.add_button("添加/预览障碍", color="red")
    with server.gui.add_folder("实时编排", expand_by_default=True):
        t2 = server.gui.add_number("t2 计算上界 / s", initial_value=report["schedule"]["compute_bound_s"], min=.01, step=.01)
        t3 = server.gui.add_number("t3 通信上界 / s", initial_value=report["schedule"]["link_bound_s"], min=0., step=.01)
        link_delay = server.gui.add_number("模拟通信延迟 / s", initial_value=.02, min=0., step=.01)
        slices = server.gui.add_number("n 小段数", initial_value=report["schedule"]["slices"], min=1, step=1)
        braking = server.gui.add_number("内部减速时长 / s", initial_value=report.get("braking_duration_s", .4), min=.05, step=.05)
        seed = server.gui.add_number("随机种子", initial_value=report.get("seed", 20261003), min=0, step=1)
        replan = server.gui.add_button("重新规划", color="green")
    play = server.gui.add_checkbox("播放", initial_value=False)
    speed = server.gui.add_slider("倍率", min=.1, max=3., step=.1, initial_value=1.)
    progress = server.gui.add_slider("进度 / %", min=0., max=100., step=.1, initial_value=65.)
    status = server.gui.add_markdown("")

    def replace_obstacle(dimensions):
        if obstacle[0] is not None:
            obstacle[0].remove()
        obstacle[0] = server.scene.add_box("/dynamic/rolling_obstacle", dimensions=tuple(dimensions),
                                           color=(235, 65, 65), opacity=.85, visible=False)

    def trajectory_lines(curves, base_transform):
        lines = []
        for curve in curves:
            points = []
            for row in curve.sample(np.linspace(0., curve.duration, 50))["positions"]:
                urdf.update_cfg(dict(zip(curve.joint_names, row)))
                points.append((base_transform @ urdf.get_transform("left_tool0", urdf.base_link))[:3, 3].copy())
            points = np.asarray(points)
            lines.append(np.stack((points[:-1], points[1:]), axis=1))
        return lines

    def install(new_report):
        for node in scene_nodes + [node for _, node in payload_nodes]:
            node.remove()
        scene_nodes.clear(); payload_nodes.clear()
        snapshot = SceneSnapshot.from_dict(new_report["initial_snapshot"])
        base_transform = pose_matrix(snapshot.state.base_pose)
        robot_frame.position = snapshot.state.base_pose.position
        robot_frame.wxyz = snapshot.state.base_pose.quaternion_wxyz
        for obj in snapshot.objects:
            if obj.object_id != "ground":
                scene_nodes.append(server.scene.add_box(f"/world/{obj.object_id}", dimensions=obj.dimensions_m,
                    position=obj.pose.position, wxyz=obj.pose.quaternion_wxyz, color=(110, 150, 170),
                    opacity=.25 if obj.object_id.startswith("wall_box_") else .06))
        for obj in snapshot.meshes:
            scene_nodes.append(server.scene.add_mesh_simple(f"/meshes/{obj.object_id}", vertices=np.asarray(obj.vertices_m),
                faces=np.asarray(obj.faces), position=obj.pose.position, wxyz=obj.pose.quaternion_wxyz,
                color=(140, 150, 160), opacity=.3))
        for item in snapshot.attachments:
            payload_nodes.append((item, server.scene.add_box(f"/payload/{item.object_id}",
                dimensions=item.dimensions_m, color=(235, 180, 100), opacity=.7)))
        entries = [{"active_at": new_report["started_at"], "trajectory": new_report["initial_trajectory"]}, *new_report["trajectories"]]
        curves = [TimedTrajectory(item["trajectory"]) for item in entries]
        started = new_report["started_at"]
        ended = new_report.get("completed_at", max((event["at"] for event in new_report["events"]), default=started))
        state.update(report=new_report, snapshot=snapshot, base_transform=base_transform, curves=curves,
                     epochs=[item["active_at"] for item in entries], paths=trajectory_lines(curves, base_transform),
                     started=started, duration=max(.001, ended-started), observations=new_report["observations"],
                     observation_times=[item["received_at"] for item in new_report["observations"]])
        original_path.points = state["paths"][0]
        replace_obstacle(new_report.get("obstacle_size_m", size.value))
        preview[0] = False
        progress.value = 65.

    def controls():
        values = dict(stage=stage.value, scenario=scenario.value, center=np.asarray(center.value, float),
                      size=np.asarray(size.value, float), velocity=np.asarray(velocity.value, float),
                      t2=float(t2.value), t3=float(t3.value), link_delay=float(link_delay.value),
                      slices=int(slices.value), braking=float(braking.value), seed=int(seed.value))
        if not all(np.isfinite(values[key]).all() for key in ("center", "size", "velocity")) or np.any(values["size"] <= 0):
            raise ValueError("位置/尺寸/速度必须是有限数，尺寸必须大于0")
        if values["t2"] <= 0 or values["t3"] < 0 or not 0 <= values["link_delay"] <= values["t3"] or values["slices"] < 1:
            raise ValueError("要求 t2>0、t3≥通信延迟≥0、n≥1")
        if values["slices"] * (values["t2"] + values["t3"]) > stage_durations[values["stage"]] + 1e-9:
            raise ValueError("n 必须满足 n ≤ t1/(t2+t3)")
        return values

    @preview_button.on_click
    def _(_event):
        try:
            values = controls()
            replace_obstacle(values["size"])
            obstacle[0].position = values["center"]
            obstacle[0].visible = True
            preview[0] = True
            play.value = False
            status.content = "**障碍已添加。** 点击“重新规划”计算绿色新轨迹。"
        except ValueError as error:
            status.content = f"**输入错误：** {error}"

    @replan.on_click
    def _(_event):
        if busy.is_set():
            return
        try:
            values = controls()
        except ValueError as error:
            status.content = f"**输入错误：** {error}"
            return
        busy.set(); play.value = False
        status.content = "**正在重新规划…** 机器人模型仍显示上一条安全轨迹。"
        def work():
            output = Path(tempfile.mkdtemp(prefix="motion292-viser-")) / "result.json"
            command = command_for(args, values, output)
            try:
                completed = subprocess.run(command, text=True, capture_output=True,
                                           timeout=args.execution_timeout_s + 240.)
                result = json.loads(output.read_text()) if output.exists() else None
                inbox.put((result, completed.stdout + completed.stderr))
            except Exception as error:
                inbox.put((None, f"{type(error).__name__}: {error}"))
        threading.Thread(target=work, name="motion292-interactive-replan", daemon=True).start()

    install(report)
    previous = time.perf_counter()
    while True:
        try:
            new_report, log = inbox.get_nowait()
            busy.clear()
            if new_report and new_report.get("success"):
                install(new_report)
                status.content = (f"**更新完成。** 接续 {new_report['splice_count']} 次；"
                                  f"直接优化 {new_report.get('direct_splice_count', 0)} 次；"
                                  f"内部停稳后重规划 {new_report.get('braking_replan_count', 0)} 次。")
            else:
                failure = (new_report or {}).get("failure_code", "PROCESS_FAILED")
                action = (new_report or {}).get("required_action", "保持上一条安全轨迹")
                status.content = f"**规划拒绝：{failure}** 需要：{action}.\n\n`{log[-500:]}`"
        except queue.Empty:
            pass
        now = time.perf_counter()
        if play.value:
            progress.value = min(100., progress.value + (now-previous) * speed.value / state["duration"] * 100.)
            if progress.value >= 100.:
                play.value = False
        previous = now
        absolute = state["started"] + state["duration"] * progress.value / 100.
        index = max(0, bisect.bisect_right(state["epochs"], absolute)-1)
        curve = state["curves"][index]
        sample = curve.sample(np.clip(absolute-state["epochs"][index], 0., curve.duration))
        cfg = dict(zip(curve.joint_names, sample["positions"]))
        urdf.update_cfg(cfg); robot.update_cfg(cfg)
        active_path.points = state["paths"][index]
        for attachment, node in payload_nodes:
            pose = matrix_pose(state["base_transform"] @ urdf.get_transform(attachment.parent_link, urdf.base_link) @ pose_matrix(attachment.tool_to_object))
            node.position, node.wxyz = pose.position, pose.quaternion_wxyz
        if not preview[0]:
            obs_index = bisect.bisect_right(state["observation_times"], absolute)-1
            obj = None if obs_index < 0 else next((item for item in state["observations"][obs_index]["snapshot"]["objects"]
                                                   if item["object_id"] == "rolling_obstacle"), None)
            obstacle[0].visible = obj is not None
            if obj is not None:
                obs = state["observations"][obs_index]
                velocity_map = dict(obs["velocities"])
                obstacle[0].position = np.asarray(obj["pose"]["position"]) + np.asarray(velocity_map["rolling_obstacle"]) * (absolute-obs["observed_at"])
        if not busy.is_set() and not status.content:
            status.content = f"阶段 **{state['report']['stage']}** · 接续 **{index}** 次"
        time.sleep(.03)


if __name__ == "__main__":
    main()
