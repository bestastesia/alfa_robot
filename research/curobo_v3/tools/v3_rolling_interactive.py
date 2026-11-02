#!/usr/bin/env python3
"""Fresh, live Viser diagnosis for one complete MOTION-292 transport round; never hardware."""
import argparse
import hashlib
from collections import deque
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
from curobo_core.contracts import ACTIVE_JOINTS, PlannerAssets
from curobo_core.scene import SceneSnapshot
from curobo_core.trajectory import TimedTrajectory
from v3_rolling_replan import PROGRESS_PREFIX, STARTUP_EXTRA_S
from v3_timed_wall_report import PHASE_LABELS, phase_planning_detail, planning_ledger


STAGES = ("approach", "contact_approach", "extract", "transport", "conveyor_place", "return_home")
# Proven per-stage budgets from the frozen acceptance matrix. One global budget cannot satisfy
# both the 1.5 s contact phase and the long free-space phases.
STAGE_BUDGETS = {
    "approach": (1., .15, .05),
    "contact_approach": (.1, .15, .05),
    "extract": (.35, .15, .05),
    "transport": (.75, .15, .05),
    "conveyor_place": (.9, .15, .05),
    "return_home": (1., .15, .05),
}
ROLL_COLORS = (
    ((30, 225, 80), (30, 155, 245)),
    ((105, 235, 45), (120, 100, 245)),
    ((230, 205, 40), (220, 90, 210)),
    ((245, 145, 35), (35, 210, 215)),
)


def route_colors(kind, sequence=0):
    if kind == "upstream":
        return ((95, 95, 95), (165, 165, 165))
    colors = ROLL_COLORS[sequence % len(ROLL_COLORS)]
    if kind == "installed":
        return colors
    if kind == "candidate":
        return tuple(tuple((value + 255) // 2 for value in color) for color in colors)
    raise ValueError(f"unknown route layer: {kind}")


def command_for(args, cycle, controls, stage, output):
    t2, t3, t4 = STAGE_BUDGETS[stage]
    scenario = controls["scenario"] if stage == controls["obstacle_stage"] else "clear"
    command = [sys.executable, str(args.runner), "--initial-cycle", str(cycle),
               "--round", "0", "--stage", stage, "--scenario", scenario,
               "--compute-bound-s", str(t2), "--first-compute-bound-s",
               str(t2 + STARTUP_EXTRA_S),
               "--link-bound-s", str(t3),
               "--admission-bound-s", str(t4),
               "--link-delay-s", str(min(controls["link_delay"], t3)),
               "--braking-duration-s", str(controls["braking"]),
               "--seed", str(controls["seed"]), "--execution-timeout-s",
               str(args.execution_timeout_s), "--output", str(output), "--stream-progress"]
    if scenario != "clear":
        command += ["--obstacle-center", *map(str, controls["center"]),
                    "--obstacle-size", *map(str, controls["size"]),
                    "--obstacle-velocity", *map(str, controls["velocity"])]
    return command


def round_command_for(args, cycle, controls, output_dir):
    return [sys.executable, str(args.round_runner), "--initial-cycle", str(cycle),
            "--obstacle-stage", controls["obstacle_stage"], "--scenario", controls["scenario"],
            "--obstacle-center", *map(str, controls["center"]),
            "--obstacle-size", *map(str, controls["size"]),
            "--obstacle-velocity", *map(str, controls["velocity"]),
            "--link-delay-s", str(controls["link_delay"]),
            "--admission-bound-s", "0.1",
            "--braking-duration-s", str(controls["braking"]),
            "--execution-timeout-s", str(args.execution_timeout_s),
            "--seed", str(controls["seed"]), "--output-dir", str(output_dir),
            "--stream-progress"]


def stage_planning_rows(result, stage):
    detail = phase_planning_detail(result, stage)
    return {
        "core_ms": sum(value for _, value in detail.get("additive", [])),
        "core_rows": detail.get("additive", []),
        "diagnostics": detail.get("diagnostics", []),
        "notes": detail.get("notes", []),
        "timing_ms": sum(value for _, value in detail.get("timing", [])),
        "timing_rows": detail.get("timing", []),
        "timing_diagnostics": detail.get("timing_diagnostics", []),
    }


def metrics_markdown(ledger, baseline):
    rows = ["## 实时耗时账", "所有时间均来自本次点击后新启动的规划；标注“嵌套”的数字不能重复相加。"]
    if baseline:
        rows += [
            f"### 主流程：从点击到六阶段初始可执行轨迹 **{baseline['wall_s']:.3f} s**",
            "以下六项可直接相加：",
            f"- 进程启动、模块导入与CUDA上下文：**{baseline['startup_ms']:.1f} ms**",
            f"- 模型、GPU、碰撞资源初始化/预热：**{baseline['initialization_ms']:.1f} ms**",
            f"- Mentor核心路径规划：**{baseline['planner_ms']:.1f} ms**",
            f"- 六阶段固定路径时间参数化：**{baseline['timing_ms']:.1f} ms**",
            f"- RNEA审计与固定q重定时：**{baseline.get('dynamics_ms', 0.):.1f} ms**",
            f"- 控制器预览导出：**{baseline.get('controller_ms', 0.):.1f} ms**",
            f"- 阶段快照生成、JSON写盘与进程通信：**{baseline['residual_ms']:.1f} ms**",
            "#### Mentor核心规划内部总账（可加和）",
        ]
        rows.extend(f"- {name}：{value:.1f} ms" for name, value in baseline["core_rows"])
        rows.append("#### 轨迹时间化内部总账（可加和）")
        rows.extend(f"- {name}：{value:.1f} ms" for name, value in baseline["timing_rows"])
        if baseline.get("rolling_service_ms") is not None:
            rows += ["#### 滚动服务一次性准备（六阶段共享，不重复执行）",
                     f"- 常驻CUDA worker与模型初始化：{baseline['rolling_service_ms']:.1f} ms",
                     f"- 空载/带载TrajOpt与碰撞资源预热：{baseline['rolling_warmup_ms']:.1f} ms"]
    for stage in STAGES:
        item = ledger[stage]
        rows.append(f"### {PHASE_LABELS[stage]}")
        if item["core_ms"] is None:
            rows.append("- 初始轨迹：等待本轮从头规划")
        else:
            rows.append(f"#### A. 核心规划归属账：**{item['core_ms']:.1f} ms**")
            rows.append("这是整轮核心规划中归属于该阶段的加和项，**不等于纯RRT，也不一定是独立阶段墙钟**：")
            rows.extend(f"- {name}：{value:.1f} ms" for name, value in item["core_rows"])
            for note in item["notes"]:
                rows.append(f"- 说明：{note}")
            if item["diagnostics"]:
                rows.append("#### B. 核心规划嵌套诊断（已包含在上级管线中，不可再相加）")
                rows.extend(f"- {name}：{value:.1f} ms" for name, value in item["diagnostics"])
            rows.append(f"#### C. 该阶段轨迹时间化：**{item['timing_ms']:.1f} ms**")
            if item["timing_rows"]:
                rows.extend(f"- {name}：{value:.1f} ms" for name, value in item["timing_rows"])
                rows.extend(f"- 诊断：{text}" for text in item["timing_diagnostics"])
            else:
                rows.append("- 当前结果没有阶段级时间化明细")
        if item["solves"]:
            rows.append("#### D. 在线滚动优化（每次为独立墙钟）")
        for index, solve in enumerate(item["solves"], 1):
            duration = solve.get("compute_wall_s")
            value = "未返回" if duration is None else f"{duration * 1000:.1f} ms"
            strategy = solve.get("strategy") or ("终点安全复核" if solve.get("terminal") else "未知")
            rows.append(f"- 滚动 #{index}：**{value}** · {strategy}")
        motion = item["motion_s"]
        rows.append(f"- 仿真实际运动：**{motion:.3f} s**" + ("（进行中）" if item["running"] else ""))
        if item["failure"]:
            rows.append(f"- 状态：**拒绝 {item['failure']}**")
    return "\n".join(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preparer", type=Path, default=Path(__file__).with_name("v3_rolling_prepare.py"))
    parser.add_argument("--runner", type=Path, default=Path(__file__).with_name("v3_rolling_replan.py"),
                        help="legacy single-stage runner; retained for compatibility")
    parser.add_argument("--round-runner", type=Path, default=Path(__file__).with_name("v3_rolling_round.py"),
                        help="persistent-worker six-stage runner")
    parser.add_argument("--round", type=int, choices=range(15), default=0)
    parser.add_argument("--urdf", type=Path, default=PlannerAssets().urdf)
    parser.add_argument("--port", type=int, default=8104)
    parser.add_argument("--execution-timeout-s", type=float, default=120.)
    # Kept only so old launch commands do not fail; precomputed routes are deliberately never loaded.
    parser.add_argument("--initial-cycle", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--initial-result", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()

    mesh_dir = args.urdf.parent / "meshes"
    urdf = yourdfpy.URDF.load(args.urdf, load_meshes=True, build_scene_graph=True,
                              filename_handler=lambda fname: str(mesh_dir / Path(fname).name))
    server = viser.ViserServer(host="127.0.0.1", port=args.port, label="MOTION-292 实时滚动优化")

    @server.on_client_connect
    def _focus_path(client):
        client.camera.position = (2.2, -1.7, 2.35)
        client.camera.look_at = (.78, .68, 1.4)
        client.camera.up_direction = (0., 0., 1.)

    robot = ViserUrdf(server, urdf, root_node_name="/robot")
    robot_frame = server.scene.add_frame("/robot", show_axes=False)
    pre_shadow = ViserUrdf(server, urdf, root_node_name="/shadow/pre_rnea")
    post_shadow = ViserUrdf(server, urdf, root_node_name="/shadow/post_rnea")
    pre_shadow_frame = server.scene.add_frame("/shadow/pre_rnea", show_axes=False, visible=False)
    post_shadow_frame = server.scene.add_frame("/shadow/post_rnea", show_axes=False, visible=False)
    obstacle = [None]
    scene_nodes, payload_nodes, path_nodes = [], [], []
    inbox = queue.Queue()
    busy = threading.Event()
    current = {"snapshot": None, "base_transform": np.eye(4), "stage": None,
               "pre_rnea": None, "post_rnea": None, "dynamics_detail": None}
    baseline = [None]
    ledger = {stage: {"core_ms": None, "core_rows": [], "diagnostics": [], "notes": [],
                      "timing_ms": 0., "timing_rows": [], "timing_diagnostics": [],
                      "solves": [], "motion_s": 0.,
                      "running": False, "failure": None} for stage in STAGES}

    server.gui.add_markdown(
        "## MOTION-292 · 一轮完整搬运实时诊断\n"
        "**灰：上游认证轨迹；滚动颜色按接续次数循环；细浅线：RNEA前候选；同色粗线：RNEA后实际安装；半透明红盒：障碍。**\n\n"
        "界面启动时不加载任何预计算路线。点击后先从头规划本轮，再按六阶段实时追加滚动结果。仅仿真。")
    run_button = server.gui.add_button("▶ 从头规划并实时演示完整一轮", color="green")
    status = server.gui.add_markdown("**就绪。** 当前没有显示任何轨迹。")
    timing = server.gui.add_markdown(metrics_markdown(ledger, None))
    overall = server.gui.add_slider("本轮进度 / %", min=0., max=100., step=.1, initial_value=0.)
    overall.disabled = True
    with server.gui.add_folder("轨迹与动力学图层", expand_by_default=True):
        show_upstream = server.gui.add_checkbox("上游认证轨迹", initial_value=True)
        show_candidate = server.gui.add_checkbox("避障候选（RNEA前）", initial_value=True)
        show_installed = server.gui.add_checkbox("实际安装（RNEA后）", initial_value=True)
        show_shadows = server.gui.add_checkbox("RNEA前后双影子", initial_value=True)
        dynamics_status = server.gui.add_markdown("尚无在线动力学候选。")
        curve_joint = server.gui.add_dropdown(
            "曲线关节", options=tuple(ACTIVE_JOINTS), initial_value="left_joint1")
        state_curve = server.gui.add_uplot(
            data=(np.array([0., 1.]), np.zeros(2), np.zeros(2)),
            series=(viser.uplot.Series(label="time"),
                    viser.uplot.Series(label="q", stroke="#3daad2"),
                    viser.uplot.Series(label="qdot", stroke="#ef923e")),
            mode=1, scales={"x": {"time": False}}, title="安装轨迹 q / qdot", height=220,
            visible=False)
        derivative_curve = server.gui.add_uplot(
            data=(np.array([0., 1.]), np.zeros(2), np.zeros(2)),
            series=(viser.uplot.Series(label="time"),
                    viser.uplot.Series(label="qddot", stroke="#a855f7"),
                    viser.uplot.Series(label="jerk", stroke="#22c55e")),
            mode=1, scales={"x": {"time": False}}, title="安装轨迹 qddot / jerk", height=220,
            visible=False)
        torque_curve = server.gui.add_uplot(
            data=(np.array([0., 1.]), np.zeros(2), np.zeros(2)),
            series=(viser.uplot.Series(label="time"),
                    viser.uplot.Series(label="RNEA torque", stroke="#ef4444"),
                    viser.uplot.Series(label="gravity", stroke="#64748b")),
            mode=1, scales={"x": {"time": False}}, title="安装轨迹 torque / gravity", height=220,
            visible=False)

    with server.gui.add_folder("本轮与障碍参数", expand_by_default=False):
        round_number = server.gui.add_number("轮次（1~15）", initial_value=args.round + 1, min=1, max=15, step=1)
        obstacle_stage = server.gui.add_dropdown("加障碍阶段", options=STAGES, initial_value="transport")
        scenario = server.gui.add_dropdown("障碍模式", options=("static", "moving", "appear", "depart"), initial_value="static")
        center = server.gui.add_vector3("中心 XYZ / m", initial_value=(.7253190279006958, .6944118738174438, 1.164883017539978), step=.005)
        size = server.gui.add_vector3("尺寸 XYZ / m", initial_value=(.08, .08, .08), min=(.005, .005, .005), step=.005)
        velocity = server.gui.add_vector3("速度 XYZ / m/s", initial_value=(0., 0., 0.), step=.005)
        link_delay = server.gui.add_number("模拟通信延迟 / s", initial_value=.02, min=0., max=.1, step=.01)
        braking = server.gui.add_number("内部减速时长 / s", initial_value=.5, min=.05, step=.05)
        seed = server.gui.add_number("随机种子", initial_value=20261003, min=0, step=1)

    def controls():
        values = {"round": int(round_number.value) - 1, "obstacle_stage": obstacle_stage.value,
                  "scenario": scenario.value, "center": np.asarray(center.value, float),
                  "size": np.asarray(size.value, float), "velocity": np.asarray(velocity.value, float),
                  "link_delay": float(link_delay.value), "braking": float(braking.value), "seed": int(seed.value)}
        if not 0 <= values["round"] < 15:
            raise ValueError("轮次必须在1~15")
        if not all(np.isfinite(values[key]).all() for key in ("center", "size", "velocity")) or np.any(values["size"] <= 0):
            raise ValueError("位置/尺寸/速度必须是有限数，尺寸必须大于0")
        if not 0 <= values["link_delay"] <= .1 or values["braking"] <= 0 or not 0 <= values["seed"] < 2**32:
            raise ValueError("通信延迟需在0~0.1s，减速时长需大于0，种子需为32位无符号整数")
        return values

    def replace_obstacle(dimensions):
        if obstacle[0] is not None:
            obstacle[0].remove()
        obstacle[0] = server.scene.add_box("/dynamic/rolling_obstacle", dimensions=tuple(dimensions),
                                           color=(235, 65, 65), opacity=.85, visible=False)

    def trajectory_lines(trajectory, base_transform):
        curve = TimedTrajectory(trajectory)
        points = {"left": [], "right": []}
        for row in curve.sample(np.linspace(0., curve.duration, 80))["positions"]:
            urdf.update_cfg(dict(zip(curve.joint_names, row)))
            for side in points:
                points[side].append((base_transform @ urdf.get_transform(f"{side}_tool0", urdf.base_link))[:3, 3].copy())
        return {side: np.stack((np.asarray(rows)[:-1], np.asarray(rows)[1:]), axis=1)
                for side, rows in points.items()}

    def add_route(stage, name, trajectory, kind="installed", sequence=0):
        lines = trajectory_lines(trajectory, current["base_transform"])
        visible = {"upstream": show_upstream.value, "candidate": show_candidate.value,
                   "installed": show_installed.value}[kind]
        for side, color in zip(("left", "right"), route_colors(kind, sequence)):
            node = server.scene.add_line_segments(
                f"/paths/{stage}/{name}/{side}", points=lines[side], colors=color,
                line_width=3. if kind == "upstream" else (4. if kind == "candidate" else 7.))
            node.visible = visible
            path_nodes.append((kind, node))

    def refresh_dynamics_curves():
        detail = current.get("dynamics_detail")
        if not detail or not detail.get("post_rnea_trajectory"):
            state_curve.visible = derivative_curve.visible = torque_curve.visible = False
            return
        trajectory = detail["post_rnea_trajectory"]
        names = trajectory["joint_names"]
        if curve_joint.value not in names:
            return
        index = names.index(curve_joint.value)
        times = np.asarray(trajectory["time_from_start_s"])
        state_curve.data = (times, np.asarray(trajectory["positions"])[:, index],
                            np.asarray(trajectory["velocities"])[:, index])
        derivative_curve.data = (times, np.asarray(trajectory["accelerations"])[:, index],
                                 np.asarray(trajectory["jerks"])[:, index])
        torque_curve.data = (times, np.asarray(detail["torques_nm"])[:, index],
                             np.asarray(detail["gravity_torques_nm"])[:, index])
        state_curve.visible = derivative_curve.visible = torque_curve.visible = True

    curve_joint.on_update(lambda _event: refresh_dynamics_curves())

    def refresh_layer_visibility():
        visible = {"upstream": show_upstream.value, "candidate": show_candidate.value,
                   "installed": show_installed.value}
        for kind, node in path_nodes:
            node.visible = visible[kind]
        active = bool(show_shadows.value and current.get("pre_rnea") and current.get("post_rnea"))
        pre_shadow_frame.visible = active
        post_shadow_frame.visible = active

    for control in (show_upstream, show_candidate, show_installed, show_shadows):
        control.on_update(lambda _event: refresh_layer_visibility())

    def install_snapshot(data):
        for node in scene_nodes + [node for _, node in payload_nodes]:
            node.remove()
        scene_nodes.clear(); payload_nodes.clear()
        snapshot = SceneSnapshot.from_dict(data)
        current["snapshot"] = snapshot
        current["base_transform"] = pose_matrix(snapshot.state.base_pose)
        robot_frame.position = snapshot.state.base_pose.position
        robot_frame.wxyz = snapshot.state.base_pose.quaternion_wxyz
        for frame in (pre_shadow_frame, post_shadow_frame):
            frame.position = snapshot.state.base_pose.position
            frame.wxyz = snapshot.state.base_pose.quaternion_wxyz
        for obj in snapshot.objects:
            if obj.object_id not in ("ground", "rolling_obstacle"):
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

    def update_robot(joint_names, positions):
        cfg = dict(zip(joint_names, positions))
        urdf.update_cfg(cfg); robot.update_cfg(cfg)
        for attachment, node in payload_nodes:
            pose = matrix_pose(current["base_transform"] @ urdf.get_transform(
                attachment.parent_link, urdf.base_link) @ pose_matrix(attachment.tool_to_object))
            node.position, node.wxyz = pose.position, pose.quaternion_wxyz

    def reset_view():
        for _, node in path_nodes:
            node.remove()
        path_nodes.clear()
        if obstacle[0] is not None:
            obstacle[0].visible = False
        baseline[0] = None
        current.update(pre_rnea=None, post_rnea=None, dynamics_detail=None)
        pre_shadow_frame.visible = post_shadow_frame.visible = False
        for stage in STAGES:
            ledger[stage].update(core_ms=None, core_rows=[], diagnostics=[], notes=[],
                                 timing_ms=0., timing_rows=[], timing_diagnostics=[], solves=[], motion_s=0.,
                                 running=False, failure=None)
        overall.value = 0.

    def run_process(command, event_type):
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   text=True, bufsize=1)
        lines = deque(maxlen=50)
        for line in process.stdout:
            line = line.rstrip()
            lines.append(line)
            if line.startswith(PROGRESS_PREFIX):
                try:
                    inbox.put(json.loads(line[len(PROGRESS_PREFIX):]))
                except json.JSONDecodeError:
                    inbox.put({"event": "log", "text": line})
            elif line:
                inbox.put({"event": event_type, "text": line})
        return process.wait(), "\n".join(lines)

    def run_round(values):
        root = Path(tempfile.mkdtemp(prefix="motion292-live-"))
        cycle = root / "fresh-cycle.json"
        began = time.perf_counter()
        code, log = run_process([sys.executable, str(args.preparer), "--output", str(cycle),
                                 "--round", str(values["round"])], "baseline_log")
        if code or not cycle.exists():
            inbox.put({"event": "run_failed", "error": f"初始整轮规划失败\n{log[-1500:]}"})
            return
        record = json.loads(cycle.read_text())
        result = record["result"]
        if not result.get("success"):
            inbox.put({"event": "run_failed", "error": f"初始整轮规划拒绝：{result.get('error')}"})
            return
        inbox.put({"event": "baseline_ready", "result": result,
                   "wall_s": time.perf_counter() - began,
                   "initialization_s": record.get("initialization_s", 0.),
                   "preparation_wall_s": record.get("preparation_wall_s", 0.),
                   "planner_ms": result.get("planning_timing_ms", {}).get(
                       "geometric_planning_ms", result.get("total_ms", 0.)),
                   "timing_ms": result.get("trajectory_timing_ms", 0.),
                   "dynamics_ms": result.get("torque_constraint_ms", 0.),
                   "controller_ms": result.get("planning_timing_ms", {}).get("controller_export_ms", 0.),
                   "end_to_end_ms": result.get("planning_timing_ms", {}).get(
                       "end_to_end_ms", result.get("measured_wall_ms", 0.))})
        output_dir = root / "rolling-stages"
        code, log = run_process(round_command_for(args, cycle, values, output_dir), "runner_log")
        for stage in STAGES:
            output = output_dir / f"{stage}.json"
            report = json.loads(output.read_text()) if output.exists() else None
            inbox.put({"event": "stage_report", "stage": stage, "report": report})
            if report is not None and not report.get("success"):
                inbox.put({"event": "run_failed", "error":
                           f"{PHASE_LABELS[stage]}：{report.get('failure_code', 'PLAN_REJECTED')}\n{log[-1500:]}"})
                return
        if code:
            inbox.put({"event": "run_failed", "error": f"常驻滚动服务失败\n{log[-1500:]}"})
            return
        inbox.put({"event": "run_finished", "round": values["round"]})

    def work(values):
        try:
            run_round(values)
        except Exception as error:
            inbox.put({"event": "run_failed", "error": f"{type(error).__name__}: {error}"})

    @run_button.on_click
    def _(_event):
        if busy.is_set():
            return
        try:
            values = controls()
        except ValueError as error:
            status.content = f"**输入错误：** {error}"
            return
        reset_view(); busy.set(); run_button.disabled = True
        status.content = f"**第 {values['round'] + 1} 轮：正在从头规划完整初始轨迹…**"
        threading.Thread(target=work, args=(values,), name="motion292-live-round", daemon=True).start()

    replace_obstacle(size.value)
    last_render = 0.
    while True:
        changed = False
        try:
            while True:
                event = inbox.get_nowait(); changed = True
                kind = event["event"]
                if kind == "baseline_log":
                    status.content = f"**正在从头规划完整初始轨迹…**\n\n`{event['text'][-500:]}`"
                elif kind == "baseline_ready":
                    result = event["result"]
                    startup_ms = max(0., event["wall_s"] * 1000 -
                                     event["initialization_s"] * 1000 - event["preparation_wall_s"] * 1000)
                    residual_ms = max(0., event["preparation_wall_s"] * 1000 -
                                      event["end_to_end_ms"])
                    timing_breakdown = result.get("trajectory_timing_breakdown_ms", {})
                    phase_details = timing_breakdown.get("phases", {})
                    phase_timing = [(f"{PHASE_LABELS[name]}阶段", sum(values.values()))
                                    for name, values in phase_details.items() if name in PHASE_LABELS]
                    event_timing = sum(sum(values.values()) for name, values in phase_details.items()
                                       if name not in PHASE_LABELS)
                    timing_rows = [
                        ("时间化准备", timing_breakdown.get("setup", 0.)),
                        *phase_timing,
                        ("attach/release/base等事件帧处理", event_timing),
                        ("整轮轨迹组装与最终约束验证",
                         timing_breakdown.get("cycle_assembly_and_validation", 0.)),
                        ("未归属微小开销", timing_breakdown.get("unattributed", 0.)),
                    ]
                    baseline[0] = {
                        "wall_s": event["wall_s"], "startup_ms": startup_ms,
                        "initialization_ms": event["initialization_s"] * 1000,
                        "planner_ms": event["planner_ms"], "timing_ms": event["timing_ms"],
                        "dynamics_ms": event["dynamics_ms"], "controller_ms": event["controller_ms"],
                        "residual_ms": residual_ms, "core_rows": planning_ledger(result),
                        "timing_rows": [(name, value) for name, value in timing_rows if value > 0.],
                        "rolling_service_ms": None, "rolling_warmup_ms": None,
                    }
                    for stage in STAGES:
                        ledger[stage].update(stage_planning_rows(result, stage))
                    status.content = "**初始整轮新轨迹已完成。** 现在逐阶段执行滚动优化；路线会在实际产出时追加。"
                elif kind == "rolling_service_ready":
                    if baseline[0] is not None:
                        baseline[0]["rolling_service_ms"] = event["service_initialization_s"] * 1000
                        baseline[0]["rolling_warmup_ms"] = event["warmup_s"] * 1000
                    status.content = ("**滚动服务已一次性初始化并完成空载/带载预热。** "
                                      "后续六阶段复用同一个CUDA worker。")
                elif kind == "round_finished":
                    pass
                elif kind == "stage_started":
                    stage = event["stage"]; current["stage"] = stage
                    install_snapshot(event["initial_snapshot"])
                    add_route(stage, "upstream", event["initial_trajectory"], kind="upstream")
                    curve = TimedTrajectory(event["initial_trajectory"])
                    update_robot(curve.joint_names, curve.sample(0.)["positions"])
                    ledger[stage]["running"] = True
                    status.content = f"**{PHASE_LABELS[stage]}：原轨迹刚刚生成，开始实时滚动优化。**"
                elif kind == "solve_finished":
                    ledger[event["stage"]]["solves"].append(event)
                    detail_ref = event.get("dynamics_detail")
                    if detail_ref and Path(detail_ref["path"]).is_file():
                        encoded = Path(detail_ref["path"]).read_bytes()
                        if hashlib.sha256(encoded).hexdigest() == detail_ref["sha256"]:
                            detail = json.loads(encoded)
                            current["dynamics_detail"] = detail
                            current["pre_rnea"] = detail.get("pre_rnea_trajectory")
                            refresh_dynamics_curves()
                            if current["pre_rnea"]:
                                add_route(event["stage"], f"candidate-{event['sequence']}",
                                          current["pre_rnea"], kind="candidate",
                                          sequence=int(event["sequence"] or 0))
                    summary = event.get("dynamics_summary") or {}
                    dynamics_status.content = (
                        f"**{event.get('strategy', 'unknown')}** · 来源 `{summary.get('source', 'upstream')}`  \n"
                        f"几何差：**{summary.get('maximum_position_delta', 0):.1e}** · "
                        f"峰值利用率：**{summary.get('maximum_utilization', 0)*100:.2f}%**  \n"
                        f"RNEA模式：`{summary.get('retiming_mode', 'certificate_reuse')}` · "
                        f"time scale：**{summary.get('time_scale_peak', 1):.3f}**")
                elif kind == "trajectory_activated":
                    current["post_rnea"] = event["trajectory"]
                    add_route(event["stage"], f"installed-{event['sequence']}", event["trajectory"],
                              kind="installed", sequence=int(event["sequence"] or 0))
                    refresh_layer_visibility()
                elif kind == "motion_sample":
                    ledger[event["stage"]]["motion_s"] = event["elapsed_s"]
                    update_robot(event["joint_names"], event["positions"])
                    if show_shadows.value and current.get("pre_rnea") and current.get("post_rnea"):
                        wall = event.get("trajectory_elapsed_s", 0.)
                        pre_curve, post_curve = (TimedTrajectory(current["pre_rnea"]),
                                                 TimedTrajectory(current["post_rnea"]))
                        pre_shadow.update_cfg(dict(zip(pre_curve.joint_names,
                            pre_curve.sample(min(wall, pre_curve.duration))["positions"])))
                        post_shadow.update_cfg(dict(zip(post_curve.joint_names,
                            post_curve.sample(min(wall, post_curve.duration))["positions"])))
                        pre_shadow_frame.visible = post_shadow_frame.visible = True
                elif kind == "observation":
                    obj = event.get("obstacle")
                    obstacle[0].visible = obj is not None
                    if obj is not None:
                        obstacle[0].position = obj["pose"]["position"]
                        obstacle[0].wxyz = obj["pose"]["quaternion_wxyz"]
                elif kind == "stage_finished":
                    item = ledger[event["stage"]]
                    item.update(motion_s=event["motion_wall_s"], running=False,
                                failure=None if event["success"] else event.get("failure_code"))
                    overall.value = 100. * (STAGES.index(event["stage"]) + 1) / len(STAGES)
                elif kind == "stage_report":
                    report = event["report"] or {}
                    if report.get("started_at") is not None:
                        ledger[event["stage"]]["motion_s"] = report.get("completed_at", report["started_at"]) - report["started_at"]
                elif kind == "run_failed":
                    busy.clear(); run_button.disabled = False
                    status.content = f"**本轮停止：** {event['error']}"
                elif kind == "run_finished":
                    busy.clear(); run_button.disabled = False; overall.value = 100.
                    total_motion = sum(item["motion_s"] for item in ledger.values())
                    status.content = f"**第 {event['round'] + 1} 轮完成。** 六阶段仿真实际运动共 {total_motion:.3f} s。"
        except queue.Empty:
            pass
        now = time.perf_counter()
        if changed or now - last_render > .25:
            timing.content = metrics_markdown(ledger, baseline[0])
            last_render = now
        time.sleep(.02)


if __name__ == "__main__":
    main()
