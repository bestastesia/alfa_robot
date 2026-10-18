#!/usr/bin/env python3
"""Fresh, live Viser diagnosis for one complete MOTION-292 transport round; never hardware."""
import argparse
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
from curobo_core.contracts import PlannerAssets
from curobo_core.scene import SceneSnapshot
from curobo_core.trajectory import TimedTrajectory
from v3_rolling_replan import PROGRESS_PREFIX
from v3_timed_wall_report import PHASE_LABELS, phase_planning_detail


STAGES = ("approach", "contact_approach", "extract", "transport", "conveyor_place", "return_home")
# Proven per-stage budgets from the frozen acceptance matrix. One global budget cannot satisfy
# both the 1.5 s contact phase and the long free-space phases.
STAGE_SCHEDULES = {
    "approach": (1.8, .1, 8),
    "contact_approach": (.2, .1, 5),
    "extract": (.2, .1, 32),
    "transport": (1.8, .1, 4),
    "conveyor_place": (.85, .1, 8),
    "return_home": (1.8, .1, 4),
}
ROLL_COLORS = (
    ((30, 225, 80), (30, 155, 245)),
    ((105, 235, 45), (120, 100, 245)),
    ((230, 205, 40), (220, 90, 210)),
    ((245, 145, 35), (35, 210, 215)),
)


def command_for(args, cycle, controls, stage, output):
    t2, t3, slices = STAGE_SCHEDULES[stage]
    scenario = controls["scenario"] if stage == controls["obstacle_stage"] else "clear"
    command = [sys.executable, str(args.runner), "--initial-cycle", str(cycle),
               "--round", "0", "--stage", stage, "--scenario", scenario,
               "--compute-bound-s", str(t2), "--link-bound-s", str(t3),
               "--link-delay-s", str(min(controls["link_delay"], t3)),
               "--slices", str(slices), "--braking-duration-s", str(controls["braking"]),
               "--seed", str(controls["seed"]), "--execution-timeout-s",
               str(args.execution_timeout_s), "--output", str(output), "--stream-progress"]
    if scenario != "clear":
        command += ["--obstacle-center", *map(str, controls["center"]),
                    "--obstacle-size", *map(str, controls["size"]),
                    "--obstacle-velocity", *map(str, controls["velocity"])]
    return command


def stage_planning_rows(result, stage):
    detail = phase_planning_detail(result, stage)
    additive = detail["additive"]
    return sum(value for _, value in additive), additive, detail["diagnostics"]


def metrics_markdown(ledger, baseline):
    rows = ["## 实时耗时账", "所有时间均来自本次点击后新启动的规划。"]
    if baseline:
        rows += [f"**整轮初始规划墙钟：{baseline['wall_s']:.3f} s**  ",
                 f"规划核心：{baseline['planner_ms']:.1f} ms · 时间化：{baseline['timing_ms']:.1f} ms"]
    for stage in STAGES:
        item = ledger[stage]
        rows.append(f"### {PHASE_LABELS[stage]}")
        if item["initial_ms"] is None:
            rows.append("- 初始完整轨迹：等待本轮从头规划")
        else:
            details = "；".join(f"{name} {value:.1f} ms" for name, value in item["initial_rows"])
            rows.append(f"- 初始完整轨迹：**{item['initial_ms']:.1f} ms**（{details or '计入整轮共享账'}）")
            for name, value in item["diagnostics"]:
                rows.append(f"  - {name}：{value:.1f} ms（嵌套诊断，不重复相加）")
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
                        help="prepared isolated-runtime rolling runner")
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
    obstacle = [None]
    scene_nodes, payload_nodes, path_nodes = [], [], []
    inbox = queue.Queue()
    busy = threading.Event()
    current = {"snapshot": None, "base_transform": np.eye(4), "stage": None}
    baseline = [None]
    ledger = {stage: {"initial_ms": None, "initial_rows": [], "diagnostics": [], "solves": [],
                      "motion_s": 0., "running": False, "failure": None} for stage in STAGES}

    server.gui.add_markdown(
        "## MOTION-292 · 一轮完整搬运实时诊断\n"
        "**灰色：本次新规划原轨迹；绿色：左末端滚动新轨迹；蓝色：右末端滚动新轨迹；红色：障碍。**\n\n"
        "界面启动时不加载任何预计算路线。点击后先从头规划本轮，再按六阶段实时追加滚动结果。仅仿真。")
    run_button = server.gui.add_button("▶ 从头规划并实时演示完整一轮", color="green")
    status = server.gui.add_markdown("**就绪。** 当前没有显示任何轨迹。")
    timing = server.gui.add_markdown(metrics_markdown(ledger, None))
    overall = server.gui.add_slider("本轮进度 / %", min=0., max=100., step=.1, initial_value=0.)
    overall.disabled = True
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

    def add_route(stage, name, trajectory, original=False, sequence=0):
        colors = ((85, 85, 85), (155, 155, 155)) if original else ROLL_COLORS[sequence % len(ROLL_COLORS)]
        lines = trajectory_lines(trajectory, current["base_transform"])
        for side, color in zip(("left", "right"), colors):
            path_nodes.append(server.scene.add_line_segments(
                f"/paths/{stage}/{name}/{side}", points=lines[side], colors=color,
                line_width=3. if original else 6.))

    def install_snapshot(data):
        for node in scene_nodes + [node for _, node in payload_nodes]:
            node.remove()
        scene_nodes.clear(); payload_nodes.clear()
        snapshot = SceneSnapshot.from_dict(data)
        current["snapshot"] = snapshot
        current["base_transform"] = pose_matrix(snapshot.state.base_pose)
        robot_frame.position = snapshot.state.base_pose.position
        robot_frame.wxyz = snapshot.state.base_pose.quaternion_wxyz
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
        for node in path_nodes:
            node.remove()
        path_nodes.clear()
        if obstacle[0] is not None:
            obstacle[0].visible = False
        baseline[0] = None
        for stage in STAGES:
            ledger[stage].update(initial_ms=None, initial_rows=[], diagnostics=[], solves=[],
                                 motion_s=0., running=False, failure=None)
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
                   "planner_ms": result.get("total_ms", 0.),
                   "timing_ms": result.get("trajectory_timing_ms", 0.)})
        for stage in STAGES:
            output = root / f"{stage}.json"
            code, log = run_process(command_for(args, cycle, values, stage, output), "runner_log")
            report = json.loads(output.read_text()) if output.exists() else None
            inbox.put({"event": "stage_report", "stage": stage, "report": report})
            if code or not report or not report.get("success"):
                failure = (report or {}).get("failure_code", "PROCESS_FAILED")
                inbox.put({"event": "run_failed", "error": f"{PHASE_LABELS[stage]}：{failure}\n{log[-1500:]}"})
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
                    baseline[0] = {key: event[key] for key in ("wall_s", "planner_ms", "timing_ms")}
                    for stage in STAGES:
                        total, rows, diagnostics = stage_planning_rows(event["result"], stage)
                        ledger[stage].update(initial_ms=total, initial_rows=rows, diagnostics=diagnostics)
                    status.content = "**初始整轮新轨迹已完成。** 现在逐阶段执行滚动优化；路线会在实际产出时追加。"
                elif kind == "stage_started":
                    stage = event["stage"]; current["stage"] = stage
                    install_snapshot(event["initial_snapshot"])
                    add_route(stage, "original", event["initial_trajectory"], original=True)
                    curve = TimedTrajectory(event["initial_trajectory"])
                    update_robot(curve.joint_names, curve.sample(0.)["positions"])
                    ledger[stage]["running"] = True
                    status.content = f"**{PHASE_LABELS[stage]}：原轨迹刚刚生成，开始实时滚动优化。**"
                elif kind == "solve_finished":
                    ledger[event["stage"]]["solves"].append(event)
                elif kind == "trajectory_activated":
                    add_route(event["stage"], f"rolling-{event['sequence']}", event["trajectory"],
                              sequence=int(event["sequence"] or 0))
                elif kind == "motion_sample":
                    ledger[event["stage"]]["motion_s"] = event["elapsed_s"]
                    update_robot(event["joint_names"], event["positions"])
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
