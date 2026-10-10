import argparse
from dataclasses import replace
import json
from pathlib import Path
import threading
import time



PHASE_NAMES = {
    "home": "第一初始姿态", "approach": "初始→吸附起点", "attach": "吸附箱体",
    "extract": "直线脱离35cm（侧退/顶升）", "transport": "抽离终点→放置姿态",
    "turn_loaded": "携箱转身180°", "release": "放置释放，箱体消失",
    "turn_empty": "空载转身回来", "return_home": "双臂回到原第一初始姿态",
    "transport_diagnostic": "失败轨迹诊断",
}



def planning_statistics(result):
    """Display recorded planning telemetry, never infer execution time from frames."""
    timing_labels = {
        "initial_checker_setup_and_validation": "初始碰撞检查器初始化与检查",
        "contact_solver_setup": "接触IK求解器初始化",
        "contact_ik_and_ranking": "接触IK求解与排序",
        "loaded_checker_setup": "载荷碰撞检查器初始化",
        "analytic_extract_and_validation": "抽离路径生成与检查",
        "approach_rrt_and_validation": "接近路径搜索与检查（含候选尝试）",
        "transport_pipeline_total": "携箱运输流程（包含下表子项）",
        "empty_turn_checker_setup": "空载转身检查器初始化",
        "empty_turn_validation": "空载转身检查",
        "home_rrt_and_validation": "回Home搜索与检查",
        "other_assembly_and_bookkeeping": "其他组装与记录",
    }
    transport_labels = {
        "setup": "运输求解器准备", "target_ik": "搬运目标IK", "target_filter": "目标筛选",
        "rrt": "携箱路径搜索", "frame_assembly": "轨迹帧组装",
        "mobile_checker_setup": "移动底盘检查器初始化",
        "transport_and_turn_validation": "运输与携箱转身检查", "cleanup": "资源清理",
    }
    ms = lambda value: "未记录" if value is None else f"{value:.2f} ms"
    lines = [f"**本轮累计规划耗时：{ms(result.get('total_ms'))}**（包含失败尝试）",
             f"选中方案耗时：{ms(result.get('selected_mode_total_ms'))}",
             "以下是规划计算耗时，不是动作执行耗时；回放帧未做时间参数化。",
             "", "| 规划阶段 | 耗时 |", "|---|---:|"]
    lines += [f"| {label} | {ms(result.get('timing_ms', {}).get(key))} |"
              for key, label in timing_labels.items()]
    if result.get("transport_timing_breakdown_ms"):
        lines += ["", "**携箱运输子项（已包含在上表，不能重复累加）**", "", "| 子项 | 耗时 |", "|---|---:|"]
        lines += [f"| {label} | {ms(result['transport_timing_breakdown_ms'].get(key))} |"
                  for key, label in transport_labels.items()]
    selected = next((a.get("approach", {}) for a in result.get("attempts", [])
                     if a.get("candidate") == result.get("selected_candidate")), {})
    lines += ["", "**搜索统计**"]
    for label, stats in (("接近", selected), ("携箱", result.get("transport_search_stats", {})),
                         ("回Home", result.get("home_search_stats", {}))):
        if stats.get("fallback"):
            label += "（反向复用；下列为失败搜索）"
            stats = stats.get("failed_search", {})
        shortcut = "未记录" if "shortcut_applied" not in stats else ("启用" if stats["shortcut_applied"] else "未启用")
        lines += ["", f"**{label} · 捷径{shortcut}**", "",
                  f"首解：{ms(stats.get('first_solution_ms'))} · 搜索：{ms(stats.get('search_ms'))}  ",
                  f"路径点（原始→精简）：{stats.get('raw_waypoints', '—')}→{stats.get('shortcut_waypoints', '—')}  ",
                  f"迭代：{stats.get('iterations', '未记录')} · 树节点：{stats.get('tree_nodes', '未记录')} · "
                  f"发现解：{stats.get('solutions_found', '未记录')}  ",
                  f"搜索种子：{stats.get('search_seed', '未记录')}"]
        optimization = stats.get("trajectory_optimization")
        if optimization:
            verdict = "采纳" if optimization.get("accepted") else "回退原RRT"
            lines.append(
                f"cuRobo TrajOpt：**{verdict}** · {ms(optimization.get('wall_ms'))} · "
                f"输出{optimization.get('output_frames', '—')}帧 · 动力学/扭矩未启用")
    if result.get("idle_contact_policy"):
        source = result.get("idle_contact_source")
        idle_text = ("接触时空闲手采用解析垂直补偿，保持初始水平位置与朝向。"
                     if source == "analytic_vertical_compensation" else
                     "接触时保留碰撞合法的IK空闲臂状态，并按空闲臂关节运动最小选优。")
        lines += ["", "**单箱动作约束**", "", idle_text,
                  f"空闲TCP世界高度偏移：{result.get('idle_contact_height_offset_m', '未记录')} m  ",
                  f"双肩关节相对初始姿态最大偏移：{result.get('shoulder_excursion_limit_deg', '未记录')}°（不是速度限制）。"]
    lines += ["", f"接触IK成功候选：{result.get('contact_ik_count', '未记录')} · "
              f"选中候选：{result.get('selected_candidate', '未记录')} · 候选尝试：{len(result.get('attempts', []))}",
              f"抓取方案尝试：{len(result.get('suction_attempts', []))} · 轨迹帧：{len(result.get('frames', []))}",
              "", "| 动作阶段 | 帧数（非耗时） |", "|---|---:|"]
    phases = result.get("phases", [])
    lines += [f"| {label} | {phases.count(key)} |" for key, label in PHASE_NAMES.items() if key in phases]
    return "\n".join(lines)


def main():
    import numpy as np
    import trimesh
    import viser
    from viser.extras import ViserUrdf
    import yourdfpy

    from v3_full_cycle_planner import FullCyclePlanner, CycleBlocked
    from curobo_core.adapter import pose_matrix
    from curobo_core.scene import Pose
    from curobo_core.fixtures import WallLayout, add_wall_arguments, wall_layout_from_args, wall_sequence
    from v3_interactive_60_tasks import (
        DEFAULT_ROBOT_CONFIG, DEFAULT_URDF, DEFAULT_NAMED_POSES, DEFAULT_BOX_FIT,
        DEFAULT_MOBILE_ROBOT_CONFIG, quaternion,
    )
    from v3_wall_ik_benchmark import canonical_side_tool_to_box, wall_center

    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-config", type=Path, default=DEFAULT_ROBOT_CONFIG)
    parser.add_argument("--mobile-robot-config", type=Path, default=DEFAULT_MOBILE_ROBOT_CONFIG)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--named-poses", type=Path, default=DEFAULT_NAMED_POSES)
    parser.add_argument("--box-fit", type=Path, default=DEFAULT_BOX_FIT)
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--comparison-json", type=Path, nargs="+")
    parser.add_argument("--search-seed", type=int, help="RRT sampling seed; defaults preserve the mentor baseline")
    parser.add_argument("--trajopt-rrt", action="store_true",
                        help="seed cuRobo TrajOpt from 15-DoF RRT segments; dynamics/torque stay disabled")
    parser.add_argument("--trajopt-interpolation-dt", type=float, default=0.025,
                        help="seconds between optimized q/qdot/qddot/jerk samples")
    parser.add_argument("--suction-mode", choices=("auto", "side", "top"), default="auto")
    parser.add_argument("--sequence", action="store_true", help="show the complete wall sequence")
    parser.add_argument("--replay-only", action="store_true", help="disable planning controls; visualization only, no GPU planning")
    parser.add_argument("--cycle-json", type=Path, help="replay a matching headless request/results file")
    parser.add_argument("--sequence-json", type=Path, help="replay a matching sequence result; not a fresh plan")
    add_wall_arguments(parser)
    args = parser.parse_args()
    if args.search_seed is not None and not 0 <= args.search_seed < 2**32:
        parser.error("--search-seed must be an unsigned 32-bit integer")
    if args.trajopt_interpolation_dt <= 0:
        parser.error("--trajopt-interpolation-dt must be positive")
    try:
        args.wall_layout = wall_layout_from_args(args)
    except (ValueError, TypeError) as error:
        parser.error(str(error))
    if args.comparison_json and args.wall_layout != WallLayout():
        parser.error("preloaded comparisons belong to the default wall; recompute a custom layout")
    if args.replay_only and not (args.sequence_json or args.cycle_json):
        parser.error("--replay-only requires --sequence-json or --cycle-json")
    if args.cycle_json and (args.sequence or args.sequence_json or args.comparison_json):
        parser.error("--cycle-json cannot be combined with sequence/comparison replay")
    if args.comparison_json and (args.sequence or args.sequence_json):
        parser.error("single-task comparisons cannot preload a wall sequence")
    comparisons = {}
    for path in args.comparison_json or []:
        comparisons.update(json.loads(path.read_text()))
    planner_labels = {"Batched RRT": "rrt", "Informed RRT": "informed_rrt",
                      "Batched RRTConnect": "rrtconnect", "GPU PRM": "prm",
                      "Connect + Informed": "informed_connect", "BIT* + GPU": "bitstar"}
    planner = FullCyclePlanner(args)
    initial_snapshot = planner.snapshot
    preloaded_sequence = None
    preloaded_cycle = None
    if args.cycle_json:
        from curobo_core.contracts import PlanRequest
        document = json.loads(args.cycle_json.read_text())
        recorded_request = PlanRequest.from_dict(document["request"])
        preloaded_cycle = document["results"][-1]
        if args.replay_only:
            # A later single-box round legitimately starts with previously removed boxes.
            remaining = {obj.object_id for obj in recorded_request.snapshot.objects}
            expected = replace(initial_snapshot, objects=tuple(
                obj for obj in initial_snapshot.objects
                if not obj.object_id.startswith("wall_box_") or obj.object_id in remaining))
            if recorded_request.snapshot.geometry_key != expected.geometry_key:
                parser.error("cycle replay model, retained geometry or collision policy differs")
            planner.set_snapshot(recorded_request.snapshot)
            initial_snapshot = recorded_request.snapshot
            planner.wall_layout = replace(planner.wall_layout, active_box_ids=tuple(
                int(name.removeprefix("wall_box_")) for name in remaining if name.startswith("wall_box_")))
            planner.selected_task = dict(recorded_request.tasks)
            label = "记录任务 · " + " / ".join(f"{side}:{box}" for side, box in recorded_request.tasks)
            planner.task_options = [(label, planner.selected_task)]
            planner.task_map = dict(planner.task_options)
        if (recorded_request.snapshot.geometry_key != initial_snapshot.geometry_key or
                not np.allclose(recorded_request.snapshot.state.ordered(tuple(planner.home)),
                                initial_snapshot.state.ordered(tuple(planner.home)), atol=1e-7, rtol=0) or
                recorded_request.identity != preloaded_cycle["request_id"] or
                dict(recorded_request.tasks) != planner.selected_task):
            parser.error("cycle replay does not match the model, scene, state or selected task")
        args.suction_mode = recorded_request.suction_mode
        if args.search_seed is not None and args.search_seed != recorded_request.search_seed:
            parser.error("cycle replay has a different search seed; recompute instead")
        args.search_seed = recorded_request.search_seed
    if args.sequence_json:
        from curobo_core.scene import SceneSnapshot
        preloaded_sequence = json.loads(args.sequence_json.read_text())
        recorded_seed = preloaded_sequence.get("search_seed")
        if args.search_seed is not None and args.search_seed != recorded_seed:
            parser.error("sequence replay has a different search seed; recompute instead")
        args.search_seed = recorded_seed
        recorded = SceneSnapshot.from_dict(preloaded_sequence["initial_snapshot"])
        if not preloaded_sequence.get("sequence") or recorded.geometry_key != initial_snapshot.geometry_key:
            parser.error("sequence replay belongs to a different model, layout or collision policy")
        if recorded.state.ordered(tuple(planner.home)) != initial_snapshot.state.ordered(tuple(planner.home)):
            parser.error("sequence replay initial robot state differs")
    config = planner.mobile_robot["kinematics"]
    joint_names = config["cspace"]["joint_names"]
    urdf = yourdfpy.URDF.load(config["urdf_path"], load_meshes=True, build_scene_graph=True)
    server = viser.ViserServer(host="127.0.0.1", port=args.port,
                               label="V3解析代理 · 抓取抽离放置回Home完整流程")
    server.scene.set_up_direction("+z")
    server.scene.add_grid("/ground", width=5, height=3)
    robot = ViserUrdf(server, urdf, root_node_name="/robot",
                      mesh_color_override=(0.68, 0.73, 0.80, 0.8))
    joint_models = {joint.name: joint for joint in urdf.robot.joints if joint.type != "fixed"}
    velocity_arrows = server.scene.add_arrows(
        "/trajectory/joint_velocity",
        points=np.zeros((len(joint_names), 2, 3), dtype=np.float32),
        colors=np.array([(61, 170, 210) if joint_models[name].type == "prismatic" else (239, 146, 62)
                         for name in joint_names], dtype=np.uint8),
        shaft_radius=.006, head_radius=.014, head_length=.025, visible=False)
    attached = {}
    sphere_mesh = trimesh.creation.icosphere(subdivisions=2)
    for side in ("left", "right"):
        attached[side] = server.scene.add_frame(f"/attached/{side}", show_axes=False, visible=False)
        server.scene.add_box(f"/attached/{side}/box", dimensions=planner.box_fit["dimensions_m"],
                             color=(239, 146, 62), opacity=0.8)
        server.scene.add_frame(f"/attached/{side}/spheres", show_axes=False, visible=False)
        for sphere_index, (center, radius) in enumerate(zip(planner.box_fit["centers"], planner.box_fit["radii"])):
            server.scene.add_mesh_simple(
                f"/attached/{side}/spheres/{sphere_index}",
                vertices=sphere_mesh.vertices*radius, faces=sphere_mesh.faces,
                position=center, color=(61, 170, 210), opacity=0.5)
    source_boxes = {}
    for item in planner.snapshot.objects:
        if not item.object_id.startswith("wall_box_"):
            continue
        box_id = int(item.object_id.removeprefix("wall_box_"))
        source_boxes[box_id] = server.scene.add_box(
            f"/wall/box_{box_id}", position=item.pose.position,
            wxyz=item.pose.quaternion_wxyz, dimensions=item.dimensions_m,
            color=(87, 145, 165), opacity=.25)
    for item in planner.snapshot.objects:
        if not item.object_id.startswith("wall_box_") and item.object_id != "ground":
            server.scene.add_box(f"/container/{item.object_id}", dimensions=item.dimensions_m,
                                 position=item.pose.position, wxyz=item.pose.quaternion_wxyz,
                                 color=(149, 160, 174), opacity=.06)
    labels = [entry[0] for entry in planner.task_options]
    with server.gui.add_folder("完整任务实时规划"):
        layout = planner.wall_layout
        server.gui.add_markdown(
            f"箱墙：**{layout.rows}×{layout.columns}** · 实箱 **{len(layout.active_box_ids)}**  \n"
            f"初始车头→箱墙近面：**{layout.distance_m:.3f} m**  \n"
            "研究碰撞策略；不代表完整网格碰撞或实机验收。"
        )
        mode = server.gui.add_dropdown("规划范围", options=("单任务", "整墙序列"),
                                       initial_value="整墙序列" if args.sequence or args.sequence_json else "单任务")
        sequence_tasks = wall_sequence(layout)
        round_labels = [f"{i+1:02d} · 排{layout.rows-next(iter(t.values()))//layout.columns} · " +
                        " / ".join(f"{side[0].upper()}{box_id:02d}" for side, box_id in t.items())
                        for i, t in enumerate(sequence_tasks)]
        round_selector = server.gui.add_dropdown("查看轮次", options=round_labels, initial_value=round_labels[0])
        with server.gui.add_folder("完整搬运顺序", expand_by_default=False) as sequence_folder:
            sequence_table = server.gui.add_markdown("")
        provenance = server.gui.add_markdown("只读回放，不启动GPU规划。" if args.replay_only else
                                             ("已保存结果回放；点击计算可实时重算。" if args.sequence_json or args.cycle_json else "尚未计算。"))
        task = server.gui.add_dropdown("组合任务", options=labels, initial_value=labels[0])
        suction_modes = {"自动（侧吸→顶吸）": "auto", "仅侧吸": "side", "仅顶吸": "top"}
        suction = server.gui.add_dropdown("吸附策略", options=tuple(suction_modes),
                                         initial_value=next(k for k, v in suction_modes.items() if v == args.suction_mode))
        planner_selector = server.gui.add_dropdown("规划器", options=list(planner_labels),
                                                   initial_value="Informed RRT")
        run = server.gui.add_button("仅回放（实时规划已禁用）" if args.replay_only else "计算完整流程并播放")
        run.disabled = args.replay_only
        mode.disabled = args.replay_only
        planner_selector.disabled = args.replay_only
        play = server.gui.add_checkbox("播放", initial_value=False)
        playback_step = server.gui.add_slider("无时间轨迹回放步进", min=1, max=20, step=1,
                                              initial_value=5 if args.sequence or args.sequence_json else 1)
        playback_speed = server.gui.add_slider("真实时间回放倍率", min=.1, max=3., step=.1, initial_value=1.)
        frame = server.gui.add_slider("轨迹帧", min=1, max=1, step=1, initial_value=1)
        stage = server.gui.add_dropdown("跳转阶段", options=list(PHASE_NAMES.values()),
                                        initial_value=PHASE_NAMES["home"])
        spheres = server.gui.add_checkbox("显示附着箱40球", initial_value=False)
        with server.gui.add_folder("优化轨迹 q / qdot", expand_by_default=False):
            show_velocity = server.gui.add_checkbox("显示关节速度轴矢量", initial_value=True)
            curve_joint = server.gui.add_dropdown(
                "曲线关节", options=tuple(joint_names), initial_value="left_joint1")
            trajectory_state = server.gui.add_markdown(
                "当前阶段没有时间化轨迹。箭头长度表示速度上限利用率，不是TCP线速度。")
            trajectory_curve = server.gui.add_uplot(
                data=(np.array([0., 1.]), np.zeros(2), np.zeros(2)),
                series=(viser.uplot.Series(label="time"),
                        viser.uplot.Series(label="q", stroke="#3daad2"),
                        viser.uplot.Series(label="qdot", stroke="#ef923e")),
                mode=1, scales={"x": {"time": False}},
                title="选中关节的时间轨迹（秒）", height=320, visible=False)
        status = server.gui.add_markdown("从第一初始姿态开始，一次计算完整流程。")
        comparison_status = server.gui.add_markdown("")
        with server.gui.add_folder("中文阶段耗时与统计", expand_by_default=False):
            statistics = server.gui.add_markdown("尚无规划记录。")
    current = {"result": None, "busy": False, "statistics_result": None,
               "curve_binding": None, "driving_playback": False,
               "playback_result": None, "playback_start_wall": 0., "playback_start_time": 0.}

    def refresh_sequence():
        whole_wall = mode.value == "整墙序列"
        task.disabled = whole_wall or current["busy"] or args.replay_only
        suction.disabled = whole_wall or current["busy"] or args.replay_only
        round_selector.visible = whole_wall
        sequence_folder.visible = whole_wall
        result = current["result"]
        records = result["rounds"] if result and result.get("sequence") else []
        states = {"completed": "通过", "failed": "失败停在此轮", "pending": "待规划"}
        lines = ["**完整搬运顺序：逐轮移除已搬箱，失败即停止。**", ""]
        for i, label in enumerate(round_labels):
            state = states[records[i]["status"]] if records else "待规划"
            selected_mode = records[i].get("result", {}).get("suction_mode") if records else None
            suction_label = " · 顶吸" if selected_mode == "top" else (" · 侧吸" if selected_mode == "side" else "")
            lines.append(f"- {label} — {state}{suction_label}")
        sequence_table.content = "\n".join(lines) if whole_wall else ""


    def refresh_comparison():
        lines = ["已计算完整流程对比（计算结果回放）", "", "| 规划器 | 结果 | 总计算 |",
                 "| --- | --- | --- |"]
        for label, kind in planner_labels.items():
            result = comparisons.get(kind)
            if result is None or not result["frames"]:
                continue
            verdict = "完成全部阶段" if result["success"] else result.get("blocked_stage", "失败")
            seconds = result.get("measured_wall_ms", result.get("total_ms", 0)) / 1000
            lines.append(f"| {label} | {verdict} | {seconds:.2f}s |")
        lines += ["", "对比表是计算结果回放。实时重算包含当次初始化状态；同一接触起点、每段2秒预算；捷径与TrajOpt是否启用以各阶段记录为准。"]
        comparison_status.content = "\n".join(lines) if comparisons else ""

    def timed_sample(cycle_result, index):
        for segment in cycle_result.get("timed_segments", ()) if cycle_result else ():
            if segment["frame_start"] <= index <= segment["frame_end"]:
                return segment, index - segment["frame_start"]
        return None, None

    def update_trajectory_visualization(cycle_result, index, values):
        segment, local_index = timed_sample(cycle_result, index)
        if segment is None:
            velocity_arrows.visible = False
            trajectory_curve.visible = False
            trajectory_state.content = "当前阶段没有时间化轨迹。"
            current["curve_binding"] = None
            return
        names = segment["joint_names"]
        velocity = dict(zip(names, segment["velocities"][local_index]))
        rows = ["| joint | q | qdot | units (q / qdot) |", "|---|---:|---:|---|"]
        for name in names:
            unit = "m" if joint_models[name].type == "prismatic" else "rad"
            rows.append(f"| {name} | {values.get(name, 0.):.5f} | {velocity[name]:.5f} | {unit} / {unit}/s |")
        trajectory_state.content = (
            f"**{segment['name']} · t={segment['time_from_start_s'][local_index]:.3f}s · "
            "研究轨迹，不可直接执行**  \n\n" + "\n".join(rows))
        binding = (id(cycle_result), segment["name"], curve_joint.value)
        if current["curve_binding"] != binding:
            joint_index = names.index(curve_joint.value)
            trajectory_curve.data = (
                np.asarray(segment["time_from_start_s"]),
                np.asarray(segment["positions"])[:, joint_index],
                np.asarray(segment["velocities"])[:, joint_index],
            )
            trajectory_curve.visible = True
            current["curve_binding"] = binding
        points = np.zeros((len(joint_names), 2, 3), dtype=np.float32)
        for joint_index, name in enumerate(joint_names):
            joint = joint_models[name]
            transform = urdf.get_transform(joint.child, urdf.base_link)
            axis = transform[:3, :3] @ np.asarray(joint.axis)
            limit = joint.limit.velocity if joint.limit and joint.limit.velocity else 1.0
            speed = velocity.get(name, 0.0)
            points[joint_index, 0] = transform[:3, 3]
            points[joint_index, 1] = transform[:3, 3] + axis * np.clip(speed / limit, -1., 1.) * .15
        velocity_arrows.points = points
        velocity_arrows.visible = show_velocity.value

    def show_frame(index):
        result = current["result"]
        if result is None or not result["frames"]:
            values = {**planner.home, "base_x": 0., "base_y": 0., "base_yaw": 0.}
            phase = "home"
            has_payload = False
        else:
            values = dict(zip(result["joint_names"], result["frames"][index]))
            phase = result["phases"][index]
            has_payload = result["payload"][index]
        cfg = np.array([values.get(name, 0.) for name in urdf.actuated_joint_names])
        urdf.update_cfg(cfg)
        robot.update_cfg(cfg)
        active_task = planner.selected_task
        removed = set()
        round_number = None
        cycle_result = result
        cycle_frame_index = index
        if result and result.get("sequence") and result["frames"]:
            round_number = result["frame_rounds"][index]
            record = result["rounds"][round_number]
            active_task = record["task"]
            removed = set(record["removed_before"])
            cycle_result = record["result"]
            cycle_frame_index = index - result["frame_rounds"].index(round_number)
        update_trajectory_visualization(cycle_result, cycle_frame_index, values)
        if cycle_result is not current["statistics_result"]:
            summary = ""
            if result and result.get("sequence"):
                summary = (f"**整墙累计规划：{result['total_ms']/1000:.3f} s · "
                           f"完成{len(result['completed_box_ids'])}/{len(layout.active_box_ids)}箱**  \n"
                           f"当前统计：第{round_number+1}轮  \n\n")
            statistics.content = summary + planning_statistics(cycle_result) if cycle_result else "尚无规划记录。"
            current["statistics_result"] = cycle_result
        attachment_transforms = {}
        if cycle_result:
            for event in cycle_result.get("predicted_scene_events", ()):
                if event["phase"] == "attach":
                    attachment_transforms = {item["parent_link"].removesuffix("_tool0"):
                                             pose_matrix(Pose(**item["tool_to_object"]))
                                             for item in event["snapshot"]["attachments"]}
        for side, node in attached.items():
            matrix = urdf.get_transform(side+"_tool0", urdf.base_link) @ attachment_transforms.get(side, canonical_side_tool_to_box(side))
            node.position = matrix[:3, 3]
            node.wxyz = quaternion(matrix)
            node.visible = has_payload and side in active_task
        for box_id, box in source_boxes.items():
            box.visible = box_id not in removed and (box_id not in active_task.values() or phase in ("home", "approach"))
        if result:
            status.content = (
                f"**{'完整流程规划成功' if result['success'] else '已停止：存在阻塞'} · {PHASE_NAMES[phase]}**  \n"
                f"帧：**{index+1}/{len(result['frames'])}**  \n"
                f"接触IK：**{result.get('contact_ik_count', 0)}** · 选中候选：**{result.get('selected_candidate', '无')}**  \n"
                f"全部规划：**{result.get('total_ms', 0)/1000:.3f}s**  \n"
                f"最终Home误差：**{result.get('final_home_error', 0):.8f}**"
            )
            if result.get("sequence"):
                status.content = (
                    f"**整墙规划{'完成' if result['success'] else '未完成'} · "
                    f"已完成{len(result['completed_box_ids'])}/{len(layout.active_box_ids)}箱**  \n"
                    f"当前第{round_number+1 if round_number is not None else 1}/{len(result['rounds'])}轮 · {PHASE_NAMES[phase]}  \n"
                    f"帧：**{index+1}/{len(result['frames'])}** · 累计规划：{result['total_ms']/1000:.3f}s"
                )
            if cycle_result.get("search_seed") is not None:
                status.content += f"  \nRRT搜索种子：{cycle_result['search_seed']}"
            mode_name = "顶吸" if cycle_result.get("suction_mode", "side") == "top" else "侧吸"
            status.content += f"  \n吸附模式：**{mode_name}**"
            if cycle_result.get("contact_offsets_m"):
                status.content += f"  \n吸附面内偏移(m)：{cycle_result['contact_offsets_m']}"
            if not result["success"]:
                blocked = f"阻塞第{result['blocked_round']}轮：" if result.get("sequence") else ""
                status.content += f"  \n{blocked}{result.get('error') or result.get('blocked_details')}"

    def reset_playback_clock():
        result = current["result"]
        times = result.get("time_from_start_s") if result else None
        index = max(0, int(frame.value)-1)
        current["playback_result"] = result
        current["playback_start_wall"] = time.monotonic()
        current["playback_start_time"] = float(times[index]) if times else 0.0

    @play.on_update
    def on_play(_event):
        if play.value:
            reset_playback_clock()

    @curve_joint.on_update
    def on_curve_joint(_event):
        current["curve_binding"] = None
        if current["result"]:
            show_frame(int(frame.value)-1)

    @show_velocity.on_update
    def on_show_velocity(_event):
        if current["result"]:
            show_frame(int(frame.value)-1)

    @frame.on_update
    def on_frame(_event):
        if current["result"]:
            show_frame(int(frame.value)-1)
            if not current["driving_playback"]:
                reset_playback_clock()

    @stage.on_update
    def on_stage(_event):
        if not current["result"]:
            return
        selected = next(key for key, label in PHASE_NAMES.items() if label == stage.value)
        phases = current["result"]["phases"]
        indices = range(len(phases))
        result = current["result"]
        if result.get("sequence"):
            number = round_labels.index(round_selector.value)
            indices = [i for i in indices if result["frame_rounds"][i] == number]
        index = next((i for i in indices if phases[i] == selected), None)
        if index is not None:
            play.value = False
            frame.value = index+1

    @round_selector.on_update
    def on_round(_event):
        result = current["result"]
        if current["busy"] or not result or not result.get("sequence"):
            return
        number = round_labels.index(round_selector.value)
        play.value = False
        if number in result["frame_rounds"]:
            frame.value = result["frame_rounds"].index(number)+1
            stage.value = PHASE_NAMES["home"]
        else:
            status.content = f"第{number+1}轮没有通过的轨迹；未跳过失败继续播放。"

    @mode.on_update
    def on_mode(_event):
        if current["busy"]:
            return
        play.value = False
        current["result"] = preloaded_sequence if mode.value == "整墙序列" else None
        result = current["result"]
        frame.max = max(1, len(result["frames"])) if result else 1
        frame.value = 1
        provenance.content = "已保存结果回放。" if result else "尚未计算。"
        refresh_sequence()
        show_frame(0)

    @spheres.on_update
    def on_spheres(_event):
        for side in ("left", "right"):
            server.scene.add_frame(f"/attached/{side}/spheres", show_axes=False,
                                    visible=spheres.value)

    @task.on_update
    def on_task(_event):
        if current["busy"]:
            return
        play.value = False
        current["result"] = None
        planner.selected_task = planner.task_map[task.value]
        comparisons.clear()
        provenance.content = "任务已改变，尚未计算。"
        planner.comparison_contact = None
        refresh_comparison()
        frame.max = 1
        frame.value = 1
        show_frame(0)

    @suction.on_update
    def on_suction(event):
        on_task(event)

    @planner_selector.on_update
    def on_planner(_event):
        if current["busy"]:
            return
        play.value = False
        planner.planner_kind = planner_labels[planner_selector.value]
        result = comparisons.get(planner.planner_kind) if mode.value == "单任务" else None
        current["result"] = result
        provenance.content = "已计算结果回放。" if result else "规划器已改变，尚未计算。"
        refresh_sequence()
        frame.max = max(1, len(result["frames"])) if result else 1
        frame.value = 1
        show_frame(0)
        if result:
            play.value = result["success"]

    @run.on_click
    def on_run(_event):
        if args.replay_only or current["busy"]:
            return
        current["busy"] = True
        run.disabled = True
        task.disabled = True
        planner_selector.disabled = True
        mode.disabled = True
        suction.disabled = True
        round_selector.disabled = True
        play.value = False
        current["result"] = None
        provenance.content = "正在实时计算，不使用预存轨迹。"
        round_selector.value = round_labels[0]
        refresh_sequence()
        show_frame(0)

        def worker():
            try:
                try:
                    planner.set_snapshot(initial_snapshot)
                    planner.comparison_contact = None
                    progress = lambda text: setattr(status, "content", text)
                    result = (planner.plan_sequence(initial_snapshot, progress, search_seed=args.search_seed) if mode.value == "整墙序列"
                              else planner.plan_request(replace(planner.demo_request(planner.selected_task),
                                                        suction_mode=suction_modes[suction.value]), progress))
                except Exception as error:
                    result = {"success": False, "frames": [], "blocked_stage": "程序异常",
                              "blocked_details": f"{type(error).__name__}: {error}"}
                path = args.robot_config.parent / ("full_wall_latest.json" if mode.value == "整墙序列" else "full_cycle_latest.json")
                path.write_text(json.dumps(result, indent=2, ensure_ascii=False)+"\n")
                current["result"] = result
                provenance.content = "本次实时计算结果；轨迹仅在当前研究碰撞策略下验收。"
                if mode.value == "单任务":
                    comparisons[planner.planner_kind] = result
                refresh_comparison()
                if result["frames"]:
                    frame.max = len(result["frames"])
                    frame.value = 1
                    show_frame(0)
                    play.value = result["success"]
                else:
                    status.content = f"**已停止** {result.get('blocked_stage')}: {result.get('blocked_details')}"
            finally:
                current["busy"] = False
                run.disabled = False
                planner_selector.disabled = False
                mode.disabled = False
                round_selector.disabled = False
                refresh_sequence()
        threading.Thread(target=worker, daemon=True).start()

    @server.on_client_connect
    def on_client(client):
        client.camera.position = (-3.2, 3.4, 2.8)
        client.camera.look_at = (.6, 0, 1.2)
        client.camera.up_direction = (0, 0, 1)

    if comparisons:
        baseline = comparisons.get("informed_rrt")
        if baseline and baseline.get("selected_candidate"):
            planner.comparison_contact = baseline["contact_candidates"][baseline["selected_candidate"]-1]
        current["result"] = baseline
        frame.max = len(baseline["frames"]) if baseline else 1
    if preloaded_cycle:
        current["result"] = preloaded_cycle
        frame.max = max(1, len(preloaded_cycle["frames"]))
    if preloaded_sequence:
        current["result"] = preloaded_sequence
        frame.max = max(1, len(preloaded_sequence["frames"]))
    refresh_comparison()
    refresh_sequence()
    show_frame(0)
    print(f"Ready: http://localhost:{args.port}", flush=True)
    while True:
        result = current["result"]
        if play.value and result:
            times = result.get("time_from_start_s")
            if times:
                if current["playback_result"] is not result:
                    reset_playback_clock()
                target_time = (current["playback_start_time"]
                               + (time.monotonic()-current["playback_start_wall"])*playback_speed.value)
                index = int(np.searchsorted(times, target_time, side="right") - 1)
                if target_time >= times[-1]:
                    index = len(times)-1
                    play.value = False
                current["driving_playback"] = True
                frame.value = index+1
                current["driving_playback"] = False
            elif int(frame.value) >= len(result["frames"]):
                play.value = False
            else:
                current["driving_playback"] = True
                frame.value = min(frame.max, int(frame.value)+int(playback_step.value))
                current["driving_playback"] = False
        time.sleep(.03)


if __name__ == "__main__":
    main()
