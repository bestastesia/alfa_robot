import argparse
import json
from pathlib import Path
import threading
import time

import numpy as np
import trimesh
import viser
from viser.extras import ViserUrdf
import yourdfpy

from v3_task_cycle_planner import TaskCyclePlanner as FullCyclePlanner, CycleBlocked
from v3_task_scene import row_obstacles, grasp_transform
from v3_interactive_60_tasks import (
    DEFAULT_ROBOT_CONFIG, DEFAULT_URDF, DEFAULT_NAMED_POSES, DEFAULT_BOX_FIT,
    DEFAULT_MOBILE_ROBOT_CONFIG, quaternion,
)
from v3_wall_ik_benchmark import canonical_side_tool_to_box, wall_center
from v3_timed_wall_report import PHASE_LABELS, phase_planning_detail, planning_ledger


PHASE_NAMES = {
    'contact_approach': '最后3cm贴近（排除目标箱碰撞）',
    "base_advance": "底盘前进0.3m", "base_advance_diagnostic": "底盘前进失败诊断",
    "home": "第一初始姿态", "approach": "初始→吸附起点", "attach": "吸附箱体",
    "extract": "固定Updown解析直线抽离35cm", "transport": "抽离终点→放置姿态",
    "turn_loaded": "携箱转身180°", "base_reposition": "导航到放置位",
    "conveyor_place": "固定底盘放箱", "release": "放置释放，箱体消失",
    "turn_empty": "空载转身回来", "return_home": "双臂回到原第一初始姿态",
    "base_return": "导航返回抓取位",
    "transport_diagnostic": "失败轨迹诊断",
}


def main(planner_class=FullCyclePlanner, default_port=8120, round_mode=False):
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-config", type=Path, default=DEFAULT_ROBOT_CONFIG)
    parser.add_argument("--mobile-robot-config", type=Path, default=DEFAULT_MOBILE_ROBOT_CONFIG)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--named-poses", type=Path, default=DEFAULT_NAMED_POSES)
    parser.add_argument("--box-fit", type=Path, default=DEFAULT_BOX_FIT)
    parser.add_argument("--port", type=int, default=default_port)
    parser.add_argument("--comparison-json", type=Path, nargs="+")
    parser.add_argument('--result', type=Path, nargs='+')
    args = parser.parse_args()
    comparisons = {}
    for path in args.comparison_json or []:
        comparisons.update(json.loads(path.read_text()))
    planner_labels = {"Connect + Informed": "informed_connect"}
    planner = planner_class(args)
    saved_results = [json.loads(path.read_text()) for path in args.result or []]
    preload = saved_results[0] if saved_results else None
    saved_tasks = {tuple(sorted(result['task'].items())):result for result in saved_results}
    if preload:
        planner.selected_task = preload['task']
    if round_mode and saved_results:
        planner.task_options = [
            (f"第 {index+1:02d} 轮 · " + " / ".join(f"{side}:{box}" for side, box in result["task"].items()),
             result["task"])
            for index, result in enumerate(saved_results)]
        planner.task_map = dict(planner.task_options)
    config = planner.mobile_robot["kinematics"]
    joint_names = config["cspace"]["joint_names"]
    urdf = yourdfpy.URDF.load(config["urdf_path"], load_meshes=True, build_scene_graph=True)
    server = viser.ViserServer(host="127.0.0.1", port=args.port,
                               label="V3初版94组任务 · 抓取抽离放置回Home")
    server.scene.set_up_direction("+z")
    server.scene.add_grid("/ground", width=5, height=3)
    robot = ViserUrdf(server, urdf, root_node_name="/robot",
                      mesh_color_override=(0.68, 0.73, 0.80, 0.8))
    joint_models = {joint.name: joint for joint in urdf.robot.joints if joint.type != "fixed"}
    velocity_arrows = server.scene.add_arrows(
        "/trajectory/joint_velocity", points=np.zeros((len(joint_names), 2, 3), dtype=np.float32),
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
    fill_nodes = [server.scene.add_box('/task_fill/' + str(index), dimensions=(0.3,0.1,0.1),
                  color=(235,65,85), opacity=0.35, visible=False) for index in range(4)]
    with server.gui.add_folder("完整25箱序列回放" if round_mode else "完整任务实时规划"):
        initial_label = next(label for label, boxes in planner.task_options if boxes == planner.selected_task)
        task = server.gui.add_dropdown("查看轮次" if round_mode else "组合任务", options=labels, initial_value=initial_label)
        planner_selector = server.gui.add_dropdown("规划器", options=list(planner_labels),
                                                   initial_value="Connect + Informed")
        run = server.gui.add_button("计算完整流程并播放")
        play = server.gui.add_checkbox("播放", initial_value=False)
        playback_speed = server.gui.add_slider("真实时间回放倍率", min=.1, max=3., step=.1,
                                               initial_value=1.)
        frame = server.gui.add_slider("轨迹帧", min=1, max=1, step=1, initial_value=1)
        stage = server.gui.add_dropdown("查看阶段", options=list(PHASE_NAMES.values()),
                                        initial_value=PHASE_NAMES["home"])
        spheres = server.gui.add_checkbox("显示附着箱40球", initial_value=False)
        diagnostic = server.gui.add_checkbox('显示接近/抽离失败候选（仅诊断）', initial_value=False)
        status = server.gui.add_markdown("从第一初始姿态开始，一次计算完整流程。")
        timed_status = server.gui.add_markdown("当前结果没有时间化轨迹。")
        with server.gui.add_folder("各阶段规划 / 运动耗时", expand_by_default=True):
            timing_stage = server.gui.add_dropdown(
                "耗时阶段", options=tuple(PHASE_LABELS.values()),
                initial_value=PHASE_LABELS["approach"])
            timing_detail = server.gui.add_markdown("请选择轮次和阶段。")
        with server.gui.add_folder("q / qdot 轨迹", expand_by_default=False):
            show_velocity = server.gui.add_checkbox("显示关节速度轴矢量", initial_value=True)
            curve_joint = server.gui.add_dropdown("曲线关节", options=tuple(joint_names),
                                                  initial_value="left_joint1")
            trajectory_curve = server.gui.add_uplot(
                data=(np.array([0., 1.]), np.zeros(2), np.zeros(2)),
                series=(viser.uplot.Series(label="time"), viser.uplot.Series(label="q", stroke="#3daad2"),
                        viser.uplot.Series(label="qdot", stroke="#ef923e")),
                mode=1, scales={"x": {"time": False}}, title="所选阶段 q / qdot", height=300,
                visible=False)
        comparison_status = server.gui.add_markdown("")
    current = {"result": None, "busy": False, "curve_binding": None}

    def refresh_stage_timing():
        result = current.get("result")
        if not result:
            timing_detail.content = "请选择轮次和阶段。"
            return
        phase = next(key for key, label in PHASE_LABELS.items() if label == timing_stage.value)
        segment = next((item for item in result.get("timed_segments", ()) if item["name"] == phase), None)
        if segment is None:
            timing_detail.content = "当前轮次没有该运动阶段。"
            return
        detail = phase_planning_detail(result, phase)
        additive = [(name, value) for name, value in detail["additive"] if value >= 10.]
        hidden = sum(value < 10. for _, value in detail["additive"])
        total = sum(value for _, value in detail["additive"])
        rows = ["| 可相加规划子阶段（≥10ms） | 耗时 |", "|---|---:|"]
        rows += [f"| {name} | {value:.2f} ms |" for name, value in additive]
        if hidden:
            rows.append(f"\n总计另含 **{hidden}** 项小于10ms的记录。")
        if detail["diagnostics"]:
            rows += ["", "**内部诊断（已包含在上表，不重复相加）**", "",
                     "| 内部记录 | 耗时 |", "|---|---:|"]
            rows += [f"| {name} | {value:.2f} ms |" for name, value in detail["diagnostics"]
                     if value >= 10.]
        timing_detail.content = (
            f"## {PHASE_LABELS[phase]}  \n"
            f"**运动时间：{segment['duration_s']:.3f}s** · "
            f"**阶段规划归属时间：{total/1000:.3f}s** · "
            f"本轮规划：{result.get('total_ms', 0)/1000:.3f}s  \n"
            + "\n".join(rows))


    def refresh_comparison():
        lines = ["已计算完整流程对比（计算结果回放）", "", "| 规划器 | 结果 | 总计算 |",
                 "| --- | --- | --- |"]
        for label, kind in planner_labels.items():
            result = comparisons.get(kind)
            if result is None:
                continue
            verdict = "完成全部阶段" if result["success"] else result.get("blocked_stage", "失败")
            seconds = result.get("measured_wall_ms", result.get("total_ms", 0)) / 1000
            lines.append(f"| {label} | {verdict} | {seconds:.2f}s |")
        lines += ["", "当前结果仅代表所选任务；94组选项不等于94组全部成功。搜索每段2秒上限，首个合法路径早停；无Shortcut/TrajOpt。"]
        comparison_status.content = "\n".join(lines) if comparisons else ""

    def show_frame(index):
        result = current["result"]
        if result and diagnostic.value and result.get('extraction_diagnostic'):
            result = dict(result, **result['extraction_diagnostic'])
        if result is None:
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
        for side, node in attached.items():
            matrix = urdf.get_transform(side+"_tool0", urdf.base_link) @ grasp_transform(side, planner.top_grasp)
            node.position = matrix[:3, 3]
            node.wxyz = quaternion(matrix)
            node.visible = has_payload
        for box_id, box in source_boxes.items():
            box.visible = box_id in planner.selected_task.values() and phase in ("home", "approach", 'contact_approach', "base_advance", "base_advance_diagnostic")
        if result:
            status.content = (
                f"**{'完整流程成功' if result['success'] else '已停止：存在阻塞'} · {PHASE_NAMES[phase]}**  \n"
                f"帧：**{index+1}/{len(result['frames'])}**  \n"
                f"接触IK：**{result.get('contact_ik_count', 0)}** · 选中候选：**{result.get('selected_candidate', '无')}**  \n"
                f"全部规划：**{result.get('total_ms', 0)/1000:.3f}s**  \n"
                f"最终Home误差：**{result.get('final_home_error', 0):.8f}**"
            )
            segment = next((item for item in result.get("timed_segments", ())
                            if item["frame_start"] <= index <= item["frame_end"]), None)
            if segment:
                local = index - segment["frame_start"]
                related = phase_planning_detail(result, segment["name"])["additive"]
                detail = "".join(f"  \n- {name}：{value:.2f}ms" for name, value in related)
                timed_status.content = (
                    f"**阶段运动：{segment['duration_s']:.3f}s** · 当前：{segment['time_from_start_s'][local]:.3f}s  \n"
                    f"时间化：`{segment.get('parameterizer', 'recorded')}` / {result.get('trajectory_timing_ms', 0):.1f}ms  \n"
                    f"本轮规划：**{result.get('total_ms', 0)/1000:.3f}s** · 本轮运动：**{result.get('motion_duration_s', 0):.3f}s**"
                    + ("  \n**相关规划记录（共享项不重复求和）**" + detail if detail else ""))
                names = segment["joint_names"]
                joint_index = names.index(curve_joint.value)
                binding = (id(result), segment["name"], curve_joint.value)
                if current["curve_binding"] != binding:
                    trajectory_curve.data = (np.asarray(segment["time_from_start_s"]),
                                             np.asarray(segment["positions"])[:, joint_index],
                                             np.asarray(segment["velocities"])[:, joint_index])
                    trajectory_curve.visible = True
                    current["curve_binding"] = binding
                velocity = dict(zip(names, segment["velocities"][local]))
                points = np.zeros((len(joint_names), 2, 3), dtype=np.float32)
                for joint_index, name in enumerate(joint_names):
                    joint = joint_models[name]
                    transform = urdf.get_transform(joint.child, urdf.base_link)
                    axis = transform[:3, :3] @ np.asarray(joint.axis)
                    limit = joint.limit.velocity if joint.limit and joint.limit.velocity else 1.0
                    points[joint_index, 0] = transform[:3, 3]
                    points[joint_index, 1] = transform[:3, 3] + axis * np.clip(velocity[name] / limit, -1., 1.) * .15
                velocity_arrows.points = points
                velocity_arrows.visible = show_velocity.value
            else:
                timed_status.content = "当前为吸附、释放或导航状态切换，不计机械臂运动时间。"
                trajectory_curve.visible = False
                velocity_arrows.visible = False
                current["curve_binding"] = None
            if not result["success"]:
                status.content += f"  \n{result.get('blocked_stage')}: {result.get('blocked_details')}"
                failures = [attempt.get('approach',{}) for attempt in result.get('attempts',[]) if attempt.get('approach',{}).get('failure')]
                if failures:
                    counts = {}
                    for failure in failures:
                        counts[failure['failure']] = counts.get(failure['failure'],0) + 1
                    status.content += '\n\n接近拒绝：' + json.dumps(counts,ensure_ascii=False)
            if result.get('audit'):
                audit = result['audit']
                pairs = audit.get('self_collisions', [])
                penetration = max((sphere['penetration_mm'] for sphere in audit.get('ground_spheres', [])), default=0)
                status.content += f'\n\n**失败候选诊断，不能执行**：{len(pairs)}组自碰撞；地面最大穿透{penetration:.8f}mm。'
                if result.get('reason'):
                    status.content += '\n\n' + result['reason']
                if 'contact_sphere_indices' in audit:
                    status.content += '\n\n满足吸盘接触几何条件的球：' + json.dumps(audit['contact_sphere_indices'])
                for pair in pairs:
                    status.content += '\n\n' + ' ↔ '.join(pair['links']) + f"：{pair['penetration_mm']:.2f}mm"

    @timing_stage.on_update
    def on_timing_stage(_event):
        refresh_stage_timing()

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

    @stage.on_update
    def on_stage(_event):
        if not current["result"]:
            return
        selected = next(key for key, label in PHASE_NAMES.items() if label == stage.value)
        phases = displayed_result()['phases']
        if selected in phases:
            play.value = False
            frame.value = phases.index(selected)+1
            if selected in PHASE_LABELS:
                timing_stage.value = PHASE_LABELS[selected]
                refresh_stage_timing()

    @spheres.on_update
    def on_spheres(_event):
        for side in ("left", "right"):
            server.scene.add_frame(f"/attached/{side}/spheres", show_axes=False,
                                    visible=spheres.value)

    def displayed_result():
        result = current['result']
        return dict(result, **result['extraction_diagnostic']) if diagnostic.value and result and result.get('extraction_diagnostic') else result

    @diagnostic.on_update
    def diagnostic_changed(_event):
        play.value = False
        result = displayed_result()
        frame.max = max(1, len(result['frames'])) if result else 1
        frame.value = 1
        show_frame(0)

    @task.on_update
    def on_task(_event):
        if current["busy"]:
            return
        play.value = False
        diagnostic.value = False
        current["result"] = None
        planner.selected_task = planner.task_map[task.value]
        refresh_task_geometry()
        comparisons.clear()
        planner.comparison_contact = None
        refresh_comparison()
        current['result'] = saved_tasks.get(tuple(sorted(planner.selected_task.items())))
        frame.max = max(1,len(current['result']['frames'])) if current['result'] else 1
        frame.value = 1
        show_frame(0)
        refresh_stage_timing()

    def refresh_task_geometry():
        centers, obstacles = planner.configure_task(planner.selected_task)
        for side, box_id in planner.selected_task.items():
            source_boxes[box_id].position = centers[side]
        for index, node in enumerate(fill_nodes):
            node.visible = index < len(obstacles)
            if index < len(obstacles):
                node.position = obstacles[index]['center']
                node.dimensions = obstacles[index]['dimensions']
        for side in ['left', 'right']:
            for sphere_index, (center, radius) in enumerate(zip(planner.box_fit['centers'], planner.box_fit['radii'])):
                server.scene.add_mesh_simple(f'/attached/{side}/spheres/{sphere_index}',
                    vertices=sphere_mesh.vertices*radius, faces=sphere_mesh.faces, position=center,
                    color=(61,170,210), opacity=0.5)
        status.content = ('顶吸：底盘前进0.3m，放置目标一起前移0.3m。' if planner.top_grasp else '侧吸任务。') + '到3cm预抓取前检查目标；最后3cm排除两个目标碰撞；吸附后启用附着箱模型。最低排及以下填满；后撤35cm；仅Informed＋Connect。'

    @planner_selector.on_update
    def on_planner(_event):
        if current["busy"]:
            return
        play.value = False
        planner.planner_kind = planner_labels[planner_selector.value]
        result = comparisons.get(planner.planner_kind)
        current["result"] = result
        frame.max = max(1, len(result["frames"])) if result else 1
        frame.value = 1
        show_frame(0)
        if result:
            play.value = result["success"]

    @run.on_click
    def on_run(_event):
        if current["busy"]:
            return
        current["busy"] = True
        run.disabled = True
        task.disabled = True
        planner_selector.disabled = True
        play.value = False
        current["result"] = None
        show_frame(0)

        def worker():
            try:
                result = planner.full_cycle(planner.selected_task,
                                            lambda text: setattr(status, "content", text))
            except CycleBlocked as error:
                result = error.partial or planner.last_partial
                result["blocked_stage"] = error.stage
                result["blocked_details"] = error.details
            except Exception as error:
                result = planner.last_partial or {"success": False, "frames": []}
                result["blocked_stage"] = "程序异常"
                result["blocked_details"] = f"{type(error).__name__}: {error}"
            finally:
                current["busy"] = False
                run.disabled = False
                task.disabled = False
                planner_selector.disabled = False
            path = Path(__file__).resolve().parents[1] / 'generated/task_dynamic_cycle/latest.json'
            path.write_text(json.dumps(result, indent=2, ensure_ascii=False)+"\n")
            current["result"] = result
            comparisons[planner.planner_kind] = result
            refresh_comparison()
            if result["frames"]:
                frame.max = len(result["frames"])
                frame.value = 1
                show_frame(0)
                play.value = result["success"]
            else:
                status.content = f"**已停止** {result.get('blocked_stage')}: {result.get('blocked_details')}"
        threading.Thread(target=worker, daemon=True).start()

    @server.on_client_connect
    def on_client(client):
        client.camera.position = (-3.2, 3.4, 2.8)
        client.camera.look_at = (.6, 0, 1.2)
        client.camera.up_direction = (0, 0, 1)

    refresh_task_geometry()
    if preload:
        current['result'] = preload
        frame.max = max(1, len(preload['frames']))
        play.value = False
    if comparisons:
        baseline = comparisons.get("informed_rrt")
        if baseline and baseline.get("selected_candidate"):
            planner.comparison_contact = baseline["contact_candidates"][baseline["selected_candidate"]-1]
        current["result"] = baseline
        frame.max = len(baseline["frames"]) if baseline else 1
    refresh_comparison()
    show_frame(0)
    refresh_stage_timing()
    print(f"Ready: http://localhost:{server.get_port()}", flush=True)
    while True:
        if play.value and current["result"]:
            if int(frame.value) >= len(displayed_result()["frames"]):
                play.value = False
            else:
                frame.value = int(frame.value)+1
        time.sleep(.025 / playback_speed.value if play.value else .025)


if __name__ == "__main__":
    main()
