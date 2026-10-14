import argparse
import json
from pathlib import Path
import threading
import time
import sys

import numpy as np
import trimesh
import viser
from viser.extras import ViserUrdf
import yourdfpy

from v3_conveyor_cycle_planner import ConveyorCyclePlanner as FullCyclePlanner
from curobo_core.planner import CycleBlocked
from v3_task_scene import row_obstacles, grasp_transform
from v3_interactive_60_tasks import (
    DEFAULT_ROBOT_CONFIG, DEFAULT_URDF, DEFAULT_NAMED_POSES, DEFAULT_BOX_FIT,
    DEFAULT_MOBILE_ROBOT_CONFIG, quaternion,
)
from v3_wall_ik_benchmark import canonical_side_tool_to_box, wall_center
from v3_playback import playback_frame_interval


PHASE_NAMES = {
    'contact_approach': '最后3cm贴近（排除目标箱碰撞）',
    "base_advance": "底盘前进0.3m", "base_advance_diagnostic": "底盘前进失败诊断",
    "home": "第一初始姿态", "approach": "初始→吸附起点", "attach": "吸附箱体",
    "extract": "固定Updown解析直线抽离35cm", "transport": "抽离终点→放置姿态",
    "turn_loaded": "携箱转身180°", "conveyor_place": "底盘冻结、cuRobo目标IK＋RRT放箱（含龙头车碰撞）",
    "base_reposition": "等待导航完成：瞬移到放置底盘位",
    "base_return": "回Home后等待导航完成：瞬移回抓取位",
    "base_pick_reposition": "等待导航完成：瞬移到顶吸抓取底盘位",
    "conveyor_retreat": "释放后沿吸盘退离5cm", "release": "台面释放，箱体移出场景",
    "turn_empty": "空载转身回来", "return_home": "双臂回到原第一初始姿态",
    "transport_diagnostic": "失败轨迹诊断",
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-config", type=Path, default=DEFAULT_ROBOT_CONFIG)
    parser.add_argument("--mobile-robot-config", type=Path, default=DEFAULT_MOBILE_ROBOT_CONFIG)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--named-poses", type=Path, default=DEFAULT_NAMED_POSES)
    parser.add_argument("--box-fit", type=Path, default=DEFAULT_BOX_FIT)
    parser.add_argument("--port", type=int, default=8150)
    parser.add_argument("--comparison-json", type=Path, nargs="+")
    parser.add_argument('--result', type=Path, nargs='+')
    parser.add_argument('--frontend-root', type=Path)
    parser.add_argument('--asset-root', type=Path)
    parser.add_argument('--floor-asset', type=Path)
    parser.add_argument('--marker-asset', type=Path)
    args = parser.parse_args()
    if args.frontend_root:
        sys.path.insert(0, str(args.frontend_root))
    from alfa_motion_viser import ReadOnlyScene, VisualAssets, default_visual_assets
    from v3_visualization_dependency import verify_frontend_package
    frontend_identity = verify_frontend_package()
    comparisons = {}
    for path in args.comparison_json or []:
        comparisons.update(json.loads(path.read_text()))
    planner_labels = {"多抽离根＋多目标Connect": "informed_connect"}
    planner = FullCyclePlanner(args)
    planner.selected_task={'left':22,'right':17}
    saved_results = [json.loads(path.read_text()) for path in args.result or []]
    preload = saved_results[0] if saved_results else None
    saved_tasks = {tuple(sorted(result['task'].items())):result for result in saved_results}
    if preload:
        planner.selected_task = preload['task']
    config = planner.mobile_robot["kinematics"]
    joint_names = config["cspace"]["joint_names"]
    server = viser.ViserServer(host="127.0.0.1", port=args.port,
                               label="V3解析代理 · 抓取抽离放置回Home完整流程")
    assets = default_visual_assets()
    if args.asset_root:
        assets = VisualAssets(args.asset_root / 'alfa_robot_material_study/reference_colors_v2',
                             args.floor_asset or assets.floor,
                             args.asset_root / 'cardboard_box_40x40x30/cardboard_box_40x40x30.glb',
                             args.asset_root / 'open_container_scene/open_container_scene.glb',
                             args.asset_root / 'roller_conveyor_collision/roller_conveyor_visual_v3_symmetric.glb',
                             args.marker_asset or assets.marker)
    else:
        from dataclasses import replace
        assets = replace(assets, floor=args.floor_asset or assets.floor, marker=args.marker_asset or assets.marker)
    renderer = ReadOnlyScene(server, planner.args.urdf, assets)
    server.gui.add_markdown(f'前端冻结版本：{frontend_identity["commit"][:12]}；碰撞与轨迹由当前核心提供。')
    references = renderer.reference_conveyors()
    references.visible = True
    reference_control = server.gui.add_checkbox('显示双龙头车与光圈（STL碰撞已启用）', initial_value=True)
    @reference_control.on_update
    def show_references(_event):
        renderer.submit(lambda: setattr(references, 'visible', reference_control.value and not renderer.collision_view))
    server.gui.add_markdown(f'只读显示：规划模型与关节来自核心，匹配{renderer.material_matches}个彩色网格。完整集装箱统一装配到快照内壁边界；外观框架不额外参与碰撞。仅最里面一面箱墙，非475箱展示模板。\n\n播放帧率按最快旋转关节30°/s统一换算，Updown等比播放；底盘到位仍为瞬移。未加入加速度或动力学时间参数化。')
    attached = {}
    sphere_mesh = trimesh.creation.icosphere(subdivisions=2)
    for side in ("left", "right"):
        attached[side] = server.scene.add_frame(f"/attached/{side}", show_axes=False, visible=False)
        renderer.carton_object(f"/attached/{side}/box", planner.box_fit["dimensions_m"])
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
        source_boxes[box_id] = renderer.snapshot_object(item)
    renderer.assembled_container(planner.snapshot.objects)
    for item in planner.snapshot.objects:
        if not item.object_id.startswith('wall_box_'):
            renderer.snapshot_object(item)
    for item in planner.snapshot.meshes:
        renderer.snapshot_mesh(item)
    cutaway = server.gui.add_checkbox('剖视集装箱（仅隐藏外观，碰撞不变）', initial_value=False)
    view_button = server.gui.add_button('切换到原碰撞场景')
    scene_view = {'collision': False}
    def refresh_scene_view():
        renderer.submit(lambda: renderer.set_scene_view(scene_view['collision'], cutaway.value))
        references.visible = reference_control.value and not scene_view['collision']
    @view_button.on_click
    def switch_scene_view(_event):
        scene_view['collision'] = not scene_view['collision']
        view_button.label = '切换到真实外观' if scene_view['collision'] else '切换到原碰撞场景'
        refresh_scene_view()
    @cutaway.on_update
    def show_container(_event):
        refresh_scene_view()
    labels = [entry[0] for entry in planner.task_options]
    fill_nodes = [server.scene.add_box('/task_fill/' + str(index), dimensions=(0.3,0.1,0.1),
                  color=(235,65,85), opacity=0.35, visible=False) for index in range(4)]
    with server.gui.add_folder("完整任务实时规划"):
        initial_label = next(label for label, boxes in planner.task_options if boxes == planner.selected_task)
        task = server.gui.add_dropdown("组合任务", options=labels, initial_value=initial_label)
        planner_selector = server.gui.add_dropdown("规划器", options=list(planner_labels),
                                                   initial_value="多抽离根＋多目标Connect")
        run = server.gui.add_button("计算完整流程并播放")
        play = server.gui.add_checkbox("播放", initial_value=False)
        playback_status = server.gui.add_markdown('播放速度：旋转关节最快约30°/s。')
        frame = server.gui.add_slider("轨迹帧", min=1, max=1, step=1, initial_value=1)
        stage = server.gui.add_dropdown("查看阶段", options=[label for key, label in PHASE_NAMES.items() if key != 'conveyor_retreat'],
                                        initial_value=PHASE_NAMES["home"])
        spheres = server.gui.add_checkbox("显示附着箱40球", initial_value=False)
        diagnostic = server.gui.add_checkbox('显示接近/抽离失败候选（仅诊断）', initial_value=False)
        collision_edge = server.gui.add_checkbox('显示被拒绝的碰撞连接（仅诊断，不能执行）', initial_value=False)
        first_collision = server.gui.add_button('定位首个碰撞帧')
        status = server.gui.add_markdown("从第一初始姿态开始，一次计算完整流程。")
        comparison_status = server.gui.add_markdown("")
    current = {"result": None, "busy": False}
    collision_overlay = server.scene.add_frame('/collision_diagnostic', show_axes=False, visible=False)
    collision_nodes = []
    server.gui.add_markdown('实验旋转验收：每箱额外旋转≤90°、回退指标≤15°、弧偏离≤45°。接触IK候选全部保留抽离验证，再验证被选候选的前后流程。')

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
        lines += ["", "94组可选。各任务首次生成当前场景无碰撞终点缓存，后续复用；多根Connect搬运搜索累计2秒上限。含双龙头车STL碰撞，无Shortcut/TrajOpt。释放后直接RRT回Home。"]
        comparison_status.content = "\n".join(lines) if comparisons else ""

    def show_frame(index):
        result = displayed_result()
        if result is None or not result.get('frames'):
            values = {**planner.home, "base_x": 0., "base_y": 0., "base_yaw": 0.}
            phase = "home"
            has_payload = False
        else:
            index = min(max(0, index), len(result['frames']) - 1)
            values = dict(zip(result["joint_names"], result["frames"][index]))
            phase = result["phases"][index]
            has_payload = result["payload"][index]
        def render_state(values=dict(values), phase=phase, has_payload=has_payload, top_grasp=planner.top_grasp, boxes=dict(planner.selected_task)):
            state = {**planner.home, **values}
            renderer.apply_state(state)
            for side, node in attached.items():
                matrix = renderer.tool_poses[side] @ grasp_transform(side, top_grasp)
                node.position = matrix[:3, 3]
                node.wxyz = quaternion(matrix)
                node.visible = has_payload
            targets = result.get('conveyor_box_targets', {}) if result else {}
            renderer.update_conveyor_targets(targets or planner.conveyor_reference_targets())
            for box_id, box in source_boxes.items():
                box.visible = box_id in boxes.values() and phase in ('home', 'approach', 'contact_approach', 'base_advance', 'base_advance_diagnostic')
            collision_overlay.visible = False
            edge = result.get('conveyor_edge_diagnostic') if result else None
            if collision_edge.value and edge and index == edge['first_bad_index']:
                for node in collision_nodes:
                    node.remove()
                collision_nodes.clear()
                collision_overlay.visible = True
                for sphere_index in edge['highlighted_sphere_indices']:
                    center = edge['spheres_map'][sphere_index]
                    collision_nodes.append(server.scene.add_icosphere(f'/collision_diagnostic/sphere_{sphere_index}',
                        radius=center[3], position=center[:3], color=(255, 30, 30), opacity=.9))
                    collision_nodes.append(server.scene.add_label(f'/collision_diagnostic/label_{sphere_index}',
                        text=f"{edge.get('sphere_roles', edge['sphere_links'])[sphere_index]} · 碰撞球{sphere_index}", position=center[:3]))
                obstacles = {item['obstacle'] for item in edge['audit']['world_collisions'] + edge['audit']['base_internal_collisions']}
                for obstacle_index, obstacle in enumerate(edge['obstacles_map']):
                    if obstacle['name'] in obstacles:
                        collision_nodes.append(server.scene.add_box(f'/collision_diagnostic/obstacle_{obstacle_index}',
                            dimensions=obstacle['dimensions'], position=obstacle['position'], wxyz=obstacle['quaternion_wxyz'],
                            color=(255, 30, 30), opacity=.18))
        renderer.submit(render_state)
        if result:
            status.content = (
                f"**{'完整流程成功' if result['success'] else '已停止：存在阻塞'} · {PHASE_NAMES[phase]}**  \n"
                f"帧：**{index+1}/{len(result['frames'])}**  \n"
                f"接触IK：**{result.get('contact_ik_count', 0)}** · 选中候选：**{result.get('selected_candidate', '无')}**  \n"
                f"全部规划：**{result.get('total_ms', 0)/1000:.3f}s**  \n"
                f"最终Home误差：**{format(result['final_home_error'], '.8f') if 'final_home_error' in result else '尚未回Home'}**"
            )
            if not result["success"]:
                status.content += f"  \n{result.get('blocked_stage')}: {result.get('blocked_details')}"
            if collision_edge.value and result.get('conveyor_edge_diagnostic'):
                edge = result['conveyor_edge_diagnostic']
                status.content += f"\n\n**被拒绝的连接，仅诊断，不能执行**。当前帧{'通过' if edge['valid'][index] else '未通过'}检查；首个碰撞在本连接第{edge['first_bad_index']+1}帧。红球和红色障碍标记首个碰撞帧。"
                for item in edge['audit']['world_collisions'] + edge['audit']['base_internal_collisions']:
                    role = edge.get('sphere_roles', edge['sphere_links'])[item['sphere_index']]
                    status.content += f"\n\n{role}（{item['link']}） ↔ {item['obstacle']}：穿入{item['penetration_mm']:.2f}mm。"
                for item in edge['audit']['self_collisions']:
                    status.content += '\n\n' + ' ↔ '.join(item['links']) + f"：穿入{item['penetration_mm']:.2f}mm。"
            if result.get('transport_rotation_metrics'):
                for side,metrics in result['transport_rotation_metrics'].items():
                    status.content+=f"  \n{side}箱累计旋转{metrics['total_rotation_deg']:.1f}°，回退{metrics['backtracking_deg']:.1f}°，偏离弧最大{metrics['max_deviation_deg']:.1f}°"
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

    @frame.on_update
    def on_frame(_event):
        if current["result"] and current['result'].get('frames'):
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

    @spheres.on_update
    def on_spheres(_event):
        for side in ("left", "right"):
            server.scene.add_frame(f"/attached/{side}/spheres", show_axes=False,
                                    visible=spheres.value)

    def displayed_result():
        result = current['result']
        if collision_edge.value and result and result.get('conveyor_edge_diagnostic'):
            edge = result['conveyor_edge_diagnostic']
            return dict(result, frames=edge['frames'], phases=['conveyor_place'] * len(edge['frames']), payload=[True] * len(edge['frames']))
        return dict(result, **result['extraction_diagnostic']) if diagnostic.value and result and result.get('extraction_diagnostic') else result

    @collision_edge.on_update
    def collision_edge_changed(_event):
        play.value = False
        result = displayed_result()
        frame.max = max(1, len(result['frames'])) if result else 1
        edge = result.get('conveyor_edge_diagnostic') if result else None
        frame.value = edge['first_bad_index'] + 1 if collision_edge.value and edge else 1
        show_frame(int(frame.value) - 1)

    @first_collision.on_click
    def locate_collision(_event):
        if current['result'] and current['result'].get('conveyor_edge_diagnostic'):
            collision_edge.value = True
            collision_edge_changed(_event)

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
        collision_edge.value = False
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
        status.content = ('顶吸：搬运目标TCP朝下z=1m；抓取底盘到位偏移0.3m。' if planner.top_grasp else '侧吸：搬运目标箱心z=1.2m、Y倾斜45°。') + '接触64候选，解析抽离35cm；随后导航到位瞬移，固定底盘用目标IK＋RRT放箱。释放后箱子消失，直接RRT回Home，再导航返回。放箱箱心(-1.45,±0.425,1.182)m，含龙头车STL碰撞。'

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
        collision_edge.value = False
        diagnostic.value = False
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
            result.setdefault('frames', [])
            result.setdefault('phases', [])
            result.setdefault('payload', [])
            result.setdefault('joint_names', joint_names)
            path = Path(__file__).resolve().parents[1] / 'generated/visualized_task_live/latest.json'
            path.parent.mkdir(parents=True,exist_ok=True)
            archive=path.parent/'runs'
            archive.mkdir(exist_ok=True)
            (archive/(str(time.time_ns())+'.json')).write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
            path.write_text(json.dumps(result, indent=2, ensure_ascii=False)+"\n")
            current["result"] = result
            comparisons[planner.planner_kind] = result
            refresh_comparison()
            if result["frames"]:
                frame.max = len(displayed_result()['frames'])
                frame.value = 1
                show_frame(0)
                play.value = result["success"]
            else:
                status.content = f"**已停止** {result.get('blocked_stage')}: {result.get('blocked_details')}"
            current['busy'] = False
            run.disabled = False
            task.disabled = False
            planner_selector.disabled = False
        threading.Thread(target=worker, daemon=True).start()

    @server.on_client_connect
    def on_client(client):
        client.camera.position = (-6.0, 4.0, 3.6)
        client.camera.look_at = (-.4, 0, 1.0)
        client.camera.up_direction = (0, 0, 1)

    refresh_task_geometry()
    if preload:
        current['result'] = preload
        frame.max = max(1, len(preload['frames']))
        play.value = False
    if comparisons:
        baseline = comparisons.get(planner.planner_kind)
        if baseline and baseline.get("selected_candidate"):
            planner.comparison_contact = baseline["contact_candidates"][baseline["selected_candidate"]-1]
        if baseline is not None:
            current["result"] = baseline
            frame.max = max(1, len(baseline["frames"]))
    refresh_comparison()
    if preload and preload.get('conveyor_edge_diagnostic'):
        cutaway.value = True
        collision_edge.value = True
        collision_edge_changed(None)
    else:
        show_frame(0)
    print(f"Ready: http://localhost:{server.get_port()}", flush=True)
    timed_frames = None
    playback_interval = .03
    observed_frame = None
    next_frame_time = 0.
    while True:
        renderer.drain()
        if play.value and current["result"]:
            result = displayed_result()
            if timed_frames is not result['frames']:
                timed_frames = result['frames']
                playback_interval = playback_frame_interval(result)
                playback_status.content = f'播放：约{1/playback_interval:.1f}帧/s；旋转关节最快30°/s，Updown同比例。'
            position = (id(timed_frames), int(frame.value))
            now = time.perf_counter()
            if position != observed_frame:
                observed_frame = position
                next_frame_time = now + playback_interval
            if int(frame.value) >= len(result["frames"]):
                play.value = False
            elif now >= next_frame_time:
                frame.value = int(frame.value)+1
                observed_frame = (id(timed_frames), int(frame.value))
                next_frame_time = time.perf_counter() + playback_interval
        else:
            observed_frame = None
        delay = min(.005, max(0., next_frame_time - time.perf_counter())) if play.value else .005
        time.sleep(delay)


if __name__ == "__main__":
    main()
