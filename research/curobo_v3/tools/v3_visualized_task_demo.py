import argparse
import json
from pathlib import Path
import threading
import time
import sys
from dataclasses import asdict

import numpy as np
import trimesh
import viser
from viser.extras import ViserUrdf
import yourdfpy

from v3_adaptive_raised_cycle_planner import AdaptiveRaisedCyclePlanner as FullCyclePlanner
from curobo_core.planner import CycleBlocked
from v3_task_scene import row_obstacles, grasp_transform
from v3_interactive_60_tasks import (
    DEFAULT_ROBOT_CONFIG, DEFAULT_URDF, DEFAULT_NAMED_POSES, DEFAULT_BOX_FIT,
    DEFAULT_MOBILE_ROBOT_CONFIG, quaternion,
)
from v3_wall_ik_benchmark import canonical_side_tool_to_box, wall_center
from v3_playback import playback_frame_interval
from v3_stage_playback import PLAYBACK_SCOPES, StagePlayback, TcpTrajectoryPreview


PHASE_NAMES = {
    'extraction_teleport': '抽离瞬移35cm（未生成/未验收抽离轨迹）',
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


def main(default_port=8150, planner_class=None, default_assets=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-config", type=Path, default=DEFAULT_ROBOT_CONFIG)
    parser.add_argument("--mobile-robot-config", type=Path, default=DEFAULT_MOBILE_ROBOT_CONFIG)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--named-poses", type=Path, default=DEFAULT_NAMED_POSES)
    parser.add_argument("--box-fit", type=Path, default=DEFAULT_BOX_FIT)
    parser.add_argument("--port", type=int, default=default_port)
    parser.add_argument("--comparison-json", type=Path, nargs="+")
    parser.add_argument('--result', type=Path, nargs='+')
    parser.add_argument('--frontend-root', type=Path)
    parser.add_argument('--asset-root', type=Path)
    parser.add_argument('--floor-asset', type=Path)
    parser.add_argument('--marker-asset', type=Path)
    if default_assets is not None:
        parser.set_defaults(**{name: getattr(default_assets, name) for name in
                              ('robot_config', 'mobile_robot_config', 'urdf', 'named_poses', 'box_fit')})
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
    planner = (planner_class or FullCyclePlanner)(args)
    planner.warmup_resources()
    server_policy = '预测64个抽离终点→距离/旋转排序→8个起分批完整抽离验收；失败扩批，运输搜索共享2s预算。每次点击重新生成候选，不复用规划结果。'
    serial_policy=getattr(planner,'serial_endpoint_policy',False)
    if serial_policy:
        server_policy='前驱构型独立串行选型，先固定终点再连接；吸附只参考Home，运输只参考抽离终点，放箱只参考运输构型。不回溯，不用后续可行性反选。抽离只瞬移，非已验证动作。'
    chained_policy=getattr(planner,'lm_chained_cycle_policy',False)
    if chained_policy:
        planner_labels={'六段LM先验＋Informed Connect':'informed_connect'}
        server_policy='六段LM先验＋Informed Connect实际串联：新接近/抽离构型交给前后阶段重规划，接缝检查关节连续性。不回溯、不因偏线或沿途朝向变化拒绝。抽离有运动；仍沿用途中空载验碰、终点携箱验碰。'
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
    server.gui.add_markdown(server_policy)
    if chained_policy:
        server.gui.add_markdown('内部底盘粗搜索余量2mm，首条原模型细验合法路径即停，每段搜索上限2秒；不强制走满预算。顶吸收束TCP高1.2m，放箱箱心高1.082m。')
    else:
        server.gui.add_markdown('首轮粗验统一约2.86°；仅放箱候选细验碰撞后，按剩余2秒预算以约1.72°粗验重搜。保留3mm搜索余量与原模型0.5°最终细验。顶吸收束TCP高1.2m（提高20cm），放箱箱心高1.082m（降低10cm）。')
    if getattr(planner, 'warmup_report', {}).get('complete'):
        server.gui.add_markdown('当前确认零位与191mm TCP；16点／64点批量LM已完成真实预热。运输候选按新TCP重求、独立URDF FK复核，侧吸升降采样加密到5mm，未改目标和限位。仅本地算法规划与回放，不连接实机。')
    server.gui.add_markdown('任务列表新增25个“单臂”任务：第1/2列左手，第3/4/5列右手。单臂仍15轴联合规划；空闲臂用收拢偏离/累计运动软代价，允许随升降避碰。')
    references = renderer.reference_conveyors()
    references.visible = True
    reference_control = server.gui.add_checkbox('显示双龙头车与光圈（等价长方体碰撞已启用）', initial_value=True)
    @reference_control.on_update
    def show_references(_event):
        renderer.submit(lambda: setattr(references, 'visible', reference_control.value and not renderer.collision_view))
    motion_policy = ('PR65后处理已开启：按记录时间播放，含速度/加速度/jerk代理限制；不是力矩或真实动力学验证。'
                     if getattr(planner, 'trajectory_optimization_enabled', False) else
                     '播放帧率按最快旋转关节30°/s统一换算，Updown等比播放；未加入加速度或动力学时间参数化。')
    server.gui.add_markdown(f'只读显示：规划模型与关节来自核心，匹配{renderer.material_matches}个彩色网格。完整集装箱统一装配到快照内壁边界；外观框架不额外参与碰撞。仅最里面一面箱墙，非475箱展示模板。\n\n{motion_policy} 底盘到位仍为瞬移。')
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
                                                   initial_value=next(iter(planner_labels)))
        run = server.gui.add_button("计算完整流程并播放")
        scope = server.gui.add_dropdown('回放范围', options=list(PLAYBACK_SCOPES), initial_value='全流程')
        replay = server.gui.add_button('从当前范围起点播放')
        play = server.gui.add_checkbox("播放", initial_value=False)
        playback_rate = server.gui.add_slider('播放倍率（只影响显示）', min=.1, max=3., step=.1, initial_value=1.)
        playback_status = server.gui.add_markdown('播放速度：旋转关节最快约30°/s。')
        frame = server.gui.add_slider("轨迹帧", min=1, max=1, step=1, initial_value=1)
        stage = server.gui.add_dropdown("范围内跳转阶段", options=[label for key, label in PHASE_NAMES.items() if key != 'conveyor_retreat'],
                                        initial_value=PHASE_NAMES["home"])
        spheres = server.gui.add_checkbox("显示附着箱40球", initial_value=False)
        diagnostic = server.gui.add_checkbox('显示接近/抽离失败候选（仅诊断）', initial_value=False)
        collision_edge = server.gui.add_checkbox('显示被拒绝的碰撞连接（仅诊断，不能执行）', initial_value=False)
        first_collision = server.gui.add_button('定位首个碰撞帧')
        status = server.gui.add_markdown("从第一初始姿态开始，一次计算完整流程。")
        timed_status = server.gui.add_markdown("当前结果没有时间化轨迹。")
        comparison_status = server.gui.add_markdown("")
    with server.gui.add_folder('末端运动轨迹与朝向'):
        trace_paths = server.gui.add_checkbox('保留左右TCP轨迹', initial_value=True)
        trace_axes = server.gui.add_checkbox('显示沿途TCP朝向', initial_value=True)
        trace_count = server.gui.add_slider('每臂朝向采样数', min=2, max=30, step=1, initial_value=10)
        server.gui.add_markdown('左TCP蓝线、右TCP橙线；朝向坐标轴X红/Y绿/Z蓝。轨迹使用world坐标，导航瞬移处不连线。单段包含起始边界帧，播放到该段末尾停止。')
        tcp_status = server.gui.add_markdown('选择任务并计算，或查看已载入的结果。')
    playback = StagePlayback()
    tcp_preview = TcpTrajectoryPreview(server, args.urdf, planner.home)
    current = {"result": None, "busy": False}
    navigation_export = {'last_values': None, 'stamp': 0.}
    navigation_export_enabled = getattr(planner, 'navigation_export_enabled', True)
    if navigation_export_enabled:
        server.gui.add_markdown('末端独立导航页：http://localhost:8180/ 。暂停本页后，在导航页读取当前场景与TCP。')
    collision_overlay = server.scene.add_frame('/collision_diagnostic', show_axes=False, visible=False)
    collision_nodes = []
    server.gui.add_markdown(server_policy if chained_policy else '串行实验：终点只选一次，碰撞/限位/末端Pose验收仍保留；不批量验收抽离运动，抽离段是未验证瞬移。' if serial_policy else '箱体旋转仅作软代价：累计额外旋转、回退和偏离弧继续影响连接代价，不再因超过90°/15°/45°直接拒绝。接触候选按8/16/32/64逐批完整抽离验收；碰撞、限位和末端Pose验收仍保留。')

    def refresh_comparison():
        lines = ["已计算完整流程对比（计算结果回放）", "", "| 规划器 | 结果 | 总计算 |",
                 "| --- | --- | --- |"]
        for label, kind in planner_labels.items():
            result = comparisons.get(kind)
            if result is None:
                continue
            verdict = "完成全部阶段" if result["success"] else result.get("blocked_stage", "失败")
            seconds = result.get("measured_wall_ms", result.get("experiment_wall_ms", result.get("total_ms", 0))) / 1000
            lines.append(f"| {label} | {verdict} | {seconds:.2f}s |")
        dual_count=sum(len(boxes)==2 for label,boxes in planner.task_options)
        single_count=sum(len(boxes)==1 for label,boxes in planner.task_options)
        lines += ["", f"{dual_count}组双臂＋{single_count}组单臂可选。首次生成当前场景终点缓存，后续复用；每次重新计算接触候选和路径。含双龙头车长方体碰撞，无Shortcut/TrajOpt，释放后RRT回Home。"]
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
            export_key = (tuple(sorted(boxes.items())), tuple(sorted(values.items())), phase)
            now = time.monotonic()
            if navigation_export_enabled and export_key != navigation_export['last_values']:
                obstacles = []
                for item in planner.snapshot.objects:
                    data = asdict(item)
                    position = list(data['pose']['position'])
                    position[0] += planner.base_advance
                    obstacles.append(dict(name=item.object_id, dimensions=list(item.dimensions_m), position=position, quaternion_wxyz=list(item.pose.quaternion_wxyz)))
                obstacles.append(dict(name='navigation_ground', dimensions=[100.,100.,.1],position=[0.,0.,-.05],quaternion_wxyz=[1.,0.,0.,0.]))
                targets = {}
                for side in boxes:
                    item = planner.snapshot.object(f'wall_box_{boxes[side]:02d}')
                    box = np.eye(4)
                    from scipy.spatial.transform import Rotation
                    box[:3, :3] = Rotation.from_quat(np.roll(item.pose.quaternion_wxyz,-1)).as_matrix()
                    box[:3, 3] = item.pose.position
                    box[0, 3] += planner.base_advance
                    targets[side] = (box @ np.linalg.inv(grasp_transform(side, top_grasp))).tolist()
                export = dict(task=boxes,phase=phase,frame_index=index+1,tcp_world={side:matrix.tolist() for side,matrix in renderer.tool_poses.items()},targets=targets,objects=obstacles,stamp_ns=time.time_ns(),source='8151 current displayed frame',payload=bool(has_payload),busy=bool(current['busy']))
                destination = Path('/tmp/alfa_curobo_8151_navigation_state.json')
                temporary = destination.with_suffix('.tmp')
                temporary.write_text(json.dumps(export,ensure_ascii=False))
                temporary.replace(destination)
                navigation_export.update(last_values=export_key,stamp=now)
            tcp_preview.update(result, trace_paths.value, trace_axes.value, int(trace_count.value))
            lines = ['当前TCP位姿（world，四元数wxyz）：']
            for side, matrix in renderer.tool_poses.items():
                position = ', '.join(f'{value:.4f}' for value in matrix[:3, 3])
                orientation = ', '.join(f'{value:.4f}' for value in quaternion(matrix))
                lines.append(f"\n\n**{'左' if side == 'left' else '右'}臂**：XYZ=({position})m；q=({orientation})")
                metric = tcp_preview.metrics.get(side)
                if metric:
                    lines.append(f"；当前范围路径长{metric['length_m']:.3f}m，累计转角{metric['rotation_deg']:.1f}°")
            tcp_status.content = ''.join(lines)
            for side, node in attached.items():
                matrix = renderer.tool_poses[side] @ grasp_transform(side, top_grasp)
                node.position = matrix[:3, 3]
                node.wxyz = quaternion(matrix)
                node.visible = has_payload and side in boxes
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
        if result and result.get('frames'):
            source_index = result['_source_indices'][index]
            location = f"；全流程帧{source_index + 1}/{len(current['result']['frames'])}" if source_index is not None else '；诊断帧'
            status.content = (
                f"**{('六段连接完成（原抽离验碰口径）' if result.get('lm_chained_cycle') else '实验阶段通过（抽离瞬移未验）' if result.get('extraction_policy',{}).get('mode')=='teleport_only' else '完整流程成功') if result['success'] else '已停止：存在阻塞'} · {PHASE_NAMES[phase]}**  \n"
                f"{scope.value} · 帧：**{index+1}/{len(result['frames'])}**{location}  \n"
                f"接触IK：**{result.get('contact_ik_count', 0)}** · 选中候选：**{result.get('selected_candidate', '无')}**  \n"
                f"{'诊断搜索' if result.get('rotation_diagnostic') else '全部规划'}：**{(result.get('measured_wall_ms',result.get('experiment_wall_ms',result.get('total_ms',0))) if result.get('lm_chained_cycle') else result.get('total_ms',0))/1000:.3f}s**  \n"
                f"最终Home误差：**{format(result['final_home_error'], '.8f') if 'final_home_error' in result else '尚未回Home'}**"
            )
            lazy = result.get('lazy_extraction')
            if result.get('guided_search_events'):
                from v3_guided_cycle_planner import guidance_summary
                if result.get('trajectory_postprocessing', {}).get('enabled'):
                    status.content += '\n\n**以下为几何搜索阶段诊断（Shortcut/平滑前），优化后轨迹见当前TCP曲线。**'
                status.content += '\n\n'+guidance_summary(result['guided_search_events'])
            if result.get('forward_precontact_policy'):
                status.content += f"\n\n**直接预接触IK：{result.get('precontact_ik_count',0)}个，{result.get('timing_ms',{}).get('precontact_ik_ms',0):.1f}ms。** 仅参考初始构型；实际到位后才求吸附IK并前向接近，没有反推3cm运动。"
            if result.get('extraction_policy',{}).get('mode')=='teleport_only':
                status.content+='\n\n**实验包含抽离瞬移：接触→后撤35cm终点之间未生成运动、未验碰撞，不能称为可执行完整抓取。其他RRT段按原完整模型验收。**'
            if lazy:
                costs = lazy['timings_ms']
                status.content += (
                    f"  \n完整抽离实际验收：**{lazy['checked_candidates']}/{lazy['candidate_pool']}**；扩批轮数：**{len(lazy['batches'])}**"
                    f"  \n候选生成 {costs['contact_generation_ms']:.1f}ms；终点预测 {costs['endpoint_prediction_ms']:.1f}ms；排序 {costs['prediction_ranking_ms']:.1f}ms"
                    f"  \n实际抽离验收 {costs['extraction_work_ms']:.1f}ms；累计运输搜索 {lazy['total_transport_search_ms']:.1f}ms"
                )
            if not result["success"]:
                status.content += f"  \n{result.get('blocked_stage')}: {result.get('blocked_details')}"
            if result.get('rotation_diagnostic'):
                status.content += '\n\n**旋转拒绝候选回放：仅显示搬运段，未通过执行验收。**'
            if result.get('rotation_quality_hard_gate') is False:
                status.content += '\n\n**旋转采用软代价，不设硬拒绝门槛；下方指标仅供观察。**'
            margin = result.get('conveyor_search_margin')
            if margin:
                status.content += f"\n\n放箱搜索余量：{margin['margin_m']*1000:.1f}mm；验边间隔：{np.rad2deg(margin.get('edge_resolution',.05)):.2f}°（加权关节单位）。"
            policy = result.get('adaptive_raised_policy')
            if policy:
                retry = policy.get('fine_collision_retry')
                if retry:
                    status.content += f"\n\n细验碰撞后加密：{'已触发' if retry['triggered'] else '未触发，保持原粗验'}；放箱搜索合计{retry['total_ms']:.1f}ms。"
                status.content += '\n\n当前高度：顶吸收束TCP z=1.2m，龙头车释放箱心z=1.082m。'
            if result.get('single_arm'):
                side_label = '左' if result['active_side'] == 'left' else '右'
                status.content += f'\n\n**单臂任务：{side_label}手工作；15轴联合规划，空闲臂可避让。**'
                idle_motion = result.get('idle_arm_motion')
                if idle_motion:
                    status.content += f"\n\n空闲臂最大偏离Home：{idle_motion['max_home_deviation_deg']:.2f}°；7关节累计运动：{idle_motion['cumulative_joint_motion_deg']:.1f}°。"
                idle_solver = result.get('single_contact_enumeration', {})
                if 'stages' in idle_solver:
                    status.content += f"\n\n空闲臂候选：GPU共筛{idle_solver['checked_candidates']}个组合，返回{idle_solver['accepted_candidates']}个；{idle_solver['total_ms']:.1f}ms，优化兜底{idle_solver['optimization_iterations']}轮。"
                envelope = result.get('transport_endpoint_envelope')
                if envelope:
                    status.content += f"\n\n运输终点整机包络（含空闲臂＋携箱）：**{envelope['radius_m']:.4f}m / {envelope['limit_m']:.3f}m**；{'通过' if envelope['passed'] else '未通过'}。"
            if collision_edge.value and result.get('conveyor_edge_diagnostic'):
                edge = result['conveyor_edge_diagnostic']
                status.content += f"\n\n**被拒绝的连接，仅诊断，不能执行**。当前帧{'通过' if edge['valid'][index] else '未通过'}检查；首个碰撞在本连接第{edge['first_bad_index']+1}帧。红球和红色障碍标记首个碰撞帧。"
                for item in edge['audit']['world_collisions'] + edge['audit']['base_internal_collisions']:
                    role = edge.get('sphere_roles', edge['sphere_links'])[item['sphere_index']]
                    status.content += f"\n\n{role}（{item['link']}） ↔ {item['obstacle']}：穿入{item['penetration_mm']:.2f}mm。"
                for item in edge['audit']['self_collisions']:
                    status.content += '\n\n' + ' ↔ '.join(item['links']) + f"：穿入{item['penetration_mm']:.2f}mm。"
            segment = next((item for item in result.get('timed_segments', ())
                            if item['frame_start'] <= index <= item['frame_end']), None)
            if segment:
                local = index - segment['frame_start']
                timed_status.content = (
                    f"**时间化阶段：{PHASE_NAMES.get(segment['name'], segment['name'])}**  \n"
                    f"阶段运动时间：**{segment['duration_s']:.3f}s** · 当前：**{segment['time_from_start_s'][local]:.3f}s**  \n"
                    f"方法：`{segment.get('parameterizer', 'recorded')}` · "
                    f"时间化计算：**{result.get('trajectory_timing_ms', 0)/1000:.3f}s**  \n"
                    f"全周期计算：**{result.get('measured_wall_ms', result.get('total_ms', 0))/1000:.3f}s** · "
                    f"运动：**{result.get('motion_duration_s', 0):.3f}s**")
            else:
                timed_status.content = "当前为吸附、释放或导航状态切换，不计机械臂运动时间。"
            if result.get('transport_rotation_metrics'):
                for side,metrics in result['transport_rotation_metrics'].items():
                    status.content+=f"  \n{side}箱累计旋转{metrics['total_rotation_deg']:.1f}°，额外{max(0.,metrics['total_rotation_deg']-metrics['required_rotation_deg']):.2f}°，回退{metrics['backtracking_deg']:.1f}°，偏离弧最大{metrics['max_deviation_deg']:.2f}°"
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

    def refresh_playback():
        result = displayed_result()
        frame.label = '全流程轨迹帧' if scope.value == '全流程' else '当前RRT段轨迹帧'
        frame.max = max(1, len(result.get('frames', []))) if result else 1
        frame.value = 1
        show_frame(0)
        if result and not result.get('frames'):
            status.content = f'当前结果没有“{scope.value}”的轨迹，该阶段可能尚未规划完成。'

    @scope.on_update
    def on_scope(_event):
        play.value = False
        diagnostic.value = False
        collision_edge.value = False
        refresh_playback()

    @replay.on_click
    def on_replay(_event):
        if current['busy']:
            return
        play.value = False
        refresh_playback()
        result = displayed_result()
        play.value = bool(result and result.get('frames'))

    @trace_paths.on_update
    @trace_axes.on_update
    @trace_count.on_update
    def on_trace_options(_event):
        show_frame(int(frame.value) - 1)

    @spheres.on_update
    def on_spheres(_event):
        for side in ("left", "right"):
            server.scene.add_frame(f"/attached/{side}/spheres", show_axes=False,
                                    visible=spheres.value)

    def displayed_result():
        result = current['result']
        mode = None
        if collision_edge.value and result and result.get('conveyor_edge_diagnostic'):
            mode = 'collision'
        elif diagnostic.value and result and result.get('extraction_diagnostic'):
            mode = 'extraction'
        return playback.select(result, PLAYBACK_SCOPES[scope.value], mode)

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
        refresh_playback()

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
        status.content = ('顶吸：搬运目标TCP朝下z=1.2m；抓取底盘到位偏移0.3m。' if planner.top_grasp else '侧吸：搬运目标箱心z=1.2m、Y倾斜45°。') + ('六段LM先验＋Informed Connect串联；新接近/抽离构型传给后续阶段。抽离有运动，仍按原途中空载/终点携箱验碰；导航到位瞬移，放箱后回Home。' if chained_policy else '接触64候选，解析抽离35cm；随后导航到位瞬移，固定底盘用目标IK＋RRT放箱。释放后箱子消失，直接RRT回Home，再导航返回。放箱箱心(-1.45,±0.425,1.082)m，含龙头车长方体碰撞。')

    @planner_selector.on_update
    def on_planner(_event):
        if current["busy"]:
            return
        play.value = False
        planner.planner_kind = planner_labels[planner_selector.value]
        result = comparisons.get(planner.planner_kind)
        current["result"] = result
        refresh_playback()
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
        scope.disabled = True
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
            result_directory = getattr(planner, 'result_directory', Path(__file__).resolve().parents[1] / 'generated/priority_visualized_task_live')
            path = result_directory / 'latest.json'
            path.parent.mkdir(parents=True,exist_ok=True)
            archive=path.parent/'runs'
            archive.mkdir(exist_ok=True)
            (archive/(str(time.time_ns())+'.json')).write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
            path.write_text(json.dumps(result, indent=2, ensure_ascii=False)+"\n")
            current["result"] = result
            comparisons[planner.planner_kind] = result
            refresh_comparison()
            if result["frames"]:
                refresh_playback()
                play.value = result["success"] and bool(displayed_result()['frames'])
            else:
                status.content = f"**已停止** {result.get('blocked_stage')}: {result.get('blocked_details')}"
            current['busy'] = False
            run.disabled = False
            task.disabled = False
            planner_selector.disabled = False
            scope.disabled = False
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
            if not result.get('frames'):
                play.value = False
                continue
            if timed_frames is not result['frames']:
                timed_frames = result['frames']
                times = np.asarray(result.get('time_from_start_s', ()), dtype=float)
                positive = np.diff(times)
                positive = positive[positive > 1e-12]
                playback_interval = float(np.median(positive)) if len(positive) else playback_frame_interval(result)
                playback_status.content = (f'播放：记录时间步长约{playback_interval*1000:.1f}ms。'
                                           if len(positive) else
                                           f'播放：约{1/playback_interval:.1f}帧/s；旋转关节最快30°/s。')
            cursor = int(frame.value) - 1
            times = result.get('time_from_start_s', ())
            if len(times) == len(timed_frames) and cursor + 1 < len(times):
                interval = max(0., times[cursor + 1] - times[cursor]) / playback_rate.value
            else:
                interval = playback_interval / playback_rate.value
            position = (id(timed_frames), int(frame.value), playback_rate.value)
            now = time.perf_counter()
            if position != observed_frame:
                observed_frame = position
                next_frame_time = now + interval
            if int(frame.value) >= len(result["frames"]):
                play.value = False
            elif now >= next_frame_time:
                frame.value = int(frame.value)+1
                observed_frame = None
        else:
            observed_frame = None
        delay = min(.005, max(0., next_frame_time - time.perf_counter())) if play.value else .005
        time.sleep(delay)


if __name__ == "__main__":
    main()
