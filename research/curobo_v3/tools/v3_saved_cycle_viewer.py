import argparse
from dataclasses import replace
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import viser

from alfa_motion_viser import ReadOnlyScene, default_visual_assets
from curobo_core.scene import SceneSnapshot
from v3_interactive_60_tasks import DEFAULT_URDF
from v3_playback import playback_frame_interval
from v3_visualization_dependency import verify_frontend_package


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--results', type=Path, nargs='+', required=True)
    parser.add_argument('--port', type=int, default=8152)
    parser.add_argument('--urdf', type=Path, default=DEFAULT_URDF)
    parser.add_argument('--title', default='顶吸运输位+20cm / 放箱位-10cm · 全流程回放')
    parser.add_argument('--description-file', type=Path)
    parser.add_argument('--smoke-test', action='store_true')
    args = parser.parse_args()
    verify_frontend_package()
    results = {}
    for path in args.results:
        result = json.loads(path.read_text())
        label = '/'.join(('L' if side == 'left' else 'R') + str(box) for side, box in result['task'].items())
        label = result.get('experiment_label', label)
        results[label] = result
    server = viser.ViserServer(host='127.0.0.1', port=args.port, label=args.title)
    renderer = ReadOnlyScene(server, args.urdf, default_visual_assets())
    conveyor_root = renderer.reference_conveyors()
    conveyor_root.visible = True
    first = next(iter(results.values()))
    snapshot = SceneSnapshot.from_dict(first.get('visualization_scene_snapshot') or first['predicted_scene_events'][0]['snapshot'])
    advance = first.get('base_advance_m', 0.)
    environment = tuple(replace(item, pose=replace(item.pose, position=(item.pose.position[0] + advance, *item.pose.position[1:])))
                        for item in snapshot.objects if not item.object_id.startswith(('task_fill_', 'wall_box_')))
    renderer.assembled_container(environment)
    for item in environment:
        renderer.snapshot_object(item)
    held = {side: renderer.carton_object('/held/' + side, (.3, .4, .4)) for side in ('left', 'right')}
    sources = {side: renderer.carton_object('/source/' + side, (.3, .4, .4)) for side in ('left', 'right')}
    task = server.gui.add_dropdown('任务', options=list(results), initial_value=next(iter(results)))
    play = server.gui.add_checkbox('播放', initial_value=False)
    frame = server.gui.add_slider('轨迹帧', min=1, max=len(first['frames']), step=1, initial_value=1)
    phase_labels = {'home': '初始姿态', 'base_pick_reposition': '导航到位：抓取底盘瞬移',
        'approach': 'Home→3cm预接触', 'contact_approach': '最后3cm贴近', 'attach': '吸附箱子',
        'extract': '35cm解析抽离', 'transport': '搬运到提高20cm的收束位', 'base_reposition': '导航到位：放置底盘瞬移',
        'conveyor_place': 'RRT到降低10cm的放箱目标', 'release': '释放：箱子移出场景',
        'return_home': 'RRT直接回Home', 'base_return': '导航返回：底盘瞬移', 'cartesian_push': '保持高度/朝向直线前推',
        'rejected_continuous_ik': '被淘汰的连续IK（仅诊断）'}
    phase = server.gui.add_dropdown('跳转阶段', options=list(phase_labels.values()), initial_value=phase_labels['home'])
    cutaway = server.gui.add_checkbox('剖视集装箱（碰撞不变）', initial_value=True)
    collision = server.gui.add_checkbox('显示碰撞场景', initial_value=False)
    has_collision_preview = any(result.get('collision_preview') for result in results.values())
    diagnostic_nodes = []
    if has_collision_preview:
        compare_frame = server.gui.add_dropdown('碰撞对照', options=['被淘汰：25.8cm', '最后合法：25.7cm'], initial_value='被淘汰：25.8cm')
        show_diagnostic = server.gui.add_checkbox('显示真实尺寸碰撞球', initial_value=True)
        zoom_wrist = server.gui.add_button('左腕近景')
        zoom_all = server.gui.add_button('恢复全景')
    server.gui.add_markdown(args.description_file.read_text() if args.description_file else '本次5组均为顶吸任务。收束运输TCP z=1.20m、箱心约1.00m；固定放箱箱心z=1.082m。保留原代价，未给预接触/放箱额外加左右R。')
    server.gui.add_markdown('红球是被淘汰状态的真实碰撞球，绿色为最后合法状态；只作诊断，不作为可执行轨迹。' if has_collision_preview else '冻结结果只读回放，最快旋转关节30°/s等比播放；导航是状态瞬移。箱底释放时仍高于0.732m台面15cm，非物理落稳仿真。')
    status = server.gui.add_markdown('')
    fill = []
    source_poses = {}

    def render(index):
        result = results[task.value]
        index = min(max(index, 0), len(result['frames']) - 1)
        values = dict(zip(result['joint_names'], result['frames'][index]))
        values.update({name: values.get(name, 0.) for name in renderer.model.actuated_joint_names})
        renderer.apply_state(values)
        display_targets={side:dict(target,table_height_m=target.get('table_height_m',.732))
                         for side,target in result['conveyor_box_targets'].items()}
        renderer.update_conveyor_targets(display_targets)
        renderer.set_scene_view(collision.value, cutaway.value)
        conveyor_root.visible = not collision.value
        for node in diagnostic_nodes:
            node.remove()
        diagnostic_nodes.clear()
        diagnostic = result.get('collision_preview')
        diagnostic_text = ''
        if diagnostic and index in (diagnostic['last_accepted_frame'], diagnostic['rejected_frame']):
            rejected = index == diagnostic['rejected_frame']
            current = diagnostic['states'][int(rejected)]
            diagnostic_text = f"\n\n**{'被淘汰25.8cm' if rejected else '最后合法25.7cm'}**：两球穿透 **{current['penetration_mm']:.6f}mm**（负数为间隙）。"
            if show_diagnostic.value:
                color = (255, 40, 30) if rejected else (30, 210, 100)
                for offset, (center, radius) in enumerate(zip(current['centers_world'], current['radii_m'])):
                    diagnostic_nodes.append(server.scene.add_icosphere(f'/diagnostic/sphere{offset}', radius=radius,
                        position=center, color=color, opacity=.35, subdivisions=3))
                    diagnostic_nodes.append(server.scene.add_icosphere(f'/diagnostic/wire{offset}', radius=radius,
                        position=center, color=color, wireframe=True, subdivisions=2))
                    diagnostic_nodes.append(server.scene.add_label(f'/diagnostic/label{offset}',
                        text=f"{diagnostic['links'][offset]} · 球{diagnostic['sphere_indices'][offset]}",
                        position=np.asarray(center)+[0., 0., radius+.015], font_screen_scale=.75))
        for side, node in held.items():
            matrix = renderer.tool_poses[side] @ np.asarray(result['tool_to_box'][side])
            node.position = matrix[:3, 3]
            node.wxyz = np.roll(Rotation.from_matrix(matrix[:3, :3]).as_quat(), 1)
            node.visible = result['payload'][index]
            sources[side].position = source_poses[side][:3, 3]
            sources[side].wxyz = np.roll(Rotation.from_matrix(source_poses[side][:3, :3]).as_quat(), 1)
            sources[side].visible = index < result['phases'].index('attach')
        current_phase = result['phases'][index]
        stage_info = phase_labels.get(current_phase, current_phase)
        outcome = result.get('preview_status') or ('完整流程成功' if result['success'] else '失败')
        home = f"Home误差 **{result['final_home_error']:.8f}**" if result.get('final_home_error') is not None else '本次未释放、未返回Home'
        push = result.get('push_extent')
        extent = f"\n\n连续推出 **{push['maximum_continuous_distance_m']*100:.1f}cm**；首个失败：{(push.get('first_failure') or {}).get('reason', '达到测试上限')}。" if push else ''
        time_label = '本次推出测试计算' if push else '总规划'
        status.content = (f"**{outcome} · {task.value} · {stage_info}**\n\n"
                          f"帧 **{index+1}/{len(result['frames'])}**；{time_label} **{result['total_ms']/1000:.3f}s**；"
                          f"{home}。\n\n"
                          f"回放 **{1/playback_frame_interval(result):.1f}fps**；底盘x={values['base_x']:+.3f}m，yaw={np.rad2deg(values['base_yaw']):.1f}°。{extent}{diagnostic_text}")

    def setup():
        result = results[task.value]
        for node in fill:
            node.remove()
        fill.clear()
        for index, item in enumerate(result.get('generated_obstacles', [])):
            fill.append(server.scene.add_box(f'/task_fill/{index}', position=item['center'], dimensions=item['dimensions'],
                                              color=(205, 145, 90), opacity=.25))
        attach = result['phases'].index('attach')
        mapping = dict(zip(result['joint_names'], result['frames'][attach]))
        mapping.update({name: mapping.get(name, 0.) for name in renderer.model.actuated_joint_names})
        renderer.apply_state(mapping)
        for side in held:
            source_poses[side] = renderer.tool_poses[side] @ np.asarray(result['tool_to_box'][side])
        frame.max = len(result['frames'])
        first_index = result['phases'].index('base_reposition') if result.get('push_extent') else 0
        if result.get('collision_preview'):
            first_index = result['collision_preview']['rejected_frame']
        frame.value = first_index+1
        render(first_index)

    @task.on_update
    def changed(_event):
        play.value = False
        renderer.submit(setup)

    @frame.on_update
    def frame_changed(_event):
        index = int(frame.value) - 1
        renderer.submit(lambda: render(index))

    @cutaway.on_update
    @collision.on_update
    def view_changed(_event):
        renderer.submit(lambda: render(int(frame.value) - 1))

    @phase.on_update
    def phase_changed(_event):
        result = results[task.value]
        key = next(key for key, label in phase_labels.items() if label == phase.value)
        if key in result['phases']:
            play.value = False
            frame.value = result['phases'].index(key) + 1

    def camera_view(client, close):
        diagnostic = results[task.value].get('collision_preview')
        if close and diagnostic:
            center = np.mean(diagnostic['states'][1]['centers_world'], axis=0)
            client.camera.look_at = center
            client.camera.position = center+[-.30, -.25, .16]
        else:
            client.camera.position = (-4.8, -3.4, 2.8)
            client.camera.look_at = (-.7, 0., 1.)
        client.camera.up_direction = (0., 0., 1.)

    if has_collision_preview:
        @compare_frame.on_update
        def compare_changed(_event):
            play.value = False
            diagnostic = results[task.value]['collision_preview']
            frame.value = diagnostic['rejected_frame' if compare_frame.value.startswith('被淘汰') else 'last_accepted_frame']+1

        @show_diagnostic.on_update
        def diagnostic_changed(_event):
            renderer.submit(lambda: render(int(frame.value)-1))

        @zoom_wrist.on_click
        def wrist_clicked(event):
            if event.client is not None:
                camera_view(event.client, True)

        @zoom_all.on_click
        def all_clicked(event):
            if event.client is not None:
                camera_view(event.client, False)

    @server.on_client_connect
    def connected(client):
        camera_view(client, has_collision_preview)

    setup()
    if args.smoke_test:
        for label,result in results.items():
            task.value=label
            setup()
            for phase_key in dict.fromkeys(result['phases']):
                render(result['phases'].index(phase_key))
            render(len(result['frames'])-1)
        print('SMOKE PASS: all task and phase transitions rendered',flush=True)
        server.stop()
        return
    next_time = 0.
    previous = None
    print(f'Ready: http://127.0.0.1:{args.port}', flush=True)
    while True:
        renderer.drain()
        if play.value:
            result = results[task.value]
            position = (task.value, int(frame.value))
            interval = playback_frame_interval(result)
            now = time.perf_counter()
            if position != previous:
                previous = position
                next_time = now + interval
            if int(frame.value) == len(result['frames']):
                play.value = False
            elif now >= next_time:
                frame.value = int(frame.value) + 1
                previous = (task.value, int(frame.value))
                next_time = time.perf_counter() + interval
        else:
            previous = None
        time.sleep(min(.005, max(0., next_time-time.perf_counter())) if play.value else .005)


if __name__ == '__main__':
    main()
