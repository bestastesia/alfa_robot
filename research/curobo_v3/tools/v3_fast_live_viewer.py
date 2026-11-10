import argparse
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
import gzip
import json
from pathlib import Path
import time
import traceback
import uuid

import numpy as np
import plotly.graph_objects as go
from scipy.spatial.transform import Rotation
import viser

from alfa_motion_viser import ReadOnlyScene, default_visual_assets
from curobo_core.fixtures import tasks
from curobo_core.scene import SceneSnapshot
from v3_fast_assets import prepare_fast_assets
from v3_visualization_dependency import verify_frontend_package

ROOT = Path(__file__).resolve().parents[1]
JOINTS = ['updown'] + [f'{side}_joint{joint}' for side in ('left', 'right') for joint in range(1, 8)]


def task_options():
    options = tasks() + tasks([1, 0])
    for box_id in range(25):
        side = 'left' if box_id % 5 >= 3 else 'right'
        options.append((f"单臂 {'左手' if side == 'left' else '右手'} {box_id:02d}(排{5-box_id//5}/列{5-box_id%5})", {side: box_id}))
    return options


def read_json(path):
    return json.loads(gzip.decompress(path.read_bytes()) if path.suffix == '.gz' else path.read_text())


def load_segments(directory, index, record, source):
    segments = []
    elapsed = 0.
    for stage in record['stages']:
        path = directory / f"{index:03d}_{stage['phase']}.npz"
        if not path.exists():
            break
        with np.load(path) as stored:
            arrays = {name: stored[name].copy() for name in stored.files}
        locations = np.flatnonzero(np.asarray(source['phases']) == stage['phase'])
        base = dict(source['visual_home'])
        base.update(zip(source['joint_names'], source['frames'][max(0, int(locations[0]) - 1)]))
        duration = float(arrays['times'][-1])
        segments.append(dict(stage=stage, arrays=arrays, base=base, start=elapsed, duration=duration))
        elapsed += duration
    return segments, elapsed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8282)
    parser.add_argument('--radius-review', action='store_true')
    args = parser.parse_args()
    options = tasks() + tasks([1, 0])
    reviews={}
    if args.radius_review:
        experiment=ROOT/'generated/radius_margin_experiment_20261108'
        for index in (85,86):
            for label,folder in [('原半径规划',experiment/f'scale1_{index:03d}'),('外部障碍半径＋10%规划',experiment/f'world_only/scale1.1_{index:03d}')]:
                reviews[f'L4/R{1 if index==85 else 2} · {label}']=(folder,index)
        options=[(label,{}) for label in reviews]
    server = viser.ViserServer(host='127.0.0.1', port=args.port, label='半径预留实验 · 已记录全流程回放' if args.radius_review else 'Fast 三层链路 · 点击实时计算')
    verify_frontend_package()
    display_assets, _ = prepare_fast_assets()
    renderer = ReadOnlyScene(server, display_assets.urdf, default_visual_assets())
    robot_layers = list(renderer.visual_layers)
    home = read_json(ROOT / 'config/rezero_home.json')
    initial = {'updown': home['updown_m'], 'base_x': 0., 'base_y': 0., 'base_yaw': 0., 'head_joint': 0., 'head_pitch_joint': 0.}
    for arm in ('left', 'right'):
        initial.update({f'{arm}_joint{joint + 1}': float(np.deg2rad(value)) for joint, value in enumerate(home[f'{arm}_deg'])})
    renderer.apply_state(initial)
    if args.radius_review:
        server.gui.add_markdown('**已记录实验结果回放，不是点击重新计算，不发送实机。** 两个顶吸任务分别对比原半径和RRT外部障碍查询半径＋10%。候选IK、LM、自碰撞保持原模型，TrajOpt使用基础半径、形状代价100倍，六段均经TimeOpt，每箱10kg。')
        server.gui.add_markdown('**所有任务仍有TrajOpt失败阶段，保留完整候选供观察，不代表几何验收通过。** 可切换任务/方案、从头播放、调整倍速、跳转阶段，并查看左右七轴速度/加速度/jerk/力矩。')
    else:
        server.gui.add_markdown('**点击才计算，不读取历史任务结果，不发送实机。** 每次重新生成候选、规划、优化与时间重映射。模型和求解器资源允许复用；首次初始化单独计时。每箱10kg。')
        server.gui.add_markdown('规划：多候选吸附 → 抽离终点预测 → 多起终点RRT选组合 → 选中后15轴LM抽离 → 同运输目标重连 → Home接近 → LM放置。**自由RRT世界查询膨胀10%，固定底盘/立柱为四盒，升降架20球保留。Fast TrajOpt只优化自由RRT段，Fast TimeOpt覆盖全部六运动段。每次点击实时计算，不复用任务答案。**')
        server.gui.add_markdown('**LM微调已接入**：双臂抽离/放箱在原LM之后、TimeOpt之前，使用64控制点和最多32轮L-BFGS；起终关节固定，笛卡尔参考为软代价。贴近及自由RRT段不变，不新增后验验碰门。')
        server.gui.add_markdown('**已知问题保留：TrajOpt间隙代价可能触发失败标记，仍保存并展示候选。** 不新增后处理验碰门，全流程TimeOpt后按真实运动时间播放。本轮开放双臂任务。')
    task = server.gui.add_dropdown('任务', options=[label for label, _ in options])
    run = server.gui.add_button('加载所选实验结果' if args.radius_review else '重新计算当前任务')
    status = server.gui.add_markdown('尚未计算。选择任务后点击按钮。')
    timing = server.gui.add_markdown('')
    play = server.gui.add_checkbox('播放当前计算结果', initial_value=False, disabled=True)
    restart = server.gui.add_button('从头播放全流程', disabled=True)
    speed = server.gui.add_dropdown('播放倍速', options=['0.25', '0.5', '1', '2'], initial_value='1')
    cursor = server.gui.add_slider('全流程时间进度 %', min=0., max=100., step=.01, initial_value=0., disabled=True)
    phase = server.gui.add_dropdown('跳转阶段 / 图表阶段', options=['等待计算'], disabled=True)
    side = server.gui.add_dropdown('图表手臂', options=['左臂', '右臂'])
    current = server.gui.add_markdown('')
    charts = [server.gui.add_plotly(go.Figure(), aspect=1.7) for _ in range(4)]
    state = dict(busy=False, segments=[], duration=0.)
    engines = {}
    executor = ThreadPoolExecutor(max_workers=1)
    held = {}
    pickup = {}
    placed = {}
    environment = []
    for arm in ('left', 'right'):
        held[arm] = server.scene.add_frame('/held/' + arm, show_axes=False, visible=False)
        renderer.carton_object('/held/' + arm + '/box', (.3, .4, .4))
        pickup[arm] = server.scene.add_frame('/pickup/' + arm, show_axes=False, visible=False)
        renderer.carton_object('/pickup/' + arm + '/box', (.3, .4, .4))
        placed[arm] = server.scene.add_frame('/placed/' + arm, show_axes=False, visible=False)
        renderer.carton_object('/placed/' + arm + '/box', (.3, .4, .4))

    @server.on_client_connect
    def connect(client):
        client.camera.position = (2.4, -3.1, 2.2)
        client.camera.look_at = (.55, 0, 1.)
        client.camera.up_direction = (0, 0, 1)

    def plot():
        if state.get('untimed_preview', False):
            return
        segment = next((item for item in state['segments'] if item['stage']['phase'] == phase.value), None)
        if segment is None:
            return
        arrays = segment['arrays']
        offset = 0 if side.value == '左臂' else 7
        colors = ['#2563eb', '#ea580c', '#16a34a', '#dc2626', '#9333ea', '#0891b2', '#a16207']
        fields = [('velocity', '速度', '°/s', 180 / np.pi), ('acceleration', '加速度', '°/s²', 180 / np.pi), ('jerk', 'jerk', 'rad/s³', 1), ('torque', '力矩', 'N·m', 1)]
        for chart, (field, title, unit, factor) in zip(charts, fields):
            figure = go.Figure()
            for joint in range(7):
                values = arrays[field][:, offset + joint + (0 if field == 'torque' else 1)] * factor
                chunks = np.array_split(np.arange(len(values)), min(140, len(values)))
                selected = np.unique(np.concatenate([[chunk[0], chunk[-1], chunk[np.argmin(values[chunk])], chunk[np.argmax(values[chunk])]] for chunk in chunks]))
                figure.add_trace(go.Scatter(x=arrays['times'][selected], y=values[selected], name=f'J{joint + 1}', line=dict(color=colors[joint])))
            figure.add_vline(x=0., line_color='#111827')
            figure.update_layout(title=f'{phase.value} · {side.value} J1–J7 · {title}', xaxis_title='阶段时间 (s)', yaxis_title=f'{title} ({unit})', template='plotly_white', height=380, legend=dict(orientation='h'), margin=dict(l=60, r=10, t=65, b=65))
            chart.figure = figure

    def update():
        if not state['segments']:
            return
        timestamp = state['duration'] * cursor.value / 100
        segment = next((item for item in state['segments'] if timestamp < item['start'] + item['duration']), state['segments'][-1])
        if play.value and phase.value != segment['stage']['phase']:
            phase.value = segment['stage']['phase']
            plot()
        arrays = segment['arrays']
        local_time = np.clip(timestamp - segment['start'], 0, segment['duration'])
        values = dict(segment['base'])
        values.update(zip(JOINTS, [float(np.interp(local_time, arrays['times'], arrays['position'][:, column])) for column in range(15)]))
        renderer.apply_state(values)
        source = state['source']
        loaded = segment['stage']['phase'] in ('extract', 'transport', 'conveyor_place')
        for arm, node in held.items():
            node.visible = loaded and arm in source['task']
            pickup[arm].visible = segment['stage']['phase'] in ('approach', 'contact_approach') and arm in source['task']
            placed[arm].visible = segment['stage']['phase'] == 'return_home' and arm in source['task']
            if node.visible:
                pose = renderer.tool_poses[arm] @ np.asarray(source['visual_tool_to_box'][arm])
                node.position = pose[:3, 3]
                node.wxyz = np.roll(Rotation.from_matrix(pose[:3, :3]).as_quat(), 1)
        unit = '帧（诊断预览，无真实时间轴）' if state.get('untimed_preview', False) else 's'
        current.content = f"**{segment['stage']['phase']} · {timestamp:.3f}/{state['duration']:.3f}{unit}**（理想模型回放）"
        if not state.get('untimed_preview', False) and (not play.value or time.perf_counter() - state.get('chart_updated', 0.) > .3):
            chart_segment = next((item for item in state['segments'] if item['stage']['phase'] == phase.value), segment)
            chart_time = np.clip(timestamp - chart_segment['start'], 0., chart_segment['duration'])
            for chart in charts:
                figure = chart.figure
                if figure.layout.shapes:
                    figure.layout.shapes[0].update(x0=chart_time, x1=chart_time)
                    chart.figure = figure
            state['chart_updated'] = time.perf_counter()

    def finish(directory, index, record, init_seconds, compute_seconds):
        state['busy'] = False
        task.disabled = run.disabled = False
        source = read_json(directory / f'{index:03d}_source.json.gz')
        state['source'] = source
        state['segments'], state['duration'] = load_segments(directory, index, record, source)
        state['untimed_preview'] = False
        review = source.get('selected_lm_review') or {}
        if not state['segments'] and review.get('lm_frames'):
            positions = np.asarray(review['lm_frames'])
            base = dict(source['visual_home'], base_x=source['visual_base_advance'], base_y=0., base_yaw=0.)
            state['duration'] = float(len(positions) - 1)
            state['segments'] = [dict(stage=dict(phase='extract'), arrays=dict(position=positions, times=np.arange(len(positions))), base=base, start=0., duration=state['duration'])]
            state['untimed_preview'] = True
        invalid = sum(item.get('posthoc_invalid_frames', 0) for item in record['stages'])
        dynamic = all(item.get('time_optimization', {}).get('dynamic_valid', False) for item in record['stages'])
        checked=sum(item.get('posthoc_checked_frames',0) for item in record['stages'])
        validation = f'动力学采样复核：{dynamic}；'+(f'后验碰撞/限位无效采样：{invalid}' if checked else '后验碰撞复检：关闭，沿用规划检查') if record['stages'] else '后处理动力学与碰撞复核：未执行'
        flags = '；'.join(f"{stage['phase']}：{'通过' if stage['trajectory_optimization']['solver_success'] else '失败候选（保留显示）'}" for stage in record['stages'] if 'trajectory_optimization' in stage)
        status.content = f"**请求 {directory.name} 完成**\n\n完整候选：{record.get('complete_candidate', False)}；{validation}。\n\nTrajOpt：{flags or '未执行'}。\n\n{record.get('blocked_stage', '')} {record.get('blocked_details', '')}\n\n仅查看本次返回数据，不代表实机验收。"
        timing.content = f"资源初始化 **{init_seconds:.3f}s**；本次计算墙钟 **{compute_seconds:.3f}s**；轨迹规划 **{record['planning_ms']/1000:.3f}s**；TrajOpt纯求解 **{record.get('geometry_optimization_ms', 0)/1000:.3f}s**；LM后处理（准备/调整/诊断） **{record.get('lm_refinement_ms', 0)/1000:.3f}s**；TimeOpt纯求解 **{record.get('time_optimization_ms', 0)/1000:.3f}s**。\n\n运动时长 **{state['duration']:.3f}s**，不计入计算耗时。墙钟包含拟合、初始化求解器、采样、诊断及保存。"
        if state['untimed_preview']:
            timing.content = f"资源初始化 **{init_seconds:.3f}s**；本次计算 **{compute_seconds:.3f}s**。\n\n当前仅显示选中候选的LM抽离诊断，共{len(positions)}帧；**未拼接运输段，未进行TimeOpt，无运动时长与动力学曲线。**"
        if not state['segments']:
            return
        renderer.visual_layers[:] = robot_layers
        renderer.container_walls.clear()
        snapshot = SceneSnapshot.from_dict(source['visual_scene_snapshot'])
        for node in environment:
            node.remove()
        environment.clear()
        for item in snapshot.objects:
            if item.object_id in ('front_wall', 'left_wall', 'right_wall', 'ceiling') or item.object_id.startswith('conveyor_pair'):
                continue
            if state['untimed_preview'] and item.object_id.startswith('wall_box_'):
                continue
            environment.append(server.scene.add_box('/environment/' + item.object_id, dimensions=item.dimensions_m,
                position=np.asarray(item.pose.position) + [source['visual_base_advance'], 0, 0],
                wxyz=item.pose.quaternion_wxyz, color=(180, 125, 60), opacity=.4))
        renderer.assembled_container(tuple(replace(item, pose=replace(item.pose, position=(item.pose.position[0] + source['visual_base_advance'], *item.pose.position[1:]))) for item in snapshot.objects))
        renderer.set_scene_view(False, True)
        renderer.reference_conveyors().visible = True
        renderer.update_conveyor_targets(source.get('conveyor_box_targets', {}))
        extraction = next((item for item in state['segments'] if item['stage']['phase'] == 'extract'), None)
        if extraction is not None:
            values = dict(extraction['base'])
            values.update(zip(JOINTS, extraction['arrays']['position'][0]))
            renderer.apply_state(values)
            for arm in source['task']:
                pose = renderer.tool_poses[arm] @ np.asarray(source['visual_tool_to_box'][arm])
                pickup[arm].position = pose[:3, 3]
                pickup[arm].wxyz = np.roll(Rotation.from_matrix(pose[:3, :3]).as_quat(), 1)
        for arm, target in source.get('conveyor_box_targets', {}).items():
            placed[arm].position = target['position']
            placed[arm].wxyz = target['quaternion_wxyz']
        phase.options = [item['stage']['phase'] for item in state['segments']]
        phase.value = phase.options[0]
        phase.disabled = cursor.disabled = play.disabled = restart.disabled = False
        cursor.value = 0.
        plot()
        update()

    def fail(message):
        state['busy'] = False
        task.disabled = run.disabled = False
        status.content = '**本次计算异常，未加载旧结果：**\n\n' + message

    def compute(index, directory):
        try:
            began = time.perf_counter()
            if 'planner' not in engines:
                renderer.submit(lambda: setattr(status, 'content', '首次初始化 Fast / 模型 / 求解资源；尚未生成任务轨迹。'))
                from v3_fast_three_layers import FastThreeLayerPlanner
                engines['planner'] = FastThreeLayerPlanner()
            init_seconds = time.perf_counter() - began
            planner = engines['planner']
            if planner.planner.task_options[index][1] != options[index][1]:
                raise ValueError('任务列表与规划器不一致')
            def progress(message):
                renderer.submit(lambda message=str(message): setattr(status, 'content', f'**请求 {directory.name} 计算中**\n\n{message}'))
            began = time.perf_counter()
            record = planner.full_cycle(index, progress=progress, output_directory=directory, benchmark_recompute=False)
            compute_seconds = time.perf_counter() - began
            (directory / 'request.json').write_text(json.dumps(dict(request=directory.name, task_index=index, initialization_s=init_seconds, computation_s=compute_seconds, record=record), ensure_ascii=False, indent=2))
            renderer.submit(lambda: finish(directory, index, record, init_seconds, compute_seconds))
        except Exception:
            message = traceback.format_exc()
            print(message, flush=True)
            renderer.submit(lambda message=message: fail(message))

    def start():
        if args.radius_review:
            directory,index=reviews[task.value]
            record=read_json(directory/f'{index:03d}_report.json')
            record['complete_candidate']=record['all_timeopt_success']
            finish(directory,index,record,0.,0.)
            flags=[f"{stage['phase']}：{'通过' if stage['trajectory_optimization']['solver_success'] else '失败候选'}" for stage in record['stages'] if 'trajectory_optimization' in stage]
            status.content='**实验回放，未重新计算。**\n\n'+'；'.join(flags)+'。\n\n全部阶段TimeOpt成功；动力学采样通过。TrajOpt未全通过。'
            timing.content=f"**运动时间 {state['duration']:.3f}s**；原记录规划耗时 {record['planning_ms']/1000:.3f}s（含冷/暖资源差异，非稳定性能基准）。"
            return
        if state['busy']:
            return
        state.update(busy=True, segments=[], duration=0.)
        play.value = False
        task.disabled = run.disabled = play.disabled = cursor.disabled = phase.disabled = restart.disabled = True
        timing.content = current.content = ''
        for chart in charts:
            chart.figure = go.Figure()
        for node in list(held.values()) + list(pickup.values()) + list(placed.values()):
            node.visible = False
        directory = ROOT / 'generated/fast_live_requests' / (time.strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:8])
        directory.mkdir(parents=True)
        status.content = f'请求 {directory.name} 已提交；从头计算。'
        executor.submit(compute, [label for label, _ in options].index(task.value), directory)

    def jump():
        segment = next((item for item in state['segments'] if item['stage']['phase'] == phase.value), None)
        if segment is not None:
            play.value = False
            cursor.value = segment['start'] / state['duration'] * 100
            plot()
            update()

    run.on_click(lambda event: renderer.submit(start))
    def replay():
        cursor.value = 0.
        play.value = True
        update()
    restart.on_click(lambda event: renderer.submit(replay))
    cursor.on_update(lambda event: renderer.submit(update) if event.client_id is not None else None)
    phase.on_update(lambda event: renderer.submit(jump) if event.client_id is not None else None)
    side.on_update(lambda event: renderer.submit(plot))
    if args.radius_review:
        task.on_update(lambda event: renderer.submit(start) if event.client_id is not None else None)
        start()
    print(f'Ready: http://127.0.0.1:{args.port}/ (no task computed)', flush=True)
    previous = time.perf_counter()
    while True:
        renderer.drain()
        now = time.perf_counter()
        if play.value and state['duration'] > 0:
            rate = 20 if state.get('untimed_preview', False) else 1
            cursor.value = min(100., cursor.value + (now - previous) * float(speed.value) * rate / state['duration'] * 100)
            update()
            if cursor.value >= 100:
                play.value = False
        previous = now
        time.sleep(.02)


if __name__ == '__main__':
    main()
