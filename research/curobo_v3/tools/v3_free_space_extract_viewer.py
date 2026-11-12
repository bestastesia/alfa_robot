"""CPU-only, full-URDF side-by-side replay of the three XYZ extraction cases.

No planner, CUDA, hardware commands, or unavailable frozen frontend imports.
Uses the saved native TimeOpt positions/timestamps, never a normalized speed.
"""
import argparse
from functools import lru_cache
import gzip
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from curobo_core.contracts import ACTIVE_JOINTS
from v3_fast_assets import local_generated_path

ROOT = Path(__file__).resolve().parents[1]
LABELS = {'top_outer': '顶层外侧 L24/R20', 'upper_inner': '上层内侧 L18/R16', 'upper_outer': '上层外侧 L19/R15'}
SCOPES = ('抽离＋运输', '仅抽离', '仅运输')


def read(path):
    path = Path(path)
    return json.loads(gzip.decompress(path.read_bytes()) if path.suffix == '.gz' else path.read_text())


def timeline(extract, transport, scope):
    selected = [('抽离', extract), ('运输', transport)]
    if scope == '仅抽离':
        selected = selected[:1]
    elif scope == '仅运输':
        selected = selected[1:]
    elif scope != '抽离＋运输':
        raise ValueError('Unknown replay scope')
    positions, times, boundaries = [], [], []
    elapsed = 0.
    for label, arrays in selected:
        q, t = arrays['position'], arrays['times']
        if q.ndim != 2 or q.shape[1] != 15 or len(t) != len(q) or not np.isfinite(q).all() or not np.isfinite(t).all() or t[0] != 0 or np.any(np.diff(t) <= 0):
            raise ValueError('Expected finite native timed canonical15 trajectory')
        if positions and np.max(abs(positions[-1][-1] - q[0])) > 1e-4:
            raise ValueError('Discontinuous loaded phase seam')
        first = int(bool(positions))
        positions.append(q[first:])
        times.append(elapsed + t[first:].astype(float))
        elapsed += float(t[-1])
        boundaries.append((label, elapsed))
    return dict(position=np.vstack(positions), times=np.concatenate(times), boundaries=boundaries, duration=elapsed)


def joint_at(track, stamp):
    stamp = np.clip(stamp, 0., track['duration'])
    return np.array([np.interp(stamp, track['times'], track['position'][:, i]) for i in range(15)])


def load_pair(root, trial):
    name = f"{trial['case']}_s{trial['seed']}"
    baseline = root/'multiseed-20261111'/name
    campaign = 'clearance-20261111' if trial['selected_campaign'] == 'clearance' else 'multiseed-20261111'
    candidate = root/campaign/name
    if not (baseline/'comparison.json').is_file():
        raise FileNotFoundError('Restore the MOTION-447 archive first; see FREE_SPACE_EXTRACT.md')
    record = read(baseline/'comparison.json')
    index = record['baseline']['index']
    source = read(baseline/f'{index:03d}_source.json.gz')
    if source['task'] != record['task'] or abs(source['visual_base_advance']) > 1e-8:
        raise ValueError('Replay currently supports these fixed-base side-grasp cases only')
    data, files = {}, {}
    for variant, directory, prefix in [('baseline', baseline, f'{index:03d}'), ('xyz', candidate, f"adaptive_rank{trial['selected_rank']}")]:
        data[variant], files[variant] = {}, {}
        for phase in ('extract', 'transport'):
            path = directory/f'{prefix}_{phase}.npz'
            with np.load(path) as arrays:
                data[variant][phase] = {key: arrays[key].copy() for key in ('times', 'position')}
            files[variant][phase] = dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    for variant in data.values():
        timeline(variant['extract'], variant['transport'], SCOPES[0])
    for first, last in ((data['baseline']['extract']['position'][0], data['xyz']['extract']['position'][0]),
                        (data['baseline']['transport']['position'][-1], data['xyz']['transport']['position'][-1])):
        if np.max(abs(first-last)) > 1e-4:
            raise ValueError('Comparison endpoints changed')
    return dict(source=source, record=record, data=data, files=files, trial=trial)


def frozen_urdf():
    path = ROOT/'generated/fast_runtime_assets/fixed.urdf'
    original = ROOT/'generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf'
    expected = ET.parse(original)
    for mesh in expected.findall('.//mesh'):
        relocated = local_generated_path(mesh.get('filename'))
        if not relocated.is_absolute():
            relocated = original.parent/relocated
        mesh.set('filename', str(relocated))
    if ET.tostring(expected.getroot()) != ET.tostring(ET.parse(path).getroot()):
        raise ValueError('Display URDF differs from relocated frozen PR82 model')
    return path


def self_check(root, report):
    for trial in report['trials']:
        pair = load_pair(root, trial)
        for name in pair['data']:
            data = pair['data'][name]
            for scope in SCOPES:
                track = timeline(data['extract'], data['transport'], scope)
                np.testing.assert_allclose(joint_at(track, 0), track['position'][0], atol=1e-7)
                np.testing.assert_allclose(joint_at(track, track['duration']+1), track['position'][-1], atol=1e-7)
                assert np.all(np.diff(track['times']) > 0)
    # A deliberately bad seam must fail rather than fabricate a connecting animation.
    q = np.zeros((2, 15)); t = np.array([0., 1.])
    try:
        timeline(dict(position=q, times=t), dict(position=q+1, times=t), SCOPES[0])
    except ValueError:
        pass
    else:
        raise AssertionError('Bad seam accepted')
    frozen_urdf()
    print('CPU SELF CHECK PASS: 21 paired cases, 126 scoped timelines, native timestamps/endpoints/seams and frozen display model', flush=True)


def main(args):
    import viser
    import yourdfpy
    from viser.extras import ViserUrdf

    report = read(args.report)
    if args.self_check:
        self_check(args.results, report)
        return
    trials = {(t['case'], t['seed']): t for t in report['trials']}
    representatives = {}
    for case in report['cases']:
        valid = [t for t in report['trials'] if t['case'] == case['case'] and t['paired']]
        representatives[case['case']] = min(valid, key=lambda t: abs(t['improvement_pct']-case['improvement_median_pct']))['seed']
    labels = {value: key for key, value in LABELS.items()}
    cached = lru_cache(maxsize=4)(lambda case, seed: load_pair(args.results, trials[case, seed]))
    # Resolve a representative before binding a port; missing archives must not leave a server open.
    cached('top_outer', representatives['top_outer'])
    args.status.parent.mkdir(parents=True, exist_ok=True)
    server = viser.ViserServer(host=args.host, port=args.port, label='PR82三箱对 · 抽离/运输时间对照')
    server.scene.set_up_direction('+z')
    server.initial_camera.position = (-4.8, -5.0, 3.6)
    server.initial_camera.look_at = (-.3, 0., 1.)
    server.initial_camera.up_direction = (0., 0., 1.)
    server.scene.add_grid('/floor', width=8, height=8, plane='xy')
    robots, models, joints, nodes, held, trails = {}, {}, {}, {}, {}, {}
    colors = {'baseline': (65, 155, 235), 'xyz': (245, 150, 55)}
    for name, offset, title in [('baseline', (0., 1.8, 0.), '基线 · 固定35cm抽离'), ('xyz', (0., -1.8, 0.), 'XYZ空域 · 实测候选')]:
        server.scene.add_frame('/'+name, position=offset, show_axes=False)
        server.scene.add_label('/'+name+'/title', text=title, position=(-.7, 0., 2.4))
        models[name] = yourdfpy.URDF.load(frozen_urdf(), load_meshes=True, build_scene_graph=True)
        robots[name] = ViserUrdf(server, models[name], root_node_name='/'+name+'/robot')
        joints[name] = list(robots[name].get_actuated_joint_limits())
        nodes[name], held[name], trails[name] = [], {}, []
    server.gui.add_markdown('# 三个双箱 · 抽离＋运输对照\n\n**离线实测轨迹回放，不重新规划、不控制硬件。** 两边共用秒表，1×为原TimeOpt时间；先完成者停住等待。完整原URDF外观＋规划场景cuboid，非缺失冻结前端的部署认证。')
    task = server.gui.add_dropdown('箱对', options=tuple(LABELS.values()), initial_value=LABELS['top_outer'])
    seed = server.gui.add_dropdown('随机种子', options=tuple(str(s) for s in (11,23,37,51,71,89,101)), initial_value=str(representatives['top_outer']))
    scope = server.gui.add_dropdown('阶段', options=SCOPES, initial_value=SCOPES[0])
    autoplay = server.gui.add_checkbox('自动轮播三个代表箱对', initial_value=True)
    play = server.gui.add_checkbox('播放', initial_value=True)
    speed = server.gui.add_slider('播放倍速', min=.25, max=2., step=.25, initial_value=1.)
    paths = server.gui.add_checkbox('显示箱心轨迹（细线）', initial_value=True)
    clock = server.gui.add_slider('共同回放时间 / s', min=0., max=5., step=.01, initial_value=0.)
    details = server.gui.add_markdown('')
    current_info = server.gui.add_markdown('')
    pending = {'load': 'case'}
    state = {}

    @task.on_update
    def task_changed(event):
        if event.client is not None:
            autoplay.value = False
            pending['load'] = 'case'
    @seed.on_update
    def seed_changed(event):
        if event.client is not None:
            autoplay.value = False
            pending['load'] = 'seed'
    @scope.on_update
    def scope_changed(event):
        if event.client is not None:
            autoplay.value = False
            pending['load'] = 'scope'
    @clock.on_update
    def clock_changed(event):
        if event.client is not None:
            play.value = False
            pending['seek'] = clock.value
    @server.on_client_connect
    def connected(client):
        pending['seek'] = 0.
        play.value = True

    def apply(name, q):
        mapping = dict(zip(ACTIVE_JOINTS, q))
        robots[name].update_cfg(np.array([mapping.get(joint, 0.) for joint in joints[name]]))
        for side, node in held[name].items():
            matrix = models[name].get_transform(side+'_tool0', models[name].base_link) @ np.asarray(state['pair']['source']['visual_tool_to_box'][side])
            node.position = matrix[:3,3]
            node.wxyz = np.roll(Rotation.from_matrix(matrix[:3,:3]).as_quat(), 1)

    def load(reason):
        case = labels[task.value]
        if reason == 'case':
            seed.value = str(representatives[case])
        pair = cached(case, int(seed.value))
        state.update(pair=pair, case=case, seed=int(seed.value), stamp=0., ended_at=None)
        state['tracks'] = {name: timeline(data['extract'], data['transport'], scope.value) for name,data in pair['data'].items()}
        state['duration'] = max(track['duration'] for track in state['tracks'].values())
        clock.max = max(.01, state['duration'])
        targets = set(pair['source']['task'].values())
        snapshot = pair['source']['visual_scene_snapshot']
        for name in robots:
            for node in nodes[name]+list(held[name].values())+trails[name]:
                node.remove()
            nodes[name], held[name], trails[name] = [], {}, []
            for item in snapshot['objects']:
                if item['object_id'] in {f'wall_box_{cell:02d}' for cell in targets}:
                    continue
                node = server.scene.add_box('/'+name+'/world/'+item['object_id'], dimensions=tuple(item['dimensions_m']),
                    position=tuple(item['pose']['position']), wxyz=tuple(item['pose']['quaternion_wxyz']),
                    color=(175,135,90) if item['object_id'].startswith('task_fill') else (110,130,145), opacity=.65)
                nodes[name].append(node)
            for side, cell in pair['source']['task'].items():
                box = next(item for item in snapshot['objects'] if item['object_id']==f'wall_box_{cell:02d}')
                held[name][side] = server.scene.add_box('/'+name+'/held/'+side, dimensions=tuple(box['dimensions_m']), color=colors[name])
                for phase in ('extract','transport'):
                    data = pair['data'][name][phase]
                    points = []
                    for q in data['position'][np.linspace(0,len(data['position'])-1,65).astype(int)]:
                        mapping = dict(zip(ACTIVE_JOINTS,q))
                        models[name].update_cfg(np.array([mapping.get(joint,0.) for joint in joints[name]]))
                        matrix = models[name].get_transform(side+'_tool0',models[name].base_link) @ np.asarray(pair['source']['visual_tool_to_box'][side])
                        points.append(matrix[:3,3])
                    points = np.asarray(points)
                    node = server.scene.add_line_segments('/'+name+'/path/'+side+'/'+phase,
                        points=np.stack((points[:-1],points[1:]),axis=1), colors=colors[name], line_width=2. if phase=='extract' else 1.)
                    trails[name].append(node)
        rows = []
        for name,label in [('baseline','基线'),('xyz','XYZ')]:
            data = pair['data'][name]
            e,t = (float(data[phase]['times'][-1]) for phase in ('extract','transport'))
            rows.append(f'|{label}|{e:.3f}|{t:.3f}|{e+t:.3f}|')
        trial = pair['trial']
        failed = [s['phase'] for s in pair['record']['baseline']['stages'] if not s.get('trajectory_optimization',{}).get('solver_success',True) or not s.get('time_optimization',{}).get('dynamic_valid',False)]
        details.content = f"## {LABELS[case]} · seed {seed.value}\n\n当前为单种子实测，不是7种子中位数。\n\n|方案|抽离s|运输s|携箱合计s|\n|---|---:|---:|---:|\n"+'\n'.join(rows)+f"\n\n抽离位移（m）：`{trial['displacement_m']}`\n\n已移除占位格：`{pair['record']['removed_cells']}`（0-based）。\n\n基线未通过阶段：`{failed}`；本种子{'属于' if trial['paired'] else '**不属于**'}严格速度对照。\n\n候选来源：`{trial['selected_campaign']}` / rank `{trial['selected_rank']}`；保留原60°/s、jerk500与载箱扭矩限制。"
        play.value = True
        render(0.)

    def render(stamp):
        lines = []
        with server.atomic():
            for name,label in [('baseline','基线'),('xyz','XYZ')]:
                track = state['tracks'][name]
                apply(name,joint_at(track,stamp))
                phase = next((label for label,end in track['boundaries'] if stamp < end), '已完成，等待另一方案')
                lines.append(f'**{label}**：{min(stamp,track["duration"]):.2f}/{track["duration"]:.2f}s · {phase}')
                for node in trails[name]:
                    node.visible = paths.value
            clock.value = min(stamp,state['duration'])
            current_info.content = '\n\n'.join(lines)

    if args.smoke_test:
        for case in LABELS:
            task.value = LABELS[case]
            load('case')
            for value in (0.,state['duration']/2,state['duration']):
                render(value)
        print('VISER SMOKE PASS: three full-mesh robot pairs, world boxes, payload FK and original-time start/mid/end states',flush=True)
        server.stop()
        return
    print(f'READY http://{args.host}:{args.port} CPU-only saved TimeOpt replay; no GPU/planning/hardware',flush=True)
    last, status_at = time.perf_counter(), 0.
    while True:
        now = time.perf_counter()
        reason = pending.pop('load',None)
        if reason is not None:
            load(reason)
            last = time.perf_counter()
            now = last
        if 'seek' in pending:
            state['stamp'] = pending.pop('seek')
            state['ended_at'] = None
        if play.value and server.get_clients():
            state['stamp'] = min(state['duration'], state['stamp']+(now-last)*speed.value)
        last = now
        if server.get_clients():
            render(state['stamp'])
        if state['stamp'] >= state['duration'] and play.value and server.get_clients():
            if state['ended_at'] is None:
                state['ended_at'] = now
            if now-state['ended_at'] > 1.2:
                if autoplay.value:
                    keys = list(LABELS)
                    task.value = LABELS[keys[(keys.index(state['case'])+1)%len(keys)]]
                    pending['load'] = 'case'
                else:
                    play.value = False
        if now-status_at > 1.:
            status_at = now
            args.status.write_text(json.dumps(dict(case=state['case'], seed=state['seed'], scope=scope.value,
                time_s=state['stamp'], max_s=state['duration'], clients=len(server.get_clients()),
                cpu_only=True, files=state['pair']['files']),ensure_ascii=False,indent=2))
        time.sleep(.025 if server.get_clients() else .5)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results',type=Path,default=ROOT/'generated/free_space_extract')
    parser.add_argument('--report',type=Path,default=ROOT/'evidence/free_space_extract_pr82_20261111/report.json')
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=8171)
    parser.add_argument('--status',type=Path,default=ROOT/'generated/free_space_extract/viewer-status.json')
    parser.add_argument('--self-check',action='store_true')
    parser.add_argument('--smoke-test',action='store_true')
    main(parser.parse_args())
