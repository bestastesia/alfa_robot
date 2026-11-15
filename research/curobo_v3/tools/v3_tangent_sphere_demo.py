import argparse
import json
from pathlib import Path
import queue
import time
import traceback

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from v3_tcp_navigation import alignment_metrics, box_arrays, forward_route, peak_normal_acceleration, route, segment_clear


def parameterize(centers, matrices, radius, normal_limit, speed=.5):
    if normal_limit <= 0:
        raise ValueError('向心加速度上限必须大于0')
    reference = np.linspace(0., 1., len(centers))
    times = np.linspace(0., 1., 512)
    progress = 10*times**3-15*times**4+6*times**5
    sampled_centers = np.column_stack([np.interp(progress, reference, centers[:, axis]) for axis in range(3)])
    rotations = Slerp(reference, Rotation.from_matrix(matrices))(progress)
    tcp = sampled_centers + radius*rotations.as_matrix()[:, :, 2]
    step = times[1]-times[0]
    velocities = np.diff(tcp, axis=0)/step
    acceleration = np.diff(velocities, axis=0)/step
    angular_velocity = rotations[:-1].apply((rotations[:-1].inv()*rotations[1:]).as_rotvec())/step
    normal = peak_normal_acceleration(tcp, times)
    duration = max(1., np.linalg.norm(velocities, axis=1).max()/speed,
                   np.sqrt(np.linalg.norm(acceleration, axis=1).max()/.5),
                   np.sqrt(normal/normal_limit),
                   np.linalg.norm(angular_velocity, axis=1).max()/np.deg2rad(45),
                   np.sqrt(np.linalg.norm(np.diff(angular_velocity, axis=0)/step, axis=1).max()/np.deg2rad(90)))*1.01
    return sampled_centers, tcp, rotations, dict(times_s=(times*duration).tolist(), duration_s=float(duration),
        peak_normal_acceleration_m_s2=peak_normal_acceleration(tcp, times*duration),
        peak_speed_m_s=float(np.linalg.norm(velocities, axis=1).max()/duration))


def plan(snapshot, objects, radius=.08, margin=.005, offset=.08, forward=True, normal_limit=.01):
    began = time.perf_counter()
    paths = {}
    boxes = box_arrays(objects, radius + margin)
    for side in ('left', 'right'):
        started = time.perf_counter()
        initial = np.asarray(snapshot['tcp_world'][side])
        target = np.asarray(snapshot['targets'][side]).copy()
        target[:3, 3] -= offset * target[:3, 2]
        start_center = initial[:3, 3] - radius * initial[:3, 2]
        goal_center = target[:3, 3] - radius * target[:3, 2]
        skeleton = None
        expanded = 0
        for lift in (0., .15, .3, .5, .7):
            candidate = np.stack([start_center, start_center+[0,0,lift], goal_center+[0,0,lift], goal_center]) if lift else np.stack([start_center,goal_center])
            if all(segment_clear(first,second,boxes) for first,second in zip(candidate[:-1],candidate[1:])):
                keep = np.r_[True,np.linalg.norm(np.diff(candidate,axis=0),axis=1)>1e-9]
                skeleton=candidate[keep]
                break
        if skeleton is None:
            skeleton, expanded = route(start_center, goal_center, objects, radius + margin, budget_s=.2)
        searched = time.perf_counter()
        if np.linalg.norm(np.diff(skeleton,axis=0),axis=1).sum()<1e-8:
            centers=np.repeat(start_center[None],256,axis=0)
            matrices=Slerp([0,1],Rotation.from_matrix([initial[:3,:3],target[:3,:3]]))(np.linspace(0,1,256)).as_matrix()
            strength=None
        elif forward:
            centers, matrices, strength = forward_route(skeleton, initial[:3, :3], target[:3, :3], boxes)
        else:
            distances = np.r_[0., np.cumsum(np.linalg.norm(np.diff(skeleton, axis=0), axis=1))]
            progress = np.linspace(0, 1, 256)
            centers = np.column_stack([np.interp(progress * distances[-1], distances, skeleton[:, axis]) for axis in range(3)])
            matrices = Slerp([0, 1], Rotation.from_matrix([initial[:3, :3], target[:3, :3]]))(progress).as_matrix()
            strength = None
        tcp = centers + radius * matrices[:, :, 2]
        geometry_done = time.perf_counter()
        centers, tcp, rotations, timing = parameterize(centers, matrices, radius, normal_limit)
        matrices = rotations.as_matrix()
        quaternions = np.roll(Rotation.from_matrix(matrices).as_quat(), 1, axis=1)
        if not all(segment_clear(first, second, boxes) for first, second in zip(centers[:-1], centers[1:])):
            raise RuntimeError('球心连续折线碰撞验收失败')
        if not np.allclose(tcp[[0, -1]], [initial[:3, 3], target[:3, 3]], atol=1e-7):
            raise RuntimeError('TCP端点不一致')
        paths[side] = dict(centers=centers.tolist(), positions=tcp.tolist(), quaternions_wxyz=quaternions.tolist(),
                           search_ms=(searched-started)*1000, total_ms=(time.perf_counter()-started)*1000,
                           geometry_ms=(geometry_done-started)*1000, **timing,
                           expanded=expanded, tangent_strength=strength,
                           velocity_z_alignment=alignment_metrics(tcp, quaternions),
                           length_m=float(np.linalg.norm(np.diff(tcp, axis=0), axis=1).sum()),
                           rotation_deg=float(np.rad2deg((Rotation.from_matrix(matrices[:-1]).inv()*Rotation.from_matrix(matrices[1:])).magnitude().sum())))
    return dict(paths=paths, total_ms=(time.perf_counter()-began)*1000, radius=radius, margin=margin, offset=offset, normal_limit_m_s2=normal_limit,
                mode='forward' if forward else 'shortest_rotation', scene_objects=objects,
                scope='Independent proxy sphere paths; no full tool/robot or mutual sphere validation')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', type=int, default=8187)
    parser.add_argument('--compute-only', action='store_true')
    args = parser.parse_args()
    snapshot = json.loads(args.snapshot.read_text())
    objects = json.loads(args.scene.read_text())['scene_objects']
    if args.compute_only:
        result = plan(snapshot, objects)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False))
        print(json.dumps({key: value for key, value in result.items() if key not in ('paths', 'scene_objects')}))
        for side, entry in result['paths'].items():
            print(side, {key: value for key, value in entry.items() if key not in ('positions', 'centers', 'quaternions_wxyz')})
        return
    import trimesh
    import viser

    server = viser.ViserServer(host='127.0.0.1', port=args.port, label='球面切点TCP导航')
    server.scene.set_up_direction('+z')
    server.scene.add_grid('/ground', width=5, height=5)
    server.gui.add_markdown('球在吸盘面后侧：球心=TCP−r·Z，TCP=球心+r·Z。青线为球心，绿线为TCP；黄色连线表示半径，切面圆盘表示吸盘面。仅球代理避障，两手独立路线，无整机IK、真实球组或双手互碰验收。')
    radius = server.gui.add_slider('代理球半径(m)', min=.02, max=.2, step=.005, initial_value=.08)
    offset = server.gui.add_slider('吸附面外侧距离(m)', min=.02, max=.25, step=.01, initial_value=.08)
    margin = server.gui.add_slider('安全余量(m)', min=.0, max=.02, step=.001, initial_value=.005)
    normal = server.gui.add_slider('TCP向心加速度上限(m/s²)', min=.001, max=.5, step=.001, initial_value=.01)
    server.gui.add_markdown('向心上限约束TCP位置轨迹的法向加速度；减小后沿同一路线减速，不限制吸盘自身转角，也不自动扩大弯曲半径。修改后点击重新规划。')
    mode = server.gui.add_dropdown('朝向方式', options=['沿路线前进＋平行运输朝向', '起终朝向最短插值'], initial_value='沿路线前进＋平行运输朝向')
    compute = server.gui.add_button('重新规划双末端')
    side_control = server.gui.add_dropdown('查看末端', options=['left', 'right'], initial_value='left')
    frame = server.gui.add_slider('轨迹帧', min=0, max=1, step=1, initial_value=0)
    play = server.gui.add_checkbox('播放（几何回放，非执行限速）', initial_value=False)
    real_time = server.gui.add_checkbox('按限速后真实时长播放', initial_value=True)
    status = server.gui.add_markdown('')
    details = server.gui.add_markdown('')
    for index, item in enumerate(objects):
        server.scene.add_box('/world/'+str(index), position=item['position'], wxyz=item['quaternion_wxyz'], dimensions=item['dimensions'], color=(180,150,110), opacity=.12)
    sphere = server.scene.add_icosphere('/proxy', radius=.08, color=(60,175,245), opacity=.23, subdivisions=3)
    tool = server.scene.add_frame('/tcp', axes_length=.15, axes_radius=.003)
    disk_mesh = trimesh.creation.cylinder(radius=.045, height=.003)
    disk_mesh.visual.vertex_colors = (230,170,50,255)
    server.scene.add_mesh_trimesh('/tcp/tangent_face', mesh=disk_mesh)
    radius_line = server.scene.add_line_segments('/radius', points=np.zeros((1, 2, 3)), colors=(255,220,40), line_width=5)
    state = dict(result=None, handles=[], key=None, rendered=None)
    requests = queue.SimpleQueue()

    @server.on_client_connect
    def connected(client):
        client.camera.position=(-1.3, -3, 2.3)
        client.camera.look_at=(1.1, -.2, .9)

    def render():
        result = state['result']
        if result is None:
            return
        side = side_control.value
        entry = result['paths'][side]
        key = (id(result), side)
        if key != state['key']:
            for handle in state['handles']:
                handle.remove()
            handles = []
            for name, values, color in [('center', entry['centers'], (50,180,245)), ('tcp', entry['positions'], (30,230,110))]:
                points = np.asarray(values)
                handles.append(server.scene.add_line_segments('/route/'+name, points=np.stack([points[:-1],points[1:]],axis=1), colors=color, line_width=3))
            for index in np.linspace(0, len(entry['positions'])-1, 16, dtype=int):
                handles.append(server.scene.add_frame('/orientation/'+str(index), position=entry['positions'][index], wxyz=entry['quaternions_wxyz'][index], axes_length=.055, axes_radius=.001))
            state['handles'], state['key'] = handles, key
            frame.max=len(entry['positions'])-1
            frame.value=0
        index=int(frame.value)
        rendered = (id(result), side, index)
        if state['rendered'] == rendered:
            return
        state['rendered'] = rendered
        center=np.asarray(entry['centers'][index])
        position=np.asarray(entry['positions'][index])
        sphere.position=center
        tool.position=position
        tool.wxyz=entry['quaternions_wxyz'][index]
        radius_line.points=np.array([[center, position]])
        radius_line.visible=True
        details.content=f"{side}：TCP路线长{entry['length_m']:.3f}m，累计旋转{entry['rotation_deg']:.1f}°；移动方向与＋Z平均夹角{entry['velocity_z_alignment']['mean_deg']:.1f}°。当前球心→TCP距离{np.linalg.norm(position-center):.3f}m。\n\n向心峰值{entry['peak_normal_acceleration_m_s2']:.5f}m/s² ≤ {result['normal_limit_m_s2']:.3f}；限速时长{entry['duration_s']:.2f}s。动态限值为512点离散差分验收。"

    def calculate():
        play.value=False
        compute.disabled=True
        state['result']=None
        sphere.visible=False
        tool.visible=False
        radius_line.visible=False
        for handle in state['handles']:
            handle.remove()
        state['handles']=[]
        try:
            status.content='正在规划并计算TCP限速时间……'
            result=plan(snapshot, objects, radius.value, margin.value, offset.value, mode.value.startswith('沿'), normal.value)
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(result, ensure_ascii=False))
            state['result']=result
            state['key']=None
            sphere.radius=result['radius']
            sphere.visible=True
            tool.visible=True
            status.content=f"**双末端代理球规划通过**；计算共{result['total_ms']:.1f}ms（不含页面绘制和文件保存），目标200ms。\n\n"+'\n\n'.join(f"{side}：搜索{entry['search_ms']:.1f}ms，含朝向生成/验收{entry['total_ms']:.1f}ms，扩展{entry['expanded']}节点。" for side,entry in result['paths'].items())+'\n\n连续球心折线按半径＋余量扩张长方体保守验收；TCP本身不是避障球心，未证明真实吸盘完全被球覆盖。'
            render()
        except Exception as error:
            traceback.print_exc()
            state['result']=None
            status.content='**未通过，不播放旧结果：** '+str(error)
        finally:
            compute.disabled=False

    @compute.on_click
    def recompute(event):
        requests.put(True)

    calculate()
    tick=time.monotonic()
    while True:
        try:
            requests.get_nowait()
            calculate()
            tick=time.monotonic()
        except queue.Empty:
            pass
        entry = state['result']['paths'][side_control.value] if state['result'] else None
        interval = entry['duration_s']/(len(entry['positions'])-1) if entry and real_time.value else 1/60
        if play.value and entry and time.monotonic()-tick>interval:
            advance=max(1,int((time.monotonic()-tick)/interval))
            frame.value=min(frame.value+advance, frame.max)
            tick+=advance*interval
            if frame.value==frame.max:
                play.value=False
        if not play.value:
            tick=time.monotonic()
        render()
        time.sleep(.015)


if __name__ == '__main__':
    main()
