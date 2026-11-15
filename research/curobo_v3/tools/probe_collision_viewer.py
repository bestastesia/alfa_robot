import argparse
import json
from pathlib import Path
import time

import numpy as np

from probe_touch_ik import ROOT


def export(path):
    from probe_touch_motion import ProbeCollision
    from curobo.types import JointState
    engine = ProbeCollision()
    checker = engine.checker
    config = checker.kinematics.config.kinematics_config
    pairs = checker.kinematics.get_self_collision_config().collision_pairs.cpu().numpy().astype(int)
    padding = checker.kinematics.get_self_collision_config().sphere_padding.cpu().numpy().reshape(-1)
    links = {}
    for name in config.link_name_to_idx_map:
        for index in config.get_sphere_index_from_link_name(name).cpu().tolist():
            links[index] = name
    output = {}
    for label, left in [('左臂零位', [0.] * 7), ('左臂Home', engine.home['left_deg'])]:
        values = {}
        for name in engine.names:
            if name == 'updown':
                values[name] = -.3
            elif name.startswith('left_joint'):
                values[name] = float(np.deg2rad(left[int(name[-1]) - 1]))
            elif name.startswith('right_joint'):
                values[name] = float(np.deg2rad(engine.home['right_deg'][int(name[-1]) - 1]))
            else:
                values[name] = 0.
        q = engine.torch.tensor([[list(values.values())]], device='cuda', dtype=engine.torch.float32)
        state = checker.kinematics.compute_kinematics(JointState.from_position(q, joint_names=engine.names))
        spheres = state.robot_spheres.cpu().numpy().reshape(-1, 4)
        collisions = []
        for first, second in pairs:
            if spheres[first, 3] <= 0 or spheres[second, 3] <= 0:
                continue
            distance = float(np.linalg.norm(spheres[first, :3] - spheres[second, :3]))
            raw_gap = distance - spheres[first, 3] - spheres[second, 3]
            padded_gap = raw_gap - padding[first] - padding[second]
            if padded_gap < -1e-6:
                collisions.append(dict(indices=[int(first), int(second)], links=[links.get(first, '?'), links.get(second, '?')],
                    raw_overlap_mm=float(-raw_gap * 1000), padded_overlap_mm=float(-padded_gap * 1000),
                    padding_mm=[float(padding[first] * 1000), float(padding[second] * 1000)]))
        collisions.sort(key=lambda row: -row['padded_overlap_mm'])
        output[label] = dict(values=values, spheres=spheres.tolist(), collisions=collisions,
            self_cost=float(checker.get_self_collision(state.robot_spheres).sum().item()))
    path.write_text(json.dumps(output, ensure_ascii=False, indent=2))
    for label, record in output.items():
        print(label, record['self_cost'], 'pairs', len(record['collisions']), json.dumps(record['collisions'][:8], ensure_ascii=False), flush=True)


def view(path, port):
    import viser
    from viser.extras import ViserUrdf
    import yourdfpy
    data = json.loads(path.read_text())
    model = yourdfpy.URDF.load(ROOT / 'generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf', load_meshes=True)
    server = viser.ViserServer(host='127.0.0.1', port=port, label='自碰撞球对定位 · 离线诊断')
    robot = ViserUrdf(server, model, root_node_name='/robot', mesh_color_override=(150, 190, 220, .23))
    server.scene.add_grid('/floor', width=3, height=3)
    selector = server.gui.add_dropdown('检查状态', options=list(data))
    pair_selector = server.gui.add_dropdown('碰撞球对', options=['加载中'])
    zoom = server.gui.add_button('近看选中碰撞球对')
    all_spheres = server.gui.add_checkbox('显示全部碰撞球', initial_value=False)
    server.gui.add_markdown('离线构造状态：Updown=-0.3m，右臂固定配置Home；不是实机当前姿态。红/黄球为选中球对，紫线连接球心。球体重叠不等于实体网格穿透，未修改碰撞规则或控制实机。')
    report = server.gui.add_markdown('')
    nodes = []
    center = np.zeros(3)
    dirty = [True]

    def labels(record):
        return [f"{index + 1}: {' ↔ '.join(row['links'])} · 球包络重叠{row['padded_overlap_mm']:.2f}mm" for index, row in enumerate(record['collisions'])] or ['无重叠球对']

    @selector.on_update
    def changed(_event):
        pair_selector.options = labels(data[selector.value])
        pair_selector.value = pair_selector.options[0]
        dirty[0] = True

    @pair_selector.on_update
    @all_spheres.on_update
    def update(_event):
        dirty[0] = True

    @zoom.on_click
    def focus(event):
        event.client.camera.look_at = center
        event.client.camera.position = center + np.array([.38, .38, .23])

    @server.on_client_connect
    def connected(client):
        client.camera.position = (2.6, 2.8, 2.1)
        client.camera.look_at = (.25, 0., 1.)
        client.camera.up_direction = (0., 0., 1.)

    changed(None)
    print(f'READY http://127.0.0.1:{port}/', flush=True)
    while True:
        if dirty[0]:
            dirty[0] = False
            for node in nodes:
                node.remove()
            nodes.clear()
            record = data[selector.value]
            robot.update_cfg(np.array([record['values'].get(name, 0.) for name in model.actuated_joint_names]))
            spheres = np.asarray(record['spheres'])
            if all_spheres.value:
                for index, sphere in enumerate(spheres):
                    if sphere[3] > 0:
                        nodes.append(server.scene.add_icosphere(f'/spheres/{index}', position=sphere[:3], radius=float(sphere[3]), color=(90, 150, 240), opacity=.12))
            rows = record['collisions']
            report.content = f"**{selector.value}：{len(rows)}个重叠球对**\n\n自碰撞cost={record['self_cost']:.8f}（不是米）。"
            if rows:
                index = pair_selector.options.index(pair_selector.value)
                row = rows[index]
                selected = spheres[row['indices']]
                center = selected[:, :3].mean(axis=0)
                for offset, (sphere, link) in enumerate(zip(selected, row['links'])):
                    nodes.append(server.scene.add_icosphere(f'/selected/{offset}', position=sphere[:3], radius=float(sphere[3]), color=(255, 50, 30) if offset == 0 else (255, 200, 20), opacity=.65))
                    nodes.append(server.scene.add_label(f'/labels/{offset}', position=sphere[:3] + [0., 0., .05 + offset * .04], text=link))
                nodes.append(server.scene.add_line_segments('/pair', points=selected[None, :, :3], colors=(230, 30, 230), line_width=5))
                report.content += f"\n\n**{' ↔ '.join(row['links'])}**\n\n球编号{row['indices']}；原始球重叠{row['raw_overlap_mm']:.3f}mm；含padding重叠{row['padded_overlap_mm']:.3f}mm；padding={row['padding_mm']}mm。"
        time.sleep(.03)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=Path, default=Path('/tmp/probe_collision_pairs.json'))
    parser.add_argument('--export', action='store_true')
    parser.add_argument('--port', type=int, default=8161)
    args = parser.parse_args()
    export(args.data) if args.export else view(args.data, args.port)
