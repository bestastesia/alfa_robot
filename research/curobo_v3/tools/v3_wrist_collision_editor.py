import argparse
from concurrent.futures import ThreadPoolExecutor
import copy
import gzip
import hashlib
import json
from pathlib import Path
import queue
import time

import numpy as np
from scipy.spatial.transform import Rotation
import viser
import yaml
import yourdfpy


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'generated/zero_model_candidate_20261018/fixed.yaml'
DRAFT = ROOT / 'generated/wrist_collision_editor/draft.json'
COLORS = {5: (255, 155, 40), 6: (130, 210, 90), 7: (40, 170, 255)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8244)
    args = parser.parse_args()
    config = yaml.safe_load(CONFIG.read_text())['kinematics']
    model = yourdfpy.URDF.load(config['urdf_path'])
    links = [f'{side}_link{number}' for side in ('left', 'right') for number in (5, 6, 7)]
    original = {link: copy.deepcopy(config['collision_spheres'][link]) for link in links}
    edited = copy.deepcopy(original)
    reviews = json.loads((ROOT / 'generated/frozen_cycle_release/failure_review.json').read_text())['records']
    source = json.loads(gzip.decompress((ROOT / 'generated/frozen_cycle_release/060.json.gz').read_bytes()))
    active = [name for name in source['joint_names'] if name == 'updown' or '_joint' in name]
    if len(active) != 15:raise ValueError('Expected 15 active joints')
    server = viser.ViserServer(host='127.0.0.1', port=args.port, label='腕部碰撞球调参 · link5/6/7')
    server.scene.set_up_direction('+z')
    server.gui.main_panel.dock_right()
    server.gui.add_markdown('**腕部自动拟合**：调整整个link的拟合参数，再点击重新拟合。橙=5、绿=6、蓝=7。结果先预览和保存草稿，正式规划配置尚未替换。')
    side = server.gui.add_dropdown('手臂', options=['right', 'left'])
    case = server.gui.add_dropdown('姿态来源', options=[item['label'] for item in reviews])
    joint_controls = {}
    with server.gui.add_folder('腕部关节角度'):
        for number in (5, 6, 7):
            limit = model.joint_map[f'right_joint{number}'].limit
            joint_controls[number] = server.gui.add_slider(f'J{number} / 度', min=float(np.rad2deg(limit.lower)), max=float(np.rad2deg(limit.upper)), step=.1, initial_value=0.)
    link_choice = server.gui.add_dropdown('编辑link', options=['5', '6', '7'])
    isolate_link = server.gui.add_checkbox('只显示当前link', initial_value=False)
    parameters = {link:dict(count=len(original[link]),iterations=200,coverage=1000.,protrusion=10.,margin=0.) for link in links}
    fit_reports = {}
    with server.gui.add_folder('MorphIt自动拟合参数', expand_by_default=True):
        controls = {
            'count':server.gui.add_number('目标球数',initial_value=15,min=1,max=128,step=1),
            'iterations':server.gui.add_number('优化迭代次数',initial_value=200,min=1,max=3000,step=10),
            'coverage':server.gui.add_number('覆盖权重',initial_value=1000.,min=0.,max=100000.,step=10.),
            'protrusion':server.gui.add_number('外凸惩罚权重',initial_value=10.,min=0.,max=100000.,step=1.),
            'margin':server.gui.add_number('拟合后半径余量 / mm',initial_value=0.,min=0.,max=20.,step=.1),
        }
        server.gui.add_markdown('球数控制精细程度（算法会裁剪冗余球，实际数量可能更少）；覆盖权重越大越重视包住模型；外凸惩罚越大越抑制球体伸出外形；余量为拟合完成后统一增加的半径。每个link独立保存参数。')
        refit = server.gui.add_button('重新拟合当前link')
        refit_all = server.gui.add_button('重新拟合当前手臂5/6/7')
        fit_status = server.gui.add_markdown('当前显示原始球模型，尚未重新拟合。')
        reset_ball = server.gui.add_button('恢复当前link原模型')
        reset_all = server.gui.add_button('恢复全部原值')
    meshes_visible = server.gui.add_checkbox('显示真实外观', initial_value=True)
    mesh_opacity = server.gui.add_slider('外观透明度', min=.05, max=1., step=.05, initial_value=.4)
    balls_visible = server.gui.add_checkbox('显示碰撞球', initial_value=True)
    ball_opacity = server.gui.add_slider('碰撞球透明度', min=.05, max=1., step=.05, initial_value=.35)
    axes_visible = server.gui.add_checkbox('显示link坐标轴', initial_value=False)
    focus = server.gui.add_button('居中腕部')
    save = server.gui.add_button('保存调参草稿')
    restore = server.gui.add_button('加载上次草稿', disabled=not DRAFT.exists())
    info = server.gui.add_markdown('')
    saved = server.gui.add_markdown('')
    roots = {}
    meshes = []
    balls = {}
    fit_meshes = {}
    axis_nodes = []
    for link in links:
        pieces = []
        roots[link] = server.scene.add_frame('/wrist/'+link, show_axes=False)
        axis_nodes.append(server.scene.add_frame('/wrist/'+link+'/axes', axes_length=.08, axes_radius=.0015, visible=False))
        for name, mesh in model.scene.geometry.items():
            if model.scene.graph.transforms.parents.get(name) != link:
                continue
            geometry = mesh.copy()
            geometry.apply_transform(model.get_transform(name, link))
            pieces.append(geometry)
            node = server.scene.add_mesh_simple('/wrist/'+link+'/mesh/'+name, vertices=geometry.vertices, faces=geometry.faces, color=(170,175,185), opacity=.4)
            meshes.append(node)
        import trimesh
        fit_meshes[link] = trimesh.util.concatenate(pieces)
        for index, sphere in enumerate(original[link]):
            balls[link,index] = server.scene.add_icosphere('/wrist/'+link+'/ball'+str(index), radius=1., scale=sphere['radius'], position=sphere['center'], color=COLORS[int(link[-1])], opacity=.35)
    marker = server.scene.add_icosphere('/overlap', radius=.002, color=(255,30,30), visible=False)
    pending = queue.SimpleQueue()
    state = {'values': {}, 'center': np.zeros(3), 'fitting':False}
    executor = ThreadPoolExecutor(max_workers=1)

    def selected():
        return side.value+'_link'+link_choice.value

    def sync_controls():
        for key,control in controls.items():control.value=parameters[selected()][key]

    def draw():
        for number, control in joint_controls.items():
            state['values'][f'{side.value}_joint{number}'] = np.deg2rad(control.value)
        model.update_cfg({name:value for name,value in state['values'].items() if name in model.actuated_joint_names})
        anchor = model.get_transform(side.value+'_link5')
        offset = anchor[:3,3]
        transformed = {}
        for link, root in roots.items():
            matrix = model.get_transform(link).copy()
            matrix[:3,3] -= offset
            root.position = matrix[:3,3]
            root.wxyz = np.roll(Rotation.from_matrix(matrix[:3,:3]).as_quat(),1)
            root.visible = link.startswith(side.value) and (not isolate_link.value or link==selected())
            for key in list(balls):
                if key[0]==link and key[1]>=len(edited[link]):balls.pop(key).remove()
            for index, sphere in enumerate(edited[link]):
                if (link,index) not in balls:
                    balls[link,index]=server.scene.add_icosphere('/wrist/'+link+'/ball'+str(index),radius=1.)
                node = balls[link,index]
                node.position = sphere['center']
                node.scale = sphere['radius']
                node.opacity = ball_opacity.value
                node.visible = balls_visible.value
                node.color = COLORS[int(link[-1])]
            transformed[link] = np.array([matrix[:3,:3]@np.array(sphere['center'])+matrix[:3,3] for sphere in edited[link]])
        for node in meshes:
            node.visible = meshes_visible.value
            node.opacity = mesh_opacity.value
        for node in axis_nodes:
            node.visible = axes_visible.value
        first, second = side.value+'_link5', side.value+'_link7'
        distances = np.linalg.norm(transformed[first][:,None]-transformed[second][None],axis=-1)
        radii_first = np.array([sphere['radius'] for sphere in edited[first]])
        radii_second = np.array([sphere['radius'] for sphere in edited[second]])
        overlap = radii_first[:,None]+radii_second[None]-distances
        first_index, second_index = np.unravel_index(np.argmax(overlap),overlap.shape)
        maximum = overlap[first_index,second_index]*1000
        weight = radii_first[first_index]/(radii_first[first_index]+radii_second[second_index])
        marker.position = transformed[first][first_index]*(1-weight)+transformed[second][second_index]*weight
        marker.visible = maximum>0
        state['center'] = (transformed[first].mean(0)+transformed[second].mean(0))/2
        info.content = f'**link5球{first_index} ↔ link7球{second_index}：'+(f'重叠 {maximum:.4f}mm**' if maximum>0 else f'最小间隙 {-maximum:.4f}mm**')+'\n\n5↔6、6↔7沿用正式配置忽略对。本页只比较腕部球体，不代表整机或携箱碰撞验收；修改后需全流程复核。'

    def load_pose():
        record = next(item for item in reviews if item['label']==case.value)
        state['values'] = dict(zip(active,record['representative']['inspection_joints']))
        for number,control in joint_controls.items():
            limit = model.joint_map[f'{side.value}_joint{number}'].limit
            control.min = float(np.rad2deg(limit.lower))
            control.max = float(np.rad2deg(limit.upper))
            control.value = float(np.rad2deg(state['values'][f'{side.value}_joint{number}']))
        sync_controls()
        draw()

    def edit():
        parameters[selected()] = {key:control.value for key,control in controls.items()}
        fit_status.content = '参数已修改，点击重新拟合后更新球模型。'

    def start_fit(all_links):
        if state['fitting']:return
        edit()
        targets = [side.value+'_link'+str(number) for number in (5,6,7)] if all_links else [selected()]
        requests = {link:copy.deepcopy(parameters[link]) for link in targets}
        state['fitting']=True
        for button in (refit,refit_all,reset_ball,reset_all,restore,save):button.disabled=True
        fit_status.content='正在自动拟合 '+', '.join(targets)+'，可继续旋转视角。'
        def work():
            import torch
            from curobo.sphere_fit import SphereFitType,fit_spheres_to_mesh
            from curobo._src.geom.sphere_fit.metrics import compute_sphere_fit_metrics
            results={}
            for link,params in requests.items():
                started=time.perf_counter()
                torch.manual_seed(20261101)
                np.random.seed(20261101)
                result=fit_spheres_to_mesh(fit_meshes[link],num_spheres=int(params['count']),fit_type=SphereFitType.MORPHIT,
                    iterations=int(params['iterations']),coverage_weight=float(params['coverage']),protrusion_weight=float(params['protrusion']),compute_metrics=False)
                centers=result.centers.detach().cpu().numpy()
                radii=result.radii.detach().cpu().numpy().reshape(-1)+params['margin']/1000
                if not len(radii) or not np.isfinite(centers).all() or not np.isfinite(radii).all() or np.any(radii<=0):
                    raise ValueError(link+'拟合输出无效，保留原预览')
                metric=compute_sphere_fit_metrics(fit_meshes[link],centers,radii,n_interior=2000,n_surface=2000,n_sphere_surface=120)
                report=dict(parameters=params,seconds=time.perf_counter()-started,count=len(radii),coverage=float(metric.coverage),
                    protrusion=float(metric.protrusion),gap_mm=float(metric.max_uncovered_gap*1000))
                results[link]=([dict(center=center.tolist(),radius=float(radius)) for center,radius in zip(centers,radii)],report)
            return results
        future=executor.submit(work)
        def finished(future):
            def apply():
                try:
                    results=future.result()
                    for link,(spheres,report) in results.items():
                        edited[link]=spheres
                        fit_reports[link]=report
                    fit_status.content='\n\n'.join(f"**{link}：{report['count']}球 / {report['seconds']:.2f}s**，采样覆盖{report['coverage']*100:.1f}%，球面外凸{report['protrusion']*100:.1f}%，最大采样缺口{report['gap_mm']:.2f}mm。" for link,(_,report) in results.items())
                    draw()
                except Exception as error:fit_status.content='拟合失败，未替换预览：'+str(error)
                finally:
                    state['fitting']=False
                    for button in (refit,refit_all,reset_ball,reset_all,save):button.disabled=False
                    restore.disabled=not DRAFT.exists()
            pending.put(apply)
        future.add_done_callback(finished)

    def reset(current_only):
        if current_only:
            link = selected()
            edited[link] = copy.deepcopy(original[link])
            fit_reports.pop(link,None)
        else:
            edited.update(copy.deepcopy(original))
            fit_reports.clear()
        fit_status.content='已恢复原始球模型。'
        sync_controls()
        draw()

    def persist():
        DRAFT.parent.mkdir(parents=True,exist_ok=True)
        temporary = DRAFT.with_suffix('.tmp')
        temporary.write_text(json.dumps(dict(source=str(CONFIG),source_sha256=hashlib.sha256(CONFIG.read_bytes()).hexdigest(),collision_spheres=edited,fit_parameters=parameters,fit_reports=fit_reports),indent=2))
        temporary.replace(DRAFT)
        saved.content = '已保存草稿：'+str(DRAFT)+'。正式规划尚未替换。'
        restore.disabled = False

    def restore_draft():
        data = json.loads(DRAFT.read_text())
        if data['source_sha256'] != hashlib.sha256(CONFIG.read_bytes()).hexdigest():
            saved.content = '源配置已变化，停止加载旧草稿。'
            return
        edited.update(data['collision_spheres'])
        parameters.update(data.get('fit_parameters',{}))
        fit_reports.clear()
        fit_reports.update(data.get('fit_reports',{}))
        sync_controls()
        draw()

    def center_camera(client):
        client.camera.position = state['center']+np.array([.5,-.65,.35])
        client.camera.look_at = state['center']
        client.camera.up_direction = (0.,0.,1.)

    def bind_update(control, callback):
        control.on_update(lambda event: pending.put(callback) if event.client_id is not None else None)

    for control in controls.values():bind_update(control,edit)
    for control in (side,case):bind_update(control,load_pose)
    bind_update(link_choice,lambda:(sync_controls(),draw()))
    for control in (*joint_controls.values(),meshes_visible,mesh_opacity,balls_visible,ball_opacity,axes_visible,isolate_link):bind_update(control,draw)
    reset_ball.on_click(lambda event:pending.put(lambda:reset(True)))
    reset_all.on_click(lambda event:pending.put(lambda:reset(False)))
    refit.on_click(lambda event:pending.put(lambda:start_fit(False)))
    refit_all.on_click(lambda event:pending.put(lambda:start_fit(True)))
    save.on_click(lambda event:pending.put(persist))
    restore.on_click(lambda event:pending.put(restore_draft))
    focus.on_click(lambda event:pending.put(lambda:center_camera(event.client)))
    server.on_client_connect(lambda client:pending.put(lambda:center_camera(client)))
    load_pose()
    print(f'Ready: http://127.0.0.1:{args.port}/',flush=True)
    while True:
        while not pending.empty():pending.get()()
        time.sleep(.02)


if __name__ == '__main__':
    main()
