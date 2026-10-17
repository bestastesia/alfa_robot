import argparse
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import viser
import yaml

from alfa_motion_viser import ReadOnlyScene, default_visual_assets
from curobo_core.scene import SceneSnapshot
from curobo_core.fixtures import tasks
from v3_tcp_scene_contract import GraspInput, reconstruct_scene, random_parking, tool_to_box
from v3_visualization_dependency import verify_frontend_package


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--port',type=int,default=8152)
    parser.add_argument('--evidence-dir',type=Path,default=Path('/tmp/tcp_scene_preview_20261018'))
    parser.add_argument('--smoke-test',action='store_true')
    args=parser.parse_args()
    args.evidence_dir.mkdir(parents=True,exist_ok=True)
    verify_frontend_package()
    mobile_config=Path(__file__).resolve().parents[1]/'generated/v3_analytic_071cb95/mobile18/alfa_v322_suction_mobile18.yml'
    config=yaml.safe_load(mobile_config.read_text())['kinematics']
    robot_urdf=Path(config['urdf_path'])
    frozen=json.loads((Path(__file__).resolve().parents[1]/'config/tcp_scene_preview.json').read_text())
    home=frozen['home']
    snapshot=SceneSnapshot.from_dict(frozen['snapshot'])
    environment=tuple(item for item in snapshot.objects if not item.object_id.startswith(('task_fill_', 'wall_box_')))
    options=dict(tasks()+tasks([1,0]))
    for box in range(25):
        side='left' if box%5>=3 else 'right'
        options[f'单臂 {side} {box:02d}']={side:box}
    server=viser.ViserServer(host='127.0.0.1',port=args.port,label='TCP输入合同 · 随机停车与箱墙生成预览')
    renderer=ReadOnlyScene(server,robot_urdf,default_visual_assets())
    cartons={side:renderer.carton_object('/preview/box_'+side,(.3,.4,.4)) for side in ('left','right')}
    renderer.reference_conveyors()
    renderer.assembled_container(environment)
    for item in environment:renderer.snapshot_object(item)
    selector=server.gui.add_dropdown('原任务选择',options=list(options),initial_value=next(iter(options)))
    reroll=server.gui.add_button('同一任务重新随机')
    fill_visible=server.gui.add_checkbox('显示实心填充碰撞',initial_value=True)
    cutaway=server.gui.add_checkbox('剖视外观（碰撞不变）',initial_value=True)
    show_axes=server.gui.add_checkbox('显示TCP坐标轴',initial_value=True)
    server.gui.add_markdown('仅场景合同预览，不规划或宣称抓取成功。任务标签/箱号只用于模拟输入；场景重建函数仅接收TCP位姿、吸附类型和world→footprint。集装箱固定，抽离始终沿world -X。')
    info=server.gui.add_markdown('')
    rng=np.random.default_rng()
    nodes=[]
    current_fills=[]
    current_axes=[]
    revision=0

    def update(task_label=None):
        nonlocal revision
        task_label=task_label or selector.value
        revision+=1
        for node in nodes:node.remove()
        nodes.clear();current_fills.clear();current_axes.clear()
        for carton in cartons.values():carton.visible=False
        world_from_footprint,yaw=random_parking(rng)
        values=home.copy()
        values.update({name:values.get(name,0.) for name in renderer.model.actuated_joint_names})
        values.update(base_x=world_from_footprint[0,3],base_y=world_from_footprint[1,3],base_yaw=np.deg2rad(yaw))
        renderer.apply_state(values)
        renderer.set_scene_view(False,cutaway.value)
        grasps=[];perturbations={};truth={}
        front=max(item.pose.position[0]-item.dimensions_m[0]/2 for item in environment if item.object_id=='front_wall')
        wall_center_x=front-.20
        selected=options[task_label]
        for side,box_id in selected.items():
            angles=np.array([rng.uniform(-2,2),rng.uniform(-2,2),rng.uniform(-5,5)])
            box=np.eye(4)
            box[:3,:3]=Rotation.from_euler('xyz',angles,degrees=True).as_matrix()
            box[:3,3]=[wall_center_x,(box_id%5-2)*.41,.201+(box_id//5)*.41]
            mode='top' if min(value//5 for value in selected.values())<=1 else 'side'
            tcp_world=box@np.linalg.inv(tool_to_box(mode))
            tcp_local=np.linalg.inv(world_from_footprint)@tcp_world
            grasps.append(GraspInput(side,mode,tcp_local))
            truth[side]=box;perturbations[side]=dict(roll_pitch_yaw_deg=angles.tolist(),grasp_mode=mode,tcp_in_footprint=tcp_local.tolist())
        scene=reconstruct_scene(grasps,world_from_footprint)
        for side,matrix in scene['boxes'].items():
            assert np.allclose(matrix,truth[side],atol=1e-10)
            carton=cartons[side]
            carton.position=matrix[:3,3];carton.wxyz=np.roll(Rotation.from_matrix(matrix[:3,:3]).as_quat(),1);carton.visible=True
            tcp=world_from_footprint@next(grasp.tcp_in_footprint for grasp in grasps if grasp.side==side)
            axes=server.scene.add_frame('/preview/tcp_'+side,position=tcp[:3,3],wxyz=np.roll(Rotation.from_matrix(tcp[:3,:3]).as_quat(),1),axes_length=.12,axes_radius=.003,visible=show_axes.value)
            nodes.append(axes);current_axes.append(axes)
        for fill in scene['fills']:
            node=server.scene.add_box('/preview/'+fill['name'],position=fill['center'],dimensions=fill['dimensions'],color=(225,135,45),opacity=.6,visible=fill_visible.value)
            nodes.append(node);current_fills.append(node)
        nodes.append(server.scene.add_line_segments('/preview/outward',points=np.array([[np.array([wall_center_x-.2,0,2.1]),np.array([wall_center_x-.7,0,2.1])]]),colors=(30,220,80),line_width=6))
        nodes.append(server.scene.add_label('/preview/outward_label',text='抽离 world -X（停车Yaw不改变）',position=(wall_center_x-.55,0,2.15)))
        text=f'**随机样本 {revision}** · 停车 dx={world_from_footprint[0,3]*100:+.2f}cm，dy={world_from_footprint[1,3]*100:+.2f}cm，Yaw={yaw:+.2f}°\n\n'
        for side,record in perturbations.items():
            angles=record['roll_pitch_yaw_deg'];text+=f"{side} · {record['grasp_mode']}：箱体 Roll={angles[0]:+.2f}° / Pitch={angles[1]:+.2f}° / Yaw={angles[2]:+.2f}°\n\n"
        text+=f"橙色实心块 {len(scene['fills'])} 个；旋转箱体左右最远边界各空开2mm。世界抽离方向在footprint中为 {np.round(scene['outward_footprint'],4).tolist()}。"
        info.content=text
        evidence=dict(revision=revision,task_label=task_label,parking_world_from_footprint=world_from_footprint.tolist(),parking_yaw_deg=yaw,
            inputs=perturbations,fills=scene['fills'],outward_world=scene['outward_world'].tolist(),outward_footprint=scene['outward_footprint'].tolist())
        (args.evidence_dir/'latest.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2))

    @selector.on_update
    @reroll.on_click
    def regenerate(_event):renderer.submit(lambda:update())

    @fill_visible.on_update
    def visibility(_event):
        for node in current_fills:node.visible=fill_visible.value

    @show_axes.on_update
    def axes_visibility(_event):
        for node in current_axes:node.visible=show_axes.value

    @cutaway.on_update
    def view(_event):renderer.submit(lambda:renderer.set_scene_view(False,cutaway.value))

    @server.on_client_connect
    def connected(client):
        client.camera.position=(-2.8,-3.8,2.5);client.camera.look_at=(.8,0.,1.0);client.camera.up_direction=(0.,0.,1.)
    update()
    if args.smoke_test:
        for task_label in list(options)*2:
            update(task_label)
        assert revision==1+2*len(options)
        print(f'SMOKE PASS: {revision} scene updates, all {len(options)} tasks twice; persistent cartons reused',flush=True)
        server.stop()
        return
    print(f'Ready http://127.0.0.1:{args.port} · {len(options)} task choices',flush=True)
    while True:renderer.drain();time.sleep(.01)


if __name__=='__main__':main()
