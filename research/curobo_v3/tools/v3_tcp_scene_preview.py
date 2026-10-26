import argparse
import json
from pathlib import Path
import time
from concurrent.futures import ThreadPoolExecutor
import threading

import numpy as np
from scipy.spatial.transform import Rotation
import viser
import yaml

from alfa_motion_viser import ReadOnlyScene, default_visual_assets
from curobo_core.scene import SceneSnapshot
from curobo_core.fixtures import tasks
from v3_tcp_scene_contract import GraspInput, reconstruct_scene, random_parking, tool_to_box
from v3_visualization_dependency import verify_frontend_package
from v3_contact_roll_policy import TOOL_ROLL


def display_robot_urdf(root):
    return root/'generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf'


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--port',type=int,default=8171)
    parser.add_argument('--evidence-dir',type=Path,default=Path('/tmp/tcp_scene_preview_20261018'))
    parser.add_argument('--smoke-test',action='store_true')
    parser.add_argument('--result',type=Path)
    parser.add_argument('--input-preview',type=Path)
    args=parser.parse_args()
    args.evidence_dir.mkdir(parents=True,exist_ok=True)
    verify_frontend_package()
    root=Path(__file__).resolve().parents[1]
    robot_urdf=display_robot_urdf(root)
    frozen=json.loads((Path(__file__).resolve().parents[1]/'config/tcp_scene_preview.json').read_text())
    home=frozen['home']
    current_home=json.loads((root/'config/rezero_home.json').read_text())
    home['updown']=current_home['updown_m']
    for side in ('left','right'):
        home.update({f'{side}_joint{index}':float(np.deg2rad(angle)) for index,angle in enumerate(current_home[side+'_deg'],1)})
    snapshot=SceneSnapshot.from_dict(frozen['snapshot'])
    environment=tuple(item for item in snapshot.objects if not item.object_id.startswith(('task_fill_', 'wall_box_')))
    options=dict(tasks()+tasks([1,0]))
    for box in range(25):
        side='left' if box%5>=3 else 'right'
        options[f'单臂 {side} {box:02d}']={side:box}
    server=viser.ViserServer(host='127.0.0.1',port=args.port,label='随机误差 · 场景、规划与轨迹回放')
    renderer=ReadOnlyScene(server,robot_urdf,default_visual_assets())
    cartons={side:renderer.carton_object('/preview/box_'+side,(.3,.4,.4)) for side in ('left','right')}
    conveyor_root=renderer.reference_conveyors()
    conveyor_root.visible=True
    from types import SimpleNamespace
    from v3_conveyor_cycle_planner import ConveyorCyclePlanner
    conveyor_layout=SimpleNamespace(conveyor_center_x_m=ConveyorCyclePlanner.conveyor_center_x_m,
        conveyor_marker_x_m=ConveyorCyclePlanner.conveyor_marker_x_m,
        conveyor_height_m=ConveyorCyclePlanner.conveyor_height_m,
        box_fit=dict(dimensions_m=[.3,.4,.4]),placement_clearance_m=.15)
    conveyor_targets=ConveyorCyclePlanner.conveyor_reference_targets(conveyor_layout)
    renderer.update_conveyor_targets(conveyor_targets)
    show_conveyors=server.gui.add_checkbox('显示双龙头车外观',initial_value=True)
    @show_conveyors.on_update
    def conveyor_visibility(_event):conveyor_root.visible=show_conveyors.value
    renderer.assembled_container(environment)
    for item in environment:renderer.snapshot_object(item)
    selector=server.gui.add_dropdown('原任务选择',options=list(options),initial_value=next(iter(options)))
    reroll=server.gui.add_button('同一任务重新随机')
    fill_visible=server.gui.add_checkbox('显示实心填充碰撞',initial_value=True)
    cutaway=server.gui.add_checkbox('剖视外观（碰撞不变）',initial_value=True)
    show_axes=server.gui.add_checkbox('显示TCP坐标轴',initial_value=True)
    server.gui.add_markdown('当前新零位模型；任务标签/箱号只用于模拟输入。集装箱固定，抽离沿world -X。只做离线规划/回放，不连接实机。放箱失败时保留完整成功前缀，不宣称全流程成功。底层箱微倾可能穿地，未自动抬箱或关闭检查。')
    info=server.gui.add_markdown('')
    calculate=server.gui.add_button('规划当前随机场景（抓取→运输→放箱→Home）')
    play=server.gui.add_checkbox('播放已规划轨迹',initial_value=False)
    frame=server.gui.add_slider('轨迹帧',min=1,max=1,step=1,initial_value=1)
    stage=server.gui.add_dropdown('跳转阶段',options=['尚无轨迹'])
    outcome=server.gui.add_markdown('选择任务/重新随机后，点击规划；结果来自当前输入，不套用旧任务轨迹。')
    playback_status=server.gui.add_markdown('')
    held={side:renderer.carton_object('/playback/held_'+side,(.3,.4,.4)) for side in ('left','right')}
    for node in held.values():node.visible=False
    execution=dict(result=None,future=None,evidence=None,last_frame=None,initial_result=None)
    executor=ThreadPoolExecutor(max_workers=1)
    cancel_flag=threading.Event()
    runner=[None]
    rng=np.random.default_rng()
    nodes=[]
    current_fills=[]
    current_axes=[]
    revision=0

    def update(task_label=None,saved=None):
        nonlocal revision
        task_label=task_label or selector.value
        revision+=1
        cancel_flag.set()
        execution['result']=None;execution['last_frame']=None
        play.value=False;frame.value=1
        for node in held.values():node.visible=False
        for node in nodes:node.remove()
        nodes.clear();current_fills.clear();current_axes.clear()
        for carton in cartons.values():carton.visible=False
        if saved is None:
            world_from_footprint,yaw=random_parking(rng)
        else:
            world_from_footprint=np.asarray(saved['parking_world_from_footprint']);yaw=saved['parking_yaw_deg']
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
            angles=np.array([rng.uniform(-2,2),rng.uniform(-2,2),rng.uniform(-5,5)]) if saved is None else np.asarray(saved['inputs'][side]['roll_pitch_yaw_deg'])
            box=np.eye(4)
            box[:3,:3]=Rotation.from_euler('xyz',angles,degrees=True).as_matrix()
            box[:3,3]=[wall_center_x,(box_id%5-2)*.41,.201+(box_id//5)*.41]
            mode='top' if min(value//5 for value in selected.values())<=1 else 'side'
            tcp_world=box@np.linalg.inv(TOOL_ROLL@tool_to_box(mode))
            tcp_local=np.linalg.inv(world_from_footprint)@tcp_world
            grasps.append(GraspInput(side,mode,tcp_local))
            truth[side]=box;perturbations[side]=dict(roll_pitch_yaw_deg=angles.tolist(),grasp_mode=mode,tcp_in_footprint=tcp_local.tolist())
        scene=reconstruct_scene(tuple(GraspInput(grasp.side,grasp.mode,grasp.tcp_in_footprint@TOOL_ROLL) for grasp in grasps),world_from_footprint)
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
        execution['evidence']=evidence
        outcome.content='当前随机输入已更新；旧轨迹失效，请重新规划。'

    def accept_result(result):
        execution['result']=result;execution['last_frame']=None
        frame.max=max(1,len(result.get('frames',[])));frame.value=1
        phases=list(dict.fromkeys(result.get('phases',[]))) or ['尚无轨迹']
        stage.options=phases;stage.value=phases[0]
        if result.get('success') and result.get('pending_stage')=='complete':
            label='全流程规划完成'
        else:
            label='存在失败：'+str(result.get('blocked_stage','未完成'))
        outcome.content=f"**{label}**\n\n{result.get('blocked_details','')}\n\n保留{len(result.get('frames',[]))}帧；计算{result.get('total_ms',0)/1000:.3f}s。底盘导航为模拟到位瞬移，未发送实机动作。"

    @calculate.on_click
    def compute(_event):
        if execution['future'] is not None:return
        cancel_flag.clear()
        evidence=json.loads(json.dumps(execution['evidence']))
        frozen_home=dict(home)
        request_revision=revision
        play.value=False
        calculate.disabled=selector.disabled=reroll.disabled=True
        execution['result']=None
        for node in held.values():node.visible=False
        def work():
            from v3_random_error_runner import RandomErrorRunner
            if runner[0] is None:runner[0]=RandomErrorRunner()
            directory=root/'generated/random_error_demo_runs'/str(time.time_ns())
            def progress(message):
                if cancel_flag.is_set():raise RuntimeError('场景已更换，停止旧规划')
                outcome.content=str(message)
            return runner[0].run(evidence,frozen_home,directory,progress)
        execution['future']=(executor.submit(work),request_revision)
        outcome.content='规划中…'

    @stage.on_update
    def jump(_event):
        result=execution['result']
        if result and stage.value in result['phases']:
            frame.value=result['phases'].index(stage.value)+1

    @selector.on_update
    @reroll.on_click
    def regenerate(_event):
        if _event.client is not None:renderer.submit(lambda:update())

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
    if args.input_preview and args.input_preview.is_file():
        saved=json.loads(args.input_preview.read_text())
        selector.value=saved['task_label']
        update(selector.value,saved)
    elif args.result and args.result.is_file():
        initial=json.loads(args.result.read_text())
        selector.value=initial['input_evidence']['task_label']
        update(selector.value,initial['input_evidence'])
        execution['initial_result']=initial
    else:update()
    if args.smoke_test:
        for task_label in list(options)*2:
            update(task_label)
        assert revision==1+2*len(options)
        print(f'SMOKE PASS: {revision} scene updates, all {len(options)} tasks twice; persistent cartons reused',flush=True)
        server.stop()
        return
    print(f'Ready http://127.0.0.1:{args.port} · {len(options)} task choices',flush=True)
    clock=time.monotonic();credit=0.
    while True:
        renderer.drain()
        if execution['initial_result'] is not None:
            accept_result(execution['initial_result']);execution['initial_result']=None
        pending=execution['future']
        if pending is not None and pending[0].done():
            execution['future']=None
            calculate.disabled=selector.disabled=reroll.disabled=False
            try:
                result=pending[0].result()
                if pending[1]!=revision:raise ValueError('随机场景已变更，丢弃旧结果')
                accept_result(result)
            except Exception as error:
                outcome.content='**规划异常，未自动修复/放宽检查：** '+str(error)
        now=time.monotonic();elapsed=now-clock;clock=now
        result=execution['result']
        if play.value and result and result.get('frames'):
            credit+=elapsed
            index=int(frame.value)-1
            frames=result['frames'];names=result['joint_names']
            joint_indices=[offset for offset,name in enumerate(names) if 'joint' in name]
            while index<len(frames)-1:
                delta=np.abs(np.asarray(frames[index+1])-frames[index])
                duration=max(.02,float(delta[joint_indices].max())/np.deg2rad(30))
                if credit<duration:break
                credit-=duration;index+=1
            frame.value=index+1
            if index==len(frames)-1:play.value=False
        else:credit=0.
        if result and result.get('frames') and execution['last_frame']!=frame.value:
            index=min(int(frame.value)-1,len(result['frames'])-1)
            values=dict(zip(result['joint_names'],result['frames'][index]))
            values.update({name:values.get(name,home.get(name,0.)) for name in renderer.model.actuated_joint_names})
            renderer.apply_state(values)
            attached=result['payload'][index]
            attached_index=result['phases'].index('attach') if 'attach' in result['phases'] else len(result['frames'])
            for side,node in held.items():
                node.visible=bool(attached and side in result.get('tool_to_box',{}))
                if node.visible:
                    pose=renderer.tool_poses[side]@np.asarray(result['tool_to_box'][side])
                    node.position=pose[:3,3];node.wxyz=np.roll(Rotation.from_matrix(pose[:3,:3]).as_quat(),1)
                cartons[side].visible=side in execution['evidence']['inputs'] and index<attached_index
            execution['last_frame']=frame.value
            playback_status.content=f"**当前阶段：{result['phases'][index]} · 帧{index+1}/{len(result['frames'])}**\n\n失败结果仅播放已返回的成功前缀；未生成的放箱/回Home动作不会补画。"
        time.sleep(.01)


if __name__=='__main__':main()
