import argparse
from dataclasses import replace
import json
from pathlib import Path
import time

import numpy as np

from curobo_core.adapter import matrix_pose
from curobo_core.scene import SceneObject,Pose
from v3_random_error_runner import RandomErrorRunner
from v3_tcp_rezero_pipeline import TcpMotionPlanner,tool_to_box
from v3_rezero_task_planner import migrated_assets
from v3_zero_model_candidate import ROOT


SCHEDULE=[{'left':24,'right':20},{'left':23,'right':21},{'left':19,'right':22},
    {'left':18,'right':15},{'left':17,'right':16},{'left':14,'right':10},
    {'left':13,'right':11},{'left':12},{'left':9,'right':5},{'left':8,'right':6},
    {'left':4,'right':7},{'left':3,'right':0},{'left':2,'right':1}]


class WallPlanner(TcpMotionPlanner):
    remaining_boxes=None

    def bind_request(self,request):
        scene=super().bind_request(request)
        selected=set(self.demo_task.values())
        objects=[item for item in self.snapshot.objects if not item.object_id.startswith('task_fill_')]
        for box_id in sorted(self.remaining_boxes-selected):
            objects.append(SceneObject(f'remaining_box_{box_id}',(.3,.4,.4),matrix_pose(self.world_boxes[box_id])))
        self.snapshot=replace(self.snapshot,objects=tuple(objects),revision=self.snapshot.revision+1)
        return scene


def pick_base_pose(task):
    pose=np.eye(4)
    if min(task.values())//5<=1:pose[0,3]=.3
    return pose


def generate(folder,resume=False):
    folder.mkdir(parents=True,exist_ok=True)
    planner=WallPlanner(migrated_assets())
    runner=RandomErrorRunner();runner.planner=planner
    config=json.loads((ROOT/'config/tcp_scene_preview.json').read_text())
    front=max(item['pose']['position'][0]-item['dimensions_m'][0]/2 for item in config['snapshot']['objects'] if item['object_id']=='front_wall')
    boxes={}
    for box_id in range(25):
        pose=np.eye(4);pose[:3,3]=[front-.20,(box_id%5-2)*.41,.201+(box_id//5)*.41]
        boxes[box_id]=pose
    home_config=json.loads((ROOT/'config/rezero_home.json').read_text())
    home=dict(config['home']);home['updown']=home_config['updown_m']
    home.update({f'{side}_joint{index}':float(np.deg2rad(angle)) for side in ('left','right') for index,angle in enumerate(home_config[side+'_deg'],1)})
    remaining=set(range(25));rounds=[]
    manifest=dict(world_boxes={str(key):value.tolist() for key,value in boxes.items()},rounds=rounds,
        success=False,completed_boxes=[],scope='独立25箱整墙离线演示；底盘导航到位瞬移；逐轮保留剩余箱碰撞，不控制实机')
    path=folder/'wall.json'
    start_round=0
    if resume and path.is_file():
        saved=json.loads(path.read_text())
        for result in saved['rounds']:
            if not result.get('success'):break
            rounds.append(result)
            remaining-=set(result['wall_task'].values())
            start_round+=1
        (folder/'wall_before_resume.json').write_text(json.dumps(saved,ensure_ascii=False))
    for index,task in enumerate(SCHEDULE,1):
        if index<=start_round:continue
        print('ROUND',index,task,flush=True)
        planner.remaining_boxes=set(remaining);planner.demo_task=task;planner.world_boxes=boxes
        mode='top' if min(task.values())//5<=1 else 'side'
        pick_base=pick_base_pose(task)
        evidence=dict(task_label='/'.join(('L' if side=='left' else 'R')+str(box) for side,box in task.items()),
            parking_world_from_footprint=pick_base.tolist(),parking_yaw_deg=0.,inputs={})
        for side,box in task.items():
            evidence['inputs'][side]=dict(grasp_mode=mode,tcp_in_footprint=(np.linalg.inv(pick_base)@boxes[box]@np.linalg.inv(tool_to_box(mode))).tolist(),roll_pitch_yaw_deg=[0.,0.,0.])
        result=runner.run(evidence,home,folder/f'round_{index:02d}',lambda text:print(text,flush=True))
        result['wall_task']=task;result['remaining_before']=sorted(remaining)
        if result.get('frames') and pick_base[0,3]:
            initial=np.asarray(result['frames'][0]).copy()
            for name in ('base_x','base_y','base_yaw'):initial[result['joint_names'].index(name)]=0.
            result['frames'].insert(0,initial.tolist());result['phases'].insert(0,'navigation_to_pick');result['payload'].insert(0,False)
        if result.get('success'):
            names=result['joint_names'];last=np.asarray(result['frames'][-1]);base_end=last.copy()
            for name in ('base_x','base_y','base_yaw'):base_end[names.index(name)]=0.
            result['frames'].append(base_end.tolist());result['phases'].append('base_return');result['payload'].append(False)
            actual=np.asarray([base_end[names.index(name)] for name in home if name in names])
            expected=np.asarray([home[name] for name in home if name in names])
            if np.max(np.abs(actual-expected))>1e-4:raise ValueError('整墙轮间Home不衔接，停止')
            remaining-=set(task.values())
        rounds.append(result)
        manifest.update(completed_boxes=sorted(set(range(25))-remaining),success=not remaining)
        path.write_text(json.dumps(manifest,ensure_ascii=False))
        print('ROUND_RESULT',index,result.get('success'),result.get('blocked_stage'),result.get('blocked_details'),flush=True)
        if not result.get('success'):break
    print('WALL_RESULT',manifest['success'],len(manifest['completed_boxes']),path,flush=True)


def view(folder,port):
    import viser
    from scipy.spatial.transform import Rotation
    from alfa_motion_viser import ReadOnlyScene,default_visual_assets
    data=json.loads((folder/'wall.json').read_text());rounds=data['rounds']
    total=len(data['world_boxes'])
    server=viser.ViserServer(host='127.0.0.1',port=port,label=f'{total}箱连续任务回放')
    renderer=ReadOnlyScene(server,ROOT/'generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf',default_visual_assets())
    from curobo_core.scene import SceneSnapshot
    snapshot=SceneSnapshot.from_dict(rounds[0]['visualization_scene_snapshot'])
    environment=[item for item in snapshot.objects if not item.object_id.startswith(('wall_box_','remaining_box_','task_fill_','future_walls_'))]
    renderer.assembled_container(environment)
    for item in environment:renderer.snapshot_object(item)
    conveyor=renderer.reference_conveyors();conveyor.visible=True
    from v3_conveyor_cycle_planner import ConveyorCyclePlanner
    from types import SimpleNamespace
    references=ConveyorCyclePlanner.conveyor_reference_targets(SimpleNamespace(conveyor_center_x_m=-2.3,conveyor_marker_x_m=.85,conveyor_height_m=.732,box_fit=dict(dimensions_m=[.3,.4,.4]),placement_clearance_m=.15))
    renderer.update_conveyor_targets(references)
    cartons={int(key):renderer.carton_object('/wall/'+key,(.3,.4,.4),np.asarray(pose)[:3,3],np.roll(Rotation.from_matrix(np.asarray(pose)[:3,:3]).as_quat(),1)) for key,pose in data['world_boxes'].items()}
    frame_map=[(cycle,frame) for cycle,result in enumerate(rounds) for frame in range(len(result['frames']))]
    play=server.gui.add_checkbox('连续播放',initial_value=False)
    speed=server.gui.add_dropdown('播放倍率',options=['1×','2×','4×','8×','16×'],initial_value='1×')
    slider=server.gui.add_slider('总轨迹帧',min=1,max=max(1,len(frame_map)),step=1,initial_value=1)
    playable=[index for index,result in enumerate(rounds) if result.get('frames')]
    cycles=server.gui.add_dropdown('跳转轮次',options=[f"{index+1}："+(f"第{rounds[index]['wall_index']+1}面 · " if 'wall_index' in rounds[index] else '')+str(rounds[index]['wall_task']) for index in playable])
    rewind=server.gui.add_button('从第一初始姿态重新播放')
    cutaway=server.gui.add_checkbox('剖视集装箱',initial_value=True)
    status=server.gui.add_markdown('')
    if 'chosen_extra_deg' in data:
        baseline=data['baseline_extra_deg'];chosen=data['chosen_extra_deg']
        improvement=100*(baseline-chosen)/baseline if baseline else 0.
        server.gui.add_markdown(f"**按整轮末端额外旋转选择配对**\n\n{data.get('baseline_scope','比较基准')}：{baseline:.1f}° → 当前{chosen:.1f}°，减少{improvement:.1f}%。覆盖两侧末端、预接触/抽离/运输/放箱/回Home；导航转身不计入臂的额外旋转。每层四种配对及顺序中择优，不宣称全局最优；不以耗时作选择依据。")
    if not data['success']:
        failure=rounds[-1]
        server.gui.add_markdown(f"**尚未完成{total}箱：已完成{len(data['completed_boxes'])}箱**\n\n停止在第{len(rounds)}轮 {failure['wall_task']}：{failure.get('blocked_stage','')}，{failure.get('blocked_details','')}。页面只播放已算出的真实轨迹，不用其他任务动画补齐。")
    server.gui.add_markdown('真实规划结果回放；导航阶段按约定瞬移到位。取箱后只移除实际已释放的箱子，不重建满墙。无实机控制。')
    @cycles.on_update
    def jump(_event):slider.value=next(index+1 for index,pair in enumerate(frame_map) if pair[0]==playable[cycles.options.index(cycles.value)])
    @rewind.on_click
    def reset(_event):slider.value=1;play.value=True
    @server.on_client_connect
    def connect(client):
        client.camera.position=(-3.,-4.,2.8);client.camera.look_at=(.4,0.,1.);client.camera.up_direction=(0.,0.,1.)
    previous=None;clock=time.monotonic();credit=0.
    while True:
        now=time.monotonic();elapsed=now-clock;clock=now
        if play.value:
            credit+=elapsed*float(speed.value.rstrip('×'))
            while slider.value<len(frame_map):
                current_cycle,current_frame=frame_map[int(slider.value)-1]
                next_cycle,next_frame=frame_map[int(slider.value)]
                duration=.02
                if current_cycle==next_cycle:
                    result=rounds[current_cycle]
                    indices=[offset for offset,name in enumerate(result['joint_names']) if 'joint' in name]
                    delta=np.abs(np.asarray(result['frames'][next_frame])-result['frames'][current_frame])
                    duration=max(duration,float(delta[indices].max())/np.deg2rad(30))
                if credit<duration:break
                slider.value+=1;credit-=duration
            if slider.value==len(frame_map):play.value=False
        else:credit=0.
        signature=(slider.value,cutaway.value)
        if signature!=previous:
            cycle,index=frame_map[int(slider.value)-1];result=rounds[cycle]
            values=dict(zip(result['joint_names'],result['frames'][index]));values.update({name:values.get(name,0.) for name in renderer.model.actuated_joint_names})
            renderer.apply_state(values);renderer.set_scene_view(False,cutaway.value)
            if 'conveyor_offset_m' in result:
                offset=result['conveyor_offset_m']
                shifted={side:dict(item,position=[item['position'][0]+offset,*item['position'][1:]],conveyor_position=[item['conveyor_position'][0]+offset,*item['conveyor_position'][1:]]) for side,item in references.items()}
                renderer.update_conveyor_targets(shifted)
            attach=result['phases'].index('attach') if 'attach' in result['phases'] else len(result['frames'])
            released=result['phases'].index('release') if 'release' in result['phases'] else len(result['frames'])
            selected=result['wall_task'];remaining=set(result['remaining_before'])
            for box,node in cartons.items():
                node.visible=box in remaining and (box not in selected.values() or index<released)
                if box in selected.values() and attach<=index<released:
                    side=next(side for side,value in selected.items() if value==box)
                    pose=renderer.tool_poses[side]@np.asarray(result['tool_to_box'][side])
                else:pose=np.asarray(data['world_boxes'][str(box)])
                node.position=pose[:3,3];node.wxyz=np.roll(Rotation.from_matrix(pose[:3,:3]).as_quat(),1)
            count=total-len(remaining)+(len(selected) if index>=released else 0)
            wall_label=f"第{result['wall_index']+1}/{data['layer_count']}面 · " if 'wall_index' in result else ''
            status.content=f"**{str(total)+'箱完整成功' if data['success'] else '任务未完成，保留失败现场'}**\n\n{wall_label}第{cycle+1}/{len(rounds)}轮 · {selected} · {result['phases'][index]}\n\n已放下{count}/{total}箱 · 帧{slider.value}/{len(frame_map)}\n\n{result.get('blocked_stage','')} {result.get('blocked_details','')}"
            if result.get('rotation_metrics'):
                metric=result['rotation_metrics']
                status.content+=f"\n\n本轮左末端额外旋转{metric['left']['extra_deg']:.1f}°；右末端{metric['right']['extra_deg']:.1f}°；累计旋转合计{metric['combined_total_deg']:.1f}°。"
            previous=signature
        time.sleep(.01)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--generate',action='store_true');parser.add_argument('--resume',action='store_true');parser.add_argument('--folder',type=Path,default=Path('/tmp/wall25_demo_20261021'));parser.add_argument('--port',type=int,default=8174)
    args=parser.parse_args()
    generate(args.folder,args.resume) if args.generate else view(args.folder,args.port)
