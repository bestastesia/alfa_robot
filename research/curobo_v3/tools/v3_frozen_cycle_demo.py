import argparse
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
import gzip
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import viser
from alfa_motion_viser import ReadOnlyScene,default_visual_assets
from curobo_core.scene import SceneSnapshot
from v3_frozen_cycle import FrozenCyclePlanner,PROFILE
from v3_current_urdf_forward import current_assets
from v3_frozen_cycle_validate import OUTPUT,save
from v3_visualization_dependency import verify_frontend_package
from v3_wall_ik_benchmark import ACTIVE_JOINTS


def main(default_port=8243):
    parser=argparse.ArgumentParser()
    parser.add_argument('--port',type=int,default=default_port)
    parser.add_argument('--result',type=Path)
    parser.add_argument('--recordings',type=Path)
    assets=current_assets()
    for name in ('robot_config','mobile_robot_config','urdf','named_poses','box_fit'):
        parser.add_argument('--'+name.replace('_','-'),type=Path,default=getattr(assets,name))
    args=parser.parse_args()
    planner=FrozenCyclePlanner(args)
    verify_frontend_package()
    server=viser.ViserServer(host='127.0.0.1',port=args.port,label='固定全流程 · 解析抽离/RRT运输/LM放箱')
    renderer=ReadOnlyScene(server,planner.args.urdf,default_visual_assets())
    server.gui.add_markdown('**已固定流程**：Home→3cm预接触→解析贴近吸附→解析直线抽离35cm→多根多目标RRT运输→导航瞬移→128点15轴连续LM放箱→释放→从实际末态RRT回Home。携箱仅用原40球模型，失败停止，不回退RRT放箱。仅离线规划与回放。')
    choices=dict(planner.task_options)
    task=server.gui.add_dropdown('任务',options=list(choices))
    recordings=args.recordings or (args.result.parent if args.result else OUTPUT)
    review_path=recordings/'failure_review.json'
    review_records={item['label']:item for item in json.loads(review_path.read_text())['records']} if review_path.exists() else {}
    with server.gui.add_folder('失败前姿态回放',expand_by_default=True):
        failure_choice=server.gui.add_dropdown('LM失败存档',options=list(review_records) or ['暂无诊断存档'])
        failure_load=server.gui.add_button('显示失败前最后合法姿态',disabled=not bool(review_records))
        failure_start=server.gui.add_button('从放箱起点回放失败前过程',disabled=not bool(review_records))
        inspect_rejected=server.gui.add_checkbox('查看被拒姿态（仅诊断）',initial_value=False,disabled=True)
        focus_collision=server.gui.add_button('放大碰撞位置')
        hide_appearance=server.gui.add_checkbox('隐藏机器人外观看碰撞球',initial_value=False)
        failure_detail=server.gui.add_markdown('选择失败存档可直接查看最后合法姿态，无需重新规划。勾选诊断后才显示被拒姿态；红点仅定位碰撞处。')
    run=server.gui.add_button('计算完整流程')
    play=server.gui.add_checkbox('播放完整流程',initial_value=False)
    frame=server.gui.add_slider('轨迹帧',min=0,max=1,step=1,initial_value=0)
    phase=server.gui.add_dropdown('跳转阶段',options=['home'])
    cutaway=server.gui.add_checkbox('集装箱剖视',initial_value=True)
    status=server.gui.add_markdown('选择任务后计算。')
    detail=server.gui.add_markdown('')
    state={}
    nodes=[]
    source_boxes=[]
    failure_markers=[]
    held={}
    for side in ('left','right'):
        held[side]=server.scene.add_frame('/held/'+side,show_axes=False,visible=False)
        renderer.carton_object('/held/'+side+'/box',(.3,.4,.4))
    planner.configure_task(choices[task.value])
    renderer.assembled_container(tuple(replace(item,pose=replace(item.pose,position=(item.pose.position[0]+planner.base_advance,*item.pose.position[1:]))) for item in planner.snapshot.objects))
    renderer.set_scene_view(False,True)
    renderer.reference_conveyors().visible=True
    renderer.apply_state(dict(planner.home,base_x=planner.base_advance,base_y=0.,base_yaw=0.))
    executor=ThreadPoolExecutor(max_workers=1)

    @server.on_client_connect
    def connect(client):
        client.camera.position=(1.,-3.2,2.1)
        client.camera.look_at=(.3,0.,1.)
        client.camera.up_direction=(0.,0.,1.)

    def update():
        if 'result' not in state:return
        result=state['result']
        index=min(int(frame.value),len(result['frames'])-1)
        review=state.get('failure_review')
        inspecting=review is not None and inspect_rejected.value
        if inspecting:index=len(result['frames'])-1
        values=dict(result['visual_home'])
        values.update(zip(result['joint_names'],result['frames'][index]))
        if inspecting:
            values.update(zip(ACTIVE_JOINTS,review['representative']['inspection_joints']))
        renderer.apply_state(values)
        for side,node in held.items():
            node.visible=bool(result['payload'][index]) and side in result['task']
            if node.visible:
                matrix=renderer.tool_poses[side]@np.asarray(result['visual_tool_to_box'][side])
                node.position=matrix[:3,3]
                node.wxyz=np.roll(Rotation.from_matrix(matrix[:3,:3]).as_quat(),1)
        picked=any(result['payload'][:index+1])
        for node in source_boxes:node.visible=not picked
        status.content=f"**{state['result_label']} · {result['phases'][index]} · 帧{index}/{len(result['frames'])-1}**\n\n"+('全流程通过。' if result['success'] else f"停止于{result.get('blocked_stage')}：{result.get('blocked_details')}。仅显示已通过前缀。")
        for node in failure_markers:node.visible=inspecting
        renderer.root.visible=not (inspecting and hide_appearance.value)
        if inspecting:
            status.content='**仅诊断：当前为被拒候选或插值段内的姿态，不属于已接受轨迹。**'
        elif review is not None and index==len(result['frames'])-1:
            status.content=f"**{state['result_label']} · 失败前最后合法姿态**\n\n放箱进度{review['completed_progress']*100:.2f}%；下一参考点{review['rejected_progress']*100:.2f}%未通过。"

    def load(result):
        if result.get('cycle_profile')!=PROFILE:raise ValueError('recording belongs to another planning profile')
        state['result']=result
        state.pop('failure_review',None)
        inspect_rejected.value=False
        inspect_rejected.disabled=True
        play.disabled=False
        frame.disabled=False
        phase.disabled=False
        failure_detail.content='选择失败存档可直接查看最后合法姿态，无需重新规划。'
        for node in failure_markers:node.remove()
        failure_markers.clear()
        state['result_label']=next(label for label,boxes in choices.items() if boxes==result['task'])
        play.value=False
        frame.max=max(1,len(result['frames'])-1)
        frame.value=0
        phase.options=list(dict.fromkeys(result['phases']))
        phase.value=phase.options[0]
        for node in nodes:node.remove()
        nodes.clear()
        source_boxes.clear()
        snapshot=SceneSnapshot.from_dict(result['visual_scene_snapshot'])
        for item in snapshot.objects:
            if item.object_id in ('front_wall','left_wall','right_wall','ceiling') or item.object_id.startswith('conveyor_pair'):continue
            node=server.scene.add_box('/environment/'+item.object_id,dimensions=item.dimensions_m,
                position=np.asarray(item.pose.position)+[result['visual_base_advance'],0,0],wxyz=item.pose.quaternion_wxyz,color=(180,125,60),opacity=.4)
            nodes.append(node)
            if item.object_id.startswith('wall_box_'):source_boxes.append(node)
        if result.get('conveyor_box_targets'):renderer.update_conveyor_targets(result['conveyor_box_targets'])
        lm=result.get('conveyor_place_lm')
        detail.content=f"本次全流程计算{result['wall_ms']/1000:.3f}s。"
        if lm:detail.content+=f" LM放箱{lm['total_ms']/1000:.3f}s，进度{lm['completed_progress']*100:.1f}%；"+('已到放箱终点。' if lm['success'] else '尚未到达放箱终点。')
        if lm and lm.get('edge_pose_policy')=='diagnostic_only':
            detail.content+=f"\n\nLM节点之间检查碰撞与关节连续性，参考位姿偏差仅统计，不作拒绝条件。本段最大位置偏差{lm['max_reference_position_error_m']*1000:.3f}mm。"
        update()

    def load_failure():
        if state.get('busy') or failure_choice.value not in review_records:return
        review=review_records[failure_choice.value]
        result=json.loads(gzip.decompress((recordings/review['result_file']).read_bytes()))
        task.value=failure_choice.value
        load(result)
        state['failure_review']=review
        inspect_rejected.disabled=False
        frame.value=len(result['frames'])-1
        phase.value='conveyor_place'
        representative=review['representative']
        counts=review['original_counts']
        if review['reason']=='collision':
            names=sorted({pair['first']+' ↔ '+pair['second'] for pair in review['collision_pairs']})
            location='、'.join(names) or '原碰撞模型拒绝'
            where='候选终点' if counts['collision_valid']==0 else '候选之间的插值边'
            failure_detail.content=f"**原因：{where}自碰撞 · {location}**\n\n代表被拒连接段最大球体重叠{representative.get('max_self_overlap_mm',0.):.4f}mm。目标位姿合格{counts['lm_valid']}个，候选终点无碰撞{counts['collision_valid']}个。"
            failure_detail.content+='\n\n这是机器人腕部自碰撞检查；1mm容差仅用于环境碰撞，不用于自碰撞。'
        else:
            failure_detail.content=f"**原因：插值参考位置误差超限**\n\n候选终点合法，但代表插值边误差最大{representative['max_position_error_mm']:.3f}mm，超过2mm；朝向误差{representative['max_orientation_error_deg']:.3f}°。该连接边未发现碰撞。"
        failure_detail.content+='\n\n本批次存档：轨迹止于最后合法帧；“仅诊断”独立显示被拒姿态。'
        base=np.asarray(review['world_base'])
        for number,pair in enumerate(review['collision_pairs'][:4]):
            position=base[:3,:3]@np.asarray(pair['point_local'])+base[:3,3]
            for member,color in (('first',(255,145,30)),('second',(30,190,255))):
                if member+'_sphere_local' not in pair:continue
                sphere=np.asarray(pair[member+'_sphere_local'])
                center=base[:3,:3]@sphere[:3]+base[:3,3]
                radius=float(sphere[3])+pair.get(member+'_padding',0.)
                failure_markers.append(server.scene.add_icosphere('/failure/sphere'+str(number)+member,radius=radius,color=color,opacity=.55,subdivisions=4,position=center,visible=False))
            failure_markers.append(server.scene.add_icosphere('/failure/point'+str(number),radius=.003,color=(235,45,45),position=position,visible=False))
            failure_markers.append(server.scene.add_line_segments('/failure/leader'+str(number),points=np.asarray([[position,position+[0,0,.12]]]),colors=(235,45,45),line_width=3.,visible=False))
            failure_markers.append(server.scene.add_label('/failure/label'+str(number),text=pair['first']+' / '+pair['second'],position=position+[0,0,.14],visible=False))
        if review['collision_pairs']:
            failure_detail.content+='\n\n橙色：link5碰撞球；蓝色：link7碰撞球；红点：重叠位置。球体使用检查器原尺寸（含原padding），不是实体外观。可点击“放大碰撞位置”，并隐藏机器人外观观察。'
        update()

    def focus_failure():
        if state.get('busy'):return
        if not state.get('failure_review'):load_failure()
        review=state.get('failure_review')
        if not review or not review['collision_pairs']:return
        inspect_rejected.value=True
        inspection_changed()
        base=np.asarray(review['world_base'])
        point=base[:3,:3]@np.asarray(review['collision_pairs'][0]['point_local'])+base[:3,3]
        for client in server.get_clients().values():
            client.camera.position=point+base[:3,:3]@np.asarray([.32,-.42,.22])
            client.camera.look_at=point
            client.camera.up_direction=(0.,0.,1.)

    def start_failure():
        load_failure()
        if state.get('failure_review'):
            frame.value=state['failure_review']['placement_start_frame']
            play.value=True

    def inspection_changed():
        play.value=False
        if state.get('failure_review'):frame.value=len(state['result']['frames'])-1
        play.disabled=inspect_rejected.value
        frame.disabled=inspect_rejected.value
        phase.disabled=inspect_rejected.value
        update()

    def compute():
        if state.get('busy'):return
        state['busy']=True
        task.disabled=True
        run.disabled=True
        play.value=False
        inspect_rejected.value=False
        state.pop('failure_review',None)
        for node in failure_markers:node.visible=False
        for control in (failure_choice,failure_load,failure_start,inspect_rejected):control.disabled=True
        selected=dict(choices[task.value])
        future=executor.submit(planner.full_cycle,selected,lambda text:renderer.submit(lambda text=text:setattr(status,'content',text)))
        def complete(future):
            def finish():
                try:
                    result=future.result()
                    save(recordings/'live'/f'{time.time_ns()}.json.gz',result)
                    load(result)
                except Exception as error:
                    status.content='计算错误，停止：'+str(error)
                finally:
                    state['busy']=False
                    task.disabled=False
                    run.disabled=False
                    for control in (failure_choice,failure_load,failure_start,inspect_rejected):control.disabled=False
                    inspect_rejected.disabled=True
            renderer.submit(finish)
        future.add_done_callback(complete)

    run.on_click(lambda event:renderer.submit(compute))
    failure_choice.on_update(lambda event:renderer.submit(load_failure))
    failure_load.on_click(lambda event:renderer.submit(load_failure))
    failure_start.on_click(lambda event:renderer.submit(start_failure))
    focus_collision.on_click(lambda event:renderer.submit(focus_failure))
    hide_appearance.on_update(lambda event:renderer.submit(update))
    inspect_rejected.on_update(lambda event:renderer.submit(inspection_changed) if event.client_id is not None else None)
    task.on_update(lambda event:setattr(play,'value',False))
    frame.on_update(lambda event:renderer.submit(update))
    phase.on_update(lambda event:setattr(frame,'value',state['result']['phases'].index(phase.value)) if event.client_id is not None and 'result' in state else None)
    cutaway.on_update(lambda event:[setattr(node,'visible',not cutaway.value) for node in renderer.container_walls])
    if args.result:
        result=json.loads(gzip.decompress(args.result.read_bytes()) if args.result.suffix=='.gz' else args.result.read_bytes())
        task.value=next(label for label,boxes in choices.items() if boxes==result['task'])
        load(result)
    print(f'Ready: http://127.0.0.1:{args.port}/',flush=True)
    tick=time.perf_counter()
    while True:
        renderer.drain()
        if play.value and 'result' in state and time.perf_counter()>=tick:
            result=state['result']
            index=int(frame.value)
            if index>=len(result['frames'])-1:play.value=False
            else:
                columns=[column for column,name in enumerate(result['joint_names']) if '_joint' in name]
                delta=max(abs(result['frames'][index+1][column]-result['frames'][index][column]) for column in columns)
                frame.value=index+1
                tick=time.perf_counter()+max(.015,delta/np.deg2rad(30))
        time.sleep(.005)


if __name__=='__main__':main()
