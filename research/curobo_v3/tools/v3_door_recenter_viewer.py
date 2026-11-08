#!/usr/bin/env python3
"""CPU-only replay of real door results (including failures); navigation jumps are labelled."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import viser
from viser.extras import ViserUrdf
import yaml
import yourdfpy
from curobo_core.adapter import pose_matrix
from curobo_core.contracts import PlannerAssets
from curobo_core.scene import Pose

LABELS = dict(time_parameterization='全周期时间参数化与复验',rnea_dynamics='RNEA动力学检查与重定时',initialize='初始化与初态验收',right_contact_model='上箱接触模型',left_contact_model='下箱接触模型',right_precontact='上箱：到预接触',right_contact='上箱：贴近',right_attach='上箱：吸附',
              left_precontact='下箱：到预接触',left_contact='下箱：贴近',left_attach='下箱：吸附',
              left_extract='下箱：抽离',left_recenter='下箱回中',right_lower='携带下箱，上箱下降',right_extract='保持下箱，上箱抽离',right_recenter='上箱回中',
              dual_recenter_align='双箱共同对齐',base_reposition='导航到位切换（非旋转轨迹）',
              conveyor_place='沿用普通放箱',release='放箱释放',return_home='空手回位',base_return='导航返回切换')


TIMING_GROUPS = (
    ('上箱支撑与吸附', ('initialize','right_contact_model','right_precontact','right_contact','right_attach')),
    ('下箱抓取、抽离与回中', ('left_contact_model','left_precontact','left_contact','left_attach','left_extract','left_recenter')),
    ('上箱下降、抽离与双箱收束', ('right_lower','right_extract','right_recenter','dual_recenter_align')),
    ('普通放箱、释放与回位', ('base_reposition','conveyor_place','release','return_home','base_return')),
    ('全周期时间化与动力学', ('time_parameterization','rnea_dynamics')),
)


def timing_stage_order(result):
    order=[]
    for span in result.get('stage_timing_spans',()):
        if span['stage'] not in order:order.append(span['stage'])
    for stage in result.get('stage_timing_ms',{}):
        if stage not in order:order.append(stage)
    return order


def motion_durations(result):
    return {item['name']:float(item.get('duration_s',0.)) for item in result.get('timed_segments',())}


def load_round_results(result,result_path):
    rounds=result.get('rounds',())
    if not rounds:return (result,)
    ranges=result.get('round_frame_ranges',())
    if len(rounds)!=len(ranges):raise ValueError('round results do not match frame ranges')
    base=Path(result_path).parent
    return tuple(json.loads((Path(item['path']) if Path(item['path']).is_absolute() else base/item['path']).read_text())
                 for item in rounds)


def summary_markdown(result):
    selected=next((item for item in result.get('contact_geometry',())
                   if item.get('side')=='left' and item.get('selected')),None)
    torque=result.get('torque_validation',{})
    compact=result.get('dual_recenter_verification',{})
    selected_roll='未选' if selected is None else f"{np.degrees(selected['roll_rad']):.1f}°"
    dynamics='通过' if torque.get('success') else '未通过/未启用'
    cache=result.get('plan_cache',{})
    cache_summary=(f"**缓存命中后的计算结算：** {result.get('total_ms',0)/1000:.3f}s  \n**缓存失效时的完整冷规划：** {cache.get('origin_total_ms',0)/1000:.3f}s  \n" if cache.get('hit') else '')
    return (f"**完整成功：** {result.get('success')}  \n"+cache_summary+
            f"**左下箱抓取滚转：** {selected_roll}  \n"
            f"**机器人轨迹执行时长（规划完成后）：** {result.get('motion_duration_s','未时间化')} s  \n"
            f"**收束包络：** {compact.get('actual_whole_robot_payload_radius_m','未记录')} m / 1.05 m  \n"
            f"**RNEA：** {dynamics}"
            + (f"；峰值利用率 {torque.get('maximum_utilization',0):.3f}" if torque else ''))


def overall_timing_markdown(result):
    stage_ms=result.get('stage_timing_ms',{})
    motion=motion_durations(result)
    cache=result.get('plan_cache',{})
    settlement=float(result.get('total_ms',sum(stage_ms.values())))
    total=float(cache.get('origin_total_ms',settlement) if cache.get('hit') else settlement)
    cache_line=(f"**认证轨迹缓存命中：缓存读取与结果结算 {settlement/1000:.3f}s；缓存失效时冷规划 {total/1000:.3f}s。**  "
                if cache.get('hit') else f"**算法内部端到端计算：{total/1000:.3f}s**  ")
    lines=["## 端到端计算耗时汇总",
           cache_line+f"**规划轨迹运动：{result.get('motion_duration_s',0):.3f}s**  ",
           "计算时间包含规划、碰撞验收、时间化与RNEA；不包含GPU锁等待、进程冷启动、模型加载和真实导航执行。", "",
           "| 大阶段 | 计算墙钟 | 占比 | 轨迹运动 |", "|---|---:|---:|---:|"]
    covered=set()
    for label,stages in TIMING_GROUPS:
        wall=sum(float(stage_ms.get(stage,0.)) for stage in stages)
        run=sum(motion.get(stage,0.) for stage in stages)
        covered.update(stages)
        lines.append(f"| {label} | {wall/1000:.3f}s | {wall/max(total,1)*100:.1f}% | {run:.3f}s |")
    other=sum(float(value) for stage,value in stage_ms.items() if stage not in covered)
    if other:lines.append(f"| 其他 | {other/1000:.3f}s | {other/max(total,1)*100:.1f}% | 0.000s |")
    lines += [f"| **合计** | **{sum(stage_ms.values())/1000:.3f}s** | **100.0%** | **{sum(motion.values()):.3f}s** |", "",
              "### 各阶段", "| 阶段 | 计算墙钟 | 占比 | 轨迹运动 |", "|---|---:|---:|---:|"]
    for stage in timing_stage_order(result):
        wall=float(stage_ms.get(stage,0.))
        lines.append(f"| {LABELS.get(stage,stage)} | {wall/1000:.3f}s | {wall/max(total,1)*100:.1f}% | {motion.get(stage,0.):.3f}s |")
    timing=result.get('timing_ms',{})
    lines += ["", "**全流程内部诊断（已包含在阶段墙钟中，不重复相加）：** " + " · ".join(
        f"{label} {float(timing.get(key,0))/1000:.3f}s" for key,label in
        (('ik','IK'),('search','RRT/搜索'),('cartesian','笛卡尔生成/验收'),('trajectory_optimization','轨迹优化')))]
    return '\n'.join(lines)


def stage_timing_markdown(result,stage,minimum_ms=10.):
    wall=float(result.get('stage_timing_ms',{}).get(stage,0.))
    detail=result.get('stage_breakdown_ms',{}).get(stage,{})
    motion=motion_durations(result).get(stage,0.)
    additive=[(key,float(detail[key])) for key in ('ik','search','cartesian','audit','trajectory_optimization') if detail.get(key) is not None]
    residual=max(0.,wall-sum(value for _,value in additive))
    names=dict(ik='IK生成与筛选',search='RRT/任务空间搜索',cartesian='笛卡尔路径生成',audit='独立轨迹验收',trajectory_optimization='轨迹优化',residual='候选生成、检查器构建、时间化/RNEA及其他')
    rows=["| 可相加子阶段 | 耗时 |", "|---|---:|"]
    rows += [f"| {names[key]} | {value:.2f} ms |" for key,value in additive if value>=minimum_ms]
    if residual>=minimum_ms:rows.append(f"| {names['residual']} | {residual:.2f} ms |")
    rows.append(f"| **阶段合计** | **{wall:.2f} ms** |")
    collision=float(detail.get('collision',0.))
    if collision>=minimum_ms:
        rows += ["",f"**碰撞/FK诊断：{collision:.2f} ms**（包含在搜索、验收等子阶段中，不可重复相加）。"]
    spans=[item for item in result.get('stage_timing_spans',()) if item['stage']==stage]
    visible=[item for item in spans if item['wall_ms']>=minimum_ms]
    if visible:
        rows += ["",f"**调用明细：{len(spans)}次；显示≥{minimum_ms:g}ms**", "", "| 调用 | 起始 | 墙钟 |", "|---:|---:|---:|"]
        rows += [f"| {i+1} | {item['start_ms']/1000:.3f}s | {item['wall_ms']:.2f} ms |" for i,item in enumerate(visible)]
        hidden=len(spans)-len(visible)
        if hidden:rows.append(f"\n另有 **{hidden}** 次小于{minimum_ms:g}ms的调用。")
    return (f"## {LABELS.get(stage,stage)}  \n**阶段计算墙钟：{wall/1000:.3f}s** · "
            f"**轨迹运动时间：{motion:.3f}s**  \n"+'\n'.join(rows))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--result',type=Path,required=True)
    p.add_argument('--port',type=int,default=8143)
    a=p.parse_args(); result=json.loads(a.result.read_text())
    round_results=load_round_results(result,a.result)
    frames=result['frames']; phases=result['phases']; names=result['joint_names']
    if not frames:
        q=result.get('failure_location',{}).get('joint_positions')
        state=result['initial_snapshot']['state']
        mapping=dict(zip(state['joint_names'],state['positions']))
        frames=[q if q is not None else [mapping.get(n,0.) for n in names]]
        phases=[result.get('error',{}).get('stage','failure')]
    model_config=Path(result.get('assets',{}).get('mobile_robot_config',{}).get('path',PlannerAssets().mobile_robot_config))
    kin=yaml.safe_load(model_config.read_text())
    kin=kin.get('robot_cfg',kin)['kinematics']
    model=yourdfpy.URDF.load(kin['urdf_path'],load_meshes=True,build_scene_graph=True)
    server=viser.ViserServer(host='127.0.0.1',port=a.port,label='门扉双箱：依次回中')
    server.scene.set_up_direction('+z')
    robot=ViserUrdf(server,model,root_node_name='/robot')
    world={}
    for item in result['initial_snapshot']['objects']:
        pose=item['pose']; oid=item['object_id']
        world[oid]=server.scene.add_box('/world/'+oid,dimensions=item['dimensions_m'],position=pose['position'],
            wxyz=pose['quaternion_wxyz'],color=(180,139,90) if oid.startswith('wall_box') else (110,130,150),opacity=.7)
    for item in result['initial_snapshot'].get('meshes',[]):
        server.scene.add_mesh_simple('/world/'+item['object_id'],vertices=np.asarray(item['vertices_m']),
            faces=np.asarray(item['faces']),position=item['pose']['position'],wxyz=item['pose']['quaternion_wxyz'],color=(100,120,140))
    targets=[]
    policy=result.get('carry_policy',{}).get('side')
    if policy:
        for side in ('left','right'):
            position=[policy['position']['x'],policy['position']['y_'+side],policy['position']['z']]
            targets.append(server.scene.add_frame('/carry_target/'+side,position=position,
                wxyz=policy['quaternion_wxyz'],axes_length=.15,axes_radius=.005))
    navigation_index=phases.index('base_reposition') if 'base_reposition' in phases else len(phases)
    loads={side:server.scene.add_box('/payload/'+side,dimensions=(.3,.4,.4),color=(80,180,210)) for side in ('left','right')}
    slider=server.gui.add_slider('帧',min=0,max=max(1,len(frames)-1),step=1,initial_value=0)
    play=server.gui.add_checkbox('播放',initial_value=False)
    speed=server.gui.add_slider('速度',min=.25,max=10,step=.25,initial_value=1)
    stage=server.gui.add_dropdown('阶段',options=tuple(dict.fromkeys(phases)))
    round_ranges=result.get('round_frame_ranges',())
    round_options=tuple(f"{item['round']+1} · 列{5-item['column']} · 箱{item['upper_box']:02d}/{item['lower_box']:02d}"
                        for item in round_ranges)
    round_selector=server.gui.add_dropdown('列轮次',options=round_options) if round_ranges else None
    status=server.gui.add_markdown('')
    def selected_timing_result():
        return result if round_selector is None else round_results[round_options.index(round_selector.value)]
    timing_result=selected_timing_result()
    summary=server.gui.add_markdown(summary_markdown(timing_result))
    timing_options=tuple(timing_stage_order(timing_result))
    with server.gui.add_folder('端到端计算耗时',expand_by_default=True):
        overall_timing=server.gui.add_markdown(overall_timing_markdown(timing_result))
    with server.gui.add_folder('阶段 / 子阶段耗时',expand_by_default=True):
        timing_stage=server.gui.add_dropdown('耗时阶段',options=timing_options,initial_value=timing_options[0])
        timing_detail=server.gui.add_markdown(stage_timing_markdown(timing_result,timing_options[0]))
    @timing_stage.on_update
    def _(_event):timing_detail.content=stage_timing_markdown(selected_timing_result(),timing_stage.value)
    times=result.get('time_from_start_s')
    def update():
        i=min(int(slider.value),len(frames)-1)
        for target in targets:target.visible=phases[i] not in ('base_reposition','conveyor_place','release','return_home','base_return')
        values=dict(zip(names,frames[i])); q=[values.get(n,0.) for n in model.actuated_joint_names]
        model.update_cfg(q); robot.update_cfg(q)
        held=(result.get('attachments_by_frame') or [[]])[min(i,len(result.get('attachments_by_frame') or [[]])-1)]
        removed={e['object_id'] for e in result.get('predicted_scene_events',[]) if e['frame_index']<=i and 'release' in e['phase']}
        for oid,node in world.items():node.visible=oid not in removed and oid not in {x['object_id'] for x in held}
        for node in loads.values():node.visible=False
        for item in held:
            side=item['parent_link'].removesuffix('_tool0')
            T=model.get_transform(item['parent_link'],model.base_link)@pose_matrix(Pose(**item['tool_to_object']))
            loads[side].position=T[:3,3];loads[side].wxyz=np.roll(Rotation.from_matrix(T[:3,:3]).as_quat(),1);loads[side].visible=True
        clock=f'{times[i]:.2f}s' if times else '未时间化：帧索引，不是执行时间'
        status.content=f"**{LABELS.get(phases[i],phases[i])}** · {clock}\n\n算法：{result.get('recenter_algorithm')}；成功：{result['success']}\n\n{result.get('error','')}\n\n导航为状态切换；不代表连续转身或实机安全验证。"
    @slider.on_update
    def _(_event):update()
    @stage.on_update
    def _(_event):
        play.value=False
        if round_selector is None:
            slider.value=phases.index(stage.value)
        else:
            selected=round_options.index(round_selector.value)
            bounds=round_ranges[selected]
            slider.value=next((i for i in range(bounds['start'],bounds['end']+1) if phases[i]==stage.value),bounds['start'])
    if round_selector is not None:
        @round_selector.on_update
        def _(_event):
            play.value=False;selected=round_options.index(round_selector.value)
            timing_result=round_results[selected]
            summary.content=summary_markdown(timing_result)
            overall_timing.content=overall_timing_markdown(timing_result)
            timing_stage.options=tuple(timing_stage_order(timing_result))
            timing_detail.content=stage_timing_markdown(timing_result,timing_stage.value)
            slider.value=round_ranges[selected]['start']
    update()
    while True:
        if play.value:
            i=min(int(slider.value),len(frames)-1)
            if i>=len(frames)-1:play.value=False
            else:
                delay=max(0.,times[i+1]-times[i])/speed.value if times else .025/speed.value
                time.sleep(delay);slider.value=i+1
        else:time.sleep(.03)

if __name__=='__main__':main()
