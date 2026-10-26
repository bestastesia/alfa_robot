import argparse
import json
from pathlib import Path

import numpy as np
import torch
from curobo.types import JointState

from v3_wall25_demo import WallPlanner,pick_base_pose,SCHEDULE
from v3_rezero_task_planner import migrated_assets
from v3_random_error_runner import RandomErrorRunner
from v3_tcp_rezero_pipeline import tool_to_box
from v3_tool_rotation_metric import rotation_metrics
from v3_zero_model_candidate import ROOT


def pair_options(row):
    left_outer,left_inner=row*5+4,row*5+3
    right_outer,right_inner=row*5,row*5+1
    return [[{'left':left_outer,'right':right_outer},{'left':left_inner,'right':right_inner}],
        [{'left':left_outer,'right':right_inner},{'left':left_inner,'right':right_outer}],
        [{'left':left_inner,'right':right_outer},{'left':left_outer,'right':right_inner}],
        [{'left':left_inner,'right':right_inner},{'left':left_outer,'right':right_outer}]]


def search(folder):
    folder.mkdir(parents=True,exist_ok=True)
    planner=WallPlanner(migrated_assets());runner=RandomErrorRunner();runner.planner=planner
    cfg=json.loads((ROOT/'config/tcp_scene_preview.json').read_text());h=json.loads((ROOT/'config/rezero_home.json').read_text())
    home=dict(cfg['home']);home['updown']=h['updown_m']
    home.update({f'{side}_joint{i}':float(np.deg2rad(angle)) for side in ('left','right') for i,angle in enumerate(h[side+'_deg'],1)})
    front=next(o['pose']['position'][0]-o['dimensions_m'][0]/2 for o in cfg['snapshot']['objects'] if o['object_id']=='front_wall')
    boxes={}
    for box in range(25):
        pose=np.eye(4);pose[:3,3]=[front-.20,(box%5-2)*.41,.201+(box//5)*.41];boxes[box]=pose
    planner.world_boxes=boxes;cache={};attempts=[];rounds=[];baseline=[];remaining=set(range(25));summary=[];original=[]

    def plan(task,present):
        key=(tuple(sorted(task.items())),tuple(sorted(present)))
        if key in cache:return cache[key]
        planner.demo_task=task;planner.remaining_boxes=set(present)
        base=pick_base_pose(task);mode='top' if min(task.values())//5<=1 else 'side'
        evidence=dict(task_label=str(task),parking_world_from_footprint=base.tolist(),parking_yaw_deg=0.,inputs={})
        for side,box in task.items():evidence['inputs'][side]=dict(grasp_mode=mode,tcp_in_footprint=(np.linalg.inv(base)@boxes[box]@np.linalg.inv(tool_to_box(mode))).tolist(),roll_pitch_yaw_deg=[0,0,0])
        index=len(attempts)+1;print('CANDIDATE',index,task,'remaining',len(present),flush=True)
        result=runner.run(evidence,home,folder/f'candidate_{index:03d}',lambda message:None)
        result.update(wall_task=task,remaining_before=sorted(present))
        if result['success']:
            names=result['joint_names'];values=np.asarray(result['frames'],dtype=np.float32)
            checker=planner.checker(planner.selected_task)
            qnames=checker.kinematics.joint_names
            quats={'left':[],'right':[]}
            for offset in range(0,len(values),512):
                q=torch.tensor(values[offset:offset+512][:,[names.index(n) for n in qnames]],device='cuda').contiguous()
                poses=checker.kinematics.compute_kinematics(JointState.from_position(q[None],joint_names=qnames)).tool_poses.to_dict()
                for side in quats:quats[side].extend(poses[side+'_tool0'].quaternion.reshape(-1,4).cpu().double().tolist())
            result['rotation_metrics']=rotation_metrics(quats,result['phases'])
            if base[0,3]:
                initial=values[0].copy();initial[:3]=0.
                result['frames'].insert(0,initial.tolist());result['phases'].insert(0,'navigation_to_pick');result['payload'].insert(0,False)
            final=np.asarray(result['frames'][-1]).copy();final[:3]=0.
            result['frames'].append(final.tolist());result['phases'].append('base_return');result['payload'].append(False)
        cache[key]=result;attempts.append(dict(task=task,remaining=sorted(present),success=result['success'],
            blocked=result.get('blocked_stage'),metrics=result.get('rotation_metrics'),file=str(folder/f'candidate_{index:03d}'/'result.json')))
        print('RESULT',index,result['success'],result.get('blocked_stage'),(result.get('rotation_metrics') or {}).get('combined_extra_deg'),flush=True)
        (folder/'attempts.json').write_text(json.dumps(attempts,ensure_ascii=False))
        return result

    def save():
        result=dict(world_boxes={str(key):value.tolist() for key,value in boxes.items()},rounds=rounds,
            completed_boxes=sorted(set(range(25))-remaining),success=not remaining,
            scope='整轮两末端分阶段额外旋转评分；同层四种双臂配对/顺序比较，未宣称全局最优',
            optimization=summary,baseline_extra_deg=sum(x['rotation_metrics']['combined_extra_deg'] for x in (original or baseline)),
            baseline_scope='原13轮直觉顺序重新规划' if original else '相同15轮下每层外侧先行顺序',
            chosen_extra_deg=sum(x['rotation_metrics']['combined_extra_deg'] for x in rounds))
        pending=folder/'wall.pending.json';pending.write_text(json.dumps(result,ensure_ascii=False));pending.replace(folder/'wall.json')

    for row in range(4,-1,-1):
        candidates=[]
        for option,pairs in enumerate(pair_options(row)):
            present=set(remaining);results=[]
            for task in pairs:
                result=plan(task,present)
                if not result['success']:break
                results.append(result);present-=set(task.values())
            if len(results)==2:
                candidates.append((sum(r['rotation_metrics']['combined_extra_deg'] for r in results),option,results))
        if not candidates:raise RuntimeError(f'第{5-row}层没有完整成功的配对方案，停止')
        base_candidate=next((value for value in candidates if value[1]==0),None)
        if base_candidate is None:raise RuntimeError('原顺序失败，无法同口径比较，不继续宣称优化')
        best=min(candidates,key=lambda value:value[0]);rounds.extend(best[2]);baseline.extend(base_candidate[2])
        for result in best[2]:remaining-=set(result['wall_task'].values())
        center=row*5+2;task={'left' if row>=2 else 'right':center};single=plan(task,remaining)
        if not single['success']:raise RuntimeError(f'中箱{center}单臂失败，停止：{single.get("blocked_details")}')
        rounds.append(single);baseline.append(single);remaining.remove(center)
        summary.append(dict(row=5-row,chosen_option=best[1],baseline_extra_deg=base_candidate[0],chosen_extra_deg=best[0],
            options=[dict(option=item[1],extra_deg=item[0]) for item in candidates]))
        save();print('ROW_DONE',5-row,'completed',25-len(remaining),'EXTRA',best[0],'BASE',base_candidate[0],flush=True)
    present=set(range(25))
    for task in SCHEDULE:
        result=plan(task,present)
        if not result['success']:raise RuntimeError('原13轮重算失败，保留15轮结果但不作13轮数值对比')
        original.append(result);present-=set(task.values())
    save()
    original_manifest=json.loads((folder/'wall.json').read_text())
    original_manifest.update(rounds=original,chosen_extra_deg=sum(x['rotation_metrics']['combined_extra_deg'] for x in original))
    original_folder=folder/'original_13';original_folder.mkdir(exist_ok=True)
    (original_folder/'wall.json').write_text(json.dumps(original_manifest,ensure_ascii=False))
    print('COMPLETE_SELECTED',sum(x['rotation_metrics']['combined_extra_deg'] for x in rounds),
        'ORIGINAL13',sum(x['rotation_metrics']['combined_extra_deg'] for x in original),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--folder',type=Path,default=Path('/tmp/rotation_wall25_20261023'))
    search(parser.parse_args().folder)
