#!/usr/bin/env python3
"""Compose one continuous five-column top-pair run from validated components."""
import argparse, fcntl, json, math, time
from dataclasses import replace
from pathlib import Path
import numpy as np, torch, yaml
from curobo.collision_checking import RobotCollisionChecker, RobotCollisionCheckerCfg
from curobo_core.adapter import to_curobo_scene
from curobo_core.contracts import ACTIVE_JOINTS, PlannerAssets
from curobo_core.planner import FullCyclePlanner
from curobo_core.scene import Pose
from curobo_core.sequential import sequential_request
from curobo_core.sequential_validity import ReducedValidity
from v3_batched_loaded_search import GpuValidity, batched_rrt_connect_multi_goal, densify
from v3_plan_cycle import gpu_processes
from v3_stage_timing import stage_label, stage_method

ORDER=(2,1,3,4,0);STATIONS={2:0.,1:-.4,3:.4,4:.5,0:-.5}

def local_snapshot(world,base_y,q):
 objects=[]
 for item in world.objects:
  if item.object_id=='ground':objects.append(item);continue
  p=list(item.pose.position);p[1]-=base_y;objects.append(replace(item,pose=Pose(tuple(p),item.pose.quaternion_wxyz)))
 state=world.state.with_positions(dict(zip(ACTIVE_JOINTS,q)))
 return replace(world,objects=tuple(objects),state=state)

def empty_validity(planner,snapshot):
 checker=RobotCollisionChecker(RobotCollisionCheckerCfg.load_from_config(robot_config=planner.robot,
  scene_model=to_curobo_scene(snapshot),n_cuboids=max(40,len(snapshot.objects)),n_meshes=0,collision_activation_distance=0.))
 return GpuValidity(checker,check_ground=True,ground_z=snapshot.policy.ground_z_m,joint_names=list(ACTIVE_JOINTS),weights=[5.]+[1.]*14)

def plan_active(planner,start,target,active,snapshot,seed):
 if np.max(np.abs(start[active]-target[active]))<1e-8:return np.asarray([start]),{'skipped':True}
 check=empty_validity(planner,snapshot);reduced=ReducedValidity(check,start,active);lower,upper=check.checker.kinematics.get_joint_limits().position
 path,stats=batched_rrt_connect_multi_goal(torch.tensor(start[active],device='cuda',dtype=torch.float32),torch.tensor(np.asarray(target[active])[None],device='cuda',dtype=torch.float32),lower[active]+1e-5,upper[active]-1e-5,reduced,2.,seed)
 if path is None:return None,stats
 expanded=reduced.expand(torch.tensor(path,device='cuda',dtype=torch.float32)).cpu().numpy();dense=np.asarray(densify(expanded));return (dense if bool(check.mask(torch.tensor(dense,device='cuda',dtype=torch.float32)).all()) else None),stats

def plan_multi(planner,start,goals,active,snapshot,seed):
 check=empty_validity(planner,snapshot);reduced=ReducedValidity(check,start,active);lower,upper=check.checker.kinematics.get_joint_limits().position
 path,stats=batched_rrt_connect_multi_goal(torch.tensor(start[active],device="cuda",dtype=torch.float32),torch.tensor(np.asarray(goals)[:,active],device="cuda",dtype=torch.float32),lower[active]+1e-5,upper[active]-1e-5,reduced,2.,seed)
 if path is None:return None,stats
 expanded=reduced.expand(torch.tensor(path,device="cuda",dtype=torch.float32)).cpu().numpy();dense=np.asarray(densify(expanded));return (dense if bool(check.mask(torch.tensor(dense,device="cuda",dtype=torch.float32)).all()) else None),stats


def move_posture(planner,q,target,snapshot,seed):
 frames=[];stats=[]
 for active in (list(range(1,8)),list(range(8,15)),[0]):
  path,detail=plan_active(planner,q,target,active,snapshot,seed+len(stats));stats.append(detail)
  if path is None:return None,stats
  frames.extend(path[1:]);q=path[-1]
 return np.asarray([q] if not frames else frames),stats

def mobile_route(planner,world,q,start_y,end_y):
 names=list(planner.mobile_robot.get('robot_cfg',planner.mobile_robot)['kinematics']['cspace']['joint_names']);checker=RobotCollisionChecker(RobotCollisionCheckerCfg.load_from_config(robot_config=planner.mobile_robot,scene_model=to_curobo_scene(world,mobile=True),n_cuboids=max(40,len(world.objects)),n_meshes=0,collision_activation_distance=0.));validity=GpuValidity(checker,check_ground=True,ground_z=world.policy.ground_z_m,joint_names=names,weights=[2.,2.,1.,5.]+[1.]*14);steps=max(1,math.ceil(abs(end_y-start_y)/.01));rows=[]
 for y in np.linspace(start_y,end_y,steps+1):
  mapping={**dict(zip(ACTIVE_JOINTS,q)),'base_x':0.,'base_y':float(y),'base_yaw':0.};rows.append([mapping.get(name,0.) for name in names])
 rows=np.asarray(rows);mask=validity.mask(torch.tensor(rows,device='cuda',dtype=torch.float32));return rows if bool(mask.all()) else None,names

def component(root,column,seed):
 if column in (1,2,3):
  name=(f'inner{column}-rear-support-home-s{seed}-20261010.json' if column in (1,3)
        else f'inner2-rear-s{seed}-20261010.json')
  r=json.loads((root/name).read_text())['result'];events=[]
  for event in r['predicted_scene_events']:
   row={key:event[key] for key in ('phase','object_id','frame_index','world_pose','target_world_pose','translation_error_m','rotation_error_deg') if key in event}
   if event['phase'].endswith('_attach'):
    row['attachment']=next(x for x in event['snapshot']['attachments'] if x['object_id']==event['object_id'])
   events.append(row)
  measured=r.get('measured_wall_ms',sum(r.get('stage_timing_ms',{}).values()))
  method='双臂顺序：cuRobo IK + GPU RRTConnect + 1cm解析笛卡尔 + 左右车后放置'
  stage_rows=[{'stage':stage_label(phase),'ms':ms,'method':stage_method(phase,r)}
              for phase,ms in r.get('stage_timing_ms',{}).items()]
  return np.asarray(r['frames']),list(r['phases']),np.asarray(r['frames'][0]),np.asarray(r['frames'][-1]),[15+column,20+column],events,measured,method,stage_rows
 u=json.loads((root/f'outer{column}-upper-rear-s{seed}-20261010.json').read_text());ua=next(x for x in u['attempts'] if x.get('candidate')==u['selected_candidate'] and x.get('trajectory_frames'));l=json.loads((root/f'outer{column}-pair-rear-s{seed}-20261010.json').read_text());la=l['selected'];frames=np.asarray(ua['trajectory_frames']+la['trajectory_frames']);phases=list(ua['trajectory_phases']+la['trajectory_phases']);upper_n=len(ua['trajectory_frames']);upper_id=f'wall_box_{20+column:02d}';lower_id=f'wall_box_{15+column:02d}';upper_attach=next(i for i,x in enumerate(ua['trajectory_phases']) if x.endswith('_lateral'));lower_attach=next(i for i,x in enumerate(la['trajectory_phases']) if x.endswith('_lower_extract'));upper_release=upper_n-len(ua['rear_place']['frames'])+ua['rear_place']['events'][-1]['frame_index'];lower_release=len(frames)-len(la['rear_place']['frames'])+la['rear_place']['events'][-1]['frame_index'];events=[{'phase':ua['trajectory_phases'][upper_attach],'object_id':upper_id,'frame_index':upper_attach,'attachment':ua['attachment']},{**{key:value for key,value in ua['rear_place']['events'][-1].items() if key!='snapshot'},'frame_index':upper_release},{'phase':la['trajectory_phases'][lower_attach],'object_id':lower_id,'frame_index':upper_n+lower_attach,'attachment':la['attachment']},{**{key:value for key,value in la['rear_place']['events'][-1].items() if key!='snapshot'},'frame_index':lower_release}];measured=u.get('measured_wall_ms',0.)+l.get('measured_wall_ms',0.)
 method='近侧单臂：RRTConnect接近 + 内移/下降/抽离 + 两次车后放置'
 labels={'contact_ik':'接触IK与候选筛选','lateral':'向内横移0.4m','lower':'下降0.4m','extract':'向外抽离0.36m','approach_contact':'RRTConnect接近与5cm接触','rear_place':'搬到车辆后方并释放','other':'候选附件与碰撞验证'}
 methods={'contact_ik':'cuRobo IK（512/500/return32）','lateral':'1cm解析笛卡尔','lower':'升降+单臂1cm解析笛卡尔','extract':'1cm解析笛卡尔，必要时局部冗余续解','approach_contact':'GPU RRTConnect + 解析接触','rear_place':'cuRobo IK + GPU RRTConnect','other':'cuRobo碰撞/附件/FK验证'}
 stage_rows=[]
 for prefix,data in (('上箱',u.get('stage_timing_ms',{})),('下箱',l.get('stage_timing_ms',{}))):
  stage_rows.extend({'stage':prefix+labels[key],'ms':ms,'method':methods[key]} for key,ms in data.items())
 return frames,phases,np.asarray(ua['approach_start_q']),np.asarray(la['final_q']),[15+column,20+column],events,measured,method,stage_rows

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--components',type=Path,required=True);p.add_argument('--seed',type=int,default=11);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 with open('/tmp/sevenova-curobo-gpu.lock','a') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);occupied=gpu_processes();
  if occupied:raise RuntimeError('GPU occupied: '+'; '.join(occupied))
  planner=FullCyclePlanner(PlannerAssets());base=sequential_request(planner);world=replace(base.snapshot,objects=tuple(x for x in base.snapshot.objects if x.object_id not in ('left_wall','right_wall')));poses=yaml.safe_load(planner.args.named_poses.read_text())['named_poses'];transit=np.asarray([poses['second_home'][n] for n in ACTIVE_JOINTS]);q=np.asarray([planner.home[n] for n in ACTIVE_JOINTS]);base_y=0.;joint_names=['base_x','base_y','base_yaw',*ACTIVE_JOINTS];frames=[];phases=[];segments=[];lifecycle=[];timing_rows=[]
  def append15(rows,phase,y):
   for row in rows:
    frames.append([0.,y,0.,*map(float,row)]);phases.append(phase)
  for order_index,column in enumerate(ORDER):
   transition_started=time.perf_counter();target_y=STATIONS[column]
   if abs(target_y-base_y)>1e-9:
    local=local_snapshot(world,base_y,q);park_path,stats=move_posture(planner,q,transit,local,a.seed*100+order_index*20)
    if park_path is None:raise RuntimeError(f'column {column}: cannot reach transit posture: {stats}')
    append15(park_path,'park_for_base_move',base_y);q=park_path[-1]
    route,mobile_names=mobile_route(planner,world,q,base_y,target_y)
    if route is None:raise RuntimeError(f'column {column}: base route invalid')
    frames.extend(route.tolist());phases.extend(['base_translate']*len(route));base_y=target_y
   part,part_phases,start,end,removed,part_events,component_ms,method,stage_rows=component(a.components,column,a.seed);local=local_snapshot(world,base_y,q);to_start,stats=move_posture(planner,q,start,local,a.seed*100+10+order_index*20);join_index=0
   if to_start is None:
    first_approach_phase=next((phase for phase in part_phases if 'precontact' in phase or 'approach' in phase),None)
    approach_indices=[i for i,phase in enumerate(part_phases) if phase==first_approach_phase]
    sampled=approach_indices[::max(1,len(approach_indices)//32)]
    if approach_indices and approach_indices[-1] not in sampled:sampled.append(approach_indices[-1])
    active=[0]+(list(range(1,8)) if column==4 else list(range(8,15)))
    connection,connection_stats=plan_multi(planner,q,part[sampled],active,local,a.seed*100+15+order_index*20)
    stats.append(connection_stats)
    if connection is None:raise RuntimeError(f'column {column}: cannot connect component approach: {stats}')
    join_index=sampled[int(connection_stats['selected_goal'])];to_start=connection
   append15(to_start,'prepare_column',base_y);q=to_start[-1]
   if np.max(np.abs(q-part[join_index]))>1e-5:raise RuntimeError(f'column {column}: join mismatch')
   transition_ms=(time.perf_counter()-transition_started)*1000
   timing_rows.append({'column':column,'transition_ms':transition_ms,'component_ms':component_ms,
                       'total_ms':transition_ms+component_ms,'method':method,
                       'stages':[{'stage':'列间准备与底盘横移','ms':transition_ms,'method':'GPU RRTConnect + 1cm底盘插值'}]+stage_rows})
   component_start=len(frames);append15(part[join_index:],'column_'+str(column),base_y)
   for event in part_events:
    if event['frame_index']<join_index:raise RuntimeError(f'column {column}: join skipped lifecycle event {event}')
    lifecycle.append({**event,'column':column,'global_frame_index':component_start+event['frame_index']-join_index})
   q=end.copy();world=replace(world,objects=tuple(x for x in world.objects if x.object_id not in {f'wall_box_{i:02d}' for i in removed}));segments.append({'column':column,'base_y_m':base_y,'component_frames':len(part)-join_index,'component_join_index':join_index,'removed_boxes':removed})
  output={'success':True,'seed':a.seed,'kind':'composed_five_column_top_pairs','order':list(ORDER),'joint_names':joint_names,'frames':frames,'phases':phases,'segments':segments,'lifecycle':lifecycle,'planning_timing':{'total_ms':sum(x['total_ms'] for x in timing_rows),'transition_ms':sum(x['transition_ms'] for x in timing_rows),'component_ms':sum(x['component_ms'] for x in timing_rows),'rows':timing_rows,'note':'累计规划计算墙钟，不是播放或动力学执行时间'},'final_q15':q.tolist(),'final_base_y_m':base_y,'remaining_wall_boxes':sorted(x.object_id for x in world.objects if x.object_id.startswith('wall_box_'))}
  a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(output,indent=2)+'\n');print(json.dumps({'success':True,'frames':len(frames),'order':list(ORDER),'segments':segments,'remaining_boxes':len(output['remaining_wall_boxes'])},indent=2))
if __name__=='__main__':main()
