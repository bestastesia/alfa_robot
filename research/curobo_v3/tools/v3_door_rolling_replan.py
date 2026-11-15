#!/usr/bin/env python3
"""Snapshot rolling replan for a saved door recenter stage with optional new cuboid obstacle."""
import argparse
from dataclasses import replace
import fcntl
import json
import math
from pathlib import Path
import subprocess
import time

import numpy as np
import torch

from curobo_core.contracts import ACTIVE_JOINTS
from curobo_core.door_model import load_assets
from curobo_core.adapter import pose_matrix
from curobo_core.door_recenter import (DoorPlanner, DoorTask, ROOT, SupportCompensation,
                                       SupportCompensatedValidity, door_request)
from curobo_core.door_sequential_validity import ReducedValidity
from curobo_core.scene import Pose, SceneObject, SceneSnapshot, SceneStore, digest
from curobo_core.torque import default_torque_model
from curobo_core.trajectory import time_parameterize_polyline
from v3_batched_loaded_search import densify
from v3_multiroot_search import batched_rrt_connect_multi_goal


def vec(text,count):
    values=tuple(float(x) for x in text.split(','))
    if len(values)!=count or not np.isfinite(values).all():
        raise argparse.ArgumentTypeError(f'expected {count} finite comma-separated values')
    return values


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--result',type=Path,required=True)
    p.add_argument('--fixture',type=Path,required=True)
    p.add_argument('--fraction',type=float,default=.25)
    p.add_argument('--obstacle-center',type=lambda x:vec(x,3))
    p.add_argument('--obstacle-size',type=lambda x:vec(x,3))
    p.add_argument('--assets-root',type=Path,required=True)
    p.add_argument('--wait-for-gpu-seconds',type=float,default=0.)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('output exists')
    if not 0<=a.fraction<1 or (a.obstacle_center is None)!=(a.obstacle_size is None):p.error('invalid fraction/obstacle')
    with open('/tmp/sevenova-curobo-gpu.lock','a') as lock:
        deadline=time.monotonic()+a.wait_for_gpu_seconds
        while True:
            try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:occupied='shared lock held'
            else:
                occupied=subprocess.check_output(['nvidia-smi','--query-compute-apps=pid','--format=csv,noheader'],text=True).strip()
                if not occupied:break
                fcntl.flock(lock,fcntl.LOCK_UN)
            if time.monotonic()>=deadline:raise RuntimeError('GPU unavailable: '+occupied)
            time.sleep(1)
        result=json.loads(a.result.read_text());fixture=json.loads(a.fixture.read_text())
        if fixture.get('identity')!=digest({k:v for k,v in fixture.items() if k!='identity'}):raise ValueError('fixture identity mismatch')
        phase=fixture['stage'];indices=np.flatnonzero(np.asarray(result['phases'])==phase)
        if not len(indices):raise ValueError('phase absent from result')
        frame=int(indices[min(len(indices)-1,math.floor(a.fraction*len(indices)))])
        names=result['joint_names'];mapping=dict(zip(names,result['frames'][frame]));current=np.asarray([mapping[n] for n in ACTIVE_JOINTS])
        assets=load_assets(a.assets_root)
        planner=DoorPlanner(assets);request=door_request(planner,int(fixture['seed']));planner.set_snapshot(request.snapshot)
        policy=json.loads((ROOT/'config/placement_task_policy.json').read_text());task=DoorTask(planner,request,'joint_rrt',policy)
        snapshot=SceneSnapshot.from_dict(fixture['snapshot'])
        snapshot=replace(snapshot,state=snapshot.state.with_positions(dict(zip(ACTIVE_JOINTS,current))))
        if a.obstacle_center:
            obstacle=SceneObject('rolling_obstacle',a.obstacle_size,Pose(a.obstacle_center))
            snapshot=replace(snapshot,objects=snapshot.objects+(obstacle,),revision=snapshot.revision+1)
        task.store=SceneStore(snapshot);task.q=current;task.support_pose=(None if fixture['support_pose'] is None else np.asarray(fixture['support_pose']))
        task.phase=phase;candidates=np.asarray(fixture['candidates'],dtype=float);active=list(fixture['active_indices']);full=task.validity();mapper=None
        if fixture.get('derived_indices'):
            side='right' if set(fixture['derived_indices'])==set(range(8,15)) else 'left'
            mapper=SupportCompensation(task,side=side)
            reduced=SupportCompensatedValidity(full,current,mapper,searched_indices=active)
        else:reduced=ReducedValidity(full,current,active)
        good=full.mask(task.tensor(candidates));candidates=candidates[good.cpu().numpy()]
        if not len(candidates):raise RuntimeError('all saved goals invalid after obstacle update')
        lower,upper=full.checker.kinematics.get_joint_limits().position;started=time.perf_counter()
        sparse,stats=batched_rrt_connect_multi_goal(task.tensor(current[active]),task.tensor(candidates[:,active]),
            lower[active]+1e-5,upper[active]-1e-5,reduced,2.,int(fixture['seed'])+900,
            stop_at_first_valid=True,batch=256,connect_candidates=64,step=.28)
        if sparse is None:path=None
        else:
            dense=np.asarray(densify(np.asarray(sparse),reduced.weights.cpu().tolist()));path=reduced.expand(task.tensor(dense)).cpu().numpy()
        output=dict(success=path is not None,phase=phase,start_frame=frame,fraction=a.fraction,
                    obstacle=None if not a.obstacle_center else dict(center=a.obstacle_center,size=a.obstacle_size),
                    search_stats=stats,planning_ms=(time.perf_counter()-started)*1000)
        if path is not None:
            limits=full.checker.kinematics.get_joint_limits();caps=[np.abs(getattr(limits,k)[1].cpu().numpy()) for k in ('velocity','acceleration','jerk')]
            timed,validation=time_parameterize_polyline(path,ACTIVE_JOINTS,*caps,sample_dt=.025)
            if not bool(full.mask(task.tensor(validation)).all()):raise RuntimeError('timed rolling path collision')
            segment=timed['sampled_trajectory'];segment['joint_names']=list(ACTIVE_JOINTS);attached=snapshot.attachments;tool_to_box={x.parent_link.removesuffix('_tool0'):pose_matrix(x.tool_to_object).tolist() for x in attached}
            model=default_torque_model(assets,payload_mass_kg=10.,payload_dimensions=planner.box_fit['dimensions_m'],torque_limit_scale=.9)
            constrained,before,constraint=model.constrain(segment,tuple(sorted(tool_to_box)),tool_to_box,deadline=time.perf_counter()+3.)
            output.update(success=constrained is not None and constraint.get('success',False),path=path.tolist(),
                          timed=constrained,torque_before=before,torque_constraint=constraint)
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(output,ensure_ascii=False)+'\n')
        print(json.dumps({k:output.get(k) for k in ('success','phase','planning_ms','search_stats','torque_constraint')},ensure_ascii=False))
        raise SystemExit(0 if output['success'] else 1)

if __name__=='__main__':main()
