import argparse
from collections import Counter
import contextlib
import gzip
import hashlib
import io
import json
from pathlib import Path

import numpy as np
import torch
from v3_frozen_cycle import FrozenCyclePlanner,PROFILE
from v3_wall_ik_benchmark import ACTIVE_JOINTS
from curobo_core.distance_metric import joint_distance_weights


OUTPUT=Path(__file__).resolve().parents[1]/'generated/frozen_cycle_release'


def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    data=json.dumps(value,ensure_ascii=False,allow_nan=False).encode()
    if path.suffix=='.gz':data=gzip.compress(data,mtime=0)
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_bytes(data)
    temporary.replace(path)


def check_cycle(result,home):
    names=result['joint_names']
    columns=[names.index(name) for name in ACTIVE_JOINTS]
    rows=np.asarray(result['frames'])[:,columns]
    phases=np.asarray(result['phases'])
    placement=np.flatnonzero(phases=='conveyor_place')
    returned=np.flatnonzero(phases=='return_home')
    report=dict(actual_lm_to_home_join=None,home_error=None)
    if len(returned):
        if not len(placement) or not result.get('conveyor_place_lm',{}).get('success'):
            raise AssertionError('Home cannot follow an absent/failed LM placement')
        join=float(np.max(np.abs(np.asarray(result['conveyor_home_start_joints'])-rows[placement[-1]])))
        if join>1e-5:raise AssertionError('Home does not start at actual LM endpoint')
        step=float(np.max(np.abs((rows[returned[0]]-rows[placement[-1]])*joint_distance_weights(ACTIVE_JOINTS))))
        if step>np.deg2rad(.501):raise AssertionError('discontinuity after exported release frame')
        report['actual_lm_to_home_join']=join
        report['first_return_step']=step
    if result['success']:
        if not len(returned):raise AssertionError('complete success requires Home stage')
        error=float(np.max(np.abs(rows[-1]-np.asarray([home[name] for name in ACTIVE_JOINTS]))))
        if error>1e-5:raise AssertionError('complete success did not return Home')
        report['home_error']=error
        if len(result['restored_phase_audits'])!=6 or not all(item['valid'] for item in result['restored_phase_audits']):
            raise AssertionError('complete success is missing a phase acceptance')
    if result.get('conveyor_place_lm',{}).get('rrt_fallback'):
        raise AssertionError('LM placement must not fall back to RRT')
    return report


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--indices',type=int,nargs='+')
    parser.add_argument('--output',type=Path,default=OUTPUT)
    args=parser.parse_args()
    planner=FrozenCyclePlanner()
    model_hashes={name:hashlib.sha256(Path(getattr(planner.args,name)).read_bytes()).hexdigest() for name in ('robot_config','mobile_robot_config','urdf')}
    algorithm_hashes={name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest() for name in ('v3_lm_conveyor.py','v3_frozen_cycle.py')}
    records=[]
    for index,(label,task) in enumerate(planner.task_options):
        if args.indices and index not in args.indices:continue
        path=args.output/f'{index:03d}.json.gz'
        if path.exists():
            result=json.loads(gzip.decompress(path.read_bytes()))
            if result.get('cycle_profile')!=PROFILE:raise RuntimeError('obsolete full-cycle recording')
            if result.get('validation_model_hashes')!=model_hashes:raise RuntimeError('model changed; choose a new validation output directory')
            if result.get('validation_algorithm_hashes')!=algorithm_hashes:raise RuntimeError('algorithm changed; choose a new validation output directory')
        else:
            np.random.seed(20261031)
            torch.manual_seed(20261031)
            with contextlib.redirect_stdout(io.StringIO()):result=planner.full_cycle(task)
            result['cycle_seams']=check_cycle(result,planner.home)
            result['validation_model_hashes']=model_hashes
            result['validation_algorithm_hashes']=algorithm_hashes
            save(path,result)
        records.append(dict(index=index,label=label,task=task,success=result['success'],blocked_stage=result.get('blocked_stage'),
            blocked_details=result.get('blocked_details'),wall_ms=result['wall_ms'],lm=result.get('conveyor_place_lm'),result_file=path.name))
        summary=dict(profile=PROFILE,total=len(planner.task_options),completed=len(records),success=sum(item['success'] for item in records),
            failures=dict(Counter(item['blocked_stage'] for item in records if not item['success'])),records=records)
        summary['model_hashes']=model_hashes
        summary['algorithm_hashes']=algorithm_hashes
        save(args.output/'summary.json',summary)
        print('RESULT',index,label,result['success'],result.get('blocked_stage'),round(result['wall_ms']),flush=True)
    print('COMPLETE',summary['success'],summary['completed'],summary['failures'],flush=True)


if __name__=='__main__':main()
