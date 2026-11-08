#!/usr/bin/env python3
"""Persistent-planner door benchmark: one resource warmup, then fresh no-plan-cache requests."""
import argparse
from dataclasses import replace
import fcntl
import hashlib
import json
from pathlib import Path
import subprocess
import time

from curobo_core.contracts import PlannerAssets
from curobo_core.door_recenter import DoorPlanner,DoorTask,ROOT,door_request


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--assets-root',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--wait-for-gpu-seconds',type=float,default=0.)
    a=p.parse_args();a.output_dir.mkdir(parents=True,exist_ok=True)
    if any(a.output_dir.iterdir()):p.error('output directory must be empty')
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
        assets=PlannerAssets();assets=replace(assets,**{name:a.assets_root/getattr(assets,name).relative_to(ROOT)
            for name in ('robot_config','mobile_robot_config','urdf','named_poses','box_fit')})
        policy_path=ROOT/'config/placement_task_policy.json';policy=json.loads(policy_path.read_text())
        planner=DoorPlanner(assets);records=[]
        source_identity={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
                         for path in sorted((ROOT/'tools').rglob('*.py'))}
        for index,seed in enumerate((11,11,29,41)):
            request=door_request(planner,seed);planner.set_snapshot(request.snapshot)
            task=DoorTask(planner,request,'joint_rrt',policy,progress=lambda _text:None)
            task.left_grasp_rolls_deg=(0.,30.,45.);task.right_grasp_rolls_deg=(0.,30.,-30.,45.,-45.)
            task.report['requested_left_grasp_rolls_deg']=list(task.left_grasp_rolls_deg)
            task.report['requested_right_grasp_rolls_deg']=list(task.right_grasp_rolls_deg)
            began=time.perf_counter();task.run();task.enter('time_parameterization');task.finalize_timing()
            task.enter('rnea_dynamics');task.apply_dynamics();task.close_stage_timing()
            task.report.update(total_ms=(time.perf_counter()-began)*1000,
                initial_snapshot=request.snapshot.to_dict(),final_snapshot=task.store.snapshot().to_dict(),
                source_identity=source_identity,model_id=planner.model_id,
                assets={name:dict(path=str(getattr(assets,name)),sha256=hashlib.sha256(getattr(assets,name).read_bytes()).hexdigest())
                        for name in ('robot_config','mobile_robot_config','urdf','named_poses','box_fit')},
                policy_sha256=hashlib.sha256(policy_path.read_bytes()).hexdigest(),
                plan_cache=dict(enabled=False,hit=False),resource_warmup=index==0)
            name='warmup-s11' if index==0 else f'measured-s{seed}'
            output=a.output_dir/(name+'.json');output.write_text(json.dumps(task.report,ensure_ascii=False)+'\n')
            for fixture_index,fixture in enumerate(task.report['recenter_inputs']):
                output.with_name(output.stem+f'-input-{fixture_index}-{fixture["stage"]}.json').write_text(
                    json.dumps(fixture,ensure_ascii=False)+'\n')
            records.append(dict(name=name,seed=seed,total_ms=task.report['total_ms'],success=task.report['success'],
                                stages=task.report['stage_timing_ms']))
            print(json.dumps(records[-1],ensure_ascii=False),flush=True)
        summary=dict(success=all(item['success'] for item in records),trajectory_cache=False,
                     resource_warmup=records[0],measured=records[1:])
        (a.output_dir/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
        return 0 if summary['success'] else 1

if __name__=='__main__':raise SystemExit(main())
