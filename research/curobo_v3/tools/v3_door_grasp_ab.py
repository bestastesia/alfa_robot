#!/usr/bin/env python3
"""Run isolated full-flow door grasp variants without changing planner safety gates."""
import argparse
import json
import os
from pathlib import Path
import subprocess


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--rolls-deg',default='0,30,45')
    p.add_argument('--seeds',default='11')
    p.add_argument('--assets-root',type=Path,required=True)
    p.add_argument('--wait-for-gpu-seconds',type=float,default=7200.)
    p.add_argument('--output-dir',type=Path,required=True)
    a=p.parse_args()
    rolls=[float(x) for x in a.rolls_deg.split(',')]
    seeds=[int(x) for x in a.seeds.split(',')]
    if a.output_dir.exists() and any(a.output_dir.iterdir()):
        p.error('output directory must be absent or empty')
    a.output_dir.mkdir(parents=True,exist_ok=True)
    runner=Path(__file__).with_name('v3_door_recenter.py')
    records=[]
    for seed in seeds:
        for roll in rolls:
            label=str(roll).replace('-','m').replace('.','p')
            result=a.output_dir/f'roll-{label}-seed-{seed}.json'
            log=result.with_suffix('.log')
            command=[str(Path('/home/astesia/Sevenova/curobo_v2/.venv/bin/python')),'-u',str(runner),
                     '--assets-root',str(a.assets_root),'--algorithm','joint_rrt','--seed',str(seed),
                     '--left-grasp-rolls-deg',str(roll),'--wait-for-gpu-seconds',str(a.wait_for_gpu_seconds),
                     '--output',str(result)]
            with log.open('w') as stream:
                completed=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,
                    env={**os.environ,'OMP_NUM_THREADS':'4','OPENBLAS_NUM_THREADS':'4'})
            document=json.loads(result.read_text()) if result.exists() else {}
            selected=next((item for item in document.get('contact_geometry',())
                           if item.get('side')=='left' and item.get('selected')),None)
            precontact=[item for item in document.get('precontact_searches',()) if item.get('side')=='left']
            record=dict(seed=seed,requested_roll_deg=roll,exit_code=completed.returncode,
                        success=document.get('success'),error=document.get('error'),
                        selected_roll_deg=None if selected is None else selected['roll_rad']*180/3.141592653589793,
                        lower_precontact=precontact[-1] if precontact else None,
                        motion_duration_s=document.get('motion_duration_s'),total_ms=document.get('total_ms'),
                        compact_radius_m=(document.get('dual_recenter_verification') or {}).get(
                            'actual_whole_robot_payload_radius_m'),result=str(result),log=str(log))
            records.append(record)
            (a.output_dir/'progress.json').write_text(json.dumps(records,ensure_ascii=False,indent=2)+'\n')
            print(json.dumps(record,ensure_ascii=False),flush=True)
    (a.output_dir/'summary.json').write_text(json.dumps({'runs':records},ensure_ascii=False,indent=2)+'\n')


if __name__=='__main__':main()
