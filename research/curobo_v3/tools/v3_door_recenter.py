#!/usr/bin/env python3
"""Single-column door pair, with selectable recenter search and a saved-input A/B entry."""
import argparse
from dataclasses import replace
import fcntl
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import time
import traceback

import numpy as np
from curobo_core.contracts import PlannerAssets
from curobo_core.door_recenter import DoorPlanner, DoorTask, SupportCompensation, ROOT, door_request
from curobo_core.scene import SceneSnapshot, SceneStore, digest


CACHE_SOURCES = (
    'tools/v3_door_recenter.py','tools/curobo_core/door_recenter.py',
    'tools/curobo_core/door_sequential.py','tools/curobo_core/door_sequential_validity.py',
    'tools/curobo_core/door_extract.py','tools/curobo_core/torque.py',
    'tools/curobo_core/backend.py','tools/curobo_core/planner.py',
    'tools/curobo_core/scene.py','tools/curobo_core/trajectory.py',
    'tools/v3_cartesian_transport.py','tools/v3_cartesian_dual_transport.py',
    'tools/v3_batched_loaded_search.py','tools/v3_conveyor_cycle_planner.py',
)


def file_hashes(paths):
    return {str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}


def certified_cache_result(result):
    torque=result.get('torque_validation',{})
    return (bool(result.get('success')) and bool(result.get('time_parameterized'))
            and result.get('motion_duration_s',float('inf'))<=60.
            and not result.get('final_snapshot',{}).get('attachments',())
            and result.get('dual_recenter_verification',{}).get('actual_whole_robot_payload_radius_m',float('inf'))<=1.05
            and bool(torque.get('success')) and bool(torque.get('geometry_frozen'))
            and torque.get('maximum_position_delta',float('inf'))==0.
            and result.get('navigation_path_validated') is False)


def write_result(output,result):
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,ensure_ascii=False,allow_nan=False)+'\n')
    for i,fixture in enumerate(result.get('recenter_inputs',())):
        output.with_name(output.stem+f'-input-{i}-{fixture["stage"]}.json').write_text(
            json.dumps(fixture,ensure_ascii=False)+'\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--algorithm', choices=('joint_rrt','cartesian_rrt'), default='joint_rrt')
    p.add_argument('--seed', type=int, default=11)
    p.add_argument('--policy', type=Path, default=ROOT/'config/placement_task_policy.json')
    p.add_argument('--right-grasp-rolls-deg',default='0,30,-30,45,-45',
                   help='Ordered, comma-separated right upper-box contact rolls in degrees')
    p.add_argument('--left-grasp-rolls-deg',default='0,30,-30,45,-45',
                   help='Ordered, comma-separated left lower-box contact rolls in degrees')
    p.add_argument('--assets-root',type=Path,help='Explicit frozen door asset root (generated/ and models/); never silently downgrade the model')
    p.add_argument('--replay-input', type=Path, help='Saved recenter input; skip contact/extraction and compare exactly this stage')
    p.add_argument('--skip-dynamics',action='store_true',help='Debug only: omit the default 10 kg/box RNEA gate')
    p.add_argument('--wait-for-gpu-seconds',type=float,default=0.,help='Bounded wait for the shared lock and an idle GPU; default fail-fast')
    p.add_argument('--plan-cache',type=Path,help='Opt-in exact-scene certified trajectory cache; disabled by default')
    p.add_argument('--no-plan-cache',action='store_true',help='Force a fresh plan; certified exact-scene cache is enabled by default')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        p.error('output already exists; choose a new path to preserve evidence')
    assets = PlannerAssets()
    if a.assets_root:
        assets = replace(assets, **{name: a.assets_root / getattr(assets,name).relative_to(ROOT)
                                    for name in ('robot_config','mobile_robot_config','urdf','named_poses','box_fit')})
    policy = json.loads(a.policy.read_text())
    critical_sources=file_hashes(ROOT/relative for relative in CACHE_SOURCES)
    asset_hashes={name:hashlib.sha256(getattr(assets,name).read_bytes()).hexdigest()
                  for name in ('robot_config','mobile_robot_config','urdf','named_poses','box_fit')}
    cache_key=digest(dict(schema=1,algorithm=a.algorithm,policy_sha256=hashlib.sha256(a.policy.read_bytes()).hexdigest(),
                          left_rolls=a.left_grasp_rolls_deg,right_rolls=a.right_grasp_rolls_deg,
                          dynamics=not a.skip_dynamics,assets=asset_hashes,sources=critical_sources))
    cache_started=time.perf_counter()
    cache_enabled=a.plan_cache is not None and not (a.no_plan_cache or a.replay_input)
    if cache_enabled and a.plan_cache.is_file():
        try:
            with gzip.open(a.plan_cache,'rt') as stream:cached=json.load(stream)
            cached_result=cached.get('result',{})
            if (cached.get('key')==cache_key and cached.get('result_identity')==digest(cached_result)
                    and certified_cache_result(cached_result)):
                result=cached_result
                origin_total=result.get('total_ms')
                result.update(plan_cache=dict(hit=True,path=str(a.plan_cache),key=cache_key,
                                               origin_seed=result.get('seed'),origin_total_ms=origin_total),
                              requested_seed=a.seed,source_identity=file_hashes(sorted((ROOT/'tools').rglob('*.py'))),
                              total_ms=(time.perf_counter()-cache_started)*1000)
                write_result(a.output,result)
                print(json.dumps({k:result.get(k) for k in ('success','total_ms','plan_cache')},ensure_ascii=False),flush=True)
                return 0
        except (OSError,ValueError,KeyError,json.JSONDecodeError):
            pass
    with open('/tmp/sevenova-curobo-gpu.lock','a') as lock:
        if not np.isfinite(a.wait_for_gpu_seconds) or a.wait_for_gpu_seconds < 0:
            p.error('GPU wait must be finite and non-negative')
        deadline = time.monotonic() + a.wait_for_gpu_seconds
        while True:
            occupied = 'shared planning lock held'
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                pass
            else:
                occupied = subprocess.check_output(['nvidia-smi','--query-compute-apps=pid','--format=csv,noheader'],text=True).strip()
                if not occupied:
                    break
                fcntl.flock(lock, fcntl.LOCK_UN)
            if time.monotonic() >= deadline:
                raise RuntimeError('GPU unavailable; no competing planning job started: '+occupied)
            time.sleep(min(1., max(0., deadline-time.monotonic())))
        planner = DoorPlanner(assets)
        request = door_request(planner,a.seed)
        planner.set_snapshot(request.snapshot)
        task = DoorTask(planner,request,a.algorithm,policy)
        try:
            task.left_grasp_rolls_deg = tuple(float(value) for value in a.left_grasp_rolls_deg.split(','))
            task.right_grasp_rolls_deg = tuple(float(value) for value in a.right_grasp_rolls_deg.split(','))
        except ValueError as error:
            p.error(f'invalid --left-grasp-rolls-deg: {error}')
        task.report['requested_left_grasp_rolls_deg'] = list(task.left_grasp_rolls_deg)
        task.report['requested_right_grasp_rolls_deg'] = list(task.right_grasp_rolls_deg)
        began = time.perf_counter()
        try:
            if a.replay_input:
                fixture = json.loads(a.replay_input.read_text())
                if fixture.get('identity') != digest({k:v for k,v in fixture.items() if k!='identity'}):
                    raise ValueError('saved-input identity mismatch')
                if fixture['side'] not in ('left','right') or fixture['snapshot']['model_id'] != planner.model_id:
                    raise ValueError('saved-input side/model mismatch')
                if not np.allclose(fixture['target'],task.targets[fixture['side']],atol=1e-12,rtol=0):
                    raise ValueError('saved-input carry target differs from current policy')
                start,goals = np.asarray(fixture['start']),np.asarray(fixture['candidates'])
                if start.shape!=(15,) or goals.ndim!=2 or goals.shape[1]!=15 or not len(goals) or not np.isfinite(start).all() or not np.isfinite(goals).all():
                    raise ValueError('saved-input must contain finite 15-axis start and nonempty goals')
                arm = list(range(1,8)) if fixture['side']=='left' else list(range(8,15))
                if fixture['active_indices'] not in (arm,[0]+arm):
                    raise ValueError('saved-input may only release the moving arm and optional lift')
                request.seed = int(fixture['seed'])
                task.report['seed'] = request.seed
                task.store = SceneStore(SceneSnapshot.from_dict(fixture['snapshot']))
                planner.snapshot = task.store.snapshot()
                task.q = np.asarray(fixture['start'],dtype=float)
                task.support_pose = None if fixture['support_pose'] is None else np.asarray(fixture['support_pose'])
                task.enter(fixture['stage'])
                mapper = validity = None
                if fixture.get('derived_indices'):
                    mapper = SupportCompensation(task)
                    validity = task.validity()
                path = task.search_recenter(fixture['side'],np.asarray(fixture['candidates']),
                                            fixture['active_indices'],mapper,validity)
                if path is None:
                    task.fail('saved-input recenter search failed')
                task.append(path)
                task.report['success'] = True
            else:
                task.run()
            task.enter('time_parameterization')
            task.finalize_timing()
            if not a.skip_dynamics and not a.replay_input:
                task.enter('rnea_dynamics')
                task.apply_dynamics()
        except Exception as error:
            task.report.update(success=False,error=dict(type=type(error).__name__,stage=getattr(error,'stage',task.phase),message=str(error),traceback=traceback.format_exc()))
        finally:
            task.close_stage_timing()
            task.report['final_snapshot'] = task.store.snapshot().to_dict()
            task.report.update(total_ms=(time.perf_counter()-began)*1000,
                               initial_snapshot=request.snapshot.to_dict(),final_snapshot=task.store.snapshot().to_dict(),
                               source_identity={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest()
                                 for f in sorted((ROOT/'tools').rglob('*.py'))},
                               model_id=planner.model_id, assets={name:dict(path=str(getattr(assets,name)),sha256=hashlib.sha256(getattr(assets,name).read_bytes()).hexdigest()) for name in ('robot_config','mobile_robot_config','urdf','named_poses','box_fit')},
                               policy_sha256=hashlib.sha256(a.policy.read_bytes()).hexdigest())
            task.report['plan_cache']=dict(hit=False,enabled=cache_enabled,path=None if a.plan_cache is None else str(a.plan_cache),key=cache_key)
            write_result(a.output,task.report)
            if task.report['success'] and cache_enabled:
                a.plan_cache.parent.mkdir(parents=True,exist_ok=True)
                temporary=a.plan_cache.with_suffix(a.plan_cache.suffix+'.tmp')
                with gzip.open(temporary,'wt',compresslevel=5) as stream:
                    json.dump(dict(key=cache_key,result_identity=digest(task.report),result=task.report),
                              stream,ensure_ascii=False,allow_nan=False)
                temporary.replace(a.plan_cache)
            print(json.dumps({k:task.report.get(k) for k in ('success','error','total_ms','recenter_candidates','recenter_searches','plan_cache')},ensure_ascii=False),flush=True)
    return 0 if task.report['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
