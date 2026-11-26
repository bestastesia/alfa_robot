"""Fresh gpuq process only: unchanged suffix geometry -> genuine native timing."""
from dataclasses import asdict, replace
from pathlib import Path
import hashlib
import importlib.metadata
import json
import sys
import time
import traceback

import numpy as np

ROOT=Path(__file__).resolve().parents[4]
HERE=Path(__file__).resolve().parent
OUT=ROOT/'generated/two_box_layout/joint_contract_20261124_suffix_native'
SUFFIX=ROOT/'generated/two_box_layout/joint_contract_20261124_suffix'
TRANSFER=ROOT/'generated/two_box_layout/joint_contract_20261124_transfer'
PATCH=ROOT/'generated/two_box_layout/joint_contract_20261124_prefix_native/numeric_fix/attempt_02'
EXPERIMENTS=HERE.parent
sys.path[:0]=[str(ROOT/'tools'),str(EXPERIMENTS/'transfer_suffix'),str(EXPERIMENTS/'transfer_prefix_native/numeric_fix')]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def frozen_inputs():
    manifest=json.loads((ROOT/'generated/two_box_layout/joint_contract_20261123/source_default_full31/manifest.json').read_text())
    bad=[p for p,digest in manifest['files'].items() if not (ROOT/p).is_file() or sha(ROOT/p)!=digest]
    assert not bad,bad
    paths=[]
    for directory in (SUFFIX,TRANSFER,EXPERIMENTS/'transfer_suffix',EXPERIMENTS/'transfer_followup'):
        paths.extend(p for p in directory.rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    return dict(production_files=len(manifest['files']),production_mismatches=bad,
        old_files={str(p.relative_to(ROOT)):sha(p) for p in paths})


def patch_gate():
    report=json.loads((PATCH/'numeric_fix_report.json').read_text())
    assert report['success'] and report['prefix_success'] and report['shared_library_unchanged']
    assert sha(PATCH/'kernels/trajectory/bspline/bspline_context.cuh')==report['patched_context_sha256']
    baseline=json.loads((PATCH/'shared_library_before.json').read_text())
    library=Path(report['curobo_file']).parent
    assert all(sha(library/key)==digest for key,digest in baseline.items()), 'shared library differs from accepted patch regression'
    sources=report['compiled_trajectory_sources']
    assert any('/bspline/' in s for s in sources) and any('/timeopt/' in s for s in sources)
    return dict(interface='install_kernel_tree()',kernel_tree=str(PATCH/'kernels'),
        context_sha256=report['patched_context_sha256'],shared_library_verified_files=len(baseline),accepted_report_sha256=sha(PATCH/'numeric_fix_report.json'))


def run(output,result):
    import curobo
    import torch
    import yourdfpy
    from isolated_kernel import install_kernel_tree
    result['kernel_patch']=install_kernel_tree(PATCH/'kernels')
    from probe_suffix import SuffixPlanner,prepared_assets,measured,validate_attachment
    from audit_suffix import inputs
    import v3_two_box_pick
    import v3_fast_three_layers
    from v3_fast_three_layers import FastThreeLayerPlanner,POLICY
    from v3_layout_native import time_pick
    from v3_ordered_layout import _observations
    from v3_box_layout import atomic_json
    from curobo_core.box_pick import layout_model_key,timed_path_checks
    from curobo_core.ordered_pick import ordered_stages
    from curobo_core.native_transport import source_progress,native_seam,concatenate_native,cpu_source_observations
    from curobo.types import JointState

    began=time.perf_counter();deadline=began+170.;calls=0
    result.update(runtime=dict(sys_executable=sys.executable,curobo_module_path=curobo.__file__,
        curobo_version=importlib.metadata.version('nvidia-curobo'),torch_version=torch.__version__),
        input_hashes_before=frozen_inputs(),stages=[],native_calls=[],native_budget_seconds=170.,max_native_calls=20,max_stage_leaves=8)
    accepted=json.loads((SUFFIX/'suffix_probe.json').read_text());assert accepted['success']
    interface,picks,start,_=inputs()
    with np.load(SUFFIX/'suffix_geometry.npz',allow_pickle=False) as z:geometry={k:z[k].copy() for k in z.files}
    picks={s:measured(replace(p,carry_joint_home=True,carry_offset_m=(0.,0.,0.),carry_updown_m=None),geometry[s+'_attachment']) for s,p in picks.items()}
    result['attachment_checks']={s:validate_attachment(p) for s,p in picks.items()}
    result['task']={s:asdict(p) for s,p in picks.items()}
    result['joint_names']=['updown']+[f'{s}_joint{i}' for s in ('left','right') for i in range(1,8)]
    result['original_request']=accepted['request']
    assert np.array_equal(geometry['upper_park'][0],start)
    v3_two_box_pick.prepare_fast_assets=prepared_assets
    v3_fast_three_layers.prepare_fast_assets=prepared_assets
    v3_fast_three_layers.OUTPUT=output/'resources'
    planner=SuffixPlanner(picks);planner.full_payload_trajopt=True
    engine=FastThreeLayerPlanner(planner=planner);planner.layout_native=(layout_model_key(picks),engine)
    assert POLICY['planning_limit_scale']==.995 and POLICY['payload_kg_per_box']==10.
    specs=ordered_stages(picks)[3:];previous=None;all_arrays=[]
    refs={'right':0.};cleared={'right':True}
    robot=yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf',load_meshes=False)
    limits=np.asarray(POLICY['arm_continuous_torque_nm']*2)*.995

    def publish():
        result['wall_seconds']=time.perf_counter()-began
        atomic_json(output/'native_result.json',result)

    def progress(event):
        if event.get('state') in ('accepted','rejected'):
            print('NATIVE_EVENT',event.get('stage'),event.get('state'),event.get('reason'),flush=True)

    def inspect(spec,stage,prior,reference,clear):
        good=bool(stage.get('success'));data=None
        audit=dict(valid=False)
        if not stage.get('trajectory_file'):return None,audit,dict(clear)
        with np.load(stage['trajectory_file'],allow_pickle=False) as z:data={k:z[k].copy() for k in z.files}
        seam=dict(valid=True) if prior is None else native_seam(prior,data)
        if prior is None:
            seam.update(position=float(np.max(np.abs(data['position'][0]-start))),
                velocity=float(np.max(np.abs(data['velocity'][0]))),acceleration=float(np.max(np.abs(data['acceleration'][0]))))
            seam['valid']=seam['position']<=1e-5 and seam['velocity']<=1e-4 and seam['acceleration']<=1e-4
        checker=planner.checker(planner.selected_task,payload=True)
        values=torch.as_tensor(data['position'],device='cuda',dtype=torch.float32)
        static_ok=planner.static_load_mask(checker,values)
        native_margin=planner.native_margin_mask(checker,values)
        # An independent RNEA resource rechecks actual native q/v/a inside time_pick; static hold retains .995 too.
        torque=[];model=engine.dynamics._model(tuple(checker.task_tool_to_box),checker.task_tool_to_box)
        for q in values.split(512):
            model.setup_batch_size(len(q));zeros=torch.zeros_like(q)
            torque.append(model.compute_inverse_dynamics(JointState(position=q.contiguous(),velocity=zeros,acceleration=zeros))[:,1:].cpu().numpy().copy())
        torque=np.concatenate(torque)
        static_file=Path(stage['trajectory_file']).with_name('static_rnea.npz');np.savez_compressed(static_file,torque=torque)
        processes={};next_clear=dict(clear)
        for name,cpu in [('gpu',False),('cpu',True)]:
            observations=_observations(planner,spec,data['position'],cpu=cpu)
            if observations:
                passed,_,observed_clear,checks=source_progress(observations,reference,clear)
                processes[name]=dict(valid=passed,checks=checks,observations=observations)
                good&=passed
                if cpu:next_clear.update(observed_clear)
        good&=bool(seam['valid'] and static_ok.all() and native_margin.all())
        audit.update(valid=bool(good),qva_seam=seam,source_history=processes,
            static_limit_scale=.995,max_static_scaled_ratio=float((np.abs(torque)/limits).max()),
            static_invalid_indices=(~static_ok).nonzero().reshape(-1).cpu().tolist(),
            native_margin_invalid_indices=(~native_margin).nonzero().reshape(-1).cpu().tolist(),
            static_rnea_file=str(static_file),samples=len(data['times']),motion_seconds=float(data['times'][-1]),
            independent_rnea=stage.get('independent_rnea'),array_checks=stage.get('array_checks'),
            trajectory_optimization=stage.get('trajectory_optimization'),cartesian_line_checks=stage.get('cartesian_line_checks'))
        return data,audit,next_clear

    for spec in specs:
        rows=geometry[spec.stage_id];stage_out=output/spec.stage_id;stage_out.mkdir()
        if previous is not None:assert np.max(np.abs(previous['position'][-1]-rows[0]))<=1e-5
        queue=[(0,len(rows)-1,'whole')];pieces=[];stage_arrays=[];rejects=[];stage_clear=dict(cleared)
        stage_record=dict(stage_id=spec.stage_id,physical_contract=asdict(spec),geometry_frames=len(rows),pieces=pieces,rejections=rejects,success=False)
        result['stages'].append(stage_record);publish()
        while queue:
            if time.perf_counter()>=deadline or calls>=20:
                stage_record['failure']='bounded native budget exhausted; not input illegality';break
            a,b,name=queue.pop(0);final=b==len(rows)-1
            piece_spec=spec if final else replace(spec,final_joint_sides=())
            planner.set_stage(piece_spec,rows[a])
            stand_off={}
            if spec.incoming_side:
                checker=planner.checker(planner.selected_task,payload=True);cap=planner.target_validity(checker,allow_contact=True)
                stand_off[spec.incoming_side]=picks[spec.incoming_side].precontact_distance(cap.targets[spec.incoming_side]['tool_cap_reach_m'])
            source=dict(geometry_success=True,task={s:asdict(p) for s,p in picks.items()},joint_names=result['joint_names'],
                frames=rows[a:b+1].tolist(),phases=[spec.phase]*(b-a+1),source_sha256={},precontact_distance_m=stand_off)
            calls+=1
            print('NATIVE_CALL',calls,spec.stage_id,name,'rows',a,b,flush=True)
            timed=time_pick(engine,source,stage_out/name,stop_on_failure=True,phase_names=(spec.phase,),
                final_transport=final if spec.phase=='transport' else True,stage_context=piece_spec,progress=progress)
            native=timed['stages'][0]
            prior=stage_arrays[-1] if stage_arrays else previous
            actual,audit,next_clear=inspect(piece_spec,native,prior,refs,stage_clear)
            trial=dict(name=name,start=a,end=b,accepted=audit['valid'],report_file=str(stage_out/name/'report.json'),
                trajectory_file=native.get('trajectory_file'),rejection_reasons=native.get('rejection_reasons'),audit=audit)
            result['native_calls'].append(dict(stage_id=spec.stage_id,name=name,accepted=audit['valid'],report_file=trial['report_file']))
            if audit['valid']:
                pieces.append(trial);stage_arrays.append(actual);stage_clear=next_clear
            else:
                rejects.append(trial)
                # ponytail:8 genuine native rest leaves/stage,20 calls total; preserve every rejected candidate.
                if spec.phase in ('approach','transport') and b-a>1 and len(queue)+len(pieces)+2<=8:
                    middle=(a+b)//2;queue[0:0]=[(a,middle,name+'a'),(middle,b,name+'b')]
                else:
                    stage_record['failure']='native gates or bounded rest-piece representation rejected; not input illegality';break
            publish()
        if stage_record.get('failure') or queue or not stage_arrays:
            result['failure']=stage_record.get('failure','no accepted native arrays');publish();break
        merged,seams=concatenate_native(stage_arrays)
        checks=timed_path_checks(merged,rows[0],rows[-1],POLICY);assert checks['valid']
        stage_file=stage_out/'native_stage.npz';np.savez_compressed(stage_file,**merged)
        stage_record.update(success=True,trajectory_file=str(stage_file),array_checks=checks,internal_qva_seams=seams,
            motion_seconds=float(merged['times'][-1]),samples=len(merged['times']))
        all_arrays.append(merged);previous=merged;cleared=stage_clear
        if spec.phase=='extract':
            observed=cpu_source_observations(robot,merged['position'][-1:],result['joint_names'],{'left':picks['left']})['left']
            refs['left']=observed['initial_slot_overlap_m'];cleared['left']=observed['first_clear_frame'] is not None
        stage_record['source_reference_after']=dict(refs);stage_record['source_cleared_after']=dict(cleared)
        publish();print('NATIVE_STAGE_PASSED',spec.stage_id,stage_record['motion_seconds'],stage_record['samples'],flush=True)
    result['native_success']=len(result['stages'])==5 and all(s['success'] for s in result['stages']) and all(cleared.get(s,False) for s in picks)
    result['success']=result['native_success'];result['both_sources_cleared']=dict(cleared)
    result['all_free_trajopt_optimizer_success']=all(
        p['audit']['trajectory_optimization']['optimizer_success'] is True
        for stage in result['stages'] if stage['physical_contract']['phase'] in ('approach','transport')
        for p in stage['pieces'])
    if all_arrays:
        merged,seams=concatenate_native(all_arrays)
        np.savez_compressed(output/'suffix_native.npz',**merged)
        result.update(motion_seconds=float(merged['times'][-1]),native_samples=len(merged['times']),qva_seams=seams,
            accepted_stage_count=len(all_arrays),merged_trajectory_file=str(output/'suffix_native.npz'))
    from curobo._src.context import get_runtime
    cache=get_runtime().get_cuda_core_cache()
    result['compiled_trajectory_sources']=sorted(set(str(path) for key in cache.compiled_kernels for path,_ in key[0] if '/trajectory/' in str(path)))
    assert result['compiled_trajectory_sources'] and all(Path(p).is_relative_to(PATCH/'kernels') for p in result['compiled_trajectory_sources'])
    assert any('/bspline/' in p for p in result['compiled_trajectory_sources']) and any('/timeopt/' in p for p in result['compiled_trajectory_sources'])
    result['patch_gate_after']=patch_gate()
    result['input_hashes_after']=frozen_inputs();assert result['input_hashes_before']==result['input_hashes_after']
    publish()


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('--attempt',default='attempt_01')
    args=parser.parse_args()
    output=OUT/args.attempt;output.mkdir(parents=True,exist_ok=False)
    result=dict(success=False,native_success=False,scope='Suffix native only; full13-stage chain/native success is separate.',
        estimate='3m provisional: suffix geometry35.95s,ordered native93s,prefix62s plus bounded native rest pieces',
        patch_gate=patch_gate(),manual_time_scaling=False,sample_arrays_clipped=False)
    started=time.perf_counter()
    try:run(output,result)
    except Exception as error:
        result.update(success=False,native_success=False,failure=repr(error),traceback=traceback.format_exc());traceback.print_exc()
    finally:
        result['total_worker_wall_seconds']=time.perf_counter()-started
        (output/'native_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print('SUFFIX_NATIVE_RESULT',result['native_success'],result['total_worker_wall_seconds'],result.get('failure'),flush=True)
    raise SystemExit(0 if result['native_success'] else 1)
