"""Native layout transport adaptation: unchanged physical checks, explicit retries."""
from pathlib import Path
import hashlib
import time
import numpy as np
import torch
from curobo_core.box_pick import source_slot_observation,source_slot_process_checks,timed_path_checks
from curobo_core.native_transport import source_progress,native_seam,concatenate_native
from v3_two_box_timing import time_pick as _time_pick,PHASES,ROOT
from v3_fast_three_layers import POLICY
from v3_fast_path import fit_path,save
from v3_lm_conveyor import box_poses


def time_pick(engine, source, output, *args, **kwargs):
    """Retry layout extraction only after its original full-box audit rejects."""
    result = _time_pick(engine, source, output, *args, **kwargs)
    failed = next((stage for stage in result['stages'] if stage['phase'] == 'extract' and not stage['success']), None)
    if (failed is None or failed.get('payload_audit', {}).get('valid', True)
            or not getattr(engine.planner, 'full_payload_trajopt', False)
            or source.get('_native_extract_full_box_guidance', False)):
        return result
    progress = kwargs.get('progress', lambda event: None)
    progress(dict(stage='extract_guidance_retry', kind='process', state='running',
        reason='original full-box extract audit rejected; retry same LM controls/iterations with full-box guidance'))
    request = dict(source, _native_extract_full_box_guidance=True)
    retry = _time_pick(engine, request, Path(output)/'extract_guidance_retry', *args, **kwargs)
    retry['extraction_guidance_initial_rejection'] = result
    retry['wall_seconds'] += result['wall_seconds']
    if 'process_validation' in request:
        source['process_validation'] = request['process_validation']
    save(Path(output)/'report.json', retry)
    progress(dict(stage='extract_guidance_retry', kind='process', state='accepted' if retry['success'] else 'rejected',
        wall_seconds=retry['wall_seconds'], reason=None if retry['success'] else 'native retry rejected; inspect original audits'))
    return retry


def time_initial_with_approach_recovery(engine,source,output,stop_on_failure=False,progress=lambda event:None,*,phase_names=PHASES):
    """Retry a rejected monolithic approach with real native32 rest-seam pieces."""
    if tuple(phase_names) not in (PHASES[:3],PHASES):
        raise ValueError('layout recovery expects the initial three phases or a full cycle')
    rejected=time_pick(engine,source,output,stop_on_failure=stop_on_failure,progress=progress,phase_names=phase_names)
    if not rejected['stages'] or rejected['stages'][0]['phase']!='approach' or rejected['stages'][0]['success']:
        return rejected
    started=time.perf_counter();directory=Path(output)/'approach_recovery';directory.mkdir(parents=True,exist_ok=False)
    for path in (Path(__file__),ROOT/'tools/curobo_core/native_transport.py'):
        source['source_sha256'][str(path.relative_to(ROOT))]=hashlib.sha256(path.read_bytes()).hexdigest()
    original=np.asarray(source['frames'],dtype=np.float32)[np.asarray(source['phases'])=='approach']
    if len(original)<3:return rejected
    rows=original[np.r_[True,np.any(np.diff(original,axis=0)!=0,axis=1)]]
    if len(rows)<3:return rejected  # No new geometric subdivision; never gamble on the same failed seed.
    middle=(len(rows)-1)//2
    pending=[(0,middle,'0a'),(middle,len(rows)-1,'0b')]
    records=[];history=[];arrays=[];failure=None
    while pending:
        a,b,name=pending.pop(0);active=rows[a:b+1]
        progress(dict(stage='approach_piece',kind='native_substage',piece=name,state='running'))
        request=dict(source,frames=active.tolist(),phases=['approach']*len(active))
        partial=time_pick(engine,request,directory/f'piece_{name}',stop_on_failure=True,phase_names=('approach',))
        stage=partial['stages'][0];seam=None;accepted=False
        if partial['success']:
            with np.load(stage['trajectory_file'],allow_pickle=False) as f:data={k:f[k].copy() for k in f.files}
            seam=native_seam(arrays[-1],data) if arrays else dict(valid=True)
            accepted=seam['valid']
        reason=None if accepted else ('actual native approach seam rejected' if partial['success'] else '; '.join(stage['rejection_reasons']))
        trial=dict(name=name,start=a,end=b,stage=stage,seam=seam,accepted=bool(accepted),failure=reason)
        progress(dict(stage='approach_piece',kind='native_substage',piece=name,state='accepted' if accepted else 'rejected',
            wall_seconds=stage['wall_seconds'],reason=reason,details=stage))
        if accepted:records.append(trial);arrays.append(data)
        else:
            history.append(trial)
            # ponytail: at most64 actual native approach pieces; exhaustion stays unknown, not input illegality.
            if b-a>1 and len(pending)+len(records)+2<=64:
                mid=(a+b)//2;pending[0:0]=[(a,mid,name+'a'),(mid,b,name+'b')]
                progress(dict(stage='approach_refine',kind='process',state='rejected',piece=name,reason='native approach representation needs finer subdivision; not input illegality'))
            else:failure='native approach refinement budget exhausted or a single edge still rejected; not proof the layout is illegal';break
    approach=dict(phase='approach',success=False,physical_success=False,pieces=records,native_refinement_history=history,
        monolithic_rejection=rejected['stages'][0],removed_exact_duplicate_geometry_rows=len(original)-len(rows))
    if failure is None:
        merged,seams=concatenate_native(arrays);checks=timed_path_checks(merged,original[0],original[-1],POLICY)
        file=directory/'approach.npz';np.savez_compressed(file,**merged)
        selected=[v['stage'] for v in records]
        physical=bool(checks['valid'] and all(v['physical_success'] for v in selected) and all(v['valid'] for v in seams))
        approach.update(success=physical,physical_success=physical,geometry_valid=True,trajectory_file=str(file.resolve()),
            representation='piecewise_native_bspline32_with_rest_seams',control_files=[v['control_file'] for v in selected],array_checks=checks,seams=seams,
            trajectory_optimization=dict(solver_success=all(v['trajectory_optimization']['solver_success'] for v in selected),
                optimizer_success=all(v['trajectory_optimization']['optimizer_success'] is True for v in selected),scope='selected native approach pieces; monolithic rejection retained'),
            time_optimization=dict(success=all(v['time_optimization']['success'] for v in selected),dynamic_valid=all(v['time_optimization']['dynamic_valid'] for v in selected),
                duration_s=float(merged['times'][-1]),status=0,scope='all real native approach pieces'),
            independent_rnea=dict(valid=all(v['independent_rnea']['valid'] for v in selected),peak_torque_ratio=max(v['independent_rnea']['peak_torque_ratio'] for v in selected),
                max_native_difference_nm=max(v['independent_rnea']['max_native_difference_nm'] for v in selected)))
        if not physical:failure='combined native approach arrays or seams rejected'
    approach.update(rejection_reasons=[] if failure is None else [failure],wall_seconds=time.perf_counter()-started)
    progress(dict(stage='approach',kind='native',state='accepted' if approach['success'] else 'rejected',details=approach,
        reason=failure,wall_seconds=approach['wall_seconds']))
    if not approach['success']:
        result=dict(rejected,success=False,complete_candidate=False,stages=[approach],all_native_geometry_results_success=False,all_trajopt_success=False)
        result.pop('motion_seconds',None)
    else:
        result=time_pick(engine,source,directory/'remaining',stop_on_failure=stop_on_failure,progress=progress,phase_names=phase_names[1:])
        if result['stages'] and 'trajectory_file' in result['stages'][0]:
            with np.load(result['stages'][0]['trajectory_file'],allow_pickle=False) as f:following={k:f[k].copy() for k in f.files}
            join=native_seam(merged,following)
        else:join=dict(valid=False,failure='remaining phase has no native trajectory')
        if result['stages']:
            first=result['stages'][0]
            first['seam_from_recovered_approach']=join
            if not join['valid']:
                first.update(success=False,physical_success=False,rejection_reasons=first['rejection_reasons']+['actual approach-to-next-phase seam rejected'])
            result['seam_checks'][0]=dict(phase=first['phase'],**join)
        result['stages']=[approach,*result['stages']]
        result['seam_checks']=[dict(phase='approach',valid=True),*result['seam_checks']]
        result['success']=bool(result['success'] and join['valid'])
        result['is_full_cycle']=tuple(phase_names)==PHASES
        result['complete_candidate']=result['is_full_cycle'] and len(result['stages'])==len(phase_names) and all('trajectory_file' in v for v in result['stages'])
        result['all_native_geometry_results_success'] &= approach['trajectory_optimization']['solver_success']
        result['all_trajopt_success'] &= approach['trajectory_optimization']['optimizer_success']
        if 'motion_seconds' in result:result['motion_seconds']+=approach['time_optimization']['duration_s']
    result['wall_seconds']=rejected['wall_seconds']+time.perf_counter()-started
    result['verification_scope']='real native32 approach pieces and actual cross-phase seam; target boxes retained; no sampled-array repair; not CCD'
    save(directory/'report.json',result)
    return result


def time_layout_pick(engine,source,output,stop_on_failure=False,progress=lambda event:None):
    """Preserve accepted native paths; recover rejected transport with real native pieces."""
    search=source['attempts'][-1].get('transport_search',{})
    started=time.perf_counter()
    if search.get('selected_start_root')!=1 or not search.get('fresh_forward_lateral_prefix'):
        monolithic=time_initial_with_approach_recovery(engine,source,output,stop_on_failure=stop_on_failure,progress=progress)
        stages=monolithic['stages']
        if monolithic['success'] and stages[-1].get('source_slot_observation'):
            process=source_slot_process_checks(stages[-1]['source_slot_observation'])
            if not process['valid']:
                rejected=dict(stages[-1],success=False,process_validation=process,
                    rejection_reasons=stages[-1]['rejection_reasons']+process['rejection_reasons'])
                stages=[*stages[:-1],rejected]
                monolithic=dict(monolithic,success=False,stages=stages)
                progress(dict(stage='transport_process',kind='process',state='rejected',
                    reason='; '.join(process['rejection_reasons'])+'; retry original native pieces',details=process))
        if (monolithic['success'] or len(stages)!=4 or stages[-1]['phase']!='transport'
                or not all(stage['success'] for stage in stages[:-1])):
            return monolithic
        initial=dict(monolithic,stages=stages[:-1],success=True,is_full_cycle=False,complete_candidate=False,
            seam_checks=monolithic['seam_checks'][:3],
            motion_seconds=sum(stage['time_optimization']['duration_s'] for stage in stages[:-1]),
            all_native_geometry_results_success=all(stage.get('trajectory_optimization',{}).get('solver_success',True) for stage in stages[:-1]),
            all_trajopt_success=all(stage.get('trajectory_optimization',{}).get('optimizer_success',True) is True for stage in stages[:-1]),
            transport_monolithic_rejection=stages[-1])
        output=Path(output)/'transport_recovery';output.mkdir(parents=True,exist_ok=False)
    else:
        output=Path(output);output.mkdir(parents=True,exist_ok=False)
        initial=time_initial_with_approach_recovery(engine,source,output/'initial',stop_on_failure=True,progress=progress,phase_names=PHASES[:3])
        if not initial['success']:return initial
    source['source_sha256'][str(Path(__file__).relative_to(ROOT))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    helper=ROOT/'tools/curobo_core/native_transport.py'
    source['source_sha256'][str(helper.relative_to(ROOT))]=hashlib.sha256(helper.read_bytes()).hexdigest()
    transport_started=time.perf_counter()
    planner=engine.planner;checker=planner.checker(planner.selected_task,payload=True)
    rows=np.asarray(source['frames'],dtype=np.float32)[np.asarray(source['phases'])=='transport']
    low,high=[v.cpu().numpy() for v in checker.kinematics.get_joint_limits().position]
    validity=planner.path_validity(checker,start=rows[0],goal=rows[-1])
    sections=search.get('fresh_forward_lateral_prefix',{}).get('native_section_ends',[]) if search.get('selected_start_root')==1 else []
    boundaries=[0,*sections,len(rows)-1]
    boundaries=sorted(set(int(value) for value in boundaries if 0<=value<len(rows)))
    pending=[(a,b,str(i)) for i,(a,b) in enumerate(zip(boundaries[:-1],boundaries[1:]))]
    records=[];rejected=[];arrays=[];poses={s:[] for s in planner.picks}
    reference=None;cleared={}
    with np.load(initial['stages'][-1]['trajectory_file'],allow_pickle=False) as f:
        previous={k:f[k].copy() for k in ('position','velocity','acceleration')}
    progress(dict(stage='transport',kind='native',state='running'))
    failure=None
    while pending:
        a,b,name=pending.pop(0);active=rows[a:b+1];final=not pending
        controls,_,_,fit,spline=fit_path(dict(positions=active),32,control_bounds=(low,high))
        controls=controls.astype(np.float32)
        seed_valid=bool(((controls>=low)&(controls<=high)).all())
        if seed_valid:
            for count in (129,4097):
                values=torch.as_tensor(np.asarray(spline(np.linspace(0,1,count)),dtype=np.float32),device='cuda')
                if not bool(validity.mask(values).all()):seed_valid=False;break
        attempts=[];accepted=None
        if seed_valid:
            for seed_only in (False,True):
                mode='validated_seed' if seed_only else 'optimized'
                progress(dict(stage='transport_piece',kind='native_substage',piece=name,state='running',candidate=mode))
                request=dict(source,frames=active.tolist(),phases=['transport']*len(active))
                partial=time_pick(engine,request,output/f'piece_{name}_{mode}',stop_on_failure=True,
                    phase_names=('transport',),final_transport=final,seed_only=seed_only)
                stage=partial['stages'][0];trial=dict(mode=mode,stage=stage,accepted=False)
                attempts.append(trial)
                if not partial['success']:trial['failure']='; '.join(stage['rejection_reasons'])
                else:
                    with np.load(stage['trajectory_file'],allow_pickle=False) as f:data={k:f[k].copy() for k in f.files}
                    seam=native_seam(previous,data)
                    actual=box_poses(checker,data['position'],checker.task_tool_to_box)
                    observation={s:source_slot_observation(planner.picks[s],T) for s,T in actual.items()}
                    process,trial_reference,trial_clear,scope=source_progress(observation,reference,cleared)
                    trial.update(source_observation=observation,global_source_checks=scope,seam=seam)
                    if not seam['valid']:trial['failure']='actual native seam rejected'
                    elif not process:trial['failure']='whole-transport original-slot process rejected'
                    else:
                        trial['accepted']=True;accepted=(data,actual,trial_reference,trial_clear)
                progress(dict(stage='transport_piece',kind='native_substage',piece=name,candidate=mode,
                    state='accepted' if trial['accepted'] else 'rejected',wall_seconds=stage['wall_seconds'],
                    reason=trial.get('failure'),details=stage))
                if accepted is not None:break
                if stage.get('trajectory_optimization',{}).get('seed_retention') is not None:break
        if accepted is None:
            rejected.append(dict(start=a,end=b,piece=name,seed_valid=seed_valid,seed_fit=fit,attempts=attempts))
            if b-a>1 and len(pending)+len(records)+2<=64:
                middle=(a+b)//2;pending[0:0]=[(a,middle,name+'a'),(middle,b,name+'b')]
                progress(dict(stage='transport_refine',kind='process',state='rejected',piece=name,
                    reason='native seed/full-candidate checks require finer representation; not input illegality'))
                continue
            failure='native transport refinement budget exhausted or one geometric edge remains invalid; not proof the layout is illegal'
            break
        data,actual,reference,cleared=accepted
        records.append(dict(name=name,start=a,end=b,final_hold_required=final,stage=stage,attempts=attempts))
        previous=data;arrays.append(data)
        for side,T in actual.items():poses[side].append(T if len(records)==1 else T[1:])
        save(output/'transport_progress.json',dict(pieces=records,rejected=rejected))
    if failure:
        stage=dict(phase='transport',success=False,physical_success=False,error=failure,
            rejection_reasons=[failure],pieces=records,native_refinement_history=rejected,wall_seconds=time.perf_counter()-started)
        result=dict(initial,success=False,is_full_cycle=True,complete_candidate=False,stages=initial['stages']+[stage],wall_seconds=time.perf_counter()-started)
        progress(dict(stage='transport',kind='native',state='rejected',reason=failure,details=stage,wall_seconds=stage['wall_seconds']))
        result.pop('motion_seconds',None)
        save(output/'report.json',result);return result
    merged,seams=concatenate_native(arrays)
    observations={s:source_slot_observation(planner.picks[s],np.concatenate(T)) for s,T in poses.items()}
    process=source_slot_process_checks(observations);source['process_validation']=process
    filename=output/'transport.npz';np.savez_compressed(filename,**merged)
    checks=timed_path_checks(merged,rows[0],rows[-1],POLICY)
    last=records[-1]['stage'];all_stages=[r['stage'] for r in records]
    timeopt_success=all(s['time_optimization']['success'] for s in all_stages)
    dynamic_valid=all(s['time_optimization']['dynamic_valid'] for s in all_stages)
    native_geometry_valid=all(s['trajectory_optimization']['solver_success'] for s in all_stages)
    rnea_valid=all(s['independent_rnea']['valid'] for s in all_stages)
    physical=bool(checks['valid'] and all(s['physical_success'] for s in all_stages) and all(s['valid'] for s in seams))
    stage=dict(phase='transport',success=physical and process['valid'],physical_success=physical,geometry_valid=True,
        trajectory_file=str(filename.resolve()),control_files=[s['control_file'] for s in all_stages],
        representation='piecewise_native_bspline32_with_rest_seams',pieces=records,native_refinement_history=rejected,
        array_checks=checks,source_slot_observation=observations,seams=seams,carry_envelope_final=last['carry_envelope_final'],
        time_optimization=dict(success=timeopt_success,dynamic_valid=dynamic_valid,duration_s=float(merged['times'][-1]),
            status=0 if timeopt_success else next(s['time_optimization']['status'] for s in all_stages if not s['time_optimization']['success']),scope='all real native pieces'),
        trajectory_optimization=dict(solver_success=native_geometry_valid,optimizer_success=all(s['trajectory_optimization']['optimizer_success'] is True for s in all_stages),scope='all selected native results, not one monolithic curve'),
        independent_rnea=dict(valid=rnea_valid,peak_torque_ratio=max(s['independent_rnea']['peak_torque_ratio'] for s in all_stages),
            max_native_difference_nm=max(s['independent_rnea']['max_native_difference_nm'] for s in all_stages),scope='pointwise maxima on identical concatenated arrays'),
        native_candidate_wall_seconds=sum(t['stage']['wall_seconds'] for r in records+rejected for t in r['attempts']),
        wall_seconds=time.perf_counter()-transport_started)
    stage['rejection_reasons']=[] if stage['success'] else process['rejection_reasons'] or ['combined native arrays rejected']
    result=dict(initial,stages=initial['stages']+[stage],is_full_cycle=True,complete_candidate=True,success=stage['success'],
        all_native_geometry_results_success=initial['all_native_geometry_results_success'] and native_geometry_valid,
        all_trajopt_success=initial['all_trajopt_success'] and stage['trajectory_optimization']['optimizer_success'],
        wall_seconds=time.perf_counter()-started,motion_seconds=initial['motion_seconds']+float(merged['times'][-1]),
        verification_scope='fresh geometry, actual native rest-seam pieces;4001 samples per piece; not CCD or hardware certification')
    progress(dict(stage='transport',kind='native',state='accepted' if result['success'] else 'rejected',
        details=stage,reason='; '.join(stage['rejection_reasons']),wall_seconds=stage['wall_seconds']))
    save(output/'report.json',result);return result
