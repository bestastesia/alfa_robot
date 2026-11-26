"""CPU-only contracts for online native representation adaptation."""
import ast
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import numpy as np
import pytest
import torch
TOOLS=Path(__file__).resolve().parents[1]/'tools'
sys.path.insert(0,str(TOOLS))
from curobo_core.native_transport import source_progress,native_seam,concatenate_native


def obs(initial,maximum,clear=None,returned=False):
    return dict(initial_slot_overlap_m=initial,maximum_slot_overlap_m=maximum,
        observation_epsilon_m=1e-6,first_clear_frame=clear,returns_after_clear=returned)


def test_whole_transport_reference_not_reset_at_pieces():
    valid,reference,clear,_=source_progress({'left':obs(.003,.0030004)})
    assert valid and not clear['left']
    assert source_progress({'left':obs(.002,.0020012)},reference,clear)[0]
    assert not source_progress({'left':obs(.0030005,.0030012)},reference,clear)[0]
    assert not source_progress({'left':obs(.001,.001)},reference,{'left':True})[0]
    assert not source_progress({'left':obs(.003,.003,1,True)},reference,clear)[0]


def arrays(start,end):
    q=np.linspace(start,end,3,dtype=np.float32)[:,None]*np.ones((1,15),np.float32)
    return dict(times=np.array([0.,.1,.2],np.float32),position=q,
        velocity=np.zeros_like(q),acceleration=np.zeros_like(q),jerk=np.zeros_like(q),torque=np.zeros((3,14),np.float32))


def test_merge_preserves_actual_arrays_and_checks_real_rest_seams():
    a,b=arrays(0.,1.),arrays(1.,2.)
    merged,seams=concatenate_native([a,b])
    np.testing.assert_array_equal(merged['position'],np.concatenate((a['position'],b['position'][1:])))
    assert merged['times'].dtype==np.float64 and np.all(np.diff(merged['times'])>0)
    assert len(merged['times'])==5 and all(v['valid'] for v in seams)
    b['velocity'][0,0]=.001
    assert not native_seam(a,b)['valid']
    with pytest.raises(ValueError):concatenate_native([a,b])
    with pytest.raises(ValueError):concatenate_native([])


def test_partial_native_validation_cannot_disable_full_cycle_hold_gate():
    file=TOOLS/'v3_two_box_timing.py';tree=ast.parse(file.read_text())
    node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='time_pick')
    env=dict(PHASES=('approach','contact','extract','transport'))
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(file),'exec'),env)
    for kwargs in (dict(phase_names=()),dict(phase_names=('unknown',)),dict(final_transport=False),dict(seed_only=True)):
        with pytest.raises(ValueError):env['time_pick'](None,None,None,**kwargs)


def test_native_seed_selection_uses_real_success_and_clones_reused_results():
    file=TOOLS/'v3_fast_three_layers.py';tree=ast.parse(file.read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='FastThreeLayerPlanner')
    function=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='trajopt_path')
    first=next(i for i,n in enumerate(function.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='optimizer_success' for t in n.targets))
    last=next(i for i,n in enumerate(function.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='native' for t in n.targets))
    selection=ast.Module(body=function.body[first:last],type_ignores=[])
    class Result:
        def __init__(self,value):self.success=torch.tensor([value]);self.debug_info={}
        def clone(self):return Result(bool(self.success[0]))
    for guide,optimized,seed in ((False,False,True),(True,True,False),(True,False,False),(True,False,True)):
        original=Result(optimized);candidate=Result(seed);calls=[]
        def evaluate(**kwargs):
            calls.append(kwargs);original.success.fill_(True)
            return candidate
        env=dict(result=original,guide_payload=guide,allow_seed_retention=guide,seed_only=False,solver=NS(evaluate_cspace_seed=evaluate),stamp=lambda:0.,
            seed_snapshot=torch.zeros((1,32,15)),initial_snapshot=object(),target_snapshot=object())
        exec(compile(selection,str(file),'exec'),env)
        assert bool(env['result'].success[0])==(optimized or (guide and seed))
        assert len(calls)==int(guide and not optimized)
        if calls:assert calls[0]['seed_traj'].shape==(1,1,32,15)
    original=Result(False);candidate=Result(True);calls=[]
    def evaluate_approach(**kwargs):calls.append(kwargs);return candidate
    env=dict(result=original,guide_payload=False,allow_seed_retention=True,seed_only=False,
        solver=NS(evaluate_cspace_seed=evaluate_approach),stamp=lambda:0.,seed_snapshot=torch.zeros((1,32,15)),
        initial_snapshot=object(),target_snapshot=object())
    exec(compile(selection,str(file),'exec'),env)
    assert len(calls)==1 and env['result'] is candidate and env['optimizer_success'] is False
    for success in (False,True):
        env=dict(result=Result(success),guide_payload=True,allow_seed_retention=True,seed_only=True)
        exec(compile(selection,str(file),'exec'),env)
        assert bool(env['result'].success[0])==success and env['optimizer_success'] is None


def test_ui_exposes_native_seed_retention_without_calling_it_optimizer_success():
    from v3_box_layout import status_markdown
    detail=dict(seed_retention=dict(success=True,optimizer_validation={'feasible':False}),optimizer_success=False)
    text=status_markdown(dict(events=[dict(stage='transport_piece',kind='native_substage',piece='x',state='accepted',
        details=dict(trajectory_optimization=detail),wall_seconds=.5)]))
    assert '并非所有优化候选都成功' in text and '同一原生API' in text and 'feasible' in text
    status_markdown(dict(events=[dict(stage='transport_piece',state='accepted',details=dict(trajectory_optimization=dict(seed_retention=None)))]))


def test_dense_native_torque_keeps_capacity_and_clones_each_borrowed_chunk():
    file=TOOLS/'v3_fast_three_layers.py'
    node=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='native_sample_torque')
    env=dict(torch=torch,JointState=NS)
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(file),'exec'),env)
    capacity=3;buffer=torch.zeros((1,capacity,2));calls=[]
    def torque(state):
        count=state.position.shape[1];calls.append(count)
        assert count<=capacity
        buffer[:,:count]=state.position+2*state.velocity+3*state.acceleration
        return buffer[:,:count]
    solver=NS(_dynamics=NS(_tau_buffer=torch.zeros((capacity,2))),_compute_torque=torque)
    q=torch.arange(16,dtype=torch.float32).reshape(1,8,2)
    dense=NS(position=q,velocity=q+1,acceleration=q+2)
    actual=env['native_sample_torque'](solver,dense)
    torch.testing.assert_close(actual,q+2*(q+1)+3*(q+2),rtol=0,atol=0)
    assert calls==[3,3,2]
    buffer.fill_(999.)
    torch.testing.assert_close(actual,q+2*(q+1)+3*(q+2),rtol=0,atol=0)
    with pytest.raises(ValueError):env['native_sample_torque'](solver,NS(position=torch.zeros((2,8,2))))


def test_recovered_approach_uses_real_arrays_and_checks_cross_phase_seam(tmp_path):
    import time
    file=TOOLS/'v3_layout_native.py';node=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='time_initial_with_approach_recovery')
    phases=('approach','contact','extract','transport')
    for name,bad_join,bad_piece,requested in (('full',False,False,phases),('partial',False,False,phases[:3]),
                                            ('join',True,False,phases),('piece',False,True,phases)):
        output=tmp_path/name;calls=[]
        rows=np.zeros((10,2),dtype=np.float32);rows[:,0]=[0.,.1,.2,.3,.3,.4,.4,.5,.5,.6]
        labels=['approach']*4+['contact']*2+['extract']*2+['transport']*2
        source=dict(frames=rows.tolist(),phases=labels,source_sha256={})
        def native(engine,request,destination,**kw):
            destination=Path(destination);destination.mkdir(parents=True,exist_ok=False);names=tuple(kw['phase_names']);calls.append(names)
            rejected=len(calls)==1;piece=names==('approach',)
            stages=[]
            for phase in (('approach',) if rejected else names):
                active=np.asarray(request['frames'])[np.asarray(request['phases'])==phase]
                start=active[0].copy();end=active[-1].copy()
                if bad_join and phase=='contact':start[0]+=.01
                arrays=dict(times=np.linspace(0,1,3),position=np.linspace(start,end,3),
                    velocity=np.zeros((3,2)),acceleration=np.zeros((3,2)),jerk=np.zeros((3,2)),torque=np.zeros((3,2)))
                filename=destination/(phase+'.npz');np.savez_compressed(filename,**arrays)
                ok=not rejected and not (piece and bad_piece)
                stages.append(dict(phase=phase,success=ok,physical_success=ok,trajectory_file=str(filename),control_file='native-controls',
                    trajectory_optimization=dict(solver_success=ok,optimizer_success=ok),
                    time_optimization=dict(success=ok,dynamic_valid=ok,duration_s=1.,status=0 if ok else 1),
                    independent_rnea=dict(valid=ok,peak_torque_ratio=.1,max_native_difference_nm=0.),
                    rejection_reasons=[] if ok else ['native scene collision'],wall_seconds=.1))
            (destination/'report.json').write_text('original-monolithic-rejection' if rejected else 'real-partial-result')
            return dict(success=all(v['success'] for v in stages),stages=stages,seam_checks=[dict(phase=v['phase'],valid=True) for v in stages],
                all_native_geometry_results_success=all(v['success'] for v in stages),all_trajopt_success=all(v['success'] for v in stages),
                motion_seconds=float(len(stages)),complete_candidate=False,is_full_cycle=names==phases,wall_seconds=.1)
        env=dict(Path=Path,np=np,time=time,hashlib=__import__('hashlib'),ROOT=TOOLS.parent,__file__=str(file),PHASES=phases,
            time_pick=native,native_seam=native_seam,concatenate_native=concatenate_native,
            timed_path_checks=lambda *args:dict(valid=True),POLICY={},save=lambda path,value:Path(path).write_text(__import__('json').dumps(value)))
        exec(compile(ast.Module(body=[node],type_ignores=[]),str(file),'exec'),env)
        result=env['time_initial_with_approach_recovery'](None,source,output,stop_on_failure=True,phase_names=requested)
        assert (output/'report.json').read_text()=='original-monolithic-rejection'
        assert result['success']==(not bad_join and not bad_piece)
        if bad_piece:
            assert len(calls)==2 and len(result['stages'])==1 and not result['complete_candidate']
            assert 'motion_seconds' not in result
            continue
        approach=result['stages'][0]
        assert len(approach['pieces'])==2 and approach['monolithic_rejection']['success'] is False
        assert result['is_full_cycle']==(requested==phases)
        assert result['complete_candidate']==(requested==phases)
        with np.load(approach['trajectory_file']) as f:merged={k:f[k] for k in f.files}
        np.testing.assert_allclose(merged['position'][:,0],[0.,.05,.1,.2,.3],atol=1e-8)
        np.testing.assert_allclose(merged['times'],[0.,.5,1.,1.5,2.])
        first=result['stages'][1]
        assert first['trajectory_optimization']['solver_success'] is True  # Never flip the native library's result.
        if bad_join:
            assert not first['success'] and not first['seam_from_recovered_approach']['valid']
            with np.load(first['trajectory_file']) as f:assert abs(f['position'][0,0]-.31)<1e-7  # No endpoint repair.
        else:assert first['seam_from_recovered_approach']['valid']


def test_approach_piece_timings_and_rejections_are_visible():
    from v3_box_layout import status_markdown
    text=status_markdown(dict(events=[dict(stage='approach_piece',kind='native_substage',piece='a',state='rejected',
        wall_seconds=.12,reason='native scene collision'),dict(stage='approach',kind='native',state='accepted',wall_seconds=.4,
        details=dict(representation='piecewise_native_bspline32_with_rest_seams'))]))
    for label in ('接近原生子段','native scene collision','0.120','含全部原生子段及拒绝'):assert label in text


def test_successful_or_later_failed_initial_path_is_not_retimed():
    file=TOOLS/'v3_layout_native.py'
    node=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='time_initial_with_approach_recovery')
    phases=('approach','contact','extract','transport')
    for result in (dict(success=True,stages=[dict(phase='approach',success=True)]),
                   dict(success=False,stages=[dict(phase='approach',success=True),dict(phase='contact',success=False)])):
        env=dict(PHASES=phases,time_pick=lambda *args,**kwargs:result)
        exec(compile(ast.Module(body=[node],type_ignores=[]),str(file),'exec'),env)
        assert env['time_initial_with_approach_recovery'](None,None,None) is result
        with pytest.raises(ValueError):env['time_initial_with_approach_recovery'](None,None,None,phase_names=('approach',))


def test_full_box_extract_retry_is_lazy_keeps_rejection_and_total_time(tmp_path):
    file=TOOLS/'v3_layout_native.py'
    node=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='time_pick')
    for layout,phase,audit,guided,retry_ok in ((True,'extract',False,False,True),
            (True,'extract',False,False,False),(False,'extract',False,False,True),
            (True,'extract',True,False,True),(True,'contact',False,False,True),
            (True,'extract',False,True,True),(True,'extract',None,False,True)):
        calls=[];saved=[];events=[]
        first=dict(success=False,stages=[dict(phase=phase,success=False)],wall_seconds=2.)
        if audit is not None:first['stages'][0]['payload_audit']=dict(valid=audit)
        second=dict(success=retry_ok,stages=[dict(phase='extract',success=retry_ok)],wall_seconds=3.)
        def native(engine,source,output,*args,**kwargs):
            calls.append((source,Path(output),kwargs))
            if len(calls)==2:source['process_validation']=dict(valid=retry_ok)
            return first if len(calls)==1 else second
        env=dict(_time_pick=native,Path=Path,save=lambda path,value:saved.append((path,value)))
        exec(compile(ast.Module(body=[node],type_ignores=[]),str(file),'exec'),env)
        source=dict(frames=[[0.]*15],_native_extract_full_box_guidance=guided)
        engine=NS(planner=NS(full_payload_trajopt=layout))
        result=env['time_pick'](engine,source,tmp_path,phase_names=('extract',),progress=events.append)
        expected=layout and phase=='extract' and audit is False and not guided
        assert len(calls)==1+expected
        assert source['_native_extract_full_box_guidance']==guided
        if expected:
            assert result is second and result['success']==retry_ok and result['wall_seconds']==5.
            assert result['extraction_guidance_initial_rejection'] is first
            assert calls[1][0]['_native_extract_full_box_guidance'] is True
            assert calls[1][0]['frames'] is source['frames']
            assert calls[1][1]==tmp_path/'extract_guidance_retry'
            assert calls[1][2]['phase_names']==('extract',)
            assert saved==[(tmp_path/'report.json',result)]
            assert source['process_validation']['valid']==retry_ok
            assert [event['state'] for event in events]==['running','accepted' if retry_ok else 'rejected']
        else:
            assert result is first and not saved and not events


def test_stateless_urdf_batch_and_boundary_reference_reject_real_gpu_false_acceptance():
    import yourdfpy
    from curobo_core.box_pick import BoxPick,source_slot_process_checks
    from curobo_core.native_transport import urdf_box_poses,reference_box_poses,cpu_source_observations
    root=TOOLS.parent;fixture=root/'tests/data/source_urdf_boundary'
    metadata=json.loads(fixture.with_suffix('.json').read_text())
    with np.load(fixture.with_suffix('.npz'),allow_pickle=False) as f:rows=f['position'].copy()
    robot=yourdfpy.URDF.load(root/'generated/fast_runtime_assets/fixed.urdf',load_meshes=False)
    cfg=robot.cfg.copy();picks={s:BoxPick(**v) for s,v in metadata['task'].items()};attachments={s:p.tool_to_box() for s,p in picks.items()}
    batch=urdf_box_poses(robot,rows,metadata['joint_names'],attachments)
    for index,row in enumerate(rows):
        reference=reference_box_poses(robot,row,metadata['joint_names'],attachments)
        for side in picks:np.testing.assert_allclose(batch[side][index],reference[side],rtol=0,atol=2e-14)
    assert source_slot_process_checks(metadata['gpu_observation'])['valid']
    observation=cpu_source_observations(robot,rows,metadata['joint_names'],picks)
    process=source_slot_process_checks(observation)
    assert not process['valid']
    assert observation['left']['first_clear_frame']==0 and observation['left']['first_return_frame']==1
    assert observation['right']['first_clear_frame']==8 and not observation['right']['returns_after_clear']
    assert observation['left']['observation_epsilon_m']==1e-6
    np.testing.assert_array_equal(robot.cfg,cfg)
    for invalid in (rows[:,1:],np.full_like(rows,np.nan)):
        with pytest.raises(ValueError):urdf_box_poses(robot,invalid,metadata['joint_names'],attachments)


def test_layout_keeps_gpu_process_rejection_and_requires_cpu_process_before_acceptance():
    import yourdfpy
    from curobo_core.box_pick import BoxPick,source_slot_process_checks
    from curobo_core.native_transport import cpu_source_observations
    root=TOOLS.parent;fixture=root/'tests/data/source_urdf_boundary';metadata=json.loads(fixture.with_suffix('.json').read_text())
    robot=yourdfpy.URDF.load(root/'generated/fast_runtime_assets/fixed.urdf',load_meshes=False)
    picks={s:BoxPick(**v) for s,v in metadata['task'].items()}
    file=TOOLS/'v3_box_layout_planner.py';fn=next(n for n in ast.walk(ast.parse(file.read_text())) if isinstance(n,ast.FunctionDef) and n.name=='_plan_layout')
    trial=next(n for n in fn.body if isinstance(n,ast.For) and ast.unparse(n.target)=='(index, (contact, reverse))')
    begin=next(i for i,n in enumerate(trial.body) if isinstance(n,ast.Assign) and any(isinstance(v,ast.Name) and v.id=='process_started' for v in n.targets))
    end=next(i for i,n in enumerate(trial.body) if isinstance(n,ast.Assign) and any(isinstance(v,ast.Subscript) and isinstance(v.value,ast.Name) and v.value.id=='attempt' and ast.literal_eval(v.slice)=='process_validation' for v in n.targets))
    code=compile(ast.Module(body=trial.body[begin:end],type_ignores=[]),str(file),'exec')
    calls=[]
    def cpu(rows):calls.append(len(rows));return cpu_source_observations(robot,rows,metadata['joint_names'],picks)
    for gpu_valid in (True,False):
        observations=json.loads(json.dumps(metadata['gpu_observation']))
        if not gpu_valid:observations['left']['returns_after_clear']=True
        report=dict(process_validation=dict(source_slot_observation=observations));calls.clear()
        env=dict(np=np,time=NS(perf_counter=lambda:0.),source_slot_process_checks=source_slot_process_checks,
            self=NS(source_observations_cpu=cpu),report=report,native=dict(stages=[dict(trajectory_file=str(fixture.with_suffix('.npz')))]))
        exec(code,env)
        assert report['process_validation']['valid'] is False
        assert len(calls)==int(gpu_valid)
        if gpu_valid:
            assert report['process_validation']['gpu_pose_process']['valid']
            assert report['process_validation']['source_slot_observation']['left']['first_return_frame']==1


def test_rrt_rejected_short_connections_cannot_starve_later_candidates():
    file=TOOLS/'v3_multiroot_search.py';fn=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='batched_rrt_connect_multi_goal')
    admission=next(n for n in ast.walk(fn) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='best_connections' for t in n.targets)
        and isinstance(n.value,ast.Subscript) and 'sorted' in ast.unparse(n.value))
    rejected=[dict(start_node=i,goal_node=100+i,cost=float(i)) for i in range(64)]
    later=dict(start_node=1000,goal_node=2000,cost=100.)
    for invalid in (set(),{(v['start_node'],v['goal_node']) for v in rejected}):
        env=dict(best_connections=[*rejected,later],fine_rejected=invalid)
        exec(compile(ast.Module(body=[admission],type_ignores=[]),str(file),'exec'),env)
        assert env['best_connections']==([later] if invalid else rejected)
    immediate=next(n for n in ast.walk(fn) if isinstance(n,ast.If) and ast.unparse(n.test)=='fine_rejected')
    valid=dict(start_node=5,goal_node=10,cost=1.)
    env=dict(best_connections=[rejected[0],valid],fine_rejected={(0,100)})
    exec(compile(ast.Module(body=[immediate],type_ignores=[]),str(file),'exec'),env)
    assert env['best_connections']==[valid]
    # Rejection reason counters must not make source-quality run after geometry already failed.
    assignments=[n for n in ast.walk(fn) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id in ('geometry_ok','quality_ok') for t in n.targets)]
    for physical in (False,True):
        seen=[];env=dict(recheck=True,dense_tensor=object(),validity=NS(mask=lambda _:torch.tensor([physical])),quality=lambda _:seen.append(1) or False)
        exec(compile(ast.Module(body=assignments,type_ignores=[]),str(file),'exec'),env)
        assert env['quality_ok'] is False and len(seen)==int(physical)


def test_directed_source_edges_reject_clear_then_return_in_true_motion_order():
    file=TOOLS/'v3_box_layout_planner.py';tree=ast.parse(file.read_text())
    fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='edge_process_quality')
    env=dict(torch=torch,checker=object(),self=NS(source_slot_depths=lambda checker,rows:rows[:,:2]))
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(file),'exec'),env)
    points=torch.zeros((4,3,15));points[:,:,0]=torch.tensor([[.006,.002,0.],[0.,.002,.003],[.006,0.,.002],[-.1,-.2,-.3]])
    torch.testing.assert_close(env['edge_process_quality'](points),torch.tensor([True,False,False,True]))
    assert bool(env['edge_process_quality'](points[1:2].flip(1))[0])
    file=TOOLS/'v3_multiroot_search.py';fn=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.FunctionDef))
    select=[n for n in ast.walk(fn) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Tuple) and any(isinstance(v,ast.Name) and v.id in ('edge_starts','connect_starts') for v in t.elts) for t in n.targets)]
    assert len(select)==2
    for directed in (False,True):
        for expand_start in (False,True):
            env=dict(torch=torch,directed=directed,expand_start=expand_start,nearest_values=torch.tensor([[1.]]),steered=torch.tensor([[2.]]),accepted=torch.tensor([[2.]]),other_values=torch.tensor([[1.]]),candidate_indices=torch.tensor([0]))
            exec(compile(ast.Module(body=select,type_ignores=[]),str(file),'exec'),env)
            reverse=directed and not expand_start
            assert env['edge_starts'].item()==(2. if reverse else 1.)
            assert env['edge_goals'].item()==(1. if reverse else 2.)
            assert env['connect_starts'].item()==(1. if reverse else 2.)
            assert env['connect_goals'].item()==(2. if reverse else 1.)


def test_edge_process_hook_uses_actual_points_and_never_restores_physical_rejection():
    file=TOOLS/'v3_batched_loaded_search.py';cls=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='GpuValidity');fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='edges')
    env=dict(torch=torch);exec(compile(ast.Module(body=[fn],type_ignores=[]),str(file),'exec'),env)
    starts=torch.zeros((2,15));goals=starts.clone();goals[:,0]=.02;seen=[]
    def evaluate(rows):return torch.tensor([True,True,True,False]),torch.zeros(len(rows))
    def process(points):seen.append(points.clone());return torch.tensor([False,True])
    obj=NS(weights=torch.ones(15),evaluate=evaluate,edge_process_quality=process)
    result=env['edges'](obj,starts,goals)
    torch.testing.assert_close(result,torch.tensor([False,False]))
    assert len(seen)==1 and seen[0].shape==(2,2,15)
    torch.testing.assert_close(seen[0][:,0],starts);torch.testing.assert_close(seen[0][:,-1],goals)
    del obj.edge_process_quality
    torch.testing.assert_close(env['edges'](obj,starts,goals),torch.tensor([True,False]))


def test_root0_native_recovery_preserves_success_and_only_reuses_valid_first_three(tmp_path):
    import copy
    file=TOOLS/'v3_layout_native.py';fn=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='time_layout_pick')
    boundary=next(i for i,n in enumerate(fn.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='transport_started' for t in n.targets))
    fn.body=fn.body[:boundary]+[ast.Return(value=ast.Tuple(elts=[ast.Name(id='initial',ctx=ast.Load()),ast.Name(id='output',ctx=ast.Load())],ctx=ast.Load()))]
    fn=ast.fix_missing_locations(fn)
    def stage(phase,valid):return dict(phase=phase,success=valid,time_optimization=dict(duration_s=1.),trajectory_optimization=dict(solver_success=valid,optimizer_success=valid))
    for kind in ('accepted','earlier_failure','transport_failure'):
        phases=('approach','contact','extract','transport');stages=[stage(p,True) for p in phases]
        if kind!='accepted':stages[-1 if kind=='transport_failure' else 0]['success']=False
        result=dict(success=kind=='accepted',stages=stages,seam_checks=[dict(phase=p,valid=True) for p in phases],wall_seconds=7.,motion_seconds=4.)
        snapshot=copy.deepcopy(result);calls=[]
        def original(*args,**kwargs):calls.append(kwargs);return result
        env=dict(Path=Path,time=NS(perf_counter=lambda:0.),time_initial_with_approach_recovery=original,
            ROOT=TOOLS.parent,hashlib=__import__('hashlib'),__file__=str(file),PHASES=phases)
        exec(compile(ast.Module(body=[fn],type_ignores=[]),str(file),'exec'),env)
        source=dict(attempts=[dict(transport_search=dict(selected_start_root=0))],source_sha256={})
        output=tmp_path/kind
        actual=env['time_layout_pick'](None,source,output,stop_on_failure=True)
        assert len(calls)==1 and calls[0]['stop_on_failure'] is True
        assert result==snapshot
        if kind!='transport_failure':assert actual is result
        else:
            initial,path=actual;assert path==output/'transport_recovery'
            assert initial['success'] and len(initial['stages'])==3
            assert not initial['is_full_cycle'] and not initial['complete_candidate']
            assert initial['motion_seconds']==3. and len(initial['seam_checks'])==3
            assert initial['transport_monolithic_rejection'] is result['stages'][-1]


def test_root0_physical_success_with_source_return_uses_same_native_recovery(tmp_path):
    import copy
    from curobo_core.box_pick import source_slot_process_checks
    file=TOOLS/'v3_layout_native.py';fn=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='time_layout_pick')
    boundary=next(i for i,n in enumerate(fn.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='transport_started' for t in n.targets))
    fn.body=fn.body[:boundary]+[ast.Return(value=ast.Name(id='initial',ctx=ast.Load()))];fn=ast.fix_missing_locations(fn)
    phases=('approach','contact','extract','transport')
    stages=[dict(phase=p,success=True,physical_success=True,rejection_reasons=[],time_optimization=dict(duration_s=1.),trajectory_optimization=dict(solver_success=True,optimizer_success=True)) for p in phases]
    source_obs={'left':obs(0.,.0005,0,True),'right':obs(0.,0.,0,False)}
    for value in source_obs.values():
        value['deeper_than_initial_m']=max(0.,value['maximum_slot_overlap_m']-value['initial_slot_overlap_m'])
        value['first_return_frame']=1 if value['returns_after_clear'] else None
    stages[-1]['source_slot_observation']=source_obs
    result=dict(success=True,stages=stages,seam_checks=[dict(phase=p,valid=True) for p in phases],wall_seconds=7.,motion_seconds=4.)
    snapshot=copy.deepcopy(result);events=[]
    env=dict(Path=Path,time=NS(perf_counter=lambda:0.),time_initial_with_approach_recovery=lambda *a,**kw:result,
        source_slot_process_checks=source_slot_process_checks,ROOT=TOOLS.parent,hashlib=__import__('hashlib'),__file__=str(file),PHASES=phases)
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(file),'exec'),env)
    source=dict(attempts=[dict(transport_search=dict(selected_start_root=0))],source_sha256={})
    initial=env['time_layout_pick'](None,source,tmp_path,progress=events.append)
    assert initial['success'] and len(initial['stages'])==3 and initial['motion_seconds']==3.
    failed=initial['transport_monolithic_rejection']
    assert not failed['success'] and failed['physical_success']
    assert not failed['process_validation']['valid'] and failed['rejection_reasons']
    assert result==snapshot and events[0]['state']=='rejected'


def test_native_extract_margin_validation_and_guidance_share_enabled_phase():
    """A collision-free spline below the native clearance must not pass extraction."""
    file=TOOLS/'v3_two_box_timing.py';tree=ast.parse(file.read_text())
    gate=next(n for n in ast.walk(tree) if isinstance(n,ast.If) and any(
        isinstance(call,ast.Call) and isinstance(call.func,ast.Attribute) and call.func.attr=='native_margin_mask'
        for call in ast.walk(n)) and isinstance(n.test,ast.BoolOp))
    flag=next(n for n in ast.walk(tree) if isinstance(n,ast.Assign)
        and "stage['native_project_margins_checked']" in ast.unparse(n))
    wrapper=ast.parse((TOOLS/'v3_lm_refinement.py').read_text())
    guidance=next(k.value for n in ast.walk(wrapper) if isinstance(n,ast.Call)
        and isinstance(n.func,ast.Name) and n.func.id=='refine_lm_path'
        for k in n.keywords if k.arg=='native_project_margins')
    for enabled in (False,True):
        for phase in ('approach','contact','extract','transport'):
            planner=NS(native_project_margins=enabled,native_margin_mask=lambda *args:torch.tensor([False,True]))
            env=dict(phase=phase,planner=planner,checker=None,values=None,stage={},mask=np.ones(2,bool))
            exec(compile(ast.Module(body=[gate,flag],type_ignores=[]),str(file),'exec'),env)
            assert bool(env['mask'][0]) == (not enabled or phase not in ('extract','transport'))
            assert env['stage']['native_project_margins_checked'] == (enabled and phase!='contact')
            assert eval(compile(ast.Expression(guidance),'<guidance>','eval'),env) == (enabled and phase=='extract')
    core=ast.parse((TOOLS/'curobo_core/lm_refinement.py').read_text())
    penalty=next(n for n in ast.walk(core) if isinstance(n,ast.If) and ast.unparse(n.test)=='base_layer is not None')
    cost=torch.tensor([.5*(.005-.00493358)**2/.005],requires_grad=True)
    env=dict(base_layer=NS(cost=lambda *args,**kwargs:cost),state=None,loss=0.,native_project_margins=True)
    exec(compile(ast.Module(body=[penalty],type_ignores=[]),'<base penalty>','exec'),env)
    torch.testing.assert_close(env['loss'],1e6*cost.mean())
    env['loss'].backward();assert float(cost.grad[0])==1e6
    selection=next(n for n in ast.walk(core) if isinstance(n,ast.Assign)
        and any(isinstance(t,ast.Name) and t.id=='ground_indices' for t in n.targets))
    ground=next(n for n in ast.walk(core) if isinstance(n,ast.Assign)
        and any(isinstance(t,ast.Name) and t.id=='penetration' for t in n.targets))
    env=dict(torch=torch,checker=NS(kinematics=NS(total_spheres=3)),
        validity=NS(ground_z=0.,ground_exempt_sphere_indices={1}),native_project_margins=True)
    exec(compile(ast.Module(body=[selection],type_ignores=[]),'<ground exemptions>','exec'),env)
    spheres=torch.tensor([[[0.,0.,.011,.01],[0.,0.,0.,.1],[0.,0.,.013,.01]]])
    env['ground']=spheres[:,env['ground_indices']]
    exec(compile(ast.Module(body=[ground],type_ignores=[]),'<ground penalty>','exec'),env)
    torch.testing.assert_close(env['penetration'],torch.tensor([[.001,0.]]))


def test_bounded_ordered_extract_uses_piece_displacements_and_real_rest_seams(tmp_path):
    from dataclasses import replace
    from curobo_core.ordered_pick import OrderedStage
    file=TOOLS/'experiments/box_layout_20261119/transfer_fresh_native/native_stages.py'
    function=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='bounded_stage')
    rows=np.zeros((5,15));rows[:,0]=np.linspace(0.,-.3,5)
    spec=OrderedStage('lower_extract','extract',('left','right'),(),
        cartesian_displacements=(('left',(0.,0.,0.)),('right',(-.3,0.,0.))))
    calls=[];stored={}
    checker=NS(task_tool_to_box={'left':np.eye(4),'right':np.eye(4)})
    planner=NS(selected_task={},set_stage=lambda context,start:None,checker=lambda boxes,payload:checker)
    def poses(checker,values,attachments):
        result={s:np.repeat(np.eye(4)[None],len(values),axis=0) for s in attachments}
        result['right'][:,0,3]=values[:,0]
        return result
    def time_pick(engine,request,output,**kwargs):
        output.mkdir();context=kwargs['stage_context'];calls.append((context,np.asarray(request['frames'])))
        q=np.asarray(request['frames']);count=len(q)
        data=dict(times=np.linspace(0.,1.,count),position=q,velocity=np.zeros_like(q),
            acceleration=np.zeros_like(q),jerk=np.zeros_like(q),torque=np.zeros((count,14)))
        path=str(output/'extract.npz');stored[path]=data
        return dict(stages=[dict(success=len(calls)>1,trajectory_file=path)])
    env=dict(np=np,replace=replace,time=NS(perf_counter=lambda:0.),time_pick=time_pick,
        load=lambda path:stored[path],box_poses=poses,concatenate_native=concatenate_native)
    exec(compile(ast.Module(body=[function],type_ignores=[]),str(file),'exec'),env)
    data,report=env['bounded_stage'](NS(planner=planner),{},rows,tmp_path,'extract',spec=spec)
    assert report['success'] and len(report['rejections'])==1 and len(report['pieces'])==2
    assert len(calls)==3
    for context,q in calls[1:]:
        np.testing.assert_array_equal(dict(context.cartesian_displacements)['left'],[0.,0.,0.])
        np.testing.assert_allclose(dict(context.cartesian_displacements)['right'],[-.15,0.,0.])
    np.testing.assert_array_equal(data['position'],rows)
    assert all(seam['valid'] for seam in report['internal_qva_seams'])
