"""Ordered native stage context; original boxes persist until their actual attachment boundary."""
from dataclasses import asdict, replace
from pathlib import Path
import argparse
import gc
import hashlib
import json
import time
import numpy as np
import torch

from curobo_core.adapter import matrix_pose, to_curobo_scene
from curobo_core.box_pick import source_slot_observation, source_slot_process_checks
from curobo_core.native_transport import concatenate_native,native_seam, source_progress, cpu_source_observations
from curobo_core.ordered_pick import OrderedStage, ordered_stages
from curobo_core.scene import AttachedObject, Pose, SceneObject
from v3_box_layout import atomic_json, parse_layout, session_source_stamp
from v3_box_layout_planner import BoxLayoutPlanner, ROOT, ACTIVE_JOINTS
from v3_single_arm_planner import SingleArmPlanner
from v3_batched_loaded_search import GpuValidity,densify
from v3_multiroot_search import batched_rrt_connect_multi_goal
from v3_lm_conveyor import SequentialPlacementLm
from curobo_core.box_pick import diverse_contact_order,layout_model_key
from curobo_core.layout_contract import carry_configuration,carry_reference
from v3_fast_three_layers import FastThreeLayerPlanner, POLICY
from v3_layout_native import time_pick
from v3_lm_conveyor import box_poses


class OrderedLayoutPlanner(BoxLayoutPlanner):
    def __init__(self, picks, owner_to_physical_id=None):
        self.owner_to_physical_id = dict({'left':'wall_box_00','right':'wall_box_01'} if owner_to_physical_id is None else owner_to_physical_id)
        if set(self.owner_to_physical_id) != {'left','right'} or set(self.owner_to_physical_id.values()) != {'wall_box_00','wall_box_01'}:
            raise ValueError('bijective actual owner to original physical ID required')
        self.native_stage = None
        super().__init__()
        self.configure_layout(picks, self.owner_to_physical_id)

    def task_attachments(self, boxes):
        if self.native_stage is None:
            return super().task_attachments(boxes)
        return tuple(AttachedObject(self.owner_to_physical_id[side], self.picks[side].dimensions_m,
            side+'_tool0', matrix_pose(self.picks[side].tool_to_box()), (side+'_link7', side+'_tool0'))
            for side in self.native_stage.payload_sides)

    def set_stage(self, stage, start):
        if not isinstance(stage, OrderedStage):raise ValueError('ordered stage contract required')
        start = np.asarray(start, dtype=float)
        if start.shape != (15,) or not np.isfinite(start).all():raise ValueError('finite complete stage start required')
        self.native_stage = stage
        selected = (stage.incoming_side,) if stage.incoming_side is not None else stage.payload_sides
        self.selected_task = {s:int(self.owner_to_physical_id[s][-2:]) for s in selected}
        self.pick = self.picks[stage.incoming_side or selected[0]]
        objects = self.environment+tuple(SceneObject(self.owner_to_physical_id[s],
            self.picks[s].dimensions_m, Pose(self.picks[s].center_m)) for s in stage.pending_sides)
        cfg = dict(zip(ACTIVE_JOINTS, start))
        state = replace(self.snapshot.state, positions=tuple(float(cfg.get(n, q)) for n,q in zip(
            self.snapshot.state.joint_names, self.snapshot.state.positions)), stamp_ns=time.time_ns())
        self.snapshot = replace(self.snapshot, state=state, objects=objects,
            attachments=self.task_attachments(self.selected_task),
            policy=replace(self.snapshot.policy, defer_payload_until_extract_end=False), revision=self.snapshot.revision+1)

    def checker(self, boxes, payload=False, mobile=False):
        if self.native_stage is not None and payload != bool(self.native_stage.payload_sides):
            raise ValueError('requested checker does not match current physical payload state')
        checker = super().checker(boxes, payload=payload, mobile=mobile)
        if payload and self.native_stage.incoming_side is not None:
            # The incoming carton has TCP contact treatment, but remains a full obstacle for every carried carton.
            incoming = self.owner_to_physical_id[self.native_stage.incoming_side]
            full_scene = to_curobo_scene(self.snapshot)
            present = {o.name for o in checker.guidance_scene.cuboid}
            checker.guidance_scene.cuboid.extend(o for o in full_scene.cuboid if o.name == incoming and o.name not in present)
        return checker


def _geometry_validity(planner, spec, checker, start):
    validity=(planner.target_validity(checker,allow_contact=spec.phase=='contact')
              if spec.incoming_side is not None else GpuValidity(checker,check_ground=True))
    validity.scene_penetration_tolerance_m=.001;validity.contact_penetration_tolerance_m=.001
    validity.steering_steps=(.28,.03,.01,.003);validity.validate_returned_path=True
    active=[s for s in planner.picks if s in checker.task_tool_to_box]
    indices=[active.index(s) for s in spec.source_sides]
    initial=(planner.source_slot_depths(checker,torch.as_tensor(np.asarray(start,dtype=np.float32)[None],device='cuda'))[:,indices]
             if indices else None)
    limit=initial.clamp(min=0)+1e-6 if initial is not None else None
    original=validity.evaluate
    def evaluate(values,base_margin_m=0.):
        count=len(values);extra=(-count)%512
        rows=values if not extra else torch.cat((values,values[-1:].expand(extra,-1)))
        mask,stability=original(rows,base_margin_m)
        if spec.payload_sides:
            candidates=mask.nonzero().reshape(-1)
            if len(candidates):mask[candidates]&=planner.payload_search_mask(checker,rows[candidates])
            mask&=planner.native_margin_mask(checker,rows)
            candidates=mask.nonzero().reshape(-1)
            if len(candidates):mask[candidates]&=planner.static_load_mask(checker,rows[candidates])
        if indices:mask&=(planner.source_slot_depths(checker,rows)[:,indices]<=limit).all(-1)
        return mask[:count],stability[:count]
    validity.evaluate=evaluate
    if spec.source_sides:
        def quality(points):
            poses=box_poses(checker,points,{s:planner.picks[s].tool_to_box() for s in spec.source_sides})
            return source_slot_process_checks({s:source_slot_observation(planner.picks[s],T) for s,T in poses.items()})['valid']
        validity.path_quality_callback=quality
    return validity


def _geometry_free_path(planner,spec,start,goals,checker,budget=8.):
    if not len(goals):return None,dict(failure='no stage goals')
    goals=np.asarray(goals,dtype=np.float32)
    validity=_geometry_validity(planner,spec,checker,start)
    lower,upper=checker.kinematics.get_joint_limits().position
    path,search=batched_rrt_connect_multi_goal(torch.as_tensor(start,device='cuda',dtype=torch.float32),
        torch.as_tensor(goals,device='cuda'),lower+1e-5,upper-1e-5,validity,budget,20261123,stop_at_first_valid=True)
    if path is not None:
        path=np.asarray(densify(path),dtype=np.float32)
        if not bool(validity.mask(torch.as_tensor(path,device='cuda')).all()):return None,dict(search,failure='actual stage path failed checked geometry/load/source masks')
    return path,search


def _joint_stage_goals(planner,spec,start,checker):
    lo,hi=carry_reference()[2]
    goals=np.repeat(np.asarray(start,dtype=np.float32)[None],101,axis=0)
    for i,lift in enumerate(np.linspace(lo,hi,101)):
        cfg=carry_configuration(lift);goals[i,0]=lift
        for column,name in enumerate(ACTIVE_JOINTS):
            if any(name.startswith(side+'_') for side in spec.final_joint_sides):goals[i,column]=cfg[name]
    mask=_geometry_validity(planner,spec,checker,start).mask(torch.as_tensor(goals,device='cuda')).cpu().numpy()
    for i in np.flatnonzero(mask):mask[i]=planner.loaded_audit(checker,goals[i:i+1])['valid']
    goals=goals[mask]
    weights=np.r_[np.deg2rad(15)/.1,np.ones(14)]
    return goals[np.argsort(np.linalg.norm((goals-start)*weights,axis=1))]


def plan_ordered_geometry(picks,output,max_contacts=16,progress=lambda event:None,*,adapt_grasp=True):
    """Compute fresh upper-first geometry with explicit pending/attached states; no recorded path input."""
    began=time.perf_counter();stamp=session_source_stamp();output=Path(output);output.mkdir(parents=True,exist_ok=False)
    original=dict(picks);upper=max(picks,key=lambda s:picks[s].center_m[2]);lower='left' if upper=='right' else 'right'
    picks=dict(picks)
    if adapt_grasp and not picks[upper].top:
        picks[upper]=replace(picks[upper],side_face='y-' if upper=='right' else 'y+',suction_roll_deg=90.,suction_offset_m=(0.,0.,0.))
    planner=OrderedLayoutPlanner(picks);engine=FastThreeLayerPlanner(planner=planner)
    planner.layout_native=(layout_model_key(picks),engine)
    stages=ordered_stages(picks);segments={};attempts=[];qhome=planner.home_values.astype(np.float32).copy()
    def checker_for(spec,start):
        planner.set_stage(spec,start)
        return planner.checker(planner.selected_task,payload=bool(spec.payload_sides))
    def publish():
        atomic_json(output/'geometry_status.json',dict(success=len(segments)==8,attempts=attempts,completed_stages=list(segments),wall_seconds=time.perf_counter()-began))
    def extract(spec,start,checker):
        payload=checker.task_tool_to_box;poses=box_poses(checker,np.asarray(start)[None],payload)
        starts={s:T[0] for s,T in poses.items()};ends={s:T.copy() for s,T in starts.items()}
        for side,delta in spec.cartesian_displacements:ends[side][:3,3]+=delta
        key=tuple(spec.payload_sides)
        if key not in planner.extraction_engines:planner.extraction_engines[key]=SequentialPlacementLm(planner,active_sides=key)
        validity=_geometry_validity(planner,spec,checker,start)
        case=dict(start=np.asarray(start,dtype=np.float32),checker=checker,attachments=payload,starts=starts,ends=ends,
                  candidate_filter=lambda values,state,actual:validity.mask(values).cpu().numpy())
        result=planner.extraction_engines[key].plan(case)
        frames=np.asarray(result['frames'],dtype=np.float32)
        if result['success']:
            result['full_box_audit']=planner.loaded_audit(checker,frames);result['success']&=result['full_box_audit']['valid']
        if result['success']:
            moved=next(side for side,delta in spec.cartesian_displacements if np.linalg.norm(delta)>0)
            active=[side for side in planner.picks if side in payload]
            depth=float(planner.source_slot_depths(checker,torch.as_tensor(frames[-1:],device='cuda'))[0,active.index(moved)])
            if depth > -5e-6:
                # Proposal only:50um numerical clearance inside the inherited2mm Cartesian gate; exact original remains fallback.
                biased={side:T.copy() for side,T in ends.items()}
                biased[moved][0,3]-=5e-5
                proposal=planner.extraction_engines[key].plan(dict(case,ends=biased))
                proposed=np.asarray(proposal['frames'],dtype=np.float32)
                if proposal['success']:
                    proposal['full_box_audit']=planner.loaded_audit(checker,proposed)
                    proposal['success']&=proposal['full_box_audit']['valid']
                diagnostics=dict(original_signed_source_depth_m=depth,proposal_extra_minus_x_m=5e-5,
                    proposal_accepted=bool(proposal['success']),proposal_failure=proposal['failure'],acceptance_tolerance_unchanged_m=.002)
                if proposal['success']:frames,result=proposed,proposal
                result['source_clearance_proposal']=diagnostics
        return frames,result
    empty=checker_for(stages[0],qhome)
    contacts=planner.enumerate_single(picks[upper].contact_tool(),qhome,empty,limit=256)
    if len(contacts):contacts=contacts[diverse_contact_order(contacts,upper)]
    # ponytail: first feasible upper parking; search other parking goals if it blocks a known feasible lower pickup.
    upper_ready=False
    for i,q in enumerate(contacts[:max_contacts]):
        spec=stages[1];checker=checker_for(spec,q)
        cap=planner.target_validity(checker,allow_contact=True)
        distance=picks[upper].precontact_distance(cap.targets[upper]['tool_cap_reach_m'])
        reverse,detail=SingleArmPlanner.cartesian_single(planner,q,np.linspace(.005,distance,int(np.ceil(distance/.005))),along_normal=True)
        trial=dict(part='upper',index=i,precontact=reverse is not None)
        if reverse is not None:
            contact=np.asarray(reverse[::-1],dtype=np.float32)
            if bool(_geometry_validity(planner,spec,checker,q).mask(torch.as_tensor(contact,device='cuda')).all()):
                loaded=checker_for(stages[2],q);rows,result=extract(stages[2],q,loaded)
                trial['extraction']={k:v for k,v in result.items() if k not in ('frames','point_reports','failed_candidates','progress')}
                if result['success']:
                    checker=checker_for(stages[0],qhome)
                    approach,search=_geometry_free_path(planner,stages[0],qhome,contact[:1],checker)
                    trial['approach_search']=search
                    if approach is not None:
                        checker=checker_for(stages[3],rows[-1]);goals=_joint_stage_goals(planner,stages[3],rows[-1],checker)
                        park,search=_geometry_free_path(planner,stages[3],rows[-1],goals,checker,32.)
                        trial['park_search']=search
                        if park is not None:
                            segments.update(upper_approach=approach,upper_contact=contact,upper_extract=rows,upper_park=park)
                            upper_ready=True
        attempts.append(trial);publish();progress(dict(stage='ordered_upper_geometry',kind='geometry',state='done' if upper_ready else 'rejected',attempt=i))
        if upper_ready:break
    if not upper_ready:return dict(success=False,failure='upper ordered geometry candidate budget exhausted; not proof of input illegality',attempts=attempts),planner,engine
    parked=segments['upper_park'][-1];checker=checker_for(stages[4],parked)
    tools=planner._tool_poses(checker,torch.as_tensor(parked,device='cuda'))
    contacts=planner.enumerate_pair({lower:picks[lower].contact_tool(),upper:tools[upper]},parked,checker,contact=True,limit=256)
    for i,q in enumerate(contacts[:max_contacts]):
        checker=checker_for(stages[5],q);cap=planner.target_validity(checker,allow_contact=True)
        distance=picks[lower].precontact_distance(cap.targets[lower]['tool_cap_reach_m'])
        contact=planner.precontact(q,{lower:distance,upper:0.},checker)
        trial=dict(part='lower',index=i,precontact=contact is not None)
        if contact is not None and planner.loaded_audit(checker,contact)['valid']:
            loaded=checker_for(stages[6],q);rows,result=extract(stages[6],q,loaded)
            trial['extraction']={k:v for k,v in result.items() if k not in ('frames','point_reports','failed_candidates','progress')}
            if result['success']:
                checker=checker_for(stages[4],parked)
                approach,search=_geometry_free_path(planner,stages[4],parked,contact[:1],checker)
                trial['approach_search']=search
                if approach is not None:
                    checker=checker_for(stages[7],rows[-1]);goals=_joint_stage_goals(planner,stages[7],rows[-1],checker)
                    transport,search=planner.pair_path(rows[-1],goals,checker,budget_s=32.) if len(goals) else (None,dict(failure='no final physical joint goal'))
                    trial['transport_search']=search
                    if transport is not None:segments.update(lower_approach=approach,lower_contact=contact,lower_extract=rows,final_transport=transport)
        attempts.append(trial);publish();progress(dict(stage='ordered_lower_geometry',kind='geometry',state='done' if len(segments)==8 else 'rejected',attempt=i))
        if len(segments)==8:break
    if len(segments)!=8:return dict(success=False,failure='lower ordered geometry candidate budget exhausted; not proof of input illegality',attempts=attempts),planner,engine
    previous=None;files=[]
    for spec in stages:
        rows=np.asarray(segments[spec.stage_id],dtype=np.float32)
        if previous is not None and np.max(np.abs(previous-rows[0]))>1e-5:raise ValueError('fresh ordered geometry seam rejected')
        path=output/(spec.stage_id+'.npz');np.savez_compressed(path,position=rows)
        files.append(dict(stage_id=spec.stage_id,file=str(path.resolve()),array='position',sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        previous=rows[-1]
    manifest=dict(success=True,workflow='ordered_upper_first',geometry_fresh=True,geometry_source_stamp=stamp,
        request=dict(frame_id='world',boxes=[dict(size=p.size,center_m=p.center_m,side=s,side_min_height_m=p.side_min_height_m,
            swap_length_height=p.swap_length_height) for s,p in original.items()]),
        selected_task={s:asdict(p) for s,p in picks.items()},stages=files,attempts=attempts,wall_seconds=time.perf_counter()-began,
        source_unchanged=session_source_stamp()==stamp)
    atomic_json(output/'manifest.json',manifest)
    return manifest,planner,engine


def plan_ordered_pick(picks,output,max_contacts=16,progress=lambda event:None,*,adapt_grasp=True):
    began=time.perf_counter();output=Path(output);output.mkdir(parents=True,exist_ok=False)
    geometry,planner,engine=plan_ordered_geometry(picks,output/'geometry',max_contacts,progress,adapt_grasp=adapt_grasp)
    if not geometry['success']:
        geometry.update(workflow='ordered_upper_first',real_robot_execution=False,geometry_fresh=True,task={s:asdict(p) for s,p in picks.items()})
        atomic_json(output/'result.json',geometry)
        planner.layout_native=(None,None);del engine,planner;gc.collect()
        return geometry
    if not geometry['source_unchanged']:raise ValueError('source changed during fresh ordered geometry')
    selected={s:asdict(p) for s,p in planner.picks.items()}
    native_manifest=dict(geometry,request=dict(geometry['request'],boxes=[dict(b,side_face=selected[b['side']]['side_face'],
        suction_roll_deg=selected[b['side']]['suction_roll_deg'],suction_offset_m=selected[b['side']]['suction_offset_m']) for b in geometry['request']['boxes']]))
    result=time_ordered_geometry(native_manifest,output/'native',progress,planner=planner,engine=engine,geometry_fresh=True)
    result['request']=geometry['request'];result['fresh_geometry']=geometry
    result['native_wall_seconds']=result['wall_seconds'];result['wall_seconds']=time.perf_counter()-began
    result['geometry_wall_seconds']=geometry['wall_seconds'];result['ordered_planner_reused']=False
    result['geometry_success']=True
    result['physical_success']=len(result['stages'])==8 and all(s.get('physical_success') and s.get('actual_stage_seam',{}).get('valid') for s in result['stages'])
    result['process_validation']=dict(valid=result['success'],policy='no_deeper_overlap_and_no_return_after_clear_v1',
        both_sources_cleared=result.get('both_sources_cleared'),scope='actual ordered GPU and CPU source history across all stages')
    result['attempts']=geometry['attempts']
    result['timing']={key:result[key] for key in ('success','workflow','stages','real_robot_execution')}
    if 'motion_seconds' in result:result['timing']['motion_seconds']=result['motion_seconds']
    source_files=[Path(__file__),ROOT/'tools/curobo_core/ordered_pick.py',ROOT/'tools/v3_two_box_timing.py',ROOT/'tools/v3_fast_three_layers.py',
        ROOT/'tools/v3_box_layout_planner.py',ROOT/'tools/curobo_core/lm_refinement.py',ROOT/'tools/v3_lm_refinement.py',ROOT/'tools/curobo_core/torque.py',
        ROOT/'tools/curobo_core/box_pick.py',ROOT/'tools/curobo_core/suction_grasp.py',ROOT/'tools/curobo_core/backend.py',
        ROOT/'generated/fast_runtime_assets/fixed.urdf',ROOT/'generated/fast_runtime_assets/fixed.yaml',ROOT/'config/fast_three_layers.json']
    result['source_sha256']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files}
    atomic_json(output/'result.json',result)
    planner.layout_native=(None,None);del engine,planner;gc.collect()
    return result


def _observations(planner, stage, rows, *, cpu=False):
    sides = stage.source_sides
    if not sides:return {}
    picks = {s:planner.picks[s] for s in sides}
    if cpu:
        if not hasattr(planner, '_ordered_cpu_robot'):
            import yourdfpy
            planner._ordered_cpu_robot = yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf', load_meshes=False)
        return cpu_source_observations(planner._ordered_cpu_robot, rows, ACTIVE_JOINTS, picks)
    checker = planner.checker(planner.selected_task, payload=bool(stage.payload_sides))
    poses = box_poses(checker, rows, {s:planner.picks[s].tool_to_box() for s in sides})
    return {s:source_slot_observation(picks[s], poses[s]) for s in sides}


def _time_native_stage(engine,source,spec,output,previous,ref_gpu,clear_gpu,ref_cpu,clear_cpu,progress):
    """Recover a rejected RRT representation using actual native32 pieces and original source history."""
    timed=time_pick(engine,source,output,stop_on_failure=True,phase_names=(spec.phase,),
                    final_transport=True,stage_context=spec,progress=progress)
    original=timed['stages'][0]
    if timed['success'] or spec.phase not in ('approach','transport'):return timed
    planner=engine.planner;rows=np.asarray(source['frames'],dtype=np.float32)
    directory=Path(output)/'native_pieces';directory.mkdir(parents=True,exist_ok=False)
    planner.set_stage(spec,rows[0]);physical_scene=planner.snapshot.to_dict()
    pending=[(0,len(rows)-1,'0')];accepted=[];history=[];arrays=[]
    gpu_clear=dict(clear_gpu);cpu_clear=dict(clear_cpu)
    prior=previous;failure=None
    while pending:
        a,b,name=pending.pop(0);final=not pending
        piece_spec=spec if final else replace(spec,final_joint_sides=())
        planner.set_stage(piece_spec,rows[a])
        request=dict(source,frames=rows[a:b+1].tolist(),phases=[spec.phase]*(b-a+1))
        trials=[];chosen=None
        modes=(False,True) if spec.phase=='transport' else (False,)
        for seed_only in modes:
            if a==0 and b==len(rows)-1 and not seed_only:
                trials.append(dict(mode='optimized',success=False,reused_monolithic_rejection=True,stage=dict(original)));continue
            mode='validated_seed' if seed_only else 'optimized'
            progress(dict(stage=spec.stage_id,kind='native_piece',piece=name,state='running',mode=mode))
            result=time_pick(engine,request,directory/(name+'_'+mode),stop_on_failure=True,phase_names=(spec.phase,),
                final_transport=final,seed_only=seed_only,stage_context=piece_spec,progress=progress)
            stage=result['stages'][0];trial=dict(mode=mode,stage=stage,success=False)
            if result['success']:
                with np.load(stage['trajectory_file'],allow_pickle=False) as data:actual={k:data[k].copy() for k in data.files}
                seam=dict(valid=True) if prior is None else native_seam(prior,actual)
                good=seam['valid'];processes={};next_gpu=dict(gpu_clear);next_cpu=dict(cpu_clear)
                for backend,cpu,reference,cleared in [('gpu',False,ref_gpu,gpu_clear),('cpu',True,ref_cpu,cpu_clear)]:
                    obs=_observations(planner,piece_spec,actual['position'],cpu=cpu)
                    if obs:
                        valid,_,next_clear,checks=source_progress(obs,reference,cleared)
                        good &= valid;processes[backend]=dict(valid=valid,checks=checks,observations=obs)
                        (next_cpu if cpu else next_gpu).update(next_clear)
                trial.update(success=bool(good),actual_seam=seam,whole_stage_source_history=processes)
                if good:chosen=(actual,stage,next_gpu,next_cpu)
            trials.append(trial)
            progress(dict(stage=spec.stage_id,kind='native_piece',piece=name,state='accepted' if trial['success'] else 'rejected',mode=mode))
            if chosen is not None:break
            if stage.get('trajectory_optimization',{}).get('seed_retention') is not None:break
        if chosen is None:
            history.append(dict(start=a,end=b,piece=name,trials=trials))
            # ponytail:64 genuine native leaves, then retain an unknown failure rather than repair sampled arrays.
            if b-a>1 and len(pending)+len(accepted)+2<=64:
                middle=(a+b)//2;pending[0:0]=[(a,middle,name+'a'),(middle,b,name+'b')]
            else:
                failure='ordered native representation recovery exhausted; not input illegality';break
        else:
            actual,stage,gpu_clear,cpu_clear=chosen
            accepted.append(dict(start=a,end=b,piece=name,stage=stage,trials=trials))
            arrays.append(actual);prior=actual
        atomic_json(directory/'progress.json',dict(accepted=accepted,rejected=history,failure=failure))
    planner.set_stage(spec,rows[0])
    if failure is not None or not arrays:
        original['ordered_representation_recovery']=dict(success=False,failure=failure,pieces=accepted,rejections=history)
        return timed
    merged,seams=concatenate_native(arrays)
    from curobo_core.box_pick import timed_path_checks
    check=timed_path_checks(merged,rows[0],rows[-1],POLICY)
    if not check['valid']:raise ValueError('merged actual native stage limits/endpoints rejected')
    file=directory/'stage.npz';np.savez_compressed(file,**merged)
    selected=[p['stage'] for p in accepted];last=selected[-1]
    combined=dict(phase=spec.phase,stage_id=spec.stage_id,success=True,physical_success=True,geometry_valid=True,
        payload_sides=list(spec.payload_sides),pending_sides=list(spec.pending_sides),physical_scene=physical_scene,
        trajectory_file=str(file.resolve()),representation='piecewise_native32_with_actual_rest_seams',
        control_files=[s['control_file'] for s in selected],array_checks=check,seams=seams,pieces=accepted,
        monolithic_rejection=original,representation_rejections=history,rejection_reasons=[],
        native_project_margins_checked=all(s.get('native_project_margins_checked') for s in selected),
        trajectory_optimization=dict(solver_success=all(s['trajectory_optimization']['solver_success'] for s in selected),
            optimizer_success=all(s['trajectory_optimization'].get('optimizer_success') is True for s in selected),scope='selected genuine native pieces; rejected whole path retained'),
        time_optimization=dict(success=True,dynamic_valid=all(s['time_optimization']['dynamic_valid'] for s in selected),
            duration_s=float(merged['times'][-1]),status=0,scope='unchanged actual timed piece arrays'),
        independent_rnea=dict(valid=all(s['independent_rnea']['valid'] for s in selected),
            peak_torque_ratio=max(s['independent_rnea']['peak_torque_ratio'] for s in selected),
            max_native_difference_nm=max(s['independent_rnea']['max_native_difference_nm'] for s in selected)))
    for key in ('carry_joint_endpoint','carry_envelope_final'):
        if key in last:combined[key]=last[key]
    return dict(timed,success=True,stages=[combined],motion_seconds=float(merged['times'][-1]),
                all_native_geometry_results_success=True,
                all_trajopt_success=combined['trajectory_optimization']['optimizer_success'])


def time_ordered_geometry(manifest, output, progress=lambda event:None, *, planner=None, engine=None, geometry_fresh=False):
    """Time recorded geometry seeds honestly; production fresh ordered geometry is a separate step."""
    began = time.perf_counter();stamp = session_source_stamp()
    from curobo_core.box_pick import BoxPick
    picks = ({side:BoxPick(**task) for side,task in manifest['selected_task'].items()} if 'selected_task' in manifest else parse_layout(manifest['request']));stages = ordered_stages(picks)
    if [r['stage_id'] for r in manifest['stages']] != [s.stage_id for s in stages]:
        raise ValueError('ordered geometry must contain the complete declared eight-stage sequence')
    output = Path(output);output.mkdir(parents=True, exist_ok=False)
    planner = OrderedLayoutPlanner(picks) if planner is None else planner
    engine = FastThreeLayerPlanner(planner=planner) if engine is None else engine
    previous = None;ref_gpu = {};clear_gpu = {};ref_cpu = {};clear_cpu = {}
    result = dict(success=False, workflow='ordered_upper_first', geometry_fresh=geometry_fresh,
        geometry_provenance='fresh_same_process_ordered_geometry' if geometry_fresh else 'recorded_component_geometry_seeds_for_native_integration',
        request=manifest['request'], task={s:asdict(p) for s,p in picks.items()}, joint_names=ACTIVE_JOINTS,
        real_robot_execution=False, payload_kg_per_box=POLICY['payload_kg_per_box'], stages=[], source_stamp=stamp,
        frame_id='world',world_from_robot_base=np.eye(4).tolist(),
        verification_scope='New real native optimization/timeopt/sample/RNEA per ordered stage; not fresh geometry planning, hardware or universal task coverage')
    atomic_json(output/'manifest.json',manifest)
    for spec, seed in zip(stages, manifest['stages']):
        path = Path(seed['file'])
        if hashlib.sha256(path.read_bytes()).hexdigest() != seed['sha256']:raise ValueError('geometry seed changed')
        with np.load(path, allow_pickle=False) as data:rows = np.asarray(data[seed['array']],dtype=np.float32)
        if rows.ndim != 2 or rows.shape[1] != 15 or len(rows)<2 or not np.isfinite(rows).all():raise ValueError('invalid geometry seed rows')
        if previous is not None and np.max(np.abs(previous['position'][-1]-rows[0]))>1e-5:raise ValueError('ordered geometry start is disconnected from actual native endpoint')
        planner.set_stage(spec,rows[0])
        if not result['stages']:result['scene']=planner.snapshot.to_dict()
        stand_off = {}
        if spec.incoming_side is not None:
            checker=planner.checker(planner.selected_task,payload=bool(spec.payload_sides))
            contact=planner.target_validity(checker,allow_contact=True)
            side=spec.incoming_side
            stand_off[side]=picks[side].precontact_distance(contact.targets[side]['tool_cap_reach_m'])
        source=dict(geometry_success=True,task={s:asdict(p) for s,p in picks.items()},joint_names=ACTIVE_JOINTS,
            frames=rows.tolist(),phases=[spec.phase]*len(rows),source_sha256={},precontact_distance_m=stand_off)
        progress(dict(stage=spec.stage_id,state='running',payload_sides=spec.payload_sides,pending_sides=spec.pending_sides))
        timed=_time_native_stage(engine,source,spec,output/spec.stage_id,previous,ref_gpu,clear_gpu,ref_cpu,clear_cpu,
            lambda e:progress(dict(e,stage_id=spec.stage_id)))
        record=dict(timed['stages'][0],stage_id=spec.stage_id,physical_contract=asdict(spec),native_result=timed)
        accepted=timed['success'];data=None
        if accepted:
            with np.load(record['trajectory_file'],allow_pickle=False) as arrays:data={k:arrays[k].copy() for k in arrays.files}
            seam=dict(valid=True) if previous is None else native_seam(previous,data)
            record['actual_stage_seam']=seam;accepted &= seam['valid']
            gpu=_observations(planner,spec,data['position'])
            cpu=_observations(planner,spec,data['position'],cpu=True)
            processes={};next_states={}
            for name,observations,reference,cleared in [('gpu',gpu,ref_gpu,clear_gpu),('cpu',cpu,ref_cpu,clear_cpu)]:
                if observations:
                    valid,_,next_clear,checks=source_progress(observations,reference,cleared)
                    processes[name]=dict(valid=valid,checks=checks,observations=observations)
                    accepted &= valid
                    next_states[name]=next_clear
            record['carried_original_slot_process']=processes
            if accepted:
                clear_gpu.update(next_states.get('gpu',{}));clear_cpu.update(next_states.get('cpu',{}))
        record['success']=bool(accepted);result['stages'].append(record)
        if accepted:
            previous=data
            if spec.phase=='extract':
                moved=next(s for s,v in spec.cartesian_displacements if np.linalg.norm(v)>0)
                for cpu,reference,cleared in [(False,ref_gpu,clear_gpu),(True,ref_cpu,clear_cpu)]:
                    end_spec=replace(spec,source_sides=(moved,))
                    obs=_observations(planner,end_spec,data['position'][-1:],cpu=cpu)[moved]
                    reference[moved]=obs['initial_slot_overlap_m'];cleared[moved]=obs['first_clear_frame'] is not None
        result.update(wall_seconds=time.perf_counter()-began,source_unchanged=session_source_stamp()==stamp)
        atomic_json(output/'result.json',result)
        if not accepted:break
    result['success']=len(result['stages'])==len(stages) and all(s['success'] for s in result['stages']) and result['source_unchanged']
    result['both_sources_cleared'] = all(clear_gpu.get(s,False) and clear_cpu.get(s,False) for s in picks)
    result['success'] &= result['both_sources_cleared']
    if result['success']:
        result['motion_seconds']=sum(s['time_optimization']['duration_s'] for s in result['stages'])
        result['task']={s:asdict(replace(p,carry_updown_m=float(previous['position'][-1,0]))) for s,p in picks.items()}
        from curobo_core.ordered_pick import verify_ordered_result
        try:result['independent_verification']=verify_ordered_result(result)
        except ValueError as error:
            result.update(success=False,independent_verification_error=str(error))
    atomic_json(output/'result.json',result)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--seed-manifest',type=Path)
    mode.add_argument('--request',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    progress=lambda event:print(json.dumps({k:v for k,v in event.items() if k!='details'},ensure_ascii=False),flush=True)
    if args.seed_manifest:
        result=time_ordered_geometry(json.loads(args.seed_manifest.read_text()),args.output,progress)
    else:
        document=json.loads(args.request.read_text());picks=parse_layout(document)
        result=plan_ordered_pick(picks,args.output,progress=progress,
            adapt_grasp=all(not any(k in row for k in ('side_face','suction_offset_m','suction_roll_deg')) for row in document['boxes']))
        result['request']=document
        if result['success']:
            from curobo_core.ordered_pick import verify_ordered_result
            result['independent_verification']=verify_ordered_result(result)
        atomic_json(args.output/'result.json',result)
    raise SystemExit(0 if result['success'] else 1)


if __name__=='__main__':main()
