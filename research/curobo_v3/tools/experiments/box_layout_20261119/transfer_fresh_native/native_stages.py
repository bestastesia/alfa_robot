"""Existing native planners, driven only by this call's fresh in-memory geometry."""
from dataclasses import replace
from pathlib import Path
import json
import time
import numpy as np
import torch
import yourdfpy
from curobo.types import JointState
from curobo_core.box_pick import BoxPick,layout_model_key,timed_path_checks
from curobo_core.adapter import matrix_pose
from curobo_core.scene import SceneObject,AttachedObject,Pose
from curobo_core.native_transport import concatenate_native,native_seam,cpu_source_observations,source_progress
from v3_layout_native import time_pick,time_initial_with_approach_recovery
from v3_fast_three_layers import FastThreeLayerPlanner,POLICY
from v3_box_layout_planner import BoxLayoutPlanner
from v3_lm_conveyor import box_poses
from v3_batched_loaded_search import GpuValidity
import v3_two_box_pick
import v3_fast_three_layers as native
from probe_suffix import SuffixPlanner,prepared_assets,measured,validate_attachment
from plan_fresh import ROOT,NAMES
from curobo_core.handover_roles import HandoverRoles,PHYSICAL_IDS

STANDARD=('times','position','velocity','acceleration','jerk','torque')

def load(path):
    with np.load(path,allow_pickle=False) as data:return {k:data[k].copy() for k in data.files}

def configure(output):
    v3_two_box_pick.prepare_fast_assets=prepared_assets;native.prepare_fast_assets=prepared_assets;native.OUTPUT=output/'resources'


def source_planner(document,report):
    picks={b['side']:BoxPick(**b) for b in document['boxes']};roles=HandoverRoles(**report.get('roles',{}))
    giver=roles.giver;source_id=PHYSICAL_IDS[roles.source];pending_id=PHYSICAL_IDS[roles.pending]
    owner=BoxPick(**(report['donor_task'] if 'donor_task' in report else report['left_donor_task']));pending=picks[roles.pending]
    if owner.side!=giver:raise ValueError('donor_task arm differs from actual giver')
    class SourcePlanner(v3_two_box_pick.TwoBoxPickPlanner):
        def task_attachments(self,boxes):
            return (AttachedObject(source_id,owner.dimensions_m,giver+'_tool0',matrix_pose(owner.tool_to_box()),(giver+'_link7',giver+'_tool0')),)
        def loaded_audit(self,checker,rows):
            audit=super().loaded_audit(checker,rows)
            strict=SuffixPlanner.strict_box_mask(self,checker,torch.as_tensor(rows,device='cuda',dtype=torch.float32))
            invalid=np.flatnonzero(~strict.cpu().numpy()).tolist();audit['strict_noncap_obb_invalid_indices']=invalid
            audit['invalid_frame_indices']=sorted(set(audit['invalid_frame_indices'])|set(invalid));audit['valid']=not audit['invalid_frame_indices'];return audit
    planner=SourcePlanner();planner.configure_pick(owner);planner.selected_task={giver:int(source_id[-2:])};planner.picks={giver:owner};planner.native_project_margins=True;planner.full_payload_trajopt=True
    planner.snapshot=replace(planner.snapshot,objects=planner.environment+(
        SceneObject(source_id,owner.dimensions_m,Pose(owner.center_m)),SceneObject(pending_id,pending.dimensions_m,Pose(pending.center_m))))
    return planner,owner


def bounded_stage(engine,source,rows,output,phase,*,spec=None,validate=lambda data:dict(valid=True),progress=lambda event:None):
    """Same existing <=8 native rest leaves, preserving every rejected candidate."""
    pending=[(0,len(rows)-1,'whole')];accepted=[];rejected=[];pieces=[];started=time.perf_counter()
    while pending:
        a,b,name=pending.pop(0);context=spec if spec is None or b==len(rows)-1 else replace(spec,final_joint_sides=())
        if context is not None:
            engine.planner.set_stage(context,rows[a])
            if phase=='extract' and (a or b!=len(rows)-1):
                checker=engine.planner.checker(engine.planner.selected_task,payload=True)
                poses=box_poses(checker,rows[[a,b]],checker.task_tool_to_box)
                translations=[]
                for side,delta in spec.cartesian_displacements:
                    delta=np.asarray(delta,dtype=float)
                    if np.any(delta):
                        fraction=float((poses[side][1,:3,3]-poses[side][0,:3,3])@delta/(delta@delta))
                        if not 0.<fraction<=1.+1e-5:raise ValueError('extract piece must progress along its original reference')
                        delta=delta*fraction
                    translations.append((side,tuple(delta)))
                context=replace(context,cartesian_displacements=tuple(translations))
                engine.planner.set_stage(context,rows[a])
        request=dict(source,frames=rows[a:b+1].tolist(),phases=[phase]*(b-a+1))
        result=time_pick(engine,request,output/name,stop_on_failure=True,progress=progress,phase_names=(phase,),
            final_transport=(False if context is None else (b==len(rows)-1 if phase=='transport' else True)),**({} if context is None else dict(stage_context=context)))
        stage=result['stages'][0];data=load(stage['trajectory_file']) if stage.get('trajectory_file') else None
        audit=validate(data) if data is not None else dict(valid=False)
        good=bool(stage['success'] and audit['valid'])
        trial=dict(name=name,stage=stage,audit=audit,accepted=good);(accepted if good else rejected).append(trial)
        if good:pieces.append(data)
        elif (phase in ('approach','transport') or phase=='extract' and spec is not None) and b-a>1 and len(pending)+len(accepted)+2<=8:
            middle=(a+b)//2;pending[:0]=[(a,middle,name+'a'),(middle,b,name+'b')]
        else:
            return None,dict(success=False,physical_success=False,pieces=accepted,rejections=rejected,failure='bounded native representation rejected; inspect genuine native gates')
    merged,seams=concatenate_native(pieces);path=output/'native_stage.npz';np.savez_compressed(path,**merged)
    return merged,dict(success=True,physical_success=True,trajectory_file=str(path.resolve()),pieces=accepted,rejections=rejected,
        internal_qva_seams=seams,wall_seconds=time.perf_counter()-started)


def prefix(geometry,geo_report,document,output,commit,progress):
    configure(output);planner,owner=source_planner(document,geo_report);engine=FastThreeLayerPlanner(planner)
    valid=planner.target_validity(planner.checker(planner.selected_task),allow_contact=True)
    keys=('approach','contact','extract','owner_transport');phases=('approach','contact','extract','transport')
    source=dict(geometry_success=True,geometry_origin='fresh in-memory current request',frames=np.concatenate([geometry[k] for k in keys]).tolist(),
        phases=[p for k,p in zip(keys,phases) for _ in geometry[k]],joint_names=NAMES,task=planner.selected_task,source_sha256={},
        precontact_distance_m=owner.precontact_distance(valid.targets[owner.side]['tool_cap_reach_m']))
    initial=time_initial_with_approach_recovery(engine,source,output/'initial',stop_on_failure=True,progress=progress,phase_names=phases[:3])
    for key,stage in zip(keys,initial['stages']):
        if not stage['success']:raise RuntimeError('prefix '+key+' rejected: '+str(stage.get('rejection_reasons')))
        commit(key,load(stage['trajectory_file']),stage)
    if not initial['success']:raise RuntimeError('initial native prefix rejected')
    planner.source_slot_depths=BoxLayoutPlanner.source_slot_depths.__get__(planner)
    loaded=planner.checker(planner.selected_task,payload=True);slots=BoxLayoutPlanner.transport_source_slots(planner,loaded,geometry['owner_transport'][0])
    planner.transport_source_slots=lambda checker,start:slots
    robot=yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf',load_meshes=False)
    observations=cpu_source_observations(robot,geometry['owner_transport'][:1],NAMES,{owner.side:owner});reference={s:o['initial_slot_overlap_m'] for s,o in observations.items()};cleared={s:o['first_clear_frame'] is not None for s,o in observations.items()}
    def validate(data):
        nonlocal cleared
        obs=cpu_source_observations(robot,data['position'],NAMES,{owner.side:owner});good,_,next_clear,checks=source_progress(obs,reference,cleared)
        # Update clear state only when all native gates accept, in bounded_stage's accepted path.
        return dict(valid=good,source_checks=checks)
    data,stage=bounded_stage(engine,source,geometry['owner_transport'],output/'owner_transport','transport',validate=validate,progress=progress)
    if data is None:raise RuntimeError('fresh owner_transport rejected: '+str(stage))
    commit('owner_transport',data,stage)


def transfer(geometry,geo_report,document,output,commit,progress):
    from native_cost import TransferCost,install
    from v3_fast_geometry_cost import GeometricCostManager
    from diagnose_endpoint import components
    from physical_cpu import geometry_audit
    from curobo_core.suction_grasp import contact_cap_spheres
    configure(output);planner,owner=source_planner(document,geo_report);planner.full_payload_trajopt=False
    checker=planner.checker(planner.selected_task,payload=False);engine=FastThreeLayerPlanner(planner)
    roles=HandoverRoles(**geo_report['roles']);giver,recipient=roles.giver,roles.recipient
    attachments={giver:geometry['giver_attachment'],recipient:geometry['recipient_attachment']}
    nominal={giver:geometry['giver_attachment'],recipient:geometry['recipient_nominal_attachment']}
    g=dict(geometry,attachments=attachments,nominal_attachments=nominal,box_dimensions_m=np.asarray(owner.dimensions_m))
    limits=np.asarray(POLICY['arm_continuous_torque_nm']*2)*.995
    kin=checker.kinematics.config.kinematics_config;cap_ids={};manifest=[]
    for side in ('left','right'):
        origin=planner.fk_robot.get_transform(side+'_tool0',side+'_link7')[:3,3]
        cap_ids[side]=[i for i,e in zip(kin.get_sphere_index_from_link_name(side+'_link7').cpu().tolist(),planner.robot['kinematics']['collision_spheres'][side+'_link7']) if e['radius']>0 and e['center'][2]+e['radius']>origin[2]]
    for link in planner.robot['kinematics']['collision_link_names']:
        for i,e in zip(kin.get_sphere_index_from_link_name(link).cpu().tolist(),planner.robot['kinematics']['collision_spheres'][link]):
            side=next((s for s in cap_ids if i in cap_ids[s]),None)
            manifest.append(dict(link=link,center=e['center'],radius=e['radius'],cap_side=side,tool_cap_reach_m=None if side is None else float(np.max(contact_cap_spheres(side)[:,2]+contact_cap_spheres(side)[:,3]))))
    obstacles=[dict(name=o.name,pose=list(o.pose),dims=list(o.dims)) for o in list(checker.guidance_scene.cuboid)+list(checker.base_cuboid_layer.scene.cuboid)]
    robot=yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf',load_meshes=False)
    def inspect(data,side):
        rows=data['position'];details=[]
        for offset in range(0,len(rows),512):details.append(components(planner,checker,rows[offset:offset+512]))
        robot_good=all(c['valid'] for c in details)
        full,C=geometry_audit(robot,rows,side,attachments,nominal,owner.dimensions_m,manifest,obstacles,owner)
        tau,_=engine.dynamics.torque(rows,data['velocity'],data['acceleration'],(side,),attachments)
        ratio=float(np.max(np.abs(tau[:,1:])/limits));delta=float(np.max(np.abs(tau[:,1:]-data['torque'])))
        return dict(valid=bool(robot_good and full['valid'] and ratio<=1 and delta<=.002),robot_components=details,full_single_obb=full,
            independent_rnea=dict(valid=ratio<=1 and delta<=.002,max_scaled_ratio=ratio,max_native_difference_nm=delta))
    original_cost=GeometricCostManager.compute_costs;original_layer=getattr(GeometricCostManager,'transfer_layer',None);install()
    try:
        for key,side,phase in [('receiver_approach',giver,'approach'),('donor_release',recipient,'transport'),('donor_continue',recipient,'return_home')]:
            GeometricCostManager.transfer_layer=TransferCost(checker,g,side,geometry[key],cap_ids,owner)
            validity=GpuValidity(checker,check_ground=True,scene_penetration_tolerance_m=.001)
            controls,start,end,_,traj=engine.optimize_path(dict(task=planner.selected_task,active_payload_sides=[]),phase,geometry[key],validity,checker.guidance_scene)
            data,timing=engine.time_path(controls,start,end,None,(side,),attachments,benchmark_recompute=False)
            stage=dict(success=False,physical_success=False,trajectory_optimization=traj,time_optimization=timing,support_side=side)
            if data is None:raise RuntimeError(key+' TimeOpt rejected '+str(stage))
            path=output/(key+'.npz');path.parent.mkdir(parents=True,exist_ok=True);np.savez_compressed(path,**data);stage['trajectory_file']=str(path.resolve())
            stage['physical_audit']=inspect(data,side);stage['success']=bool(traj['solver_success'] and timing['success'] and stage['physical_audit']['valid']);stage['physical_success']=stage['success']
            if not stage['success']:raise RuntimeError(key+' native rejected '+str(stage))
            if key=='receiver_approach':
                commit(key,data,stage)
                q=geometry['support_transfer'];z=np.zeros_like(q);alpha=geometry['support_alpha'];extremes=[]
                for support in (giver,recipient):
                    tau,_=engine.dynamics.torque(q,z,z,(support,),attachments);extremes.append(tau[:,1:])
                tau=(1-alpha[:,None])*extremes[0]+alpha[:,None]*extremes[1]
                stationary=dict(times=np.linspace(0,1,len(q)),position=q.copy(),velocity=z,acceleration=z.copy(),jerk=z.copy(),torque=tau,support_alpha=alpha.copy())
                support_audit=inspect(dict(stationary,torque=extremes[0]),giver)
                both=geometry_audit(robot,q,recipient,attachments,nominal,owner.dimensions_m,manifest,obstacles,owner)[0]
                good=bool(support_audit['valid'] and both['valid'] and all(n==len(q) for n in both['cap_permitted_frames'].values()) and np.all(np.abs(np.stack(extremes))<=limits))
                path=output/'support_transfer.npz';np.savez_compressed(path,**stationary)
                if not good:raise RuntimeError('stationary support transfer rejected')
                commit('support_transfer',stationary,dict(success=True,physical_success=True,trajectory_file=str(path.resolve()),
                    stationary_support=dict(mass_kg=10.,duration_s=1.,max_scaled_ratio=float(np.max(np.abs(np.stack(extremes))/limits)),dual_contact=True),physical_audit=support_audit))
            else:commit(key,data,stage)
    finally:
        # Transfer uses robot-only sphere indices; restore before loaded suffix TrajOpt.
        GeometricCostManager.compute_costs=original_cost;GeometricCostManager.transfer_layer=original_layer


def suffix(geometry,geo_report,document,output,commit,progress):
    from curobo_core.ordered_pick import ordered_stages
    from v3_ordered_layout import _observations
    configure(output)
    original={b['side']:BoxPick(**b) for b in document['boxes']}
    roles=HandoverRoles(**geo_report['roles']);giver,recipient=roles.giver,roles.recipient
    picks={owner:measured(replace(original[source],side=owner,carry_joint_home=True,carry_offset_m=(0.,0.,0.),carry_updown_m=None),geometry['pending_attachment'] if owner==giver else geometry['recipient_attachment']) for owner,source in ((giver,roles.pending),(recipient,roles.source))}
    for p in picks.values():validate_attachment(p)
    planner=SuffixPlanner(picks,owner_to_physical_id=roles.owner_to_physical_id);planner.full_payload_trajopt=True;engine=FastThreeLayerPlanner(planner);planner.layout_native=(layout_model_key(picks),engine)
    specs=ordered_stages(picks)[3:];reference={recipient:0.};cleared={recipient:True};robot=yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf',load_meshes=False)
    for spec in specs:
        rows=geometry[spec.stage_id];planner.set_stage(spec,rows[0]);stand_off={}
        if spec.incoming_side is not None:
            checker=planner.checker(planner.selected_task,payload=True);cap=planner.target_validity(checker,allow_contact=True)
            stand_off[spec.incoming_side]=picks[spec.incoming_side].precontact_distance(cap.targets[spec.incoming_side]['tool_cap_reach_m'])
        source=dict(geometry_success=True,geometry_origin='fresh in-memory current request',joint_names=NAMES,task=planner.selected_task,source_sha256={},precontact_distance_m=stand_off)
        def inspect(data):
            checker=planner.checker(planner.selected_task,payload=True);values=torch.as_tensor(data['position'],device='cuda',dtype=torch.float32)
            static=planner.static_load_mask(checker,values);margin=planner.native_margin_mask(checker,values);history={};good=bool(static.all() and margin.all())
            for name,cpu in [('gpu',False),('cpu',True)]:
                obs=_observations(planner,spec,data['position'],cpu=cpu)
                if obs:
                    passed,_,_,checks=source_progress(obs,reference,cleared);good&=passed;history[name]=dict(valid=passed,checks=checks)
            return dict(valid=bool(good),source_history=history,static_invalid_count=int((~static).sum()),native_margin_invalid_count=int((~margin).sum()))
        data,stage=bounded_stage(engine,source,rows,output/spec.stage_id,spec.phase,spec=spec,validate=inspect,progress=progress)
        if data is None:raise RuntimeError(spec.stage_id+' suffix rejected '+str(stage))
        commit(spec.stage_id,data,stage)
        if spec.phase=='extract':
            obs=cpu_source_observations(robot,data['position'][-1:],NAMES,{giver:picks[giver]})[giver];reference[giver]=obs['initial_slot_overlap_m'];cleared[giver]=obs['first_clear_frame'] is not None
