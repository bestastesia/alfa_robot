"""gpuq only. Construct all13 geometric stages from a size/center request, never cached solutions."""
from dataclasses import asdict,replace
from pathlib import Path
import argparse
import hashlib
import importlib.metadata
import json
import sys
import time
import traceback
from types import SimpleNamespace
import numpy as np

ROOT=Path(__file__).resolve().parents[4]
HERE=Path(__file__).resolve().parent
OUT=ROOT/'generated/two_box_layout/joint_contract_20261124_fresh'
REQUEST=ROOT/'generated/two_box_layout/joint_contract_20261123/broader_same_side_y85_request.json'
sys.path[:0]=[str(ROOT/'tools'),str(HERE.parent/'transfer_suffix'),str(HERE.parent/'transfer_native')]
NAMES=['updown']+[f'{s}_joint{i}' for s in ('left','right') for i in range(1,8)]
PHASES=('approach','contact','extract','owner_transport','receiver_approach','support_transfer','donor_release','donor_continue','upper_park','lower_approach','lower_contact','lower_extract','final_transport')
HANDOVER_CENTERS=((.85,.2,1.2),(.90,.15,1.2),(.95,.10,1.25))

def save(path,value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')

def source_hashes():
    paths=list((ROOT/'tools').rglob('*.py'))+list((ROOT/'config').rglob('*.json'))
    paths+=list((ROOT/'generated/fast_runtime_assets').glob('*.yaml'))+list((ROOT/'generated/fast_runtime_assets').glob('*.urdf'))
    return {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}

def guard_old_solutions(request_path,report):
    """Fail closed on any historical layout JSON/NPZ reads after request parsing."""
    allowed={request_path.resolve()};history=ROOT/'generated/two_box_layout'
    reads=set()
    def audit(event,args):
        if event!='open' or not isinstance(args[0],(str,bytes)):return
        path=Path(args[0]).resolve()
        if path.suffix not in ('.json','.npz','.npy') or not path.is_relative_to(history):return
        if path.is_relative_to(OUT):return
        if path not in allowed:raise PermissionError('fresh construction cannot read historical solution '+str(path))
        reads.add(str(path.relative_to(ROOT)))
    sys.addaudithook(audit)
    report['historical_solution_read_guard']='Enforced: historical layout JSON/NPZ/NPY denied; only original request and this new output tree allowed.'
    return reads

def dense(rows,step=.001):
    # ponytail: <=1mrad sampled checks; upgrade to swept CCD for a continuous certificate.
    rows=np.asarray(rows,dtype=np.float32);result=[rows[0]]
    for a,b in zip(rows[:-1],rows[1:]):
        count=max(1,int(np.ceil(np.max(np.abs(b-a))/step)))
        result.extend(a+(b-a)*t for t in np.linspace(0,1,count+1)[1:-1]);result.append(b)
    return np.asarray(result,dtype=np.float32)

def donor_retreat(robot,start,side="left"):
    """Warm-start all seven bounded donor joints; preserve lift, recipient and actual pose gates."""
    from curobo_core.handover_roles import arm_slice
    columns=arm_slice(side)
    from physical_cpu import forward
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation
    start=np.asarray(start,dtype=np.float64)
    target=forward(robot,start[None])[0][side+'_tool0'][0]
    bounds=np.asarray([[robot.joint_map[n].limit.lower,robot.joint_map[n].limit.upper] for n in NAMES[columns]]).T
    # Reserve1mrad at donor joint bounds before fitting a native cubic; never clip sampled q.
    bounds[0]+=.001;bounds[1]-=.001
    if np.any(start[columns]<bounds[0]) or np.any(start[columns]>bounds[1]):
        return None,dict(success=False,failure='donor start lacks1mrad native spline joint margin',steps=[])
    rows=[start.copy()];steps=[]
    for distance in np.linspace(.002,.07,35):
        desired=target.copy();desired[:3,3]-=distance*target[:3,2];row=rows[-1].copy()
        def residual(values):
            row[columns]=values;T=forward(robot,row[None])[0][side+'_tool0'][0]
            return np.r_[T[:3,3]-desired[:3,3],Rotation.from_matrix(desired[:3,:3].T@T[:3,:3]).as_rotvec()]
        try:
            fit=least_squares(residual,rows[-1][columns],bounds=bounds,ftol=1e-12,xtol=1e-12,gtol=1e-12,max_nfev=60,tr_solver='lsmr')
        except ValueError as error:
            return None,dict(success=False,failed_distance_m=float(distance),failure=str(error),steps=steps)
        error=residual(fit.x)
        step=dict(distance_m=float(distance),position_error_m=float(np.linalg.norm(error[:3])),angle_error_rad=float(np.linalg.norm(error[3:])),nfev=fit.nfev)
        steps.append(step)
        if step['position_error_m']>1e-6 or step['angle_error_rad']>1e-5:
            return None,dict(success=False,failed_distance_m=float(distance),failure='bounded seven-joint pose residual rejected',steps=steps)
        rows.append(row.copy())
    return np.asarray(rows,dtype=np.float32),dict(success=True,steps=steps)

def plan_document(document,output,report):
    """Input contains physical request only. Other layouts remain unproven, with real failures preserved."""
    import torch
    from curobo.types import JointState
    from curobo_core.box_pick import BoxPick,diverse_contact_order,source_slot_observation,source_slot_process_checks
    from curobo_core.scene import SceneObject,AttachedObject,Pose
    from curobo_core.adapter import matrix_pose,pose_matrix
    from curobo_core.torque import TorqueModel
    from curobo_core.native_transport import urdf_box_poses
    from curobo_core.suction_grasp import contact_cap_spheres
    import v3_two_box_pick
    from v3_single_arm_planner import SingleArmPlanner
    from v3_box_layout_planner import BoxLayoutPlanner
    from v3_batched_loaded_search import GpuValidity
    from v3_multiroot_search import batched_rrt_connect_multi_goal
    from v3_lm_conveyor import box_poses
    from v3_box_layout import parse_layout
    import probe_suffix as suffix
    from physical_cpu import geometry_audit,forward
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation
    import yourdfpy
    allowed={'frame_id','front_distance_m','boxes','world_from_robot_base'}
    if set(document)-allowed:raise ValueError('request accepts physical layout only, no flow/IK hints')
    for row in document.get('boxes',[]):
        if set(row)-{'size','side','center_m','side_min_height_m'}:raise ValueError('box accepts size/center/identity/height threshold only')
    from curobo_core.handover_roles import select_roles,arm_slice,PHYSICAL_IDS
    originals=parse_layout(document);roles=select_roles(originals)
    giver,recipient=roles.giver,roles.recipient;donor_columns=arm_slice(giver)
    owner=replace(originals[roles.source],side=giver,carry_joint_home=False,carry_offset_m=None)
    pending=originals[roles.pending];source_id=PHYSICAL_IDS[roles.source];pending_id=PHYSICAL_IDS[roles.pending]
    v3_two_box_pick.prepare_fast_assets=suffix.prepared_assets
    class SourcePlanner(v3_two_box_pick.TwoBoxPickPlanner):
        def configure_pick(self,pick):
            super().configure_pick(pick);self.selected_task={giver:int(source_id[-2:])}
            self.snapshot=replace(self.snapshot,objects=self.environment+(
                SceneObject(source_id,pick.dimensions_m,Pose(pick.center_m)),SceneObject(pending_id,pending.dimensions_m,Pose(pending.center_m))))
        def task_attachments(self,boxes):
            return (AttachedObject(source_id,self.pick.dimensions_m,giver+'_tool0',matrix_pose(self.pick.tool_to_box()),(giver+'_link7',giver+'_tool0')),)
    planner=SourcePlanner();planner.configure_pick(owner);planner.native_project_margins=True
    empty=planner.checker(planner.selected_task,payload=False);loaded=planner.checker(planner.selected_task,payload=True)
    assert {o.name for o in empty.guidance_scene.cuboid}&{source_id,pending_id}=={pending_id}
    policy=json.loads((ROOT/'config/fast_three_layers.json').read_text());limits=np.asarray(policy['arm_continuous_torque_nm']*2)*.995
    assert policy['payload_kg_per_box']==10. and policy['planning_limit_scale']==.995
    dynamics=TorqueModel(ROOT/'generated/fast_runtime_assets/dynamics.yaml',payload_mass_kg=10.,payload_dimensions=owner.dimensions_m)
    A_giver=owner.tool_to_box();robot=yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf',load_meshes=False)
    kin=empty.kinematics.config.kinematics_config;manifest=[];cap_ids={}
    for side in ('left','right'):
        origin=planner.fk_robot.get_transform(side+'_tool0',side+'_link7')[:3,3]
        ids=kin.get_sphere_index_from_link_name(side+'_link7').cpu().tolist();entries=planner.robot['kinematics']['collision_spheres'][side+'_link7']
        cap_ids[side]=[i for i,e in zip(ids,entries) if e['radius']>0 and e['center'][2]+e['radius']>origin[2]]
        assert len(cap_ids[side])==len(contact_cap_spheres(side))
    for link in planner.robot['kinematics']['collision_link_names']:
        for i,e in zip(kin.get_sphere_index_from_link_name(link).cpu().tolist(),planner.robot['kinematics']['collision_spheres'][link]):
            side=next((s for s in cap_ids if i in cap_ids[s]),None)
            manifest.append(dict(link=link,center=e['center'],radius=e['radius'],cap_side=side,tool_cap_reach_m=None if side is None else float(np.max(contact_cap_spheres(side)[:,2]+contact_cap_spheres(side)[:,3]))))
    obstacles=[dict(name=o.name,pose=list(o.pose),dims=list(o.dims)) for o in list(empty.guidance_scene.cuboid)+list(empty.base_cuboid_layer.scene.cuboid)]
    validity=GpuValidity(empty,check_ground=True,scene_penetration_tolerance_m=.001)
    def torque(rows,side,A):
        q=torch.as_tensor(rows,device='cuda',dtype=torch.float32).contiguous();z=torch.zeros_like(q);model=dynamics._model((side,),{side:A});model.setup_batch_size(len(q))
        return model.compute_inverse_dynamics(JointState(position=q,velocity=z,acceleration=z))[:,1:].detach().cpu().numpy().copy()
    arrays={};report.update(request=document,phase_audits={},prefix_trials=[],handover_candidates=[],joint_names=NAMES)
    def publish():save(output/'fresh_report.json',report)
    def transfer_check(rows,side,A_recipient,nominal_recipient,*,both=False):
        mask=[]
        for q in torch.as_tensor(rows,device='cuda',dtype=torch.float32).split(512):mask.extend((validity.mask(q)&planner.native_margin_mask(empty,q)).cpu().tolist())
        check,C=geometry_audit(robot,rows,side,{giver:A_giver,recipient:A_recipient},{giver:A_giver,recipient:nominal_recipient},owner.dimensions_m,manifest,obstacles,owner)
        taus=torque(rows,side,A_giver if side==giver else A_recipient)
        check.update(robot_valid=bool(all(mask)),max_scaled_gravity_ratio=float(np.max(np.abs(taus)/limits)))
        check['valid']=bool(check['valid'] and check['robot_valid'] and np.all(np.abs(taus)<=limits))
        if both:
            other=torque(rows,recipient,A_recipient);check['recipient_support_ratio']=float(np.max(np.abs(other)/limits));check['valid'] &= bool(np.all(np.abs(other)<=limits) and all(v==len(rows) for v in check['cap_permitted_frames'].values()))
        return check,C,taus
    # Every lift, swivel and pair comes from current analytic IK and current URDF.
    candidates=[];retreat_cache={}
    report['retreat_admission']=dict(unique_kinematic_queries=0,kinematic_rejections=0,physical_rejections=0,accepted=0,first_rejections=[])
    centers=tuple((x,-y if giver=='right' else y,z) for x,y,z in HANDOVER_CENTERS)
    for center in centers:
        donor=BoxPick(owner.size,center,giver,side_min_height_m=owner.side_min_height_m)
        receiver=BoxPick(owner.size,center,recipient,side_min_height_m=owner.side_min_height_m,side_face='y-' if recipient=='right' else 'y+',suction_roll_deg=90.)
        C_mid=np.eye(4);C_mid[:3,3]=center
        nominal_recipient=receiver.tool_to_box();targets={giver:C_mid@np.linalg.inv(A_giver),recipient:receiver.contact_tool()};accepted=[];raw_count=0
        for lift in np.linspace(-1.,0.,21):
            row=planner.home_values.copy();row[0]=lift;planner.fk(row);carriage=np.linalg.inv(planner.fk_robot.get_transform('arm_carriage',planner.fk_robot.base_link));arms=[]
            for arm,side in enumerate(('left','right')):
                pieces=[planner.analytic.solve(arm,carriage@targets[side],np.deg2rad(angle),row[1+7*arm:8+7*arm]) for angle in range(-180,180,10)]
                values=np.concatenate(pieces);values=np.unique(np.round(values,7),axis=0)
                if not len(values):break
                arms.append(values[np.argsort(np.linalg.norm(values-row[1+7*arm:8+7*arm],axis=1))][:32])
            if len(arms)!=2:continue
            left,right=arms;rows=np.repeat(row[None],len(left)*len(right),axis=0);rows[:,1:8]=np.repeat(left,len(right),axis=0);rows[:,8:]=np.tile(right,(len(left),1));rows=rows.astype(np.float32);raw_count+=len(rows)
            for chunk in np.array_split(rows,max(1,int(np.ceil(len(rows)/512)))):
                q=torch.as_tensor(chunk,device='cuda');mask=(validity.mask(q)&planner.native_margin_mask(empty,q)).cpu().numpy()
                ids=np.flatnonzero(mask)
                if len(ids):
                    mask[ids]&=(np.abs(torque(chunk[ids],giver,A_giver))<=limits).all(-1)&(np.abs(torque(chunk[ids],recipient,nominal_recipient))<=limits).all(-1)
                for row2 in chunk[mask]:
                    poses=forward(robot,row2[None])[0];C=poses[giver+'_tool0'][0]@A_giver
                    A_recipient=np.linalg.inv(poses[recipient+'_tool0'][0])@C
                    # Both nominal tool contacts must still agree with this measured physical box.
                    other=poses[recipient+'_tool0'][0]@nominal_recipient
                    if np.linalg.norm(C[:3,3]-other[:3,3])>.002 or Rotation.from_matrix(C[:3,:3].T@other[:3,:3]).magnitude()>np.deg2rad(1):continue
                    check,_,_=transfer_check(row2[None],giver,A_recipient,nominal_recipient,both=True)
                    if not check['valid']:continue
                    previous=planner.selected_task;planner.selected_task={recipient:int(source_id[-2:])}
                    try:reverse,detail=SingleArmPlanner.cartesian_single(planner,row2,np.linspace(.005,.05,10),along_normal=True)
                    finally:planner.selected_task=previous
                    if reverse is None:continue
                    contact=dense(reverse[::-1]);check,_,_=transfer_check(contact,giver,A_recipient,nominal_recipient)
                    if check['valid']:
                        # Reuse only this call's donor IK; recipient posture still gets its full physical gate.
                        cache_key=np.r_[row2[0],row2[donor_columns]].tobytes()
                        if cache_key not in retreat_cache:
                            path,ik=donor_retreat(robot,row2,giver)
                            retreat_cache[cache_key]=(None if path is None else path[:,donor_columns].copy(),ik)
                            report['retreat_admission']['unique_kinematic_queries']+=1
                        donor_path,ik=retreat_cache[cache_key]
                        if donor_path is None:
                            report['retreat_admission']['kinematic_rejections']+=1
                            if len(report['retreat_admission']['first_rejections'])<8:
                                report['retreat_admission']['first_rejections'].append(dict(updown_m=float(row2[0]),ik=ik))
                            continue
                        path=np.repeat(row2[None],len(donor_path),axis=0);path[:,donor_columns]=donor_path
                        release,continued=dense(path[:31]),dense(path[30:])
                        release_check,_,_=transfer_check(release,recipient,A_recipient,nominal_recipient)
                        continue_check,_,_=transfer_check(continued,recipient,A_recipient,nominal_recipient)
                        if not release_check['valid'] or not continue_check['valid']:
                            report['retreat_admission']['physical_rejections']+=1
                            if len(report['retreat_admission']['first_rejections'])<8:
                                report['retreat_admission']['first_rejections'].append(dict(updown_m=float(row2[0]),release=release_check,continued=continue_check))
                            continue
                        report['retreat_admission']['accepted']+=1
                        accepted.append(dict(q=row2.copy(),contact=contact,A_recipient=A_recipient,nominal_recipient=nominal_recipient,center=center,
                            donor_release=release,donor_continue=continued,retreat_ik=ik))
                    if len(accepted)>=8:break
                if len(accepted)>=8:break
            report['retreat_admission']['last_lift_m']=float(lift);publish()
            print('RETREAT_ADMISSION',float(lift),report['retreat_admission']['unique_kinematic_queries'],len(accepted),flush=True)
            if len(accepted)>=8:break
        report['handover_candidates'].append(dict(center_m=center,fresh_raw_ik_pairs=raw_count,accepted=len(accepted)));publish()
        candidates.extend(accepted)
        if candidates:break
    if not candidates:raise RuntimeError('fresh_dual_contact_pool_empty; finite three-center/21-lift/10deg-swivel search only')
    goals=np.asarray([v['contact'][0] for v in candidates],dtype=np.float32)
    contacts=planner.enumerate_single(owner.contact_tool(),planner.home_values,empty,limit=256)
    if len(contacts):contacts=contacts[diverse_contact_order(contacts,giver)]
    valid=planner.target_validity(empty,allow_contact=True);valid.scene_penetration_tolerance_m=.001;valid.contact_penetration_tolerance_m=.001
    stand_off=owner.precontact_distance(valid.targets[giver]['tool_cap_reach_m']);selected=None
    for i,q in enumerate(contacts[:16]):
        trial=dict(index=i);report['prefix_trials'].append(trial)
        reverse,detail=SingleArmPlanner.cartesian_single(planner,q,np.linspace(.005,stand_off,int(np.ceil(stand_off/.005))),along_normal=True)
        trial['precontact']=reverse is not None
        if reverse is None:continue
        contact=np.asarray(reverse[::-1],dtype=np.float32)
        if not bool(valid.mask(torch.as_tensor(contact,device='cuda')).all()):trial['failure']='contact_collision';continue
        result=planner.extract(q,loaded);trial['extraction_success']=result['success']
        if not result['success']:continue
        extraction=np.asarray(result['frames'],dtype=np.float32);audit=planner.loaded_audit(loaded,extraction);trial['extraction_audit']=audit
        if not audit['valid']:continue
        approach,search=planner.single_path(planner.home_values,contact[:1],empty,strict=planner.target_validity(empty),budget=8.);trial['home_approach_exists']=approach is not None
        if approach is None:continue
        planner.picks={giver:owner};free=planner.single_validity(loaded,extraction[-1],goals[0]);free.scene_penetration_tolerance_m=.001;ordinary=free.evaluate
        initial=BoxLayoutPlanner.source_slot_depths(planner,loaded,torch.as_tensor(extraction[-1:],device='cuda'));limit=initial.clamp(min=0)+1e-6
        def evaluate(values,base_margin_m=0.):
            mask,stable=ordinary(values,base_margin_m);indices=mask.nonzero().reshape(-1)
            if len(indices):mask[indices]&=torch.as_tensor((np.abs(torque(values[indices].cpu().numpy(),giver,A_giver))<=limits).all(-1),device='cuda')
            mask&=(BoxLayoutPlanner.source_slot_depths(planner,loaded,values)<=limit).all(-1)
            return mask,stable
        free.evaluate=evaluate;free.steering_steps=(.28,.03,.01,.003);free.validate_returned_path=True
        candidate_mask=free.mask(torch.as_tensor(goals,device='cuda')).cpu().numpy()
        for j in np.flatnonzero(candidate_mask):candidate_mask[j]=planner.loaded_audit(loaded,goals[j:j+1])['valid']
        ids=np.flatnonzero(candidate_mask)
        if not len(ids):trial['failure']='standoff_loaded_or_static_gate';publish();continue
        planner.run_single_search=batched_rrt_connect_multi_goal
        transport,search=planner.single_path(extraction[-1],goals[ids],loaded,strict=free,budget=32.);trial['transport_search']=search;publish()
        if transport is None:continue
        audit=planner.loaded_audit(loaded,transport);process=source_slot_process_checks({giver:source_slot_observation(owner,box_poses(loaded,transport,{giver:A_giver})[giver])})
        if not audit['valid'] or not process['valid'] or not bool(free.mask(torch.as_tensor(transport,device='cuda',dtype=torch.float32)).all()):continue
        j=int(np.argmin(np.max(np.abs(goals-transport[-1]),axis=1)));assert np.max(np.abs(goals[j]-transport[-1]))<=1e-5
        selected=candidates[j];arrays.update(approach=np.asarray(approach,dtype=np.float32),contact=contact,extract=extraction,owner_transport=np.asarray(transport,dtype=np.float32));break
    if selected is None:raise RuntimeError('fresh_prefix_failed; inspect bounded16 contact trials, no cached path fallback')
    A_recipient=selected['A_recipient'];nominal_recipient=selected['nominal_recipient'];arrays['receiver_approach']=selected['contact'];switch=arrays['receiver_approach'][-1]
    prefix_rows=np.concatenate([arrays['extract'],arrays['owner_transport']])
    check,C,tau=transfer_check(prefix_rows,giver,A_recipient,nominal_recipient)
    assert check['valid'],dict(phase='extract_and_transport',audit=check)
    report['phase_audits']['extract_and_transport']=check
    split=len(arrays['extract'])
    arrays['extract_torques_nm']=tau[:split];arrays['owner_transport_torques_nm']=tau[split:]
    arrays['support_transfer']=np.repeat(switch[None],11,axis=0);arrays['support_alpha']=np.linspace(0,1,11)
    arrays['donor_release']=selected['donor_release'];arrays['donor_continue']=selected['donor_continue']
    report['donor_retreat_ik']=selected['retreat_ik']
    assert np.array_equal(arrays['donor_release'][0],switch)
    for key,side in [('receiver_approach',giver),('support_transfer',giver),('donor_release',recipient),('donor_continue',recipient)]:
        check,C,tau=transfer_check(arrays[key],side,A_recipient,nominal_recipient,both=key=='support_transfer')
        assert check['valid'],dict(phase=key,audit=check)
        arrays[key+'_high_box']=C
        if key=='support_transfer':
            recipient_tau=torque(arrays[key],recipient,A_recipient);alpha=arrays['support_alpha']
            arrays['support_extreme_torques_nm']=np.stack((tau[0],recipient_tau[0]));tau=(1-alpha[:,None])*tau+alpha[:,None]*recipient_tau
            check['max_scaled_support_ramp_ratio']=float(np.max(np.abs(tau)/limits))
        arrays[key+'_torques_nm']=tau;report['phase_audits'][key]=check
        print('FRESH_TRANSFER',key,len(arrays[key]),flush=True)
    arrays['giver_attachment']=A_giver;arrays['recipient_attachment']=A_recipient;arrays['recipient_nominal_attachment']=nominal_recipient
    if giver=='left' and roles.source=='right':
        arrays.update(left_donor_attachment=A_giver,right_attachment=A_recipient,right_nominal_attachment=nominal_recipient)
    # Reuse the existing suffix solver with an in-memory interface from THIS run.
    suffix.OUT=output
    suffix_picks={giver:replace(pending,side=giver),recipient:replace(originals[roles.source],side=recipient)}
    suffix.inputs=lambda:({},suffix_picks,arrays['donor_continue'][-1].copy(),A_recipient.copy())
    suffix.hashes=lambda:dict(request_sha256=hashlib.sha256(json.dumps(document,sort_keys=True).encode()).hexdigest())
    np.savez_compressed(output/'fresh_partial_geometry.npz',**arrays)
    suffix_report=dict(success=False);suffix_arrays={};suffix.run(suffix_report,suffix_arrays,roles=roles)
    save(output/'suffix_fresh_report.json',suffix_report)
    if not suffix_report['success']:raise RuntimeError('fresh_suffix_failed '+str(suffix_report.get('failure')))
    for key in PHASES[8:]:
        arrays[key]=suffix_arrays[key];arrays[key+'_torques_nm']=suffix_arrays[key+'_torques_nm'];report['phase_audits'][key]=suffix_report['phase_audits'][key]
    arrays['pending_attachment']=suffix_arrays['pending_attachment']
    if giver=='left' and roles.source=='right':arrays['left_final_attachment']=arrays['pending_attachment']
    # Every phase exposes real existing SceneSnapshot/AttachedObject records with stable IDs.
    stages=[];previous=None
    for index,key in enumerate(PHASES):
        rows=arrays[key];assert rows.ndim==2 and rows.shape[1]==15 and np.isfinite(rows).all()
        if previous is not None:assert np.max(np.abs(rows[0]-previous))<=1e-5,(key,np.max(np.abs(rows[0]-previous)))
        previous=rows[-1]
        high_parent=None if index<2 else giver if index<=5 else recipient
        low_parent=giver if index>=11 else None
        attached={};world=[]
        for physical,side,parent,A in [(source_id,roles.source,high_parent,A_giver if high_parent==giver else A_recipient),(pending_id,roles.pending,low_parent,arrays['pending_attachment'])]:
            pick=originals[side]
            if parent is None:world.append(SceneObject(physical,pick.dimensions_m,Pose(pick.center_m)))
            else:attached[physical]=AttachedObject(physical,pick.dimensions_m,parent+'_tool0',matrix_pose(A),(parent+'_link7',parent+'_tool0'))
        snapshot=replace(planner.snapshot,objects=planner.environment+tuple(world),attachments=tuple(attached.values()),
            state=replace(planner.snapshot.state,positions=tuple(dict(zip(NAMES,rows[0])).get(n,v) for n,v in zip(planner.snapshot.state.joint_names,planner.snapshot.state.positions))))
        poses=forward(robot,rows)[0]
        for physical,side,parent,A in [(source_id,roles.source,high_parent,A_giver if high_parent==giver else A_recipient),(pending_id,roles.pending,low_parent,arrays['pending_attachment'])]:
            C=np.repeat(np.eye(4)[None],len(rows),axis=0);C[:,:3,3]=originals[side].center_m
            if parent is not None:C=poses[parent+'_tool0']@A
            arrays[key+'_'+side+'_box']=C
        native_phase=('contact' if 'contact' in key or key=='receiver_approach' else 'extract' if 'extract' in key or key in ('donor_release','donor_continue') else 'approach' if key=='approach' else 'transport')
        task={a.parent_link.removesuffix('_tool0'):int(p[-2:]) for p,a in attached.items()}
        if index<2:task={giver:int(source_id[-2:])}
        if index in (9,10):task={giver:int(pending_id[-2:]),**task}
        stages.append(dict(stage_id=key,native_phase=native_phase,array_key=key,samples=len(rows),scene=snapshot.to_dict(),
            native_time_optimization_required=key!='support_transfer',active_payload_sides=[a.parent_link.removesuffix('_tool0') for a in attached.values()],
            tool_to_box={a.parent_link.removesuffix('_tool0'):pose_matrix(a.tool_to_object).tolist() for a in attached.values()},
            payload_dimensions_m={a.parent_link.removesuffix('_tool0'):list(a.dimensions_m) for a in attached.values()},
            payload_mass_kg=10.*len(attached),object_to_parent={p:a.parent_link for p,a in attached.items()},
            task=task,
            tool_to_object={p:asdict(a.tool_to_object) for p,a in attached.items()},
            rest_boundaries_required=True,stationary_support_transfer=key=='support_transfer'))
    report.update(success=True,fresh_geometry_success=True,stages=stages,selected_handover_center_m=selected['center'],
        donor_task=asdict(owner),roles=asdict(roles),original_tasks={s:asdict(p) for s,p in originals.items()},
        stage_samples={k:len(arrays[k]) for k in PHASES},physical_ids=['wall_box_00','wall_box_01'],
        source_loaded_stage_details=dict(extract=report['prefix_trials'][-1]['extraction_audit']),
        scope='One fresh saved-request geometry run; no native timing/dynamics acceptance and no universal input claim.')
    if giver=='left':report['left_donor_task']=report['donor_task']
    save(output/'stage_interface.json',dict(request=document,joint_names=NAMES,geometry='fresh_geometry.npz',stages=stages,
        support_transfer=dict(physical_id=source_id,mass_kg=10.,parent_before=giver+'_tool0',parent_after=recipient+'_tool0',alpha_key='support_alpha',stationary_only=True),
        roles=asdict(roles),original_high_source_slot=originals[roles.source].center_m,original_low_source_slot=originals[roles.pending].center_m))
    return arrays

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--request',type=Path,default=REQUEST);parser.add_argument('--attempt',default='attempt_01');args=parser.parse_args()
    assert Path(args.attempt).name==args.attempt
    output=OUT/args.attempt;output.mkdir(parents=True,exist_ok=False)
    began=time.perf_counter();report=dict(success=False,estimate='3m provisional; measured prefix23.7s+transfer4.26s+suffix35.95s plus integration and initialization margin')
    arrays={}
    try:
        document=json.loads(args.request.read_text());report['request_file']=str(args.request.resolve());report['input_request_sha256']=hashlib.sha256(args.request.read_bytes()).hexdigest()
        before=source_hashes();reads=guard_old_solutions(args.request,report)
        import curobo,torch
        report['runtime']=dict(executable=sys.executable,curobo_path=curobo.__file__,curobo_version=importlib.metadata.version('nvidia-curobo'),torch_version=torch.__version__)
        arrays=plan_document(document,output,report)
        after=source_hashes()
        loaded_files={str(Path(module.__file__).resolve().relative_to(ROOT)) for module in list(sys.modules.values()) if getattr(module,'__file__',None) and Path(module.__file__).resolve().is_relative_to(ROOT)}
        tracked={p for p in before if not p.endswith('.py') or p in loaded_files}
        before_used={p:before[p] for p in sorted(tracked)};after_used={p:after.get(p) for p in sorted(tracked)}
        report.update(source_sha256_before=before_used,source_sha256_after=after_used,source_files_unchanged=before_used==after_used,historical_layout_data_reads=sorted(reads),
            unrelated_source_changes=[p for p in before if p not in tracked and before[p]!=after.get(p)],
            source_hash_scope='Actual loaded Python modules plus configuration/model inputs; unrelated concurrent experiment scripts recorded separately.')
        assert before_used==after_used,'loaded sources/config/model changed during construction'
    except Exception as error:
        report.update(success=False,failure=repr(error),traceback=traceback.format_exc());traceback.print_exc()
    finally:
        report['wall_seconds']=time.perf_counter()-began
        if arrays:np.savez_compressed(output/'fresh_geometry.npz',**arrays)
        save(output/'fresh_report.json',report);print('FRESH_RESULT',report['success'],'wall',report['wall_seconds'],flush=True)
    sys.exit(0 if report['success'] else 1)
