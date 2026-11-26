"""gpuq only: bounded ordered geometry suffix from the accepted measured handover."""
from dataclasses import dataclass, replace
from pathlib import Path
import hashlib
import importlib.metadata
import json
import sys
import time
import traceback
from types import SimpleNamespace

import curobo
import numpy as np
import torch
from curobo.types import JointState
from curobo._src.geom.transform import torch_quaternion_to_matrix

from audit_suffix import ROOT, TRANSFER, OUT, NAMES, hashes, inputs, fk
from curobo_core.box_pick import BoxPick, layout_model_key, source_slot_observation, source_slot_process_checks, diverse_contact_order
from curobo_core.contracts import PlannerAssets
from curobo_core.layout_contract import carry_configuration
from curobo_core.ordered_pick import ordered_stages
from curobo_core.suction_grasp import contact_cap_spheres
from curobo_core.torque import TorqueModel
import v3_two_box_pick
from v3_ordered_layout import OrderedLayoutPlanner, _geometry_validity, _geometry_free_path, _joint_stage_goals
from v3_batched_loaded_search import densify
from v3_lm_conveyor import SequentialPlacementLm, box_poses
from v3_box_layout import atomic_json, session_source_stamp


@dataclass(frozen=True)
class MeasuredPick(BoxPick):
    measured_attachment: tuple = ()
    def tool_to_box(self):
        return np.asarray(self.measured_attachment) if self.measured_attachment else super().tool_to_box()


def prepared_assets():
    folder=ROOT/'generated/fast_runtime_assets'
    return replace(PlannerAssets(),robot_config=folder/'fixed.yaml',mobile_robot_config=folder/'mobile.yaml',
        urdf=folder/'fixed.urdf',named_poses=ROOT/'generated/zero_model_candidate_20261018/named_poses.yaml'),folder/'dynamics.yaml'


def measured(pick,A):
    values={k:getattr(pick,k) for k in BoxPick.__dataclass_fields__}
    return MeasuredPick(**values,measured_attachment=tuple(tuple(v) for v in A))


def validate_attachment(pick):
    T=np.linalg.inv(pick.tool_to_box());caps=contact_cap_spheres(pick.side)
    centers=caps[:,:3]@T[:3,:3].T+T[:3,3];half=np.asarray(pick.dimensions_m)/2
    normal=T[:3,2];axis=int(np.argmax(np.abs(normal)));tangent=[i for i in range(3) if i!=axis]
    excess=float(np.max(np.abs(centers[:,tangent])+caps[:,3,None]-half[tangent]))
    plane_error=float(abs(T[axis,3]+np.sign(normal[axis])*half[axis]))
    assert excess<=.001 and plane_error<=.001 and abs(normal[axis])>=np.cos(np.deg2rad(1.))
    return dict(cap_count=len(caps),footprint_excess_m=excess,plane_error_m=plane_error)


class SuffixPlanner(OrderedLayoutPlanner):
    def strict_box_mask(self,checker,values):
        """All original robot spheres vs OBB; only rigid, plane-validated own terminal caps omitted."""
        config=checker.kinematics.config.kinematics_config
        payload_ids=set();caps={}
        for side in checker.task_tool_to_box:
            ids=config.get_sphere_index_from_link_name(side+'_link7').cpu().tolist()
            original=self.robot['kinematics']['collision_spheres'][side+'_link7']
            payload_ids.update(ids[len(original):])
            origin=self.fk_robot.get_transform(side+'_tool0',side+'_link7')[2,3]
            caps[side]=[i for i,s in zip(ids,original) if s['radius']>0 and s['center'][2]+s['radius']>origin]
        masks=[]
        for q in values.detach().split(512):
            state=checker.kinematics.compute_kinematics(JointState.from_position(q[None].contiguous(),joint_names=NAMES))
            spheres=state.robot_spheres.reshape(len(q),-1,4);ok=torch.ones(len(q),device=q.device,dtype=torch.bool)
            for side,A in checker.task_tool_to_box.items():
                pose=state.tool_poses.to_dict()[side+'_tool0'];R=torch_quaternion_to_matrix(pose.quaternion.reshape(-1,4).double())
                attachment=torch.as_tensor(A,device=q.device,dtype=torch.float64)
                center=pose.position.reshape(-1,3).double()+(R@attachment[:3,3,None])[...,0];R=R@attachment[:3,:3]
                ids=[i for i in range(spheres.shape[1]) if i not in payload_ids and i not in caps[side]]
                local=(spheres[:,ids,:3].double()-center[:,None])@R
                delta=local.abs()-torch.as_tensor(self.picks[side].dimensions_m,device=q.device)/2
                distance=delta.clamp(min=0).norm(dim=-1)+delta.amax(-1).clamp(max=0)
                depth=spheres[:,ids,3].double()-distance
                ok&=~((depth>1e-6)&(spheres[:,ids,3]>0)).any(-1)
            masks.append(ok)
        return torch.cat(masks)

    def payload_search_mask(self,checker,values,**kwargs):
        return super().payload_search_mask(checker,values,**kwargs)&self.strict_box_mask(checker,values)

    def loaded_audit(self,checker,rows):
        audit=super().loaded_audit(checker,rows)
        values=torch.as_tensor(rows,device='cuda',dtype=torch.float32)
        invalid=np.flatnonzero(~self.strict_box_mask(checker,values).cpu().numpy()).tolist()
        audit['strict_noncap_obb_invalid_indices']=invalid
        audit['invalid_frame_indices']=sorted(set(audit['invalid_frame_indices'])|set(invalid))
        audit['valid']=not audit['invalid_frame_indices']
        audit['terminal_cap_policy']='Only plane/whole-footprint-validated own caps; every original noncap sphere checked, including own link7.'
        return audit


def run(report,arrays,*,roles=None):
    from curobo_core.handover_roles import HandoverRoles, admitted_precontacts
    roles=roles or HandoverRoles();giver,recipient=roles.giver,roles.recipient
    began=time.perf_counter();deadline=began+110.
    interface,picks,start,A_right=inputs();report['input_hashes_before']=hashes()
    report['source_stamp_at_start']=session_source_stamp()
    arrays.update(start=start,recipient_attachment=A_right,physical_box_ids=np.array(['wall_box_00','wall_box_01']))
    picks={side:replace(p,carry_joint_home=True,carry_offset_m=(0.,0.,0.),carry_updown_m=None) for side,p in picks.items()}
    picks[recipient]=measured(picks[recipient],A_right)
    report['attachment_checks']={s:validate_attachment(p) for s,p in picks.items()}
    report['request']=dict(front_distance_m=.66,side_min_height_m=.79,boxes=[dict(side=s,size=p.size,center_m=p.center_m,dimensions_m=p.dimensions_m) for s,p in picks.items()])
    v3_two_box_pick.prepare_fast_assets=prepared_assets
    planner=SuffixPlanner(picks,owner_to_physical_id=roles.owner_to_physical_id)
    policy=json.loads((ROOT/'config/fast_three_layers.json').read_text())
    assert policy['planning_limit_scale']==.995 and policy['payload_kg_per_box']==10.
    engine=SimpleNamespace(dynamics=TorqueModel(ROOT/'generated/fast_runtime_assets/dynamics.yaml',payload_mass_kg=10.,
        payload_dimensions={s:p.dimensions_m for s,p in picks.items()}))
    planner.layout_native=(layout_model_key(picks),engine)
    specs=ordered_stages(picks)[3:];segments={};report.update(phase_audits={},trials=[],completed_stages=[])
    limits=np.array(policy['arm_continuous_torque_nm']*2)*.995;arrays['scaled_torque_limits_nm']=limits

    def publish():
        report['worker_elapsed_seconds']=time.perf_counter()-began
        atomic_json(OUT/'suffix_probe.json',report)

    def checker_for(spec,q):
        planner.set_stage(spec,q)
        return planner.checker(planner.selected_task,payload=bool(spec.payload_sides))

    def budget(wanted):
        remaining=deadline-time.perf_counter()
        if remaining<=1.:raise TimeoutError('110s bounded geometry budget exhausted; no input illegality claim')
        return min(wanted,remaining)

    def accept(spec,path):
        rows=np.asarray(densify(path),dtype=np.float32);checker=checker_for(spec,rows[0])
        validity=_geometry_validity(planner,spec,checker,rows[0]);mask=validity.mask(torch.as_tensor(rows,device='cuda'))
        audit=planner.loaded_audit(checker,rows)
        assert bool(mask.all()) and audit['valid'],dict(stage=spec.stage_id,mask_invalid=(~mask).nonzero().reshape(-1).cpu().tolist(),audit=audit)
        tau=[];model=engine.dynamics._model(tuple(checker.task_tool_to_box),checker.task_tool_to_box)
        for q in torch.as_tensor(rows,device='cuda').split(512):
            model.setup_batch_size(len(q));zeros=torch.zeros_like(q)
            tau.append(model.compute_inverse_dynamics(JointState(position=q.contiguous(),velocity=zeros,acceleration=zeros))[:,1:].cpu().numpy().copy())
        tau=np.concatenate(tau);assert (np.abs(tau)<=limits).all()
        poses=box_poses(checker,rows,checker.task_tool_to_box)
        pending=np.eye(4);pending[:3,3]=picks[giver].center_m
        for side in picks:arrays[spec.stage_id+'_'+side+'_box']=poses[side] if side in poses else np.repeat(pending[None],len(rows),axis=0)
        if segments:
            previous=next(reversed(segments.values()));assert np.max(np.abs(previous[-1]-rows[0]))<=1e-5
        else:assert np.array_equal(rows[0],start)
        segments[spec.stage_id]=rows;arrays[spec.stage_id]=rows;arrays[spec.stage_id+'_torques_nm']=tau
        report['phase_audits'][spec.stage_id]=dict(valid=True,frames=len(rows),payload_sides=list(spec.payload_sides),
            physical_payload_kg=10.*len(spec.payload_sides),max_scaled_gravity_ratio=float((np.abs(tau)/limits).max()),full_obb=audit)
        report['completed_stages']=list(segments);publish()
        print('SUFFIX_STAGE',spec.stage_id,len(rows),'static',report['phase_audits'][spec.stage_id]['max_scaled_gravity_ratio'],flush=True)
        return rows

    # Diagnose hard final endpoints before spending the path budget. Updown remains a101-goal variable.
    endpoint_spec=replace(specs[-1],source_sides=())
    reference=np.array([carry_configuration(-.5)[n] for n in NAMES],dtype=np.float32)
    checker=checker_for(endpoint_spec,reference)
    final_goals=_joint_stage_goals(planner,endpoint_spec,reference,checker)
    if len(final_goals):
        depths=planner.source_slot_depths(checker,torch.as_tensor(final_goals,device='cuda')).cpu().numpy()
        final_goals=final_goals[(depths<=1e-6).all(-1)]
    report['final_endpoint_valid_goals']=len(final_goals);publish()
    if not len(final_goals):
        report.update(failure_class='endpoint',failure='No final14-angle goal passed the finite101-lift collision/load/source screen; not a continuous infeasibility proof.');return
    park_spec=specs[0];checker=checker_for(park_spec,start)
    park_goals=_joint_stage_goals(planner,park_spec,start,checker)
    report['recipient7_park_goal_count']=len(park_goals);publish()
    if not len(park_goals):
        report.update(failure_class='parking_endpoint',failure='No recipient7 parking goal with current giver pose in finite101 lifts; extra giver reconfiguration not searched.');return
    park,search=_geometry_free_path(planner,park_spec,start,park_goals,checker,budget=budget(20.))
    report['parking_search']=search;publish()
    if park is None:
        report.update(failure_class='parking_path',failure='Bounded path to recipient7 parking goals not found; no input illegality claim.');return
    park=accept(park_spec,park);parked=park[-1]
    checker=checker_for(specs[1],parked);tools=planner._tool_poses(checker,torch.as_tensor(parked,device='cuda'))
    contacts=planner.enumerate_pair({giver:picks[giver].contact_tool(),recipient:tools[recipient]},parked,checker,contact=True,limit=128)
    if len(contacts):contacts=contacts[diverse_contact_order(contacts,giver)]
    report['contact_enumeration']=dict(planner.last_pair_enumeration);report['contact_candidates']=len(contacts);publish()
    if not len(contacts):
        report.update(failure_class='contact',failure='Finite calibrated two-tool IK contact pool empty after physical checks; not proof of input illegality.');return
    original_pending=picks[giver]
    report['precontact_admission']=[]
    def admit_contact(index,q):
        budget(1.)  # Same task deadline, including every admission query.
        picks[giver]=original_pending;planner.picks=picks;planner.layout_native=(layout_model_key(picks),engine)
        checker=checker_for(specs[2],q);cap=planner.target_validity(checker,allow_contact=True)
        distance=picks[giver].precontact_distance(cap.targets[giver]['tool_cap_reach_m'])
        contact=planner.precontact(q,{giver:distance,recipient:0.},checker)
        detail=dict(raw_index=index,contact_q=q.tolist(),giver_q=(q[1:8] if giver=='left' else q[8:15]).tolist())
        detail['shared_precontact_valid']=contact is not None
        if contact is not None:
            validity=_geometry_validity(planner,specs[2],checker,contact[0])
            valid=bool(validity.mask(torch.as_tensor(contact,device='cuda',dtype=torch.float32)).all())
            audit=planner.loaded_audit(checker,contact)
            detail.update(existing_stage_geometry_valid=valid,existing_full_obb_audit=audit)
            if not valid or not audit['valid']:contact=None
        detail['admitted']=contact is not None
        report['precontact_admission'].append(detail);publish()
        return contact
    for i,q,contact in admitted_precontacts(contacts,giver,admit_contact,limit=4):
        budget(1.);trial=dict(index=i,precontact=True);report['trials'].append(trial)
        checker=checker_for(specs[1],parked)
        approach,search=_geometry_free_path(planner,specs[1],parked,contact[:1],checker,budget=budget(12.));trial['approach_search']=search;publish()
        if approach is None:continue
        # Absorb the actual pending low box at contact, with no world-pose substitution.
        T=fk(planner.fk_robot,q[None])[giver][0];C=np.eye(4);C[:3,3]=original_pending.center_m
        A_pending=np.linalg.inv(T)@C;picks[giver]=measured(original_pending,A_pending);planner.picks=picks
        planner.layout_native=(layout_model_key(picks),engine);report['attachment_checks'][giver]=validate_attachment(picks[giver])
        checker=checker_for(specs[3],q);payload=checker.task_tool_to_box
        poses=box_poses(checker,q[None],payload);starts={s:T[0] for s,T in poses.items()};ends={s:T.copy() for s,T in starts.items()};ends[giver][0,3]-=picks[giver].extraction_distance_m
        validity=_geometry_validity(planner,specs[3],checker,q)
        lm=SequentialPlacementLm(planner,active_sides=('left','right'))
        result=lm.plan(dict(start=q,checker=checker,attachments=payload,starts=starts,ends=ends,
            candidate_filter=lambda values,state,actual:validity.mask(values).cpu().numpy()))
        trial['extraction']={k:v for k,v in result.items() if k not in ('frames','point_reports','failed_candidates','progress')};publish()
        if not result['success']:continue
        extraction=np.asarray(result['frames'],dtype=np.float32)
        checker=checker_for(specs[-1],extraction[-1]);goals=_joint_stage_goals(planner,specs[-1],extraction[-1],checker)
        trial['final_goals']=len(goals);publish()
        if not len(goals):continue
        final,search=_geometry_free_path(planner,specs[-1],extraction[-1],goals,checker,budget=budget(32.));trial['final_search']=search;publish()
        if final is None:continue
        arrays['pending_attachment']=A_pending
        if giver=='left' and roles.source=='right':arrays['left_attachment']=A_pending;arrays['right_attachment']=A_right
        for spec,path in zip(specs[1:],(approach,contact,extraction,final)):accept(spec,path)
        report['success']=True;break
    if not report['success']:
        report.update(failure_class='candidate_or_path_budget',failure='Existing finite contact pool admitted fewer than4 full precontact paths, or4 admitted full trials failed, or110s task deadline; inspect separate admission/trial records, not input illegality.')
    if report['success']:
        observations={}
        for side in picks:
            names=list(segments) if side==recipient else ['final_transport']
            poses=np.concatenate([arrays[k+'_'+side+'_box'] for k in names])
            observations[side]=source_slot_observation(picks[side],poses)
        report['source_process']=source_slot_process_checks(observations);assert report['source_process']['valid']
        report['final_updown_m']=float(arrays['final_transport'][-1,0]);report['attachment_pending_actual']=arrays['pending_attachment'].tolist()
        if giver=='left' and roles.source=='right':report['attachment_left_actual']=report['attachment_pending_actual']
        report['source_history_reference']=dict(physical_id=roles.owner_to_physical_id[recipient],continued_from='this run donor_continue[-1]',source_process='prefix history and suffix no-return masks')


if __name__=='__main__':
    started=time.perf_counter();arrays={}
    report=dict(success=False,geometry_only=True,native_timed=False,real_robot_execution=False,
        runtime=dict(sys_executable=sys.executable,curobo_module_path=curobo.__file__,curobo_version=getattr(curobo,'__version__',None),
            curobo_distribution_version=importlib.metadata.version('nvidia-curobo'),torch_version=torch.__version__,numpy_version=np.__version__),
        estimate='2m provisional based on owner23.7s and ordered93s; not a timeout',bounded_geometry_budget_seconds=110.)
    try:run(report,arrays)
    except TimeoutError as error:report.update(failure_class='budget',failure=str(error))
    except Exception as error:
        report.update(success=False,failure_class='experiment_error',failure=repr(error),traceback=traceback.format_exc());traceback.print_exc()
    finally:
        report['wall_seconds']=time.perf_counter()-started
        report['input_hashes_after']=hashes()
        if 'input_hashes_before' in report:assert report['input_hashes_before']==report['input_hashes_after']
        report['script_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.glob('*.py')}
        report['source_stamp_at_end']=session_source_stamp()
        np.savez_compressed(OUT/'suffix_geometry.npz',**arrays);atomic_json(OUT/'suffix_probe.json',report)
        print('SUFFIX_RESULT',report['success'],report.get('failure_class'),report['wall_seconds'],flush=True)
    raise SystemExit(0 if report['success'] else 1)
