"""CPU checks/replay for the supported thirteen-stage single-box handover."""
from pathlib import Path
import hashlib
import json

import numpy as np
from scipy.spatial.transform import Rotation

from .adapter import pose_matrix
from .box_pick import BoxPick,timed_path_checks,source_slot_observation,source_slot_process_checks
from .layout_contract import CARRY_DEGREES,DEFAULT_SIDE_MIN_HEIGHT_M,validate_pair
from .native_transport import concatenate_native,native_seam,urdf_box_poses
from .ordered_pick import cartesian_check
from .scene import SceneSnapshot

ROOT = Path(__file__).resolve().parents[2]
STAGES = ('approach','contact','extract','owner_transport','receiver_approach','support_transfer',
          'donor_release','donor_continue','upper_park','lower_approach','lower_contact','lower_extract','final_transport')
NAMES = ['updown']+[f'{s}_joint{i}' for s in ('left','right') for i in range(1,8)]
FIELDS = ('times','position','velocity','acceleration','jerk','torque')
IDS = {'wall_box_00':'left','wall_box_01':'right'}


def _require(condition,message):
    if not condition:raise ValueError(message)


def _document_picks(document):
    _require(document.get('frame_id')=='world' and np.array_equal(document.get('world_from_robot_base',np.eye(4)),np.eye(4)),
        'handover requires the unchanged world frame and fixed robot base')
    rows=document['boxes']
    _require(all(row.get('carry_updown_m') is None for row in rows),'request cannot fix the planner-chosen carry updown')
    _require(len(rows)==2 and {p['side'] for p in rows}=={'left','right'},'handover requires both original physical boxes')
    picks={row['side']:BoxPick(**dict(row,carry_joint_home=row.get('carry_joint_home',True),
        side_min_height_m=row.get('side_min_height_m',document.get('side_min_height_m',DEFAULT_SIDE_MIN_HEIGHT_M)))) for row in rows}
    contract=validate_pair(picks)
    if 'front_distance_m' in document:
        _require(abs(float(document['front_distance_m'])-contract['front_distance_m'])<=1e-9,'requested box plane/front distance changed')
    return picks


def _same_physical(first,second):
    fields=('size','side','center_m','dimensions_m','side_min_height_m','swap_length_height',
            'precontact_margin_m','side_face','suction_offset_m','suction_roll_deg')
    _require(all(np.array_equal(getattr(first,k),getattr(second,k)) for k in fields),'original physical request changed')


def _contract(record):
    timing=record['timing'];interface=record['stage_interface']
    _require(record.get('workflow')=='handover_same_side' and record.get('success') is True and timing.get('success') is True,
        'handover requires a complete accepted native candidate')
    _require(record['joint_names']==NAMES and interface['joint_names']==NAMES,'native handover joint order changed')
    specs=interface['stages'];native=timing['stages']
    _require([s['stage_id'] for s in specs]==list(STAGES) and [s['stage_id'] for s in native]==list(STAGES),
        'only the complete supported thirteen-stage handover is accepted')
    transfer=interface['support_transfer']
    _require(transfer['physical_id'] in IDS and {transfer['parent_before'],transfer['parent_after']}==
        {'left_tool0','right_tool0'} and transfer['mass_kg']==10. and transfer.get('stationary_only') is True
        and transfer.get('alpha_key')=='support_alpha','unsupported source identity/parent/mass support contract')
    _require(all(s.get('success') is True and s.get('physical_success') is True for s in native),
        'native worker physical gate rejected or missing; optimizer success alone is insufficient')
    return specs,native


def _load(stage):
    with np.load(stage['trajectory_file'],allow_pickle=False) as data:
        arrays={k:data[k].copy() for k in FIELDS}
        alpha=data['support_alpha'].copy() if stage['stage_id']=='support_transfer' else None
    count=len(arrays['times'])
    for field,width in (('position',15),('velocity',15),('acceleration',15),('jerk',15),('torque',14)):
        _require(arrays[field].shape==(count,width) and np.isfinite(arrays[field]).all(),'invalid actual native '+field)
    t=arrays['times']
    _require(t.ndim==1 and count>=2 and np.isfinite(t).all() and t[0]==0 and np.all(np.diff(t)>0),'invalid actual native clock')
    if alpha is not None:
        _require(alpha.shape==(count,) and np.isfinite(alpha).all() and alpha[0]==0 and alpha[-1]==1 and np.all(np.diff(alpha)>=0),
            'invalid stationary support_alpha')
        _require(np.array_equal(arrays['position'],np.repeat(arrays['position'][:1],count,axis=0))
            and all(np.all(arrays[k]==0) for k in ('velocity','acceleration','jerk')),'support transfer is not stationary')
    return arrays,alpha


def _boxes(robot,spec,rows):
    scene=SceneSnapshot.from_dict(spec['scene'])
    physical={o.object_id:o for o in (*scene.objects,*scene.attachments) if o.object_id.startswith('wall_box_')}
    _require(set(physical)==set(IDS) and all(a.object_id in IDS for a in scene.attachments),'physical box identity missing or duplicated')
    attached={a.object_id:a for a in scene.attachments}
    tools=urdf_box_poses(robot,rows,NAMES,{s:np.eye(4) for s in ('left','right')})
    boxes={}
    for identity,obj in physical.items():
        if identity in attached:
            side=obj.parent_link.removesuffix('_tool0')
            _require(side in tools,'unsupported payload parent')
            boxes[identity]=tools[side]@pose_matrix(obj.tool_to_object)
        else:boxes[identity]=np.broadcast_to(pose_matrix(obj.pose),(len(rows),4,4)).copy()
    return boxes


def handover_native_samples(record):
    """Return unchanged native (times,q,stage labels); validate alpha separately, merge only standard keys."""
    _,native=_contract(record)
    arrays=[_load(stage)[0] for stage in native]
    merged,_=concatenate_native(arrays)
    labels=np.concatenate([np.full(len(a['times'])-bool(i),STAGES[i]) for i,a in enumerate(arrays)])
    return merged['times'],merged['position'],labels


def replay_handover_boxes(robot,record,phase):
    """Return physical box poses at the robot's CURRENT configuration, with stable box IDs."""
    specs,_=_contract(record)
    _require(phase in STAGES,'unknown handover stage')
    scene=SceneSnapshot.from_dict(specs[STAGES.index(phase)]['scene'])
    boxes={obj.object_id:pose_matrix(obj.pose) for obj in scene.objects if obj.object_id in IDS}
    for obj in scene.attachments:
        _require(obj.object_id in IDS,'unknown physical payload in replay')
        boxes[obj.object_id]=robot.get_transform(obj.parent_link,robot.base_link)@pose_matrix(obj.tool_to_object)
    _require(set(boxes)==set(IDS),'missing physical box in replay')
    return boxes


def _payload_gravity(robot,q,center,side):
    robot.update_cfg(dict(zip(NAMES,q)))
    result=np.zeros(14)
    for i in range(1,8):
        joint=robot.joint_map[f'{side}_joint{i}'];T=robot.get_transform(joint.child,robot.base_link)
        result[i-1+(7 if side=='right' else 0)]=-(T[:3,:3]@joint.axis)@np.cross(center-T[:3,3],[0.,0.,-98.1])
    return result


def verify_handover_result(record):
    """Verify the supported actual chain; native collision/dynamic RNEA remain worker evidence.

    Returns process_validation from the actual combined physical-box source histories;
    caller may assign that result before computing final worker task_success.
    """
    import yourdfpy
    specs,native=_contract(record)
    transfer=record['stage_interface']['support_transfer'];source=transfer['physical_id']
    other=next(key for key in IDS if key!=source)
    donor=transfer['parent_before'];recipient=transfer['parent_after']
    donor_side=donor.removesuffix('_tool0');recipient_side=recipient.removesuffix('_tool0')
    picks=_document_picks(record['request']);canonical=_document_picks(record['stage_interface']['request'])
    _require(set(record['task'])=={'left','right'},'missing original tasks')
    task={}
    for side,row in record['task'].items():
        _require(not set(row)-set(BoxPick.__dataclass_fields__),'task must contain only BoxPick fields; exact A belongs in scenes')
        task[side]=BoxPick(**row)
        _same_physical(picks[side],canonical[side]);_same_physical(picks[side],task[side])
    validate_pair(task)
    frames=np.asarray(record['frames']);phases=np.asarray(record['phases'])
    _require(frames.ndim==2 and frames.shape[1]==15 and len(frames)==len(phases) and np.isfinite(frames).all()
        and phases.ndim==1 and len(phases)>0,'invalid original thirteen-stage geometry')
    sequence=phases[np.r_[True,phases[1:]!=phases[:-1]]].tolist()
    _require(sequence==list(STAGES),'raw geometry stages missing or reordered')
    geometry=[frames[phases==stage] for stage in STAGES]
    _require(all(len(q)>=2 for q in geometry),'raw stage has no complete endpoints')
    arrays=[];alpha=None
    for stage in native:
        values,share=_load(stage);arrays.append(values)
        if share is not None:alpha=share
    robot=yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf',load_meshes=False)
    policy=json.loads((ROOT/'config/fast_three_layers.json').read_text())
    previous_boxes=None;attachments={};histories={key:[] for key in IDS};checks=[];all_boxes=[];environment=None
    for i,(spec,a,g) in enumerate(zip(specs,arrays,geometry)):
        scene=SceneSnapshot.from_dict(spec['scene']);objects={o.object_id:o for o in scene.objects}
        attached={o.object_id:o for o in scene.attachments}
        expected={}
        if i>=2:expected[source]=donor if i<=5 else recipient
        if i>=11:expected[other]=donor
        _require({key:a.parent_link for key,a in attached.items()}==expected,'physical payload ownership or pending-box stage changed')
        _require(scene.frame_id=='map' and np.array_equal(pose_matrix(scene.state.base_pose),np.eye(4)),'stage base/frame changed')
        current_environment={o.object_id:o for o in scene.objects if not o.object_id.startswith('wall_box_')}
        if environment is None:environment=(scene.model_id,current_environment)
        _require(environment==(scene.model_id,current_environment),'handover environment/model changed between stages')
        _require(np.max(np.abs(np.asarray(scene.state.ordered(NAMES))-g[0]))<=1e-5,'stage scene state differs from original geometric start')
        if 'request' in spec:
            stage_picks=_document_picks(spec['request'])
            for side in picks:_same_physical(picks[side],stage_picks[side])
        for identity,side in IDS.items():
            obj=attached.get(identity,objects.get(identity))
            _require(obj is not None and np.array_equal(obj.dimensions_m,picks[side].dimensions_m),'physical carton dimensions changed')
            if identity not in attached:
                _require(np.array_equal(obj.pose.position,picks[side].center_m) and np.array_equal(pose_matrix(obj.pose)[:3,:3],np.eye(3)),
                    'pending original carton moved or disappeared')
            else:
                key=(identity,obj.parent_link);A=pose_matrix(obj.tool_to_object)
                if key in attachments:_require(np.max(np.abs(attachments[key]-A))<=1e-12,'measured attachment changed within an ownership interval')
                attachments[key]=A
        box=_boxes(robot,spec,a['position']);all_boxes.append(box)
        array_check=timed_path_checks(a,g[0],g[-1],policy)
        _require(array_check['valid'],'actual native time/dynamic limits/rest/endpoints rejected')
        seam=dict(valid=True) if i==0 else native_seam(arrays[i-1],a)
        _require(seam['valid'],'actual native q/v/a stage seam rejected')
        for column,name in enumerate(NAMES):
            limit=robot.joint_map[name].limit
            _require(np.all((a['position'][:,column]>=limit.lower)&(a['position'][:,column]<=limit.upper)),name+' violates native URDF bounds')
        pose_seam={}
        for identity,C in box.items():
            if previous_boxes is not None:
                error=float(np.linalg.norm(C[0,:3,3]-previous_boxes[identity][:3,3]))
                angle=float(Rotation.from_matrix(previous_boxes[identity][:3,:3].T@C[0,:3,:3]).magnitude())
                _require(error<=.002 and angle<=np.deg2rad(1.),'physical world box pose jumps at a stage boundary')
                pose_seam[identity]=dict(position_m=error,angle_rad=angle)
            if identity in attached:histories[identity].append(C)
        previous_boxes={key:C[-1] for key,C in box.items()}
        checks.append(dict(stage_id=STAGES[i],array_checks=array_check,seam=seam,box_pose_seam=pose_seam,
            physical_box_to_parent=expected,pending_box_ids=sorted(set(IDS)-set(expected))))
    for after,identity in ((2,source),(11,other)):
        target=np.eye(4);target[:3,3]=picks[IDS[identity]].center_m
        C=all_boxes[after][identity][0]
        _require(np.linalg.norm(C[:3,3]-target[:3,3])<=.002 and Rotation.from_matrix(C[:3,:3]).magnitude()<=np.deg2rad(1.),
            'original source absorption pose rejected')
    extractions={}
    for i,identity in ((2,source),(11,other)):
        distance=picks[IDS[identity]].extraction_distance_m
        C=all_boxes[i][identity];end=C[0].copy();end[0,3]-=distance
        check=cartesian_check(C,C[0],end)
        _require(check['valid'],'actual requested world-X extraction rejected')
        extractions[identity]=dict(check,actual_world_m=(C[-1,:3,3]-C[0,:3,3]).tolist())
    observations={key:source_slot_observation(picks[IDS[key]],np.concatenate(history)) for key,history in histories.items()}
    process=source_slot_process_checks(observations)
    _require(process['valid'],'combined physical source history deepens/reenters or never clears')
    jumps=[]
    for i,identity,side in ((2,source,donor_side),(11,other,donor_side)):
        expected=_payload_gravity(robot,arrays[i]['position'][0],all_boxes[i][identity][0,:3,3],side)
        actual=arrays[i]['torque'][0]-arrays[i-1]['torque'][-1]
        error=float(np.max(np.abs(actual-expected)))
        _require(error<=.002,'actual attachment torque jump does not match the real 10kg box')
        jumps.append(dict(stage_id=STAGES[i],physical_id=identity,max_error_nm=error,actual_delta_nm=actual.tolist(),expected_delta_nm=expected.tolist()))
    support=arrays[5];q=support['position'][0]
    recipient_box=_boxes(robot,specs[6],q[None])[source][0]
    donor_box=all_boxes[5][source][0]
    _require(np.max(np.abs(recipient_box-donor_box))<=2e-5,'measured recipient attachment disagrees with the same physical box')
    expected=_payload_gravity(robot,q,recipient_box[:3,3],recipient_side)-_payload_gravity(robot,q,donor_box[:3,3],donor_side)
    error=float(np.max(np.abs(support['torque']-support['torque'][0]-alpha[:,None]*expected)))
    _require(error<=.002,'stationary support ramp torque does not match single-box gravity transfer')
    jumps.insert(1,dict(stage_id='support_transfer',physical_id=source,max_error_nm=error,
        actual_delta_nm=(support['torque'][-1]-support['torque'][0]).tolist(),expected_delta_nm=expected.tolist()))
    torque_seams=[]
    for i in range(1,len(arrays)):
        delta=float(np.max(np.abs(arrays[i]['torque'][0]-arrays[i-1]['torque'][-1])))
        if i not in (2,11):_require(delta<=.002,'rest torque jumps without a payload change')
        torque_seams.append(dict(stage_id=STAGES[i],max_torque_jump_nm=delta,load_attachment=i in (2,11)))
    final=arrays[-1]['position'][-1];angles=np.deg2rad(np.r_[CARRY_DEGREES,-CARRY_DEGREES])
    angle_error=float(np.max(np.abs(final[1:]-angles)))
    _require(angle_error<=1e-5,'final fourteen requested carry angles rejected')
    for pick in task.values():
        if pick.carry_updown_m is not None:_require(abs(final[0]-pick.carry_updown_m)<=1e-5,'actual chosen carry updown differs')
    times,q,labels=handover_native_samples(record)
    return dict(valid=True,samples=len(times),duration_s=float(times[-1]),stages=checks,mass_switches=jumps,
        torque_seams=torque_seams,extraction=extractions,process_validation=process,both_sources_cleared=True,
        final_14angle_max_error_rad=angle_error,final_updown_m=float(final[0]),
        support_transfer=dict(physical_id=source,mass_kg=10.,parent_before=donor,parent_after=recipient,
            times=support['times'].tolist(),alpha_recipient=alpha.tolist(),
            mass_kg_by_parent={donor:(10*(1-alpha)).tolist(),recipient:(10*alpha).tolist()}),
        trajectory_sha256={s['stage_id']:hashlib.sha256(Path(s['trajectory_file']).read_bytes()).hexdigest() for s in native},
        scope='CPU actual native arrays/clock/rest/seams/identity/request/URDF/FK/source history and point-gravity load jumps for this thirteen-stage mode. '
              'Full native sampled collision and dynamic RNEA are worker physical gates, not independently recomputed here; no universal, CCD or hardware claim.')
