"""CPU contract for ordered pickup: every original carton is pending or actually attached."""
from dataclasses import dataclass, asdict
import re
import numpy as np
from scipy.spatial.transform import Rotation

SIDES = {'left', 'right'}


@dataclass(frozen=True)
class OrderedStage:
    stage_id: str
    phase: str
    payload_sides: tuple
    pending_sides: tuple
    incoming_side: str | None = None
    cartesian_displacements: tuple = ()
    source_sides: tuple = ()
    final_joint_sides: tuple = ()

    def __post_init__(self):
        if not re.fullmatch('[a-z][a-z0-9_]*', self.stage_id) or self.phase not in ('approach', 'contact', 'extract', 'transport'):
            raise ValueError('invalid ordered native stage')
        payload, pending = set(self.payload_sides), set(self.pending_sides)
        if (len(payload) != len(self.payload_sides) or len(pending) != len(self.pending_sides)
                or payload & pending or payload | pending != SIDES):
            raise ValueError('every carton must be pending or attached, exactly once')
        if self.incoming_side is not None and (self.incoming_side not in pending or self.phase not in ('approach', 'contact')):
            raise ValueError('contact target must remain a pending physical carton')
        if not set(self.source_sides) <= payload or not set(self.final_joint_sides) <= payload:
            raise ValueError('source and joint goals require real attached payloads')
        translations = dict(self.cartesian_displacements)
        if len(translations) != len(self.cartesian_displacements) or any(
                side not in payload or np.asarray(delta).shape != (3,) or not np.isfinite(delta).all()
                for side, delta in translations.items()):
            raise ValueError('invalid Cartesian extraction references')
        if self.phase == 'extract' and (set(translations) != payload or sum(np.linalg.norm(v) > 0 for v in translations.values()) != 1):
            raise ValueError('ordered extraction moves one carton and explicitly holds the other')


def ordered_stages(picks):
    upper = max(('left', 'right'), key=lambda s:picks[s].center_m[2])
    lower = 'left' if upper == 'right' else 'right'
    both = ('left', 'right')
    upper_pull = ((upper, (-picks[upper].extraction_distance_m, 0., 0.)),)
    lower_pull = tuple((s, (-picks[s].extraction_distance_m, 0., 0.) if s == lower else (0., 0., 0.)) for s in both)
    return (
        OrderedStage('upper_approach', 'approach', (), both, upper),
        OrderedStage('upper_contact', 'contact', (), both, upper),
        OrderedStage('upper_extract', 'extract', (upper,), (lower,), cartesian_displacements=upper_pull),
        OrderedStage('upper_park', 'transport', (upper,), (lower,), source_sides=(upper,), final_joint_sides=(upper,)),
        OrderedStage('lower_approach', 'approach', (upper,), (lower,), lower, source_sides=(upper,)),
        OrderedStage('lower_contact', 'contact', (upper,), (lower,), lower, source_sides=(upper,)),
        OrderedStage('lower_extract', 'extract', both, (), cartesian_displacements=lower_pull, source_sides=(upper,)),
        OrderedStage('final_transport', 'transport', both, (), source_sides=both, final_joint_sides=both),
    )


def cartesian_check(poses, start, end):
    """Same2mm/1deg gates for a moving carton and a previously carried fixed carton."""
    poses, start, end = map(lambda x:np.asarray(x, dtype=float), (poses, start, end))
    if poses.ndim != 3 or poses.shape[1:] != (4, 4) or not len(poses) or start.shape != (4, 4) or end.shape != (4, 4):
        raise ValueError('Cartesian stage requires finite rigid pose arrays')
    if not all(np.isfinite(x).all() for x in (poses, start, end)):
        raise ValueError('Cartesian stage contains nonfinite poses')
    vector = end[:3, 3]-start[:3, 3]
    length = float(np.linalg.norm(vector))
    delta = poses[:, :3, 3]-start[:3, 3]
    axial = delta@(vector/length) if length > 1e-12 else np.zeros(len(poses))
    lateral = np.linalg.norm(delta-axial[:, None]*(vector/length), axis=1) if length > 1e-12 else np.linalg.norm(delta, axis=1)
    angles = np.rad2deg(Rotation.from_matrix(start[:3, :3].T@poses[:, :3, :3]).magnitude())
    first, last = float(np.linalg.norm(poses[0, :3, 3]-start[:3, 3])), float(np.linalg.norm(poses[-1, :3, 3]-end[:3, 3]))
    valid = bool(lateral.max() <= .002 and angles.max() <= 1. and axial.min() >= -.002
                 and axial.max() <= length+.002 and max(first, last) <= .002)
    return dict(valid=valid, max_lateral_error_m=float(lateral.max()), max_orientation_error_deg=float(angles.max()),
                progress_min_m=float(axial.min()), progress_max_m=float(axial.max()),
                start_error_m=first, end_error_m=last, reference_start=start.tolist(), reference_end=end.tolist())


def ordered_native_samples(record):
    """Unmodified actual native samples with unique ordered-stage labels."""
    from .box_pick import BoxPick
    timing = record.get('timing', record)
    if not record.get('success') or not timing.get('success'):raise ValueError('ordered replay requires an accepted native result')
    picks = {s:BoxPick(**p) for s,p in record['task'].items()}
    specs = ordered_stages(picks)
    if [s.get('stage_id') for s in timing['stages']] != [s.stage_id for s in specs]:raise ValueError('ordered replay is missing stages')
    clocks, rows, labels = [], [], [];elapsed=0.;previous=None
    for spec, stage in zip(specs,timing['stages']):
        if not stage.get('success'):raise ValueError('ordered replay contains a rejected stage')
        with np.load(stage['trajectory_file'],allow_pickle=False) as arrays:
            t,q=arrays['times'].copy(),arrays['position'].copy()
        if (t.ndim!=1 or len(t)<2 or q.shape!=(len(t),15) or t[0]!=0. or not np.isfinite(t).all()
                or not np.isfinite(q).all() or not np.all(np.diff(t)>0)):
            raise ValueError('ordered replay has invalid actual native arrays')
        if previous is not None and np.max(np.abs(q[0]-previous))>1e-5:raise ValueError('ordered replay joint seam rejected')
        offset=0 if previous is None else 1
        clocks.extend(t[offset:].astype(np.float64)+elapsed);rows.extend(q[offset:]);labels.extend([spec.stage_id]*(len(t)-offset))
        elapsed+=float(t[-1]);previous=q[-1]
    return np.asarray(clocks),np.asarray(rows),np.asarray(labels)


def verify_ordered_result(record):
    """Independent CPU state/FK/array/source and attachment point-load audit; GPU collision/RNEA stay separate."""
    from pathlib import Path
    import json
    import xml.etree.ElementTree as ET
    import yourdfpy
    import yaml
    from .box_pick import BoxPick,timed_path_checks,source_slot_observation
    from .layout_contract import validate_pair,DEFAULT_SIDE_MIN_HEIGHT_M
    from .native_transport import native_seam,source_progress,urdf_box_poses
    root=Path(__file__).resolve().parents[2]
    timing=record.get('timing',record)
    if record.get('workflow')!='ordered_upper_first' or not record.get('success') or not timing.get('success'):
        raise ValueError('ordered verification requires a complete accepted ordered candidate')
    picks={s:BoxPick(**p) for s,p in record['task'].items()};validate_pair(picks)
    request=record['request']
    if request.get('frame_id')!='world' or not np.array_equal(np.asarray(request.get('world_from_robot_base',np.eye(4))),np.eye(4)):
        raise ValueError('ordered input frame/base changed')
    original={p['side']:p for p in request['boxes']}
    attachments={};normal_axes={};cap_geometry={}
    config=yaml.safe_load((root/'generated/fast_runtime_assets/fixed.yaml').read_text())['kinematics']
    tree=ET.parse(root/'generated/fast_runtime_assets/fixed.urdf').getroot()
    for side,pick in picks.items():
        source=original[side]
        if (source['size']!=pick.size or not np.array_equal(source['center_m'],pick.center_m)
                or source.get('swap_length_height',False)!=pick.swap_length_height
                or source.get('side_min_height_m',DEFAULT_SIDE_MIN_HEIGHT_M)!=pick.side_min_height_m):
            raise ValueError('ordered task changed the requested physical carton')
        for key in ('side_face','suction_offset_m','suction_roll_deg'):
            if key in source:
                equal=(np.array_equal(source[key],getattr(pick,key)) if key=='suction_offset_m' else source[key]==getattr(pick,key))
                if not equal:raise ValueError('explicit input suction choice changed')
        dimensions=pick.dimensions_m;tool=np.eye(4)
        if pick.top:
            axis=2;tool[:3,:3]=np.diag([-1.,1.,-1.]);tool[2,3]=dimensions[2]/2
        else:
            axis,sign,yaw={'x-':(0,-1,0.),'x+':(0,1,180.),'y-':(1,-1,90.),'y+':(1,1,-90.)}[pick.side_face]
            tool[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
            tool[axis,3]=sign*dimensions[axis]/2
        tool[:3,3]+=pick.suction_offset_m
        tool[:3,:3]=tool[:3,:3]@Rotation.from_euler('z',pick.suction_roll_deg,degrees=True).as_matrix()
        attachments[side]=np.linalg.inv(tool);normal_axes[side]=axis
        origin=np.fromstring(tree.find(f"joint[@name='{side}_tool0_fixed']/origin").get('xyz'),sep=' ')
        caps=[c for c in config['collision_spheres'][side+'_link7'] if c['radius']>0 and c['center'][2]+c['radius']>origin[2]]
        cap_geometry[side]=(np.asarray([c['center'] for c in caps])-origin,np.asarray([c['radius'] for c in caps]))
    specs=ordered_stages(picks);stages=timing['stages']
    if [s.get('stage_id') for s in stages]!=[s.stage_id for s in specs]:raise ValueError('ordered workflow lacks its full stage sequence')
    names=['updown']+[f'{s}_joint{i}' for s in ('left','right') for i in range(1,8)]
    if record['joint_names']!=names:raise ValueError('ordered native joint order changed')
    policy=json.loads((root/'config/fast_three_layers.json').read_text())
    robot=yourdfpy.URDF.load(root/'generated/fast_runtime_assets/fixed.urdf',load_meshes=False)
    previous=None;stage_arrays=[];checks=[];reference={};cleared={};environment=None;mass_switches=[]
    expected_left=np.deg2rad([40.,-46.,0.,-100.,-30.,-75.,90.])
    for spec,stage in zip(specs,stages):
        expected_contract=json.loads(json.dumps(asdict(spec)))
        if (json.loads(json.dumps(stage.get('physical_contract')))!=expected_contract or stage.get('payload_sides')!=list(spec.payload_sides)
                or not stage.get('success') or not stage.get('physical_success')
                or not stage.get('actual_stage_seam',{}).get('valid')):
            raise ValueError('ordered physical stage/load contract mismatch')
        scene=stage['physical_scene'];objects={o['object_id']:o for o in scene['objects']}
        if len(objects)!=len(scene['objects']):raise ValueError('duplicated ordered world object')
        expected_ids={f'wall_box_{0 if s=="left" else 1:02d}' for s in spec.pending_sides}
        if {k for k in objects if k.startswith('wall_box_')}!=expected_ids:raise ValueError('pending carton missing or attached carton left in world')
        for side in spec.pending_sides:
            obj=objects[f'wall_box_{0 if side=="left" else 1:02d}']
            if (not np.array_equal(obj['dimensions_m'],picks[side].dimensions_m)
                    or not np.array_equal(obj['pose']['position'],picks[side].center_m)
                    or not np.array_equal(obj['pose']['quaternion_wxyz'],[1.,0.,0.,0.])):
                raise ValueError('unpicked carton moved or changed shape')
        env={k:v for k,v in objects.items() if k not in expected_ids}
        if environment is None:environment=env
        if env!=environment:raise ValueError('ordered environment changed')
        base=scene['state']['base_pose']
        if (set(base)!={'position','quaternion_wxyz'} or not np.array_equal(base['position'],[0.,0.,0.])
                or not np.array_equal(base['quaternion_wxyz'],[1.,0.,0.,0.])):
            raise ValueError('ordered robot base moved')
        actual_attachments={a['parent_link'].removesuffix('_tool0'):a for a in scene['attachments']}
        if len(actual_attachments)!=len(scene['attachments']) or set(actual_attachments)!=set(spec.payload_sides):
            raise ValueError('actual physical scene lost or invented a payload')
        for side,attached in actual_attachments.items():
            pose=attached['tool_to_object'];matrix=np.eye(4)
            matrix[:3,:3]=Rotation.from_quat(np.roll(pose['quaternion_wxyz'],-1)).as_matrix();matrix[:3,3]=pose['position']
            if (attached['object_id']!=f'wall_box_{0 if side=="left" else 1:02d}'
                    or not np.array_equal(attached['dimensions_m'],picks[side].dimensions_m)
                    or not np.allclose(matrix,attachments[side],atol=1e-8,rtol=0.)):
                raise ValueError('ordered attachment geometry changed')
        with np.load(stage['trajectory_file'],allow_pickle=False) as data:arrays={k:data[k].copy() for k in data.files}
        piece_check=None
        if stage.get('pieces'):
            from .native_transport import concatenate_native
            real_pieces=[]
            for piece in stage['pieces']:
                selected=piece['stage'];physical=selected['physical_scene']
                if (not selected.get('success') or not selected.get('physical_success')
                        or selected.get('payload_sides')!=list(spec.payload_sides)
                        or selected.get('pending_sides')!=list(spec.pending_sides)
                        or json.loads(json.dumps(physical['objects']))!=json.loads(json.dumps(scene['objects']))
                        or json.loads(json.dumps(physical['attachments']))!=json.loads(json.dumps(scene['attachments']))
                        or physical['state']['base_pose']!=scene['state']['base_pose']
                        or not selected['independent_rnea']['valid'] or not selected['time_optimization']['dynamic_valid']):
                    raise ValueError('native recovery piece changed payload/world or lacks physical acceptance')
                with np.load(selected['trajectory_file'],allow_pickle=False) as data:real_pieces.append({k:data[k].copy() for k in data.files})
            reconstructed,piece_seams=concatenate_native(real_pieces)
            if set(reconstructed)!=set(arrays) or any(not np.array_equal(reconstructed[k],arrays[k]) for k in arrays):
                raise ValueError('ordered native merged stage does not match its actual piece arrays')
            piece_check=dict(valid=True,count=len(real_pieces),exact_arrays_reconstructed=True,seams=piece_seams)
        check=timed_path_checks(arrays,arrays['position'][0],arrays['position'][-1],policy)
        if not check['valid']:raise ValueError('ordered actual time/limits/rest arrays rejected')
        seam=dict(valid=True) if previous is None else native_seam(previous,arrays)
        if not seam['valid']:raise ValueError('ordered actual q/v/a stage seam rejected')
        for i,name in enumerate(names):
            limit=robot.joint_map[name].limit
            if np.any(arrays['position'][:,i]<limit.lower-1e-6) or np.any(arrays['position'][:,i]>limit.upper+1e-6):
                raise ValueError('ordered actual native joints exceed URDF limits')
        if not stage['independent_rnea']['valid'] or not stage['time_optimization']['dynamic_valid']:
            raise ValueError('ordered native dynamic/RNEA gate is not accepted')
        needed=set(spec.payload_sides)|({spec.incoming_side} if spec.incoming_side else set())
        boxes=urdf_box_poses(robot,arrays['position'],names,{s:attachments[s] for s in needed})
        details=dict(stage_id=spec.stage_id,array_checks=check,seam=seam,payload_sides=spec.payload_sides,native_piece_reconstruction=piece_check)
        if spec.phase=='contact':
            side=spec.incoming_side;target=np.eye(4);target[:3,3]=picks[side].center_m
            position_error=float(np.linalg.norm(boxes[side][-1,:3,3]-target[:3,3]))
            angle_error=float(np.rad2deg(Rotation.from_matrix(boxes[side][-1,:3,:3]).magnitude()))
            if position_error>.002 or angle_error>1.:raise ValueError('ordered source absorption pose rejected')
            tool=boxes[side][-1]@np.linalg.inv(attachments[side]);centers,radii=cap_geometry[side]
            world=centers@tool[:3,:3].T+tool[:3,3];axes=[i for i in range(3) if i!=normal_axes[side]]
            excess=float(np.max(np.abs(world[:,axes]-np.asarray(picks[side].center_m)[axes])+radii[:,None]-np.asarray(picks[side].dimensions_m)[axes]/2))
            if excess>.001:raise ValueError('ordered actual contact proxy footprint exits carton face')
            details['absorption']=dict(side=side,position_error_m=position_error,angle_error_deg=angle_error,cap_footprint_excess_m=excess)
        if spec.phase=='extract':
            translations=dict(spec.cartesian_displacements);details['cartesian']={}
            for side,delta in translations.items():
                start=boxes[side][0];end=start.copy();end[:3,3]+=delta
                item=cartesian_check(boxes[side],start,end)
                if not item['valid']:raise ValueError('ordered extracted/held carton line rejected')
                details['cartesian'][side]=item
            moved=next(s for s,v in translations.items() if np.linalg.norm(v)>0)
            if previous is None:raise ValueError('payload attachment has no preceding rest contact')
            if max(np.abs(previous['velocity'][-1]).max(),np.abs(previous['acceleration'][-1]).max(),np.abs(arrays['velocity'][0]).max(),np.abs(arrays['acceleration'][0]).max())>1e-4:
                raise ValueError('payload switched during motion')
            robot.update_cfg(dict(zip(names,arrays['position'][0])))
            com=boxes[moved][0,:3,3];expected=np.zeros(14)
            for j in range(1,8):
                joint=robot.joint_map[f'{moved}_joint{j}'];T=robot.get_transform(joint.child,robot.base_link)
                expected[(0 if moved=='left' else 7)+j-1]=-(T[:3,:3]@joint.axis)@np.cross(com-T[:3,3],[0.,0.,-10.*9.81])
            actual=arrays['torque'][0]-previous['torque'][-1]
            error=float(np.max(np.abs(actual-expected)))
            if error>.01:raise ValueError('rest torque jump does not match attaching the real10kg carton')
            mass_switches.append(dict(stage_id=spec.stage_id,attached_side=moved,mass_kg=10.,max_point_gravity_jump_error_nm=error,
                                      actual_torque_jump_nm=actual.tolist(),cpu_expected_torque_jump_nm=expected.tolist()))
        observations={s:source_slot_observation(picks[s],boxes[s]) for s in spec.source_sides}
        if observations:
            processes=stage.get('carried_original_slot_process',{})
            if not all(processes.get(backend,{}).get('valid') for backend in ('gpu','cpu')):
                raise ValueError('primary ordered source gate rejected or missing')
            valid,_,next_clear,source_checks=source_progress(observations,reference,cleared)
            if not valid:raise ValueError('independent ordered original-slot history rejected')
            cleared.update(next_clear);details['source_checks']=source_checks
        if spec.phase=='extract':
            obs=source_slot_observation(picks[moved],boxes[moved][-1:])
            reference[moved]=obs['initial_slot_overlap_m'];cleared[moved]=obs['first_clear_frame'] is not None
        if spec.final_joint_sides:
            errors={s:float(np.max(np.abs(np.asarray([arrays['position'][-1,names.index(f'{s}_joint{i}')] for i in range(1,8)])-expected_left*(1 if s=='left' else -1)))) for s in spec.final_joint_sides}
            if max(errors.values())>1e-5:raise ValueError('ordered actual user joint goal rejected')
            details['user_joint_errors_rad']=errors
        previous=arrays;stage_arrays.append(arrays);checks.append(details)
    if not all(cleared.get(s,False) for s in picks):raise ValueError('ordered original source not cleared')
    times,positions,labels=ordered_native_samples(record)
    return dict(valid=True,samples=len(times),duration_s=float(times[-1]),stages=checks,mass_switches=mass_switches,
                final_updown_m=float(positions[-1,0]),both_sources_cleared=True,
                scope='Independent CPU native array/rest/qva/URDF/FK/contact footprint/original-slot and exact10kg attachment gravity-jump checks. Native sampled full collision and dynamic RNEA are retained worker gates, not independently recomputed here; not CCD/hardware/universal coverage.')


def replay_payload_sides(record, phase):
    """Actual attachment state; an ordered contact phase can already carry the other carton."""
    if record.get('workflow')=='ordered_upper_first':
        from .box_pick import BoxPick
        specs=ordered_stages({s:BoxPick(**p) for s,p in record['task'].items()})
        states={s.stage_id:s.payload_sides for s in specs}
        if phase not in states:raise ValueError('unknown ordered replay stage')
        return states[phase]
    if phase not in ('approach','contact','extract','transport'):raise ValueError('unknown pickup replay phase')
    task=record['task'];sides=(task['side'],) if 'side' in task else tuple(task)
    return sides if phase in ('extract','transport') else ()


def replay_box_poses(robot,record,phase):
    """Pending cartons stay at their original pose; attached cartons use their resolved actual grasp."""
    if record.get("workflow") == "handover_same_side":
        from .handover import IDS, replay_handover_boxes
        return {IDS[identity]:pose for identity,pose in replay_handover_boxes(robot, record, phase).items()}
    from .box_pick import BoxPick, actual_owner_ids
    task=record['task'];picks={task['side']:BoxPick(**task)} if 'side' in task else {s:BoxPick(**p) for s,p in task.items()}
    carried=set(replay_payload_sides(record,phase));poses={}
    owners=actual_owner_ids(record)
    labels={'wall_box_00':'left','wall_box_01':'right'}
    for side,pick in picks.items():
        pose=np.eye(4);pose[:3,3]=pick.center_m
        if side in carried:pose=robot.get_transform(side+'_tool0',robot.base_link)@pick.tool_to_box()
        poses[labels[owners[side]]]=pose
    return poses
