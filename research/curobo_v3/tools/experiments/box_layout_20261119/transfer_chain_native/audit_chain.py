"""CPU-only integration of the three cached native components; never replans or repairs samples."""
from pathlib import Path
import hashlib
import json
import sys
import time
import traceback

import numpy as np
from scipy.spatial.transform import Rotation
import yourdfpy

ROOT = Path(__file__).resolve().parents[4]
DATA = ROOT/'generated/two_box_layout'
OUT = DATA/'joint_contract_20261124_native_chain'
sys.path[:0] = [str(ROOT/'tools'),str(Path(__file__).resolve().parent.parent/'transfer_native')]
from curobo_core.box_pick import BoxPick,timed_path_checks,source_slot_observation,source_slot_process_checks
from curobo_core.native_transport import concatenate_native,native_seam,urdf_box_poses
from curobo_core.payload_geometry import box_overlap_depth
from curobo_core.layout_contract import CARRY_DEGREES
from curobo_core.adapter import pose_matrix
from curobo_core.scene import Pose
from physical_cpu import forward,NAMES

FIELDS = ('position','velocity','acceleration','jerk','torque','times')
SOURCE_HASHES = {}


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path, arrays=False):
    path = Path(path)
    SOURCE_HASHES[str(path.relative_to(ROOT))] = sha(path)
    if arrays:
        with np.load(path,allow_pickle=False) as data:return {k:data[k].copy() for k in data.files}
    return json.loads(path.read_text())


def payload_gravity(robot, q, side, attachment, mass=10.):
    """Rest payload torque only: minus URDF world joint-axis dot ((box C-joint) cross m*g)."""
    poses = forward(robot,np.asarray(q)[None])[0]
    C = (poses[side+'_tool0'][0]@attachment)[:3,3]
    result = np.zeros(14)
    for number in range(1,8):
        joint = robot.joint_map[f'{side}_joint{number}']
        T = poses[joint.child][0]
        axis = T[:3,:3]@joint.axis
        result[number-1+(7 if side=='right' else 0)] = -axis@np.cross(C-T[:3,3],[0.,0.,-9.81*mass])
    return result


def main(report):
    for path in (Path(__file__),Path(__file__).resolve().parent.parent/'transfer_native/physical_cpu.py'):
        SOURCE_HASHES[str(path.relative_to(ROOT))]=sha(path)
    prefix_dir = DATA/'joint_contract_20261124_prefix_native/numeric_fix/attempt_02/prefix'
    handover_dir = DATA/'joint_contract_20261124_transfer_native/patched_06'
    suffix_dir = DATA/'joint_contract_20261124_suffix_native/attempt_01'
    prefix = read(prefix_dir/'prefix_report.json')
    handover = read(handover_dir/'native_report.json')
    suffix = read(suffix_dir/'native_result.json')
    for path in (prefix_dir.parent/'handoff.json',DATA/'joint_contract_20261124_transfer_native/takeover_native_interface.json',
                 DATA/'joint_contract_20261124_suffix_native/native_takeover_interface.json',handover_dir/'cpu_audit.json'):
        read(path)
    assert prefix['success'] and handover['success'] and suffix['native_success']
    request = prefix['request']
    originals = {b['side']:b for b in request['boxes']}
    suffix_request=suffix['original_request']
    assert request['frame_id']=='world' and suffix_request['front_distance_m']==request['front_distance_m']
    assert {b['side'] for b in suffix_request['boxes']}==set(originals)
    for box in suffix_request['boxes']:
        original=originals[box['side']]
        assert all(box[k]==original[k] for k in ('side','size','center_m'))
        assert suffix_request['side_min_height_m']==original['side_min_height_m']
        assert np.array_equal(box['dimensions_m'],BoxPick(**original).dimensions_m)
    assert handover['physical_box_id']=='right' and handover['pending_original_box']==originals['left']
    picks = {key:BoxPick(**value) for key,value in originals.items()}
    world = {key:np.eye(4) for key in picks}
    for key in world:world[key][:3,3]=picks[key].center_m
    pgeom = read(DATA/'joint_contract_20261123/transfer_owner_transport_geometry.npz',True)
    hgeom = read(DATA/'joint_contract_20261124_transfer/transfer_geometry.npz',True)
    sgeom = read(DATA/'joint_contract_20261124_suffix/suffix_geometry.npz',True)
    attachment = {('right','left'):hgeom['left_attachment'],('right','right'):hgeom['right_measured_attachment'],
                  ('left','left'):sgeom['left_attachment']}
    assert np.array_equal(attachment['right','left'],prefix['physical_checks']['attachment'])
    assert np.array_equal(attachment['right','right'],sgeom['right_attachment'])
    assert np.array_equal(attachment['right','right'],suffix['task']['right']['measured_attachment'])
    assert np.array_equal(attachment['left','left'],suffix['task']['left']['measured_attachment'])
    policy = read(ROOT/'config/fast_three_layers.json')
    manifest = read(DATA/'joint_contract_20261123/source_default_full31/manifest.json')
    assert len(manifest['files'])==284 and all(sha(ROOT/p)==v for p,v in manifest['files'].items())
    model = ROOT/'generated/fast_runtime_assets/dynamics.urdf'
    fixed = ROOT/'generated/fast_runtime_assets/fixed.urdf'
    for path in (model,fixed):SOURCE_HASHES[str(path.relative_to(ROOT))]=sha(path)
    robot = yourdfpy.URDF.load(model,load_meshes=False)
    scene = read(handover_dir/'physical_scene.json')
    obstacles = [o for o in scene['obstacles'] if o['name']!='pending_original_left']
    pending_obstacle = next(o for o in scene['obstacles'] if o['name']=='pending_original_left')
    assert np.array_equal(pending_obstacle['pose'][:3],picks['left'].center_m)
    assert np.array_equal(pending_obstacle['dims'],picks['left'].dimensions_m)
    pending_prefix = next(o for o in prefix['physical_snapshot']['objects'] if o['object_id']=='pending_original_left')
    assert np.array_equal(pending_prefix['pose']['position'],picks['left'].center_m)
    assert np.array_equal(pending_prefix['dimensions_m'],picks['left'].dimensions_m)
    names = ('approach','contact','extract','owner_transport','receiver_approach','support_transfer',
             'donor_release','donor_continue','upper_park','lower_approach','lower_contact','lower_extract','final_transport')
    files = [Path(s['trajectory_file']) for s in prefix['initial']['stages']]+[prefix_dir/'transport_native.npz']
    files += [handover_dir/(name+'.npz') for name in names[4:8]]
    files += [Path(s['trajectory_file']) for s in suffix['stages']]
    source_rows = [pgeom[k] for k in names[:4]]+[hgeom[k] for k in names[4:8]]+[sgeom[k] for k in names[8:]]
    physical = [{}, {}, {'right':'left'},{'right':'left'},{'right':'left'},{'right':'left'},
        {'right':'right'},{'right':'right'},{'right':'right'},{'right':'right'},{'right':'right'},
        {'right':'right','left':'left'},{'right':'right','left':'left'}]
    # Reopen actual suffix scene declarations and actual leaf arrays, including seed-retained final geometry.
    suffix_provenance = []
    for record in suffix['stages']:
        leaves=[]
        for leaf in record['pieces']:
            raw = read(leaf['report_file'])['stages'][0]
            data = read(leaf['trajectory_file'],True)
            leaves.append({k:data[k] for k in FIELDS})
            assert timed_path_checks(leaves[-1],sgeom[record['stage_id']][leaf['start']],
                sgeom[record['stage_id']][leaf['end']],policy)['valid']
            pending=set(raw['pending_sides']);objects=raw['physical_scene']['objects']
            low=[o for o in objects if o['object_id']=='wall_box_00']
            assert bool(low)==('left' in pending)
            if low:
                assert np.array_equal(low[0]['pose']['position'],picks['left'].center_m)
                assert np.array_equal(low[0]['dimensions_m'],picks['left'].dimensions_m)
            for attached in raw['physical_scene']['attachments']:
                side=attached['parent_link'].removesuffix('_tool0')
                assert np.allclose(pose_matrix(Pose(**attached['tool_to_object'])),sgeom[side+'_attachment'],atol=1e-12,rtol=0.)
            geom=raw.get('trajectory_optimization',{})
            assert geom.get('solver_success',True) and raw['success']
            suffix_provenance.append(dict(stage=record['stage_id'],optimizer_success=geom.get('optimizer_success'),
                selected_trajectory_source=geom.get('selected_trajectory_source'),report=leaf['report_file']))
        merged,_=concatenate_native(leaves);saved=read(record['trajectory_file'],True)
        assert all(np.array_equal(merged[k],saved[k]) for k in FIELDS)
    full = [read(path,True) for path in files]
    arrays = [{k:a[k] for k in FIELDS} for a in full]
    ledger=[];box_history={key:[] for key in picks};owned_history={key:[] for key in picks}
    previous_boxes=world;elapsed=0.;global_index=0;payload_obb=[];pose_seams=[]
    for i,(name,a,owners,geometry) in enumerate(zip(names,arrays,physical,source_rows)):
        checks=timed_path_checks(a,geometry[0],geometry[-1],policy)
        assert checks['valid'],(name,checks)
        for column,joint_name in enumerate(NAMES):
            limit=robot.joint_map[joint_name].limit
            assert np.all((a['position'][:,column]>=limit.lower)&(a['position'][:,column]<=limit.upper)),(name,joint_name)
        if i:assert native_seam(arrays[i-1],a)['valid']
        tools=urdf_box_poses(robot,a['position'],NAMES,{side:np.eye(4) for side in ('left','right')})
        boxes={key:(tools[owners[key]]@attachment[key,owners[key]] if key in owners else
            np.broadcast_to(world[key],(len(a['times']),4,4)).copy()) for key in picks}
        mass=np.zeros((len(a['times']),2))
        for side in owners.values():mass[:,0 if side=='left' else 1]+=10.
        owner_record=dict(owners)
        if name=='support_transfer':
            alpha=full[i]['support_alpha']
            assert alpha[0]==0 and alpha[-1]==1 and np.all(np.diff(alpha)>=0)
            assert np.array_equal(a['position'],np.repeat(a['position'][:1],len(alpha),axis=0))
            assert all(np.all(a[k]==0) for k in ('velocity','acceleration','jerk'))
            recipient=tools['right']@attachment['right','right']
            assert np.max(np.abs(recipient-boxes['right']))<=2e-5
            mass[:,0]=10*(1-alpha);mass[:,1]=10*alpha
            owner_record['right']='left -> right (stationary support shares)'
            assert np.all(mass.sum(-1)==10.),'single high box was duplicated in mass ledger'
        pending=sorted(set(picks)-set(owners))
        seam={key:dict(position_m=float(np.linalg.norm(boxes[key][0,:3,3]-previous_boxes[key][:3,3])),
            angle_rad=float(Rotation.from_matrix(previous_boxes[key][:3,:3].T@boxes[key][0,:3,:3]).magnitude())) for key in picks}
        assert max(v['position_m'] for v in seam.values())<=.002 and max(v['angle_rad'] for v in seam.values())<=np.deg2rad(1.)
        pose_seams.append(seam)
        for key,C in boxes.items():
            half=np.asarray(picks[key].dimensions_m)/2
            depths={o['name']:float(box_overlap_depth(C[:,:3,3],C[:,:3,:3],half,np.asarray(o['pose'][:3]),
                Rotation.from_quat(np.roll(o['pose'][3:7],-1)).as_matrix(),np.asarray(o['dims'])/2).max()) for o in obstacles}
            bottom=float((C[:,2,3]-(np.abs(C[:,2,:3])*half).sum(-1)).min())
            assert max(depths.values())<=.0010001 and bottom>=-.001,(name,key,depths,bottom)
            payload_obb.append(dict(stage=name,physical_box_id=key,maximum_world_depth_m=max(depths.values()),minimum_bottom_m=bottom))
            box_history[key].append(C[(1 if i else 0):])
            if key in owners:owned_history[key].append(C)
        aC,bC=boxes['left'],boxes['right']
        pair=box_overlap_depth(aC[:,:3,3],aC[:,:3,:3],np.asarray(picks['left'].dimensions_m)/2,
            bC[:,:3,3],bC[:,:3,:3],np.asarray(picks['right'].dimensions_m)/2)
        assert pair.max()<=.0010001,(name,'physical boxes overlap',float(pair.max()))
        count=len(a['times'])-bool(i)
        ledger.append(dict(stage_index=i,stage=name,trajectory_file=str(files[i]),native_samples=len(a['times']),
            chain_start_index=global_index-1 if i else 0,chain_end_index=global_index+count-1,
            start_time_s=elapsed,end_time_s=elapsed+float(a['times'][-1]),physical_box_to_owner=owner_record,
            pending_physical_box_ids=pending,pending_world_matrices={k:world[k].tolist() for k in pending},
            supported_mass_start_kg=mass[0].tolist(),supported_mass_end_kg=mass[-1].tolist(),
            total_supported_mass_min_kg=float(mass.sum(-1).min()),total_supported_mass_max_kg=float(mass.sum(-1).max()),
            array_checks=checks,maximum_box_pair_overlap_m=float(pair.max())))
        if name=='support_transfer':
            ledger[-1]['support_allocation_samples']=[dict(time_s=elapsed+float(t),alpha_right=float(alpha[k]),
                left_kg=float(mass[k,0]),right_kg=float(mass[k,1])) for k,t in enumerate(a['times'])]
        global_index+=count;elapsed+=float(a['times'][-1]);previous_boxes={k:C[-1] for k,C in boxes.items()}
        if name in ('contact','lower_contact'):
            side='left';key='right' if name=='contact' else 'left'
            candidate=tools[side][-1]@attachment[key,side]
            assert np.linalg.norm(candidate[:3,3]-world[key][:3,3])<=.002
            assert Rotation.from_matrix(candidate[:3,:3]).magnitude()<=np.deg2rad(1.)
    merged,seams=concatenate_native(arrays)
    assert timed_path_checks(merged,source_rows[0][0],source_rows[-1][-1],policy)['valid']
    processes={key:source_slot_process_checks({key:source_slot_observation(picks[key],np.concatenate(history))})
        for key,history in owned_history.items()}
    assert all(value['valid'] for value in processes.values()),processes
    pulls={}
    for index,key,distance in ((2,'right',.255),(11,'left',.300)):
        a=arrays[index];side=physical[index][key]
        C=urdf_box_poses(robot,a['position'],NAMES,{side:attachment[key,side]})[side]
        delta=C[-1,:3,3]-C[0,:3,3]
        assert np.linalg.norm(delta-[-distance,0,0])<=.002
        assert np.linalg.norm(C[0,:3,3]-picks[key].center_m)<=.002
        pulls[key]=dict(actual_world_m=delta.tolist(),error_m=float(np.linalg.norm(delta-[-distance,0,0])))
    jumps=[]
    for index,key,side in ((2,'right','left'),(11,'left','left')):
        before=arrays[index-1]['torque'][-1];after=arrays[index]['torque'][0]
        expected=payload_gravity(robot,arrays[index]['position'][0],side,attachment[key,side])
        error=float(np.max(np.abs(after-before-expected)))
        assert error<=.002,(names[index],error)
        jumps.append(dict(stage=names[index],type='stationary absorption',physical_box_id=key,
            time_s=ledger[index]['start_time_s'],mass_before_kg=ledger[index-1]['supported_mass_end_kg'],
            mass_after_kg=ledger[index]['supported_mass_start_kg'],native_before_nm=before.tolist(),native_after_nm=after.tolist(),
            independent_payload_delta_nm=expected.tolist(),max_error_nm=error))
    support=arrays[5];alpha=full[5]['support_alpha'];q=support['position'][0]
    expected=payload_gravity(robot,q,'right',attachment['right','right'])-payload_gravity(robot,q,'left',attachment['right','left'])
    ramp_error=float(np.max(np.abs(support['torque']-support['torque'][0]-alpha[:,None]*expected)))
    assert ramp_error<=.002
    jumps.insert(1,dict(stage='support_transfer',type='stationary single-box alpha ramp',physical_box_id='right',
        time_s=ledger[5]['start_time_s'],end_time_s=ledger[5]['end_time_s'],mass_before_kg=[10.,0.],mass_after_kg=[0.,10.],
        native_before_nm=support['torque'][0].tolist(),native_after_nm=support['torque'][-1].tolist(),
        independent_payload_delta_nm=expected.tolist(),max_error_nm=ramp_error))
    torque_seams=[]
    for i in range(1,len(arrays)):
        jump=float(np.max(np.abs(arrays[i]['torque'][0]-arrays[i-1]['torque'][-1])))
        if i not in (2,11):assert jump<=.002,(names[i],jump)
        torque_seams.append(dict(to_stage=names[i],native_torque_jump_nm=jump,load_switch=i in (2,11)))
    final=merged['position'][-1]
    final_error=float(np.max(np.abs(final[1:]-np.deg2rad(np.r_[CARRY_DEGREES,-CARRY_DEGREES]))))
    assert final_error<=1e-5 and abs(final[0]-sgeom['final_transport'][-1,0])<=1e-5
    # Save all native stage arrays, including both sides of absorption seams, in addition to the strict-clock chain.
    artifact=dict(merged)
    artifact.update(phase_index=np.concatenate([np.full(len(a['times'])-bool(i),i,dtype=np.int16) for i,a in enumerate(arrays)]),
        event_times=np.asarray([j['time_s'] for j in jumps]),event_before_torque=np.asarray([j['native_before_nm'] for j in jumps]),
        event_after_torque=np.asarray([j['native_after_nm'] for j in jumps]),
        event_end_times=np.asarray([j.get('end_time_s',j['time_s']) for j in jumps]),
        event_mass_before_kg=np.asarray([j['mass_before_kg'] for j in jumps]),
        event_mass_after_kg=np.asarray([j['mass_after_kg'] for j in jumps]))
    for key,history in box_history.items():artifact['physical_'+key+'_box_world']=np.concatenate(history)
    for i,a in enumerate(full):
        for key,value in a.items():artifact[f'stage_{i:02d}_{key}']=value
    np.savez_compressed(OUT/'native_chain.npz',**artifact)
    assert all(sha(ROOT/p)==digest for p,digest in SOURCE_HASHES.items()),'read-only component inputs changed'
    assert 'torch' not in sys.modules and 'curobo' not in sys.modules,'CPU audit imported a GPU library'
    report.update(success=True,cached_component_candidate=True,production_dispatch=False,fresh_planning=False,universal_claim=False,
        cpu_executable=sys.executable,gpu_initialized=False,request=request,native_samples=len(merged['times']),
        motion_seconds=float(merged['times'][-1]),physical_stages=13,phase_ledger=ledger,qva_seams=seams,
        box_pose_seams=pose_seams,source_history=processes,extraction=pulls,payload_world_ground_checks=payload_obb,
        mass_switches=jumps,mass_switch_tolerance_nm=.002,max_mass_switch_error_nm=max(j['max_error_nm'] for j in jumps),
        torque_seams=torque_seams,final_14angle_max_error_rad=final_error,final_updown_m=float(final[0]),
        dynamic_array_checks=timed_path_checks(merged,source_rows[0][0],source_rows[-1][-1],policy),
        suffix_native_provenance=suffix_provenance,source_sha256=SOURCE_HASHES,
        chain_npz_sha256=sha(OUT/'native_chain.npz'),
        scope='Unchanged cached-component native candidate; direct CPU q/v/a/clock/limits, physical identity/FK/OBB/world/ground/source history and gravity load-jump integration checks. '
              'Robot self/world collision and full dynamic RNEA retain original component sample evidence; no new GPU/CCD/hardware/universal or production/fresh claim.',
        merge_policy='Native concatenate_native drops the next duplicate rest q/v/a sample and rebases clocks only. '
                     'All original per-stage samples and both sides of torque load jumps are also retained in this NPZ.')


if __name__=='__main__':
    began=time.perf_counter();OUT.mkdir(parents=True,exist_ok=True);report=dict(success=False)
    try:main(report)
    except Exception as error:
        report.update(error=repr(error),traceback=traceback.format_exc());traceback.print_exc()
    report['wall_seconds']=time.perf_counter()-began
    (OUT/'audit.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:report.get(k) for k in ('success','native_samples','motion_seconds','max_mass_switch_error_nm','final_updown_m','wall_seconds','error')}))
    sys.exit(0 if report['success'] else 1)
