"""CPU-only audit of this run's13-stage geometry, physical IDs and actual URDF transforms."""
from pathlib import Path
import argparse
import hashlib
import importlib.metadata
import json
import sys
import time
import numpy as np
import yourdfpy
from scipy.spatial.transform import Rotation
from plan_fresh import ROOT,HERE,NAMES,PHASES,source_hashes
from physical_cpu import forward
from curobo_core.box_pick import BoxPick,source_slot_observation,source_slot_process_checks
from curobo_core.scene import SceneSnapshot
from curobo_core.adapter import pose_matrix
from curobo_core.layout_contract import CARRY_DEGREES
from curobo_core.suction_grasp import contact_cap_spheres
from audit_suffix import gravity

def audit(folder):
    began=time.perf_counter();report=json.loads((folder/'fresh_report.json').read_text());assert report['success'],report.get('failure')
    request=json.loads(Path(report['request_file']).read_text());assert request==report['request']
    assert hashlib.sha256(Path(report['request_file']).read_bytes()).hexdigest()==report['input_request_sha256']
    interface=json.loads((folder/'stage_interface.json').read_text());assert interface['request']==request and interface['joint_names']==NAMES
    with np.load(folder/'fresh_geometry.npz',allow_pickle=False) as d:arrays={k:d[k].copy() for k in d.files}
    assert [s['stage_id'] for s in interface['stages']]==list(PHASES) and len(interface['stages'])==13
    robot=yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf',load_meshes=False)
    dynamics=yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/dynamics.urdf',load_meshes=False)
    originals={r['side']:BoxPick(**r) for r in request['boxes']};previous=None;previous_boxes=None;seams={};pose_seams={};history={s:[] for s in originals};maxfk=0.;maxgravity=0.
    for i,(key,stage) in enumerate(zip(PHASES,interface['stages'])):
        q=arrays[key];assert q.ndim==2 and q.shape[1]==15 and np.isfinite(q).all()
        seam=0. if previous is None else float(np.max(np.abs(q[0]-previous)));assert seam<=1e-5,(key,seam)
        seams[key]=seam;previous=q[-1]
        snapshot=SceneSnapshot.from_dict(stage['scene']);allobjects=list(snapshot.objects)+list(snapshot.attachments)
        physical=[o.object_id for o in allobjects if o.object_id.startswith('wall_box_')]
        assert sorted(physical)==['wall_box_00','wall_box_01']
        attached={a.object_id:a for a in snapshot.attachments};assert stage['payload_mass_kg']==10.*len(attached)
        assert stage['object_to_parent']=={k:a.parent_link for k,a in attached.items()}
        expected_high=None if i<2 else 'left_tool0' if i<=5 else 'right_tool0'
        expected_low='left_tool0' if i>=11 else None
        for objectid,side,expected in [('wall_box_01','right',expected_high),('wall_box_00','left',expected_low)]:
            obj=next(o for o in allobjects if o.object_id==objectid);assert np.array_equal(obj.dimensions_m,originals[side].dimensions_m)
            assert (attached[objectid].parent_link if objectid in attached else None)==expected
            if expected is None:assert np.array_equal(obj.pose.position,originals[side].center_m)
        poses=forward(robot,q)[0];actual={};attachments={}
        for objectid,side in [('wall_box_01','right'),('wall_box_00','left')]:
            if objectid in attached:
                a=attached[objectid];A=pose_matrix(a.tool_to_object);C=poses[a.parent_link]@A;attachments[a.parent_link.removesuffix('_tool0')]=A
            else:
                C=np.repeat(np.eye(4)[None],len(q),axis=0);C[:,:3,3]=originals[side].center_m
            actual[side]=C;error=float(np.max(np.abs(C-arrays[key+'_'+side+'_box'])));assert error<=2e-5
            maxfk=max(maxfk,error)
            if previous_boxes is not None:assert np.max(np.abs(C[0]-previous_boxes[side]))<=2e-5,(key,side)
            if side=='right' and i>=2 or side=='left' and i>=11:history[side].append(C)
        pose_seams[key]={s:0. if previous_boxes is None else float(np.max(np.abs(C[0]-previous_boxes[s]))) for s,C in actual.items()}
        previous_boxes={s:C[-1] for s,C in actual.items()}
        for col,name in enumerate(NAMES):
            limit=robot.joint_map[name].limit;assert np.all(q[:,col]>=limit.lower) and np.all(q[:,col]<=limit.upper),(key,name)
        if key+'_torques_nm' in arrays:
            for sample in sorted(set((0,len(q)//2,len(q)-1))):
                expected=gravity(dynamics,q[sample],attachments)
                if key=='support_transfer':
                    alpha=arrays['support_alpha'][sample];left=gravity(dynamics,q[sample],{'left':arrays['left_donor_attachment']});right=gravity(dynamics,q[sample],{'right':arrays['right_attachment']});expected=(1-alpha)*left+alpha*right
                error=float(np.max(np.abs(expected-arrays[key+'_torques_nm'][sample])))
                assert error<=.01,(key,error);maxgravity=max(maxgravity,error)
    high=arrays['extract_right_box'];low=arrays['lower_extract_left_box'];pulls={}
    for side,C,pick in [('right',high,originals['right']),('left',low,originals['left'])]:
        assert np.linalg.norm(C[0,:3,3]-pick.center_m)<=.002
        vector=C[-1,:3,3]-C[0,:3,3];expected=[-pick.extraction_distance_m,0,0]
        assert np.linalg.norm(vector-expected)<=.002
        assert np.linalg.norm((C[:,:3,3]-C[0,:3,3])[:,1:],axis=-1).max()<=.002
        assert Rotation.from_matrix(C[0,:3,:3].T@C[:,:3,:3]).magnitude().max()<=np.deg2rad(1)
        pulls[side]=vector.tolist()
    contact=arrays['contact_right_box'][-1];assert np.linalg.norm(contact[:3,3]-originals['right'].center_m)<=1e-10
    switchq=arrays['support_transfer'];assert np.max(np.abs(switchq-switchq[0]))==0
    alpha=arrays['support_alpha'];assert alpha[0]==0 and alpha[-1]==1 and np.all(np.diff(alpha)>=0)
    tools=forward(robot,switchq[:1])[0];C=tools['left_tool0'][0]@arrays['left_donor_attachment'];measured=np.linalg.inv(tools['right_tool0'][0])@C
    assert np.max(np.abs(measured-arrays['right_attachment']))<=2e-5
    switchjump=float(np.max(np.abs(tools['right_tool0'][0]@arrays['right_attachment']-C)));assert switchjump<=2e-5
    final=np.deg2rad(np.r_[CARRY_DEGREES,-CARRY_DEGREES]);angle_error=float(np.max(np.abs(arrays['final_transport'][-1,1:]-final)));assert angle_error<=1e-5
    for side,A in [('left',arrays['left_final_attachment']),('right',arrays['right_attachment'])]:
        inv=np.linalg.inv(A);caps=contact_cap_spheres(side);points=caps[:,:3]@inv[:3,:3].T+inv[:3,3];axis=0 if side=='left' else 1;tangent=[n for n in range(3) if n!=axis]
        assert np.max(np.abs(points[:,tangent])+caps[:,3,None]-np.asarray(originals[side].dimensions_m)[tangent]/2)<=.001
    process=source_slot_process_checks({s:source_slot_observation(originals[s],np.concatenate(h)) for s,h in history.items()});assert process['valid'],process
    current=source_hashes();mismatch=[p for p,h in report['source_sha256_after'].items() if current.get(p)!=h];assert not mismatch,mismatch
    result=dict(valid=True,geometry_only=True,native_acceptance=False,request_preserved=True,physical_ids=['wall_box_00','wall_box_01'],phase_count=13,stage_samples=report['stage_samples'],
        total_phase_samples=sum(report['stage_samples'].values()),q_seams=seams,box_pose_seams=pose_seams,max_fk_element_error=maxfk,
        measured_right_attachment_max_error=float(np.max(np.abs(measured-arrays['right_attachment']))),support_switch_box_pose_jump=switchjump,
        actual_extraction_world_m=pulls,final_14angle_error_rad=angle_error,final_updown_m=float(arrays['final_transport'][-1,0]),
        independent_static_gravity_max_error_nm=maxgravity,source_process=process,source_hashes_match=True,
        runtime=dict(executable=sys.executable,numpy_version=np.__version__,yourdfpy_version=importlib.metadata.version('yourdfpy')),wall_seconds=time.perf_counter()-began,
        scope='Independent actual URDF FK, original source absorption/extraction, every physical-ID/box/q seam, measured handover, source history, sampled gravity and final14 angles. GPU collision evidence remains separate.')
    (folder/'fresh_cpu_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ('q_seams','box_pose_seams','source_process')},indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);audit(p.parse_args().folder)
