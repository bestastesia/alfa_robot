"""Independent CPU suffix identity/FK/seam/extraction/final-angle and static-gravity audit."""
from pathlib import Path
import hashlib
import importlib.metadata
import importlib.util
import json
import sys

import numpy as np
from scipy.spatial.transform import Rotation
import yourdfpy

ROOT = Path(__file__).resolve().parents[4]
TRANSFER = ROOT/'generated/two_box_layout/joint_contract_20261124_transfer'
OUT = ROOT/'generated/two_box_layout/joint_contract_20261124_suffix'
sys.path.insert(0, str(ROOT/'tools'))
from curobo_core.box_pick import BoxPick, source_slot_observation, source_slot_process_checks
from curobo_core.layout_contract import CARRY_DEGREES
from curobo_core.suction_grasp import contact_cap_spheres
NAMES = ['updown']+[f'{s}_joint{i}' for s in ('left', 'right') for i in range(1, 8)]


def hashes():
    path = ROOT/'generated/two_box_layout/joint_contract_20261123/source_default_full31/manifest.json'
    manifest = json.loads(path.read_text())
    bad = [p for p, digest in manifest['files'].items() if not (ROOT/p).is_file() or hashlib.sha256((ROOT/p).read_bytes()).hexdigest() != digest]
    assert not bad, bad
    return dict(existing_manifest_files=len(manifest['files']), mismatches=bad,
        transfer_files={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in TRANSFER.iterdir() if p.is_file()})


def inputs():
    interface = json.loads((TRANSFER/'takeover_interface.json').read_text())
    prior = json.loads((ROOT/'generated/two_box_layout/joint_contract_20261123/transfer_owner_transport_probe.json').read_text())
    picks = {b['side']:BoxPick(**b) for b in prior['request']['boxes']}
    with np.load(TRANSFER/'transfer_geometry.npz', allow_pickle=False) as data:
        start = data['donor_continue'][-1].copy()
        A = data['right_measured_attachment'].copy()
    assert np.array_equal(start, interface['continuation_end_joint_values'])
    assert np.array_equal(A, interface['receiver_measured_tool_to_box'])
    return interface, picks, start, A


def fk(robot, rows):
    result = {s:[] for s in ('left', 'right')}
    for q in rows:
        robot.update_cfg(dict(zip(NAMES, q)))
        for side in result:result[side].append(robot.get_transform(side+'_tool0', robot.base_link).copy())
    return {s:np.asarray(T) for s,T in result.items()}


def gravity(robot, q, attachments):
    robot.update_cfg(dict(zip(NAMES, q)))
    children = {}
    for joint in robot.joint_map.values():children.setdefault(joint.parent, []).append(joint.child)
    def descendants(link):return [link]+[n for child in children.get(link, []) for n in descendants(child)]
    box = {s:robot.get_transform(s+'_tool0', robot.base_link)@A for s,A in attachments.items()}
    torques = []
    for side in ('left', 'right'):
        for i in range(1, 8):
            joint = robot.joint_map[f'{side}_joint{i}'];T = robot.get_transform(joint.child, robot.base_link)
            axis = T[:3,:3]@joint.axis;value = 0.
            for name in descendants(joint.child):
                inertial = robot.link_map[name].inertial
                if inertial is not None:
                    C = robot.get_transform(name, robot.base_link)@inertial.origin
                    value -= axis@np.cross(C[:3,3]-T[:3,3], [0.,0.,-9.81*inertial.mass])
            if side in box:value -= axis@np.cross(box[side][:3,3]-T[:3,3], [0.,0.,-98.1])
            torques.append(value)
    return np.asarray(torques)


def main():
    report = json.loads((OUT/'suffix_probe.json').read_text())
    interface, picks, start, A_right = inputs()
    current_hashes = hashes()
    assert current_hashes == report['input_hashes_before'] == report['input_hashes_after']
    result = dict(success=report['success'], runtime=dict(sys_executable=sys.executable,
        curobo_module_path=importlib.util.find_spec('curobo').origin,
        curobo_version=importlib.metadata.version('nvidia-curobo')), hashes=current_hashes)
    if not report['success']:
        result.update(valid=False, classification=report.get('failure_class'), failure=report.get('failure'),
            scope='Failed or partial geometry remains failed; CPU checks do not upgrade it.')
        (OUT/'suffix_cpu_audit.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2));return
    with np.load(OUT/'suffix_geometry.npz', allow_pickle=False) as z:data={k:z[k] for k in z.files}
    phases = ('upper_park', 'lower_approach', 'lower_contact', 'lower_extract', 'final_transport')
    assert np.array_equal(data['upper_park'][0], start)
    assert np.array_equal(data['right_attachment'], A_right)
    assert data['physical_box_ids'].tolist() == ['left', 'right']
    robot = yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf', load_meshes=False)
    dynamics = yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/dynamics.urdf', load_meshes=False)
    previous = None;seams = {};boxes = {s:[] for s in picks};previous_box = None;pose_seams = {};fk_error=0.;gravity_error=0.
    source_left = np.eye(4);source_left[:3,3] = picks['left'].center_m
    for key in phases:
        rows=data[key];assert rows.ndim==2 and rows.shape[1]==15 and np.isfinite(rows).all()
        seam=0. if previous is None else float(np.max(np.abs(previous-rows[0])));assert seam<=1e-5
        seams[key]=seam;previous=rows[-1]
        tools=fk(robot,rows)
        actual={'right':tools['right']@A_right,
            'left':tools['left']@data['left_attachment'] if key in ('lower_extract','final_transport') else np.repeat(source_left[None],len(rows),axis=0)}
        if previous_box is not None:
            pose_seams[key]={s:float(np.max(np.abs(actual[s][0]-previous_box[s]))) for s in picks}
            assert max(pose_seams[key].values())<=2e-5
        previous_box={s:T[-1] for s,T in actual.items()}
        for side,T in actual.items():
            assert np.max(np.abs(T-data[key+'_'+side+'_box']))<=2e-5
            fk_error=max(fk_error,float(np.max(np.abs(T-data[key+'_'+side+'_box']))))
            if side=='right' or key=='final_transport':boxes[side].append(T)
        attachments={'right':A_right}
        if key in ('lower_extract','final_transport'):attachments['left']=data['left_attachment']
        for i in sorted(set((0,len(rows)//2,len(rows)-1))):
            error=float(np.max(np.abs(gravity(dynamics,rows[i],attachments)-data[key+'_torques_nm'][i])))
            gravity_error=max(gravity_error,error);assert error<=.01
    low=data['lower_extract_left_box'];pull=low[-1,:3,3]-low[0,:3,3]
    assert np.linalg.norm(pull-[-.3,0.,0.])<=.002
    vector=low[:,:3,3]-low[0,:3,3];assert np.linalg.norm(vector[:,1:],axis=1).max()<=.002
    angle=np.rad2deg(Rotation.from_matrix(low[0,:3,:3].T@low[:,:3,:3]).magnitude()).max();assert angle<=1.
    held=data['lower_extract_right_box'];assert np.linalg.norm(held[:,:3,3]-held[0,:3,3],axis=1).max()<=.002
    final=np.deg2rad(np.r_[CARRY_DEGREES,-CARRY_DEGREES]);angle_error=float(np.max(np.abs(data['final_transport'][-1,1:]-final)));assert angle_error<=1e-5
    process=source_slot_process_checks({s:source_slot_observation(picks[s],np.concatenate(T)) for s,T in boxes.items()});assert process['valid'],process
    footprints={}
    for side,A in [('left',data['left_attachment']),('right',A_right)]:
        local=np.linalg.inv(A);caps=contact_cap_spheres(side);points=caps[:,:3]@local[:3,:3].T+local[:3,3]
        axis=0 if side=='left' else 1;tangent=[i for i in range(3) if i!=axis];half=np.asarray(picks[side].dimensions_m)/2
        excess=float(np.max(np.abs(points[:,tangent])+caps[:,3,None]-half[tangent]));assert excess<=.001;footprints[side]=excess
    result.update(valid=True,q_seams=seams,box_pose_seams=pose_seams,fk_max_error=fk_error,actual_low_extraction_m=pull.tolist(),
        extraction_orientation_error_deg=float(angle),final_14angle_max_error_rad=angle_error,final_updown_m=float(data['final_transport'][-1,0]),
        independent_static_gravity_max_error_nm=gravity_error,footprint_excess_m=footprints,source_process=process,
        scope='CPU identity, actual FK/attachments, all box/q seams, .300m extraction, preserved right-held box, actual cap footprints, source histories and14-angle contract. Full sampled collision/static masks remain GPU evidence; no native timing.')
    (OUT/'suffix_cpu_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()
