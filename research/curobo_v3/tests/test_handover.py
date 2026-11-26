"""CPU check: adapt the real accepted cached chain to the fresh interface, then reject tampering."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import hashlib
import json
import sys
import tempfile

import numpy as np
import yourdfpy

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from curobo_core.handover import STAGES,NAMES,verify_handover_result,handover_native_samples,replay_handover_boxes
from curobo_core.box_pick import BoxPick
from curobo_core.scene import SceneSnapshot,SceneObject,AttachedObject,Pose
from curobo_core.adapter import matrix_pose


def load(path):
    with np.load(path,allow_pickle=False) as data:return {k:data[k].copy() for k in data.files}


def cached_record():
    base=ROOT/'generated/two_box_layout'
    p=base/'joint_contract_20261124_prefix_native/numeric_fix/attempt_02/prefix'
    h=base/'joint_contract_20261124_transfer_native/patched_06'
    s=base/'joint_contract_20261124_suffix_native/attempt_01'
    prefix=json.loads((p/'prefix_report.json').read_text());transfer=json.loads((h/'native_report.json').read_text())
    suffix=json.loads((s/'native_result.json').read_text())
    assert prefix['success'] and transfer['success'] and suffix['native_success']
    raw=deepcopy(prefix['request']);canonical=deepcopy(raw)
    # The adapter may expand defaults; equality is physical, not JSON spelling.
    raw.pop('front_distance_m')
    for box in raw['boxes']:box['carry_joint_home']=True;box.pop('side_min_height_m')
    pg=load(base/'joint_contract_20261123/transfer_owner_transport_geometry.npz')
    hg=load(base/'joint_contract_20261124_transfer/transfer_geometry.npz')
    sg=load(base/'joint_contract_20261124_suffix/suffix_geometry.npz')
    geometry=[pg[k] for k in STAGES[:4]]+[hg[k] for k in STAGES[4:8]]+[sg[k] for k in STAGES[8:]]
    files=[Path(stage['trajectory_file']) for stage in prefix['initial']['stages']]+[p/'transport_native.npz']
    files += [h/(stage+'.npz') for stage in STAGES[4:8]]+[Path(stage['trajectory_file']) for stage in suffix['stages']]
    picks={box['side']:BoxPick(**dict(box,carry_joint_home=True)) for box in canonical['boxes']}
    template=SceneSnapshot.from_dict(prefix['physical_snapshot'])
    environment=tuple(o for o in template.objects if o.object_id not in ('wall_box_00','pending_original_left'))
    specs=[]
    for i,(stage,q) in enumerate(zip(STAGES,geometry)):
        objects=list(environment);attachments=[]
        for identity,side,parent,A in (
            ('wall_box_01','right',None if i<2 else 'left_tool0' if i<=5 else 'right_tool0',hg['left_attachment'] if i<=5 else hg['right_measured_attachment']),
            ('wall_box_00','left','left_tool0' if i>=11 else None,sg['left_attachment'])):
            pick=picks[side]
            if parent is None:objects.append(SceneObject(identity,pick.dimensions_m,Pose(pick.center_m)))
            else:attachments.append(AttachedObject(identity,pick.dimensions_m,parent,matrix_pose(A)))
        state=template.state.with_positions(dict(zip(NAMES,q[0])))
        scene=replace(template,state=state,objects=tuple(objects),attachments=tuple(attachments)).to_dict()
        specs.append(dict(stage_id=stage,scene=scene))
    task={side:{key:value for key,value in row.items() if key in BoxPick.__dataclass_fields__} for side,row in suffix['task'].items()}
    final_q=load(files[-1])['position'][-1]
    for row in task.values():row['carry_updown_m']=float(final_q[0])
    timing=dict(success=True,stages=[dict(stage_id=stage,trajectory_file=str(path),success=True,physical_success=True)
        for stage,path in zip(STAGES,files)])
    timing['stages'][-1]['trajectory_optimization']=suffix['stages'][-1]['pieces'][0]['audit']['trajectory_optimization']
    return dict(workflow='handover_same_side',success=True,request=raw,task=task,joint_names=NAMES,
        frames=np.concatenate(geometry).tolist(),phases=[name for name,q in zip(STAGES,geometry) for _ in q],
        stage_interface=dict(request=canonical,joint_names=NAMES,stages=specs,support_transfer=dict(
            physical_id='wall_box_01',mass_kg=10.,parent_before='left_tool0',parent_after='right_tool0',
            alpha_key='support_alpha',stationary_only=True)),timing=timing)


def test_handover():
    gpu_modules={name for name in sys.modules if name.split('.')[0] in ('torch','curobo','cuda')}
    from v3_box_layout import verify_layout_result
    from experiments.two_box_pick_20261119.replay_native import native_samples
    from curobo_core.ordered_pick import replay_box_poses
    record=cached_record()
    original_hashes={s['trajectory_file']:hashlib.sha256(Path(s['trajectory_file']).read_bytes()).hexdigest()
        for s in record['timing']['stages']}
    check=verify_layout_result(record)
    assert check['valid'] and check['process_validation']['valid'] and check['samples']==56011
    assert abs(check['duration_s']-29.213902354240417)<1e-8
    assert max(v['max_error_nm'] for v in check['mass_switches'])<2e-5
    assert not record['timing']['stages'][-1]['trajectory_optimization']['optimizer_success']
    times,q,labels=native_samples(record)
    assert q.shape==(len(times),15) and len(times)==len(labels) and np.all(np.diff(times)>0)
    assert labels[0]==STAGES[0] and labels[-1]==STAGES[-1]
    robot=yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf',load_meshes=False)
    for phase in ('owner_transport','support_transfer','donor_release'):
        index=STAGES.index(phase)
        rows=load(record['timing']['stages'][index]['trajectory_file'])['position']
        robot.update_cfg(dict(zip(NAMES,rows[len(rows)//2])))
        boxes=replay_handover_boxes(robot,record,phase)
        assert set(boxes)=={'wall_box_00','wall_box_01'} and all(value.shape==(4,4) for value in boxes.values())
        assert np.array_equal(boxes['wall_box_00'][:3,3],[1.1680520510673524,.85,.8])
        owner='left_tool0' if phase!='donor_release' else 'right_tool0'
        attached=record['stage_interface']['stages'][index]['scene']['attachments'][0]
        from curobo_core.adapter import pose_matrix
        expected=robot.get_transform(owner,robot.base_link)@pose_matrix(Pose(**attached['tool_to_object']))
        np.testing.assert_allclose(boxes['wall_box_01'],expected,atol=1e-12)
        display=replay_box_poses(robot,record,phase)
        assert set(display)=={'left','right'}
        np.testing.assert_allclose(display['right'],boxes['wall_box_01'],atol=1e-12)
    # Historical physical labels are independent of the actual final arm owners.
    relabeled=deepcopy(record)
    flip_side={'left':'right','right':'left'};flip_id={'wall_box_00':'wall_box_01','wall_box_01':'wall_box_00'}
    relabeled['task']={flip_side[side]:dict(row,side=flip_side[side]) for side,row in relabeled['task'].items()}
    for document in (relabeled['request'],relabeled['stage_interface']['request']):
        for box in document['boxes']:box['side']=flip_side[box['side']]
    for stage in relabeled['stage_interface']['stages']:
        for obj in stage['scene']['objects']+stage['scene']['attachments']:
            if obj['object_id'] in flip_id:obj['object_id']=flip_id[obj['object_id']]
    relabeled['stage_interface']['support_transfer']['physical_id']='wall_box_00'
    assert verify_handover_result(relabeled)['valid']
    def rejected(value):
        try:verify_handover_result(value)
        except ValueError:return
        raise AssertionError('tampered handover was accepted')
    bad=deepcopy(record);bad['stage_interface']['support_transfer']['physical_id']='wall_box_00';rejected(bad)
    bad=deepcopy(record);bad['stage_interface']['stages'][6]['scene']['attachments'][0]['tool_to_object']['position']= [.02,0.,.2325];rejected(bad)
    bad=deepcopy(record)
    for box in bad['stage_interface']['request']['boxes']:box['center_m'][1]+=.01
    rejected(bad)
    bad=deepcopy(record);bad['timing']['stages'][-1]['physical_success']=False;rejected(bad)
    with tempfile.TemporaryDirectory() as folder:
        path=Path(folder)/'wrong_mass_jump.npz';arrays=load(record['timing']['stages'][2]['trajectory_file'])
        arrays['torque'][0,0]+=.1;np.savez_compressed(path,**arrays)
        bad=deepcopy(record);bad['timing']['stages'][2]['trajectory_file']=str(path);rejected(bad)
    assert all(hashlib.sha256(Path(path).read_bytes()).hexdigest()==digest for path,digest in original_hashes.items())
    assert gpu_modules=={name for name in sys.modules if name.split('.')[0] in ('torch','curobo','cuda')}


if __name__=='__main__':
    test_handover()
    print('handover CPU check passed: real 13-stage chain, semantic request defaults, source/A/mass/physical-gate tamper rejection; no GPU imports')
