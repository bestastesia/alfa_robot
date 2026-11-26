"""CPU-only: reopen unchanged native leaf arrays, validate suffix identity/FK/qva/source/limits."""
from pathlib import Path
from dataclasses import replace
import hashlib
import importlib.metadata
import importlib.util
import json
import sys
import numpy as np
import yourdfpy

ROOT=Path(__file__).resolve().parents[4]
HERE=Path(__file__).resolve().parent
OUT=ROOT/'generated/two_box_layout/joint_contract_20261124_suffix_native/attempt_01'
SUFFIX=ROOT/'generated/two_box_layout/joint_contract_20261124_suffix'
sys.path[:0]=[str(ROOT/'tools'),str(HERE.parent/'transfer_suffix')]
from curobo_core.box_pick import BoxPick,timed_path_checks
from curobo_core.native_transport import concatenate_native,native_seam,cpu_source_observations,reference_box_poses,urdf_box_poses,source_progress
from curobo_core.layout_contract import CARRY_DEGREES
from curobo_core.ordered_pick import cartesian_check
from audit_suffix import NAMES,gravity


def load(path):
    with np.load(path,allow_pickle=False) as z:return {k:z[k].copy() for k in z.files}


def main():
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--attempt',default='attempt_01');args=parser.parse_args()
    global OUT
    OUT=OUT.parent/args.attempt
    result=json.loads((OUT/'native_result.json').read_text());policy=json.loads((ROOT/'config/fast_three_layers.json').read_text())
    source=load(SUFFIX/'suffix_geometry.npz')
    robot=yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf',load_meshes=False)
    dynamics=yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/dynamics.urdf',load_meshes=False)
    picked={}
    for side,values in result['task'].items():
        fields={key:value for key,value in values.items() if key in BoxPick.__dataclass_fields__}
        picked[side]=BoxPick(**fields)
    class HeldPick:
        def __init__(self,side):self.side=side
        def __getattr__(self,key):return getattr(picked[self.side],key)
        def tool_to_box(self):return source[self.side+'_attachment']
    picks={s:HeldPick(s) for s in picked}
    assert policy['planning_limit_scale']==.995
    assert result['input_hashes_before']==result['input_hashes_after']
    assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==digest for p,digest in result['input_hashes_after']['old_files'].items())
    refs={'right':0.};cleared={'right':True};arrays=[];stage_arrays=[];records=[];previous=None;gravity_error=0.;endpoint_box=None;pose_seams=[]
    for record in result['stages']:
        pieces=[];phase=record['physical_contract']['phase'];side_set=tuple(record['physical_contract']['payload_sides'])
        for leaf in record['pieces']:
            actual=load(leaf['trajectory_file'])
            rows=source[record['stage_id']]
            checks=timed_path_checks(actual,rows[leaf['start']],rows[leaf['end']],policy);assert checks['valid'],checks
            if previous is not None:assert native_seam(previous,actual)['valid']
            else:assert np.max(np.abs(actual['position'][0]-source['upper_park'][0]))<=1e-5
            for i,name in enumerate(NAMES):
                bound=robot.joint_map[name].limit
                assert np.all(actual['position'][:,i]>=bound.lower) and np.all(actual['position'][:,i]<=bound.upper),name
            assert leaf['audit']['valid'] and leaf['audit']['independent_rnea']['valid']
            static=load(leaf['audit']['static_rnea_file'])['torque']
            limits=np.array(policy['arm_continuous_torque_nm']*2)*.995
            assert np.all(np.abs(static)<=limits)
            attachments={s:source[s+'_attachment'] for s in side_set}
            for i in (0,len(actual['times'])//2,len(actual['times'])-1):
                difference=float(np.max(np.abs(gravity(dynamics,actual['position'][i],attachments)-static[i])))
                gravity_error=max(gravity_error,difference);assert difference<=.01
            before=reference_box_poses(robot,actual['position'][0],NAMES,attachments)
            after=reference_box_poses(robot,actual['position'][-1],NAMES,attachments)
            if endpoint_box is not None:
                delta={s:float(np.max(np.abs(before[s]-endpoint_box[s]))) for s in before if s in endpoint_box}
                assert max(delta.values(),default=0.)<=2e-5;pose_seams.append(delta)
            endpoint_box=after
            source_sides=tuple(record['physical_contract']['source_sides'])
            if source_sides:
                observations=cpu_source_observations(robot,actual['position'],NAMES,{s:picks[s] for s in source_sides})
                passed,_,next_clear,process=source_progress(observations,refs,cleared);assert passed,process;cleared.update(next_clear)
            previous=actual;pieces.append(actual);arrays.append(actual)
            records.append(dict(stage_id=record['stage_id'],piece=leaf['name'],checks=checks,samples=len(actual['times'])))
        if not record['success']:break
        merged,seams=concatenate_native(pieces);saved=load(record['trajectory_file'])
        assert all(np.array_equal(merged[k],saved[k]) for k in saved)
        stage_arrays.append(merged)
        if phase=='extract':
            observation=cpu_source_observations(robot,merged['position'][-1:],NAMES,{'left':picks['left']})['left']
            refs['left']=observation['initial_slot_overlap_m'];cleared['left']=observation['first_clear_frame'] is not None
    output=dict(valid=True,native_success=result['native_success'],accepted_native_leaves=len(arrays),accepted_stage_count=len(stage_arrays),
        runtime=dict(sys_executable=sys.executable,curobo_module_path=importlib.util.find_spec('curobo').origin,curobo_version=importlib.metadata.version('nvidia-curobo')),
        gravity_crosscheck_error_nm=gravity_error,box_pose_seams=pose_seams,source_cleared=cleared,checks=records,
        scope='CPU array reconstruction, qva/bounds/dynamic-limit checks, exact measured attachments/FK/source history and static RNEA spot checks. Full collision/OBB and independent dynamic RNEA are actual GPU native leaf reports.')
    if arrays:
        merged,seams=concatenate_native(arrays)
        if result.get('merged_trajectory_file'):
            saved=load(result['merged_trajectory_file']);assert all(np.array_equal(merged[k],saved[k]) for k in saved)
        output.update(qva_seams=seams,native_samples=len(merged['times']),motion_seconds=float(merged['times'][-1]))
    if result['native_success']:
        assert len(stage_arrays)==5 and all(cleared.get(s,False) for s in picks)
        final=stage_arrays[-1]['position'][-1]
        angle_error=float(np.max(np.abs(final[1:]-np.deg2rad(np.r_[CARRY_DEGREES,-CARRY_DEGREES]))));assert angle_error<=1e-5
        extract=stage_arrays[3];boxes=urdf_box_poses(robot,extract['position'],NAMES,{s:picks[s].tool_to_box() for s in picks})
        pull=boxes['left'][-1,:3,3]-boxes['left'][0,:3,3];assert np.linalg.norm(pull-[-.3,0,0])<=.002
        for side in picks:
            start=reference_box_poses(robot,extract['position'][0],NAMES,{side:picks[side].tool_to_box()})[side]
            end=start.copy()
            if side=='left':end[0,3]-=.3
            assert cartesian_check(boxes[side],start,end)['valid']
        first_low=boxes['left'][0];assert np.linalg.norm(first_low[:3,3]-picks['left'].center_m)<=2e-5
        output.update(final_14angle_max_error_rad=angle_error,final_updown_m=float(final[0]),actual_low_extraction_m=pull.tolist())
    (OUT/'cpu_audit.json').write_text(json.dumps(output,indent=2)+'\n');print(json.dumps(output,indent=2))


if __name__=='__main__':main()
