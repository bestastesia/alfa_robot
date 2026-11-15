import gzip
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from v3_frozen_cycle import PROFILE,FrozenCyclePlanner
from v3_frozen_cycle_validate import OUTPUT,check_cycle
from v3_wall_ik_benchmark import ACTIVE_JOINTS


def test_export_deduplication_does_not_hide_wrong_home_start():
    home=dict.fromkeys(ACTIVE_JOINTS,0.)
    placed=np.zeros(15)
    placed[1]=.1
    first=placed.copy()
    first[1]-=.005
    result=dict(success=False,joint_names=ACTIVE_JOINTS,frames=[placed.tolist(),placed.tolist(),first.tolist()],
        phases=['conveyor_place','release','return_home'],conveyor_place_lm=dict(success=True,rrt_fallback=False),
        conveyor_home_start_joints=placed.tolist())
    assert check_cycle(result,home)['actual_lm_to_home_join']==0.
    result['conveyor_home_start_joints']=np.zeros(15).tolist()
    with pytest.raises(AssertionError,match='actual LM endpoint'):check_cycle(result,home)


def test_lm_failure_cannot_have_a_home_suffix():
    home=dict.fromkeys(ACTIVE_JOINTS,0.)
    result=dict(success=False,joint_names=ACTIVE_JOINTS,frames=[np.zeros(15).tolist()]*2,
        phases=['conveyor_place','return_home'],conveyor_place_lm=dict(success=False,rrt_fallback=False))
    with pytest.raises(AssertionError,match='absent/failed'):check_cycle(result,home)


def test_frozen_profile_does_not_construct_numerical_placement_ik():
    planner=object.__new__(FrozenCyclePlanner)
    assert planner.conveyor_ik_solver({'left':24,'right':20}) is None


def test_full_cycle_records_keep_payload_and_actual_terminal_state():
    paths=sorted(OUTPUT.glob('[0-9][0-9][0-9].json.gz'))
    if not paths:pytest.skip('full-cycle recordings unavailable')
    for path in paths:
        result=json.loads(gzip.decompress(path.read_bytes()))
        assert result['cycle_profile']==PROFILE
        assert result['strategy']['placement']=='sequential15axis LM128points32seeds32iterations'
        assert result['strategy']['payload_model']=='frozen40'
        assert result['strategy']['rrt_placement_fallback'] is False
        check_cycle(result,result['visual_home'])
        phases=np.asarray(result['phases'])
        payload=np.asarray(result['payload'])
        assert payload[np.isin(phases,['extract','transport','conveyor_place'])].all()
        assert not payload[phases=='return_home'].any()
        lm=result.get('conveyor_place_lm')
        if not lm:continue
        assert lm['reference_samples']==128 and lm['seeds_per_point']==32 and lm['iterations']==32
        assert not lm['rrt_fallback'] and not lm['main_optimizer_ran']
        if not lm['success']:
            assert not np.isin(phases,['release','return_home','base_return']).any()
        if len(result['task'])==1:
            side=next(iter(result['task']))
            idle=[result['joint_names'].index(name) for name in ACTIVE_JOINTS if name.startswith('right_' if side=='left' else 'left_')]
            rows=np.asarray(result['frames'])[phases=='conveyor_place'][:,idle]
            if len(rows):assert np.max(np.ptp(rows,axis=0))<=1e-5


def test_lm_reference_with_independent_urdf():
    import yourdfpy
    from scipy.spatial.transform import Rotation,Slerp
    samples=[OUTPUT/f'{index:03d}.json.gz' for index in (0,64,75,116)]
    if not all(path.exists() for path in samples):pytest.skip('representative recordings unavailable')
    model=yourdfpy.URDF.load(OUTPUT.parent/'zero_model_candidate_20261018/robot_rezero_candidate.urdf',load_meshes=False)
    for path in samples:
        result=json.loads(gzip.decompress(path.read_bytes()))
        names=result['joint_names']
        ids=[names.index(name) for name in ACTIVE_JOINTS]
        rows=np.asarray(result['frames'])
        phases=np.asarray(result['phases'])
        placed=rows[phases=='conveyor_place']
        navigation=rows[np.flatnonzero(phases=='base_reposition')[-1]]
        base=np.eye(4)
        base[:3,:3]=Rotation.from_euler('z',navigation[names.index('base_yaw')]).as_matrix()
        base[:2,3]=[navigation[names.index('base_x')],navigation[names.index('base_y')]]
        model.update_cfg(dict(zip(ACTIVE_JOINTS,navigation[ids])))
        attachments={side:np.asarray(matrix) for side,matrix in result['visual_tool_to_box'].items()}
        starts={side:base@model.get_transform(side+'_tool0',model.base_link)@matrix for side,matrix in attachments.items()}
        progress=result['conveyor_place_lm']['progress']
        if len(progress)==len(placed)+1:progress=progress[1:]
        assert len(progress)==len(placed)
        for row,fraction in zip(placed,progress):
            model.update_cfg(dict(zip(ACTIVE_JOINTS,row[ids])))
            for side,attachment in attachments.items():
                actual=base@model.get_transform(side+'_tool0',model.base_link)@attachment
                goal=result['conveyor_box_targets'][side]
                expected=starts[side][:3,3]+fraction*(np.asarray(goal['position'])-starts[side][:3,3])
                rotations=Rotation.from_matrix(np.stack([starts[side][:3,:3],Rotation.from_quat(np.roll(goal['quaternion_wxyz'],-1)).as_matrix()]))
                assert np.linalg.norm(actual[:3,3]-expected)<=.00201
                assert (Slerp([0.,1.],rotations)(fraction).inv()*Rotation.from_matrix(actual[:3,:3])).magnitude()<=np.deg2rad(1.001)
