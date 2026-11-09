import gzip
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import torch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import v3_frozen_cycle as frozen
from v3_lm_conveyor import SequentialPlacementLm
from v3_restored_analytic_cycle import RestoredAnalyticPlanner
from v3_wall_ik_benchmark import ACTIVE_JOINTS


@pytest.fixture
def extraction(monkeypatch):
    planner=object.__new__(frozen.FrozenCyclePlanner)
    planner.selected_task={'left':23,'right':22}
    planner.placement_lm_solvers={}
    attachment=np.eye(4)
    attachment[:3,3]=[.1,.2,.3]
    checker=SimpleNamespace(task_tool_to_box={side:attachment.copy() for side in planner.selected_task},guidance_payload=True)
    planner.checker=Mock(return_value=checker)
    tool=np.eye(4)
    tool[:3,:3]=[[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]]
    tool[:3,3]=[.4,.5,1.2]
    planner._tool_poses=Mock(return_value={side:tool.copy() for side in planner.selected_task})
    frames=np.zeros((2,15),dtype=np.float32)
    frames[-1,0]=.04
    engine=SimpleNamespace(setup_ms=12.,plan=Mock(return_value=dict(success=True,frames=frames.tolist(),rrt_fallback=False)))
    factory=Mock(return_value=engine)
    monkeypatch.setattr(frozen,'SequentialPlacementLm',factory)
    tensor=torch.tensor
    monkeypatch.setattr(frozen.torch,'tensor',lambda *args,**kwargs:tensor(*args,**dict(kwargs,device='cpu')))
    return planner,checker,engine,factory


@pytest.mark.parametrize('direction',[None,[-1.,0.,0.],[0.,1.,0.],[0.,0.,1.],[.6,-.8,0.]])
def test_engine_receives_loaded_case_and_existing_frame_direction(extraction,direction):
    planner,checker,engine,factory=extraction
    if direction is not None:planner.extraction_direction=direction
    contact=np.zeros(15)
    frames,detail=planner.analytic_extract(contact,object(),SimpleNamespace(checker=checker))
    factory.assert_called_once_with(planner,seeds=32,iterations=32,samples=128,active_sides=('left','right'))
    case=engine.plan.call_args.args[0]
    assert case['checker'] is checker
    planner.checker.assert_not_called()
    for side in planner.selected_task:
        expected=planner._tool_poses.return_value[side]@checker.task_tool_to_box[side]
        np.testing.assert_allclose(case['starts'][side],expected)
        np.testing.assert_allclose(case['ends'][side][:3,3]-expected[:3,3],.35*np.asarray(direction if direction is not None else [-1.,0.,0.]))
        np.testing.assert_array_equal(case['ends'][side][:3,:3],expected[:3,:3])
    assert frames[-1,0]==pytest.approx(.04)
    assert detail['updown_free'] and not detail['rrt_fallback']
    assert planner.placement_solver() is engine
    assert factory.call_count==1


@pytest.mark.parametrize('success',[True,False])
def test_lazy_deferral_caches_success_and_failure_once(extraction,success):
    planner,checker,engine,factory=extraction
    engine.plan.return_value['success']=success
    contact=np.zeros(15)
    key=tuple(contact)
    planner._lazy_preparing=True
    planner._contact_indices={key:7}
    planner._allowed_contacts=set()
    planner._extraction_cache={}
    planner.lazy_timings={'extraction_work_ms':0.}
    assert planner.analytic_extract(contact,None,None)==(None,'deferred to next historical8/16/32/64 batch')
    factory.assert_not_called()
    assert not planner._extraction_cache
    assert planner.lazy_timings['extraction_work_ms']==0.
    planner._allowed_contacts={7}
    first=planner.analytic_extract(contact,None,None)
    elapsed=planner.lazy_timings['extraction_work_ms']
    assert elapsed>0.
    assert planner.analytic_extract(contact+1e-9,None,None) is first
    assert engine.plan.call_count==1
    assert planner.lazy_timings['extraction_work_ms']==elapsed
    assert (first[0] is not None)==success
    planner._lazy_preparing=False
    planner.analytic_extract(contact,None,None)
    assert engine.plan.call_count==2


def test_single_arm_extraction_and_normal_approach_dispatch(extraction,monkeypatch):
    planner,checker,engine,factory=extraction
    planner.selected_task={'right':22}
    checker.task_tool_to_box.pop('left')
    planner.cartesian_single(np.zeros(15),np.linspace(.01,.35,35))
    factory.assert_called_once_with(planner,seeds=32,iterations=32,samples=128,active_sides=('right',))
    planner.checker.assert_called_once_with(planner.selected_task,payload=True)
    normal=Mock(return_value='normal approach')
    monkeypatch.setattr(RestoredAnalyticPlanner,'cartesian_single',normal)
    assert planner.cartesian_single(np.zeros(15),[.01],True)=='normal approach'
    assert engine.plan.call_count==1
    assert frozen.FrozenCyclePlanner.predict_endpoint is RestoredAnalyticPlanner.predict_endpoint


def test_cycle_reports_lm_extraction_without_claiming_strict_edges(extraction,monkeypatch):
    planner,checker,engine,factory=extraction
    monkeypatch.setattr(RestoredAnalyticPlanner,'full_cycle',lambda *args:dict(strict_cartesian_extraction=True))
    result=planner.full_cycle(planner.selected_task)
    assert result['cycle_profile']==frozen.PROFILE
    assert 'LM35cm' in result['strategy']['extraction']
    assert not result['strategy']['rrt_extraction_fallback']
    assert not result['strict_cartesian_extraction']
    assert 'ranking only' in result['endpoint_selection_policy']


@pytest.mark.parametrize('enabled,success,expected',[(False,False,0),(True,True,0),(True,False,1)])
def test_optional_failed_reviews_preserve_actual_prefix_without_accepting_it(extraction,monkeypatch,enabled,success,expected):
    planner,checker,engine,factory=extraction
    planner.record_lm_extraction_candidates=enabled
    failed=np.full((2,15),.2).tolist()
    engine.plan.return_value.update(success=success,failed_candidates=failed,
        failure=None if success else 'interpolated edge collision',progress=[0.,.25],completed_progress=.25,
        point_reports=[dict(edge_rejections=2)])
    contact=np.zeros(15)
    def cycle(*args):
        frames,detail=planner.analytic_extract(contact,None,None)
        assert (frames is not None)==success
        assert detail['frames'] is None
        return dict(success=success,frames=[])
    monkeypatch.setattr(RestoredAnalyticPlanner,'full_cycle',cycle)
    result=planner.full_cycle(planner.selected_task)
    reviews=result['lm_extraction_candidate_reviews']
    assert len(reviews)==expected
    assert result['frames']==[]
    if expected:
        review=reviews[0]
        assert review['frames']==engine.plan.return_value['frames']
        assert review['failed_candidates']==failed
        assert review['failure']=='interpolated edge collision'
        assert review['progress']==[0.,.25] and review['completed_progress']==.25
        assert review['point_reports']==[dict(edge_rejections=2)]
        assert review['contact']==contact.tolist() and review['start']==contact.tolist()
        assert review['joint_names']==ACTIVE_JOINTS
        engine.plan.return_value['frames'][1][0]=99.
        failed[0][0]=99.
        assert review['frames'][1][0]!=99. and review['failed_candidates'][0][0]!=99.
    monkeypatch.setattr(RestoredAnalyticPlanner,'full_cycle',lambda *args:dict(success=False))
    assert planner.full_cycle(planner.selected_task)['lm_extraction_candidate_reviews']==[]
    assert len(reviews)==expected


def test_failed_reviews_obey_cap_and_lazy_cache(extraction):
    planner,checker,engine,factory=extraction
    planner.record_lm_extraction_candidates=True
    engine.plan.return_value.update(success=False,failed_candidates=[],failure='starting state collision',
        progress=[0.],completed_progress=0.)
    planner._lazy_preparing=True
    planner._contact_indices={tuple(np.zeros(15)):0}
    planner._allowed_contacts=set()
    planner._extraction_cache={}
    planner.lazy_timings={'extraction_work_ms':0.}
    planner.lm_extraction_candidate_reviews=[]
    planner.analytic_extract(np.zeros(15),None,None)
    assert planner.lm_extraction_candidate_reviews==[]
    planner._allowed_contacts={0}
    planner.analytic_extract(np.zeros(15),None,None)
    planner.analytic_extract(np.zeros(15),None,None)
    assert len(planner.lm_extraction_candidate_reviews)==1 and engine.plan.call_count==1
    planner._lazy_preparing=False
    for index in range(1,66):
        frames,detail=planner.analytic_extract(np.full(15,index),None,None)
        assert frames is None and not detail['success']
    assert len(planner.lm_extraction_candidate_reviews)==64
    assert planner.lm_extraction_candidate_reviews[-1]['contact']==[63.]*15
    assert engine.plan.call_count==66


@pytest.mark.skipif(os.environ.get('RUN_LM_EXTRACTION_GPU')!='1',reason='opt-in single-candidate CUDA integration')
def test_real_lm_engine_extracts_one_current_model_contact(monkeypatch):
    from v3_batched_loaded_search import GpuValidity
    from v3_lm_conveyor import box_poses

    source=Path(__file__).resolve().parents[1]/'generated/search_accepted_cycle_20261105/000.json.gz'
    if not torch.cuda.is_available() or not source.exists():pytest.skip('CUDA and current-model contact fixture required')
    record=json.loads(gzip.decompress(source.read_bytes()))
    planner=frozen.FrozenCyclePlanner()
    planner.configure_task(record['task'])
    columns=[record['joint_names'].index(name) for name in ACTIVE_JOINTS]
    contact=np.asarray(record['frames'])[np.flatnonzero(np.asarray(record['phases'])=='contact_approach')[-1],columns]
    checker=planner.checker(planner.selected_task,payload=True)
    engine=planner.placement_solver()
    assert isinstance(engine,SequentialPlacementLm)
    assert engine.solver.dof==15
    solve=Mock(wraps=engine.solver.solve_single)
    monkeypatch.setattr(engine.solver,'solve_single',solve)
    def forbidden(*args,**kwargs):pytest.fail('extraction used forbidden fallback')
    monkeypatch.setattr(engine.owner,'solve_pose',forbidden)
    monkeypatch.setattr(planner,'verified_rrt',forbidden)
    frames,detail=planner.analytic_extract(contact,None,SimpleNamespace(checker=checker))
    assert solve.call_count>0
    assert detail['success'],detail.get('failure')
    assert detail['reference_samples']==128 and not detail['rrt_fallback']
    assert np.ptp(frames[:,0])>.001
    assert GpuValidity(checker,check_ground=True,scene_penetration_tolerance_m=.001).mask(torch.tensor(frames,device='cuda',dtype=torch.float32)).all()
    poses=box_poses(checker,frames[[0,-1]],checker.task_tool_to_box)
    direction=np.asarray(getattr(planner,'extraction_direction',[-1.,0.,0.]))
    for side,matrices in poses.items():
        np.testing.assert_allclose(matrices[1,:3,3]-matrices[0,:3,3],.35*direction,atol=.0021)
        config=checker.kinematics.config.kinematics_config
        assert len(config.get_sphere_index_from_link_name(side+'_link7'))-len(planner.robot['kinematics']['collision_spheres'][side+'_link7'])==40
    print(dict(success=detail['success'],lm_calls=solve.call_count,total_ms=detail['total_ms'],updown_range_m=float(np.ptp(frames[:,0]))))
