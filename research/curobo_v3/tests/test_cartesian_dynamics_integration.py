"""CPU checks for the PR73 / PR71 timing boundary; no CUDA context required."""
import copy
import sys
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import v3_cartesian_transport_planner as cartesian
from curobo_core.contracts import ACTIVE_JOINTS
from v3_timed_single_arm_planner import TimedSingleArmPlanner


def planner_and_result(source='cartesian'):
    planner = object.__new__(cartesian.CartesianTransportPlanner)
    planner._timed_joint_names = list(ACTIVE_JOINTS)
    planner._timed_limits = (np.ones(15), np.ones(15)*2, np.ones(15)*10)
    planner.trajectory_sample_dt = .025
    planner.torque_model = None
    planner.use_trajopt = False
    planner.transport_timing = 'toppra'
    planner.transport_search_mode = 'cartesian'
    planner.cartesian_options = {}
    planner.snapshot = None
    planner.configure_task = lambda boxes: None
    planner._timed_phase_valid = lambda *args: True
    rows = np.zeros((5, 15))
    rows[:, 8] = [0, .01, .025, .04, .05]
    result = dict(frames=rows.tolist(), phases=['transport']*len(rows), payload=[True]*len(rows),
                  task={'right': 1}, joint_names=list(ACTIVE_JOINTS), tool_to_box={},
                  transport_search_stats={'path_source': source}, predicted_scene_events=[])
    return planner, result


def test_constructor_forwards_dynamics_without_changing_existing_arguments():
    torque = object()
    with patch.object(TimedSingleArmPlanner, '__init__') as init:
        cartesian.CartesianTransportPlanner(None, torque_model=torque)
    assert init.call_args.kwargs == {'torque_model': torque, 'use_trajopt': False}


def test_cartesian_actual_time_parameterization_never_uses_joint_baseline():
    planner, result = planner_and_result()
    with patch.object(TimedSingleArmPlanner, '_baseline_waypoints', side_effect=AssertionError('joint shortcut')):
        output = planner.time_parameterize_result(result)
    assert output['timed_segments'][0]['parameterizer'] in ('validated_toppra', 'validated_cartesian_cubic')
    assert output['baseline_trajectory_contract']['geometry_policy'] != 'validated_sparse_rrt_baseline'


def test_wrist_flip_cannot_compete_with_unsegmented_timing():
    planner, result = planner_and_result('wrist_flip')
    result['transport_search_stats']['swing_indices'] = [1, 3]
    with patch.object(cartesian, 'time_parameterize_toppra', side_effect=AssertionError('unsegmented')):
        output = planner.time_parameterize_result(result)
    segment = output['timed_segments'][0]
    assert segment['parameterizer'] == 'validated_toppra_wrist_flip'
    for value in (.01, .04):
        index = np.argmin(np.abs(np.asarray(segment['positions'])[:, 8] - value))
        np.testing.assert_allclose(segment['velocities'][index], 0, atol=1e-7)


def test_wrist_failure_is_not_replaced_by_unsegmented_candidate():
    planner, result = planner_and_result('wrist_flip')
    result['transport_search_stats']['swing_indices'] = [1, 3]
    with patch.object(cartesian, 'time_parameterize_toppra_pieces', side_effect=RuntimeError('infeasible')):
        with pytest.raises(RuntimeError, match='infeasible'):
            planner.time_parameterize_result(result)


@pytest.mark.parametrize('sparse', [[], [[0.0]], [[float('nan')]*15]*2, [[1.0]*15]*2])
def test_joint_missing_or_invalid_sparse_baseline_is_rejected(sparse):
    planner, result = planner_and_result('joint')
    result['transport_search_stats']['trajectory_fallback_waypoints'] = sparse
    with pytest.raises((RuntimeError, ValueError), match='sparse|baseline'):
        planner.time_parameterize_result(result)


def test_joint_fallback_uses_result_source_not_previous_wrist_state():
    planner, result = planner_and_result('joint')
    planner.last_wrist_flip = {'rows': 5, 'swing': [1, 3]}
    rows = np.asarray(result['frames'])[[0, -1]]
    result['transport_search_stats']['trajectory_fallback_waypoints'] = rows.tolist()
    with patch.object(cartesian, 'time_parameterize_toppra_pieces', side_effect=AssertionError('stale wrist')):
        output = planner.time_parameterize_result(result)
    assert output['timed_segments'][0]['parameterizer'] == 'validated_sparse_baseline_stops'


def test_geometric_entrypoint_override_is_used_by_timed_wrapper():
    planner, result = planner_and_result()
    result['success'] = False
    planner.plan_geometric_cycle = lambda boxes, progress: result
    assert TimedSingleArmPlanner.full_cycle(planner, {'right': 1}) is result


def test_dynamics_binding_rejects_changed_payload_before_rnea():
    from types import SimpleNamespace
    from curobo_core.torque import constrain_timed_result
    planner, result = planner_and_result()
    output = planner.time_parameterize_result(result)
    model = SimpleNamespace(certificate_identity={'payload_mass_kg': 10.0})
    planner.torque_model = model
    output['baseline_trajectory_contract'] = planner._baseline_contract(output, output['frames'])
    model.certificate_identity = {'payload_mass_kg': 20.0}
    with pytest.raises(ValueError, match='binding'):
        constrain_timed_result(output, model)


def test_segment_sub_float32_change_cannot_hide_behind_cycle_hash():
    from curobo_core.torque import constrain_timed_result, geometry_digest
    planner, result = planner_and_result()
    output = planner.time_parameterize_result(result)
    assert geometry_digest([[1.]]) != geometry_digest([[1.+1e-10]])
    output['timed_segments'][0]['positions'] = copy.deepcopy(output['timed_segments'][0]['positions'])
    output['timed_segments'][0]['positions'][1][8] += 1e-10
    with pytest.raises(ValueError, match='segment geometry'):
        constrain_timed_result(output, object())


def test_zero_time_arm_jump_is_rejected_but_navigation_reset_is_not():
    from curobo_core.trajectory import validate_timed_cycle
    args = (['home', 'base_return'], [0., 0.], [[0., 0.]]*2, [[0., 0.]]*2,
            [[0., 0.]]*2, [1., 1.], [1., 1.], [1., 1.], [])
    assert validate_timed_cycle(*args, positions=[[0., 0.], [1., 0.]],
                                joint_names=['base_x', 'joint'])['success']
    with pytest.raises(ValueError, match='zero-time'):
        validate_timed_cycle(*args, positions=[[0., 0.], [0., .1]], joint_names=['base_x', 'joint'])


def test_fallback_promotes_its_sparse_waypoints_and_resets_previous_wrist_state():
    from test_core_transport_fallback import stub_planner, row
    planner = stub_planner(fallback_joint_rrt=True)
    planner.last_wrist_flip = {'swing': [1, 3]}
    planner.last_toppra_error = 'previous cycle'
    sparse = [row((1, -1, 1)).tolist(), row((1, -1, 1)).tolist()]
    planner.single_path = lambda *args, **kwargs: (np.asarray(sparse), {'trajectory_fallback_waypoints': sparse})
    with patch.object(cartesian, 'plan_cartesian_transport', return_value=(None, {'failure': 'test'})):
        _, stats = planner.transport_path(np.asarray(sparse[0]), np.asarray(sparse[1:]), None, np.asarray(sparse[1:]))
    assert stats['path_source'] == 'joint'
    assert stats['trajectory_fallback_waypoints'] == sparse
    assert planner.last_wrist_flip is None and planner.last_toppra_error is None


def test_dynamics_overlay_cannot_change_planning_kinematics(tmp_path):
    from curobo_core.torque import write_dynamics_robot_config
    import yaml
    original = tmp_path / 'original.urdf'
    overlay = tmp_path / 'overlay.urdf'
    original.write_text('<robot><joint name="joint" type="fixed"><origin xyz="0 0 1"/></joint></robot>')
    overlay.write_text(original.read_text().replace('0 0 1', '0 0 2'))
    config = tmp_path / 'robot.yml'
    config.write_text(yaml.safe_dump({'kinematics': {'urdf_path': str(original)}}))
    with pytest.raises(ValueError, match='kinematics differ'):
        write_dynamics_robot_config(config, overlay, tmp_path, tmp_path/'out.yml')
    overlay.write_text(original.read_text())
    write_dynamics_robot_config(config, overlay, tmp_path, tmp_path/'out.yml')
    assert yaml.safe_load((tmp_path/'out.yml').read_text())['kinematics']['urdf_path'] == str(overlay)


def test_replay_discloses_actual_source_model_timing_and_dynamics():
    from types import SimpleNamespace
    from experiments.motion312.replay import summary
    result = dict(success=True, transport_search_stats={'path_source': 'wrist_flip'},
                  model_id='old-model-identity', dynamics_enabled=True, controller_ready=False,
                  torque_validation={'success': True, 'geometry_frozen': True},
                  timed_segments=[{'name': 'transport', 'duration_s': 1., 'parameterizer': 'validated_toppra_wrist_flip'}])
    text = summary(SimpleNamespace(result=result, extra_rotation=None))
    assert all(item in text for item in ['wrist_flip', 'old-model', 'validated_toppra_wrist_flip', 'RNEA通过/q冻结', '非controller-ready'])
