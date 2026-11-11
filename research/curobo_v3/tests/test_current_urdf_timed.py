from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from v3_current_urdf_timed import TimedCurrentUrdfPlanner
from v3_stage_playback import StagePlayback
import v3_forward_precontact_demo


def test_current_timed_planner_keeps_current_model_and_conservative_arm_limits():
    planner = TimedCurrentUrdfPlanner()
    assert planner.args.urdf.name == 'robot_rezero_candidate.urdf'
    for side in ('left', 'right'):
        for joint in range(1, 8):
            index = planner._timed_joint_names.index(f'{side}_joint{joint}')
            assert planner._timed_limits[0][index] <= np.deg2rad(15 if joint < 5 else 30)
            assert planner._timed_limits[1][index] <= np.deg2rad(30 if joint < 5 else 60)


def test_timed_stage_view_slices_clocks_and_derivatives_with_frame_indices():
    source = dict(frames=[[index] for index in range(6)], phases=['home', 'approach', 'attach', 'transport', 'transport', 'release'],
                  payload=[False, False, True, True, True, False], time_from_start_s=[0, 1, 1, 1, 3, 3],
                  velocities=[[index] for index in range(6)], accelerations=[[0]] * 6, jerks=[[0]] * 6,
                  timed_segments=[dict(name='transport', frame_start=3, frame_end=4)])
    view = StagePlayback().select(source, 'transport')
    assert view['_source_indices'] == [2, 3, 4]
    assert view['time_from_start_s'] == [1, 1, 3]
    assert view['velocities'] == [[2], [3], [4]]
    assert view['timed_segments'][0]['frame_start'] == 1
    assert view['timed_segments'][0]['frame_end'] == 2
    assert source['timed_segments'][0]['frame_start'] == 3


def test_timing_adapter_does_not_add_another_planning_stage():
    planner = TimedCurrentUrdfPlanner()
    assert planner.forward_precontact_policy
    assert planner.lm_chained_cycle_policy
    assert planner.trajectory_optimization_enabled
    assert issubclass(v3_forward_precontact_demo.TimedNewWarpPlanner, TimedCurrentUrdfPlanner)
