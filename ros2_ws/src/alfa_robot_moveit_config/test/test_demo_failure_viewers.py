#!/usr/bin/env python3
"""Exercise the real V3 viewer parsers/timers, without opening windows or executing ROS actions."""
import json
import numpy as np
from types import SimpleNamespace
from unittest.mock import MagicMock, patch
from alfa_robot_rerun.demo_failure import replay_frames
from alfa_robot_rerun import v3_single_arm_box_extract_viewer as single
from alfa_robot_rerun import v3_dual_arm_cartesian_box_viewer as dual


def check(module, cls, single_arm):
    viewer = cls.__new__(cls)
    viewer.robot = SimpleNamespace(fk=lambda _: {"left_tool0": np.eye(4)})
    viewer.tool_link = 'left_tool0'
    viewer.generation = 0
    viewer.global_frame = 1
    viewer.minimum_frame_period = viewer.stage_pause_s = .01
    viewer.playback_joint_speed_deg_s = 25
    viewer.get_logger = MagicMock(return_value=MagicMock())
    viewer.update_scene = MagicMock()
    viewer.log_summary = MagicMock()
    viewer.log_boxes = MagicMock()
    frame = dict(stage='safe_prefix', joints=[0., 0.], box_center=[1., 2., 3.],
                 box_attached=False, box_visible=True, scene_index=0)
    rejected = dict(frame, stage='REJECTED_SNAPSHOT: collision', joints=[.2, .3], box_attached=True)
    payload = dict(kind='result', success=False, generation=1, joint_names=['joint1', 'joint2'],
        frames=[frame], diagnostic_frames=[frame, rejected], failure_stage='collision',
        diagnostic=dict(diagnostic_only=True, freeze_at_end=True, stage='collision',
            reason='robot<->box', contacts=[dict(position=[1., 2., 3.], bodies=['robot', 'box'])]))
    with patch.object(module.rr, 'log'), patch.object(module.rr, 'set_time') as clock, \
            patch.object(module, 'log_robot_state') as robot, \
            patch.object(module, 'log_failure') as failure, \
            patch.object(module.rr, 'send_columns') as columns, \
            patch.object(module.rr, 'get_global_data_recording', return_value=None):
        viewer.on_task(SimpleNamespace(data=json.dumps(payload)))
        assert len(viewer.frames) == 2 and viewer.frames[-1].joints == (.2, .3)
        if single_arm:
            assert viewer.frame_index == 2  # complete before on_task returns, no timer
            assert columns.call_count == 1
        else:
            for _ in range(5):
                viewer.next_frame_time = 0
                viewer.on_timer()
            assert robot.call_count == 2  # no restart/replay of failed frames
        assert viewer.last_frame.joints == (.2, .3)
        failure.assert_called_once_with(payload['diagnostic'])
        last_tick = viewer.global_frame - 1
        clock.reset_mock()
        payload['success'] = True  # stale diagnostic frames must never override a success
        viewer.on_task(SimpleNamespace(data=json.dumps(payload)))
        assert len(viewer.frames) == 1 and not viewer.diagnostic
        if single_arm:
            assert all(call.kwargs["sequence"] > last_tick for call in clock.call_args_list), \
                "new result overwrote the previous failure endpoint"


def main():
    assert replay_frames({'success': False, 'frames': [1], 'diagnostic_frames': [2]}) == [2]
    assert replay_frames({'success': True, 'frames': [1], 'diagnostic_frames': [2]}) == [1]
    check(single, single.V3SingleArmBoxExtractViewer, True)
    check(dual, dual.V3DualArmCartesianBoxViewer, False)
    print('PASS real single/dual viewer parsers: diagnostic selection, final freeze, new-task clearing')


if __name__ == '__main__':
    main()
