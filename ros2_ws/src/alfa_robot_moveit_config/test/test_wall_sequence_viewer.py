#!/usr/bin/env python3
"""No window/ROS node: exercise the real viewer's scene switches and release frames."""
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np

from alfa_robot_rerun import v3_single_arm_box_extract_viewer as module


def main():
    viewer = module.V3SingleArmBoxExtractViewer.__new__(module.V3SingleArmBoxExtractViewer)
    viewer.tool_link = 'left_tool0'
    viewer.box_center = np.zeros(3)
    viewer.box_size = np.array([.3, .4, .5])
    viewer.tool_to_box_center = np.zeros(3)
    viewer.robot = SimpleNamespace(fk=lambda _: {'left_tool0': np.eye(4), 'right_tool0': np.eye(4)})
    viewer.scenes = [
        dict(box_center=[1, 2, 3], box_size=[.3, .4, .5], tool_link='left_tool0',
             tool_to_box_center=[0, 0, .15], tool_to_box_rotation=[[0, 0, -1], [0, 1, 0], [1, 0, 0]],
             neighbor_centers=[[4, 5, 6]]),
        dict(box_center=[4, 5, 6], box_size=[.3, .4, .5], tool_link='right_tool0',
             tool_to_box_center=[0, 0, .25], tool_to_box_rotation=[[1, 0, 0], [0, -1, 0], [0, 0, -1]],
             neighbor_centers=[]),
    ]
    viewer.scene_index = -1
    viewer.frames = [module.PlaybackFrame('attach_box', (), True, True, 0),
                     module.PlaybackFrame('release_box', (), False, False, 0),
                     module.PlaybackFrame('attach_box', (), True, True, 1),
                     module.PlaybackFrame('release_box', (), False, False, 1)]
    viewer.frame_index = 0
    viewer.global_frame = 0
    viewer.generation = 1
    viewer.joint_names = ()
    viewer.last_frame = None
    viewer.minimum_frame_period = viewer.stage_pause_s = .01
    viewer.playback_joint_speed_deg_s = 25
    state = {}
    def log(path, value):
        if isinstance(value, str) and value == 'clear':
            state.pop(path, None)
        else:
            state[path] = value
    with patch.object(module.rr, 'log', side_effect=log), \
            patch.object(module.rr, 'set_time'), \
            patch.object(module, 'log_robot_state'), \
            patch.object(module.rr, 'Clear', return_value='clear'), \
            patch.object(module.rr, 'Boxes3D', side_effect=lambda **kwargs: kwargs):
        for i in range(4):
            viewer.log_frame(viewer.robot.fk({}))
            if i % 2:
                assert 'world/boxes/target' not in state and 'world/boxes/carried' not in state
            else:
                carried = state['world/boxes/carried']
                assert np.allclose(carried['half_sizes'], [[.15, .2, .25]])
                assert np.allclose(carried['centers'], [[0, 0, .15 if i == 0 else .25]])
                assert np.allclose(carried['quaternions'], [module.matrix_to_quaternion(
                    np.asarray(viewer.scenes[i // 2]['tool_to_box_rotation']))])
            if i >= 2:
                assert viewer.tool_link == 'right_tool0'
                assert 'world/boxes/neighbors' not in state  # last neighbor never reappears
    print('PASS viewer: scene/tool switch, non-cubic attachment, release hiding, empty-neighbor clearing')


if __name__ == '__main__':
    main()
