from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from motion311_learning.stage_env import map_frames, validate_checkpoint


class FrameContractTest(unittest.TestCase):
    def test_restore_rejects_different_task_or_unvalidated_history(self):
        from v3_candidate_ranker import ACTIVE_JOINTS
        import copy
        state = dict(item={'task': {'left': 1}}, boxes={'left': 1}, stage=1, previous=np.zeros(15),
                     report=dict(task={'left': 1}, joint_names=ACTIVE_JOINTS, frames=[np.zeros(15).tolist()]),
                     timed_blocks=[dict(timed_validation={'success': True})])
        validate_checkpoint(state, ACTIVE_JOINTS, True)
        bad = copy.deepcopy(state); bad['boxes'] = {'left': 2}
        with self.assertRaises(ValueError):validate_checkpoint(bad, ACTIVE_JOINTS, True)
        bad = copy.deepcopy(state); bad['timed_blocks'] = []
        with self.assertRaises(ValueError):validate_checkpoint(bad, ACTIVE_JOINTS, True)
        bad = copy.deepcopy(state); bad['previous'][0] = 1.
        with self.assertRaises(ValueError):validate_checkpoint(bad, ACTIVE_JOINTS, True)

    def test_local_frames_are_translated_once_for_timing(self):
        report = dict(joint_names=['base_x', 'base_y', 'base_yaw', 'updown'],
                      frames=[[0., 0., 0., -.3], [-1.2, .1, np.pi, -.2]])
        mapped = map_frames(report, .3)
        np.testing.assert_allclose(np.asarray(mapped['frames'])[:, 0], [.3, -.9])
        np.testing.assert_allclose(np.asarray(report['frames'])[:, 0], [0., -1.2])
        np.testing.assert_array_equal(np.asarray(mapped['frames'])[:, 1:], np.asarray(report['frames'])[:, 1:])


if __name__ == '__main__':
    unittest.main()
