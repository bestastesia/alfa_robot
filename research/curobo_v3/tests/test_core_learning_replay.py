from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from motion311_learning.contracts import Observation, StepResult
from motion311_learning.dataset import finish_episode, learning_transition, load_episodes, normalized_actions, normalized_state, transition_record, write_episode
from motion311_learning.features import STATE_DIM, PHYSICS_DIM, state_features


class DatasetTest(unittest.TestCase):
    def observation(self, stage=0):
        return Observation(stage, np.zeros(STATE_DIM), np.ones((2, 15)), np.zeros((2, 3)), np.ones((2, PHYSICS_DIM)),
                           np.repeat(np.eye(4)[None], 2, 0))

    def test_train_excludes_dev_and_preserves_candidate_precision(self):
        obs = self.observation()
        final = StepResult(None, 1., True, False, dict(success=True))
        rows = [transition_record(obs, 1, final)]
        with tempfile.TemporaryDirectory() as folder:
            write_episode(folder, 'one', dict(instance='a', split='train', task={'left': 1}, offset_m=[0., 0.]), rows, 'teacher', {}, 1.)
            write_episode(folder, 'two', dict(instance='b', split='dev', task={'left': 1}, offset_m=[0., 0.]), rows, 'teacher', {}, 1.)
            loaded = list(load_episodes(folder))
            self.assertEqual(len(loaded), 1)
            np.testing.assert_array_equal(loaded[0]['transitions'][0]['candidates'], obs.candidates)
            self.assertEqual(loaded[0]['transitions'][0]['return'], 1.)

    def test_partial_teacher_tree_is_excluded_until_done(self):
        obs = self.observation()
        rows = [transition_record(obs, 0, StepResult(None, 1., True, False, dict(success=True)))]
        with tempfile.TemporaryDirectory() as folder:
            base = Path(folder)
            item = dict(instance='train-one', split='train', group='train', task={'left': 1}, offset_m=[0., 0.])
            write_episode(base / 'episodes', 'train-one-r0-0', item, rows, 'branching_teacher', {}, 1.)
            self.assertEqual(list(load_episodes(base / 'episodes')), [])
            (base / 'done').mkdir()
            (base / 'done/train-one-r1.json').write_text('{}')
            self.assertEqual(list(load_episodes(base / 'episodes')), [])
            (base / 'done/train-one-r0.json').write_text('{}')
            self.assertEqual(len(list(load_episodes(base / 'episodes'))), 1)

    def test_truncation_never_trains_negative_label(self):
        row = transition_record(self.observation(), 0, StepResult(None, 0., False, True, dict(cause='oom')))
        finish_episode([row])
        self.assertFalse(row['trainable'])
        with self.assertRaises(ValueError):
            learning_transition(row)

    def test_continuous_empty_candidate_observation_keeps_bootstrap(self):
        obs = self.observation()
        empty = Observation(1, np.zeros(STATE_DIM), np.empty((0, 15)), np.empty((0, 3)),
                            np.empty((0, PHYSICS_DIM)), np.repeat(np.eye(4)[None], 2, 0))
        record = transition_record(obs, 0, StepResult(empty, -.01, False, False, {}))
        continuous = learning_transition(record, continuous=True)
        self.assertIsNone(continuous['next_candidates'])
        self.assertFalse(continuous['terminated'])
        self.assertIsNotNone(continuous['next_state'])
        self.assertEqual(normalized_actions([], np.empty((0, PHYSICS_DIM))).shape, (0, 15 + PHYSICS_DIM))

    def test_normalization_dimensions(self):
        self.assertEqual(normalized_state(np.ones(STATE_DIM)).shape, (STATE_DIM,))
        self.assertEqual(normalized_actions(np.ones((2, 15)), np.ones((2, PHYSICS_DIM))).shape, (2, 15 + PHYSICS_DIM))

    def test_future_fields_not_present_in_state_features(self):
        args = dict(stage=0, previous=np.zeros(15), home=np.zeros(15), targets={'left':np.eye(4)}, carry_targets={'left':np.eye(4)},
                    box_centers={'left':np.zeros(3)}, active_sides=['left'], top=True, offset=[0., 0.], cost=0.)
        values = state_features(**args)
        self.assertEqual(values.shape, (STATE_DIM,))
        with self.assertRaises(TypeError):
            state_features(**args, future_best=np.ones(15))


if __name__ == '__main__':
    unittest.main()
