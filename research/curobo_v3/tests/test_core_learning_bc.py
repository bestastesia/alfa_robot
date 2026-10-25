from pathlib import Path
import copy
import json
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from motion311_learning.features import STATE_DIM
from motion311_learning.networks import SACAgent
from motion311_learning.train import BC_TARGET_VERSION, main, select_bc_targets, warm_start


def episode(action=(.8, .2, 0.), value=.8, weight=1., stage=2, state=None,
            success=True, truncated=False, identity='one'):
    if state is None:
        state = np.zeros(STATE_DIM)
        state[stage] = 1.
        state[5] = 1.
    row = dict(stage=stage, state=np.asarray(state).tolist(), action_params=list(action),
               action_q=np.zeros(15).tolist(), action_physics=None, next=None,
               reward=value, terminated=not truncated, truncated=truncated, trainable=not truncated,
               episode_success=success, suffix_weight=weight, info={'success':success}, **{'return':value})
    return dict(identity=identity, split='train', group='train', task={'left':1}, transitions=[row])


class SingleTargetBCTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.old_threads = torch.get_num_threads()
        torch.set_num_threads(4)

    @classmethod
    def tearDownClass(cls):
        torch.set_num_threads(cls.old_threads)

    def test_weighted_mean_selects_real_action_not_lucky_leaf_or_parameter_mean(self):
        a, b = (-.8, .2, 0.), (.8, -.2, 0.)
        episodes = [episode(a, 1., .1), episode(a, .2, .9), episode(b, .7, .5)]
        selected = select_bc_targets(episodes)
        self.assertEqual(len(selected), 1)
        self.assertEqual(selected[0]['action_params'], list(b))
        self.assertAlmostEqual(selected[0]['bc_weighted_return'], .7)
        self.assertEqual(selected[0]['bc_action_alternatives'], 2)
        self.assertNotEqual(selected[0]['action_params'], ((np.array(a) + b) / 2).tolist())
        self.assertEqual(select_bc_targets(list(reversed(episodes))), selected)

    def test_tie_break_uses_stable_parameter_tuple_and_retains_boundaries(self):
        left, right = (-1., -.9, 0.), (1., .9, 0.)
        selected = select_bc_targets([episode(right, .8), episode(left, .8)])
        self.assertEqual(selected[0]['action_params'], list(left))
        self.assertEqual(selected[0]['bc_target_version'], BC_TARGET_VERSION)
        self.assertEqual(select_bc_targets([episode(left, .8), episode(right, .8)]), selected)

    def test_exact_state_and_stage_groups_do_not_merge_close_states(self):
        first = episode(stage=0)
        second = copy.deepcopy(first)
        second['transitions'][0]['state'][7] = 1e-12
        third = copy.deepcopy(first)
        third['transitions'][0]['stage'] = 1
        result = select_bc_targets([third, second, first])
        self.assertEqual(len(result), 3)
        self.assertEqual([row['stage'] for row in result], [0, 0, 1])

    def test_failures_and_truncation_excluded_and_nontrain_rejected(self):
        accepted = episode(value=.5)
        excluded = [episode(value=10., success=False), episode(value=10., truncated=True)]
        row_untrainable = episode(value=10.)
        row_untrainable['transitions'][0]['trainable'] = False
        self.assertEqual(select_bc_targets(excluded + [row_untrainable, accepted]), select_bc_targets([accepted]))
        for group, split in (('seen_test', 'train'), ('train', 'dev'), ('reserve', 'train')):
            forbidden = episode(); forbidden['group'], forbidden['split'] = group, split
            with self.assertRaisesRegex(ValueError, 'train'):
                select_bc_targets([forbidden])
        for field, value in (('suffix_weight', 0.), ('suffix_weight', float('nan')), ('return', float('inf'))):
            invalid = episode(); invalid['transitions'][0][field] = value
            with self.assertRaisesRegex(ValueError, 'invalid'):
                select_bc_targets([invalid])

    def test_warm_start_fits_chosen_action_without_changing_critic_or_alpha(self):
        agent = SACAgent(STATE_DIM, width=8, seed=2, lr=.01, action_mask_state_indices=(5, 6))
        data = [episode((.8, .25, 0.), .9), episode((-.8, -.25, 0.), .2)]
        target = np.array([.8, .25, 0.], dtype=np.float32)
        state = np.asarray(data[0]['transitions'][0]['state'])
        before = agent.select(state, 2, deterministic=True)
        critics = [parameter.detach().clone() for parameter in agent.critics.parameters()]
        targets = [parameter.detach().clone() for parameter in agent.target_critics.parameters()]
        with patch.object(agent, 'update', side_effect=AssertionError('steps=0 must not update critics')):
            warm_start(agent, data, 'r4', steps=0)
        after = agent.select(state, 2, deterministic=True)
        self.assertLess(np.linalg.norm(after - target), np.linalg.norm(before - target))
        self.assertLess(np.linalg.norm(after - target), .04)
        self.assertEqual(float(after[2]), 0.)
        self.assertEqual(agent.alpha, .05)
        for expected, actual in zip(critics, agent.critics.parameters()):
            torch.testing.assert_close(expected, actual, rtol=0, atol=0)
        for expected, actual in zip(targets, agent.target_critics.parameters()):
            torch.testing.assert_close(expected, actual, rtol=0, atol=0)

    def test_dqn_warm_does_not_call_bc_target_selection(self):
        class FakeDQN:
            config = {'seed':0}
            def __init__(self): self.updates = 0
            def update(self, batch): self.updates += 1
        agent = FakeDQN()
        with patch('motion311_learning.train.select_bc_targets', side_effect=AssertionError('DQN BC forbidden')):
            rows = warm_start(agent, [episode()], 'r3', steps=2)
        self.assertEqual(agent.updates, 2)
        self.assertEqual(len(rows), 1)

    def test_training_config_records_bc_version_before_environment_start(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            registry = root / 'registry.json'
            registry.write_text(json.dumps({'instances':[dict(instance='one', task={'left':1}, kind='single',
                top_grasp=False, split='train', group='train', offset_m=[0.,0.])]}))
            with patch('motion311_learning.train.signal.signal'), patch('builtins.print'):
                status = main(['--registry', str(registry), '--output', str(root / 'run'), '--route', 'r4',
                               '--episodes', '1', '--max-minutes', '0.0000000001', '--threads', '4'])
            self.assertEqual(status, 3)
            config = json.loads((root / 'run/config.json').read_text())
            self.assertEqual(config['bc_target_version'], BC_TARGET_VERSION)


if __name__ == '__main__':
    unittest.main()
