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
from motion311_learning.contracts import SCHEMA
from motion311_learning.features import PHYSICS_DIM, STATE_DIM
from motion311_learning.networks import DQNAgent, SACAgent
from motion311_learning.train import (
    _read_record, balanced_items, commit_pending, initial_training_state,
    load_training_checkpoint, main, policy_rng_state, rebuild_replay, save_agent,
    warm_data_snapshot, warm_replay, write_pending,
)


def configuration(route='r3'):
    return dict(route=route, seed=12, updates=2, episodes=10, schema=SCHEMA,
                warm_data={'sha256': 'fixed'}, registry_sha256='registry')


def task_item(index=0):
    return dict(instance=f'instance-{index}', split='train', group='train', task={'left': index + 1},
                offset_m=[0.0, 0.0], top_grasp=False, kind='single')


def episode(index=0, truncated=False):
    rng = np.random.default_rng(index + 40)
    states = rng.normal(size=(3, STATE_DIM)) * .1
    states[:, 5:7] = [1.0, 0.0]
    states[:, -1] = [0.0, .01, .02]
    rows = []
    for stage in range(3):
        future = None if stage == 2 else dict(stage=stage + 1, state=states[stage + 1].tolist(),
                candidates=rng.uniform(-1, 1, (3, 15)).tolist(), physics=np.zeros((3, PHYSICS_DIM)).tolist())
        params = rng.uniform(-.5, .5, 3); params[2] = 0.0
        rows.append(dict(stage=stage, state=states[stage].tolist(), action_q=rng.uniform(-1, 1, 15).tolist(),
                         action_physics=np.zeros(PHYSICS_DIM).tolist(), action_params=params.tolist(),
                         reward=1.0 if stage == 2 else -.01, terminated=stage == 2 and not truncated,
                         truncated=stage == 2 and truncated, trainable=not truncated, episode_success=not truncated,
                         next=future, info=dict(success=stage == 2 and not truncated, cumulative_cost_s=stage + 1),
                         **{'return': 1.0 - .01 * (2 - stage)}))
    return dict(task_item(index), schema=SCHEMA, identity=f'warm-{index}', transitions=rows)


class TrainingResumeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.old_threads = torch.get_num_threads()
        torch.set_num_threads(4)

    @classmethod
    def tearDownClass(cls):
        torch.set_num_threads(cls.old_threads)

    def assertNestedEqual(self, left, right):
        if isinstance(left, torch.Tensor):
            torch.testing.assert_close(left, right, atol=0, rtol=0)
        elif isinstance(left, np.ndarray):
            np.testing.assert_array_equal(left, right)
        elif isinstance(left, dict):
            self.assertEqual(left.keys(), right.keys())
            for key in left:
                self.assertNestedEqual(left[key], right[key])
        elif isinstance(left, (list, tuple)):
            self.assertEqual(len(left), len(right))
            for a, b in zip(left, right):
                self.assertNestedEqual(a, b)
        else:
            self.assertEqual(left, right)

    def agent(self, route):
        if route == 'r4':
            return SACAgent(STATE_DIM, width=8, seed=12, action_mask_state_indices=(5, 6))
        return DQNAgent(STATE_DIM, 15, width=8, seed=12)

    def test_pending_recovery_matches_uninterrupted_dqn_and_sac_after_partial_updates(self):
        warm = [episode(index) for index in range(12)]
        for route in ('r3', 'r4'):
            with self.subTest(route=route), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                config = configuration(route)
                initial = self.agent(route)
                state = initial_training_state(config)
                checkpoint = root / 'agent.pt'
                save_agent(initial, checkpoint, state)
                # Consume action RNG as the actual completed environment run does.
                if route == 'r4':
                    for row in episode(15)['transitions']:
                        initial.select(np.asarray(row['state']), row['stage'])
                else:
                    initial.select(np.zeros(STATE_DIM), np.zeros((3, 15)), 0, epsilon=1.0)
                record, path = write_pending(root, config, 0, task_item(15), copy.deepcopy(episode(15)['transitions']),
                                             initial, np.random.default_rng(849), 1.2)
                replay = warm_replay(warm, route)
                reference, reference_state = load_training_checkpoint(checkpoint, config)
                reference_state = commit_pending(reference, copy.deepcopy(replay), reference_state, record, path,
                                                   root / 'reference.pt', config)
                failed, failed_state = load_training_checkpoint(checkpoint, config)
                def interrupt(count):
                    if count == 2:
                        raise InterruptedError('simulated process exit after two optimizer steps')
                with self.assertRaises(InterruptedError):
                    commit_pending(failed, copy.deepcopy(replay), failed_state, record, path, checkpoint,
                                   config, after_update=interrupt)
                resumed, resumed_state = load_training_checkpoint(checkpoint, config)
                self.assertEqual(resumed_state['last_committed_episode'], -1)
                pending, pending_path = _read_record(root, config, 0)
                restored_replay = rebuild_replay(root, config, warm, resumed_state)
                resumed_state = commit_pending(resumed, restored_replay, resumed_state, pending, pending_path,
                                                checkpoint, config)
                self.assertNestedEqual(resumed.state_dict(), reference.state_dict())
                self.assertEqual(resumed_state['last_committed_episode'], 0)
                self.assertEqual(resumed_state['gradient_updates'], 6)
                self.assertEqual(resumed_state['committed_records'], reference_state['committed_records'])
                # Generic evaluation loader ignores extra training envelope fields.
                loaded = (SACAgent if route == 'r4' else DQNAgent).load(checkpoint)
                self.assertNestedEqual(loaded.state_dict(), resumed.state_dict())

    def test_pending_before_any_update_and_missing_metadata_are_not_silently_skipped(self):
        with tempfile.TemporaryDirectory() as folder:
            root, config = Path(folder), configuration()
            agent, state = self.agent('r3'), initial_training_state(config)
            checkpoint = root / 'agent.pt'
            save_agent(agent, checkpoint, state)
            record, path = write_pending(root, config, 0, task_item(), episode()['transitions'], agent,
                                         np.random.default_rng(1), 0.0)
            restored, restored_state = load_training_checkpoint(checkpoint, config)
            self.assertEqual(restored_state['last_committed_episode'], -1)
            self.assertIsNotNone(_read_record(root, config, 0)[0])
            committed = commit_pending(restored, [], restored_state, record, path, checkpoint, config)
            self.assertEqual(committed['last_committed_episode'], 0)
            self.assertEqual(committed['gradient_updates'], 0)
            with self.assertRaisesRegex(ValueError, 'contiguous'):
                commit_pending(restored, [], committed, record, path, checkpoint, config)
            agent.save(root / 'legacy.pt')
            with self.assertRaisesRegex(ValueError, 'metadata'):
                load_training_checkpoint(root / 'legacy.pt', config)

    def test_scene_failure_and_truncation_commit_without_training(self):
        with tempfile.TemporaryDirectory() as folder:
            root, config = Path(folder), configuration('r4')
            agent, state = self.agent('r4'), initial_training_state(config)
            checkpoint = root / 'agent.pt'
            save_agent(agent, checkpoint, state)
            initial = copy.deepcopy(agent.state_dict())
            replay = warm_replay([episode(index) for index in range(12)], 'r4')
            size = len(replay)
            scene, scene_path = write_pending(root, config, 0, task_item(), [], agent, np.random.default_rng(1),
                                              0.0, scene_reason='scene_no_contact_candidates')
            state = commit_pending(agent, replay, state, scene, scene_path, checkpoint, config)
            truncated, path = write_pending(root, config, 1, task_item(), episode(truncated=True)['transitions'],
                                            agent, np.random.default_rng(2), 0.0)
            state = commit_pending(agent, replay, state, truncated, path, checkpoint, config)
            self.assertEqual(state['last_committed_episode'], 1)
            self.assertEqual(state['gradient_updates'], 0)
            self.assertEqual(len(replay), size)
            self.assertNestedEqual(initial, agent.state_dict())
            self.assertTrue(scene_path.parent.name == 'scene-failures')
            self.assertFalse(list(root.rglob('*.tmp')))

    def test_warm_replay_deduplicates_copied_prefixes_and_is_order_stable(self):
        source = [episode(0), episode(1)]
        for route in ('r3', 'r4'):
            unique = warm_replay(source, route)
            repeated = warm_replay(source + copy.deepcopy(source) * 3, route)
            self.assertEqual(len(unique), 6)
            self.assertNestedEqual(unique, repeated)
        changed = copy.deepcopy(source[0])
        changed['transitions'][0]['next']['candidates'][0][0] += .5
        self.assertEqual(len(warm_replay([source[0], changed], 'r3')), 4)

    def test_rebuild_replay_validates_committed_record_hashes(self):
        with tempfile.TemporaryDirectory() as folder:
            root, config = Path(folder), configuration()
            agent, state = self.agent('r3'), initial_training_state(config)
            warm = [episode(index) for index in range(12)]
            replay = warm_replay(warm, 'r3')
            checkpoint = root / 'agent.pt'
            save_agent(agent, checkpoint, state)
            for index in range(2):
                record, path = write_pending(root, config, index, task_item(index), episode(index + 20)['transitions'],
                                             agent, np.random.default_rng(index), 0.0)
                state = commit_pending(agent, replay, state, record, path, checkpoint, config)
            restored, restored_state = load_training_checkpoint(checkpoint, config)
            self.assertNestedEqual(rebuild_replay(root, config, warm, restored_state), replay)
            data = json.loads(path.read_text()); data['elapsed_s'] += .1
            path.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, 'modified'):
                rebuild_replay(root, config, warm, restored_state)

    def test_warm_data_hash_and_explicit_train_group_filter(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            path = root / 'one.json'
            path.write_text(json.dumps(episode()))
            records, first = warm_data_snapshot(root)
            self.assertEqual(len(records), 1)
            value = episode(); value['transitions'][0]['reward'] -= .1
            path.write_text(json.dumps(value))
            _, second = warm_data_snapshot(root)
            self.assertNotEqual(first['sha256'], second['sha256'])
            for group in ('seen_test', 'reserve', None):
                value = episode()
                if group is None:
                    del value['group']
                else:
                    value['group'] = group
                path.write_text(json.dumps(value))
                with self.assertRaisesRegex(ValueError, 'group=train'):
                    warm_data_snapshot(root)
            value = episode(); value['split'] = 'dev'
            path.write_text(json.dumps(value))
            with self.assertRaisesRegex(ValueError, 'group=train'):
                warm_data_snapshot(root)

    def test_main_recovers_pending_without_importing_or_running_environment(self):
        import builtins
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            registry, output = root / 'registry.json', root / 'run'
            registry.write_text(json.dumps({'instances': [task_item()]}))
            argv = ['--registry', str(registry), '--output', str(output), '--route', 'r3',
                    '--episodes', '1', '--threads', '4', '--max-minutes', '0.0000000001']
            with patch('builtins.print'):
                self.assertEqual(main(argv), 3)
            config = json.loads((output / 'config.json').read_text())
            agent, state = load_training_checkpoint(output / 'agent.pt', config)
            write_pending(output, config, 0, task_item(), episode()['transitions'], agent,
                          np.random.default_rng(311), 1.0)
            original_import = builtins.__import__
            def guarded_import(name, *args, **kwargs):
                if name == 'motion311_learning.stage_env':
                    raise AssertionError('pending replay must not import the environment')
                return original_import(name, *args, **kwargs)
            argv[-1] = '24'
            with patch('builtins.print'), patch('builtins.__import__', side_effect=guarded_import):
                self.assertEqual(main(argv), 0)
                self.assertEqual(main(argv), 0)
            restored, committed = load_training_checkpoint(output / 'agent.pt', config)
            self.assertEqual(committed['last_committed_episode'], 0)
            self.assertEqual(len(list((output / 'episodes').glob('*.json'))), 1)

    def test_changed_warm_hash_and_out_of_order_commit_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root, config = Path(folder), configuration()
            agent, state = self.agent('r3'), initial_training_state(config)
            save_agent(agent, root / 'agent.pt', state)
            changed = copy.deepcopy(config); changed['warm_data']['sha256'] = 'different'
            with self.assertRaisesRegex(ValueError, 'configuration'):
                load_training_checkpoint(root / 'agent.pt', changed)
            record, path = write_pending(root, config, 2, task_item(), episode()['transitions'], agent,
                                         np.random.default_rng(1), 0.0)
            with self.assertRaisesRegex(ValueError, 'contiguous'):
                commit_pending(agent, [], state, record, path, root / 'agent.pt', config)
            self.assertEqual(balanced_items([task_item(1), task_item(0)])[0]['instance'], 'instance-0')
            with self.assertRaisesRegex(ValueError, 'no group=train'):
                balanced_items([])


if __name__ == '__main__':
    unittest.main()
