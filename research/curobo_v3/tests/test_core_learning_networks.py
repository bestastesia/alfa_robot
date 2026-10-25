from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import torch
from torch import nn

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from motion311_learning.networks import (
    DQNAgent, GaussianActor, SACAgent, StageQ, StageSupervised, TwinCritic,
)


def transition(stage=0, terminated=False, state_dim=4, action_dim=3, **overrides):
    row = dict(stage=stage, state=np.zeros(state_dim, dtype=np.float32),
               action=np.zeros(action_dim, dtype=np.float32), reward=2.0,
               next_state=np.ones(state_dim, dtype=np.float32),
               next_candidates=np.zeros((2, action_dim), dtype=np.float32),
               terminated=terminated)
    row.update(overrides)
    return row


def replay(size=18):
    rng = np.random.default_rng(18)
    rows = []
    for i in range(size):
        stage = i % 3
        rows.append(transition(stage=stage, terminated=stage == 2,
                               state=rng.normal(size=4).astype(np.float32),
                               action=rng.uniform(-1, 1, size=3).astype(np.float32),
                               reward=float(rng.normal()),
                               next_state=rng.normal(size=4).astype(np.float32),
                               next_candidates=rng.uniform(-1, 1, size=(1 + i % 5, 3)).astype(np.float32)))
    return rows


class AffineQ(nn.Module):
    def __init__(self, slope, offset=0.0):
        super().__init__()
        self.slope, self.offset = slope, offset

    def forward(self, state, action):
        return self.slope * action[:, 0] + self.offset


class FixedActor(nn.Module):
    def __init__(self, action, logprob):
        super().__init__()
        self.action, self.logprob = action, logprob
        self.calls = 0

    def sample(self, state, generator=None):
        self.calls += 1
        return (state.new_full((len(state), 3), self.action),
                state.new_full((len(state),), self.logprob))


class AffineTwin(nn.Module):
    def __init__(self, slope):
        super().__init__()
        self.slope = slope

    def forward(self, state, action):
        value = action[:, 0] * self.slope
        return value, value + 1.0


class CoreLearningNetworksTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.old_threads = torch.get_num_threads()
        torch.set_num_threads(4)

    @classmethod
    def tearDownClass(cls):
        torch.set_num_threads(cls.old_threads)

    def assertParametersEqual(self, left, right):
        for a, b in zip(left.parameters(), right.parameters()):
            torch.testing.assert_close(a, b, rtol=0, atol=0)

    def test_shapes_and_candidate_order_equivariance(self):
        state = torch.zeros(7, 4)
        action = torch.randn(7, 3, generator=torch.Generator().manual_seed(9))
        permutation = torch.tensor([6, 2, 1, 5, 3, 0, 4])
        for model, shape in ((StageQ(4, 3), (7,)), (StageSupervised(4, 3), (7, 3))):
            output = model(state, action)
            self.assertEqual(output.shape, shape)
            torch.testing.assert_close(model(state[permutation], action[permutation]), output[permutation])
            self.assertTrue(all(parameter.device.type == "cpu" for parameter in model.parameters()))
        q1, q2 = TwinCritic(4)(state, action)
        self.assertEqual(q1.shape, (7,))
        self.assertEqual(q2.shape, (7,))

    def test_dqn_selects_current_pool_and_respects_permutation(self):
        agent = DQNAgent(4, 3, width=8)
        agent.online[0] = AffineQ(1.0)
        for count in (1, 2, 17):
            candidates = np.zeros((count, 3), dtype=np.float32)
            candidates[:, 0] = np.arange(count)
            permutation = np.random.default_rng(count).permutation(count)
            self.assertEqual(agent.select(np.zeros(4), candidates, 0), count - 1)
            chosen = agent.select(np.zeros(4), candidates[permutation], 0)
            self.assertEqual(permutation[chosen], count - 1)
        for stage in (0, 1, 2):
            with self.assertRaisesRegex(ValueError, "empty"):
                agent.select(np.zeros(4), np.empty((0, 3)), stage, epsilon=1.0)

    def test_dqn_exploration_uses_supplied_rng_and_candidate_count(self):
        agent = DQNAgent(4, 3, width=8)
        candidates = np.zeros((5, 3), dtype=np.float32)
        a, b = np.random.default_rng(45), np.random.default_rng(45)
        first = [agent.select(np.zeros(4), candidates, 0, 1.0, a) for _ in range(40)]
        second = [agent.select(np.zeros(4), candidates, 0, 1.0, b) for _ in range(40)]
        self.assertEqual(first, second)
        self.assertEqual(set(first), set(range(5)))

    def test_double_dqn_online_selects_target_evaluates_next_stage(self):
        agent = DQNAgent(4, 3, width=8)
        agent.online[0], agent.target[0] = AffineQ(0.0, 999.0), AffineQ(0.0, 999.0)
        agent.online[1], agent.target[1] = AffineQ(1.0), AffineQ(-20.0, 100.0)
        agent.online[2], agent.target[2] = AffineQ(-1.0), AffineQ(10.0)
        pool = np.array([[1, 0, 0], [3, 0, 0], [2, 0, 0]], dtype=np.float32)
        rows = [transition(stage=0, next_candidates=pool),
                transition(stage=1, next_candidates=pool),
                transition(stage=2, next_candidates=pool)]
        targets = agent.compute_targets(rows)
        torch.testing.assert_close(targets, torch.tensor([42.0, 12.0, 2.0]))
        self.assertFalse(targets.requires_grad)
        rows[0]["next_candidates"] = pool[[2, 0, 1]]
        torch.testing.assert_close(agent.compute_targets(rows), targets)

    def test_dqn_terminal_empty_and_missing_next_data_do_not_bootstrap(self):
        agent = DQNAgent(4, 3, width=8)
        rows = [transition(terminated=True, next_state=np.full(4, np.nan)),
                transition(next_candidates=np.empty((0, 3))),
                transition(next_candidates=[]), transition(next_candidates=None),
                transition(next_state=None), transition(stage=2, terminated=False)]
        torch.testing.assert_close(agent.compute_targets(rows), torch.full((len(rows),), 2.0))

    def test_dqn_uses_huber_loss(self):
        agent = DQNAgent(4, 3, width=8)
        with torch.no_grad():
            for parameter in agent.online.parameters():
                parameter.zero_()
        metrics = agent.update([transition(stage=2, terminated=True, reward=10.0)])
        self.assertAlmostEqual(metrics["loss"], 9.5)
        self.assertAlmostEqual(metrics["target_mean"], 10.0)

    def test_dqn_soft_target_update(self):
        agent = DQNAgent(4, 3, width=8, tau=0.25)
        before = [parameter.detach().clone() for parameter in agent.target.parameters()]
        agent.update(replay())
        for initial, online, target in zip(before, agent.online.parameters(), agent.target.parameters()):
            torch.testing.assert_close(target, initial * 0.75 + online.detach() * 0.25)
            self.assertFalse(target.requires_grad)
            self.assertIsNone(target.grad)

    def test_actor_logprob_matches_transformed_normal(self):
        actor = GaussianActor(4, 3, width=8)
        with torch.no_grad():
            for parameter in actor.parameters():
                parameter.zero_()
            actor.net[-1].bias[3:] = -0.5
        state = torch.zeros(8, 4)
        action, logprob = actor(state, generator=torch.Generator().manual_seed(24))
        distribution = torch.distributions.Normal(torch.zeros_like(action), torch.full_like(action, np.exp(-0.5)))
        expected = (distribution.log_prob(torch.atanh(action)) - torch.log1p(-action.square())).sum(-1)
        torch.testing.assert_close(logprob, expected, atol=2e-6, rtol=2e-6)
        torch.testing.assert_close(actor.deterministic(state), torch.zeros_like(action))

    def test_actor_saturated_actions_have_finite_logprob_and_gradients(self):
        actor = GaussianActor(4, 3, width=8)
        with torch.no_grad():
            for parameter in actor.parameters():
                parameter.zero_()
            actor.net[-1].bias[:3] = torch.tensor([100.0, -100.0, 0.0])
            actor.net[-1].bias[3:] = -20.0
        action, logprob = actor.sample(torch.zeros(4, 4), generator=torch.Generator().manual_seed(3))
        self.assertTrue((action.abs() <= 1.0).all())
        self.assertTrue(torch.isfinite(logprob).all())
        self.assertTrue(torch.equal(action[:, :2], torch.tensor([[1.0, -1.0]]).expand(4, -1)))
        (action.sum() + logprob.sum()).backward()
        self.assertTrue(all(parameter.grad is not None and torch.isfinite(parameter.grad).all()
                            for parameter in actor.parameters()))

    def test_sac_targets_use_next_actor_next_target_critics_and_entropy(self):
        agent = SACAgent(4, width=8)
        agent.actors[0], agent.actors[1], agent.actors[2] = (
            FixedActor(0.99, -99.0), FixedActor(0.2, -2.0), FixedActor(0.7, -3.0))
        agent.target_critics[0], agent.target_critics[1], agent.target_critics[2] = (
            AffineTwin(999.0), AffineTwin(10.0), AffineTwin(20.0))
        rows = [transition(stage=0, next_candidates=None),
                transition(stage=1, next_candidates=None),
                transition(stage=2, terminated=False),
                transition(terminated=True, next_state=np.full(4, np.nan)),
                transition(next_candidates=np.empty((0, 15))),
                transition(next_state=None)]
        targets = agent.compute_targets(rows)
        torch.testing.assert_close(targets, torch.tensor([4.1, 16.15, 2.0, 2.0, 2.0, 2.0]))
        self.assertEqual([actor.calls for actor in agent.actors], [0, 1, 1])
        self.assertFalse(targets.requires_grad)
        self.assertEqual(agent.config["alpha"], 0.05)
        self.assertEqual(agent.config["gamma"], 1.0)

    def test_sac_actor_receives_q_gradient_even_with_zero_entropy(self):
        agent = SACAgent(4, width=8, alpha=0.0, seed=3)
        actor, critic = agent.actors[0], agent.critics[0]
        with torch.no_grad():
            for parameter in actor.parameters():
                parameter.zero_()
            for q in (critic.q1, critic.q2):
                for parameter in q.parameters():
                    parameter.zero_()
                q.net[0].weight[0, 4] = 1.0
                q.net[0].bias[0] = 2.0
                q.net[2].weight[0, 0] = 1.0
                q.net[4].weight[0, 0] = 1.0
        # Hold Q fixed at action[0]+2 to isolate the actor's Q derivative.
        agent.critic_optimizers[0].param_groups[0]["lr"] = 0.0
        agent.update([transition(terminated=True) for _ in range(6)])
        self.assertGreater(float(actor.net[-1].bias[0].detach()), 0.0)
        self.assertTrue(all(parameter.requires_grad for parameter in critic.parameters()))
        self.assertTrue(all(parameter.grad is None for parameter in critic.parameters()))
        self.assertTrue(all(parameter.grad is None for parameter in agent.target_critics.parameters()))

    def test_sac_soft_target_update(self):
        agent = SACAgent(4, width=8, tau=0.2)
        before = [parameter.detach().clone() for parameter in agent.target_critics.parameters()]
        agent.update(replay())
        for initial, online, target in zip(before, agent.critics.parameters(), agent.target_critics.parameters()):
            torch.testing.assert_close(target, initial * 0.8 + online.detach() * 0.2)
            self.assertFalse(target.requires_grad)

    def test_short_updates_are_finite_and_change_all_stage_parameters(self):
        for agent in (DQNAgent(4, 3, width=8), SACAgent(4, width=8)):
            modules = ([agent.online] if isinstance(agent, DQNAgent)
                       else [agent.actors, agent.critics])
            before = [[list(model.parameters())[-1].detach().clone() for model in collection]
                      for collection in modules]
            for _ in range(4):
                metrics = agent.update(replay())
                self.assertTrue(all(np.isfinite(value) for value in metrics.values()))
            for initial_collection, collection in zip(before, modules):
                for initial, model in zip(initial_collection, collection):
                    self.assertFalse(torch.equal(initial, list(model.parameters())[-1]))
                    self.assertTrue(all(parameter.device.type == "cpu" for parameter in model.parameters()))

    def test_fixed_seeds_reproduce_initialization_and_updates(self):
        global_rng = torch.random.get_rng_state().clone()
        for cls in (DQNAgent, SACAgent):
            first = cls(4, 3, width=8, seed=68)
            second = cls(4, 3, width=8, seed=68)
            for _ in range(3):
                self.assertEqual(first.update(replay()), second.update(replay()))
            for name in (("online", "target") if cls is DQNAgent else ("actors", "critics", "target_critics")):
                self.assertParametersEqual(getattr(first, name), getattr(second, name))
        torch.testing.assert_close(torch.random.get_rng_state(), global_rng, atol=0, rtol=0)

    def test_save_load_restores_predictions_optimizers_and_random_streams(self):
        with tempfile.TemporaryDirectory() as folder:
            for cls in (DQNAgent, SACAgent):
                agent = cls(4, 3, width=8, seed=68)
                agent.update(replay())
                path = Path(folder) / (cls.__name__ + ".pt")
                agent.save(path)
                restored = cls.load(path)
                self.assertEqual(agent.config, restored.config)
                if cls is DQNAgent:
                    for _ in range(10):
                        self.assertEqual(agent.select(np.zeros(4), np.zeros((9, 3)), 1, 1.0),
                                         restored.select(np.zeros(4), np.zeros((9, 3)), 1, 1.0))
                    self.assertParametersEqual(agent.online, restored.online)
                    self.assertParametersEqual(agent.target, restored.target)
                else:
                    for deterministic in (True, False, False):
                        np.testing.assert_array_equal(agent.select(np.zeros(4), 1, deterministic),
                                                      restored.select(np.zeros(4), 1, deterministic))
                    self.assertParametersEqual(agent.critics, restored.critics)
                    self.assertParametersEqual(agent.target_critics, restored.target_critics)
                self.assertEqual(agent.update(replay()), restored.update(replay()))
                online = "online" if cls is DQNAgent else "actors"
                self.assertParametersEqual(getattr(agent, online), getattr(restored, online))
                for state in restored.state_dict().values():
                    if isinstance(state, dict):
                        self.assertTrue(all(value.device.type == "cpu" for value in state.values()
                                            if isinstance(value, torch.Tensor)))

    def test_actor_mask_zeros_idle_and_excludes_idle_entropy_and_gradients(self):
        actor = GaussianActor(7, 3, width=8)
        with torch.no_grad():
            for parameter in actor.parameters():
                parameter.zero_()
            actor.net[-1].bias.copy_(torch.tensor([.1, .2, .3, -.4, -.5, -.6]))
        states = torch.zeros(4, 7)
        mask = torch.tensor([[1., 1., 0.]]).expand(4, -1)
        sampled, logprob = actor.sample(states, generator=torch.Generator().manual_seed(91), action_mask=mask)
        noise = torch.randn((4, 3), generator=torch.Generator().manual_seed(91))
        mean, log_std = actor._parameters_for(states)
        latent = mean + log_std.exp() * noise
        expected = (-.5 * (noise.square() + 2 * log_std + np.log(2 * np.pi))
                    - 2 * (np.log(2) - latent - torch.nn.functional.softplus(-2 * latent)))
        torch.testing.assert_close(logprob, (expected * mask).sum(-1))
        self.assertTrue((sampled[:, 2] == 0).all())
        self.assertTrue((actor.deterministic(states, action_mask=mask)[:, 2] == 0).all())
        (sampled.sum() + logprob.sum()).backward()
        self.assertEqual(float(actor.net[-1].bias.grad[2]), 0.0)
        self.assertEqual(float(actor.net[-1].bias.grad[5]), 0.0)

    def test_sac_mask_applies_to_replay_current_actor_and_next_target(self):
        agent = SACAgent(7, width=8, action_mask_state_indices=(5, 6), seed=32)
        states = [np.array([0., 0., 0., 0., 0., 1., 0.]),
                  np.array([0., 0., 0., 0., 0., 0., 1.])]
        batch = [transition(state_dim=7, state=state, next_state=state.copy(),
                            action=np.array([.2, .3, .4]), next_candidates=None) for state in states]
        calls = {'current': [], 'next': []}
        current_hook = agent.critics[0].register_forward_pre_hook(
            lambda module, args: calls['current'].append(args[1].detach().clone()))
        target_hook = agent.target_critics[1].register_forward_pre_hook(
            lambda module, args: calls['next'].append(args[1].detach().clone()))
        try:
            metrics = agent.update(batch)
        finally:
            current_hook.remove(); target_hook.remove()
        self.assertTrue(all(np.isfinite(value) for value in metrics.values()))
        self.assertEqual(len(calls['current']), 2)
        self.assertEqual(len(calls['next']), 1)
        for actions in calls['current'] + calls['next']:
            self.assertEqual(float(actions[0, 2]), 0.)
            self.assertEqual(float(actions[1, 1]), 0.)
        for state, inactive in zip(states, (2, 1)):
            self.assertEqual(agent.select(state, 0, deterministic=False)[inactive], 0.)
            self.assertEqual(agent.select(state, 0, deterministic=True)[inactive], 0.)

    def test_sac_mask_contract_roundtrips_and_rejects_bad_flags(self):
        agent = SACAgent(7, width=8, action_mask_state_indices=(5, 6))
        state = np.array([0., 0., 0., 0., 0., 1., 0.])
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'masked.pt'
            agent.save(path)
            restored = SACAgent.load(path)
            self.assertEqual(restored.config['action_mask_state_indices'], [5, 6])
            np.testing.assert_array_equal(agent.select(state, 0), restored.select(state, 0))
        for flags in ((0, 0), (.5, 1), (1, 2)):
            invalid = state.copy(); invalid[5:] = flags
            with self.assertRaisesRegex(ValueError, 'flags'):
                agent.select(invalid, 0)
        for indices in ((5, 5), (5, 7), (5,)):
            with self.assertRaisesRegex(ValueError, 'mask'):
                SACAgent(7, width=8, action_mask_state_indices=indices)
        with self.assertRaisesRegex(ValueError, 'mask'):
            SACAgent(7, action_dim=4, action_mask_state_indices=(5, 6))

    def test_rejects_cuda_empty_batches_bad_shapes_and_nonfinite_inputs(self):
        for cls in (DQNAgent, SACAgent):
            with self.assertRaisesRegex(ValueError, "CPU-only"):
                cls(4, 3, device="cuda")
            agent = cls(4, 3, width=8)
            with self.assertRaises(ValueError):
                agent.update([])
            with self.assertRaises(ValueError):
                agent.update([transition(stage=3)])
            with self.assertRaises(ValueError):
                agent.update([transition(state=np.zeros(5))])
            with self.assertRaises(ValueError):
                agent.update([transition(reward=float("nan"))])
        agent = SACAgent(4, width=8)
        with self.assertRaisesRegex(ValueError, r"\[-1, 1\]"):
            agent.update([transition(action=np.full(3, 1.01))])


if __name__ == "__main__":
    unittest.main()
