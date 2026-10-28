"""Candidate orderings for the screened selector; the first candidate passing the gates is executed."""
import numpy as np
import torch

from motion311_learning.dataset import normalized_actions, normalized_state


def heuristic_order(observation, velocity):
    """MOTION-311 local heuristic as a full ordering: current-stage movement time, plus return-home at placement."""
    state, rows = observation.state, observation.candidates
    previous, home = state[7:22], state[22:37]
    costs = np.max(np.abs(rows - previous) / velocity, axis=1)
    if observation.stage == 2:
        costs = costs + np.max(np.abs(rows - home) / velocity, axis=1)
    return np.argsort(costs, kind='stable')


def lookahead_heuristic_order(observation):
    """Analytic lookahead baseline on the same features: at contact, approach + extraction + nearest gated carry
    goal from the solved extraction endpoint; later stages equal the local heuristic (move, plus Home at placement)."""
    extra = np.asarray(observation.extra)
    if observation.stage == 0:
        costs = extra[:, 0] + extra[:, 1] + extra[:, 3]
    elif observation.stage == 1:
        costs = extra[:, 0]
    else:
        costs = extra[:, 0] + extra[:, 1]
    return np.argsort(costs, kind='stable')


def uniform_order(observation, rng):
    return rng.permutation(len(observation.candidates))


@torch.no_grad()
def dqn_scores(agent, observation):
    rows = torch.as_tensor(normalized_actions(observation.candidates))
    state = torch.as_tensor(normalized_state(observation.state)).expand(len(rows), -1)
    return agent.online[observation.stage](state, rows).numpy()


def dqn_order(agent, observation):
    """Descending Q; the stable sort keeps DQNAgent.select's first-maximum tie rule for the head."""
    return np.argsort(-dqn_scores(agent, observation), kind='stable')
