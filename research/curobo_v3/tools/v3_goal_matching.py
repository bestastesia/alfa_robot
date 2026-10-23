"""Per-root placement-goal matching shared by the side and top root preparation.

The default order is the mentor rule: weighted joint distance plus the 17-point straight-line box
rotation penalty, then up to 8 goals spaced more than 0.35 apart. A planner may define
`placement_goal_order(root, shortlisted, distances, baseline_scores)` returning an index order over the
512 shortlisted goals (or None for the default); `baseline_scores()` computes the mentor scores lazily.
"""
import torch

from v3_box_rotation_cost import BoxRotationCost, quaternion_multiply, rotation_vector
from v3_wall_ik_benchmark import ACTIVE_JOINTS


def baseline_goal_scores(planner, root, shortlisted, distances, loaded):
    metric = BoxRotationCost(loaded, ACTIVE_JOINTS, root, shortlisted[0], planner.top_grasp)
    values = (root[None, None] + torch.linspace(0, 1, 17, device='cuda')[None, :, None] * (shortlisted[:, None] - root[None, None])).reshape(-1, 15).contiguous()
    count = len(shortlisted)
    orientations = metric.orientations(values)
    penalty = torch.zeros(count, device='cuda')
    for side, quaternions in orientations.items():
        quaternion = quaternions.reshape(count, 17, 4)
        motion = torch.linalg.vector_norm(rotation_vector(quaternion_multiply(metric.conjugate(quaternion[:, :-1]), quaternion[:, 1:])), dim=-1)
        metric.current_side = side
        progress, deviation = metric.terms(quaternions)
        progress = progress.reshape(count, 17)
        deviation = deviation.reshape(count, 17)
        penalty += 2 * torch.relu(motion.sum(-1) - metric.angles[side]) + deviation.mean(-1) + torch.relu(progress[:, :-1] - progress[:, 1:]).sum(-1)
    return distances + penalty


def match_goals(planner, root, candidates, weights, loaded):
    distances = torch.linalg.vector_norm((candidates - root) * weights, dim=-1)
    indices = torch.topk(distances, min(512, len(candidates)), largest=False).indices
    shortlisted = candidates[indices].contiguous()
    nearest = distances[indices]
    hook = getattr(planner, 'placement_goal_order', None)
    order = None if hook is None else hook(root, shortlisted, nearest, lambda: baseline_goal_scores(planner, root, shortlisted, nearest, loaded))
    if order is None:
        order = torch.argsort(baseline_goal_scores(planner, root, shortlisted, nearest, loaded)).cpu().tolist()
    return shortlisted[greedy_diverse(shortlisted, order, weights)].cpu().tolist()


def greedy_diverse(shortlisted, order, weights, spacing=.35, count=8):
    selected = []
    for index in order:
        if not selected or torch.linalg.vector_norm((shortlisted[selected] - shortlisted[index]) * weights, dim=-1).min() > spacing:
            selected.append(index)
        if len(selected) == count:
            break
    return selected
