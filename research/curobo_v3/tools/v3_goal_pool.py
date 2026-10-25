"""Exact goal-pool acceleration for MOTION-311 (inputs already weighted).

Fast vector norms identify a conservative superset of the nearest goals; the original direct cdist kernel
recomputes that superset before the original topk. Values outside it cannot reach the boundary. If a dense
cluster or nonfinite input defeats the guard, use the original full direct kernel. No approximate IK ranking.
"""
import torch


def direct_goal_pool(ends, rows, count=512):
    distances = torch.cdist(ends, rows, compute_mode='donot_use_mm_for_euclid_dist')
    return torch.topk(distances, min(count, len(rows)), largest=False).indices


def exact_goal_pool(ends, rows, count=512):
    count = min(count, len(rows))
    if len(rows) <= 2 * count or not len(ends):
        return direct_goal_pool(ends, rows, count)
    # Both norms sum 15 nonnegative squared float32 differences, but reduction order differs.
    # 64 eps with an absolute floor is conservative for that rounding error on these joint values.
    distances = torch.linalg.vector_norm(ends[:, None] - rows[None], dim=-1)
    width = min(2 * count, len(rows))
    approx, indices = torch.topk(distances, width, largest=False)
    cutoff = approx[:, count - 1]
    guard = 64 * torch.finfo(ends.dtype).eps * cutoff.abs().clamp_min(1.)
    uncertain = ~torch.isfinite(approx).all() | ((approx[:, -1] - cutoff) <= guard).any()
    if bool(uncertain):
        return direct_goal_pool(ends, rows, count)
    candidates = rows[indices]
    exact = torch.cdist(ends[:, None], candidates, compute_mode='donot_use_mm_for_euclid_dist')[:, 0]
    # topk tie order can depend on discarded values, even at full width. Recompute
    # tied query rows with the original kernel, including ties inside the retained pool.
    nearest = torch.topk(exact, count + 1, largest=False).values
    tied = (nearest[:, 1:] == nearest[:, :-1]).any(dim=1)
    distances.fill_(torch.inf)
    distances.scatter_(1, indices, exact)
    result = torch.topk(distances, count, largest=False).indices
    if bool(tied.any()):
        result[tied] = direct_goal_pool(ends[tied], rows, count)
    return result
