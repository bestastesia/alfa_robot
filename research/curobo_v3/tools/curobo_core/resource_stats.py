"""Per-planner build counters for heavy GPU resources (checkers, IK solvers, torque models)."""
import time

import torch


def record_build(owner, kind, started):
    """Count one resource build on `owner` and its synchronized wall time since `started`."""
    torch.cuda.synchronize()
    stats = owner.__dict__.setdefault('resource_stats', {})
    item = stats.setdefault(kind, {'builds': 0, 'ms': 0.})
    item['builds'] += 1
    item['ms'] += (time.perf_counter()-started)*1000


def resource_snapshot(owner):
    return {kind: dict(item) for kind, item in getattr(owner, 'resource_stats', {}).items()}


def resource_delta(before, after):
    """Builds and milliseconds added between two snapshots, per resource kind."""
    return {kind: {'builds': item['builds']-before.get(kind, {}).get('builds', 0),
                   'ms': item['ms']-before.get(kind, {}).get('ms', 0.)}
            for kind, item in after.items()
            if item['builds'] != before.get(kind, {}).get('builds', 0)}


def bounded_get(cache, key, build, limit):
    """LRU lookup in a planner-resident resource cache: hits move to the end, and a build
    beyond `limit` entries drops the least recently used one (live users keep their reference)."""
    value = cache.pop(key, None)
    if value is None:
        value = build()
    cache[key] = value
    while len(cache) > limit:
        cache.pop(next(iter(cache)))
    return value
