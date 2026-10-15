import argparse
import copy
from dataclasses import replace
import gc
import json
from pathlib import Path
import time

import torch

from curobo_core.contracts import PlannerAssets
from curobo_core.scene import Pose
from v3_batched_loaded_search import GpuValidity
from v3_persistent_collision_planner import PersistentCollisionPlanner
from v3_priority_contact_planner import PriorityContactPlanner


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cycles', action='store_true')
    args = parser.parse_args()
    persistent = PersistentCollisionPlanner(PlannerAssets())
    fresh = PriorityContactPlanner(PlannerAssets())
    original = (persistent.snapshot.identity, copy.deepcopy(persistent.target_poses), dict(persistent.selected_task))
    persistent.warmup_resources()
    assert original == (persistent.snapshot.identity, persistent.target_poses, persistent.selected_task)
    cases = [({'left': 22, 'right': 17}, False, False, False),
             ({'left': 18, 'right': 16}, False, False, False),
             ({'left': 18, 'right': 16}, True, False, False),
             ({'left': 22, 'right': 17}, True, False, True),
             ({'left': 18, 'right': 16}, False, False, True),
             ({'left': 9, 'right': 1}, False, True, False),
             ({'left': 9, 'right': 1}, True, False, False),
             ({'left': 4, 'right': 2}, True, False, True),
             ({'left': 4, 'right': 2}, False, False, True),
             ({'left': 13, 'right': 15}, False, False, False),
             ({'left': 13, 'right': 15}, False, False, False)]
    summaries = []
    for index, (boxes, payload, mobile, parked) in enumerate(cases):
        persistent.configure_task(boxes)
        if parked:
            persistent.snapshot = replace(persistent.snapshot, state=persistent.snapshot.state.with_positions({}, Pose((-.1, 0., 0.), (0., 0., 0., 1.))))
        if index == 9:
            persistent.snapshot = replace(persistent.snapshot, objects=tuple(item for item in persistent.snapshot.objects if not item.object_id.startswith('task_fill_')))
        if index == 10:
            item = persistent.snapshot.objects[-1]
            persistent.snapshot = replace(persistent.snapshot, objects=persistent.snapshot.objects[:-1] +
                (replace(item, dimensions_m=tuple(value * 1.01 for value in item.dimensions_m)),))
        fresh.configure_task(boxes)
        fresh.snapshot = persistent.snapshot
        fresh._cached_task = None
        fresh._cached_checkers.clear()
        began = time.perf_counter()
        actual = persistent.checker(boxes, payload=payload, mobile=mobile)
        update_ms = (time.perf_counter() - began) * 1000
        expected = fresh.checker(boxes, payload=payload, mobile=mobile)
        lower, upper = expected.kinematics.get_joint_limits().position
        generator = torch.Generator(device='cuda').manual_seed(20261015 + index)
        values = lower + (upper - lower) * torch.rand((1024, len(lower)), device='cuda', generator=generator)
        names = expected.kinematics.joint_names
        first = GpuValidity(actual, check_ground=True, joint_names=names).mask(values)
        second = GpuValidity(expected, check_ground=True, joint_names=names).mask(values)
        assert torch.equal(first, second), f'world update differs from fresh checker in case {index}'
        summaries.append({'case': index, 'task': boxes, 'payload': payload, 'mobile': mobile,
                          'parked': parked, 'states': len(values), 'matches_fresh': True, 'call_ms': update_ms})
        del actual, expected, values, first, second
        fresh._cached_checkers.clear()
        gc.collect()
        torch.cuda.empty_cache()
        print(json.dumps(summaries[-1]), flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cycles = []
    if args.cycles:
        from v3_conveyor_regression import audit
        for boxes in [cases[index][0] for index in (0, 1, 5, 7)]:
            persistent.configure_task(boxes)
            result = persistent.full_cycle(boxes, lambda text: print(text, flush=True))
            path = args.output.parent / f"persistent_L{boxes['left']}_R{boxes['right']}.json"
            path.write_text(json.dumps(result, ensure_ascii=False))
            if not result['success']:
                raise SystemExit(f"Unexpected cycle failure: {boxes}: {result.get('blocked_details')}")
            cycles.append({'task': boxes, 'total_ms': result['total_ms'], 'audit': audit(result, PlannerAssets()),
                           'resource_created': sum(event['created'] for event in result['checker_resource_events']),
                           'resource_updated': sum(event['updated'] for event in result['checker_resource_events'])})
    args.output.write_text(json.dumps({'cases': summaries, 'warmup_restored_state': True,
        'scope': 'actual cuboid scene updates; no synthetic unsupported mesh test', 'total_states': 11264,
        'cycles': cycles}, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
