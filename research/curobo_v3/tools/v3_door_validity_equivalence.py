#!/usr/bin/env python3
"""Shadow-check the native-bracketed loaded self-collision against the float64 reference.

Every loaded `_self_collision_cost` call of one real five-column sequence is decided twice
(new bracketed native kernel, and the float64 zero-tolerance pair check on every row). Rows
from those calls are then bisected to the reference decision boundary and both decisions are
compared again at and around the boundary.
"""
import argparse
import json
from pathlib import Path
import time

import torch

from curobo_core.door_recenter import DoorPlanner, ROOT
from curobo_core.door_sequential_validity import SequentialValidity
from v3_door_five_columns import acquire_gpu, load_assets, run_sequence


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--assets-root', type=Path, required=True)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--seed', type=int, default=11)
    p.add_argument('--samples-per-call', type=int, default=8)
    p.add_argument('--boundary-pairs', type=int, default=4000)
    p.add_argument('--wait-for-gpu-seconds', type=float, default=0.)
    a = p.parse_args()
    a.output_dir.mkdir(parents=True, exist_ok=True)
    if any(a.output_dir.iterdir()):
        p.error('output directory must be empty')
    stats = dict(calls=0, rows=0, mismatches=0, reference_collisions=0, band_rows=0)
    samples = {}
    original = SequentialValidity._self_collision_cost
    generator = torch.Generator(device='cuda')
    generator.manual_seed(20261108)

    def shadow(self, spheres):
        new = original(self, spheres)
        if not self.payload_indices:
            return new
        rows = spheres.reshape(-1, spheres.shape[-2], 4)
        reference = self.exact_robot_self_collision(rows)
        decided = new.reshape(-1) > 0
        stats['calls'] += 1
        stats['rows'] += len(rows)
        stats['mismatches'] += int((decided != reference).sum())
        stats['reference_collisions'] += int(reference.sum())
        bucket = samples.setdefault(id(self), dict(validity=self, hit=[], free=[]))
        for name, selected in (('hit', reference), ('free', ~reference)):
            index = torch.nonzero(selected).reshape(-1)
            if len(index):
                pick = index[torch.randint(len(index), (min(a.samples_per_call, len(index)),),
                                           device='cuda', generator=generator)]
                bucket[name].append(rows[pick].clone())
        return new

    SequentialValidity._self_collision_cost = shadow
    log = lambda text: print(text, flush=True)
    with open('/tmp/sevenova-curobo-gpu.lock', 'a') as lock:
        acquire_gpu(lock, a.wait_for_gpu_seconds)
        assets = load_assets(a.assets_root)
        policy_path = ROOT/'config/placement_task_policy.json'
        policy = json.loads(policy_path.read_text())
        planner = DoorPlanner(assets)
        began = time.perf_counter()
        aggregate = run_sequence(planner, a.seed, policy, policy_path, assets, a.output_dir/'sequence', progress=log)
        sequence_ms = (time.perf_counter()-began)*1000
        SequentialValidity._self_collision_cost = original
        boundary = dict(pairs=0, rows=0, mismatches=0, band_rows=0)
        usable = [item for item in samples.values() if item['hit'] and item['free']]
        per_bucket = max(1, a.boundary_pairs // max(1, len(usable)))
        offsets = torch.tensor([0., 1e-9, -1e-9, 1e-7, -1e-7, 1e-5, -1e-5, 1e-3, -1e-3], device='cuda',
                               dtype=torch.float64)
        for item in usable:
            validity = item['validity']
            hit, free = torch.cat(item['hit']), torch.cat(item['free'])
            count = min(per_bucket, len(hit), len(free))
            first = hit[torch.randperm(len(hit), device='cuda', generator=generator)[:count]].double()
            second = free[torch.randperm(len(free), device='cuda', generator=generator)[:count]].double()
            low = torch.zeros(count, device='cuda', dtype=torch.float64)  # reference collides at t=0
            high = torch.ones(count, device='cuda', dtype=torch.float64)  # reference is free at t=1

            def rows_at(t):
                values = first.clone()
                values[..., :3] = first[..., :3]+t[:, None, None]*(second[..., :3]-first[..., :3])
                return values.float()

            for _ in range(48):
                middle = (low+high)/2
                collide = validity.exact_robot_self_collision(rows_at(middle))
                low = torch.where(collide, middle, low)
                high = torch.where(collide, high, middle)
            before = validity.self_band_rows
            for offset in offsets:
                t = ((low+high)/2+offset).clamp(0., 1.)
                values = rows_at(t)
                reference = validity.exact_robot_self_collision(values)
                decided = validity._self_collision_cost(values[None]).reshape(-1) > 0
                boundary['mismatches'] += int((decided != reference).sum())
                boundary['rows'] += len(values)
            boundary['band_rows'] += validity.self_band_rows-before
            boundary['pairs'] += count
        stats['band_rows'] = sum(item['validity'].self_band_rows for item in samples.values())
        # finalize_timing reads limits once: every loaded/empty checker must report the same limits.
        checkers = list(getattr(planner, '_door_checker_cache', {}).values())
        reference = checkers[0].kinematics.get_joint_limits()
        limits_equal = all(all(torch.equal(getattr(reference, k), getattr(item.kinematics.get_joint_limits(), k))
                               for k in ('position', 'velocity', 'acceleration', 'jerk')) for item in checkers)
        sphere_counts = sorted({int(item.kinematics.get_joint_limits().position.shape[-1]) for item in checkers})
    output = dict(success=aggregate['success'] and stats['mismatches'] == 0 and boundary['mismatches'] == 0
                  and stats['rows'] > 0 and boundary['rows'] > 0 and limits_equal,
                  sequence_success=aggregate['success'], sequence_ms=sequence_ms,
                  rounds=[{k: item[k] for k in ('column', 'success', 'total_ms', 'error')} for item in aggregate['rounds']],
                  real_states=stats, boundary=boundary, margin_m=1e-5,
                  joint_limits=dict(checkers=len(checkers), identical=limits_equal, dof=sphere_counts))
    (a.output_dir/'validity-equivalence.json').write_text(json.dumps(output, ensure_ascii=False, indent=1)+'\n')
    log(json.dumps(output, ensure_ascii=False))
    return 0 if output['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
