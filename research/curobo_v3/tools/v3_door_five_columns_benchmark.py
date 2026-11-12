#!/usr/bin/env python3
"""Resident-planner five-column door benchmark: discarded warmup pass(es), then fresh no-plan-cache sequences.

Every measured round builds a new request and task and replans IK/RRT/collision/timing/RNEA;
only checker/solver/kinematics/torque resources may persist on the planner between rounds.
"""
import argparse
import json
from pathlib import Path
import time

from curobo_core.door_recenter import DOOR_COLUMN_ORDER, DoorPlanner, ROOT
from v3_door_five_columns import acquire_gpu, load_assets, run_sequence

LIMIT_MS = 10000.


def round_rows(aggregate):
    rows = []
    for record in aggregate['rounds']:
        report = json.loads(Path(record['path']).read_text())
        rows.append(dict(column=record['column'], success=record['success'], total_ms=record['total_ms'],
                         motion_duration_s=record['motion_duration_s'], error=record['error'],
                         symmetry_derived=bool(report.get('plan_cache', {}).get('symmetry_derived')),
                         plan_cache=report.get('plan_cache'), stage_timing_ms=report.get('stage_timing_ms', {}),
                         stage_breakdown_ms=report.get('stage_breakdown_ms', {}),
                         resource_builds=record['resource_builds'], peak_gpu_memory_mb=record['peak_gpu_memory_mb'],
                         resident_gpu_memory_mb=record['resident_gpu_memory_mb']))
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--assets-root', type=Path, required=True)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--seeds', type=int, nargs='+', default=(11, 29, 41))
    p.add_argument('--warmup-seed', type=int, default=5)
    p.add_argument('--warmup-passes', type=int, default=1,
                   help='discarded full five-column passes before measuring; 0 measures first sight')
    p.add_argument('--wait-for-gpu-seconds', type=float, default=0.)
    p.add_argument('--free-space-inflation', type=float, default=None,
                   help="override the model's free-space search geometry scale (0: no inflation)")
    p.add_argument('--free-space-stages', nargs='+', default=None,
                   help="override the stages whose free-space searches inflate (e.g. conveyor_suffix)")
    p.add_argument('--no-coverage-margins', action='store_true',
                   help="plan without the model's MOTION-429 sphere coverage margins")
    a = p.parse_args()
    a.output_dir.mkdir(parents=True, exist_ok=True)
    if any(a.output_dir.iterdir()):
        p.error('output directory must be empty')
    log = lambda text: print(text, flush=True)
    with open('/tmp/sevenova-curobo-gpu.lock', 'a') as lock:
        acquire_gpu(lock, a.wait_for_gpu_seconds)
        assets = load_assets(a.assets_root)
        policy_path = ROOT/'config/placement_task_policy.json'
        policy = json.loads(policy_path.read_text())
        started = time.perf_counter()
        planner = DoorPlanner(assets)
        construction_ms = (time.perf_counter()-started)*1000
        if a.free_space_inflation is not None:
            planner.free_space_inflation = a.free_space_inflation or None
        if a.free_space_stages is not None:
            planner.free_space_stages = tuple(a.free_space_stages)
        if a.no_coverage_margins:
            planner.sphere_margins = None
        warmups = []
        for index in range(a.warmup_passes):
            began = time.perf_counter()
            aggregate = run_sequence(planner, a.warmup_seed, policy, policy_path, assets,
                                     a.output_dir/f'warmup-{index+1}-s{a.warmup_seed}', progress=log)
            warmups.append(dict(seed=a.warmup_seed, success=aggregate['success'],
                                wall_ms=(time.perf_counter()-began)*1000, rounds=round_rows(aggregate)))
        measured = []
        for seed in a.seeds:
            aggregate = run_sequence(planner, seed, policy, policy_path, assets, a.output_dir/f's{seed}', progress=log)
            measured.append(dict(seed=seed, success=aggregate['success'], total_ms=aggregate['total_ms'],
                                 motion_duration_s=aggregate['motion_duration_s'], rounds=round_rows(aggregate)))
    columns = {}
    for item in measured:
        for row in item['rounds']:
            entry = columns.setdefault(row['column'], dict(column=row['column'], seeds={}, symmetry_derived=False))
            entry['seeds'][item['seed']] = dict(success=row['success'], total_ms=row['total_ms'],
                                               motion_duration_s=row['motion_duration_s'])
            entry['symmetry_derived'] |= row['symmetry_derived']
    for entry in columns.values():
        values = [item['total_ms'] for item in entry['seeds'].values()]
        entry.update(max_ms=max(values), all_success=all(item['success'] for item in entry['seeds'].values()) and
                     len(entry['seeds']) == len(a.seeds),
                     under_limit=max(values) < LIMIT_MS)
    planned = [columns.get(column) for column in DOOR_COLUMN_ORDER]
    summary = dict(trajectory_cache=False, limit_ms=LIMIT_MS, seeds=list(a.seeds), warmup_seed=a.warmup_seed,
                   warmup_passes=a.warmup_passes, planner_construction_ms=construction_ms,
                   model_revision=planner.model_revision, free_space_inflation=planner.free_space_inflation,
                   free_space_stages=planner.free_space_stages,
                   coverage_margins=None if planner.sphere_margins is None else
                   {link: round(float(max(values)), 4) for link, values in planner.sphere_margins.items()},
                   warmups=[{k: v for k, v in item.items() if k != 'rounds'} |
                            dict(rounds=[dict(column=r['column'], success=r['success'], total_ms=r['total_ms'])
                                         for r in item['rounds']]) for item in warmups],
                   columns=[item for item in planned if item],
                   independently_planned_under_limit=all(item and item['all_success'] and item['under_limit']
                                                         for item in planned if not (item and item['symmetry_derived'])),
                   measured=measured)
    (a.output_dir/'benchmark-summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=1)+'\n')
    log(json.dumps({k: summary[k] for k in ('independently_planned_under_limit', 'columns')}, ensure_ascii=False))
    return 0 if all(item['success'] for item in measured) else 1


if __name__ == '__main__':
    raise SystemExit(main())
