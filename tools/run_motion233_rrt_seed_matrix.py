#!/usr/bin/env python3
"""Run the original public seed list once each, under the shared GPU lock; never cherry-pick reruns."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from verify_motion233_curobo_rebase import ROOT, SDK, cycle, git


def inventory(root):
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(root.rglob('*')) if p.is_file() and p.suffix in ('.py', '.yml', '.yaml', '.urdf', '.so', '.stl')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime-root', type=Path, required=True)
    parser.add_argument('--sdk-repository', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--case-timeout', type=float, default=600.)
    args = parser.parse_args()
    if args.case_timeout <= 0:
        parser.error('--case-timeout must be positive')
    original = git('show', 'b72ba30:tools/v3_scoop_golden_20260921/OMPL_SEEDS.json')
    manifest = ROOT/'research/curobo_v3/tests/fixtures/OMPL_SEEDS.json'
    assert manifest.read_bytes() == original, 'public seed set changed'
    seeds = json.loads(original)['seeds']
    assert git('rev-parse', 'HEAD', root=args.sdk_repository).decode().strip() == SDK
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if (args.output_dir/'matrix-plan.json').exists() or list(args.output_dir.glob('seed-*.json')):
        parser.error('output directory already contains a matrix; choose a fresh directory')
    before = inventory(args.runtime_root)
    plan = {'seeds': seeds, 'source_seed_manifest_sha256': hashlib.sha256(original).hexdigest(),
            'runtime_sha256': before, 'sdk_commit': SDK, 'planner': 'informed_rrt',
            'wall_distance_m': .75, 'case_timeout_s': args.case_timeout,
            'scope': 'RRT sampling only; f044 GPU-sphere policy, not full-mesh/FCL/hardware certification; wall-time budget does not guarantee bitwise replay',
            'source_commit_at_start': git('rev-parse', 'HEAD').decode().strip()}
    (args.output_dir/'matrix-plan.json').write_text(json.dumps(plan, indent=2)+'\n')
    env = dict(os.environ, PYTHONPATH=str(args.sdk_repository), PYTORCH_ALLOC_CONF='expandable_segments:True')
    records = []
    with open('/tmp/sevenova-curobo-gpu.lock', 'a') as lock:
        for seed in seeds:
            fcntl.flock(lock, fcntl.LOCK_EX)
            try:
                active = subprocess.check_output(['nvidia-smi', '--query-compute-apps=pid', '--format=csv,noheader'], text=True).strip()
                if active:
                    raise SystemExit(f'Another compute PID exists despite lock; not starting: {active}')
                output = args.output_dir/f'seed-{seed}.json'
                command = [sys.executable, '-u', 'tools/v3_plan_cycle.py', '--sequence', '--wall-distance', '.75',
                           '--search-seed', str(seed), '--output', str(output.resolve())]
                started = time.perf_counter()
                record = {'seed': seed, 'command': command, 'success': False}
                with (args.output_dir/f'seed-{seed}.log').open('w') as log:
                    try:
                        process = subprocess.run(command, cwd=args.runtime_root, env=env, stdout=log,
                                                 stderr=subprocess.STDOUT, timeout=args.case_timeout)
                        record['exit_code'] = process.returncode
                    except subprocess.TimeoutExpired:
                        record['error'] = 'PROCESS_TIMEOUT'
                record['process_wall_ms'] = (time.perf_counter()-started)*1000
                if output.exists():
                    data = json.loads(output.read_text())
                    assert data.get('search_seed') == seed
                    for r in data['rounds']:
                        if r['status'] != 'pending':
                            assert r['request'].get('search_seed') == seed
                            assert r['result'].get('search_seed') == seed
                            assert r['result'].get('planner') == 'informed_rrt'
                        if r['status'] == 'completed':
                            cycle(r['result'])
                    record.update(completed_boxes=len(data['completed_box_ids']), blocked_round=data.get('blocked_round'),
                                  error=record.get('error') or data.get('error'), planning_ms=data['total_ms'],
                                  per_round_planning_ms=[r['result']['total_ms'] for r in data['rounds'] if r['status'] == 'completed'])
                    record['success'] = (record.get('exit_code') == 0 and data['success'] and
                                         sorted(data['completed_box_ids']) == list(range(25)) and not data['remaining_box_ids'])
                else:
                    record.setdefault('error', 'NO_RESULT_FILE')
                records.append(record)
                status = {'planned_seeds': seeds, 'completed_runs': len(records), 'results': records,
                          'geometric_matrix_passed': len(records) == len(seeds) and all(r['success'] for r in records),
                          'legacy_timing_and_bitwise_equivalence_claimed': False,
                          'full_mesh_validation_claimed': False,
                          'collision_scope': 'f044_gpu_spheres_with_declared_exemptions'}
                (args.output_dir/'matrix-status.json').write_text(json.dumps(status, indent=2)+'\n')
                print(json.dumps({k:v for k,v in record.items() if k != 'command'}, ensure_ascii=False), flush=True)
            finally:
                fcntl.flock(lock, fcntl.LOCK_UN)
            time.sleep(.05)  # Allow an already-waiting peer to acquire the shared GPU between cases.
    assert inventory(args.runtime_root) == before, 'runtime changed during seed matrix'
    status['runtime_unchanged'] = True
    (args.output_dir/'matrix-status.json').write_text(json.dumps(status, indent=2)+'\n')
    raise SystemExit(0 if status['geometric_matrix_passed'] else 1)


if __name__ == '__main__':
    main()
