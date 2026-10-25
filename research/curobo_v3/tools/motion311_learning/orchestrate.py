#!/usr/bin/env python3
"""Sequential, resumable four-route experiment orchestration, using the validated GPU driver.

The user approved the four-route execution budget. This orchestrator does not authorize test or publications:
all frozen evaluations use dev. Each child has its own source/config/output identity; no silent overwrite.
"""
import argparse
import hashlib
import json
import os
import signal
import shutil
from pathlib import Path
import subprocess
import time

from motion311_learning.driver import run_driver, StopRequest, PYTHON
from motion311_learning.freeze_data import freeze
from motion311_learning.dataset import load_episodes


def sha(path):
    with Path(path).open('rb') as handle:
        return hashlib.file_digest(handle, 'sha256').hexdigest()


def atomic(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=1, allow_nan=False)+'\n')
    tmp.replace(path)


def source_identity(runtime):
    return {str(p.relative_to(runtime)): sha(p) for p in sorted((runtime / 'motion311_learning').glob('*.py'))}


def cpu_module(runtime, module, arguments, log):
    command = [PYTHON, '-u', '-P', '-c',
               f"import sys,runpy;sys.path.insert(0,{str(runtime)!r});runpy.run_module({module!r},run_name='__main__')", *map(str, arguments)]
    env = dict(os.environ, CUDA_VISIBLE_DEVICES='', OPENBLAS_NUM_THREADS='4', OMP_NUM_THREADS='4')
    with log.open('a') as handle:
        subprocess.run(command, stdout=handle, stderr=subprocess.STDOUT, env=env, check=True, timeout=7200)


def atomic_directory(target, build):
    """Publish only fully built directories. Preserve interrupted scratch directories for audit."""
    if (target / 'READY.json').exists():
        return
    if target.exists():
        raise RuntimeError(f'unmarked final directory {target}; inspect rather than overwrite')
    temporary = target.with_name(target.name + '.building')
    if temporary.exists():
        temporary.rename(target.with_name(target.name + '.interrupted-' + str(time.time_ns())))
    build(temporary)
    atomic(temporary / 'READY.json', dict(complete=True))
    temporary.rename(target)


def budgeted_worker(module, log, hours, runtime, arguments, stop):
    """Reserve active allowance durably before launch. Interrupted launch retains its conservative charge."""
    log.mkdir(parents=True, exist_ok=True)
    journal = log / 'budget.json'
    budget = json.loads(journal.read_text()) if journal.exists() else dict(total_hours=hours, charged_s=0., attempts=[])
    if budget['total_hours'] != hours:
        raise RuntimeError('budget configuration changed')
    if budget['attempts'] and budget['attempts'][-1].get('complete'):
        return 0, budget['attempts'][-1]['summary']
    remaining = hours * 3600 - budget['charged_s']
    if remaining <= 0:
        return 3, dict(complete=False, exit=3, active_s=budget['charged_s'], budget_exhausted=True)
    attempt = log / f"attempt-{len(budget['attempts']):03d}"
    entry = dict(directory=str(attempt), reserved_s=remaining, complete=False)
    budget['charged_s'] += remaining
    budget['attempts'].append(entry)
    atomic(journal, budget)
    code, summary = run_driver(module, attempt, remaining / 3600, runtime, arguments, stop=stop)
    budget['charged_s'] -= max(0., remaining - summary['active_s'])
    entry.update(summary=summary, complete=code == 0, charged_s=summary['active_s'])
    atomic(journal, budget)
    return code, summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--runtime', type=Path, required=True)
    parser.add_argument('--teacher', type=Path, required=True)
    parser.add_argument('--teacher-summary', type=Path, required=True)
    parser.add_argument('--wait-hours', type=float, default=6.)
    parser.add_argument('--registry', type=Path, required=True)
    args = parser.parse_args()
    root = args.root / 'formal-v3'
    stop = StopRequest()
    signal.signal(signal.SIGTERM, stop.request)
    signal.signal(signal.SIGINT, stop.request)
    root.mkdir(parents=True, exist_ok=True)
    identity = dict(source=source_identity(args.runtime), registry_sha256=sha(args.registry),
                    teacher=str(args.teacher.resolve()), schedule=dict(seeds=[0, 1, 2], episodes=3000, r3_hours=4., r4_hours=4.5,
                                                                      evaluation_full_seed=101, evaluation_noise_seeds=[102, 103],
                                                                      evaluation_noise_per_mode=4, evaluation_total_hours=4., group='dev'))
    config_path = root / 'identity.json'
    if config_path.exists() and json.loads(config_path.read_text()) != identity:
        raise RuntimeError('formal source/registry/schedule changed; new output required')
    atomic(config_path, identity)
    began = time.monotonic()
    # Only the external teacher driver writes this terminal marker. Its failure is not success.
    while not args.teacher_summary.exists():
        if stop.event.is_set():
            raise InterruptedError('stop requested before teacher freeze')
        if time.monotonic() - began > args.wait_hours * 3600:
            raise TimeoutError('teacher terminal summary not produced')
        time.sleep(30.)
    teacher = json.loads(args.teacher_summary.read_text())
    if teacher['exit'] not in (0, 3):
        raise RuntimeError('teacher failed; refusing to freeze a failed experiment')
    snapshot = root / 'data-final'
    atomic_directory(snapshot, lambda temporary: freeze(args.teacher, temporary, max_scenes=100000))
    episodes = list(load_episodes(snapshot / 'episodes'))
    if len({json.dumps(r['task'], sort_keys=True) for r in episodes}) < 12:
        raise RuntimeError('too few distinct teacher tasks for four-route comparison')
    atomic(root / 'data-counts.json', dict(episodes=len(episodes), tasks=len({json.dumps(r['task'], sort_keys=True) for r in episodes}),
                                          stages={str(s):sum(t['stage']==s for r in episodes for t in r['transitions']) for s in range(3)},
                                          successful_leaves=sum(r['transitions'][-1]['info'].get('success',False) for r in episodes)))
    print(json.dumps(dict(event='data_frozen', episodes=len(episodes))), flush=True)
    del episodes
    for route in ('r1', 'r2'):
        model = root / route
        extra = ['--physical'] if route == 'r2' else []
        def build_model(temporary):
            cpu_module(args.runtime, 'motion311_learning.supervised',
                       ['--data', snapshot, '--output', temporary, '--seed', 311, '--epochs', 250, '--threads', 4, *extra], root / (route + '.log'))
            from motion311_learning.supervised import SupervisedPolicy
            SupervisedPolicy.load(temporary / 'checkpoint.pt')
        atomic_directory(model, build_model)
        print(json.dumps(dict(event='supervised_trained', route=route)), flush=True)
    outcomes = []
    for seed in (0, 1, 2):
        for route, hours in (('r3', 4.), ('r4', 4.5)):
            if source_identity(args.runtime) != identity['source']:
                raise RuntimeError('runtime modified during formal experiment')
            output = root / f'{route}-seed{seed}'
            logs = output / 'driver'
            code, summary = budgeted_worker('motion311_learning.train', logs, hours, args.runtime,
                ['--registry', str(args.registry), '--output', str(output), '--route', route, '--warm-data', str(snapshot),
                 '--curriculum-dir', str(snapshot), '--episodes', '3000', '--seed', str(seed), '--updates', '4'], stop)
            if code not in (0, 3):
                raise RuntimeError(f'{route} seed{seed} failed with {code}')
            if not (output / 'agent.pt').exists():
                raise RuntimeError(f'{route} seed{seed} has no checkpoint')
            outcomes.append(dict(route=route, seed=seed, driver=summary, checkpoint_sha256=sha(output / 'agent.pt')))
            atomic(root / 'training-outcomes.json', outcomes)
            print(json.dumps(dict(event='online_trained', route=route, seed=seed, complete=summary['complete'], active_s=summary['active_s'])), flush=True)
    # Shared controls and supervised ensembles are evaluated once, not three times per RL training seed.
    # The noise subset is fixed before any formal result; separate registry/report prevents accidental pooling.
    registry = json.loads(args.registry.read_text())
    dev = [item for item in registry['instances'] if item['group'] == item['split'] == 'dev']
    groups = {}
    for item in dev:
        key = ('dual' if item['kind'] == 'dual' else 'single-' + next(iter(item['task']))) + ('-top' if item['top_grasp'] else '-side')
        groups.setdefault(key, []).append(item)
    noise_items = [item for _, items in sorted(groups.items()) for item in items[:4]]
    noise_registry = root / 'noise-dev24.json'
    atomic(noise_registry, dict(purpose='predefined planning-seed sensitivity; dev only', instances=noise_items))
    jobs = []
    for scope, source_registry, planning_seeds in (('full', args.registry, (101,)), ('noise', noise_registry, (102, 103))):
        for planner_seed in planning_seeds:
            for route in ('heuristic', 'uniform', 'r1', 'r2'):
                jobs.append((scope, source_registry, planner_seed, route, None))
            for seed in (0, 1, 2):
                for route in ('r3', 'r4'):
                    jobs.append((scope, source_registry, planner_seed, route, seed))
    evaluation_outcomes = []
    total_cap_s = 4 * 3600.
    ledger = root / 'evaluation-budget.json'
    spent = json.loads(ledger.read_text()).get('charged_s', 0.) if ledger.exists() else 0.
    for scope, source_registry, planner_seed, route, seed in jobs:
        if stop.event.is_set():
            raise InterruptedError('formal experiment stop requested')
        if source_identity(args.runtime) != identity['source']:
            raise RuntimeError('runtime changed before frozen evaluation')
        bucket = 'shared' if seed is None else f'training-seed{seed}'
        evaluation = root / f'eval-{scope}-{bucket}'
        checkpoint = root / route / 'checkpoint.pt' if route in ('r1', 'r2') else root / f'{route}-seed{seed}' / 'agent.pt'
        arguments = ['--registry', str(source_registry), '--output', str(evaluation), '--route', route, '--group', 'dev', '--seed', str(planner_seed)]
        if route not in ('heuristic', 'uniform'):
            arguments += ['--models', str(checkpoint)]
        log = evaluation / f'driver-{route}-{planner_seed}'
        saved = log / 'budget.json'
        if saved.exists():
            budget = json.loads(saved.read_text())
            if budget['attempts'] and budget['attempts'][-1].get('complete'):
                continue
            raise RuntimeError('partial evaluation exists; explicit budget reconciliation required')
        allowance = min(1800., total_cap_s - spent)
        if allowance <= 0:
            atomic(root / 'EVALUATION_BUDGET_EXHAUSTED.json', dict(complete=False, charged_s=spent, next_job=[scope, planner_seed, route, seed]))
            return 3
        # Conservative reservation survives an orchestrator crash; never grants extra time on resume.
        atomic(ledger, dict(charged_s=spent + allowance, active_limit_s=total_cap_s))
        code, summary = budgeted_worker('motion311_learning.evaluate', log, allowance / 3600., args.runtime, arguments, stop)
        spent += summary['active_s']
        atomic(ledger, dict(charged_s=spent, active_limit_s=total_cap_s))
        evaluation_outcomes.append(dict(scope=scope, route=route, training_seed=seed, planner_seed=planner_seed, driver=summary))
        atomic(root / 'evaluation-outcomes.json', evaluation_outcomes)
        if code != 0:
            raise RuntimeError(f'evaluation {route} seed{seed}/{planner_seed} incomplete: {code}')
        print(json.dumps(dict(event='evaluated', scope=scope, route=route, training_seed=seed, planner_seed=planner_seed)), flush=True)
    atomic(root / 'COMPLETE.json', dict(complete=True, created_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                                      scope='four-route frozen dev comparison, no heldout acceptance or publication'))
    print(json.dumps(dict(event='formal_complete', output=str(root))), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
