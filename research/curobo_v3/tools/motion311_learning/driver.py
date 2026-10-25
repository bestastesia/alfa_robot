#!/usr/bin/env python3
"""Chunk launcher with the shared GPU lock and total experiment budget.

Launch this module from the source checkout; it launches worker modules from the prepared runtime explicitly,
so cwd cannot accidentally import unprepared URDF paths. No background shell '&'; the host tracks this driver.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

PYTHON = '/home/astesia/Sevenova/curobo_v2/.venv/bin/python'
RUNTIME = '/home/astesia/Sevenova/motion311-fourroutes/runtime/research/curobo_v3/tools'
LOCK = '/tmp/sevenova-curobo-gpu.lock'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--module', required=True)
    parser.add_argument('--log-dir', type=Path, required=True)
    parser.add_argument('--hours', type=float, required=True)
    parser.add_argument('--runtime', type=Path, default=Path(RUNTIME))
    parser.add_argument('arguments', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if not args.module.startswith('motion311_learning.'):
        raise ValueError('only learning worker modules supported')
    args.log_dir.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, OPENBLAS_NUM_THREADS='4', OMP_NUM_THREADS='4', PYTORCH_CUDA_ALLOC_CONF='expandable_segments:True')
    script = f"import sys,runpy;sys.path.insert(0,{str(args.runtime)!r});runpy.run_module({args.module!r},run_name='__main__')"
    forwarded = args.arguments[1:] if args.arguments[:1] == ['--'] else args.arguments
    start = time.monotonic()
    code = 3
    chunk = 0
    while code == 3 and time.monotonic() - start < args.hours * 3600:
        remaining = args.hours * 3600 - (time.monotonic() - start)
        minutes = min(24., max(.01, remaining / 60. - .5))
        command = ['flock', '-w', '1200', LOCK, PYTHON, '-u', '-P', '-c', script, *forwarded, '--max-minutes', str(minutes)]
        began = time.time()
        with (args.log_dir / 'run.log').open('a') as handle:
            process = subprocess.run(command, stdout=handle, stderr=subprocess.STDOUT, env=env, timeout=minutes * 60 + 1500)
        code = process.returncode
        event = dict(chunk=chunk, exit=code, elapsed_s=time.time() - began, time=time.strftime('%Y-%m-%dT%H:%M:%S%z'))
        with (args.log_dir / 'chunks.jsonl').open('a') as handle:
            handle.write(json.dumps(event)+'\n')
        print(json.dumps(event), flush=True)
        chunk += 1
        if code == 3:
            time.sleep(2.)
    print(json.dumps(dict(finished=code == 0, exit=code, budget_hours=args.hours, elapsed_s=time.monotonic()-start)), flush=True)
    raise SystemExit(0 if code in (0, 3) else code)


if __name__ == '__main__':
    main()
