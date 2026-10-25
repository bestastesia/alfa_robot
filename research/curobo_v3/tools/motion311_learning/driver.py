#!/usr/bin/env python3
"""Chunk launcher: shared lock waits are not GPU-active experiment time.

Workers inherit the locked descriptor and run in a dedicated process group. The
driver can terminate the whole group, and its unexpected exit does not unlock a
still-running worker. Exit 0 means completed, 3 means more work/budget exhausted,
and 75 means queue acquisition timed out (not a training failure).
"""
from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time


PYTHON = '/home/astesia/Sevenova/curobo_v2/.venv/bin/python'
RUNTIME = '/home/astesia/Sevenova/motion311-fourroutes/runtime/research/curobo_v3/tools'
LOCK = '/tmp/sevenova-curobo-gpu.lock'


class StopRequest:
    def __init__(self):
        self.event = threading.Event()
        self.signum = None

    def request(self, signum=signal.SIGTERM, frame=None):
        self.signum = signum
        self.event.set()


def _append_event(directory, event):
    event = {**event, 'time': time.strftime('%Y-%m-%dT%H:%M:%S%z')}
    with (directory / 'chunks.jsonl').open('a') as handle:
        handle.write(json.dumps(event, allow_nan=False) + '\n')
        handle.flush()
        os.fsync(handle.fileno())
    print(json.dumps(event), flush=True)
    return event


def _group_alive(pid):
    try:
        os.killpg(pid, 0)
    except ProcessLookupError:
        return False
    return True


def _group_signal(pid, signum):
    try:
        os.killpg(pid, signum)
    except ProcessLookupError:
        pass


def _drain_group(process, grace_s, poll_s):
    """TERM all children, then KILL any remaining group after a bounded grace."""
    _group_signal(process.pid, signal.SIGTERM)
    deadline = time.monotonic() + grace_s
    while time.monotonic() < deadline:
        process.poll()  # reap the session leader when it exits
        if not _group_alive(process.pid):
            return False
        time.sleep(min(poll_s, max(0., deadline - time.monotonic())))
    killed = _group_alive(process.pid)
    if killed:
        _group_signal(process.pid, signal.SIGKILL)
    process.wait()
    return killed


def _acquire_lock(descriptor, stop, timeout_s, attempts, poll_s):
    began = time.monotonic()
    for attempt in range(1, attempts + 1):
        deadline = time.monotonic() + timeout_s
        while True:
            if stop.event.is_set():
                return False, time.monotonic() - began, attempt, 'signal'
            try:
                fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
                return True, time.monotonic() - began, attempt, None
            except BlockingIOError:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                stop.event.wait(min(poll_s, remaining))
    return False, time.monotonic() - began, attempts, 'queue_timeout'


def run_driver(module, log_dir, hours, runtime=RUNTIME, arguments=(), *, python=PYTHON,
               lock_path=LOCK, wait_timeout_s=1800., wait_attempts=3,
               chunk_seconds=24 * 60., grace_s=120., poll_s=.1, stop=None):
    """Run bounded worker chunks; keyword timing/path overrides are for CPU tests."""
    if not module.startswith('motion311_learning.'):
        raise ValueError('only learning worker modules supported')
    if (not math.isfinite(hours) or hours <= 0 or not math.isfinite(wait_timeout_s) or wait_timeout_s <= 0
            or not isinstance(wait_attempts, int) or not 1 <= wait_attempts <= 3
            or not 0 < chunk_seconds <= 24 * 60 or not 0 <= grace_s <= 120 or poll_s <= 0):
        raise ValueError('invalid active/queue/chunk/grace budget')
    stop = StopRequest() if stop is None else stop
    log_dir, runtime = Path(log_dir), Path(runtime).resolve()
    log_dir.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, OPENBLAS_NUM_THREADS='4', OMP_NUM_THREADS='4',
               PYTORCH_CUDA_ALLOC_CONF='expandable_segments:True')
    # Runtime path is inserted explicitly, not inferred from driver cwd.
    script = f"import sys,runpy;sys.path.insert(0,{str(runtime)!r});runpy.run_module({module!r},run_name='__main__')"
    forwarded = list(arguments)
    if forwarded[:1] == ['--']:
        forwarded = forwarded[1:]
    budget = hours * 3600.
    active_total, wait_total = 0., 0.
    wall_start = time.monotonic()
    events, chunk, final_code, complete, exhausted = [], 0, 3, False, False
    while True:
        if stop.event.is_set():
            final_code = 128 + int(stop.signum or signal.SIGTERM)
            break
        remaining = budget - active_total
        if remaining <= 0:
            exhausted = True
            final_code = 3
            break
        descriptor = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o666)
        process = None
        try:
            acquired, waited, attempts, reason = _acquire_lock(descriptor, stop, wait_timeout_s,
                                                               wait_attempts, poll_s)
            wait_total += waited
            if not acquired:
                final_code = 75 if reason == 'queue_timeout' else 128 + int(stop.signum or signal.SIGTERM)
                events.append(_append_event(log_dir, dict(chunk=chunk, status=reason, wait_s=waited, active_s=0.,
                              queue_attempts=attempts, exit=None, driver_exit=final_code, complete=False,
                              budget_exhausted=False, active_total_s=active_total)))
                break
            if stop.event.is_set():
                final_code = 128 + int(stop.signum or signal.SIGTERM)
                break
            allowance = min(chunk_seconds, remaining)
            command = [str(python), '-u', '-P', '-c', script, *forwarded,
                       '--max-minutes', str(allowance / 60.)]
            active_start = time.monotonic()
            timed_out, interrupted, killed, orphaned = False, False, False, False
            try:
                with (log_dir / 'run.log').open('a') as handle:
                    process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, env=env,
                                               start_new_session=True, pass_fds=(descriptor,))
                    deadline = active_start + allowance
                    while process.poll() is None:
                        if stop.event.is_set():
                            interrupted = True
                            killed = _drain_group(process, grace_s, poll_s)
                            break
                        if time.monotonic() >= deadline:
                            timed_out = True
                            killed = _drain_group(process, grace_s, poll_s)
                            break
                        stop.event.wait(min(poll_s, max(0., deadline - time.monotonic())))
                    code = process.wait()
                    # A worker must not leave detached children in its session.
                    if not killed and _group_alive(process.pid):
                        orphaned = True
                        killed = _drain_group(process, grace_s, poll_s) or killed
            except OSError as error:
                active = time.monotonic() - active_start
                active_total += active
                final_code = 127
                events.append(_append_event(log_dir, dict(chunk=chunk, status='launch_error', wait_s=waited,
                              active_s=active, queue_attempts=attempts, exit=None, driver_exit=127,
                              error=str(error), complete=False, budget_exhausted=active_total >= budget,
                              active_total_s=active_total)))
                break
            active = time.monotonic() - active_start
            active_total += active
            interrupted = interrupted or stop.event.is_set()
            complete = code == 0 and not killed and not orphaned
            exhausted = not complete and active_total >= budget
            if interrupted:
                final_code = 128 + int(stop.signum or signal.SIGTERM)
                status = 'interrupted'
            elif exhausted and timed_out:
                # The final active-budget slice can end before a worker has
                # installed its handlers. Preserve its raw exit below, but the
                # experiment outcome is incomplete budget, not training failure.
                final_code, status = 3, 'budget_exhausted'
            elif killed or orphaned:
                final_code, status = 124, 'worker_timeout' if timed_out else 'orphaned_children'
            elif timed_out and code not in (0, 3):
                final_code, status = 124, 'worker_timeout'
            elif code not in (0, 3):
                final_code = code if code > 0 else 128 - code
                status = 'worker_failure'
            elif complete:
                final_code, status = 0, 'complete'
            else:
                final_code, status = 3, 'budget_exhausted' if exhausted else 'more'
            events.append(_append_event(log_dir, dict(chunk=chunk, status=status, wait_s=waited, active_s=active,
                          queue_attempts=attempts, exit=code, driver_exit=final_code, complete=complete,
                          budget_exhausted=exhausted, deadline_reached=timed_out, killed=killed,
                          active_total_s=active_total, chunk_allowance_s=allowance, grace_s=grace_s)))
            chunk += 1
            if complete or exhausted or interrupted or final_code != 3:
                break
        finally:
            # Do not LOCK_UN here: the inherited fd shares the open-file
            # description. Closing only our copy preserves worker ownership.
            if process is not None and process.poll() is None:
                _drain_group(process, grace_s, poll_s)
            os.close(descriptor)
    summary = dict(finished=complete, complete=complete, exit=final_code, budget_hours=hours,
                   budget_exhausted=exhausted, active_s=active_total, wait_s=wait_total,
                   elapsed_s=time.monotonic() - wall_start, chunks=len(events))
    with (log_dir / 'summary.json').open('w') as handle:
        json.dump(summary, handle, indent=2)
        handle.write('\n')
    print(json.dumps(summary), flush=True)
    return final_code, summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--module', required=True)
    parser.add_argument('--log-dir', type=Path, required=True)
    parser.add_argument('--hours', type=float, required=True)
    parser.add_argument('--runtime', type=Path, default=Path(RUNTIME))
    parser.add_argument('arguments', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    stop = StopRequest()
    previous = {kind: signal.getsignal(kind) for kind in (signal.SIGTERM, signal.SIGINT)}
    for kind in previous:
        signal.signal(kind, stop.request)
    try:
        code, _ = run_driver(args.module, args.log_dir, args.hours, args.runtime, args.arguments, stop=stop)
    finally:
        for kind, handler in previous.items():
            signal.signal(kind, handler)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
