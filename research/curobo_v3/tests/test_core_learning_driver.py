from pathlib import Path
import fcntl
import json
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from motion311_learning.driver import StopRequest, run_driver


WORKER = '''import argparse, fcntl, os, signal, subprocess, sys, time
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument('--max-minutes',type=float,required=True)
p.add_argument('--kind',required=True)
p.add_argument('--marker',required=True)
p.add_argument('--lock',required=True)
a=p.parse_args()
marker=Path(a.marker)
probe=os.open(a.lock,os.O_RDWR|os.O_CREAT,0o666)
try:
    fcntl.flock(probe,fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError:
    pass
else:
    raise RuntimeError('worker did not inherit held lock')
finally:
    os.close(probe)
marker.write_text(str(os.getpid()))
if a.kind=='complete':
    raise SystemExit(0)
if a.kind=='failure':
    raise SystemExit(7)
if a.kind=='more':
    signal.signal(signal.SIGTERM,lambda *args: sys.exit(3))
    time.sleep(max(.05,a.max_minutes*60-.02))
    raise SystemExit(3)
if a.kind=='graceful':
    def stop(*args):
        marker.with_suffix('.drained').write_text('atomic data flushed')
        raise SystemExit(3)
    signal.signal(signal.SIGTERM,stop)
    while True: time.sleep(.02)
if a.kind=='orphan':
    child=subprocess.Popen([sys.executable,'-c', 'import time;time.sleep(60)'])
    marker.with_suffix('.child').write_text(str(child.pid))
    raise SystemExit(0)
if a.kind=='ignore-tree':
    signal.signal(signal.SIGTERM,signal.SIG_IGN)
    child=subprocess.Popen([sys.executable,'-c',
        'import signal,time;signal.signal(signal.SIGTERM,signal.SIG_IGN);time.sleep(60)'])
    marker.with_suffix('.child').write_text(str(child.pid))
    while True: time.sleep(.02)
if a.kind=='parent-kill':
    time.sleep(.8)
    marker.with_suffix('.done').write_text('worker finished')
    raise SystemExit(0)
'''


def alive(pid):
    try:
        value = Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1].split()[0]
        return value != 'Z'
    except FileNotFoundError:
        return False


def wait_for(predicate, timeout=3.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(.01)
    return bool(predicate())


class LearningDriverTest(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.root = Path(self.folder.name)
        self.runtime = self.root / 'runtime'
        package = self.runtime / 'motion311_learning'
        package.mkdir(parents=True)
        (package / '__init__.py').write_text('')
        (package / 'dummy.py').write_text(WORKER)
        self.lock = self.root / 'gpu.lock'
        self.marker = self.root / 'worker.pid'
        self.logs = self.root / 'logs'

    def run_worker(self, kind, budget=.4, **kwargs):
        forwarded = ['--kind', kind, '--marker', str(self.marker), '--lock', str(self.lock)]
        options = dict(python=sys.executable, lock_path=self.lock, wait_timeout_s=.1,
                       wait_attempts=3, chunk_seconds=.25, grace_s=.12, poll_s=.01)
        options.update(kwargs)
        with patch('builtins.print'):
            return run_driver('motion311_learning.dummy', self.logs, budget/3600., self.runtime,
                              forwarded, **options)

    def events(self):
        return [json.loads(line) for line in (self.logs / 'chunks.jsonl').read_text().splitlines()]

    def test_queue_wait_does_not_consume_active_budget(self):
        descriptor = os.open(self.lock, os.O_CREAT|os.O_RDWR, 0o666)
        fcntl.flock(descriptor, fcntl.LOCK_EX|fcntl.LOCK_NB)
        def release():
            fcntl.flock(descriptor, fcntl.LOCK_UN)
            os.close(descriptor)
        timer = threading.Timer(.3, release)
        timer.start()
        try:
            code, summary = self.run_worker('complete', budget=.2, wait_timeout_s=.5)
        finally:
            timer.join()
        self.assertEqual(code, 0)
        self.assertTrue(summary['finished'])
        self.assertGreater(summary['wait_s'], .25)
        self.assertLess(summary['active_s'], .2)
        self.assertFalse(summary['budget_exhausted'])
        self.assertEqual(len(self.events()), 1)

    def test_queue_timeout_is_distinct_from_worker_failure(self):
        descriptor = os.open(self.lock, os.O_CREAT|os.O_RDWR, 0o666)
        fcntl.flock(descriptor, fcntl.LOCK_EX|fcntl.LOCK_NB)
        try:
            code, summary = self.run_worker('complete', wait_timeout_s=.025)
        finally:
            os.close(descriptor)
        self.assertEqual(code, 75)
        self.assertEqual(summary['active_s'], 0)
        self.assertFalse(self.marker.exists())
        event = self.events()[0]
        self.assertEqual(event['queue_attempts'], 3)
        self.assertEqual(event['status'], 'queue_timeout')
        self.assertIsNone(event['exit'])
        self.logs = self.root / 'failure-logs'
        code, summary = self.run_worker('failure')
        self.assertEqual(code, 7)
        self.assertEqual(self.events()[0]['status'], 'worker_failure')
        self.assertEqual(self.events()[0]['exit'], 7)

    def test_more_work_exhausts_only_active_budget_and_returns_three(self):
        code, summary = self.run_worker('more', budget=.36, chunk_seconds=.22)
        self.assertEqual(code, 3)
        self.assertFalse(summary['finished'])
        self.assertTrue(summary['budget_exhausted'])
        events = self.events()
        self.assertGreaterEqual(len(events), 2)
        self.assertEqual(events[0]['status'], 'more')
        self.assertFalse(events[0]['budget_exhausted'])
        self.assertEqual(events[0]['exit'], 3)
        self.assertTrue(events[-1]['budget_exhausted'])
        self.assertFalse(events[-1]['complete'])
        self.assertLessEqual(max(row['chunk_allowance_s'] for row in events), .22)

    def test_timeout_kills_entire_process_group_and_releases_lock(self):
        began = time.monotonic()
        code, summary = self.run_worker('ignore-tree', budget=1., chunk_seconds=.25, grace_s=.08)
        self.assertLess(time.monotonic()-began, 1.0)
        self.assertEqual(code, 124)
        parent = int(self.marker.read_text())
        child = int(self.marker.with_suffix('.child').read_text())
        self.assertTrue(wait_for(lambda: not alive(parent) and not alive(child)))
        self.assertTrue(self.events()[0]['killed'])
        self.assertEqual(self.events()[0]['status'], 'worker_timeout')
        descriptor = os.open(self.lock, os.O_RDWR)
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX|fcntl.LOCK_NB)
        finally:
            os.close(descriptor)

    def test_worker_exit_does_not_leave_descendants_or_false_completion(self):
        code, summary = self.run_worker('orphan', budget=1., chunk_seconds=.5, grace_s=.06)
        self.assertEqual(code, 124)
        self.assertFalse(summary['finished'])
        self.assertEqual(self.events()[0]['status'], 'orphaned_children')
        child = int(self.marker.with_suffix('.child').read_text())
        self.assertTrue(wait_for(lambda: not alive(child)))
        descriptor = os.open(self.lock, os.O_RDWR)
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX|fcntl.LOCK_NB)
        finally:
            os.close(descriptor)

    def test_driver_stop_drains_worker_and_does_not_launch_next_chunk(self):
        stop = StopRequest()
        def interrupt_when_started():
            if wait_for(self.marker.exists):
                stop.request(signal.SIGTERM)
        thread = threading.Thread(target=interrupt_when_started)
        thread.start()
        try:
            code, summary = self.run_worker('graceful', budget=3., chunk_seconds=2., grace_s=.2, stop=stop)
        finally:
            thread.join()
        self.assertEqual(code, 143)
        self.assertFalse(summary['finished'])
        self.assertEqual(len(self.events()), 1)
        self.assertTrue(self.marker.with_suffix('.drained').exists())
        self.assertEqual(self.events()[0]['status'], 'interrupted')
        self.assertEqual(self.events()[0]['exit'], 3)

    def test_killed_driver_leaves_lock_owned_by_live_worker(self):
        tools = str(Path(__file__).resolve().parents[1] / 'tools')
        script = (
            'import sys;sys.path.insert(0,' + repr(tools) + ');'
            'from motion311_learning.driver import run_driver;'
            'run_driver("motion311_learning.dummy",' + repr(str(self.logs)) + ',1/3600,' + repr(str(self.runtime)) + ','
            + repr(['--kind','parent-kill','--marker',str(self.marker),'--lock',str(self.lock)]) + ','
            'python=' + repr(sys.executable) + ',lock_path=' + repr(str(self.lock)) + ',chunk_seconds=2,grace_s=.1,poll_s=.01)'
        )
        driver = subprocess.Popen([sys.executable, '-I', '-B', '-c', script],
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        worker_pid = None
        try:
            self.assertTrue(wait_for(self.marker.exists))
            worker_pid = int(self.marker.read_text())
            driver.kill(); driver.wait(timeout=2.)
            self.assertTrue(alive(worker_pid))
            descriptor = os.open(self.lock, os.O_RDWR)
            try:
                with self.assertRaises(BlockingIOError):
                    fcntl.flock(descriptor, fcntl.LOCK_EX|fcntl.LOCK_NB)
                self.assertTrue(wait_for(self.marker.with_suffix('.done').exists))
                self.assertTrue(wait_for(lambda: not alive(worker_pid)))
                fcntl.flock(descriptor, fcntl.LOCK_EX|fcntl.LOCK_NB)
            finally:
                os.close(descriptor)
        finally:
            if driver.poll() is None:
                driver.kill(); driver.wait(timeout=2.)
            if worker_pid is not None and alive(worker_pid):
                try: os.killpg(worker_pid, signal.SIGKILL)
                except ProcessLookupError: pass


if __name__ == '__main__':
    unittest.main()
