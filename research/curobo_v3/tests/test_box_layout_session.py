"""CPU-only checks for bounded gpuq ownership, snapshot IPC and model invalidation."""
from collections import OrderedDict
from dataclasses import replace
import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
import v3_box_layout as layout
from curobo_core.layout_contract import base_front_x
from curobo_core.box_pick import BoxPick, layout_model_key


class BoxLayoutSessionTest(unittest.TestCase):
    def args(self, root):
        return NS(runs_root=root, estimate='6m', session_idle_seconds=30., session_seconds=90.,
                  session_max_requests=3, gpu_session=root)

    def document(self):
        return dict(frame_id='world', boxes=[dict(size='40x40x30', side=s, center_m=[base_front_x()+.68, sign*.45, 1.02])
            for s, sign in [('left', 1), ('right', -1)]])

    def test_model_key_ignores_xyz_but_not_dimensions_or_suction(self):
        picks = layout.parse_layout(self.document()); key = layout_model_key(picks)
        changed = {s: replace(p, center_m=(1.2, p.center_m[1], 1.03), carry_updown_m=-.41) for s, p in picks.items()}
        self.assertEqual(layout_model_key(changed), key)
        for variant in (replace(picks['left'], size='46.5x25.5x58.2'), replace(picks['left'], center_m=(1.18, .45, .491))):
            self.assertNotEqual(layout_model_key(dict(picks, left=variant)), key)
        tall = dict(picks, left=replace(picks['left'], size='46.5x25.5x58.2'))
        self.assertNotEqual(layout_model_key(tall), layout_model_key(dict(tall, left=replace(tall['left'], swap_length_height=True))))

    def test_two_requests_share_one_queue_entry_and_frozen_files(self):
        class Process:
            pid = 123456789
            returncode = None
            def poll(self):return self.returncode
            def wait(self, timeout):self.returncode = 0; return 0
        with tempfile.TemporaryDirectory() as tmp, patch('subprocess.Popen', side_effect=[Process(), Process()]) as popen, \
                patch('os.killpg') as kill, patch.object(layout, 'session_source_stamp', return_value='frozen'):
            root = Path(tmp); worker = layout.LayoutGpuWorker(self.args(root))
            runs = [root/f'layout_{i}' for i in range(3)]
            for run in runs:
                run.mkdir(); layout.atomic_json(run/'input.json', self.document())
            self.assertFalse(worker.submit(runs[0], 16))
            command = popen.call_args.args[0]
            self.assertEqual(command[:2], ['gpuq', '--name'])
            self.assertEqual(command[3:6], ['--estimate', '450s', '--'])
            self.assertIn('--gpu-session', command)
            layout.atomic_json(worker.directory/'status.json', dict(state='idle', requests=1,
                deadline_unix=time.time()+90, idle_deadline_unix=time.time()+30, source_stamp='frozen'))
            layout.atomic_json(runs[1]/'input.json', dict(self.document(), request_id='second'))
            self.assertTrue(worker.submit(runs[1], 16))
            # Final result may precede the idle status by one tick; busy time
            # must not count towards an expired idle deadline.
            layout.atomic_json(worker.directory/'status.json', dict(state='busy', requests=1,
                deadline_unix=time.time()+90, idle_deadline_unix=time.time()-30, source_stamp='frozen'))
            self.assertTrue(worker.submit(runs[2], 16))
            self.assertEqual(popen.call_count, 1)
            requests = [json.loads(p.read_text()) for p in worker.directory.glob('request_*.json')]
            self.assertEqual({r['output'] for r in requests}, {str(r/'plan') for r in runs})
            self.assertNotIn('request_id', json.loads((runs[0]/'input.json').read_text()))
            self.assertTrue((runs[1]/'worker.log').is_symlink())
            worker.stop()
            run4 = root/'layout_3'; run4.mkdir(); layout.atomic_json(run4/'input.json', self.document())
            self.assertFalse(worker.submit(run4, 16), 'never enqueue on a worker already signalled to stop')
            self.assertEqual(popen.call_count, 2)
            worker.close(); self.assertEqual(kill.call_count, 2)

    def test_solver_pool_is_bounded_and_new_xyz_does_not_return_old_result(self):
        instances = []
        class Planner:
            def __init__(self):instances.append(self)
            def plan_layout(self, picks, output, max_contacts, progress, adapt_carry, adapt_grasp):
                return dict(success=False, failure='synthetic gate rejection', requested_x=picks['left'].center_m[0])
        with tempfile.TemporaryDirectory() as tmp, patch.dict(sys.modules, {'v3_box_layout_planner': NS(BoxLayoutPlanner=Planner)}):
            root = Path(tmp); pool = OrderedDict()
            documents = [self.document() for _ in range(5)]

            for row in documents[1]['boxes']:row['center_m'][0] = base_front_x()+.70
            documents[2]['boxes'][0]['size'] = '46.5x25.5x58.2'
            documents[3]['boxes'][0].update(size='46.5x25.5x58.2', swap_length_height=True)
            reused = []
            for i, document in enumerate(documents):
                source = root/f'{i}.json'; layout.atomic_json(source, document)
                args = NS(output=root/f'plan_{i}', solve_file=source, max_contacts=16)
                self.assertEqual(layout.solve_file(args, pool), 1)
                result = json.loads((args.output/'result.json').read_text())
                reused.append(result['planner_reused'])
                self.assertEqual(result['requested_x'], document['boxes'][0]['center_m'][0])
                self.assertLessEqual(len(pool), 2)
            self.assertEqual(reused, [False, True, False, False, False])
            self.assertEqual(len(instances), 4)

    def test_worker_limits_and_source_change_stop_without_solving(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); args = self.args(root)
            layout.atomic_json(root/'request_1.json', dict(input=str(root/'input.json'), output=str(root/'plan'), max_contacts=16))
            with patch.object(layout, 'session_source_stamp', side_effect=['old', 'new']), patch.object(layout, 'solve_file') as solve:
                self.assertEqual(layout.serve_session(args), 0)
                solve.assert_not_called()
                self.assertEqual(json.loads((root/'status.json').read_text())['reason'], 'source_changed')
            def solve(job, planners, session):
                job.output.mkdir()
                layout.atomic_json(job.output/'result.json', dict(success=False))
                return 1
            (root/'status.json').unlink()
            args.session_max_requests = 1
            with patch.object(layout, 'session_source_stamp', return_value='same'), patch.object(layout, 'solve_file', side_effect=solve) as solve:
                self.assertEqual(layout.serve_session(args), 0)
                self.assertEqual(solve.call_count, 1)
                state = json.loads((root/'status.json').read_text())
                self.assertEqual((state['state'], state['requests']), ('stopped', 1))
            (root/'request_1.json').unlink()
            (root/'status.json').unlink()
            args.session_idle_seconds = .001
            with patch.object(layout, 'session_source_stamp', return_value='same'):
                self.assertEqual(layout.serve_session(args), 0)
                self.assertEqual(json.loads((root/'status.json').read_text())['reason'], 'idle_timeout')


if __name__ == '__main__':unittest.main()
