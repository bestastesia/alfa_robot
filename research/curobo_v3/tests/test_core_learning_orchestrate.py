from pathlib import Path
import json
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from motion311_learning.orchestrate import atomic_directory, budgeted_worker
from motion311_learning.driver import StopRequest


class OrchestrationTest(unittest.TestCase):
    def test_interrupted_directory_never_looks_complete(self):
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder) / 'data'
            def interrupted(path):
                path.mkdir(); (path / 'partial').write_text('partial'); raise RuntimeError('interrupted')
            with self.assertRaises(RuntimeError):atomic_directory(target, interrupted)
            self.assertFalse(target.exists())
            def complete(path):
                path.mkdir(); (path / 'data').write_text('complete')
            atomic_directory(target, complete)
            self.assertTrue((target / 'READY.json').exists())
            self.assertEqual(len(list(Path(folder).glob('data.interrupted-*'))), 1)

    def test_budget_reservation_survives_crash(self):
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder)
            with patch('motion311_learning.orchestrate.run_driver', side_effect=RuntimeError('crash')):
                with self.assertRaises(RuntimeError):budgeted_worker('motion311_learning.train', log, 1., log, [], StopRequest())
            with patch('motion311_learning.orchestrate.run_driver') as worker:
                code, summary = budgeted_worker('motion311_learning.train', log, 1., log, [], StopRequest())
                worker.assert_not_called()
            self.assertEqual(code, 3)
            self.assertFalse(summary['complete'])

    def test_completed_worker_is_not_repeated_and_stop_forwarded(self):
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder); stop = StopRequest()
            with patch('motion311_learning.orchestrate.run_driver', return_value=(0, dict(active_s=10., complete=True, exit=0))) as worker:
                code, summary = budgeted_worker('motion311_learning.train', log, 1., log, [], stop)
                self.assertIs(worker.call_args.kwargs['stop'], stop)
                budgeted_worker('motion311_learning.train', log, 1., log, [], stop)
                worker.assert_called_once()
            self.assertEqual(json.loads((log/'budget.json').read_text())['charged_s'], 10.)


if __name__ == '__main__':
    unittest.main()
