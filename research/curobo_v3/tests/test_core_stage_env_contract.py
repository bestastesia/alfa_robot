from pathlib import Path
import ast
import sys
from types import SimpleNamespace
import unittest

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from motion311_learning.contracts import StepResult, reward

SOURCE = Path(__file__).resolve().parents[1] / 'tools/motion311_learning/stage_env.py'


def step_method():
    tree = ast.parse(SOURCE.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'StageEnv')
    step = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'step')
    class Failure(Exception):
        pass
    namespace = dict(np=np, torch=torch, time=SimpleNamespace(perf_counter=lambda: 0.), StepResult=StepResult,
                     reward=reward, EpisodeFailure=Failure)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[step], type_ignores=[])), str(SOURCE), 'exec'), namespace)
    return namespace['step']


class TimingTransitionTest(unittest.TestCase):
    def test_timing_failure_at_prefix_never_returns_next_observation(self):
        from unittest.mock import patch
        class Recorder:
            def __init__(self, *args):pass
            def __enter__(self):return self
            def __exit__(self, *args):return False
        import v3_search_recorder
        for stage in (0, 1):
            calls = []
            def failed_timing(start):
                raise RuntimeError('extract timed samples failed the mentor collision gate')
            env = SimpleNamespace(done=False, stage=stage, cost_s=0., seed=0, generation_ms=0., feature_ms=0.,
                    continuous=False, observation=SimpleNamespace(candidates=np.zeros((1, 15)), actions=np.zeros((1, 3))),
                    _check_pose=lambda row: None, report={'frames': [[0.] * 15], 'success': False}, timed=True,
                    _contact=lambda row: (row, 1.), _carry=lambda row: (row, 1.), _place=lambda row: (row, 1.),
                    _validate_timed_stage=failed_timing, observe=lambda: calls.append('future'))
            with patch.object(v3_search_recorder, 'SearchRecorder', Recorder):
                result = step_method()(env, 0)
            self.assertTrue(result.terminated)
            self.assertFalse(result.truncated)
            self.assertIsNone(result.observation)
            self.assertEqual(calls, [])
            self.assertFalse(env.report['success'])
            self.assertEqual(result.info['reason'], 'timing_validation')


if __name__ == '__main__':
    unittest.main()
