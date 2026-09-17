"""Regression: choose feasible, measured candidates, never closest or failed-fast."""
import importlib.util
from pathlib import Path

script = Path(__file__).resolve().parents[4] / 'tools/benchmark_v3_transfer_midpoints.py'
spec = importlib.util.spec_from_file_location('benchmark', script)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
records = [
    dict(candidate='near', mode='use', success=True, total_ms=100,
         retreat_to_midpoint_tcp_distance_m=.1),
    dict(candidate='far', mode='use', success=True, total_ms=20,
         retreat_to_midpoint_tcp_distance_m=.5),
    dict(candidate='failed_fast', mode='use', success=False, total_ms=1),
    dict(candidate='far', mode='build', success=True, total_ms=500),
]
ranking = module.summarize(records)
assert [r['candidate'] for r in ranking] == ['far', 'near', 'failed_fast']
assert ranking[0]['median_online_ms'] == 20
assert ranking[-1]['median_online_ms'] is None
assert ranking[0]['runs'] == 1
records.append(dict(candidate='far', mode='use', success=False, total_ms=1))
assert module.summarize(records)[0]['candidate'] == 'near'
print('PASS feasibility before latency; distance is not the objective')
