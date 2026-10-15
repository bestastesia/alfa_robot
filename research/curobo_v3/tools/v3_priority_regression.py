import argparse
import json
from pathlib import Path
import time

import numpy as np

from curobo_core.contracts import PlannerAssets
from v3_mixed_placement_planner import AnalyticContacts
from v3_priority_contact_planner import PriorityContactPlanner
from v3_conveyor_regression import audit


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    assets = PlannerAssets()
    planner = PriorityContactPlanner(assets)
    summaries = []
    for boxes in ({'left': 22, 'right': 17}, {'left': 18, 'right': 16},
                  {'left': 9, 'right': 1}, {'left': 4, 'right': 2}):
        planner.configure_task(boxes)
        actual = planner.build_contacts(boxes).js_solution.position.cpu().numpy().reshape(-1, 15)
        priority = dict(planner.priority_selection_report)
        baseline = AnalyticContacts.build_contacts(planner, boxes).js_solution.position.cpu().numpy().reshape(-1, 15)
        assert actual.shape == baseline.shape == (64, 15)
        assert {tuple(row) for row in np.round(actual, 6)} == {tuple(row) for row in np.round(baseline, 6)}
        planner.analytic_task = None
        started = time.perf_counter()
        result = planner.full_cycle(boxes, lambda text: print(text, flush=True))
        name = f"L{boxes['left']}_R{boxes['right']}"
        (args.output_dir / (name + '.json')).write_text(json.dumps(result, ensure_ascii=False))
        if not result['success']:
            raise SystemExit(f"Unexpected failure: {name}: {result.get('blocked_details')}")
        summary = {'task': boxes, 'priority': priority, 'same_64_as_exhaustive': True,
                   'wall_ms': (time.perf_counter() - started) * 1000, 'audit': audit(result, assets)}
        summaries.append(summary)
        print(json.dumps(summary, ensure_ascii=False), flush=True)
    (args.output_dir / 'summary.json').write_text(json.dumps(summaries, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
