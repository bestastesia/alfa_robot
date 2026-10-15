import argparse
import json
from pathlib import Path

from curobo_core.contracts import PlannerAssets
from v3_conveyor_regression import audit
from v3_lazy_extraction_planner import LazyExtractionPlanner


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    planner = LazyExtractionPlanner(PlannerAssets())
    planner.warmup_resources()
    summaries = []
    for boxes in ({'left': 18, 'right': 22}, {'left': 18, 'right': 16}, {'left': 9, 'right': 1}):
        planner.configure_task(boxes)
        result = planner.full_cycle(boxes, lambda text: print(text, flush=True))
        (args.output_dir / f"L{boxes['left']}_R{boxes['right']}.json").write_text(json.dumps(result, ensure_ascii=False))
        if not result['success']:
            raise SystemExit(f"Unexpected failure: {boxes}: {result.get('blocked_details')}")
        lazy = result['lazy_extraction']
        assert not result['rotation_quality_hard_gate'] and result['box_rotation_cost_enabled']
        assert len(set(lazy['ranking'])) == lazy['candidate_pool'] == 64
        assert lazy['checked_candidates'] <= 64
        assert [entry['candidate_limit'] for entry in lazy['batches']] == [8, 16, 32, 64][:len(lazy['batches'])]
        assert abs(sum(entry['search_ms'] for entry in lazy['batches']) - lazy['total_transport_search_ms']) < .01
        assert len(planner._extraction_cache) == lazy['checked_candidates']
        summaries.append({'task': boxes, 'total_ms': result['total_ms'], 'lazy': lazy,
                          'rotation_metrics': result.get('transport_rotation_metrics'), 'audit': audit(result, PlannerAssets())})
        print(json.dumps({'task': boxes, 'checked': lazy['checked_candidates'], 'total_ms': result['total_ms']}), flush=True)
    (args.output_dir / 'summary.json').write_text(json.dumps(summaries, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
