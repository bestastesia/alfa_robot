import argparse
import json
from pathlib import Path

import torch

from v3_current_urdf_forward import CurrentUrdfForwardPlanner, ROOT


STAGES = ('approach', 'contact_approach', 'extract', 'transport', 'conveyor_place', 'return_home')
TASKS = ({'left': 22, 'right': 17}, {'left': 3, 'right': 2}, {'right': 2}, {'left': 22, 'right': 17})


def audit_result(result):
    if not result['success']:
        raise RuntimeError(f"{result.get('blocked_stage')}: {result.get('blocked_details')}")
    if any(result['phases'].count(stage) < 2 for stage in STAGES):
        raise ValueError('六个运动阶段未完整生成')
    if not all(check['passed'] for check in result['seam_checks']) or result['final_home_error'] > 1e-5:
        raise ValueError('阶段连续性或回Home误差验收失败')
    return dict(task=result['task'], success=True, frames=len(result['frames']),
                wall_ms=result['measured_wall_ms'], final_home_error=result['final_home_error'],
                seams_passed=True, payload_path_validated=result['extraction_policy']['payload_path_validated'],
                stages={stage: result['phases'].count(stage) for stage in STAGES})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', type=Path, default=ROOT / 'generated/forward_precontact_current_urdf/verification')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    planner = CurrentUrdfForwardPlanner()
    warmup = planner.warmup_resources()
    records = []
    for index, task in enumerate(TASKS):
        result = planner.full_cycle(task, print)
        (args.output_dir / f'{index:03d}.json').write_text(json.dumps(result, ensure_ascii=False))
        records.append(audit_result(result))
        summary = dict(coordinate_revision=result['coordinate_revision'], tool_offset_m=result['tool_offset_m'],
                       warmup=warmup, records=records, distinct_tasks=len({tuple(sorted(item['task'].items())) for item in records}),
                       peak_gpu_allocated_mib=torch.cuda.max_memory_allocated() / 2**20,
                       validation_scope='representative tasks only, not all 117; no hardware commands')
        (args.output_dir / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
        print(json.dumps(records[-1], ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
