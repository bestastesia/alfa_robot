import argparse
import gc
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import torch

from v3_new_warp import NewWarpPlanner, TimedNewWarpPlanner, ROOT, POLICY_PATH
from v3_current_urdf_smoke import audit_result


def validate_cycle(result, height):
    report = audit_result(result)
    events = result['guided_search_events']
    if len(events) != 6 or any(event.get('lm_used') or event.get('joint_iterations') != 30 for event in events):
        raise ValueError('新WARP六段解析/30轮合同失败')
    if not np.isclose(result['conveyor_release_adjustment']['box_center_z_m'], height):
        raise ValueError('释放高度不匹配')
    for pose in result['conveyor_box_targets'].values():
        if not np.isclose(pose['position'][2], height, atol=.002):
            raise ValueError('实际放箱目标没有使用配置箱心高度')
    return dict(report, soft_reference_stages=sum(bool(event.get('reference_used_soft_fallback')) for event in events),
                release_box_center_z_m=height, path_prior_lm_used=False, rounds_per_stage=30)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--height', type=float, default=.982)
    parser.add_argument('--all-tasks', action='store_true')
    parser.add_argument('--timed', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    planner = TimedNewWarpPlanner() if args.timed else NewWarpPlanner(release_box_center_z_m=args.height)
    if args.timed and not np.isclose(args.height, planner.release_box_center_z_m):
        raise ValueError('后处理验证仅使用当前权威高度')
    tasks = [dict(task) for label, task in planner.task_options] if args.all_tasks else [dict(left=22, right=17), dict(left=3, right=2), dict(right=2)]
    source_paths = [POLICY_PATH, planner.args.urdf, planner.args.robot_config, planner.args.mobile_robot_config,
                    ROOT / 'tools/v3_polar_refined_cycle.py', ROOT / 'tools/v3_new_warp.py',
                    *sorted((ROOT / 'tools/curobo_core').glob('new_warp*.py'))]
    protocol = dict(height=args.height, timed=args.timed, tasks=tasks,
                    source_sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in source_paths},
                    warmup=planner.warmup_resources())
    (args.output / 'protocol.json').write_text(json.dumps(protocol, indent=2))
    records = []
    for index, task in enumerate(tasks):
        np.random.seed(20261027)
        torch.manual_seed(20261027)
        began = time.perf_counter()
        result = planner.full_cycle(task)
        (args.output / f'{index:03d}.json').write_text(json.dumps(result, ensure_ascii=False))
        record = validate_cycle(result, args.height)
        record['inclusive_wall_ms'] = (time.perf_counter() - began) * 1000
        if args.timed:
            if not result['timed_validation']['success']:
                raise ValueError('后处理时间化验收失败')
            record.update(timed_validation=result['timed_validation'], motion_duration_s=result['motion_duration_s'])
        records.append(record)
        summary = dict(completed=len(records), total=len(tasks), success=len(records), height=args.height, timed=args.timed,
                       records=records, peak_gpu_allocated_mib=torch.cuda.max_memory_allocated() / 2**20)
        (args.output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
        print('PROGRESS', index + 1, '/', len(tasks), task, round(record['wall_ms']), flush=True)
    del planner
    gc.collect()
    torch.cuda.empty_cache()


if __name__ == '__main__':
    main()
