#!/usr/bin/env python3
"""Offline generation + fresh-process read-only midpoint comparison (simulation only)."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time
import xml.etree.ElementTree as ET


TOPIC = '/v3_box_wall_grasp_demo'


def summarize(records):
    """Coverage first; never interpret failed or missing runs as faster planning."""
    groups = {}
    for record in records:
        if record['mode'] == 'build':
            continue
        groups.setdefault(record['candidate'], []).append(record)
    reports = []
    for candidate, runs in groups.items():
        success = [r for r in runs if r['success']]
        def median(key):
            values = [r[key] for r in success if key in r]
            return statistics.median(values) if values else None
        reports.append(dict(candidate=candidate, success=len(success), runs=len(runs),
                            median_online_ms=median('total_ms'),
                            median_return_weighted_length=median('return_weighted_length'),
                            median_return_joint_reversals=median('return_joint_reversals'),
                            median_collision_checks=median('collision_checks'),
                            median_ik_calls=median('ik_calls'),
                            median_dynamic_ms=median('dynamic_return_ms'),
                            median_suffix_validation_ms=median('suffix_validation_ms'),
                            median_tcp_distance_m=median('retreat_to_midpoint_tcp_distance_m')))
    # Multi-objective evidence, not an assertion that minimum Cartesian distance wins.
    for report in reports:
        comparable = [r for r in reports if r['success'] / r['runs'] >= report['success'] / report['runs']
                      and r['median_online_ms'] is not None and r['median_return_weighted_length'] is not None]
        report['pareto_efficient'] = report['median_return_weighted_length'] is not None and not any(
            r['median_online_ms'] <= report['median_online_ms'] and
            r['median_return_weighted_length'] <= report['median_return_weighted_length'] and
            (r['median_online_ms'] < report['median_online_ms'] or
             r['median_return_weighted_length'] < report['median_return_weighted_length'])
            for r in comparable)
    return sorted(reports, key=lambda r: (-r['success'] / r['runs'],
                                        r['median_online_ms'] if r['median_online_ms'] is not None else math.inf))


def motion_metrics(task):
    indices = [task['joint_names'].index(f"{task['side']}_joint{i}") for i in range(1, 8)]
    stages = {'rrt_return', 'rrt_to_midpoint', 'offline_carry'}
    states = []
    previous = None
    for frame in task['frames']:
        if frame['stage'] in stages:
            if not states and previous is not None:
                states.append(previous)
            states.append(frame['joints'])
        previous = frame['joints']
    weights = [1, 1, 1, 1, 2, 3, 5]
    length, reversals = 0., 0
    signs = [0] * 7
    for a, b in zip(states, states[1:]):
        deltas = [b[j] - a[j] for j in indices]  # bounded V3 joints: no fictitious +/-pi wrap
        length += math.sqrt(sum(w*d*d for w, d in zip(weights, deltas)))
        for i, delta in enumerate(deltas):
            if abs(delta) > 1e-6:
                sign = 1 if delta > 0 else -1
                reversals += int(signs[i] != 0 and sign != signs[i])
                signs[i] = sign
    return dict(return_weighted_length=length, return_joint_reversals=reversals)


def main():
    import rclpy
    from alfa_robot_moveit_config.srv import PlanWallBoxDemo
    from rcl_interfaces.srv import GetParameters
    from ament_index_python.packages import get_package_prefix

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts', required=True, type=Path)
    parser.add_argument('--x', type=float, default=.5)
    parser.add_argument('--box-ids', type=int, nargs='+', default=[9, 14, 19])
    parser.add_argument('--seeds', type=int, nargs='+', default=[42, 43, 44])
    parser.add_argument('--yaw-offset-deg', type=float, nargs='+', default=[0, 20, 40])
    parser.add_argument('--elbow-deg', type=float, nargs='+', default=[90, 140])
    parser.add_argument('--suction-mode', choices=['auto', 'top'], default='auto')
    parser.add_argument('--arm', choices=['left', 'right'], default='left')
    parser.add_argument('--timeout', type=float, default=300)
    parser.add_argument('--validator', type=Path, default=Path(__file__).resolve().parents[1] /
                        'ros2_ws/build/alfa_robot_moveit_config/validate_v3_wall_replay')
    args = parser.parse_args()
    if any(seed <= 0 for seed in args.seeds):
        parser.error('positive seeds required for reproducible comparisons')
    root = args.artifacts.resolve()
    root.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('ROS_DOMAIN_ID', '196')
    os.environ['ROS_LOG_DIR'] = str(root / 'ros')
    rclpy.init()
    executable = Path(get_package_prefix('alfa_robot_moveit_config')) / 'lib/alfa_robot_moveit_config/v3_single_arm_box_extract_demo'
    binary_hash = hashlib.sha256(executable.read_bytes()).hexdigest()
    (root / 'manifest.json').write_text(json.dumps(dict(
        git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        git_status=subprocess.check_output(['git', 'status', '--short'], text=True),
        executable=str(executable), executable_sha256=binary_hash,
        source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in [
            Path(__file__), Path(__file__).resolve().parents[1] /
            'ros2_ws/src/alfa_robot_moveit_config/src/v3_single_arm_box_extract_demo.cpp']}
    ), indent=2))
    records = []
    nominal = {}

    def wait(node, future):
        deadline = time.monotonic() + args.timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if not future.done():
            raise TimeoutError('planning/parameter timeout; inspect launch.log')
        if future.exception():
            raise future.exception()
        return future.result()

    def run(candidate, mode, seed, joints=None):
        assert hashlib.sha256(executable.read_bytes()).hexdigest() == binary_hash, 'binary rebuilt during benchmark'
        directory = root / candidate / f'{mode}_{seed}'
        directory.mkdir(parents=True, exist_ok=True)
        cache = root / candidate / 'suffix.json'
        before = hashlib.sha256(cache.read_bytes()).hexdigest() if mode == 'use' else None
        command = ['ros2', 'launch', 'alfa_robot_moveit_config', 'v3_box_wall_grasp_demo.launch.py',
                   f'x:={args.x}', f'arm:={args.arm}', f'suction_mode:={args.suction_mode}', 'auto_run_once:=false',
                   'start_rviz:=false', 'start_rerun:=false', 'check_environment:=true',
                   'wall_context:=sequence_prefix', f'planning_seed:={seed}', f'transfer_mode:={mode}',
                   f'transfer_cache_file:={cache}', 'transfer_joints_json:=' + json.dumps(joints or {})]
        process = None
        node = None
        try:
            with (directory / 'launch.log').open('w') as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                           start_new_session=True)
                node = rclpy.create_node(f'transfer_midpoint_benchmark_{time.time_ns()}')
                client = node.create_client(PlanWallBoxDemo, TOPIC + '/plan_wall_box')
                parameters = node.create_client(GetParameters, TOPIC + '/get_parameters')
                if not client.wait_for_service(timeout_sec=40):
                    raise RuntimeError('service startup failed: ' + str(directory / 'launch.log'))
                response = wait(node, parameters.call_async(GetParameters.Request(
                    names=['robot_description', 'robot_description_semantic'])))
                for name, value in zip(['robot.urdf', 'robot.srdf'], response.values):
                    (directory / name).write_text(value.string_value)
                if not nominal:
                    srdf = ET.fromstring(response.values[1].string_value)
                    home = next(s for s in srdf.findall('group_state')
                                if s.get('name') == 'home' and s.get('group') == 'whole_body')
                    values = {j.get('name'): float(j.get('value')) for j in home.findall('joint')}
                    nominal.update({side: [values[f'{side}_joint{i}'] for i in range(1, 8)]
                                    for side in ['left', 'right']})
                for box_id in args.box_ids:
                    started = time.monotonic()
                    response = wait(node, client.call_async(PlanWallBoxDemo.Request(
                        x=args.x, box_id=box_id, arm=args.arm)))
                    task = json.loads(response.result_json)
                    result_file = directory / f'box_{box_id}.json'
                    result_file.write_text(json.dumps(task, indent=2))
                    record = dict(candidate=candidate, mode=mode, seed=seed, box_id=box_id,
                                  success=task['success'], total_ms=task['total_ms'],
                                  request_wall_ms=1000*(time.monotonic()-started),
                                  failure_stage=task['failure_stage'], failure_reason=task['failure_reason'],
                                  collision_checks=task['metrics']['collision_checks'],
                                  ik_calls=task['metrics']['ik_calls'])
                    record.update(task.get('transfer', {}))
                    # task.transfer.mode is identical; do not permit cache metadata to change identity.
                    record['mode'] = mode
                    if task['success']:
                        record.update(motion_metrics(task))
                        if mode == 'use':
                            transfer = task['transfer']
                            assert transfer['cache_hit'] and transfer['suffix_planner_calls'] == 0
                            assert transfer['placement_ik_calls'] == 0
                            dynamic = [f for f in task['frames'] if f['stage'] == 'rrt_to_midpoint']
                            carry = [f for f in task['frames'] if f['stage'] == 'offline_carry']
                            assert dynamic and carry and dynamic[-1]['joints'] == carry[0]['joints']
                            # Compare the replayed arm suffix to persisted data, not a regenerated path.
                            indices = [task['joint_names'].index(f'{args.arm}_joint{i}') for i in range(1, 8)]
                            replay = [[f['joints'][i] for i in indices] for f in carry]
                            entries = json.loads(cache.read_text())['entries'].values()
                            assert replay in entries, 'cached suffix changed during online planning'
                        if args.validator:
                            check = subprocess.run([str(args.validator.resolve()), str(result_file),
                                str(directory / 'robot.urdf'), str(directory / 'robot.srdf')],
                                capture_output=True, text=True)
                            (directory / f'validate_{box_id}.log').write_text(check.stdout + check.stderr)
                            if check.returncode:
                                raise AssertionError(check.stdout + check.stderr)
                            record['independent_replay_valid'] = True
                    records.append(record)
                    (root / 'records.json').write_text(json.dumps(records, indent=2))
                    print(json.dumps(record), flush=True)
        finally:
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            if node is not None:
                node.destroy_node()
        if mode == 'use':
            assert hashlib.sha256(cache.read_bytes()).hexdigest() == before, 'online cache write'

    try:
        for seed in args.seeds:
            run('baseline', 'disabled', seed)
        for offset, elbow in [(o, e) for o in args.yaw_offset_deg for e in args.elbow_deg]:
            candidate = f'home_yaw_{offset:g}_elbow_{elbow:g}'
            joints = {side: list(values) for side, values in nominal.items()}
            joints['left'][0] -= math.radians(offset)
            joints['right'][0] += math.radians(offset)
            for side in joints:
                joints[side][3] = math.radians(elbow)
            (root / candidate).mkdir(exist_ok=True)
            (root / candidate / 'midpoint.json').write_text(json.dumps(joints, indent=2))
            run(candidate, 'build', args.seeds[0], joints)
            if not (root / candidate / 'suffix.json').exists():
                # No feasible offline suffix: report failures instead of silently dropping the candidate.
                for seed in args.seeds:
                    for box_id in args.box_ids:
                        records.append(dict(candidate=candidate, mode='use', seed=seed,
                                            box_id=box_id, success=False,
                                            failure_stage='offline_cache_unavailable'))
                continue
            for seed in args.seeds:
                run(candidate, 'use', seed, joints)
        report = dict(parameters=vars(args) | {'artifacts': str(root), 'validator': str(args.validator)},
                      ranking=summarize(records), scope='finite-sample geometric/positional comparison; not hardware timing or global optimum')
        (root / 'records.json').write_text(json.dumps(records, indent=2))
        # Export an experimental, fully covered candidate; never change launch defaults from a smoke test.
        feasible = [r for r in report['ranking'] if r['candidate'] != 'baseline' and
                    r['success'] == r['runs'] and r['pareto_efficient']]
        if feasible:
            choice = feasible[0]['candidate']
            report['experimental_selection'] = choice
            (root / 'selected_midpoint.json').write_text(json.dumps(dict(
                transfer_mode='use', transfer_cache_file=str(root / choice / 'suffix.json'),
                transfer_joints_json=(root / choice / 'midpoint.json').read_text(),
                requires_heldout_validation=True, measurements=feasible[0]), indent=2))
        else:
            report['experimental_selection'] = None
        (root / 'summary.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
