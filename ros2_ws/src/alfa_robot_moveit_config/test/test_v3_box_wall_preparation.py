#!/usr/bin/env python3
"""Installed-launch regression for simulation geometry, full state and input rejection."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
import yaml


TOPIC = '/v3_single_arm_box_extract_demo'


def run_case(root, name, parameters, *, plan=False, error=None):
    directory = root / name
    directory.mkdir(parents=True, exist_ok=True)
    config = directory / 'parameters.yaml'
    if parameters is not None:
        config.write_text(yaml.safe_dump({'v3_single_arm_box_extract_demo':
                                        {'ros__parameters': parameters}}))
    received = {}
    node = rclpy.create_node('box_wall_preparation_check')
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, TOPIC + '/task_json',
                             lambda m: received.update(task=json.loads(m.data)), durable)
    node.create_subscription(JointState, TOPIC + '/joint_states',
                             lambda m: received.update(joints=dict(zip(m.name, m.position))), 10)
    node.create_subscription(MarkerArray, TOPIC + '/scene_markers',
                             lambda m: received.update(box_markers=[
                                 {'ns': b.ns, 'center': [b.pose.position.x, b.pose.position.y,
                                                       b.pose.position.z],
                                  'size': [b.scale.x, b.scale.y, b.scale.z]}
                                 for b in m.markers if b.type == b.CUBE]), durable)
    log_path = directory / 'launch.log'
    with log_path.open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', 'alfa_robot_moveit_config',
             'v3_single_arm_box_extract_demo.launch.py',
             'start_rviz:=false', 'start_rerun:=false',
             f'auto_run_once:={str(plan).lower()}',
             *([f'demo_config:={config}'] if parameters is not None else [])],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 45
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if error and error in log_path.read_text():
                    break
                if not error and {'task', 'joints', 'box_markers'} <= received.keys():
                    if not plan or received['task']['kind'] == 'result':
                        break
                if process.poll() is not None:
                    break
            (directory / 'received.json').write_text(json.dumps(received, indent=2))
            if error:
                assert error in log_path.read_text(), log_path
                assert 'task' not in received, 'invalid configuration published a task'
                print(f'PASS {name}: rejected invalid input')
                return None
            assert {'task', 'joints', 'box_markers'} <= received.keys(), log_path
            if plan:
                assert received['task']['kind'] == 'result', log_path
            return received
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            node.destroy_node()


def check_scene(received, wall):
    task = received['task']
    center, size = task['box_center'], task['box_size']
    neighbors = task['neighbor_centers']
    assert len(neighbors) == (24 if wall else 4)
    expected_names = {f'{side}_joint{i}' for side in ('left', 'right') for i in range(1, 8)}
    expected_names.update(('updown', 'head_joint'))
    assert set(received['joints']) == expected_names
    assert received['joints']['updown'] == received['joints']['head_joint'] == 0.0
    assert task['world_frame'] == 'world'
    if wall:
        spec = task['wall']
        assert task['collision_inset'] == 0.0
        expected = [[center[0], center[1] + (c - spec['target_column']) * (size[1] + 0.01),
                     center[2] + (r - spec['target_row']) * (size[2] + 0.01)]
                    for r in range(5) for c in range(5)
                    if (r, c) != (spec['target_row'], spec['target_column'])]
        assert spec['gap'] == 0.01
        for actual, wanted in zip(neighbors, expected):
            assert all(math.isclose(a, b, abs_tol=1e-12) for a, b in zip(actual, wanted))
        assert len({tuple(p) for p in neighbors + [center]}) == 25
    else:
        assert task['collision_inset'] == 0.002
    markers = received['box_markers']
    assert len(markers) == len(neighbors) + 1
    assert sorted(m['center'] for m in markers) == sorted(neighbors + [center])
    assert all(m['size'] == size for m in markers)
    assert math.isclose(task['contact'][0], center[0] - size[0] / 2)
    for frame in task.get('frames', []):
        values = dict(zip(task['joint_names'], frame['joints']))
        assert len(frame['joints']) == 16 and set(values) == expected_names
        assert values['updown'] == values['head_joint'] == 0.0
        assert all(math.isfinite(v) for v in values.values())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts', required=True, type=Path)
    args = parser.parse_args()
    root = args.artifacts.resolve()
    root.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('ROS_DOMAIN_ID', '182')
    os.environ['ROS_LOG_DIR'] = str(root / 'ros')
    rclpy.init()
    try:
        baseline = run_case(root, 'legacy', None, plan=True)
        check_scene(baseline, wall=False)
        assert baseline['task']['success'], baseline['task']['failure_reason']
        stages = list(dict.fromkeys(f['stage'] for f in baseline['task']['frames']))
        assert stages == ['rrt_to_precontact', 'cartesian_approach', 'attach_box',
                          'cartesian_retreat', 'rrt_return'], stages
        assert all(f['box_attached'] == (f['stage'] in
                   ('attach_box', 'cartesian_retreat', 'rrt_return'))
                   for f in baseline['task']['frames'])
        print('PASS legacy: four planning stages, 16-axis replay (not hardware execution)')
        for row, column in [(0, 0), (2, 2), (4, 4)]:
            # Synthetic geometry checks, not a measured or reachable wall pose.
            wall = run_case(root, f'wall_{row}_{column}', {
                'scene_layout': 'wall_5x5', 'wall_target_row': row,
                'wall_target_column': column, 'wall_origin': [1.23, -0.82, 0.20]})
            check_scene(wall, wall=True)
            all_centers = wall['task']['neighbor_centers'] + [wall['task']['box_center']]
            canonical = sorted(tuple(round(v, 8) for v in p) for p in all_centers)
            if row == 0:
                fixed_wall = canonical
            else:
                assert canonical == fixed_wall, 'target selection moved the wall'
            print(f'PASS wall_{row}_{column}: 25 boxes, 1cm gaps, JSON/markers agree')
        unreachable = run_case(root, 'wall_unreachable', {
            'scene_layout': 'wall_5x5', 'wall_origin': [5.0, -0.82, 0.20]}, plan=True)
        check_scene(unreachable, wall=True)
        result = unreachable['task']
        assert not result['success'] and result['failure_stage'] == 'precontact_ik', result
        assert result['metrics']['collision_checks'] >= 2
        print('PASS wall_unreachable: actual planning rejects unreachable target with diagnostics')
        run_case(root, 'invalid_index', {'scene_layout': 'wall_5x5', 'wall_target_row': 5},
                 error='wall target indices must be 0..4')
        run_case(root, 'invalid_gap', {'wall_gap': -0.01},
                 error='wall target indices must be 0..4')
        run_case(root, 'missing_origin', {'scene_layout': 'wall_5x5'},
                 error='wall_origin must contain')
        run_case(root, 'invalid_step', {'edge_joint_resolution_deg': 0.0},
                 error='must be finite/positive')
        run_case(root, 'invalid_inset', {'collision_inset': 0.20},
                 error='collision_inset must preserve positive box dimensions')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
