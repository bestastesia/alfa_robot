#!/usr/bin/env python3
"""Installed-launch regression: fail fast, planner lifetime, and repeated service restart.

Run after sourcing the workspace, on an unused localhost ROS_DOMAIN_ID.
"""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import rclpy
from alfa_robot_moveit_config.srv import PlanWallBoxDemo
from test_v3_box_wall_grasp_demo import stop


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts', type=Path, required=True)
    parser.add_argument('--launch', default='v3_box_wall_grasp_demo.launch.py',
                        choices=['v3_box_wall_grasp_demo.launch.py', 'v3_box_wall_comfort_grasp_demo.launch.py'])
    args = parser.parse_args()
    root = args.artifacts.resolve()
    root.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('ROS_DOMAIN_ID', '188')
    os.environ['ROS_LOG_DIR'] = str(root / 'ros')
    command = ['ros2', 'launch', 'alfa_robot_moveit_config',
               args.launch, 'align_height:=true', 'wall_context:=target_only',
               'post_extract_policy:=loaded_home', 'start_rviz:=false',
               'start_rerun:=false', 'auto_run_once:=false']
    checks = []

    def assert_children_stopped(log):
        statuses = [Path(f'/proc/{pid}/status')
                    for pid in re.findall(r'process started with pid \[(\d+)\]', log)]
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if all(not status.exists() or 'State:\tZ' in status.read_text()
                   for status in statuses):
                return
            time.sleep(.05)
        live = [status.parent.name for status in statuses
                if status.exists() and 'State:\tZ' not in status.read_text()]
        assert not live, live

    # The original typo must fail before launching even robot_state_publisher.
    for label, parameters, expected in [
        ('invalid_arm', ['x:=0.30', 'arm:=auto~'], 'is not valid'),
        ('invalid_x', ['x:=-0.30', 'arm:=auto'], 'x must be finite/positive'),
        ('negative_gap', ['x:=0.30', 'contact_numerical_gap:=-0.000001'],
         'contact_numerical_gap must be finite'),
        ('oversized_gap', ['x:=0.30', 'contact_numerical_gap:=0.01'],
         'contact_numerical_gap must be finite'),
    ] + ([
        ('invalid_band', ['x:=0.90', 'comfort_ratio_min:=2.0'], 'invalid comfort interval'),
        ('nan_ratio', ['x:=0.90', 'comfort_ratio_preferred:=nan'], 'comfort geometry must be finite'),
        ('invalid_branch', ['x:=0.90', 'comfort_branch:=other'], 'invalid comfort interval'),
        ('invalid_seed', ['x:=0.90', 'planning_seed:=-1'], 'planning_seed must be nonnegative'),
    ] if 'comfort' in args.launch else []):
        path = root / f'{label}.log'
        with path.open('w') as log:
            process = subprocess.Popen(command + parameters, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            try:
                process.wait(timeout=40)
            finally:
                stop(process)
        text = path.read_text()
        assert expected in text, text
        if label == 'invalid_arm':
            assert process.returncode != 0, text
            assert 'process started with pid' not in text, text
        else:
            assert 'process has died' in text and 'sending signal' in text, text
        assert_children_stopped(text)
        checks.append({'case': label, 'passed': True, 'returncode': process.returncode})

    rclpy.init()
    node = rclpy.create_node('wall_grasp_startup_check')
    client = node.create_client(PlanWallBoxDemo, '/v3_box_wall_grasp_demo/plan_wall_box')
    try:
        assert not client.wait_for_service(timeout_sec=2), 'Use an unused ROS domain'
        for cycle in range(3):
            path = root / f'restart_{cycle + 1}.log'
            with path.open('w') as log:
                process = subprocess.Popen(command + ['x:=0.90', 'arm:=auto'],
                                           stdout=log, stderr=subprocess.STDOUT,
                                           start_new_session=True)
                try:
                    assert client.wait_for_service(timeout_sec=40), path.read_text()
                    request = PlanWallBoxDemo.Request(x=.50, box_id=9, arm='left')
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=120)
                    assert future.done(), 'Service response timeout'
                    response = future.result()
                    assert response.success, response.failure_reason
                    assert response.generation == 1, 'Not a fresh server'
                    task = json.loads(response.result_json)
                    stages = list(dict.fromkeys(frame['stage'] for frame in task['frames']))
                    assert stages == ['lower_to_box_height', 'rrt_to_precontact',
                                      'cartesian_approach', 'attach_box',
                                      'cartesian_retreat', 'rrt_return',
                                      'updown_return', 'loaded_home'], stages
                    assert task['post_extract_policy'] == 'loaded_home'
                    assert not task['release_after_transfer']
                    assert task['frames'][-1]['box_attached']
                    (root / f'restart_{cycle + 1}_task.json').write_text(
                        json.dumps(task, indent=2))
                    checks.append({'case': f'restart_{cycle + 1}', 'passed': True,
                                   'success': response.success, 'generation': response.generation,
                                   'selected_arm': response.selected_arm,
                                   'frames': len(task['frames']), 'stages': stages})
                    if cycle == 1:
                        # Even an unexpected planner exit must terminate the whole launch.
                        match = re.search(r'v3_single_arm_box_extract_demo[^\n]*'
                                          r'process started with pid \[(\d+)\]', path.read_text())
                        assert match, path.read_text()
                        os.kill(int(match[1]), signal.SIGTERM)
                        process.wait(timeout=20)
                        checks.append({'case': 'planner_exit_shuts_down_launch', 'passed': True})
                    else:
                        # A terminal close sends SIGHUP; test that as well as Ctrl+C.
                        if cycle == 2:
                            os.killpg(process.pid, signal.SIGHUP)
                            process.wait(timeout=20)
                        else:
                            stop(process)
                finally:
                    stop(process)
            assert_children_stopped(path.read_text())
            # Abrupt terminal closure may leave discovery cached until the DDS lease expires.
            deadline = time.monotonic() + 60
            while client.service_is_ready() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.2)
            assert not client.service_is_ready(), 'Service survived shutdown'
    finally:
        node.destroy_node()
        rclpy.shutdown()
    (root / 'startup_summary.json').write_text(json.dumps(checks, indent=2))
    print(json.dumps(checks, indent=2))


if __name__ == '__main__':
    main()
