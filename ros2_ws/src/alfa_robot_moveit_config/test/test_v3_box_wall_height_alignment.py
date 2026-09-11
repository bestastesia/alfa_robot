#!/usr/bin/env python3
"""Installed-launch height alignment/FK/replay checks; simulation only, no hardware.

Source Humble + workspace; use an unused localhost ROS_DOMAIN_ID.
"""
import argparse
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from alfa_robot_moveit_config.srv import PlanWallBoxDemo
from alfa_robot_rerun.visualize_rerun import UrdfRobot, render_current_urdf
from test_v3_box_wall_grasp_demo import stop

TOPIC = '/v3_box_wall_grasp_demo'
STAGES = ['rrt_to_precontact', 'cartesian_approach', 'attach_box',
          'cartesian_retreat', 'rrt_return']


def shoulder_z(robot, joints):
    # Independent URDF/FK common-axis intersection, not the C++ solver geometry.
    transforms = robot.fk(joints)
    centers = []
    for side in ('left', 'right'):
        lhs, rhs = np.zeros((3, 3)), np.zeros(3)
        for i in range(1, 4):
            transform = transforms[f'{side}_joint{i}']
            axis = transform[:3, 2]
            projector = np.eye(3) - np.outer(axis, axis)
            lhs += projector
            rhs += projector @ transform[:3, 3]
        centers.append(np.linalg.solve(lhs, rhs))
    return float(np.mean(centers, axis=0)[2])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts', required=True, type=Path)
    args = parser.parse_args()
    root = args.artifacts.resolve()
    root.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('ROS_DOMAIN_ID', '188')
    os.environ['ROS_LOG_DIR'] = str(root / 'ros')
    robot = UrdfRobot(render_current_urdf())
    initial_z = shoulder_z(robot, {})
    command = ['ros2', 'launch', 'alfa_robot_moveit_config',
               'v3_box_wall_grasp_demo.launch.py', 'x:=0.30',
               'auto_run_once:=false', 'start_rviz:=false', 'start_rerun:=false']
    summary = []

    @contextmanager
    def launch(label, extra=()):
        with (root / f'{label}.log').open('w') as log:
            process = subprocess.Popen(command + list(extra), stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            node = rclpy.create_node('wall_height_check')
            received = {}
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            node.create_subscription(String, TOPIC + '/task_json',
                                     lambda m: received.update(task=json.loads(m.data)), qos)
            node.create_subscription(MarkerArray, TOPIC + '/scene_markers',
                                     lambda m: received.update(markers=m.markers), qos)
            client = node.create_client(PlanWallBoxDemo, TOPIC + '/plan_wall_box')

            def spin_until(predicate, timeout=60):
                end = time.monotonic() + timeout
                while not predicate() and time.monotonic() < end:
                    assert process.poll() is None, (root / f'{label}.log').read_text()
                    rclpy.spin_once(node, timeout_sec=.05)
                assert predicate(), f'{label}: timeout'

            def call(box_id, arm='auto', x=.3):
                future = client.call_async(PlanWallBoxDemo.Request(x=x, box_id=box_id, arm=arm))
                spin_until(future.done, 180)
                response = future.result()
                task = json.loads(response.result_json)
                assert task['success'] == response.success
                assert task['generation'] == response.generation
                assert task['failure_stage'] == response.failure_stage
                assert response.selected_arm == (task['side'] if response.success else '')
                path = root / f'{label}_{response.generation}_box{box_id}_{arm}.json'
                path.write_text(json.dumps(task, indent=2))
                summary.append(dict(case=path.stem, box_id=box_id, success=response.success,
                                    attempts=task['attempts'], frames=len(task['frames'])))
                return task

            try:
                assert client.wait_for_service(timeout_sec=45)
                spin_until(lambda: 'task' in received)
                yield call, spin_until, received
            finally:
                stop(process)
                node.destroy_node()
                rclpy.shutdown()

    def check(task):
        alignment = task['height_alignment']
        assert math.isclose(alignment['initial_shoulder_z'], initial_z, abs_tol=1e-6)
        descent = max(0., initial_z - task['box_center'][2] - alignment['shoulder_box_offset'])
        assert math.isclose(alignment['descent'], descent, abs_tol=1e-6)
        frames = task['frames']
        assert all(v == 0 for v in frames[0]['joints']), 'Request did not reset initial state'
        stages = list(dict.fromkeys(frame['stage'] for frame in frames))
        if task['success']:
            assert stages == (['lower_to_box_height'] if descent > 0 else []) + STAGES, stages
        else:
            assert task['failure_stage'], 'Failure without feedback'
        lift = task['joint_names'].index('updown')
        other = 'right' if task['side'] == 'left' else 'left'
        previous = 0.
        for frame in frames:
            joints = dict(zip(task['joint_names'], frame['joints']))
            assert len(joints) == 16 and all(math.isfinite(v) for v in joints.values())
            assert -1. <= joints['updown'] <= 0. and joints['head_joint'] == 0.
            assert all(joints[f'{other}_joint{i}'] == 0. for i in range(1, 8))
            if frame['stage'] == 'lower_to_box_height':
                assert not frame['box_attached']
                assert all(v == 0 for name, v in joints.items() if name != 'updown')
                assert -0.005000001 <= joints['updown'] - previous <= 1e-12
                previous = joints['updown']
            else:
                assert math.isclose(joints['updown'], -descent, abs_tol=1e-6)
        assert math.isclose(frames[-1]['joints'][lift], -descent, abs_tol=1e-6)
        if descent > 0:
            assert math.isclose(shoulder_z(robot, {'updown': -descent}),
                                task['box_center'][2] + alignment['shoulder_box_offset'], abs_tol=1e-6)
        if task['success']:
            for frame in frames:
                if frame['stage'] == 'attach_box':
                    joints = dict(zip(task['joint_names'], frame['joints']))
                    tool = robot.fk(joints)[task['tool_link']]
                    center = tool[:3, 3] + tool[:3, :3] @ task['tool_to_box_center']
                    assert np.allclose(center, task['box_center'], atol=1e-6), 'Attach teleported box'
            final = dict(zip(task['joint_names'], frames[-1]['joints']))
            assert all(abs(final[f'{task["side"]}_joint{i}']) < 1e-8 for i in range(1, 8))
            assert frames[-1]['box_attached']

    with launch('default') as (call, spin_until, received):
        assert received['task']['height_alignment']['enabled']
        coverage = []
        for box_id in range(10):
            task = call(box_id)
            check(task)
            if task['success']:
                coverage.append(box_id)
            else:
                assert [a['arm'] for a in task['attempts']] == ['left', 'right']
        # Positive regressions, not a claim that other IDs are physically unreachable.
        assert {0, 4, 5, 9}.issubset(coverage), coverage
        for box_id, arm in [(0, 'left'), (4, 'right'), (5, 'left'), (9, 'right'),
                            (10, 'auto'), (14, 'auto'), (20, 'auto'), (24, 'auto'), (0, 'auto')]:
            task = call(box_id, arm)
            assert task['success'], task['attempts']
            check(task)
        # The last low request follows high requests; replay must match independent Rerun FK.
        final = dict(zip(task['joint_names'], task['frames'][-1]['joints']))
        tool = robot.fk(final)[task['tool_link']]
        expected = tool[:3, 3] + tool[:3, :3] @ task['tool_to_box_center']

        def final_marker():
            return any(m.ns == 'target_box' and np.allclose(
                [m.pose.position.x, m.pose.position.y, m.pose.position.z], expected, atol=1e-6)
                for m in received.get('markers', []))
        spin_until(final_marker, max(60, len(task['frames']) * .05 + 5))
        (root / 'marker_check.json').write_text(json.dumps(dict(
            expected_final_center=expected.tolist(), rviz_matches_rerun_fk=True,
            lower_two_rows_success=coverage), indent=2))
        task = call(0, x=2.)
        assert not task['success'] and task['failure_stage'] == 'precontact_ik'
        check(task)  # Failed grasp retains validated lift prefix, no teleport back to zero.

    with launch('overtravel', ['shoulder_box_offset:=0.0']) as (call, _, received):
        assert received['task']['height_alignment']['shoulder_box_offset'] == 0.
        task = call(0)
        assert not task['success'] and task['failure_stage'] == 'height_alignment_limits'
        assert task['height_alignment']['target_updown'] < -1.
        assert len(task['frames']) == 1 and all(v == 0 for v in task['frames'][0]['joints'])
        assert all(a['failure_stage'] == 'height_alignment_limits' for a in task['attempts'])

    with launch('no_lower', ['shoulder_box_offset:=2.0']) as (call, _, received):
        task = call(6)
        assert task['success'] and task['height_alignment']['descent'] == 0.
        assert task['height_alignment']['height_difference'] < 0.
        check(task)

    # Just inside the URDF lower limit; requested height is not silently clamped.
    offset = initial_z - .2 - .999999
    with launch('lower_limit', [f'shoulder_box_offset:={offset}']) as (call, _, received):
        task = call(0)
        assert not task['failure_stage'].startswith('height_alignment_'), task['failure_reason']
        check(task)
        assert math.isclose(task['frames'][-1]['joints'][14], -.999999, abs_tol=1e-6)

    for value in ('-0.01', 'nan', 'inf'):
        path = root / f'invalid_{value}.log'
        with path.open('w') as log:
            process = subprocess.Popen(command + [f'shoulder_box_offset:={value}'], stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            try:
                process.wait(timeout=40)
            finally:
                stop(process)
        text = path.read_text()
        assert 'shoulder_box_offset must be finite and nonnegative' in text, text
        assert 'process has died' in text and 'sending signal' in text
    (root / 'summary.json').write_text(json.dumps(summary, indent=2))
    print(f'PASS: {len(summary)} requests; 3 invalid offsets; lower rows success={coverage}; '
          'FK, lift prefix, bounds, reset, both arms, attach and RViz replay')


if __name__ == '__main__':
    main()
