#!/usr/bin/env python3
"""Headless installed-launch checks of the simulation service and RViz marker replay."""
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
from alfa_robot_moveit_config.srv import PlanWallBoxDemo
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

TOPIC = '/v3_box_wall_grasp_demo'


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts', required=True, type=Path)
    args = parser.parse_args()
    root = args.artifacts.resolve()
    root.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('ROS_DOMAIN_ID', '185')
    os.environ['ROS_LOG_DIR'] = str(root / 'ros')
    rclpy.init()
    node = rclpy.create_node('wall_grasp_check')
    received, markers = {}, []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, TOPIC + '/task_json',
                             lambda m: received.update(task=json.loads(m.data)), qos)
    node.create_subscription(JointState, TOPIC + '/joint_states',
                             lambda m: received.update(joints=dict(zip(m.name, m.position))), 10)
    node.create_subscription(MarkerArray, TOPIC + '/scene_markers', markers.append, qos)
    client = node.create_client(PlanWallBoxDemo, TOPIC + '/plan_wall_box')
    summary = []

    def spin_until(predicate, timeout=45):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert predicate(), 'Timed out; inspect launch.log'

    def call(x, box_id, arm='auto'):
        future = client.call_async(PlanWallBoxDemo.Request(x=x, box_id=box_id, arm=arm))
        spin_until(future.done, 120)
        response = future.result()
        assert response is not None
        if response.result_json:
            task = json.loads(response.result_json)
            (root / f'result_{response.generation}.json').write_text(json.dumps(task, indent=2))
            assert task['success'] == response.success
            assert task['generation'] == response.generation
            assert response.selected_arm == (task['side'] if response.success else '')
            assert task['box_id'] == box_id and task['x'] == x
            assert task['verdict'] == ('path_found' if response.success else 'no_path_found')
            summary.append(dict(generation=response.generation, x=x, box_id=box_id,
                                success=response.success, attempts=task['attempts']))
            return response, task
        return response, None

    command = ['ros2', 'launch', 'alfa_robot_moveit_config', 'v3_box_wall_grasp_demo.launch.py', 'check_environment:=false', 'align_height:=true',
               'x:=0.5', 'box_id:=9', 'auto_run_once:=false', 'start_rviz:=false',
               'start_rerun:=false']
    try:
        with (root / 'launch.log').open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            try:
                assert client.wait_for_service(timeout_sec=40)
                spin_until(lambda: 'task' in received and 'joints' in received and markers)
                preview = received['task']
                front = preview['chassis_front_x']
                assert math.isclose(front, 0.5000000050, abs_tol=1e-6), front
                fixed_wall = None
                for box_id in range(25):
                    response, task = call(2.0, box_id)
                    assert not response.success and response.failure_stage == 'precontact_ik'
                    assert [a['arm'] for a in task['attempts']] == ['left', 'right'] * (1 if box_id < 5 else 2)
                    center, size = task['box_center'], task['box_size']
                    assert math.isclose(center[0] - size[0] / 2 - front, 2.0, abs_tol=1e-9)
                    assert math.isclose(center[1], (box_id % 5 - 2) * .41, abs_tol=1e-9)
                    assert math.isclose(center[2], .2 + (box_id // 5) * .41, abs_tol=1e-9)
                    wall = sorted(tuple(round(v, 8) for v in p)
                                  for p in task['neighbor_centers'] + [center])
                    assert len(set(wall)) == 25 and task['collision_inset'] == 0
                    if fixed_wall is None:
                        fixed_wall = wall
                    assert fixed_wall == wall, 'selecting target moved the wall'
                generation = response.generation
                for x, box_id, arm in [(0., 6, 'auto'), (-.1, 6, 'auto'),
                                      (float('nan'), 6, 'auto'), (float('inf'), 6, 'auto'),
                                      (.3, -1, 'auto'), (.3, 25, 'auto'), (.3, 6, 'both')]:
                    response, task = call(x, box_id, arm)
                    assert not response.success and response.failure_stage == 'invalid_request'
                    assert task is None
                response, task = call(.5, 9, 'left')
                assert response.generation == generation + 1, 'invalid input changed generation'
                assert response.success, task['attempts']
                assert [a['arm'] for a in task['attempts']] == ['left']
                stages = list(dict.fromkeys(f['stage'] for f in task['frames']))
                assert stages == ['rrt_to_precontact', 'cartesian_approach', 'attach_box',
                                  'cartesian_retreat', 'rrt_return', 'rear_placement',
                                  'release_box'], stages
                for frame in task['frames']:
                    joints = dict(zip(task['joint_names'], frame['joints']))
                    assert len(joints) == 16 and all(math.isfinite(v) for v in joints.values())
                    assert -1.0 <= joints['updown'] <= 0.0
                    assert joints['head_joint'] == 0
                    assert all(math.isclose(joints[f'right_joint{i}'], task['initial_joints'][7 + i - 1], abs_tol=1e-8) for i in range(1, 8))
                # Wait until RViz reaches the released final frame and hides the transported target.
                from alfa_robot_rerun.visualize_rerun import UrdfRobot, render_current_urdf
                robot = UrdfRobot(render_current_urdf({'model_ground_offset': '0.000005'}))
                import numpy as np
                def final_marker():
                    return (bool(markers) and markers[-1].markers[0].action == markers[-1].markers[0].DELETEALL
                            and not any(marker.ns == 'target_box' for marker in markers[-1].markers))
                spin_until(final_marker, max(45, len(task['frames']) * .05 + 5))
                assert sum(m.type == m.CUBE for m in markers[-1].markers) == 24
                assert not task['frames'][-1]['box_attached'] and not task['frames'][-1]['box_visible']
                (root / 'successful_task.json').write_text(json.dumps(task, indent=2))
                (root / 'marker_check.json').write_text(json.dumps({
                    'frames': len(task['frames']), 'rviz_final_target_hidden': True}, indent=2))
                # Opposite wall edges must exercise both arms, not only left-arm success.
                for box_id, arm in [(9, 'left'), (5, 'right')]:
                    for requested in (arm, 'auto'):
                        response, task = call(.5, box_id, requested)
                        assert response.success and response.selected_arm == arm, task['attempts']
                        assert [a['arm'] for a in task['attempts']] == (
                            ['left', 'right'] if requested == 'auto' and arm == 'right' else [arm])
                        assert math.isclose(task['contact_numerical_gap'], 1e-6)
                        offset = np.array(task['tool_to_box_center'])
                        assert math.isclose(offset[2], task['box_size'][0] / 2 + 1e-6)
                        assert list(dict.fromkeys(f['stage'] for f in task['frames'])) == stages
                        other = 'left' if arm == 'right' else 'right'
                        for frame in task['frames']:
                            values = dict(zip(task['joint_names'], frame['joints']))
                            assert -1.0 <= values['updown'] <= 0.0
                            assert values['head_joint'] == 0
                            offset_index = 0 if other == 'left' else 7
                            assert all(math.isclose(values[f'{other}_joint{i}'], task['initial_joints'][offset_index + i - 1], abs_tol=1e-8) for i in range(1, 8))
                        attach = next(f for f in task['frames'] if f['stage'] == 'attach_box')
                        contact_tool = robot.fk(dict(zip(task['joint_names'], attach['joints'])))[task['tool_link']]
                        # The numerical gap must not teleport the box when attaching it.
                        assert np.allclose(contact_tool[:3, 3] + contact_tool[:3, :3] @ offset,
                                           task['box_center'], atol=2e-7, rtol=0)
                        rear = next(f for f in task['frames'] if f['stage'] == 'rear_placement')
                        rear_tool = robot.fk(dict(zip(task['joint_names'], rear['joints'])))[task['tool_link']]
                        rear_box = rear_tool[:3, 3] + rear_tool[:3, :3] @ offset
                        extent_x = np.abs(rear_tool[0, :3] @ np.array(task['tool_to_box_rotation'])) @ (np.array(task['box_size']) / 2)
                        assert rear_box[0] + extent_x <= task['chassis_rear_x'] - .01 + 1e-6
                        spin_until(final_marker, max(45, len(task['frames']) * .05 + 5))
                print('PASS: 9 left / 5 right, explicit and auto, fixed inactive arm, continuous attach, RViz FK')
                found = []
                for box_id in range(25):
                    scan_response, _ = call(.5, box_id)
                    if scan_response.success:
                        found.append(box_id)
                print(f'INFO x=0.50m bounded-search successful IDs: {found}')
                # Explicit right and auto requests also keep the interface/replay usable after success.
                right, right_task = call(.5, 5, 'right')
                assert [a['arm'] for a in right_task['attempts']] == ['right']
                auto, auto_task = call(.5, 9, 'auto')
                assert right.success and right.selected_arm == 'right'
                assert auto.success and auto.selected_arm == 'left', auto_task['attempts']
                (root / 'summary.json').write_text(json.dumps(summary, indent=2))
                print('PASS: 25 fixed-wall IDs; 7 invalid inputs; left/right/auto; complete grasp; RViz FK replay')
            finally:
                stop(process)
        # Zero gap remains a valid exact-contact configuration on the V3 wrist geometry.
        received.clear()
        with (root / 'zero_contact_gap.log').open('w') as log:
            process = subprocess.Popen(command + ['contact_numerical_gap:=0.0'], stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            try:
                spin_until(lambda: received.get('task', {}).get('contact_numerical_gap') == 0.0)
                response, task = call(.5, 5, 'right')
                assert response.success, task['attempts']
                assert task['tool_to_box_center'][2] == task['box_size'][0] / 2
                (root / 'zero_contact_gap.json').write_text(json.dumps(task, indent=2))
                print('PASS: V3 zero-gap exact contact remains plannable')
            finally:
                stop(process)
        # A calibrated front override must change wall X exactly, not the definition of x.
        received.clear()
        with (root / 'override.log').open('w') as log:
            process = subprocess.Popen(command + ['chassis_front_x:=0.42'], stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            try:
                spin_until(lambda: received.get('task', {}).get('chassis_front_x') == .42)
                assert math.isclose(received['task']['box_center'][0], .42 + .5 + .15)
                (root / 'override.json').write_text(json.dumps(received['task'], indent=2))
                print('PASS: explicit calibrated chassis-front override')
            finally:
                stop(process)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
