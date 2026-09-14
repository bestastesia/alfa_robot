#!/usr/bin/env python3
"""Installed top-only single-box diagnostic; retains the other 24 boxes and environment."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray
from alfa_robot_moveit_config.srv import PlanWallBoxDemo
from test_v3_box_wall_grasp_demo import stop
from alfa_robot_rerun.visualize_rerun import UrdfRobot, render_current_urdf


def reach_certificate(task, robot):
    """Independent URDF joint-axis intersections, no analytic solver internals."""
    fk = robot.fk({})
    measurements = []
    for side in ('left', 'right'):
        def center(indices):
            lhs, rhs = np.zeros((3, 3)), np.zeros(3)
            for i in indices:
                name = f'{side}_joint{i}'
                tf = fk[name]
                axis = tf[:3, :3] @ robot.joints[name].axis
                projector = np.eye(3) - np.outer(axis, axis)
                lhs += projector
                rhs += projector @ tf[:3, 3]
            return np.linalg.solve(lhs, rhs)
        shoulder, elbow, wrist = (center(indices) for indices in ((1, 2, 3), (3, 4, 5), (5, 6, 7)))
        length = np.linalg.norm(elbow - shoulder) + np.linalg.norm(wrist - elbow)
        tool = fk[f'{side}_tool0']
        offset = tool[:3, :3].T @ (wrist - tool[:3, 3])
        # Downward toolZ: any yaw leaves the wrist's horizontal offset this small.
        wrist_xy_offset = np.linalg.norm(offset[:2])
        closest = np.clip(shoulder[:2], np.asarray(task['box_center'])[:2] - np.asarray(task['box_size'])[:2] / 2,
                          np.asarray(task['box_center'])[:2] + np.asarray(task['box_size'])[:2] / 2)
        lower_bound = np.linalg.norm(closest - shoulder[:2]) - wrist_xy_offset
        assert lower_bound > length, 'Cannot certify all top-face points unreachable'
        measurements.append(dict(arm=side, shoulder=shoulder.tolist(), arm_length=float(length),
            tcp_to_wrist=offset.tolist(), nearest_top_wrist_xy_lower_bound=float(lower_bound)))
    return measurements


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts', required=True, type=Path)
    args = parser.parse_args()
    root = args.artifacts.resolve()
    root.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('ROS_DOMAIN_ID', '199')
    os.environ['ROS_LOG_DIR'] = str(root / 'ros')
    robot = UrdfRobot(render_current_urdf({'model_ground_offset': '0.402201'}))
    rclpy.init(args=[])
    node = rclpy.create_node('top_suction_check')
    received = {}
    topic = '/v3_box_wall_grasp_demo'
    client = node.create_client(PlanWallBoxDemo, topic + '/plan_wall_box')
    assert not client.wait_for_service(timeout_sec=1), 'Use an unused ROS domain'
    node.create_subscription(String, topic + '/task_json',
        lambda m: received.update(task=json.loads(m.data)),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    joints = []
    markers = []
    node.create_subscription(JointState, topic + '/joint_states',
        lambda message: joints.append(list(message.position)), 10)
    node.create_subscription(MarkerArray, topic + '/scene_markers',
        lambda message: markers.append(message), 10)
    command = ['ros2', 'launch', 'alfa_robot_moveit_config',
               'v3_box_wall_comfort_grasp_demo.launch.py', 'x:=0.8', 'box_id:=7',
               'suction_mode:=top', 'start_rviz:=false', 'start_rerun:=false',
               'planning_seed:=104729']
    with (root / 'launch.log').open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        def wait(predicate):
            end = time.monotonic() + 180
            while not predicate() and time.monotonic() < end:
                assert process.poll() is None, 'Launch exited; inspect launch.log'
                rclpy.spin_once(node, timeout_sec=.05)
            assert predicate(), 'Timeout; inspect launch.log'

        def check(task):
            (root / f"{task['generation']}_box{task['box_id']}_{task['requested_arm']}_x{task['x']}.json").write_text(
                json.dumps(task, indent=2))
            assert task['requested_suction_mode'] == task['suction_mode'] == 'top'
            assert task['environment']['enabled'] and len(task['neighbor_centers']) == 24
            assert not task['removed_box_ids'] and not task['release_after_transfer']
            expected = ['left', 'right'] if task['requested_arm'] == 'auto' else [task['requested_arm']]
            assert [a['arm'] for a in task['attempts']] == expected[:len(task['attempts'])]
            assert all(a['suction_mode'] == 'top' for a in task['attempts'])
            assert task['precontact'][2] > task['contact'][2] > task['box_center'][2]
            if not task['success']:
                assert len(task['attempts']) == len(expected)
                assert len(task['frames']) == 1
                assert task['frames'][0]['joints'] == task['initial_joints']
                diagnostic = task['diagnostic']
                assert diagnostic['diagnostic_only'] and diagnostic['freeze_at_end']
                assert diagnostic['stage'] == task['failure_stage']
                assert task['diagnostic_frames'] and all(f['diagnostic_only'] for f in task['diagnostic_frames'])
                assert task['failure_stage'] == 'precontact_ik', task['failure_reason']
                assert 'reach' in task['failure_reason']
                assert not diagnostic['contacts']
                assert all(not f['box_attached'] and f['joints'] == task['initial_joints']
                           for f in task['diagnostic_frames']), 'Unreachable pickup must not move/attach'
                assert task['height_alignment']['strategy'] == 'top_wrist_alignment'
            if task['box_id'] == 7 and task['x'] == .9:
                (root / 'x090_reach_certificate.json').write_text(json.dumps(reach_certificate(task, robot), indent=2))
            print(f"box={task['box_id']} x={task['x']} success={task['success']}")
            for attempt in task['attempts']:
                print(attempt['arm'], attempt['failure_stage'], attempt['failure_reason'])

        try:
            assert client.wait_for_service(timeout_sec=45)
            wait(lambda: received.get('task', {}).get('kind') == 'result')
            check(received['task'])  # Auto-run must respect the launch parameter too.
            final = received['task']['diagnostic_frames'][-1]['joints']
            wait(lambda: len(joints) >= 8 and all(sample == final for sample in joints[-8:])
                 and markers and any(m.ns == 'failure_diagnostic' and m.action == m.ADD for m in markers[-1].markers))
            # A failure must stay visible rather than looping or returning to home.
            held = time.monotonic() + 1.0
            while time.monotonic() < held:
                rclpy.spin_once(node, timeout_sec=.05)
                assert joints[-1] == final
            (root / 'freeze_check.json').write_text(json.dumps(dict(
                frozen_joints=final, contacts=received['task']['diagnostic']['contacts'], held_seconds=1)))
            for x, box_id, arm in ((.8, 7, 'left'), (.8, 7, 'right'), (.9, 7, 'auto'), (.9, 0, 'auto')):
                future = client.call_async(PlanWallBoxDemo.Request(x=x, box_id=box_id, arm=arm))
                wait(future.done)
                response = future.result()
                task = json.loads(response.result_json)
                assert task['success'] == response.success
                check(task)
        finally:
            stop(process)
            node.destroy_node()
            rclpy.shutdown()
    print('PASS top-only: startup/service, both arms, bottom/non-bottom, collision retention and rollback')


if __name__ == '__main__':
    main()
