#!/usr/bin/env python3
"""Installed launches: keep collision rejection safe while displaying/fixing the final failure frame."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray
from std_srvs.srv import Trigger
from test_v3_box_wall_grasp_demo import stop


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts', type=Path, required=True)
    parser.add_argument('--case', default='all')
    args = parser.parse_args()
    root = args.artifacts.resolve()
    root.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('ROS_DOMAIN_ID', '198')
    # An isolated mid-descent obstacle; never disable collision checks.
    obstacle = root / 'lift_environment.json'
    obstacle.write_text(json.dumps(dict(frame_id='world', anchor='world', description='Mid-descent collision fixture with ground', boxes=[dict(
        id='lift_midpath', center=[.7565, .60755, .8], size=[.04, .04, .04]),
        dict(id='ground', center=[0., 0., -.05], size=[10., 10., .1])])))
    return_obstacle = root / 'return_environment.json'
    return_obstacle.write_text(json.dumps(dict(frame_id='world', anchor='world',
        description='Blocked rear-placement goal after connected top pickup', boxes=[
            dict(id='blocked_rear', center=[-1., -.5, 1.34], size=[.3, .5, .5]),
            dict(id='ground', center=[0., 0., -.05], size=[10., 10., .1])])) )
    cases = [
        ('dual_translate', 'v3_dual_arm_cartesian_box_demo', ['target_offset_y:=1.5']),
        ('dual_roll', 'v3_dual_arm_box_roll_demo', ['initial_target_roll_deg:=25', 'maximum_joint_step_deg:=0.01']),
        ('dual_collision', 'v3_dual_arm_cartesian_box_demo', ['target_offset_x:=-0.6']),
        ('dual_asymmetric', 'v3_dual_arm_asymmetric_box_demo', ['target_offset_z:=1.5']),
        ('dual_initial_ik', 'v3_dual_arm_cartesian_box_demo', ['initial_box_x:=5.0']),
        ('single_lift', 'v3_box_wall_grasp_demo', ['x:=0.9', 'box_id:=5', 'arm:=left', f'environment_file:={obstacle}']),
        ('single_return', 'v3_box_wall_comfort_grasp_demo', ['x:=0.5', 'box_id:=7', 'arm:=left', 'suction_mode:=top', 'wall_context:=sequence_prefix', 'planning_seed:=104731', f'environment_file:={return_obstacle}']),
        ('single_ik', 'v3_box_wall_grasp_demo', ['x:=5.0', 'box_id:=20', 'arm:=left']),
    ]
    for label, launch, extra in cases:
        if args.case not in ('all', label):
            continue
        path = root / label
        path.mkdir(exist_ok=True)
        os.environ['ROS_LOG_DIR'] = str(path / 'ros')
        topic = '/v3_dual_arm_cartesian_box_demo' if label.startswith('dual') else '/v3_box_wall_grasp_demo'
        rclpy.init(args=[])
        node = rclpy.create_node('failure_replay_check')
        received = {}
        samples = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, topic + '/task_json', lambda m: received.update(task=json.loads(m.data)), qos)
        node.create_subscription(JointState, topic + '/joint_states', lambda m: samples.append(list(m.position)), 10)
        node.create_subscription(MarkerArray, topic + '/scene_markers', lambda m: received.update(markers=m.markers), qos)
        service = node.create_client(Trigger, topic + '/run_current_target')
        assert not service.wait_for_service(timeout_sec=.5), 'Use an unused ROS domain'
        with (path / 'launch.log').open('w') as log:
            process = subprocess.Popen(['ros2', 'launch', 'alfa_robot_moveit_config', launch + '.launch.py',
                'auto_run_once:=true', 'start_rviz:=false', 'start_rerun:=false'] + extra,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            def wait(predicate, timeout=240):
                end = time.monotonic() + timeout
                while not predicate() and time.monotonic() < end:
                    assert process.poll() is None, f'{label}: launch exited; inspect {path}/launch.log'
                    rclpy.spin_once(node, timeout_sec=.05)
                assert predicate(), f'{label}: timeout; inspect {path}/launch.log'
            try:
                wait(lambda: received.get('task', {}).get('kind') == 'result')
                task = received['task']
                (path / 'result.json').write_text(json.dumps(task, indent=2))
                assert not task['success'], label
                diagnostic = task['diagnostic']
                assert diagnostic['diagnostic_only'] and diagnostic['freeze_at_end']
                assert diagnostic['stage'] == task['failure_stage']
                assert diagnostic['reason'] == task['failure_reason']
                frames = task['diagnostic_frames']
                assert frames and all(f['diagnostic_only'] for f in frames)
                final = frames[-1]['joints']
                wait(lambda: len(samples) >= 8 and all(s == final for s in samples[-8:])
                     and any(m.ns == 'failure_diagnostic' and m.action == m.ADD
                             for m in received.get('markers', [])))
                until = time.monotonic() + .5
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=.05)
                    assert samples[-1] == final
                if label == 'single_lift':
                    assert task['failure_stage'] == 'height_alignment_collision'
                    assert diagnostic['snapshot'] == 'first_rejected_lift_sample'
                    assert len(frames) > 2 and diagnostic['contacts']
                    lift = task['joint_names'].index('updown')
                    assert abs(frames[-1]['joints'][lift] - frames[-2]['joints'][lift]) <= .00501
                if label == 'single_return':
                    assert task['failure_stage'] == 'rear_placement'
                    assert diagnostic['snapshot'] == 'unconnected_rejected_state_not_replayed'
                    assert diagnostic['rejected_contacts'] and not diagnostic['contacts']
                    assert final != diagnostic['rejected_joints']
                    assert frames[-1]['stage'].startswith('FAILED_HOLD')
                    attaches = [i for i, f in enumerate(frames) if f['stage'] == 'attach_box']
                    assert len(attaches) == 1 and attaches[0] > 0
                    i = attaches[0]
                    assert not frames[i-1]['box_attached'] and frames[i]['box_attached']
                    assert frames[i]['joints'] == frames[i-1]['joints'], 'Pickup teleported'
                    assert any(f['stage'] == 'cartesian_retreat' for f in frames[i+1:])
                    assert all('SNAPSHOT' not in f['stage'] for f in frames)
                if label == 'dual_initial_ik':
                    assert task['failure_stage'] == 'initial_grasp'
                    assert not diagnostic['contacts'] and len(frames) == 1
                if label.startswith('single'):
                    assert len(task['frames']) == 1
                    assert set(task['removed_box_ids']) == (set(range(10, 25)) | {5, 6} if label == 'single_return' else set())
                print(f"PASS {label}: {task['failure_stage']}, {len(frames)} diagnostic frames, "
                      f"{len(diagnostic.get('contacts', []))} contacts, final state frozen", flush=True)
            finally:
                stop(process)
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    main()
