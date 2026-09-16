#!/usr/bin/env python3
"""Headless installed-launch check; source Humble and this workspace first."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, InteractiveMarkerFeedback


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts', type=Path, required=True)
    args = parser.parse_args()
    args.artifacts.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('ROS_DOMAIN_ID', '181')
    os.environ['ROS_LOG_DIR'] = str(args.artifacts.resolve() / 'ros')
    rclpy.init()
    node = rclpy.create_node('v3_redundant_ik_startup_check')
    received = {}
    topic = '/v3_redundant_ik_interactive_demo'
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, topic + '/solution_family_json',
                             lambda msg: received.update(family=json.loads(msg.data)), durable)
    node.create_subscription(JointState, '/joint_states',
                             lambda msg: received.update(joints=dict(zip(msg.name, msg.position))), 10)
    node.create_subscription(MarkerArray, topic + '/status_markers',
                             lambda msg: received.update(status=[m.text for m in msg.markers]), durable)
    feedback_publisher = node.create_publisher(InteractiveMarkerFeedback, '/v3_redundant_ik_demo/feedback', 10)
    with (args.artifacts / 'launch.log').open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', 'alfa_robot_moveit_config',
             'v3_redundant_ik_interactive_demo.launch.py',
             'start_rviz:=false', 'start_rerun:=false'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 25
            while time.monotonic() < deadline and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=0.2)
                if 'family' in received and 'joints' in received:
                    break
            (args.artifacts / 'received.json').write_text(json.dumps(received, indent=2))
            assert 'family' in received, f'No IK family; status={received.get("status")}'
            family = received['family']
            assert family['valid_solution_count'] > 0 and family['segment_count'] > 0
            expected = {f'{side}_joint{i}' for side in ('left', 'right') for i in range(1, 8)}
            expected.update(('updown', 'head_joint'))
            assert set(received['joints']) == expected, received['joints']
            assert received['joints']['updown'] == received['joints']['head_joint'] == 0.0
            print('PASS:', {key: family[key] for key in
                            ('valid_solution_count', 'segment_count', 'solve_ms')}, '16 joints')
            # An unreachable drag target must retain the real last pose, not animate fake IK.
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            previous_joints = dict(received['joints'])
            feedback = InteractiveMarkerFeedback()
            feedback.header.frame_id = 'world'
            feedback.client_id = 'failure_replay_check'
            feedback.marker_name = 'target_pose'
            feedback.event_type = InteractiveMarkerFeedback.MOUSE_UP
            feedback.pose.position.x = 5.0
            feedback.pose.position.z = 1.0
            feedback.pose.orientation.w = 1.0
            feedback_publisher.publish(feedback)
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                if received['family']['generation'] > family['generation']:
                    break
            failed = received['family']
            assert failed['valid_solution_count'] == 0
            assert failed['diagnostic']['freeze_at_end'] and not failed['collision_checked']
            deadline = time.monotonic() + .5
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                assert received['joints'] == previous_joints
            (args.artifacts / 'failed_target.json').write_text(json.dumps(received, indent=2))
            print('PASS: unreachable IK target marked, last joint state held; no fictional collision pose')
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
