#!/usr/bin/env python3
"""Box7 top pickup in an EXPLICIT sequence-prefix fixture; full collision/replay check.

Not evidence that x=.90/full-wall is reachable or that the preceding17 cycles ran.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time
from unittest.mock import patch

import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from alfa_robot_moveit_config.srv import PlanWallBoxDemo
from alfa_robot_rerun.visualize_rerun import UrdfRobot
from alfa_robot_rerun import v3_single_arm_box_extract_viewer as viewer_module
from test_v3_box_wall_grasp_demo import stop


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts', required=True, type=Path)
    parser.add_argument('--validator', required=True, type=Path)
    parser.add_argument('--arm', choices=['left', 'right', 'auto'], default='left')
    parser.add_argument('--seed', type=int, default=104729)
    parser.add_argument('--box-id', type=int, choices=range(25), default=7)
    parser.add_argument('--suction-mode', choices=['auto', 'top'], default='top')
    parser.add_argument('--wall-bottom-z', type=float, default=0.)
    parser.add_argument('--front-ratio', type=float)

    args = parser.parse_args()
    root = args.artifacts.resolve()
    root.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('ROS_DOMAIN_ID', '197')
    os.environ['ROS_LOG_DIR'] = str(root / 'ros')
    rclpy.init(args=[])
    node = rclpy.create_node('box7_continuous_replay_check')
    topic = '/v3_box_wall_grasp_demo'
    client = node.create_client(PlanWallBoxDemo, topic + '/plan_wall_box')
    assert not client.wait_for_service(timeout_sec=1), 'Use an unused ROS domain'
    received, joints, boxes = {}, [], []
    node.create_subscription(String, topic + '/task_json',
        lambda m: received.update(task=json.loads(m.data)),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(JointState, topic + '/joint_states',
        lambda m: joints.append(list(m.position)), 100)
    def markers(message):
        received["markers"] = message.markers
        targets = [m for m in message.markers if m.ns == 'target_box' and m.action == m.ADD]
        if targets:
            p = targets[0].pose.position
            boxes.append([p.x, p.y, p.z])
    node.create_subscription(MarkerArray, topic + '/scene_markers', markers, 100)
    command = ['ros2', 'launch', 'alfa_robot_moveit_config',
        'v3_box_wall_comfort_grasp_demo.launch.py', 'x:=0.50', f'box_id:={args.box_id}',
        f'suction_mode:={args.suction_mode}', 'wall_context:=sequence_prefix', f'arm:={args.arm}',
        f'planning_seed:={args.seed}', 'start_rviz:=false', 'start_rerun:=false', f'wall_bottom_z:={args.wall_bottom_z}']
    if args.front_ratio is not None:
        command.extend(f'comfort_ratio_{key}:={args.front_ratio}' for key in ('min', 'preferred', 'max'))
    with (root / 'launch.log').open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        def wait(predicate, timeout=240):
            end = time.monotonic() + timeout
            while not predicate() and time.monotonic() < end:
                assert process.poll() is None, 'Launch exited; inspect launch.log'
                rclpy.spin_once(node, timeout_sec=.02)
            assert predicate(), 'Timeout; inspect launch.log'
        try:
            parameters = node.create_client(GetParameters, topic + '/get_parameters')
            wait(parameters.service_is_ready, 45)
            future = parameters.call_async(GetParameters.Request(
                names=['robot_description', 'robot_description_semantic']))
            wait(future.done)
            descriptions = [v.string_value for v in future.result().values]
            for name, text in zip(('robot.urdf', 'robot.srdf'), descriptions):
                assert text, 'Missing installed model parameter'
                (root / name).write_text(text)
            robot = UrdfRobot(descriptions[0])
            wait(lambda: received.get('task', {}).get('kind') == 'result')
            task = received['task']
            (root / 'result.json').write_text(json.dumps(task, indent=2))
            assert task['success'], task['attempts']
            assert task['wall_context'] == 'sequence_prefix' and task['x'] == .5
            order = [r * 5 + c for r in range(4, -1, -1) for c in range(5)]
            prior = set(order[:order.index(args.box_id)])
            remaining = set(range(25)) - prior - {args.box_id}
            assert set(task['removed_box_ids']) == prior
            assert len(task['neighbor_centers']) == len(remaining) and len(task['environment']['boxes']) == 5
            assert task['collision_inset'] == 0 and task['release_after_transfer']
            assert not task['diagnostic_frames']
            frames = task['frames']
            assert any(f['stage'] == 'cartesian_retreat' and f['box_attached'] for f in frames)
            assert frames[-1]['stage'] == 'release_box' and not frames[-1]['box_visible']
            if task['suction_mode'] == 'top':
                assert task['height_alignment']['shoulder_above_wrist'] == .1
                lift_frames = [f for f in frames if f['stage'] == 'cartesian_lift']
                assert len(lift_frames) == 5
                last_lift = robot.fk(dict(zip(task['joint_names'], lift_frames[-1]['joints'])))[task['tool_link']]
                lifted_center = last_lift[:3, :3] @ task['tool_to_box_center'] + last_lift[:3, 3]
                assert np.allclose(lifted_center, np.asarray(task['box_center']) + [-.02, 0, .05], atol=1e-6)
            final = frames[-1]['joints']
            tf = robot.fk(dict(zip(task['joint_names'], final)))[task['tool_link']]
            final_center = tf[:3, :3] @ task['tool_to_box_center'] + tf[:3, 3]
            wait(lambda: len(joints) >= 8 and all(q == final for q in joints[-8:])
                 and boxes and not any(m.ns == 'target_box' and m.action == m.ADD for m in received.get('markers', [])),
                 max(240, len(frames) * .1))
            # Prefix boxes must disappear from RViz as well as the collision scene.
            scene_markers = received['markers']
            assert any(m.action == m.DELETEALL for m in scene_markers), 'Old wall markers may remain'
            neighbors = [m for m in scene_markers if m.ns == 'neighbor_boxes' and m.action == m.ADD]
            assert len(neighbors) == len(remaining)
            assert np.allclose([[m.pose.position.x, m.pose.position.y, m.pose.position.z]
                                for m in neighbors], task['neighbor_centers'], atol=1e-6)
            assert {m.id for m in scene_markers if m.ns == 'wall_box_ids' and m.action == m.ADD} == remaining
            # Check actual ROS playback against planned frames, not just the returned JSON.
            planned = {tuple(f['joints']) for f in frames}
            observed = {tuple(q) for q in joints}
            assert observed <= planned, 'Publisher emitted an unplanned posture'
            assert len(observed) >= .9 * len(planned), 'Most of the trajectory was not observed'
            expected = []
            for f in frames:
                tf = robot.fk(dict(zip(task['joint_names'], f['joints'])))[task['tool_link']]
                expected.append(tf[:3, :3] @ task['tool_to_box_center'] + tf[:3, 3]
                                if f['box_attached'] else task['box_center'])
            expected = np.asarray(expected)
            index = 0
            for center in boxes:
                matches = np.flatnonzero(np.linalg.norm(expected[index:] - center, axis=1) < 1e-6)
                assert len(matches), 'Box marker left the continuous planned trajectory'
                index += int(matches[0])
            check = subprocess.run([str(args.validator.resolve()), str(root / 'result.json'),
                str(root / 'robot.urdf'), str(root / 'robot.srdf')], capture_output=True, text=True)
            (root / 'independent_check.log').write_text(check.stdout + check.stderr)
            assert check.returncode == 0, check.stdout + check.stderr
            # Exercise the real Rerun box-transform path too, without opening a user window.
            viewer = viewer_module.V3SingleArmBoxExtractViewer.__new__(viewer_module.V3SingleArmBoxExtractViewer)
            viewer.robot, viewer.tool_link = robot, task['tool_link']
            for key in ('box_center', 'box_size', 'tool_to_box_center', 'tool_to_box_rotation'):
                setattr(viewer, key, np.asarray(task[key]))
            viewer.neighbor_centers = task['neighbor_centers']
            drawn = {}
            with patch.object(viewer_module.rr, 'log', side_effect=lambda path, value: drawn.update({path: value})), \
                    patch.object(viewer_module.rr, 'Clear', return_value='clear'), \
                    patch.object(viewer_module.rr, 'Boxes3D', side_effect=lambda **kwargs: kwargs):
                for frame, center in zip(frames, expected):
                    viewer.log_boxes(dict(zip(task['joint_names'], frame['joints'])), attached=frame['box_attached'], visible=frame['box_visible'])
                    if not frame['box_visible']:
                        assert drawn['world/boxes/target'] == drawn['world/boxes/carried'] == 'clear'
                        continue
                    key = 'carried' if frame['box_attached'] else 'target'
                    assert np.allclose(drawn['world/boxes/' + key]['centers'][0], center, atol=1e-6)
                    assert drawn['world/boxes/' + ('target' if frame['box_attached'] else 'carried')] == 'clear'
            # Prove the independent checker rejects the original teleport symptom and real collisions.
            for label in ('teleport', 'collision'):
                invalid = json.loads(json.dumps(task))
                if label == 'teleport':
                    next(f for f in invalid['frames'] if f['stage'] == 'attach_box')['joints'] = task['initial_joints']
                else:
                    invalid['environment']['boxes'].append(dict(id='injected_block', center=[0, 0, 1], size=[10, 10, 10]))
                path = root / f'rejected_{label}.json'
                path.write_text(json.dumps(invalid))
                rejected = subprocess.run([str(args.validator.resolve()), str(path),
                    str(root / 'robot.urdf'), str(root / 'robot.srdf')], capture_output=True, text=True)
                (root / f'rejected_{label}.log').write_text(rejected.stdout + rejected.stderr)
                assert rejected.returncode != 0, f'Validator accepted {label}'
            summary = dict(frames=len(frames), distinct_joint_samples=len(observed),
                box_marker_samples=len(boxes), final_box_center=final_center.tolist(),
                fixture=f'x=.50, sequence_prefix before{args.box_id}, prior cycles NOT tested by this single-box check')
            (root / 'replay_check.json').write_text(json.dumps(summary, indent=2))
            print('PASS live ROS replay:', summary)
            print(check.stdout)
        finally:
            stop(process)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
