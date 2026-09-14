#!/usr/bin/env python3
"""Installed continuous-wall acceptance, including real failure/rollback and replay checks."""
import argparse
import json
import os
import re
from pathlib import Path
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters
from alfa_robot_moveit_config.srv import PlanWallBoxDemo
from alfa_robot_rerun.visualize_rerun import UrdfRobot, render_current_urdf
from test_v3_box_wall_grasp_demo import stop

TOPIC = '/v3_box_wall_grasp_demo'
ORDER = [r * 5 + c for r in range(4, -1, -1) for c in range(5)]


def check_result(task, robot):
    assert task['sequence_order'] == ORDER
    n = task['completed_count']
    assert task['success'] == (n == 25)
    assert task['remaining_count'] == 25 - n
    assert set(task['final_removed_box_ids']) == set(ORDER[:n])
    assert task['failed_box_id'] == (-1 if n == 25 else ORDER[n])
    assert len(task['boxes']) == n + (n < 25)
    assert task['environment']['enabled']
    if not task['success']:
        diagnostic = task['diagnostic']
        assert diagnostic['diagnostic_only'] and diagnostic['freeze_at_end']
        replay = task['diagnostic_frames']
        assert replay and all(f['diagnostic_only'] for f in replay)
        context = task['scenes'][replay[-1]['scene_index']]
        assert context['box_id'] == task['failed_box_id']
        assert set(context['removed_box_ids']) == set(task['final_removed_box_ids'])
        assert replay[-1]['box_visible']  # failed box never disappears/commits
        if n:
            assert [f['joints'] for f in replay[:len(task['frames'])]] == [f['joints'] for f in task['frames']]
    previous = task['initial_joints']
    for i, box in enumerate(task['boxes']):
        assert box['box_id'] == ORDER[i]
        assert set(box['removed_box_ids']) == set(ORDER[:i])
        assert len(box['neighbor_centers']) == 24 - i
        expected = [('front', 'left'), ('front', 'right'), ('top', 'left'), ('top', 'right')]
        if box['box_id'] < 5:
            expected = expected[2:]
        actual = [(a['suction_mode'], a['arm']) for a in box['attempts']]
        assert actual == expected[:len(actual)]
        assert all(not a['success'] for a in box['attempts'][:-1])
        assert box['attempts'][-1]['success'] == box['success']
        assert np.allclose(box['initial_joints'], previous)
        frames = task['frames'][box['frame_begin']:box['frame_end']]
        if not box['success']:
            assert not frames and i == n
            assert actual == expected
            break
        releases = [j for j, f in enumerate(frames) if f['stage'] == 'release_box']
        assert len(releases) == 1
        release = releases[0]
        assert frames[release - 1]['stage'] == 'rear_placement'
        assert frames[release - 1]['box_attached'] and frames[release - 1]['box_visible']
        assert all(not f['box_visible'] and not f['box_attached'] for f in frames[release:])
        assert all(f['box_visible'] for f in frames[:release])
        context = task['scenes'][frames[0]['scene_index']]
        assert context['box_id'] == box['box_id']
        offset = np.eye(4)
        offset[:3, :3] = context['tool_to_box_rotation']
        offset[:3, 3] = context['tool_to_box_center']
        attach = next(f for f in frames if f['stage'] == 'attach_box')
        fk = robot.fk(dict(zip(task['joint_names'], attach['joints'])))
        world_box = fk[context['tool_link']] @ offset
        assert np.allclose(world_box[:3, 3], context['box_center'], atol=1e-5)
        assert np.allclose(world_box[:3, :3], np.eye(3), atol=1e-5)
        fk = robot.fk(dict(zip(task['joint_names'], frames[release]['joints'])))
        world_box = fk[context['tool_link']] @ offset
        extent = np.abs(world_box[0, :3]) @ (np.asarray(context['box_size']) / 2)
        assert world_box[0, 3] + extent <= task['chassis_rear_x'] - .01 + 1e-6
        assert frames[-1]['stage'] == 'release_box'
        assert frames[0]['joints'] == previous
        assert frames[release]['joints'] == frames[release-1]['joints']
        previous = frames[-1]['joints']
    assert np.allclose(previous, task['final_joints'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts', type=Path, required=True)
    parser.add_argument('--x', type=float, default=.9)
    parser.add_argument('--initial-pose', choices=['home', 'arms_down'], default='home')
    parser.add_argument('--wall-bottom-z', type=float, default=0.)
    parser.add_argument('--seed', type=int, default=104729)
    parser.add_argument('--front-ratio', type=float)
    parser.add_argument('--validator', type=Path)
    parser.add_argument('--wall-context', choices=['full', 'sequence_prefix'], default='full',
                        help='Sequence must still start with all25 even when this single-box fixture is requested')
    parser.add_argument('--incremental-rerun', action='store_true', help='Record real incremental Rerun writes and timings')
    parser.add_argument('--require-complete', action='store_true')
    parser.add_argument('--environment-file', type=Path, help='Custom collision fixture')
    parser.add_argument('--wait-failure-playback', action='store_true', help='Verify live joints/markers freeze after the entire replay')
    parser.add_argument('--preflight', action='store_true', help='Exercise unreachable single-box fallbacks before sequence')
    args = parser.parse_args()
    root = args.artifacts.resolve()
    root.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('ROS_DOMAIN_ID', '197')
    os.environ['ROS_LOG_DIR'] = str(root / 'ros')
    robot = UrdfRobot(render_current_urdf({'model_ground_offset': '0.402201'}))
    rclpy.init(args=[])
    node = rclpy.create_node('wall_sequence_check')
    received = {}
    segments = {}
    received_at = {}
    def receive_task(message):
        task = json.loads(message.data)
        received['task'] = task
        received_at[task['kind']] = time.perf_counter()
    def receive_segment(message):
        task = json.loads(message.data)
        segments[task['segment_index']] = task
        (root / f"segment_{task['segment_index']:02d}.json").write_text(message.data)
    if args.incremental_rerun:
        node.create_subscription(String, TOPIC + '/task_json_segments', receive_segment,
                                 QoSProfile(depth=32, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    samples = []
    node.create_subscription(JointState, TOPIC + '/joint_states', lambda m: samples.append(list(m.position)), 10)
    node.create_subscription(MarkerArray, TOPIC + '/scene_markers', lambda m: received.update(markers=m.markers), 10)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, TOPIC + '/task_json',
                             receive_task, qos)
    client = node.create_client(Trigger, TOPIC + '/plan_wall_sequence')
    single = node.create_client(PlanWallBoxDemo, TOPIC + '/plan_wall_box')
    assert not client.wait_for_service(timeout_sec=1), 'Use an unused ROS domain'
    command = ['ros2', 'launch', 'alfa_robot_moveit_config', 'v3_box_wall_sequence_demo.launch.py',
               f'x:={args.x}', 'auto_run_once:=false', 'start_rviz:=false', f'start_rerun:={str(args.incremental_rerun).lower()}',
               'spawn_viewer:=false', f'rerun_recording_path:={root / "live.rrd"}',
               f'planning_seed:={args.seed}', f'initial_pose:={args.initial_pose}', f'wall_bottom_z:={args.wall_bottom_z}']
    if args.front_ratio is not None:
        command.extend(f'comfort_ratio_{key}:={args.front_ratio}' for key in ('min', 'preferred', 'max'))
    if args.environment_file:
        command.append(f'environment_file:={args.environment_file.resolve()}')
    command.append(f'wall_context:={args.wall_context}')
    with (root / 'launch.log').open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        def wait(predicate, timeout=1800):
            end = time.monotonic() + timeout
            while not predicate() and time.monotonic() < end:
                assert process.poll() is None, 'Launch exited; inspect launch.log'
                rclpy.spin_once(node, timeout_sec=.05)
            assert predicate(), 'Timeout; inspect launch.log'
        try:
            assert client.wait_for_service(timeout_sec=45)
            parameters = node.create_client(GetParameters, TOPIC + '/get_parameters')
            assert parameters.wait_for_service(timeout_sec=10)
            description = parameters.call_async(GetParameters.Request(names=['robot_description', 'robot_description_semantic']))
            wait(description.done)
            for name, value in zip(('robot.urdf', 'robot.srdf'), description.result().values):
                assert value.string_value
                (root / name).write_text(value.string_value)
            robot = UrdfRobot(description.result().values[0].string_value)
            # An unreachable non-bottom single request must try front on both arms, then top.
            for box_id in ((20, 0) if args.preflight else ()):
                future = single.call_async(PlanWallBoxDemo.Request(x=5., box_id=box_id, arm='auto'))
                wait(future.done)
                response = future.result()
                task = json.loads(response.result_json)
                (root / f'unreachable_{box_id}.json').write_text(json.dumps(task, indent=2))
                assert not task['success'] and len(task['frames']) == 1
                assert task['frames'][0]['joints'] == task['initial_joints']
                expected = ['front', 'front', 'top', 'top'] if box_id == 20 else ['top', 'top']
                assert [a['suction_mode'] for a in task['attempts']] == expected
                assert not task['removed_box_ids']
            request_started = time.perf_counter()
            future = client.call_async(Trigger.Request())
            wait(future.done)
            assert future.result().success, future.result().message
            wait(lambda: received.get('task', {}).get('sequence', False))
            task = received['task']
            (root / 'sequence.json').write_text(json.dumps(task, indent=2))
            check_result(task, robot)
            if args.incremental_rerun:
                wait(lambda: len(segments) == task['segment_count'])
                from alfa_robot_rerun.demo_failure import replay_frames
                canonical = lambda fs: [{k: v for k, v in f.items() if k != 'diagnostic_only'} for f in fs]
                joined = []
                for index, segment in sorted(segments.items()):
                    assert segment['task_id'] == task['task_id']
                    assert segment['frame_begin'] == len(joined)
                    assert segment['scenes'] == task['scenes'][:len(segment['scenes'])]
                    joined.extend(canonical(replay_frames(segment)))
                    assert segment['frame_end'] == len(joined)
                    assert segment['success'] == task['boxes'][index]['success']
                assert joined == canonical(replay_frames(task))
                wait(lambda: 'RERUN_SEQUENCE_READY' in (root / 'launch.log').read_text())
                ready_at = time.perf_counter()
                # Real late transient-local subscription gets the authoritative full result,
                # independently of whether the 32-deep segment history is available.
                late = []
                subscription = node.create_subscription(String, TOPIC + '/task_json',
                    lambda message: late.append(json.loads(message.data)), qos)
                wait(lambda: bool(late), 10)
                assert late[-1] == task
                node.destroy_subscription(subscription)
                text = (root / 'launch.log').read_text()
                write_times = [float(v) for v in re.findall(r'RERUN_TIMELINE_READY[^\n]*write_elapsed_s=([0-9.]+)', text)]
                assert len(write_times) == task['segment_count'], 'missing or duplicate Rerun writes'
                report = dict(first_box_available_s=segments[0]['planning_elapsed_ms'] / 1000,
                    first_segment_write_s=write_times[0], planning_s=task['total_ms'] / 1000,
                    request_to_all_written_s=ready_at-request_started,
                    planning_received_to_all_written_s=ready_at-received_at.get('planning', request_started),
                    segment_write_sum_s=sum(write_times), segments=len(segments), frames=len(joined),
                    segment_equals_final=True, late_ros_snapshot_equals_final=True,
                    completed_count=task['completed_count'], final_stage=joined[-1]['stage'])
                (root / 'incremental_check.json').write_text(json.dumps(report, indent=2) + '\n')
                print('PASS incremental:', json.dumps(report), flush=True)
            if args.validator and task['success']:
                check = subprocess.run([str(args.validator.resolve()), str(root / 'sequence.json'),
                    str(root / 'robot.urdf'), str(root / 'robot.srdf')], capture_output=True, text=True)
                (root / 'independent_check.log').write_text(check.stdout + check.stderr)
                assert check.returncode == 0, check.stdout + check.stderr
            if task['completed_count']:
                # A request during sequence playback must not reset the committed scene.
                rejected = single.call_async(PlanWallBoxDemo.Request(x=.9, box_id=0, arm='auto'))
                wait(rejected.done, 10)
                assert rejected.result().failure_stage == 'busy'
                rejected = client.call_async(Trigger.Request())
                wait(rejected.done, 10)
                assert not rejected.result().success
            print(f"PASS sequence invariants: {task['completed_count']}/25 completed; "
                  f"failed_box={task['failed_box_id']} stage={task['failure_stage']}", flush=True)
            if args.wait_failure_playback and not task['success']:
                final = task['diagnostic_frames'][-1]['joints']
                wait(lambda: len(samples) >= 8 and all(s == final for s in samples[-8:])
                     and any(m.ns == 'failure_diagnostic' and m.action == m.ADD for m in received.get('markers', [])))
                (root / 'freeze_check.json').write_text(json.dumps(dict(joints=final, samples=8)))
                print('PASS complete sequence replay frozen at rejected frame, failed scene retained', flush=True)
            if args.require_complete:
                assert task['success'], task['failure_reason']
        finally:
            stop(process)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
