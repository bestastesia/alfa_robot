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
    rows = [[(r * 5 + 4, r * 5), (r * 5 + 3, r * 5 + 1), (r * 5 + 2,)]
            for r in range(4, -1, -1)]
    rounds = [group for row in rows for group in row]
    assert task['sequence_order'] == ORDER
    assert task['dual_target_count'] == 10
    assert task['segment_count'] == len(task['boxes'])
    assert task['environment']['enabled']

    completed = [box_id for box in task['boxes'] if box['success'] for box_id in box['box_ids']]
    assert task['completed_count'] == len(completed)
    assert task['remaining_count'] == 25 - len(completed)
    assert set(task['final_removed_box_ids']) == set(completed)
    assert task['success'] == (len(completed) == 25)
    assert task['failed_box_id'] == -1 if task['success'] else task['failed_box_id'] >= 0
    assert task['dual_success_count'] == sum(box['success'] and box['dual'] for box in task['boxes'])
    assert task['full_dual_pass'] == (
        task['dual_success_count'] == task['dual_target_count'] and task['fallback_count'] == 0)

    if not task['success']:
        assert task['failed_box_id'] not in completed
        diagnostic = task['diagnostic']
        assert diagnostic['diagnostic_only'] and diagnostic['freeze_at_end']
        replay = task['diagnostic_frames']
        assert replay and all(f['diagnostic_only'] for f in replay)
        context = task['scenes'][replay[-1]['scene_index']]
        assert task['failed_box_id'] in context['box_ids']
        assert set(context['removed_box_ids']) == set(completed)
        assert replay[-1]['box_visible']

    previous = task['initial_joints']
    removed = []
    for i, box in enumerate(task['boxes']):
        assert tuple(box['box_ids']) == rounds[i]
        assert box['dual'] == (len(box['box_ids']) == 2)
        assert set(box['removed_box_ids']) == set(removed)
        assert len(box['neighbor_centers']) == 24 - len(removed)
        assert np.allclose(box['initial_joints'], previous)
        frames = task['frames'][box['frame_begin']:box['frame_end']]
        if not box['success']:
            assert not frames
            break
        assert frames and frames[0]['joints'] == previous
        assert all(frame['scene_index'] == i for frame in frames)
        assert box['requested_trajectory_variant'] == task['requested_trajectory_variant']
        assert box['top_k_complete'] == task['top_k_complete']
        assert 1 <= box['complete_candidate_count'] <= task['top_k_complete']
        assert 1 <= box['selected_candidate_rank'] <= box['complete_candidate_count']
        assert box['selection_score'] >= 0

        times = [frame['time_from_start_s'] for frame in frames]
        assert times[0] == 0
        assert all(a <= b for a, b in zip(times, times[1:]))

        requested = box['requested_trajectory_variant']
        effective = box['effective_trajectory_variant']
        if requested == 'topk':
            assert effective == 'topk'
            assert not box['timing_valid']
        else:
            assert requested in ('shortcut_ruckig', 'chomp_ruckig')
            assert effective == 'shortcut_ruckig' or requested == effective
            assert box['timing_valid']
            assert np.isclose(box['execution_duration_s'], times[-1])

        if box['dual']:
            carried_ids = {item['box_id'] for item in frames[0]['carried_boxes']}
            assert carried_ids == set(box['box_ids'])
            attached = [j for j, frame in enumerate(frames)
                        if frame['carried_boxes'] and all(item['attached'] for item in frame['carried_boxes'])]
            released = [j for j, frame in enumerate(frames)
                        if j > attached[0] and all(not item['visible'] for item in frame['carried_boxes'])]
            assert attached and released
            attach, release = attached[0], released[0]
            assert all(item['visible'] for item in frames[attach]['carried_boxes'])
            assert all(item['attached'] for item in frames[release - 1]['carried_boxes'])
            assert all(not item['attached'] and not item['visible']
                       for item in frames[release]['carried_boxes'])
            carried_boxes = frames[attach]['carried_boxes']
        else:
            attach = next(j for j, frame in enumerate(frames) if frame['stage'] == 'attach_box')
            release = next(j for j, frame in enumerate(frames) if frame['stage'] == 'release_box')
            assert frames[release - 1]['stage'] == 'rear_placement'
            assert frames[release - 1]['box_attached'] and frames[release - 1]['box_visible']
            assert not frames[release]['box_attached'] and not frames[release]['box_visible']
            context = task['scenes'][frames[0]['scene_index']]
            carried_boxes = [{
                'box_center': context['box_center'],
                'box_id': context['box_id'],
                'tool_link': context['tool_link'],
                'tool_to_box_center': context['tool_to_box_center'],
                'tool_to_box_rotation': context['tool_to_box_rotation'],
            }]
        assert frames[release]['joints'] == frames[release - 1]['joints']

        attach_fk = robot.fk(dict(zip(task['joint_names'], frames[attach]['joints'])))
        release_fk = robot.fk(dict(zip(task['joint_names'], frames[release]['joints'])))
        for carried in carried_boxes:
            offset = np.eye(4)
            offset[:3, :3] = carried['tool_to_box_rotation']
            offset[:3, 3] = carried['tool_to_box_center']
            world_box = attach_fk[carried['tool_link']] @ offset
            assert np.allclose(world_box[:3, 3], carried['box_center'], atol=1e-5)
            assert np.allclose(world_box[:3, :3], np.eye(3), atol=1e-5)
            world_box = release_fk[carried['tool_link']] @ offset
            extent = np.abs(world_box[0, :3]) @ (np.asarray(box['box_size']) / 2)
            assert world_box[0, 3] + extent <= task['chassis_rear_x'] - .01 + 1e-6

        removed.extend(box['box_ids'])
        previous = frames[-1]['joints']
    assert set(removed) == set(completed)
    assert np.allclose(previous, task['final_joints'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts', type=Path, required=True)
    parser.add_argument('--x', type=float, default=.5)
    parser.add_argument('--initial-pose', choices=['home', 'arms_down'], default='home')
    parser.add_argument('--wall-bottom-z', type=float, default=0.)
    parser.add_argument('--seed', type=int, default=104729)
    parser.add_argument('--trajectory-variant', choices=['topk', 'shortcut_ruckig', 'chomp_ruckig'], default='topk')
    parser.add_argument('--top-k-complete', type=int, choices=range(1, 9), default=3)
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
    robot = UrdfRobot(render_current_urdf({'model_ground_offset': '0.000005'}))
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
               f'planning_seed:={args.seed}', f'trajectory_variant:={args.trajectory_variant}',
               f'top_k_complete:={args.top_k_complete}', f'initial_pose:={args.initial_pose}',
               f'wall_bottom_z:={args.wall_bottom_z}']
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
            assert task['requested_trajectory_variant'] == args.trajectory_variant, \
                'sequence requested_trajectory_variant does not match --trajectory-variant'
            assert task['planning_seed'] == args.seed, \
                'sequence planning_seed does not match --seed'
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
                rejected = single.call_async(PlanWallBoxDemo.Request(x=args.x, box_id=0, arm='auto'))
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
                assert task['segment_count'] == 15
                assert task['dual_success_count'] == 10
                assert task['fallback_count'] == 0
                assert task.get('full_dual_pass') is True
        finally:
            stop(process)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
