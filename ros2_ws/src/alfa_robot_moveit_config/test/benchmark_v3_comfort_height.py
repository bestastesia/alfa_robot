#!/usr/bin/env python3
"""Offline installed-launch comparison; simulation only. One fresh process per policy/seed.

Each process replays the full ordered request list: never resume by skipping requests
because OMPL's RNG stream depends on preceding plans. Raw replay JSON is gzip'd.
Source tools/ros_humble_env.sh and ros2_ws/install/setup.bash; use /usr/bin/python3.
"""
import argparse
import gzip
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from alfa_robot_moveit_config.srv import PlanWallBoxDemo
from alfa_robot_rerun.visualize_rerun import UrdfRobot, render_current_urdf


def geometry(robot, side, joints):
    transforms = robot.fk(joints)
    centers = []
    for start in (1, 3, 5):
        lhs, rhs = np.zeros((3, 3)), np.zeros(3)
        for i in range(start, start + 3):
            tf = transforms[f'{side}_joint{i}']
            axis = tf[:3, 2]
            projector = np.eye(3) - np.outer(axis, axis)
            lhs += projector
            rhs += projector @ tf[:3, 3]
        centers.append(np.linalg.solve(lhs, rhs))
    return centers[0], sum(np.linalg.norm(b-a) for a, b in zip(centers, centers[1:]))


def inspect(task, robot, limits):
    """Independent FK/geometry/replay contracts, including failed attempts."""
    assert task['environment']['enabled'] and len(task['environment']['boxes']) == 5
    assert task['collision_inset'] == 0 and task['contact_numerical_gap'] == 1e-6
    names = task['joint_names']
    home = dict(zip(names, [-math.pi/2, -math.pi/2, 0, -math.pi/2, 0, 0, 0]*2+[0, 0]))
    frames = task['frames']
    assert frames and np.allclose(frames[0]['joints'], list(home.values()))
    attempts = task['attempts']
    expected = (['left'] if attempts[0]['success'] else ['left', 'right']) \
        if task['requested_arm'] == 'auto' else [task['requested_arm']]
    assert [a['arm'] for a in attempts] == expected
    assert task['success'] == any(a['success'] for a in attempts)
    assert task['height_alignment'] == attempts[-1]['height_alignment']
    all_joints = np.array([f['joints'] for f in frames])
    for i, name in enumerate(names):
        assert np.all(all_joints[:, i] >= limits[name][0]-1e-6)
        assert np.all(all_joints[:, i] <= limits[name][1]+1e-6)
        if name != 'updown' and not name.startswith(task['side']+'_joint'):
            assert np.allclose(all_joints[:, i], home[name])
    for attempt in attempts:
        assert attempt['height_selections'] == 1
        h = attempt['height_alignment']
        if h['strategy'] != 'comfort_radius':
            continue
        shoulder, length = geometry(robot, attempt['arm'], home)
        target = np.array(task['contact'])
        assert np.allclose(h['shoulder_world'], shoulder, atol=1e-6)
        assert np.allclose(h['target_world'], target, atol=1e-9)
        assert math.isclose(h['arm_length'], length, abs_tol=1e-6)
        xy = np.linalg.norm((target-shoulder)[:2])
        assert math.isclose(xy, h['xy'], abs_tol=1e-6)
        q = h['target_updown']
        assert h['lower_limit'] <= q <= h['upper_limit']
        # Historical artifacts predate clearance bounds; current launches must
        # expose them (checked below at capture time).
        clearance = h.get('reachable_lift', {})
        if clearance.get('checked'):
            lower, upper = clearance['lower'], clearance['upper']
            assert h['lower_limit'] <= lower <= h['initial_updown'] <= upper <= h['upper_limit']
            assert lower <= q <= upper
            for blocked in clearance['blocked']:
                endpoint = clearance[blocked['direction']]
                assert 0 < abs(blocked['updown'] - endpoint) <= .005001
                assert blocked['reason']
            if h['branch_policy'] == 'auto':
                grid = np.linspace(lower, upper, 10001)
                ratios = np.hypot(xy, shoulder[2]+grid-target[2])/length
                gap = lambda r: np.maximum(np.maximum(h['ratio_min']-r, r-h['ratio_max']), 0)
                assert gap(h['actual_ratio']) <= np.min(gap(ratios)) + 1e-6
        rho = math.hypot(xy, shoulder[2]+q-target[2])/length
        assert math.isclose(rho, h['actual_ratio'], abs_tol=1e-6)
        assert h['inside_band'] == (h['ratio_min']-1e-9 <= h['actual_ratio'] <= h['ratio_max']+1e-9)
        if h['inside_band']:
            assert h['ratio_min']-1e-6 <= rho <= h['ratio_max']+1e-6
    h = task['height_alignment']
    lift_index = names.index('updown')
    lift = np.array([f['joints'][lift_index] for f in frames])
    for frame in frames:
        if frame['stage'] in ('rrt_to_precontact', 'cartesian_approach', 'attach_box',
                              'cartesian_retreat', 'rrt_return'):
            assert math.isclose(frame['joints'][lift_index], h['target_updown'], abs_tol=1e-6)
        if frame['stage'] == 'attach_box':
            tf = robot.fk(dict(zip(names, frame['joints'])))[task['tool_link']]
            assert np.allclose(tf[:3, 3], task['contact'], atol=1e-6)
            assert np.allclose(tf[:3, 3]+tf[:3, :3] @ task['tool_to_box_center'],
                               task['box_center'], atol=1e-6)
    for before, after in zip(frames, frames[1:]):
        if after['stage'] in ('move_to_grasp_height', 'lower_to_box_height', 'restore_default_height'):
            assert abs(after['joints'][lift_index]-before['joints'][lift_index]) <= .005001
    if task['success']:
        assert np.allclose(frames[-1]['joints'], list(home.values()))
        assert frames[-1]['box_attached']
        assert {'attach_box', 'cartesian_retreat', 'rrt_return'}.issubset(f['stage'] for f in frames)
    indices = [i for i, name in enumerate(names) if name.startswith(task['side']+'_joint')]
    assert len(indices) == 7, names
    lower, upper = np.array([limits[names[i]] for i in indices]).T
    joints = np.array([f['joints'] for f in frames])[:, indices]
    cart = np.array([f['joints'] for f in frames if f['stage'] in
                    ('cartesian_approach', 'attach_box', 'cartesian_retreat')])
    margin = None if not len(cart) else float(np.min(np.minimum(
        (cart[:, indices]-lower)/(upper-lower), (upper-cart[:, indices])/(upper-lower))))
    shoulder, length = geometry(robot, task['side'], home)
    xy = np.linalg.norm((np.array(task['contact'])-shoulder)[:2])
    actual_ratio = math.hypot(xy, shoulder[2]+h['target_updown']-task['contact'][2])/length
    return dict(margin=margin, motion=float(np.sum(np.abs(np.diff(joints, axis=0))/(upper-lower))),
                lift_travel=float(np.sum(np.abs(np.diff(lift)))), actual_ratio=actual_ratio,
                outside_reason=h.get('outside_reason'), height=h['target_updown'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts', required=True, type=Path)
    parser.add_argument('--policy', choices=['fixed_offset', 'comfort_radius'], default='comfort_radius')
    parser.add_argument('--ratio', type=float, default=.8)
    parser.add_argument('--band', nargs=2, type=float)
    parser.add_argument('--branch', choices=['auto', 'above', 'below'], default='auto')
    parser.add_argument('--seed', type=int, default=104729)
    parser.add_argument('--distances', nargs='+', type=float, default=[.8, .9, 1.])
    parser.add_argument('--arms', nargs='+', choices=['left', 'right', 'auto'], default=['left', 'right'])
    parser.add_argument('--boxes', nargs='+', type=int, default=list(range(25)))
    parser.add_argument('--domain', type=int, default=181)
    args = parser.parse_args()
    root = args.artifacts.resolve()
    root.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=str(args.domain), ROS_LOG_DIR=str(root/'ros'))
    config = vars(args).copy(); config['artifacts'] = str(root)
    (root/'config.json').write_text(json.dumps(config, indent=2))
    urdf = render_current_urdf({'model_ground_offset': '0.402201'})
    robot = UrdfRobot(urdf)
    limits = {j.attrib['name']: (float(j.find('limit').attrib['lower']),
                               float(j.find('limit').attrib['upper']))
              for j in ET.fromstring(urdf).findall('joint') if j.find('limit') is not None}
    lo, hi = args.band or [args.ratio, args.ratio]
    command = ['ros2', 'launch', 'alfa_robot_moveit_config', 'v3_box_wall_comfort_grasp_demo.launch.py',
               'x:=0.9', 'auto_run_once:=false', 'start_rviz:=false', 'start_rerun:=false',
               f'height_strategy:={args.policy}', f'comfort_ratio_min:={lo}',
               f'comfort_ratio_preferred:={args.ratio}', f'comfort_ratio_max:={hi}',
               f'comfort_branch:={args.branch}', f'planning_seed:={args.seed}']
    # Baseline uses the original entry point; wrapper always selects comfort.
    if args.policy == 'fixed_offset':
        command[3] = 'v3_box_wall_grasp_demo.launch.py'
    rclpy.init()
    node = rclpy.create_node('comfort_benchmark')
    client = node.create_client(PlanWallBoxDemo, '/v3_box_wall_grasp_demo/plan_wall_box')
    assert not client.wait_for_service(timeout_sec=1), 'Use an unused ROS domain'
    rows = []
    with (root/'launch.log').open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            assert client.wait_for_service(timeout_sec=60), (root/'launch.log').read_text()[-8000:]
            for x in args.distances:
                for arm in args.arms:
                    for box in args.boxes:
                        future = client.call_async(PlanWallBoxDemo.Request(x=x, box_id=box, arm=arm))
                        deadline = time.monotonic()+240
                        while not future.done() and time.monotonic() < deadline:
                            assert process.poll() is None, 'Planner exited'
                            rclpy.spin_once(node, timeout_sec=.05)
                        assert future.done(), f'timeout x={x} arm={arm} box={box}'
                        response = future.result()
                        task = json.loads(response.result_json)
                        name = f'x{x:.2f}_{arm}_box{box:02d}'
                        with gzip.open(root/f'{name}.json.gz', 'wt') as raw:
                            json.dump(task, raw)
                        assert task['height_alignment']['strategy'] == args.policy
                        assert task['planning_seed'] == args.seed
                        if args.policy == 'comfort_radius':
                            for attempt in task['attempts']:
                                checked = attempt['height_alignment']['reachable_lift']['checked']
                                assert checked == (attempt['failure_stage'] != 'initial_state')
                        assert task['success'] == response.success
                        assert task['failure_stage'] == response.failure_stage
                        row = dict(case=name, x=x, arm=arm, box=box, success=task['success'],
                                   failure_stage=task['failure_stage'], failure_reason=task['failure_reason'],
                                   total_ms=task['total_ms'], selected_arm=task['side'],
                                   **inspect(task, robot, limits))
                        rows.append(row)
                        with (root/'rows.jsonl').open('a') as out:
                            out.write(json.dumps(row)+'\n')
                        print(f'{root.name}: {len(rows)} {name} {row["success"]} {row["failure_stage"]}', flush=True)
            (root/'summary.json').write_text(json.dumps(dict(config=config, count=len(rows),
                success=sum(r['success'] for r in rows), rows=rows), indent=2))
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
