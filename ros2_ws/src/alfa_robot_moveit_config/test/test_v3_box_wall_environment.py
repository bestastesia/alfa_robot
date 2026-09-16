#!/usr/bin/env python3
"""Real installed-launch environment regressions. Source Humble/install; use an unused ROS domain."""
import argparse
from contextlib import contextmanager
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import struct
import xml.etree.ElementTree as ET
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from alfa_robot_moveit_config.srv import PlanWallBoxDemo
from alfa_robot_rerun.visualize_rerun import UrdfRobot, render_current_urdf, package_uri_to_path, transform_from_origin
from test_v3_box_wall_grasp_demo import stop

TOPIC = '/v3_box_wall_grasp_demo'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts', type=Path, required=True)
    parser.add_argument('--scan-wall', action='store_true',
                        help='Also scan all 25 IDs in both default and grounded walls at x=0.30m')
    args = parser.parse_args()
    root = args.artifacts.resolve()
    root.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('ROS_DOMAIN_ID', '189')
    os.environ['ROS_LOG_DIR'] = str(root / 'ros')
    config = json.loads((Path(get_package_share_directory('alfa_robot_moveit_config')) /
                         'config/v3_box_wall_environment.json').read_text())
    urdf = render_current_urdf({'model_ground_offset': '0.000005'})
    robot = UrdfRobot(urdf)
    joint_names = [f'{side}_joint{index}' for side in ('left', 'right') for index in range(1, 8)]
    joint_names += ['updown', 'head_joint']
    initial = yaml.safe_load((Path(get_package_share_directory('alfa_robot_moveit_config')) /
                              'config/initial_positions.yaml').read_text())['initial_positions']
    home = [initial[name] for name in joint_names]
    command = ['ros2', 'launch', 'alfa_robot_moveit_config', 'v3_box_wall_grasp_demo.launch.py',
               'x:=0.5', 'auto_run_once:=false', 'start_rviz:=false', 'start_rerun:=false']
    summary = []

    @contextmanager
    def launch(label, custom=None, extra=()):
        args = list(extra)
        if custom is not None:
            path = root / f'{label}_environment.json'
            path.write_text(json.dumps(custom))
            args += [f'environment_file:={path}']
        rclpy.init()
        node = rclpy.create_node('environment_check')
        received = {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, TOPIC + '/task_json',
                                 lambda m: received.update(task=json.loads(m.data)), qos)
        node.create_subscription(MarkerArray, TOPIC + '/scene_markers',
                                 lambda m: received.update(markers=m.markers), qos)
        client = node.create_client(PlanWallBoxDemo, TOPIC + '/plan_wall_box')
        assert not client.wait_for_service(timeout_sec=1), 'Use an unused ROS domain'
        with (root / f'{label}.log').open('w') as log:
            process = subprocess.Popen(command + args, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)

            def spin_until(predicate, timeout=180):
                end = time.monotonic() + timeout
                while not predicate() and time.monotonic() < end:
                    assert process.poll() is None, (root / f'{label}.log').read_text()
                    rclpy.spin_once(node, timeout_sec=.05)
                assert predicate(), f'{label}: timeout'

            def call(box_id, arm='auto', x=.5):
                future = client.call_async(PlanWallBoxDemo.Request(x=x, box_id=box_id, arm=arm))
                spin_until(future.done)
                response = future.result()
                task = json.loads(response.result_json)
                assert task['success'] == response.success
                assert task['failure_stage'] == response.failure_stage
                assert task['failure_reason'] == response.failure_reason
                assert task['environment']['enabled']
                expected = deepcopy(custom if custom is not None else config)
                if expected.get('anchor') == 'box_wall_back':
                    for box in expected['boxes']:
                        box['center'][0] += task['chassis_front_x'] + x + task['box_size'][0] + task['contact_numerical_gap']
                        box['center'][1] += task['wall_center_y']
                assert task['environment']['boxes'] == [dict(b, id='environment_' + b['id'])
                                                        for b in expected['boxes']]
                path = root / f'{label}_{response.generation}_box{box_id}_{arm}.json'
                path.write_text(json.dumps(task, indent=2))
                summary.append(dict(case=path.stem, scene=label, box_id=box_id,
                                    selected_arm=response.selected_arm, success=response.success,
                                    stage=response.failure_stage, reason=response.failure_reason,
                                    attempts=task['attempts'], total_ms=task['total_ms']))
                assert task['box_id'] == box_id and task['x'] == x
                assert all(a['failure_stage'] != 'exception' for a in task['attempts'])
                assert np.allclose(task['initial_joints'], home, atol=1e-8)
                if not response.success:
                    assert response.failure_stage and response.failure_reason
                    if response.failure_stage == 'initial_state':
                        assert task['diagnostic']['contacts'], 'Real initial collision must remain visible'
                        assert all(np.allclose(f['joints'], home) and not f['box_attached']
                                   for f in task['diagnostic_frames'])
                    if arm == 'auto':
                        assert [a['arm'] for a in task['attempts']] == ['left', 'right'] * (1 if box_id < 5 else 2)
                else:
                    assert response.selected_arm == task['side']
                    assert task['frames'][-1]['stage'] == 'release_box'
                    assert np.allclose(task['frames'][0]['joints'], home, atol=1e-8)
                    assert {'rrt_to_precontact', 'cartesian_approach', 'attach_box',
                            'cartesian_retreat', 'rrt_return', 'rear_placement',
                            'release_box'} <= {f['stage'] for f in task['frames']}
                    if task['height_alignment']['descent'] > 0:
                        assert task['frames'][0]['stage'] == 'lower_to_box_height'
                    last_lift = 0.
                    idle = 'right' if task['side'] == 'left' else 'left'
                    idle_offset = 0 if idle == 'left' else 7
                    for frame in task['frames']:
                        joints = dict(zip(task['joint_names'], frame['joints']))
                        assert np.allclose([joints[f'{idle}_joint{i}'] for i in range(1, 8)],
                                           home[idle_offset:idle_offset + 7])
                        assert joints['head_joint'] == 0
                        assert abs(joints['updown'] - last_lift) <= .005001
                        last_lift = joints['updown']
                        if frame['stage'] == 'attach_box':
                            tool = robot.fk(joints)[task['tool_link']]
                            center = tool[:3, 3] + tool[:3, :3] @ task['tool_to_box_center']
                            assert np.allclose(center, task['box_center'], atol=1e-6)
                    assert not task['frames'][-1]['box_attached']
                    # Independent full FK checks payload-floor clearance in every successful replay.
                    ground = next(b for b in expected['boxes'] if b['id'] == 'ground')
                    floor = ground['center'][2] + ground['size'][2] / 2
                    for frame in task['frames']:
                        if not frame['box_attached']:
                            continue
                        tool = robot.fk(dict(zip(task['joint_names'], frame['joints'])))[task['tool_link']]
                        center = tool[:3, 3] + tool[:3, :3] @ task['tool_to_box_center']
                        half = np.array(task['box_size'])[[2, 1, 0]] / 2
                        extent = np.abs(tool[:3, :3]) @ half
                        assert center[2] - extent[2] > floor
                print(summary[-1], flush=True)
                return task

            try:
                assert client.wait_for_service(timeout_sec=45)
                spin_until(lambda: 'task' in received and 'markers' in received)
                yield call, received
            finally:
                stop(process)
                node.destroy_node()
                rclpy.shutdown()

    with launch('default') as (call, received):
        preview = received['task']
        assert np.allclose(preview['initial_joints'], home, atol=1e-8)
        assert preview['wall_bottom_z'] == 0
        env = {b['id'].removeprefix('environment_'): b for b in preview['environment']['boxes']}
        assert set(env) == {'ground', 'left_wall', 'right_wall', 'front_wall', 'ceiling'}
        def face(name, axis, sign):
            b = env[name]
            return b['center'][axis] + sign * b['size'][axis] / 2
        assert np.isclose(face('ground', 2, 1), 0)
        assert np.isclose(face('ceiling', 2, -1), 2.35)
        assert np.isclose(face('right_wall', 1, -1) - face('left_wall', 1, 1), 2.38)
        assert np.isclose(face('front_wall', 0, -1) - face('left_wall', 0, -1), 4.)
        back = preview['box_center'][0] + preview['box_size'][0] / 2
        assert np.isclose(face('front_wall', 0, -1) - back, preview['contact_numerical_gap'], atol=1e-9)
        assert env['front_wall']['center'][1] == preview['wall_center_y'] == 0
        # Independent imported collision-mesh measurement, not a guessed base_link height.
        fk = robot.fk(dict(zip(preview['joint_names'], home)))
        bounds = {}
        for link in ET.fromstring(urdf).findall('link'):
            points = []
            for collision in link.findall('collision'):
                mesh = collision.find('geometry/mesh')
                assert mesh is not None, 'Update this check for new primitive robot geometry'
                data = Path(package_uri_to_path(mesh.get('filename'))).read_bytes()
                count = struct.unpack_from('<I', data, 80)[0]
                assert len(data) == 84 + 50 * count, 'Expected binary STL'
                vertices = np.ndarray((count, 3, 3), '<f4', buffer=data, offset=96,
                                      strides=(50, 12, 4)).reshape(-1, 3)
                vertices = vertices * np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
                transform = fk[link.get('name')] @ transform_from_origin(collision.find('origin'))
                points.append(vertices @ transform[:3, :3].T + transform[:3, 3])
            if not points:
                continue
            points = np.concatenate(points)
            low, high = points.min(axis=0), points.max(axis=0)
            assert low[0] > face('left_wall', 0, -1) and high[0] < face('front_wall', 0, -1)
            assert low[1] > -1.19 and high[1] < 1.19
            assert low[2] >= 0 and high[2] < 2.35
            bounds[link.get('name')] = [low.tolist(), high.tolist()]
        chassis_links = ('model_base', 'chassis_base', 'active_suspension_carriage',
                         'caster01', 'caster02', 'caster03', 'caster04',
                         'wheel01', 'wheel02', 'wheel03', 'wheel04')
        chassis_ground_z = min(bounds[name][0][2] for name in chassis_links)
        assert 0 <= chassis_ground_z <= 1e-5
        assert np.allclose(fk['world'][:3, 3], [0, 0, 0])
        assert np.allclose(fk['base_footprint'][:3, 3], [0, 0, 5e-6])
        assert np.allclose(fk['base_link'][:3, 3], [.195, .015, .400005])
        (root / 'home_collision_mesh_bounds.json').write_text(json.dumps(bounds, indent=2))
        assert np.isclose(min(c[2] for c in preview['neighbor_centers']) - .2, 0)
        env_markers = [m for m in received['markers'] if m.ns == 'environment']
        assert len(env_markers) == 5
        for marker, box in zip(env_markers, preview['environment']['boxes']):
            assert marker.type == marker.CUBE
            assert np.allclose([marker.pose.position.x, marker.pose.position.y,
                                marker.pose.position.z], box['center'])
            assert np.allclose([marker.scale.x, marker.scale.y, marker.scale.z], box['size'])
        # Complete loaded transfers on both arms; the inactive arm stays at documented home.
        known_success = (5, 9, 10, 14, 20, 24)
        for box_id in range(25) if args.scan_wall else (0, *known_success):
            task = call(box_id)
            if box_id in known_success:
                assert task['success'], task['attempts']
            if box_id < 5:
                assert task['failure_stage'] == 'precontact_ik'
                assert task['suction_mode'] == 'top'
                assert task['height_alignment']['strategy'] == 'top_wrist_alignment'
                assert any(a['height_alignment']['xy'] > a['height_alignment']['arm_length']
                           for a in task['attempts'])
                assert not any(f['box_attached'] for f in task['diagnostic_frames'])
                assert np.allclose(task['frames'][0]['joints'], home)
        for box_id, arm in [(5, 'right'), (9, 'left')]:
            assert call(box_id, arm)['success']
        # Independent requests: changing distance reanchors all warehouse objects, not just boxes.
        task = call(20, x=1.)
        assert task['success'], task['attempts']
        task = call(5)  # high -> low -> full default height; not an accumulated lift offset
        assert task['success']
        task = call(5, x=.3)
        assert task['failure_stage'] == 'initial_state'  # home extends farther than the old zero pose
        task = call(5, x=.75)
        assert task['failure_stage'] == 'return_goal'  # loaded home collides with remaining wall
        task = call(5, x=2.)
        assert task['failure_stage'] == 'precontact_ik'

    # Custom files remain fixed world coordinates unless explicitly anchored.
    world = deepcopy(config)
    world.pop('anchor')
    world['boxes'] = [dict(b, id=b['id'].removeprefix('environment_'))
                      for b in preview['environment']['boxes']]
    def obstruct(id, center, size):
        custom = deepcopy(world)
        custom['boxes'].append(dict(id=id, center=center, size=size))
        return custom

    # Each of the five warehouse solids really participates in full-robot collision checks.
    for id, axis, value in [('ground', 2, -.03), ('left_wall', 1, -.60755),
                            ('right_wall', 1, .60755), ('front_wall', 0, .75),
                            ('ceiling', 2, 1.75)]:
        custom = deepcopy(world)
        next(b for b in custom['boxes'] if b['id'] == id)['center'][axis] = value
        with launch('intrude_' + id, custom) as (call, _):
            task = call(5)
            assert task['failure_stage'] == 'initial_state', task['attempts']
            assert 'environment_' + id in task['failure_reason']
            assert len(task['frames']) == 1 and np.allclose(task['frames'][0]['joints'], home)

    for label, center, size, stage, collision in (
        ('idle_arm', [.7565, .60755, 1.255875], [.04, .04, .04], 'initial_state', 'right_'),
        ('lift_midpath', [.7565, .60755, .8], [.04, .04, .04], 'height_alignment_collision', 'right_'),
        ('loaded_return', [1.16, -.60755, .37124], [.02, .02, .02], 'return_goal', 'carried_target_box'),
        ('restore_height', [1.16, -.60755, .8], [.02, .02, .02], 'return_lift_collision', 'carried_target_box'),
        ('loaded_retreat', [1.02, -1.019, .61], [.08, .001, .02], 'cartesian_retreat', 'carried_target_box'),
    ):
        with launch(label, obstruct(label, center, size)) as (call, _):
            task = call(5, 'left')
            front = task['attempts'][0]  # A later top fallback has its own failure stage.
            assert front['suction_mode'] == 'front' and front['failure_stage'] == stage, task['attempts']
            assert 'environment_' + label in front['failure_reason'] and collision in front['failure_reason']

    comfort = ('height_strategy:=comfort_radius', 'comfort_ratio_min:=1.10',
               'comfort_ratio_preferred:=1.15', 'comfort_ratio_max:=1.15', 'planning_seed:=104729')
    with launch('comfort_clearance', extra=comfort) as (call, received):
        assert not received['task']['height_alignment']['reachable_lift']['checked']
        # Alternate box/arm in one process: no stale clearance or selected height.
        for arm in ('left', 'right', 'auto'):
            for box in (6, 7, 6):
                task = call(box, arm)
                assert task['height_alignment'] == next(a['height_alignment'] for a in task['attempts']
                    if a['arm'] == task['side'] and a['suction_mode'] == task['suction_mode'])
                for attempt in task['attempts']:
                    h = attempt['height_alignment']
                    c = h['reachable_lift']
                    if attempt['suction_mode'] == 'top':
                        assert h['strategy'] == 'top_wrist_alignment'
                        assert h['xy'] > h['arm_length'] and not c['checked']
                        assert attempt['failure_stage'] == 'precontact_ik'
                        continue  # Proved XY-unreachable before any descent/preparation.
                    assert c['checked'] and np.isclose(c['lower'], -.990)
                    assert c['lower'] <= h['target_updown'] <= c['upper']
                    assert attempt['height_selections'] == 1
                    assert attempt['failure_stage'] not in ('initial_state', 'height_alignment_collision')
                    assert 'environment_ground' in c['blocked'][0]['reason']
                    if box == 7:
                        assert np.isclose(h['target_updown'], c['lower'])
                    elif attempt['arm'] == 'left':
                        assert np.isclose(h['target_updown'], -.682676949)
                if box == 6 and arm in ('left', 'auto'):
                    assert task['success'], task['attempts']

    for label, center in (('comfort_initial', [.7565, .60755, 1.255875]),
                          ('comfort_midpath', [.7565, .60755, .8])):
        with launch(label, obstruct(label, center, [.04, .04, .04]), comfort) as (call, _):
            task = call(7, 'left')
            h = task['height_alignment']
            c = h['reachable_lift']
            if label == 'comfort_initial':
                assert task['failure_stage'] == 'initial_state' and not c['checked']
                assert len(task['frames']) == 1
            else:
                assert c['checked'] and c['lower'] > -.990
                assert c['lower'] <= h['target_updown'] <= c['upper']
                assert 'environment_' + label in c['blocked'][0]['reason']
                assert task['failure_stage'] not in ('initial_state', 'height_alignment_collision')
                lift = task['joint_names'].index('updown')
                assert all(f['joints'][lift] >= c['lower']-1e-9 for f in task['frames'])

    invalid = []
    for label, change in (
        ('missing_ground', lambda c: c['boxes'].pop(0)),
        ('negative_size', lambda c: c['boxes'][0]['size'].__setitem__(0, -1)),
        ('wrong_frame', lambda c: c.update(frame_id='map')),
        ('wrong_anchor', lambda c: c.update(anchor='robot')),
        ('duplicate_id', lambda c: c['boxes'].append(c['boxes'][0])),
        ('rotation', lambda c: c['boxes'][0].update(yaw=1)),
        ('infinite_center', lambda c: c['boxes'][0]['center'].__setitem__(0, 1e309)),
    ):
        custom = deepcopy(config)
        change(custom)
        path = root / f'{label}.json'
        path.write_text(json.dumps(custom))
        with (root / f'{label}.log').open('w') as log:
            process = subprocess.Popen(command + [f'environment_file:={path}'], stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            try:
                process.wait(timeout=45)
            finally:
                stop(process)
        text = (root / f'{label}.log').read_text()
        assert 'invalid environment configuration' in text, text
        assert 'process has died' in text and 'sending signal' in text
        invalid.append(label)
    (root / 'summary.json').write_text(json.dumps(dict(requests=summary, invalid=invalid), indent=2))
    print(f'PASS: {len(summary)} requests, {len(invalid)} invalid configs, same-source RViz geometry')


if __name__ == '__main__':
    main()
