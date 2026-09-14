#!/usr/bin/env python3
"""Time real Rerun writes and read the RRD back; no planning or collision settings changed."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import time

import numpy as np
import rclpy
import rerun as rr
from rerun.experimental import RrdReader
from std_msgs.msg import String
from alfa_robot_rerun.demo_failure import replay_frames
from alfa_robot_rerun.v3_single_arm_box_extract_viewer import V3SingleArmBoxExtractViewer, matrix_to_quaternion


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--artifacts', type=Path, required=True)
    args = parser.parse_args()
    root = args.artifacts.resolve()
    root.mkdir(parents=True, exist_ok=True)
    recording = root / 'trajectory.rrd'
    message = String(data=args.result.read_text())
    payload = json.loads(message.data)
    frames = replay_frames(payload)
    assert frames
    rclpy.init(args=['--ros-args', '-p', 'spawn_viewer:=false', '-p', f'recording_path:={recording}',
                    '-p', 'robot_description:=' + args.urdf.read_text()])
    node = V3SingleArmBoxExtractViewer()
    assert not list(node.timers), 'Rerun must not pace writes with a playback timer'
    rr.get_global_data_recording().flush()
    begin = time.perf_counter()
    node.on_task(message)
    elapsed = time.perf_counter() - begin
    assert node.frame_index == len(frames)
    assert node.last_frame.joints == tuple(frames[-1]['joints'])
    assert node.last_frame.box_visible == frames[-1].get('box_visible', True)
    rr.get_global_data_recording().disconnect()
    # Read actual serialized components, not mocked log calls. Ignore wall-clock timelines.
    rows = defaultdict(dict)
    store = RrdReader(recording).store()
    for chunk in store.stream():
        if chunk.is_static or 'task_frame' not in chunk.timeline_names:
            continue
        batch = chunk.to_record_batch()
        ticks = batch.column('task_frame').to_pylist()
        for name in batch.schema.names:
            if name in ('TextLog:text', 'Transform3D:translation', 'Transform3D:quaternion',
                        'Boxes3D:centers', 'Boxes3D:quaternions', 'Clear:is_recursive'):
                for tick, value in zip(ticks, batch.column(name).to_pylist()):
                    if value is not None:
                        rows[(chunk.entity_path, name)][tick] = value
    stages = rows[('/world/current_stage', 'TextLog:text')]
    assert set(stages) == set(range(1, len(frames) + 1)), 'missing or extra trajectory ticks'
    for link in node.robot.links:
        for component in ('translation', 'quaternion'):
            assert set(rows[(f'/world/robot/{link}', f'Transform3D:{component}')]) == set(stages)
    # Every raw frame's transforms and payload visibility survive SDK serialization.
    for tick, frame in enumerate(frames, 1):
        assert stages[tick] == [f"generation={node.generation} frame={tick}/{len(frames)} "
            f"stage={frame['stage']} attached={frame['box_attached']} "
            f"visible={frame.get('box_visible', True)} scene={frame.get('scene_index', 0)}"]
        tf = node.robot.fk(dict(zip(payload['joint_names'], frame['joints'])))
        for link, pose in tf.items():
            assert np.allclose(rows[(f'/world/robot/{link}', 'Transform3D:translation')][tick],
                               [pose[:3, 3]], atol=2e-6)
            assert np.allclose(rows[(f'/world/robot/{link}', 'Transform3D:quaternion')][tick],
                               [matrix_to_quaternion(pose[:3, :3])], atol=2e-6)
        visible = frame.get('box_visible', True)
        attached = frame['box_attached']
        for name, shown in [('target', visible and not attached), ('carried', visible and attached)]:
            path = '/world/boxes/' + name
            assert (tick in rows[(path, 'Boxes3D:centers')]) == shown
            if not shown:
                assert rows[(path, 'Clear:is_recursive')][tick] == [True]
        if attached and visible:
            scene = payload.get('scenes', [payload])[frame.get('scene_index', 0)]
            tool = tf[scene['tool_link']]
            center = tool[:3, 3] + tool[:3, :3] @ scene['tool_to_box_center']
            assert np.allclose(rows[('/world/boxes/carried', 'Boxes3D:centers')][tick], [center], atol=2e-6)
            rotation = tool[:3, :3] @ scene['tool_to_box_rotation']
            assert np.allclose(rows[('/world/boxes/carried', 'Boxes3D:quaternions')][tick],
                               [matrix_to_quaternion(rotation)], atol=2e-6)
    failure = rows[('/summary/failure', 'TextLog:text')]
    assert set(failure) == ({len(frames)} if not payload['success'] and payload.get('diagnostic') else set())
    report = dict(frames=len(frames),write_elapsed_s=node.write_elapsed_s,callback_elapsed_s=elapsed,
                  recording_bytes=recording.stat().st_size,all_frames_read_back=True,
                  all_link_transforms_checked=True,attachment_and_disappearance_checked=True,
                  final_stage=frames[-1]['stage'],last_tick=len(frames),success=payload['success'])
    (root / 'check.json').write_text(json.dumps(report, indent=2) + '\n')
    print('PASS Rerun timeline:', json.dumps(report), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
