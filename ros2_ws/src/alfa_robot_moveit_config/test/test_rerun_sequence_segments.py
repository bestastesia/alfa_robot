#!/usr/bin/env python3
"""No GUI: out-of-order segments, failures, duplicates, repair and task/scene switches."""
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import patch
import json
import numpy as np

from alfa_robot_rerun.sequence_timeline import SequenceTimeline
from alfa_robot_rerun import v3_single_arm_box_extract_viewer as module


def fixture():
    scenes = [dict(box_center=[i, 0, 1], box_size=[.3, .4, .5],
                   tool_link='left_tool0' if i == 0 else 'right_tool0',
                   neighbor_centers=[[1, 0, 1]] if i == 0 else [],
                   environment={'boxes': [dict(id='old', center=[0, 3, 0], size=[1, 1, 1])]} if i == 0 else {})
              for i in range(2)]
    frames = [dict(stage=stage, joints=[joint], box_attached=attached, box_visible=visible, scene_index=i)
              for i, stage, joint, attached, visible in
              [(0, 'attach_box', 0., True, True), (0, 'release_box', 1., False, False),
               (1, 'approach', 1., False, True), (1, 'FAILED', 2., True, True)]]
    common = dict(sequence=True, distance_demo=True, x=.5, box_id=20, chassis_front_x=.4, task_id='a:1', publisher_id='a', generation=1,
                  joint_names=['joint'], diagnostic_frames=[])
    segments = [dict(common, kind='segment', segment_index=i, frame_begin=2*i, frame_end=2*i+2,
                     success=i == 0, frames=frames[2*i:2*i+2], scenes=scenes[:i+1],
                     diagnostic={} if i == 0 else dict(stage='collision', reason='test')) for i in range(2)]
    segments[1]['diagnostic_frames'] = segments[1].pop('frames')
    final = dict(common, kind='result', segment_count=2, frame_begin=0, frame_end=4,
                 frames=frames[:2], diagnostic_frames=[dict(f, diagnostic_only=True) for f in frames],
                 success=False, scenes=scenes, diagnostic=segments[1]['diagnostic'], completed_count=1, failed_box_id=21)
    return segments, final


def main():
    segments, final = fixture()
    timeline = SequenceTimeline()
    assert timeline.select(segments[0])
    assert not timeline.append(segments[1]) and not timeline.frames
    assert not timeline.append(segments[1])  # pending duplicate
    ready = timeline.append(segments[0])
    assert len(ready) == 2 and len(timeline.frames) == 4
    assert not timeline.append(final) and not timeline.append(segments[0])
    assert timeline.frames[-1]['joints'] == [2.]
    for change in ('frame', 'scene', 'range', 'segment_id'):
        bad = deepcopy(segments[0])
        if change == 'frame': bad['frames'][0]['joints'][0] = 10.
        if change == 'scene': bad['scenes'][0]['neighbor_centers'] = []
        if change == 'range': bad['frame_end'] += 1
        if change == 'segment_id': bad['segment_index'] = 1
        try: timeline.append(bad)
        except ValueError: pass
        else: raise AssertionError(change)
    for received in ([], [segments[0]], [segments[1]]):
        late = SequenceTimeline()
        late.select(final)
        for item in received: late.append(item)
        late.append(final)
        assert late.frames == timeline.frames and not late.pending
    newer = dict(final, task_id='a:2', generation=2)
    assert timeline.select(newer) and not timeline.frames
    assert not timeline.select(final)
    assert timeline.select(dict(final, task_id='b:1', publisher_id='b'))
    assert not timeline.select(dict(final, task_id='a:99', generation=99))

    # Exercise actual writer calls, including their trajectory timestamps and scene clears.
    viewer = module.V3SingleArmBoxExtractViewer.__new__(module.V3SingleArmBoxExtractViewer)
    viewer.sequence_timeline = SequenceTimeline()
    viewer.global_frame = 1
    viewer.generation = 0
    viewer.tool_link = 'left_tool0'
    viewer.tool_to_box_center = np.zeros(3)
    viewer.box_center = np.zeros(3)
    viewer.box_size = np.ones(3)
    viewer.robot = SimpleNamespace(fk=lambda _: {'left_tool0': np.eye(4), 'right_tool0': np.eye(4)})
    times, logs, transforms = [0], [], []
    logger = SimpleNamespace(info=lambda _: None, warning=lambda _: None,
                             error=lambda message: (_ for _ in ()).throw(AssertionError(message)))
    with patch.object(module.rr, 'set_time', side_effect=lambda _, sequence: times.__setitem__(0, sequence)), \
         patch.object(module.rr, 'log', side_effect=lambda path, value: logs.append((times[0], path, value))), \
         patch.object(module.rr, 'send_columns', side_effect=lambda *a, **k: transforms.append((a, k))), \
         patch.object(module.rr, 'get_global_data_recording', return_value=None), \
         patch.object(module.rr, 'send_blueprint') as blueprint, \
         patch.object(module.V3SingleArmBoxExtractViewer, 'get_logger', return_value=logger):
        send = lambda p: viewer.on_task(SimpleNamespace(data=json.dumps(p)))
        send(segments[1])
        assert not logs and not transforms  # future scene must not disappear on receipt
        send(segments[0])
        assert viewer.global_frame == 5 and viewer.last_frame.stage == 'FAILED'
        assert viewer.tool_link == 'right_tool0' and not viewer.neighbor_centers
        count = len(logs), len(transforms)
        send(final); send(segments[0]); send(segments[1])
        assert (len(logs), len(transforms)) == count and viewer.global_frame == 5
        assert [t for t, p, _ in logs if p == 'world/current_stage'] == [1, 2, 3, 4]
        assert [t for t, p, _ in logs if p == 'world/environment'] == [1, 3]
        assert [t for t, p, v in logs if p == 'summary/failure' and isinstance(v, module.rr.TextLog)] == [4]
        assert not blueprint.called  # never overwrite user's pause, speed or cursor
        send(newer)
        assert viewer.global_frame == 9
        count = len(logs)
        send(final)
        assert len(logs) == count  # stale result cannot switch back to an old scene
    print('PASS segments: failure endpoint, no early scene changes, dedupe/conflicts, gaps/final repair, restart, no UI reset')


if __name__ == '__main__':
    main()
