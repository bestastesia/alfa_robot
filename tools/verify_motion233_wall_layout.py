#!/usr/bin/env python3
"""Check fresh layout evidence; selected pairs only, never whole-wall certification."""
import argparse
import json
import math
from pathlib import Path
import sys

from verify_motion233_curobo_rebase import ROOT, cycle

sys.path.insert(0, str(ROOT / 'research/curobo_v3/tools'))
from curobo_core.contracts import PlanRequest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts', type=Path, required=True)
    args = parser.parse_args()
    def load(name):
        return json.loads((args.artifacts / name).read_text())
    baseline = load('cycle.json')['request']
    base_scene = baseline['snapshot']
    first = next(o for o in base_scene['objects'] if o['object_id'] == 'wall_box_00')
    front = first['pose']['position'][0] - .15 - .9
    cases = [
        ('sparse-3x4', 3, 4, .9, 0., .82, [0, 1, 2, 4, 5, 6, 7, 8, 9, 11], {'left': 11, 'right': 8}),
        ('sparse-4x3', 4, 3, .91, .02, .41, [0, 2, 3, 5, 6, 8, 9, 11], {'left': 11, 'right': 9}),
        ('standoff-075', 5, 5, .75, 0., 0., list(range(25)), {'left': 24, 'right': 20}),
    ]
    report = {'scope': 'selected dual-arm pairs only; frozen f044 research collision policy', 'cases': {}}
    for name, rows, columns, distance, center, bottom, ids, pair in cases:
        data = load(name + '-cycle.json')
        request = data['request']
        scene = request['snapshot']
        assert request['tasks'] == pair and request['targets'] == baseline['targets'], name
        for key in ('policy', 'model_id', 'attachments', 'meshes', 'frame_id'):
            assert scene[key] == base_scene[key], (name, key)
        for key in ('joint_names', 'positions', 'base_pose'):
            assert scene['state'][key] == base_scene['state'][key], (name, key)
        boxes = [o for o in scene['objects'] if o['object_id'].startswith('wall_box_')]
        assert [o['object_id'] for o in boxes] == [f'wall_box_{i:02d}' for i in ids], name
        enclosure = lambda s: [o for o in s['objects'] if not o['object_id'].startswith('wall_box_')]
        # Upstream make_scene translates the fixed-size enclosure with wall standoff.
        actual_room, base_room = enclosure(scene), enclosure(base_scene)
        assert len(actual_room) == len(base_room)
        for actual, original in zip(actual_room, base_room):
            for key in ('object_id', 'dimensions_m'):
                assert actual[key] == original[key], (name, key)
            assert actual['pose']['quaternion_wxyz'] == original['pose']['quaternion_wxyz']
            a, b = actual['pose']['position'], original['pose']['position']
            assert a[1:] == b[1:] and math.isclose(a[0]-b[0], distance-.9, abs_tol=1e-12)
        for box, i in zip(boxes, ids):
            assert 0 <= i < rows * columns
            assert box['dimensions_m'] == [.3, .4, .4]
            assert box['pose']['quaternion_wxyz'] == [1., 0., 0., 0.]
            expected = (front + distance + .15, center + (i % columns - (columns-1)/2)*.41,
                        bottom + .2 + (i // columns)*.41)
            assert all(math.isclose(a, b, abs_tol=1e-12) for a, b in zip(box['pose']['position'], expected)), name
        assert len(data['results']) == 1
        result = data['results'][0]
        cycle(result)
        assert result['request_id'] == PlanRequest.from_dict(request).identity
        assert result['task'] == pair and result['policy'] == base_scene['policy']
        report['cases'][name] = {'occupied_boxes': len(ids), 'tested_pair': pair,
                                'front_to_wall_m': distance, 'total_ms': result['total_ms']}
    for prefix, pair in [('custom', cases[0][-1]), ('075', cases[2][-1])]:
        ui, result = load(f'ui-{prefix}-check.json'), load(f'ui-{prefix}-cycle.json')
        assert all(ui[k] for k in ('livePlanningClicked', 'sawBusy', 'success', 'finalFrameConfirmed'))
        cycle(result)
        assert result['task'] == pair and result['policy'] == base_scene['policy']
        assert ui['frames'] == len(result['frames'])
    rejected = load('negative-input-checks.json')
    assert {r['case'] for r in rejected} == {'invalid-wall', 'conflicting-scene', 'stale-preload'}
    assert all(r['rejected'] and r['exit_code'] == 2 for r in rejected)
    report['live_custom_and_075_ui_passed'] = True
    report['invalid_inputs_rejected'] = True
    (args.artifacts / 'layout-audit.json').write_text(json.dumps(report, indent=2) + '\n')
    print('PASS: sparse 3x4, sparse 4x3, 0.75m; live custom/0.75m UI; invalid inputs rejected.')


if __name__ == '__main__':
    main()
