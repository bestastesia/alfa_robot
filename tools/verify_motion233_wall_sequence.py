#!/usr/bin/env python3
"""Strict sequence evidence audit; research collision policy, not mesh/hardware certification."""
import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import yaml

from verify_motion233_curobo_rebase import BASE, ROOT, SDK, cycle, git, verify_source

sys.path.insert(0, str(ROOT / 'research/curobo_v3/tools'))
from curobo_core.contracts import ACTIVE_JOINTS, PlanRequest
from curobo_core.fixtures import WallLayout, wall_sequence
from curobo_core.scene import SceneSnapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--runtime-root', type=Path, required=True)
    parser.add_argument('--sdk-repository', type=Path, required=True)
    parser.add_argument('--source-ref', default='HEAD')
    args = parser.parse_args()
    source = git('rev-parse', args.source_ref + '^{commit}').decode().strip()
    git('merge-base', '--is-ancestor', BASE, source)
    assert git('rev-parse', 'HEAD', root=args.sdk_repository).decode().strip() == SDK
    hashes = verify_source(source, args.runtime_root)
    data = json.loads(args.result.read_text())
    assert data['sequence']
    assert data['success'] == (data['error'] is None)
    initial = SceneSnapshot.from_dict(data['initial_snapshot'])
    layout = WallLayout(distance_m=.75)
    objects = {o.object_id: o for o in initial.objects}
    front = objects['wall_box_00'].pose.position[0] - .15 - .75
    # f044 chassis geometry: do not accept a mislabeled 0.9m wall.
    assert math.isclose(front, .5080520510673523, abs_tol=1e-9)
    for expected in layout.objects(front):
        actual = objects[expected.object_id]
        assert actual.dimensions_m == expected.dimensions_m
        assert actual.pose.quaternion_wxyz == expected.pose.quaternion_wxyz
        assert all(math.isclose(a, b, abs_tol=1e-12) for a, b in zip(actual.pose.position, expected.pose.position))
    policy = initial.to_dict()['policy']
    assert policy == {'exclude_task_objects_before_contact': True, 'defer_payload_until_extract_end': True,
                      'ground_z_m': 0., 'max_box_tilt_deg': 89., 'ground_support_link': 'base_link'}
    assert [r['task'] for r in data['rounds']] == wall_sequence(layout)
    assert len(data['frames']) == len(data['phases']) == len(data['payload']) == len(data['frame_rounds'])
    mobile = yaml.safe_load((args.runtime_root/'generated/v3_analytic_071cb95/mobile18/alfa_v322_suction_mobile18.yml').read_text())
    limits = {j.attrib['name']: j.find('limit').attrib for j in ET.parse(mobile['kinematics']['urdf_path']).findall('joint')
              if j.find('limit') is not None}
    limit_overshoot = {}
    for column, name in enumerate(data['joint_names']):
        bounds = limits[name]
        low, high = float(bounds['lower']), float(bounds['upper'])
        values = [row[column] for row in data['frames']]
        excess = max(0., low-min(values), max(values)-high)
        assert excess <= 1e-6, (name, excess)
        limit_overshoot[name] = excess
    current = data['initial_snapshot']
    completed, offset, fallbacks = [], 0, []
    for number, record in enumerate(data['rounds']):
        if record['status'] != 'completed':
            assert record['status'] == 'failed' and not record['result']['success']
            assert record['request']['snapshot'] == current
            assert record['result']['error'] == data['error'] and data['blocked_round'] == number+1
            assert all(r['status'] == 'pending' for r in data['rounds'][number+1:])
            break
        request, result = record['request'], record['result']
        assert request['snapshot'] == current
        assert record['removed_before'] == completed
        assert request['tasks'] == result['task'] == record['task']
        assert result['planner'] == 'informed_rrt'
        seed = data.get('search_seed')
        assert request.get('search_seed') == result.get('search_seed') == seed
        if seed is not None:
            assert result['seed_policy']['effective_search_seed_base'] == seed
            assert result['seed_policy']['ik_random_seed_unchanged']
            assert result['transport_search_stats']['search_seed'] == seed
            for attempt in result['attempts']:
                if 'approach' in attempt:
                    assert attempt['approach']['search_seed'] == (seed + attempt['candidate']-1) % 2**32
            home_stats = result['home_search_stats']
            if home_stats.get('fallback'):
                home_stats = home_stats['failed_search']
            assert home_stats['search_seed'] == (seed+1) % 2**32
        assert result['request_id'] == PlanRequest.from_dict(request).identity
        assert result['policy'] == policy and result['joint_names'] == data['joint_names']
        cycle(result)
        count = len(result['frames'])
        for key in ('frames', 'phases', 'payload'):
            assert data[key][offset:offset+count] == result[key]
        assert data['frame_rounds'][offset:offset+count] == [number] * count
        if offset:
            assert all(abs(a-b) <= 1e-7 for a, b in zip(data['frames'][offset-1], data['frames'][offset]))
        attach, release = result['predicted_scene_events']
        assert attach['phase'] == 'attach' and release['phase'] == 'release'
        ids = [f'wall_box_{i:02d}' for i in record['task'].values()]
        assert {a['object_id'] for a in attach['snapshot']['attachments']} == set(ids)
        assert {a['parent_link'] for a in attach['snapshot']['attachments']} == {s+'_tool0' for s in record['task']}
        assert not release['snapshot']['attachments']
        expected_objects = [o for o in current['objects'] if o['object_id'] not in ids]
        assert record['final_snapshot']['objects'] == release['snapshot']['objects'] == expected_objects
        current = record['final_snapshot']
        assert current['revision'] > request['snapshot']['revision']
        assert current['policy'] == policy and current['model_id'] == initial.model_id
        state = SceneSnapshot.from_dict(current).state
        assert state.ordered(ACTIVE_JOINTS) == tuple(result['frames'][-1][result['joint_names'].index(j)] for j in ACTIVE_JOINTS)
        completed.extend(record['task'].values())
        offset += count
        stats = result['home_search_stats']
        if stats.get('fallback'):
            assert stats['strategy'] == 'reverse_validated_empty_path' and stats['dense_validated']
            fallbacks.append(number+1)
    assert offset == len(data['frames'])
    assert len(set(completed)) == len(completed)
    assert data['completed_box_ids'] == completed
    assert data['remaining_box_ids'] == sorted(set(range(25)) - set(completed))
    assert data['success'] == (len(completed) == 25)
    assert data['final_snapshot'] == current and data['fallback_rounds'] == fallbacks
    report = {'source_commit': source, 'base_commit': BASE, 'sdk_commit': SDK,
              'research_files_verified': len(hashes), 'source_sha256': hashes,
              'passed': data['success'], 'completed_boxes': len(completed), 'rounds': 15,
              'completed_rounds': sum(r['status'] == 'completed' for r in data['rounds']),
              'error': data['error'], 'joint_limit_overshoot': limit_overshoot,
              'single_arm_rounds': 5, 'planner': 'informed_rrt', 'search_seed': data.get('search_seed'), 'front_to_wall_m': .75, 'frames': offset,
              'fallback_rounds': fallbacks, 'total_ms': data['total_ms'],
              'scope': 'full ordered wall and validated completed prefix; original f044 research collision policy; no mesh/hardware certification'}
    output = args.result.with_suffix('.audit.json')
    output.write_text(json.dumps(report, indent=2) + '\n')
    if not data['success']:
        raise SystemExit(f'INCOMPLETE: {len(completed)}/25 boxes, {offset} validated frames; '
                         f'stopped at round {data["blocked_round"]}: {data["error"]}')
    print(f'PASS: 25/25 boxes, 15 rounds, {offset} frames; explicit empty-return fallbacks: {fallbacks}.')


if __name__ == '__main__':
    main()
