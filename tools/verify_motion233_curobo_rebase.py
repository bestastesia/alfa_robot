#!/usr/bin/env python3
"""Verify the pinned mentor baseline, relocated assets and fresh cycle/UI evidence."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml

BASE = '42002471841a584684cfeb0f5799cc58741fd2c6'
SDK = '78fd485fa82d9b9a063fb4985e371814587e666a'
ROOT = Path(__file__).resolve().parents[1]
PREFIX = 'research/curobo_v3/'


def git(*args, root=ROOT):
    return subprocess.check_output(['git', '-C', str(root), *args])


def relocated(value):
    if isinstance(value, str) and '/generated/' in value:
        return 'generated/' + value.split('/generated/', 1)[1]
    if isinstance(value, dict):
        return {key: relocated(item) for key, item in value.items()}
    if isinstance(value, list):
        return [relocated(item) for item in value]
    return value


def xml(element):
    return (element.tag, sorted(relocated(element.attrib).items()),
            (element.text or '').strip(), tuple(xml(child) for child in element))


def cycle(result):
    assert result['success'], result.get('error', result.get('blocked_stage'))
    assert result['final_home_error'] <= 1e-7
    assert result['max_adjacent_joint_step_deg'] <= 0.50001
    frames = result['frames']
    assert len(frames) == len(result['phases']) == len(result['payload']) and len(frames) > 1
    assert all(len(row) == len(result['joint_names']) and all(math.isfinite(v) for v in row)
               for row in frames)
    assert max(abs(a-b) for a, b in zip(frames[0], frames[-1])) <= 1e-7
    assert {'attach', 'extract', 'transport', 'turn_loaded', 'release', 'turn_empty',
            'return_home'} <= set(result['phases'])


def verify_source(source, runtime_root):
    hashes = {}
    for name in git('ls-tree', '-r', '--name-only', source, '--', PREFIX).decode().splitlines():
        expected = git('show', f'{source}:{name}')
        assert (ROOT / name).read_bytes() == expected, f'uncommitted candidate source: {name}'
        path = Path(name.removeprefix(PREFIX))
        if path.parts[0] in ('generated', 'models', 'vendor'):
            assert expected == git('show', f'{BASE}:{name}'), f'mentor model/solver asset changed: {name}'
        actual = (runtime_root / path).read_bytes()
        if path.parts[0] == 'generated' and path.suffix == '.yml':
            assert relocated(yaml.safe_load(expected)) == relocated(yaml.safe_load(actual)), path
        elif path.parts[0] == 'generated' and path.suffix == '.urdf':
            assert xml(ET.fromstring(expected)) == xml(ET.fromstring(actual)), path
        else:
            assert expected == actual, f'runtime source/model changed: {path}'
        hashes[str(path)] = hashlib.sha256(expected).hexdigest()
    return hashes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-ref', default=BASE, help='exact candidate commit to audit; default freezes mentor source')
    parser.add_argument('--check-optional-planners', action='store_true',
                        help='also gate optional planner comparisons; not required for the RRT migration')
    parser.add_argument('--artifacts', required=True, type=Path)
    parser.add_argument('--runtime-root', required=True, type=Path)
    parser.add_argument('--sdk-repository', required=True, type=Path)
    parser.add_argument('--reference-cycle', required=True, type=Path)
    args = parser.parse_args()
    source = git('rev-parse', args.source_ref + '^{commit}').decode().strip()
    git('merge-base', '--is-ancestor', BASE, source)
    git('merge-base', '--is-ancestor', source, 'HEAD')
    assert git('rev-parse', 'HEAD', root=args.sdk_repository).decode().strip() == SDK
    import curobo
    assert Path(curobo.__file__).resolve().is_relative_to(args.sdk_repository.resolve())
    hashes = verify_source(source, args.runtime_root)
    load = lambda name: json.loads((args.artifacts / name).read_text())
    runs = load('cycle.json')
    reference = json.loads(args.reference_cycle.read_text())['request']
    request = runs['request']
    for key in ('tasks', 'targets'):
        assert request[key] == reference[key], key
    for key in ('objects', 'attachments', 'frame_id', 'revision', 'policy', 'meshes'):
        assert request['snapshot'][key] == reference['snapshot'][key], key
    for key in ('joint_names', 'positions', 'base_pose'):
        assert request['snapshot']['state'][key] == reference['snapshot']['state'][key], key
    assert len(runs['results']) == 2
    for result in runs['results']:
        assert result['planner'] == 'informed_rrt'
        cycle(result)
    failed_planners = {}
    if args.check_optional_planners:
        planners = load('six-planners.json')
        assert set(planners) == {'informed_rrt', 'rrt', 'rrtconnect', 'prm', 'informed_connect', 'bitstar'}
        for kind, result in planners.items():
            if result['success']:
                cycle(result)
            else:
                failed_planners[kind] = result.get('error', result.get('blocked_stage'))
    imports = load('imports.json')
    assert imports['import_pairs_checked'] == 21 and not imports['errors']
    smoke = load('core-smoke.json')
    assert all(smoke[key] for key in ('gpu_cache_reused', 'scene_update_rebuilt_cache',
        'payload_update_rebuilt_cache', 'stale_plan_rejected', 'core_did_not_load_viser'))
    cycle(smoke['success_result'])
    assert smoke['stale_result']['error']['code'] == 'SCENE_CHANGED'
    conveyor = load('conveyor.json')
    assert conveyor['success'] and conveyor['far']['valid'] and not conveyor['near']['valid']
    assert conveyor['disabled']['valid']
    ui = load('ui-check.json')
    assert ui['livePlanningClicked'] and ui['sawBusy'] and ui['success'] and ui['finalFrameConfirmed']
    cycle(load('ui-live-cycle.json'))
    report = {'base_commit': BASE, 'source_commit': source, 'head_commit': git('rev-parse', 'HEAD').decode().strip(),
        'branch': git('branch', '--show-current').decode().strip(),
        'curobo_commit': SDK, 'research_files_verified': len(hashes),
        'normalized_relocated_assets_identical': True, 'same_reference_geometry_state_targets': True,
        'request_strategy': {'suction_mode': request.get('suction_mode', 'side'), 'search_seed': request.get('search_seed')},
        'two_rrt_cycles_passed': True, 'optional_planners_checked': args.check_optional_planners,
        'six_planners_passed': (not failed_planners) if args.check_optional_planners else None,
        'failed_planners': failed_planners, 'acceptance_passed': not failed_planners,
        'live_visual_cycle_passed': True,
        'policy': request['snapshot']['policy'], 'source_sha256': hashes,
        'scope': 'f044bf1 research L24/R20 only; not 25-box, mesh/FCL/OBB or hardware acceptance'}
    (args.artifacts / 'rebase-audit.json').write_text(json.dumps(report, indent=2) + '\n')
    if failed_planners:
        raise SystemExit(f'FAIL: {len(hashes)} files/models and default/live cycles verified, '
                         f'but planner regression failed: {failed_planners}')
    print(f'PASS: {len(hashes)} candidate files; mentor models identical; same default task/policy; 2 default Informed RRT cycles and live UI (optional planners checked: {args.check_optional_planners}).')


if __name__ == '__main__':
    main()
