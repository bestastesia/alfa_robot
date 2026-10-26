#!/usr/bin/env python3
"""CPU FCL audit of stored frames, stopping at the first failure and retaining declared research exemptions."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'research/curobo_v3/tools'))
from curobo_core.mesh_validation import MeshStateChecker
from curobo_core.scene import SceneSnapshot, digest
from verify_motion233_curobo_rebase import cycle


def audit_record(record, config):
    result = record['result']
    cycle(result)
    if len(set(result['joint_names'])) != len(result['joint_names']) or set(result['joint_names']) != set(config['cspace']['joint_names']):
        raise ValueError('mesh audit joint names differ from the configured active model')
    snapshot = SceneSnapshot.from_dict(record['request']['snapshot'])
    attached = next(e['snapshot'] for e in result.get('predicted_scene_events', ()) if e['phase'] == 'attach')
    attachments = SceneSnapshot.from_dict(attached).attachments
    checker = MeshStateChecker(config, snapshot, record['task'], attachments)
    last_extract = max(i for i, p in enumerate(result['phases']) if p == 'extract')
    for index, (frame, phase, payload) in enumerate(zip(result['frames'], result['phases'], result['payload'])):
        if payload != (phase in ('attach', 'extract', 'transport', 'turn_loaded')):
            raise ValueError('payload flag does not match the phase contract')
        loaded = payload and (not snapshot.policy.defer_payload_until_extract_end or
                              phase not in ('attach', 'extract') or index == last_extract)
        failure = checker.check(dict(zip(result['joint_names'], frame)), attached=phase not in ('home', 'approach'), payload_enabled=loaded)
        if failure:
            return {'passed': False, 'checked_frames': index+1, 'frame_index': index, 'phase': phase,
                    'payload_checked': loaded, 'failure': failure, 'joint_names': result['joint_names'], 'positions': frame}
    return {'passed': True, 'checked_frames': len(result['frames'])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sequence', type=Path, required=True)
    parser.add_argument('--runtime-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    raw = args.sequence.read_bytes(); data = json.loads(raw)
    if not data.get('success') or not data.get('sequence') or not data.get('rounds') or not data.get('frames'):
        raise ValueError('mesh acceptance requires a completed, nonempty sequence')
    config = yaml.safe_load((args.runtime_root/'generated/v3_analytic_071cb95/mobile18/alfa_v322_suction_mobile18.yml').read_text())['kinematics']
    fixed = yaml.safe_load((args.runtime_root/'generated/v3_analytic_071cb95/alfa_v322_suction_final.yml').read_text())
    mobile = yaml.safe_load((args.runtime_root/'generated/v3_analytic_071cb95/mobile18/alfa_v322_suction_mobile18.yml').read_text())
    model_id = digest({'urdf': hashlib.sha256(Path(fixed['kinematics']['urdf_path']).read_bytes()).hexdigest(), 'fixed': fixed, 'mobile': mobile})
    if any(r['request']['snapshot']['model_id'] != model_id for r in data['rounds']):
        raise ValueError('sequence belongs to a different prepared robot model')
    report = {'model_id': model_id, 'sequence_sha256': hashlib.sha256(raw).hexdigest(), 'urdf_sha256': hashlib.sha256(Path(config['urdf_path']).read_bytes()).hexdigest(),
              'scope': 'all recorded frames until first failure; FCL convex/BVH meshes and box primitives; declared task/payload exemptions retained; no nonconvex solid-containment, continuous-motion or hardware certification',
              'passed': False, 'rounds': [], 'checked_frames': 0}
    started = time.perf_counter()
    for number, record in enumerate(data['rounds']):
        if record['status'] != 'completed':
            report['failure'] = 'sequence includes an uncompleted round'
            break
        result = audit_record(record, config)
        result['round'] = number+1
        report['rounds'].append(result); report['checked_frames'] += result['checked_frames']
        print(json.dumps(result, ensure_ascii=False), flush=True)
        if not result['passed']:
            break
    else:
        report['passed'] = True
    report['elapsed_s'] = time.perf_counter()-started
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
