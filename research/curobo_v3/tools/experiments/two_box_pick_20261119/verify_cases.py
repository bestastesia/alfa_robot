"""CPU-only independent URDF endpoint/seam audit; not a continuum/CCD certificate."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation
import yourdfpy

TOOLS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TOOLS))
from curobo_core.box_pick import BoxPick
from curobo_core.contracts import ACTIVE_JOINTS

ROOT = TOOLS.parent


def recorded_task(record, original):
    original = dict(original)
    if 'carry_target_contract' not in record:
        # Old omitted offsets meant the fixed PR82 anchor, not today's size/mode profile.
        original.setdefault('carry_offset_m', (0., 0., 0.))
    task = BoxPick(**record['task'])
    assert task == BoxPick(**original), 'input changed or wrong task replayed'
    return task


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results', type=Path, required=True)
    parser.add_argument('--source-snapshot', type=Path)
    args = parser.parse_args()
    folder = args.results.resolve()
    summary = json.loads((folder/'summary.json').read_text())
    inputs = json.loads(Path(summary['cases_file']).read_text())
    assert summary['passed'] == len(summary['results']) == len(inputs) > 0, 'incomplete or failed batch'
    urdf = ROOT/'generated/fast_runtime_assets/fixed.urdf'
    robot = yourdfpy.URDF.load(urdf, load_meshes=False, build_scene_graph=True)
    verified = []
    for row in summary['results']:
        path = Path(row['output']).resolve()
        assert path.parent == folder, 'report belongs to a different batch'
        record = json.loads(path.read_text())
        assert record['success'] and tuple(record['joint_names']) == ACTIVE_JOINTS
        task = recorded_task(record, inputs[row['index']])
        for relative, expected in record['source_sha256'].items():
            source = (args.source_snapshot/relative) if args.source_snapshot else ROOT/relative
            if not source.is_file():source = ROOT/relative
            source = source.resolve()
            assert source.is_relative_to(ROOT) and hashlib.sha256(source.read_bytes()).hexdigest() == expected, relative
        frames = np.asarray(record['frames'])
        phases = np.asarray(record['phases'])
        assert frames.shape == (len(phases), 15) and np.isfinite(frames).all()
        transitions = np.flatnonzero(phases[1:] != phases[:-1])+1
        assert [phases[0], *phases[transitions]] == ['approach', 'contact', 'extract', 'transport']
        seam_error = float(np.max(np.abs(frames[transitions]-frames[transitions-1])))
        assert seam_error <= 1e-5, 'disconnected stages'

        def box_at(values):
            robot.update_cfg(dict(zip(ACTIVE_JOINTS, values)))
            return robot.get_transform(task.side+'_tool0', robot.base_link) @ task.tool_to_box()

        extraction = frames[phases == 'extract']
        first, last = box_at(extraction[0]), box_at(extraction[-1])
        carry = box_at(frames[-1])
        start_error = float(np.linalg.norm(first[:3, 3]-task.center_m))
        pull_error = float(np.linalg.norm(last[:3, 3]-first[:3, 3]-[-task.extraction_distance_m, 0., 0.]))
        carry_error = float(np.linalg.norm(carry[:3, 3]-task.carry_box()[:3, 3]))
        angles = [float(np.rad2deg(Rotation.from_matrix(target.T @ actual[:3, :3]).magnitude()))
                  for target, actual in ((np.eye(3), first), (np.eye(3), last), (task.carry_box()[:3, :3], carry))]
        assert max(start_error, pull_error, carry_error) <= .002 and max(angles) <= 1., 'independent FK mismatch'
        accepted = [attempt for attempt in record['attempts'] if not attempt.get('failure')]
        assert len(accepted) == 1
        for phase, key in (('extract', 'extraction_audit'), ('transport', 'transport_audit')):
            audit = accepted[0][key]
            assert audit['valid'] and not audit['invalid_frame_indices'] and audit['frames'] == int(sum(phases == phase))
        verified.append(dict(index=row['index'], task=record['task'], seam_error_rad=seam_error,
            independent_start_error_m=start_error, independent_pull_error_m=pull_error,
            independent_carry_error_m=carry_error, independent_orientation_errors_deg=angles))
    output = dict(verified_at=datetime.now(timezone.utc).isoformat(), passed=len(verified),
        urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
        scope='independent CPU URDF endpoints and seams; stored collision gates inspected, not independently repeated',
        arbitrary_position_coverage_claim=False,
        experimental_carry_offset_cases=sum(any(v != 0 for v in row['task']['carry_offset_m']) for row in verified),
        results=verified)
    (folder/'independent_fk_verification.json').write_text(json.dumps(output, indent=2)+'\n')
    print(json.dumps({key: value for key, value in output.items() if key != 'results'}, indent=2))


if __name__ == '__main__':
    main()
