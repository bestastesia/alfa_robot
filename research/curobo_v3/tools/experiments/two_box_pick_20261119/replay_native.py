"""CPU-only, read-only playback of accepted native arrays; no planner or robot connection."""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

TOOLS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TOOLS))
from curobo_core.box_pick import BoxPick


def native_samples(record):
    if record.get('workflow')=='handover_same_side':
        from curobo_core.handover import handover_native_samples
        return handover_native_samples(record)
    if record.get('workflow')=='ordered_upper_first':
        from curobo_core.ordered_pick import ordered_native_samples
        return ordered_native_samples(record)
    if not record.get('success') or not record.get('timing', {}).get('success'):
        raise ValueError('only fully accepted native results may be replayed')
    stages = record['timing']['stages']
    if [s['phase'] for s in stages] != ['approach', 'contact', 'extract', 'transport']:
        raise ValueError('expected all four native stages')
    clocks, positions, phases = [], [], []
    elapsed = 0.
    previous = None
    for stage in stages:
        if not stage['success']:
            raise ValueError('rejected native stage')
        with np.load(stage['trajectory_file'], allow_pickle=False) as arrays:
            t, q = arrays['times'].copy(), arrays['position'].copy()
        if (t.ndim != 1 or len(t) < 2 or q.shape != (len(t), 15) or
                not np.isfinite(t).all() or not np.isfinite(q).all() or
                t[0] != 0 or not np.all(np.diff(t) > 0)):
            raise ValueError('invalid native samples')
        if previous is not None and not np.allclose(previous, q[0], rtol=0, atol=1e-5):
            raise ValueError('discontinuous native stage seam')
        start = 0 if previous is None else 1
        clocks.extend(t[start:] + elapsed)
        positions.extend(q[start:])
        phases.extend([stage['phase']] * (len(t)-start))
        elapsed += t[-1]
        previous = q[-1]
    return np.asarray(clocks), np.asarray(positions), np.asarray(phases)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results', type=Path, required=True)
    parser.add_argument('--port', type=int, default=8294)
    args = parser.parse_args()
    summary = json.loads((args.results/'summary.json').read_text())
    accepted = [json.loads(Path(row['output']).read_text()) for row in summary['results'] if row['success']]
    if not accepted:
        parser.error('no fully accepted native result to replay')
    # Installed viewer + native URDF suffice; no frontend package installation or CUDA import.
    import viser
    from viser.extras import ViserUrdf
    from scipy.spatial.transform import Rotation
    import yourdfpy
    urdf = TOOLS.parent/'generated/fast_runtime_assets/fixed.urdf'
    robot = yourdfpy.URDF.load(urdf, load_meshes=False)
    server = viser.ViserServer(host='127.0.0.1', port=args.port, label='PR82 two-box native replay (read-only)')
    visual = ViserUrdf(server, urdf, root_node_name='/robot')
    labels = [f"{i}: {r['task']['size']} {r['task']['side']} {r['suction']} {r['task']['center_m']}" for i, r in enumerate(accepted)]
    choice = server.gui.add_dropdown('Accepted case', options=labels)
    play = server.gui.add_checkbox('Play', initial_value=False)
    speed = server.gui.add_slider('Speed', min=.1, max=2., step=.1, initial_value=1.)
    progress = server.gui.add_slider('Native time (s)', min=0., max=1., step=.01, initial_value=0.)
    status = server.gui.add_markdown('')
    server.gui.add_markdown(f"Batch: **{summary['passed']}/{len(summary['results'])}**. Accepted cases only. "
        'Actual timed arrays, nearest recorded sample; no interpolated or repaired path. '
        'Not full-wall/continuous-collision/hardware acceptance.')
    box = None
    obstacles = []
    selected = None
    previous_time = time.monotonic()
    while True:
        now = time.monotonic()
        if choice.value != selected:
            selected = choice.value
            record = accepted[labels.index(selected)]
            task = BoxPick(**record['task'])
            times, rows, phases = native_samples(record)
            progress.max = float(times[-1]); progress.value = 0.; play.value = False
            if box is not None:box.remove()
            for node in obstacles:node.remove()
            obstacles = []
            for obj in record['scene']['objects']:
                if obj['object_id'] == 'wall_box_00':continue
                obstacles.append(server.scene.add_box('/scene/'+obj['object_id'], color=(120, 140, 160),
                    dimensions=obj['dimensions_m'], position=obj['pose']['position'],
                    wxyz=obj['pose']['quaternion_wxyz'], opacity=.12))
            box = server.scene.add_box('/payload', color=(193, 140, 66), dimensions=task.dimensions_m)
        if play.value:
            progress.value = min(float(times[-1]), progress.value+(now-previous_time)*speed.value)
            if progress.value >= times[-1]:play.value = False
        previous_time = now
        i = int(np.clip(np.searchsorted(times, progress.value), 0, len(times)-1))
        if i > 0 and progress.value-times[i-1] < times[i]-progress.value:i -= 1
        cfg = dict(zip(record['joint_names'], rows[i]))
        visual.update_cfg({name: cfg.get(name, 0.) for name in robot.actuated_joint_names})
        robot.update_cfg(cfg)
        pose = np.eye(4); pose[:3, 3] = task.center_m
        if phases[i] in ('extract', 'transport'):
            pose = robot.get_transform(task.side+'_tool0', robot.base_link) @ task.tool_to_box()
        box.position = pose[:3, 3]
        box.wxyz = np.roll(Rotation.from_matrix(pose[:3, :3]).as_quat(), 1)
        status.content = f"**{phases[i]}** · sample {i}/{len(times)-1} · {times[i]:.3f}s / {times[-1]:.3f}s"
        time.sleep(1/30)


if __name__ == '__main__':
    main()
