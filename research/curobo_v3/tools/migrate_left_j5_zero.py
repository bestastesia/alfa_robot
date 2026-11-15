import json
from pathlib import Path

import numpy as np
import yaml

from v3_zero_model_candidate import ROOT, build, COORDINATE_REVISION


def convert(value):
    shifted = np.asarray(value) - np.pi
    return (shifted + np.pi) % (2 * np.pi) - np.pi


def main():
    folder = ROOT / 'generated/zero_model_candidate_20261018'
    if json.loads((folder / 'manifest.json').read_text()).get('coordinate_revision') == COORDINATE_REVISION:
        raise ValueError('该目录已经迁移，禁止重复转换')
    for name in ('fixed', 'mobile'):
        path = folder / (name + '.yaml')
        config = yaml.safe_load(path.read_text())
        space = config['kinematics']['cspace']
        index = space['joint_names'].index('left_joint5')
        for key in ('nominal_joint_position', 'retract_config'):
            if space.get(key) is not None:
                space[key][index] = float(convert(space[key][index]))
        path.write_text(yaml.safe_dump(config, sort_keys=False))
    path = folder / 'named_poses.yaml'
    named = yaml.safe_load(path.read_text())
    for pose in named['named_poses'].values():
        if 'left_joint5' in pose:
            pose['left_joint5'] = float(convert(pose['left_joint5']))
    named['metadata']['model_revision'] = COORDINATE_REVISION
    path.write_text(yaml.safe_dump(named, sort_keys=False))
    for label in ('side', 'top'):
        path = folder / (label + '_atlas.npz')
        with np.load(path, allow_pickle=False) as stored:
            arrays = {key: stored[key].copy() for key in stored.files}
        arrays['joints'][:, 5] = convert(arrays['joints'][:, 5])
        np.savez_compressed(path, **arrays)
        from v3_contact_roll_policy import aligned_atlas
        aligned_atlas(path, folder / (label + '_contact_roll_atlas.npz'))
    build(folder)
    build(folder / 'mobile', ROOT / 'generated/v3_analytic_071cb95/mobile18/model/robot_meter_meshes_mobile18.urdf')
    print('MIGRATED', COORDINATE_REVISION)


if __name__ == '__main__':
    main()
