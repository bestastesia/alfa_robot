from dataclasses import replace
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from curobo_core.contracts import PlannerAssets


ROOT = Path(__file__).resolve().parents[1]


def local_generated_path(value):
    marker = '/generated/'
    if marker in value:
        return ROOT / 'generated' / value.split(marker, 1)[1]
    return Path(value)


def prepare_fast_assets():
    model = ROOT / 'generated/zero_model_candidate_20261018'
    output = ROOT / 'generated/fast_runtime_assets'
    output.mkdir(parents=True, exist_ok=True)
    configs = {}
    for label, source in (('fixed', model / 'fixed.yaml'), ('mobile', model / 'mobile.yaml'),
                          ('dynamics', ROOT / 'generated/pr71_current_dynamics/robot.yaml')):
        document = yaml.safe_load(source.read_text())
        kin = document.get('robot_cfg', document)['kinematics']
        urdf_source = local_generated_path(kin['urdf_path'])
        tree = ET.parse(urdf_source)
        for mesh in tree.findall('.//mesh'):
            path = local_generated_path(mesh.get('filename'))
            if not path.is_absolute():path = urdf_source.parent / path
            if not path.is_file():raise FileNotFoundError(f'Prepare checkout model assets first: {path}')
            mesh.set('filename', str(path))
        destination = output / f'{label}.urdf'
        tree.write(destination, encoding='utf-8', xml_declaration=True)
        kin['urdf_path'] = str(destination)
        kin['asset_root_path'] = str(output)
        configs[label] = output / f'{label}.yaml'
        configs[label].write_text(yaml.safe_dump(document, sort_keys=False))
    assets = replace(PlannerAssets(), robot_config=configs['fixed'], mobile_robot_config=configs['mobile'],
                     urdf=output / 'fixed.urdf', named_poses=model / 'named_poses.yaml')
    return assets, configs['dynamics']
