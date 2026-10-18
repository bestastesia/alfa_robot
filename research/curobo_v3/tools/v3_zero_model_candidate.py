import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yourdfpy


ROOT=Path(__file__).resolve().parents[1]
OLD=ROOT/'generated/v3_analytic_071cb95/model/robot_meter_meshes.urdf'
COLLEAGUE=Path('/mnt/mydisk/ALFA/backpack/real2sim_v3.3.1/urdf/robot.urdf')
OFFSETS_DEG={'left':[-90.,15.,0.,0.,90.,0.,90.], 'right':[90.,-15.,0.,0.,-90.,0.,-90.]}
J2_LIMITS_DEG={'left':(-90.,120.),'right':(-120.,90.)}
JOINT_SIGNS=np.array([1.,-1.,1.,1.,1.,-1.,1.])


def legacy_angles(side,new_angles):
    return np.deg2rad(OFFSETS_DEG[side])+JOINT_SIGNS*np.asarray(new_angles)


def build(output,source=OLD):
    output.mkdir(parents=True,exist_ok=True)
    root=ET.parse(source).getroot()
    for joint in root.findall('joint'):
        name=joint.get('name','')
        for side in ('left','right'):
            if name not in [f'{side}_joint{index}' for index in range(1,8)]:continue
            index=int(name.removeprefix(side+'_joint'))-1
            origin=joint.find('origin')
            axis=joint.find('axis')
            vector=np.fromstring(axis.get('xyz'),sep=' ')
            rotation=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' '))*Rotation.from_rotvec(vector*np.deg2rad(OFFSETS_DEG[side][index]))
            matrix=rotation.as_matrix()
            pitch=np.arctan2(-matrix[2,0],np.hypot(matrix[0,0],matrix[1,0]))
            if abs(np.cos(pitch))>1e-12:
                rpy=[np.arctan2(matrix[2,1],matrix[2,2]),pitch,np.arctan2(matrix[1,0],matrix[0,0])]
            else:
                rpy=rotation.as_euler('xyz')
            origin.set('rpy',' '.join(f'{value:.17g}' for value in rpy))
            if JOINT_SIGNS[index]<0:
                axis.set('xyz',' '.join(f'{value:.17g}' for value in -vector))
            if index==1:
                joint.find('limit').set('lower',str(np.deg2rad(J2_LIMITS_DEG[side][0])))
                joint.find('limit').set('upper',str(np.deg2rad(J2_LIMITS_DEG[side][1])))
    for mesh in root.findall('.//mesh'):
        filename=mesh.get('filename')
        if not Path(filename).is_absolute():mesh.set('filename',str((source.parent/filename).resolve()))
    ET.indent(root)
    path=output/'robot_rezero_candidate.urdf'
    ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
    manifest=dict(source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        reference_colleague_urdf=str(COLLEAGUE),offsets_deg=OFFSETS_DEG,
        signs=JOINT_SIGNS.astype(int).tolist(),mapping='q_legacy = offsets + signs * q_candidate; J6 direction reversed without changing zero',
        limits='left J2 [-90,120]; right J2 [-120,90]; J3 +/-180; other numerical limits unchanged',
        status='true coordinate redefinition, not Home change; legacy installation and confirmed legacy tool retained',
        rights_source='mirrored legacy configuration; checked against shoulder/elbow/wrist reflection')
    old=yourdfpy.URDF.load(source,load_meshes=False)
    candidate=yourdfpy.URDF.load(path,load_meshes=False)
    rng=np.random.default_rng(20261018)
    maximum=0.
    for trial in range(100):
        current={name:0. for name in candidate.actuated_joint_names}
        legacy=current.copy()
        for side in ('left','right'):
            angles=rng.uniform(-.4,.4,7)
            current.update({f'{side}_joint{index}':value for index,value in enumerate(angles,1)})
            legacy.update({f'{side}_joint{index}':value for index,value in enumerate(legacy_angles(side,angles),1)})
        old.update_cfg(legacy);candidate.update_cfg(current)
        for link in old.link_map:
            error=np.max(np.abs(old.get_transform(link,old.base_link)-candidate.get_transform(link,candidate.base_link)))
            maximum=max(maximum,float(error))
    assert maximum<1e-6,maximum
    manifest['random_reparameterization_trials']=100
    manifest['maximum_all_link_fk_matrix_error']=maximum
    (output/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    print(json.dumps(manifest,ensure_ascii=False),flush=True)
    return path


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--output-dir',type=Path,default=ROOT/'generated/zero_model_candidate_20261018')
    build(parser.parse_args().output_dir)
