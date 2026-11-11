import json
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from v3_zero_model_candidate import ROOT,OLD,build,COORDINATE_REVISION
from v3_dual_zero_review import old_to_review


def convert(names,values):
    result=np.asarray(values).copy()
    for side in ('left','right'):
        for index in range(1,8):
            name=f'{side}_joint{index}'
            if name not in names:continue
            column=names.index(name)
            if side=='left' and index==5:result[...,column]=(result[...,column]-np.pi+np.pi)%(2*np.pi)-np.pi
            if index==6:result[...,column]*=-1
    return result


def main():
    folder=ROOT/'generated/zero_model_candidate_20261018'
    prior=json.loads((folder/'manifest.json').read_text())
    if prior.get('coordinate_revision')==COORDINATE_REVISION:
        raise ValueError('已迁移，拒绝重复应用零位/方向变换')
    if prior.get('coordinate_revision')!='rezero-left-j5-plus180-20261020':
        raise ValueError('该迁移只接受原20261020坐标版本，不能重复转换后续版本')
    paths={label:folder/name for label,name in [('fixed','fixed.yaml'),('mobile','mobile.yaml')]}
    configs={label:yaml.safe_load(path.read_text()) for label,path in paths.items()}
    fixed=build(folder)
    mobile=build(folder/'mobile',ROOT/'generated/v3_analytic_071cb95/mobile18/model/robot_meter_meshes_mobile18.urdf')
    for label,path in paths.items():
        config=configs[label];kin=config['kinematics'];names=kin['cspace']['joint_names']
        kin['urdf_path']=str(fixed if label=='fixed' else mobile)
        for field in ('nominal_joint_position','retract_config'):
            if kin['cspace'].get(field) is not None:kin['cspace'][field]=convert(names,kin['cspace'][field]).tolist()
        config.pop('coordinate_revision',None)
        path.write_text(yaml.safe_dump(config,sort_keys=False))
    named_path=folder/'named_poses.yaml';named=yaml.safe_load(named_path.read_text())
    for name,values in named['named_poses'].items():
        converted=old_to_review(values);named['named_poses'][name]=converted
    named_path.write_text(yaml.safe_dump(named,sort_keys=False))
    home_path=ROOT/'config/rezero_home.json';home=json.loads(home_path.read_text())
    for side in ('left','right'):
        names=[f'{side}_joint{i}' for i in range(1,8)]
        values=old_to_review(dict(zip(names,np.deg2rad(home[side+'_deg']))))
        home[side+'_deg']=[float(np.rad2deg(values[name])) for name in names]
    home.update(coordinate_revision=COORDINATE_REVISION,
        source='Converted prior Home to approved dual zero/J6 coordinates; physical joint configuration preserved, TCP extended40mm',
        left_j5_policy='Previous left J5=180deg now zero',j6_policy='Both positive axes reversed relative to prior model')
    home_path.write_text(json.dumps(home,ensure_ascii=False,indent=2))
    for label,path in paths.items():
        config=yaml.safe_load(path.read_text());cspace=config['kinematics']['cspace']
        mapping={'updown':home['updown_m']}
        mapping.update({f'{side}_joint{i}':float(np.deg2rad(angle)) for side in ('left','right') for i,angle in enumerate(home[side+'_deg'],1)})
        cspace['default_joint_position']=[mapping.get(name,0.) for name in cspace['joint_names']]
        path.write_text(yaml.safe_dump(config,sort_keys=False))
    names=['updown']+[f'{side}_joint{i}' for side in ('left','right') for i in range(1,8)]
    for label in ('side','top'):
        for suffix in ('_atlas.npz','_contact_roll_atlas.npz'):
            path=folder/(label+suffix)
            if not path.is_file():continue
            with np.load(path,allow_pickle=False) as archive:arrays={key:archive[key] for key in archive.files}
            arrays['joints']=convert(names,arrays['joints']).astype(np.float32)
            arrays['coordinate_revision']=np.array(COORDINATE_REVISION)
            arrays['tcp_goal_revalidation_required']=np.array(True)
            np.savez_compressed(path,**arrays)
    print('ACTIVE_MODEL',COORDINATE_REVISION,'fixed/mobile/home/named poses converted; planner cache gate enabled')


if __name__=='__main__':main()
