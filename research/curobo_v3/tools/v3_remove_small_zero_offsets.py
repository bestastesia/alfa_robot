import json

import numpy as np
import yaml

from v3_zero_model_candidate import ROOT,build,COORDINATE_REVISION


def main():
    folder=ROOT/'generated/zero_model_candidate_20261018'
    prior=json.loads((folder/'manifest.json').read_text())
    if prior['coordinate_revision']!='dual-zero-j6-positive-up-tool191-20261024':
        raise ValueError('仅允许从误含J4偏置版本迁移一次')
    for label,path in [('fixed',folder/'fixed.yaml'),('mobile',folder/'mobile.yaml')]:
        config=yaml.safe_load(path.read_text());cspace=config['kinematics']['cspace'];index=cspace['joint_names'].index('left_joint4')
        for field in ('default_joint_position','nominal_joint_position','retract_config'):
            if cspace.get(field) is not None:cspace[field][index]-=float(np.deg2rad(1.3))
        path.write_text(yaml.safe_dump(config,sort_keys=False))
    named_path=folder/'named_poses.yaml';named=yaml.safe_load(named_path.read_text())
    for pose in named['named_poses'].values():
        if 'left_joint4' in pose:pose['left_joint4']-=float(np.deg2rad(1.3))
    named_path.write_text(yaml.safe_dump(named,sort_keys=False))
    home_path=ROOT/'config/rezero_home.json';home=json.loads(home_path.read_text());home['left_deg'][3]-=1.3
    home.update(coordinate_revision=COORDINATE_REVISION,source='Discrete-only zero convention; accidental left J4 -1.3deg removed; no fitted zero offsets')
    home_path.write_text(json.dumps(home,ensure_ascii=False,indent=2))
    for label in ('side','top'):
        for suffix in ('_atlas.npz','_contact_roll_atlas.npz'):
            path=folder/(label+suffix)
            with np.load(path,allow_pickle=False) as archive:arrays={key:archive[key] for key in archive.files}
            arrays['joints'][:,4]-=np.deg2rad(1.3)
            arrays['coordinate_revision']=np.array(COORDINATE_REVISION)
            arrays['tcp_goal_revalidation_required']=np.array(True)
            np.savez_compressed(path,**arrays)
    build(folder)
    build(folder/'mobile',ROOT/'generated/v3_analytic_071cb95/mobile18/model/robot_meter_meshes_mobile18.urdf')
    policy_path=ROOT/'tools/ipc_left_prefix/coordinate_policy.json';policy=json.loads(policy_path.read_text())
    policy.update(motor_minus_model_deg=[0]*7,required_coordinate_revision=COORDINATE_REVISION,
        calibration_name='User requested removal of all fitted small-angle offsets',
        calibration_source='2026-10-24 user correction; former contact-fit candidate is historical only',
        execution_mapping_verified=False,
        legacy_replay_blocked='新模型零位/J6/tool191与旧实机坐标策略未重新验收，禁止发送')
    policy_path.write_text(json.dumps(policy,ensure_ascii=False,indent=2))
    print('REMOVED unintended J4 zero and local fitted motor offsets; no IPC deployment')


if __name__=='__main__':main()
