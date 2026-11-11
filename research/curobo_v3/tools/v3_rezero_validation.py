import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yourdfpy

from v3_rezero_analytic import RezeroAnalytic
from v3_zero_model_candidate import ROOT,build


def centers(model,side):
    points=[];axes=[]
    for index in range(1,8):
        joint=model.joint_map[f'{side}_joint{index}']
        matrix=model.get_transform(joint.child,model.base_link)
        points.append(matrix[:3,3]);axes.append(matrix[:3,:3]@joint.axis)
    output=[]
    for group in [[0,1,2],[2,3,4],[4,5,6]]:
        projection=[np.eye(3)-np.outer(axes[index],axes[index]) for index in group]
        output.append(np.linalg.lstsq(np.concatenate(projection),np.concatenate([matrix@points[index] for matrix,index in zip(projection,group)]),rcond=None)[0])
    return np.asarray(output)


def main():
    folder=ROOT/'generated/zero_model_candidate_20261018'
    path=build(folder)
    model=yourdfpy.URDF.load(path,load_meshes=False,build_scene_graph=True)
    model.update_cfg(np.zeros(len(model.actuated_joint_names)))
    left=centers(model,'left');right=centers(model,'right')
    mirrored=left*np.array([1.,-1.,1.])
    mirror_error=float(np.linalg.norm(mirrored-right,axis=1).max())
    assert mirror_error<1e-5,mirror_error
    analytic=RezeroAnalytic()
    rng=np.random.default_rng(20261018)
    reports=[]
    for arm,side in enumerate(('left','right')):
        count=0;position_error=0.;rotation_error=0.;failures=[]
        for trial in range(100):
            angles=rng.uniform(.1,.9,7)*(analytic.upper[arm]-analytic.lower[arm])+analytic.lower[arm]
            values={name:0. for name in model.actuated_joint_names}
            values.update({f'{side}_joint{index}':value for index,value in enumerate(angles,1)})
            model.update_cfg(values)
            carriage=np.linalg.inv(model.get_transform('arm_carriage',model.base_link))
            target=carriage@model.get_transform(side+'_tool0',model.base_link)
            solutions=analytic.solve(arm,target,analytic.swivel(arm,angles),angles)
            if not len(solutions):
                failures.append(trial);continue
            actuals=[]
            for solution in solutions:
                values.update({f'{side}_joint{index}':value for index,value in enumerate(solution,1)})
                model.update_cfg(values)
                actual=carriage@model.get_transform(side+'_tool0',model.base_link)
                actuals.append((float(np.linalg.norm(target[:3,3]-actual[:3,3])),float((Rotation.from_matrix(target[:3,:3]).inv()*Rotation.from_matrix(actual[:3,:3])).magnitude())))
            best=min(actuals,key=lambda value:value[0]+value[1])
            position_error=max(position_error,best[0]);rotation_error=max(rotation_error,best[1]);count+=1
        reports.append(dict(side=side,passed=count,trials=100,failed_trials=failures,max_position_error_mm=position_error*1000,max_rotation_error_deg=float(np.rad2deg(rotation_error))))
    result=dict(arms=reports,right_mirror_shoulder_elbow_wrist_error_mm=mirror_error*1000,
        scope='Only re-zero candidate with legacy installation/tool; not colleague v3.3.1 IK, collision, or task planning validation')
    (folder/'analytic_validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print('VALIDATION',json.dumps(result,ensure_ascii=False),flush=True)
    assert all(report['passed']==100 and report['max_position_error_mm']<.002 and report['max_rotation_error_deg']<.001 for report in reports)


if __name__=='__main__':main()
