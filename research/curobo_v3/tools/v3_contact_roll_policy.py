import hashlib
import json

import numpy as np
from scipy.spatial.transform import Rotation

from v3_task_scene import grasp_transform as legacy_grasp_transform


TOOL_ROLL=np.diag([-1.,-1.,1.,1.])


def aligned_tool_pose(pose):
    rotation=Rotation.from_quat(np.roll(pose['quaternion'],-1)).as_matrix()
    return dict(pose,quaternion=np.roll(Rotation.from_matrix(rotation@TOOL_ROLL[:3,:3]).as_quat(),1).tolist())


def aligned_grasp_transform(side,top):
    return TOOL_ROLL@legacy_grasp_transform(side,top)


def aligned_atlas(source,destination):
    identity=hashlib.sha256(source.read_bytes()).hexdigest()
    provenance=destination.with_suffix('.source.json')
    if destination.is_file() and provenance.is_file():
        if json.loads(provenance.read_text()).get('source_sha256')==identity:
            return destination
    with np.load(source,allow_pickle=False) as stored:
        arrays={key:stored[key] for key in stored.files}
    values=arrays['joints'].copy()
    values[:,[7,14]]=(values[:,[7,14]]+2*np.pi)%(2*np.pi)-np.pi
    arrays['joints']=values
    np.savez_compressed(destination,**arrays)
    provenance.write_text(json.dumps(dict(source_sha256=identity,tool_roll_deg=180)))
    return destination
