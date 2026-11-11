import io
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yourdfpy


ZERO_DEG={'left':[0.,0.,0.,0.,180.,0.,0.],'right':[0.,0.,0.,0.,0.,0.,0.]}
REVIEW_REVISION='dual-zero-discrete-only-tool191-20261024'


def old_to_review(values):
    result=dict(values)
    for side in ('left','right'):
        for index,offset in enumerate(ZERO_DEG[side],1):
            name=f'{side}_joint{index}'
            value=(values.get(name,0.)-np.deg2rad(offset))*(-1 if index==6 else 1)
            if index==5:value=(value+np.pi)%(2*np.pi)-np.pi
            result[name]=float(value)
    return result


def review_definition(path):
    definition=ET.parse(path)
    if definition.getroot().get('alfa_coordinate_revision')==REVIEW_REVISION:
        return definition
    for side in ('left','right'):
        for index,offset in enumerate(ZERO_DEG[side],1):
            joint=definition.find(f"joint[@name='{side}_joint{index}']")
            origin=joint.find('origin');axis=joint.find('axis')
            vector=np.fromstring(axis.get('xyz'),sep=' ')
            if offset:
                rotation=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' '))*Rotation.from_rotvec(vector*np.deg2rad(offset))
                matrix=rotation.as_matrix()
                pitch=np.arctan2(-matrix[2,0],np.hypot(matrix[0,0],matrix[1,0]))
                rpy=[np.arctan2(matrix[2,1],matrix[2,2]),pitch,np.arctan2(matrix[1,0],matrix[0,0])]
                origin.set('rpy',' '.join(f'{number:.17g}' for number in rpy))
            sign=-1 if index==6 else 1
            axis.set('xyz',' '.join(f'{number:.17g}' for number in vector*sign))
            limit=joint.find('limit')
            if index!=5:
                bounds=sorted((float(limit.get(key))-np.deg2rad(offset))*sign for key in ('lower','upper'))
                limit.set('lower',str(bounds[0]));limit.set('upper',str(bounds[1]))
        definition.find(f"joint[@name='{side}_tool0_fixed']/origin").set('xyz','0 0 0.191')
    definition.getroot().set('alfa_coordinate_revision',REVIEW_REVISION)
    return definition


def load_review(path,load_meshes=True):
    definition=review_definition(path)
    return yourdfpy.URDF.load(io.BytesIO(ET.tostring(definition.getroot())),load_meshes=load_meshes,filename_handler=lambda fname:fname)
