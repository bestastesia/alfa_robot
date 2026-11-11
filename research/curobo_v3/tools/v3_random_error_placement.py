import numpy as np
from scipy.spatial.transform import Rotation


def navigation_and_box_goals(held_boxes, references, push_m=.8):
    turn=Rotation.from_euler('z',np.pi).as_matrix()
    required=[]
    for side,held in held_boxes.items():
        pushed=held[:3,3].copy()
        pushed[0]+=push_m
        required.append((np.asarray(references[side]['position'])-turn@pushed)[:2])
    parking=np.mean(required,axis=0)
    if np.max(np.linalg.norm(np.asarray(required)-parking,axis=1))>.002:
        raise ValueError('左右目标无法共享同一既定停车位，停止而不平均掩盖差异')
    base=np.eye(4);base[:3,:3]=turn;base[:2,3]=parking
    goals={}
    for side,held in held_boxes.items():
        heading=np.arctan2(held[1,0],held[0,0])
        world_box=np.eye(4)
        world_box[:3,:3]=turn@Rotation.from_euler('z',heading).as_matrix()
        world_box[:3,3]=references[side]['position']
        goals[side]=np.linalg.inv(base)@world_box
    return base,goals
