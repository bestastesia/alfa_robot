from dataclasses import dataclass
from itertools import product

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class GraspInput:
    side: str
    mode: str
    tcp_in_footprint: np.ndarray


def tool_to_box(mode, dimensions=(.3, .4, .4)):
    matrix = np.eye(4)
    if mode == 'side':
        matrix[:3, :3] = [[0, 0, 1], [0, -1, 0], [1, 0, 0]]
        matrix[2, 3] = dimensions[0]/2
    elif mode == 'top':
        matrix[:3, :3] = np.diag([1., -1., -1.])
        matrix[2, 3] = dimensions[2]/2
    else:
        raise ValueError(f'Unknown grasp mode: {mode}')
    return matrix


def box_corners(matrix, dimensions=(.3, .4, .4)):
    vertices = np.asarray(list(product((-.5, .5), repeat=3)))*dimensions
    return vertices@matrix[:3, :3].T+matrix[:3, 3]


def reconstruct_scene(grasps, world_from_footprint, dimensions=(.3, .4, .4), lateral_bounds=(-1.2, 1.2), gap=.002):
    if not grasps:
        raise ValueError('At least one grasp is required')
    boxes = {grasp.side: world_from_footprint@grasp.tcp_in_footprint@tool_to_box(grasp.mode, dimensions) for grasp in grasps}
    bounds = {side: (corners.min(0), corners.max(0)) for side, matrix in boxes.items() for corners in [box_corners(matrix, dimensions)]}
    minimum_bottom = min(lower[2] for lower, upper in bounds.values())
    lowest_center = min(matrix[2, 3] for matrix in boxes.values())
    working_top = max(bounds[side][1][2] for side, matrix in boxes.items() if matrix[2, 3] <= lowest_center+.02)
    front = min(lower[0] for lower, upper in bounds.values())
    rear = max(upper[0] for lower, upper in bounds.values())
    intervals = [lateral_bounds]
    for lower, upper in bounds.values():
        opening = (lower[1]-gap, upper[1]+gap)
        updated = []
        for begin, end in intervals:
            if opening[1] <= begin or opening[0] >= end:
                updated.append((begin, end))
            else:
                if begin < opening[0]: updated.append((begin, opening[0]))
                if end > opening[1]: updated.append((opening[1], end))
        intervals = updated
    fills = []
    def cuboid(lower, upper, name):
        if np.min(upper-lower) > 1e-8:
            fills.append(dict(name=name, center=((lower+upper)/2).tolist(), dimensions=(upper-lower).tolist()))
    cuboid(np.array([front, lateral_bounds[0], 0.]), np.array([rear, lateral_bounds[1], max(0.,minimum_bottom)]), 'lower_solid')
    for index, (begin, end) in enumerate(intervals):
        cuboid(np.array([front, begin, max(0.,minimum_bottom)]), np.array([rear, end, working_top]), f'working_solid_{index}')
    outward_world = np.array([-1., 0., 0.])
    return dict(boxes=boxes, fills=fills, bounds=bounds, outward_world=outward_world,
                outward_footprint=world_from_footprint[:3, :3].T@outward_world)


def random_parking(rng):
    radius=.05*np.sqrt(rng.random())
    angle=rng.uniform(-np.pi,np.pi)
    yaw=rng.uniform(-5.,5.)
    matrix=np.eye(4)
    matrix[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix()
    matrix[:2,3]=radius*np.array([np.cos(angle),np.sin(angle)])
    return matrix, yaw
