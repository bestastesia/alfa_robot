import numpy as np

from v3_box_geometry import CONTAINER_HALF_WIDTH_M, DEFAULT_BOX_DIMS, box_dims
from v3_box_geometry import wall_center as _wall_center


def wall_center(front, distance, box_id, dims=DEFAULT_BOX_DIMS):
    return _wall_center(front, distance, box_id, dims)


def row_obstacles(front, boxes, dims=DEFAULT_BOX_DIMS):
    depth, width, height = box_dims(dims)
    centers = {side: wall_center(front, 0.9, box_id, dims) for side, box_id in boxes.items()}
    for center in centers.values():
        center[2] += 0.001
    lowest = min(centers.values(), key=lambda center: center[2])
    intervals = [(-CONTAINER_HALF_WIDTH_M, CONTAINER_HALF_WIDTH_M)]
    for center in centers.values():
        if abs(center[2] - lowest[2]) > 1e-6:
            continue
        remaining = []
        cut_lower, cut_upper = center[1] - width / 2, center[1] + width / 2
        for lower, upper in intervals:
            if cut_upper <= lower or cut_lower >= upper:
                remaining.append((lower, upper))
            else:
                if cut_lower > lower:
                    remaining.append((lower, cut_lower))
                if cut_upper < upper:
                    remaining.append((cut_upper, upper))
        intervals = remaining
    obstacles = [dict(center=[lowest[0], (lower + upper) / 2, lowest[2]],
                      dimensions=[depth, upper - lower, height]) for lower, upper in intervals if upper - lower > 1e-8]
    bottom_height = lowest[2] - height / 2
    if bottom_height - 0.001 > 1e-8:
        obstacles.append(dict(center=[lowest[0], 0.0, (bottom_height + 0.001) / 2],
                              dimensions=[depth, 2 * CONTAINER_HALF_WIDTH_M, bottom_height - 0.001]))
    return centers, obstacles


def grasp_transform(side, top, dims=DEFAULT_BOX_DIMS):
    depth, _, height = box_dims(dims)
    if not top:
        matrix = np.eye(4)
        matrix[:3, :3] = [[0, 0, 1], [0, -1, 0], [1, 0, 0]]
        matrix[2, 3] = depth / 2
        return matrix
    matrix = np.eye(4)
    matrix[:3, :3] = np.diag([1.0, -1.0, -1.0])
    matrix[2, 3] = height / 2
    return matrix
