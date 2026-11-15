"""Batched float64 URDF forward kinematics for serial tool chains (same joint math as yourdfpy)."""
import numpy as np


def axis_rotations(axis, angles):
    """Rodrigues rotations about one unit axis, as trimesh.transformations.rotation_matrix."""
    x, y, z = np.asarray(axis, dtype=float)/np.linalg.norm(axis)
    c, s = np.cos(angles), np.sin(angles)
    t = 1.-c
    return np.stack((c+x*x*t, x*y*t-z*s, x*z*t+y*s,
                     y*x*t+z*s, c+y*y*t, y*z*t-x*s,
                     z*x*t-y*s, z*y*t+x*s, c+z*z*t), -1).reshape(len(angles), 3, 3)


class BatchToolFk:
    """Base-to-frame transforms for many joint rows at once; joints outside `joint_names` read 0."""
    def __init__(self, urdf, frames, joint_names):
        child = {joint.child: joint for joint in urdf.joint_map.values()}
        index = {name: position for position, name in enumerate(joint_names)}
        self.chains = {}
        for frame in frames:
            chain, link = [], frame
            while link != urdf.base_link:
                joint = child[link]
                if joint.mimic is not None or joint.type not in ('fixed', 'revolute', 'continuous', 'prismatic'):
                    raise ValueError(f'unsupported joint on {frame} chain: {joint.name}')
                chain.append((np.eye(4) if joint.origin is None else np.asarray(joint.origin, dtype=float),
                              joint.type, None if joint.axis is None else np.asarray(joint.axis, dtype=float),
                              index.get(joint.name)))
                link = joint.parent
            self.chains[frame] = chain[::-1]

    def transforms(self, rows, frame):
        rows = np.asarray(rows, dtype=float).reshape(len(rows), -1)
        output = np.repeat(np.eye(4)[None], len(rows), axis=0)
        for origin, kind, axis, column in self.chains[frame]:
            output = output@origin
            if kind == 'fixed':
                continue
            value = rows[:, column] if column is not None else np.zeros(len(rows))
            motion = np.repeat(np.eye(4)[None], len(rows), axis=0)
            if kind == 'prismatic':
                motion[:, :3, 3] = value[:, None]*axis
            else:
                motion[:, :3, :3] = axis_rotations(axis, value)
            output = output@motion
        return output
