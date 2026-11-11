import numpy as np
import yourdfpy


def unit(vector, fallback):
    length = np.linalg.norm(vector, axis=-1, keepdims=True)
    return np.where(length > 1e-8, vector / np.maximum(length, 1e-8), fallback)


class ArmReferenceGeometry:
    def __init__(self, urdf, wrist_geometry):
        self.model = yourdfpy.URDF.load(urdf, load_meshes=False)
        self.wrist_geometry = wrist_geometry
        self.joints = {joint.name: joint for joint in self.model.robot.joints}

    def describe(self, values, names, side):
        self.model.update_cfg(dict(zip(names, values)))
        points, axes = [], []
        for index in (1, 2, 3):
            joint = self.joints[f'{side}_joint{index}']
            transform = self.model.get_transform(joint.child, self.model.base_link)
            points.append(transform[:3, 3])
            axes.append(transform[:3, :3] @ joint.axis)
        axes = np.asarray(axes)
        projections = np.eye(3)[None] - np.einsum('bi,bj->bij', axes, axes)
        shoulder = np.linalg.solve(projections.sum(0), np.einsum('bij,bj->i', projections, points))
        tool = self.model.get_transform(side + '_tool0', self.model.base_link)
        wrist = (tool @ np.r_[self.wrist_geometry['arms'][side]['center_tool'], 1])[:3]
        joint = self.joints[side + '_joint4']
        transform = self.model.get_transform(joint.child, self.model.base_link)
        axis = transform[:3, :3] @ joint.axis
        elbow = transform[:3, 3] + axis * np.dot(wrist - transform[:3, 3], axis)
        direction = unit(wrist - shoulder, np.array([1., 0., 0.]))
        bend = elbow - shoulder - direction * np.dot(elbow - shoulder, direction)
        bend = unit(bend, np.array([0., 0., 1.]))
        return dict(shoulder=shoulder, wrist=wrist, elbow=elbow, bend=bend,
                    upper_length=float(np.linalg.norm(elbow - shoulder)),
                    forearm_length=float(np.linalg.norm(wrist - elbow)))

    def context(self, start, goal, names):
        result = {}
        for side in ('left', 'right'):
            initial = self.describe(start, names, side)
            target = self.describe(goal, names, side)
            upper = (initial['upper_length'] + target['upper_length']) / 2
            forearm = (initial['forearm_length'] + target['forearm_length']) / 2
            result[side] = dict(shoulders=np.stack([initial['shoulder'], target['shoulder']]),
                                bends=np.stack([initial['bend'], target['bend']]),
                                upper_length=upper, forearm_length=forearm,
                                preferred_radius=float(np.hypot(upper, forearm)))
        return result


def virtual_arm(states, parameter, context):
    phase = 10 * parameter ** 3 - 15 * parameter ** 4 + 6 * parameter ** 5
    shoulder = (1 - phase[:, None]) * context['shoulders'][0] + phase[:, None] * context['shoulders'][1]
    wrist = states['centers']
    displacement = wrist - shoulder
    radius = np.linalg.norm(displacement, axis=1)
    direction = unit(displacement, np.array([1., 0., 0.]))
    preferred_bend = (1 - phase[:, None]) * context['bends'][0] + phase[:, None] * context['bends'][1]
    bend = preferred_bend - np.sum(preferred_bend * direction, axis=1)[:, None] * direction
    fallback = np.cross(direction, np.array([0., 0., 1.]))
    fallback = unit(fallback, np.array([0., 1., 0.]))
    bend = unit(bend, fallback)
    upper, forearm = context['upper_length'], context['forearm_length']
    distance = np.clip(radius, abs(upper - forearm) + 1e-5, upper + forearm - 1e-5)
    axial = (upper ** 2 - forearm ** 2 + distance ** 2) / (2 * distance)
    height = np.sqrt(np.maximum(upper ** 2 - axial ** 2, 0.))
    elbow = shoulder + axial[:, None] * direction + height[:, None] * bend
    forearm_direction = unit(wrist - elbow, direction)
    tool_axis = states['rotations'].apply(np.tile([0., 0., 1.], (len(parameter), 1)))
    cosine = np.sum(forearm_direction * tool_axis, axis=1)
    envelope = np.sin(np.pi * parameter) ** 2
    radial = float(np.mean(envelope * (radius - context['preferred_radius']) ** 2))
    bending = float(np.mean(envelope * np.maximum(.15 - cosine, 0.) ** 2))
    reach = float(np.mean(np.maximum(radius - upper - forearm, 0.) ** 2))
    return dict(score=4. * radial + .15 * bending + 10. * reach, radial_cost=radial,
                wrist_bend_cost=bending, reach_cost=reach, radius=radius, elbows=elbow,
                shoulders=shoulder, forearm_cosine=cosine)


def guided_midpoint(start, end, context, strength):
    middle = (start + end) / 2
    shoulder = context['shoulders'].mean(0)
    delta = middle - shoulder
    direction = unit(delta, np.array([1., 0., 0.]))
    correction = np.clip(context['preferred_radius'] - np.linalg.norm(delta), -.22, .22)
    return middle + strength * correction * direction
