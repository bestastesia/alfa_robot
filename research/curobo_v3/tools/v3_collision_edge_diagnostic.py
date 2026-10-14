import numpy as np
import torch
from scipy.spatial.transform import Rotation
from curobo.types import JointState
from curobo.scene import Cuboid, Scene
from curobo_core.adapter import matrix_pose
from v3_search_helpers import audit_state
from v3_wall_ik_benchmark import ACTIVE_JOINTS
from v3_task_scene import grasp_transform


def record_collision_edge(planner, checker, validity, dense, valid, endpoint, names, fraction):
    first_bad = int((~valid).nonzero()[0].item())
    values = torch.tensor(dense[first_bad:first_bad + 1], device='cuda', dtype=torch.float32)
    joint_state = JointState.from_position(values[None], joint_names=ACTIVE_JOINTS)
    state = checker.kinematics.compute_kinematics(joint_state)
    spheres = state.robot_spheres.reshape(-1, 4).detach().cpu().numpy()
    config = checker.kinematics.config.kinematics_config
    links = [''] * len(spheres)
    for link in config.link_name_to_idx_map:
        for index in config.get_sphere_index_from_link_name(link).cpu().tolist():
            links[index] = link
    roles = links.copy()
    for side in ('left', 'right'):
        pose = state.tool_poses.to_dict()[side + '_tool0']
        tool = np.eye(4)
        tool[:3, :3] = Rotation.from_quat(np.roll(pose.quaternion.detach().cpu().numpy().reshape(4), -1)).as_matrix()
        tool[:3, 3] = pose.position.detach().cpu().numpy().reshape(3)
        box = tool @ grasp_transform(side, planner.top_grasp)
        centers = np.asarray(planner.box_fit['centers']) @ box[:3, :3].T + box[:3, 3]
        distances = np.linalg.norm(spheres[:, None, :3] - centers[None], axis=-1)
        closest = distances.argmin(axis=1)
        for index in np.where((distances.min(axis=1) < 1e-5) &
                              (np.abs(spheres[:, 3] - np.asarray(planner.box_fit['radii'])[closest]) < 1e-6))[0]:
            roles[index] = ('左' if side == 'left' else '右') + '手附着箱'
    scene = planner.scene(planner.selected_task)
    cuboids = list(scene.cuboid or [])
    for item in scene.mesh or []:
        vertices = np.asarray(item.vertices)
        lower, upper = vertices.min(0), vertices.max(0)
        if len(np.unique(vertices, axis=0)) != 8 or not np.all(np.isclose(vertices, lower) | np.isclose(vertices, upper)):
            raise ValueError('mesh diagnostic expects the existing rectangular conveyor STL')
        rotation = Rotation.from_quat(np.roll(item.pose[3:7], -1)).as_matrix()
        center = rotation @ ((lower + upper) / 2) + item.pose[:3]
        cuboids.append(Cuboid(name=item.name, dims=(upper - lower).tolist(), pose=[*center, *item.pose[3:7]]))
    audit = audit_state(checker.kinematics, joint_state, Scene(cuboid=cuboids))
    checker.setup_batch_tensors(1, 1)
    checker.collision_constraint.update_num_spheres(len(spheres), batch_size=1, horizon=1)
    components = {'self': float(checker.get_self_collision(state.robot_spheres).sum()),
                  'environment': float(checker.collision_constraint.forward(state).sum()),
                  'joint_bounds': float(checker.get_bound(values[None]).sum()),
                  'base_internal': float(checker.base_cuboid_layer.cost(state, hard=True).sum())}
    ground = [int(index) for index, sphere in enumerate(spheres) if index not in validity.ground_exempt_sphere_indices
              and sphere[3] > 0 and sphere[2] - sphere[3] < validity.ground_z]
    internal = []
    for box in checker.base_cuboid_layer.scene.cuboid:
        rotation = Rotation.from_quat(np.roll(box.pose[3:7], -1)).as_matrix()
        centers = (spheres[:, :3] - box.pose[:3]) @ rotation
        delta = np.abs(centers) - np.asarray(box.dims) / 2
        distance = np.linalg.norm(np.maximum(delta, 0), axis=1) + np.minimum(delta.max(axis=1), 0)
        penetration = spheres[:, 3] - distance
        for index in checker.base_cuboid_layer.indices.cpu().tolist():
            if spheres[index, 3] > 0 and penetration[index] > 1e-6:
                internal.append({'link': links[index], 'obstacle': box.name, 'sphere_index': int(index),
                                 'penetration_mm': float(penetration[index] * 1000)})
    audit['base_internal_collisions'] = internal
    audit['ground_sphere_indices'] = ground
    audit['native_components'] = components
    base = np.eye(4)
    base[:3, :3] = Rotation.from_euler('z', endpoint[names.index('base_yaw')]).as_matrix()
    base[:2, 3] = endpoint[[names.index('base_x'), names.index('base_y')]]
    base[0, 3] += planner.base_advance
    spheres[:, :3] = spheres[:, :3] @ base[:3, :3].T + base[:3, 3]
    highlighted = set(ground)
    for pair in audit['self_collisions']:
        highlighted.update(pair['sphere_indices'])
    for item in audit['world_collisions'] + internal:
        highlighted.add(item['sphere_index'])
    mobile = np.repeat(endpoint[None], len(dense), axis=0)
    mobile[:, [names.index(name) for name in ACTIVE_JOINTS]] = dense
    mobile[:, names.index('base_x')] += planner.base_advance
    obstacles = []
    for box in cuboids + list(checker.base_cuboid_layer.scene.cuboid):
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat(np.roll(box.pose[3:7], -1)).as_matrix()
        matrix[:3, 3] = box.pose[:3]
        pose = matrix_pose(base @ matrix)
        obstacles.append({'name': box.name, 'position': pose.position, 'quaternion_wxyz': pose.quaternion_wxyz,
                          'dimensions': box.dims})
    return {'frames': mobile.tolist(), 'first_bad_index': first_bad, 'valid': valid.cpu().tolist(),
            'cartesian_fraction': float(fraction), 'audit': audit, 'spheres_map': spheres.tolist(),
            'sphere_links': links, 'sphere_roles': roles, 'highlighted_sphere_indices': sorted(highlighted), 'obstacles_map': obstacles,
            'joint_delta_deg': np.rad2deg(dense[-1, 1:] - dense[0, 1:]).tolist(),
            'purpose': 'rejected edge visualization only; never executable'}
