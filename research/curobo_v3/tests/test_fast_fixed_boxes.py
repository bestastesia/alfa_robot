from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import torch
import yaml

from curobo.scene import Cuboid, Scene
from v3_fast_path import fit_path
from v3_fixed_body_boxes import FixedBodyWorld, WorldMarginChecker, disable_fixed_body_spheres, obb_penetration


def box(center, dims=(2., 2., 2.)):
    return Cuboid(name='box', dims=list(dims), pose=[*center, 1., 0., 0., 0.])


def test_box_world_supports_translation_yaw_and_scene_update():
    layer = FixedBodyWorld(Scene(cuboid=[box([0., 0., 0.], (4., .4, .4))]),
                           Scene(cuboid=[box([0., 1.5, 0.], (.4, .4, .4))]),
                           ['base_x', 'base_y', 'base_yaw'], device='cpu')
    values = torch.tensor([[[0., 0., 0.], [0., 0., np.pi / 2], [4., 0., np.pi / 2]]])
    assert (layer.cost(values) > 0).tolist() == [[False, True, False]]
    layer.update(Scene(cuboid=[box([20., 20., 20.])]))
    assert torch.equal(layer.cost(values), torch.zeros(1, 3))


def test_box_contact_tolerance_and_percentage_margin():
    layer = FixedBodyWorld(Scene(cuboid=[box([0., 0., 0.])]),
                           Scene(cuboid=[box([2.05, 0., 0.])]), ['updown'], device='cpu')
    values = torch.zeros(1, 3, 1)
    assert not bool((layer.cost(values) > 0).any())
    assert bool((layer.cost(values, scale=1.1) > 0).all())
    layer.update(Scene(cuboid=[box([1.9995, 0., 0.])]))
    assert bool((layer.cost(values) > 0).all())
    assert not bool((layer.cost(values, tolerance=.001) > 0).any())


def test_obb_sat_has_edge_cross_axes():
    generator = np.random.default_rng(10)
    for _ in range(30):
        centers = generator.uniform(-2, 2, (2, 3))
        rotations = Rotation.random(2, random_state=generator).as_matrix()
        halves = generator.uniform(.1, 1., (2, 3))
        axes = list(rotations[0].T) + list(rotations[1].T)
        axes += [np.cross(first, second) for first in rotations[0].T for second in rotations[1].T]
        penetrations = []
        for axis in axes:
            if np.linalg.norm(axis) < 1e-8:continue
            axis = axis / np.linalg.norm(axis)
            radii = sum(np.abs(axis @ rotation) @ half for rotation, half in zip(rotations, halves))
            penetrations.append(radii - abs(axis @ (centers[0] - centers[1])))
        result = obb_penetration(*[torch.tensor(array) for array in
            (centers[:1], rotations[:1], halves[:1], centers[1:], rotations[1:], halves[1:])])
        assert float(result[0, 0]) == pytest.approx(max(0., min(penetrations)), abs=1e-7)


def test_fixed_body_slots_disabled_but_carriage_and_payload_unchanged():
    root = Path(__file__).resolve().parents[1]
    robot = yaml.safe_load((root / 'generated/zero_model_candidate_20261018/fixed.yaml').read_text())
    changed = disable_fixed_body_spheres(robot)
    original = robot['kinematics']['collision_spheres']
    spheres = changed['kinematics']['collision_spheres']
    assert len(spheres['base_link']) == len(original['base_link']) == 71
    assert all(sphere['radius'] < 0 for sphere in spheres['base_link'])
    assert sum(sphere['radius'] > 0 for sphere in spheres['arm_carriage']) == 20
    assert {key: value for key, value in spheres.items() if key != 'base_link'} == {
        key: value for key, value in original.items() if key != 'base_link'}
    tree = ET.parse(root / 'generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf')
    parents = {joint.find('child').get('link'): joint.find('parent').get('link') for joint in tree.findall('joint')}
    ancestors = []
    link = 'arm_carriage'
    while link in parents:
        link = parents[link]
        ancestors.append(link)
    assert ancestors == ['base_link', 'base_footprint']


def test_sphere_world_margin_keeps_disabled_slots_and_internal_checker():
    captured = []
    checker = SimpleNamespace(get_collision_constraint=lambda query: captured.append(query) or query,
                              get_self_collision=lambda query: query)
    proxy = WorldMarginChecker(checker, 1.1, .001)
    query = torch.tensor([[[[0., 0., 0., .099], [0., 0., 0., -1.]]]])
    result = proxy.get_collision_constraint(query)
    assert float(result[0, 0, 0, 3]) == pytest.approx(.109)
    assert float(result[0, 0, 1, 3]) == -1.
    assert torch.equal(proxy.get_self_collision(query), query)
    assert float(query[0, 0, 0, 3]) == pytest.approx(.099)


def test_constant_native_path_and_endpoints():
    positions = np.tile(np.arange(15), (4, 1)).astype(float)
    controls, start, goal, report, curve = fit_path({'positions': positions}, 32)
    assert controls.shape == (32, 15)
    assert np.allclose(curve([0., 1.]), positions[[0, -1]])
    assert np.allclose(start, goal)
    assert report['arm_max_deviation_deg'] < 1e-8
