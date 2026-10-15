import copy
import time
from dataclasses import asdict

from curobo.collision_checking import RobotCollisionChecker, RobotCollisionCheckerCfg
from curobo_core.adapter import pose_matrix
from curobo_core.cache import resource_key
from curobo_core.scene import digest
from v3_base_cuboid_layer import BaseCuboidLayer
from v3_batched_loaded_search import loaded_robot
from v3_cuboid_full_cycle_planner import MobileCuboidLayer
from v3_priority_contact_planner import PriorityContactPlanner
from v3_research_assets import ROOT


class PersistentCollisionPlanner(PriorityContactPlanner):
    def __init__(self, args):
        super().__init__(args)
        self.persistent_checkers = {}
        self.checker_resource_events = []

    def warmup_resources(self):
        original_task = dict(self.selected_task)
        original_snapshot = self.snapshot
        original_targets = copy.deepcopy(self.target_poses)
        original_flags = (self.top_grasp, self.base_advance, self.planner_kind)
        for boxes in [{'left': 22, 'right': 17}, {'left': 9, 'right': 1}]:
            self.configure_task(boxes)
            for payload, mobile in [(False, False), (True, False), (False, True)]:
                checker = self.collision_checker(boxes, payload, mobile)
                names = checker.kinematics.joint_names
                import torch
                from v3_batched_loaded_search import GpuValidity
                values = torch.tensor([[self.home.get(name, 0.0) for name in names]], device='cuda', dtype=torch.float32)
                GpuValidity(checker, check_ground=True, joint_names=names).mask(values)
        self.selected_task = original_task
        self.snapshot = original_snapshot
        self.target_poses = original_targets
        self.top_grasp, self.base_advance, self.planner_kind = original_flags

    def prepare_task(self, boxes):
        key = resource_key(self.snapshot, boxes, self.box_fit)
        if key != self._cached_task:
            self._cached_solver = None
            self._cached_task = key

    def collision_checker(self, boxes, payload=False, mobile=False):
        self.prepare_task(boxes)
        scene = self.scene(boxes, mobile=mobile)
        if scene.mesh or len(scene.cuboid or []) > 40:
            raise ValueError('Persistent checker supports the current cuboid scene capacity only')
        robot = self.mobile_robot if mobile else self.robot
        attachments = self.task_attachments(boxes) if payload else ()
        attachment_key = tuple((item.parent_link, item.dimensions_m, asdict(item.tool_to_object), item.touch_links) for item in attachments)
        topology = digest((robot, self.box_fit if payload else None, attachment_key, mobile))
        geometry = digest([(item.name, item.pose, item.dims) for item in scene.cuboid or []])
        began = time.perf_counter()
        created = topology not in self.persistent_checkers
        if created:
            configured = loaded_robot(robot, self.box_fit, attachments=attachments) if payload else copy.deepcopy(robot)
            checker = RobotCollisionChecker(RobotCollisionCheckerCfg.load_from_config(
                robot_config=configured, scene_model=scene, n_cuboids=40, n_meshes=0,
                collision_activation_distance=0.0))
            definition = ROOT / 'generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json'
            if mobile:
                local = self.collision_checker(boxes, payload, False)
                layer = BaseCuboidLayer(local, self.robot, definition)
                checker.base_cuboid_layer = MobileCuboidLayer(checker, layer, list(robot['kinematics']['cspace']['joint_names']))
            else:
                checker.base_cuboid_layer = BaseCuboidLayer(checker, self.robot, definition)
            checker.base_mesh_layer = checker.base_cuboid_layer
            self.persistent_checkers[topology] = {'checker': checker, 'geometry': geometry}
        resource = self.persistent_checkers[topology]
        checker = resource['checker']
        updated = not created and resource['geometry'] != geometry
        if updated:
            resource['geometry'] = None
            checker.update_world(scene)
            resource['geometry'] = geometry
            layer = checker.base_cuboid_layer
            if hasattr(layer, 'query_cache'):
                layer.query_cache.clear()
        checker.task_tool_to_box = {item.parent_link.removesuffix('_tool0'): pose_matrix(item.tool_to_object) for item in self.task_attachments(boxes)}
        self.checker_resource_events.append(dict(created=created, updated=updated, payload=payload,
                                               mobile=mobile, cuboids=len(scene.cuboid or []),
                                               ms=(time.perf_counter()-began)*1000,
                                               checker_id=id(checker), geometry=geometry))
        return checker

    def full_cycle(self, boxes, progress=lambda text: None):
        self.checker_resource_events.clear()
        result = super().full_cycle(boxes, progress)
        result['checker_resource_events'] = list(self.checker_resource_events)
        result['checker_resource_policy'] = 'persistent topology resources; update_world for changed geometry, no result reuse'
        return result
