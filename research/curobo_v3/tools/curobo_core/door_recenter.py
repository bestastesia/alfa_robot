"""Door upper/lower pair: retain both loads, recenter, then use the ordinary conveyor suffix.

Research-only; navigation is an arrival-state transition, never a checked base trajectory.
"""
import copy
from dataclasses import asdict, replace
import json
import math
import hashlib
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import torch
from scipy.spatial.transform import Rotation
import yourdfpy
from curobo.collision_checking import RobotCollisionChecker, RobotCollisionCheckerCfg
from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.types import GoalToolPose, JointState, Pose as GpuPose

from .adapter import from_curobo_scene, matrix_pose, pose_matrix, relative_pose, to_curobo_scene
from .contracts import ACTIVE_JOINTS
from .conveyor import conveyor_pair
from .coverage_margins import coverage_margins
from .door_extract import DoorExtraction
from .door_model import MODEL_FILES, REZERO_REVISION, FixedBodyBoxes, model_revision
from .door_sequential import (_SequentialTask,check_frozen,corners,mesh_contact_proof,pose_error,
                              tool_mesh_vertices)
from .door_sequential_validity import CollisionMargins, ReducedValidity, SequentialValidity
from .fast_builds import install as install_fast_builds
from .resource_stats import record_build
from .urdf_fk import BatchToolFk
from .planner import CycleBlocked, FullCyclePlanner
from .scene import AttachedObject, CollisionPolicy, Pose, SceneObject, SceneStore, digest
from .torque import default_torque_model
from .trajectory import assemble_timed_cycle, time_parameterize_path, time_parameterize_polyline_shortest, time_parameterize_stops
from v3_batched_loaded_search import batched_rrt_connect_multi_goal, densify, loaded_robot
from v3_cartesian_transport import BatchAnalyticIk, RezeroBatchAnalyticIk, batch_ik, joint_frame, plan_cartesian_transport
from v3_cartesian_dual_transport import plan_dual_cartesian_transport
from v3_cartesian_math import LEGACY_FRAME, CarriageModel, JointFrame
from v3_multiroot_search import batched_rrt_connect_multi_goal as constrained_rrt_connect
from v3_conveyor_cycle_planner import ConveyorCyclePlanner
from v3_wall_ik_benchmark import make_scene
from v3_task_scene import grasp_transform

ROOT = Path(__file__).resolve().parents[2]
LEFT = list(range(1, 8))
RIGHT = list(range(8, 15))
DOOR_COLUMN_ORDER = (4, 0, 3, 1, 2)  # original mentor order: outer pair, inner pair, center


def door_pair(column):
    column=int(column)
    if not 0<=column<5:
        raise ValueError('door column must be in [0,4]')
    return 15+column,20+column  # lower, upper


def door_pick_base_pose(column):
    door_pair(column)  # validate once, alongside the box mapping
    offsets={0:(-.2,-30.),1:(-.4,0.),2:(0.,0.),3:(.4,0.),4:(.2,30.)}
    y,yaw_deg=offsets[int(column)];yaw=math.radians(yaw_deg)
    return Pose((0.,y,0.),(math.cos(yaw/2),0.,0.,math.sin(yaw/2)))


def snapshot_in_base(snapshot):
    return replace(snapshot,
        objects=tuple(replace(item,pose=relative_pose(item.pose,snapshot.state.base_pose)) for item in snapshot.objects),
        meshes=tuple(replace(item,pose=relative_pose(item.pose,snapshot.state.base_pose)) for item in snapshot.meshes),
        state=replace(snapshot.state,base_pose=Pose()))


def snapshot_in_world(snapshot,base_pose):
    base=pose_matrix(base_pose)
    world=lambda pose:matrix_pose(base@pose_matrix(pose))
    return replace(snapshot,
        objects=tuple(replace(item,pose=world(item.pose)) for item in snapshot.objects),
        meshes=tuple(replace(item,pose=world(item.pose)) for item in snapshot.meshes),
        state=replace(snapshot.state,base_pose=base_pose))


def pose_goal(poses, frames=None):
    frames = list(poses) if frames is None else frames
    return GoalToolPose.from_poses({name: GpuPose(
        position=torch.tensor(poses[name]['position'], device='cuda', dtype=torch.float32).reshape(1, 3),
        quaternion=torch.tensor(poses[name]['quaternion'], device='cuda', dtype=torch.float32).reshape(1, 4))
        for name in frames}, ordered_tool_frames=frames)


def door_request(planner, seed, column=2, removed_box_ids=()):
    lower_box,upper_box=door_pair(column)
    removed={int(value) for value in removed_box_ids}
    if lower_box in removed or upper_box in removed:
        raise ValueError('requested door pair was already removed')
    front = planner.front + .75
    objects = [x for x in from_curobo_scene(make_scene(planner.front, .75, set()))
               if not x.object_id.startswith('wall_box_')]
    objects += [SceneObject(f'wall_box_{r*5+c:02d}', (.3, .4, .4),
                           Pose((front+.15, (c-2)*.4, .2+r*.4)))
                for r in range(5) for c in range(5) if r*5+c not in removed]
    objects.append(SceneObject('warehouse_top_door_leaf', (.04, 2., .47), Pose((front-.035, 0., 2.115))))
    snapshot = replace(planner.snapshot, objects=tuple(objects), attachments=(), revision=0,
                       state=planner.snapshot.state.with_positions(planner.home,door_pick_base_pose(column)),
                       meshes=conveyor_pair(pose=Pose((planner.conveyor_center_x_m, 0., 0.))).world_parts(),
                       policy=CollisionPolicy(False, False, max_box_tilt_deg=89.))
    return SimpleNamespace(snapshot=snapshot, mode='door_recenter',
                           tasks=(('left',lower_box),('right',upper_box)),
                           upper_box=upper_box,lower_box=lower_box,column=int(column),
                           removed_box_ids=tuple(sorted(removed)),seed=int(seed),extraction_m=.36,
                           support_elbow_rise_m=.18,cartesian_geometry_tolerance_m=.0001)


class DoorPlanner(DoorExtraction, FullCyclePlanner):
    # Reuse ordinary placement methods, not its contact/extraction orchestration.
    conveyor_height_m = ConveyorCyclePlanner.conveyor_height_m
    placement_clearance_m = ConveyorCyclePlanner.placement_clearance_m
    conveyor_push_m = ConveyorCyclePlanner.conveyor_push_m
    conveyor_center_x_m = ConveyorCyclePlanner.conveyor_center_x_m
    conveyor_marker_x_m = ConveyorCyclePlanner.conveyor_marker_x_m
    conveyor_reference_targets = ConveyorCyclePlanner.conveyor_reference_targets
    tool_matrices = ConveyorCyclePlanner.tool_matrices
    conveyor_ik_solver = ConveyorCyclePlanner.conveyor_ik_solver
    conveyor_ik_num_seeds = 16  # MOTION-342 latency choice, resident across rounds
    conveyor_solver_cache_limit = 8
    conveyor_suffix = ConveyorCyclePlanner.conveyor_suffix
    top_grasp = False
    base_advance = 0.
    shortcut_suffix_paths = True  # placement/Home RRT paths are shortcut, then dense-validated as before
    # Free-space RRT geometry scale of the PR #80 model (None: none), and the weighted joint distance
    # from a search's start and goals within which states keep the original geometry.
    free_space_inflation = None
    free_space_anchor_radius = .1
    free_space_stages = None  # stages whose searches inflate (None: every free-space search; suffix: 'conveyor_suffix')
    # Arm-joint distance to a limit below which IK candidates rank last (None: plain distance order).
    ik_limit_margin_rad = None
    # Recenter atlas pose-score bucket (10*m+rad per side) within which nearer rows rank first (None: score).
    pose_score_bucket = None
    # Right-regrasp bases must let the right arm reach the upper box's lowered slot at a lift where the
    # left arm still holds the lower box clear of the robot (analytic IK only; False: no such filter).
    regrasp_lowering_prefilter = False
    # Right precontact IK candidates also enumerate the analytic solutions of every branch, as the left
    # precontact does (the numerical IK from the re-zeroed model's Home finds few, often unreachable ones).
    right_precontact_analytic_ik = False
    # The center column (2) takes the outer columns' rack-supported flow: in the re-zeroed model the
    # center flow's compensated upper extraction is out of the right arm's reach.
    center_column_outer_flow = False
    # MOTION-429: the frozen spheres leave up to ~2.8 cm of each arm mesh uncovered (re-zeroed model;
    # exact mesh audit: link7 and its suction cap up to 13 mm inside neighbouring boxes mid-approach).
    # Arms moving in free space grow every sphere by its coverage margin against the world and, for
    # the forearms and wrists (coverage_cross_pairs), against the other arm, keeping their original
    # spheres within the exempt radius of the contact being approached or left (the cap reached 9 mm
    # into the other forearm in the inner columns' upper precontact). Pairs the audit found penetrating
    # keep fixed margins in every phase: the two wrists crossing there (15 mm), and a carried box at the
    # arm carriage, whose spheres leave up to 5.5 cm uncovered (7 mm through a 1 cm margin). Per-sphere
    # margins of coverage_margin_links (None: no margins, the legacy model).
    sphere_margins = None
    coverage_margin_links = tuple(f'{side}_link{index}' for side in ('left', 'right') for index in range(1, 8))
    coverage_self_pairs = tuple(pair for side in ('left', 'right') for pair in
                                (('arm_carriage', f'{side}_link5', .02), (f'{side}_link4', f'{side}_link7', .015)))+(
                                    ('left_link7', 'right_link7', .02),)
    coverage_payload_links = (('arm_carriage', .025), ('left_link4', .01), ('right_link4', .01))
    coverage_cross_pairs = tuple((f'left_link{first}', f'right_link{second}') for first in range(4, 8) for second in range(4, 8))
    free_space_exempt_radius_m = .06
    # Budget (s) of the latest-pair seeds searching for their first path while the bank has none, up
    # to three of them; the rest refine in 0.75 s (None: 0.75 s seeds only).
    latest_pair_fallback_s = None
    # The latest pair keeps the first path even when it misses the rotation gates (None/False: refining
    # seeds follow). The re-zeroed column 4's gates are out of reach (the ~200 deg wrist flip): with the
    # slower edge checks no refining seed found a path (v7-v12: 0 of 56); with screened checks they did,
    # at a lower rotation score (243 against 282) but with 14.7 s instead of 9.0 s of motion.
    latest_pair_first_path = False
    # Straight retreat (m) of both tools from the released boxes before the return to Home, whose
    # search then keeps those boxes in the world (None: released boxes leave the scene at release).
    release_retreat_m = None
    # Rotation gates of the precontact/recenter searches are no stricter than their endpoints allow:
    # at least the tool rotation of the joint-space straight line to the best goal plus this slack
    # (deg). In the re-zeroed model Home holds J5 at 0 while every wall contact needs J5 near +-160,
    # and the carry poses hold J6 at its 110 deg limit with the wrist crossed against the contact
    # configuration, so the fixed gates are out of reach and every search spent its whole budget
    # (None: the fixed gates).
    rotation_gate_slack_deg = None
    # Latest-pair atlas rows keep the already recentered left arm within this joint change (deg)
    # first, within a pose-score bucket (None: pose score bucket, then distance).
    latest_pair_settled_arm_deg = None
    # Column 0 planned on its own in the mirrored frame (arms' roles swapped) instead of derived from
    # column 4's trajectory by reflection: see door_mirror.plan_mirrored_round (None/False: the
    # column-4 symmetry derivation of v3_door_five_columns.mirror_round, which needs a symmetric scene).
    column0_mirrored_frame = False
    # Search budgets where the re-zeroed model's paths differ: the recenter's Cartesian quality search,
    # and the upper precontact trying its joint routes before the Cartesian strategies. With
    # rotation_gate_slack_deg, two paths that both miss a gate end that search: its straight-line
    # reference was out of reach (obstacles), and further seeds came out alike.
    recenter_cartesian_budget_s = 8.
    precontact_joint_first = False
    # Seeds of the recenter's Cartesian quality search while it finds no path and keeps at least two goals
    # on the arm's IK branch: it keeps that branch, and its tree succeeds for some seeds only (the
    # re-zeroed column 4: 2 of 5 for one request); the RRT fallback may change the branch, from which
    # the later placement became unreachable.
    recenter_cartesian_attempts = 1
    # The single-arm recenter's Cartesian quality search (no support compensation) runs its task-space
    # RRT only with at least this many goals on the arm's IK branch, and otherwise tries their straight
    # lines only: in the re-zeroed model a
    # single same-branch goal is left where the other arm blocks the carry (columns 1 and 2), and none of
    # those ten searches (v11-v12) found a path in its 0.76 s (0: always).
    recenter_cartesian_rrt_min_goals = 0
    # Side precontact joint RRT attempts: first densely verified path plus this refinement window,
    # stopping at the first path within the 30 deg gate (None: three whole 2 s attempts).
    side_precontact_refinement_s = None
    # The single-arm recenter search first tries the joint-space straight lines to its goals
    # (DoorTask.straight_line_path): the re-zeroed column 3 lower box's line is free and within its
    # gate, where the Cartesian and RRT searches spent 4 s (False: searches only). The latest pair's and
    # the precontacts' lines all cross the door, the rack or the other arm.
    direct_line_first = False
    # Whether the ranked lower contact still searches its support-compensated extraction (2 s) when the
    # outer flow defers the extraction to the rack-supported partial line; in the re-zeroed model that
    # search found no path in any column and the partial line is what ran (False: skipped).
    deferred_extract_search = True
    # Coarse resolution (rad) at which DoorTask.shortcut_states screens its candidate edges before the
    # 0.5 deg check: the full check of every later point of a 129-point path took 0.5-1.9 s per call
    # in the re-zeroed latest pair and upper precontacts (None: 0.5 deg check of every edge).
    shortcut_screen_rad = None
    # Joint RRT dense checks one tree edge at a time with each edge's result kept (the
    # edge_cache of batched_rrt_connect_multi_goal: same decisions, fewer checks). The re-zeroed latest
    # pair rejected up to 96 coarse-valid connections through shared edges (0.9 s of rechecks, 30 ms cached).
    rrt_fine_edge_cache = False
    # ... and the subtree below a tree edge that fails that check leaves the search (prune_invalid_edges):
    # the coarse checks step over thin obstacles, and the subtrees grown across them kept offering the
    # cheapest connections. Re-zeroed column 4: the latest pair's first path 2.9 -> 1.8 s (seed 5), and
    # the upper precontact's first seed, which found no path in 0.8 s, now finds one (False: kept).
    rrt_prune_invalid_edges = False
    # The single-arm recenter's joint RRT first searches toward the goals whose joint-space straight
    # line turns the tool within its rotation gate, and toward all goals once those give no path: in
    # the re-zeroed columns 1 and 2, 22 of the lower box's 30 carry goals need a further wrist turn
    # (line rotation 297-416 deg against 44-72 deg), and the RRT to all goals mostly ended at one of
    # them, about 300 deg of extra tool rotation (False: all goals).
    recenter_line_goals = False
    # Each of those RRT attempts takes its first densely verified path plus this refinement window
    # (None: whole 2 s attempts). Those attempts settled on their path within 1 s (the 1 and 2 s budgets
    # chose the same paths for column 1); with the line goals 0.7 s attempts came out alike.
    recenter_rrt_refinement_s = None
    # Door validities check edges over each edge's own samples instead of the grid padded to the
    # longest edge (GpuValidity.edges compact_edges: the same states and results). The latest pair's
    # RRT connections padded 73k sampled states to 107k per first path (False: padded grid).
    compact_edge_checks = False
    # With those, the joint RRT's extension and connection checks screen each edge's last and every
    # fourth sample first (screen_edges; the same edges pass). The latest pair's iterations spent
    # ~105 ms checking 1024 extensions, ~90% of them invalid, and ~100 mostly invalid connections
    # (False: every sample).
    rrt_screened_edges = False

    def __init__(self, assets):
        install_fast_builds()
        super().__init__(assets)
        self.adopt_model(assets)
        self.batch_fk = BatchToolFk(self.fk_robot, ('left_tool0', 'right_tool0', 'arm_carriage'), ACTIVE_JOINTS)
        self.planner_kind = 'rrtconnect'
        self._door_ik_key = None
        self._door_ik = None
        self._door_checker_cache = {}
        self._conveyor_solver_cache = {}
        self._torque_model_cache = {}
        self.held = ()
        self.active_side = 'left'
        self.suffix_inflation = []

    def adopt_model(self, assets):
        """Joint coordinates, tool point, fixed body, dynamics and recenter atlas of the URDF's model."""
        self.model_revision = model_revision(assets.urdf)
        self.joint_frame = LEGACY_FRAME
        self.fixed_body = None
        self.dynamics_root = getattr(assets, 'dynamics_root', None)
        self.recenter_atlas = getattr(assets, 'recenter_atlas', None)
        if self.model_revision is None:
            if any(getattr(assets, name, None) is not None for name in MODEL_FILES):
                raise ValueError('the legacy door model takes no re-zeroed model files')
            return
        if any(getattr(assets, name, None) is None for name in MODEL_FILES):
            raise ValueError(f'the re-zeroed door model requires {MODEL_FILES}')
        from v3_fixed_body_boxes import disable_fixed_body_spheres
        from v3_prepare_rezero_profiles import convert_row
        from v3_rezero_analytic import RezeroAnalytic
        from v3_zero_model_candidate import COORDINATE_REVISION, JOINT_SIGNS, OFFSETS_DEG, TOOL_OFFSET_M
        if COORDINATE_REVISION != REZERO_REVISION:
            raise ValueError('analytic adapter and door model disagree on the joint coordinates')
        for side in ('left', 'right'):
            origin = self.fk_robot.joint_map[side+'_tool0_fixed'].origin
            if not np.allclose(origin, [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, TOOL_OFFSET_M], [0, 0, 0, 1]], atol=1e-12):
                raise ValueError(f'the re-zeroed model requires tool0 at J7+Z{TOOL_OFFSET_M*1000:g} mm')
        self.analytic = RezeroAnalytic()
        self.joint_frame = JointFrame(np.deg2rad([OFFSETS_DEG['left'], OFFSETS_DEG['right']]), JOINT_SIGNS)
        self.tool_offset_m = TOOL_OFFSET_M
        self.robot = disable_fixed_body_spheres(self.robot)
        self.mobile_robot = disable_fixed_body_spheres(self.mobile_robot)
        self.fixed_body = FixedBodyBoxes(assets.fixed_body_boxes, self.fk_robot)
        kinematics = self.robot['kinematics']
        self.sphere_margins = coverage_margins(kinematics, yourdfpy.URDF.load(
            kinematics['urdf_path'], load_meshes=False, build_scene_graph=False), self.coverage_margin_links)
        self.free_space_inflation = 1.1
        self.latest_pair_fallback_s = 6.
        self.latest_pair_first_path = True
        self.release_retreat_m = .08
        self.rotation_gate_slack_deg = 30.
        self.latest_pair_settled_arm_deg = 45.
        self.recenter_cartesian_budget_s = 1.
        self.recenter_cartesian_attempts = 3
        self.recenter_cartesian_rrt_min_goals = 2
        self.precontact_joint_first = True
        self.side_precontact_refinement_s = .25
        self.direct_line_first = True
        self.shortcut_screen_rad = .05
        self.rrt_fine_edge_cache = True
        self.rrt_prune_invalid_edges = True
        self.recenter_line_goals = True
        self.recenter_rrt_refinement_s = .5
        self.compact_edge_checks = True
        self.rrt_screened_edges = True
        self.deferred_extract_search = False
        self.ik_limit_margin_rad = math.radians(15.)
        self.pose_score_bucket = .05
        self.regrasp_lowering_prefilter = True
        self.right_precontact_analytic_ik = True
        self.center_column_outer_flow = True
        self.free_space_stages = ('conveyor_suffix',)  # placement and Home searches only (user choice A)
        self.column0_mirrored_frame = True
        self.target_values = convert_row(list(ACTIVE_JOINTS), self.target_values)
        self.model_id = self._model_identity()
        self.snapshot = replace(self.snapshot, model_id=self.model_id)

    @property
    def joint_limits(self):
        """(lower, upper) of ACTIVE_JOINTS from the URDF."""
        limits = [self.fk_robot.joint_map[name].limit for name in ACTIVE_JOINTS]
        return np.array([item.lower for item in limits]), np.array([item.upper for item in limits])

    def make_batch_analytic_ik(self):
        if self.model_revision is None:
            return BatchAnalyticIk()
        return RezeroBatchAnalyticIk(self.joint_frame, self.tool_offset_m, self.analytic.lower, self.analytic.upper)

    def _model_identity(self):
        identity = super()._model_identity()
        fixed_body = getattr(self, 'fixed_body', None)
        return identity if fixed_body is None else digest(dict(robot=identity, fixed_body_boxes=fixed_body.sha256))

    def search_path(self, start, goals, lower, upper, validity, budget, seed, stop_at_first_valid=None):
        """Placement and return-Home RRT (the ordinary suffix): free-space inflation of the model."""
        if self.free_space_inflation is not None and (self.free_space_stages is None or
                                                      'conveyor_suffix' in self.free_space_stages):
            from .door_sequential_validity import SearchInflation
            record = dict(stage='conveyor_suffix')
            self.suffix_inflation.append(record)
            validity = validity.inflated(SearchInflation(
                torch.vstack([start.reshape(1, -1), goals.reshape(-1, start.shape[-1])]), validity.weights,
                scale=self.free_space_inflation, radius=self.free_space_anchor_radius, record=record))
        return super().search_path(start, goals, lower, upper, validity, budget, seed, stop_at_first_valid)

    @property
    def idle_indices(self):
        return RIGHT if self.active_side == 'left' else LEFT

    def ik_solver(self, boxes, *, robot=None, snapshot=None, random_seed=None):
        if robot is None:
            return super().ik_solver(boxes)
        key = (snapshot.geometry_key, digest(robot), random_seed)
        self.ik_cache_hit = self._door_ik is not None and key == self._door_ik_key
        if not self.ik_cache_hit:
            self._door_ik = None
            build_started = time.perf_counter()
            self._door_ik = InverseKinematics(InverseKinematicsCfg.create(
                robot=copy.deepcopy(robot), scene_model=to_curobo_scene(snapshot),
                collision_cache={'cuboid': max(40, len(snapshot.objects)), 'mesh': len(snapshot.meshes)},
                num_seeds=512, self_collision_check=True, use_cuda_graph=False,
                position_tolerance=.002, orientation_tolerance=math.radians(1),
                # cuRobo-Fast runs L-BFGS in blocks of inner_iters=80 (0.8.0: 20); 500 is not a multiple.
                override_iters_for_multi_link_ik=480, optimizer_collision_activation_distance=.005,
                random_seed=random_seed))
            record_build(self, 'door_ik', build_started)
            self._door_ik_key = key
        return self._door_ik

    def task_attachments(self, boxes):
        expected = {f'wall_box_{i:02d}' for i in boxes.values()}
        if {x.object_id for x in self.held} != expected:
            raise ValueError('ordinary suffix requires the two actual door attachments')
        return self.held

    def payload_transform(self, side):
        return pose_matrix(next(x for x in self.held if x.parent_link == side+'_tool0').tool_to_object)

    def placement_box_seed(self, side, tool):
        # Preserve the ordinary conveyor's fixed box targets despite off-centre door contacts.
        # Actual contact offsets are still used to derive the tool IK target and release pose.
        return self.carry_target_poses[side] @ grasp_transform(side, False)

    def shared_parking_pose(self, required_bases):
        parking = np.mean(required_bases, axis=0)
        residual = float(np.max(np.linalg.norm(np.asarray(required_bases)-parking, axis=1)))
        # Current carry targets are 13.3 mm per side inward of the conveyor target.
        # The final arm IK owns this correction; keep a bounded 50 mm ceiling.
        if residual > .05:
            raise CycleBlocked('反算放置底盘位', f'门扉回中位需要的双手修正过大: {residual}')
        return parking, residual

    def checker(self, boxes, payload=False, mobile=False, objects=()):
        """Placement/Home checker of the current scene plus `objects` (map frame; released boxes, no payload)."""
        if mobile or (payload and objects):
            raise ValueError('door suffix does not plan navigation, nor carry boxes it keeps in the world')
        build_started = time.perf_counter()
        robot = loaded_robot(self.robot, self.box_fit, attachments=self.held) if payload else self.robot
        snapshot = (replace(self.snapshot, attachments=(), objects=self.snapshot.objects+tuple(objects)) if objects
                    else self.snapshot)
        checker = RobotCollisionChecker(RobotCollisionCheckerCfg.load_from_config(
            robot_config=robot, scene_model=to_curobo_scene(snapshot),
            n_cuboids=max(40, len(snapshot.objects)), n_meshes=len(snapshot.meshes),
            collision_activation_distance=0.))
        record_build(self, 'placement_checker_loaded' if payload else 'placement_checker_empty', build_started)
        return checker

    def placement_validity(self, checker, payload, exempt_points=None, objects=()):
        # The ordinary suffix works in a fixed *arrived* base frame; retain the real world geometry.
        snapshot = replace(self.snapshot, attachments=self.held if payload else (),
                           objects=tuple(replace(x, pose=relative_pose(x.pose, self.snapshot.state.base_pose))
                                         for x in self.snapshot.objects+tuple(objects)),
                           meshes=tuple(replace(x, pose=relative_pose(x.pose, self.snapshot.state.base_pose))
                                        for x in self.snapshot.meshes),
                           state=replace(self.snapshot.state, base_pose=Pose()))
        margins = None
        if self.sphere_margins is not None:
            if exempt_points is None:
                # Return to Home: original spheres next to the released start (consumed here) and Home.
                rows = [row for row in (self.__dict__.pop('_released_q', None), self.home_values) if row is not None]
                exempt_points = {side: [self.tool_matrices(row)[side][:3, 3] for row in rows] for side in ('left', 'right')}
            # Both arms move in free space here (MOTION-429 margins, see coverage_margin_links).
            margins = CollisionMargins(
                world=self.coverage_margin_links, exempt_radius_m=self.free_space_exempt_radius_m,
                exempt=tuple((side, tuple(sorted({tuple(float(x) for x in np.round(point, 4)) for point in points})))
                             for side, points in exempt_points.items()),
                self_pairs=self.coverage_self_pairs, payload=self.coverage_payload_links,
                cross_pairs=self.coverage_cross_pairs)
        return SequentialValidity(checker, snapshot, self.robot,
            {side: tool_mesh_vertices(self, side) for side in ('left', 'right')}, geometry_tolerance=.0001,
            fixed_body=self.fixed_body, margins=margins, sphere_margins=self.sphere_margins)

    def plan_conveyor_target(self, start, targets, checker, validity, solver, report, append):
        """Placement RRT; with coverage margins the arms keep their original spheres only near the start
        and the placement (MOTION-429)."""
        if self.sphere_margins is not None:
            tools = self.tool_matrices(start)
            validity = self.placement_validity(checker, True, exempt_points={
                side: [tools[side][:3, 3], target[:3, 3]] for side, target in targets.items()})
        placed = ConveyorCyclePlanner.plan_conveyor_target(self, start, targets, checker, validity, solver, report, append)
        self._released_q = placed
        if self.release_retreat_m:
            # Released boxes stay on the conveyor where they were let go: the return to Home avoids them.
            base, tools = pose_matrix(self.snapshot.state.base_pose), self.tool_matrices(placed)
            self._released_boxes = np.asarray(placed).copy(), tuple(SceneObject(item.object_id, item.dimensions_m, matrix_pose(
                base@tools[item.parent_link.removesuffix('_tool0')]@pose_matrix(item.tool_to_object))) for item in self.held)
        return placed

    def joint_plan(self, start, target, checker, validity, seed, stop_at_first_valid=None, shortcut=False):
        """Return to Home: with released boxes pending, both tools first retreat straight back from them,
        then the search keeps them in the world (the retreat itself is checked without them)."""
        placed, released = self.__dict__.pop('_released_boxes', (None, None))
        if released is None or not np.allclose(start, placed, atol=1e-9, rtol=0):
            return super().joint_plan(start, target, checker, validity, seed, stop_at_first_valid, shortcut)
        retreat, details = self.release_retreat(start, validity)
        if retreat is None:
            return None, dict(failure='release_retreat', details=details)
        boxed = self.checker(None, objects=released)
        tools, home = self.tool_matrices(retreat[-1]), self.tool_matrices(target)
        boxed_validity = self.placement_validity(boxed, False, objects=released, exempt_points={
            side: [tools[side][:3, 3], home[side][:3, 3]] for side in ('left', 'right')})
        path, stats = super().joint_plan(retreat[-1], target, boxed, boxed_validity, seed, stop_at_first_valid, shortcut)
        stats = dict(stats, release_retreat=details, released_boxes=[item.object_id for item in released])
        if path is None:
            return None, stats
        return np.vstack([retreat, np.asarray(path)[1:]]), stats

    def release_retreat(self, start, validity):
        """Analytic straight lines of the tools back along their tool axes: both at once on a fixed
        swivel when the axes are parallel, else one after the other with swivel continuation."""
        tools = self.tool_matrices(start)
        axes = {side: tools[side][:3, 2] for side in ('left', 'right')}
        direction = -(axes['left']+axes['right'])/np.linalg.norm(axes['left']+axes['right'])
        failures = []
        if min(float(-axis@direction) for axis in axes.values()) >= math.cos(math.radians(2.)):
            path, details = self.analytic_extract(start, validity, validity, sides=('left', 'right'), direction=direction,
                                                  distance_m=self.release_retreat_m)
            if path is not None:
                return path, details
            failures.append(dict(order='both', details=details))
        for order in (('left', 'right'), ('right', 'left')):
            path, details = np.asarray(start, dtype=float)[None], []
            for side in order:
                segment, record = self.analytic_extract(
                    path[-1], validity, validity, sides=(side,), direction=-axes[side], distance_m=self.release_retreat_m,
                    hold_sides=tuple(other for other in order if other != side), swivel_continuation=True)
                if segment is None:
                    failures.append(dict(order='+'.join(order), side=side, details=record))
                    break
                path, details = np.vstack([path, segment[1:]]), details+[record]
            else:
                return path, dict(order='+'.join(order), segments=details, earlier_failures=failures)
        return None, failures


class SupportCompensation:
    """Map lift + moving-arm states to full rows while holding the upper box in world."""
    def __init__(self, task, side='right', reference=None, tool_target=None):
        self.task = task
        self.reference = task.q.copy() if reference is None else np.asarray(reference, dtype=float).copy()
        self.support_side = side
        self.side_index = 0 if side == 'left' else 1
        self.support_indices = LEFT if side == 'left' else RIGHT
        self.support_tool = (task.tool(self.reference, side) if tool_target is None
                             else np.asarray(tool_target, dtype=float).copy())
        self.carriage = CarriageModel(task.planner, self.reference)
        self.ik = batch_ik(task.planner)
        arm = self.reference[self.support_indices]
        self.swivel = task.planner.analytic.swivel(self.side_index, arm)
        self.branch = joint_frame(task.planner).branch(self.side_index, arm)
        self.calls = 0
        self.failures = 0

    def __call__(self, rows):
        values = np.asarray(rows, dtype=np.float64).copy().reshape(-1, 15)
        count = len(values)
        targets = self.carriage.inverse(values[:, 0]) @ self.support_tool
        arms, found = self.ik.solve(
            self.side_index, targets, np.full(count, self.swivel),
            np.repeat(self.reference[None, self.support_indices], count, axis=0), self.branch)
        values[:, self.support_indices] = arms
        at_reference = np.isclose(values[:, 0], self.reference[0], atol=1e-10, rtol=0)
        values[np.ix_(np.flatnonzero(at_reference), self.support_indices)] = self.reference[self.support_indices]
        found[at_reference] = True
        self.calls += count
        self.failures += int((~found).sum())
        return values, found


class SupportCompensatedValidity(ReducedValidity):
    """Eight searched variables; the right arm is a deterministic support constraint."""
    def __init__(self, full, reference, mapper, searched_indices=None):
        super().__init__(full, reference, [0] + LEFT if searched_indices is None else searched_indices)
        self.mapper = mapper

    def mapped(self, values):
        expanded = self.reference.expand(len(values), -1).clone()
        expanded[:, self.indices] = values
        rows, found = self.mapper(expanded.detach().cpu().numpy())
        return torch.as_tensor(rows, device=values.device, dtype=values.dtype), torch.as_tensor(
            found, device=values.device, dtype=torch.bool)

    def expand(self, values):
        rows, found = self.mapped(values)
        if not bool(found.all()):
            raise ValueError('support compensation IK failed while expanding an accepted path')
        return rows

    def evaluate(self, values):
        rows, found = self.mapped(values)
        valid, stability = self.full.evaluate(rows)
        return valid & found, stability


class DoorTask(_SequentialTask):
    def __init__(self, planner, request, algorithm, policy, progress=print):
        self.world_request=request
        self.pick_base_pose=request.snapshot.state.base_pose
        local_request=SimpleNamespace(**vars(request))
        local_request.snapshot=snapshot_in_base(request.snapshot)
        super().__init__(planner, local_request, progress)
        if algorithm not in ('joint_rrt', 'cartesian_rrt'):
            raise ValueError('unknown recenter planner')
        box_rotation=pose_matrix(local_request.snapshot.object(f'wall_box_{local_request.lower_box:02d}').pose)[:3,:3]
        self.wall_inward_direction=box_rotation[:,0]
        self.wall_outward_direction=-self.wall_inward_direction
        self.upper_first=False
        self.algorithm = algorithm
        self.policy = policy
        side_policy = policy['side']
        self.canonical_targets = {
            side: pose_matrix(Pose((side_policy['position']['x'], side_policy['position']['y_'+side],
                                    side_policy['position']['z']), tuple(side_policy['quaternion_wxyz'])))
            for side in ('left', 'right')}
        self.targets = {side: matrix.copy() for side, matrix in self.canonical_targets.items()}
        planner.carry_target_poses = self.canonical_targets
        self.radius = float(policy['side']['placement_radius_m'])
        self.lift_grid = np.asarray(policy['side']['placement_updown_grid_m'], dtype=float)
        self.swivel_step_deg = float(policy['contact_candidates']['swivel_step_deg'])
        if (self.lift_grid.ndim != 1 or not len(self.lift_grid) or not np.isfinite(self.lift_grid).all()
                or not np.isfinite(self.swivel_step_deg) or not 0 < self.swivel_step_deg <= 180):
            raise ValueError('invalid existing carry lift/swivel grid')
        self.report.update(recenter_algorithm=algorithm, recenter_inputs=[], recenter_searches=[],
                           pick_base_pose=asdict(self.pick_base_pose),
                           world_scene_geometry_key=request.snapshot.geometry_key,
                           carry_policy=policy, base_transition_policy='navigation arrival state switch; no base trajectory',
                           navigation_path_validated=False, collision_model='inherited spheres + exact payload OBB/contact proof',
                           hardware_validated=False)
        self.motion_segments=[]
        self.base_pose_by_frame=[]

    def contact_target(self, side, item, roll):
        target,proof=super().contact_target(side,item,roll)
        offset=(float(getattr(self,'upper_right_regrasp_z_offset',.06)) if self.outer_flow() and side=='right' and item.object_id==f'wall_box_{self.request.upper_box:02d}' else
                float(getattr(self,'upper_regrasp_z_offset',0.)) if side=='left' and item.object_id==f'wall_box_{self.request.upper_box:02d}' else 0.)
        if offset:
            target=target.copy();target[2,3]-=offset
            proof=mesh_contact_proof(tool_mesh_vertices(self.planner,side),np.linalg.inv(pose_matrix(item.pose))@target,
                item.dimensions_m,.001,tolerance=self.request.cartesian_geometry_tolerance_m)
        return target,proof

    def event(self, object_id, local_pose):
        world=pose_matrix(self.pick_base_pose)@local_pose
        self.report['predicted_scene_events'].append({'phase':self.phase,'object_id':object_id,
            'frame_index':len(self.report['frames'])-1,'world_pose':asdict(matrix_pose(world)),
            'snapshot':snapshot_in_world(self.store.snapshot(),self.pick_base_pose).to_dict()})

    def upper_extraction_endpoint_count(self,start):
        validity=self.validity();item=next(item for item in self.store.snapshot().attachments if item.parent_link=='right_tool0')
        distance=.005-self.wall_clearance(start,item,self.request.snapshot.object(item.object_id));origin=self.tool(start,'right');target=origin.copy();target[:3,3]+=self.wall_outward_direction*distance
        mapper=SupportCompensation(self,side='left',reference=start,tool_target=self.targets['left']);limits=validity.checker.kinematics.get_joint_limits().position
        low,high=(float(values[0].item()) for values in limits);lifts=[value for value in self.lift_grid if low+1e-5<=value<=high-1e-5];carriage=CarriageModel(self.planner,start);endpoints=[]
        for lift in lifts:
            template=start.copy();template[0]=lift;mapped,found=mapper(template[None])
            if not found[0]:continue
            local=carriage.inverse(np.asarray([lift]))[0]@target
            for swivel in np.deg2rad(np.arange(-180.,180.,self.swivel_step_deg)):
                for arm in self.planner.analytic.solve(1,local,float(swivel),start[RIGHT]):
                    row=mapped[0].copy();row[RIGHT]=arm;endpoints.append(row)
        endpoints=np.unique(np.round(endpoints,7),axis=0) if endpoints else np.empty((0,15))
        if len(endpoints):endpoints=endpoints[validity.mask(self.tensor(endpoints)).cpu().numpy()]
        return int(self.exact_pose_mask(endpoints,'right',target).sum())

    def outer_flow(self):
        """Whether this pair takes the rack-supported outer-column flow (DoorPlanner.center_column_outer_flow)."""
        column=getattr(self.request,'column',2)
        return column in (0,1,3,4) or (column==2 and getattr(getattr(self,'planner',None),'center_column_outer_flow',False))

    def held_lowering_lifts(self):
        """Lifts, from the current one down, at which the left arm holds the lower box in place collision-free.

        The compensated upper lowering ends at a lift (validate_future_lowering, plan_compensated_upper_lowering);
        while it descends, the held box closes on the robot. Checked with the right arm unmoved."""
        validity=self.validity();limits=validity.checker.kinematics.get_joint_limits().position
        low,high=(float(values[0].item()) for values in limits);current=float(self.q[0])
        lifts=[current]+sorted((float(value) for value in self.lift_grid if low+1e-5<=value<current-1e-9),reverse=True)
        templates=np.repeat(self.q[None],len(lifts),axis=0);templates[:,0]=lifts
        rows,found=SupportCompensation(self,side='left',reference=self.q,tool_target=self.targets['left'])(templates)
        valid=found&validity.mask(self.tensor(rows)).cpu().numpy()
        return lifts[:len(valid) if valid.all() else max(1,int(np.argmin(valid)))]

    def regrasp_lowering_reachable(self,upper,lifts):
        """Whether the right arm reaches `upper`'s slot lower_distance below a regrasp contact (any z offset of
        the base search) at one of `lifts`: analytic IK only, a necessary condition of the lowering."""
        target,_=super().contact_target('right',upper,0.);carriages=CarriageModel(self.planner,self.q).inverse(np.asarray(lifts))
        swivels=np.deg2rad(np.arange(-180.,180.,self.swivel_step_deg))
        for z_offset in (0.,.03,.06):
            lowered=target.copy();lowered[2,3]-=z_offset+self.lower_distance
            if any(len(self.planner.analytic.solve(1,carriage@lowered,float(swivel),self.q[RIGHT])) for carriage in carriages for swivel in swivels):
                return True
        return False

    def find_right_upper_regrasp_base(self):
        search_started=time.perf_counter()
        lowering_lifts=self.held_lowering_lifts() if getattr(self.planner,'regrasp_lowering_prefilter',False) else None
        old=self.pick_base_pose;world_store=snapshot_in_world(self.store.snapshot(),old);world_request=snapshot_in_world(self.request.snapshot,old)
        matrix=pose_matrix(old);world_outward=matrix[:3,:3]@self.wall_outward_direction;world_lateral=matrix[:3,1]
        old_yaw=2*math.atan2(old.quaternion_wxyz[3],old.quaternion_wxyz[0]);attempts=[]
        laterals=(.1,0.,-.1) if self.request.column<=1 else (-.1,0.,.1)
        yaw_deltas=(-10.,0.,-20.,10.,20.) if self.request.column<=1 else (10.,0.,20.,-10.,-20.)
        inward_values=((.35,.3,.25,.2,.15,.1,.05,0.) if self.request.column==0 else
                       (0.,.05,.1,.15,.2,.25) if self.request.column==4 else (.1,.05,0.,.15,.2,.25,.3,.35))
        for inward in inward_values:
            for lateral in laterals:
                for yaw_delta in yaw_deltas:
                    position=np.asarray(old.position)-world_outward*inward+world_lateral*lateral;yaw=old_yaw+math.radians(yaw_delta)
                    candidate=Pose(tuple(position),(math.cos(yaw/2),0.,0.,math.sin(yaw/2)));self.pick_base_pose=candidate
                    self.store=SceneStore(snapshot_in_base(replace(world_store,state=replace(world_store.state,base_pose=candidate))))
                    self.request.snapshot=snapshot_in_base(replace(world_request,state=replace(world_request.state,base_pose=candidate)))
                    upper=self.store.snapshot().object(f'wall_box_{self.request.upper_box:02d}');self.wall_inward_direction=pose_matrix(upper.pose)[:3,0];self.wall_outward_direction=-self.wall_inward_direction
                    if lowering_lifts is not None and not self.regrasp_lowering_reachable(upper,lowering_lifts):
                        attempts.append(dict(inward_m=inward,lateral_m=lateral,yaw_delta_deg=yaw_delta,lowering_reachable=False));continue
                    self.validity_cache={};self.cached_key=None;validity=self.validity();state_valid=bool(validity.mask(self.tensor(self.q[None]))[0]);contact_path=False;selected_offset=None;count=0
                    if state_valid:
                        base_target,_=super().contact_target('right',upper,0.);self.planner.fk(self.q);carriage=np.linalg.inv(self.planner.fk_robot.get_transform('arm_carriage',self.planner.fk_robot.base_link))
                        for z_offset in (0.,.03,.06):
                            contact_target=base_target.copy();contact_target[2,3]-=z_offset;proof=mesh_contact_proof(tool_mesh_vertices(self.planner,'right'),np.linalg.inv(pose_matrix(upper.pose))@contact_target,upper.dimensions_m,.001,tolerance=self.request.cartesian_geometry_tolerance_m)
                            if not proof['valid']:continue
                            pre=contact_target.copy();pre[:3,3]+=self.wall_outward_direction*.05;rows=[]
                            for swivel in np.deg2rad(np.arange(-180.,180.,self.swivel_step_deg/2)):
                                for arm in self.planner.analytic.solve(1,carriage@pre,float(swivel),self.q[RIGHT]):
                                    row=self.q.copy();row[RIGHT]=arm;rows.append(row)
                            rows=np.unique(np.round(rows,7),axis=0) if rows else np.empty((0,15))
                            if len(rows):rows=rows[validity.mask(self.tensor(rows)).cpu().numpy()]
                            rows=sorted(rows,key=lambda row:np.linalg.norm(row[RIGHT]-self.q[RIGHT]))[:16];count=max(count,len(rows))
                            for row in rows:
                                self.contact_pair=('right',upper.object_id);path=self.cartesian('right',self.wall_inward_direction,.05,RIGHT,start=row,swivel_continuation=True)
                                if path is None:path=self.analytic_task_space_bridge(row,'right',contact_target,steps=16)
                                self.contact_pair=None
                                if path is not None:contact_path=True;selected_offset=z_offset;break
                            if contact_path:break
                    attempts.append(dict(inward_m=inward,lateral_m=lateral,yaw_delta_deg=yaw_delta,state_valid=state_valid,precontact_candidates=count,selected_z_offset_m=selected_offset,certified_contact_path=contact_path))
                    if contact_path:
                        self.upper_right_regrasp_z_offset=float(selected_offset or 0.);self.world_request.snapshot=replace(world_request,state=replace(world_request.state,base_pose=candidate))
                        self.record_operation_time('regrasp_base_search',search_started);self.enter('base_pick_reposition');self.append([self.q]);self.report['right_upper_regrasp_search']=dict(success=True,attempts=attempts,selected=attempts[-1],held_lowering_lifts=lowering_lifts);return
        self.pick_base_pose=old;self.store=SceneStore(snapshot_in_base(world_store));self.request.snapshot=snapshot_in_base(world_request);upper=self.request.snapshot.object(f'wall_box_{self.request.upper_box:02d}');self.wall_inward_direction=pose_matrix(upper.pose)[:3,0];self.wall_outward_direction=-self.wall_inward_direction;self.validity_cache={};self.cached_key=None
        self.report['right_upper_regrasp_search']=dict(success=False,attempts=attempts,held_lowering_lifts=lowering_lifts);self.record_operation_time('regrasp_base_search',search_started);self.fail('no base arrival state can regrasp the upper box with the right arm')

    def find_upper_regrasp_base(self):
        old=self.pick_base_pose;world_store=snapshot_in_world(self.store.snapshot(),old);world_request=snapshot_in_world(self.request.snapshot,old)
        old_matrix=pose_matrix(old);world_outward=old_matrix[:3,:3]@self.wall_outward_direction;world_lateral=old_matrix[:3,1]
        old_yaw=2*math.atan2(old.quaternion_wxyz[3],old.quaternion_wxyz[0]);attempts=[]
        candidates=((inward,lateral,yaw_delta) for inward in (0.,.02,.04)
                    for lateral in (0.,-.075,.075,-.15,.15)
                    for yaw_delta in (0.,-5.,5.,-10.,10.,-15.,15.))
        for inward,lateral,yaw_delta in candidates:
            position=np.asarray(old.position)-world_outward*inward+world_lateral*lateral;yaw=old_yaw+math.radians(yaw_delta)
            candidate=Pose(tuple(position),(math.cos(yaw/2),0.,0.,math.sin(yaw/2)));self.pick_base_pose=candidate
            self.store=SceneStore(snapshot_in_base(replace(world_store,state=replace(world_store.state,base_pose=candidate))))
            self.request.snapshot=snapshot_in_base(replace(world_request,state=replace(world_request.state,base_pose=candidate)))
            upper=self.store.snapshot().object(f'wall_box_{self.request.upper_box:02d}');self.wall_inward_direction=pose_matrix(upper.pose)[:3,0];self.wall_outward_direction=-self.wall_inward_direction
            self.validity_cache={};self.cached_key=None;validity=self.validity();state_valid=bool(validity.mask(self.tensor(self.q[None]))[0]);analytic=[];contact_path=False
            selected_offset=None
            if state_valid:
                base_target,_=super().contact_target('left',upper,0.);self.planner.fk(self.q);carriage=np.linalg.inv(self.planner.fk_robot.get_transform('arm_carriage',self.planner.fk_robot.base_link))
                for z_offset in (0.,.03,.06):
                    contact_target=base_target.copy();contact_target[2,3]-=z_offset;proof=mesh_contact_proof(tool_mesh_vertices(self.planner,'left'),
                        np.linalg.inv(pose_matrix(upper.pose))@contact_target,upper.dimensions_m,.001,tolerance=self.request.cartesian_geometry_tolerance_m)
                    if not proof['valid']:continue
                    precontact=contact_target.copy();precontact[:3,3]+=self.wall_outward_direction*.05;analytic=[]
                    for swivel in np.deg2rad(np.arange(-180.,180.,self.swivel_step_deg/2)):
                        for arm in self.planner.analytic.solve(0,carriage@precontact,float(swivel),self.q[LEFT]):
                            row=self.q.copy();row[LEFT]=arm;analytic.append(row)
                    analytic=np.unique(np.round(analytic,7),axis=0) if analytic else np.empty((0,15))
                    if len(analytic):analytic=analytic[validity.mask(self.tensor(analytic)).cpu().numpy()]
                    analytic=sorted(analytic,key=lambda row:np.linalg.norm(row[LEFT]-self.q[LEFT]))[:16]
                    for row in analytic:
                        self.contact_pair=('left',upper.object_id);path=self.cartesian('left',self.wall_inward_direction,.05,LEFT,start=row,swivel_continuation=True)
                        if path is None:path=self.analytic_task_space_bridge(row,'left',contact_target,steps=16)
                        self.contact_pair=None
                        if path is not None:contact_path=True;selected_offset=z_offset;break
                    if contact_path:break
            attempts.append(dict(inward_m=inward,lateral_m=lateral,yaw_delta_deg=yaw_delta,state_valid=state_valid,
                                 upper_precontact_candidates=len(analytic),selected_z_offset_m=selected_offset,certified_contact_path=contact_path))
            if contact_path:
                self.upper_regrasp_z_offset=float(selected_offset or 0.)
                self.world_request.snapshot=replace(world_request,state=replace(world_request.state,base_pose=candidate))
                self.enter('base_pick_reposition');self.append([self.q]);self.report['base_regrasp_search']=dict(success=True,attempts=attempts,
                    selected=dict(inward_m=inward,lateral_m=lateral,yaw_delta_deg=yaw_delta));return
        self.pick_base_pose=old;self.store=SceneStore(snapshot_in_base(world_store));self.request.snapshot=snapshot_in_base(world_request)
        original_upper=self.request.snapshot.object(f'wall_box_{self.request.upper_box:02d}');self.wall_inward_direction=pose_matrix(original_upper.pose)[:3,0];self.wall_outward_direction=-self.wall_inward_direction
        self.validity_cache={};self.cached_key=None;self.report['base_regrasp_search']=dict(success=False,attempts=attempts)
        self.fail('no collision-valid base arrival state can regrasp the rack-supported upper box')

    def switch_pick_base(self,distance_m):
        old=self.pick_base_pose;world_direction=pose_matrix(old)[:3,:3]@self.wall_outward_direction
        position=np.asarray(old.position)+world_direction*distance_m
        new=Pose(tuple(position),old.quaternion_wxyz)
        world_store=snapshot_in_world(self.store.snapshot(),old)
        world_request=snapshot_in_world(self.request.snapshot,old)
        self.pick_base_pose=new
        self.store=SceneStore(snapshot_in_base(replace(world_store,state=replace(world_store.state,base_pose=new))))
        self.request.snapshot=snapshot_in_base(replace(world_request,state=replace(world_request.state,base_pose=new)))
        self.world_request.snapshot=replace(world_request,state=replace(world_request.state,base_pose=new))
        self.validity_cache={};self.cached_key=None
        self.enter('base_pick_reposition');self.append([self.q])
        self.report.setdefault('pick_base_transitions',[]).append(dict(distance_m=distance_m,
            from_pose=asdict(old),to_pose=asdict(new),navigation_path_validated=False))

    def finish_wall_clearance(self,side,item,active,minimum=.005):
        original=self.request.snapshot.object(item.object_id);segments=[]
        for _ in range(40):
            clearance=self.wall_clearance(self.q,item,original)
            if clearance>=minimum-1e-6:
                self.report['segmented_extract']=dict(side=side,segments=segments,final_clearance_m=clearance)
                return True
            distance=min(.01 if clearance>-.1 else .05,minimum-clearance)
            path=self.cartesian(side,self.wall_outward_direction,distance,active,start=self.q,swivel_continuation=True)
            achieved=distance
            if path is None:
                partial=np.asarray(getattr(self.planner,'last_extract_frames',[]),dtype=float)
                if len(partial)>1:
                    valid=self.validity().mask(self.tensor(partial)).cpu().numpy();first_bad=int(np.flatnonzero(~valid)[0]) if not valid.all() else len(partial);partial=partial[:first_bad]
                    if len(partial)>1:
                        candidate=float(np.dot(self.tool(partial[-1],side)[:3,3]-self.tool(partial[0],side)[:3,3],self.wall_outward_direction))
                        if candidate>.001:path,achieved=partial,candidate
            if path is None and self.support_pose is not None:
                path,_=self.plan_supported_extract_candidate(side,self.q,distance)
                if path is not None:achieved=distance
            if path is None:path=self.cartesian_line_fallback(side,self.q,self.wall_outward_direction,distance,active)
            if path is None:
                self.report['segmented_extract']=dict(side=side,segments=segments,final_clearance_m=clearance,incomplete=True)
                return False
            origin=self.tool(self.q,side)
            self.audit(path,active,self.q,line=(side,origin,self.wall_outward_direction,achieved));self.append(path)
            segments.append(dict(distance_m=achieved,points=len(path)))
        self.fail('segmented Cartesian extraction exceeded forty adaptive stages')

    def cartesian_line_fallback(self,side,start,direction,distance,active):
        saved_q=self.q.copy();self.q=np.asarray(start,dtype=float).copy()
        try:
            target=self.tool(self.q,side);target[:3,3]+=np.asarray(direction)*distance
            goals=self.ik(side,target,active)
            if not len(goals):return None
            validity=self.validity();self.planner.active_side=side
            path,stats=plan_cartesian_transport(
                self.planner,self.q,goals,validity.checker,budget_s=2.,seed=self.request.seed+701,
                options={'edge_resolution':.04,'batch':256,'connect_candidates':64,
                         'shortcut':True,'monotonic_progress':True,'strict_budget':True,
                         'budget_reserve_s':.1,'workspace_margin_m':.5},
                validity=validity,active_indices=active)
            if path is None:return None
            path=np.asarray(path,dtype=float);origin=self.tool(path[0],side);direction=np.asarray(direction)
            line_error=angle_error=0.;progress=[]
            for row in path:
                actual=self.tool(row,side);delta=actual[:3,3]-origin[:3,3];along=float(np.dot(delta,direction));progress.append(along)
                line_error=max(line_error,float(np.linalg.norm(delta-direction*along)))
                angle_error=max(angle_error,pose_error(actual,origin)[1])
            accepted=bool(line_error<=.001 and angle_error<=math.radians(.5) and
                          abs(progress[-1]-distance)<=.001 and min(np.diff(progress),default=0.)>=-1e-4)
            self.report['attempts'].append(dict(stage=self.phase,operation='cartesian_rrt_line_fallback',
                side=side,goals=len(goals),stats=stats,accepted=accepted,max_line_error_mm=line_error*1000,
                max_orientation_error_deg=math.degrees(angle_error),distance_m=distance))
            return path if accepted else None
        finally:self.q=saved_q

    def cartesian_contact_fallback(self, side, item, start, target, active):
        saved_q,saved_pair=self.q.copy(),self.contact_pair
        self.q=np.asarray(start,dtype=float).copy()
        self.contact_pair=(side,item.object_id)
        try:
            goals=self.ik(side,target,active)
            if not len(goals):
                return None
            validity=self.validity();self.planner.active_side=side
            path,stats=plan_cartesian_transport(
                self.planner,self.q,goals,validity.checker,budget_s=.75,
                seed=self.request.seed+700,
                options={'edge_resolution':.02,'batch':128,'connect_candidates':32,
                         'shortcut':True,'monotonic_progress':True,'strict_budget':True,
                         'budget_reserve_s':.05,'workspace_margin_m':.08},
                validity=validity,active_indices=active)
            if path is None:
                return None
            path=np.asarray(path,dtype=float)
            origin=self.tool(path[0],side);direction=self.wall_inward_direction
            line_error=angle_error=0.;progress=[]
            for row in path:
                actual=self.tool(row,side);delta=actual[:3,3]-origin[:3,3]
                along=float(np.dot(delta,direction));progress.append(along)
                line_error=max(line_error,float(np.linalg.norm(delta-direction*along)))
                angle_error=max(angle_error,pose_error(actual,origin)[1])
            accepted=bool(line_error<=.001 and angle_error<=math.radians(.5) and
                      abs(progress[-1]-.05)<=.001 and min(np.diff(progress),default=0.)>=-1e-4)
            self.report['attempts'].append(dict(stage=self.phase,operation='cartesian_rrt_contact_fallback',
                side=side,goals=len(goals),stats=stats,accepted=accepted,
                max_line_error_mm=line_error*1000,max_orientation_error_deg=math.degrees(angle_error)))
            return path if accepted else None
        finally:
            self.q,self.contact_pair=saved_q,saved_pair

    def supported_lower_contact_rows(self,item,target,support_mapper,lifts):
        rows=[]
        for lift in lifts:
            local=support_mapper.carriage.inverse(np.asarray([lift]))[0]@target
            for swivel in np.deg2rad(np.arange(-180.,180.,self.swivel_step_deg/2)):
                for arm in self.planner.analytic.solve(0,local,float(swivel),self.q[1:8]):
                    row=self.q.copy();row[0],row[1:8]=lift,arm;rows.append(row)
        rows=np.unique(np.round(rows,7),axis=0) if rows else np.empty((0,15))
        rows,found=support_mapper(rows);rows=rows[found]
        self.contact_pair=('left',item.object_id)
        if len(rows):rows=rows[self.validity().mask(self.tensor(rows)).cpu().numpy()]
        self.contact_pair=None
        rows=rows[self.exact_pose_mask(rows,'left',target)]
        if len(rows):
            weights=np.array([5.]+[1.]*14);rows=rows[self.margin_order(rows,np.linalg.norm((rows-self.q)*weights,axis=1))]
        return rows

    def upper_escape_distance(self,item,endpoint):
        saved=self.store,self.q.copy(),self.support_pose,self.contact_pair
        try:
            self.q=np.asarray(endpoint,dtype=float).copy();self.support_pose=None;self.contact_pair=None
            tool=self.tool(self.q,'right');attachment=AttachedObject(item.object_id,item.dimensions_m,'right_tool0',
                matrix_pose(np.linalg.inv(tool)@pose_matrix(item.pose)),('right_link7',))
            self.store=SceneStore(saved[0].snapshot());self.store.attach(attachment,self.store.snapshot().revision)
            validity=self.validity()
            rows,_=self.planner.analytic_extract(self.q,validity,validity,sides=('right',),
                direction=self.wall_outward_direction,distance_m=self.request.extraction_m)
            candidate=np.asarray(rows if rows is not None else getattr(self.planner,'last_extract_frames',[self.q]),dtype=float)
            valid=validity.mask(self.tensor(candidate)).cpu().numpy()
            first_bad=int(np.flatnonzero(~valid)[0]) if not valid.all() else len(candidate)
            candidate=candidate[:first_bad]
            return (float(np.dot(self.tool(candidate[-1],'right')[:3,3]-self.tool(candidate[0],'right')[:3,3],
                                 self.wall_outward_direction)) if len(candidate)>1 else 0.)
        finally:self.store,self.q,self.support_pose,self.contact_pair=saved

    def upper_endpoint_lower_reachability(self,item,endpoint):
        saved=self.store,self.q.copy(),self.support_pose,self.contact_pair
        counts=[]
        try:
            self.q=np.asarray(endpoint,dtype=float).copy();self.contact_pair=None
            tool=self.tool(self.q,'right')
            attachment=AttachedObject(item.object_id,item.dimensions_m,'right_tool0',
                matrix_pose(np.linalg.inv(tool)@pose_matrix(item.pose)),('right_link7',))
            self.store=SceneStore(saved[0].snapshot());self.store.attach(attachment,self.store.snapshot().revision)
            self.support_pose=self.box_pose(self.q,attachment)
            lower=self.store.snapshot().object(f'wall_box_{self.request.lower_box:02d}')
            validity=self.validity();limits=validity.checker.kinematics.get_joint_limits().position
            low,high=(float(values[0].item()) for values in limits)
            lifts=sorted({float(self.q[0]),*(float(value) for value in self.lift_grid)},key=lambda value:abs(value-float(self.q[0])))
            lifts=[value for value in lifts if low+1e-5<=value<=high-1e-5]
            mapper=SupportCompensation(self,side='right',reference=self.q)
            requested=self.grasp_roll_candidates('left')
            preferred=math.radians(float(getattr(self,'preferred_left_grasp_roll_deg',45.)))
            for roll in sorted(requested,key=lambda value:(abs(value-preferred),requested.index(value))):
                target,proof=self.contact_target('left',lower,roll)
                rows=self.supported_lower_contact_rows(lower,target,mapper,lifts) if proof['valid'] else ()
                counts.append(dict(roll_deg=math.degrees(roll),candidates=len(rows)))
                if len(rows):break
            return any(entry['candidates'] for entry in counts),counts
        finally:
            self.store,self.q,self.support_pose,self.contact_pair=saved

    def contact_upper_ranked(self, object_id, active):
        item = self.store.snapshot().object(object_id)
        self.enter('right_contact_model')
        target, proof = self.contact_target('right', item, 0.)
        entry = dict(side='right', object_id=item.object_id, roll_rad=0.,
                     target=asdict(matrix_pose(target)), proof=proof, selected=False)
        self.report.setdefault('contact_geometry', []).append(entry)
        if not proof['valid']:
            self.fail('upper-box contact mesh proof failed')
        precontact = target.copy();precontact[:3, 3] += self.wall_outward_direction*.05
        self.current_contact_roll_rad = 0.
        self.enter('right_precontact')
        candidates = self.ik('right', precontact, RIGHT)
        options = []
        for candidate in candidates[:64]:
            self.enter('right_contact');self.contact_pair=('right',item.object_id)
            contact=self.cartesian('right',self.wall_inward_direction,.05,active,start=candidate,swivel_continuation=True)
            if contact is None:contact=self.analytic_task_space_bridge(candidate,'right',target,steps=16)
            self.contact_pair=None
            if contact is None:contact=self.cartesian_contact_fallback('right',item,candidate,target,active)
            if contact is not None:
                if getattr(self,'skip_upper_lower_reachability',False):
                    options.append((candidate,contact));break
                if getattr(self,'upper_first',False):
                    escape=self.upper_escape_distance(item,contact[-1])
                    self.report['attempts'].append(dict(stage='right_contact',operation='upper_escape_gate',achieved_m=escape))
                    options.append((candidate,contact,escape))
                    if escape>=.1 or len(options)>=8:break
                    continue
                gate_started=time.perf_counter()
                reachable,downstream=self.upper_endpoint_lower_reachability(item,contact[-1])
                self.record_operation_time('lower_reachability_gate',gate_started)
                self.report['attempts'].append(dict(stage='right_contact',operation='lower_reachability_gate',
                                                    reachable=reachable,rolls=downstream))
                if reachable:
                    options.append((candidate,contact));break
        entry['precontact_candidates'] = len(candidates)
        entry['contact_path_candidates'] = len(options)
        if not options:
            self.fail('upper-box ranked candidates have no certified contact path')
        # Prefer the branch with the largest certified normal escape for upper-first outer columns.
        if getattr(self,'upper_first',False):
            options=[max(options,key=lambda option:option[2])[:2]]
        else:
            options=options[:1]
        entry['downstream_safe_endpoint_pool'] = len(options)
        self.enter('right_precontact')
        goals = np.asarray([item[0] for item in options])
        path = self.plan_precontact_path('right', goals, active)
        if path is None or not self.accept_precontact_path('right', path):
            self.fail('upper-box ranked precontact has no path within the rotation gate')
        selected = int(np.argmin(np.linalg.norm(goals[:, RIGHT]-np.asarray(path)[-1, RIGHT],axis=1)))
        self.audit(path,getattr(self,'precontact_audit_active',active),self.q,
                   target=('right',precontact));self.append(path)
        self.enter('right_contact');self.contact_pair=('right',item.object_id)
        contact=self.cartesian('right',self.wall_inward_direction,.05,active,start=self.q,swivel_continuation=True)
        if contact is None:contact=self.analytic_task_space_bridge(self.q,'right',target,steps=16)
        self.contact_pair=None
        if contact is None:contact=self.cartesian_contact_fallback('right',item,self.q,target,active)
        if contact is None:
            self.fail('selected upper-box contact path changed after ranked approach')
        self.enter('right_contact');self.contact_pair=('right',item.object_id)
        self.audit(contact,active,self.q,target=('right',target),
                   line=('right',self.tool(contact[0],'right'),self.wall_inward_direction,.05))
        self.append(contact);self.contact_pair=None
        self.attach('right',item);entry['selected']=True
        entry['selected_candidate_index']=selected

    def ik(self, side, target, active, support_target=None):
        candidates = super().ik(side, target, active, support_target)
        eligible = ((side == 'left' and self.phase in ('left_extract_target','left_precontact')) or
                    (side == 'right' and self.phase == 'right_precontact' and self.planner.right_precontact_analytic_ik))
        if not eligible or 0 in active:
            return candidates
        indices = LEFT if side == 'left' else RIGHT
        side_index = 0 if side == 'left' else 1
        self.planner.fk(self.q)
        carriage = np.linalg.inv(self.planner.fk_robot.get_transform(
            'arm_carriage', self.planner.fk_robot.base_link))
        rows = []
        for swivel in np.deg2rad(np.arange(-180., 180., self.swivel_step_deg/2)):
            for arm in self.planner.analytic.solve(side_index, carriage@target, float(swivel), self.q[indices]):
                row = self.q.copy();row[indices] = arm;rows.append(row)
        rows = np.unique(np.round(rows, 7), axis=0) if rows else np.empty((0,15))
        if len(rows):
            rows = rows[self.validity().mask(self.tensor(rows)).cpu().numpy()]
        merged = np.unique(np.round(np.vstack([candidates, rows]), 7), axis=0) if len(rows) else candidates
        if len(merged):
            weights = np.array([5.] + [1.]*14)
            distance = np.linalg.norm((merged-self.q)*weights, axis=1)
            frame = joint_frame(self.planner)
            same_branch = np.array([frame.branch(side_index, row[indices]) == frame.branch(side_index, self.q[indices])
                                    for row in merged])
            merged = merged[np.lexsort((distance, ~same_branch, self.limit_tight(merged)))]
        self.report['attempts'].append(dict(stage=self.phase, operation='analytic_ik_enrichment',side=side,
            numerical_candidates=len(candidates), analytic_candidates=len(rows), merged_candidates=len(merged),
            same_branch_candidates=int(sum(joint_frame(self.planner).branch(side_index, row[indices]) ==
                                           joint_frame(self.planner).branch(side_index, self.q[indices]) for row in merged)),
            swivel_step_deg=self.swivel_step_deg/2))
        return merged

    def grasp_roll_candidates(self, side):
        degrees = getattr(self, f'{side}_grasp_rolls_deg', (0., 30., -30., 45., -45.))
        values = tuple(math.radians(float(value)) for value in degrees)
        if not values or not np.isfinite(values).all() or len(set(values)) != len(values):
            raise ValueError('left grasp roll candidates must be finite, nonempty and unique')
        return values

    def accept_precontact_path(self, side, path):
        default_limit = 105. if side == 'right' else 30.
        limit = float(getattr(self, f'{side}_precontact_extra_rotation_max_deg', default_limit))
        metrics = self.path_rotation_metrics(path, side)
        accepted = metrics['extra_deg'] <= limit
        self.report.setdefault('precontact_quality_rejections', []).append(
            dict(stage=self.phase, side=side, roll_deg=math.degrees(getattr(self,'current_contact_roll_rad',0.)),
                 accepted=accepted, limit_deg=limit, rotation=metrics))
        return accepted

    def shortcut_states(self, points, validity):
        """Greedy shortcut: from each kept point, the farthest later point joined by an edge valid at
        0.5 deg. With planner.shortcut_screen_rad, every candidate edge is first checked at that coarser
        resolution and only the coarse-valid ones, farthest first, densely: an edge whose coarse sample
        collides is not taken even where the 0.5 deg samples miss that collision."""
        values = np.asarray(points, dtype=np.float64)
        if len(values) <= 2:
            return values
        screen = getattr(self.planner, 'shortcut_screen_rad', None)
        kept = [0]
        while kept[-1] < len(values)-1:
            source = kept[-1]
            targets = np.arange(len(values)-1, source, -1)
            starts = self.tensor(np.repeat(values[source][None], len(targets), axis=0))
            ends = self.tensor(values[targets])
            if screen is None:
                good = validity.edges(starts, ends, resolution=math.radians(.5)).cpu().numpy()
                kept.append(int(targets[np.argmax(good)]) if good.any() else source+1)
                continue
            chosen = source+1
            screened = np.flatnonzero(validity.edges(starts, ends, resolution=screen).cpu().numpy())
            for offset in range(0, len(screened), 8):
                batch = torch.as_tensor(screened[offset:offset+8], device=starts.device)
                good = validity.edges(starts[batch], ends[batch], resolution=math.radians(.5)).cpu().numpy()
                if good.any():
                    chosen = int(targets[screened[offset+int(np.argmax(good))]])
                    break
            kept.append(chosen)
        return values[kept]

    def tool_poses(self, rows, side):
        """self.tool for many rows: the same URDF joint chain in batched float64.

        Exact for every row. A per-row yourdfpy read can lag it by ~1e-8 per joint: trimesh keeps
        an edge matrix when its replacement is within 1e-8 of the previous read.
        """
        rows = np.asarray(rows, dtype=float).reshape(-1, 15)
        batch_fk = getattr(getattr(self, 'planner', None), 'batch_fk', None)
        if batch_fk is None:
            return np.asarray([self.tool(row, side) for row in rows]).reshape(-1, 4, 4)
        return batch_fk.transforms(rows, side+'_tool0')

    def pose_errors(self, rows, side, target):
        poses = self.tool_poses(rows, side)
        return (np.linalg.norm(poses[:, :3, 3]-target[:3, 3], axis=1),
                Rotation.from_matrix(np.transpose(poses[:, :3, :3], (0, 2, 1))@target[:3, :3]).magnitude())

    def exact_pose_mask(self, rows, side, target, translation_m=.001, angle_rad=math.radians(.5), margin=1e-9):
        """pose_error(self.tool(row, side), target) within tolerance, per row.

        Batched FK decides rows farther than `margin` from both limits; only rows inside that
        band repeat the per-row URDF FK. Decisions equal the per-row ones unless a stale yourdfpy
        read (see tool_poses) puts a row on the other side of a limit within ~1e-8.
        """
        rows = np.asarray(rows, dtype=float).reshape(-1, 15)
        if not len(rows):
            return np.zeros(0, dtype=bool)
        translation, angle = self.pose_errors(rows, side, target)
        result = (translation <= translation_m-margin) & (angle <= angle_rad-margin)
        outside = (translation > translation_m+margin) | (angle > angle_rad+margin)
        for index in np.flatnonzero(~result & ~outside):
            exact_translation, exact_angle = pose_error(self.tool(rows[index], side), target)
            result[index] = exact_translation <= translation_m and exact_angle <= angle_rad
        return result

    def max_pose_drift(self, rows, side, target, translation_m=.001, angle_rad=math.radians(.5), margin=1e-9):
        """Largest tool pose error along rows; rows near or over a limit use the URDF FK."""
        rows = np.asarray(rows, dtype=float).reshape(-1, 15)
        if not len(rows):
            return 0., 0.
        translation, angle = self.pose_errors(rows, side, target)
        for index in np.flatnonzero((translation > translation_m-margin) | (angle > angle_rad-margin)):
            translation[index], angle[index] = pose_error(self.tool(rows[index], side), target)
        return float(translation.max()), float(angle.max())

    def path_rotation_metrics(self, rows, side):
        transforms = self.tool_poses(rows, side)
        rotations = Rotation.from_matrix(np.asarray([matrix[:3, :3] for matrix in transforms]))
        steps = ((rotations[:-1].inv()*rotations[1:]).magnitude()
                 if len(rotations) > 1 else np.zeros(0))
        net = ((rotations[0].inv()*rotations[-1]).magnitude()
               if len(rotations) > 1 else 0.)
        return dict(total_deg=math.degrees(float(steps.sum())), net_deg=math.degrees(float(net)),
                    extra_deg=math.degrees(float(max(0., steps.sum()-net))))

    def line_rotation(self, start, goals, sides, samples=24):
        """Per goal and side, the extra tool rotation (deg) of the joint-space straight line from start."""
        goals = np.asarray(goals, dtype=float).reshape(-1, 15)
        fractions = np.linspace(0., 1., samples)
        rows = start[None, None]+fractions[None, :, None]*(goals[:, None]-start[None, None])
        output = {}
        for side in sides:
            rotations = self.tool_poses(rows.reshape(-1, 15), side)[:, :3, :3].reshape(len(goals), samples, 3, 3)
            relative = np.einsum('gsji,gsjk->gsik', rotations[:, :-1], rotations[:, 1:])
            steps = np.arccos(np.clip((np.trace(relative, axis1=2, axis2=3)-1)/2, -1., 1.)).sum(1)
            net = np.einsum('gji,gjk->gik', rotations[:, 0], rotations[:, -1])
            net = np.arccos(np.clip((np.trace(net, axis1=1, axis2=2)-1)/2, -1., 1.))
            output[side] = np.degrees(np.maximum(0., steps-net))
        return output

    def rotation_gates(self, start, goals, fixed):
        """{side: gate deg}: the fixed gates, or for the goal whose straight line exceeds them least
        its line rotation plus DoorPlanner.rotation_gate_slack_deg where that is larger."""
        slack = getattr(self.planner, 'rotation_gate_slack_deg', None)
        if slack is None or not len(goals):
            return dict(fixed)
        lines = self.line_rotation(start, goals, tuple(fixed))
        best = int(np.argmin(np.max([lines[side]-limit for side, limit in fixed.items()], axis=0)))
        return {side: max(limit, float(lines[side][best])+slack) for side, limit in fixed.items()}

    def straight_line_path(self, start, goals, validity, sides, limit=16):
        """(path, goal index) of the densified joint-space straight line from `start` to the first goal,
        in order of line tool rotation (5 deg buckets of the worst of `sides`, then the given goal
        order), whose rows all pass `validity`; (None, None) if none of the first `limit` lines does."""
        goals = np.asarray(goals, dtype=float).reshape(-1, 15)
        if not len(goals):
            return None, None
        lines = self.line_rotation(start, goals, sides)
        worst = np.max([lines[side] for side in sides], axis=0)
        order = np.lexsort((np.arange(len(goals)), np.floor(worst/5.)))[:limit]
        paths = [np.asarray(densify([np.asarray(start, dtype=float), goals[index]])) for index in order]
        valid = validity.mask(self.tensor(np.vstack(paths))).cpu().numpy()
        end = 0
        for index, path in zip(order, paths):
            end += len(path)
            if valid[end-len(path):end].all():
                return path, int(index)
        return None, None

    def upper_precontact_staging_goals(self, right_goals):
        lower = self.store.snapshot().object(f'wall_box_{self.request.lower_box:02d}')
        variants = []
        for right_goal in np.asarray(right_goals):
            self.planner.fk(right_goal)
            carriage = np.linalg.inv(self.planner.fk_robot.get_transform(
                'arm_carriage', self.planner.fk_robot.base_link))
            for roll in self.grasp_roll_candidates('left'):
                target, proof = self.contact_target('left', lower, roll)
                if not proof['valid']:
                    continue
                target = target.copy();target[:3,3] += self.wall_outward_direction*.05
                arms = []
                for swivel in np.deg2rad(np.arange(-180.,180.,self.swivel_step_deg)):
                    arms.extend(self.planner.analytic.solve(0,carriage@target,float(swivel),self.q[1:8]))
                if not arms:
                    continue
                arms = np.unique(np.round(arms,7),axis=0)
                rows = np.repeat(right_goal[None], len(arms), axis=0);rows[:,1:8] = arms
                order = self.margin_order(rows, np.linalg.norm(arms-self.q[1:8],axis=1))[:4]
                for arm in arms[order]:
                    row=right_goal.copy();row[1:8]=arm;variants.append(row)
        rows=np.unique(np.round(variants,7),axis=0) if variants else np.empty((0,15))
        if len(rows):rows=rows[self.validity().mask(self.tensor(rows)).cpu().numpy()]
        return rows

    def sequential_upper_precontact(self, staged, validity, deadline=None):
        """Stage the lower arm first, then approach the upper box in Cartesian space (by `deadline`)."""
        def budget(cap):
            return cap if deadline is None else min(cap, max(0., deadline-time.perf_counter()))
        if not len(staged):
            return None, None
        staged = np.asarray(staged, dtype=float)
        frame = joint_frame(self.planner)
        if not any(frame.branch(1, row[RIGHT]) == frame.branch(1, self.q[RIGHT]) for row in staged):
            return None, dict(failure='right_branch_change_requires_joint_route')
        left_goals = staged.copy()
        left_goals[:, 0] = self.q[0]
        left_goals[:, RIGHT] = self.q[RIGHT]
        left_goals = np.unique(np.round(left_goals, 7), axis=0)
        left_goals = left_goals[validity.mask(self.tensor(left_goals)).cpu().numpy()]
        if not len(left_goals):
            return None, dict(failure='no_valid_left_staging_goal')
        self.planner.active_side = 'left'
        left_path, left_stats = plan_cartesian_transport(
            self.planner, self.q, left_goals, validity.checker, budget_s=budget(2.),
            seed=self.request.seed+450,
            options={'edge_resolution': .08, 'batch': 128, 'connect_candidates': 32,
                     'shortcut': True, 'monotonic_progress': True,
                     'strict_budget': True, 'budget_reserve_s': .1,
                     'workspace_margin_m': .5},
            validity=self.search_inflation(validity, self.q, left_goals), active_indices=LEFT)
        if left_path is None:
            joint_paths = []
            for _ in range(4):
                if budget(2.) <= .05:
                    break
                candidate = self.search(left_goals, LEFT, budget_s=budget(2.), free_space=True)
                if candidate is not None:
                    joint_paths.append((self.path_rotation_metrics(candidate, 'left')['extra_deg'],
                                        np.asarray(candidate),
                                        self.report['attempts'][-1].get('stats')))
            if not joint_paths:
                self.planner.active_side = 'right'
                return None, dict(left=left_stats, failure='left_staging_failed')
            _, left_path, joint_stats = min(joint_paths, key=lambda item:item[0])
            left_stats = dict(cartesian=left_stats, joint=joint_stats,
                              source='left_joint_rrt_fallback')
        staged = np.asarray(staged, dtype=float)
        distance = np.linalg.norm(staged[:, LEFT]-np.asarray(left_path)[-1, LEFT], axis=1)
        right_goals = staged[distance <= distance.min()+1e-6]
        start = np.asarray(left_path)[-1]
        active_right = RIGHT if np.allclose(right_goals[:, 0], start[0], atol=1e-6, rtol=0) else [0]+RIGHT
        self.planner.active_side = 'right'
        if budget(2.) <= .05:
            return None, dict(left=left_stats, failure='precontact_budget_exhausted')
        right_path, right_stats = plan_cartesian_transport(
            self.planner, start, right_goals, validity.checker, budget_s=budget(2.),
            seed=self.request.seed+451,
            options={'edge_resolution': .08, 'batch': 128, 'connect_candidates': 32,
                     'shortcut': True, 'monotonic_progress': True,
                     'strict_budget': True, 'budget_reserve_s': .1,
                     'workspace_margin_m': .5},
            validity=self.search_inflation(validity, start, right_goals), active_indices=active_right)
        if right_path is None:
            return None, dict(left=left_stats, right=right_stats, failure='right_approach_failed')
        path = np.vstack([np.asarray(left_path), np.asarray(right_path)[1:]])
        return path, dict(left=left_stats, right=right_stats,
                          split_index=len(left_path)-1,
                          active_right=list(active_right))

    def plan_precontact_path(self, side, goals, active):
        """Free-space approach to the precontact goals: searches inflated per DoorPlanner.free_space_inflation,
        coverage-grown spheres for the moving arms except next to the start and the goals (MOTION-429)."""
        self.planner.active_side = side
        if side == 'right' and not self.store.snapshot().attachments:
            staged = self.upper_precontact_staging_goals(goals)
            with self.free_space(('left', 'right'), [self.q, *np.asarray(goals), *staged]):
                return self.plan_upper_precontact_path(goals, staged)
        with self.free_space((side,), [self.q, *np.asarray(goals)]):
            return self.plan_side_precontact_path(side, goals, active)

    def plan_upper_precontact_path(self, goals, staged):
        side = 'right'
        validity = self.validity()
        alternatives = []
        budget_s = 6.
        deadline = time.perf_counter()+budget_s
        gates = self.rotation_gates(self.q, staged if len(staged) else np.asarray(goals), dict(
            left=65., right=float(getattr(self,'right_precontact_extra_rotation_max_deg',105.))))
        staging_limit, right_limit = gates['left'], gates['right']
        # Joint routes run until one passes both rotation limits or the budget ends; the Cartesian
        # strategies stop early enough to leave at least three of them their 0.75 s each.
        joint_seeds=(47,20,38,65,41,17)
        cartesian_deadline = deadline-.75*3

        def remaining(cap=None, until=None):
            value=max(0.,(deadline if until is None else until)-time.perf_counter())
            return value if cap is None else min(cap,value)

        def add(path, stats, source):
            if path is None:
                return False
            path=np.asarray(path,dtype=float).reshape(-1,15)
            left=self.path_rotation_metrics(path,'left')
            right=self.path_rotation_metrics(path,'right')
            score=max(left['extra_deg'],right['extra_deg'])+.1*(left['extra_deg']+right['extra_deg'])
            alternatives.append((score,path,left,right,stats,source))
            return left['extra_deg']<=staging_limit and right['extra_deg']<=right_limit

        right_active = RIGHT if np.allclose(np.asarray(goals)[:, 0], self.q[0], atol=1e-6, rtol=0) else [0]+RIGHT
        target_rows = staged if len(staged) else np.asarray(goals)
        quality_good=False
        right_stats=sequential_stats=dual_stats=None
        joint_attempts=0

        def cartesian_routes(until):
            nonlocal quality_good, right_stats, sequential_stats, dual_stats
            direct_budget=remaining(2.,until)
            right_path=None
            if direct_budget>.05:
                right_path, right_stats = plan_cartesian_transport(
                    self.planner, self.q, goals, validity.checker, budget_s=direct_budget,
                    seed=self.request.seed+440,
                    options={'edge_resolution': .08, 'batch': 128, 'connect_candidates': 32,
                             'shortcut': True, 'monotonic_progress': True,
                             'strict_budget': True, 'budget_reserve_s': .1,
                             'workspace_margin_m': .5},
                    validity=self.search_inflation(validity, self.q, goals), active_indices=right_active)
            quality_good=add(right_path,right_stats,'right_cartesian_precontact') or quality_good
            if not quality_good and remaining(until=until)>.05:
                sequential_path, sequential_stats = self.sequential_upper_precontact(staged, validity, until)
                quality_good=add(sequential_path,sequential_stats,'sequential_cartesian_staging')
            if not quality_good and remaining(until=until)>.05:
                dual_budget=remaining(2.,until)
                dual_path, dual_stats = plan_dual_cartesian_transport(
                    self.planner, self.q[None], target_rows, validity.checker,
                    budget_s=dual_budget, seed=self.request.seed+500,
                    options={'shortcut': True, 'monotonic_progress': True,
                             'strict_budget': True, 'batch': 128,
                             'connect_candidates': 32, 'dual_direct_pairs': 64,
                             'dual_direct_share': .5, 'dual_postprocess_reserve_s': .05},
                    validity=self.search_inflation(validity, self.q, target_rows))
                quality_good=add(dual_path,dual_stats,'dual_cartesian_rrt')

        def joint_routes(enough=None):
            nonlocal quality_good, joint_attempts
            for joint_seed in joint_seeds:
                if quality_good or remaining()<=.05 or (enough is not None and len(alternatives) >= enough):
                    break
                # First densely verified path, then a bounded refinement window (the shortcut and
                # rotation gate below are unchanged) instead of always spending the whole cap.
                path = self.search(target_rows,list(range(15)),budget_s=remaining(.75),
                                   seed_value=joint_seed,stop_at_first_valid=True,refinement_s=.25,free_space=True)
                joint_attempts+=1
                if path is None:
                    continue
                stride=max(1,len(path)//128)
                sparse=np.asarray(path)[::stride]
                if not np.allclose(sparse[-1],np.asarray(path)[-1],atol=1e-12,rtol=0):
                    sparse=np.vstack([sparse,np.asarray(path)[-1]])
                path=np.asarray(densify(self.shortcut_states(sparse,validity)))
                quality_good=add(path,self.report['attempts'][-1].get('stats'),'dual_joint_rrt_shortcut')

        if getattr(self.planner, 'precontact_joint_first', False):
            # The joint routes are what reach the re-zeroed model's contacts from Home; the
            # Cartesian strategies stay as a fallback for the remaining budget.
            joint_routes(enough=2)
            if not alternatives:
                cartesian_routes(deadline)
        else:
            cartesian_routes(cartesian_deadline)
            joint_routes()
        if alternatives:
            _,path,left,right,stats,source=min(alternatives,key=lambda item:item[0])
            self.precontact_audit_active=list(range(15))
            self.report.setdefault('precontact_searches',[]).append(dict(
                stage=self.phase,side=side,roll_deg=math.degrees(getattr(self,'current_contact_roll_rad',0.)),
                source=source+'_with_lower_staging',budget_s=budget_s,joint_attempts=joint_attempts,
                rotation_gates_deg=gates,
                success=True,selected_stats=stats,sequential_stats=sequential_stats,
                left_rotation=left,right_rotation=right,staging_goals=len(staged),
                alternatives=[dict(source=item[5],left_rotation=item[2],right_rotation=item[3])
                              for item in alternatives]))
            return path
        self.report.setdefault('precontact_searches',[]).append(dict(
            stage=self.phase,side=side,source=None,budget_s=budget_s,joint_attempts=joint_attempts,
            success=False,sequential_stats=sequential_stats,staging_goals=len(staged),
            right_cartesian_stats=right_stats,dual_cartesian_stats=dual_stats))
        return None

    def plan_side_precontact_path(self, side, goals, active):
        validity = self.validity()
        mapper = None
        search_active = list(active)
        derived = []
        if side == 'left' and self.support_pose is not None:
            mapper = SupportCompensation(self, side='right', reference=self.q)
            search_active = [0]+LEFT
            derived = RIGHT
        path, stats = plan_cartesian_transport(
            self.planner, self.q, goals, validity.checker, budget_s=4.,
            seed=self.request.seed+self.counter,
            options={'edge_resolution': .08, 'batch': 128, 'connect_candidates': 32,
                     'shortcut': True, 'monotonic_progress': True,
                     'strict_budget': True, 'budget_reserve_s': .25,
                     'workspace_margin_m': .5},
            validity=self.search_inflation(validity, self.q, goals), active_indices=search_active,
            row_mapper=mapper, derived_indices=derived)
        alternatives = []
        if path is not None:
            metrics = self.path_rotation_metrics(path, side)
            alternatives.append(('cartesian_rrt', path, metrics, stats))
        if path is None or alternatives[0][2]['extra_deg'] > 30.:
            if mapper is None:
                refinement = self.planner.side_precontact_refinement_s
                options = {} if refinement is None else dict(stop_at_first_valid=True, refinement_s=refinement)
                for _ in range(3):
                    candidate = self.search(goals, active, free_space=True, **options)
                    if candidate is not None:
                        metrics = self.path_rotation_metrics(candidate, side)
                        alternatives.append(('joint_rrt', candidate, metrics,
                                             self.report['attempts'][-1].get('stats')))
                        if refinement is not None and metrics['extra_deg'] <= 30.:
                            break
            else:
                reduced = SupportCompensatedValidity(validity,self.q,mapper,searched_indices=search_active)
                searched = SupportCompensatedValidity(self.search_inflation(validity,self.q,goals),self.q,mapper,
                                                      searched_indices=search_active)
                lower,upper = validity.checker.kinematics.get_joint_limits().position
                for attempt in range(3):
                    sparse,joint_stats=constrained_rrt_connect(
                        self.tensor(self.q[search_active]),self.tensor(np.asarray(goals)[:,search_active]),
                        lower[search_active]+1e-5,upper[search_active]-1e-5,searched,2.,
                        self.request.seed+200+attempt,stop_at_first_valid=True,
                        batch=256,connect_candidates=64,step=.28)
                    if sparse is None:
                        continue
                    sparse=self.shortcut_states(np.asarray(sparse),reduced)
                    dense=np.asarray(densify(sparse,reduced.weights.cpu().tolist()))
                    candidate=reduced.expand(self.tensor(dense)).cpu().numpy()
                    # A repeated path cannot change the min-rotation choice below; stop retrying.
                    if any(item[0]=='support_compensated_joint_rrt' and len(item[1])==len(candidate) and
                           np.allclose(item[1],candidate,atol=1e-9,rtol=0) for item in alternatives):
                        break
                    alternatives.append(('support_compensated_joint_rrt',candidate,
                                         self.path_rotation_metrics(candidate,side),joint_stats))
        if not alternatives:
            self.report.setdefault('precontact_searches', []).append(
                dict(stage=self.phase, side=side, source=None,
                     cartesian_stats=stats, success=False))
            return None
        source, path, metrics, selected_stats = min(
            alternatives, key=lambda item: (item[2]['extra_deg'], item[2]['total_deg'], len(item[1])))
        path = np.asarray(path, dtype=float).reshape(-1, 15)
        self.precontact_audit_active = sorted(set(search_active)|set(derived))
        self.report.setdefault('precontact_searches', []).append(dict(
            stage=self.phase, side=side, roll_deg=math.degrees(getattr(self,'current_contact_roll_rad',0.)),
            source=source, cartesian_stats=stats,
            selected_stats=selected_stats, success=True, rotation=metrics,
            alternatives=[dict(source=item[0], rotation=item[2]) for item in alternatives]))
        return path

    def plan_supported_extract_candidate(self,side,contact_q,distance_m=None):
        distance_m=self.request.extraction_m if distance_m is None else float(distance_m)
        validity=self.validity();support_side='right' if side=='left' else 'left'
        moving=LEFT if side=='left' else RIGHT;support=RIGHT if side=='left' else LEFT;side_index=0 if side=='left' else 1
        mapper=SupportCompensation(self,side=support_side,reference=contact_q)
        limits=validity.checker.kinematics.get_joint_limits().position;low,high=(float(values[0].item()) for values in limits)
        lifts=sorted({float(contact_q[0]),*(float(value) for value in self.lift_grid)},key=lambda value:abs(value-float(contact_q[0])))
        lifts=[value for value in lifts if low+1e-5<=value<=high-1e-5]
        origin=self.tool(contact_q,side);target=origin.copy();target[:3,3]+=self.wall_outward_direction*distance_m
        endpoints=[]
        for lift in lifts:
            local=mapper.carriage.inverse(np.asarray([lift]))[0]@target
            for swivel in np.deg2rad(np.arange(-180.,180.,self.swivel_step_deg)):
                for arm in self.planner.analytic.solve(side_index,local,float(swivel),contact_q[moving]):
                    row=contact_q.copy();row[0],row[moving]=lift,arm;endpoints.append(row)
        endpoints=np.unique(np.round(endpoints,7),axis=0) if endpoints else np.empty((0,15))
        mapped,found=mapper(endpoints);endpoints=mapped[found]
        if len(endpoints):endpoints=endpoints[validity.mask(self.tensor(endpoints)).cpu().numpy()]
        self.planner.active_side=side;path=stats=None;search_seed=self.request.seed+101+self.counter;self.counter+=1
        if len(endpoints):
            path,stats=plan_cartesian_transport(self.planner,contact_q,endpoints,validity.checker,budget_s=2.,seed=search_seed,
                options={'edge_resolution':.05,'batch':128,'connect_candidates':32,'shortcut':True,
                         'monotonic_progress':True,'strict_budget':True,'budget_reserve_s':.25,'workspace_margin_m':.05},
                validity=validity,active_indices=[0]+moving,row_mapper=mapper,derived_indices=support)
        details=dict(strategy='support_compensated_cartesian_rrt',search_seed=search_seed,
                     audit_active_indices=list(range(15)),endpoint_candidates=len(endpoints),stats=stats,success=path is not None)
        if path is not None:
            path=np.asarray(path);maximum_line=maximum_angle=0.
            for row in path:
                actual=self.tool(row,side);delta=actual[:3,3]-origin[:3,3];progress=float(np.dot(delta,self.wall_outward_direction))
                maximum_line=max(maximum_line,float(np.linalg.norm(delta-self.wall_outward_direction*progress)))
                maximum_angle=max(maximum_angle,pose_error(actual,origin)[1])
            details.update(max_line_error_mm=maximum_line*1000,max_orientation_error_deg=math.degrees(maximum_angle),
                           rotation=self.path_rotation_metrics(path,side))
            if maximum_line>.001 or maximum_angle>math.radians(.5):path=None;details.update(success=False,rejection='task_space_line_gate')
        return path,details

    def append(self, rows):
        rows = np.asarray(rows, dtype=float)
        if len(self.report['frames']) and not np.allclose(rows[0], self.q, atol=1e-6, rtol=0):
            rows = np.vstack([self.q, rows])
        first = len(self.report['frames'])
        snapshot, support, contact = self.store.snapshot(), self.support_pose, self.contact_pair
        super().append(rows)
        added=len(self.report['frames'])-first
        self.base_pose_by_frame.extend([self.pick_base_pose]*added)
        self.motion_segments.append((first,len(self.report['frames'])-1,snapshot,support,contact))

    def envelope(self, rows):
        validity = self.validity()
        state = validity.checker.kinematics.compute_kinematics(
            JointState.from_position(self.tensor(rows)[None], joint_names=list(ACTIVE_JOINTS)))
        spheres = state.robot_spheres.reshape(len(rows), -1, 4)
        distances = torch.linalg.vector_norm(spheres[..., :2], dim=-1) + spheres[..., 3]
        return torch.where(spheres[..., 3] > 0, distances, 0.).max(-1).values.cpu().numpy()

    def coordinated_candidates(self):
        """Generate shared A/B goals with lift free and the support box fixed in world."""
        mapper = SupportCompensation(self)
        validity = self.validity()
        limits = validity.checker.kinematics.get_joint_limits().position
        low, high = (float(values[0].item()) for values in limits)
        lifts = sorted({float(self.q[0]), *(float(x) for x in self.lift_grid)},
                       key=lambda value: abs(value-float(self.q[0])))
        lifts = [value for value in lifts if low+1e-5 <= value <= high-1e-5
                 and value-self.lower_distance >= low+1e-5]
        raw = []
        for lift in lifts:
            target = mapper.carriage.inverse(np.asarray([lift]))[0] @ self.targets['left']
            for swivel in np.deg2rad(np.arange(-180., 180., self.swivel_step_deg)):
                for arm in self.planner.analytic.solve(0, target, float(swivel), self.q[1:8]):
                    row = self.q.copy()
                    row[0], row[1:8] = lift, arm
                    raw.append(row)
        raw = np.unique(np.round(raw, 7), axis=0) if raw else np.empty((0, 15))
        mapped, found = mapper(raw)
        mapped = mapped[found]
        if len(mapped):
            mapped = mapped[validity.mask(self.tensor(mapped)).cpu().numpy()]
        mapped = mapped[self.exact_pose_mask(mapped, 'left', self.targets['left'])].reshape(-1, 15)
        before_sweep = len(mapped)
        self.future_lowering_paths = {}
        if len(mapped):
            weights = np.array([5.] + [1.]*14)
            order = self.margin_order(mapped, np.linalg.norm((mapped-self.q)*weights, axis=1))
            mapped = mapped[order[:64]]
        diagnostics = dict(lifts=lifts, analytic_candidates=len(raw), support_ik_candidates=int(found.sum()),
                           current_state_valid_candidates=before_sweep,
                           future_lowering_valid=None, selected_goal_cap=64,
                           support_mapper_calls=mapper.calls, support_mapper_failures=mapper.failures,
                           lowering_attempts=0, lowering_successes=0,
                           lowering_validation='lazy: validate only the selected recenter endpoint',
                           lowering_constraint='existing dual Cartesian bridge: upper descends while left TCP remains fixed')
        return mapped, mapper, validity, diagnostics

    def validate_future_lowering(self, candidate):
        """Validate the one recenter endpoint selected by search; blacklist/replan on failure."""
        candidate=np.asarray(candidate,dtype=float)
        support=self.support_pose
        attempts=[]
        self.support_pose=None
        try:
            future=self.validity()
            limits=future.checker.kinematics.get_joint_limits().position
            low,high=(float(values[0].item()) for values in limits)
            preferred=float(candidate[0]-self.lower_distance)
            current_lift=float(candidate[0])
            remaining={preferred,*(float(value) for value in self.lift_grid)}-{current_lift}
            lift_ends=[current_lift]+sorted(remaining,key=lambda value:abs(value-current_lift))
            lift_ends=[value for value in lift_ends if low+1e-5<=value<=high-1e-5]
            for lift_end in lift_ends:
                rows,details=self.planner.analytic_extract(
                    candidate,future,future,sides=('right',),direction=(0.,0.,-1.),
                    distance_m=self.lower_distance,lift_end=lift_end,
                    hold_sides=('left',),swivel_continuation=True)
                attempts.append(dict(start_updown=current_lift,end_updown=lift_end,
                                     success=rows is not None,details=details))
                if rows is not None:
                    line_limit_mm=1.2 if getattr(self,'report',{}).get('pair_strategy')=='outer_standard_rack_supported_lower_recenter' else 1.
                    if isinstance(details,dict) and details.get('max_line_error_mm',0.)>line_limit_mm:
                        attempts[-1]['success']=False;attempts[-1]['rejection']='strict_1mm_line_gate';continue
                    return np.asarray(rows),attempts
        finally:
            self.support_pose=support
        return None,attempts

    def latest_compact_candidates(self):
        """Adapt the latest placement atlas to the exact door attachment transforms."""
        # The re-zeroed model's atlas is the policy atlas re-solved for its tool point (DOOR_MODEL.json).
        atlas_path = getattr(self.planner, 'recenter_atlas', None) or ROOT / self.policy['side']['atlas_path']
        atlas = np.load(atlas_path, allow_pickle=False)['joints'].astype(np.float32)
        # ponytail: fixed local cloud around the approved atlas; replace with an offline regenerated
        # attachment-conditioned atlas if this central door scenario becomes a production family.
        rng = np.random.default_rng(20261104)
        noise_scale = np.r_[.015, np.full(14, math.radians(6))]
        rows = np.concatenate([atlas] + [atlas+rng.normal(0., noise_scale, atlas.shape)
                                        for _ in range(32)]).astype(np.float32)
        validity = self.validity()
        limits = validity.checker.kinematics.get_joint_limits().position
        low, high = limits[0].cpu().numpy()+1e-5, limits[1].cpu().numpy()-1e-5
        rows = np.clip(rows, low, high)
        # The selection is the first 64 rows in (pose score, distance) order that pass collision and
        # the envelope. Rank all rows by FK first, then gate them in that order and stop at 64: the
        # same rows in the same order as gating every row, without checking ~90k loaded states.
        # With planner.pose_score_bucket, scores within one bucket are equally good poses and the nearer
        # row comes first (MOTION-429: the 64 lowest float32 scores can lie 9 rad away). No joint-limit
        # margin here: the carry pose itself holds J5/J6 within 15 deg of their limits in every exact row.
        target_poses = {side: matrix_pose(target) for side, target in self.targets.items()}
        scores, pose_errors = [], []
        for offset in range(0, len(rows), 1024):
            values = self.tensor(rows[offset:offset+1024])
            state = validity.checker.kinematics.compute_kinematics(
                JointState.from_position(values[None], joint_names=list(ACTIVE_JOINTS)))
            tools = state.tool_poses.to_dict()
            batch_score = torch.zeros(len(values), device='cuda')
            batch_error = torch.zeros(len(values), device='cuda')
            for side, target in target_poses.items():
                pose = tools[side+'_tool0']
                position = torch.linalg.vector_norm(
                    pose.position.reshape(-1, 3)-self.tensor(target.position), dim=-1)
                quaternion = pose.quaternion.reshape(-1, 4)
                cosine = (quaternion*self.tensor(target.quaternion_wxyz)).sum(-1).abs().clamp(0., 1.)
                angle = 2*torch.acos(cosine)
                batch_score += 10*position+angle
                batch_error = torch.maximum(batch_error, position+angle)
            scores.extend(batch_score.cpu().tolist())
            pose_errors.extend(batch_error.cpu().tolist())
        scores, pose_errors = np.asarray(scores), np.asarray(pose_errors)
        bucket = getattr(self.planner, 'pose_score_bucket', None)
        settled = getattr(self.planner, 'latest_pair_settled_arm_deg', None)
        keys = [np.linalg.norm(rows-self.q, axis=1)]
        if settled is not None:
            # The left arm recentered its box already: rows that leave it in place before rows that
            # reach the same pose in another wrist configuration (up to 500 deg of joint travel).
            keys.append(np.degrees(np.abs(rows[:, LEFT]-self.q[LEFT])).max(1) > settled)
        keys.append(scores if bucket is None else np.floor(scores/bucket))
        ranked = np.lexsort(tuple(keys))
        order, radii, scanned, collision_valid = [], [], 0, 0
        for offset in range(0, len(ranked), 4096):
            batch = ranked[offset:offset+4096]
            valid = validity.mask(self.tensor(rows[batch])).cpu().numpy()
            scanned += len(batch)
            collision_valid += int(valid.sum())
            batch = batch[valid]
            for start in range(0, len(batch), 1024):
                radius = self.envelope(rows[batch[start:start+1024]])
                good = radius <= self.radius
                order.extend(batch[start:start+1024][good].tolist())
                radii.extend(radius[good].tolist())
            if len(order) >= 64:
                break
        envelope_valid = len(order)
        order, radii = np.asarray(order[:64], dtype=int), np.asarray(radii[:64])
        if not len(order):
            return np.empty((0, 15)), dict(atlas_rows=len(atlas), sampled_rows=33*len(atlas), scanned_rows=scanned,
                                           collision_valid=collision_valid, envelope_valid=0)
        selected = rows[order]
        details = dict(atlas_path=str(atlas_path), atlas_sha256=hashlib.sha256(atlas_path.read_bytes()).hexdigest(),
                       atlas_rows=len(atlas), sampled_rows=33*len(atlas), scanned_rows=scanned,
                       gate_order=('pose score' if bucket is None else 'pose score bucket')
                                  +('' if settled is None else f', left arm within {settled:g} deg')
                                  +' then distance; collision and envelope counts cover scanned rows only',
                       pose_score_bucket=bucket,
                       collision_valid=collision_valid, envelope_valid=envelope_valid, selected=len(selected),
                       selected_radius_max_m=float(np.max(radii)),
                       selected_pose_score_max=float(np.max(scores[order])),
                       selected_pose_error_proxy_max=float(np.max(pose_errors[order])),
                       policy_radius_m=self.radius, sampling_seed=20261104,
                       noise_updown_std_m=.015, noise_joint_std_deg=6.)
        return selected, details

    def recenter_latest_pair(self):
        self.enter('right_recenter')
        # Both boxes are out of the wall: both arms plan with coverage-grown spheres (MOTION-429).
        with self.free_space(('left', 'right'), [self.q]):
            path = self.latest_pair_path()
        self.audit(path, list(range(15)), self.q)
        self.append(path)
        radius = float(self.envelope(self.q[None])[0])
        if radius > self.radius:
            self.fail('selected latest compact endpoint exceeded the whole-robot payload envelope')

    def latest_pair_path(self):
        began = time.perf_counter()
        candidates, details = self.latest_compact_candidates()
        details['generation_ms'] = self.record_operation_time('atlas_candidates', began)
        self.report['latest_compact_atlas'] = details
        if not len(candidates):
            self.fail('door-conditioned latest compact atlas has no valid endpoint')
        fixture = dict(stage=self.phase, side='right', start=self.q.tolist(),
                       candidates=np.asarray(candidates).tolist(), active_indices=list(range(15)),
                       derived_indices=[], snapshot=self.store.snapshot().to_dict(), support_pose=None,
                       target=None, seed=self.request.seed, budget_s=2.)
        fixture['identity'] = digest(fixture)
        self.report['recenter_inputs'].append(fixture)
        alternatives = []
        stats = []
        stop_reason = 'bank_exhausted_or_gate_met'
        seed_bank=(65,41,17,71,53,89)
        fallback_s=self.planner.latest_pair_fallback_s
        gates=self.rotation_gates(self.q, candidates, dict(left=65., right=65.))

        def scored(path, search_stats):
            left = self.path_rotation_metrics(path, 'left')
            right = self.path_rotation_metrics(path, 'right')
            joints = np.asarray(path)[:, 1:]
            backtrack = float(np.degrees(
                np.abs(np.diff(joints, axis=0)).sum()-np.abs(joints[-1]-joints[0]).sum()))
            extra_sum = left['extra_deg']+right['extra_deg']
            score = max(left['extra_deg'],right['extra_deg'])+.1*extra_sum+.01*max(0.,backtrack)
            return score, path, left, right, search_stats

        refine_s = .75

        def bank():
            if not fallback_s:
                yield from ((seed, refine_s) for seed in seed_bank)
                return
            # Short trees miss the far wrist branches under the coverage margins: until a seed has a
            # path, up to three seeds search up to fallback_s each for their first path (a found one can
            # still fail the dense check); the others refine the rotation in refine_s.
            long_attempts = 0
            for seed in seed_bank:
                if not alternatives and long_attempts < 3:
                    long_attempts += 1
                    yield seed, fallback_s
                else:
                    yield seed, refine_s
        misses = 0
        for search_seed, budget in bank():
            seed_started = time.perf_counter()
            # Each seed contributes its first densely verified path, as when one 0.75 s iteration
            # was all the loaded checker allowed: weighted-length refinement does not optimize
            # the rotation score used below and measurably worsened it once checks became fast.
            path = self.search(candidates,list(range(15)),budget_s=budget,seed_value=search_seed,
                               stop_at_first_valid=True,free_space=True)
            self.record_operation_time('seed_bank_search', seed_started)
            search_stats = self.report['attempts'][-1].get('stats')
            stats.append(dict(seed=search_seed,stats=search_stats))
            post_started = time.perf_counter()
            if path is None and fallback_s and alternatives:
                misses += 1
                if misses == 2:
                    stop_reason = 'consecutive_misses'
                    break
            if path is not None:
                misses = 0
                validity = self.validity()
                stride = max(1, len(path)//128)
                sparse = np.asarray(path)[::stride]
                if not np.allclose(sparse[-1], np.asarray(path)[-1], atol=1e-12, rtol=0):
                    sparse = np.vstack([sparse, np.asarray(path)[-1]])
                path = np.asarray(densify(self.shortcut_states(sparse, validity)))
                # A repeated path adds nothing to the min-score choice; treat it as the bank converging.
                duplicate = any(len(item[1]) == len(path) and np.allclose(item[1], path, atol=1e-9, rtol=0)
                                for item in alternatives)
                alternatives.append(scored(path, search_stats))
                _, _, left, right, _ = alternatives[-1]
                self.record_operation_time('seed_bank_postprocess', post_started)
                if left['extra_deg'] <= gates['left'] and right['extra_deg'] <= gates['right']:
                    break
                if duplicate:
                    stop_reason = 'duplicate_path'
                    break
                if self.planner.latest_pair_first_path:
                    stop_reason = 'first_path'
                    break
        self.report['latest_compact_search'] = dict(
            seed_bank=list(seed_bank), fallback_s=fallback_s, rotation_gates_deg=gates, stop_reason=stop_reason,
            attempts=stats, alternatives=[dict(score=item[0],left_rotation=item[2],
                                                right_rotation=item[3],stats=item[4])
                                         for item in alternatives])
        if not alternatives:
            self.fail('latest compact atlas multi-seed bank exhausted')
        _, path, _, _, _ = min(alternatives,key=lambda item:item[0])
        return path

    def coordinated_right_candidates(self):
        """Move the upper box to its carry target while the lower box stays recentered."""
        mapper = SupportCompensation(self, side='left', tool_target=self.targets['left'])
        validity = self.validity()
        limits = validity.checker.kinematics.get_joint_limits().position
        low, high = (float(values[0].item()) for values in limits)
        lifts = sorted({float(self.q[0]), *(float(x) for x in self.lift_grid)},
                       key=lambda value: abs(value-float(self.q[0])))
        lifts = [value for value in lifts if low+1e-5 <= value <= high-1e-5]
        raw = []
        for lift in lifts:
            target = mapper.carriage.inverse(np.asarray([lift]))[0] @ self.targets['right']
            for swivel in np.deg2rad(np.arange(-180., 180., self.swivel_step_deg)):
                for arm in self.planner.analytic.solve(1, target, float(swivel), self.q[8:15]):
                    row = self.q.copy()
                    row[0], row[8:15] = lift, arm
                    raw.append(row)
        raw = np.unique(np.round(raw, 7), axis=0) if raw else np.empty((0, 15))
        mapped, found = mapper(raw)
        mapped = mapped[found]
        if len(mapped):
            mapped = mapped[validity.mask(self.tensor(mapped)).cpu().numpy()]
        exact = []
        exact_mask = (self.exact_pose_mask(mapped, 'right', self.targets['right']) &
                      self.exact_pose_mask(mapped, 'left', self.targets['left'])) if len(mapped) else np.zeros(0, dtype=bool)
        for row, accepted in zip(mapped, exact_mask):
            if accepted:
                exact.append(row)
        mapped = np.asarray(exact).reshape(-1, 15)
        if len(mapped):
            weights = np.array([5.] + [1.]*14)
            mapped = mapped[self.margin_order(mapped, np.linalg.norm((mapped-self.q)*weights, axis=1))[:64]]
        details = dict(stage=self.phase, lifts=lifts, analytic_candidates=len(raw),
                       compensation_ik_candidates=int(found.sum()), valid_candidates=len(mapped),
                       selected_goal_cap=64, constraint='lift + right arm searched; left arm compensates; lower box stays at carry target')
        return mapped, mapper, validity, details

    def search_recenter(self, side, candidates, active, mapper=None, full_validity=None):
        start = self.q.copy()
        validity = full_validity or self.validity()
        self.planner.active_side = side
        seed = self.request.seed
        derived = list(mapper.support_indices) if mapper is not None else []
        fixture = dict(stage=self.phase, side=side, start=start.tolist(), candidates=np.asarray(candidates).tolist(),
                       active_indices=active, derived_indices=derived, snapshot=self.store.snapshot().to_dict(),
                       support_pose=None if self.support_pose is None else self.support_pose.tolist(),
                       target=self.targets[side].tolist(), seed=seed, budget_s=2.)
        fixture['identity'] = digest(fixture)
        self.report['recenter_inputs'].append(fixture)
        began = time.perf_counter()
        alternatives = []
        attempts_run = 0
        cartesian_attempts = []
        searched = self.search_inflation(validity, start, candidates)
        limit = self.rotation_gates(start, candidates, {side: 30.})[side] if mapper is None else 30.
        if self.algorithm == 'cartesian_rrt':
            attempts_run = 1
            path, stats = plan_cartesian_transport(
                self.planner, start, candidates, validity.checker, budget_s=2., seed=seed,
                options={'edge_resolution': .08, 'batch': 128, 'connect_candidates': 32,
                         'shortcut': True, 'monotonic_progress': True,
                         'strict_budget': True, 'budget_reserve_s': .25},
                validity=searched, active_indices=active,
                row_mapper=mapper, derived_indices=derived)
            if path is not None:
                alternatives.append((path, stats, seed))
        else:
            quality_good = False
            if mapper is None and self.planner.direct_line_first:
                line_started = time.perf_counter()
                line, goal = self.straight_line_path(start, candidates, searched, (side,))
                self.record_operation_time('direct_line', line_started)
                if line is not None:
                    alternatives.append((line, dict(direct_line_goal=goal), None))
                    quality_good = self.path_rotation_metrics(line, side)['extra_deg'] <= limit
            for retry in range(0 if quality_good else self.planner.recenter_cartesian_attempts):
                attempts_run += 1
                quality_seed = seed+1000*(retry+1)
                quality_path, quality_stats = plan_cartesian_transport(
                    self.planner, start, candidates, validity.checker, budget_s=self.planner.recenter_cartesian_budget_s,
                    seed=quality_seed,
                    options={'edge_resolution': .08, 'batch': 256, 'connect_candidates': 64,
                             'shortcut': True, 'monotonic_progress': True,
                             'strict_budget': True, 'budget_reserve_s': .25,
                             'rrt_min_goals': self.planner.recenter_cartesian_rrt_min_goals if mapper is None else 0},
                    validity=searched, active_indices=active,
                    row_mapper=mapper, derived_indices=derived)
                quality_stats = quality_stats or {}
                cartesian_attempts.append(dict(seed=quality_seed, success=quality_path is not None,
                                               **{key: quality_stats.get(key) for key in
                                                  ('failure', 'same_branch_goals', 'search_ms')}))
                if quality_path is not None:
                    alternatives.append((quality_path, quality_stats, quality_seed))
                    moving_quality = self.path_rotation_metrics(quality_path, side)['extra_deg']
                    held_quality = (self.path_rotation_metrics(quality_path, mapper.support_side)['extra_deg']
                                    if mapper is not None else 0.)
                    quality_good = moving_quality <= limit and held_quality <= 5.
                    break
                if (quality_stats.get('same_branch_goals') or 0) < 2:
                    break
            reduced, reduced_search = ((SupportCompensatedValidity(item, start, mapper, searched_indices=active)
                                        if mapper is not None else ReducedValidity(item, start, active))
                                       for item in (validity, searched))
            lo, hi = validity.checker.kinematics.get_joint_limits().position
            search = constrained_rrt_connect if mapper is not None else batched_rrt_connect_multi_goal
            options = (dict(stop_at_first_valid=True, batch=256, connect_candidates=64, step=.28)
                       if mapper is not None else dict(edge_cache=self.planner.rrt_fine_edge_cache,
                                                       prune_invalid_edges=self.planner.rrt_prune_invalid_edges,
                                                       screen_edges=self.planner.rrt_screened_edges))
            goals, line_goals = np.asarray(candidates), None
            if mapper is None and self.planner.recenter_line_goals:
                line_goals = np.flatnonzero(self.line_rotation(start, goals, (side,))[side] <= limit)
                if not 0 < len(line_goals) < len(goals):
                    line_goals = None
            if mapper is None and self.planner.recenter_rrt_refinement_s is not None:
                options.update(stop_at_first_valid=True, refinement_s=self.planner.recenter_rrt_refinement_s)
            for attempt in range(0 if quality_good else 4):
                attempts_run += 1
                rows = goals if line_goals is None else goals[line_goals]
                sparse, stats = search(
                    self.tensor(start[active]), self.tensor(rows[:, active]),
                    lo[active]+1e-5, hi[active]-1e-5, reduced_search, 2., seed+attempt, **options)
                if sparse is None:
                    line_goals = None  # those goals gave no path: all goals from the next attempt
                    continue
                if line_goals is not None:
                    stats = dict(stats, line_goals=len(line_goals), selected_goal=int(line_goals[stats['selected_goal']]))
                sparse = self.shortcut_states(np.asarray(sparse), reduced)
                stats = dict(stats or {}, shortcut_waypoints=len(sparse))
                if mapper is not None:
                    dense_reduced = np.asarray(densify(
                        sparse, reduced.weights.cpu().tolist()))
                    path = reduced.expand(self.tensor(dense_reduced)).cpu().numpy()
                else:
                    path = np.asarray(densify(reduced.expand(self.tensor(sparse)).cpu().numpy()))
                alternatives.append((path, stats, seed+attempt))
                quality = self.path_rotation_metrics(path, side)
                if quality['extra_deg'] <= limit or (self.planner.rotation_gate_slack_deg is not None and
                                                     len(alternatives) >= 2):
                    break
        if alternatives:
            def quality(item):
                path = item[0]
                moving = self.path_rotation_metrics(path, side)['extra_deg']
                held = (self.path_rotation_metrics(path, mapper.support_side)['extra_deg']
                        if mapper is not None else 0.)
                joints = np.asarray(path)[:, 1:]
                backtrack = float(np.degrees(
                    np.abs(np.diff(joints, axis=0)).sum()-np.abs(joints[-1]-joints[0]).sum()))
                return moving+held+.1*max(0.,backtrack)
            path, stats, selected_seed = min(alternatives, key=quality)
        else:
            path, stats, selected_seed = None, None, None
        torch.cuda.synchronize()
        elapsed = (time.perf_counter()-began)*1000
        record = dict(stage=self.phase, algorithm=self.algorithm, input_identity=fixture['identity'],
                      per_attempt_budget_ms=2000., attempts_run=attempts_run, rotation_gate_deg=limit,
                      cartesian_attempts=cartesian_attempts,
                      actual_search_ms=elapsed,
                      success=path is not None, stats=stats, selected_seed=selected_seed,
                      support_compensated=mapper is not None,
                      alternatives=[dict(seed=item[2],stats=item[1],
                          moving_rotation=self.path_rotation_metrics(item[0],side),
                          support_rotation=(self.path_rotation_metrics(item[0],mapper.support_side)
                                            if mapper is not None else None))
                                    for item in alternatives])
        self.report['recenter_searches'].append(record)
        if path is not None:
            audit_active = sorted(set(active) | set(derived))
            self.audit(path, audit_active, start, target=(side, self.targets[side]))
            transforms = [self.tool(q, side) for q in path]
            positions = np.array([t[:3, 3] for t in transforms])
            rotations = Rotation.from_matrix(np.array([t[:3, :3] for t in transforms]))
            total = float((rotations[:-1].inv()*rotations[1:]).magnitude().sum()) if len(path)>1 else 0.
            chord = float((rotations[0].inv()*rotations[-1]).magnitude())
            support_translation = support_angle = 0.
            if mapper is not None:
                for row in path:
                    translation, angle = pose_error(
                        self.tool(row, mapper.support_side), mapper.support_tool)
                    support_translation = max(support_translation, translation)
                    support_angle = max(support_angle, angle)
                if support_translation > .001 or support_angle > math.radians(.5):
                    self.fail('compensated support tool drifted during recenter')
            record.update(support_translation_drift_m=support_translation,
                          support_rotation_drift_deg=math.degrees(support_angle),
                          tcp_length_m=float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum()),
                          extra_rotation_deg=math.degrees(max(0., total-chord)),
                          joint_motion_rad=float(np.abs(np.diff(path[:, 1:], axis=0)).sum()),
                          lift_travel_m=float(np.abs(np.diff(path[:, 0])).sum()),
                          support_mapper_calls=None if mapper is None else mapper.calls,
                          support_mapper_failures=None if mapper is None else mapper.failures)
        return path

    def recenter(self, side, active, check_lowering=False):
        self.enter(side+'_recenter')
        # The arm carries its box in free space: coverage-grown spheres except next to the start,
        # where it has just left the wall (MOTION-429); the future lowering keeps original spheres.
        with self.free_space((side,), [self.q]):
            path = self.recenter_path(side, active, check_lowering)
        self.append(path)

    def recenter_path(self, side, active, check_lowering):
        began = time.perf_counter()
        mapper = full_validity = None
        if side == 'right' and self.support_pose is not None:
            active=[0]+RIGHT
            candidates,mapper,full_validity,details=self.coordinated_moving_candidates('right','left')
            initial_count=len(candidates);details.update(stage=self.phase,generation_ms=(time.perf_counter()-began)*1000,
                constraint='lift + right arm searched; left arm analytically compensates; upper box fixed in world')
            self.report.setdefault('recenter_candidates',[]).append(details)
        elif side == 'left' and self.support_pose is not None:
            active = [0] + LEFT
            candidates, mapper, full_validity, details = self.coordinated_candidates()
            initial_count = details['current_state_valid_candidates']
            details.update(stage=self.phase, generation_ms=(time.perf_counter()-began)*1000,
                           constraint='lift + left arm searched; right arm analytically compensates; upper box fixed in world')
            self.report.setdefault('recenter_candidates', []).append(details)
        elif side == 'right' and any(item.parent_link == 'left_tool0'
                                     for item in self.store.snapshot().attachments):
            active = [0] + RIGHT
            candidates, mapper, full_validity, details = self.coordinated_right_candidates()
            initial_count = len(candidates)
            details['generation_ms'] = (time.perf_counter()-began)*1000
            self.report.setdefault('recenter_candidates', []).append(details)
        else:
            candidates = self.ik(side, self.targets[side], active)
            initial_count = len(candidates)
            candidates = np.asarray(candidates).reshape(-1, 15)
            candidates = candidates[self.exact_pose_mask(candidates, side, self.targets[side])]
            self.report.setdefault('recenter_candidates', []).append(dict(stage=self.phase,
                ik_valid=initial_count, future_sweep_valid=len(candidates),
                generation_ms=(time.perf_counter()-began)*1000))
        if not len(candidates):
            self.report['recenter_rejection'] = dict(stage=self.phase, start=self.q.tolist(),
                target=self.targets[side].tolist(), active_indices=active, derived_indices=RIGHT if mapper else [],
                snapshot=self.store.snapshot().to_dict(), search_attempted=False)
            self.fail('no carry target IK survives the full load, support-pose and future-lowering gates')
        lazy_lowering = side=='left' and self.support_pose is not None and check_lowering
        remaining=np.asarray(candidates,dtype=float)
        path=None
        lazy_attempts=[]
        max_lazy_replans=min(8,len(remaining)) if lazy_lowering else 1
        for lazy_replans in range(max_lazy_replans):
            path=self.search_recenter(side,remaining,active,mapper,full_validity)
            if path is None:
                break
            if not lazy_lowering:
                break
            with self.free_space(()):
                lowering,attempts=self.validate_future_lowering(path[-1])
            lazy_attempts.extend(attempts)
            if lowering is not None:
                self.future_lowering_paths[tuple(np.round(path[-1],6))]=lowering
                record=self.report['recenter_candidates'][-1]
                record.update(future_lowering_valid=1,lowering_attempts=len(lazy_attempts),
                              lowering_successes=1,lowering_attempt_sample=lazy_attempts[:20],
                              lazy_replans=lazy_replans)
                break
            selected=int(np.argmin(np.linalg.norm(remaining-path[-1],axis=1)))
            remaining=np.delete(remaining,selected,axis=0)
            path=None
            if not len(remaining):
                break
        if path is None and mapper is None:
            target=self.targets[side];rotation_stage=self.tool(self.q,side);rotation_stage[:3,:3]=target[:3,:3]
            rotate=self.analytic_task_space_bridge(self.q,side,rotation_stage);translate=(self.analytic_task_space_bridge(rotate[-1],side,target) if rotate is not None else None)
            if translate is not None:path=np.vstack([rotate,translate[1:]]);self.report['recenter_analytic_bridge']=dict(side=side,points=len(path))
        if path is None:
            if lazy_lowering:
                self.report['recenter_candidates'][-1].update(future_lowering_valid=0,lowering_attempts=len(lazy_attempts),
                    lowering_successes=0,lowering_attempt_sample=lazy_attempts[:20])
            self.fail('recenter search/future lowering failed within the fixed budget; no implicit fallback')
        return path

    def extract_upper_with_lower_hold(self):
        self.enter('right_extract')
        start = self.q.copy()
        validity = self.validity()
        item = next(item for item in self.store.snapshot().attachments if item.parent_link == 'right_tool0')
        original = self.request.snapshot.object(item.object_id)
        clearance=self.wall_clearance(start,item,original)
        distance=.005-clearance
        if distance <= 0 or distance > self.request.extraction_m+1e-6:
            self.fail(f'invalid upper extraction clearance distance: {distance}')
        self.planner.fk(start)
        origin = self.planner.fk_robot.get_transform('right_tool0', self.planner.fk_robot.base_link).copy()
        target = origin.copy()
        target[:3, 3] += self.wall_outward_direction*distance
        carriage = CarriageModel(self.planner, start)
        limits = validity.checker.kinematics.get_joint_limits().position
        low, high = (float(values[0].item()) for values in limits)
        lifts = np.asarray([value for value in self.lift_grid if low+1e-5 <= value <= high-1e-5])
        direct_path=None;direct_attempts=[];direct_limit_mm=2. if self.report.get('pair_strategy')=='outer_standard_rack_supported_lower_recenter' else 1.
        direct_lifts=[float(start[0]),*lifts.tolist()]
        # Search endpoints at the current lift too: the held box may allow no lift change at all (a lower
        # box recentered at its J6 limit cannot follow a rising carriage).
        lifts=np.unique(np.r_[float(start[0]),lifts])
        for lift_end in direct_lifts:
            rows,details=self.planner.analytic_extract(start,validity,validity,sides=('right',),direction=self.wall_outward_direction,
                distance_m=distance,lift_end=lift_end,hold_sides=('left',),swivel_continuation=True)
            accepted=rows is not None and (not isinstance(details,dict) or details.get('max_line_error_mm',0.)<=direct_limit_mm)
            direct_attempts.append(dict(lift_end=lift_end,success=accepted,details=details))
            if accepted:direct_path=np.asarray(rows);break
        left_mapper = SupportCompensation(self, side='left', reference=start, tool_target=self.targets['left'])
        templates = np.repeat(start[None], len(lifts), axis=0)
        templates[:, 0] = lifts
        templates, left_found = left_mapper(templates)
        endpoints = []
        for lift_index, lift in enumerate(lifts):
            if not left_found[lift_index]:
                continue
            local = carriage.inverse(np.asarray([lift]))[0] @ target
            for swivel in np.deg2rad(np.arange(-180., 180., self.swivel_step_deg)):
                for arm in self.planner.analytic.solve(1, local, float(swivel), start[8:15]):
                    row = templates[lift_index].copy()
                    row[8:15] = arm
                    endpoints.append(row)
        endpoints = np.unique(np.round(endpoints, 7), axis=0) if endpoints else np.empty((0, 15))
        analytic_count = len(endpoints)
        if len(endpoints):
            endpoints = endpoints[validity.mask(self.tensor(endpoints)).cpu().numpy()]
        endpoints = np.asarray(endpoints).reshape(-1, 15)
        endpoints = endpoints[self.exact_pose_mask(endpoints, 'right', target) &
                              self.exact_pose_mask(endpoints, 'left', self.targets['left'])]
        self.planner.active_side='right'
        extraction_tolerance=.05 if self.report.get('pair_strategy')=='outer_standard_rack_supported_lower_recenter' else .001
        path=direct_path;stats={'tier':'analytic_compensated','attempts':direct_attempts} if direct_path is not None else None
        line_rejections=[]
        if path is None and len(endpoints):
            # The task-space search does not constrain the line; retry another seed only when its
            # path would fail the same line/orientation/order audit applied below.
            for attempt in range(3):
                path, stats = plan_cartesian_transport(
                    self.planner, start, endpoints, validity.checker,
                    budget_s=2., seed=self.request.seed+1+1000*attempt,
                    options={'strict_budget': True, 'workspace_margin_m': .15,
                             'batch': 256, 'connect_candidates': 64, 'step': .28},
                    validity=validity, active_indices=[0]+RIGHT,
                    row_mapper=left_mapper, derived_indices=LEFT)
                if path is None or self.line_gate_ok(path,'right',origin,self.wall_outward_direction,distance,extraction_tolerance):
                    break
                # Keep the endpoint the search chose, but replace its off-line connection by the
                # compensated outward line plus a tool-fixed swivel self-motion onto that endpoint.
                straight,straight_stats=self.straight_extraction_to(start,np.asarray(path)[-1],validity,distance,direct_limit_mm)
                line_rejections.append(dict(seed=self.request.seed+1+1000*attempt,stats=stats,straightened=straight_stats))
                if straight is not None:
                    path,stats=straight,dict(stats,tier='rrt_endpoint_straightened',straightened=straight_stats)
                    break
        self.report['upper_extraction_compensation'] = dict(
            method=('analytic compensated line' if direct_path is not None else 'single-arm Cartesian RRT with lower-box compensation'),
            held_side='left', moving_side='right', required_clearance_m=distance,
            lift_grid=lifts.tolist(),direct_attempts=direct_attempts,analytic_endpoint_candidates=analytic_count,
            valid_endpoint_candidates=len(endpoints), search_stats=stats, line_gate_rejections=line_rejections,
            success=path is not None)
        if path is None:
            self.fail('compensated Cartesian upper extraction found no fully validated path')
        path = np.asarray(path)
        progress=[];lateral=[];angles=[]
        for actual in self.tool_poses(path,'right'):
            delta=actual[:3,3]-origin[:3,3];along=float(np.dot(delta,self.wall_outward_direction));progress.append(along)
            lateral.append(float(np.linalg.norm(delta-self.wall_outward_direction*along)));angles.append(float(pose_error(actual,origin)[1]))
        self.report['upper_extraction_line_metrics']=dict(max_lateral_m=max(lateral),max_rotation_deg=math.degrees(max(angles)),
            final_distance_m=progress[-1],minimum_progress_step_m=min(np.diff(progress),default=0.),required_distance_m=distance)
        self.audit(path,list(range(15)),start,line=('right',origin,self.wall_outward_direction,distance,extraction_tolerance))
        max_translation, max_angle = self.max_pose_drift(path, 'left', self.targets['left'])
        if max_translation > .001 or max_angle > math.radians(.5):
            self.fail('lower box drifted during compensated upper extraction')
        self.report['upper_extraction_compensation'].update(
            points=len(path), max_tool_translation_drift_m=max_translation,
            max_tool_rotation_drift_deg=math.degrees(max_angle))
        self.append(path)
        clearance=self.wall_clearance(self.q,item,original)
        self.report['stage_audits'][-1]['wall_clearance_m'] = clearance
        if clearance < 1e-6:
            self.fail('actual upper box has not cleared the wall front plane')

    def straight_extraction_to(self, start, end, validity, distance, line_limit_mm):
        """Compensated outward line from `start` to the extraction target, then a swivel self-motion
        with the tool fixed onto the endpoint row `end`.

        The box can rest flush against its neighbours (zero-gap wall) with the arm next to a joint
        limit, where 1 cm joint-space steps sag ~0.1 mm into a neighbour; 2.5 mm steps sag ~6 um.
        Returns (rows, details); rows is None unless every densified row validates and the
        self-motion lands on `end`.
        """
        rows, details = self.planner.analytic_extract(start, validity, validity, sides=('right',),
            direction=self.wall_outward_direction, distance_m=distance, lift_end=float(end[0]), hold_sides=('left',),
            swivel_continuation=True, step_m=.0025)
        if rows is None or details.get('max_line_error_mm', 0.) > line_limit_mm:
            return None, dict(line=details)
        rows, planner = np.asarray(rows), self.planner
        planner.fk(rows[-1])
        carriage = np.linalg.inv(planner.fk_robot.get_transform('arm_carriage', planner.fk_robot.base_link))
        tool = planner.fk_robot.get_transform('right_tool0', planner.fk_robot.base_link).copy()
        first, last = planner.analytic.swivel(1, rows[-1][8:15]), planner.analytic.swivel(1, end[8:15])
        tail = [rows[-1]]
        for swivel in np.linspace(first, last, max(2, math.ceil(abs(last-first)/math.radians(.5))+1))[1:]:
            solutions = planner.analytic.solve(1, carriage@tool, float(swivel), tail[-1][8:15])
            if not len(solutions):
                return None, dict(line=details, self_motion_failure=f'no IK at swivel {swivel:.4f}')
            row = tail[-1].copy()
            row[8:15] = solutions[np.argmin(np.sum((solutions-tail[-1][8:15])**2, axis=1))]
            tail.append(row)
        gap = float(np.abs(tail[-1]-end).max())
        if gap > 1e-3:
            return None, dict(line=details, self_motion_failure=f'lands {gap:.2e} rad from the endpoint')
        tail[-1] = np.asarray(end, dtype=float)
        dense = np.asarray(densify(np.asarray(tail)))
        if not bool(validity.mask(self.tensor(dense)).all()):
            return None, dict(line=details, self_motion_failure='self-motion collision/stability')
        return np.vstack([rows, dense[1:]]), dict(line=details, swivel_rad=[float(first), float(last)],
                                                  self_motion_rows=len(dense), landing_gap_rad=gap)

    def line_gate_ok(self, rows, side, origin, direction, length, tolerance):
        """audit()'s GPU-FK line/orientation/order criteria, to retry a search before auditing."""
        state = self.validity().checker.kinematics.compute_kinematics(
            JointState.from_position(self.tensor(rows)[None], joint_names=list(ACTIVE_JOINTS)))
        pose = state.tool_poses.to_dict()[side+'_tool0']
        positions, quaternions = pose.position.reshape(-1, 3).cpu().numpy(), pose.quaternion.reshape(-1, 4).cpu().numpy()
        delta = positions-origin[:3, 3]
        along = delta@direction
        lateral = np.linalg.norm(delta-along[:, None]*direction, axis=1)
        angles = (Rotation.from_matrix(origin[:3, :3]).inv()*Rotation.from_quat(np.roll(quaternions, -1, axis=1))).magnitude()
        return bool(lateral.max() <= tolerance and angles.max() <= math.radians(.5) and not np.any(np.diff(along) < -1e-5)
                    and along.min() >= -1e-5 and along.max() <= length+.001 and abs(along[-1]-length) <= .001)

    def contact_lower_ranked(self, object_id, active):
        item = self.store.snapshot().object(object_id)
        geometry_entries = []
        support_mapper = SupportCompensation(self, side='right', reference=self.q)
        validity = self.validity()
        limits = validity.checker.kinematics.get_joint_limits().position
        low, high = (float(values[0].item()) for values in limits)
        lifts = sorted({float(self.q[0]), *(float(value) for value in self.lift_grid)},
                       key=lambda value: abs(value-float(self.q[0])))
        lifts = [value for value in lifts if low+1e-5 <= value <= high-1e-5]

        def try_options(options):
            while options:
                self.enter('left_precontact')
                goals=np.asarray([option['precontact'] for option in options])
                path=self.plan_precontact_path('left',goals,active)
                if path is None:break
                selected=int(np.argmin(np.linalg.norm(goals-path[-1],axis=1)))
                option=options.pop(selected)
                if not self.accept_precontact_path('left',path):continue
                if getattr(self,'defer_supported_extract',False) and not self.planner.deferred_extract_search:
                    extracted,details=None,dict(strategy='deferred_to_rack_supported_partial_line',success=False)
                else:
                    saved=self.store
                    try:
                        self.store=SceneStore(saved.snapshot());self.store.attach(
                            option['attachment'],self.store.snapshot().revision)
                        extracted,details=self.plan_supported_extract_candidate('left',option['contact_q'])
                    finally:self.store=saved
                self.report['attempts'].append(dict(stage='left_extract',operation='ranked_supported_extract',
                                                    roll_rad=option['entry']['roll_rad'],details=details))
                if extracted is None and not getattr(self,'defer_supported_extract',False):continue
                self.enter('left_precontact');self.audit(
                    path,getattr(self,'precontact_audit_active',active),self.q,
                    target=('left',option['precontact_target']));self.append(path)
                self.enter('left_contact');self.contact_pair=('left',item.object_id)
                self.audit(option['contact'],active,self.q,target=('left',option['target']),
                           line=('left',self.tool(option['contact'][0],'left'),self.wall_inward_direction,.05))
                self.append(option['contact']);self.contact_pair=None;self.attach('left',item)
                option['entry']['selected']=True
                if extracted is not None:self.pending_extract=(extracted,details.get('audit_active_indices',list(range(15))))
                self.report['ranked_grasp_selection']=dict(
                    candidates=len(geometry_entries),selected_roll_deg=math.degrees(option['entry']['roll_rad']),
                    approach_rotation=self.path_rotation_metrics(path,'left'),extraction=details)
                return True
            return False

        requested=self.grasp_roll_candidates('left')
        preferred=math.radians(float(getattr(self,'preferred_left_grasp_roll_deg',45.)))
        rolls=sorted(requested,key=lambda roll:(abs(roll-preferred),requested.index(roll)))
        self.report['ranked_grasp_roll_order_deg']=[math.degrees(roll) for roll in rolls]
        for roll in rolls:
            options=[]
            self.enter('left_contact_model')
            target, proof = self.contact_target('left', item, roll)
            precontact_target=target.copy();precontact_target[:3,3]+=self.wall_outward_direction*.05
            entry = dict(side='left', object_id=item.object_id, roll_rad=roll,
                         target=asdict(matrix_pose(target)), proof=proof, selected=False)
            geometry_entries.append(entry)
            self.report.setdefault('contact_geometry', []).append(entry)
            if not proof['valid']:
                continue
            rows_started=time.perf_counter()
            rows=self.supported_lower_contact_rows(item,target,support_mapper,lifts)
            self.record_operation_time('lower_contact_rows',rows_started)
            entry['support_compensated_contact_candidates']=len(rows)
            entry['ranked_contact_candidates_tested']=min(8,len(rows))
            for contact_q in rows[:8]:
                tool=self.tool(contact_q,'left');attachment=AttachedObject(
                    item.object_id,item.dimensions_m,'left_tool0',
                    matrix_pose(np.linalg.inv(tool)@pose_matrix(item.pose)),('left_link7',))
                self.enter('left_contact');self.contact_pair=('left',item.object_id)
                outward=self.cartesian('left',self.wall_outward_direction,.05,active,start=contact_q)
                self.contact_pair=None
                if outward is None:continue
                options.append(dict(precontact=outward[-1],contact=outward[::-1].copy(),
                                    contact_q=contact_q,attachment=attachment,
                                    entry=entry,target=target.copy(),
                                    precontact_target=precontact_target.copy()))
            if try_options(options):
                return
        self.fail('ranked lower-box grasp candidates exhausted')

    def mirrored_upper_staging_goals(self,left_goals):
        lower=self.store.snapshot().object(f'wall_box_{self.request.lower_box:02d}');variants=[]
        for left_goal in np.asarray(left_goals):
            self.planner.fk(left_goal);carriage=np.linalg.inv(self.planner.fk_robot.get_transform('arm_carriage',self.planner.fk_robot.base_link))
            for roll in self.grasp_roll_candidates('right'):
                target,proof=self.contact_target('right',lower,roll)
                if not proof['valid']:continue
                target=target.copy();target[:3,3]+=self.wall_outward_direction*.05;arms=[]
                for swivel in np.deg2rad(np.arange(-180.,180.,self.swivel_step_deg)):
                    arms.extend(self.planner.analytic.solve(1,carriage@target,float(swivel),self.q[8:15]))
                if not arms:continue
                arms=np.unique(np.round(arms,7),axis=0);rows=np.repeat(left_goal[None],len(arms),axis=0);rows[:,8:15]=arms
                order=self.margin_order(rows,np.linalg.norm(arms-self.q[8:15],axis=1))[:4]
                for arm in arms[order]:row=left_goal.copy();row[8:15]=arm;variants.append(row)
        rows=np.unique(np.round(variants,7),axis=0) if variants else np.empty((0,15))
        if len(rows):rows=rows[self.validity().mask(self.tensor(rows)).cpu().numpy()]
        return rows

    def analytic_task_space_bridge(self,start,side,target,steps=32):
        current=np.asarray(start,dtype=float).copy();origin=self.tool(current,side);validity=self.validity()
        side_index=0 if side=='left' else 1;indices=LEFT if side=='left' else RIGHT
        start_rotation=Rotation.from_matrix(origin[:3,:3]);end_rotation=Rotation.from_matrix(target[:3,:3])
        slerp=__import__('scipy.spatial.transform',fromlist=['Slerp']).Slerp([0.,1.],Rotation.concatenate([start_rotation,end_rotation]))
        rows=[current.copy()]
        for fraction in np.linspace(0.,1.,steps+1)[1:]:
            desired=np.eye(4);desired[:3,3]=origin[:3,3]+fraction*(target[:3,3]-origin[:3,3]);desired[:3,:3]=slerp([fraction]).as_matrix()[0]
            self.planner.fk(current);carriage=np.linalg.inv(self.planner.fk_robot.get_transform('arm_carriage',self.planner.fk_robot.base_link))
            psi=self.planner.analytic.swivel(side_index,current[indices]);swivels=[psi+d for d in (0.,.02,-.02,.05,-.05,.1,-.1,.2,-.2,.4,-.4)]
            candidates=[]
            for swivel in swivels:
                for arm in self.planner.analytic.solve(side_index,carriage@desired,float(swivel),current[indices]):
                    row=current.copy();row[indices]=arm;candidates.append(row)
            if not candidates:
                self.report['analytic_bridge_failure']=dict(side=side,fraction=float(fraction),reason='no_analytic_candidates')
                return None
            candidates=np.unique(np.round(candidates,7),axis=0);mask=validity.mask(self.tensor(candidates)).cpu().numpy();candidates=candidates[mask]
            if not len(candidates):
                self.report['analytic_bridge_failure']=dict(side=side,fraction=float(fraction),reason='no_valid_candidates')
                return None
            starts=self.tensor(np.repeat(current[None],len(candidates),axis=0));good=validity.edges(starts,self.tensor(candidates),resolution=math.radians(.5)).cpu().numpy();candidates=candidates[good]
            if not len(candidates):
                self.report['analytic_bridge_failure']=dict(side=side,fraction=float(fraction),reason='no_valid_edge')
                return None
            current=min(candidates,key=lambda row:np.linalg.norm(row[indices]-rows[-1][indices]));rows.append(current.copy())
        return np.asarray(rows)

    def sequential_mirrored_upper_precontact(self,staged):
        if not len(staged):return None
        staged=np.asarray(staged,dtype=float);validity=self.validity();right_goals=staged.copy();right_goals[:,0]=self.q[0];right_goals[:,LEFT]=self.q[LEFT]
        right_goals=np.unique(np.round(right_goals,7),axis=0);right_goals=right_goals[validity.mask(self.tensor(right_goals)).cpu().numpy()]
        if not len(right_goals):return None
        self.planner.active_side='right';right_path,_=plan_cartesian_transport(self.planner,self.q,right_goals,validity.checker,budget_s=2.,
            seed=self.request.seed+452,options={'edge_resolution':.08,'batch':128,'connect_candidates':32,'shortcut':True,
            'monotonic_progress':True,'strict_budget':True,'budget_reserve_s':.1,'workspace_margin_m':.5,
            'route_branch_change':True,'fallback_joint_rrt':True,'wrist_flip':True},
            validity=self.search_inflation(validity,self.q,right_goals),active_indices=RIGHT)
        if right_path is None:
            joint=[]
            for _ in range(4):
                candidate=self.search(right_goals,RIGHT,free_space=True)
                if candidate is not None:joint.append((self.path_rotation_metrics(candidate,'right')['extra_deg'],np.asarray(candidate)))
            if not joint:return None
            _,right_path=min(joint,key=lambda item:item[0])
        distance=np.linalg.norm(staged[:,RIGHT]-np.asarray(right_path)[-1,RIGHT],axis=1);left_goals=staged[distance<=distance.min()+1e-6]
        start=np.asarray(right_path)[-1];active_left=LEFT if np.allclose(left_goals[:,0],start[0],atol=1e-6,rtol=0) else [0]+LEFT
        target_tool=self.tool(left_goals[0],'left');left_path=self.analytic_task_space_bridge(start,'left',target_tool)
        self.planner.active_side='left'
        if left_path is None:left_path,_=plan_cartesian_transport(self.planner,start,left_goals,validity.checker,budget_s=2.,
            seed=self.request.seed+453,options={'edge_resolution':.08,'batch':128,'connect_candidates':32,'shortcut':True,
            'monotonic_progress':True,'strict_budget':True,'budget_reserve_s':.1,'workspace_margin_m':.5,
            'route_branch_change':True,'fallback_joint_rrt':True,'wrist_flip':True},
            validity=self.search_inflation(validity,start,left_goals),active_indices=active_left)
        if left_path is None:
            saved_q=self.q.copy();self.q=start.copy();joint=[]
            try:
                for _ in range(4):
                    candidate=self.search(left_goals,active_left,free_space=True)
                    if candidate is not None:joint.append((self.path_rotation_metrics(candidate,'left')['extra_deg'],np.asarray(candidate)))
            finally:self.q=saved_q
            if not joint:return None
            _,left_path=min(joint,key=lambda item:item[0])
        return np.vstack([np.asarray(right_path),np.asarray(left_path)[1:]])

    def contact_upper_left_ranked(self,object_id,active):
        item=self.store.snapshot().object(object_id);self.enter('left_contact_model');target,proof=self.contact_target('left',item,0.)
        entry=dict(side='left',object_id=item.object_id,roll_rad=0.,target=asdict(matrix_pose(target)),proof=proof,selected=False)
        self.report.setdefault('contact_geometry',[]).append(entry)
        if not proof['valid']:self.fail('mirrored upper-box contact mesh proof failed')
        pre=target.copy();pre[:3,3]+=self.wall_outward_direction*.05;self.enter('left_precontact');candidates=self.ik('left',pre,LEFT)
        options=[]
        for candidate in candidates[:32]:
            self.enter('left_contact');self.contact_pair=('left',item.object_id)
            contact=self.cartesian('left',self.wall_inward_direction,.05,active,start=candidate,swivel_continuation=True)
            if contact is None:contact=self.analytic_task_space_bridge(candidate,'left',target,steps=16)
            self.contact_pair=None
            if contact is None:contact=self.cartesian_contact_fallback('left',item,candidate,target,active)
            if contact is not None:
                options.append((candidate,contact))
                if len(options)>=8:break
        entry['precontact_candidates']=len(candidates);entry['contact_path_candidates']=len(options)
        if not options:self.fail('mirrored upper-box ranked candidates have no certified contact path')
        goals=np.asarray([option[0] for option in options]);self.enter('left_precontact')
        target_tool=self.tool(goals[0],'left');rotation_stage=self.tool(self.q,'left');rotation_stage[:3,:3]=target_tool[:3,:3]
        rotate_path=self.analytic_task_space_bridge(self.q,'left',rotation_stage)
        translate_path=(self.analytic_task_space_bridge(rotate_path[-1],'left',target_tool) if rotate_path is not None else None)
        path=(np.vstack([rotate_path,translate_path[1:]]) if translate_path is not None else None)
        if path is None:path=self.plan_precontact_path('left',goals,active)
        if path is None or not self.accept_precontact_path('left',path):self.fail('mirrored upper precontact has no path')
        option=min(options,key=lambda value:np.linalg.norm(value[0]-np.asarray(path)[-1]));self.audit(path,active,self.q,target=('left',pre));self.append(path)
        self.enter('left_contact');self.contact_pair=('left',item.object_id);self.audit(option[1],active,self.q,target=('left',target),
            line=('left',self.tool(option[1][0],'left'),self.wall_inward_direction,.05));self.append(option[1]);self.contact_pair=None;self.attach('left',item);entry['selected']=True

    def supported_right_contact_rows(self,item,target,mapper,lifts):
        rows=[]
        for lift in lifts:
            local=mapper.carriage.inverse(np.asarray([lift]))[0]@target
            for swivel in np.deg2rad(np.arange(-180.,180.,self.swivel_step_deg/2)):
                for arm in self.planner.analytic.solve(1,local,float(swivel),self.q[8:15]):
                    row=self.q.copy();row[0],row[8:15]=lift,arm;rows.append(row)
        rows=np.unique(np.round(rows,7),axis=0) if rows else np.empty((0,15));rows,found=mapper(rows);rows=rows[found]
        self.contact_pair=('right',item.object_id)
        if len(rows):rows=rows[self.validity().mask(self.tensor(rows)).cpu().numpy()]
        self.contact_pair=None;rows=np.asarray(rows).reshape(-1,15);rows=rows[self.exact_pose_mask(rows,'right',target)]
        if len(rows):rows=rows[self.margin_order(rows,np.linalg.norm((rows-self.q)*np.array([5.]+[1.]*14),axis=1))]
        return rows

    def contact_lower_right_ranked(self,object_id,active):
        item=self.store.snapshot().object(object_id);mapper=SupportCompensation(self,side='left',reference=self.q);validity=self.validity()
        limits=validity.checker.kinematics.get_joint_limits().position;low,high=(float(values[0].item()) for values in limits)
        lifts=[value for value in sorted({float(self.q[0]),*(float(v) for v in self.lift_grid)},key=lambda v:abs(v-float(self.q[0]))) if low+1e-5<=value<=high-1e-5]
        for roll in self.grasp_roll_candidates('right'):
            self.enter('right_contact_model');target,proof=self.contact_target('right',item,roll);entry=dict(side='right',object_id=item.object_id,
                roll_rad=roll,target=asdict(matrix_pose(target)),proof=proof,selected=False);self.report.setdefault('contact_geometry',[]).append(entry)
            if not proof['valid']:continue
            rows=self.supported_right_contact_rows(item,target,mapper,lifts);entry['support_compensated_contact_candidates']=len(rows);options=[]
            pre_target=target.copy();pre_target[:3,3]+=self.wall_outward_direction*.05
            for contact_q in rows[:8]:
                self.enter('right_contact');self.contact_pair=('right',item.object_id)
                outward=self.cartesian('right',self.wall_outward_direction,.05,active,start=contact_q,swivel_continuation=True);self.contact_pair=None
                if outward is None:continue
                tool=self.tool(contact_q,'right');attachment=AttachedObject(item.object_id,item.dimensions_m,'right_tool0',
                    matrix_pose(np.linalg.inv(tool)@pose_matrix(item.pose)),('right_link7',))
                saved=self.store
                try:
                    self.store=SceneStore(saved.snapshot());self.store.attach(attachment,self.store.snapshot().revision)
                    extracted,details=self.plan_supported_extract_candidate('right',contact_q)
                    direct,_=self.planner.analytic_extract(contact_q,self.validity,self.validity,sides=('right',),
                        direction=self.wall_outward_direction,distance_m=self.request.extraction_m,hold_sides=('left',),swivel_continuation=True)
                    raw_partial=np.asarray(direct if direct is not None else getattr(self.planner,'last_extract_frames',[contact_q]),dtype=float)
                    partial=raw_partial.copy()
                    valid=self.validity().mask(self.tensor(partial)).cpu().numpy();first_bad=int(np.flatnonzero(~valid)[0]) if not valid.all() else len(partial);partial=partial[:first_bad]
                    escape=(float(np.dot(self.tool(partial[-1],'right')[:3,3]-self.tool(partial[0],'right')[:3,3],self.wall_outward_direction)) if len(partial)>1 else 0.)
                finally:self.store=saved
                options.append(dict(precontact=outward[-1],contact=outward[::-1].copy(),contact_q=contact_q,
                    attachment=attachment,extracted=extracted,escape_path=partial,raw_escape_path=raw_partial,details=details,escape_m=escape,target=target,pre_target=pre_target,entry=entry))
            options=sorted(options,key=lambda option:-option['escape_m'])[:4]
            entry['ranked_escape_m']=[option['escape_m'] for option in options]
            while options:
                self.enter('right_precontact');goals=np.asarray([o['precontact'] for o in options]);path=self.plan_precontact_path('right',goals,active)
                if path is None:break
                selected=int(np.argmin(np.linalg.norm(goals-path[-1],axis=1)));option=options.pop(selected)
                if not self.accept_precontact_path('right',path):continue
                self.audit(path,getattr(self,'precontact_audit_active',active),self.q,target=('right',pre_target));self.append(path)
                self.enter('right_contact');self.contact_pair=('right',item.object_id);self.audit(option['contact'],active,self.q,target=('right',target),
                    line=('right',self.tool(option['contact'][0],'right'),self.wall_inward_direction,.05));self.append(option['contact']);self.contact_pair=None
                self.attach('right',item);entry['selected']=True
                if len(option['raw_escape_path'])>1:self.pending_extract=(option['raw_escape_path'],list(range(15)))
                elif option['extracted'] is not None:self.pending_extract=(option['extracted'],option['details'].get('audit_active_indices',list(range(15))))
                self.report['mirrored_pending_extract_points']=0 if self.pending_extract is None else len(self.pending_extract[0])
                return
        self.fail('mirrored ranked lower-box grasp candidates exhausted')

    def coordinated_moving_candidates(self,moving_side,support_side):
        moving=LEFT if moving_side=='left' else RIGHT;side_index=0 if moving_side=='left' else 1;mapper=SupportCompensation(self,side=support_side)
        validity=self.validity();limits=validity.checker.kinematics.get_joint_limits().position;low,high=(float(values[0].item()) for values in limits)
        lifts=[value for value in sorted({float(self.q[0]),*(float(v) for v in self.lift_grid)},key=lambda v:abs(v-float(self.q[0]))) if low+1e-5<=value<=high-1e-5];raw=[]
        for lift in lifts:
            target=mapper.carriage.inverse(np.asarray([lift]))[0]@self.targets[moving_side]
            for swivel in np.deg2rad(np.arange(-180.,180.,self.swivel_step_deg)):
                for arm in self.planner.analytic.solve(side_index,target,float(swivel),self.q[moving]):
                    row=self.q.copy();row[0],row[moving]=lift,arm;raw.append(row)
        raw=np.unique(np.round(raw,7),axis=0) if raw else np.empty((0,15));mapped,found=mapper(raw);mapped=mapped[found]
        if len(mapped):mapped=mapped[validity.mask(self.tensor(mapped)).cpu().numpy()]
        mapped=np.asarray(mapped).reshape(-1,15);mapped=mapped[self.exact_pose_mask(mapped,moving_side,self.targets[moving_side])]
        return mapped,mapper,validity,dict(lifts=lifts,analytic_candidates=len(raw),support_ik_candidates=int(found.sum()),valid_candidates=len(mapped))

    def dual_support_bridge(self,goal,steps=32):
        start=self.q.copy();targets={side:self.tool(start,side) for side in ('left','right')};carriage=CarriageModel(self.planner,start)
        psi0=[self.planner.analytic.swivel(i,start[1+i*7:8+i*7]) for i in range(2)]
        psi1=[self.planner.analytic.swivel(i,goal[1+i*7:8+i*7]) for i in range(2)]
        delta=[(b-a+math.pi)%(2*math.pi)-math.pi for a,b in zip(psi0,psi1)];validity=self.validity();rows=[start]
        for fraction in np.linspace(0.,1.,steps+1)[1:]:
            lift=start[0]+fraction*(goal[0]-start[0]);row=rows[-1].copy();row[0]=lift;inverse=carriage.inverse(np.asarray([lift]))[0]
            for side_index,side in enumerate(('left','right')):
                indices=LEFT if side=='left' else RIGHT;swivel=psi0[side_index]+fraction*delta[side_index]
                solutions=self.planner.analytic.solve(side_index,inverse@targets[side],float(swivel),row[indices])
                if not len(solutions):return None
                row[indices]=min(solutions,key=lambda arm:np.linalg.norm(arm-row[indices]))
            if not bool(validity.mask(self.tensor(row[None]))[0]) or not bool(validity.edges(self.tensor(rows[-1][None]),self.tensor(row[None]),resolution=math.radians(.5))[0]):return None
            rows.append(row.copy())
        return np.asarray(rows)

    def base_extract_with_upper_hold(self,distance_m=.12,steps=12):
        old=self.pick_base_pose;old_matrix=pose_matrix(old);world_direction=old_matrix[:3,:3]@self.wall_outward_direction
        world_scene=snapshot_in_world(self.store.snapshot(),old);world_request=snapshot_in_world(self.request.snapshot,old)
        world_tool=old_matrix@self.tool(self.q,'left');upper=next(item for item in self.store.snapshot().attachments if item.parent_link=='left_tool0')
        world_upper=old_matrix@self.box_pose(self.q,upper);self.enter('base_pick_reposition');records=[]
        for fraction in np.linspace(0.,1.,steps+1)[1:]:
            position=np.asarray(old.position)+world_direction*(distance_m*fraction);base=Pose(tuple(position),old.quaternion_wxyz);base_matrix=pose_matrix(base)
            self.pick_base_pose=base;self.store=SceneStore(snapshot_in_base(replace(world_scene,state=replace(world_scene.state,base_pose=base))))
            self.request.snapshot=snapshot_in_base(replace(world_request,state=replace(world_request.state,base_pose=base)))
            upper_object=matrix_pose(np.linalg.inv(base_matrix)@world_upper);self.support_pose=pose_matrix(upper_object);self.support_side='left';self.contact_pair=None
            self.validity_cache={};self.cached_key=None;target=np.linalg.inv(base_matrix)@world_tool;self.planner.fk(self.q)
            carriage=np.linalg.inv(self.planner.fk_robot.get_transform('arm_carriage',self.planner.fk_robot.base_link));candidates=[]
            for swivel in np.deg2rad(np.arange(-180.,180.,self.swivel_step_deg/2)):
                for arm in self.planner.analytic.solve(0,carriage@target,float(swivel),self.q[LEFT]):
                    row=self.q.copy();row[LEFT]=arm;candidates.append(row)
            candidates=np.unique(np.round(candidates,7),axis=0) if candidates else np.empty((0,15));validity=self.validity()
            if len(candidates):candidates=candidates[validity.mask(self.tensor(candidates)).cpu().numpy()]
            if not len(candidates):self.fail(f'upper-hold base extraction lost analytic support at fraction {fraction:.3f}')
            next_q=min(candidates,key=lambda row:np.linalg.norm(row[LEFT]-self.q[LEFT]));self.q=next_q
            self.audit([self.q],[],self.q);self.append([self.q]);records.append(dict(fraction=float(fraction),base_pose=asdict(base),analytic_candidates=len(candidates)))
        self.world_request.snapshot=replace(world_request,state=replace(world_request.state,base_pose=self.pick_base_pose))
        self.report['base_extraction_with_upper_hold']=dict(distance_m=distance_m,steps=steps,states=records,navigation_path_validated=False)

    def prepare_pair_for_base_shift(self,distance_m=.12):
        start=self.q.copy();targets={side:self.tool(start,side) for side in ('left','right')};carriage=CarriageModel(self.planner,start);validity=self.validity();raw=[]
        for lift in sorted({float(start[0]),*(float(value) for value in self.lift_grid)},key=lambda value:abs(value-float(start[0]))):
            arms=[]
            for side_index,side in enumerate(('left','right')):
                indices=LEFT if side=='left' else RIGHT;solutions=[]
                for swivel in np.deg2rad(np.arange(-180.,180.,self.swivel_step_deg)):
                    solutions.extend(self.planner.analytic.solve(side_index,carriage.inverse(np.asarray([lift]))[0]@targets[side],float(swivel),start[indices]))
                solutions=np.unique(np.round(solutions,7),axis=0) if solutions else np.empty((0,7))
                if not len(solutions):arms=[];break
                arms.append(solutions[np.argsort(np.linalg.norm(solutions-start[indices],axis=1))[:8]])
            if len(arms)!=2:continue
            for left in arms[0]:
                for right in arms[1]:
                    row=start.copy();row[0],row[LEFT],row[RIGHT]=lift,left,right;raw.append(row)
        raw=np.unique(np.round(raw,7),axis=0) if raw else np.empty((0,15))
        if len(raw):raw=raw[validity.mask(self.tensor(raw)).cpu().numpy()]
        old=self.pick_base_pose;world=snapshot_in_world(self.store.snapshot(),old);world_direction=pose_matrix(old)[:3,:3]@self.wall_outward_direction
        new=Pose(tuple(np.asarray(old.position)+world_direction*distance_m),old.quaternion_wxyz);saved=self.store,self.pick_base_pose,self.request.snapshot,self.validity_cache,self.cached_key,self.support_pose,self.support_side,self.contact_pair
        request_world=snapshot_in_world(saved[2],old);self.pick_base_pose=new;self.store=SceneStore(snapshot_in_base(replace(world,state=replace(world.state,base_pose=new))));self.request.snapshot=snapshot_in_base(replace(request_world,state=replace(request_world.state,base_pose=new)));self.support_pose=None;self.support_side=None;self.contact_pair=None;self.validity_cache={};self.cached_key=None
        future=self.validity();future_valid=future.mask(self.tensor(raw)).cpu().numpy() if len(raw) else np.zeros(0,dtype=bool)
        self.store,self.pick_base_pose,self.request.snapshot,self.validity_cache,self.cached_key,self.support_pose,self.support_side,self.contact_pair=saved
        candidates=raw[future_valid];candidates=candidates[np.argsort(np.linalg.norm(candidates-start,axis=1))] if len(candidates) else candidates
        self.enter('dual_support_reconfigure')
        for candidate in candidates[:64]:
            path=self.dual_support_bridge(candidate)
            if path is not None:
                self.audit(path,list(range(15)),start);self.append(path);self.report['base_shift_support_posture']=dict(
                    generated=len(raw),future_valid=len(candidates),path_points=len(path),distance_m=distance_m);return
        self.fail(f'no dual-support posture survives {distance_m:.3f} m base extraction')

    def lower_upper_before_base_shift(self,distance_m=.12):
        self.enter('left_lower');validity=self.validity();limits=validity.checker.kinematics.get_joint_limits().position
        low,high=(float(value[0].item()) for value in limits);preferred=float(self.q[0]-distance_m)
        lifts=[preferred]+[float(value) for value in self.lift_grid if abs(float(value)-preferred)>1e-9]
        lifts=[value for value in lifts if low+1e-5<=value<=high-1e-5];rows=None
        for lift in lifts:
            rows,_=self.planner.analytic_extract(self.q,validity,validity,sides=('left',),direction=(0.,0.,-1.),
                distance_m=distance_m,lift_end=lift,hold_sides=('right',),swivel_continuation=True)
            if rows is not None:break
        if rows is None:self.fail('mirrored upper lowering before base extraction failed')
        self.audit(rows,list(range(15)),self.q,line=('left',self.tool(self.q,'left'),np.array((0.,0.,-1.)),distance_m));self.append(rows)

    def lower_and_extract_upper_mirrored(self):
        lower_side,upper_side='right','left';validity=self.validity();limits=validity.checker.kinematics.get_joint_limits().position;low,high=(float(v[0].item()) for v in limits)
        lifts=[float(self.q[0])]+[float(v) for v in self.lift_grid if abs(float(v)-float(self.q[0]))>1e-9 and low+1e-5<=float(v)<=high-1e-5]
        rows=None
        self.enter('left_lower')
        for lift in lifts:
            rows,_=self.planner.analytic_extract(self.q,validity,validity,sides=(upper_side,),direction=(0.,0.,-1.),
                distance_m=self.lower_distance,lift_end=lift,hold_sides=(lower_side,),swivel_continuation=True)
            if rows is not None:break
        if rows is None:self.fail('mirrored compensated upper lowering failed')
        self.audit(rows,list(range(15)),self.q,line=(upper_side,self.tool(self.q,upper_side),np.array((0.,0.,-1.)),self.lower_distance));self.append(rows)
        self.enter('left_extract');upper=next(item for item in self.store.snapshot().attachments if item.parent_link=='left_tool0')
        original=self.request.snapshot.object(upper.object_id);distance=.005-self.wall_clearance(self.q,upper,original);rows=None;validity=self.validity()
        for lift in lifts:
            rows,_=self.planner.analytic_extract(self.q,validity,validity,sides=(upper_side,),direction=self.wall_outward_direction,
                distance_m=distance,lift_end=lift,hold_sides=(lower_side,),swivel_continuation=True)
            if rows is not None:break
        if rows is None:self.fail('mirrored compensated upper extraction failed')
        self.audit(rows,list(range(15)),self.q,line=(upper_side,self.tool(self.q,upper_side),self.wall_outward_direction,distance));self.append(rows)

    def plan_compensated_upper_lowering(self):
        start=self.q.copy();validity=self.validity();origin=self.tool(start,'right');target=origin.copy();target[2,3]-=self.lower_distance
        mapper=SupportCompensation(self,side='left',reference=start,tool_target=self.targets['left']);limits=validity.checker.kinematics.get_joint_limits().position
        low,high=(float(values[0].item()) for values in limits);lifts=[float(start[0]),*(float(value) for value in self.lift_grid if abs(float(value)-float(start[0]))>1e-9)]
        lifts=[value for value in lifts if low+1e-5<=value<=high-1e-5];endpoints=[]
        for lift in lifts:
            template=start.copy();template[0]=lift;mapped,found=mapper(template[None])
            if not found[0]:continue
            carriage=CarriageModel(self.planner,start);local=carriage.inverse(np.asarray([lift]))[0]@target
            for swivel in np.deg2rad(np.arange(-180.,180.,self.swivel_step_deg)):
                for arm in self.planner.analytic.solve(1,local,float(swivel),start[RIGHT]):
                    row=mapped[0].copy();row[RIGHT]=arm;endpoints.append(row)
        endpoints=np.unique(np.round(endpoints,7),axis=0) if endpoints else np.empty((0,15))
        if len(endpoints):endpoints=endpoints[validity.mask(self.tensor(endpoints)).cpu().numpy()]
        endpoints=np.asarray(endpoints).reshape(-1,15);endpoints=endpoints[self.exact_pose_mask(endpoints,'right',target)]
        self.planner.active_side='right';path=stats=None
        if len(endpoints):
            path,stats=plan_cartesian_transport(self.planner,start,endpoints,validity.checker,budget_s=4.,seed=self.request.seed+733,
                options={'edge_resolution':.05,'batch':256,'connect_candidates':64,'shortcut':True,'monotonic_progress':True,
                         'strict_budget':True,'budget_reserve_s':.15,'workspace_margin_m':.2},
                validity=validity,active_indices=[0]+RIGHT,row_mapper=mapper,derived_indices=LEFT)
        self.report['upper_lowering_rrt_fallback']=dict(endpoint_candidates=len(endpoints),stats=stats,success=path is not None)
        return None if path is None else np.asarray(path)

    def lower_upper_with_lower_hold(self):
        self.enter('right_lower')
        paths = getattr(self, 'future_lowering_paths', {})
        key = tuple(np.round(self.q, 6))
        rows = paths.get(key)
        matched_error = 0.
        if rows is None and paths:
            keys = np.asarray(list(paths), dtype=float)
            errors = np.max(np.abs(keys-self.q[None]), axis=1)
            selected = int(np.argmin(errors))
            matched_error = float(errors[selected])
            if matched_error <= 1e-5:
                rows = paths[tuple(keys[selected])]
        if rows is None:
            self.fail('selected recenter goal has no prevalidated compensated upper-lowering path')
        rows = np.asarray(rows)
        origin = self.tool(self.q, 'right')
        tolerance=.0012 if self.report.get('pair_strategy')=='outer_standard_rack_supported_lower_recenter' else .001
        self.audit(rows,list(range(15)),self.q,line=('right',origin,np.array((0.,0.,-1.)),self.lower_distance,tolerance))
        max_translation, max_angle = self.max_pose_drift(rows, 'left', self.targets['left'])
        if max_translation > .001 or max_angle > math.radians(.5):
            self.fail('recentered lower box drifted during compensated upper lowering')
        self.report['lowering_compensation'] = dict(
            held_side='left', moving_side='right', points=len(rows),
            matched_goal_max_error=matched_error,
            max_tool_translation_drift_m=max_translation,
            max_tool_rotation_drift_deg=math.degrees(max_angle))
        self.append(rows)

    def align_pair(self):
        self.enter('dual_recenter_align')
        self.audit([self.q], [], self.q)
        radius = float(self.envelope(self.q[None])[0])
        if radius > self.radius:
            self.fail('latest compact endpoint exceeded the approved 1.05 m envelope')
        errors = {}
        for side in ('left', 'right'):
            translation, angle = pose_error(self.tool(self.q, side), self.targets[side])
            errors[side] = dict(translation_m=translation, rotation_deg=math.degrees(angle))
        self.report['dual_recenter_verification'] = dict(
            target_source=str(ROOT/self.policy['side']['atlas_path']), target_errors=errors,
            actual_whole_robot_payload_radius_m=radius, side_placement_radius_m=self.radius,
            side_placement_radius_applied=True,
            reason='door attachment-conditioned candidates sampled around the latest approved atlas')

    def run(self):
        self.enter('initialize')
        self.lower_distance = (self.request.snapshot.object(f'wall_box_{self.request.upper_box:02d}').pose.position[2]
                               - self.request.snapshot.object(f'wall_box_{self.request.lower_box:02d}').pose.position[2])
        self.audit([self.q], [], self.q)
        self.append([self.q])
        if self.outer_flow():
            self.report['pair_strategy']='outer_standard_rack_supported_lower_recenter'
            self.right_precontact_extra_rotation_max_deg=120.
            self.left_precontact_extra_rotation_max_deg=70.
            self.contact_upper_ranked(f'wall_box_{self.request.upper_box:02d}',[0]+RIGHT)
            upper=next(x for x in self.store.snapshot().attachments if x.object_id==f'wall_box_{self.request.upper_box:02d}')
            self.support_pose=self.box_pose(self.q,upper);self.support_side='right';self.defer_supported_extract=True
            self.contact_lower_ranked(f'wall_box_{self.request.lower_box:02d}',LEFT);self.defer_supported_extract=False
            lower=next(x for x in self.store.snapshot().attachments if x.object_id==f'wall_box_{self.request.lower_box:02d}')
            self.enter('rack_support_transfer');original_upper=self.request.snapshot.object(upper.object_id)
            self.store.release(upper.object_id,self.store.snapshot().revision,original_upper.pose)
            self.support_pose=pose_matrix(original_upper.pose);self.support_side='right';self.contact_pair=('right',upper.object_id);self.validity_cache={};self.cached_key=None;self.append([self.q])
            self.allow_partial_extract=True;self.extract('left',[0]+LEFT);self.allow_partial_extract=False
            extraction_shift={0:.35,1:.35,2:.35,3:.35,4:.25}[self.request.column]
            self.switch_pick_base(extraction_shift);self.contact_pair=None;self.support_pose=None;self.support_side=None;self.validity_cache={};self.cached_key=None;self.audit([self.q],[],self.q)
            self.finish_wall_clearance('left',lower,[0]+LEFT);self.recenter('left',[0]+LEFT,check_lowering=False)
            clearance=self.wall_clearance(self.q,lower,self.request.snapshot.object(lower.object_id));self.report['lower_wait_clearance_m']=clearance
            if clearance<1e-6:self.fail('rack-supported lower recenter did not clear the wall')
            self.find_right_upper_regrasp_base();self.skip_upper_lower_reachability=True
            self.contact_upper_ranked(f'wall_box_{self.request.upper_box:02d}',[0]+RIGHT);self.skip_upper_lower_reachability=False
            lowering_started=time.perf_counter();lowering,attempts=self.validate_future_lowering(self.q);self.record_operation_time('future_lowering_validation',lowering_started)
            self.report['post_regrasp_lowering_attempts']=attempts
            if lowering is None:lowering_started=time.perf_counter();lowering=self.plan_compensated_upper_lowering();self.record_operation_time('compensated_lowering_search',lowering_started)
            if lowering is not None:
                self.future_lowering_paths={tuple(np.round(self.q,6)):lowering};self.lower_upper_with_lower_hold()
            else:self.report['upper_lowering_skipped']='no certified compensated descent; extract upper at its current height'
            self.extract_upper_with_lower_hold();self.recenter_latest_pair();self.align_pair();self.append_suffix();self.report['success']=True;return self.report
        if False:
            self.report['pair_strategy']='mirrored_support_upper_then_lower_recenter_then_upper_recenter'
            self.contact_upper_left_ranked(f'wall_box_{self.request.upper_box:02d}',[0]+LEFT)
            upper=next(x for x in self.store.snapshot().attachments if x.object_id==f'wall_box_{self.request.upper_box:02d}')
            self.support_pose=self.box_pose(self.q,upper);self.support_side='left';self.contact_lower_right_ranked(f'wall_box_{self.request.lower_box:02d}',RIGHT)
            lower=next(x for x in self.store.snapshot().attachments if x.object_id==f'wall_box_{self.request.lower_box:02d}')
            self.enter('rack_support_transfer');original_upper=self.request.snapshot.object(upper.object_id)
            self.store.release(upper.object_id,self.store.snapshot().revision,original_upper.pose)
            self.support_pose=pose_matrix(original_upper.pose);self.support_side='left';self.contact_pair=('left',upper.object_id);self.validity_cache={};self.cached_key=None;self.append([self.q])
            self.allow_partial_extract=True;self.extract('right',[0]+RIGHT);self.allow_partial_extract=False
            self.switch_pick_base(.12);self.contact_pair=None;self.support_pose=None;self.support_side=None;self.validity_cache={};self.cached_key=None;self.audit([self.q],[],self.q)
            self.finish_wall_clearance('right',lower,[0]+RIGHT);self.recenter('right',[0]+RIGHT,check_lowering=False)
            clearance=self.wall_clearance(self.q,lower,self.request.snapshot.object(lower.object_id));self.report['lower_wait_clearance_m']=clearance
            if clearance<1e-6:self.fail('rack-supported lower recenter did not clear the wall')
            self.find_upper_regrasp_base();self.contact_upper_left_ranked(f'wall_box_{self.request.upper_box:02d}',[0]+LEFT)
            self.lower_and_extract_upper_mirrored();self.recenter_latest_pair();self.align_pair();self.append_suffix();self.report['success']=True;return self.report
        self.contact_upper_ranked(f'wall_box_{self.request.upper_box:02d}', [0]+RIGHT)
        if getattr(self,'upper_first',False):
            self.report['pair_strategy']='upper_extract_recenter_then_lower_extract_recenter'
            self.allow_partial_extract=True
            self.extract('right',[0]+RIGHT)
            self.allow_partial_extract=False
            upper=next(x for x in self.store.snapshot().attachments if x.object_id==f'wall_box_{self.request.upper_box:02d}')
            self.finish_wall_clearance('right',upper,[0]+RIGHT)
            self.recenter_latest_pair()
            upper=next(x for x in self.store.snapshot().attachments if x.object_id==f'wall_box_{self.request.upper_box:02d}')
            upper_clearance=self.wall_clearance(self.q,upper,self.request.snapshot.object(upper.object_id))
            self.report['upper_direct_recenter_clearance_m']=upper_clearance
            if upper_clearance<1e-6:self.fail('upper direct recenter did not clear the wall')
            self.support_pose=self.box_pose(self.q,upper)
            self.contact_lower_ranked(f'wall_box_{self.request.lower_box:02d}',LEFT)
            self.extract('left',LEFT)
            self.recenter('left',LEFT,check_lowering=False)
            self.support_pose=None
        else:
            self.report['pair_strategy']='support_upper_then_lower_recenter_then_upper_recenter'
            upper = next(x for x in self.store.snapshot().attachments if x.object_id==f'wall_box_{self.request.upper_box:02d}')
            self.support_pose = self.box_pose(self.q, upper)
            self.support_side='right'
            self.contact_lower_ranked(f'wall_box_{self.request.lower_box:02d}', LEFT)
            self.extract('left', LEFT)
            self.recenter('left', LEFT, check_lowering=True)
            self.support_pose = None
            self.support_side=None
            self.lower_upper_with_lower_hold()
            self.extract_upper_with_lower_hold()
            self.recenter_latest_pair()
        self.align_pair()
        self.append_suffix()
        self.report['success'] = True
        return self.report

    def append_suffix(self):
        self.store=SceneStore(snapshot_in_world(self.store.snapshot(),self.pick_base_pose))
        held = self.store.snapshot().attachments
        expected={f'wall_box_{value:02d}' for value in (self.request.lower_box,self.request.upper_box)}
        if {x.object_id for x in held} != expected:
            self.fail('both loads must still be attached at the suffix boundary')
        self.planner.snapshot = self.store.snapshot()
        self.planner.held = held
        # Extend the same report so an ordinary-placement failure retains its partial path.
        names = ['base_x', 'base_y', 'base_yaw', *ACTIVE_JOINTS]
        if len(self.base_pose_by_frame)!=len(self.report['frames']):
            self.fail('base pose/frame history mismatch')
        self.report['frames']=[[*pose.position[:2],2*math.atan2(pose.quaternion_wxyz[3],pose.quaternion_wxyz[0]),*q]
                               for pose,q in zip(self.base_pose_by_frame,self.report['frames'])]
        self.report['joint_names'] = names
        self.suffix_segments = []
        def append(rows, phase, payload):
            self.enter(phase)
            rows = np.asarray(rows, dtype=float)
            previous = np.asarray(self.report['frames'][-1])
            if phase not in ('base_reposition', 'base_return') and not np.allclose(rows[0], previous, atol=1e-6, rtol=0):
                rows = np.vstack([previous, rows])
            if phase in ('base_reposition', 'base_return'):
                yaw = rows[-1, 2]
                base_pose = Pose((rows[-1, 0], rows[-1, 1], 0.), (math.cos(yaw/2), 0., 0., math.sin(yaw/2)))
                self.planner.snapshot = replace(self.planner.snapshot,
                    state=self.planner.snapshot.state.with_positions(dict(zip(names, rows[-1])), base_pose))
            # conveyor_suffix already validates the navigation endpoint, loaded placement path,
            # released start, and dense Home path. Do not rebuild the same checker and repeat
            # the same mask in the append callback.
            self.report['conveyor_suffix_append_validation']='prevalidated by conveyor_suffix before append'
            first = len(self.report['frames'])
            self.report['frames'].extend(rows.tolist())
            self.report['phases'].extend([phase]*len(rows))
            self.report['payload'].extend([payload]*len(rows))
            self.report['attachments_by_frame'].extend([[asdict(x) for x in held] if payload else [] for _ in rows])
            self.suffix_segments.append((first, len(self.report['frames'])-1, self.planner.snapshot, payload))
            if phase == 'release':
                for item in held:
                    side = item.parent_link.removesuffix('_tool0')
                    pose = self.report['placed_boxes'][side]
                    self.report['predicted_scene_events'].append(dict(phase='release', object_id=item.object_id,
                        frame_index=first, world_pose=dict(position=pose['position'], quaternion_wxyz=pose['quaternion_wxyz'])))
                    self.store.release(item.object_id, self.store.snapshot().revision)
            self.q = rows[-1, 3:].copy()
            snapshot = self.store.snapshot()
            state = snapshot.state.with_positions(dict(zip(names, rows[-1])), self.planner.snapshot.state.base_pose)
            self.store.update_state(state, snapshot.revision)
        self.planner.suffix_inflation = self.report.setdefault('free_space_inflation', [])
        self.planner.conveyor_suffix(dict(self.request.tasks), self.report, append, self.progress)
        self.report['final_snapshot'] = self.store.snapshot().to_dict()

    def apply_dynamics(self, payload_mass_kg=10., torque_limit_scale=.9, budget_s=3.):
        if not self.report.get('time_parameterized'):
            self.fail('dynamics gate requires a time-parameterized result')
        model_key=(float(payload_mass_kg),tuple(self.planner.box_fit['dimensions_m']),float(torque_limit_scale))
        torque_cache=getattr(self.planner,'_torque_model_cache',{})
        model=torque_cache.get(model_key)
        if model is None:
            build_started = time.perf_counter()
            model = default_torque_model(
                self.planner.args, payload_mass_kg=payload_mass_kg,
                payload_dimensions=self.planner.box_fit['dimensions_m'],
                torque_limit_scale=torque_limit_scale, dynamics_root=getattr(self.planner, 'dynamics_root', None))
            record_build(self.planner, 'torque_model', build_started)
            torque_cache[model_key]=model
            self.planner._torque_model_cache=torque_cache
        names = list(self.report['joint_names'])
        source_positions = np.asarray(self.report['frames'], dtype=np.float32).copy()
        deadline = time.perf_counter()+budget_s
        segments, reports = [], []
        for metadata in self.report['timed_segments']:
            first, last = metadata['frame_start'], metadata['frame_end']
            selection = slice(first, last+1)
            time_values = np.asarray(self.report['time_from_start_s'][selection], dtype=float)
            time_values -= time_values[0]
            segment = dict(metadata, joint_names=names,
                           time_from_start_s=time_values.tolist(),
                           duration_s=float(time_values[-1]),
                           positions=self.report['frames'][selection],
                           velocities=self.report['velocities'][selection],
                           accelerations=self.report['accelerations'][selection],
                           jerks=self.report['jerks'][selection])
            attached = self.report['attachments_by_frame'][first]
            tool_to_box = {item['parent_link'].removesuffix('_tool0'):
                           pose_matrix(Pose(**item['tool_to_object'])).tolist()
                           for item in attached}
            active_sides = tuple(sorted(tool_to_box))
            if len(time_values) >= 3 and time_values[-1] > 0:
                output, before, constraint = model.constrain(
                    segment, active_sides, tool_to_box, deadline=deadline)
                if output is None or not constraint.get('success'):
                    self.report['torque_validation'] = dict(
                        success=False, failed_phase=metadata['name'], before=before,
                        constraint=constraint, payload_mass_kg=payload_mass_kg)
                    self.fail(f"RNEA torque gate failed: {metadata['name']}: {constraint}")
                segment = output
                after = constraint.get('after') or model.audit(segment, active_sides, tool_to_box)
            else:
                before = after = None
                constraint = dict(success=True, retiming_mode='stationary_event',
                                  time_scale=1., duration_before_s=0., duration_after_s=0.)
            segments.append(segment)
            reports.append(dict(phase=metadata['name'], active_sides=list(active_sides),
                                tool_to_box=tool_to_box, before=before,
                                constraint=constraint, after=after))
        self.report['timed_segments'] = segments
        self.report.update(assemble_timed_cycle(len(self.report['frames']), segments))
        final_positions = np.asarray(self.report['frames'], dtype=np.float32)
        geometry_delta = (float(np.max(np.abs(final_positions-source_positions)))
                          if final_positions.shape == source_positions.shape else float('inf'))
        if geometry_delta != 0.:
            self.fail(f'RNEA retiming changed fixed geometry: {geometry_delta}')
        self.report['dynamics_enabled'] = True
        self.report['torque_constraints_enabled'] = True
        self.report['torque_validation'] = dict(
            success=True, geometry_frozen=True, maximum_position_delta=geometry_delta,
            payload_mass_kg=payload_mass_kg,
            torque_limit_scale=torque_limit_scale,
            retimed_segments=sum(item['constraint'].get('time_scale',1.)>1.000001 for item in reports),
            local_retimed_segments=sum(item['constraint'].get('retiming_mode')=='local_window' for item in reports),
            maximum_utilization=max((item['after'] or {}).get('maximum_utilization',0.) for item in reports),
            maximum_gravity_utilization=max((item['after'] or {}).get('maximum_gravity_utilization',0.) for item in reports),
            segments=reports)
        self.report['motion_duration_s'] = self.report['time_from_start_s'][-1]

    def finalize_timing(self):
        """Same cubic/stopped-path timing and gates for A/B; navigation has no invented clock."""
        if not self.report['frames']:
            return
        names = self.report['joint_names']
        mobile = len(names) == 18
        original = {k: self.report[k] for k in ('frames', 'phases', 'payload', 'attachments_by_frame')}
        out = {k: [] for k in original}
        out.update(time_from_start_s=[], velocities=[], accelerations=[], jerks=[], timed_segments=[])
        saved = self.store, self.support_pose, self.contact_pair, self.q.copy(), self.planner.snapshot
        saved_phase = getattr(self, 'phase', 'initialize')
        remap = {}
        segments = [(a,b,s,support,contact,None) for a,b,s,support,contact in self.motion_segments]
        segments += [(a,b,s,None,None,payload) for a,b,s,payload in getattr(self,'suffix_segments',[])]
        clock = 0.
        caps = None
        try:
            for first,last,snapshot,support,contact,suffix_payload in segments:
                rows = np.asarray(original['frames'][first:last+1], dtype=float)
                phase = original['phases'][first]
                self.phase = phase
                self.store, self.support_pose, self.contact_pair = SceneStore(snapshot), support, contact
                self.q = rows[0,-15:]
                self.planner.snapshot = snapshot
                # Polyline retiming reuses audited geometry; only cubic/stops fallbacks need a checker.
                validity = None
                if caps is None:
                    # Joint limits belong to the robot model: identical for every loaded/empty checker.
                    limits = self.validity().checker.kinematics.get_joint_limits()
                    caps = [np.abs(getattr(limits,k)[1].cpu().numpy()) for k in ('velocity','acceleration','jerk')]
                    if mobile:
                        caps = [np.r_[np.ones(3), c] for c in caps]
                navigation=phase in ('base_pick_reposition','base_reposition','base_return')
                trajectory = {}
                if navigation or len(rows)<2 or np.allclose(rows,rows[0],atol=1e-12,rtol=0):
                    sample = dict(positions=rows.tolist(), time_from_start_s=[0.]*len(rows),
                                  **{k:np.zeros_like(rows).tolist() for k in ('velocities','accelerations','jerks')})
                    method = 'navigation_event_unvalidated' if navigation else 'stationary_event'
                else:
                    selected = False
                    # Polyline has won every accepted door segment; validate it first and only
                    # pay for cubic/stops as safety fallbacks instead of evaluating all three.
                    # Polyline is shortened as well as lengthened to the binding sampled limit, under
                    # the quintic and smooth-trapezoid time scalings (shortest wins); the 1.05 safety
                    # factor and every derivative/RNEA gate below are unchanged.
                    for method, parameterize in (
                            ('validated_polyline',time_parameterize_polyline_shortest),
                            ('validated_cubic',time_parameterize_path),
                            ('validated_stops',time_parameterize_stops)):
                        trajectory, validation = parameterize(rows, names, *caps, sample_dt=.025)
                        sample = trajectory['sampled_trajectory']
                        geometry_reused = method=='validated_polyline'
                        # Polyline retiming changes timestamps only: every segment is the same
                        # previously validated dense geometry. Cubic/stops remain full fallbacks.
                        if not geometry_reused:
                            if validity is None:
                                validity = (self.validity() if suffix_payload is None else
                                            self.planner.placement_validity(self.planner.checker(dict(self.request.tasks), payload=suffix_payload), suffix_payload))
                            if not bool(validity.mask(self.tensor(np.asarray(validation)[:,-15:])).all()):
                                continue
                        if suffix_payload is None:
                            active = np.flatnonzero(np.ptp(rows[:,-15:],axis=0)>1e-9).tolist()
                            line = None
                            if phase in ('left_contact','right_contact','left_extract','right_extract','right_lower'):
                                side = phase.split('_')[0]
                                origin = self.tool(rows[0,-15:], side)
                                end = self.tool(rows[-1,-15:], side)
                                direction = np.array((0.,0.,-1.) if phase=='right_lower' else
                                                     (self.wall_inward_direction if phase.endswith('_contact') else self.wall_outward_direction))
                                line = (side, origin, direction, float(np.dot(end[:3,3]-origin[:3,3],direction)))
                            timed_rows=np.asarray(sample['positions'])[:,-15:]
                            if geometry_reused:
                                try:
                                    check_frozen(timed_rows,rows[0,-15:],active)
                                except ValueError:
                                    continue
                            else:
                                try:
                                    self.audit(timed_rows,active,rows[0,-15:],**({'line':line} if line else {}))
                                except CycleBlocked:
                                    self.report.pop('failure_location', None)
                                    continue
                        selected = True
                        if geometry_reused:
                            self.report.setdefault('retiming_geometry_reuse',[]).append(
                                dict(phase=phase,method=method,time_scaling=trajectory.get('time_scaling'),
                                     raw_points=len(rows),sampled_points=len(sample['positions']),
                                     proof='same piecewise-linear path; source geometry already stage-audited'))
                        break
                    if not selected:
                        self.fail('time parameterization failed collision/geometry validation: '+phase)
                start = len(out['frames'])
                n = len(sample['positions'])
                out['frames'].extend(sample['positions'])
                for k in ('phases','payload','attachments_by_frame'):
                    out[k].extend([original[k][first]]*n)
                out['time_from_start_s'].extend(clock+float(t) for t in sample['time_from_start_s'])
                for k,cap in zip(('velocities','accelerations','jerks'),caps):
                    values = np.asarray(sample[k])
                    if not np.isfinite(values).all() or np.max(np.abs(values)/cap)>1.0001:
                        self.fail('time derivative limit failed: '+phase)
                    out[k].extend(sample[k])
                duration = float(sample['time_from_start_s'][-1])
                out['timed_segments'].append(dict(name=phase, frame_start=start, frame_end=start+n-1,
                                                 duration_s=duration, method=method,
                                                 time_scaling=trajectory.get('time_scaling')))
                remap[first], remap[last] = start, start+n-1
                clock += duration
            for event in self.report['predicted_scene_events']:
                event['geometric_frame_index'] = event['frame_index']
                event['frame_index'] = remap[event['frame_index']]
            self.report.pop('failure_location', None)
            self.report.update(out, time_parameterized=True, motion_duration_s=clock,
                               timing_navigation_excluded=True)
        finally:
            self.store, self.support_pose, self.contact_pair, self.q, self.planner.snapshot = saved
            self.phase = saved_phase
