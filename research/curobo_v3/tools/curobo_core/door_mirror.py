"""A door column planned in the mirrored frame: world y -> -y, the arms' roles swapped.

The door flow takes the lower box with the left arm and the upper box with the right arm. At column 0,
the leftmost, the right arm can regrasp the upper box from no base state: all 120 base candidates of
find_right_upper_regrasp_base fail, on the legacy and on the re-zeroed model. Reflected across the world
xz-plane, which the robot model, its joint frames and the pick bases share, column 0 becomes column 4
with the roles swapped. plan_mirrored_round reflects the column's actual scene, plans it there with the
standard flow and maps the timed path back (mirror_result). Then it checks the path on the actual arms:
RNEA, and every door frame against the actual scene (validate_door_frames). Unlike the column-4 symmetry
derivation (v3_door_five_columns.mirror_round), it does not need a symmetric scene and shares no timing
with another round.
"""
import math
import time
import traceback
from dataclasses import asdict, replace
from types import SimpleNamespace

import numpy as np

from .adapter import matrix_pose, pose_matrix
from .contracts import ACTIVE_JOINTS
from .scene import AttachedObject, MeshObject, Pose, SceneObject


def mirror_side(value):
    return value.replace('left', '__swap__').replace('right', 'left').replace('__swap__', 'right')


def mirror_box(index):
    """The wall box at the mirrored column of the same row (5 x 5 grid, column = index % 5)."""
    row, column = divmod(int(index), 5)
    return row*5+4-column


# Side-named environment objects, swapped by the reflection (the involution maps each to the other).
SIDE_OBJECT_IDS = {'left_wall': 'right_wall', 'right_wall': 'left_wall',
                   'conveyor_pair/left': 'conveyor_pair/right', 'conveyor_pair/right': 'conveyor_pair/left'}


def mirror_object_id(object_id):
    if object_id.startswith('wall_box_'):
        return f"wall_box_{mirror_box(object_id.removeprefix('wall_box_')):02d}"
    return SIDE_OBJECT_IDS.get(object_id, object_id)


def mirror_pose(pose):
    (x, y, z), (w, qx, qy, qz) = pose.position, pose.quaternion_wxyz
    return Pose((x, -y, z), (w, -qx, qy, -qz))


def mirror_joints(values):
    """ACTIVE_JOINTS values with the arms swapped: each arm joint takes the negated other-side value."""
    values = np.asarray(values, dtype=float)
    out = values.copy()
    out[1:8], out[8:15] = -values[8:15], -values[1:8]
    return out


def mirror_row(row):
    """A trajectory row (base x, y, yaw, then ACTIVE_JOINTS) in the mirrored frame."""
    row = np.asarray(row, dtype=float)
    out = row.copy()
    out[1] *= -1
    out[2] *= -1
    out[3:] = mirror_joints(row[3:])
    return out


def mirror_snapshot(snapshot):
    """The scene reflected across the world xz-plane, with the wall boxes renamed to their mirrored slot."""
    if snapshot.attachments:
        raise ValueError('only a scene without attachments can be mirrored')
    positions = dict(zip(snapshot.state.joint_names, snapshot.state.positions))
    mirrored = dict(zip(ACTIVE_JOINTS, mirror_joints(snapshot.state.ordered(ACTIVE_JOINTS))))
    state = replace(snapshot.state, positions=tuple(mirrored.get(name, value) for name, value in positions.items()),
                    base_pose=mirror_pose(snapshot.state.base_pose))
    objects = tuple(SceneObject(mirror_object_id(item.object_id), item.dimensions_m, mirror_pose(item.pose))
                    for item in snapshot.objects)
    # Reflection reverses orientation: reversing each triangle keeps the faces' outward normals.
    meshes = tuple(MeshObject(mirror_object_id(item.object_id), tuple((x, -y, z) for x, y, z in item.vertices_m),
                              tuple((a, c, b) for a, b, c in item.faces), mirror_pose(item.pose))
                   for item in snapshot.meshes)
    return replace(snapshot, state=state, objects=objects, meshes=meshes)


def mirrored_request(request):
    """A door_request in the mirrored frame: mirrored column, scene and box ids, the same arm roles."""
    return SimpleNamespace(**{**vars(request), 'snapshot': mirror_snapshot(request.snapshot),
                              'column': 4-int(request.column), 'upper_box': mirror_box(request.upper_box),
                              'lower_box': mirror_box(request.lower_box),
                              'tasks': tuple((side, mirror_box(box)) for side, box in request.tasks),
                              'removed_box_ids': tuple(sorted(mirror_box(value) for value in request.removed_box_ids))})


def base_pose_of(row):
    yaw = row[2]
    return Pose((row[0], row[1], 0.), (math.cos(yaw/2), 0., 0., math.sin(yaw/2)))


def mirror_result(source, planner, request):
    """A timed door round planned for mirrored_request(request), mapped back to request's actual frame.

    Frames, phases, task sides, timing arrays and segments are reflected; each attachment is re-derived
    from the tool pose and the actual box pose when the box is picked, and the release events keep the
    reflected release poses."""
    frames = np.asarray([mirror_row(row) for row in source['frames']])
    phases = [mirror_side(value) for value in source['phases']]
    wall = {item.object_id: item.pose for item in request.snapshot.objects if item.object_id.startswith('wall_box_')}
    active, attachments, previous = {}, [], set()
    for frame, items in zip(frames, source['attachments_by_frame']):
        current = {mirror_object_id(item['object_id']) for item in items}
        for object_id in current-previous:
            original = next(item for item in items if mirror_object_id(item['object_id']) == object_id)
            side = mirror_side(original['parent_link'].removesuffix('_tool0'))
            planner.fk(frame[3:])
            tool = planner.fk_robot.get_transform(side+'_tool0', planner.fk_robot.base_link)
            local = np.linalg.inv(pose_matrix(base_pose_of(frame)))@pose_matrix(wall[object_id])
            active[object_id] = AttachedObject(object_id, tuple(original['dimensions_m']), side+'_tool0',
                                               matrix_pose(np.linalg.inv(tool)@local), (side+'_link7',))
        for object_id in previous-current:
            active.pop(object_id, None)
        attachments.append([asdict(active[item]) for item in sorted(current)])
        previous = current
    result = dict(source)
    first, final = frames[0], frames[-1]
    initial = replace(request.snapshot, state=request.snapshot.state.with_positions(
        dict(zip(ACTIVE_JOINTS, first[3:])), base_pose_of(first)))
    pair = {f'wall_box_{request.lower_box:02d}', f'wall_box_{request.upper_box:02d}'}
    events = []
    for event in source.get('predicted_scene_events', ()):
        mapped = {key: value for key, value in event.items() if key != 'snapshot'}
        mapped.update(phase=mirror_side(event['phase']), object_id=mirror_object_id(event['object_id']))
        if event.get('world_pose'):
            mapped['world_pose'] = asdict(mirror_pose(Pose(**event['world_pose'])))
        events.append(mapped)
    if {event['object_id'] for event in events if event['phase'] == 'release'} != pair:
        raise ValueError('the mirrored round does not release exactly its pair')
    result.update(success=True, column=int(request.column), frames=frames.tolist(), phases=phases,
                  task={mirror_side(side): mirror_box(box) for side, box in source['task'].items()},
                  attachments_by_frame=attachments, payload=[bool(item) for item in attachments],
                  initial_snapshot=initial.to_dict(), pick_base_pose=asdict(base_pose_of(first)),
                  predicted_scene_events=events, navigation_path_validated=False,
                  plan_cache=dict(enabled=False, hit=False),
                  final_snapshot=replace(request.snapshot, attachments=(),
                                         objects=tuple(item for item in request.snapshot.objects if item.object_id not in pair),
                                         state=request.snapshot.state.with_positions(dict(zip(ACTIVE_JOINTS, final[3:])),
                                                                                     base_pose_of(final))).to_dict())
    for key in ('velocities', 'accelerations', 'jerks'):
        result[key] = [mirror_row(row).tolist() for row in source[key]]
    result['timed_segments'] = [dict(segment, **{key: [mirror_row(row).tolist() for row in segment[key]]
                                                 for key in ('positions', 'velocities', 'accelerations', 'jerks')
                                                 if key in segment})
                                for segment in source.get('timed_segments', ())]
    return result


def door_frame_groups(planner, result, request):
    """Door frames (up to base_reposition) as groups of equal (scene, attachments, permitted contact).

    Each frame's scene is the round-start scene minus the boxes attached in that frame, and minus the
    released pair from the release on. The contact phases delegate their tool-box pair to the exact
    mesh-plane proof, as the door validity does when planning; the upper box's regrasp approach that
    ends on its face passes the same proof. Returns {(geometry_key, contact): [snapshot, contact,
    frame indices, joint rows]}, each snapshot in the base frame. A scene is built and hashed once per
    (base pose, attachments): the objects are static, so joints only select the row to check."""
    from dataclasses import replace as replace_dataclass
    from .adapter import pose_matrix
    from .door_recenter import snapshot_in_base
    from .door_sequential import mesh_contact_proof, tool_mesh_vertices
    frames, phases = np.asarray(result['frames']), result['phases']
    door_end = next((index for index, phase in enumerate(phases) if phase == 'base_reposition'), len(frames))
    upper, lower = f'wall_box_{request.upper_box:02d}', f'wall_box_{request.lower_box:02d}'
    side_of = {f'wall_box_{box:02d}': side for side, box in result['task'].items()}
    upper_side = side_of[upper]
    contacts = {f'{upper_side}_contact': (upper_side, upper), f'{side_of[lower]}_contact': (side_of[lower], lower),
                'rack_support_transfer': (upper_side, upper)}
    release = next((index for index, phase in enumerate(phases) if phase == 'release'), len(frames))
    tool_meshes = {side: tool_mesh_vertices(planner, side) for side in ('left', 'right')}
    groups, scenes, geometries = {}, {}, {}
    for index in range(door_end):
        frame = frames[index]
        items = tuple(AttachedObject(item['object_id'], item['dimensions_m'], item['parent_link'], Pose(**item['tool_to_object']),
                                     item.get('touch_links', ())) for item in result['attachments_by_frame'][index])
        attached = {item.object_id for item in items}
        # The scene depends on the base pose and the attached objects only, not on the joints; hashing one
        # snapshot per (base pose, attachments) keeps the per-frame cost to a tuple key.
        scene_key = (round(frame[0], 9), round(frame[1], 9), round(frame[2], 9), index >= release,
                     tuple((item.object_id, item.parent_link, item.dimensions_m, item.tool_to_object.position,
                            item.tool_to_object.quaternion_wxyz) for item in items))
        geometry = geometries.get(scene_key)
        if geometry is None:
            objects = tuple(item for item in request.snapshot.objects
                            if item.object_id not in attached and not (index >= release and item.object_id in (upper, lower)))
            snapshot = snapshot_in_base(replace_dataclass(request.snapshot, objects=objects, attachments=items, revision=0,
                state=request.snapshot.state.with_positions(dict(zip(ACTIVE_JOINTS, frame[3:])), base_pose_of(frame))))
            scenes[scene_key] = snapshot
            # The scene digest covers every object and mesh: once per scene, not once per frame.
            geometry = geometries[scene_key] = snapshot.geometry_key
        snapshot = scenes[scene_key]
        contact = contacts.get(phases[index])
        if contact is None and upper not in attached and any(item.object_id == upper for item in snapshot.objects):
            # The upper box's regrasp approach ends on its face: the same exact contact proof as planning.
            planner.fk(frame[3:])
            tool = planner.fk_robot.get_transform(upper_side+'_tool0', planner.fk_robot.base_link)
            item = snapshot.object(upper)
            if mesh_contact_proof(tool_meshes[upper_side], np.linalg.inv(pose_matrix(item.pose))@tool, item.dimensions_m, .001,
                                  tolerance=request.cartesian_geometry_tolerance_m)['valid']:
                contact = (upper_side, upper)
        group = groups.setdefault((geometry, contact), [snapshot, contact, [], []])
        group[2].append(index)
        group[3].append(frame[3:])
    return groups, {index: (group[0], group[1]) for group in groups.values() for index in group[2]}


def validate_door_frames(planner, result, request, rebuild_per_contact=False):
    """The door frames (up to base_reposition) of an actual-frame round against request's actual scene.

    Each frame's scene is the round-start scene minus the boxes attached in that frame, and minus the
    released pair from the release on; the contact phases delegate their tool-box pair to the exact
    mesh-plane proof, as the door validity does when planning. GPU sphere checks without the planning
    margins, then an exact mesh check of each failing frame (deeper than 0.1 mm fails), as
    v3_verify_mirrored_door. The mirrored-frame plan was checked with the mirrored robot, whose spheres
    are not an exact mirror of the actual arms'.

    One checker is built per distinct scene and kept on the planner across rounds like the planning ones,
    and the permitted contact box is disabled (cuRobo's obstacle enable flag) only for its own frames;
    `rebuild_per_contact` builds a fresh checker per scene-minus-contact instead, for equivalence checks.
    """
    import torch
    from curobo.collision_checking import RobotCollisionChecker, RobotCollisionCheckerCfg
    from v3_batched_loaded_search import loaded_robot
    from .adapter import to_curobo_scene
    from .door_sequential import tool_mesh_vertices
    from .door_sequential_validity import SequentialValidity
    from .mesh_validation import MeshStateChecker
    began = time.perf_counter()
    frames, phases = np.asarray(result['frames']), result['phases']
    door_end = next((index for index, phase in enumerate(phases) if phase == 'base_reposition'), len(frames))
    groups, frame_scenes = door_frame_groups(planner, result, request)
    grouping_ms = (time.perf_counter()-began)*1000
    tool_meshes = {side: tool_mesh_vertices(planner, side) for side in ('left', 'right')}
    from .resource_stats import bounded_get, record_build
    cache = getattr(planner, '_door_validation_checker_cache', {})
    built = [0, 0.]

    def build(scene, objects):
        def create():
            started = time.perf_counter()
            robot = loaded_robot(planner.robot, planner.box_fit, attachments=scene.attachments)
            checker = RobotCollisionChecker(RobotCollisionCheckerCfg.load_from_config(
                robot_config=robot, scene_model=to_curobo_scene(objects), n_cuboids=max(40, len(scene.objects)),
                n_meshes=len(scene.meshes), collision_activation_distance=0.))
            record_build(planner, 'door_validation_checker', started)
            built[0] += 1
            built[1] += (time.perf_counter()-started)*1000
            return checker
        return create

    gpu_failures, mask_ms, validity_ms, toggle_ms = [], 0., 0., 0.
    for snapshot, contact, indices, rows in groups.values():
        native = snapshot if contact is None else replace(snapshot, objects=tuple(
            item for item in snapshot.objects if item.object_id != contact[1]))
        if rebuild_per_contact:
            checker = build(native, native)()
        else:
            checker = bounded_get(cache, snapshot.geometry_key, build(snapshot, snapshot), 32)
        planner._door_validation_checker_cache = cache
        # The contact proof reads the target's pose and dimensions from the snapshot; only the checker's
        # world omits it.
        validity_started = time.perf_counter()
        validity = SequentialValidity(checker, snapshot, planner.robot, tool_meshes, contact,
                                      geometry_tolerance=request.cartesian_geometry_tolerance_m,
                                      fixed_body=getattr(planner, 'fixed_body', None))
        validity_ms += (time.perf_counter()-validity_started)*1000
        disabled = (contact is not None and not rebuild_per_contact
                    and any(item.object_id == contact[1] for item in snapshot.objects))
        try:
            if disabled:
                toggle_started = time.perf_counter()
                checker.scene_model.enable_obstacle(contact[1], False)
                toggle_ms += (time.perf_counter()-toggle_started)*1000
            mask_started = time.perf_counter()
            mask = validity.mask(torch.tensor(np.asarray(rows), device='cuda', dtype=torch.float32)).cpu().numpy()
            mask_ms += (time.perf_counter()-mask_started)*1000
        finally:
            if disabled:
                toggle_started = time.perf_counter()
                checker.scene_model.enable_obstacle(contact[1], True)
                toggle_ms += (time.perf_counter()-toggle_started)*1000
        gpu_failures.extend(indices[position] for position in np.flatnonzero(~mask))
        del validity, checker
    checks_ms = (time.perf_counter()-began)*1000
    failures, repairs = [], []
    for index in gpu_failures:
        snapshot, contact = frame_scenes[index]
        exact = snapshot if contact is None else replace(snapshot, objects=tuple(
            item for item in snapshot.objects if item.object_id != contact[1]))
        found = MeshStateChecker(planner.robot['kinematics'], exact, {}, exact.attachments).check(
            dict(zip(ACTIVE_JOINTS, frames[index, 3:])), attached=False, payload_enabled=True)
        entry = dict(frame=int(index), phase=phases[index], mesh_result=found)
        (repairs if found is None or found.get('contact_depth_m', float('inf')) <= 1e-4 else failures).append(entry)
    return dict(checked_frames=door_end, door_end_frame=door_end, scene_groups=len(groups),
                gpu_failures=len(gpu_failures), mesh_repaired=len(repairs), mesh_repairs=repairs[:20],
                failures=failures[:20], failure_count=len(failures), grouping_ms=grouping_ms,
                checkers_built=built[0], checker_ms=built[1], mask_ms=mask_ms, validity_ms=validity_ms,
                toggle_ms=toggle_ms, checks_ms=checks_ms,
                repair_ms=(time.perf_counter()-began)*1000-checks_ms, wall_ms=(time.perf_counter()-began)*1000)


def plan_mirrored_round(planner, request, policy, progress=lambda text: None, grasp_rolls=None):
    """Plan request's door pair in the mirrored frame and return (report, error) in the actual frame.

    The mirrored request is planned and time-parameterized with the standard flow (DoorTask), mapped
    back by mirror_result, RNEA-checked on the actual arms and frame-checked by validate_door_frames;
    a frame failing the exact mesh check fails the round."""
    from .door_recenter import DoorTask
    began = time.perf_counter()
    virtual = mirrored_request(request)
    planner.set_snapshot(virtual.snapshot)
    task = DoorTask(planner, virtual, 'joint_rrt', policy, progress=progress)
    for name, values in (grasp_rolls or {}).items():
        setattr(task, name, tuple(values))
        task.report['requested_'+name] = list(values)
    error, report, closed = None, task.report, False
    try:
        task.run()
        task.enter('time_parameterization')
        task.finalize_timing()
        task.close_stage_timing()
        closed = True
        planned_ms = (time.perf_counter()-began)*1000
        task.phase = 'mirror_to_actual_frame'
        mirror_started = time.perf_counter()
        planner.set_snapshot(request.snapshot)
        report = mirror_result(task.report, planner, request)
        timing = dict(report.get('stage_timing_ms', {}))
        timing['mirror_to_actual_frame'] = (time.perf_counter()-mirror_started)*1000
        dynamics = DoorTask.__new__(DoorTask)
        dynamics.planner, dynamics.report, dynamics.phase = planner, report, 'rnea_dynamics'
        dynamics.fail = lambda message: (_ for _ in ()).throw(RuntimeError(message))
        dynamics_started = time.perf_counter()
        dynamics.apply_dynamics()
        timing['rnea_dynamics'] = (time.perf_counter()-dynamics_started)*1000
        task.phase = 'actual_frame_check'
        validation = validate_door_frames(planner, report, request)
        timing['actual_frame_validation'] = validation['wall_ms']
        report.update(mode='door_recenter_mirrored_frame', stage_timing_ms=timing, actual_frame_validation=validation,
                      scene_geometry_key=request.snapshot.geometry_key,
                      world_scene_geometry_key=request.snapshot.geometry_key,
                      planning_frame=dict(mirrored=True, planned_column=virtual.column, planned_ms=planned_ms,
                                          virtual_scene_geometry_key=virtual.snapshot.geometry_key))
        if validation['failure_count']:
            raise RuntimeError(f"actual-frame check failed at {validation['failure_count']} door frames "
                               f"(first {validation['failures'][0]['frame']}, {validation['failures'][0]['phase']})")
    except Exception as exc:
        error = dict(type=type(exc).__name__, stage=task.phase, message=str(exc), traceback=traceback.format_exc())
        report.update(success=False, error=error)
    finally:
        planner.set_snapshot(request.snapshot)
        if report is task.report and not closed:
            task.close_stage_timing()
    report.update(initial_snapshot=request.snapshot.to_dict(), plan_cache=dict(enabled=False, hit=False))
    return report, error
