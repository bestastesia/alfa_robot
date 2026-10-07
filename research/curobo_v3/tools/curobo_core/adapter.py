import math
import numpy as np
from scipy.spatial.transform import Rotation

from dataclasses import replace

from .scene import AttachedObject, Pose, SceneObject

# Contact-plane extents measured from both frozen 071cb95 link7 meshes at Tool0 (z=.151).
TOP_PAD_SIZE_M = (.175, .355)
TOP_EDGE_CLEARANCE_M = .005


def top_contact_offset(dimensions):
    margins = (np.array(dimensions[:2]) - TOP_PAD_SIZE_M) / 2 - TOP_EDGE_CLEARANCE_M
    if np.any(margins < 0):
        raise ValueError("suction plate does not fit the top face with edge clearance")
    # ponytail: nearest robot-facing landing point; add a face search only if obstacles demand it.
    return np.array([-margins[0], 0., 0.])


def contact_offset(dimensions, mode, offset=None):
    """World-frame tangential offset, bounded by the entire frozen suction face."""
    if mode not in ("side", "top"):
        raise ValueError("contact mode must be side or top")
    if offset is None:
        return top_contact_offset(dimensions) if mode == "top" else np.zeros(3)
    offset = np.asarray(offset, dtype=float)
    normal = 0 if mode == "side" else 2
    extent = np.array([0., TOP_PAD_SIZE_M[1]/2, TOP_PAD_SIZE_M[0]/2] if mode == "side"
                      else [TOP_PAD_SIZE_M[0]/2, TOP_PAD_SIZE_M[1]/2, 0.])
    margin = np.array(dimensions)/2 - extent - TOP_EDGE_CLEARANCE_M
    margin[normal] = 0.
    if offset.shape != (3,) or not np.isfinite(offset).all() or np.any(np.abs(offset) > margin + 1e-12):
        raise ValueError("contact offset leaves the suction face or violates edge clearance")
    return offset


def side_face_variants(snapshot, tasks):
    # ponytail: center and two interior vertical candidates; no box-ID pose presets.
    for sign in (1., -1.):
        yield {side: (0., 0., sign * (snapshot.object(f"wall_box_{box_id:02d}").dimensions_m[2]/2
                                     - TOP_PAD_SIZE_M[0]/2 - TOP_EDGE_CLEARANCE_M)/2)
               for side, box_id in tasks.items()}


def pose_matrix(pose):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(np.roll(pose.quaternion_wxyz, -1)).as_matrix()
    matrix[:3, 3] = pose.position
    return matrix


def matrix_pose(matrix):
    return Pose(tuple(matrix[:3, 3]), tuple(np.roll(Rotation.from_matrix(matrix[:3, :3]).as_quat(), 1)))


def relative_pose(world_pose, base_pose):
    return matrix_pose(np.linalg.inv(pose_matrix(base_pose)) @ pose_matrix(world_pose))


def from_curobo_scene(scene):
    return tuple(SceneObject(item.name, tuple(item.dims),
                             Pose(tuple(item.pose[:3]), tuple(item.pose[3:7])))
                 for item in scene.cuboid or [])


def world_objects(snapshot, task_ids=(), mobile=False):
    if snapshot.frame_id != "map":
        raise ValueError("scene poses must be in map")
    excluded = set(task_ids) if snapshot.policy.exclude_task_objects_before_contact else set()
    output = []
    for item in snapshot.objects:
        if item.object_id == "ground":
            if (item.pose.quaternion_wxyz != Pose().quaternion_wxyz or
                    abs(item.pose.position[2] + item.dimensions_m[2] / 2 - snapshot.policy.ground_z_m) > 1e-9):
                raise ValueError("ground object must match the configured horizontal ground plane")
            continue
        if item.object_id in excluded:
            continue
        pose = item.pose if mobile else relative_pose(item.pose, snapshot.state.base_pose)
        output.append(SceneObject(item.object_id, item.dimensions_m, pose))
    return tuple(output)


def world_meshes(snapshot, mobile=False):
    if snapshot.frame_id != "map":
        raise ValueError("scene poses must be in map")
    return tuple(replace(item, pose=item.pose if mobile else relative_pose(
        item.pose, snapshot.state.base_pose)) for item in snapshot.meshes)


def to_curobo_scene(snapshot, task_ids=(), mobile=False):
    from curobo.scene import Cuboid, Mesh, Scene

    return Scene(cuboid=[Cuboid(name=item.object_id,
                               pose=[*item.pose.position, *item.pose.quaternion_wxyz],
                               dims=list(item.dimensions_m))
                        for item in world_objects(snapshot, task_ids, mobile)],
                 mesh=[Mesh(name=item.object_id, pose=[*item.pose.position, *item.pose.quaternion_wxyz],
                            vertices=[list(vertex) for vertex in item.vertices_m],
                            faces=[list(face) for face in item.faces])
                       for item in world_meshes(snapshot, mobile)])


def task_attachments(snapshot, tasks, suction_mode="side", offsets=None):
    attachments = []
    for side, box_id in tasks.items():
        item = snapshot.object(f"wall_box_{box_id:02d}")
        attachments.append(AttachedObject(
            item.object_id, item.dimensions_m, side + "_tool0",
            matrix_pose(tool_to_box(side, item.dimensions_m, suction_mode, (offsets or {}).get(side))), (side + "_link7", side + "_tool0"),
        ))
    return tuple(attachments)


def task_contact_positions(snapshot, tasks, gap_m=0.0, suction_mode="side", offsets=None):
    """Derive contacts from actual task objects, never from a fixed wall index."""
    if not math.isfinite(gap_m) or gap_m < 0:
        raise ValueError("contact gap must be finite and nonnegative")
    if suction_mode not in ("side", "top"):
        raise ValueError("contact requires a resolved side/top suction mode")
    positions = {}
    for side, box_id in tasks.items():
        item = snapshot.object(f"wall_box_{box_id:02d}")
        if side not in ("left", "right") or item.pose.quaternion_wxyz != Pose().quaternion_wxyz:
            raise ValueError("side contacts require left/right and axis-aligned task boxes")
        x, y, z = item.pose.position
        base = (x - (item.dimensions_m[0]/2 + gap_m), y, z) if suction_mode == "side" else (x, y, z + item.dimensions_m[2]/2 + gap_m)
        positions[side] = tuple(np.array(base) + contact_offset(item.dimensions_m, suction_mode, (offsets or {}).get(side)))
    return positions


def suction_quaternion(side, mode):
    from v3_wall_ik_benchmark import align_tool_z, canonical_side_suction_quaternion_wxyz

    reference = canonical_side_suction_quaternion_wxyz(side)
    if mode == "side":
        return reference
    if mode != "top":
        raise ValueError("suction mode must be side or top")
    return align_tool_z(reference, np.array([0., 0., -1.]))


def tool_to_box(side, dimensions, mode, offset=None):
    from v3_wall_ik_benchmark import canonical_side_tool_to_box

    delta = contact_offset(dimensions, mode, offset)
    if mode == "side" and not np.any(delta):
        return canonical_side_tool_to_box(side)
    transform = pose_matrix(Pose(quaternion_wxyz=tuple(suction_quaternion(side, mode))))
    transform[:3, :3] = transform[:3, :3].T
    face_center = np.array([-dimensions[0]/2, 0., 0.]) if mode == "side" else np.array([0., 0., dimensions[2]/2])
    transform[:3, 3] = transform[:3, :3] @ (-delta - face_center)
    return transform


def carry_targets(snapshot, tasks, targets, mode, offsets=None):
    """Keep the same BOX destination when the suction face changes."""
    output = dict(targets)
    if mode == "side" and not offsets:
        return output
    for side, box_id in tasks.items():
        name = side + "_tool0"
        item = snapshot.object(f"wall_box_{box_id:02d}")
        desired_tool = Pose(tuple(targets[name]["position"]), tuple(targets[name]["quaternion"]))
        desired_box = pose_matrix(desired_tool) @ tool_to_box(side, item.dimensions_m, "side")
        pose = matrix_pose(desired_box @ np.linalg.inv(tool_to_box(side, item.dimensions_m, mode, (offsets or {}).get(side))))
        output[name] = {"position": pose.position, "quaternion": pose.quaternion_wxyz}
    return output


def top_lift_blockers(snapshot, tasks, lift_m=0.35):
    """Conservative world AABB check of the box's vertical swept volume, including meshes."""
    if not math.isfinite(lift_m) or lift_m <= 0:
        raise ValueError("top lift must be finite and positive")
    blockers = set()
    for box_id in tasks.values():
        item = snapshot.object(f"wall_box_{box_id:02d}")
        center, half = np.array(item.pose.position), np.array(item.dimensions_m)/2
        low, high = center-half, center+half+np.array([0., 0., lift_m])
        for obstacle in snapshot.objects + snapshot.meshes:
            if obstacle.object_id == item.object_id:
                continue
            transform = pose_matrix(obstacle.pose)
            if hasattr(obstacle, "dimensions_m"):
                extent = np.abs(transform[:3, :3]) @ (np.array(obstacle.dimensions_m)/2)
                other_low, other_high = transform[:3, 3]-extent, transform[:3, 3]+extent
            else:
                vertices = np.array(obstacle.vertices_m) @ transform[:3, :3].T + transform[:3, 3]
                other_low, other_high = vertices.min(axis=0), vertices.max(axis=0)
            if np.all(np.minimum(high, other_high)-np.maximum(low, other_low) > 1e-6):
                blockers.add(obstacle.object_id)
    return sorted(blockers)
