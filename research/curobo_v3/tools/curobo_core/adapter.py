import numpy as np
from scipy.spatial.transform import Rotation

from dataclasses import replace

from .scene import AttachedObject, Pose, SceneObject


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


def task_attachments(snapshot, tasks):
    from v3_wall_ik_benchmark import canonical_side_tool_to_box

    attachments = []
    for side, box_id in tasks.items():
        item = snapshot.object(f"wall_box_{box_id:02d}")
        attachments.append(AttachedObject(
            item.object_id, item.dimensions_m, side + "_tool0",
            matrix_pose(canonical_side_tool_to_box(side)), (side + "_link7", side + "_tool0"),
        ))
    return tuple(attachments)
