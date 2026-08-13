from __future__ import annotations

import copy
import math

from geometry_msgs.msg import Pose, Transform


def _normalized_quaternion(values: tuple[float, float, float, float]):
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        raise ValueError("四元数为零")
    return tuple(value / norm for value in values)


def _quaternion_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    left_x, left_y, left_z, left_w = left
    right_x, right_y, right_z, right_w = right
    return (
        left_w * right_x + left_x * right_w + left_y * right_z - left_z * right_y,
        left_w * right_y - left_x * right_z + left_y * right_w + left_z * right_x,
        left_w * right_z + left_x * right_y - left_y * right_x + left_z * right_w,
        left_w * right_w - left_x * right_x - left_y * right_y - left_z * right_z,
    )


def _quaternion_conjugate(
    value: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    return -value[0], -value[1], -value[2], value[3]


def _rotate_vector(
    orientation: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    vector_quaternion = vector[0], vector[1], vector[2], 0.0
    rotated = _quaternion_multiply(
        _quaternion_multiply(orientation, vector_quaternion),
        _quaternion_conjugate(orientation),
    )
    return rotated[0], rotated[1], rotated[2]


def _compose(
    left: tuple[tuple[float, float, float], tuple[float, float, float, float]],
    right: tuple[tuple[float, float, float], tuple[float, float, float, float]],
):
    rotated_translation = _rotate_vector(left[1], right[0])
    return (
        tuple(left[0][index] + rotated_translation[index] for index in range(3)),
        _normalized_quaternion(_quaternion_multiply(left[1], right[1])),
    )


def _inverse(
    value: tuple[tuple[float, float, float], tuple[float, float, float, float]],
):
    inverse_orientation = _quaternion_conjugate(value[1])
    inverse_translation = _rotate_vector(
        inverse_orientation,
        tuple(-component for component in value[0]),
    )
    return inverse_translation, inverse_orientation


def pose_at_zero_turn(
    pose_in_base_link: Pose,
    base_to_current_turn: Transform,
    current_turn_rad: float,
) -> Pose:
    """Map a base_link pose measured at the current Turn angle into the Turn=0 model."""
    if not math.isfinite(current_turn_rad):
        raise ValueError("当前 Turn 角度不是有限值")
    current_transform = (
        (
            float(base_to_current_turn.translation.x),
            float(base_to_current_turn.translation.y),
            float(base_to_current_turn.translation.z),
        ),
        _normalized_quaternion(
            (
                float(base_to_current_turn.rotation.x),
                float(base_to_current_turn.rotation.y),
                float(base_to_current_turn.rotation.z),
                float(base_to_current_turn.rotation.w),
            )
        ),
    )
    half_angle = -0.5 * current_turn_rad
    undo_turn = (
        (0.0, 0.0, 0.0),
        (0.0, 0.0, math.sin(half_angle), math.cos(half_angle)),
    )
    zero_turn_transform = _compose(current_transform, undo_turn)
    correction = _compose(zero_turn_transform, _inverse(current_transform))
    target_transform = (
        (
            float(pose_in_base_link.position.x),
            float(pose_in_base_link.position.y),
            float(pose_in_base_link.position.z),
        ),
        _normalized_quaternion(
            (
                float(pose_in_base_link.orientation.x),
                float(pose_in_base_link.orientation.y),
                float(pose_in_base_link.orientation.z),
                float(pose_in_base_link.orientation.w),
            )
        ),
    )
    corrected_translation, corrected_orientation = _compose(correction, target_transform)
    corrected = copy.deepcopy(pose_in_base_link)
    corrected.position.x, corrected.position.y, corrected.position.z = corrected_translation
    (
        corrected.orientation.x,
        corrected.orientation.y,
        corrected.orientation.z,
        corrected.orientation.w,
    ) = corrected_orientation
    return corrected


def compensate_pose_y(pose: Pose, offset_m: float) -> Pose:
    if not math.isfinite(offset_m):
        raise ValueError("Y 补偿不是有限值")
    corrected = copy.deepcopy(pose)
    corrected.position.y = float(corrected.position.y) + float(offset_m)
    return corrected
