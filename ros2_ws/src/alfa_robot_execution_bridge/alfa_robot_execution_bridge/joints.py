"""Joint naming and real-machine direction/updown contract for ALFA execution.

This module is the runtime single source of truth for:

- execution-layer joint names
- real EtherCAT controller joint order
- ROS/Rerun semantics -> real EtherCAT command direction signs
- ROS/Rerun updown logical meters -> real updown controller physical meters

Do not duplicate the sign table or the updown offset in launch files, demo
scripts, or YAML configs. Import from here instead.
"""

from __future__ import annotations

from typing import Iterable


EXECUTION_JOINT_NAMES = [
    'left_joint1',
    'left_joint2',
    'left_joint3',
    'left_joint4',
    'left_joint5',
    'left_joint6',
    'right_joint1',
    'right_joint2',
    'right_joint3',
    'right_joint4',
    'right_joint5',
    'right_joint6',
    'turn',
]

REAL_CONTROLLER_JOINT_NAMES = [
    'right_joint1',
    'right_joint2',
    'right_joint3',
    'right_joint4',
    'right_joint5',
    'right_joint6',
    'left_joint1',
    'left_joint2',
    'left_joint3',
    'left_joint4',
    'left_joint5',
    'left_joint6',
    'turn',
]

ROS_TO_ETHERCAT_SIGN_BY_JOINT = {
    'left_joint1': 1.0,
    'left_joint2': 1.0,
    'left_joint3': -1.0,
    'left_joint4': 1.0,
    'left_joint5': -1.0,
    'left_joint6': 1.0,
    'right_joint1': 1.0,
    'right_joint2': -1.0,
    'right_joint3': 1.0,
    'right_joint4': -1.0,
    'right_joint5': 1.0,
    'right_joint6': 1.0,
    'turn': 1.0,
}

DEFAULT_JOINT_NAMES = EXECUTION_JOINT_NAMES
DEFAULT_DIRECTION_SIGNS = [
    ROS_TO_ETHERCAT_SIGN_BY_JOINT[name]
    for name in DEFAULT_JOINT_NAMES
]
FLIPPED_JOINT_NAMES = tuple(
    name for name, sign in ROS_TO_ETHERCAT_SIGN_BY_JOINT.items()
    if sign < 0.0
)


def require_matching_lengths(joint_names: Iterable[str], values: Iterable[float], label: str) -> None:
    names = list(joint_names)
    items = list(values)
    if len(names) != len(items):
        raise ValueError(f'{label} length {len(items)} does not match joint count {len(names)}')


def direction_sign_for(joint_name: str) -> float:
    try:
        return ROS_TO_ETHERCAT_SIGN_BY_JOINT[joint_name]
    except KeyError as exc:
        raise KeyError(f'unknown ALFA execution joint: {joint_name}') from exc


def direction_signs_for(joint_names: Iterable[str]) -> list[float]:
    return [direction_sign_for(name) for name in joint_names]


def ros_to_ethercat_position(joint_name: str, value: float) -> float:
    return float(value) * direction_sign_for(joint_name)


def ethercat_to_ros_position(joint_name: str, value: float) -> float:
    return float(value) * direction_sign_for(joint_name)


# updown (lift) axis: logical meters (ROS/MoveIt/Rerun semantics, matches the
# 'updown' URDF prismatic joint and joint_limits.yaml) vs physical meters
# (raw value sent to /canopen/updown_position_controller/commands).
#
# physical = logical + UPDOWN_PHYSICAL_ZERO_OFFSET_M
#
# Source of the 0.08m offset and the 0.0-0.7 logical range:
# ros2_ws/src/alfa_robot_moveit_config/config/motion_baselines/current_motion_baseline.yaml
UPDOWN_LOGICAL_LOWER_M = 0.0
UPDOWN_LOGICAL_UPPER_M = 0.7
UPDOWN_PHYSICAL_ZERO_OFFSET_M = 0.08
UPDOWN_PHYSICAL_LOWER_M = UPDOWN_LOGICAL_LOWER_M + UPDOWN_PHYSICAL_ZERO_OFFSET_M
UPDOWN_PHYSICAL_UPPER_M = UPDOWN_LOGICAL_UPPER_M + UPDOWN_PHYSICAL_ZERO_OFFSET_M


def require_updown_logical_in_range(logical_m: float) -> None:
    if not (UPDOWN_LOGICAL_LOWER_M - 1e-9 <= logical_m <= UPDOWN_LOGICAL_UPPER_M + 1e-9):
        raise ValueError(
            f'updown logical value {logical_m} m is outside the valid range '
            f'[{UPDOWN_LOGICAL_LOWER_M}, {UPDOWN_LOGICAL_UPPER_M}] m'
        )


def require_updown_physical_in_range(physical_m: float) -> None:
    if not (UPDOWN_PHYSICAL_LOWER_M - 1e-9 <= physical_m <= UPDOWN_PHYSICAL_UPPER_M + 1e-9):
        raise ValueError(
            f'updown physical value {physical_m} m is outside the valid range '
            f'[{UPDOWN_PHYSICAL_LOWER_M}, {UPDOWN_PHYSICAL_UPPER_M}] m'
        )


def logical_to_physical_updown(logical_m: float) -> float:
    logical_m = float(logical_m)
    require_updown_logical_in_range(logical_m)
    return logical_m + UPDOWN_PHYSICAL_ZERO_OFFSET_M


def physical_to_logical_updown(physical_m: float) -> float:
    physical_m = float(physical_m)
    require_updown_physical_in_range(physical_m)
    return physical_m - UPDOWN_PHYSICAL_ZERO_OFFSET_M
