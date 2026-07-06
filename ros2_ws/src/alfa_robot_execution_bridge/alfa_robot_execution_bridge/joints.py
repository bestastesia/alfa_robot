"""Joint naming and real-machine direction contract for ALFA execution.

This module is the runtime single source of truth for:

- execution-layer joint names
- real EtherCAT controller joint order
- ROS/Rerun semantics -> real EtherCAT command direction signs

Do not duplicate the sign table in launch files, demo scripts, or YAML configs.
Import from here instead.
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
    'right_joint4': 1.0,
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
