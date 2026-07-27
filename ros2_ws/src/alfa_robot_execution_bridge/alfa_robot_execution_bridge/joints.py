"""Joint naming and real-machine direction/updown contract for ALFA execution.

This module is the runtime single source of truth for:

- execution-layer joint names
- real EtherCAT controller joint order
- ROS/Rerun semantics -> real EtherCAT command direction signs
- real EtherCAT zero offsets -> ROS/Rerun zero semantics
- ROS/Rerun updown logical meters -> real updown controller physical meters

Do not duplicate the sign/offset tables or the updown offset in launch files, demo
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

# Raw EtherCAT command/feedback value that corresponds to ROS/URDF zero.
# 2026-07-22 calibration: the last motor drivers cannot be mechanically
# zeroed, so J6 zero is compensated here at the runtime contract boundary.
ETHERCAT_ZERO_OFFSET_BY_JOINT = {
    'left_joint1': 0.0,
    'left_joint2': 0.0,
    'left_joint3': 0.0,
    'left_joint4': 0.0,
    'left_joint5': 0.0,
    'left_joint6': 0.05235987755982989,  # +3 deg
    'right_joint1': 0.0,
    'right_joint2': 0.0,
    'right_joint3': 0.0,
    'right_joint4': 0.0,
    'right_joint5': 0.0,
    'right_joint6': -0.03490658503988659,  # -2 deg
    'turn': 0.0,
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


def ethercat_zero_offset_for(joint_name: str) -> float:
    try:
        return ETHERCAT_ZERO_OFFSET_BY_JOINT[joint_name]
    except KeyError as exc:
        raise KeyError(f'unknown ALFA execution joint: {joint_name}') from exc


def direction_signs_for(joint_names: Iterable[str]) -> list[float]:
    return [direction_sign_for(name) for name in joint_names]


def ros_to_ethercat_position(joint_name: str, value: float) -> float:
    return float(value) * direction_sign_for(joint_name) + ethercat_zero_offset_for(joint_name)


def ros_to_ethercat_velocity(joint_name: str, value: float) -> float:
    return float(value) * direction_sign_for(joint_name)


def ethercat_to_ros_position(joint_name: str, value: float) -> float:
    return (float(value) - ethercat_zero_offset_for(joint_name)) * direction_sign_for(joint_name)


# updown (lift) axis: logical meters (ROS/MoveIt/Rerun/URDF 'updown' prismatic
# joint semantics, what IK 规划出来的值) vs physical meters (raw value sent to
# /canopen/updown_position_controller/commands，电机侧原始位置).
#
# 2026-07-19 实机复测确认：电机位置与 URDF/MoveIt 位置无需零点平移，即
#   logical(URDF) = physical(电机) + UPDOWN_PHYSICAL_ZERO_OFFSET_M
#   physical(电机) = logical(URDF) - UPDOWN_PHYSICAL_ZERO_OFFSET_M
# 且 UPDOWN_PHYSICAL_ZERO_OFFSET_M = 0，因此 logical == physical。
#
# 逻辑/URDF 规划范围 [0, 0.7] 与电机物理行程 [0, 0.7] 精确一一对应：
# MoveIt 的 urdf 'updown' limit 与 joint_limits.yaml、IK 的 h 采样范围都已统一为
# [0, 0.7]。转换函数仍是强制合同边界，不能绕过；clamp 仍作为最后一道硬防线。
UPDOWN_PHYSICAL_ZERO_OFFSET_M = 0.0
# 电机侧物理行程硬限位（不可超出）：
UPDOWN_PHYSICAL_LOWER_M = 0.0
UPDOWN_PHYSICAL_UPPER_M = 0.7
# MoveIt/URDF 规划的 logical 范围（与 urdf 'updown' limit、joint_limits.yaml 保持一致）：
UPDOWN_LOGICAL_LOWER_M = 0.0
UPDOWN_LOGICAL_UPPER_M = 0.7


def clamp_updown_physical(physical_m: float) -> float:
    """把电机侧目标位置强制夹紧到物理硬行程 [0, 0.7]，杜绝下发超程指令。"""
    return max(UPDOWN_PHYSICAL_LOWER_M, min(UPDOWN_PHYSICAL_UPPER_M, float(physical_m)))


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
    """URDF/MoveIt 逻辑值 -> 电机侧物理指令值。当前零偏移，仍统一经合同转换并强制夹紧到
    电机物理行程 [0, 0.7]（超出部分被安全裁剪，绝不下发超程指令）。"""
    physical_m = float(logical_m) - UPDOWN_PHYSICAL_ZERO_OFFSET_M
    return clamp_updown_physical(physical_m)


def physical_to_logical_updown(physical_m: float) -> float:
    """电机侧物理反馈值 -> URDF/MoveIt 逻辑值。当前零偏移，logical == physical。"""
    return float(physical_m) + UPDOWN_PHYSICAL_ZERO_OFFSET_M
