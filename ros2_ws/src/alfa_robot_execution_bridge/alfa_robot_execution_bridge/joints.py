"""Joint naming and sign conventions for the ALFA execution bridge."""

from __future__ import annotations

from typing import Iterable


DEFAULT_JOINT_NAMES = [
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

DEFAULT_DIRECTION_SIGNS = [
    1.0,
    1.0,
    -1.0,
    1.0,
    -1.0,
    1.0,
    1.0,
    -1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
]


def require_matching_lengths(joint_names: Iterable[str], values: Iterable[float], label: str) -> None:
    names = list(joint_names)
    items = list(values)
    if len(names) != len(items):
        raise ValueError(f'{label} length {len(items)} does not match joint count {len(names)}')
