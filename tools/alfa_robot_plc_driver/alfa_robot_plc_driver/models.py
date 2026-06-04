"""Domain models returned by the PLC communication core."""

from dataclasses import dataclass


@dataclass(frozen=True)
class PlcSystemInfo:
    protocol_mode: int
    active_axis_count: int
    max_axis_capacity: int
    axis_block_size: int
    position_scale: int
    speed_scale: int
    cmd_base_mw: int
    sts_base_mw: int
    sys_base_mw: int
    heartbeat: int
    raw_words: tuple[int, ...]


@dataclass(frozen=True)
class AxisStatus:
    axis: int
    cmd_base: int
    sts_base: int
    control_word: int
    command_id: int
    command_target_deg: float
    velocity: float
    acceleration: float
    deceleration: float
    emergency_deceleration: float
    status_word: int
    ack_command_id: int
    error_code: int
    feedback_pos_deg: float
    feedback_single_deg: float
    last_target_deg: float
    raw_cmd_words: tuple[int, ...]
    raw_sts_words: tuple[int, ...]

    @property
    def has_error(self) -> bool:
        return self.error_code != 0


@dataclass(frozen=True)
class AxisCommandResult:
    axis: int
    target_deg: float
    command_id: int
    status: AxisStatus
