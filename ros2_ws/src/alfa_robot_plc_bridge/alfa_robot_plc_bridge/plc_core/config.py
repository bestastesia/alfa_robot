"""Configuration for the ALFA PLC MB_CMD/MB_STS protocol."""

from dataclasses import dataclass


@dataclass(frozen=True)
class PlcConnectionConfig:
    host: str = "192.168.1.88"
    port: int = 502
    unit_id: int = 255
    timeout_s: float = 3.0


@dataclass(frozen=True)
class PlcProtocolConfig:
    cmd_base: int = 0
    sts_base: int = 1000
    sys_base: int = 1960
    axis_block_words: int = 32
    max_axes: int = 12
    default_active_axes: int = 6
    position_scale: int = 100
    speed_scale: int = 100
    allow_inactive_axis_writes: bool = False

    def cmd_axis_base(self, axis: int) -> int:
        self.validate_axis_number(axis)
        return self.cmd_base + (axis - 1) * self.axis_block_words

    def sts_axis_base(self, axis: int) -> int:
        self.validate_axis_number(axis)
        return self.sts_base + (axis - 1) * self.axis_block_words

    def validate_axis_number(self, axis: int) -> None:
        if axis < 1 or axis > self.max_axes:
            raise ValueError(f"axis must be in 1..{self.max_axes}, got {axis}")
