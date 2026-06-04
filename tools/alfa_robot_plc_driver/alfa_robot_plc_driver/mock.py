"""In-memory MB_CMD/MB_STS mock for tests and dry development."""

from __future__ import annotations

from .codec import decode_dint_x100, encode_dint_x100
from .config import PlcProtocolConfig
from .register_map import AxisCmdOffset, AxisStsOffset, ControlWord


class MockPlcTransport:
    def __init__(self, protocol: PlcProtocolConfig | None = None, active_axes: int = 6):
        self.protocol = protocol or PlcProtocolConfig(default_active_axes=active_axes)
        self.active_axes = active_axes
        self.registers: dict[int, int] = {}
        sys_values = [
            6,
            active_axes,
            30,
            self.protocol.axis_block_words,
            self.protocol.position_scale,
            self.protocol.speed_scale,
            self.protocol.cmd_base,
            self.protocol.sts_base,
            self.protocol.sys_base,
            0,
        ]
        for offset, value in enumerate(sys_values):
            self.registers[self.protocol.sys_base + offset] = value
        for axis in range(1, self.protocol.max_axes + 1):
            cmd_base = self.protocol.cmd_axis_base(axis)
            sts_base = self.protocol.sts_axis_base(axis)
            for offset in range(self.protocol.axis_block_words):
                self.registers.setdefault(cmd_base + offset, 0)
                self.registers.setdefault(sts_base + offset, 0)

    def read_holding_registers(self, address: int, count: int) -> list[int]:
        return [self.registers.get(address + offset, 0) for offset in range(count)]

    def write_register(self, address: int, value: int) -> None:
        self.registers[address] = value & 0xFFFF
        self._maybe_execute_command(address)

    def write_registers(self, address: int, values: list[int] | tuple[int, ...]) -> None:
        for offset, value in enumerate(values):
            self.registers[address + offset] = value & 0xFFFF
        for offset in range(len(values)):
            self._maybe_execute_command(address + offset)

    def _maybe_execute_command(self, address: int) -> None:
        for axis in range(1, self.active_axes + 1):
            cmd_base = self.protocol.cmd_axis_base(axis)
            if address != cmd_base + AxisCmdOffset.CONTROL_WORD:
                continue
            control_word = self.registers.get(cmd_base + AxisCmdOffset.CONTROL_WORD, 0)
            if control_word & ControlWord.MOVE_ABS:
                self._execute_move_abs(axis)

    def _execute_move_abs(self, axis: int) -> None:
        cmd_base = self.protocol.cmd_axis_base(axis)
        sts_base = self.protocol.sts_axis_base(axis)
        low = self.registers.get(cmd_base + AxisCmdOffset.TARGET_SINGLE_POS_LOW, 0)
        high = self.registers.get(cmd_base + AxisCmdOffset.TARGET_SINGLE_POS_HIGH, 0)
        target = decode_dint_x100(low, high)
        feedback_low, feedback_high = encode_dint_x100(target)
        command_id = self.registers.get(cmd_base + AxisCmdOffset.COMMAND_ID, 0)
        self.registers[sts_base + AxisStsOffset.ACK_COMMAND_ID] = command_id
        self.registers[sts_base + AxisStsOffset.ERROR_CODE] = 0
        self.registers[sts_base + AxisStsOffset.FEEDBACK_POS_LOW] = feedback_low
        self.registers[sts_base + AxisStsOffset.FEEDBACK_POS_HIGH] = feedback_high
        self.registers[sts_base + AxisStsOffset.FEEDBACK_SINGLE_LOW] = feedback_low
        self.registers[sts_base + AxisStsOffset.FEEDBACK_SINGLE_HIGH] = feedback_high
        self.registers[sts_base + AxisStsOffset.LAST_TARGET_LOW] = feedback_low
        self.registers[sts_base + AxisStsOffset.LAST_TARGET_HIGH] = feedback_high
