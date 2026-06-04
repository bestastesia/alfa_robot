"""In-memory MB_CMD/MB_STS virtual PLC for tests and dry development."""

from __future__ import annotations

import time

from .codec import decode_dint_x100, encode_dint_x100, encode_word_x100
from .config import PlcProtocolConfig
from .register_map import AxisCmdOffset, AxisStsOffset, ControlWord


class MockPlcTransport:
    """A deterministic virtual PLC with the same MB_CMD/MB_STS register shape.

    It accepts the same holding-register writes as the real PLC. MoveAbs commands
    update AckCommandID/LastTarget immediately, while feedback position advances
    toward the latest target at the configured velocity limit whenever the mock
    is read or written.
    """

    def __init__(self, protocol: PlcProtocolConfig | None = None, active_axes: int = 6):
        self.protocol = protocol or PlcProtocolConfig(default_active_axes=active_axes)
        self.active_axes = active_axes
        self.registers: dict[int, int] = {}
        self._axis_targets: dict[int, float] = {axis: 0.0 for axis in range(1, active_axes + 1)}
        self._axis_feedback: dict[int, float] = {axis: 0.0 for axis in range(1, active_axes + 1)}
        self._last_update_s = time.monotonic()
        self._emergency_active = False
        sys_values = [
            12,
            active_axes,
            self.protocol.max_axes,
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
            self._write_axis_feedback(axis, 0.0)
            self._write_axis_last_target(axis, 0.0)

    def open(self) -> None:
        self._update_motion()

    def close(self) -> None:
        self._update_motion()

    def read_holding_registers(self, address: int, count: int) -> list[int]:
        self._update_motion()
        return [self.registers.get(address + offset, 0) for offset in range(count)]

    def write_register(self, address: int, value: int) -> None:
        self._update_motion()
        self.registers[address] = value & 0xFFFF
        self._maybe_execute_command(address)

    def write_registers(self, address: int, values: list[int] | tuple[int, ...]) -> None:
        self._update_motion()
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
            sts_base = self.protocol.sts_axis_base(axis)
            self.registers[sts_base + AxisStsOffset.STATUS_WORD] = 0
            self.registers[sts_base + AxisStsOffset.ERROR_CODE] = 0
            if control_word & ControlWord.EMERGENCY_STOP:
                self._emergency_active = True
                self.registers[sts_base + AxisStsOffset.STATUS_WORD] = ControlWord.EMERGENCY_STOP
                self._axis_targets[axis] = self._axis_feedback[axis]
                self._write_axis_last_target(axis, self._axis_feedback[axis])
                continue
            if control_word & ControlWord.RESET_EMERGENCY:
                self._emergency_active = False
                self.registers[sts_base + AxisStsOffset.STATUS_WORD] = 0
                continue
            if control_word & ControlWord.STOP:
                self._axis_targets[axis] = self._axis_feedback[axis]
                self._write_axis_last_target(axis, self._axis_feedback[axis])
                continue
            if control_word & ControlWord.RESET_FAULT:
                self.registers[sts_base + AxisStsOffset.ERROR_CODE] = 0
                continue
            if control_word & ControlWord.MOVE_ABS and not self._emergency_active:
                self._accept_move_abs(axis)

    def _accept_move_abs(self, axis: int) -> None:
        cmd_base = self.protocol.cmd_axis_base(axis)
        sts_base = self.protocol.sts_axis_base(axis)
        low = self.registers.get(cmd_base + AxisCmdOffset.TARGET_SINGLE_POS_LOW, 0)
        high = self.registers.get(cmd_base + AxisCmdOffset.TARGET_SINGLE_POS_HIGH, 0)
        target = decode_dint_x100(low, high)
        command_id = self.registers.get(cmd_base + AxisCmdOffset.COMMAND_ID, 0)
        self._axis_targets[axis] = target
        self.registers[sts_base + AxisStsOffset.ACK_COMMAND_ID] = command_id
        self.registers[sts_base + AxisStsOffset.ERROR_CODE] = 0
        self._write_axis_last_target(axis, target)

    def _update_motion(self) -> None:
        now = time.monotonic()
        elapsed_s = max(0.0, now - self._last_update_s)
        self._last_update_s = now
        if elapsed_s == 0:
            return
        for axis in range(1, self.active_axes + 1):
            current = self._axis_feedback[axis]
            target = self._axis_targets[axis]
            delta = target - current
            if abs(delta) <= 1e-9:
                self._write_axis_feedback(axis, target)
                continue
            velocity = self._axis_velocity(axis)
            max_step = velocity * elapsed_s
            if abs(delta) <= max_step:
                current = target
            else:
                current += max_step if delta > 0 else -max_step
            self._axis_feedback[axis] = current
            self._write_axis_feedback(axis, current)

    def _axis_velocity(self, axis: int) -> float:
        cmd_base = self.protocol.cmd_axis_base(axis)
        raw_velocity = self.registers.get(cmd_base + AxisCmdOffset.VELOCITY, 0)
        return max(1.0, raw_velocity / self.protocol.speed_scale if raw_velocity else 50.0)

    def _write_axis_feedback(self, axis: int, value: float) -> None:
        sts_base = self.protocol.sts_axis_base(axis)
        feedback_low, feedback_high = encode_dint_x100(value)
        self.registers[sts_base + AxisStsOffset.FEEDBACK_POS_LOW] = feedback_low
        self.registers[sts_base + AxisStsOffset.FEEDBACK_POS_HIGH] = feedback_high
        self.registers[sts_base + AxisStsOffset.FEEDBACK_SINGLE_LOW] = feedback_low
        self.registers[sts_base + AxisStsOffset.FEEDBACK_SINGLE_HIGH] = feedback_high

    def _write_axis_last_target(self, axis: int, value: float) -> None:
        sts_base = self.protocol.sts_axis_base(axis)
        target_low, target_high = encode_dint_x100(value)
        self.registers[sts_base + AxisStsOffset.LAST_TARGET_LOW] = target_low
        self.registers[sts_base + AxisStsOffset.LAST_TARGET_HIGH] = target_high

    def set_axis_feedback(self, axis: int, position_deg: float) -> None:
        self.protocol.validate_axis_number(axis)
        self._axis_feedback[axis] = position_deg
        self._axis_targets[axis] = position_deg
        self._write_axis_feedback(axis, position_deg)
        self._write_axis_last_target(axis, position_deg)

    def set_axis_error(self, axis: int, error_code: int) -> None:
        self.protocol.validate_axis_number(axis)
        self.registers[self.protocol.sts_axis_base(axis) + AxisStsOffset.ERROR_CODE] = error_code & 0xFFFF
