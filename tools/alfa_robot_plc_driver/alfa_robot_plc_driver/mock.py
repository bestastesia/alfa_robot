from __future__ import annotations

from dataclasses import dataclass

from .codec import WordOrder, decode_lreal, decode_udint, encode_lreal, encode_udint
from .config import DriverConfig
from .transport import MockTransport


@dataclass
class MockAxisRuntime:
    move_cmd_id: int = 0
    return_zero_cmd_id: int = 0


class MockPlcRuntime:
    def __init__(self, config: DriverConfig, transport: MockTransport):
        self.config = config
        self.transport = transport
        self.axes = {axis_id: MockAxisRuntime() for axis_id in config.axes}
        self.lreal_word_order = config.encoding.lreal_word_order or WordOrder.BIG
        self.udint_word_order = config.encoding.udint_word_order or WordOrder.BIG

    def initialize_ready_axes(self) -> None:
        for axis in self.config.axes.values():
            self._write_coil(axis, "power_status", True)
            self._write_coil(axis, "emergency_latched", False)
            self._write_register(axis, "active_cmd_type", 0)
            self._write_lreal(axis, "feedback_pos_deg", 0.0)
            self._write_lreal(axis, "target_input_deg", 999999.0)
            for field in (
                "move_cmd_id",
                "move_cmd_done_id",
                "move_cmd_error_id",
                "return_zero_ack_id",
                "return_zero_done_id",
                "return_zero_error_id",
            ):
                self._write_udint(axis, field, 0)

    def process_once(self) -> None:
        for axis_id, axis in self.config.axes.items():
            target_address = axis.registers.get("target_input_deg")
            if target_address is not None:
                target = self._read_lreal(axis, "target_input_deg")
                if target != 999999.0:
                    runtime = self.axes[axis_id]
                    runtime.move_cmd_id = (runtime.move_cmd_id + 1) & 0xFFFFFFFF
                    self._write_udint(axis, "move_cmd_id", runtime.move_cmd_id)
                    self._write_udint(axis, "move_cmd_done_id", runtime.move_cmd_id)
                    self._write_lreal(axis, "feedback_pos_deg", target)
                    self._write_lreal(axis, "target_input_deg", 999999.0)

            cmd_address = axis.registers.get("return_zero_cmd_id")
            if cmd_address is not None:
                command_id = self._read_udint(axis, "return_zero_cmd_id")
                runtime = self.axes[axis_id]
                if command_id != 0 and command_id != runtime.return_zero_cmd_id:
                    runtime.return_zero_cmd_id = command_id
                    self._write_udint(axis, "return_zero_ack_id", command_id)
                    self._write_udint(axis, "return_zero_done_id", command_id)
                    self._write_lreal(axis, "feedback_pos_deg", 0.0)

    def _write_coil(self, axis, name: str, value: bool) -> None:
        address = axis.coils.get(name)
        if address is not None:
            self.transport.coils[int(address)] = bool(value)

    def _write_register(self, axis, name: str, value: int) -> None:
        address = axis.registers.get(name)
        if address is not None:
            self.transport.registers[int(address)] = int(value) & 0xFFFF

    def _write_udint(self, axis, name: str, value: int) -> None:
        address = axis.registers.get(name)
        if address is not None:
            for offset, register in enumerate(encode_udint(value, self.udint_word_order)):
                self.transport.registers[int(address) + offset] = register

    def _read_udint(self, axis, name: str) -> int:
        address = int(axis.registers[name])
        return decode_udint([self.transport.registers.get(address + offset, 0) for offset in range(2)], self.udint_word_order)

    def _write_lreal(self, axis, name: str, value: float) -> None:
        address = axis.registers.get(name)
        if address is not None:
            for offset, register in enumerate(encode_lreal(value, self.lreal_word_order)):
                self.transport.registers[int(address) + offset] = register

    def _read_lreal(self, axis, name: str) -> float:
        address = int(axis.registers[name])
        return decode_lreal([self.transport.registers.get(address + offset, 0) for offset in range(4)], self.lreal_word_order)
