from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum

from .codec import WordOrder, decode_lreal, decode_udint, encode_lreal, encode_udint
from .config import AxisConfig
from .transport import ModbusTransport


class StopPolicy(str, Enum):
    STOP = "stop"
    EMERGENCY = "emergency"
    NONE = "none"


@dataclass(frozen=True)
class AxisStatus:
    axis_id: int
    joint_name: str
    power_status: bool
    emergency_latched: bool
    active_cmd_type: int
    move_cmd_id: int
    move_cmd_done_id: int
    move_cmd_error_id: int
    return_zero_ack_id: int
    return_zero_done_id: int
    return_zero_error_id: int
    feedback_pos_deg: float


@dataclass(frozen=True)
class AxisMoveResult:
    axis_id: int
    command_id: int | None
    ok: bool
    reason: str


class DryRunTransport:
    def __init__(self):
        self.operations: list[str] = []

    def connect(self) -> None:
        self.operations.append("connect")

    def close(self) -> None:
        self.operations.append("close")

    def write_coil(self, address: int, value: bool) -> None:
        self.operations.append(f"write_coil address={address} value={value}")

    def read_coils(self, address: int, count: int) -> list[bool]:
        self.operations.append(f"read_coils address={address} count={count}")
        return [False] * count

    def write_registers(self, address: int, values: list[int]) -> None:
        self.operations.append(f"write_registers address={address} values={values}")

    def read_registers(self, address: int, count: int) -> list[int]:
        self.operations.append(f"read_registers address={address} count={count}")
        return [0] * count


class PlcAxisClient:
    def __init__(
        self,
        axis_id: int,
        config: AxisConfig,
        transport: ModbusTransport,
        lreal_word_order: WordOrder,
        udint_word_order: WordOrder,
        tick=None,
    ):
        self.axis_id = axis_id
        self.config = config
        self.transport = transport
        self.lreal_word_order = lreal_word_order
        self.udint_word_order = udint_word_order
        self.tick = tick or (lambda: None)

    def pulse(self, coil_name: str) -> None:
        address = self._coil(coil_name)
        self.transport.write_coil(address, True)

    def enable(self) -> None:
        self.pulse("enable_cmd")

    def disable(self) -> None:
        self.pulse("disable_cmd")

    def stop(self) -> None:
        self.pulse("stop_execute")

    def emergency_stop(self) -> None:
        self.pulse("emergency_stop")

    def reset_fault(self) -> None:
        self.pulse("reset_fault")

    def reset_emergency(self) -> None:
        self.pulse("reset_emergency")

    def set_zero(self) -> None:
        self.pulse("set_zero")

    def write_target_deg(self, target_deg: float) -> None:
        self._write_lreal("target_input_deg", target_deg)

    def write_return_zero_cmd_id(self, command_id: int) -> None:
        self._write_udint("return_zero_cmd_id", command_id)

    def read_status(self) -> AxisStatus:
        return AxisStatus(
            axis_id=self.axis_id,
            joint_name=self.config.joint_name,
            power_status=self._read_bool("power_status"),
            emergency_latched=self._read_bool("emergency_latched"),
            active_cmd_type=self._read_int("active_cmd_type"),
            move_cmd_id=self._read_udint("move_cmd_id"),
            move_cmd_done_id=self._read_udint("move_cmd_done_id"),
            move_cmd_error_id=self._read_udint("move_cmd_error_id"),
            return_zero_ack_id=self._read_udint("return_zero_ack_id"),
            return_zero_done_id=self._read_udint("return_zero_done_id"),
            return_zero_error_id=self._read_udint("return_zero_error_id"),
            feedback_pos_deg=self._read_lreal("feedback_pos_deg"),
        )

    def move_to_deg(self, target_deg: float, accept_timeout_s: float = 1.0, done_timeout_s: float = 10.0) -> AxisMoveResult:
        before = self.read_status()
        if not before.power_status:
            return AxisMoveResult(self.axis_id, None, False, "axis_not_powered")
        if before.emergency_latched:
            return AxisMoveResult(self.axis_id, None, False, "emergency_latched")
        if before.active_cmd_type != 0:
            return AxisMoveResult(self.axis_id, None, False, "axis_busy")

        self.write_target_deg(target_deg)
        self.tick()
        accepted = self._wait_for(lambda status: status.move_cmd_id != before.move_cmd_id, accept_timeout_s)
        if accepted is None:
            return AxisMoveResult(self.axis_id, None, False, "move_not_accepted")

        command_id = accepted.move_cmd_id
        finished = self._wait_for(
            lambda status: status.move_cmd_done_id == command_id or status.move_cmd_error_id == command_id,
            done_timeout_s,
        )
        if finished is None:
            return AxisMoveResult(self.axis_id, command_id, False, "move_timeout")
        if finished.move_cmd_done_id == command_id:
            return AxisMoveResult(self.axis_id, command_id, True, "done")
        return AxisMoveResult(self.axis_id, command_id, False, "move_error")

    def return_zero(self, done_timeout_s: float = 10.0) -> AxisMoveResult:
        current = self.read_status()
        command_id = (max(current.return_zero_ack_id, current.return_zero_done_id, current.return_zero_error_id) + 1) & 0xFFFFFFFF
        self.write_return_zero_cmd_id(command_id)
        self.tick()

        accepted = self._wait_for(lambda status: status.return_zero_ack_id == command_id, 1.0)
        if accepted is None:
            return AxisMoveResult(self.axis_id, command_id, False, "return_zero_not_accepted")

        finished = self._wait_for(
            lambda status: status.return_zero_done_id == command_id or status.return_zero_error_id == command_id,
            done_timeout_s,
        )
        if finished is None:
            return AxisMoveResult(self.axis_id, command_id, False, "return_zero_timeout")
        if finished.return_zero_done_id == command_id:
            return AxisMoveResult(self.axis_id, command_id, True, "done")
        return AxisMoveResult(self.axis_id, command_id, False, "return_zero_error")

    def _wait_for(self, predicate, timeout_s: float, period_s: float = 0.02) -> AxisStatus | None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            status = self.read_status()
            if predicate(status):
                return status
            time.sleep(period_s)
        return None

    def _coil(self, name: str) -> int:
        address = self.config.coils.get(name)
        if address is None:
            raise ValueError(f"{self.config.name}.coils.{name} is unresolved")
        return int(address)

    def _register(self, name: str) -> int:
        address = self.config.registers.get(name)
        if address is None:
            raise ValueError(f"{self.config.name}.registers.{name} is unresolved")
        return int(address)

    def _read_bool(self, name: str) -> bool:
        return bool(self.transport.read_coils(self._coil(name), 1)[0])

    def _read_int(self, name: str) -> int:
        return self.transport.read_registers(self._register(name), 1)[0]

    def _read_udint(self, name: str) -> int:
        return decode_udint(self.transport.read_registers(self._register(name), 2), self.udint_word_order)

    def _write_udint(self, name: str, value: int) -> None:
        self.transport.write_registers(self._register(name), encode_udint(value, self.udint_word_order))

    def _read_lreal(self, name: str) -> float:
        return decode_lreal(self.transport.read_registers(self._register(name), 4), self.lreal_word_order)

    def _write_lreal(self, name: str, value: float) -> None:
        self.transport.write_registers(self._register(name), encode_lreal(value, self.lreal_word_order))
