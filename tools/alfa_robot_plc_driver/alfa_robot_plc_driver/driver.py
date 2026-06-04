"""High-level PLC axis driver built on the MB_CMD/MB_STS protocol."""

from __future__ import annotations

import time
from collections.abc import Mapping, Sequence

from .codec import decode_dint_x100, decode_word_x100, encode_dint_x100, encode_word_x100
from .config import PlcConnectionConfig, PlcProtocolConfig
from .models import AxisCommandResult, AxisStatus, PlcSystemInfo
from .register_map import AxisCmdOffset, AxisStsOffset, ControlWord, SysOffset
from .transport import HoldingRegisterTransport, ModbusTcpTransport


class PlcDriver:
    def __init__(
        self,
        connection: PlcConnectionConfig | None = None,
        protocol: PlcProtocolConfig | None = None,
        transport: HoldingRegisterTransport | None = None,
    ):
        self.connection = connection or PlcConnectionConfig()
        self.protocol = protocol or PlcProtocolConfig()
        self.transport = transport or ModbusTcpTransport(self.connection)

    def connect(self) -> PlcSystemInfo:
        return self.read_system()

    def read_system(self) -> PlcSystemInfo:
        words = self.transport.read_holding_registers(self.protocol.sys_base, 10)
        return PlcSystemInfo(
            protocol_mode=words[SysOffset.PROTOCOL_MODE],
            active_axis_count=words[SysOffset.ACTIVE_AXIS_COUNT] or self.protocol.default_active_axes,
            max_axis_capacity=words[SysOffset.MAX_AXIS_CAPACITY] or self.protocol.max_axes,
            axis_block_size=words[SysOffset.AXIS_BLOCK_SIZE] or self.protocol.axis_block_words,
            position_scale=words[SysOffset.POSITION_SCALE] or self.protocol.position_scale,
            speed_scale=words[SysOffset.SPEED_SCALE] or self.protocol.speed_scale,
            cmd_base_mw=words[SysOffset.CMD_BASE_MW],
            sts_base_mw=words[SysOffset.STS_BASE_MW] or self.protocol.sts_base,
            sys_base_mw=words[SysOffset.SYS_BASE_MW] or self.protocol.sys_base,
            heartbeat=words[SysOffset.HEARTBEAT],
            raw_words=tuple(words),
        )

    def active_axis_count(self) -> int:
        return min(self.read_system().active_axis_count, self.protocol.max_axes)

    def read_axis(self, axis: int) -> AxisStatus:
        self._validate_axis_number(axis)
        cmd_base = self.protocol.cmd_axis_base(axis)
        sts_base = self.protocol.sts_axis_base(axis)
        cmd = self.transport.read_holding_registers(cmd_base, self.protocol.axis_block_words)
        sts = self.transport.read_holding_registers(sts_base, self.protocol.axis_block_words)
        return AxisStatus(
            axis=axis,
            cmd_base=cmd_base,
            sts_base=sts_base,
            control_word=cmd[AxisCmdOffset.CONTROL_WORD],
            command_id=cmd[AxisCmdOffset.COMMAND_ID],
            command_target_deg=decode_dint_x100(
                cmd[AxisCmdOffset.TARGET_SINGLE_POS_LOW],
                cmd[AxisCmdOffset.TARGET_SINGLE_POS_HIGH],
            ),
            velocity=decode_word_x100(cmd[AxisCmdOffset.VELOCITY]),
            acceleration=decode_word_x100(cmd[AxisCmdOffset.ACC]),
            deceleration=decode_word_x100(cmd[AxisCmdOffset.DEC]),
            emergency_deceleration=decode_word_x100(cmd[AxisCmdOffset.EMERGENCY_DEC]),
            status_word=sts[AxisStsOffset.STATUS_WORD],
            ack_command_id=sts[AxisStsOffset.ACK_COMMAND_ID],
            error_code=sts[AxisStsOffset.ERROR_CODE],
            feedback_pos_deg=decode_dint_x100(
                sts[AxisStsOffset.FEEDBACK_POS_LOW],
                sts[AxisStsOffset.FEEDBACK_POS_HIGH],
            ),
            feedback_single_deg=decode_dint_x100(
                sts[AxisStsOffset.FEEDBACK_SINGLE_LOW],
                sts[AxisStsOffset.FEEDBACK_SINGLE_HIGH],
            ),
            last_target_deg=decode_dint_x100(
                sts[AxisStsOffset.LAST_TARGET_LOW],
                sts[AxisStsOffset.LAST_TARGET_HIGH],
            ),
            raw_cmd_words=tuple(cmd),
            raw_sts_words=tuple(sts),
        )

    def read_axes(self, axes: Sequence[int] | None = None) -> list[AxisStatus]:
        if axes is None:
            axes = range(1, self.active_axis_count() + 1)
        return [self.read_axis(axis) for axis in axes]

    def move_abs(
        self,
        targets_deg: Mapping[int, float],
        *,
        velocity: float = 3.0,
        acceleration: float = 10.0,
        deceleration: float = 10.0,
        emergency_deceleration: float = 30.0,
        clear_first: bool = True,
        trigger_delay_s: float = 0.0,
    ) -> list[AxisCommandResult]:
        if not targets_deg:
            raise ValueError("targets_deg must not be empty")
        self._validate_active_axes(targets_deg.keys())
        before = {axis: self.read_axis(axis) for axis in targets_deg}
        prepared: list[tuple[int, float, int]] = []
        for axis, target in targets_deg.items():
            cmd_base = self.protocol.cmd_axis_base(axis)
            command_id = self._next_command_id(before[axis])
            target_low, target_high = encode_dint_x100(target)
            if clear_first:
                self.transport.write_register(cmd_base + AxisCmdOffset.CONTROL_WORD, ControlWord.CLEAR)
            self.transport.write_register(cmd_base + AxisCmdOffset.VELOCITY, encode_word_x100(velocity))
            self.transport.write_register(cmd_base + AxisCmdOffset.ACC, encode_word_x100(acceleration))
            self.transport.write_register(cmd_base + AxisCmdOffset.DEC, encode_word_x100(deceleration))
            self.transport.write_register(cmd_base + AxisCmdOffset.EMERGENCY_DEC, encode_word_x100(emergency_deceleration))
            self.transport.write_registers(cmd_base + AxisCmdOffset.TARGET_SINGLE_POS_LOW, [target_low, target_high])
            self.transport.write_register(cmd_base + AxisCmdOffset.COMMAND_ID, command_id)
            prepared.append((axis, target, command_id))
        if trigger_delay_s > 0:
            time.sleep(trigger_delay_s)
        for axis, _, _ in prepared:
            self.transport.write_register(
                self.protocol.cmd_axis_base(axis) + AxisCmdOffset.CONTROL_WORD,
                ControlWord.ENABLE_MOVE_ABS,
            )
        return [
            AxisCommandResult(axis=axis, target_deg=target, command_id=command_id, status=self.read_axis(axis))
            for axis, target, command_id in prepared
        ]

    def prepare_move_abs_stream(
        self,
        axes: Sequence[int],
        *,
        velocity: float = 30.0,
        acceleration: float = 50.0,
        deceleration: float = 50.0,
        emergency_deceleration: float = 80.0,
    ) -> dict[int, int]:
        self._validate_active_axes(axes)
        statuses = {axis: self.read_axis(axis) for axis in axes}
        command_ids: dict[int, int] = {}
        for axis in axes:
            cmd_base = self.protocol.cmd_axis_base(axis)
            command_id = self._next_command_id(statuses[axis])
            self.transport.write_registers(
                cmd_base + AxisCmdOffset.CONTROL_WORD,
                [
                    ControlWord.CLEAR,
                    command_id,
                    statuses[axis].raw_cmd_words[AxisCmdOffset.TARGET_SINGLE_POS_LOW],
                    statuses[axis].raw_cmd_words[AxisCmdOffset.TARGET_SINGLE_POS_HIGH],
                    encode_word_x100(velocity),
                    encode_word_x100(acceleration),
                    encode_word_x100(deceleration),
                    statuses[axis].raw_cmd_words[AxisCmdOffset.STEP_DEG],
                    statuses[axis].raw_cmd_words[AxisCmdOffset.RESERVED_8],
                    encode_word_x100(emergency_deceleration),
                ],
            )
            command_ids[axis] = command_id
        return command_ids

    def stream_move_abs_tick(self, targets_deg: Mapping[int, float], command_ids: Mapping[int, int]) -> None:
        self._validate_active_axes(targets_deg.keys())
        for axis, target in targets_deg.items():
            cmd_base = self.protocol.cmd_axis_base(axis)
            target_low, target_high = encode_dint_x100(target)
            self.transport.write_registers(
                cmd_base + AxisCmdOffset.CONTROL_WORD,
                [ControlWord.ENABLE_MOVE_ABS, command_ids[axis], target_low, target_high],
            )

    def next_command_ids(self, command_ids: Mapping[int, int]) -> dict[int, int]:
        return {axis: 1 if command_id >= 0xFFFF else command_id + 1 for axis, command_id in command_ids.items()}

    def move_delta(self, deltas_deg: Mapping[int, float], **kwargs) -> list[AxisCommandResult]:
        if not deltas_deg:
            raise ValueError("deltas_deg must not be empty")
        self._validate_active_axes(deltas_deg.keys())
        current = {axis: self.read_axis(axis) for axis in deltas_deg}
        targets = {axis: current[axis].feedback_single_deg + delta for axis, delta in deltas_deg.items()}
        return self.move_abs(targets, **kwargs)

    def clear_command(self, axis: int) -> AxisStatus:
        self._validate_active_axes([axis])
        self.transport.write_register(self.protocol.cmd_axis_base(axis) + AxisCmdOffset.CONTROL_WORD, ControlWord.CLEAR)
        return self.read_axis(axis)

    def clear_commands(self, axes: Sequence[int] | None = None) -> list[AxisStatus]:
        if axes is None:
            axes = range(1, self.active_axis_count() + 1)
        return [self.clear_command(axis) for axis in axes]

    def clear_all_commands(self, *, include_inactive: bool = False) -> list[AxisStatus]:
        if include_inactive:
            axes = range(1, self.protocol.max_axes + 1)
            for axis in axes:
                self._validate_axis_number(axis)
                self.transport.write_register(
                    self.protocol.cmd_axis_base(axis) + AxisCmdOffset.CONTROL_WORD,
                    ControlWord.CLEAR,
                )
            return [self.read_axis(axis) for axis in axes]
        return self.clear_commands()

    def write_control_word(self, axes: Sequence[int], control_word: int) -> list[AxisStatus]:
        self._validate_active_axes(axes)
        for axis in axes:
            self.transport.write_register(
                self.protocol.cmd_axis_base(axis) + AxisCmdOffset.CONTROL_WORD,
                control_word,
            )
        return self.read_axes(axes)

    def emergency_stop(self, axes: Sequence[int]) -> list[AxisStatus]:
        return self.write_control_word(axes, ControlWord.ENABLE_EMERGENCY_STOP)

    def reset_emergency(self, axes: Sequence[int]) -> list[AxisStatus]:
        return self.write_control_word(axes, ControlWord.ENABLE_RESET_EMERGENCY)

    def reset_fault(self, axes: Sequence[int]) -> list[AxisStatus]:
        return self.write_control_word(axes, ControlWord.ENABLE_RESET_FAULT)

    def _next_command_id(self, status: AxisStatus) -> int:
        command_id = max(status.command_id, status.ack_command_id) + 1
        return 1 if command_id > 0xFFFF else command_id

    def _validate_axis_number(self, axis: int) -> None:
        self.protocol.validate_axis_number(axis)

    def _validate_active_axes(self, axes) -> None:
        for axis in axes:
            self._validate_axis_number(axis)
        if self.protocol.allow_inactive_axis_writes:
            return
        active_axis_count = self.active_axis_count()
        for axis in axes:
            if axis > active_axis_count:
                raise ValueError(
                    f"Axis{axis} is not active according to PLC ActiveAxisCount={active_axis_count}; refusing write"
                )
