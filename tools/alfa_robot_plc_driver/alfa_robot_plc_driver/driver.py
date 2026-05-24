from __future__ import annotations

from dataclasses import dataclass

from .axis import AxisMoveResult, PlcAxisClient, StopPolicy
from .codec import WordOrder
from .config import DriverConfig, require_complete_addresses, with_mock_addresses
from .mock import MockPlcRuntime
from .transport import MockTransport, ModbusTransport, PymodbusTransport


@dataclass
class MultiAxisResult:
    results: dict[int, AxisMoveResult]
    stopped_axes: list[int]

    @property
    def ok(self) -> bool:
        return all(result.ok for result in self.results.values())


class PlcDriver:
    def __init__(self, config: DriverConfig, transport: ModbusTransport, mock_runtime: MockPlcRuntime | None = None):
        lreal_word_order = config.encoding.lreal_word_order or WordOrder.BIG
        udint_word_order = config.encoding.udint_word_order or WordOrder.BIG
        self.config = config
        self.transport = transport
        self.mock_runtime = mock_runtime
        tick = mock_runtime.process_once if mock_runtime is not None else None
        self.axes = {
            axis_id: PlcAxisClient(axis_id, axis_config, transport, lreal_word_order, udint_word_order, tick)
            for axis_id, axis_config in config.axes.items()
        }

    @classmethod
    def for_mock(cls, config: DriverConfig) -> "PlcDriver":
        config = with_mock_addresses(config)
        transport = MockTransport()
        transport.connect()
        runtime = MockPlcRuntime(config, transport)
        runtime.initialize_ready_axes()
        return cls(config, transport, runtime)

    @classmethod
    def for_real_plc(cls, config: DriverConfig) -> "PlcDriver":
        require_complete_addresses(config)
        transport = PymodbusTransport(config.connection)
        transport.connect()
        return cls(config, transport)

    def close(self) -> None:
        self.transport.close()

    def process_mock_once(self) -> None:
        if self.mock_runtime is not None:
            self.mock_runtime.process_once()

    def axis(self, axis_id: int) -> PlcAxisClient:
        try:
            return self.axes[int(axis_id)]
        except KeyError as exc:
            raise KeyError(f"unknown axis: {axis_id}") from exc

    def move_many(
        self,
        targets: dict[int, float],
        stop_policy: StopPolicy = StopPolicy.STOP,
        accept_timeout_s: float = 1.0,
        done_timeout_s: float = 10.0,
    ) -> MultiAxisResult:
        for axis_id in targets:
            status = self.axis(axis_id).read_status()
            if not status.power_status:
                result = AxisMoveResult(axis_id, None, False, "axis_not_powered")
                return self._handle_multi_axis_failure({axis_id: result}, list(targets), stop_policy)
            if status.emergency_latched:
                result = AxisMoveResult(axis_id, None, False, "emergency_latched")
                return self._handle_multi_axis_failure({axis_id: result}, list(targets), stop_policy)
            if status.active_cmd_type != 0:
                result = AxisMoveResult(axis_id, None, False, "axis_busy")
                return self._handle_multi_axis_failure({axis_id: result}, list(targets), stop_policy)

        before_ids = {axis_id: self.axis(axis_id).read_status().move_cmd_id for axis_id in targets}
        for axis_id, target_deg in targets.items():
            self.axis(axis_id).write_target_deg(target_deg)
        self.process_mock_once()

        command_ids: dict[int, int] = {}
        for axis_id, before_id in before_ids.items():
            status = self.axis(axis_id)._wait_for(lambda item, old=before_id: item.move_cmd_id != old, accept_timeout_s)
            if status is None:
                result = AxisMoveResult(axis_id, None, False, "move_not_accepted")
                return self._handle_multi_axis_failure({axis_id: result}, list(targets), stop_policy)
            command_ids[axis_id] = status.move_cmd_id

        results: dict[int, AxisMoveResult] = {}
        for axis_id, command_id in command_ids.items():
            status = self.axis(axis_id)._wait_for(
                lambda item, expected=command_id: item.move_cmd_done_id == expected or item.move_cmd_error_id == expected,
                done_timeout_s,
            )
            if status is None:
                results[axis_id] = AxisMoveResult(axis_id, command_id, False, "move_timeout")
            elif status.move_cmd_done_id == command_id:
                results[axis_id] = AxisMoveResult(axis_id, command_id, True, "done")
            else:
                results[axis_id] = AxisMoveResult(axis_id, command_id, False, "move_error")

        if not all(result.ok for result in results.values()):
            return self._handle_multi_axis_failure(results, list(targets), stop_policy)
        return MultiAxisResult(results=results, stopped_axes=[])

    def _handle_multi_axis_failure(
        self,
        results: dict[int, AxisMoveResult],
        involved_axes: list[int],
        stop_policy: StopPolicy,
    ) -> MultiAxisResult:
        stopped_axes: list[int] = []
        if stop_policy != StopPolicy.NONE:
            for axis_id in involved_axes:
                try:
                    if stop_policy == StopPolicy.EMERGENCY:
                        self.axis(axis_id).emergency_stop()
                    else:
                        self.axis(axis_id).stop()
                    stopped_axes.append(axis_id)
                except Exception:
                    pass
        return MultiAxisResult(results=results, stopped_axes=stopped_axes)
