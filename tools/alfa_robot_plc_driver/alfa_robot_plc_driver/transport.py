from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

from .config import ConnectionConfig


class ModbusTransport(Protocol):
    def connect(self) -> None: ...
    def close(self) -> None: ...
    def write_coil(self, address: int, value: bool) -> None: ...
    def read_coils(self, address: int, count: int) -> list[bool]: ...
    def write_registers(self, address: int, values: list[int]) -> None: ...
    def read_registers(self, address: int, count: int) -> list[int]: ...


class PymodbusTransport:
    def __init__(self, config: ConnectionConfig):
        self._config = config
        self._client = None

    def connect(self) -> None:
        from pymodbus.client import ModbusTcpClient

        self._client = ModbusTcpClient(self._config.host, port=self._config.port, timeout=self._config.timeout_s)
        if not self._client.connect():
            raise ConnectionError(f"cannot connect to PLC {self._config.host}:{self._config.port}")

    def close(self) -> None:
        if self._client is not None:
            self._client.close()
            self._client = None

    def _call(self, method_name: str, **kwargs):
        if self._client is None:
            raise RuntimeError("transport is not connected")
        method = getattr(self._client, method_name)
        last_error = None
        for unit_kw in ("slave", "unit", "device_id"):
            try:
                result = method(**kwargs, **{unit_kw: self._config.slave_id})
                break
            except TypeError as exc:
                text = str(exc)
                if unit_kw in text or "unexpected keyword argument" in text:
                    last_error = exc
                    continue
                raise
        else:
            try:
                result = method(**kwargs)
            except TypeError:
                if last_error is not None:
                    raise last_error
                raise
        if result is None:
            raise IOError(f"{method_name} failed: no response")
        if hasattr(result, "isError") and result.isError():
            raise IOError(f"{method_name} failed: {result}")
        return result

    def write_coil(self, address: int, value: bool) -> None:
        self._call("write_coil", address=address, value=bool(value))

    def read_coils(self, address: int, count: int) -> list[bool]:
        result = self._call("read_coils", address=address, count=count)
        return list(result.bits[:count])

    def write_registers(self, address: int, values: list[int]) -> None:
        self._call("write_registers", address=address, values=[int(value) & 0xFFFF for value in values])

    def read_registers(self, address: int, count: int) -> list[int]:
        result = self._call("read_holding_registers", address=address, count=count)
        return list(result.registers)


@dataclass
class MockTransport:
    coils: dict[int, bool] = field(default_factory=dict)
    registers: dict[int, int] = field(default_factory=dict)
    connected: bool = False

    def connect(self) -> None:
        self.connected = True

    def close(self) -> None:
        self.connected = False

    def write_coil(self, address: int, value: bool) -> None:
        self._require_connected()
        self.coils[int(address)] = bool(value)

    def read_coils(self, address: int, count: int) -> list[bool]:
        self._require_connected()
        return [bool(self.coils.get(int(address) + offset, False)) for offset in range(count)]

    def write_registers(self, address: int, values: list[int]) -> None:
        self._require_connected()
        for offset, value in enumerate(values):
            self.registers[int(address) + offset] = int(value) & 0xFFFF

    def read_registers(self, address: int, count: int) -> list[int]:
        self._require_connected()
        return [int(self.registers.get(int(address) + offset, 0)) for offset in range(count)]

    def _require_connected(self) -> None:
        if not self.connected:
            raise RuntimeError("mock transport is not connected")
