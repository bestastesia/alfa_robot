"""Minimal Modbus TCP transport for holding-register access."""

from __future__ import annotations

import socket
import struct
from typing import Protocol

from .config import PlcConnectionConfig


class HoldingRegisterTransport(Protocol):
    def read_holding_registers(self, address: int, count: int) -> list[int]: ...
    def write_register(self, address: int, value: int) -> None: ...
    def write_registers(self, address: int, values: list[int] | tuple[int, ...]) -> None: ...


class ModbusTcpTransport:
    def __init__(self, config: PlcConnectionConfig):
        self.config = config
        self._transaction_id = 0
        self._socket: socket.socket | None = None

    def open(self) -> None:
        if self._socket is not None:
            return
        self._socket = socket.create_connection((self.config.host, self.config.port), timeout=self.config.timeout_s)
        self._socket.settimeout(self.config.timeout_s)

    def close(self) -> None:
        if self._socket is None:
            return
        self._socket.close()
        self._socket = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def read_holding_registers(self, address: int, count: int) -> list[int]:
        if count < 1:
            raise ValueError("count must be positive")
        response = self._request(struct.pack(">BHH", 3, address, count))
        byte_count = response[1]
        expected = count * 2
        if byte_count != expected:
            raise RuntimeError(f"unexpected Modbus byte count: got {byte_count}, expected {expected}")
        return list(struct.unpack(">" + "H" * count, response[2:2 + byte_count]))

    def write_register(self, address: int, value: int) -> None:
        self._request(struct.pack(">BHH", 6, address, value & 0xFFFF))

    def write_registers(self, address: int, values: list[int] | tuple[int, ...]) -> None:
        if not values:
            raise ValueError("values must not be empty")
        words = [value & 0xFFFF for value in values]
        pdu = struct.pack(">BHHB", 16, address, len(words), len(words) * 2)
        pdu += struct.pack(">" + "H" * len(words), *words)
        self._request(pdu)

    def _request(self, pdu: bytes) -> bytes:
        self._transaction_id = (self._transaction_id + 1) & 0xFFFF
        if self._transaction_id == 0:
            self._transaction_id = 1
        packet = struct.pack(
            ">HHHB",
            self._transaction_id,
            0,
            len(pdu) + 1,
            self.config.unit_id,
        ) + pdu
        if self._socket is None:
            with socket.create_connection((self.config.host, self.config.port), timeout=self.config.timeout_s) as sock:
                sock.settimeout(self.config.timeout_s)
                sock.sendall(packet)
                response = sock.recv(2048)
        else:
            self._socket.sendall(packet)
            response = self._socket.recv(2048)
        if len(response) < 8:
            raise RuntimeError(f"short Modbus response: {response.hex()}")
        function = response[7]
        if function & 0x80:
            code = response[8] if len(response) > 8 else None
            raise RuntimeError(f"Modbus exception code={code} raw={response.hex()}")
        return response[7:]
