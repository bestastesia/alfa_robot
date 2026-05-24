from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .codec import WordOrder


@dataclass(frozen=True)
class ConnectionConfig:
    host: str
    port: int
    slave_id: int
    timeout_s: float


@dataclass(frozen=True)
class EncodingConfig:
    lreal_word_order: WordOrder | None
    udint_word_order: WordOrder | None


@dataclass(frozen=True)
class AxisConfig:
    name: str
    joint_name: str
    coils: dict[str, int | None]
    registers: dict[str, int | None]


@dataclass(frozen=True)
class DriverConfig:
    connection: ConnectionConfig
    encoding: EncodingConfig
    axes: dict[int, AxisConfig]


def _word_order(value: Any) -> WordOrder | None:
    if value is None or str(value).lower() == "unknown":
        return None
    text = str(value).lower()
    if text in ("big", "high_first", "abcd"):
        return WordOrder.BIG
    if text in ("little", "low_first", "dcba"):
        return WordOrder.LITTLE
    raise ValueError(f"unsupported word order: {value!r}")


def load_config(path: str | Path) -> DriverConfig:
    with Path(path).open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}

    connection = raw.get("connection") or {}
    encoding = raw.get("encoding") or {}
    axes_raw = raw.get("axes") or {}

    axes: dict[int, AxisConfig] = {}
    for key, value in axes_raw.items():
        axis_id = int(str(key).removeprefix("axis"))
        axes[axis_id] = AxisConfig(
            name=f"axis{axis_id}",
            joint_name=str(value.get("joint_name", f"axis{axis_id}")),
            coils=dict(value.get("coils") or {}),
            registers={name: (None if field is None else field.get("address")) for name, field in (value.get("registers") or {}).items()},
        )

    return DriverConfig(
        connection=ConnectionConfig(
            host=str(connection.get("host", "192.168.1.88")),
            port=int(connection.get("port", 502)),
            slave_id=int(connection.get("slave_id", 1)),
            timeout_s=float(connection.get("timeout_s", 1.0)),
        ),
        encoding=EncodingConfig(
            lreal_word_order=_word_order(encoding.get("lreal_word_order")),
            udint_word_order=_word_order(encoding.get("udint_word_order")),
        ),
        axes=axes,
    )


def require_complete_addresses(config: DriverConfig) -> None:
    missing: list[str] = []
    for axis_id, axis in sorted(config.axes.items()):
        for name, address in axis.coils.items():
            if address is None:
                missing.append(f"axis{axis_id}.coils.{name}")
        for name, address in axis.registers.items():
            if address is None:
                missing.append(f"axis{axis_id}.registers.{name}")
    if config.encoding.lreal_word_order is None:
        missing.append("encoding.lreal_word_order")
    if config.encoding.udint_word_order is None:
        missing.append("encoding.udint_word_order")
    if missing:
        joined = "\n  - ".join(missing)
        raise ValueError(f"real PLC mode requires resolved Modbus map fields:\n  - {joined}")


def with_mock_addresses(config: DriverConfig) -> DriverConfig:
    """Return a config where unresolved addresses are replaced by synthetic mock addresses."""
    next_coil = 10_000
    next_register = 20_000
    axes: dict[int, AxisConfig] = {}
    for axis_id, axis in sorted(config.axes.items()):
        coils: dict[str, int | None] = {}
        for name, address in axis.coils.items():
            if address is None:
                coils[name] = next_coil
                next_coil += 1
            else:
                coils[name] = address

        registers: dict[str, int | None] = {}
        for name, address in axis.registers.items():
            if address is None:
                registers[name] = next_register
                next_register += 4
            else:
                registers[name] = address

        axes[axis_id] = AxisConfig(axis.name, axis.joint_name, coils, registers)

    return DriverConfig(
        connection=config.connection,
        encoding=EncodingConfig(
            lreal_word_order=config.encoding.lreal_word_order or WordOrder.BIG,
            udint_word_order=config.encoding.udint_word_order or WordOrder.BIG,
        ),
        axes=axes,
    )
