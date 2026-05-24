from __future__ import annotations

import struct
from enum import Enum
from typing import Iterable


class WordOrder(str, Enum):
    BIG = "big"
    LITTLE = "little"


def _validate_register(value: int) -> int:
    if not 0 <= int(value) <= 0xFFFF:
        raise ValueError(f"register out of range: {value!r}")
    return int(value)


def _ordered_words(data: bytes, word_order: WordOrder) -> list[int]:
    words = [int.from_bytes(data[offset : offset + 2], "big") for offset in range(0, len(data), 2)]
    if word_order == WordOrder.LITTLE:
        words.reverse()
    return words


def _bytes_from_words(registers: Iterable[int], word_order: WordOrder) -> bytes:
    words = [_validate_register(value) for value in registers]
    if word_order == WordOrder.LITTLE:
        words.reverse()
    return b"".join(value.to_bytes(2, "big") for value in words)


def encode_lreal(value: float, word_order: WordOrder = WordOrder.BIG) -> list[int]:
    return _ordered_words(struct.pack(">d", float(value)), word_order)


def decode_lreal(registers: Iterable[int], word_order: WordOrder = WordOrder.BIG) -> float:
    data = _bytes_from_words(registers, word_order)
    if len(data) != 8:
        raise ValueError(f"LREAL requires 4 registers, got {len(data) // 2}")
    return struct.unpack(">d", data)[0]


def encode_udint(value: int, word_order: WordOrder = WordOrder.BIG) -> list[int]:
    if not 0 <= int(value) <= 0xFFFFFFFF:
        raise ValueError(f"UDINT out of range: {value!r}")
    return _ordered_words(int(value).to_bytes(4, "big"), word_order)


def decode_udint(registers: Iterable[int], word_order: WordOrder = WordOrder.BIG) -> int:
    data = _bytes_from_words(registers, word_order)
    if len(data) != 4:
        raise ValueError(f"UDINT requires 2 registers, got {len(data) // 2}")
    return int.from_bytes(data, "big")
