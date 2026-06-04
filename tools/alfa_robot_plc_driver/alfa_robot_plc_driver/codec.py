"""PLC WORD encoding helpers for MB_CMD/MB_STS."""

DINT_MIN = -(1 << 31)
DINT_MAX = (1 << 31) - 1
WORD_MAX = 0xFFFF


def encode_dint_x100(value: float) -> tuple[int, int]:
    raw = int(round(value * 100))
    if raw < DINT_MIN or raw > DINT_MAX:
        raise ValueError(f"DINT x100 value out of range: {value}")
    if raw < 0:
        raw = (1 << 32) + raw
    return raw & WORD_MAX, (raw >> 16) & WORD_MAX


def decode_dint_x100(low_word: int, high_word: int) -> float:
    raw = ((high_word & WORD_MAX) << 16) | (low_word & WORD_MAX)
    if raw & 0x80000000:
        raw -= 0x100000000
    return raw / 100.0


def encode_word_x100(value: float) -> int:
    raw = int(round(value * 100))
    if raw < 0 or raw > WORD_MAX:
        raise ValueError(f"WORD x100 value out of range: {value}")
    return raw


def decode_word_x100(word: int) -> float:
    return (word & WORD_MAX) / 100.0
