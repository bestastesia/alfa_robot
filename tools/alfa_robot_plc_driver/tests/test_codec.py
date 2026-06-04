from alfa_robot_plc_driver.codec import (
    decode_dint_x100,
    decode_word_x100,
    encode_dint_x100,
    encode_word_x100,
)


def test_dint_x100_round_trip_positive_and_negative():
    for value in [0.0, 2.0, -1.23, 180.0, -179.98]:
        low, high = encode_dint_x100(value)
        assert decode_dint_x100(low, high) == round(value, 2)


def test_dint_x100_low_word_first():
    assert encode_dint_x100(2.0) == (200, 0)
    assert encode_dint_x100(-0.01) == (0xFFFF, 0xFFFF)


def test_word_x100_round_trip():
    assert encode_word_x100(5.0) == 500
    assert decode_word_x100(3000) == 30.0
