from alfa_robot_plc_driver.codec import WordOrder, decode_lreal, decode_udint, encode_lreal, encode_udint


def test_lreal_round_trip_big_and_little():
    for order in (WordOrder.BIG, WordOrder.LITTLE):
        regs = encode_lreal(60.0, order)
        assert len(regs) == 4
        assert decode_lreal(regs, order) == 60.0


def test_udint_round_trip_big_and_little():
    for order in (WordOrder.BIG, WordOrder.LITTLE):
        regs = encode_udint(123456789, order)
        assert len(regs) == 2
        assert decode_udint(regs, order) == 123456789
