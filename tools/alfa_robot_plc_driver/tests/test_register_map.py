from alfa_robot_plc_driver.config import PlcProtocolConfig


def test_axis_base_addresses_for_12_axis_model():
    protocol = PlcProtocolConfig(max_axes=12)
    assert protocol.cmd_axis_base(1) == 0
    assert protocol.sts_axis_base(1) == 1000
    assert protocol.cmd_axis_base(6) == 160
    assert protocol.sts_axis_base(6) == 1160
    assert protocol.cmd_axis_base(12) == 352
    assert protocol.sts_axis_base(12) == 1352
