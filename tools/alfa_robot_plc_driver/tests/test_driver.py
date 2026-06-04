import pytest

from alfa_robot_plc_driver import PlcDriver
from alfa_robot_plc_driver.mock import MockPlcTransport


def test_read_system_and_axes_from_mock():
    driver = PlcDriver(transport=MockPlcTransport(active_axes=6))
    system = driver.read_system()
    assert system.active_axis_count == 6
    axes = driver.read_axes()
    assert [axis.axis for axis in axes] == [1, 2, 3, 4, 5, 6]


def test_move_abs_writes_command_and_mirrors_status():
    driver = PlcDriver(transport=MockPlcTransport(active_axes=6))
    [command] = driver.move_abs({4: 2.0}, velocity=5, acceleration=20, deceleration=20)
    assert command.axis == 4
    assert command.command_id == 1
    assert command.target_deg == 2.0
    assert command.status.control_word == 5
    assert command.status.ack_command_id == 1
    assert command.status.feedback_single_deg == 2.0
    assert command.status.last_target_deg == 2.0


def test_move_delta_uses_feedback_single_as_base():
    driver = PlcDriver(transport=MockPlcTransport(active_axes=6))
    driver.move_abs({2: 1.0})
    [command] = driver.move_delta({2: 0.25})
    assert command.status.last_target_deg == 1.25


def test_reject_inactive_axis_write():
    driver = PlcDriver(transport=MockPlcTransport(active_axes=6))
    with pytest.raises(ValueError, match="not active"):
        driver.move_abs({7: 1.0})
