from pathlib import Path

from alfa_robot_plc_driver.axis import StopPolicy
from alfa_robot_plc_driver.config import load_config, require_complete_addresses
from alfa_robot_plc_driver.driver import PlcDriver

CONFIG = Path(__file__).resolve().parents[1] / "config" / "plc_modbus_map.example.yaml"


def test_example_config_rejects_real_mode_with_placeholders():
    config = load_config(CONFIG)
    try:
        require_complete_addresses(config)
    except ValueError as exc:
        assert "axis1.coils.enable_cmd" in str(exc)
    else:
        raise AssertionError("expected unresolved config to fail real PLC validation")


def test_mock_single_axis_move():
    driver = PlcDriver.for_mock(load_config(CONFIG))
    try:
        result = driver.axis(1).move_to_deg(30.0)
        assert result.ok
        status = driver.axis(1).read_status()
        assert status.move_cmd_done_id == result.command_id
        assert status.feedback_pos_deg == 30.0
    finally:
        driver.close()


def test_mock_multi_axis_move():
    driver = PlcDriver.for_mock(load_config(CONFIG))
    try:
        result = driver.move_many({1: 10.0, 2: -20.0, 7: 15.0}, StopPolicy.STOP)
        assert result.ok
        assert result.stopped_axes == []
        assert driver.axis(2).read_status().feedback_pos_deg == -20.0
    finally:
        driver.close()


def test_mock_move_to_zero_is_valid_target():
    driver = PlcDriver.for_mock(load_config(CONFIG))
    try:
        result = driver.axis(1).move_to_deg(0.0)
        assert result.ok
        assert driver.axis(1).read_status().feedback_pos_deg == 0.0
    finally:
        driver.close()
