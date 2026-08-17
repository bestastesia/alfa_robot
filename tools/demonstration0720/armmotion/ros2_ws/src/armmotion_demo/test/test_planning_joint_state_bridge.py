from sensor_msgs.msg import JointState

from armmotion_demo.planning_joint_state_bridge import with_fixed_pitch


def test_fixed_pitch_is_added_without_changing_turn() -> None:
    message = JointState()
    message.name = ["turn", "updown", "left_joint1"]
    message.position = [-1.4, 0.3, 0.2]
    message.velocity = [0.1, 0.0, 0.0]

    output = with_fixed_pitch(message)

    assert output.name == ["turn", "updown", "left_joint1", "pitch"]
    assert list(output.position) == [-1.4, 0.3, 0.2, 0.0]
    assert list(output.velocity) == [0.1, 0.0, 0.0, 0.0]
    assert message.name == ["turn", "updown", "left_joint1"]


def test_existing_pitch_is_preserved() -> None:
    message = JointState()
    message.name = ["turn", "pitch"]
    message.position = [-1.2, 0.05]

    output = with_fixed_pitch(message)

    assert output.name == ["turn", "pitch"]
    assert list(output.position) == [-1.2, 0.05]
