import rclpy
import pytest
from sensor_msgs.msg import JointState

from robot_motion_interfaces.srv import SetRobotMotionScene, SetRobotMotionState
from robot_motion_runtime.motion_scene_source_node import MotionSceneSourceNode
from robot_motion_runtime.motion_state_source_node import MotionStateSourceNode


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init()
    yield
    if owns_context and rclpy.ok():
        rclpy.shutdown()


def test_set_state_rejects_incomplete_joint_state():
    node = MotionStateSourceNode()
    try:
        request = SetRobotMotionState.Request()
        request.joint_state.name = ["left_joint1"]
        response = node.on_set_state(request, SetRobotMotionState.Response())
        assert not response.success
        assert response.message == "joint_state.position shorter than name"
        assert node.latest_state is None
    finally:
        node.destroy_node()


def test_set_state_publishes_single_authoritative_fact_with_context():
    node = MotionStateSourceNode()
    try:
        request = SetRobotMotionState.Request()
        request.context.request_id = "request-1"
        request.context.frame_id = "base_link"
        request.context.scene_id = "scene-7"
        request.context.state_id = "state-9"
        request.source = "test_fixture"
        request.authoritative = True
        request.joint_state.name = ["updown", "left_joint1"]
        request.joint_state.position = [0.3, 0.2]

        response = node.on_set_state(request, SetRobotMotionState.Response())

        assert response.success
        assert node.latest_state is not None
        assert response.state.context.request_id == "request-1"
        assert response.state.context.frame_id == "base_link"
        assert response.state.context.scene_id == "scene-7"
        assert response.state.context.state_id == "state-9"
        assert response.state.source == "test_fixture"
        assert response.state.authoritative
        assert list(response.state.joint_state.position) == [0.3, 0.2]
    finally:
        node.destroy_node()


def test_joint_state_subscription_normalizes_hardware_names_to_model_names():
    node = MotionStateSourceNode()
    try:
        msg = JointState()
        msg.name = [
            "right_joint1",
            "left_joint2",
            "turn",
            "left_track_joint",
            "updown",
        ]
        msg.position = [0.1, 0.2, 0.3, 99.0, 0.4]

        node.on_joint_state(msg)

        assert node.latest_state is not None
        assert list(node.latest_state.joint_state.name) == [
            "updown",
            "turn",
            "pitch",
            "left_joint2",
            "right_joint1",
        ]
        assert list(node.latest_state.joint_state.position) == [0.4, 0.3, 0.0, 0.2, 0.1]
    finally:
        node.destroy_node()


def test_set_scene_preserves_scene_identity_and_objects():
    node = MotionSceneSourceNode()
    try:
        request = SetRobotMotionScene.Request()
        request.context.request_id = "request-scene"
        request.context.frame_id = "base_link"
        request.context.scene_id = "scene-11"
        request.source = "test_scene"
        request.authoritative = True

        response = node.on_set_scene(request, SetRobotMotionScene.Response())

        assert response.success
        assert node.latest_scene is not None
        assert response.scene.context.request_id == "request-scene"
        assert response.scene.context.frame_id == "base_link"
        assert response.scene.context.scene_id == "scene-11"
        assert response.scene.source == "test_scene"
        assert response.scene.authoritative
        assert len(response.scene.scene_objects) == 0
        assert len(response.scene.attached_collision_objects) == 0
    finally:
        node.destroy_node()
