import pytest

from alfa_task_interfaces.msg import PickTarget
from builtin_interfaces.msg import Time
from armmotion_demo.domain_contract import task_spec_from_pick_targets
from armmotion_demo.manual_domain_task import ManualDomainTask, _quaternion_from_rpy


def target(arm_id, box_id, row, column, y, mode="front"):
    message = PickTarget()
    message.arm_id = arm_id
    message.box_id = box_id
    message.row = row
    message.column = column
    message.suction_mode = mode
    for stamped in (
        message.refined_geometry.body_pose,
        message.refined_geometry.suction_surface_pose,
    ):
        stamped.header.frame_id = "base_link"
        stamped.header.stamp.sec = 10
        stamped.pose.pose.position.x = 0.9
        stamped.pose.pose.position.y = y
        stamped.pose.pose.position.z = 0.8
        stamped.pose.pose.orientation.w = 1.0
    message.refined_geometry.size_m.x = 0.3
    message.refined_geometry.size_m.y = 0.4
    message.refined_geometry.size_m.z = 0.4
    message.refined_geometry.size_valid = [True, True, True]
    return message


def test_domain_task_uses_explicit_left_right_poses():
    left = target(PickTarget.ARM_LEFT, 1, 1, 1, 0.5)
    right = target(PickTarget.ARM_RIGHT, 3, 1, 3, -0.5)
    task = task_spec_from_pick_targets("request-1", "task-1", 1, [left, right])
    assert task.task_id == "task-1"
    assert task.sequence_id == 1
    assert task.effective_distance_m == pytest.approx(0.9)
    assert task.scene_y_shift == pytest.approx(0.0)
    assert task.explicit_targets["left"]["position"] == pytest.approx([0.9, 0.5, 0.8])


def test_domain_task_rejects_reversed_arm_order():
    left = target(PickTarget.ARM_LEFT, 1, 1, 1, 0.5)
    right = target(PickTarget.ARM_RIGHT, 3, 1, 3, -0.5)
    with pytest.raises(ValueError, match="LEFT、RIGHT"):
        task_spec_from_pick_targets("request-1", "task-1", 1, [right, left])


def test_domain_task_rejects_non_base_link_body_pose():
    left = target(PickTarget.ARM_LEFT, 1, 1, 1, 0.5)
    right = target(PickTarget.ARM_RIGHT, 3, 1, 3, -0.5)
    left.refined_geometry.body_pose.header.frame_id = "map"
    with pytest.raises(ValueError, match="body_pose"):
        task_spec_from_pick_targets("request-1", "task-1", 1, [left, right])


def test_domain_task_rejects_invalid_size_evidence():
    left = target(PickTarget.ARM_LEFT, 1, 1, 1, 0.5)
    right = target(PickTarget.ARM_RIGHT, 3, 1, 3, -0.5)
    left.refined_geometry.size_valid = [True, False, True]
    with pytest.raises(ValueError, match="size_valid"):
        task_spec_from_pick_targets("request-1", "task-1", 1, [left, right])


def test_direct_top_pose_reconstructs_box_center():
    geometry = ManualDomainTask._geometry_from_suction_pose(
        [0.85, -0.4, 1.2, 0.0, 0.0, 0.0],
        "top_suction",
        Time(),
    )
    center = geometry.body_pose.pose.pose.position
    assert (center.x, center.y, center.z) == pytest.approx((0.70, -0.4, 1.0))
    suction = geometry.suction_surface_pose.pose.pose.position
    assert (suction.x, suction.y, suction.z) == pytest.approx((0.85, -0.4, 1.2))


def test_rpy_is_converted_to_normalized_quaternion():
    quaternion = _quaternion_from_rpy(3.141592653589793, 0.0, 1.5707963267948966)
    assert sum(value * value for value in quaternion) == pytest.approx(1.0)
