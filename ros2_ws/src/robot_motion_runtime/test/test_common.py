import math
import threading

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from alfa_robot_execution_bridge.joints import RT_CONTROL_JOINT_NAMES
from robot_motion_runtime.execute_trajectory_service_node import ExecuteTrajectoryServiceNode

from robot_motion_runtime.common import (
    concatenate_trajectories,
    duration_seconds,
    make_fixed_rate_interpolated_trajectory,
    make_interpolated_trajectory,
    resample_trajectory,
)


def joint_state(names, positions):
    state = JointState()
    state.name = list(names)
    state.position = list(positions)
    return state


def test_interpolation_respects_joint_and_updown_step_limits():
    start = joint_state(["updown", "left_joint1"], [0.0, 0.0])
    goal = joint_state(["updown", "left_joint1"], [0.03, math.radians(12.0)])

    trajectory = make_interpolated_trajectory(
        start,
        goal,
        duration_s=1.2,
        max_joint_step_rad=math.radians(5.0),
        max_linear_step_m=0.01,
    )

    assert len(trajectory.points) == 4
    assert list(trajectory.points[0].positions) == [0.0, 0.0]
    assert list(trajectory.points[-1].positions) == list(goal.position)
    assert duration_seconds(trajectory.points[-1].time_from_start) == 1.2


def test_fixed_rate_interpolation_uses_point_cadence_for_duration():
    start = joint_state(["left_joint1"], [0.0])
    goal = joint_state(["left_joint1"], [math.radians(18.0)])

    trajectory = make_fixed_rate_interpolated_trajectory(
        start,
        goal,
        rate_hz=10.0,
        max_joint_step_rad=math.radians(4.5),
    )

    assert len(trajectory.points) == 5
    assert [round(duration_seconds(point.time_from_start), 6) for point in trajectory.points] == [
        0.0,
        0.1,
        0.2,
        0.3,
        0.4,
    ]
    assert list(trajectory.points[-1].positions) == list(goal.position)


def test_concatenate_removes_duplicate_seam_and_offsets_time():
    zero = joint_state(["left_joint1"], [0.0])
    middle = joint_state(["left_joint1"], [0.5])
    goal = joint_state(["left_joint1"], [1.0])
    first = make_interpolated_trajectory(zero, middle, duration_s=1.0, max_joint_step_rad=1.0)
    second = make_interpolated_trajectory(middle, goal, duration_s=2.0, max_joint_step_rad=1.0)

    combined = concatenate_trajectories(first, second)

    assert len(combined.points) == len(first.points) + len(second.points) - 1
    assert list(combined.points[-1].positions) == [1.0]
    assert duration_seconds(combined.points[-1].time_from_start) == 3.0


def test_resample_produces_fixed_rate_and_keeps_endpoints():
    start = joint_state(["left_joint1"], [0.0])
    goal = joint_state(["left_joint1"], [1.0])
    trajectory = make_interpolated_trajectory(start, goal, duration_s=1.0, max_joint_step_rad=2.0)

    sampled = resample_trajectory(trajectory, rate_hz=4.0)

    assert len(sampled.points) == 5
    assert list(sampled.points[0].positions) == [0.0]
    assert list(sampled.points[-1].positions) == [1.0]
    assert duration_seconds(sampled.points[-1].time_from_start) == 1.0


def test_execution_adapter_builds_direct_semantic_full_14_axis_goal():
    trajectory = JointTrajectory()
    trajectory.joint_names = ["left_joint3", "updown"]
    point = JointTrajectoryPoint()
    point.positions = [-0.4, 0.31]
    point.velocities = [-0.2, 0.02]
    trajectory.points = [point]

    node = ExecuteTrajectoryServiceNode.__new__(ExecuteTrajectoryServiceNode)
    node.adapt_to_hardware_joint_order = True
    node.hold_missing_from_joint_states = True
    node.joint_state_lock = threading.RLock()
    node.latest_joint_positions = {name: 0.0 for name in RT_CONTROL_JOINT_NAMES}

    outgoing = node.adapt_trajectory_for_hardware(trajectory)

    assert outgoing.joint_names == RT_CONTROL_JOINT_NAMES
    left_joint3 = outgoing.joint_names.index("left_joint3")
    updown = outgoing.joint_names.index("updown")
    assert outgoing.points[0].positions[left_joint3] == -0.4
    assert outgoing.points[0].velocities[left_joint3] == -0.2
    assert outgoing.points[0].positions[updown] == 0.31
    assert outgoing.points[0].velocities[updown] == 0.02
