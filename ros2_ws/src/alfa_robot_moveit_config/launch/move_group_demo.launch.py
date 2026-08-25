from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("alfa_robot", package_name="alfa_robot_moveit_config")
        .robot_description_kinematics(file_path="config/kinematics.demo.yaml")
        .to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config)
