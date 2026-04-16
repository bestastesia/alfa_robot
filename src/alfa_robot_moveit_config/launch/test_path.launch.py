from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 自动加载 MoveIt 的所有配置 (URDF, SRDF, Kinematics, 等等)
    # 注意：这里的 package_name 请确保与你的功能包名称一致
    moveit_config = MoveItConfigsBuilder("alfa_robot", package_name="alfa_robot_moveit_config").to_moveit_configs()

    # 配置你的自定义节点
    my_node = Node(
        package="alfa_robot_moveit_config",
        executable="path",  # 注意：这是你在 CMakeLists 中 add_executable 定义的名字
        output="screen",
        # 将所有的 moveit 配置转化为字典，作为参数传给节点！这就是 ros2 run 缺失的一环
        parameters=[moveit_config.to_dict()], 
        # 传递你的参数，注意不要加逗号，用空格隔开
        #arguments=[
        #    "right_arm", 
        #    "1.446", "-0.275", "0.591",
        #    "0", "0.6817", "0.6915", "0.2388"
        #]
        arguments=[
            "right_arm", 
            "1.13280000", "-0.36430000", "0.62330000",
            "0.70706939", " -0.00727212", "-0.04296990", "0.70579996"
        ]
    )

    return LaunchDescription([my_node])
  
