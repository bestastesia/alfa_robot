from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 创建MoveIt配置
    moveit_config = MoveItConfigsBuilder("alfa_robot", package_name="alfa_robot_moveit_config").to_moveit_configs()
    
    # 生成完整的demo启动描述
    demo_launch = generate_demo_launch(moveit_config)
    
    # 提取demo启动中的所有动作
    actions = []
    
    # 添加面接近节点
    surface_approach_node = Node(
        package='alfa_robot_moveit_config',
        executable='surface_approach_node',
        name='surface_approach_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time', default='false')}
        ]
    )
    
    # 由于无法直接访问demo_launch的actions，我们创建一个新的启动描述
    # 直接使用generate_demo_launch的结果，并在其基础上添加我们的节点
    # 注意：这种方法可能会重复一些参数声明，但应该能正常工作
    
    # 创建一个新的启动描述，包含demo的所有功能加上我们的节点
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    import os
    from ament_index_python.packages import get_package_share_directory
    
    # 获取demo.launch.py的路径
    demo_launch_path = os.path.join(
        get_package_share_directory('alfa_robot_moveit_config'),
        'launch',
        'demo.launch.py'
    )
    
    # 创建启动描述
    launch_description = LaunchDescription([
        # 包含demo启动文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(demo_launch_path)
        ),
        # 添加面接近节点
        surface_approach_node
    ])
    
    return launch_description
