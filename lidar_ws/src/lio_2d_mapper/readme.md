运行建图节点后，保存地图的命令：

ros2 topic pub /save_map std_msgs/msg/Empty --once

将3d 点云转换为2d 网格地图
3d 点云转换为2d 网格地图的命令：

ros2 run lio_2d_mapper pcd_to_grid_map_node --ros-args -p file_directory:=/home/ar/FastLio/PCD/ -p file_name:=0428_2

重定位测试流程（使用0428_2.pcd 作为定位地图）
ros2 launch fast_lio_localization localization.launch.py map:=/home/ar/FastLio/PCD/0428_2.pcd
ros2 launch lio_2d_mapper 2d_mapper.launch.py 

导航规划测试流程（使用0428_2.pcd 作为定位地图）
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node   --ros-args   -p target_frame:=2d_body   -p min_height:=-0.5   -p max_height:=0   --remap cloud_in:=/cloud_registered   --remap scan:=/scan

ros2 launch lio_2d_mapper navigation.launch.py 