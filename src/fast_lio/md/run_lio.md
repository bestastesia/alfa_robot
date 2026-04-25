# 这个 launch 会自动把输出格式切成 PointCloud2 并在 Rviz 显示
source /home/ar/fast_lio_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py

source /home/ar/fast_lio_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false


source /home/ar/fast_lio_ws/install/setup.bash
ros2 topic hz /livox/lidar/imu  
<!-- 应该为200hz -->
ros2 topic list
