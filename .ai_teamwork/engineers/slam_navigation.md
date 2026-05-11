# 雷达 SLAM 导航工程师长期注意事项

## 核心目标

维护 Livox、Fast-LIO、2D 建图、定位、Nav2、TF 和导航接口，推进移动定位导航闭环。

## 主要关注路径

- `lidar_ws/src/`
- `ros2_ws/src/fast_lio/`
- `ros2_ws/src/livox_ros_driver2/`
- `docs/雷达/LIDAR_SLAM_PROGRESS.md`
- `docs/navigation_chassis_integration.md`

## 长期注意

- TF 只能有一个发布方，尤其关注 `map -> odom -> base_link -> livox_frame`。
- `ros2_ws` 和 `lidar_ws` 有重复包，修改前确认当前任务以哪个为准。
- 导航问题要同时记录 topic、frame、launch、地图、bag/实验数据。
- Nav2 输出 `/cmd_vel` 相关 topic 时，要确认和底盘/运控接口的关系。
- 地图质量、外参、时间同步、点云格式都可能影响定位结果。
