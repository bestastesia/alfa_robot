**Terminal 1：启动 Rviz 可视化**
这个命令会启动 Rviz 并加载 Fast-LIO 预设的配置文件。
```bash
cd /home/ar/fast_lio_ws
source install/setup.bash
ros2 launch fast_lio vis_bag.launch.py
```

**Terminal 2：播放 Bag 数据**
这个命令会开始回放数据，您应该能看到 Rviz 中的点云和轨迹开始更新。
```bash
cd /home/ar/fast_lio_ws
source install/setup.bash
ros2 bag play bags/2026-01-20-08-58-52_ros2
ros2 bag play bags/2026-01-20-08-58-52_ros2 --topic /livox/imu /livox/lidar

```

