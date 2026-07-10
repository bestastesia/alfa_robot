# ALFA Robot

Dual-arm industrial robot platform based on ROS2.

## Repository Structure

```
alfa_robot/
├── ros2_ws/         ROS2 workspace (colcon build here)
│   └── src/         ROS2 packages
├── scripts/         Motion-control experiments and validation tools
│   └── ik_benchmark/  IK benchmark and Rerun helpers
└── docs/            Architecture, interfaces and validation records
```

## Build

```bash
cd ros2_ws
colcon build
source install/setup.bash
```
