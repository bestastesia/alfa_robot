# ALFA Robot

Dual-arm industrial robot platform based on ROS2.

## Repository Structure

```
alfa_robot/
├── ros2_ws/         ROS2 workspace (colcon build here)
│   └── src/         ROS2 packages
├── scripts/         Standalone scripts & tools
│   └── ik_benchmark/  IK benchmark tool
├── simulation/      Simulation environments
└── docs/            Documentation
```

## Build

```bash
cd ros2_ws
colcon build
source install/setup.bash
```
