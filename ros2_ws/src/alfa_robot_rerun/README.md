# alfa_robot_rerun

Minimal Rerun visualization case for ALFA Robot.

This first version is intentionally read-only and small:

- reads the current `alfa_robot_description` xacro;
- loads link visual meshes into Rerun;
- subscribes to `/joint_states`;
- runs a simple URDF forward-kinematics pass;
- updates every link transform in the Rerun viewer.

It does not replace MoveIt/RViz interaction. It is only a base example for future
Rerun visualization and recording work.

## Dependency

Use the ROS system Python environment:

```bash
/usr/bin/python3 -m pip install --user rerun-sdk
```

The Rerun CLI/viewer should also be available:

```bash
rerun --version
```

## Build

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
colcon build --packages-select alfa_robot_rerun
source install/setup.bash
```

## Run with an existing robot system

Start any source of `/joint_states`, for example MoveIt demo or a real bringup.
Then run:

```bash
ros2 launch alfa_robot_rerun basic_robot_viewer.launch.py
```

If you only want to validate the ROS node without opening the viewer:

```bash
ros2 launch alfa_robot_rerun basic_robot_viewer.launch.py spawn:=false
```

Optional recording:

```bash
ros2 launch alfa_robot_rerun basic_robot_viewer.launch.py recording_path:=/tmp/alfa_basic.rrd
```

## Minimal test with joint_state_publisher_gui

In one terminal:

```bash
ros2 launch alfa_robot_description view_alfa_robot.launch.py
```

In another terminal:

```bash
ros2 launch alfa_robot_rerun basic_robot_viewer.launch.py
```

This keeps RViz from the description launch open, but the Rerun package itself is
independent and only requires `/joint_states` plus the robot description.

## Current scope

This package is deliberately a minimal case:

- no MoveIt planning control;
- no interactive end-effector marker;
- no perception or force overlays;
- no collision/planning-scene display.

Those can be added later as separate loggers once the basic robot-state display
is stable.
