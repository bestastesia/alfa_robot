# Perception-MoveIt Integration Design

## Overview

Integrate the radar+camera box perception pipeline (currently in `beta_robot_ws`) into `alfa_robot` workspace, transform detected box 6D poses from `lidar_link` frame to `base_link` frame via TF2, and feed them to MoveIt for right-arm motion planning and execution through the existing ros2_control hardware interface.

## Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                       alfa_robot workspace                        │
├──────────────────────────────────────────────────────────────────┤
│                                                                    │
│  ┌──────────────┐   ┌──────────────┐                              │
│  │ livox_driver  │   │ realsense    │    Sensor Drivers            │
│  │ /cloud_reg..  │   │ /camera/..   │                              │
│  └──────┬───────┘   └──────┬───────┘                              │
│         └────────┬─────────┘                                      │
│                  ▼                                                 │
│  ┌──────────────────────────┐                                     │
│  │    box_perception_node    │  frame_id: lidar_link               │
│  │  /target_odometry1 (Odom) │                                    │
│  └────────────┬─────────────┘                                     │
│               ▼                                                    │
│  ┌──────────────────────────┐                                     │
│  │   pose_transform_node     │  TF2: lidar_link → base_link       │
│  │  /box_pose_base (PoseStamped)│                                 │
│  └────────────┬─────────────┘                                     │
│               ▼                                                    │
│  ┌──────────────────────────┐   ┌─────────────────────┐          │
│  │  moveit_pick_node         │──▶│ move_group (MoveIt)  │          │
│  │  /trigger_pick (Service)  │   │ right_arm group      │          │
│  └──────────────────────────┘   └──────────┬──────────┘          │
│                                             ▼                     │
│                              ┌──────────────────────────┐         │
│                              │ right_arm_controller      │         │
│                              │ (JointTrajectoryController)│        │
│                              └──────────┬───────────────┘         │
│                                         ▼                         │
│                              ┌──────────────────────────┐         │
│                              │ alfa_robot_hardware (CAN) │         │
│                              │ RMD + CANopen motors      │         │
│                              └──────────────────────────┘         │
│                                                                    │
│  robot_state_publisher (URDF → TF tree)                           │
└──────────────────────────────────────────────────────────────────┘
```

## Key Topics and Frames

| Node | Subscribes | Publishes | Frame |
|------|-----------|-----------|-------|
| livox_ros_driver2 | — | `/cloud_registered_body` (PointCloud2) | lidar_link |
| realsense camera | — | `/camera/camera/color/image_raw` (Image) | camera_link |
| box_perception_node | cloud + image | `/target_odometry1` (Odometry) | lidar_link |
| pose_transform_node | `/target_odometry1` | `/box_pose_base` (PoseStamped) | base_link |
| moveit_pick_node | `/box_pose_base` | MoveIt action calls | base_link |

## Component Details

### 1. Code Migration (beta_robot_ws → alfa_robot)

Copy these packages into `alfa_robot/src/`:

- **box_perception** — YOLO segmentation + RANSAC face fitting perception node (Python)
- **box_perception_msgs** — Custom message definitions (BoxResult, BoxPerceptionResult)
- **livox_ros_driver2** — Livox MID360 LiDAR driver

**Modifications to box_perception:**

In `perception_node.py`, change the output `frame_id` from `"body"` to `"lidar_link"` in all published Odometry messages. Internal computation stays the same (all math is in the LiDAR coordinate system already).

**Modification to livox_ros_driver2:**

Configure the `frame_id` parameter to `"lidar_link"` (default is `"livox_frame"`) so the published PointCloud2 frame matches the URDF link name. This is done via the launch file parameter or MID360_config.json.

### 2. box_pose_bridge Package (New)

A new Python package `box_pose_bridge` containing two nodes.

#### 2.1 pose_transform_node

**Purpose:** Transform box poses from `lidar_link` to `base_link` using TF2.

- **Subscribes:** `/target_odometry1` (nav_msgs/Odometry)
- **Publishes:** `/box_pose_base` (geometry_msgs/PoseStamped)
- **Mechanism:**
  1. Initialize `tf2_ros.Buffer` + `TransformListener`
  2. On receiving Odometry message, extract pose
  3. Create PoseStamped with `frame_id = "lidar_link"`
  4. Call `tf_buffer.transform(pose_stamped, "base_link", timeout=0.1s)`
  5. Publish transformed PoseStamped
  6. On TF lookup failure: log WARN, skip this frame

#### 2.2 moveit_pick_node

**Purpose:** Cache latest box pose and execute MoveIt right-arm planning on service trigger.

- **Subscribes:** `/box_pose_base` (geometry_msgs/PoseStamped)
- **Service:** `/trigger_pick` (std_srvs/Trigger)
- **Planning group:** `right_arm`
- **End effector frame:** `right_ee_link`

**Workflow on trigger:**

1. Check if a valid box pose has been received; if not, return `success=false`
2. Compute end-effector target pose from the transformed box pose:
   - The box_perception Odometry message encodes `face_center` as position and derives orientation from `face_normal` (the **x-axis** of the Odometry orientation aligns with the face normal — computed as rotation from [1,0,0] to face_normal)
   - After TF2 transform by pose_transform_node, both position and orientation are in `base_link` frame
   - Extract `face_normal` as the **x-axis** of the transformed orientation quaternion
   - **Position:** `target_pos = face_center - approach_distance * face_normal`
     (default `approach_distance = 0.10m`)
   - **Orientation:** Use the transformed orientation directly (EE x-axis already aligns with face normal)
3. Plan to target pose using MoveIt (`planning_time=5.0s`, `num_attempts=3`)
4. Execute planned trajectory via `FollowJointTrajectory` action → `right_arm_controller`
5. Return `success=true/false` with message

**Parameters (config/params.yaml):**

```yaml
moveit_pick_node:
  ros__parameters:
    approach_distance: 0.10
    planning_time: 5.0
    max_attempts: 3
    planning_group: "right_arm"
    ee_link: "right_ee_link"
```

### 3. Launch File

**File:** `alfa_robot_bringup/launch/perception_moveit_pipeline.launch.py`

**Startup sequence:**

1. **robot_state_publisher** — Load URDF xacro, publish `/robot_description` and TF
2. **ros2_control controller_manager** — Hardware interface startup
3. **Controller spawners** (with delays):
   - `joint_state_broadcaster` (delay 3s)
   - `right_arm_controller` (delay 5s)
   - `torso_group_controller` (delay 5s)
4. **move_group** — MoveIt motion planning server
5. **livox_ros_driver2** — LiDAR driver (MID360_config.json)
6. **realsense camera** — Camera driver (rs_launch.py include)
7. **box_perception_node** — Perception pipeline (params.yaml)
8. **pose_transform_node** — TF2 coordinate transform bridge
9. **moveit_pick_node** — MoveIt planning with service trigger

### 4. End-Effector Target Pose Computation

The box_perception Odometry output encodes:

- **Position** = `nearest_face_center` (center of the nearest box face)
- **Orientation** = derived from `nearest_face_normal` (**x-axis** of quaternion aligns with face normal; computed as rotation from [1,0,0] to face_normal)

After TF2 transformation to `base_link` by `pose_transform_node`, the moveit_pick_node:

1. Extracts the face normal as the **x-axis** of the orientation quaternion
2. Computes the approach position:
   ```
   target_position = face_center - approach_distance * face_normal
   ```
3. Uses the transformed orientation directly as the EE target orientation (x-axis faces the box surface)

### 5. Error Handling

| Scenario | Handling |
|----------|----------|
| TF lookup timeout | pose_transform_node logs WARN, skips frame |
| No box detected | moveit_pick_node cache empty, trigger returns success=false |
| MoveIt planning failure | Retry up to 3 times, then return success=false with error message |
| Joint limits exceeded | MoveIt handles automatically, planning fails with report |
| Execution timeout | FollowJointTrajectory action has built-in timeout |

### 6. File Structure

```
alfa_robot/src/
├── box_perception/              (copied from beta_robot_ws, frame_id modified)
├── box_perception_msgs/         (copied from beta_robot_ws, no changes)
├── livox_ros_driver2/           (copied from beta_robot_ws, no changes)
├── box_pose_bridge/             (NEW package)
│   ├── package.xml
│   ├── setup.py
│   ├── box_pose_bridge/
│   │   ├── __init__.py
│   │   ├── pose_transform_node.py
│   │   └── moveit_pick_node.py
│   └── config/
│       └── params.yaml
├── alfa_robot_bringup/
│   └── launch/
│       └── perception_moveit_pipeline.launch.py  (NEW launch file)
├── alfa_robot_description/      (existing, no changes)
├── alfa_robot_hardware/         (existing, no changes)
└── alfa_robot_moveit_config/    (existing, no changes)
```

### 7. Dependencies

**box_pose_bridge package.xml:**

- rclpy
- geometry_msgs
- nav_msgs
- std_srvs
- tf2_ros
- tf2_geometry_msgs
- moveit_commander
- box_perception_msgs

### 8. Scope: Simple Version (updown fixed)

This initial implementation assumes:

- The `updown` joint position is fixed during perception and grasping
- Since `lidar_link` and `camera_link` are both children of `updown_link`, the TF2 transform from `lidar_link` → `base_link` will automatically account for the current updown position via the TF tree published by `robot_state_publisher`
- Only `right_arm` (5 joints: rightarmbase, rightjoint1-4) is planned
- `torso_group` (turn + updown) is not included in the pick planning

**Future extension:** Add torso_group to planning for reach optimization.

### 9. Usage

```bash
# Terminal 1: Launch the full pipeline
ros2 launch alfa_robot_bringup perception_moveit_pipeline.launch.py

# Terminal 2: Trigger a pick when a box is detected
ros2 service call /trigger_pick std_srvs/srv/Trigger
```
