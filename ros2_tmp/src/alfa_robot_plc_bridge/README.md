# ALFA Robot PLC Bridge

`alfa_robot_plc_bridge` is the ROS2 process that connects the ALFA upper-control stack to the PLC over Modbus TCP.

## Package layout

```text
alfa_robot_plc_bridge/          # ROS package discovered by colcon/ros2
  alfa_robot_plc_bridge/
    plc_core/                   # ROS-agnostic internal library
    plc_bridge_node.py          # ROS process/node
  config/plc_bridge.yaml
  launch/plc_bridge.launch.py
  launch/execution_layer.launch.py    # 默认 mock 的长期执行层入口
```

`plc_core` is the library layer. It does not import ROS and owns:

- Modbus TCP holding-register transport.
- `MB_CMD` / `MB_STS` / `MB_SYS` register map.
- Axis address calculation.
- `DINT x100` and `WORD x100` encoding.
- ControlWord / CommandID handling.
- Position-overwrite trajectory execution.
- Emergency stop / reset commands.

`plc_bridge_node.py` is the ROS bridge process. It owns:

- Joint name to PLC Axis mapping.
- Ignoring trajectory joints that are not mapped to a PLC Axis yet.
- `/joint_states` publication from PLC feedback.
- `plc_joint_trajectory` subscription.
- Conversion from ROS radians to PLC degrees.
- Calling `plc_core.PositionOverwriteTrajectoryExecutor`.

## Run

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select alfa_robot_plc_bridge --symlink-install
source install/setup.bash
ros2 launch alfa_robot_plc_bridge plc_bridge.launch.py
```

Mock mode without PLC:

```bash
ros2 run alfa_robot_plc_bridge plc_bridge_node --ros-args -p mock:=true
```

## Topics and services

- Publishes `joint_states` (`sensor_msgs/JointState`).
- Subscribes `plc_joint_trajectory` (`trajectory_msgs/JointTrajectory`).
- Provides `plc_emergency_stop` (`std_srvs/Trigger`).
- Provides `plc_reset_emergency` (`std_srvs/Trigger`).

Only joints listed in `config/plc_bridge.yaml` under `joint_names` are sent to PLC axes. Other joints in incoming trajectories, such as `updown` or future non-arm PLC functions, are ignored while `ignore_unmapped_joints: true`.

## Current PLC trajectory semantics

The current PLC build keeps the original per-axis register addresses. High-frequency motion uses position overwrite semantics:

1. Set velocity / acceleration / deceleration as drive-side limits.
2. Keep `ControlWord=5 = Enable + MoveAbs`.
3. Periodically write `ControlWord + CommandID + TargetLow + TargetHigh`.
4. Increment `CommandID` on every target update.
5. PLC follows the latest target instead of restarting a stop-start MoveAbs plan.

## PLC execution and safety behavior

`plc_bridge_node` has the same public behavior in real and mock modes:

- Real mode connects to the PLC over Modbus TCP and writes/reads `MB_CMD`/`MB_STS` holding registers.
- Mock mode attaches an in-process virtual PLC with the same register layout and control-word behavior.
- Both modes accept `/plc_joint_trajectory` and `/dual_v5_arm_controller/follow_joint_trajectory`.
- Both modes publish `/joint_states` from PLC/virtual-PLC feedback.
- Both modes publish `/plc_bridge_state` as a compact text state for debugging.

Safety/control services:

```bash
ros2 service call /plc_soft_stop std_srvs/srv/Trigger {}
ros2 service call /plc_emergency_stop std_srvs/srv/Trigger {}
ros2 service call /plc_reset_emergency std_srvs/srv/Trigger {}
ros2 service call /plc_reset_fault std_srvs/srv/Trigger {}
ros2 service call /plc_clear_commands std_srvs/srv/Trigger {}
```

State topic example:

```text
mode=mock;state=normal;executing=false;last_error=;last_command_count=21;last_trajectory_duration_s=1.000
```

Important states:

- `normal`: ready to accept a trajectory.
- `executing`: currently streaming trajectory targets to PLC.
- `soft_stopped`: software stop/cancel was requested; reset or clear before the next formal run.
- `emergency_stopped`: emergency stop was written to PLC; call reset only after mechanical safety is confirmed.
- `fault`: PLC axis error or execution validation failure.
- `plc_comm_error`: real PLC read/write failed.

## Main-branch integration contract

This package owns only Terminal 5 / execution-layer behavior for the current main-branch demo. Task publishing, orchestration, IK and MoveIt planning live outside this package.

Recommended execution-layer launch for main-branch integration:

```bash
source /opt/ros/humble/setup.bash
cd /mnt/mydisk/ALFA/alfa_robot_ec/ros2_ws
source install/setup.bash
ros2 launch alfa_robot_plc_bridge execution_layer.launch.py
```

The launch defaults to `mock:=true`, so it starts an in-process virtual PLC. For real PLC after network setup:

```bash
ros2 launch alfa_robot_plc_bridge execution_layer.launch.py mock:=false
```

Upstream orchestration / MoveIt may send either interface:

- Action: `/dual_v5_arm_controller/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`).
- Topic: `/plc_joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`).

Preferred formal interface is the action because it returns success/cancel/failure. The topic remains useful for simple tests.

Trajectory contract:

- `joint_names` use ROS/URDF names, not PLC axis names.
- `points[].positions` are absolute joint positions in radians.
- `points[].time_from_start` is the desired trajectory time.
- Unmapped joints are ignored while `ignore_unmapped_joints: true`.
- Axis direction adaptation is internal: PLC Axis `3, 5, 8` are inverted at the bridge boundary.

Execution-layer outputs:

- `/joint_states`: current robot state in ROS/URDF direction, sourced from PLC or virtual PLC feedback.
- `/plc_bridge_state`: compact text state for orchestration/debug monitoring.

Safety-layer contract:

- Call `/plc_soft_stop` for software stop/cancel of the current trajectory stream.
- Call `/plc_emergency_stop` for PLC emergency-stop control word.
- Call `/plc_reset_emergency` only after mechanical safety is confirmed.
- Call `/plc_reset_fault` after PLC/drive fault recovery is allowed.
- Call `/plc_clear_commands` to clear current control words during test cleanup.

The execution node uses a multi-threaded executor, so service-based stop/emergency requests can interrupt a running action trajectory.

## Split execution and safety processes

For the main-branch architecture, execution and safety can run as two ROS processes:

```bash
ros2 launch alfa_robot_plc_bridge execution_with_safety.launch.py
```

This starts:

- `plc_bridge_node`: execution process. Owns PLC/virtual-PLC access, accepts trajectory commands, publishes `/joint_states` and `/plc_bridge_state`.
- `plc_safety_node`: safety process. Owns external safety API and forwards stop/reset requests to the execution process.

Safety process public API:

```bash
ros2 service call /alfa_safety/soft_stop std_srvs/srv/Trigger {}
ros2 service call /alfa_safety/emergency_stop std_srvs/srv/Trigger {}
ros2 service call /alfa_safety/reset std_srvs/srv/Trigger {}
ros2 service call /alfa_safety/clear_commands std_srvs/srv/Trigger {}
ros2 topic echo /alfa_safety/state
```

Responsibility boundary:

- The safety process does not write Modbus directly, so PLC writes stay serialized in `plc_bridge_node`.
- The safety process can still be launched independently from execution with `safety_layer.launch.py`.
- Hardware buttons, keyboard stop, watchdogs, and future PLC safety-state monitoring should call or extend the safety process, not the task orchestration layer.
