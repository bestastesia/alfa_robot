# ALFA MuJoCo scene

This directory is the first MuJoCo side of TIM-41.

## Current robot model

`alfa_robot.xml` is generated from the current ROS description:

- source xacro: `ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`
- current arm naming: `leftjoint1..6`, `rightjoint1..6`
- current tool frames/sites: `left_ee`, `right_ee`
- current mesh source: `ros2_ws/src/alfa_robot_description/meshes/`
- MuJoCo keeps the same root-frame zero yaw as ROS/MoveIt; the logistics scene is
  placed on the robot-facing side instead of rotating `base_link`.

Regenerate after description changes:

```bash
/usr/bin/python3 simulation/mujoco/tools/generate_mujoco.py
```

## Current logistics scene

`scene.xml` includes `alfa_robot.xml` and creates a movable container/cargo scene:

- container inner width: `2.2 m`
- container inner height: `2.4 m`
- container inner length: `4.0 m` in the robot-facing direction
- container floor top height: `0.07 m` above the world ground plane
- container opening faces the robot and starts about `0.75 m` in front of the robot
- normal cargo box: `0.40 m × 0.40 m` face toward the robot, `0.20 m` depth
- width packing: `5` normal columns, with the leftover `0.20 m` distributed as even side/inter-box gaps
- current generated cargo count: `25` movable boxes in the front row (`1` depth × `5` width columns × `5` height layers)

The cargo bodies are named `cargo_xXX_yYY_zZZ` so a later ROS bridge can convert
them into MoveIt planning-scene obstacles. Each cargo body has a `freejoint`, so
boxes can be pushed by the robot. The old side-rotated `yR` column is intentionally
removed to reduce persistent box-box contacts and MuJoCo viewer load.

## Quick checks

```bash
/usr/bin/python3 - <<'PY'
import mujoco
for path in ['simulation/mujoco/alfa_robot.xml', 'simulation/mujoco/scene_robot_only.xml', 'simulation/mujoco/scene.xml']:
    model = mujoco.MjModel.from_xml_path(path)
    print(path, model.nq, model.nu, model.nbody, model.ngeom)
PY
```

## MoveIt PlanningScene bridge experiment

First launch MoveIt, then run the 1 Hz MuJoCo-to-MoveIt scene bridge:

```bash
cd ros2_ws
source install/setup.bash
ros2 launch alfa_robot_moveit_config demo.launch.py

# another terminal
cd ros2_ws
source install/setup.bash
ros2 launch alfa_robot_moveit_config mujoco_planning_scene_bridge.launch.py
```

To see both RViz and MuJoCo at the same time, keep the MoveIt demo running and
start the combined MuJoCo viewer + PlanningScene bridge:

```bash
cd ros2_ws
source install/setup.bash
ros2 launch alfa_robot_moveit_config mujoco_sync_view.launch.py
```

Optional: use `robot_mode:=actuator` only when you intentionally want MuJoCo
servos to chase ROS targets physically; it can lag/oscillate if ROS updates jump
far from the current MuJoCo state.

This opens the MuJoCo viewer with the same `scene.xml`, subscribes to
`/joint_states`, and mirrors ROS robot joint positions into MuJoCo. The default
`robot_mode=kinematic` keeps the robot exactly aligned with RViz while MuJoCo
still advances physics for movable cargo and contacts at `200 Hz`; the viewer
refreshes at `30 Hz`.

Runtime obstacle sync now uses a compact semantic topic. `mujoco_sync_bridge.py`
publishes `/mujoco_semantic_scene` at `semantic_rate=1 Hz`: container front pose,
cargo poses, and table pose only. `semantic_scene_to_planning_scene.py` rebuilds
MoveIt objects from those parameters: container extended backward from the front
frame, fixed-size cargo boxes, and a fixed-size floating table plane.

The converter publishes `/planning_scene` and `/collision_object`; set
`pointcloud_rate:=1.0` to also publish `/mujoco_scene_points` as a lightweight
PointCloud2 demo. The older `mujoco_planning_scene_bridge.launch.py` remains a
static XML snapshot path, and `mujoco_sync_bridge.py --scene-rate ...` remains a
legacy raw-geom direct PlanningScene mode for comparison.

Quick parser-only check:

```bash
/usr/bin/python3 ros2_ws/src/alfa_robot_moveit_config/scripts/mujoco_planning_scene_bridge.py \
  --dry-run --xml simulation/mujoco/scene.xml
```

Measured 1 Hz topic check:

```bash
ros2 topic hz /planning_scene --window 3
```

## Simple control commands

After `demo.launch.py` and `mujoco_sync_view.launch.py` are running, send direct
joint goals to the active ros2_control controllers:

```bash
cd ros2_ws
source install/setup.bash

# one-shot joint command
ros2 run alfa_robot_moveit_config set_joints.py --turn 0.2 --updown 0.10 --time 2.0

# small repeatable motion demo for checking RViz and MuJoCo move together
ros2 run alfa_robot_moveit_config mujoco_joint_demo_commander.py --loops 1 --duration 2.0
```

`set_joints.py` talks directly to `/torso_controller/follow_joint_trajectory`
and `/dual_arm_controller/follow_joint_trajectory`. The MuJoCo viewer follows
through `/joint_states`, so it should mirror whatever the ROS controllers report.

## Dynamics/collision note

The robot still uses MuJoCo gravity (`0 0 -9.81`), but generated robot bodies set
`gravcomp="1"` and strong position actuators so the first TIM-41 scene behaves as
a commanded-position robot instead of collapsing under its own mesh inertia.

Robot collision geoms are contact-enabled (`contype="1" conaffinity="3"`) with
harder contact parameters. Adjacent links and known internal STL overlaps are
excluded from self-collision, while non-adjacent robot/environment/cargo contacts
remain active. Robot joints use higher-gain position actuators and damping so the
model behaves like a stiff commanded robot without becoming a fully fixed body.

The scene also includes `placement_platform` behind the robot, offset far enough
to avoid initial collision with `base_link`, `turn`, or `pitch`. Its long side
is along world `Y`, with a blue front edge facing the robot side.

`AlfaEnv.reset()` writes initial joint state once. `AlfaEnv.step()` only updates
position actuator targets and then advances MuJoCo physics, so commanded motion
does not teleport `qpos` or inject artificial impulses.

Interactive viewer:

```bash
cd simulation/mujoco
/usr/bin/python3 view_scene.py
# or
/usr/bin/python3 demo.py
```

`demo.py` loads `scene.xml` so the container and front cargo row are visible by
default. `AlfaEnv()` still defaults to `scene_robot_only.xml` for lightweight
programmatic tests unless a model path is provided explicitly.

The interactive demo and ROS viewer sync both use realtime catch-up loops with
`physics_rate=200 Hz` / `timestep=0.005` by default. If a scene is too heavy,
MuJoCo physics time is still correct, but wall-clock playback can be slower than
realtime unless the loop runs multiple physics steps per viewer frame.

## Scope note

This step only fixes the MuJoCo robot version and static logistics geometry. The
TIM-41 bridge that publishes MuJoCo objects to RViz/MoveIt planning scene is the
next layer.
