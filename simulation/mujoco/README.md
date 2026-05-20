# ALFA MuJoCo scene

This directory is the first MuJoCo side of TIM-41.

## Current robot model

`alfa_robot.xml` is generated from the current ROS description:

- source xacro: `ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`
- current arm naming: `left_v5_joint1..6`, `right_v5_joint1..6`
- current tool frames/sites: `left_ee`, `right_ee`
- current mesh source: `ros2_ws/src/alfa_robot_description/meshes/`
- MuJoCo keeps the same root-frame zero yaw as ROS/MoveIt; the logistics scene is
  placed on the robot-facing side instead of rotating `base_link`.

Regenerate after description changes:

```bash
/usr/bin/python3 simulation/mujoco/tools/generate_v5_mujoco.py
```

## Current logistics scene

`scene.xml` includes `alfa_robot.xml` and creates a movable container/cargo scene:

- container inner width: `2.2 m`
- container inner height: `2.4 m`
- container inner length: `4.0 m` in the robot-facing direction
- container floor top height: `0.07 m` above the world ground plane
- container opening faces the robot and starts about `1.0 m` in front of the robot
- normal cargo box: `0.40 m × 0.40 m` face toward the robot, `0.20 m` depth
- width packing: `5` normal columns plus one rotated `0.20 m` column for the remaining width
- current generated cargo count: `30` movable boxes in the front row (`1` depth × `6` width columns × `5` height layers)

The cargo bodies are named `cargo_xXX_yYY_zZZ` and `cargo_xXX_yR_zZZ` so a later
ROS bridge can convert them into MoveIt planning-scene obstacles. Each cargo body
has a `freejoint`, so boxes can be pushed by the robot.

## Quick checks

```bash
/usr/bin/python3 - <<'PY'
import mujoco
for path in ['simulation/mujoco/alfa_robot.xml', 'simulation/mujoco/scene_robot_only.xml', 'simulation/mujoco/scene.xml']:
    model = mujoco.MjModel.from_xml_path(path)
    print(path, model.nq, model.nu, model.nbody, model.ngeom)
PY
```

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

The interactive demo uses a realtime catch-up loop with `timestep=0.005`. If a
scene is too heavy, MuJoCo physics time is still correct, but wall-clock playback
can be slower than realtime unless the loop runs multiple physics steps per
viewer frame.

## Scope note

This step only fixes the MuJoCo robot version and static logistics geometry. The
TIM-41 bridge that publishes MuJoCo objects to RViz/MoveIt planning scene is the
next layer.
