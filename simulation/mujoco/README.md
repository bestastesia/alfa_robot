# ALFA MuJoCo scene

This directory is the first MuJoCo side of TIM-41.

## Current robot model

`alfa_robot.xml` is generated from the current ROS description:

- source xacro: `ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`
- current arm naming: `left_v5_joint1..6`, `right_v5_joint1..6`
- current tool frames/sites: `left_ee`, `right_ee`
- current mesh source: `ros2_ws/src/alfa_robot_description/meshes/`

Regenerate after description changes:

```bash
/usr/bin/python3 simulation/mujoco/tools/generate_v5_mujoco.py
```

## Current logistics scene

`scene.xml` includes `alfa_robot.xml` and creates a static container/cargo scene:

- container inner width: `2.2 m`
- container inner height: `2.4 m`
- container inner length: `5.2 m` for this first layout
- cargo layout: `5` vertical Z layers × `10` boxes across width × forward/depth direction tightly packed
- current generated cargo count: `500` static boxes (`10` depth × `10` width × `5` height)

The cargo bodies are named `cargo_xXX_yYY_zZZ` so a later ROS bridge can convert
them into MoveIt planning-scene obstacles.

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

Robot self-collision is disabled by using robot collision geoms with
`contype="2" conaffinity="1"`; environment/cargo geoms use `contype="1"`, so
robot-to-scene contacts are still active. This avoids current STL collision
meshes pushing adjacent robot links apart at reset.

Interactive viewer:

```bash
cd simulation/mujoco
/usr/bin/python3 view_scene.py
# or
/usr/bin/python3 demo.py
```

## Scope note

This step only fixes the MuJoCo robot version and static logistics geometry. The
TIM-41 bridge that publishes MuJoCo objects to RViz/MoveIt planning scene is the
next layer.
