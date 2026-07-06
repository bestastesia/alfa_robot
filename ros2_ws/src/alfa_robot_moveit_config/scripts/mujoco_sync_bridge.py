#!/usr/bin/python3
"""Open MuJoCo viewer and follow ROS2 /joint_states.

This is the visual half of the MuJoCo/RViz sync demo:
- subscribes to `/joint_states` from MoveIt/ros2_control;
- mirrors matching joint positions into the MuJoCo model;
- advances MuJoCo physics so free cargo/environment contacts remain active;
- opens a passive MuJoCo viewer for the current logistics scene.

It is intentionally a viewer/sync node, not the PlanningScene bridge. Use
`mujoco_planning_scene_bridge.py` in parallel to publish MuJoCo scene obstacles to
MoveIt.
"""

from __future__ import annotations

import argparse
import math
import re
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import mujoco.viewer

from semantic_scene_utils import collision_objects_to_points, make_box_collision_object, make_point_cloud2


PROJECT_SCENE_RELATIVE = Path("simulation/mujoco/scene.xml")
INITIAL_POSITIONS_RELATIVE = Path("ros2_ws/src/alfa_robot_moveit_config/config/initial_positions.yaml")
DEFAULT_FRAME_ID = "base_link"
DEFAULT_OBJECT_PREFIX = "mj_"
JOINT_NAMES = [
    "base_x", "base_y", "base_yaw",
    "pitch", "turn", "updown",
    "leftjoint1", "leftjoint2", "leftjoint3",
    "leftjoint4", "leftjoint5", "leftjoint6",
    "rightjoint1", "rightjoint2", "rightjoint3",
    "rightjoint4", "rightjoint5", "rightjoint6",
]


@dataclass(frozen=True)
class RuntimeBox:
    object_id: str
    geom_id: int
    dimensions: tuple[float, float, float]


def find_default_scene() -> Path:
    start = Path(__file__).resolve()
    for parent in (start.parent, *start.parents):
        candidate = parent / PROJECT_SCENE_RELATIVE
        if candidate.exists():
            return candidate
    return Path.cwd() / PROJECT_SCENE_RELATIVE


def find_default_initial_positions() -> Path:
    start = Path(__file__).resolve()
    for parent in (start.parent, *start.parents):
        candidate = parent / INITIAL_POSITIONS_RELATIVE
        if candidate.exists():
            return candidate
    return Path.cwd() / INITIAL_POSITIONS_RELATIVE


def load_initial_positions(path: Path | None) -> dict[str, float]:
    if path is None or not path.exists():
        return {}

    positions: dict[str, float] = {}
    in_initial_positions = False
    for raw_line in path.read_text().splitlines():
        line_without_comment = raw_line.split("#", 1)[0].rstrip()
        if not line_without_comment.strip():
            continue

        stripped = line_without_comment.strip()
        if stripped == "initial_positions:":
            in_initial_positions = True
            continue

        if not in_initial_positions:
            continue
        if raw_line and not raw_line[0].isspace():
            break
        if ":" not in stripped:
            continue

        name, value = stripped.split(":", 1)
        value = value.strip()
        if name.strip() in JOINT_NAMES and value:
            positions[name.strip()] = float(value)
    return positions


def sanitize_object_id(name: str, prefix: str = DEFAULT_OBJECT_PREFIX) -> str:
    clean = re.sub(r"[^A-Za-z0-9_]+", "_", name).strip("_")
    return f"{prefix}{clean}"


def matrix_to_quat_xyzw(matrix_values) -> tuple[float, float, float, float]:
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = [float(v) for v in matrix_values]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (qx / norm, qy / norm, qz / norm, qw / norm)


def quat_xyzw_conjugate(q) -> tuple[float, float, float, float]:
    qx, qy, qz, qw = q
    return (-qx, -qy, -qz, qw)


def quat_xyzw_multiply(a, b) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    qx = aw * bx + ax * bw + ay * bz - az * by
    qy = aw * by - ax * bz + ay * bw + az * bx
    qz = aw * bz + ax * by - ay * bx + az * bw
    qw = aw * bw - ax * bx - ay * by - az * bz
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (qx / norm, qy / norm, qz / norm, qw / norm)


def quat_xyzw_rotate(q, v) -> tuple[float, float, float]:
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def parse_bool_arg(value: str) -> bool:
    return value.lower() in {"1", "true", "yes", "on"}


class MujocoJointStateViewer:
    def __init__(self, xml_path: Path, joint_state_topic: str, initial_positions_path: Path | None = None, robot_mode: str = "kinematic"):
        self.xml_path = xml_path
        self.joint_state_topic = joint_state_topic
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        self.robot_mode = robot_mode
        self.joint_targets: dict[str, float] = {}
        self.qpos_adrs: dict[str, int] = {}
        self.qvel_adrs: dict[str, int] = {}
        self.actuator_ids: dict[str, int] = {}
        self.runtime_boxes: list[RuntimeBox] = []
        self.geom_ids_by_name: dict[str, int] = {}
        self.base_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")

        for joint_name in JOINT_NAMES:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id >= 0:
                self.qpos_adrs[joint_name] = int(self.model.jnt_qposadr[joint_id])
                self.qvel_adrs[joint_name] = int(self.model.jnt_dofadr[joint_id])

        for actuator_id in range(self.model.nu):
            actuator_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
            if actuator_name:
                self.actuator_ids[actuator_name] = actuator_id
                if actuator_name.startswith("act_"):
                    self.actuator_ids[actuator_name[4:]] = actuator_id

        for geom_id in range(self.model.ngeom):
            geom_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or f"geom_{geom_id}"
            self.geom_ids_by_name[geom_name] = geom_id
            if self.model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_BOX:
                continue
            if int(self.model.geom_contype[geom_id]) == 0 and int(self.model.geom_conaffinity[geom_id]) == 0:
                continue
            half_size = self.model.geom_size[geom_id]
            self.runtime_boxes.append(RuntimeBox(
                object_id=sanitize_object_id(geom_name),
                geom_id=geom_id,
                dimensions=(float(half_size[0] * 2.0), float(half_size[1] * 2.0), float(half_size[2] * 2.0)),
            ))

        self.seed_joint_state(load_initial_positions(initial_positions_path))
        self.model.opt.timestep = 0.005

    def seed_joint_state(self, positions: dict[str, float]):
        for name, value in positions.items():
            self.set_joint_position(name, value)
            self.set_actuator_target(name, value)
            self.joint_targets[name] = value
        mujoco.mj_forward(self.model, self.data)

    def set_joint_position(self, name: str, value: float):
        qpos_adr = self.qpos_adrs.get(name)
        qvel_adr = self.qvel_adrs.get(name)
        if qpos_adr is not None:
            self.data.qpos[qpos_adr] = value
        if qvel_adr is not None:
            self.data.qvel[qvel_adr] = 0.0

    def set_actuator_target(self, name: str, value: float):
        actuator_id = self.actuator_ids.get(name)
        if actuator_id is None:
            return
        ctrl_min, ctrl_max = self.model.actuator_ctrlrange[actuator_id]
        self.data.ctrl[actuator_id] = min(max(value, ctrl_min), ctrl_max)

    def on_joint_state(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in self.qpos_adrs:
                self.joint_targets[name] = float(position)

    def apply_targets(self):
        for name, value in self.joint_targets.items():
            if self.robot_mode == "actuator":
                self.set_actuator_target(name, value)
            else:
                self.set_joint_position(name, value)
                self.set_actuator_target(name, value)

    def settle_kinematic_targets(self):
        if self.robot_mode != "kinematic":
            return
        self.apply_targets()
        mujoco.mj_forward(self.model, self.data)

    def make_collision_objects(self, frame_id, CollisionObject, SolidPrimitive):
        objects = []
        for box in self.runtime_boxes:
            pose = self.runtime_box_pose(box.geom_id, frame_id)
            objects.append(make_box_collision_object(CollisionObject, SolidPrimitive, frame_id, box.object_id, box.dimensions, pose))
        return objects

    def make_semantic_scene(self, frame_id, node, SemanticScene, SemanticCargo):
        scene = SemanticScene()
        scene.header.stamp = node.get_clock().now().to_msg()
        scene.header.frame_id = frame_id

        front_geom_id = self.geom_ids_by_name.get("container_open_top")
        if front_geom_id is not None:
            scene.container_front_pose = self.runtime_box_pose(front_geom_id, frame_id)
        table_geom_id = self.geom_ids_by_name.get("placement_platform_surface")
        if table_geom_id is not None:
            scene.table_pose = self.runtime_box_pose(table_geom_id, frame_id)

        for box in self.runtime_boxes:
            geom_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, box.geom_id) or ""
            if not geom_name.startswith("cargo_"):
                continue
            cargo = SemanticCargo()
            cargo.id = sanitize_object_id(geom_name, "")
            cargo.pose = self.runtime_box_pose(box.geom_id, frame_id)
            cargo.is_remainder = "_yR_" in geom_name
            scene.cargo.append(cargo)
        return scene

    def runtime_box_pose(self, geom_id, frame_id=DEFAULT_FRAME_ID):
        from geometry_msgs.msg import Pose

        pose = Pose()
        xyz = self.data.geom_xpos[geom_id]
        qx, qy, qz, qw = matrix_to_quat_xyzw(self.data.geom_xmat[geom_id])
        quat = (qx, qy, qz, qw)
        if frame_id == "base_link" and self.base_body_id >= 0:
            base_xyz = self.data.xpos[self.base_body_id]
            base_quat = matrix_to_quat_xyzw(self.data.xmat[self.base_body_id])
            inv_base_quat = quat_xyzw_conjugate(base_quat)
            rel_xyz = quat_xyzw_rotate(inv_base_quat, (
                float(xyz[0] - base_xyz[0]),
                float(xyz[1] - base_xyz[1]),
                float(xyz[2] - base_xyz[2]),
            ))
            quat = quat_xyzw_multiply(inv_base_quat, quat)
        else:
            rel_xyz = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
        pose.position.x = rel_xyz[0]
        pose.position.y = rel_xyz[1]
        pose.position.z = rel_xyz[2]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose
def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="MuJoCo viewer following ROS2 /joint_states")
    parser.add_argument("--xml", default=str(find_default_scene()), help="MuJoCo XML path")
    parser.add_argument("--joint-states", default="/joint_states", help="JointState topic")
    parser.add_argument("--initial-positions", default=str(find_default_initial_positions()), help="MoveIt initial_positions.yaml used to seed MuJoCo before first JointState")
    parser.add_argument("--robot-mode", choices=["kinematic", "actuator"], default="kinematic", help="kinematic mirrors ROS joint_states exactly; actuator lets MuJoCo servos chase targets")
    parser.add_argument("--rate", type=float, default=30.0, help="Viewer sync rate in Hz")
    parser.add_argument("--physics-rate", type=float, default=200.0, help="MuJoCo physics step rate in Hz")
    parser.add_argument("--max-steps-per-frame", type=int, default=20)
    parser.add_argument("--semantic-rate", type=float, default=0.0, help="Publish compact SemanticScene messages at this Hz; 0 disables")
    parser.add_argument("--scene-rate", type=float, default=0.0, help="Legacy direct PlanningScene publishing at this Hz; 0 disables")
    parser.add_argument("--scene-model-mode", choices=["raw"], default="raw", help="Legacy direct PlanningScene mode; semantic is handled by semantic_scene_to_planning_scene.py")
    parser.add_argument("--frame-id", default=DEFAULT_FRAME_ID, help="Frame for published MoveIt collision objects")
    parser.add_argument("--semantic-scene-topic", default="/mujoco_semantic_scene")
    parser.add_argument("--planning-scene-topic", default="/planning_scene")
    parser.add_argument("--collision-object-topic", default="/collision_object")
    parser.add_argument("--pointcloud-topic", default="/mujoco_scene_points")
    parser.add_argument("--pointcloud-rate", type=float, default=0.0, help="Publish a lightweight PointCloud2 demo generated from current semantic/raw boxes; 0 disables")
    parser.add_argument("--apply-on-start", default="false", help="Call /apply_planning_scene once when service is available")
    parser.add_argument("--camera-distance", type=float, default=4.0)
    parser.add_argument("--camera-elevation", type=float, default=-18.0)
    parser.add_argument("--camera-azimuth", type=float, default=135.0)
    args, _ros_args = parser.parse_known_args(argv)
    return args


def main(argv=None) -> int:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from moveit_msgs.msg import PlanningScene
    from moveit_msgs.srv import ApplyPlanningScene
    from alfa_robot_moveit_config.msg import SemanticCargo, SemanticScene

    args = parse_args(argv)
    viewer_sync = MujocoJointStateViewer(Path(args.xml), args.joint_states, Path(args.initial_positions), args.robot_mode)

    rclpy.init()
    node = Node("mujoco_sync_bridge")
    node.create_subscription(JointState, args.joint_states, viewer_sync.on_joint_state, 10)
    planning_scene_pub = None
    collision_object_pub = None
    pointcloud_pub = None
    semantic_scene_pub = None
    apply_scene_client = None
    apply_scene_sent = False
    if args.scene_rate > 0.0:
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive

        planning_scene_pub = node.create_publisher(PlanningScene, args.planning_scene_topic, 10)
        collision_object_pub = node.create_publisher(CollisionObject, args.collision_object_topic, 10)
        if parse_bool_arg(args.apply_on_start):
            apply_scene_client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    else:
        CollisionObject = None
        SolidPrimitive = None
    if args.pointcloud_rate > 0.0:
        from sensor_msgs.msg import PointCloud2

        pointcloud_pub = node.create_publisher(PointCloud2, args.pointcloud_topic, 10)
    if args.semantic_rate > 0.0:
        semantic_scene_pub = node.create_publisher(SemanticScene, args.semantic_scene_topic, 10)
    node.get_logger().info(
        f"MuJoCo viewer sync ready: xml={args.xml}, topic={args.joint_states}, "
        f"joints={len(viewer_sync.qpos_adrs)}, robot_mode={args.robot_mode}, "
        f"runtime_boxes={len(viewer_sync.runtime_boxes)}, semantic_rate={args.semantic_rate}, "
        f"scene_rate={args.scene_rate}, pointcloud_rate={args.pointcloud_rate}"
    )

    viewer_period = 1.0 / max(args.rate, 1.0)
    physics_period = 1.0 / max(args.physics_rate, 1.0)
    scene_period = 1.0 / args.scene_rate if args.scene_rate > 0.0 else 0.0
    semantic_period = 1.0 / args.semantic_rate if args.semantic_rate > 0.0 else 0.0
    pointcloud_period = 1.0 / args.pointcloud_rate if args.pointcloud_rate > 0.0 else 0.0
    viewer_sync.model.opt.timestep = physics_period
    next_viewer_tick = time.monotonic()
    next_physics_tick = next_viewer_tick
    next_scene_tick = next_viewer_tick
    next_semantic_tick = next_viewer_tick
    next_pointcloud_tick = next_viewer_tick

    try:
        with mujoco.viewer.launch_passive(viewer_sync.model, viewer_sync.data) as viewer:
            viewer.cam.distance = args.camera_distance
            viewer.cam.elevation = args.camera_elevation
            viewer.cam.azimuth = args.camera_azimuth
            viewer.cam.lookat[:] = [1.4, 0.0, 0.9]
            while viewer.is_running() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.0)
                now = time.monotonic()
                steps = 0
                while next_physics_tick <= now and steps < args.max_steps_per_frame:
                    viewer_sync.apply_targets()
                    mujoco.mj_step(viewer_sync.model, viewer_sync.data)
                    viewer_sync.settle_kinematic_targets()
                    next_physics_tick += physics_period
                    steps += 1
                if steps >= args.max_steps_per_frame:
                    next_physics_tick = time.monotonic()

                if planning_scene_pub is not None and time.monotonic() >= next_scene_tick:
                    collision_objects = viewer_sync.make_collision_objects(args.frame_id, CollisionObject, SolidPrimitive)
                    scene = PlanningScene()
                    scene.is_diff = True
                    scene.world.collision_objects = collision_objects
                    planning_scene_pub.publish(scene)
                    for collision_object in collision_objects:
                        collision_object_pub.publish(collision_object)
                    if apply_scene_client is not None and not apply_scene_sent and apply_scene_client.service_is_ready():
                        request = ApplyPlanningScene.Request()
                        request.scene = scene
                        apply_scene_client.call_async(request)
                        apply_scene_sent = True
                    next_scene_tick += scene_period

                if semantic_scene_pub is not None and time.monotonic() >= next_semantic_tick:
                    semantic_scene_pub.publish(viewer_sync.make_semantic_scene(args.frame_id, node, SemanticScene, SemanticCargo))
                    next_semantic_tick += semantic_period

                if pointcloud_pub is not None and time.monotonic() >= next_pointcloud_tick:
                    pointcloud_objects = viewer_sync.make_collision_objects(args.frame_id, CollisionObject, SolidPrimitive)
                    points = collision_objects_to_points(pointcloud_objects)
                    pointcloud_pub.publish(make_point_cloud2(points, args.frame_id, node))
                    next_pointcloud_tick += pointcloud_period
                viewer.sync()

                next_viewer_tick += viewer_period
                sleep_time = next_viewer_tick - time.monotonic()
                if sleep_time > 0.0:
                    time.sleep(min(sleep_time, viewer_period))
                else:
                    next_viewer_tick = time.monotonic()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
