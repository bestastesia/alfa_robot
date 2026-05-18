#!/usr/bin/env python3
"""Generate MuJoCo ALFA v5 robot and logistics scene XML from current ROS URDF."""

from __future__ import annotations

import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
MUJOCO_DIR = REPO_ROOT / "simulation" / "mujoco"
URDF_XACRO = REPO_ROOT / "ros2_ws" / "src" / "alfa_robot_description" / "urdf" / "alfa_robot.urdf.xacro"
ROBOT_XML = MUJOCO_DIR / "alfa_robot.xml"
SCENE_XML = MUJOCO_DIR / "scene.xml"
ROBOT_ONLY_XML = MUJOCO_DIR / "scene_robot_only.xml"

MESH_PREFIX = "package://alfa_robot_description/meshes/"
MESH_ROOT = "../../ros2_ws/src/alfa_robot_description/meshes"

JOINT_DAMPING = {
    "pitch": 300,
    "turn": 300,
    "updown": 500,
}
POSITION_KP = {
    "pitch": 12000,
    "turn": 12000,
    "updown": 60000,
}
DEFAULT_KP_REVOLUTE = 12000
DEFAULT_KP_PRISMATIC = 60000
BASE_FORCE_RANGE = "-200000 200000"
JOINT_FORCE_RANGE = "-100000 100000"


def render_urdf() -> ET.Element:
    command = (
        "source /opt/ros/humble/setup.bash; "
        f"source {REPO_ROOT / 'ros2_ws/install/setup.bash'} 2>/dev/null || true; "
        f"xacro {URDF_XACRO}"
    )
    result = subprocess.run(["bash", "-lc", command], cwd=REPO_ROOT, check=True, text=True, stdout=subprocess.PIPE)
    return ET.fromstring(result.stdout)


def attrs(items: dict[str, object]) -> str:
    return " ".join(f'{key}="{value}"' for key, value in items.items() if value is not None)


def parse_vec(text: str | None, default: str) -> str:
    return text if text else default


def rpy_to_quat_wxyz(rpy_text: str) -> str:
    roll, pitch, yaw = [float(value) for value in rpy_text.split()]
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return f"{w:.12g} {x:.12g} {y:.12g} {z:.12g}"


def inertial_xml(link: ET.Element) -> str:
    inertial = link.find("inertial")
    if inertial is None:
        return '<inertial pos="0 0 0" mass="0.1" diaginertia="0.001 0.001 0.001"/>'
    origin = inertial.find("origin")
    mass = inertial.find("mass")
    inertia = inertial.find("inertia")
    pos = parse_vec(origin.get("xyz") if origin is not None else None, "0 0 0")
    mass_value = mass.get("value") if mass is not None else "0.1"
    if inertia is None:
        return f'<inertial pos="{pos}" mass="{mass_value}" diaginertia="0.001 0.001 0.001"/>'
    full = " ".join(
        inertia.get(name, "0")
        for name in ["ixx", "iyy", "izz", "ixy", "ixz", "iyz"]
    )
    return f'<inertial pos="{pos}" mass="{mass_value}" fullinertia="{full}"/>'


def mesh_name(link_name: str, kind: str) -> str | None:
    if link_name in {"world", "left_v5_link0", "right_v5_link0", "left_v5_tool0", "right_v5_tool0"}:
        return None
    return f"{kind}_{link_name}"


def link_mesh_path(link: ET.Element, kind: str) -> str | None:
    element = link.find(kind)
    if element is None:
        return None
    mesh = element.find("geometry/mesh")
    if mesh is None:
        return None
    filename = mesh.get("filename")
    if not filename or not filename.startswith(MESH_PREFIX):
        return None
    return filename[len(MESH_PREFIX):]


def collect_model(root: ET.Element):
    links = {link.get("name"): link for link in root.findall("link") if link.get("name")}
    joints = []
    children = {}
    child_links = set()
    for joint in root.findall("joint"):
        name = joint.get("name")
        parent = joint.find("parent").get("link") if joint.find("parent") is not None else ""
        child = joint.find("child").get("link") if joint.find("child") is not None else ""
        joints.append(joint)
        children.setdefault(parent, []).append(joint)
        child_links.add(child)
    roots = [name for name in links if name not in child_links]
    return links, joints, children, roots[0] if roots else "world"


def joint_xml(joint: ET.Element) -> str:
    joint_type = joint.get("type", "fixed")
    if joint_type == "fixed":
        return ""
    name = joint.get("name")
    axis = parse_vec(joint.find("axis").get("xyz") if joint.find("axis") is not None else None, "1 0 0")
    limit = joint.find("limit")
    mj_type = "hinge" if joint_type in {"revolute", "continuous"} else "slide"
    damping = JOINT_DAMPING.get(name, 50)
    if joint_type == "continuous" or limit is None:
        return f'<joint name="{name}" type="{mj_type}" axis="{axis}" limited="false" damping="{damping}" armature="2.0"/>'
    lower = limit.get("lower", "0")
    upper = limit.get("upper", "0")
    return f'<joint name="{name}" type="{mj_type}" axis="{axis}" range="{lower} {upper}" damping="{damping}" armature="2.0"/>'


def geom_xml(link_name: str, kind: str, material: str | None = None) -> str:
    mesh = mesh_name(link_name, "vis" if kind == "visual" else "col")
    if not mesh:
        return ""
    cls = "visual" if kind == "visual" else "collision"
    material_attr = f' material="{material}"' if material else ""
    return f'<geom name="{kind}_{link_name}" class="{cls}" type="mesh" mesh="{mesh}"{material_attr}/>'


def material_for_link(link_name: str) -> str:
    if link_name == "base_link":
        return "base_blue"
    if link_name in {"pitch"}:
        return "white"
    if link_name in {"turn", "updown"}:
        return "grey"
    return "arm_white"


def origin_attrs(joint: ET.Element) -> str:
    origin = joint.find("origin")
    if origin is None:
        return ""
    xyz = origin.get("xyz")
    rpy = origin.get("rpy")
    parts = []
    if xyz:
        parts.append(f'pos="{xyz}"')
    if rpy and rpy != "0 0 0":
        parts.append(f'quat="{rpy_to_quat_wxyz(rpy)}"')
    return " " + " ".join(parts) if parts else ""


def body_xml(link_name: str, links, children, depth: int = 2) -> list[str]:
    indent = "  " * depth
    lines = []
    link = links[link_name]
    lines.append(f'{indent}<body name="{link_name}">')
    lines.append(f'{indent}  {inertial_xml(link)}')
    visual = geom_xml(link_name, "visual", material_for_link(link_name))
    collision = geom_xml(link_name, "collision")
    if visual:
        lines.append(f'{indent}  {visual}')
    if collision:
        lines.append(f'{indent}  {collision}')
    if link_name == "left_v5_tool0":
        lines.extend(suction_xml("left", indent + "  "))
    elif link_name == "right_v5_tool0":
        lines.extend(suction_xml("right", indent + "  "))
    for joint in children.get(link_name, []):
        child = joint.find("child").get("link")
        child_indent = indent + "  "
        lines.append(f'{child_indent}<body name="{child}"{origin_attrs(joint)}>')
        joint_line = joint_xml(joint)
        if joint_line:
            lines.append(f'{child_indent}  {joint_line}')
        child_link = links[child]
        lines.append(f'{child_indent}  {inertial_xml(child_link)}')
        visual = geom_xml(child, "visual", material_for_link(child))
        collision = geom_xml(child, "collision")
        if visual:
            lines.append(f'{child_indent}  {visual}')
        if collision:
            lines.append(f'{child_indent}  {collision}')
        if child == "left_v5_tool0":
            lines.extend(suction_xml("left", child_indent + "  "))
        elif child == "right_v5_tool0":
            lines.extend(suction_xml("right", child_indent + "  "))
        for grand in children.get(child, []):
            # recursion emits grandchildren as nested bodies without duplicating current child wrapper
            lines.extend(joint_subtree_xml(grand, links, children, depth + 2))
        lines.append(f'{child_indent}</body>')
    lines.append(f'{indent}</body>')
    return lines


def joint_subtree_xml(joint: ET.Element, links, children, depth: int) -> list[str]:
    indent = "  " * depth
    child = joint.find("child").get("link")
    link = links[child]
    lines = [f'{indent}<body name="{child}"{origin_attrs(joint)} gravcomp="1">']
    joint_line = joint_xml(joint)
    if joint_line:
        lines.append(f'{indent}  {joint_line}')
    lines.append(f'{indent}  {inertial_xml(link)}')
    visual = geom_xml(child, "visual", material_for_link(child))
    collision = geom_xml(child, "collision")
    if visual:
        lines.append(f'{indent}  {visual}')
    if collision:
        lines.append(f'{indent}  {collision}')
    if child == "left_v5_tool0":
        lines.extend(suction_xml("left", indent + "  "))
    elif child == "right_v5_tool0":
        lines.extend(suction_xml("right", indent + "  "))
    for grand in children.get(child, []):
        lines.extend(joint_subtree_xml(grand, links, children, depth + 1))
    lines.append(f'{indent}</body>')
    return lines


def suction_xml(side: str, indent: str) -> list[str]:
    return [
        f'{indent}<geom name="{side}_suction_geom" type="cylinder" size="0.045 0.012" rgba="0.05 0.05 0.05 0.85" contype="2" conaffinity="1"/>',
        f'{indent}<site name="{side}_ee" pos="0 0 0" size="0.025" rgba="0 1 0 0.8"/>',
        f'{indent}<site name="{side}_suction_dir" pos="0 0 0" size="0.01" zaxis="0 0 1"/>',
    ]


def actuator_xml(joints: list[ET.Element]) -> list[str]:
    lines = ["  <actuator>"]
    lines.append(f'    <position name="act_base_x" joint="base_x" kp="80000" ctrlrange="-10 10" ctrllimited="true" forcerange="{BASE_FORCE_RANGE}"/>')
    lines.append(f'    <position name="act_base_y" joint="base_y" kp="80000" ctrlrange="-10 10" ctrllimited="true" forcerange="{BASE_FORCE_RANGE}"/>')
    lines.append(f'    <position name="act_base_yaw" joint="base_yaw" kp="80000" ctrlrange="-6.28318530718 6.28318530718" ctrllimited="true" forcerange="{BASE_FORCE_RANGE}"/>')
    for joint in joints:
        name = joint.get("name")
        joint_type = joint.get("type")
        if joint_type == "fixed" or name == "world_to_base":
            continue
        limit = joint.find("limit")
        if joint_type == "continuous" or limit is None:
            ctrlrange = "-6.28318530718 6.28318530718"
        else:
            ctrlrange = f'{limit.get("lower", "0")} {limit.get("upper", "0")}'
        kp = POSITION_KP.get(name, DEFAULT_KP_PRISMATIC if joint_type == "prismatic" else DEFAULT_KP_REVOLUTE)
        lines.append(f'    <position name="act_{name}" joint="{name}" kp="{kp}" ctrlrange="{ctrlrange}" ctrllimited="true" forcerange="{JOINT_FORCE_RANGE}"/>')
    lines.append('    <adhesion name="act_right_suction" body="right_v5_tool0" ctrlrange="0 1" gain="50000" forcerange="0 50000"/>')
    lines.append('    <adhesion name="act_left_suction" body="left_v5_tool0" ctrlrange="0 1" gain="50000" forcerange="0 50000"/>')
    lines.append("  </actuator>")
    return lines


def sensor_xml(joints: list[ET.Element]) -> list[str]:
    lines = ["  <sensor>"]
    for name in ["base_x", "base_y", "base_yaw"]:
        lines.append(f'    <jointpos name="s_{name}" joint="{name}"/>')
    for joint in joints:
        name = joint.get("name")
        if joint.get("type") != "fixed" and name != "world_to_base":
            lines.append(f'    <jointpos name="s_{name}" joint="{name}"/>')
    for side in ["right", "left"]:
        lines.append(f'    <framepos name="s_{side}_ee_pos" objtype="site" objname="{side}_ee"/>')
        lines.append(f'    <framezaxis name="s_{side}_suction_normal" objtype="site" objname="{side}_suction_dir"/>')
        lines.append(f'    <touch name="s_{side}_suction_touch" site="{side}_ee"/>')
    lines.append("  </sensor>")
    return lines


def generate_robot_xml(root: ET.Element) -> str:
    links, joints, children, _root = collect_model(root)
    mesh_lines = []
    for link_name, link in links.items():
        for kind, prefix in [("visual", "vis"), ("collision", "col")]:
            path = link_mesh_path(link, kind)
            name = mesh_name(link_name, prefix)
            if path and name:
                mesh_lines.append(f'    <mesh name="{name}" file="{path}"/>')

    base_joint = next(j for j in joints if j.get("name") == "world_to_base")
    base_pos = base_joint.find("origin").get("xyz", "0 0 0")

    lines = [
        '<?xml version="1.0" encoding="utf-8"?>',
        '<mujoco model="alfa_robot_v5_current">',
        f'  <compiler angle="radian" meshdir="{MESH_ROOT}" autolimits="true"/>',
        '  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"/>',
        '',
        '  <default>',
        '    <joint damping="50" armature="2.0"/>',
        '    <velocity ctrllimited="true" forcelimited="true"/>',
        '    <position forcelimited="true"/>',
        '    <default class="visual"><geom contype="0" conaffinity="0" group="2"/></default>',
        '    <default class="collision"><geom contype="2" conaffinity="1" group="3" friction="1.0 0.05 0.01"/></default>',
        '  </default>',
        '',
        '  <asset>',
        '    <texture type="skybox" builtin="gradient" rgb1="0.6 0.8 1.0" rgb2="0.0 0.0 0.0" width="512" height="512"/>',
        '    <texture name="grid" type="2d" builtin="checker" rgb1="0.85 0.85 0.85" rgb2="0.65 0.65 0.65" width="512" height="512"/>',
        '    <material name="grid_mat" texture="grid" texrepeat="4 4" reflectance="0.2"/>',
        '    <material name="base_blue" rgba="0.18 0.32 0.75 1"/>',
        '    <material name="grey" rgba="0.45 0.45 0.45 1"/>',
        '    <material name="white" rgba="0.9 0.9 0.9 1"/>',
        '    <material name="arm_white" rgba="0.92 0.92 0.92 1"/>',
        *mesh_lines,
        '  </asset>',
        '',
        '  <worldbody>',
        '    <geom name="floor" type="plane" size="5 5 0.1" material="grid_mat"/>',
        f'    <body name="base_link" pos="{base_pos}" gravcomp="1">',
        '      <joint name="base_x" type="slide" axis="1 0 0" damping="3000" limited="false"/>',
        '      <joint name="base_y" type="slide" axis="0 1 0" damping="3000" limited="false"/>',
        '      <joint name="base_yaw" type="hinge" axis="0 0 1" damping="3000" limited="false"/>',
        f'      {inertial_xml(links["base_link"])}',
        '      ' + geom_xml("base_link", "visual", material_for_link("base_link")),
        '      ' + geom_xml("base_link", "collision"),
    ]
    for joint in children.get("base_link", []):
        lines.extend(joint_subtree_xml(joint, links, children, 3))
    lines.extend([
        '    </body>',
        '  </worldbody>',
        '',
        *actuator_xml(joints),
        '',
        *sensor_xml(joints),
        '',
        '</mujoco>',
        '',
    ])
    return "\n".join(lines)


def box_body(name: str, pos: tuple[float, float, float], size: tuple[float, float, float], rgba: str) -> str:
    sx, sy, sz = size
    return (
        f'    <body name="{name}" pos="{pos[0]:.3f} {pos[1]:.3f} {pos[2]:.3f}">\n'
        f'      <geom type="box" size="{sx:.3f} {sy:.3f} {sz:.3f}" rgba="{rgba}" contype="1" conaffinity="1" friction="0.8 0.03 0.001"/>\n'
        f'      <site name="{name}_top" pos="0 0 {sz:.3f}" size="0.012" rgba="0 1 0 0.6"/>\n'
        f'    </body>'
    )


def generate_scene_xml() -> str:
    # Container inner dimensions: width 2.2 m, height 2.4 m.
    # User semantics: "5 rows" means 5 vertical Z layers. Width keeps about
    # 10 boxes per layer, and the forward/depth direction is tightly packed.
    inner_width = 2.2
    inner_height = 2.4
    inner_length = 5.2
    x0 = 1.0
    center_x = x0 + inner_length / 2.0
    box_sx = 0.24
    box_sy = 0.10
    box_sz = 0.22
    floor_top_z = 0.07
    x_count = max(1, int(inner_length // (2.0 * box_sx)))
    y_count = 10
    z_count = 5
    x_pitch = (inner_length - 2.0 * box_sx) / max(x_count - 1, 1)
    y_pitch = inner_width / y_count
    z_pitch = (inner_height - 2.0 * box_sz) / max(z_count - 1, 1)
    x_start = x0 + box_sx
    y_start = -inner_width / 2.0 + y_pitch / 2.0
    z_start = floor_top_z + box_sz
    colors = ["0.55 0.40 0.25 1", "0.72 0.50 0.30 1", "0.90 0.68 0.42 1"]
    cargo_lines = []
    for z_index in range(z_count):
        z = z_start + z_index * z_pitch
        for x_index in range(x_count):
            x = x_start + x_index * x_pitch
            for y_index in range(y_count):
                y = y_start + y_index * y_pitch
                name = f"cargo_x{x_index:02d}_y{y_index:02d}_z{z_index:02d}"
                color = colors[z_index % len(colors)]
                cargo_lines.append(box_body(name, (x, y, z), (box_sx, box_sy * 0.92, box_sz), color))

    lines = [
        '<?xml version="1.0" encoding="utf-8"?>',
        '<mujoco model="alfa_logistics_scene_v5">',
        '  <compiler angle="radian" autolimits="true"/>',
        '  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic" noslip_iterations="3"/>',
        '  <include file="alfa_robot.xml"/>',
        '  <worldbody>',
        '    <light name="container_light_front" pos="1.5 0 2.8" dir="0 0 -1" diffuse="0.8 0.8 0.7" specular="0.1 0.1 0.1"/>',
        '    <light name="container_light_back" pos="5.0 0 2.8" dir="0 0 -1" diffuse="0.7 0.7 0.65" specular="0.1 0.1 0.1"/>',
        '    <geom name="scene_floor" type="plane" size="15 15 0.1" material="grid_mat" pos="0 0 0" contype="1" conaffinity="1"/>',
        '    <body name="container" pos="0 0 0">',
        f'      <geom name="container_floor" type="box" size="{inner_length/2:.3f} {inner_width/2:.3f} 0.035" pos="{center_x:.3f} 0 0.035" rgba="0.50 0.43 0.34 1" contype="1" conaffinity="1"/>',
        f'      <geom name="container_wall_left" type="box" size="{inner_length/2:.3f} 0.045 {inner_height/2:.3f}" pos="{center_x:.3f} {inner_width/2+0.045:.3f} {inner_height/2:.3f}" rgba="0.55 0.55 0.50 1" contype="1" conaffinity="1"/>',
        f'      <geom name="container_wall_right" type="box" size="{inner_length/2:.3f} 0.045 {inner_height/2:.3f}" pos="{center_x:.3f} {-inner_width/2-0.045:.3f} {inner_height/2:.3f}" rgba="0.55 0.55 0.50 1" contype="1" conaffinity="1"/>',
        f'      <geom name="container_roof" type="box" size="{inner_length/2:.3f} {inner_width/2+0.045:.3f} 0.035" pos="{center_x:.3f} 0 {inner_height+0.035:.3f}" rgba="0.50 0.50 0.46 1" contype="1" conaffinity="1"/>',
        f'      <geom name="container_back" type="box" size="0.045 {inner_width/2+0.045:.3f} {inner_height/2:.3f}" pos="{x0+inner_length+0.045:.3f} 0 {inner_height/2:.3f}" rgba="0.55 0.55 0.50 1" contype="1" conaffinity="1"/>',
        f'      <geom name="container_door_left_post" type="box" size="0.045 0.035 {inner_height/2:.3f}" pos="{x0-0.045:.3f} {inner_width/2+0.025:.3f} {inner_height/2:.3f}" rgba="0.36 0.34 0.30 1" contype="1" conaffinity="1"/>',
        f'      <geom name="container_door_right_post" type="box" size="0.045 0.035 {inner_height/2:.3f}" pos="{x0-0.045:.3f} {-inner_width/2-0.025:.3f} {inner_height/2:.3f}" rgba="0.36 0.34 0.30 1" contype="1" conaffinity="1"/>',
        f'      <geom name="container_door_top" type="box" size="0.045 {inner_width/2:.3f} 0.035" pos="{x0-0.045:.3f} 0 {inner_height+0.025:.3f}" rgba="0.36 0.34 0.30 1" contype="1" conaffinity="1"/>',
        f'      <geom name="container_outer_left" type="box" size="{inner_length/2:.3f} 0.025 {inner_height/2+0.035:.3f}" pos="{center_x:.3f} {inner_width/2+0.095:.3f} {inner_height/2:.3f}" rgba="0.20 0.50 0.24 0.45" contype="0" conaffinity="0" group="2"/>',
        f'      <geom name="container_outer_right" type="box" size="{inner_length/2:.3f} 0.025 {inner_height/2+0.035:.3f}" pos="{center_x:.3f} {-inner_width/2-0.095:.3f} {inner_height/2:.3f}" rgba="0.20 0.50 0.24 0.45" contype="0" conaffinity="0" group="2"/>',
        '    </body>',
        *cargo_lines,
        '    <body name="conveyor" pos="-0.5 3.0 0">',
        '      <geom name="conv_surface" type="box" size="1.0 0.35 0.03" pos="0 0 0.53" rgba="0.2 0.2 0.2 1" contype="1" conaffinity="1" friction="0.4 0.01 0.001"/>',
        '      <site name="conveyor_target" pos="0 0 0.58" size="0.05" rgba="0 1 1 0.5"/>',
        '    </body>',
        '    <geom name="robot_dock_mark" type="box" size="0.5 0.5 0.002" pos="-0.5 0 0.001" rgba="1 0.9 0 0.6" contype="0" conaffinity="0" group="2"/>',
        '  </worldbody>',
        '</mujoco>',
        '',
    ]
    return "\n".join(lines)


def generate_robot_only_xml() -> str:
    return "\n".join([
        '<?xml version="1.0" encoding="utf-8"?>',
        '<mujoco model="robot_only_scene_v5">',
        '  <compiler angle="radian" autolimits="true"/>',
        '  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic" noslip_iterations="3"/>',
        '  <include file="alfa_robot.xml"/>',
        '  <worldbody>',
        '    <light name="sun" pos="5 -5 8" dir="-0.4 0.4 -1" diffuse="0.9 0.9 0.85" specular="0.3 0.3 0.3" castshadow="true"/>',
        '    <light name="fill" pos="-3 -3 6" dir="0.3 0.3 -1" diffuse="0.4 0.4 0.5" specular="0.1 0.1 0.1"/>',
        '    <geom name="scene_floor" type="plane" size="15 15 0.1" material="grid_mat" pos="0 0 0" contype="1" conaffinity="1"/>',
        '  </worldbody>',
        '</mujoco>',
        '',
    ])


def main() -> None:
    root = render_urdf()
    ROBOT_XML.write_text(generate_robot_xml(root))
    SCENE_XML.write_text(generate_scene_xml())
    ROBOT_ONLY_XML.write_text(generate_robot_only_xml())
    print(f"wrote {ROBOT_XML}")
    print(f"wrote {SCENE_XML}")
    print(f"wrote {ROBOT_ONLY_XML}")


if __name__ == "__main__":
    main()
