#!/usr/bin/env python3
"""MOTION-334: the collision model before and after the patch, in viser.

Shows the robot meshes with the frozen collision model (spheres per link, the base cuboids) and the
patch (appended spheres, appended base cuboids) in its own color, at chosen frames of audited cycles,
together with the pick-side scene of the cycle's attach snapshot and the task box at its pick pose
(the box pose at the attach frame, as phase_audit.py places it; held at the tool from extract to
conveyor_place). The model can be switched between after the patch (appended parts highlighted),
before the patch, and the appended parts only.

  python model_viewer.py --frozen FROZEN_MOBILE18_YML --patched PATCHED_MOBILE18_YML \
      --frozen-cuboids frozen_cuboids.json --patched-cuboids cuboids.json \
      --pose LABEL RUN_JSON PHASE INDEX NOTE [--pose ...] [--port 8334]
"""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
import viser
from viser.extras import ViserUrdf
import yaml
import yourdfpy

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from curobo_core.adapter import pose_matrix
from curobo_core.scene import SceneSnapshot

FROZEN_COLOR, ADDED_COLOR = (71, 171, 210), (240, 135, 30)
MODES = ("改变后（新增部分标橙）", "改变前（冻结模型）", "只看新增部分")
HIDDEN_OBJECTS = {"ground", "ceiling", "left_wall", "right_wall"}


def wxyz(matrix):
    return np.roll(Rotation.from_matrix(matrix[:3, :3]).as_quat(), 1)


def arrays(balls):
    kept = [ball for ball in balls if ball["radius"] > 0]
    return (np.asarray([ball["center"] for ball in kept], dtype=float).reshape(-1, 3),
            np.asarray([ball["radius"] for ball in kept], dtype=float))


def spheres(config):
    return {link: arrays(balls) for link, balls in config["collision_spheres"].items()}


def appended(frozen, patched):
    """Spheres the patch appended per link (the patched list starts with the frozen one)."""
    out = {}
    for link, balls in patched["collision_spheres"].items():
        base = frozen["collision_spheres"].get(link, [])
        if balls[:len(base)] != base:
            raise SystemExit(f"{link}: the patched spheres do not start with the frozen ones")
        if len(balls) > len(base):
            out[link] = arrays(balls[len(base):])
    return out


def cuboids(path, names=None):
    rows = []
    for item in json.loads(Path(path).read_text())["cuboids"]:
        if names is None or item["name"] in names:
            pose = np.eye(4)
            pose[:3, :3] = Rotation.from_euler("xyz", item["rotation_deg"], degrees=True).as_matrix()
            pose[:3, 3] = item["center"]
            rows.append((item["name"], pose, np.asarray(item["dimensions"], dtype=float)))
    return rows


class ViewPose:
    """One frame of an audited cycle: joint values in the attach snapshot's world, scene objects, task boxes."""

    def __init__(self, label, run, phase, index, note, urdf, locked):
        result = json.loads(Path(run).read_text())
        event = next(item for item in result["predicted_scene_events"] if item["phase"] == "attach")
        snapshot = SceneSnapshot.from_dict(event["snapshot"])
        names, advance = result["joint_names"], float(result.get("base_advance_m", 0.))
        base = {**locked, **dict(zip(snapshot.state.joint_names, snapshot.state.positions))}

        def values(row):
            out = {**base, **dict(zip(names, row))}
            out["base_x"] -= advance
            return out

        frames = result["frames"]
        rows = [i for i, name in enumerate(result["phases"]) if name == phase]
        self.label, self.note = label, note
        self.values = values(frames[rows[int(index)]])
        held = phase in ("extract", "transport", "conveyor_place")  # the box moves with the tool after the attach
        holder = self.values if held else values(frames[event["frame_index"]])
        urdf.update_cfg({name: holder[name] for name in urdf.actuated_joint_names})
        self.tasks = [(item.object_id, urdf.get_transform(item.parent_link, urdf.base_link) @ pose_matrix(item.tool_to_object),
                       np.asarray(item.dimensions_m)) for item in snapshot.attachments]
        self.objects = [(item.object_id, pose_matrix(item.pose), np.asarray(item.dimensions_m))
                        for item in snapshot.objects if item.object_id not in HIDDEN_OBJECTS]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frozen", type=Path, required=True, help="frozen mobile18 robot config")
    parser.add_argument("--patched", type=Path, required=True, help="patched mobile18 robot config")
    parser.add_argument("--frozen-cuboids", type=Path, required=True)
    parser.add_argument("--patched-cuboids", type=Path, required=True, help="frozen + appended base cuboids")
    parser.add_argument("--pose", nargs=5, action="append", required=True, metavar=("LABEL", "RUN_JSON", "PHASE", "INDEX", "NOTE"))
    parser.add_argument("--port", type=int, default=8334)
    parser.add_argument("--initial-pose", help="label of the pose shown first (default: the first --pose)")
    parser.add_argument("--initial-view", default="整机", help="camera on connect: 整机, 右臂 link7, 左臂 link7 or 底座")
    args = parser.parse_args()

    frozen = yaml.safe_load(args.frozen.read_text())["kinematics"]
    patched = yaml.safe_load(args.patched.read_text())["kinematics"]
    urdf = yourdfpy.URDF.load(frozen["urdf_path"], load_meshes=True, build_scene_graph=True)
    locked = dict(frozen.get("lock_joints", {}))
    poses = [ViewPose(*item, urdf, locked) for item in args.pose]
    before, added = spheres(frozen), appended(frozen, patched)
    frozen_boxes = cuboids(args.frozen_cuboids)
    added_names = set(json.loads(args.patched_cuboids.read_text())["patch"]["added"])
    added_boxes = cuboids(args.patched_cuboids, added_names)

    server = viser.ViserServer(host="127.0.0.1", port=args.port, label="MOTION-334 · 碰撞模型补丁前后对比")
    server.scene.set_up_direction("+z")
    server.scene.add_grid("/grid", width=6.0, height=6.0)
    robot = ViserUrdf(server, urdf, root_node_name="/robot", mesh_color_override=(0.72, 0.76, 0.82, 0.4))
    sphere_mesh = trimesh.creation.icosphere(subdivisions=2, radius=1.0)
    link_frames, groups = {}, {"frozen": [], "added": []}
    for link in sorted(set(before) | set(added)):
        link_frames[link] = server.scene.add_frame(f"/model/{link}", show_axes=False)
        for key, table, color, opacity in (("frozen", before, FROZEN_COLOR, 0.3), ("added", added, ADDED_COLOR, 0.95)):
            if link in table and len(table[link][1]):
                centers, radii = table[link]
                groups[key].append(server.scene.add_batched_meshes_simple(
                    f"/model/{link}/{key}", vertices=sphere_mesh.vertices, faces=sphere_mesh.faces,
                    batched_positions=centers, batched_wxyzs=np.tile([1., 0., 0., 0.], (len(radii), 1)),
                    batched_scales=radii, batched_colors=color, opacity=opacity))
    base_frame = server.scene.add_frame("/model/base_cuboids", show_axes=False)
    for key, rows, color, opacity in (("frozen", frozen_boxes, FROZEN_COLOR, 0.12), ("added", added_boxes, ADDED_COLOR, 0.35)):
        for name, pose, dimensions in rows:
            groups[key].append(server.scene.add_box(f"/model/base_cuboids/{key}/{name}", dimensions=tuple(dimensions),
                                                    position=pose[:3, 3], wxyz=wxyz(pose), color=color, opacity=opacity))
    scene_nodes = []

    total_before = sum(len(r) for _, r in before.values())
    total_added = sum(len(r) for _, r in added.values())
    changed = ", ".join(f"{link} {len(before.get(link, ((), ()))[1])}→{len(before.get(link, ((), ()))[1]) + len(r)}"
                        for link, (_, r) in sorted(added.items()))
    with server.gui.add_folder("模型"):
        mode = server.gui.add_dropdown("显示", MODES, initial_value=MODES[0])
        pose_choice = server.gui.add_dropdown("姿态", tuple(pose.label for pose in poses),
                                              initial_value=args.initial_pose or poses[0].label)
        show_robot = server.gui.add_checkbox("机器人实体", initial_value=True)
        show_boxes = server.gui.add_checkbox("底座长方体", initial_value=True)
        show_scene = server.gui.add_checkbox("场景（待抓箱红色）", initial_value=True)
        server.gui.add_markdown(
            f"球：**{total_before} → {total_before + total_added}**（新增 {total_added}）  \n"
            f"{changed}  \n底座长方体：**{len(frozen_boxes)} → {len(frozen_boxes) + len(added_boxes)}**  \n"
            "蓝：冻结模型；橙：补丁新增")
        note = server.gui.add_markdown("")
    with server.gui.add_folder("视角"):
        views = {"整机": ((-2.2, 1.8, 1.0), "base_link"), "右臂 link7": ((0.15, -0.75, 0.35), "right_link7"),
                 "左臂 link7": ((0.15, 0.75, 0.35), "left_link7"), "底座": ((-1.6, 1.3, 0.9), "base_link")}
        buttons = {label: server.gui.add_button(label) for label in views}

    def apply_mode():
        state = {"frozen": mode.value != MODES[2], "added": mode.value != MODES[1]}
        for key, handles in groups.items():
            for handle in handles:
                handle.visible = state[key]
        base_frame.visible = show_boxes.value

    def apply_pose():
        pose = next(item for item in poses if item.label == pose_choice.value)
        urdf.update_cfg({name: pose.values[name] for name in urdf.actuated_joint_names})
        robot.update_cfg(np.asarray([pose.values[name] for name in urdf.actuated_joint_names]))
        for link, frame in link_frames.items():
            matrix = urdf.get_transform(link, urdf.base_link)
            frame.position, frame.wxyz = matrix[:3, 3], wxyz(matrix)
        matrix = urdf.get_transform("base_link", urdf.base_link)
        base_frame.position, base_frame.wxyz = matrix[:3, 3], wxyz(matrix)
        for handle in scene_nodes:
            handle.remove()
        scene_nodes.clear()
        for name, matrix, dimensions in pose.objects:
            scene_nodes.append(server.scene.add_box(f"/scene/{name}", dimensions=tuple(dimensions), position=matrix[:3, 3],
                                                   wxyz=wxyz(matrix), color=(170, 170, 170), opacity=0.18))
        for name, matrix, dimensions in pose.tasks:
            scene_nodes.append(server.scene.add_box(f"/scene/task/{name}", dimensions=tuple(dimensions), position=matrix[:3, 3],
                                                   wxyz=wxyz(matrix), color=(220, 60, 60), opacity=0.45))
        for handle in scene_nodes:
            handle.visible = show_scene.value
        note.content = pose.note

    def camera(label):
        offset, link = views[label]
        target = urdf.get_transform(link, urdf.base_link)[:3, 3] + (np.array([0., 0., 0.9]) if label == "整机" else 0.)
        return tuple(target + np.asarray(offset)), tuple(target)

    def look(client, label):
        client.camera.position, client.camera.look_at = camera(label)
        client.camera.up_direction = (0., 0., 1.)

    mode.on_update(lambda _: apply_mode())
    show_boxes.on_update(lambda _: apply_mode())
    pose_choice.on_update(lambda _: apply_pose())
    show_robot.on_update(lambda _: setattr(robot, "show_visual", show_robot.value))
    show_scene.on_update(lambda _: [setattr(handle, "visible", show_scene.value) for handle in scene_nodes])
    for label, button in buttons.items():
        button.on_click(lambda event, label=label: look(event.client, label))
    apply_pose()
    apply_mode()
    server.initial_camera.position, server.initial_camera.look_at = camera(args.initial_view)
    server.initial_camera.up = (0., 0., 1.)
    print(f"Ready at http://localhost:{args.port}; spheres {total_before}->{total_before + total_added}, "
          f"cuboids {len(frozen_boxes)}->{len(frozen_boxes) + len(added_boxes)}", flush=True)
    while True:
        time.sleep(1.)


if __name__ == "__main__":
    main()
