#!/usr/bin/env python3
"""MOTION-312: CPU-only replay of recorded full-cycle results (`full/*.json` of the A/B runs).

Results of one case (for example joint_rrt and cartesian_v2_toppra with the same task, offset and
seed) are overlaid on one robot base, each in its own colour, with its carried box and the tool path
of its transport phase. One time slider drives all of them on their recorded timing; "align
transport" shifts each result so that the transport phases start together. With several --cases a
dropdown switches between them, and each arm can be hidden. Scene boxes come from the attach
snapshot (shifted into the map frame by base_advance_m). Failed runs show the frames recorded up to
the blocking stage. Needs no cuRobo planner and no GPU.

  python replay.py --runtime <runtime>/research/curobo_v3 --results-dir <set>/full \
      --cases right22-o0-s20261015,right20-o0-s20261015 --arms joint_rrt,cartesian_v2_toppra --port 8091
"""
import argparse
import json
from pathlib import Path
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation
import viser
from viser.extras import ViserUrdf
import yaml
import yourdfpy

COLORS = ((0.62, 0.68, 0.78, 0.55), (0.93, 0.55, 0.20, 0.85), (0.30, 0.70, 0.45, 0.85), (0.75, 0.35, 0.75, 0.85))
COLOR_NAMES = ("灰", "橙", "绿", "紫")
ARM_LABELS = {"joint_rrt": "基线（关节 RRT）", "cartesian_toppra": "笛卡尔 v1", "cartesian_v2_toppra": "笛卡尔 v2",
              "cartesian_v21_toppra": "A：笛卡尔 + 回退", "cartesian_v21r_toppra": "B：笛卡尔 + 换分支路由 + 回退"}
TIER_LABELS = {"direct_line": "笛卡尔直线", "rrt": "笛卡尔 RRT", "joint_rrt_routed": "换分支，路由给关节 RRT",
               "joint_rrt_fallback": "笛卡尔无解，回退关节 RRT"}
HIDDEN_BY_DEFAULT = ("ceiling", "ground")
MOBILE = "generated/v3_analytic_071cb95/mobile18/alfa_v322_suction_mobile18.yml"


def pose_matrix(pose):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(np.roll(pose["quaternion_wxyz"], -1)).as_matrix()
    matrix[:3, 3] = pose["position"]
    return matrix


def wxyz(matrix):
    return tuple(np.roll(Rotation.from_matrix(matrix[:3, :3]).as_quat(), 1))


class Track:
    """One recorded result: its robot, carried box, transport tool path and timing."""

    def __init__(self, server, urdf_path, result, label, color, index):
        self.result, self.label = result, label
        self.names, self.frames = result["joint_names"], np.asarray(result["frames"], dtype=float)
        self.phases, self.payload = result["phases"], result["payload"]
        count = len(self.frames)
        times = result.get("time_from_start_s") or []
        self.times = np.asarray(times if len(times) == count else np.arange(count) * 0.025, dtype=float)
        self.urdf = yourdfpy.URDF.load(urdf_path, load_meshes=True, build_scene_graph=True)
        self.robot = ViserUrdf(server, self.urdf, root_node_name=f"/robots/{index}", mesh_color_override=color)
        event = next((item for item in result.get("predicted_scene_events", []) if item["phase"] == "attach"), None)
        self.attachments = event["snapshot"]["attachments"] if event else []
        self.boxes = [server.scene.add_box(f"/payload/{index}/{item['object_id']}", dimensions=item["dimensions_m"],
                                           color=tuple(int(255 * c) for c in color[:3]), opacity=0.9, visible=False)
                      for item in self.attachments]
        transport = [i for i, phase in enumerate(self.phases) if phase == "transport"]
        self.transport_start = self.times[transport[0]] if transport else None
        self.offset, self.extra_rotation, self.path, self.visible = 0., None, None, True
        if transport and self.attachments:
            item = self.attachments[0]
            points, rotations = [], []
            for i in transport:
                self.set_cfg(i, robot=False)
                tool = self.urdf.get_transform(item["parent_link"], self.urdf.base_link)
                points.append(tool[:3, 3])
                rotations.append((tool @ pose_matrix(item["tool_to_object"]))[:3, :3])
            rotations = Rotation.from_matrix(np.asarray(rotations))
            total = np.degrees((rotations[:-1].inv() * rotations[1:]).magnitude()).sum()
            self.extra_rotation = total - np.degrees((rotations[0].inv() * rotations[-1]).magnitude())
            self.path = server.scene.add_spline_catmull_rom(f"/paths/{index}", positions=np.asarray(points[::4] + points[-1:]),
                                                            color=tuple(int(255 * c) for c in color[:3]), line_width=3.)

    def set_visible(self, visible):
        self.visible = visible
        self.robot.show_visual = visible
        if self.path is not None:
            self.path.visible = visible

    def remove(self):
        self.robot.remove()
        for node in self.boxes + ([self.path] if self.path is not None else []):
            node.remove()

    def set_cfg(self, index, robot=True):
        values = dict(zip(self.names, self.frames[index]))
        cfg = np.array([values.get(name, 0.) for name in self.urdf.actuated_joint_names])
        self.urdf.update_cfg(cfg)
        if robot:
            self.robot.update_cfg(cfg)

    def show(self, t):
        if not len(self.frames):
            return "无轨迹帧"
        index = int(np.clip(np.searchsorted(self.times, t - self.offset, side="right") - 1, 0, len(self.frames) - 1))
        self.set_cfg(index)
        for node, item in zip(self.boxes, self.attachments):
            matrix = self.urdf.get_transform(item["parent_link"], self.urdf.base_link) @ pose_matrix(item["tool_to_object"])
            node.position, node.wxyz = matrix[:3, 3], wxyz(matrix)
            node.visible = self.visible and bool(self.payload[index])
        return f"帧 {index + 1}/{len(self.frames)} · {self.phases[index]} · {self.times[index]:.2f}/{self.times[-1]:.2f} s"


def summary(track):
    result = track.result
    segment = {item["name"]: item for item in result.get("timed_segments", [])}
    parts = ["成功" if result["success"] else f"失败：{result.get('blocked_stage')}（{result.get('blocked_details')}）"]
    tier = (result.get("transport_search_stats") or {}).get("tier")
    if tier in TIER_LABELS:
        parts.append(TIER_LABELS[tier])
    if "transport" in segment:
        parts.append(f"transport {segment['transport']['duration_s']:.2f} s")
    if result.get("motion_duration_s"):
        parts.append(f"整轮运动 {result['motion_duration_s']:.1f} s")
    if track.extra_rotation is not None:
        parts.append(f"额外旋转 {track.extra_rotation:.1f}°")
    return " · ".join(parts)


def scene_from(server, results):
    """Scene boxes of the first result with an attach event (map frame); returns name -> handle."""
    first = next((result for result in results if any(item["phase"] == "attach" for item in result.get("predicted_scene_events", []))),
                 results[0])
    event = next((item for item in first.get("predicted_scene_events", []) if item["phase"] == "attach"), None)
    nodes = {}
    if event:
        advance = float(first.get("base_advance_m", 0.))
        carried = {item["object_id"] for item in event["snapshot"]["attachments"]}
        for item in event["snapshot"]["objects"]:
            if item["object_id"] in carried:
                continue
            matrix = pose_matrix(item["pose"])
            matrix[0, 3] += advance
            nodes[item["object_id"]] = server.scene.add_box(
                f"/scene/{item['object_id']}", dimensions=item["dimensions_m"], position=matrix[:3, 3], wxyz=wxyz(matrix),
                color=(149, 160, 174), opacity=0.12, visible=item["object_id"] not in HIDDEN_BY_DEFAULT)
    return nodes


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--runtime", type=Path, required=True, help="research/curobo_v3 of a prepared runtime (for the URDF)")
    parser.add_argument("--result", type=Path, action="append", default=[], help="full/<run>.json, repeatable (one custom case)")
    parser.add_argument("--results-dir", type=Path, help="full/ directory, used with --case/--cases and --arms")
    parser.add_argument("--case", help="run name without the arm suffix, e.g. right22-o0-s20261015")
    parser.add_argument("--cases", help="comma separated run names without the arm suffix; a dropdown switches between them")
    parser.add_argument("--arms", default="joint_rrt,cartesian_v2_toppra")
    parser.add_argument("--port", type=int, default=8091)
    args = parser.parse_args()
    names = ([args.case] if args.case else []) + (args.cases.split(",") if args.cases else [])
    if names and not args.results_dir:
        parser.error("--case/--cases need --results-dir")
    arms = args.arms.split(",")
    cases = {name: [args.results_dir / f"{name}-{arm}.json" for arm in arms] for name in names}
    if args.result:
        cases["自选结果"] = list(args.result)
    if not cases:
        parser.error("give --result or --results-dir with --case/--cases")
    urdf_path = yaml.safe_load((args.runtime / MOBILE).read_text())["kinematics"]["urdf_path"]
    server = viser.ViserServer(host="127.0.0.1", port=args.port, label="MOTION-312 回放")
    server.scene.set_up_direction("+z")
    server.scene.add_grid("/grid", width=6, height=4)
    state = dict(tracks=[], scene={}, legend="")
    lock = threading.Lock()  # the playback thread and viser's callback workers both render
    with server.gui.add_folder("回放"):
        case = server.gui.add_dropdown("案例", options=list(cases), initial_value=next(iter(cases)))
        slider = server.gui.add_slider("时间 (s)", min=0., max=1., step=0.025, initial_value=0.)
        playing = server.gui.add_checkbox("播放", initial_value=False)
        speed = server.gui.add_slider("倍速", min=0.1, max=10., step=0.1, initial_value=1.)
        align = server.gui.add_checkbox("transport 起点对齐", initial_value=False)
        walls = server.gui.add_checkbox("显示顶棚/地面", initial_value=False)
        shown = [server.gui.add_checkbox(f"显示{COLOR_NAMES[i % 4]}：{ARM_LABELS.get(arm, arm)}", initial_value=True)
                 for i, arm in enumerate(arms if names else [path.stem for path in args.result])]
        status = server.gui.add_markdown("")

    def render(_=None):
        with lock:
            lines = [f"{COLOR_NAMES[i % 4]} {track.label}：{track.show(slider.value)}" for i, track in enumerate(state["tracks"])]
            status.content = state["legend"] + "\n\n" + "  \n".join(lines)

    def update_offsets():
        tracks = state["tracks"]
        starts = [track.transport_start for track in tracks if track.transport_start is not None]
        latest = max(starts, default=0.)
        for track in tracks:  # runs without a transport phase keep their own timing
            track.offset = latest - track.transport_start if align.value and track.transport_start is not None else 0.
        slider.max = float(max((track.times[-1] + track.offset for track in tracks if len(track.times)), default=1.))

    def load(name):
        with lock:
            for track in state["tracks"]:
                track.remove()
            for node in state["scene"].values():
                node.remove()
            paths = [path for path in cases[name] if path.exists()]
            results = [json.loads(path.read_text()) for path in paths]
            state["scene"] = scene_from(server, results) if results else {}
            for node_name in HIDDEN_BY_DEFAULT:
                if node_name in state["scene"]:
                    state["scene"][node_name].visible = walls.value
            state["tracks"] = [Track(server, urdf_path, result, ARM_LABELS.get(path.stem.removeprefix(f"{name}-"), path.stem),
                                     COLORS[i % len(COLORS)], i) for i, (path, result) in enumerate(zip(paths, results))]
            for track, box in zip(state["tracks"], shown):
                track.set_visible(box.value)
            state["legend"] = f"**{name}**  \n" + "  \n".join(
                f"{COLOR_NAMES[i % 4]}：**{track.label}** — {summary(track)}" for i, track in enumerate(state["tracks"]))
        update_offsets()
        slider.value = 0.
        render()

    @case.on_update
    def _(_):
        load(case.value)

    @align.on_update
    def _(_):
        update_offsets()
        render()

    @walls.on_update
    def _(_):
        for name in HIDDEN_BY_DEFAULT:
            if name in state["scene"]:
                state["scene"][name].visible = walls.value

    for index, box in enumerate(shown):
        def toggle(_, index=index, box=box):
            if index < len(state["tracks"]):
                state["tracks"][index].set_visible(box.value)
                render()
        box.on_update(toggle)

    slider.on_update(render)
    load(case.value)

    def loop():
        last = time.perf_counter()
        while True:
            time.sleep(0.03)
            now = time.perf_counter()
            if playing.value:
                value = slider.value + (now - last) * speed.value
                try:
                    slider.value = 0. if value > slider.max else value
                except Exception as error:  # keep playing after a failed render
                    print(f"render failed: {error}", flush=True)
            last = now

    threading.Thread(target=loop, daemon=True).start()
    print(f"replay at http://127.0.0.1:{args.port}", flush=True)
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    main()
