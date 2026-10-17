#!/usr/bin/env python3
"""Read-only Viser replay of recorded rolling replanning; no CUDA or hardware IO."""
import argparse
import bisect
import json
from pathlib import Path
import time

import numpy as np
import yourdfpy
import viser
from viser.extras import ViserUrdf

from curobo_core.adapter import matrix_pose, pose_matrix
from curobo_core.contracts import PlannerAssets
from curobo_core.scene import SceneSnapshot
from curobo_core.trajectory import TimedTrajectory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, default=PlannerAssets().urdf)
    parser.add_argument("--port", type=int, default=8099)
    args = parser.parse_args()
    report = json.loads(args.result.read_text())
    if report.get("source") != "wall_clock_simulation" or "started_at" not in report:
        parser.error("record must contain an initialized rolling simulation")
    started = report["started_at"]
    ended = report.get("completed_at", max((e["at"] for e in report["events"]), default=started))
    duration = max(.001, ended-started)
    entries = [{"active_at": started, "trajectory": report["initial_trajectory"]}, *report["trajectories"]]
    curves = [TimedTrajectory(x["trajectory"]) for x in entries]
    epochs = [x["active_at"] for x in entries]
    observations = report["observations"]
    observation_times = [x["received_at"] for x in observations]
    snapshot = SceneSnapshot.from_dict(report["initial_snapshot"])
    urdf = yourdfpy.URDF.load(args.urdf, load_meshes=True, build_scene_graph=True)
    server = viser.ViserServer(host="127.0.0.1", port=args.port)
    robot = ViserUrdf(server, urdf, root_node_name="/robot")
    base_transform = pose_matrix(snapshot.state.base_pose)
    server.scene.add_frame("/robot", position=snapshot.state.base_pose.position,
                           wxyz=snapshot.state.base_pose.quaternion_wxyz, show_axes=False)
    nodes = {}
    for obj in snapshot.objects:
        if obj.object_id == "ground":
            continue
        nodes[obj.object_id] = server.scene.add_box(
            f"/world/{obj.object_id}", dimensions=obj.dimensions_m, position=obj.pose.position,
            wxyz=obj.pose.quaternion_wxyz, color=(110, 150, 170),
            opacity=.25 if obj.object_id.startswith("wall_box_") else .06)
    for obj in snapshot.meshes:
        server.scene.add_mesh_simple(f"/meshes/{obj.object_id}", vertices=np.asarray(obj.vertices_m),
                                     faces=np.asarray(obj.faces), position=obj.pose.position,
                                     wxyz=obj.pose.quaternion_wxyz, color=(140, 150, 160), opacity=.3)
    payloads = [(item, server.scene.add_box(f"/payload/{item.object_id}", dimensions=item.dimensions_m,
                                          color=(235, 180, 100), opacity=.7)) for item in snapshot.attachments]
    obstacle = server.scene.add_box("/dynamic/rolling_obstacle", dimensions=(.08, .08, .08),
                                    color=(235, 65, 65), opacity=.85, visible=False)
    # Same FK/Viser primitives as the baseline viewer; cache paths only for display.
    paths = []
    for curve in curves:
        positions = curve.sample(np.linspace(0, curve.duration, 40))["positions"]
        points = []
        for row in positions:
            urdf.update_cfg(dict(zip(curve.joint_names, row)))
            points.append((base_transform @ urdf.get_transform("left_tool0", urdf.base_link))[:3, 3].copy())
        points = np.asarray(points)
        paths.append(np.stack((points[:-1], points[1:]), axis=1))
    server.scene.add_line_segments("/path/original", points=paths[0], colors=(160, 160, 160), line_width=2.)
    current_path = server.scene.add_line_segments("/path/active", points=paths[0], colors=(35, 200, 180), line_width=4.)
    server.gui.add_markdown("## MOTION-292 · 只读回放\n灰线：原路线；绿线：当前路线；红箱：动态障碍。\n\n**模拟，不代表实机安全。**")
    server.gui.add_markdown(f"结果：{'完成' if report['success'] else '拒绝/中止'} · "
                            f"接续 {report['splice_count']} 次 · 直接优化接续 {report.get('direct_splice_count', '旧记录未区分')} 次 · "
                            f"减速后规划 {report.get('braking_replan_count', 0)} 次（含内部停稳，非保护停机） · "
                            f"复核旧路接续 {report.get('revalidated_splice_count', '旧记录未区分')} 次\n\n{report.get('error', '')}")
    play = server.gui.add_checkbox("播放", initial_value=False)
    speed = server.gui.add_slider("倍率", min=.1, max=3., step=.1, initial_value=1.)
    slider = server.gui.add_slider("时间 / s", min=0., max=duration, step=.01, initial_value=0.)
    status = server.gui.add_markdown("")
    joint = server.gui.add_dropdown("关节", options=curves[0].joint_names, initial_value="left_joint1")
    samples = report["samples"]
    times = np.array([x["at"]-started for x in samples])
    plot = server.gui.add_uplot(
        data=(times, np.zeros(len(times)), np.zeros(len(times))),
        series=(viser.uplot.Series(label="t"), viser.uplot.Series(label="q", stroke="#35aacb"),
                viser.uplot.Series(label="qdot", stroke="#ed9745")), scales={"x": {"time": False}},
        title="实际模拟执行采样（秒；不是规划节点）")
    with server.gui.add_folder("编排与延迟", expand_by_default=True):
        budget = report["schedule"]
        server.gui.add_markdown(f"周期 **{budget['duration_s']/budget['slices']:.3f}s** · "
                                f"t2预算 **{budget['compute_bound_s']:.3f}s** · "
                                f"t3假定 **{budget['link_bound_s']:.3f}s**")
        timeline = server.gui.add_markdown("")
    selected = None
    previous = time.perf_counter()
    while True:
        now = time.perf_counter()
        if play.value:
            slider.value = min(duration, slider.value + (now-previous)*speed.value)
            if slider.value >= duration:
                play.value = False
        previous = now
        absolute = started + slider.value
        index = max(0, bisect.bisect_right(epochs, absolute)-1)
        curve = curves[index]
        state = curve.sample(np.clip(absolute-epochs[index], 0., curve.duration))
        cfg = dict(zip(curve.joint_names, state["positions"]))
        urdf.update_cfg(cfg); robot.update_cfg(cfg)
        current_path.points = paths[index]
        for attachment, node in payloads:
            pose = matrix_pose(base_transform @ urdf.get_transform(attachment.parent_link, urdf.base_link) @
                               pose_matrix(attachment.tool_to_object))
            node.position, node.wxyz = pose.position, pose.quaternion_wxyz
        obs_index = bisect.bisect_right(observation_times, absolute)-1
        if obs_index >= 0:
            obs = observations[obs_index]
            obj = next((x for x in obs["snapshot"]["objects"] if x["object_id"] == "rolling_obstacle"), None)
            obstacle.visible = obj is not None
            if obj:
                velocity = dict(obs["velocities"])["rolling_obstacle"]
                obstacle.position = np.asarray(obj["pose"]["position"]) + np.asarray(velocity)*(absolute-obs["observed_at"])
        status.content = (f"**t={slider.value:.3f}s · 已接续 {index} 次**\n\n"
                          f"场景版本：{observations[obs_index]['snapshot']['revision'] if obs_index >= 0 else snapshot.revision}\n\n"
                          f"当前最大关节速度：{np.max(np.abs(state['velocities'])):.4f}（rad/s；updown 为 m/s）")
        recent = [event for event in report["events"] if event["at"] <= absolute][-5:]
        labels = {"requested": "发起计算", "staged": "候选已接收", "activated": "按时接续",
                  "safety_required": "安全处置请求", "terminal_certificate_reused": "尾段预测复核通过"}
        lines = [f"- {event['at']-started:.3f}s · {labels.get(event['event'], event['event'])}"
                 + (f" · 观测→可用 {event['available_latency_s']*1000:.1f}ms"
                    if "available_latency_s" in event else "") for event in recent]
        timeline.content = "\n".join(lines) or "尚未开始"
        if joint.value != selected:
            selected = joint.value; j = curves[0].joint_names.index(selected)
            plot.data = (times, np.array([x["positions"][j] for x in samples]),
                         np.array([x["velocities"][j] for x in samples]))
        time.sleep(.03)


if __name__ == "__main__":
    main()
