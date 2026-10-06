import argparse
import json
from pathlib import Path
import threading
import time

import numpy as np
import trimesh
import viser
from viser.extras import ViserUrdf
import yourdfpy

from v3_full_cycle_planner import FullCyclePlanner, CycleBlocked
from curobo_core.fixtures import WallLayout, add_wall_arguments, wall_layout_from_args
from v3_interactive_60_tasks import (
    DEFAULT_ROBOT_CONFIG, DEFAULT_URDF, DEFAULT_NAMED_POSES, DEFAULT_BOX_FIT,
    DEFAULT_MOBILE_ROBOT_CONFIG, quaternion,
)
from v3_wall_ik_benchmark import canonical_side_tool_to_box, wall_center


PHASE_NAMES = {
    "home": "第一初始姿态", "approach": "初始→吸附起点", "attach": "吸附箱体",
    "extract": "固定Updown解析直线抽离35cm", "transport": "抽离终点→放置姿态",
    "turn_loaded": "携箱转身180°", "release": "放置释放，箱体消失",
    "turn_empty": "空载转身回来", "return_home": "双臂回到原第一初始姿态",
    "transport_diagnostic": "失败轨迹诊断",
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-config", type=Path, default=DEFAULT_ROBOT_CONFIG)
    parser.add_argument("--mobile-robot-config", type=Path, default=DEFAULT_MOBILE_ROBOT_CONFIG)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--named-poses", type=Path, default=DEFAULT_NAMED_POSES)
    parser.add_argument("--box-fit", type=Path, default=DEFAULT_BOX_FIT)
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--comparison-json", type=Path, nargs="+")
    add_wall_arguments(parser)
    args = parser.parse_args()
    try:
        args.wall_layout = wall_layout_from_args(args)
    except (ValueError, TypeError) as error:
        parser.error(str(error))
    if args.comparison_json and args.wall_layout != WallLayout():
        parser.error("preloaded comparisons belong to the default wall; recompute a custom layout")
    comparisons = {}
    for path in args.comparison_json or []:
        comparisons.update(json.loads(path.read_text()))
    planner_labels = {"Batched RRT": "rrt", "Informed RRT": "informed_rrt",
                      "Batched RRTConnect": "rrtconnect", "GPU PRM": "prm",
                      "Connect + Informed": "informed_connect", "BIT* + GPU": "bitstar"}
    planner = FullCyclePlanner(args)
    config = planner.mobile_robot["kinematics"]
    joint_names = config["cspace"]["joint_names"]
    urdf = yourdfpy.URDF.load(config["urdf_path"], load_meshes=True, build_scene_graph=True)
    server = viser.ViserServer(host="127.0.0.1", port=args.port,
                               label="V3解析代理 · 抓取抽离放置回Home完整流程")
    server.scene.set_up_direction("+z")
    server.scene.add_grid("/ground", width=5, height=3)
    robot = ViserUrdf(server, urdf, root_node_name="/robot",
                      mesh_color_override=(0.68, 0.73, 0.80, 0.8))
    attached = {}
    sphere_mesh = trimesh.creation.icosphere(subdivisions=2)
    for side in ("left", "right"):
        attached[side] = server.scene.add_frame(f"/attached/{side}", show_axes=False, visible=False)
        server.scene.add_box(f"/attached/{side}/box", dimensions=planner.box_fit["dimensions_m"],
                             color=(239, 146, 62), opacity=0.8)
        server.scene.add_frame(f"/attached/{side}/spheres", show_axes=False, visible=False)
        for sphere_index, (center, radius) in enumerate(zip(planner.box_fit["centers"], planner.box_fit["radii"])):
            server.scene.add_mesh_simple(
                f"/attached/{side}/spheres/{sphere_index}",
                vertices=sphere_mesh.vertices*radius, faces=sphere_mesh.faces,
                position=center, color=(61, 170, 210), opacity=0.5)
    source_boxes = {}
    for item in planner.snapshot.objects:
        if not item.object_id.startswith("wall_box_"):
            continue
        box_id = int(item.object_id.removeprefix("wall_box_"))
        source_boxes[box_id] = server.scene.add_box(
            f"/wall/box_{box_id}", position=item.pose.position,
            wxyz=item.pose.quaternion_wxyz, dimensions=item.dimensions_m,
            color=(87, 145, 165), opacity=.25)
    for item in planner.snapshot.objects:
        if not item.object_id.startswith("wall_box_") and item.object_id != "ground":
            server.scene.add_box(f"/container/{item.object_id}", dimensions=item.dimensions_m,
                                 position=item.pose.position, wxyz=item.pose.quaternion_wxyz,
                                 color=(149, 160, 174), opacity=.06)
    labels = [entry[0] for entry in planner.task_options]
    with server.gui.add_folder("完整任务实时规划"):
        layout = planner.wall_layout
        server.gui.add_markdown(
            f"箱墙：**{layout.rows}×{layout.columns}** · 实箱 **{len(layout.active_box_ids)}**  \n"
            f"初始车头→箱墙近面：**{layout.distance_m:.3f} m**  \n"
            "仅当前双箱任务；研究碰撞策略，不代表整墙或实机验收。"
        )
        task = server.gui.add_dropdown("组合任务", options=labels, initial_value=labels[0])
        planner_selector = server.gui.add_dropdown("规划器", options=list(planner_labels),
                                                   initial_value="Informed RRT")
        run = server.gui.add_button("计算完整流程并播放")
        play = server.gui.add_checkbox("播放", initial_value=False)
        frame = server.gui.add_slider("轨迹帧", min=1, max=1, step=1, initial_value=1)
        stage = server.gui.add_dropdown("查看阶段", options=list(PHASE_NAMES.values()),
                                        initial_value=PHASE_NAMES["home"])
        spheres = server.gui.add_checkbox("显示附着箱40球", initial_value=False)
        status = server.gui.add_markdown("从第一初始姿态开始，一次计算完整流程。")
        comparison_status = server.gui.add_markdown("")
    current = {"result": None, "busy": False}

    def refresh_comparison():
        lines = ["已计算完整流程对比（计算结果回放）", "", "| 规划器 | 结果 | 总计算 |",
                 "| --- | --- | --- |"]
        for label, kind in planner_labels.items():
            result = comparisons.get(kind)
            if result is None:
                continue
            verdict = "完成全部阶段" if result["success"] else result.get("blocked_stage", "失败")
            seconds = result.get("measured_wall_ms", result.get("total_ms", 0)) / 1000
            lines.append(f"| {label} | {verdict} | {seconds:.2f}s |")
        lines += ["", "预载首组对比为热态。实时重算包含当次初始化状态；同一接触起点、每段2秒预算；无Shortcut/TrajOpt。"]
        comparison_status.content = "\n".join(lines) if comparisons else ""

    def show_frame(index):
        result = current["result"]
        if result is None:
            values = {**planner.home, "base_x": 0., "base_y": 0., "base_yaw": 0.}
            phase = "home"
            has_payload = False
        else:
            values = dict(zip(result["joint_names"], result["frames"][index]))
            phase = result["phases"][index]
            has_payload = result["payload"][index]
        cfg = np.array([values.get(name, 0.) for name in urdf.actuated_joint_names])
        urdf.update_cfg(cfg)
        robot.update_cfg(cfg)
        for side, node in attached.items():
            matrix = urdf.get_transform(side+"_tool0", urdf.base_link) @ canonical_side_tool_to_box(side)
            node.position = matrix[:3, 3]
            node.wxyz = quaternion(matrix)
            node.visible = has_payload
        for box_id, box in source_boxes.items():
            box.visible = box_id not in planner.selected_task.values() or phase in ("home", "approach")
        if result:
            status.content = (
                f"**{'完整流程成功' if result['success'] else '已停止：存在阻塞'} · {PHASE_NAMES[phase]}**  \n"
                f"帧：**{index+1}/{len(result['frames'])}**  \n"
                f"接触IK：**{result.get('contact_ik_count', 0)}** · 选中候选：**{result.get('selected_candidate', '无')}**  \n"
                f"全部规划：**{result.get('total_ms', 0)/1000:.3f}s**  \n"
                f"最终Home误差：**{result.get('final_home_error', 0):.8f}**"
            )
            if not result["success"]:
                status.content += f"  \n{result.get('blocked_stage')}: {result.get('blocked_details')}"

    @frame.on_update
    def on_frame(_event):
        if current["result"]:
            show_frame(int(frame.value)-1)

    @stage.on_update
    def on_stage(_event):
        if not current["result"]:
            return
        selected = next(key for key, label in PHASE_NAMES.items() if label == stage.value)
        phases = current["result"]["phases"]
        if selected in phases:
            play.value = False
            frame.value = phases.index(selected)+1

    @spheres.on_update
    def on_spheres(_event):
        for side in ("left", "right"):
            server.scene.add_frame(f"/attached/{side}/spheres", show_axes=False,
                                    visible=spheres.value)

    @task.on_update
    def on_task(_event):
        if current["busy"]:
            return
        play.value = False
        current["result"] = None
        planner.selected_task = planner.task_map[task.value]
        comparisons.clear()
        planner.comparison_contact = None
        refresh_comparison()
        frame.max = 1
        frame.value = 1
        show_frame(0)

    @planner_selector.on_update
    def on_planner(_event):
        if current["busy"]:
            return
        play.value = False
        planner.planner_kind = planner_labels[planner_selector.value]
        result = comparisons.get(planner.planner_kind)
        current["result"] = result
        frame.max = max(1, len(result["frames"])) if result else 1
        frame.value = 1
        show_frame(0)
        if result:
            play.value = result["success"]

    @run.on_click
    def on_run(_event):
        if current["busy"]:
            return
        current["busy"] = True
        run.disabled = True
        task.disabled = True
        planner_selector.disabled = True
        play.value = False
        current["result"] = None
        show_frame(0)

        def worker():
            try:
                result = planner.full_cycle(planner.selected_task,
                                            lambda text: setattr(status, "content", text))
            except CycleBlocked as error:
                result = error.partial or planner.last_partial
                result["blocked_stage"] = error.stage
                result["blocked_details"] = error.details
            except Exception as error:
                result = planner.last_partial or {"success": False, "frames": []}
                result["blocked_stage"] = "程序异常"
                result["blocked_details"] = f"{type(error).__name__}: {error}"
            finally:
                current["busy"] = False
                run.disabled = False
                task.disabled = False
                planner_selector.disabled = False
            path = args.robot_config.parent / "full_cycle_latest.json"
            path.write_text(json.dumps(result, indent=2, ensure_ascii=False)+"\n")
            current["result"] = result
            comparisons[planner.planner_kind] = result
            refresh_comparison()
            if result["frames"]:
                frame.max = len(result["frames"])
                frame.value = 1
                show_frame(0)
                play.value = result["success"]
            else:
                status.content = f"**已停止** {result.get('blocked_stage')}: {result.get('blocked_details')}"
        threading.Thread(target=worker, daemon=True).start()

    @server.on_client_connect
    def on_client(client):
        client.camera.position = (-3.2, 3.4, 2.8)
        client.camera.look_at = (.6, 0, 1.2)
        client.camera.up_direction = (0, 0, 1)

    if comparisons:
        baseline = comparisons.get("informed_rrt")
        if baseline and baseline.get("selected_candidate"):
            planner.comparison_contact = baseline["contact_candidates"][baseline["selected_candidate"]-1]
        current["result"] = baseline
        frame.max = len(baseline["frames"]) if baseline else 1
    refresh_comparison()
    show_frame(0)
    print(f"Ready: http://localhost:{args.port}", flush=True)
    while True:
        if play.value and current["result"]:
            if int(frame.value) >= len(current["result"]["frames"]):
                play.value = False
            else:
                frame.value = int(frame.value)+1
        time.sleep(.03)


if __name__ == "__main__":
    main()
