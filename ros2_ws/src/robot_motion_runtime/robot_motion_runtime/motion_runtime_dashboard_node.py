from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import urlparse

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from robot_motion_interfaces.msg import RobotMotionScene, RobotMotionState
from robot_motion_runtime.common import joint_state_positions, now_ms


DASHBOARD_HTML = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Robot Motion Runtime Dashboard</title>
  <style>
    :root{--bg:#08111f;--card:#111c2f;--line:#22324d;--text:#edf6ff;--muted:#95a8c3;--ok:#6ee7b7;--warn:#fdba74;--bad:#fca5a5;--blue:#93c5fd;--purple:#c4b5fd}
    *{box-sizing:border-box} body{margin:0;background:radial-gradient(circle at 20% 0%,#164e6355,transparent 30rem),linear-gradient(135deg,#08111f,#0d1630);color:var(--text);font-family:Inter,system-ui,-apple-system,"Segoe UI","Microsoft YaHei",sans-serif}
    header{position:sticky;top:0;z-index:2;padding:18px clamp(16px,4vw,44px);display:flex;justify-content:space-between;gap:16px;align-items:center;background:#08111fcc;border-bottom:1px solid var(--line);backdrop-filter:blur(14px)}
    h1{margin:0;font-size:clamp(24px,4vw,42px);letter-spacing:-.04em}.sub{color:var(--muted);margin-top:6px}.badge{display:inline-flex;border:1px solid var(--line);border-radius:999px;padding:6px 10px;color:var(--blue);background:#0f1b2fcc;font-weight:700}
    main{padding:24px clamp(16px,4vw,44px);display:grid;gap:18px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:14px}.card{background:linear-gradient(180deg,#ffffff12,#ffffff08);border:1px solid var(--line);border-radius:18px;padding:16px;box-shadow:0 20px 55px #0003}
    .card h2,.card h3{margin:0 0 10px}.muted{color:var(--muted)}.row{display:flex;justify-content:space-between;gap:12px;border-top:1px solid var(--line);padding:10px 0}.row:first-child{border-top:0}.ok{color:var(--ok)}.warn{color:var(--warn)}.bad{color:var(--bad)}.blue{color:var(--blue)}.purple{color:var(--purple)}
    .service{display:grid;gap:8px}.service strong{overflow-wrap:anywhere}.pill{display:inline-flex;width:max-content;max-width:100%;padding:4px 8px;border-radius:999px;border:1px solid var(--line);font-size:12px;font-weight:800}.up{color:var(--ok);border-color:#6ee7b755}.down{color:var(--bad);border-color:#fca5a555}.unknown{color:var(--muted)}
    pre{white-space:pre-wrap;background:#050b14;border:1px solid var(--line);border-radius:12px;padding:12px;max-height:260px;overflow:auto}.flow{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:10px}.flow div{border:1px solid var(--line);border-radius:14px;padding:12px;background:#0e1a2d}
  </style>
</head>
<body>
  <header>
    <div><h1>Robot Motion Runtime Dashboard</h1><div class="sub">服务、事实状态、任务阶段和运行时节点的实时视图</div></div>
    <div><span id="stamp" class="badge">connecting...</span></div>
  </header>
  <main>
    <section class="card">
      <h2>当前闭环</h2>
      <div class="flow">
        <div><b>1 固定事实状态</b><p class="muted">/robot_motion/set_state + /robot_motion/set_scene</p></div>
        <div><b>2 启动任务</b><p class="muted">/robot_motion/run_box_pair_task → RunDualArmPoseTask</p></div>
        <div><b>3 分阶段规划</b><p class="muted">PlanDualArmIk → PlanExtract → PlanLoaded → ExecuteTrajectory</p></div>
        <div><b>4 观测</b><p class="muted">runtime_status + RobotMotionState + ROS graph</p></div>
      </div>
    </section>
    <section class="grid">
      <article class="card"><h2>机器人事实状态</h2><div id="state"></div></article>
      <article class="card"><h2>场景事实状态</h2><div id="scene"></div></article>
      <article class="card"><h2>服务状态</h2><div id="services" class="grid"></div></article>
    </section>
    <section class="grid">
      <article class="card"><h2>运行节点</h2><div id="nodes"></div></article>
      <article class="card"><h2>阶段事件</h2><div id="events"></div></article>
    </section>
  </main>
  <script>
    const knownOrder = ["/robot_motion/set_state","/robot_motion/set_scene","/robot_motion/run_box_pair_task","/robot_motion/run_dual_arm_pose_task","/robot_motion/plan_dual_arm_ik","/robot_motion/solve_arm_ik","/robot_motion/plan_extract","/robot_motion/plan_loaded","/robot_motion/check_collision","/robot_motion/execute_trajectory","/robot_motion/run_task","/dual_arm_jtc/follow_joint_trajectory","/alfa_execution/execute_joint_trajectory"];
    function esc(s){return String(s ?? "").replace(/[&<>"']/g,m=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#039;"}[m]))}
    function serviceCard(s){
      const cls=s.available?"up":"down";
      const status=s.runtime_status;
      return `<div class="card service"><span class="pill ${cls}">${s.available?"UP":"DOWN"}</span><strong>${esc(s.name)}</strong><span class="muted">${esc((s.types||[]).join(", ")||"not discovered")}</span>${status?`<span class="pill blue">${esc(status.state)}</span><span class="muted">${esc(status.detail)}</span><small class="muted">requests ${status.request_count}, ok ${status.success_count}, fail ${status.failure_count}</small>`:"<span class='muted'>no runtime_status yet</span>"}</div>`
    }
    async function refresh(){
      const r=await fetch("/api/status",{cache:"no-store"});
      const data=await r.json();
      document.getElementById("stamp").textContent = new Date(data.stamp_ms).toLocaleTimeString();
      const st=data.latest_state;
      document.getElementById("state").innerHTML = st ? `
        <div class="row"><span>state_id</span><b class="blue">${esc(st.state_id)}</b></div>
        <div class="row"><span>source</span><b>${esc(st.source)}</b></div>
        <div class="row"><span>authoritative</span><b class="${st.authoritative?"ok":"warn"}">${st.authoritative}</b></div>
        <div class="row"><span>joints</span><b>${st.joint_count}</b></div>
        <pre>${esc(JSON.stringify(st.positions,null,2))}</pre>` : `<p class="bad">还没有收到 RobotMotionState。仿真/mock 请先调用 /robot_motion/set_state。</p>`;
      const sc=data.latest_scene;
      document.getElementById("scene").innerHTML = sc ? `
        <div class="row"><span>scene_id</span><b class="blue">${esc(sc.scene_id)}</b></div>
        <div class="row"><span>source</span><b>${esc(sc.source)}</b></div>
        <div class="row"><span>authoritative</span><b class="${sc.authoritative?"ok":"warn"}">${sc.authoritative}</b></div>
        <div class="row"><span>collision objects</span><b>${sc.scene_object_count}</b></div>
        <div class="row"><span>attached collision objects</span><b>${sc.attached_collision_object_count}</b></div>` : `<p class="bad">还没有收到 RobotMotionScene。仿真/mock 请先调用 /robot_motion/set_scene。</p>`;
      const services = data.services.sort((a,b)=>knownOrder.indexOf(a.name)-knownOrder.indexOf(b.name));
      document.getElementById("services").innerHTML = services.map(serviceCard).join("");
      document.getElementById("nodes").innerHTML = data.nodes.map(n=>`<div class="row"><span>${esc(n.namespace)}</span><b>${esc(n.name)}</b></div>`).join("");
      document.getElementById("events").innerHTML = Object.values(data.runtime_status).sort((a,b)=>(b.stamp_ms||0)-(a.stamp_ms||0)).map(e=>`<div class="row"><span><b>${esc(e.node)}</b><br><small class="muted">${esc(e.service)}</small></span><span><b class="${e.state==="error"?"bad":e.state==="running"?"warn":"ok"}">${esc(e.state)}</b><br><small class="muted">${esc(e.detail)}</small></span></div>`).join("");
    }
    refresh(); setInterval(refresh,1000);
  </script>
</body>
</html>"""


class MotionRuntimeDashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("motion_runtime_dashboard")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8766)
        self.declare_parameter("state_topic", "/robot_motion/state")
        self.declare_parameter("scene_topic", "/robot_motion/scene")

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.scene_topic = str(self.get_parameter("scene_topic").value)
        self.latest_state: dict[str, Any] | None = None
        self.latest_scene: dict[str, Any] | None = None
        self.runtime_status: dict[str, dict[str, Any]] = {}
        self.known_services = [
            "/robot_motion/set_state",
            "/robot_motion/set_scene",
            "/robot_motion/run_box_pair_task",
            "/robot_motion/run_dual_arm_pose_task",
            "/robot_motion/plan_dual_arm_ik",
            "/robot_motion/solve_arm_ik",
            "/robot_motion/plan_extract",
            "/robot_motion/plan_loaded",
            "/robot_motion/check_collision",
            "/robot_motion/execute_trajectory",
            "/robot_motion/run_task",
            "/dual_arm_jtc/follow_joint_trajectory",
            "/alfa_execution/execute_joint_trajectory",
        ]

        self.state_sub = self.create_subscription(RobotMotionState, self.state_topic, self.on_state, 10)
        self.scene_sub = self.create_subscription(RobotMotionScene, self.scene_topic, self.on_scene, 10)
        self.status_sub = self.create_subscription(String, "/robot_motion/runtime_status", self.on_status, 10)
        self.httpd = ThreadingHTTPServer((self.host, self.port), self.make_handler())
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(f"Motion runtime dashboard: http://{self.host}:{self.port}")

    def on_state(self, state: RobotMotionState) -> None:
        positions = {
            name: float(state.joint_state.position[index])
            for index, name in enumerate(state.joint_state.name)
            if index < len(state.joint_state.position)
        }
        self.latest_state = {
            "state_id": state.context.state_id,
            "scene_id": state.context.scene_id,
            "source": state.source,
            "authoritative": bool(state.authoritative),
            "joint_count": len(state.joint_state.name),
            "positions": positions,
        }

    def on_scene(self, scene: RobotMotionScene) -> None:
        self.latest_scene = {
            "scene_id": scene.context.scene_id,
            "state_id": scene.context.state_id,
            "source": scene.source,
            "authoritative": bool(scene.authoritative),
            "scene_object_count": len(scene.scene_objects),
            "attached_collision_object_count": len(scene.attached_collision_objects),
            "scene_object_ids": [item.id for item in scene.scene_objects],
            "attached_collision_object_ids": [item.object.id for item in scene.attached_collision_objects],
        }

    def on_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        key = payload.get("service") or payload.get("node") or str(time.time())
        self.runtime_status[key] = payload

    def status_payload(self) -> dict[str, Any]:
        service_map = {name: types for name, types in self.get_service_names_and_types()}
        action_map = {}
        if hasattr(self, "get_action_names_and_types"):
            action_map = {
                name: [f"{type_name} (action)" for type_name in types]
                for name, types in self.get_action_names_and_types()
            }
        services = []
        for name in self.known_services:
            services.append(
                {
                    "name": name,
                    "available": name in service_map or name in action_map,
                    "types": service_map.get(name, action_map.get(name, [])),
                    "runtime_status": self.runtime_status.get(name),
                }
            )
        nodes = [
            {"name": name, "namespace": namespace}
            for name, namespace in self.get_node_names_and_namespaces()
        ]
        return {
            "stamp_ms": now_ms(),
            "latest_state": self.latest_state,
            "latest_scene": self.latest_scene,
            "runtime_status": self.runtime_status,
            "services": services,
            "nodes": nodes,
        }

    def make_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):  # noqa: N802
                return

            def do_GET(self):  # noqa: N802
                path = urlparse(self.path).path
                if path == "/" or path == "/index.html":
                    body = DASHBOARD_HTML.encode("utf-8")
                    self.send_response(200)
                    self.send_header("content-type", "text/html; charset=utf-8")
                    self.send_header("content-length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if path == "/api/status":
                    body = json.dumps(node.status_payload(), ensure_ascii=False).encode("utf-8")
                    self.send_response(200)
                    self.send_header("content-type", "application/json; charset=utf-8")
                    self.send_header("cache-control", "no-store")
                    self.send_header("content-length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                self.send_error(404)

        return Handler

    def destroy_node(self):
        try:
            self.httpd.shutdown()
            self.httpd.server_close()
        except Exception:
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MotionRuntimeDashboardNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
