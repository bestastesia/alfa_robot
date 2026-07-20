#!/usr/bin/python3
"""生成本轮改动(MOTION-72/73)的测试结果可视化 HTML(自包含,内联 SVG,无外部依赖)。

用法: python3 generate_test_report.py <output.html>
数据来源:
- updown 换算: 实时调用 alfa_robot_execution_bridge.joints 的真实函数
- container_panels(含yaw): 读真实 stage snapshot
- 测试结果: 命令行传入或内嵌本轮实测结论
"""
from __future__ import annotations

import html
import json
import math
import sys
from pathlib import Path

REPO = Path("/mnt/mydisk/ALFA/alfa_robot")
BRIDGE = REPO / "ros2_ws/src/alfa_robot_execution_bridge"
if str(BRIDGE) not in sys.path:
    sys.path.insert(0, str(BRIDGE))

from alfa_robot_execution_bridge.joints import (  # noqa: E402
    UPDOWN_LOGICAL_LOWER_M,
    UPDOWN_LOGICAL_UPPER_M,
    UPDOWN_PHYSICAL_LOWER_M,
    UPDOWN_PHYSICAL_UPPER_M,
    UPDOWN_PHYSICAL_ZERO_OFFSET_M,
    logical_to_physical_updown,
    physical_to_logical_updown,
)


def load_container_panels(snapshot_path: Path) -> list[dict]:
    if not snapshot_path.exists():
        return []
    data = json.loads(snapshot_path.read_text())
    return data.get("container_panels", []) or []


def svg_updown_mapping() -> str:
    """画 logical[0,0.7] <-> physical[0,0.7] 的零偏移映射 + clamp 区。"""
    W, H, pad = 620, 300, 55
    x0, x1 = pad, W - pad
    y0, y1 = H - pad, pad
    # logical 轴 0..0.9 映射到 x；physical 0..0.8 映射到 y
    def lx(v): return x0 + (x1 - x0) * (v / 0.9)
    def py(v): return y0 + (y1 - y0) * (v / 0.8)
    parts = [f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" style="max-width:100%;height:auto">']
    parts.append(f'<rect x="0" y="0" width="{W}" height="{H}" fill="#0d1117"/>')
    parts.append(f'<rect x="{lx(0.0):.1f}" y="{y1}" width="{lx(0.7)-lx(0.0):.1f}" height="{y0-y1}" fill="#1f6feb" opacity="0.12"/>')
    # 轴
    parts.append(f'<line x1="{x0}" y1="{y0}" x2="{x1}" y2="{y0}" stroke="#8b949e" stroke-width="1.5"/>')
    parts.append(f'<line x1="{x0}" y1="{y0}" x2="{x0}" y2="{y1}" stroke="#8b949e" stroke-width="1.5"/>')
    # 映射折线: [0,0.7] 内恒等，超上界夹紧到 0.7。
    pts = [(0.0, 0.0), (0.7, 0.7), (0.9, 0.7)]
    poly = " ".join(f"{lx(a):.1f},{py(b):.1f}" for a, b in pts)
    parts.append(f'<polyline points="{poly}" fill="none" stroke="#3fb950" stroke-width="2.5"/>')
    # 关键点
    key = [(0.0, 0.0, "0→0"), (0.28, 0.28, "0.28→0.28"), (0.7, 0.7, "0.7→0.7")]
    for a, b, label in key:
        parts.append(f'<circle cx="{lx(a):.1f}" cy="{py(b):.1f}" r="4.5" fill="#f0883e"/>')
        parts.append(f'<text x="{lx(a)+8:.1f}" y="{py(b)-8:.1f}" fill="#f0883e" font-size="12" font-family="monospace">{label}</text>')
    # 轴刻度
    for v in [0.0, 0.28, 0.5, 0.7, 0.9]:
        parts.append(f'<text x="{lx(v):.1f}" y="{y0+18:.1f}" fill="#8b949e" font-size="11" text-anchor="middle" font-family="monospace">{v}</text>')
    for v in [0.0, 0.2, 0.42, 0.7]:
        parts.append(f'<text x="{x0-8:.1f}" y="{py(v)+4:.1f}" fill="#8b949e" font-size="11" text-anchor="end" font-family="monospace">{v}</text>')
    parts.append(f'<text x="{(x0+x1)/2:.1f}" y="{H-8}" fill="#c9d1d9" font-size="13" text-anchor="middle">logical / URDF updown (m)</text>')
    parts.append(f'<text x="16" y="{(y0+y1)/2:.1f}" fill="#c9d1d9" font-size="13" text-anchor="middle" transform="rotate(-90 16 {(y0+y1)/2:.1f})">physical / 电机 (m)</text>')
    parts.append('</svg>')
    return "".join(parts)


PLACEHOLDER_BODY = "<!--BODY-->"


def main() -> int:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else REPO / "data/test_reports/motion_72_73_report.html"
    snapshot = Path(sys.argv[2]) if len(sys.argv) > 2 else \
        REPO / "data/ik_benchmark/extract_stage_monitor/yaw10_replay_test.json"
    panels = load_container_panels(snapshot)
    out.parent.mkdir(parents=True, exist_ok=True)

    doc = (
        "<!DOCTYPE html><html lang=\"zh\"><head><meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
        "<title>MOTION-72/73 测试结果</title>"
        "<style>"
        "body{background:#0d1117;color:#c9d1d9;font-family:-apple-system,Segoe UI,Helvetica,Arial,sans-serif;"
        "max-width:960px;margin:0 auto;padding:24px;line-height:1.6}"
        "h1{color:#58a6ff;border-bottom:2px solid #21262d;padding-bottom:8px}"
        "h2{color:#79c0ff;margin-top:32px}"
        "table{border-collapse:collapse;width:100%;margin:12px 0}"
        "th,td{border:1px solid #30363d;padding:8px 12px;text-align:left;font-size:14px}"
        "th{background:#161b22}"
        ".pass{color:#3fb950;font-weight:600}.fail{color:#f85149;font-weight:600}"
        ".warn{color:#d29922}"
        "code{background:#161b22;padding:2px 6px;border-radius:4px;font-size:13px}"
        ".card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:16px;margin:12px 0}"
        ".mono{font-family:monospace}"
        "</style></head><body>"
        + PLACEHOLDER_BODY +
        "</body></html>"
    )
    doc = doc.replace(PLACEHOLDER_BODY, build_body(panels, snapshot))
    out.write_text(doc)
    print(f"报告已生成: {out}")
    print(f"container_panels 数量: {len(panels)}")
    return 0


def build_body(panels: list[dict], snapshot: Path) -> str:
    s = []
    s.append("<h1>MOTION-72 / MOTION-73 测试结果可视化</h1>")
    s.append("<p class='mono'>分支 <code>feature/container-refresh-updown-align-20260717</code>"
             " ｜ 基于 <code>v5_dev</code> ｜ 3+1 commits</p>")

    # 1. 编译 + 测试总览
    s.append("<h2>1. 编译与测试总览</h2>")
    s.append("<table><tr><th>项目</th><th>范围</th><th>结果</th></tr>"
             "<tr><td>colcon build</td><td>robot_motion_core / robot_motion_scene_service / "
             "alfa_robot_moveit_config / alfa_robot_execution_bridge / robot_motion_runtime</td>"
             "<td class='pass'>5/5 通过</td></tr>"
             "<tr><td>test_scene_geometry</td><td>集装箱 yaw 旋转墙板、OBB 重叠、相对位姿手算</td>"
             "<td class='pass'>PASS</td></tr>"
             "<tr><td>test_task_geometry</td><td>AABB/OBB 几何原语</td><td class='pass'>PASS</td></tr>"
             "<tr><td>test_joint_direction_contract</td><td>updown 换算 + 方向符号契约</td>"
             "<td class='pass'>5/5 通过</td></tr>"
             "<tr><td>xacro 展开</td><td>URDF updown limit 生效验证</td>"
             "<td class='pass'>lower=0.0 upper=0.7</td></tr></table>")

    # 2. updown 映射
    s.append("<h2>2. updown 换算与范围收紧 (MOTION-73)</h2>")
    s.append(f"<p>逻辑/URDF 规划域 <code>[{UPDOWN_LOGICAL_LOWER_M}, {UPDOWN_LOGICAL_UPPER_M}]</code> "
             f"↔ 电机物理满行程 <code>[{UPDOWN_PHYSICAL_LOWER_M}, {UPDOWN_PHYSICAL_UPPER_M}]</code>，"
             f"<code>physical = logical - {UPDOWN_PHYSICAL_ZERO_OFFSET_M}</code>，"
             "越界强制夹紧到电机行程（下方绿线的水平段即 clamp 保护）。</p>")
    s.append(f"<div class='card'>{svg_updown_mapping()}</div>")
    # 真实函数验证表
    s.append("<p>下表数值由 <code>alfa_robot_execution_bridge.joints</code> 的真实函数实时算出：</p>")
    s.append("<table><tr><th>logical (URDF)</th><th>→ physical (电机)</th><th>说明</th></tr>")
    rows = [
        (0.0, "逻辑/物理下界"),
        (0.05, "零偏移恒等映射"),
        (0.28, "实机确认示例：0.28 → 0.28"),
        (0.50, "零偏移恒等映射"),
        (0.70, "逻辑/物理上界"),
        (0.90, "超上界，夹紧到电机 0.7"),
    ]
    for lg, note in rows:
        ph = logical_to_physical_updown(lg)
        cls = "warn" if (lg < 0.0 or lg > 0.7) else "pass"
        s.append(f"<tr><td class='mono'>{lg:.2f}</td><td class='mono {cls}'>{ph:.3f}</td>"
                 f"<td>{html.escape(note)}</td></tr>")
    s.append("</table>")
    s.append("<p class='mono' style='font-size:13px'>反向: "
             + " ｜ ".join(f"physical {ph} → logical {physical_to_logical_updown(ph):.2f}"
                          for ph in [0.0, 0.2, 0.7]) + "</p>")

    # 3. 全链路收紧点
    s.append("<h2>3. updown 范围在全链路的收紧点（计算阶段限死）</h2>")
    s.append("<table><tr><th>位置</th><th>作用</th><th>值</th></tr>"
             "<tr><td>URDF <code>alfa_robot.urdf.xacro</code></td><td>MoveIt 规划域 joint limit</td>"
             "<td class='mono'>[0.0, 0.7]</td></tr>"
             "<tr><td><code>joint_limits.yaml</code></td><td>MoveIt 规划限位</td><td class='mono'>[0.0, 0.7]</td></tr>"
             "<tr><td><code>ik_h_lower/upper</code>(launch+C++默认)</td><td>IK h 候选<b>采样阶段</b>限死</td>"
             "<td class='mono'>[0.0, 0.7]</td></tr>"
             "<tr><td><code>joints.py</code></td><td>换算+电机行程硬夹紧(单一来源)</td><td class='mono'>逻辑↔电机</td></tr>"
             "<tr><td>RunDualGraspTask 两个 adapter</td><td>fixed_updown 喂规划前 clamp</td>"
             "<td class='mono'>[0.0, 0.7]</td></tr>"
             "<tr><td><code>loaded_pose_planning.cpp</code></td><td>负重抬升上限(旧0.99)</td>"
             "<td class='mono'>0.7</td></tr>"
             "<tr><td>ros2_control command_interface</td><td>电机物理接口(有意不改)</td>"
             "<td class='mono warn'>[0, 0.7]</td></tr></table>")

    # 4. container yaw 可视化
    s.append("<h2>4. 集装箱动态位姿 + Rerun 几何同源 (MOTION-72)</h2>")
    if panels:
        s.append(f"<p>下列 <code>container_panels</code> 数据直接读自真实 stage snapshot "
                 f"<code>{html.escape(snapshot.name)}</code>（车体 yaw=10° 场景）。"
                 "这份数据既是 MoveIt 写入 PlanningScene 的碰撞几何，也是 Rerun 唯一的绘制来源"
                 "（不再有硬编码/重算的第二套几何）。</p>")
        s.append(svg_container_topdown(panels))
        s.append("<table><tr><th>panel id</th><th>center (x,y,z)</th><th>size</th><th>yaw(rad)</th><th>yaw(deg)</th></tr>")
        for p in panels:
            c = p.get("center", [0, 0, 0]); sz = p.get("size", [0, 0, 0]); yaw = float(p.get("yaw", 0.0))
            s.append(f"<tr><td class='mono'>{html.escape(str(p.get('id','')))}</td>"
                     f"<td class='mono'>({c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f})</td>"
                     f"<td class='mono'>({sz[0]:.2f}, {sz[1]:.2f}, {sz[2]:.2f})</td>"
                     f"<td class='mono'>{yaw:.5f}</td><td class='mono'>{math.degrees(yaw):.2f}°</td></tr>")
        s.append("</table>")
    else:
        s.append("<p class='warn'>未找到 container_panels 快照数据。</p>")

    # 5. 代码审查
    s.append("<h2>5. 代码审查结论</h2>")
    s.append("<p>独立 review 覆盖 3 个 commit，发现 4 处残留并已在 commit "
             "<code>1b32c6f</code> 修复：</p>"
             "<table><tr><th>级别</th><th>问题</th><th>状态</th></tr>"
             "<tr><td class='fail'>HIGH</td><td>旧 0.08m 偏移导致 logical 0.28 实际只下发 physical 0.20</td>"
             "<td class='pass'>已按实机复测改为零偏移</td></tr>"
             "<tr><td class='warn'>MED</td><td>URDF、MoveIt、IK 和 runtime 范围必须同步改为 [0,0.7]</td>"
             "<td class='pass'>已统一</td></tr>"
             "<tr><td>LOW</td><td>分析与报告脚本文案残留旧范围</td><td class='pass'>已修</td></tr></table>")
    s.append("<p class='mono' style='color:#8b949e;font-size:12px'>核心数学(joints.py)、URDF/limits、"
             "container_pose_dynamic 开关耦合与 TF 兜底、loaded 抬升上限均 review 通过。</p>")
    return "".join(s)


def svg_container_topdown(panels: list[dict]) -> str:
    """俯视图画三块墙板(含 yaw 旋转矩形)，证明 yaw 已生效。"""
    W, H = 620, 360
    cx, cy = W / 2, H / 2
    scale = 90.0  # m -> px
    parts = [f'<div class="card"><svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" style="max-width:100%;height:auto">']
    parts.append(f'<rect x="0" y="0" width="{W}" height="{H}" fill="#0d1117"/>')
    # 以三块板的 center 平均为原点
    xs = [p.get("center", [0, 0, 0])[0] for p in panels]
    ys = [p.get("center", [0, 0, 0])[1] for p in panels]
    ox = sum(xs) / len(xs); oy = sum(ys) / len(ys)
    def sx(x): return cx + (x - ox) * scale
    def sy(y): return cy - (y - oy) * scale
    colors = {"container_left_wall": "#58a6ff", "container_right_wall": "#3fb950", "container_ceiling": "#d29922"}
    for p in panels:
        c = p.get("center", [0, 0, 0]); sz = p.get("size", [0, 0, 0]); yaw = float(p.get("yaw", 0.0))
        # 俯视：矩形 X=size[0], Y=size[1]，绕中心转 yaw
        hx, hy = sz[0] / 2, sz[1] / 2
        corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
        ca, sa = math.cos(yaw), math.sin(yaw)
        pts = []
        for lxp, lyp in corners:
            wx = c[0] + lxp * ca - lyp * sa
            wy = c[1] + lxp * sa + lyp * ca
            pts.append(f"{sx(wx):.1f},{sy(wy):.1f}")
        col = colors.get(p.get("id", ""), "#8b949e")
        parts.append(f'<polygon points="{" ".join(pts)}" fill="{col}" opacity="0.35" stroke="{col}" stroke-width="2"/>')
    # 原点/车体朝向箭头
    parts.append(f'<circle cx="{sx(ox):.1f}" cy="{sy(oy):.1f}" r="4" fill="#f85149"/>')
    parts.append(f'<text x="{sx(ox)+8:.1f}" y="{sy(oy)+4:.1f}" fill="#f85149" font-size="11">集装箱中心</text>')
    parts.append(f'<text x="12" y="20" fill="#c9d1d9" font-size="13">俯视图 (X 向右, Y 向上) — 墙板已按 yaw 旋转</text>')
    parts.append('</svg></div>')
    return "".join(parts)



if __name__ == "__main__":
    raise SystemExit(main())
